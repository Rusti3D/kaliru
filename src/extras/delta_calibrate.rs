// Delta calibration support (port of klippy/extras/delta_calibrate.py)
//
// Copyright (C) 2017-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// mathutil.background_coordinate_descent runs the descent in a child
// process while the reactor pauses; here it runs inline, followed by one
// reactor pause (the first iteration of python's wait loop).
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::probe::{FinalizeResult, ProbePointsHelper};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::kinematics::delta::{math_kerror, Calibration};
use crate::logging;
use crate::mathutil::coordinate_descent;
use crate::printer::{ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_float_repr};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

/// a stable position; python keeps some as lists and some as tuples
#[derive(Clone)]
pub struct Stable {
    v: Vec<f64>,
    is_tuple: bool,
}

impl Stable {
    fn list(v: Vec<f64>) -> Self {
        Stable { v, is_tuple: false }
    }
    fn repr(&self) -> String {
        let items: Vec<String> = self.v.iter().map(|x| py_float_repr(*x)).collect();
        if self.is_tuple {
            format!("({})", items.join(", "))
        } else {
            format!("[{}]", items.join(", "))
        }
    }
    fn fmt3(&self) -> String {
        format!("{:.3},{:.3},{:.3}", self.v[0], self.v[1], self.v[2])
    }
}

// Load a stable position from a config entry
fn load_config_stable(config: &ConfigWrapper, option: &str) -> KResult<Stable> {
    // (getfloatlist returns a tuple)
    Ok(Stable { v: config.getfloatlist(option, Dflt::Required, ",", Some(3))?.unwrap_or_default(), is_tuple: true })
}

// The angles and distances of the calibration object found in
// docs/prints/calibrate_size.stl
pub const MEASURE_ANGLES: [f64; 6] = [210.0, 270.0, 330.0, 30.0, 90.0, 150.0];
pub const MEASURE_OUTER_RADIUS: f64 = 65.0;
pub const MEASURE_RIDGE_RADIUS: f64 = 5.0 - 0.5;

// How much to prefer a distance measurement over a height measurement
pub const MEASURE_WEIGHT: f64 = 0.5;

type Distance = (f64, Stable, Stable);

// Convert distance measurements made on the calibration object to
// 3-tuples of (actual_distance, stable_position1, stable_position2)
fn measurements_to_distances(mp: &IndexMap<String, Vec<f64>>, dp: &dyn Calibration) -> KResult<Vec<Distance>> {
    let scale = mp["SCALE"][0];
    let cpw = &mp["CENTER_PILLAR_WIDTHS"];
    let center_widths = [cpw[0], cpw[2], cpw[1], cpw[0], cpw[2], cpw[1]];
    let center_dists: Vec<f64> = mp["CENTER_DISTS"].iter().zip(center_widths).map(|(od, cw)| od - cw).collect();
    let outer_dists: Vec<f64> = mp["OUTER_DISTS"].iter().zip(&mp["OUTER_PILLAR_WIDTHS"]).map(|(od, opw)| od - opw).collect();
    // Convert angles in degrees to an XY multiplier
    let xy_angles: Vec<(f64, f64)> = MEASURE_ANGLES.iter().map(|a| a.to_radians()).map(|a| (a.cos(), a.sin())).collect();
    let stable = |c: [f64; 3]| dp.calc_stable_position(&c).map(Stable::list).map_err(math_kerror);
    // Calculate stable positions for center measurements
    let inner_ridge = MEASURE_RIDGE_RADIUS * scale;
    let outer_ridge = (MEASURE_OUTER_RADIUS + MEASURE_RIDGE_RADIUS) * scale;
    let mut out = vec![];
    for ((cd, (ax, ay)), _) in center_dists.iter().zip(&xy_angles).zip(0..6) {
        let ip = [ax * inner_ridge, ay * inner_ridge, 0.0];
        let op = [ax * outer_ridge, ay * outer_ridge, 0.0];
        out.push((*cd, stable(ip)?, stable(op)?));
    }
    // Calculate positions of outer measurements
    let outer_center = MEASURE_OUTER_RADIUS * scale;
    let start_pos: Vec<(f64, f64)> = xy_angles.iter().map(|(ax, ay)| (ax * outer_center, ay * outer_center)).collect();
    let mut shifted_angles = xy_angles[2..].to_vec();
    shifted_angles.extend_from_slice(&xy_angles[..2]);
    for ((od, (ax, ay)), (spx, spy)) in outer_dists.iter().zip(&shifted_angles).zip(&start_pos) {
        let fp = [ax * inner_ridge + spx, ay * inner_ridge + spy, 0.0];
        let sp = [ax * outer_ridge + spx, ay * outer_ridge + spy, 0.0];
        out.push((*od, stable(fp)?, stable(sp)?));
    }
    Ok(out)
}

const CMD_DELTA_CALIBRATE_HELP: &str = "Delta calibration script";
const CMD_DELTA_ANALYZE_HELP: &str = "Extended delta calibration tool";

pub struct DeltaCalibrate {
    printer: Rc<Printer>,
    probe_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    last_probe_positions: RefCell<Vec<(f64, Stable)>>,
    manual_heights: RefCell<Vec<(f64, Stable)>>,
    delta_analyze_entry: RefCell<IndexMap<String, Vec<f64>>>,
    last_distances: Vec<Distance>,
    gcode: Rc<GCodeDispatch>,
}

fn kin_delta(printer: &Printer) -> KResult<Rc<dyn Calibration>> {
    let kin = printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
    kin.get_calibration().ok_or_else(|| KError::Other("object has no attribute 'get_calibration'".into()))
}

impl DeltaCalibrate {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let p = printer.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| {
                let kin = p.lookup::<ToolHead>("toolhead")?.get_kinematics();
                if kin.get_calibration().is_none() {
                    return Err(KError::config("Delta calibrate is only for delta printers"));
                }
                Ok(())
            }),
        );
        // Calculate default probing points
        let radius = config.getfloat("radius", Dflt::Required, Lim::new().above(0.0))?;
        let mut points = vec![(0.0, 0.0)];
        let scatter = [0.95, 0.90, 0.85, 0.70, 0.75, 0.80];
        for (i, s) in scatter.iter().enumerate() {
            let r = (90.0 + 60.0 * i as f64).to_radians();
            let dist = radius * s;
            points.push((r.cos() * dist, r.sin() * dist));
        }
        let slot: Rc<RefCell<Weak<DeltaCalibrate>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        let probe_helper = ProbePointsHelper::new(
            config,
            Rc::new(move |offsets, positions| match sl.borrow().upgrade() {
                Some(d) => d.probe_finalize(offsets, positions),
                None => Ok(FinalizeResult::Done),
            }),
            Some(points),
            "points",
            false,
            false,
        )?;
        probe_helper.minimum_points(3)?;
        // Restore probe stable positions
        let mut last_probe_positions = vec![];
        for i in 0..999 {
            let Some(height) = config.getfloat_opt(&format!("height{}", i), Lim::new())? else { break };
            let height_pos = load_config_stable(config, &format!("height{}_pos", i))?;
            last_probe_positions.push((height, height_pos));
        }
        // Restore manually entered heights
        let mut manual_heights = vec![];
        for i in 0..999 {
            let Some(height) = config.getfloat_opt(&format!("manual_height{}", i), Lim::new())? else { break };
            let height_pos = load_config_stable(config, &format!("manual_height{}_pos", i))?;
            manual_heights.push((height, height_pos));
        }
        // Restore distance measurements
        let mut delta_analyze_entry = IndexMap::new();
        delta_analyze_entry.insert("SCALE".to_string(), vec![1.0]);
        let mut last_distances = vec![];
        for i in 0..999 {
            let Some(dist) = config.getfloat_opt(&format!("distance{}", i), Lim::new())? else { break };
            let distance_pos1 = load_config_stable(config, &format!("distance{}_pos1", i))?;
            let distance_pos2 = load_config_stable(config, &format!("distance{}_pos2", i))?;
            last_distances.push((dist, distance_pos1, distance_pos2));
        }
        // Register gcode commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let dc = Rc::new(DeltaCalibrate {
            printer: printer.clone(),
            probe_helper: RefCell::new(Some(probe_helper)),
            last_probe_positions: RefCell::new(last_probe_positions),
            manual_heights: RefCell::new(manual_heights),
            delta_analyze_entry: RefCell::new(delta_analyze_entry),
            last_distances,
            gcode: gcode.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&dc);
        let d = dc.clone();
        gcode.register_command(
            "DELTA_CALIBRATE",
            Some(Rc::new(move |g: &GCodeCommand| d.probe_helper.borrow().clone().expect("probe helper").start_probe(g))),
            false,
            Some(CMD_DELTA_CALIBRATE_HELP),
        )?;
        let d = dc.clone();
        gcode.register_command("DELTA_ANALYZE", Some(Rc::new(move |g| d.cmd_delta_analyze(g))), false, Some(CMD_DELTA_ANALYZE_HELP))?;
        Ok(dc)
    }

    fn save_state(&self, probe_positions: &[(f64, Stable)], distances: &[Distance], delta_params: &dyn Calibration) -> KResult<()> {
        // Save main delta parameters
        delta_params.save_state(&self.printer)?;
        // Save probe stable positions
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let mut cf = configfile.pconfig.borrow_mut();
        let section = "delta_calibrate";
        cf.remove_section(section);
        for (i, (z_offset, spos)) in probe_positions.iter().enumerate() {
            cf.set(section, &format!("height{}", i), &py_float_repr(*z_offset))?;
            cf.set(section, &format!("height{}_pos", i), &spos.fmt3())?;
        }
        // Save manually entered heights
        for (i, (z_offset, spos)) in self.manual_heights.borrow().iter().enumerate() {
            cf.set(section, &format!("manual_height{}", i), &py_float_repr(*z_offset))?;
            cf.set(section, &format!("manual_height{}_pos", i), &spos.fmt3())?;
        }
        // Save distance measurements
        for (i, (dist, spos1, spos2)) in distances.iter().enumerate() {
            cf.set(section, &format!("distance{}", i), &py_float_repr(*dist))?;
            cf.set(section, &format!("distance{}_pos1", i), &spos1.fmt3())?;
            cf.set(section, &format!("distance{}_pos2", i), &spos2.fmt3())?;
        }
        Ok(())
    }

    fn probe_finalize(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        // Convert positions into (z_offset, stable_position) pairs
        let z_offset = offsets.2;
        let delta_params = kin_delta(&self.printer)?;
        let mut probe_positions = vec![];
        for p in positions {
            probe_positions.push((z_offset, Stable::list(delta_params.calc_stable_position(p).map_err(math_kerror)?)));
        }
        // Perform analysis
        self.calculate_params(&probe_positions, &self.last_distances)?;
        Ok(FinalizeResult::Done)
    }

    fn calculate_params(&self, probe_positions: &[(f64, Stable)], distances: &[Distance]) -> KResult<()> {
        let mut height_positions = self.manual_heights.borrow().clone();
        height_positions.extend_from_slice(probe_positions);
        // Setup for coordinate descent analysis
        let odp = kin_delta(&self.printer)?;
        let (adj_params, params) = odp.coordinate_descent_params(!distances.is_empty());
        let hp: Vec<String> = height_positions.iter().map(|(z, s)| format!("({}, {})", py_float_repr(*z), s.repr())).collect();
        let dr: Vec<String> = distances.iter().map(|(d, a, b)| format!("({}, {}, {})", py_float_repr(*d), a.repr(), b.repr())).collect();
        let pr: Vec<String> = params.iter().map(|(k, v)| format!("'{}': {}", k, py_float_repr(*v))).collect();
        logging::info(&format!(
            "Calculating delta_calibrate with:\n[{}]\n[{}]\nInitial delta_calibrate parameters: {{{}}}",
            hp.join(", "),
            dr.join(", "),
            pr.join(", ")
        ));
        let mut z_weight = 1.0;
        if !distances.is_empty() {
            z_weight = distances.len() as f64 / (MEASURE_WEIGHT * probe_positions.len() as f64);
        }

        // Perform coordinate descent
        let delta_errorfunc = |params: &IndexMap<String, f64>| -> f64 {
            let r = (|| -> Result<f64, crate::mathutil::MathError> {
                // Build new delta_params for params under test
                let delta_params = odp.new_calibration(params);
                // Calculate z height errors
                let mut total_error = 0.0;
                for (z_offset, stable_pos) in &height_positions {
                    let p = delta_params.get_position_from_stable(&stable_pos.v)?;
                    total_error += (p[2] - z_offset).powf(2.0);
                }
                total_error *= z_weight;
                // Calculate distance errors
                for (dist, stable_pos1, stable_pos2) in distances {
                    let p1 = delta_params.get_position_from_stable(&stable_pos1.v)?;
                    let p2 = delta_params.get_position_from_stable(&stable_pos2.v)?;
                    let d = ((p1[0] - p2[0]).powf(2.0) + (p1[1] - p2[1]).powf(2.0) + (p1[2] - p2[2]).powf(2.0)).sqrt();
                    total_error += (d - dist).powf(2.0);
                }
                Ok(total_error)
            })();
            // (python catches ValueError only)
            r.unwrap_or(9999999999999.9)
        };

        let new_params = coordinate_descent(&adj_params, &params, &delta_errorfunc);
        let reactor = self.printer.get_reactor();
        reactor.pause(reactor.monotonic() + 0.1)?;
        // Log and report results
        let pr: Vec<String> = new_params.iter().map(|(k, v)| format!("'{}': {}", k, py_float_repr(*v))).collect();
        logging::info(&format!("Calculated delta_calibrate parameters: {{{}}}", pr.join(", ")));
        let new_delta_params = odp.new_calibration(&new_params);
        for (z_offset, spos) in &height_positions {
            logging::info(&format!(
                "height orig: {:.6} new: {:.6} goal: {:.6}",
                odp.get_position_from_stable(&spos.v).map_err(math_kerror)?[2],
                new_delta_params.get_position_from_stable(&spos.v).map_err(math_kerror)?[2],
                z_offset
            ));
        }
        let dist_of = |dp: &dyn Calibration, a: &Stable, b: &Stable| -> KResult<f64> {
            let p1 = dp.get_position_from_stable(&a.v).map_err(math_kerror)?;
            let p2 = dp.get_position_from_stable(&b.v).map_err(math_kerror)?;
            Ok(((p1[0] - p2[0]).powf(2.0) + (p1[1] - p2[1]).powf(2.0) + (p1[2] - p2[2]).powf(2.0)).sqrt())
        };
        for (dist, spos1, spos2) in distances {
            let orig_dist = dist_of(&*odp, spos1, spos2)?;
            let new_dist = dist_of(&*new_delta_params, spos1, spos2)?;
            logging::info(&format!("distance orig: {:.6} new: {:.6} goal: {:.6}", orig_dist, new_dist, dist));
        }
        // Store results for SAVE_CONFIG
        self.save_state(probe_positions, distances, &*new_delta_params)?;
        self.gcode.respond_info(
            "The SAVE_CONFIG command will update the printer config file\nwith these parameters and restart the printer.",
            true,
        );
        Ok(())
    }

    fn add_manual_height(&self, height: f64) -> KResult<()> {
        // Determine current location of toolhead
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.flush_step_generation()?;
        let kin = toolhead.get_kinematics();
        let kin_spos: IndexMap<String, f64> = kin.get_steppers().iter().map(|s| (s.get_name(false), s.get_commanded_position())).collect();
        let kin_pos = kin.calc_position(&kin_spos);
        // Convert location to a stable position
        let delta_params = kin_delta(&self.printer)?;
        let stable_pos = Stable { v: delta_params.calc_stable_position(&kin_pos).map_err(math_kerror)?, is_tuple: true };
        // Add to list of manual heights
        self.manual_heights.borrow_mut().push((height, stable_pos));
        self.gcode.respond_info(
            &format!("Adding manual height: {:.3},{:.3},{:.3} is actually z={:.3}", kin_pos[0], kin_pos[1], kin_pos[2], height),
            true,
        );
        Ok(())
    }

    fn do_extended_calibration(&self) -> KResult<()> {
        // Extract distance positions
        let entry = self.delta_analyze_entry.borrow().clone();
        let distances = if entry.len() <= 1 {
            self.last_distances.clone()
        } else if entry.len() < 5 {
            return Err(KError::command("Not all measurements provided"));
        } else {
            let delta_params = kin_delta(&self.printer)?;
            measurements_to_distances(&entry, &*delta_params)?
        };
        let last = self.last_probe_positions.borrow().clone();
        if last.is_empty() {
            return Err(KError::command("Must run basic calibration with DELTA_CALIBRATE first"));
        }
        // Perform analysis
        self.calculate_params(&last, &distances)
    }

    fn cmd_delta_analyze(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Check for manual height entry
        if let Some(mheight) = gcmd.get_float_opt("MANUAL_HEIGHT", Lim::new())? {
            return self.add_manual_height(mheight);
        }
        // Parse distance measurements
        let args = [("CENTER_DISTS", 6), ("CENTER_PILLAR_WIDTHS", 3), ("OUTER_DISTS", 6), ("OUTER_PILLAR_WIDTHS", 6), ("SCALE", 1)];
        for (name, count) in args {
            let Some(data) = gcmd.get_opt(name) else { continue };
            let parts: Option<Vec<f64>> = data.split(',').map(py_float).collect();
            let parts = parts.ok_or_else(|| gcmd.error(format!("Unable to parse parameter '{}'", name)))?;
            if parts.len() != count {
                return Err(gcmd.error(format!("Parameter '{}' must have {} values", name, count)));
            }
            let pr: Vec<String> = parts.iter().map(|x| py_float_repr(*x)).collect();
            logging::info(&format!("DELTA_ANALYZE {} = [{}]", name, pr.join(", ")));
            self.delta_analyze_entry.borrow_mut().insert(name.to_string(), parts);
        }
        // Perform analysis if requested
        if let Some(action) = gcmd.get_opt("CALIBRATE") {
            if action != "extended" {
                return Err(gcmd.error("Unknown calibrate action"));
            }
            self.do_extended_calibration()?;
        }
        Ok(())
    }
}

impl PrinterObject for DeltaCalibrate {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        *self.probe_helper.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(DeltaCalibrate::new(config)?)
}
