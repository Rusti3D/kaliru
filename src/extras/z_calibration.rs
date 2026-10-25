// Klipper plugin for a self-calibrating Z offset
// (port of klippy/extras/z_calibration.py)
//
// Copyright (C) 2021-2023  Titus Meyer <info@protoloft.org>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bed_mesh::BedMesh;
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::extras::gcode_move::GCodeMove;
use crate::extras::homing::{Homing, PrinterHoming};
use crate::extras::probe::{as_printer_probe, probe_list_get_list, PrinterProbe};
use crate::extras::query_endstops::QueryEndstops;
use crate::extras::safe_z_home::SafeZHoming;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::EndstopRef;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_float_repr, py_max, py_min, py_sum};
use crate::pyval::PyVal;
use crate::stepper::RailRef;
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// a site: [x, y, z] with python's None as None
type Site = Vec<Option<f64>>;

const CMD_CALIBRATE_Z_HELP: &str = "Automatically calibrates the nozzle offset to the print surface";
const CMD_PROBE_Z_ACCURACY_HELP: &str = "Probe Z-Endstop accuracy at Nozzle-Endstop position";
const CMD_CALCULATE_SWITCH_OFFSET_HELP: &str = "Calculates a switch_offset based on the current z position";

/// the numbers of a site for %.3f ("None" where python would raise)
fn site_val(site: &Option<Site>, i: usize) -> KResult<f64> {
    site.as_ref()
        .and_then(|s| s.get(i).copied().flatten())
        .ok_or_else(|| KError::Other("must be real number, not NoneType".into()))
}

fn need(v: Option<f64>) -> KResult<f64> {
    v.ok_or_else(|| KError::Other("must be real number, not NoneType".into()))
}

pub struct ZCalibrationHelper {
    me: Weak<ZCalibrationHelper>,
    z_endstop: RefCell<Option<EndstopRef>>,
    z_homing: Cell<Option<f64>>,
    last_state: Cell<bool>,
    last_z_offset: Cell<f64>,
    position_z_endstop: Cell<Option<f64>>,
    config: ConfigWrapper,
    printer: Rc<Printer>,
    switch_offset: f64,
    max_deviation: Option<f64>,
    offset_margins: Vec<f64>,
    speed: f64,
    clearance: Cell<Option<f64>>,
    safe_z_height: Cell<Option<f64>>,
    samples: Cell<Option<i64>>,
    tolerance: Cell<Option<f64>>,
    retries: Cell<Option<i64>>,
    samples_result: RefCell<Option<String>>,
    lift_speed: Cell<Option<f64>>,
    probing_speed: Cell<Option<f64>>,
    second_speed: Cell<Option<f64>>,
    retract_dist: Cell<Option<f64>>,
    position_min: Cell<Option<f64>>,
    first_fast: bool,
    nozzle_site: RefCell<Option<Site>>,
    switch_site: RefCell<Option<Site>>,
    switch_xy_offsets: Option<Site>,
    bed_site: RefCell<Option<Site>>,
    wiggle_offsets: Option<Site>,
    start_gcode: Rc<Template>,
    switch_gcode: Rc<Template>,
    end_gcode: Rc<Template>,
    query_endstops: Rc<QueryEndstops>,
    gcode: Rc<GCodeDispatch>,
}

impl ZCalibrationHelper {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let switch_offset = config.getfloat("switch_offset", 0.0, Lim::new())?;
        // max_deviation is deprecated
        let max_deviation = config.getfloat_opt("max_deviation", Lim::new().above(0.0))?;
        config.deprecate("max_deviation", None);
        let offset_margins = get_offset_margins(config, "offset_margins", "-1.0,1.0")?;
        let speed = config.getfloat("speed", 50.0, Lim::new().above(0.0))?;
        // clearance is deprecated
        let clearance = config.getfloat_opt("clearance", Lim::new().above(0.0))?;
        config.deprecate("clearance", None);
        let safe_z_height = config.getfloat_opt("safe_z_height", Lim::new().above(0.0))?;
        let samples = config.getint_opt("samples", Lim::new().min(1.0))?;
        let tolerance = config.getfloat_opt("samples_tolerance", Lim::new().above(0.0))?;
        let retries = config.getint_opt("samples_tolerance_retries", Lim::new().min(0.0))?;
        let samples_result = config.getchoice(
            "samples_result",
            &[("none", None), ("median", Some("median".to_string())), ("average", Some("average".to_string()))],
            "none",
        )?;
        let lift_speed = config.getfloat_opt("lift_speed", Lim::new().above(0.0))?;
        let probing_speed = config.getfloat_opt("probing_speed", Lim::new().above(0.0))?;
        let second_speed = config.getfloat_opt("probing_second_speed", Lim::new().above(0.0))?;
        let retract_dist = config.getfloat_opt("probing_retract_dist", Lim::new().above(0.0))?;
        let position_min = config.getfloat_opt("position_min", Lim::new())?;
        let first_fast = config.getboolean("probing_first_fast", false)?;
        let nozzle_site = get_xy(config, "nozzle_xy_position", true)?;
        let switch_site = get_xy(config, "switch_xy_position", true)?;
        let switch_xy_offsets = get_xy(config, "switch_xy_offsets", true)?;
        let bed_site = get_xy(config, "bed_xy_position", true)?;
        let wiggle_offsets = get_xy(config, "wiggle_xy_offsets", true)?;
        let gcode_macro = load_gcode_macro(config)?;
        let start_gcode = gcode_macro.load_template(config, "start_gcode", Some(""))?;
        let switch_gcode = gcode_macro.load_template(config, "before_switch_gcode", Some(""))?;
        let end_gcode = gcode_macro.load_template(config, "end_gcode", Some(""))?;
        let query_endstops = printer.load_as::<QueryEndstops>(config, "query_endstops")?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let h = Rc::new_cyclic(|me| ZCalibrationHelper {
            me: me.clone(),
            z_endstop: RefCell::new(None),
            z_homing: Cell::new(None),
            last_state: Cell::new(false),
            last_z_offset: Cell::new(0.0),
            position_z_endstop: Cell::new(None),
            config: config.clone(),
            printer: printer.clone(),
            switch_offset,
            max_deviation,
            offset_margins,
            speed,
            clearance: Cell::new(clearance),
            safe_z_height: Cell::new(safe_z_height),
            samples: Cell::new(samples),
            tolerance: Cell::new(tolerance),
            retries: Cell::new(retries),
            samples_result: RefCell::new(samples_result),
            lift_speed: Cell::new(lift_speed),
            probing_speed: Cell::new(probing_speed),
            second_speed: Cell::new(second_speed),
            retract_dist: Cell::new(retract_dist),
            position_min: Cell::new(position_min),
            first_fast,
            nozzle_site: RefCell::new(nozzle_site),
            switch_site: RefCell::new(switch_site),
            switch_xy_offsets,
            bed_site: RefCell::new(bed_site),
            wiggle_offsets,
            start_gcode,
            switch_gcode,
            end_gcode,
            query_endstops,
            gcode: gcode.clone(),
        });
        let w = h.me.clone();
        crate::events::KlippyConnect::on(&printer, move || match w.upgrade() {
            Some(h) => h.handle_connect(),
            None => Ok(()),
        });
        let w = h.me.clone();
        crate::events::HomingHomeRailsEnd::on(&printer, move |homing_state, rails| {
            if let Some(h) = w.upgrade() {
                h.handle_home_rails_end(&homing_state, &rails);
            }
            Ok(())
        });
        type ZCmd = fn(&ZCalibrationHelper, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, ZCmd, &str); 3] = [
            ("CALIBRATE_Z", ZCalibrationHelper::cmd_calibrate_z, CMD_CALIBRATE_Z_HELP),
            ("PROBE_Z_ACCURACY", ZCalibrationHelper::cmd_probe_z_accuracy, CMD_PROBE_Z_ACCURACY_HELP),
            ("CALCULATE_SWITCH_OFFSET", ZCalibrationHelper::cmd_calculate_switch_offset, CMD_CALCULATE_SWITCH_OFFSET_HELP),
        ];
        for (name, f, desc) in cmds {
            let w = h.me.clone();
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(h) => f(&h, g),
                    None => Ok(()),
                })),
                false,
                Some(desc),
            )?;
        }
        Ok(h)
    }

    fn handle_connect(&self) -> KResult<()> {
        // get z-endstop object
        for (endstop, name) in self.query_endstops.endstops() {
            if name == "z" {
                // check for virtual endstops..
                if !endstop.is_mcu_endstop() {
                    return Err(KError::config(format!("A virtual endstop for z is not supported for {}", self.config.get_name())));
                }
                *self.z_endstop.borrow_mut() = Some(endstop);
            }
        }
        // get z-endstop position from safe_z_home
        if self.nozzle_site.borrow().is_none() {
            let safe_z_home = self.printer.lookup_object_opt("safe_z_home").and_then(downcast_rc::<SafeZHoming>);
            let Some(safe_z_home) = safe_z_home else {
                return Err(KError::config(format!("No nozzle position configured for {}", self.config.get_name())));
            };
            *self.nozzle_site.borrow_mut() = Some(vec![Some(safe_z_home.home_x_pos), Some(safe_z_home.home_y_pos), None]);
        }
        // check/calculate switch position by offsets
        if self.switch_site.borrow().is_none() {
            let Some(offsets) = &self.switch_xy_offsets else {
                return Err(KError::config(format!("No switch position configured for {}", self.config.get_name())));
            };
            let nozzle = self.nozzle_site.borrow().clone();
            let x = site_val(&nozzle, 0)? + offsets[0].unwrap_or(0.0);
            let y = site_val(&nozzle, 1)? + offsets[1].unwrap_or(0.0);
            *self.switch_site.borrow_mut() = Some(vec![Some(x), Some(y), None]);
        }
        Ok(())
    }

    fn load_probe(&self, probe: &PrinterProbe) {
        if self.samples.get().is_none() {
            self.samples.set(Some(probe.sample_count));
        }
        if self.tolerance.get().is_none() {
            self.tolerance.set(Some(probe.samples_tolerance));
        }
        if self.retries.get().is_none() {
            self.retries.set(Some(probe.samples_retries));
        }
        if self.lift_speed.get().is_none() {
            self.lift_speed.set(Some(probe.lift_speed));
        }
        // clearance is deprecated
        if self.clearance.get() == Some(0.0) {
            self.clearance.set(Some(20.0)); // defaults to 20mm
        }
        if self.safe_z_height.get().is_none() {
            self.safe_z_height.set(Some(probe.z_offset * 2.0));
        }
        if self.safe_z_height.get().unwrap_or(0.0) < 3.0 {
            self.safe_z_height.set(Some(20.0)); // defaults to 20mm
        }
        if self.samples_result.borrow().is_none() {
            *self.samples_result.borrow_mut() = Some(probe.samples_result.clone());
        }
    }

    fn handle_home_rails_end(&self, _homing_state: &Rc<Homing>, rails: &[RailRef]) {
        // get z homing position
        for rail in rails {
            if rail.get_steppers()[0].is_active_axis('z') {
                // get homing settings from z rail
                let hi = rail.get_homing_info();
                self.z_homing.set(Some(hi.position_endstop));
                if self.probing_speed.get().is_none() {
                    self.probing_speed.set(Some(hi.speed));
                }
                if self.second_speed.get().is_none() {
                    self.second_speed.set(Some(hi.second_homing_speed));
                }
                if self.retract_dist.get().is_none() {
                    self.retract_dist.set(Some(hi.retract_dist));
                }
                if self.position_min.get().is_none() {
                    self.position_min.set(Some(rail.get_range().0));
                }
                self.position_z_endstop.set(Some(hi.position_endstop));
            }
        }
    }

    fn command_probe(&self, gcmd: &GCodeCommand) -> KResult<Rc<PrinterProbe>> {
        let probe = probe_list_get_list(&self.printer)?.get_command_probe(gcmd, None, None)?;
        probe.as_ref().and_then(as_printer_probe).ok_or_else(|| KError::Other("'NoneType' object has no attribute 'sample_count'".into()))
    }

    fn cmd_calibrate_z(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let probe = self.command_probe(gcmd)?;
        self.load_probe(&probe);
        if self.z_homing.get().is_none() {
            return Err(gcmd.error("Must home axes first"));
        }
        if let Some(site_attr) = gcmd.get_opt("BED_POSITION") {
            // set bed site from BED_POSITION parameter
            *self.bed_site.borrow_mut() = Some(parse_xy(&self.config, "BED_POSITION", &site_attr)?);
        } else if get_xy(&self.config, "bed_xy_position", true)?.is_some() {
            // set bed site from configuration
            *self.bed_site.borrow_mut() = get_xy(&self.config, "bed_xy_position", false)?;
        } else {
            // else get the mesh's zero reference position
            let zero_ref = self.printer.lookup_object_opt("bed_mesh").and_then(downcast_rc::<BedMesh>).and_then(|m| m.zero_ref_pos());
            // (the deprecated relative_reference_index no longer exists)
            let Some((x, y)) = zero_ref else {
                return Err(gcmd.error(format!(
                    "Either use the BED_POSITION parameter, configure a bed_xy_position or define a mesh with a zero_reference_position for {}",
                    self.config.get_name()
                )));
            };
            *self.bed_site.borrow_mut() = Some(vec![Some(x), Some(y)]);
            logging::debug(&format!("Z-CALIBRATION probe bed_x={:.3} bed_y={:.3}", x, y));
        }
        self.log_config()?;
        let state = CalibrationState::new(probe, self, gcmd)?;
        state.calibrate_z()
    }

    fn cmd_probe_z_accuracy(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let probe = self.command_probe(gcmd)?;
        self.load_probe(&probe);
        if self.z_homing.get().is_none() {
            return Err(gcmd.error("Must home axes first"));
        }
        let speed = gcmd.get_float("PROBE_SPEED", self.second_speed.get(), Lim::new().above(0.0))?;
        let lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed.get(), Lim::new().above(0.0))?;
        let sample_count = gcmd.get_int("SAMPLES", self.samples.get(), Lim::new().min(1.0))?;
        let sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST", self.retract_dist.get(), Lim::new().above(0.0))?;
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let pos = toolhead.get_position();
        self.move_safe_z(&pos, lift_speed)?;
        // move to z-endstop position
        let nozzle = self.nozzle_site.borrow().clone().unwrap_or_default();
        self.do_move(&nozzle, self.speed)?;
        let pos = toolhead.get_position();
        gcmd.respond_info(
            &format!(
                "PROBE_ACCURACY at X:{:.3} Y:{:.3} Z:{:.3} (samples={} retract={:.3} speed={:.1} lift_speed={:.1})\n",
                pos[0], pos[1], pos[2], sample_count, sample_retract_dist, speed, lift_speed
            ),
            true,
        );
        // Probe bed sample_count times
        let mut positions: Vec<Vec<f64>> = vec![];
        let z_endstop = self.z_endstop()?;
        while (positions.len() as i64) < sample_count {
            // Probe position
            let pos = self.probe(&z_endstop, need(self.position_min.get())?, speed, false)?;
            // Retract
            let liftpos = [None, None, Some(pos[2] + sample_retract_dist)];
            positions.push(pos);
            self.do_move(&liftpos, lift_speed)?;
        }
        // Calculate maximum, minimum and average values
        let max_value = positions.iter().map(|p| p[2]).reduce(py_max).unwrap_or(0.0);
        let min_value = positions.iter().map(|p| p[2]).reduce(py_min).unwrap_or(0.0);
        let range_value = max_value - min_value;
        let avg_value = calc_mean(&positions)[2];
        let median = calc_median(&positions)[2];
        // calculate the standard deviation
        let mut deviation_sum = 0.0;
        for p in &positions {
            deviation_sum += (p[2] - avg_value).powf(2.0);
        }
        let sigma = (deviation_sum / positions.len() as f64).powf(0.5);
        // Show information
        gcmd.respond_info(
            &format!(
                "probe accuracy results: maximum {:.6}, minimum {:.6}, range {:.6}, average {:.6}, median {:.6}, standard deviation {:.6}",
                max_value, min_value, range_value, avg_value, median, sigma
            ),
            true,
        );
        Ok(())
    }

    fn cmd_calculate_switch_offset(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let pos = toolhead.get_position();
        let new_switch_offset = self.switch_offset - (pos[2] - self.last_z_offset.get());
        if new_switch_offset > 0.0 {
            gcmd.respond_info(
                &format!(
                    "switch_offset={:.3} - (current_z={:.3} - z_offset={:.3}) --> new switch_offset={:.3}",
                    self.switch_offset,
                    pos[2],
                    self.last_z_offset.get(),
                    new_switch_offset
                ),
                true,
            );
        } else {
            gcmd.respond_info(
                "The resulting switch offset is negative! Either the nozzle is still too far away or something else is wrong...",
                true,
            );
        }
        Ok(())
    }

    fn z_endstop(&self) -> KResult<EndstopRef> {
        self.z_endstop.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_mcu'".into()))
    }

    fn probe(&self, mcu_endstop: &EndstopRef, z_position: f64, speed: f64, wiggle: bool) -> KResult<Vec<f64>> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let mut pos = toolhead.get_position();
        pos[2] = z_position;
        // probe
        let phoming = self.printer.lookup::<PrinterHoming>("homing")?;
        let curpos = phoming.probing_move(mcu_endstop.clone(), &pos, speed)?;
        // retract
        self.do_move(&[None, None, Some(curpos[2] + need(self.retract_dist.get())?)], need(self.lift_speed.get())?)?;
        if wiggle {
            if let Some(w) = &self.wiggle_offsets {
                self.do_move(&[Some(curpos[0] + w[0].unwrap_or(0.0)), Some(curpos[1] + w[1].unwrap_or(0.0)), None], self.speed)?;
                self.do_move(&[Some(curpos[0]), Some(curpos[1]), None], self.speed)?;
            }
        }
        self.gcode.respond_info(&format!("probe at {:.3},{:.3} is z={:.6}", curpos[0], curpos[1], curpos[2]), true);
        Ok(curpos)
    }

    fn do_move(&self, coord: &[Option<f64>], speed: f64) -> KResult<()> {
        self.printer.lookup::<ToolHead>("toolhead")?.manual_move(coord, speed)
    }

    fn move_safe_z(&self, pos: &[f64], lift_speed: f64) -> KResult<()> {
        // clearance is deprecated
        if let Some(clearance) = self.clearance.get() {
            if pos[2] < clearance {
                // no clearance, better to move up (relative)
                self.do_move(&[None, None, Some(pos[2] + clearance)], lift_speed)?;
            }
        } else {
            let safe_z_height = need(self.safe_z_height.get())?;
            if pos[2] < safe_z_height {
                // no safe z position, better to move up (absolute)
                self.do_move(&[None, None, Some(safe_z_height)], lift_speed)?;
            }
        }
        Ok(())
    }

    fn log_config(&self) -> KResult<()> {
        let samples_result = self.samples_result.borrow().clone().unwrap_or_else(|| "None".into());
        let (nozzle, switch, bed) = (self.nozzle_site.borrow().clone(), self.switch_site.borrow().clone(), self.bed_site.borrow().clone());
        let msg = format!(
            "Z-CALIBRATION: switch_offset={:.3}, offset_margins={:.3},{:.3}, speed={:.3}, samples={}, tolerance={:.3}, retries={}, samples_result={}, lift_speed={:.3}, safe_z_height={:.3}, probing_speed={:.3}, second_speed={:.3}, retract_dist={:.3}, position_min={:.3}, probe_nozzle_x={:.3}, probe_nozzle_y={:.3}, probe_switch_x={:.3}, probe_switch_y={:.3}, probe_bed_x={:.3}, probe_bed_y={:.3}",
            self.switch_offset,
            self.offset_margins[0],
            self.offset_margins[1],
            self.speed,
            self.samples.get().ok_or_else(|| KError::Other("%i format: a real number is required, not NoneType".into()))?,
            need(self.tolerance.get())?,
            self.retries.get().ok_or_else(|| KError::Other("%i format: a real number is required, not NoneType".into()))?,
            samples_result,
            need(self.lift_speed.get())?,
            need(self.safe_z_height.get())?,
            need(self.probing_speed.get())?,
            need(self.second_speed.get())?,
            need(self.retract_dist.get())?,
            need(self.position_min.get())?,
            site_val(&nozzle, 0)?,
            site_val(&nozzle, 1)?,
            site_val(&switch, 0)?,
            site_val(&switch, 1)?,
            site_val(&bed, 0)?,
            site_val(&bed, 1)?,
        );
        logging::debug(&msg);
        Ok(())
    }
}

impl PrinterObject for ZCalibrationHelper {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let d = crate::pydict! {
            "last_query" => PyVal::Bool(self.last_state.get()),
            "last_z_offset" => PyVal::Float(self.last_z_offset.get()),
        };
        Some(PyVal::Dict(d))
    }
}

fn get_xy(config: &ConfigWrapper, name: &str, optional: bool) -> KResult<Option<Site>> {
    if optional && config.get_opt(name)?.is_none() {
        return Ok(None);
    }
    let site = config.get(name, Dflt::Required)?;
    Ok(Some(parse_xy(config, name, &site)?))
}

fn parse_xy(config: &ConfigWrapper, name: &str, site: &str) -> KResult<Site> {
    let parts: Vec<&str> = site.split(',').collect();
    let cause = match crate::pyfmt::py_unpack_error(2, parts.len()) {
        Some(e) => e,
        None => match (py_float(parts[0]), py_float(parts[1])) {
            (Some(x), Some(y)) => return Ok(vec![Some(x), Some(y), None]),
            (None, _) => crate::pyfmt::py_float_error(parts[0]),
            (_, None) => crate::pyfmt::py_float_error(parts[1]),
        },
    };
    Err(KError::config(format!("Unable to parse {} in {}", name, config.get_name())).with_context(KError::Py("ValueError".into(), cause)))
}

fn get_offset_margins(config: &ConfigWrapper, name: &str, default: &str) -> KResult<Vec<f64>> {
    let err = |cause: KError| KError::config(format!("Unable to parse {} in {}", name, config.get_name())).with_context(cause);
    let raw = config.get(name, default).map_err(|e| err(e.into()))?;
    let mut margins = vec![];
    for val in raw.split(',') {
        margins.push(py_float(val).ok_or_else(|| err(KError::Py("ValueError".into(), crate::pyfmt::py_float_error(val))))?);
    }
    if margins.len() == 1 {
        let val = margins[0].abs();
        margins[0] = -val;
        margins.push(val);
    }
    Ok(margins)
}

fn calc_mean(positions: &[Vec<f64>]) -> Vec<f64> {
    let count = positions.len() as f64;
    (0..3).map(|i| py_sum(positions.iter().map(|pos| pos[i])) / count).collect()
}

fn calc_median(positions: &[Vec<f64>]) -> Vec<f64> {
    let mut z_sorted = positions.to_vec();
    z_sorted.sort_by(|a, b| a[2].partial_cmp(&b[2]).unwrap_or(std::cmp::Ordering::Equal));
    let middle = positions.len() / 2;
    if positions.len() & 1 == 1 {
        // odd number of samples
        return z_sorted[middle].clone();
    }
    // even number of samples
    calc_mean(&z_sorted[middle - 1..middle + 1])
}

struct CalibrationState<'a> {
    helper: &'a ZCalibrationHelper,
    gcmd: &'a GCodeCommand,
    gcode: Rc<GCodeDispatch>,
    z_endstop: Option<EndstopRef>,
    probe: Rc<PrinterProbe>,
    toolhead: Rc<ToolHead>,
    gcode_move: Rc<GCodeMove>,
    max_deviation: Option<f64>,
    offset_margins: Vec<f64>,
}

impl<'a> CalibrationState<'a> {
    fn new(probe: Rc<PrinterProbe>, helper: &'a ZCalibrationHelper, gcmd: &'a GCodeCommand) -> KResult<Self> {
        Ok(CalibrationState {
            helper,
            gcmd,
            gcode: helper.gcode.clone(),
            z_endstop: helper.z_endstop.borrow().clone(),
            probe,
            toolhead: helper.printer.lookup::<ToolHead>("toolhead")?,
            gcode_move: helper.printer.lookup::<GCodeMove>("gcode_move")?,
            max_deviation: helper.max_deviation,
            offset_margins: helper.offset_margins.clone(),
        })
    }

    fn z_endstop(&self) -> KResult<EndstopRef> {
        self.z_endstop.clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_mcu'".into()))
    }

    fn probe_on_site(&self, endstop: &EndstopRef, site: &[Option<f64>], check_probe: bool, split_xy: bool, wiggle: bool) -> KResult<f64> {
        let h = self.helper;
        let pos = self.toolhead.get_position();
        h.move_safe_z(&pos, need(h.lift_speed.get())?)?;
        // move to position
        if split_xy {
            h.do_move(&[site[0], Some(pos[1]), None], h.speed)?;
            h.do_move(&[site[0], site[1], site.get(2).copied().flatten()], h.speed)?;
        } else {
            h.do_move(site, h.speed)?;
        }
        if check_probe {
            // check if probe is attached and switch is closed
            let time = self.toolhead.get_last_move_time()?;
            if self.probe.mcu_probe.query_endstop(time)? != 0 {
                return Err(KError::command("Probe switch not closed - Probe not attached?"));
            }
        }
        let position_min = need(h.position_min.get())?;
        if h.first_fast {
            // first probe just to get down faster
            h.probe(endstop, position_min, need(h.probing_speed.get())?, wiggle)?;
        }
        let mut retries = 0;
        let mut positions: Vec<Vec<f64>> = vec![];
        let samples = h.samples.get().unwrap_or(0);
        while (positions.len() as i64) < samples {
            // probe with second probing speed
            let curpos = h.probe(endstop, position_min, need(h.second_speed.get())?, wiggle)?;
            positions.push(curpos[..3].to_vec());
            // check tolerance
            let max_z = positions.iter().map(|p| p[2]).reduce(py_max).unwrap_or(0.0);
            let min_z = positions.iter().map(|p| p[2]).reduce(py_min).unwrap_or(0.0);
            if max_z - min_z > need(h.tolerance.get())? {
                if retries >= h.retries.get().unwrap_or(0) {
                    h.end_gcode.run_gcode_from_command(None)?;
                    return Err(self.gcmd.error("Probe samples exceed tolerance"));
                }
                self.gcmd.respond_info("Probe samples exceed tolerance. Retrying...", true);
                retries += 1;
                positions.clear();
            }
        }
        // calculate result
        if h.samples_result.borrow().as_deref() == Some("median") {
            return Ok(calc_median(&positions)[2]);
        }
        Ok(calc_mean(&positions)[2])
    }

    fn add_probe_offset(&self, site: &[Option<f64>]) -> Site {
        // calculate bed position by using the probe's offsets
        let probe_offsets = self.probe.get_offsets();
        let mut probe_site = site.to_vec();
        probe_site[0] = probe_site[0].map(|v| v - probe_offsets.0);
        probe_site[1] = probe_site[1].map(|v| v - probe_offsets.1);
        probe_site
    }

    fn set_new_gcode_offset(&self, offset: f64) -> KResult<()> {
        // reset gcode z offset to 0
        let mut params = IndexMap::default();
        params.insert("Z".into(), "0.0".to_string());
        let gcmd_offset = self.gcode.create_gcode_command("SET_GCODE_OFFSET", "SET_GCODE_OFFSET", params);
        self.gcode_move.cmd_set_gcode_offset(&gcmd_offset)?;
        // set new gcode z offset
        let mut params = IndexMap::default();
        params.insert("Z_ADJUST".into(), py_float_repr(offset));
        let gcmd_offset = self.gcode.create_gcode_command("SET_GCODE_OFFSET", "SET_GCODE_OFFSET", params);
        self.gcode_move.cmd_set_gcode_offset(&gcmd_offset)
    }

    fn calibrate_z(&self) -> KResult<()> {
        let h = self.helper;
        h.start_gcode.run_gcode_from_command(None)?;
        // probe the nozzle
        let nozzle_site = h.nozzle_site.borrow().clone().unwrap_or_default();
        let nozzle_zero = self.probe_on_site(&self.z_endstop()?, &nozzle_site, false, true, true)?;
        // probe the probe-switch
        h.switch_gcode.run_gcode_from_command(None)?;
        self.probe.multi_probe_begin(false)?;
        // probe the body of the switch
        let switch_site = h.switch_site.borrow().clone().unwrap_or_default();
        let switch_zero = self.probe_on_site(&self.z_endstop()?, &switch_site, true, false, false)?;
        // probe position on bed
        let bed_site = h.bed_site.borrow().clone().unwrap_or_default();
        let probe_site = self.add_probe_offset(&bed_site);
        let mcu_probe: EndstopRef = self.probe.mcu_probe.clone();
        let probe_zero = self.probe_on_site(&mcu_probe, &probe_site, true, false, false)?;
        self.probe.multi_probe_end()?;
        // calculate the offset
        let offset = probe_zero - (switch_zero - nozzle_zero + h.switch_offset);
        // print result
        self.gcmd.respond_info(
            &format!(
                "Z-CALIBRATION: probe={:.3} - (switch={:.3} - nozzle={:.3} + switch_offset={:.3}) --> new offset={:.6}",
                probe_zero, switch_zero, nozzle_zero, h.switch_offset, offset
            ),
            true,
        );
        let position_z_endstop = need(h.position_z_endstop.get())?;
        self.gcmd.respond_info(
            &format!(
                "HINT: z position_endstop={:.3} - offset={:.6} --> possible z position_endstop={:.3}",
                position_z_endstop,
                offset,
                position_z_endstop - offset
            ),
            true,
        );
        // check offset margins
        if let Some(max_deviation) = self.max_deviation.filter(|m| offset.abs() > *m) {
            // (max_deviation is deprecated)
            h.end_gcode.run_gcode_from_command(None)?;
            return Err(KError::command(format!(
                "Offset is greater than allowed: offset={:.3} > max_deviation={:.3}",
                offset, max_deviation
            )));
        } else if offset < self.offset_margins[0] || offset > self.offset_margins[1] {
            h.end_gcode.run_gcode_from_command(None)?;
            return Err(KError::command(format!(
                "Offset {:.3} is outside the configured range of min={:.3} and max={:.3}",
                offset, self.offset_margins[0], self.offset_margins[1]
            )));
        }
        // set new offset
        self.set_new_gcode_offset(offset)?;
        // set states
        h.last_state.set(true);
        h.last_z_offset.set(offset);
        h.end_gcode.run_gcode_from_command(None)
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ZCalibrationHelper::new(config)?)
}
