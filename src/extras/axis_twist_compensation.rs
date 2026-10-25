// Axis Twist Compensation (port of klippy/extras/axis_twist_compensation.py)
//
// Copyright (C) 2022  Jeremy Tan <jeremytkw98@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim, ListParser};
use crate::error::{KError, KResult};
use crate::extras::bed_mesh::{constrain, lerp};
use crate::extras::manual_probe::{verify_no_manual_probe, ManualProbeHelper};
use crate::extras::probe::{as_printer_probe, probe_list_get_list, PrinterProbe};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float_repr, py_sum};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const DEFAULT_SAMPLE_COUNT: i64 = 3;
const DEFAULT_SPEED: f64 = 50.0;
const DEFAULT_HORIZONTAL_MOVE_Z: f64 = 5.0;

/// config.getlists(option, default=[], parser=float)
fn float_lists(config: &ConfigWrapper, option: &str) -> KResult<Vec<f64>> {
    let v = config.getlists(option, PyVal::List(vec![]), &[","], None, ListParser::Float, true)?;
    Ok(v.as_ref().and_then(|v| v.items()).unwrap_or(&[]).iter().filter_map(|x| x.as_f64()).collect())
}

pub struct AxisTwistCompensation {
    horizontal_move_z: f64,
    speed: f64,
    calibrate_start_x: f64,
    calibrate_end_x: f64,
    calibrate_y: f64,
    z_compensations: RefCell<Vec<f64>>,
    compensation_start_x: Cell<Option<f64>>,
    compensation_end_x: Cell<Option<f64>>,
    calibrate_start_y: Option<f64>,
    calibrate_end_y: Option<f64>,
    calibrate_x: Option<f64>,
    compensation_start_y: Cell<Option<f64>>,
    compensation_end_y: Cell<Option<f64>>,
    zy_compensations: RefCell<Vec<f64>>,
    calibrater: RefCell<Option<Rc<Calibrater>>>,
}

impl AxisTwistCompensation {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        // get values from [axis_twist_compensation] section in printer .cfg
        let horizontal_move_z = config.getfloat("horizontal_move_z", DEFAULT_HORIZONTAL_MOVE_Z, Lim::new())?;
        let speed = config.getfloat("speed", DEFAULT_SPEED, Lim::new())?;
        let calibrate_start_x = config.getfloat("calibrate_start_x", Dflt::Required, Lim::new())?;
        let calibrate_end_x = config.getfloat("calibrate_end_x", Dflt::Required, Lim::new())?;
        let calibrate_y = config.getfloat("calibrate_y", Dflt::Required, Lim::new())?;
        let z_compensations = float_lists(config, "z_compensations")?;
        let compensation_start_x = config.getfloat_opt("compensation_start_x", Lim::new())?;
        let compensation_end_x = config.getfloat_opt("compensation_end_x", Lim::new())?;
        let calibrate_start_y = config.getfloat_opt("calibrate_start_y", Lim::new())?;
        let calibrate_end_y = config.getfloat_opt("calibrate_end_y", Lim::new())?;
        let calibrate_x = config.getfloat_opt("calibrate_x", Lim::new())?;
        let compensation_start_y = config.getfloat_opt("compensation_start_y", Lim::new())?;
        let compensation_end_y = config.getfloat_opt("compensation_end_y", Lim::new())?;
        let zy_compensations = float_lists(config, "zy_compensations")?;
        // validate that compensation values have required start/end points
        if !z_compensations.is_empty() && (compensation_start_x.is_none() || compensation_end_x.is_none()) {
            return Err(KError::config("z_compensations requires compensation_start_x and compensation_end_x to be set"));
        }
        if !zy_compensations.is_empty() && (compensation_start_y.is_none() || compensation_end_y.is_none()) {
            return Err(KError::config("zy_compensations requires compensation_start_y and compensation_end_y to be set"));
        }
        let c = Rc::new(AxisTwistCompensation {
            horizontal_move_z,
            speed,
            calibrate_start_x,
            calibrate_end_x,
            calibrate_y,
            z_compensations: RefCell::new(z_compensations),
            compensation_start_x: Cell::new(compensation_start_x),
            compensation_end_x: Cell::new(compensation_end_x),
            calibrate_start_y,
            calibrate_end_y,
            calibrate_x,
            compensation_start_y: Cell::new(compensation_start_y),
            compensation_end_y: Cell::new(compensation_end_y),
            zy_compensations: RefCell::new(zy_compensations),
            calibrater: RefCell::new(None),
        });
        // setup calibrater
        *c.calibrater.borrow_mut() = Some(Calibrater::new(&c, config)?);
        Ok(c)
    }

    pub fn get_z_compensation_value(&self, pos: &[f64]) -> f64 {
        let mut total_interpolated_z_compensation = 0.0;
        let z = self.z_compensations.borrow();
        if !z.is_empty() {
            total_interpolated_z_compensation += Self::get_interpolated_z_compensation(
                pos[0],
                &z,
                self.compensation_start_x.get().unwrap_or(0.0),
                self.compensation_end_x.get().unwrap_or(0.0),
            );
        }
        let zy = self.zy_compensations.borrow();
        if !zy.is_empty() {
            total_interpolated_z_compensation += Self::get_interpolated_z_compensation(
                pos[1],
                &zy,
                self.compensation_start_y.get().unwrap_or(0.0),
                self.compensation_end_y.get().unwrap_or(0.0),
            );
        }
        total_interpolated_z_compensation
    }

    fn get_interpolated_z_compensation(coord: f64, z_compensations: &[f64], comp_start: f64, comp_end: f64) -> f64 {
        let sample_count = z_compensations.len();
        let spacing = (comp_end - comp_start) / (sample_count as f64 - 1.0);
        let mut interpolate_t = (coord - comp_start) / spacing;
        let interpolate_i = interpolate_t.floor();
        let interpolate_i = constrain(interpolate_i, 0.0, sample_count as f64 - 2.0) as usize;
        interpolate_t -= interpolate_i as f64;
        lerp(interpolate_t, z_compensations[interpolate_i], z_compensations[interpolate_i + 1])
    }

    fn clear_compensations(&self, axis: Option<&str>) {
        match axis {
            None => {
                self.z_compensations.borrow_mut().clear();
                self.zy_compensations.borrow_mut().clear();
            }
            Some("X") => self.z_compensations.borrow_mut().clear(),
            Some("Y") => self.zy_compensations.borrow_mut().clear(),
            _ => {}
        }
    }
}

impl PrinterObject for AxisTwistCompensation {
    fn teardown(&self) {
        *self.calibrater.borrow_mut() = None;
    }
}

const CMD_AXIS_TWIST_COMPENSATION_CALIBRATE_HELP: &str = "\n    Performs the x twist calibration wizard\n    Measure z probe offset at n points along the x axis,\n    and calculate x twist compensation\n    ";

pub struct Calibrater {
    me: Weak<Calibrater>,
    compensation: Weak<AxisTwistCompensation>,
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    probe: RefCell<Option<Rc<PrinterProbe>>>,
    lift_speed: Cell<f64>,
    speed: f64,
    horizontal_move_z: f64,
    x_start_point: (Option<f64>, Option<f64>),
    x_end_point: (Option<f64>, Option<f64>),
    y_start_point: (Option<f64>, Option<f64>),
    y_end_point: (Option<f64>, Option<f64>),
    results: RefCell<Vec<f64>>,
    current_point_index: Cell<usize>,
    current_measured_z: Cell<f64>,
    current_axis: RefCell<String>,
    configname: String,
}

impl Calibrater {
    fn new(compensation: &Rc<AxisTwistCompensation>, config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let c = Rc::new_cyclic(|me| Calibrater {
            me: me.clone(),
            compensation: Rc::downgrade(compensation),
            printer: printer.clone(),
            gcode: gcode.clone(),
            // probe settings are set to none, until they are available
            probe: RefCell::new(None),
            lift_speed: Cell::new(0.0),
            speed: compensation.speed,
            horizontal_move_z: compensation.horizontal_move_z,
            x_start_point: (Some(compensation.calibrate_start_x), Some(compensation.calibrate_y)),
            x_end_point: (Some(compensation.calibrate_end_x), Some(compensation.calibrate_y)),
            y_start_point: (compensation.calibrate_x, compensation.calibrate_start_y),
            y_end_point: (compensation.calibrate_x, compensation.calibrate_end_y),
            results: RefCell::new(vec![]),
            current_point_index: Cell::new(0),
            current_measured_z: Cell::new(0.0),
            current_axis: RefCell::new(String::new()),
            configname: config.get_name().to_string(),
        });
        // register gcode handlers
        let w = c.me.clone();
        gcode.register_command(
            "AXIS_TWIST_COMPENSATION_CALIBRATE",
            Some(Rc::new(move |g| match w.upgrade() {
                Some(c) => c.cmd_axis_twist_compensation_calibrate(g),
                None => Ok(()),
            })),
            false,
            Some(CMD_AXIS_TWIST_COMPENSATION_CALIBRATE_HELP),
        )?;
        Ok(c)
    }

    fn compensation(&self) -> KResult<Rc<AxisTwistCompensation>> {
        self.compensation.upgrade().ok_or_else(|| KError::Other("axis_twist_compensation gone".into()))
    }

    fn cmd_axis_twist_compensation_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let plist = probe_list_get_list(&self.printer)?;
        let probe = plist.get_command_probe(gcmd, None, None)?.as_ref().and_then(as_printer_probe);
        let probe = probe.ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_lift_speed'".into()))?;
        self.lift_speed.set(probe.get_lift_speed(None)?);
        let (probe_x_offset, probe_y_offset, _) = probe.get_offsets();
        *self.probe.borrow_mut() = Some(probe);

        let sample_count = gcmd.get_int("SAMPLE_COUNT", Some(DEFAULT_SAMPLE_COUNT), Lim::new())?;
        let axis = gcmd.get_or("AXIS", "X").to_uppercase();

        // check for valid sample_count
        if sample_count < 2 {
            return Err(gcmd.error("SAMPLE_COUNT to probe must be at least 2"));
        }

        // calculate the points to put the probe at, returned as a list of tuples
        let mut nozzle_points = vec![];
        let interval_dist;
        let compensation = self.compensation()?;
        if axis == "X" {
            compensation.clear_compensations(Some("X"));
            let (Some(sx), Some(ex), Some(sy)) = (self.x_start_point.0, self.x_end_point.0, self.x_start_point.1) else {
                return Err(gcmd.error(
                    "AXIS_TWIST_COMPENSATION for X axis requires\n                    calibrate_start_x, calibrate_end_x and calibrate_y\n                    to be defined\n                    ",
                ));
            };
            let x_axis_range = ex - sx;
            interval_dist = x_axis_range / (sample_count - 1) as f64;
            for i in 0..sample_count {
                nozzle_points.push((sx + i as f64 * interval_dist, sy));
            }
        } else if axis == "Y" {
            compensation.clear_compensations(Some("Y"));
            let (Some(sx), Some(_ex), Some(sy)) = (self.y_start_point.0, self.y_end_point.0, self.y_start_point.1) else {
                return Err(gcmd.error(
                    "AXIS_TWIST_COMPENSATION for Y axis requires\n                    calibrate_start_y, calibrate_end_y and calibrate_x\n                    to be defined\n                    ",
                ));
            };
            let ey = self.y_end_point.1.ok_or_else(|| KError::Other("unsupported operand type(s) for -: 'NoneType' and 'float'".into()))?;
            let y_axis_range = ey - sy;
            interval_dist = y_axis_range / (sample_count - 1) as f64;
            for i in 0..sample_count {
                nozzle_points.push((sx, sy + i as f64 * interval_dist));
            }
        } else {
            return Err(gcmd.error("AXIS_TWIST_COMPENSATION_CALIBRATE: Invalid axis."));
        }

        let probe_points: Vec<(f64, f64)> = nozzle_points.iter().map(|(x, y)| (x - probe_x_offset, y - probe_y_offset)).collect();

        // verify no other manual probe is in progress
        verify_no_manual_probe(&self.printer)?;

        // begin calibration
        self.current_point_index.set(0);
        self.results.borrow_mut().clear();
        *self.current_axis.borrow_mut() = axis;
        self.calibration(gcmd, Rc::new(probe_points), Rc::new(nozzle_points))
    }

    fn move_helper(&self, target: [Option<f64>; 3], override_speed: Option<f64>) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let speed = if target[2].is_none() { self.speed } else { self.lift_speed.get() };
        toolhead.manual_move(&target, override_speed.unwrap_or(speed))
    }

    fn calibration(&self, gcmd: &GCodeCommand, probe_points: Rc<Vec<(f64, f64)>>, nozzle_points: Rc<Vec<(f64, f64)>>) -> KResult<()> {
        let idx = self.current_point_index.get();
        // begin the calibration process
        gcmd.respond_info(&format!("AXIS_TWIST_COMPENSATION_CALIBRATE: Probing point {} of {}", idx + 1, probe_points.len()), true);

        // horizontal_move_z (to prevent probe trigger or hitting bed)
        self.move_helper([None, None, Some(self.horizontal_move_z)], None)?;

        // move to point to probe
        self.move_helper([Some(probe_points[idx].0), Some(probe_points[idx].1), None], None)?;

        // probe the point
        let probe = self.probe.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'run_probe'".into()))?;
        let pos = probe.run_probe(gcmd, None)?;
        self.current_measured_z.set(pos[2]);

        // horizontal_move_z (to prevent probe trigger or hitting bed)
        self.move_helper([None, None, Some(self.horizontal_move_z)], None)?;

        // move the nozzle over the probe point
        self.move_helper([Some(nozzle_points[idx].0), Some(nozzle_points[idx].1), None], None)?;

        // start the manual (nozzle) probe
        let is_end = idx == probe_points.len() - 1;
        let (w, g) = (self.me.clone(), gcmd.clone());
        ManualProbeHelper::start(
            &self.printer,
            gcmd,
            Rc::new(move |kin_pos| match w.upgrade() {
                Some(c) => c.manual_probe_callback(&g, kin_pos, is_end, probe_points.clone(), nozzle_points.clone()),
                None => Ok(()),
            }),
        )
    }

    fn manual_probe_callback(
        &self,
        gcmd: &GCodeCommand,
        kin_pos: Option<Vec<f64>>,
        is_end: bool,
        probe_points: Rc<Vec<(f64, f64)>>,
        nozzle_points: Rc<Vec<(f64, f64)>>,
    ) -> KResult<()> {
        let Some(kin_pos) = kin_pos else {
            // probe was cancelled
            gcmd.respond_info("AXIS_TWIST_COMPENSATION_CALIBRATE: Probe cancelled, calibration aborted", true);
            return Ok(());
        };
        let z_offset = self.current_measured_z.get() - kin_pos[2];
        self.results.borrow_mut().push(z_offset);
        if is_end {
            // end of calibration
            self.finalize_calibration(gcmd)
        } else {
            // move to next point
            self.current_point_index.set(self.current_point_index.get() + 1);
            self.calibration(gcmd, probe_points, nozzle_points)
        }
    }

    fn finalize_calibration(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // finalize the calibration process
        // calculate average of results
        let results = self.results.borrow().clone();
        let avg = py_sum(results.iter().copied()) / results.len() as f64;
        // subtract average from each result
        // so that they are independent of z_offset
        let results: Vec<f64> = results.iter().map(|x| avg - x).collect();
        *self.results.borrow_mut() = results.clone();
        // save the config
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let values_as_str = results.iter().map(|x| format!("{:.6}", x)).collect::<Vec<_>>().join(", ");
        let compensation = self.compensation()?;
        let axis = self.current_axis.borrow().clone();
        {
            let mut cf = configfile.pconfig.borrow_mut();
            let opt = |v: Option<f64>| v.map(py_float_repr).unwrap_or_else(|| "None".into());
            if axis == "X" {
                cf.set(&self.configname, "z_compensations", &values_as_str)?;
                cf.set(&self.configname, "compensation_start_x", &opt(self.x_start_point.0))?;
                cf.set(&self.configname, "compensation_end_x", &opt(self.x_end_point.0))?;
                *compensation.z_compensations.borrow_mut() = results.clone();
                compensation.compensation_start_x.set(self.x_start_point.0);
                compensation.compensation_end_x.set(self.x_end_point.0);
            } else if axis == "Y" {
                cf.set(&self.configname, "zy_compensations", &values_as_str)?;
                cf.set(&self.configname, "compensation_start_y", &opt(self.y_start_point.1))?;
                cf.set(&self.configname, "compensation_end_y", &opt(self.y_end_point.1))?;
                *compensation.zy_compensations.borrow_mut() = results.clone();
                compensation.compensation_start_y.set(self.y_start_point.1);
                compensation.compensation_end_y.set(self.y_end_point.1);
            }
        }
        self.gcode.respond_info(
            "AXIS_TWIST_COMPENSATION state has been saved for the current session.  The SAVE_CONFIG command will update the printer config file and restart the printer.",
            true,
        );
        // output result
        let list: Vec<String> = results.iter().map(|v| py_float_repr(*v)).collect();
        gcmd.respond_info(
            &format!("AXIS_TWIST_COMPENSATION_CALIBRATE: Calibration complete, offsets: [{}], mean z_offset: {:.6}", list.join(", "), avg),
            true,
        );
        Ok(())
    }
}

// klipper's entry point using [axis_twist_compensation] section in printer.cfg
pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(AxisTwistCompensation::new(config)?)
}
