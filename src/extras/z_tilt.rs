// Mechanical bed tilt calibration with multiple Z steppers
// (port of klippy/extras/z_tilt.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim, ListParser};
use crate::error::{KError, KResult};
use crate::extras::probe::{FinalizeResult, ProbePointsHelper};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mathutil::coordinate_descent;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float_repr, py_max, py_min, py_round};
use crate::pyval::PyVal;
use crate::stepper::McuStepper;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// getlists(option, seps=(",", "\n"), parser=float, count=2) as pairs
pub fn get_pairs(config: &ConfigWrapper, option: &str) -> KResult<Vec<(f64, f64)>> {
    let raw = config.getlists(option, Dflt::Required, &[",", "\n"], Some(2), ListParser::Float, true)?;
    Ok(raw
        .as_ref()
        .and_then(|v| v.items())
        .unwrap_or(&[])
        .iter()
        .map(|p| {
            let i = p.items().unwrap_or(&[]);
            (i[0].as_f64().unwrap_or(0.0), i[1].as_f64().unwrap_or(0.0))
        })
        .collect())
}

/// repr() of a list of float lists
pub fn positions_repr(positions: &[Vec<f64>]) -> String {
    PyVal::List(positions.iter().map(|p| PyVal::List(p.iter().map(|v| PyVal::Float(*v)).collect())).collect()).repr()
}

pub struct ZAdjustHelper {
    printer: Rc<Printer>,
    name: String,
    z_count: usize,
    z_steppers: RefCell<Vec<Rc<McuStepper>>>,
}

impl ZAdjustHelper {
    pub fn new(config: &ConfigWrapper, z_count: usize) -> Rc<Self> {
        let printer = config.get_printer();
        let h = Rc::new(ZAdjustHelper { printer: printer.clone(), name: config.get_name().to_string(), z_count, z_steppers: RefCell::new(vec![]) });
        let hh = h.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| hh.handle_connect()));
        h
    }

    fn handle_connect(&self) -> KResult<()> {
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        let z_steppers: Vec<Rc<McuStepper>> = kin.get_steppers().into_iter().filter(|s| s.is_active_axis('z')).collect();
        if z_steppers.len() != self.z_count {
            return Err(KError::config(format!("{} z_positions needs exactly {} items", self.name, z_steppers.len())));
        }
        if z_steppers.len() < 2 {
            return Err(KError::config(format!("{} requires multiple z steppers", self.name)));
        }
        *self.z_steppers.borrow_mut() = z_steppers;
        Ok(())
    }

    pub fn adjust_steppers(&self, adjustments: &[f64], speed: f64) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        let mut curpos = toolhead.get_position();
        let z_steppers = self.z_steppers.borrow().clone();
        // Report on movements
        let stepstrs: Vec<String> = z_steppers.iter().zip(adjustments).map(|(s, a)| format!("{} = {:.6}", s.get_name(false), a)).collect();
        let msg = format!("Making the following Z adjustments:\n{}", stepstrs.join("\n"));
        gcode.respond_info(&msg, true);
        // Disable Z stepper movements
        toolhead.flush_step_generation()?;
        for s in &z_steppers {
            s.set_trapq(None);
        }
        // Move each z stepper (sorted from lowest to highest) until they match
        let mut positions: Vec<(f64, Rc<McuStepper>)> = adjustments.iter().zip(&z_steppers).map(|(a, s)| (-a, s.clone())).collect();
        positions.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let first_stepper_offset = positions[0].0;
        let z_low = curpos[2] - first_stepper_offset;
        for i in 0..positions.len() - 1 {
            let stepper = &positions[i].1;
            let next_stepper_offset = positions[i + 1].0;
            toolhead.flush_step_generation()?;
            stepper.set_trapq(Some(toolhead.get_trapq()));
            curpos[2] = z_low + next_stepper_offset;
            let r = toolhead.move_to(&curpos, speed).and_then(|_| toolhead.set_position(&curpos, ""));
            if let Err(e) = r {
                logging::exception_err("ZAdjustHelper adjust_steppers", &e);
                toolhead.flush_step_generation()?;
                for s in &z_steppers {
                    s.set_trapq(Some(toolhead.get_trapq()));
                }
                return Err(e);
            }
        }
        // Z should now be level - do final cleanup
        let last_stepper = &positions[positions.len() - 1].1;
        toolhead.flush_step_generation()?;
        last_stepper.set_trapq(Some(toolhead.get_trapq()));
        curpos[2] += first_stepper_offset;
        toolhead.set_position(&curpos, "")
    }
}

/// what RetryHelper.check_retry returns: "done" or a float
#[derive(Clone, Copy)]
pub enum RetryResult {
    Done,
    Value(f64),
}

pub struct ZAdjustStatus {
    applied: Cell<bool>,
}

impl ZAdjustStatus {
    pub fn new(printer: &Rc<Printer>) -> Rc<Self> {
        let s = Rc::new(ZAdjustStatus { applied: Cell::new(false) });
        let ss = s.clone();
        printer.register_event_handler(
            "stepper_enable:motor_off",
            Rc::new(move |_| {
                ss.reset();
                Ok(())
            }),
        );
        s
    }

    pub fn check_retry_result(&self, retry_result: RetryResult) -> FinalizeResult {
        match retry_result {
            RetryResult::Done => {
                self.applied.set(true);
                FinalizeResult::Done
            }
            RetryResult::Value(v) => {
                if v == 0.0 {
                    self.applied.set(true);
                }
                FinalizeResult::Num(v)
            }
        }
    }

    pub fn reset(&self) {
        self.applied.set(false);
    }

    pub fn get_status(&self) -> PyVal {
        let mut d = IndexMap::new();
        d.insert("applied".to_string(), PyVal::Bool(self.applied.get()));
        PyVal::Dict(d)
    }
}

pub struct RetryHelper {
    gcode: Rc<GCodeDispatch>,
    default_max_retries: i64,
    default_retry_tolerance: f64,
    default_increasing_threshold: f64,
    value_label: String,
    error_msg_extra: String,
    max_retries: Cell<i64>,
    retry_tolerance: Cell<f64>,
    increasing_threshold: Cell<f64>,
    current_retry: Cell<i64>,
    previous: Cell<Option<f64>>,
    increasing: Cell<i64>,
}

impl RetryHelper {
    pub fn new(config: &ConfigWrapper, error_msg_extra: &str) -> KResult<Self> {
        let gcode = config.get_printer().lookup::<GCodeDispatch>("gcode")?;
        Ok(RetryHelper {
            gcode,
            default_max_retries: config.getint("retries", 0, Lim::new().min(0))?,
            default_retry_tolerance: config.getfloat("retry_tolerance", 0.0, Lim::new().above(0.0))?,
            default_increasing_threshold: config.getfloat("increasing_threshold", 0.0000001, Lim::new().above(0.0))?,
            value_label: "Probed points range".into(),
            error_msg_extra: error_msg_extra.into(),
            max_retries: Cell::new(0),
            retry_tolerance: Cell::new(0.0),
            increasing_threshold: Cell::new(0.0),
            current_retry: Cell::new(0),
            previous: Cell::new(None),
            increasing: Cell::new(0),
        })
    }

    pub fn start(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.max_retries.set(gcmd.get_int("RETRIES", Some(self.default_max_retries), Lim::new().min(0).max(30))?);
        self.retry_tolerance.set(gcmd.get_float("RETRY_TOLERANCE", Some(self.default_retry_tolerance), Lim::new().min(0.0).max(1.0))?);
        self.increasing_threshold.set(gcmd.get_float(
            "INCREASING_THRESHOLD",
            Some(self.default_increasing_threshold),
            Lim::new().above(0.0),
        )?);
        self.current_retry.set(0);
        self.previous.set(None);
        self.increasing.set(0);
        Ok(())
    }

    fn check_increase(&self, error: f64) -> bool {
        match self.previous.get() {
            Some(p) if p != 0.0 && error > p + self.increasing_threshold.get() => self.increasing.set(self.increasing.get() + 1),
            _ if self.increasing.get() > 0 => self.increasing.set(self.increasing.get() - 1),
            _ => {}
        }
        self.previous.set(Some(error));
        self.increasing.get() > 1
    }

    pub fn check_retry(&self, z_positions: &[f64]) -> KResult<RetryResult> {
        if self.max_retries.get() == 0 {
            return Ok(RetryResult::Done);
        }
        let zmax = z_positions[1..].iter().fold(z_positions[0], |a, &b| py_max(a, b));
        let zmin = z_positions[1..].iter().fold(z_positions[0], |a, &b| py_min(a, b));
        let error = py_round(zmax - zmin, 6);
        self.gcode.respond_info(
            &format!(
                "Retries: {}/{} {}: {:.6} tolerance: {:.6}",
                self.current_retry.get(),
                self.max_retries.get(),
                self.value_label,
                error,
                self.retry_tolerance.get()
            ),
            true,
        );
        if self.check_increase(error) {
            return Err(KError::command(format!("Retries aborting: {} is increasing. {}", self.value_label, self.error_msg_extra)));
        }
        if error <= self.retry_tolerance.get() {
            return Ok(RetryResult::Value(0.0));
        }
        self.current_retry.set(self.current_retry.get() + 1);
        if self.current_retry.get() > self.max_retries.get() {
            return Err(KError::command("Too many retries"));
        }
        Ok(RetryResult::Value(error))
    }
}

const CMD_Z_TILT_ADJUST_HELP: &str = "Adjust the Z tilt";

pub struct ZTilt {
    z_positions: Vec<(f64, f64)>,
    retry_helper: RetryHelper,
    probe_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    z_status: Rc<ZAdjustStatus>,
    z_helper: Rc<ZAdjustHelper>,
    use_adjustments: bool,
}

impl ZTilt {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let z_positions = get_pairs(config, "z_positions")?;
        let retry_helper = RetryHelper::new(config, "")?;
        let slot: Rc<RefCell<Weak<ZTilt>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        let probe_helper = ProbePointsHelper::new(
            config,
            Rc::new(move |offsets, positions| match sl.borrow().upgrade() {
                Some(z) => z.probe_finalize(offsets, positions),
                None => Ok(FinalizeResult::Done),
            }),
            None,
            "points",
            false,
            false,
        )?;
        probe_helper.minimum_points(2)?;
        let z_status = ZAdjustStatus::new(&printer);
        let z_helper = ZAdjustHelper::new(config, z_positions.len());
        let use_adjustments = config.getboolean("use_adjustments", false)?;
        let zt = Rc::new(ZTilt {
            z_positions,
            retry_helper,
            probe_helper: RefCell::new(Some(probe_helper)),
            z_status,
            z_helper,
            use_adjustments,
        });
        *slot.borrow_mut() = Rc::downgrade(&zt);
        // Register Z_TILT_ADJUST command
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let z = zt.clone();
        gcode.register_command("Z_TILT_ADJUST", Some(Rc::new(move |g| z.cmd_z_tilt_adjust(g))), false, Some(CMD_Z_TILT_ADJUST_HELP))?;
        Ok(zt)
    }

    fn probe_helper(&self) -> Rc<ProbePointsHelper> {
        self.probe_helper.borrow().clone().expect("probe helper")
    }

    fn cmd_z_tilt_adjust(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.z_status.reset();
        self.retry_helper.start(gcmd)?;
        self.probe_helper().start_probe(gcmd)
    }

    fn probe_finalize(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        // Setup for coordinate descent analysis
        let z_offset = offsets.2;
        logging::info(&format!("Calculating bed tilt with: {}", positions_repr(positions)));
        let mut params = IndexMap::new();
        params.insert("x_adjust".to_string(), 0.0);
        params.insert("y_adjust".to_string(), 0.0);
        params.insert("z_adjust".to_string(), z_offset);

        // Perform coordinate descent
        let adjusted_height = |pos: &[f64], params: &IndexMap<String, f64>| {
            let (x, y, z) = (pos[0], pos[1], pos[2]);
            z - x * params["x_adjust"] - y * params["y_adjust"] - params["z_adjust"]
        };
        let errorfunc = |params: &IndexMap<String, f64>| {
            let mut total_error = 0.0;
            for pos in positions {
                total_error += adjusted_height(pos, params).powf(2.0);
            }
            total_error
        };
        let keys: Vec<String> = params.keys().cloned().collect();
        let new_params = coordinate_descent(&keys, &params, &errorfunc);
        // Apply results
        let speed = self.probe_helper().get_lift_speed(None)?;
        let pr: Vec<String> = new_params.iter().map(|(k, v)| format!("'{}': {}", k, py_float_repr(*v))).collect();
        logging::info(&format!("Calculated bed tilt parameters: {{{}}}", pr.join(", ")));
        let x_adjust = new_params["x_adjust"];
        let y_adjust = new_params["y_adjust"];
        let z_adjust = new_params["z_adjust"] - z_offset - x_adjust * offsets.0 - y_adjust * offsets.1;
        let adjustments: Vec<f64> = self.z_positions.iter().map(|(x, y)| x * x_adjust + y * y_adjust + z_adjust).collect();
        self.z_helper.adjust_steppers(&adjustments, speed)?;
        let zs: Vec<f64> = if self.use_adjustments { adjustments } else { positions.iter().map(|p| p[2]).collect() };
        Ok(self.z_status.check_retry_result(self.retry_helper.check_retry(&zs)?))
    }
}

impl PrinterObject for ZTilt {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(self.z_status.get_status())
    }
    fn teardown(&self) {
        *self.probe_helper.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ZTilt::new(config)?)
}
