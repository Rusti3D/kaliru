// Printer stepper support (port of klippy/stepper.py)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
use crate::chelper::itersolve::{self, SkRef};
use crate::chelper::steppersync::SyncEmitterRef;
use crate::chelper::trapq::TrapQRef;
use crate::configfile::{ConfigWrapper, Dflt, Lim, ListParser, REQUIRED};
use crate::error::{KError, KResult};
use crate::extras::motion_queuing::{FlushCallback, PrinterMotionQueuing};
use crate::extras::force_move::ForceMove;
use crate::extras::motion_report::PrinterMotionReport;
use crate::extras::query_endstops::QueryEndstops;
use crate::extras::stepper_enable::PrinterStepperEnable;
use crate::mcu::{any_to_endstop, get_printer_mcu, pint, CommandQueryWrapper, EndstopRef, Mcu};
use crate::msgproto::Val;
use crate::pins::{PinsObject, SharedPinParams};
use crate::printer::{downcast_rc, Printer};
use crate::serialhdl::params_time;
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// stepper.error
fn error(msg: impl Into<String>) -> KError {
    KError::Other(msg.into())
}

pub const MIN_BOTH_EDGE_DURATION: f64 = 0.000000500;
pub const MIN_OPTIMIZED_BOTH_EDGE_DURATION: f64 = 0.000000150;
pub const MAX_STEPCOMPRESS_ERROR: f64 = 0.000025;

/// an active callback: cb(print_time)
pub type ActiveCallback = Rc<dyn Fn(f64) -> KResult<()>>;

// Interface to low-level mcu and chelper code
pub struct McuStepper {
    me: Weak<McuStepper>,
    printer: Rc<Printer>,
    name: String,
    rotation_dist: Cell<f64>,
    steps_per_rotation: f64,
    step_pulse_duration: Cell<Option<f64>>,
    units_in_radians: bool,
    step_dist: Cell<f64>,
    mcu: Rc<Mcu>,
    oid: i64,
    step_pin: String,
    invert_step: i64,
    dir_pin: String,
    invert_dir: Cell<bool>,
    orig_invert_dir: bool,
    step_both_edge: Cell<bool>,
    req_step_both_edge: Cell<bool>,
    mcu_position_offset: Cell<f64>,
    reset_cmd_tag: Cell<u32>,
    get_position_cmd: RefCell<Option<CommandQueryWrapper>>,
    active_callbacks: RefCell<Vec<ActiveCallback>>,
    check_active_cb: RefCell<Option<FlushCallback>>,
    syncemitter: SyncEmitterRef,
    stepper_kinematics: RefCell<Option<SkRef>>,
    trapq: RefCell<Option<TrapQRef>>,
    tmc_current_helper: RefCell<Option<Rc<dyn TmcCurrentHelper>>>,
}

impl McuStepper {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        config: &ConfigWrapper,
        step_pin_params: &SharedPinParams,
        dir_pin_params: &SharedPinParams,
        rotation_dist: f64,
        steps_per_rotation: f64,
        step_pulse_duration: Option<f64>,
        units_in_radians: bool,
    ) -> KResult<Rc<McuStepper>> {
        let printer = config.get_printer();
        let name = config.get_name().to_string();
        let (step_chip, step_pin, invert_step) = {
            let p = step_pin_params.borrow();
            (p.chip_name.clone(), p.pin.clone(), p.invert)
        };
        // the "chip" of a stepper pin is an mcu (python calls mcu methods on it)
        let mcu = get_printer_mcu(&printer, &step_chip)?;
        let oid = mcu.create_oid();
        let (dir_chip, dir_pin, invert_dir) = {
            let p = dir_pin_params.borrow();
            (p.chip_name.clone(), p.pin.clone(), p.invert != 0)
        };
        if dir_chip != step_chip {
            return Err(KError::config("Stepper dir pin must be on same mcu as step pin"));
        }
        let motion_queuing = printer
            .load_object(config, "motion_queuing", true)?
            .and_then(downcast_rc::<PrinterMotionQueuing>)
            .ok_or_else(|| KError::Other("motion_queuing has an unexpected type".into()))?;
        let sname = name.split_whitespace().last().unwrap_or("").to_string();
        let syncemitter = motion_queuing.allocate_syncemitter(&mcu, &sname, true);
        if let Some(sc) = syncemitter.get_stepcompress() {
            sc.borrow_mut().set_invert_sdir(invert_dir as u32);
        }
        let s = Rc::new_cyclic(|me| McuStepper {
            me: me.clone(),
            printer: printer.clone(),
            name,
            rotation_dist: Cell::new(rotation_dist),
            steps_per_rotation,
            step_pulse_duration: Cell::new(step_pulse_duration),
            units_in_radians,
            step_dist: Cell::new(rotation_dist / steps_per_rotation),
            mcu: mcu.clone(),
            oid,
            step_pin,
            invert_step,
            dir_pin,
            invert_dir: Cell::new(invert_dir),
            orig_invert_dir: invert_dir,
            step_both_edge: Cell::new(false),
            req_step_both_edge: Cell::new(false),
            mcu_position_offset: Cell::new(0.0),
            reset_cmd_tag: Cell::new(0),
            get_position_cmd: RefCell::new(None),
            active_callbacks: RefCell::new(vec![]),
            check_active_cb: RefCell::new(None),
            syncemitter,
            stepper_kinematics: RefCell::new(None),
            trapq: RefCell::new(None),
            tmc_current_helper: RefCell::new(None),
        });
        // python registers the config callback right after create_oid()
        let sc = s.clone();
        mcu.register_config_callback(Rc::new(move || sc.build_config()));
        let sc = s.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| sc.query_mcu_position()));
        Ok(s)
    }

    fn rc(&self) -> Rc<McuStepper> {
        self.me.upgrade().expect("stepper")
    }

    pub fn get_tmc_current_helper(&self) -> Option<Rc<dyn TmcCurrentHelper>> {
        self.tmc_current_helper.borrow().clone()
    }

    pub fn set_tmc_current_helper(&self, tmc_current_helper: Option<Rc<dyn TmcCurrentHelper>>) {
        *self.tmc_current_helper.borrow_mut() = tmc_current_helper;
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn get_name(&self, short: bool) -> String {
        if short {
            if let Some(n) = self.name.strip_prefix("stepper_") {
                return n.to_string();
            }
        }
        self.name.clone()
    }

    pub fn units_in_radians(&self) -> bool {
        // Returns true if distances are in radians instead of millimeters
        self.units_in_radians
    }

    pub fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (self.step_pulse_duration.get(), self.step_both_edge.get())
    }

    pub fn setup_default_pulse_duration(&self, pulse_duration: f64, step_both_edge: bool) {
        if self.step_pulse_duration.get().is_none() {
            self.step_pulse_duration.set(Some(pulse_duration));
        }
        self.req_step_both_edge.set(step_both_edge);
    }

    /// setup_itersolve(alloc_func, *params): the caller allocates the sk
    pub fn setup_itersolve(&self, sk: SkRef) {
        self.set_stepper_kinematics(Some(sk));
    }

    fn build_config(&self) -> KResult<()> {
        if self.step_pulse_duration.get().is_none() {
            self.step_pulse_duration.set(Some(0.000002));
        }
        let mut step_pulse_duration = self.step_pulse_duration.get().unwrap_or(0.);
        let mut invert_step = self.invert_step;
        // Check if can enable "step on both edges"
        let constants = self.mcu.get_constants();
        let constant = |name: &str| -> KResult<i64> {
            let v = constants.get(name).map(crate::pyfmt::py_str).unwrap_or_else(|| "0".into());
            crate::pyfmt::py_int(&v)
                .ok_or_else(|| KError::Other(format!("invalid literal for int() with base 10: {}", crate::pyfmt::py_str_repr(&v))))
        };
        let ssbe = constant("STEPPER_STEP_BOTH_EDGE")?;
        let sbe = constant("STEPPER_BOTH_EDGE")?;
        let sou = constant("STEPPER_OPTIMIZED_UNSTEP")?;
        let mut want_both_edges = self.req_step_both_edge.get();
        if step_pulse_duration > MIN_BOTH_EDGE_DURATION {
            // If user has requested a very large step pulse duration
            // then disable step on both edges (rise and fall times may
            // not be symetric)
            want_both_edges = false;
        } else if sbe != 0 && step_pulse_duration > MIN_OPTIMIZED_BOTH_EDGE_DURATION {
            // Older MCU and user has requested large pulse duration
            want_both_edges = false;
        } else if sbe == 0 && ssbe == 0 {
            // Older MCU that doesn't support step on both edges
            want_both_edges = false;
        } else if sou != 0 {
            // MCU has optimized step/unstep - better to use that
            want_both_edges = false;
        }
        if want_both_edges {
            self.step_both_edge.set(true);
            invert_step = -1;
            if sbe != 0 {
                // Older MCU requires setting step_pulse_ticks=0 to enable
                step_pulse_duration = 0.0;
                self.step_pulse_duration.set(Some(0.0));
            }
        }
        // Configure stepper object
        let step_pulse_ticks = self.mcu.seconds_to_clock(step_pulse_duration);
        self.mcu.add_config_cmd(
            &format!(
                "config_stepper oid={} step_pin={} dir_pin={} invert_step={} step_pulse_ticks={}",
                self.oid, self.step_pin, self.dir_pin, invert_step, step_pulse_ticks
            ),
            false,
            false,
        );
        self.mcu.add_config_cmd(&format!("reset_step_clock oid={} clock=0", self.oid), false, true);
        let step_cmd_tag = self.mcu.lookup_command("queue_step oid=%c interval=%u count=%hu add=%hi", None)?.get_command_tag();
        let dir_cmd_tag = self.mcu.lookup_command("set_next_step_dir oid=%c dir=%c", None)?.get_command_tag();
        self.reset_cmd_tag.set(self.mcu.lookup_command("reset_step_clock oid=%c clock=%u", None)?.get_command_tag());
        *self.get_position_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "stepper_get_position oid=%c",
            "stepper_position oid=%c pos=%i",
            Some(self.oid),
            None,
            false,
        )?);
        let max_error_ticks = self.mcu.seconds_to_clock(MAX_STEPCOMPRESS_ERROR);
        if let Some(sc) = self.syncemitter.get_stepcompress() {
            sc.borrow_mut().fill(self.oid as u32, max_error_ticks as u32, step_cmd_tag as i32, dir_cmd_tag as i32);
        }
        Ok(())
    }

    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    pub fn get_step_dist(&self) -> f64 {
        self.step_dist.get()
    }

    pub fn get_rotation_distance(&self) -> (f64, f64) {
        (self.rotation_dist.get(), self.steps_per_rotation)
    }

    pub fn set_rotation_distance(&self, rotation_dist: f64) {
        let mcu_pos = self.get_mcu_position(None);
        self.rotation_dist.set(rotation_dist);
        self.step_dist.set(rotation_dist / self.steps_per_rotation);
        let tq = self.trapq.borrow().clone();
        self.set_trapq(tq);
        self.set_mcu_position(mcu_pos);
    }

    pub fn get_dir_inverted(&self) -> (bool, bool) {
        (self.invert_dir.get(), self.orig_invert_dir)
    }

    pub fn set_dir_inverted(&self, invert_dir: bool) -> KResult<()> {
        if invert_dir == self.invert_dir.get() {
            return Ok(());
        }
        self.invert_dir.set(invert_dir);
        if let Some(sc) = self.syncemitter.get_stepcompress() {
            sc.borrow_mut().set_invert_sdir(invert_dir as u32);
        }
        let me = self.rc();
        self.printer.send_event("stepper:set_dir_inverted", &[&me])
    }

    fn sk(&self) -> SkRef {
        self.stepper_kinematics.borrow().clone().expect("stepper kinematics")
    }

    pub fn calc_position_from_coord(&self, coord: &[f64]) -> f64 {
        itersolve::calc_position_from_coord(&self.sk().borrow(), coord[0], coord[1], coord[2])
    }

    pub fn set_position(&self, coord: &[f64]) {
        let mcu_pos = self.get_mcu_position(None);
        itersolve::set_position(&mut self.sk().borrow_mut(), coord[0], coord[1], coord[2]);
        self.set_mcu_position(mcu_pos);
    }

    pub fn get_commanded_position(&self) -> f64 {
        itersolve::get_commanded_pos(&self.sk().borrow())
    }

    pub fn get_mcu_position(&self, cmd_pos: Option<f64>) -> i64 {
        let cmd_pos = cmd_pos.unwrap_or_else(|| self.get_commanded_position());
        let mcu_pos_dist = cmd_pos + self.mcu_position_offset.get();
        let mcu_pos = mcu_pos_dist / self.step_dist.get();
        if mcu_pos >= 0.0 {
            return (mcu_pos + 0.5) as i64;
        }
        (mcu_pos - 0.5) as i64
    }

    fn set_mcu_position(&self, mcu_pos: i64) {
        let mcu_pos_dist = mcu_pos as f64 * self.step_dist.get();
        self.mcu_position_offset.set(mcu_pos_dist - self.get_commanded_position());
    }

    pub fn get_past_mcu_position(&self, print_time: f64) -> i64 {
        let clock = self.mcu.print_time_to_clock(print_time);
        match self.syncemitter.get_stepcompress() {
            Some(sc) => sc.borrow().find_past_position(clock as u64),
            None => 0,
        }
    }

    pub fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64 {
        mcu_pos as f64 * self.step_dist.get() - self.mcu_position_offset.get()
    }

    /// mcu_to_commanded_position() of a float mcu position
    pub fn mcu_to_commanded_position_f(&self, mcu_pos: f64) -> f64 {
        mcu_pos * self.step_dist.get() - self.mcu_position_offset.get()
    }

    pub fn dump_steps(&self, count: usize, start_clock: u64, end_clock: u64) -> Vec<crate::chelper::stepcompress::PullHistorySteps> {
        match self.syncemitter.get_stepcompress() {
            Some(sc) => sc.borrow().extract_old(count, start_clock, end_clock),
            None => vec![],
        }
    }

    pub fn get_stepper_kinematics(&self) -> Option<SkRef> {
        self.stepper_kinematics.borrow().clone()
    }

    pub fn set_stepper_kinematics(&self, sk: Option<SkRef>) -> Option<SkRef> {
        let old_sk = self.stepper_kinematics.borrow().clone();
        let mut mcu_pos = 0;
        if old_sk.is_some() {
            mcu_pos = self.get_mcu_position(None);
        }
        *self.stepper_kinematics.borrow_mut() = sk.clone();
        self.syncemitter.set_stepper_kinematics(sk);
        let tq = self.trapq.borrow().clone();
        self.set_trapq(tq);
        self.set_mcu_position(mcu_pos);
        old_sk
    }

    pub fn note_homing_end(&self) -> KResult<()> {
        let ret = match self.syncemitter.get_stepcompress() {
            Some(sc) => sc.borrow_mut().reset(0),
            None => 0,
        };
        if ret != 0 {
            return Err(error("Internal error in stepcompress"));
        }
        let data = [self.reset_cmd_tag.get(), self.oid as u32, 0];
        self.syncemitter.queue_msg(0, &data);
        self.query_mcu_position()
    }

    fn query_mcu_position(&self) -> KResult<()> {
        if self.mcu.is_fileoutput() || self.mcu.non_critical_disconnected.get() {
            return Ok(());
        }
        let params = match self.get_position_cmd.borrow().as_ref() {
            Some(cmd) => cmd.send(&[Val::Int(self.oid)], 0, 0, true)?,
            None => return Err(KError::Other("'NoneType' object has no attribute 'send'".into())),
        };
        let mut last_pos = pint(&params, "pos");
        if self.invert_dir.get() {
            last_pos = -last_pos;
        }
        let print_time = self.mcu.estimated_print_time(params_time(&params, "#receive_time"));
        let clock = self.mcu.print_time_to_clock(print_time);
        let ret = match self.syncemitter.get_stepcompress() {
            Some(sc) => sc.borrow_mut().set_last_position(clock as u64, last_pos),
            None => 0,
        };
        if ret != 0 {
            return Err(error("Internal error in stepcompress"));
        }
        self.set_mcu_position(last_pos);
        let me = self.rc();
        self.printer.send_event("stepper:sync_mcu_position", &[&me])
    }

    pub fn get_trapq(&self) -> Option<TrapQRef> {
        self.trapq.borrow().clone()
    }

    pub fn set_trapq(&self, tq: Option<TrapQRef>) -> Option<TrapQRef> {
        if let Some(sk) = self.stepper_kinematics.borrow().as_ref() {
            itersolve::set_trapq(&mut sk.borrow_mut(), tq.clone(), self.step_dist.get());
        }
        self.trapq.replace(tq)
    }

    pub fn add_active_callback(&self, cb: ActiveCallback) -> KResult<()> {
        self.active_callbacks.borrow_mut().push(cb);
        if self.active_callbacks.borrow().len() == 1 {
            let motion_queuing = self.printer.lookup::<PrinterMotionQueuing>("motion_queuing")?;
            let w = self.me.clone();
            let check: FlushCallback = Rc::new(move |ft, sgt| match w.upgrade() {
                Some(s) => s.check_active(ft, sgt),
                None => Ok(()),
            });
            *self.check_active_cb.borrow_mut() = Some(check.clone());
            motion_queuing.register_flush_callback(check, false);
        }
        Ok(())
    }

    fn check_active(&self, _must_flush_time: f64, max_step_gen_time: f64) -> KResult<()> {
        let ret = itersolve::check_active_time(&self.sk().borrow(), max_step_gen_time);
        if ret == 0. {
            // Stepper motor still not active
            return Ok(());
        }
        // Motor is active, disable future checking
        let motion_queuing = self.printer.lookup::<PrinterMotionQueuing>("motion_queuing")?;
        if let Some(cb) = self.check_active_cb.borrow_mut().take() {
            motion_queuing.unregister_flush_callback(&cb);
        }
        let cbs = std::mem::take(&mut *self.active_callbacks.borrow_mut());
        // Invoke callbacks
        for cb in cbs {
            cb(ret)?;
        }
        Ok(())
    }

    pub fn is_active_axis(&self, axis: char) -> bool {
        itersolve::is_active_axis(&self.sk().borrow(), axis as u8) != 0
    }
}

// Parse stepper gear_ratio config parameter
pub fn parse_gear_ratio(config: &ConfigWrapper, note_valid: bool) -> KResult<f64> {
    let gear_ratio =
        config.getlists("gear_ratio", Dflt::Val(crate::pyval::PyVal::Tuple(vec![])), &[":", ","], Some(2), ListParser::Float, note_valid)?;
    let mut result = 1.0;
    for pair in gear_ratio.as_ref().and_then(|v| v.items()).unwrap_or(&[]) {
        let items = pair.items().unwrap_or(&[]);
        let (g1, g2) = (items[0].as_f64().unwrap_or(0.), items[1].as_f64().unwrap_or(0.));
        result *= g1 / g2;
    }
    Ok(result)
}

// Obtain "step distance" information from a config section
pub fn parse_step_distance(config: &ConfigWrapper, units_in_radians: Option<bool>, note_valid: bool) -> KResult<(f64, f64)> {
    // Check rotation_distance and gear_ratio
    let units_in_radians = match units_in_radians {
        Some(u) => u,
        None => {
            // Caller doesn't know if units are in radians - infer it
            let rd = config.get_value("rotation_distance", Dflt::None, false)?;
            let gr = config.get_value("gear_ratio", Dflt::None, false)?;
            rd.is_none() && gr.is_some()
        }
    };
    let lim = |l: Lim| if note_valid { l } else { l.no_note() };
    let rotation_dist = if units_in_radians {
        config.get_value("gear_ratio", REQUIRED, note_valid)?;
        2.0 * std::f64::consts::PI
    } else {
        config.getfloat("rotation_distance", REQUIRED, lim(Lim::new().above(0.0)))?
    };
    // Check microsteps and full_steps_per_rotation
    let microsteps = config.getint("microsteps", REQUIRED, lim(Lim::new().min(1)))?;
    let full_steps = config.getint("full_steps_per_rotation", 200, lim(Lim::new().min(1)))?;
    if full_steps % 4 != 0 {
        return Err(KError::config(format!("full_steps_per_rotation invalid in section '{}'", config.get_name())));
    }
    let gearing = parse_gear_ratio(config, note_valid)?;
    Ok((rotation_dist, (full_steps * microsteps) as f64 * gearing))
}

/// tmc.py's current helper as homing.py uses it
pub trait TmcCurrentHelper {
    /// set_current_for_homing(print_time, pre_homing) -> dwell time
    fn set_current_for_homing(&self, print_time: f64, pre_homing: bool) -> KResult<f64>;
    fn as_any(&self) -> &dyn std::any::Any;
}

/// stepper.PrinterRail.get_homing_info()'s namedtuple
#[derive(Clone, Debug)]
pub struct HomingInfo {
    pub speed: f64,
    pub position_endstop: f64,
    pub retract_speed: f64,
    pub retract_dist: f64,
    pub positive_dir: bool,
    pub second_homing_speed: f64,
    pub use_sensorless_homing: bool,
    pub min_home_dist: f64,
    pub accel: Option<f64>,
}

/// a stepper rail as homing.py / toolhead.py use it (PrinterRail and the
/// dual carriage rails)
pub trait Rail {
    fn get_name(&self, short: bool) -> String;
    /// (position_min, position_max)
    fn get_range(&self) -> (f64, f64);
    fn get_steppers(&self) -> Vec<Rc<McuStepper>>;
    fn get_endstops(&self) -> Vec<(crate::mcu::EndstopRef, String)>;
    fn get_homing_info(&self) -> HomingInfo;
    fn get_tmc_current_helpers(&self) -> Vec<Option<Rc<dyn TmcCurrentHelper>>>;
}

pub type RailRef = Rc<dyn Rail>;

// ----------------------------------------------------------------------
// Stepper and rail construction from config sections
// ----------------------------------------------------------------------

/// printer.load_object(config, name) as a concrete type

fn lookup_pin(printer: &Printer, pin_desc: &str, can_invert: bool, can_pullup: bool) -> KResult<SharedPinParams> {
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let r = ppins.0.borrow_mut().lookup_pin(pin_desc, can_invert, can_pullup, None);
    r.map_err(|e| KError::Pins(e.0))
}

// Helper code to build a stepper object from a config section
pub fn printer_stepper(config: &ConfigWrapper, units_in_radians: bool) -> KResult<Rc<McuStepper>> {
    let printer = config.get_printer();
    // Stepper definition
    let step_pin = config.get("step_pin", Dflt::Required)?;
    let step_pin_params = lookup_pin(&printer, &step_pin, true, false)?;
    let dir_pin = config.get("dir_pin", Dflt::Required)?;
    let dir_pin_params = lookup_pin(&printer, &dir_pin, true, false)?;
    let (rotation_dist, steps_per_rotation) = parse_step_distance(config, Some(units_in_radians), true)?;
    let step_pulse_duration = config.getfloat_opt("step_pulse_duration", Lim::new().min(0.0).max(0.001))?;
    let mcu_stepper = McuStepper::new(
        config,
        &step_pin_params,
        &dir_pin_params,
        rotation_dist,
        steps_per_rotation,
        step_pulse_duration,
        units_in_radians,
    )?;
    // Register with helper modules
    printer.load_as::<PrinterStepperEnable>(config, "stepper_enable")?.register_stepper(config, &mcu_stepper)?;
    printer.load_as::<ForceMove>(config, "force_move")?.register_stepper(config, &mcu_stepper);
    printer.load_as::<PrinterMotionReport>(config, "motion_report")?.register_stepper(config, &mcu_stepper)?;
    Ok(mcu_stepper)
}

// A motor control "rail" with one (or more) steppers and one (or more)
// endstops.
pub struct PrinterRail {
    stepper_units_in_radians: bool,
    steppers: RefCell<Vec<Rc<McuStepper>>>,
    endstops: RefCell<Vec<(EndstopRef, String)>>,
    /// endstop_map: pin name -> (endstop, invert, pullup)
    endstop_map: RefCell<IndexMap<String, (EndstopRef, i64, i64)>>,
    tmc_current_helpers: RefCell<Option<Vec<Option<Rc<dyn TmcCurrentHelper>>>>>,
    pub position_endstop: Cell<f64>,
    pub position_min: Cell<f64>,
    pub position_max: Cell<f64>,
    use_sensorless_homing: Cell<bool>,
    homing_speed: Cell<f64>,
    second_homing_speed: Cell<f64>,
    homing_retract_speed: Cell<f64>,
    homing_retract_dist: Cell<f64>,
    homing_positive_dir: Cell<bool>,
    min_home_dist: Cell<f64>,
    homing_accel: Cell<Option<f64>>,
}

impl PrinterRail {
    pub fn new(
        config: &ConfigWrapper,
        need_position_minmax: bool,
        default_position_endstop: Option<f64>,
        units_in_radians: bool,
    ) -> KResult<Rc<Self>> {
        // Primary stepper and endstop
        let rail = Rc::new(PrinterRail {
            stepper_units_in_radians: units_in_radians,
            steppers: RefCell::new(vec![]),
            endstops: RefCell::new(vec![]),
            endstop_map: RefCell::new(IndexMap::new()),
            tmc_current_helpers: RefCell::new(None),
            position_endstop: Cell::new(0.0),
            position_min: Cell::new(0.0),
            position_max: Cell::new(0.0),
            use_sensorless_homing: Cell::new(false),
            homing_speed: Cell::new(0.0),
            second_homing_speed: Cell::new(0.0),
            homing_retract_speed: Cell::new(0.0),
            homing_retract_dist: Cell::new(0.0),
            homing_positive_dir: Cell::new(false),
            min_home_dist: Cell::new(0.0),
            homing_accel: Cell::new(None),
        });
        rail.add_extra_stepper(config)?;
        // Primary endstop position
        let mcu_endstop = rail.endstops.borrow()[0].0.clone();
        let position_endstop = match (mcu_endstop.get_position_endstop(), default_position_endstop) {
            (Some(p), _) => p,
            (None, None) => config.getfloat("position_endstop", Dflt::Required, Lim::new())?,
            (None, Some(d)) => config.getfloat("position_endstop", d, Lim::new())?,
        };
        rail.position_endstop.set(position_endstop);
        let endstop_pin = config.get_opt("endstop_pin")?;
        // check for ":virtual_endstop" to make sure we don't detect ":z_virtual_endstop"
        let endstop_is_virtual = endstop_pin.map(|p| p.contains(":virtual_endstop")).unwrap_or(false);

        // Axis range
        if need_position_minmax {
            rail.position_min.set(config.getfloat("position_min", 0.0, Lim::new())?);
            rail.position_max.set(config.getfloat("position_max", Dflt::Required, Lim::new().above(rail.position_min.get()))?);
        } else {
            rail.position_min.set(0.0);
            rail.position_max.set(position_endstop);
        }
        let (position_min, position_max) = (rail.position_min.get(), rail.position_max.get());
        if position_endstop < position_min || position_endstop > position_max {
            return Err(KError::config(format!(
                "position_endstop in section '{}' must be between position_min and position_max",
                config.get_name()
            )));
        }
        // Homing mechanics
        rail.use_sensorless_homing.set(config.getboolean("use_sensorless_homing", endstop_is_virtual)?);

        let homing_speed = config.getfloat("homing_speed", 5.0, Lim::new().above(0.0))?;
        rail.homing_speed.set(homing_speed);

        let default_second_homing_speed = if rail.use_sensorless_homing.get() { homing_speed } else { homing_speed / 2.0 };

        rail.second_homing_speed.set(config.getfloat("second_homing_speed", default_second_homing_speed, Lim::new().above(0.0))?);
        rail.homing_retract_speed.set(config.getfloat("homing_retract_speed", homing_speed, Lim::new().above(0.0))?);
        let homing_retract_dist = config.getfloat("homing_retract_dist", 5.0, Lim::new().min(0.0))?;
        rail.homing_retract_dist.set(homing_retract_dist);
        let homing_positive_dir = config.getboolean_opt("homing_positive_dir")?;

        rail.min_home_dist.set(config.getfloat("min_home_dist", homing_retract_dist, Lim::new().min(0.0))?);

        rail.homing_accel.set(config.getfloat_opt("homing_accel", Lim::new().above(0.0))?);

        match homing_positive_dir {
            None => {
                let axis_len = position_max - position_min;
                let dir = if position_endstop <= position_min + axis_len / 4.0 {
                    false
                } else if position_endstop >= position_max - axis_len / 4.0 {
                    true
                } else {
                    return Err(KError::config(format!(
                        "Unable to infer homing_positive_dir in section '{}'",
                        config.get_name()
                    )));
                };
                rail.homing_positive_dir.set(dir);
                config.getboolean("homing_positive_dir", dir)?;
            }
            Some(dir) => {
                rail.homing_positive_dir.set(dir);
                if (dir && position_endstop == position_min) || (!dir && position_endstop == position_max) {
                    return Err(KError::config(format!(
                        "Invalid homing_positive_dir / position_endstop in '{}'",
                        config.get_name()
                    )));
                }
            }
        }
        Ok(rail)
    }

    pub fn get_commanded_position(&self) -> f64 {
        self.steppers.borrow()[0].get_commanded_position()
    }

    pub fn calc_position_from_coord(&self, coord: &[f64]) -> f64 {
        self.steppers.borrow()[0].calc_position_from_coord(coord)
    }

    pub fn get_range(&self) -> (f64, f64) {
        (self.position_min.get(), self.position_max.get())
    }

    pub fn add_extra_stepper(&self, config: &ConfigWrapper) -> KResult<()> {
        let stepper = printer_stepper(config, self.stepper_units_in_radians)?;
        self.steppers.borrow_mut().push(stepper.clone());
        let endstop_pin = config.get_opt("endstop_pin")?;
        if !self.endstops.borrow().is_empty() && endstop_pin.is_none() {
            // No endstop defined - use primary endstop
            let e = self.endstops.borrow()[0].0.clone();
            return e.add_stepper(&stepper);
        }
        let endstop_pin = config.get("endstop_pin", Dflt::Required)?;
        let printer = config.get_printer();
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin_params = ppins.0.borrow().parse_pin(&endstop_pin, true, true).map_err(|e| KError::Pins(e.0))?;
        // Normalize pin name
        let pin_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        // Look for already-registered endstop
        let endstop = self.endstop_map.borrow().get(&pin_name).cloned();
        let mcu_endstop = match endstop {
            None => {
                // New endstop, register it
                let mcu_endstop = crate::pins::setup_pin(&printer, "endstop", &endstop_pin)
                    .and_then(|o| {
                        // tools_calibrate's probe_multi_axis chip returns a list of probes
                        let is_list = o.is::<Vec<EndstopRef>>();
                        any_to_endstop(o).ok_or_else(|| match is_list {
                            true => KError::Py("AttributeError".into(), "'list' object has no attribute 'add_stepper'".into()),
                            false => KError::Other("endstop pin has an unexpected type".into()),
                        })
                    })?;
                self.endstop_map.borrow_mut().insert(pin_name, (mcu_endstop.clone(), pin_params.invert, pin_params.pullup));
                let name = stepper.get_name(true);
                self.endstops.borrow_mut().push((mcu_endstop.clone(), name.clone()));
                printer.load_as::<QueryEndstops>(config, "query_endstops")?.register_endstop(mcu_endstop.clone(), &name);
                mcu_endstop
            }
            Some((mcu_endstop, invert, pullup)) => {
                let changed_invert = pin_params.invert != invert;
                let changed_pullup = pin_params.pullup != pullup;
                if changed_invert || changed_pullup {
                    return Err(error(format!(
                        "Printer rail {} shared endstop pin {} must specify the same pullup/invert settings",
                        self.get_name(false),
                        pin_name
                    )));
                }
                mcu_endstop
            }
        };
        mcu_endstop.add_stepper(&stepper)
    }

    /// setup_itersolve(alloc_func, *params): alloc() makes each stepper's sk
    pub fn setup_itersolve(&self, alloc: impl Fn() -> SkRef) {
        for stepper in self.steppers.borrow().iter() {
            stepper.setup_itersolve(alloc());
        }
    }

    pub fn set_trapq(&self, trapq: Option<TrapQRef>) {
        for stepper in self.steppers.borrow().iter() {
            stepper.set_trapq(trapq.clone());
        }
    }

    pub fn set_position(&self, coord: &[f64]) {
        for stepper in self.steppers.borrow().iter() {
            stepper.set_position(coord);
        }
    }
}

impl Rail for PrinterRail {
    fn get_name(&self, short: bool) -> String {
        self.steppers.borrow()[0].get_name(short)
    }

    fn get_range(&self) -> (f64, f64) {
        PrinterRail::get_range(self)
    }

    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.steppers.borrow().clone()
    }

    fn get_endstops(&self) -> Vec<(EndstopRef, String)> {
        self.endstops.borrow().clone()
    }

    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: self.homing_speed.get(),
            position_endstop: self.position_endstop.get(),
            retract_speed: self.homing_retract_speed.get(),
            retract_dist: self.homing_retract_dist.get(),
            positive_dir: self.homing_positive_dir.get(),
            second_homing_speed: self.second_homing_speed.get(),
            use_sensorless_homing: self.use_sensorless_homing.get(),
            min_home_dist: self.min_home_dist.get(),
            accel: self.homing_accel.get(),
        }
    }

    fn get_tmc_current_helpers(&self) -> Vec<Option<Rc<dyn TmcCurrentHelper>>> {
        if self.tmc_current_helpers.borrow().is_none() {
            let h = self.steppers.borrow().iter().map(|s| s.get_tmc_current_helper()).collect();
            *self.tmc_current_helpers.borrow_mut() = Some(h);
        }
        self.tmc_current_helpers.borrow().clone().unwrap()
    }
}

// Wrapper for dual stepper motor support
pub fn lookup_multi_rail(
    config: &ConfigWrapper,
    need_position_minmax: bool,
    default_position_endstop: Option<f64>,
    units_in_radians: bool,
) -> KResult<Rc<PrinterRail>> {
    let rail = PrinterRail::new(config, need_position_minmax, default_position_endstop, units_in_radians)?;
    for i in 1..99 {
        let name = format!("{}{}", config.get_name(), i);
        if !config.has_section(&name) {
            break;
        }
        rail.add_extra_stepper(&config.getsection(&name))?;
    }
    Ok(rail)
}
