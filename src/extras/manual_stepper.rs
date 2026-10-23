// Support for a manual controlled stepper (port of klippy/extras/manual_stepper.py)
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::itersolve::SkRef;
use crate::chelper::kin_cartesian::cartesian_stepper_alloc;
use crate::chelper::trapq::TrapQRef;
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::force_move::calc_move_time;
use crate::extras::homing::{HomingToolhead, PrinterHoming};
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::extras::stepper_enable::PrinterStepperEnable;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::EndstopRef;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::reactor::ReactorCompletion;
use crate::stepper::{printer_stepper, McuStepper, PrinterRail, Rail};
use crate::toolhead::{ExtraAxis, ExtraAxisRef, Move, ToolHead};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// self.rail: a PrinterRail (with an endstop) or a single PrinterStepper
enum RailOrStepper {
    Rail(Rc<PrinterRail>),
    Stepper(Rc<McuStepper>),
}

impl RailOrStepper {
    fn setup_itersolve(&self, alloc: impl Fn() -> SkRef) {
        match self {
            RailOrStepper::Rail(r) => r.setup_itersolve(alloc),
            RailOrStepper::Stepper(s) => s.setup_itersolve(alloc()),
        }
    }
    fn set_trapq(&self, tq: TrapQRef) {
        match self {
            RailOrStepper::Rail(r) => r.set_trapq(Some(tq)),
            RailOrStepper::Stepper(s) => {
                s.set_trapq(Some(tq));
            }
        }
    }
    fn set_position(&self, coord: &[f64]) {
        match self {
            RailOrStepper::Rail(r) => r.set_position(coord),
            RailOrStepper::Stepper(s) => s.set_position(coord),
        }
    }
    fn get_name(&self) -> String {
        match self {
            RailOrStepper::Rail(r) => r.get_name(false),
            RailOrStepper::Stepper(s) => s.get_name(false),
        }
    }
}

const CMD_MANUAL_STEPPER_HELP: &str = "Command a manually configured stepper";

pub struct ManualStepper {
    me: Weak<ManualStepper>,
    printer: Rc<Printer>,
    can_home: bool,
    rail: RailOrStepper,
    steppers: Vec<Rc<McuStepper>>,
    velocity: f64,
    accel: f64,
    homing_accel: Cell<f64>,
    next_cmd_time: Cell<f64>,
    commanded_pos: Cell<f64>,
    pos_min: Option<f64>,
    pos_max: Option<f64>,
    motion_queuing: Rc<PrinterMotionQueuing>,
    trapq: TrapQRef,
    // Registered with toolhead as an axtra axis
    axis_gcode_id: RefCell<Option<String>>,
    instant_corner_v: Cell<f64>,
    gaxis_limit_velocity: Cell<f64>,
    gaxis_limit_accel: Cell<f64>,
}

impl ManualStepper {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let (can_home, rail, steppers) = if config.get_opt("endstop_pin")?.is_some() {
            let rail = PrinterRail::new(config, false, Some(0.0), false)?;
            let steppers = rail.get_steppers();
            (true, RailOrStepper::Rail(rail), steppers)
        } else {
            let s = printer_stepper(config, false)?;
            (false, RailOrStepper::Stepper(s.clone()), vec![s])
        };
        let velocity = config.getfloat("velocity", 5.0, Lim::new().above(0.0))?;
        let accel = config.getfloat("accel", 0.0, Lim::new().min(0.0))?;
        let pos_min = config.getfloat_opt("position_min", Lim::new())?;
        let pos_max = config.getfloat_opt("position_max", Lim::new())?;
        // Setup iterative solver
        let motion_queuing = printer.load_as::<PrinterMotionQueuing>(config, "motion_queuing")?;
        let trapq = motion_queuing.allocate_trapq();
        rail.setup_itersolve(|| cartesian_stepper_alloc('x').expect("axis"));
        rail.set_trapq(trapq.clone());
        let ms = Rc::new_cyclic(|me| ManualStepper {
            me: me.clone(),
            printer: printer.clone(),
            can_home,
            rail,
            steppers,
            velocity,
            accel,
            homing_accel: Cell::new(accel),
            next_cmd_time: Cell::new(0.0),
            commanded_pos: Cell::new(0.0),
            pos_min,
            pos_max,
            motion_queuing,
            trapq,
            axis_gcode_id: RefCell::new(None),
            instant_corner_v: Cell::new(0.0),
            gaxis_limit_velocity: Cell::new(0.0),
            gaxis_limit_accel: Cell::new(0.0),
        });
        // Register commands
        let stepper_name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let w = ms.me.clone();
        gcode.register_mux_command(
            "MANUAL_STEPPER",
            "STEPPER",
            Some(&stepper_name),
            Rc::new(move |g| match w.upgrade() {
                Some(m) => m.cmd_manual_stepper(g),
                None => Ok(()),
            }),
            Some(CMD_MANUAL_STEPPER_HELP),
        )?;
        Ok(ms)
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.printer.lookup::<ToolHead>("toolhead")
    }

    fn sync_print_time(&self) -> KResult<()> {
        let toolhead = self.toolhead()?;
        let print_time = toolhead.get_last_move_time()?;
        if self.next_cmd_time.get() > print_time {
            toolhead.dwell(self.next_cmd_time.get() - print_time)?;
        } else {
            self.next_cmd_time.set(print_time);
        }
        Ok(())
    }

    fn do_enable(&self, enable: bool) -> KResult<()> {
        let stepper_names: Vec<String> = self.steppers.iter().map(|s| s.get_name(false)).collect();
        let stepper_enable = self.printer.lookup::<PrinterStepperEnable>("stepper_enable")?;
        stepper_enable.set_motors_enable(&stepper_names, enable)?;
        Ok(())
    }

    fn do_set_position(&self, setpos: f64) -> KResult<()> {
        self.toolhead()?.flush_step_generation()?;
        self.commanded_pos.set(setpos);
        self.rail.set_position(&[setpos, 0.0, 0.0]);
        Ok(())
    }

    fn submit_move(&self, movetime: f64, movepos: f64, speed: f64, accel: f64) -> f64 {
        let cp = self.commanded_pos.get();
        let dist = movepos - cp;
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(dist, speed, accel);
        self.trapq.borrow_mut().append(movetime, accel_t, cruise_t, accel_t, cp, 0.0, 0.0, axis_r, 0.0, 0.0, 0.0, cruise_v, accel);
        self.commanded_pos.set(movepos);
        movetime + accel_t + cruise_t + accel_t
    }

    fn do_move(&self, movepos: f64, speed: f64, accel: f64, sync: bool) -> KResult<()> {
        self.sync_print_time()?;
        self.next_cmd_time.set(self.submit_move(self.next_cmd_time.get(), movepos, speed, accel));
        self.motion_queuing.note_mcu_movequeue_activity(self.next_cmd_time.get(), true);
        if sync {
            self.sync_print_time()?;
        }
        Ok(())
    }

    fn do_homing_move(&self, movepos: f64, speed: f64, accel: f64, triggered: bool, check_trigger: bool) -> KResult<()> {
        if !self.can_home {
            return Err(KError::command("No endstop for this manual stepper"));
        }
        self.homing_accel.set(accel);
        let pos = [movepos, 0.0, 0.0, 0.0];
        let endstops: Vec<(EndstopRef, String)> = match &self.rail {
            RailOrStepper::Rail(r) => r.get_endstops(),
            RailOrStepper::Stepper(_) => vec![],
        };
        let phoming = self.printer.lookup::<PrinterHoming>("homing")?;
        let me: Rc<dyn HomingToolhead> = self.me.upgrade().expect("manual stepper");
        phoming.manual_home(&me, endstops, &pos, speed, triggered, check_trigger)
    }

    fn out_of_range(&self, movepos: f64) -> bool {
        self.pos_min.map(|m| movepos < m).unwrap_or(false) || self.pos_max.map(|m| movepos > m).unwrap_or(false)
    }

    fn cmd_manual_stepper(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if gcmd.get_opt("GCODE_AXIS").is_some() {
            return self.command_with_gcode_axis(gcmd);
        }
        if self.axis_gcode_id.borrow().is_some() {
            return Err(gcmd.error("Must unregister from gcode axis first"));
        }
        if let Some(enable) = gcmd.get_int_opt("ENABLE", Lim::new())? {
            self.do_enable(enable != 0)?;
        }
        if let Some(setpos) = gcmd.get_float_opt("SET_POSITION", Lim::new())? {
            self.do_set_position(setpos)?;
        }
        let speed = gcmd.get_float("SPEED", Some(self.velocity), Lim::new().above(0.0))?;
        let accel = gcmd.get_float("ACCEL", Some(self.accel), Lim::new().min(0.0))?;
        let homing_move = gcmd.get_int("STOP_ON_ENDSTOP", Some(0), Lim::new())?;
        if homing_move != 0 {
            let movepos = gcmd.get_float("MOVE", None, Lim::new())?;
            if self.out_of_range(movepos) {
                return Err(gcmd.error("Move out of range"));
            }
            self.do_homing_move(movepos, speed, accel, homing_move > 0, homing_move.abs() == 1)?;
        } else if gcmd.get_float_opt("MOVE", Lim::new())?.is_some() {
            let movepos = gcmd.get_float("MOVE", None, Lim::new())?;
            if self.out_of_range(movepos) {
                return Err(gcmd.error("Move out of range"));
            }
            let sync = gcmd.get_int("SYNC", Some(1), Lim::new())?;
            self.do_move(movepos, speed, accel, sync != 0)?;
        } else if gcmd.get_int("SYNC", Some(0), Lim::new())? != 0 {
            self.sync_print_time()?;
        }
        Ok(())
    }

    // Register as a gcode axis
    fn command_with_gcode_axis(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let toolhead = self.toolhead()?;
        let gcode_axis = gcmd.get("GCODE_AXIS")?.to_uppercase();
        let instant_corner_v = gcmd.get_float("INSTANTANEOUS_CORNER_VELOCITY", Some(1.0), Lim::new().min(0.0))?;
        let limit_velocity = gcmd.get_float("LIMIT_VELOCITY", Some(999999.9), Lim::new().above(0.0))?;
        let limit_accel = gcmd.get_float("LIMIT_ACCEL", Some(999999.9), Lim::new().above(0.0))?;
        let me: ExtraAxisRef = self.me.upgrade().expect("manual stepper");
        if self.axis_gcode_id.borrow().is_some() {
            if !gcode_axis.is_empty() {
                return Err(gcmd.error("Must unregister axis first"));
            }
            // Unregister
            toolhead.remove_extra_axis(&me)?;
            *self.axis_gcode_id.borrow_mut() = None;
            return Ok(());
        }
        let is_upper = gcode_axis.chars().any(|c| c.is_uppercase()) && !gcode_axis.chars().any(|c| c.is_lowercase());
        if gcode_axis.chars().count() != 1 || !is_upper || "XYZEFN".contains(gcode_axis.as_str()) {
            if gcode_axis.is_empty() {
                // Request to unregister already unregistered axis
                return Ok(());
            }
            return Err(gcmd.error("Not a valid GCODE_AXIS"));
        }
        for ea in toolhead.get_extra_axes().into_iter().flatten() {
            if ea.get_axis_gcode_id() == gcode_axis {
                return Err(gcmd.error(format!("Axis '{}' already registered", gcode_axis)));
            }
        }
        *self.axis_gcode_id.borrow_mut() = Some(gcode_axis);
        self.instant_corner_v.set(instant_corner_v);
        self.gaxis_limit_velocity.set(limit_velocity);
        self.gaxis_limit_accel.set(limit_accel);
        toolhead.add_extra_axis(me, self.commanded_pos.get())
    }
}

impl ExtraAxis for ManualStepper {
    fn process_move(&self, print_time: f64, mv: &Move, ea_index: usize) -> KResult<()> {
        let axis_r = mv.axes_r[ea_index];
        let start_pos = mv.start_pos[ea_index];
        let accel = mv.accel * axis_r;
        let start_v = mv.start_v * axis_r;
        let cruise_v = mv.cruise_v * axis_r;
        self.trapq.borrow_mut().append(
            print_time, mv.accel_t, mv.cruise_t, mv.decel_t, start_pos, 0.0, 0.0, 1.0, 0.0, 0.0, start_v, cruise_v, accel,
        );
        self.commanded_pos.set(mv.end_pos[ea_index]);
        Ok(())
    }

    fn check_move(&self, mv: &mut Move, ea_index: usize) -> KResult<()> {
        // Check move is in bounds
        let movepos = mv.end_pos[ea_index];
        if self.out_of_range(movepos) {
            return Err(mv.move_error("Move out of range"));
        }
        // Check if need to limit maximum velocity and acceleration
        let axis_ratio = mv.move_d / mv.axes_d[ea_index].abs();
        let limit_velocity = self.gaxis_limit_velocity.get() * axis_ratio;
        let mut limit_accel = self.gaxis_limit_accel.get() * axis_ratio;
        if !mv.is_kinematic_move && self.accel != 0.0 {
            limit_accel = py_min(limit_accel, self.accel * axis_ratio);
        }
        mv.limit_speed(limit_velocity, limit_accel);
        Ok(())
    }

    fn calc_junction(&self, prev_move: &Move, mv: &Move, ea_index: usize) -> f64 {
        let diff_r = mv.axes_r[ea_index] - prev_move.axes_r[ea_index];
        if diff_r != 0.0 {
            return (self.instant_corner_v.get() / diff_r.abs()).powf(2.0);
        }
        mv.max_cruise_v2
    }

    /// (python's manual_stepper has no find_past_position)
    fn find_past_position(&self, _print_time: f64) -> f64 {
        0.0
    }

    /// (python's manual_stepper has no get_name)
    fn get_name(&self) -> String {
        String::new()
    }

    fn get_trapq(&self) -> Option<TrapQRef> {
        Some(self.trapq.clone())
    }

    fn get_axis_gcode_id(&self) -> String {
        self.axis_gcode_id.borrow().clone().unwrap_or_default()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

// Toolhead wrappers to support homing
impl HomingToolhead for ManualStepper {
    fn flush_step_generation(&self) -> KResult<()> {
        self.toolhead()?.flush_step_generation()
    }

    fn get_position(&self) -> Vec<f64> {
        vec![self.commanded_pos.get(), 0.0, 0.0, 0.0]
    }

    fn set_position(&self, newpos: &[f64], _homing_axes: &str) -> KResult<()> {
        self.do_set_position(newpos[0])
    }

    fn get_last_move_time(&self) -> KResult<f64> {
        self.sync_print_time()?;
        Ok(self.next_cmd_time.get())
    }

    fn dwell(&self, delay: f64) -> KResult<()> {
        self.next_cmd_time.set(self.next_cmd_time.get() + py_max(0.0, delay));
        Ok(())
    }

    fn drip_move(&self, newpos: &[f64], speed: f64, drip_completion: &Rc<ReactorCompletion<bool>>) -> KResult<()> {
        // Submit move to trapq
        self.sync_print_time()?;
        let start_time = self.next_cmd_time.get();
        let end_time = self.submit_move(start_time, newpos[0], speed, self.homing_accel.get());
        // Drip updates to motors
        self.motion_queuing.drip_update_time(start_time, end_time, drip_completion)?;
        // Clear trapq of any remaining parts of movement
        self.motion_queuing.wipe_trapq(&self.trapq);
        self.rail.set_position(&[self.commanded_pos.get(), 0.0, 0.0]);
        self.sync_print_time()
    }

    fn kin_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.steppers.clone()
    }

    fn kin_calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        vec![stepper_positions[&self.rail.get_name()], 0.0, 0.0]
    }
}

impl PrinterObject for ManualStepper {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ManualStepper::new(config)?)
}
