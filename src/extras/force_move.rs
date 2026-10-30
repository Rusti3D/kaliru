// Utility for manually moving a stepper for diagnostic purposes
// (port of klippy/extras/force_move.py)
//
// Copyright (C) 2018-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::itersolve::SkRef;
use crate::chelper::kin_cartesian::cartesian_stepper_alloc;
use crate::chelper::trapq::TrapQRef;
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::extras::stepper_enable::PrinterStepperEnable;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::stepper::McuStepper;
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

pub const BUZZ_DISTANCE: f64 = 1.0;
pub const BUZZ_VELOCITY: f64 = BUZZ_DISTANCE / 0.250;
pub const BUZZ_RADIANS_DISTANCE: f64 = 1.0 * std::f64::consts::PI / 180.0;
pub const BUZZ_RADIANS_VELOCITY: f64 = BUZZ_RADIANS_DISTANCE / 0.250;

// Calculate a move's accel_t, cruise_t, and cruise_v
pub fn calc_move_time(mut dist: f64, mut speed: f64, accel: f64) -> (f64, f64, f64, f64) {
    let mut axis_r = 1.0;
    if dist < 0.0 {
        axis_r = -1.0;
        dist = -dist;
    }
    if accel == 0.0 || dist == 0.0 {
        return (axis_r, 0.0, dist / speed, speed);
    }
    let max_cruise_v2 = dist * accel;
    if max_cruise_v2 < speed * speed {
        speed = max_cruise_v2.sqrt();
    }
    let accel_t = speed / accel;
    let accel_decel_d = accel_t * speed;
    let cruise_t = (dist - accel_decel_d) / speed;
    (axis_r, accel_t, cruise_t, speed)
}

const CMD_STEPPER_BUZZ_HELP: &str = "Oscillate a given stepper to help id it";
const CMD_FORCE_MOVE_HELP: &str = "Manually move a stepper; invalidates kinematics";
const CMD_SET_KINEMATIC_POSITION_HELP: &str = "Force a low-level kinematic position";

pub struct ForceMove {
    pub printer: Rc<Printer>,
    pub steppers: RefCell<IndexMap<String, Rc<McuStepper>>>,
    pub motion_queuing: Rc<PrinterMotionQueuing>,
    pub trapq: TrapQRef,
    pub stepper_kinematics: SkRef,
}

impl ForceMove {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Setup iterative solver
        let motion_queuing = printer
            .load_object(config, "motion_queuing", true)?
            .and_then(downcast_rc::<PrinterMotionQueuing>)
            .ok_or_else(|| KError::Other("motion_queuing has an unexpected type".into()))?;
        let trapq = motion_queuing.allocate_trapq();
        let stepper_kinematics = cartesian_stepper_alloc('x').expect("x axis");
        let fm = Rc::new(ForceMove { printer: printer.clone(), steppers: RefCell::new(IndexMap::default()), motion_queuing, trapq, stepper_kinematics });
        // Register commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let f = fm.clone();
        gcode.register_command("STEPPER_BUZZ", Some(Rc::new(move |g| f.cmd_stepper_buzz(g))), false, Some(CMD_STEPPER_BUZZ_HELP))?;
        if !config.getboolean("enable_force_move", true)? {
            return Ok(fm);
        }

        let f = fm.clone();
        gcode.register_command("FORCE_MOVE", Some(Rc::new(move |g| f.cmd_force_move(g))), false, Some(CMD_FORCE_MOVE_HELP))?;
        let f = fm.clone();
        gcode.register_command(
            "SET_KINEMATIC_POSITION",
            Some(Rc::new(move |g| f.cmd_set_kinematic_position(g))),
            false,
            Some(CMD_SET_KINEMATIC_POSITION_HELP),
        )?;
        Ok(fm)
    }

    pub fn register_stepper(&self, _config: &ConfigWrapper, mcu_stepper: &Rc<McuStepper>) {
        self.steppers.borrow_mut().insert(mcu_stepper.get_name(false), mcu_stepper.clone());
    }

    pub fn lookup_stepper(&self, name: &str) -> KResult<Rc<McuStepper>> {
        self.steppers.borrow().get(name).cloned().ok_or_else(|| KError::config(format!("Unknown stepper {}", name)))
    }

    fn stepper_enable(&self) -> KResult<Rc<PrinterStepperEnable>> {
        self.printer.lookup::<PrinterStepperEnable>("stepper_enable")
    }

    pub fn force_enable(&self, stepper: &McuStepper) -> KResult<bool> {
        let stepper_name = stepper.get_name(false);
        self.stepper_enable()?.set_motors_enable(&[stepper_name], true)
    }

    pub fn restore_enable(&self, stepper: &McuStepper, did_enable: bool) -> KResult<()> {
        if !did_enable {
            return Ok(());
        }
        let stepper_name = stepper.get_name(false);
        self.stepper_enable()?.set_motors_enable(&[stepper_name], false)?;
        Ok(())
    }

    pub fn manual_move(&self, stepper: &McuStepper, dist: f64, speed: f64, accel: f64) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.flush_step_generation()?;
        let prev_sk = stepper.set_stepper_kinematics(Some(self.stepper_kinematics.clone()));
        let prev_trapq = stepper.set_trapq(Some(self.trapq.clone()));
        stepper.set_position(&[0.0, 0.0, 0.0]);
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(dist, speed, accel);
        let mut print_time = toolhead.get_last_move_time()?;
        self.trapq.borrow_mut().append(
            print_time, accel_t, cruise_t, accel_t, 0.0, 0.0, 0.0, axis_r, 0.0, 0.0, 0.0, cruise_v, accel,
        );
        print_time = print_time + accel_t + cruise_t + accel_t;
        self.motion_queuing.note_mcu_movequeue_activity(print_time, true);
        toolhead.dwell(accel_t + cruise_t + accel_t)?;
        toolhead.flush_step_generation()?;
        stepper.set_trapq(prev_trapq);
        stepper.set_stepper_kinematics(prev_sk);
        self.motion_queuing.wipe_trapq(&self.trapq);
        Ok(())
    }

    pub fn lookup_gcmd_stepper(&self, gcmd: &GCodeCommand) -> KResult<Rc<McuStepper>> {
        let name = gcmd.get("STEPPER")?;
        self.steppers.borrow().get(&name).cloned().ok_or_else(|| gcmd.error(format!("Unknown stepper {}", name)))
    }

    pub fn cmd_stepper_buzz(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let stepper = self.lookup_gcmd_stepper(gcmd)?;
        logging::info(&format!("Stepper buzz {}", stepper.get_name(false)));
        let did_enable = self.force_enable(&stepper)?;
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let (mut dist, mut speed) = (BUZZ_DISTANCE, BUZZ_VELOCITY);
        if stepper.units_in_radians() {
            dist = BUZZ_RADIANS_DISTANCE;
            speed = BUZZ_RADIANS_VELOCITY;
        }
        for _ in 0..10 {
            self.manual_move(&stepper, dist, speed, 0.0)?;
            toolhead.dwell(0.050)?;
            self.manual_move(&stepper, -dist, speed, 0.0)?;
            toolhead.dwell(0.450)?;
        }
        self.restore_enable(&stepper, did_enable)
    }

    pub fn cmd_force_move(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let stepper = self.lookup_gcmd_stepper(gcmd)?;
        let distance = gcmd.get_float("DISTANCE", None, Lim::new())?;
        let speed = gcmd.get_float("VELOCITY", None, Lim::new().above(0.0))?;
        let accel = gcmd.get_float("ACCEL", Some(0.0), Lim::new().min(0.0))?;
        logging::info(&format!(
            "FORCE_MOVE {} distance={:.3} velocity={:.3} accel={:.3}",
            stepper.get_name(false),
            distance,
            speed,
            accel
        ));
        self.force_enable(&stepper)?;
        self.manual_move(&stepper, distance, speed, accel)
    }

    pub fn cmd_set_kinematic_position(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.get_last_move_time()?;
        let curpos = toolhead.get_position();
        let x = gcmd.get_float("X", Some(curpos[0]), Lim::new())?;
        let y = gcmd.get_float("Y", Some(curpos[1]), Lim::new())?;
        let z = gcmd.get_float("Z", Some(curpos[2]), Lim::new())?;
        let set_homed = gcmd.get_or("SET_HOMED", "xyz").to_lowercase();
        let set_homed_axes: String = "xyz".chars().filter(|a| set_homed.contains(*a)).collect();
        let clear_homed = match gcmd.get_opt("CLEAR_HOMED") {
            // "CLEAR" is an alias for "CLEAR_HOMED"; should deprecate
            None => gcmd.get_or("CLEAR", "").to_lowercase(),
            Some(c) => c.to_lowercase(),
        };
        let clear_homed_axes: String = "xyz".chars().filter(|a| clear_homed.contains(*a)).collect();
        logging::info(&format!(
            "SET_KINEMATIC_POSITION pos={:.3},{:.3},{:.3} set_homed={} clear_homed={}",
            x, y, z, set_homed_axes, clear_homed_axes
        ));
        toolhead.set_position(&[x, y, z], &set_homed_axes)?;
        toolhead.get_kinematics().clear_homing_state(&clear_homed_axes);
        Ok(())
    }
}

impl PrinterObject for ForceMove {
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ForceMove::new(config)?)
}
