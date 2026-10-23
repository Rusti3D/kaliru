// BLTouch support (port of klippy/extras/bltouch.py)
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::extras::homing::{HomingMove, PrinterHoming};
use crate::extras::probe::{McuProbe, PrinterProbe};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::{Endstop, EndstopRef, Mcu, McuEndstop, PwmPin, StepperRef};
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::reactor::ReactorCompletion;
use crate::toolhead::ToolHead;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const SIGNAL_PERIOD: f64 = 0.020;
pub const MIN_CMD_TIME: f64 = 5.0 * SIGNAL_PERIOD;

pub const TEST_TIME: f64 = 5.0 * 60.0;
pub const RETRY_RESET_TIME: f64 = 1.0;

/// Commands: (name, pulse width)
pub const COMMANDS: &[(&str, f64)] = &[
    ("pin_down", 0.000650),
    ("touch_mode", 0.001165),
    ("pin_up", 0.001475),
    ("self_test", 0.001780),
    ("reset", 0.002190),
    ("set_5V_output_mode", 0.001988),
    ("set_OD_output_mode", 0.002091),
    ("output_mode_store", 0.001884),
];

fn command(name: &str) -> f64 {
    COMMANDS.iter().find(|(n, _)| *n == name).map(|(_, v)| *v).unwrap_or(0.0)
}

#[derive(Clone, Copy, PartialEq)]
enum Multi {
    Off,
    First,
    On,
}

const CMD_BLTOUCH_DEBUG_HELP: &str = "Send a command to the bltouch for debugging";
const CMD_BLTOUCH_STORE_HELP: &str = "Store an output mode in the BLTouch EEPROM";

// BLTouch "endstop" wrapper
pub struct BLTouchEndstopWrapper {
    me: Weak<BLTouchEndstopWrapper>,
    printer: Rc<Printer>,
    position_endstop: f64,
    stow_on_each_sample: bool,
    probe_touch_mode: bool,
    mcu_pwm: Rc<dyn PwmPin>,
    next_cmd_time: Cell<f64>,
    action_end_time: Cell<f64>,
    finish_home_complete: RefCell<Option<Rc<ReactorCompletion<bool>>>>,
    wait_trigger_complete: RefCell<Option<Rc<ReactorCompletion<()>>>>,
    mcu_endstop: Rc<McuEndstop>,
    output_mode: Option<String>,
    next_test_time: Cell<f64>,
    pin_up_not_triggered: bool,
    pin_up_touch_triggered: bool,
    pin_move_time: f64,
    gcode: Rc<GCodeDispatch>,
    multi: Cell<Multi>,
    // the danger_options overrides of the module constants
    endstop_rest_time: f64,
    endstop_sample_time: f64,
    endstop_sample_count: i64,
}

impl BLTouchEndstopWrapper {
    fn new(config: &ConfigWrapper, endstop_rest_time: f64, endstop_sample_time: f64, endstop_sample_count: i64) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // klippy:connect / klippy:mcu_identify are registered first
        let slot: Rc<RefCell<Weak<BLTouchEndstopWrapper>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match sl.borrow().upgrade() {
                Some(b) => b.handle_connect(),
                None => Ok(()),
            }),
        );
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:mcu_identify",
            Rc::new(move |_| match sl.borrow().upgrade() {
                Some(b) => b.handle_mcu_identify(),
                None => Ok(()),
            }),
        );
        let position_endstop = config.getfloat("z_offset", Dflt::Required, Lim::new().min(0.0))?;
        let stow_on_each_sample = config.getboolean("stow_on_each_sample", true)?;
        let probe_touch_mode = config.getboolean("probe_with_touch_mode", false)?;
        // Create a pwm object to handle the control pin
        let mcu_pwm = crate::pins::setup_pwm_pin(&printer, &config.get("control_pin", Dflt::Required)?)?;
        mcu_pwm.setup_max_duration(0.0);
        mcu_pwm.setup_cycle_time(SIGNAL_PERIOD, false)?;
        // Create an "endstop" object to handle the sensor pin
        let pin = config.get("sensor_pin", Dflt::Required)?;
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(&pin, true, true, None).map_err(|e| KError::Pins(e.0))?;
        let chip = pin_params.borrow().chip.clone();
        let mcu_endstop = chip
            .setup_pin("endstop", &pin_params)
            .map_err(KError::Pins)?
            .downcast::<McuEndstop>()
            .map_err(|_| KError::Other("bltouch sensor pin chip has no mcu endstop".into()))?;
        // output mode
        let output_mode = config.getchoice_opt("set_output_mode", &["5V", "OD"])?;
        // Setup for sensor test
        let pin_up_not_triggered = config.getboolean("pin_up_reports_not_triggered", true)?;
        let pin_up_touch_triggered = config.getboolean("pin_up_touch_mode_reports_triggered", true)?;
        // Calculate pin move time
        let pin_move_time = config.getfloat("pin_move_time", 0.680, Lim::new().above(0.0))?;
        // Register BLTOUCH_DEBUG command
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let b = Rc::new_cyclic(|me| BLTouchEndstopWrapper {
            me: me.clone(),
            printer: printer.clone(),
            position_endstop,
            stow_on_each_sample,
            probe_touch_mode,
            mcu_pwm,
            // Command timing
            next_cmd_time: Cell::new(0.0),
            action_end_time: Cell::new(0.0),
            finish_home_complete: RefCell::new(None),
            wait_trigger_complete: RefCell::new(None),
            mcu_endstop,
            output_mode,
            next_test_time: Cell::new(0.0),
            pin_up_not_triggered,
            pin_up_touch_triggered,
            pin_move_time,
            gcode: gcode.clone(),
            // multi probes state
            multi: Cell::new(Multi::Off),
            endstop_rest_time,
            endstop_sample_time,
            endstop_sample_count,
        });
        *slot.borrow_mut() = Rc::downgrade(&b);
        let bb = b.clone();
        gcode.register_command("BLTOUCH_DEBUG", Some(Rc::new(move |g| bb.cmd_bltouch_debug(g))), false, Some(CMD_BLTOUCH_DEBUG_HELP))?;
        let bb = b.clone();
        gcode.register_command("BLTOUCH_STORE", Some(Rc::new(move |g| bb.cmd_bltouch_store(g))), false, Some(CMD_BLTOUCH_STORE_HELP))?;
        Ok(b)
    }

    fn handle_mcu_identify(&self) -> KResult<()> {
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        for stepper in kin.get_steppers() {
            if stepper.is_active_axis('z') {
                self.mcu_endstop.add_stepper(&stepper)?;
            }
        }
        Ok(())
    }

    fn handle_connect(&self) -> KResult<()> {
        self.sync_mcu_print_time();
        self.next_cmd_time.set(self.next_cmd_time.get() + 0.200);
        self.set_output_mode(self.output_mode.as_deref())?;
        match self.raise_probe().and_then(|_| self.verify_raise_probe()) {
            Err(e) if e.is_command_error() => {
                logging::warning(&format!("BLTouch raise probe error: {}", e.message()));
                Ok(())
            }
            r => r,
        }
    }

    fn sync_mcu_print_time(&self) {
        let curtime = self.printer.get_reactor().monotonic();
        let est_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime);
        self.next_cmd_time.set(py_max(self.next_cmd_time.get(), est_time + MIN_CMD_TIME));
    }

    fn sync_print_time(&self) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let print_time = toolhead.get_last_move_time()?;
        if self.next_cmd_time.get() > print_time {
            toolhead.dwell(self.next_cmd_time.get() - print_time)?;
        } else {
            self.next_cmd_time.set(print_time);
        }
        Ok(())
    }

    fn send_cmd(&self, cmd: &str, duration: f64) -> KResult<()> {
        // Translate duration to ticks to avoid any secondary mcu clock skew
        let mcu = self.mcu_pwm.get_mcu();
        let mut cmd_clock = mcu.print_time_to_clock(self.next_cmd_time.get());
        let pulse = ((duration - MIN_CMD_TIME) / SIGNAL_PERIOD).trunc() * SIGNAL_PERIOD;
        cmd_clock += mcu.seconds_to_clock(py_max(MIN_CMD_TIME, pulse));
        let end_time = mcu.clock_to_print_time(cmd_clock as f64);
        // Schedule command followed by PWM disable
        self.mcu_pwm.set_pwm(self.next_cmd_time.get(), command(cmd) / SIGNAL_PERIOD)?;
        self.mcu_pwm.set_pwm(end_time, 0.0)?;
        // Update time tracking
        self.action_end_time.set(self.next_cmd_time.get() + duration);
        self.next_cmd_time.set(py_max(self.action_end_time.get(), end_time + MIN_CMD_TIME));
        Ok(())
    }

    fn verify_state(&self, triggered: bool) -> KResult<bool> {
        // Perform endstop check to verify bltouch reports desired state
        Endstop::home_start(
            &*self.mcu_endstop,
            self.action_end_time.get(),
            self.endstop_sample_time,
            self.endstop_sample_count,
            self.endstop_rest_time,
            triggered,
        )?;
        match Endstop::home_wait(&*self.mcu_endstop, self.action_end_time.get() + 0.100) {
            Ok(trigger_time) => Ok(trigger_time > 0.0),
            Err(e) if e.is_command_error() => Ok(false),
            Err(e) => Err(e),
        }
    }

    fn raise_probe(&self) -> KResult<()> {
        self.sync_mcu_print_time();
        if !self.pin_up_not_triggered {
            self.send_cmd("reset", MIN_CMD_TIME)?;
        }
        self.send_cmd("pin_up", self.pin_move_time)
    }

    fn verify_raise_probe(&self) -> KResult<()> {
        if !self.pin_up_not_triggered {
            // No way to verify raise attempt
            return Ok(());
        }
        for retry in 0..3 {
            let success = self.verify_state(false)?;
            if success {
                // The "probe raised" test completed successfully
                break;
            }
            if retry >= 2 {
                return Err(KError::command("BLTouch failed to raise probe"));
            }
            let msg = "Failed to verify BLTouch probe is raised; retrying.";
            self.gcode.respond_info(msg, true);
            self.sync_mcu_print_time();
            self.send_cmd("reset", RETRY_RESET_TIME)?;
            self.send_cmd("pin_up", self.pin_move_time)?;
        }
        Ok(())
    }

    fn lower_probe(&self) -> KResult<()> {
        self.test_sensor()?;
        self.sync_print_time()?;
        self.send_cmd("pin_down", self.pin_move_time)?;
        if self.probe_touch_mode {
            self.send_cmd("touch_mode", MIN_CMD_TIME)?;
        }
        Ok(())
    }

    fn test_sensor(&self) -> KResult<()> {
        if !self.pin_up_touch_triggered {
            // Nothing to test
            return Ok(());
        }
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let print_time = toolhead.get_last_move_time()?;
        if print_time < self.next_test_time.get() {
            self.next_test_time.set(print_time + TEST_TIME);
            return Ok(());
        }
        // Raise the bltouch probe and test if probe is raised
        self.sync_print_time()?;
        for retry in 0..3 {
            self.send_cmd("pin_up", self.pin_move_time)?;
            self.send_cmd("touch_mode", MIN_CMD_TIME)?;
            let success = self.verify_state(true)?;
            self.sync_print_time()?;
            if success {
                // The "bltouch connection" test completed successfully
                self.next_test_time.set(print_time + TEST_TIME);
                return Ok(());
            }
            let msg = "BLTouch failed to verify sensor state";
            if retry >= 2 {
                return Err(KError::command(msg));
            }
            self.gcode.respond_info(&format!("{}; retrying.", msg), true);
            self.send_cmd("reset", RETRY_RESET_TIME)?;
        }
        Ok(())
    }

    fn wait_for_trigger(&self) -> KResult<()> {
        let c = self.finish_home_complete.borrow().clone();
        if let Some(c) = c {
            c.wait(crate::reactor::NEVER, None)?;
        }
        if self.multi.get() == Multi::Off {
            self.raise_probe()?;
        }
        Ok(())
    }

    fn set_output_mode(&self, mode: Option<&str>) -> KResult<()> {
        // If this is inadvertently/purposely issued for a
        // BLTOUCH pre V3.0 and clones:
        //   No reaction at all.
        // BLTOUCH V3.0 and V3.1:
        //   This will set the mode.
        let Some(mode) = mode else { return Ok(()) };
        logging::info(&format!("BLTouch set output mode: {}", mode));
        self.sync_mcu_print_time();
        if mode == "5V" {
            self.send_cmd("set_5V_output_mode", MIN_CMD_TIME)?;
        }
        if mode == "OD" {
            self.send_cmd("set_OD_output_mode", MIN_CMD_TIME)?;
        }
        Ok(())
    }

    fn store_output_mode(&self, mode: &str) -> KResult<()> {
        // If this command is inadvertently/purposely issued for a
        // BLTOUCH pre V3.0 and clones:
        //   No reaction at all to this sequence apart from a pin-down/pin-up
        // BLTOUCH V3.0:
        //   This will set the mode (twice) and sadly, a pin-up is needed at
        //   the end, because of the pin-down
        // BLTOUCH V3.1:
        //   This will set the mode and store it in the eeprom.
        //   The pin-up is not needed but does not hurt
        logging::info(&format!("BLTouch store output mode: {}", mode));
        self.sync_print_time()?;
        self.send_cmd("pin_down", MIN_CMD_TIME)?;
        let set = if mode == "5V" { "set_5V_output_mode" } else { "set_OD_output_mode" };
        self.send_cmd(set, MIN_CMD_TIME)?;
        self.send_cmd("output_mode_store", MIN_CMD_TIME)?;
        self.send_cmd(set, MIN_CMD_TIME)?;
        self.send_cmd("pin_up", MIN_CMD_TIME)
    }

    fn cmd_bltouch_debug(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cmd = gcmd.get_opt("COMMAND");
        let Some(cmd) = cmd.filter(|c| COMMANDS.iter().any(|(n, _)| n == c)) else {
            let mut names: Vec<&str> = COMMANDS.iter().map(|(n, _)| *n).collect();
            names.sort();
            gcmd.respond_info(&format!("BLTouch commands: {}", names.join(", ")), true);
            return Ok(());
        };
        gcmd.respond_info(&format!("Sending BLTOUCH_DEBUG COMMAND={}", cmd), true);
        self.sync_print_time()?;
        self.send_cmd(&cmd, self.pin_move_time)?;
        self.sync_print_time()
    }

    fn cmd_bltouch_store(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cmd = gcmd.get_opt("MODE");
        let Some(cmd) = cmd.filter(|c| c == "5V" || c == "OD") else {
            gcmd.respond_info("BLTouch output modes: 5V, OD", true);
            return Ok(());
        };
        gcmd.respond_info(&format!("Storing BLTouch output mode: {}", cmd), true);
        self.sync_print_time()?;
        self.store_output_mode(&cmd)?;
        self.sync_print_time()
    }
}

impl Endstop for BLTouchEndstopWrapper {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_endstop.get_mcu()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.mcu_endstop.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        Endstop::get_steppers(&*self.mcu_endstop)
    }
    fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        let rest_time = py_min(rest_time, self.endstop_rest_time);
        let c = Endstop::home_start(&*self.mcu_endstop, print_time, sample_time, sample_count, rest_time, triggered)?;
        *self.finish_home_complete.borrow_mut() = Some(c.clone());
        // Schedule wait_for_trigger callback
        let r = self.printer.get_reactor();
        let w = self.me.clone();
        let wc = r.register_callback(
            move |_| {
                if let Some(b) = w.upgrade() {
                    if let Err(e) = b.wait_for_trigger() {
                        logging::exception_err("BLTouch wait_for_trigger", &e);
                    }
                }
            },
            crate::reactor::NOW,
        );
        *self.wait_trigger_complete.borrow_mut() = Some(wc);
        Ok(c)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        Endstop::home_wait(&*self.mcu_endstop, home_end_time)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        Endstop::query_endstop(&*self.mcu_endstop, print_time)
    }
    fn get_position_endstop(&self) -> Option<f64> {
        Some(self.position_endstop)
    }
}

impl McuProbe for BLTouchEndstopWrapper {
    fn multi_probe_begin(&self, _always_restore_toolhead: bool) -> KResult<()> {
        if self.stow_on_each_sample {
            return Ok(());
        }
        self.multi.set(Multi::First);
        Ok(())
    }

    fn multi_probe_end(&self) -> KResult<()> {
        if self.stow_on_each_sample {
            return Ok(());
        }
        self.sync_print_time()?;
        self.raise_probe()?;
        self.verify_raise_probe()?;
        self.sync_print_time()?;
        self.multi.set(Multi::Off);
        Ok(())
    }

    fn probing_move(&self, pos: &[f64], speed: f64, _gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        let phoming = self.printer.lookup::<PrinterHoming>("homing")?;
        let me: EndstopRef = self.me.upgrade().expect("bltouch");
        Ok((phoming.probing_move(me, pos, speed)?, Some(true)))
    }

    fn probe_prepare(&self, _hmove: &Rc<HomingMove>) -> KResult<()> {
        let multi = self.multi.get();
        if multi == Multi::Off || multi == Multi::First {
            self.lower_probe()?;
            if multi == Multi::First {
                self.multi.set(Multi::On);
            }
        }
        self.sync_print_time()
    }

    fn probe_finish(&self, hmove: &Rc<HomingMove>) -> KResult<()> {
        let wc = self.wait_trigger_complete.borrow().clone();
        match wc {
            Some(c) => {
                c.wait(crate::reactor::NEVER, None)?;
            }
            None => return Err(KError::Other("'NoneType' object has no attribute 'wait'".into())),
        }
        if self.multi.get() == Multi::Off {
            self.verify_raise_probe()?;
        }
        self.sync_print_time()?;
        if hmove.check_no_movement().is_some() {
            return Err(KError::command("BLTouch failed to deploy"));
        }
        Ok(())
    }
}

impl PrinterObject for BLTouchEndstopWrapper {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        *self.finish_home_complete.borrow_mut() = None;
        *self.wait_trigger_complete.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let d = get_danger_options();
    let blt = BLTouchEndstopWrapper::new(config, d.homing_start_delay, d.endstop_sample_time, d.endstop_sample_count)?;
    PrinterProbe::new(config, blt.clone())?;
    Ok(blt)
}
