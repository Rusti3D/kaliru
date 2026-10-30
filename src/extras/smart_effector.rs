// SmartEffector support (port of klippy/extras/smart_effector.py)
//
// Copyright (C) 2021  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// SmartEffector communication protocol implemented here originates from
// https://github.com/Duet3D/SmartEffectorFirmware
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::pin_mcu;
use crate::extras::homing::{HomingMove, PrinterHoming};
use crate::extras::probe::{McuProbe, PrinterProbe, ProbeEndstopWrapper};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::{CommandWrapper, Endstop, EndstopRef, Mcu, StepperRef};
use crate::msgproto::Val;
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::ReactorCompletion;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const BITS_PER_SECOND: f64 = 1000.0;

pub struct ControlPinHelper {
    pub mcu: Rc<Mcu>,
    pub pin: String,
    pub start_value: i64,
    pub invert: i64,
    pub oid: Cell<Option<i64>>,
    pub set_cmd: RefCell<Option<CommandWrapper>>,
}

impl ControlPinHelper {
    pub fn build_config(&self) -> KResult<()> {
        self.mcu.request_move_queue_slot();
        let oid = self.mcu.create_oid();
        self.oid.set(Some(oid));
        self.mcu.add_config_cmd(
            &format!("config_digital_out oid={} pin={} value={} default_value={} max_duration={}", oid, self.pin, self.start_value, self.start_value, 0),
            false,
            false,
        );
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.set_cmd.borrow_mut() = Some(self.mcu.lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue))?);
        Ok(())
    }

    pub fn write_bits(&self, start_time: f64, bit_stream: &[i64]) -> KResult<f64> {
        let set_cmd = self.set_cmd.borrow();
        let set_cmd = set_cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        let oid = self.oid.get().unwrap_or(0);
        let bit_step = 1.0 / BITS_PER_SECOND;
        let mut last_value = self.start_value;
        let mut bit_time = start_time;
        let mut value = last_value;
        for &b in bit_stream {
            value = (b != 0) as i64 ^ self.invert;
            if value != last_value {
                let clock = self.mcu.print_time_to_clock(bit_time);
                set_cmd.send(&[Val::Int(oid), Val::Int(clock), Val::Int(value)], 0, 0)?;
                last_value = value;
            }
            bit_time += bit_step;
        }
        // After the last bit, the signal on the control pin must go back
        // to its start value.
        if value != self.start_value {
            let clock = self.mcu.print_time_to_clock(bit_time);
            set_cmd.send(&[Val::Int(oid), Val::Int(clock), Val::Int(self.start_value)], 0, 0)?;
            bit_time += bit_step;
        }
        Ok(bit_time)
    }
}

const CMD_SET_SMART_EFFECTOR_HELP: &str = "Set SmartEffector parameters";
const CMD_RESET_SMART_EFFECTOR_HELP: &str = "Reset SmartEffector settings (sensitivity)";

pub struct SmartEffectorEndstopWrapper {
    me: Weak<SmartEffectorEndstopWrapper>,
    pub printer: Rc<Printer>,
    pub gcode: Rc<GCodeDispatch>,
    pub probe_accel: Cell<f64>,
    pub recovery_time: Cell<f64>,
    pub old_max_accel: Cell<Option<f64>>,
    pub probe_wrapper: Rc<ProbeEndstopWrapper>,
    pub control_pin: Option<Rc<ControlPinHelper>>,
}

impl SmartEffectorEndstopWrapper {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let probe_accel = config.getfloat("probe_accel", 0.0, Lim::new().min(0.0))?;
        let recovery_time = config.getfloat("recovery_time", 0.4, Lim::new().min(0.0))?;
        let probe_wrapper = ProbeEndstopWrapper::new(config)?;
        // SmartEffector control
        let mut control_pin = None;
        if let Some(pin) = config.get_opt("control_pin")?.filter(|p| !p.is_empty()) {
            let ppins = printer.lookup::<PinsObject>("pins")?;
            let pin_params = ppins.0.borrow_mut().lookup_pin(&pin, true, false, None).map_err(|e| KError::Pins(e.0))?;
            let mcu = pin_mcu(config, &pin_params)?;
            let invert = pin_params.borrow().invert;
            let helper = Rc::new(ControlPinHelper {
                mcu: mcu.clone(),
                pin: pin_params.borrow().pin.clone(),
                start_value: invert,
                invert,
                oid: Cell::new(None),
                set_cmd: RefCell::new(None),
            });
            let h = helper.clone();
            mcu.register_config_callback(Rc::new(move || h.build_config()));
            control_pin = Some(helper);
        }
        let s = Rc::new_cyclic(|me| SmartEffectorEndstopWrapper {
            me: me.clone(),
            printer: printer.clone(),
            gcode: gcode.clone(),
            probe_accel: Cell::new(probe_accel),
            recovery_time: Cell::new(recovery_time),
            old_max_accel: Cell::new(None),
            probe_wrapper,
            control_pin: control_pin.clone(),
        });
        if control_pin.is_some() {
            let w = s.me.clone();
            gcode.register_command(
                "RESET_SMART_EFFECTOR",
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(s) => s.cmd_reset_smart_effector(g),
                    None => Ok(()),
                })),
                false,
                Some(CMD_RESET_SMART_EFFECTOR_HELP),
            )?;
        }
        let w = s.me.clone();
        gcode.register_command(
            "SET_SMART_EFFECTOR",
            Some(Rc::new(move |g| match w.upgrade() {
                Some(s) => s.cmd_set_smart_effector(g),
                None => Ok(()),
            })),
            false,
            Some(CMD_SET_SMART_EFFECTOR_HELP),
        )?;
        Ok(s)
    }

    pub fn send_command(&self, buf: &[i64]) -> KResult<()> {
        // Each byte is sent to the SmartEffector as
        // [0 0 1 0 b7 b6 b5 b4 !b4 b3 b2 b1 b0 !b0]
        let mut bit_stream = vec![];
        for &b in buf {
            let b = b & 0xFF;
            bit_stream.extend([0, 0, 1, 0]);
            bit_stream.extend([b & 0x80, b & 0x40, b & 0x20, b & 0x10]);
            bit_stream.push((!b) & 0x10);
            bit_stream.extend([b & 0x08, b & 0x04, b & 0x02, b & 0x01]);
            bit_stream.push((!b) & 0x01);
        }
        // Wait for previous actions to finish
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.wait_moves()?;
        let start_time = toolhead.get_last_move_time()?;
        // Write generated bits to the control pin
        let control_pin = self.control_pin.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'write_bits'".into()))?;
        let end_time = control_pin.write_bits(start_time, &bit_stream)?;
        // Dwell to make sure no subseqent actions are queued together
        // with the SmartEffector programming
        toolhead.dwell(end_time - start_time)?;
        toolhead.wait_moves()
    }

    pub fn cmd_set_smart_effector(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let sensitivity = gcmd.get_int_opt("SENSITIVITY", Lim::new().min(0.0).max(255.0))?;
        let mut respond_info = vec![];
        if let Some(sensitivity) = sensitivity {
            if self.control_pin.is_some() {
                let buf = [105, sensitivity, 255 - sensitivity];
                self.send_command(&buf)?;
                respond_info.push(format!("sensitivity: {}", sensitivity));
            } else {
                return Err(gcmd.error("control_pin must be set in [smart_effector] for sensitivity programming"));
            }
        }
        self.probe_accel.set(gcmd.get_float("ACCEL", Some(self.probe_accel.get()), Lim::new().min(0.0))?);
        self.recovery_time.set(gcmd.get_float("RECOVERY_TIME", Some(self.recovery_time.get()), Lim::new().min(0.0))?);
        if self.probe_accel.get() != 0.0 {
            respond_info.push(format!("probing accelartion: {:.3}", self.probe_accel.get()));
        } else {
            respond_info.push("probing acceleration control disabled".into());
        }
        if self.recovery_time.get() != 0.0 {
            respond_info.push(format!("probe recovery time: {:.3}", self.recovery_time.get()));
        } else {
            respond_info.push("probe recovery time disabled".into());
        }
        gcmd.respond_info(&format!("SmartEffector:\n{}", respond_info.join("\n")), true);
        Ok(())
    }

    pub fn cmd_reset_smart_effector(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.send_command(&[131, 131])?;
        gcmd.respond_info("SmartEffector sensitivity was reset", true);
        Ok(())
    }
}

impl Endstop for SmartEffectorEndstopWrapper {
    fn get_mcu(&self) -> Rc<Mcu> {
        Endstop::get_mcu(&*self.probe_wrapper)
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.probe_wrapper.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        Endstop::get_steppers(&*self.probe_wrapper)
    }
    fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        self.probe_wrapper.home_start(print_time, sample_time, sample_count, rest_time, triggered)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.probe_wrapper.home_wait(home_end_time)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        self.probe_wrapper.query_endstop(print_time)
    }
    fn get_position_endstop(&self) -> Option<f64> {
        // (python's wrapper does not forward get_position_endstop)
        None
    }
}

impl McuProbe for SmartEffectorEndstopWrapper {
    fn multi_probe_begin(&self, always_restore_toolhead: bool) -> KResult<()> {
        self.probe_wrapper.multi_probe_begin(always_restore_toolhead)
    }

    fn multi_probe_end(&self) -> KResult<()> {
        self.probe_wrapper.multi_probe_end()
    }

    fn probing_move(&self, pos: &[f64], speed: f64, _gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        let phoming = self.printer.lookup::<PrinterHoming>("homing")?;
        let me: EndstopRef = self.me.upgrade().expect("smart_effector");
        Ok((phoming.probing_move(me, pos, speed)?, Some(true)))
    }

    fn probe_prepare(&self, hmove: &Rc<HomingMove>) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        self.probe_wrapper.probe_prepare(hmove)?;
        if self.probe_accel.get() != 0.0 {
            let systime = self.printer.get_reactor().monotonic();
            let max_accel = match PrinterObject::get_status(&*toolhead, systime) {
                Some(PyVal::Dict(d)) => d.get("max_accel").and_then(|v| v.as_f64()),
                _ => None,
            };
            self.old_max_accel.set(max_accel);
            self.gcode.run_script_from_command(&format!("M204 S{:.3}", self.probe_accel.get()))?;
        }
        if self.recovery_time.get() != 0.0 {
            toolhead.dwell(self.recovery_time.get())?;
        }
        Ok(())
    }

    fn probe_finish(&self, hmove: &Rc<HomingMove>) -> KResult<()> {
        if self.probe_accel.get() != 0.0 {
            let old = self.old_max_accel.get().ok_or_else(|| KError::Other("'SmartEffectorEndstopWrapper' object has no attribute 'old_max_accel'".into()))?;
            self.gcode.run_script_from_command(&format!("M204 S{:.3}", old))?;
        }
        self.probe_wrapper.probe_finish(hmove)
    }
}

impl PrinterObject for SmartEffectorEndstopWrapper {
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let smart_effector = SmartEffectorEndstopWrapper::new(config)?;
    PrinterProbe::new(config, smart_effector.clone())?;
    Ok(smart_effector)
}
