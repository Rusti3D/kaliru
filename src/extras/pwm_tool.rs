// Queued PWM gpio output (port of klippy/extras/pwm_tool.py)
//
// Copyright (C) 2017-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::steppersync::SyncEmitterRef;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::{get_printer_mcu, Mcu};
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::cell::Cell;
use std::rc::{Rc, Weak};

pub struct McuQueuedPwm {
    me: Weak<McuQueuedPwm>,
    mcu: Rc<Mcu>,
    hardware_pwm: Cell<bool>,
    cycle_time: Cell<f64>,
    max_duration: Cell<f64>,
    oid: i64,
    motion_queuing: Rc<PrinterMotionQueuing>,
    syncemitter: SyncEmitterRef,
    pin: String,
    invert: bool,
    start_value: Cell<f64>,
    shutdown_value: Cell<f64>,
    last_clock: Cell<i64>,
    last_value: Cell<i64>,
    default_value: Cell<f64>,
    duration_ticks: Cell<i64>,
    pwm_max: Cell<f64>,
    set_cmd_tag: Cell<Option<u32>>,
}

impl McuQueuedPwm {
    fn new(config: &ConfigWrapper, mcu: &Rc<Mcu>, pin: String, invert: bool) -> KResult<Rc<Self>> {
        let oid = mcu.create_oid();
        let printer = mcu.get_printer();
        let sname = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let motion_queuing = printer.load_as::<PrinterMotionQueuing>(config, "motion_queuing")?;
        let syncemitter = motion_queuing.allocate_syncemitter(mcu, &sname, false);
        let p = Rc::new_cyclic(|me| McuQueuedPwm {
            me: me.clone(),
            mcu: mcu.clone(),
            hardware_pwm: Cell::new(false),
            cycle_time: Cell::new(0.100),
            max_duration: Cell::new(2.0),
            oid,
            motion_queuing,
            syncemitter,
            pin,
            invert,
            start_value: Cell::new(invert as i64 as f64),
            shutdown_value: Cell::new(invert as i64 as f64),
            last_clock: Cell::new(0),
            last_value: Cell::new(0),
            default_value: Cell::new(0.0),
            duration_ticks: Cell::new(0),
            pwm_max: Cell::new(0.0),
            set_cmd_tag: Cell::new(None),
        });
        let pp = p.clone();
        mcu.register_config_callback(Rc::new(move || pp.build_config()));
        // (python looks up the toolhead at connect; it is not used)
        Ok(p)
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }

    pub fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
        self.cycle_time.set(cycle_time);
        self.hardware_pwm.set(hardware_pwm);
    }

    pub fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        let (mut start_value, mut shutdown_value) = (start_value, shutdown_value);
        if self.invert {
            start_value = 1.0 - start_value;
            shutdown_value = 1.0 - shutdown_value;
        }
        self.start_value.set(py_max(0.0, py_min(1.0, start_value)));
        self.shutdown_value.set(py_max(0.0, py_min(1.0, shutdown_value)));
    }

    fn build_config(&self) -> KResult<()> {
        let mcu = &self.mcu;
        let printer = mcu.get_printer();
        let (start_value, shutdown_value) = (self.start_value.get(), self.shutdown_value.get());
        if self.max_duration.get() != 0.0 && start_value != shutdown_value {
            return Err(KError::config("Pin with max duration must have start value equal to shutdown value"));
        }
        let cmd_queue = mcu.alloc_command_queue();
        let curtime = printer.get_reactor().monotonic();
        let printtime = mcu.estimated_print_time(curtime);
        self.last_clock.set(mcu.print_time_to_clock(printtime + 0.200));
        let cycle_ticks = mcu.seconds_to_clock(self.cycle_time.get());
        if cycle_ticks >= 1 << 31 {
            return Err(KError::config("PWM pin cycle time too large"));
        }
        self.duration_ticks.set(mcu.seconds_to_clock(self.max_duration.get()));
        if self.duration_ticks.get() >= 1 << 31 {
            return Err(KError::config("PWM pin max duration too large"));
        }
        if self.duration_ticks.get() != 0 {
            let w = self.me.clone();
            self.motion_queuing.register_flush_callback(
                Rc::new(move |must_flush_time, max_step_gen_time| match w.upgrade() {
                    Some(p) => p.flush_notification(must_flush_time, max_step_gen_time),
                    None => Ok(()),
                }),
                false,
            );
        }
        if self.hardware_pwm.get() {
            self.pwm_max.set(mcu.get_constant_float("PWM_MAX")?);
            let pwm_max = self.pwm_max.get();
            self.default_value.set(shutdown_value * pwm_max);
            // "%d" of a float truncates
            mcu.add_config_cmd(
                &format!(
                    "config_pwm_out oid={} pin={} cycle_ticks={} value={} default_value={} max_duration={}",
                    self.oid,
                    self.pin,
                    cycle_ticks,
                    (start_value * pwm_max).trunc() as i64,
                    self.default_value.get().trunc() as i64,
                    self.duration_ticks.get()
                ),
                false,
                false,
            );
            self.last_value.set((start_value * pwm_max + 0.5).trunc() as i64);
            mcu.add_config_cmd(
                &format!("queue_pwm_out oid={} clock={} value={}", self.oid, self.last_clock.get(), self.last_value.get()),
                false,
                true,
            );
            self.set_cmd_tag.set(Some(mcu.lookup_command("queue_pwm_out oid=%c clock=%u value=%hu", Some(cmd_queue))?.get_command_tag()));
            return Ok(());
        }
        // Software PWM
        if shutdown_value != 0.0 && shutdown_value != 1.0 {
            return Err(KError::config("shutdown value must be 0.0 or 1.0 on soft pwm"));
        }
        mcu.add_config_cmd(
            &format!(
                "config_digital_out oid={} pin={} value={} default_value={} max_duration={}",
                self.oid,
                self.pin,
                (start_value >= 1.0) as i64,
                (shutdown_value >= 0.5) as i64,
                self.duration_ticks.get()
            ),
            false,
            false,
        );
        self.default_value.set(((shutdown_value >= 0.5) as i64 * cycle_ticks) as f64);
        mcu.add_config_cmd(&format!("set_digital_out_pwm_cycle oid={} cycle_ticks={}", self.oid, cycle_ticks), false, false);
        self.pwm_max.set(cycle_ticks as f64);
        self.last_value.set((start_value * self.pwm_max.get() + 0.5).trunc() as i64);
        mcu.add_config_cmd(&format!("queue_digital_out oid={} clock={} on_ticks={}", self.oid, self.last_clock.get(), self.last_value.get()), true, false);
        self.set_cmd_tag.set(Some(mcu.lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue))?.get_command_tag()));
        Ok(())
    }

    fn send_update(&self, clock: i64, val: i64) {
        let clock = self.last_clock.get().max(clock);
        self.last_clock.set(clock);
        self.last_value.set(val);
        let tag = self.set_cmd_tag.get().unwrap_or(0);
        let data = [tag, self.oid as u32, (clock & 0xFFFFFFFF) as u32, val as u32];
        self.syncemitter.queue_msg(clock as u64, &data);
        // Notify toolhead so that it will flush this update
        let mut wakeclock = clock;
        if self.last_value.get() as f64 != self.default_value.get() {
            // Continue flushing to resend time
            wakeclock += self.duration_ticks.get();
        }
        let wake_print_time = self.mcu.clock_to_print_time(wakeclock as f64);
        self.motion_queuing.note_mcu_movequeue_activity(wake_print_time, false);
    }

    fn gen_intermediate_updates(&self, clock: i64) {
        if self.last_value.get() as f64 == self.default_value.get() {
            return;
        }
        while clock >= self.last_clock.get() + self.duration_ticks.get() {
            self.send_update(self.last_clock.get() + self.duration_ticks.get(), self.last_value.get());
        }
    }

    pub fn set_pwm(&self, print_time: f64, value: f64) {
        let clock = self.mcu.print_time_to_clock(print_time);
        let value = if self.invert { 1.0 - value } else { value };
        let v = (py_max(0.0, py_min(1.0, value)) * self.pwm_max.get() + 0.5).trunc() as i64;
        if self.duration_ticks.get() != 0 {
            self.gen_intermediate_updates(clock - 1);
        }
        self.send_update(clock, v);
    }

    fn flush_notification(&self, must_flush_time: f64, _max_step_gen_time: f64) -> KResult<()> {
        let clock = self.mcu.print_time_to_clock(must_flush_time);
        self.gen_intermediate_updates(clock);
        Ok(())
    }
}

const CMD_SET_PIN_HELP: &str = "Set the value of an output pin";

pub struct PrinterOutputPin {
    printer: Rc<Printer>,
    mcu_pin: Rc<McuQueuedPwm>,
    scale: f64,
    last_print_time: Cell<f64>,
    last_value: Cell<f64>,
}

impl PrinterOutputPin {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let ppins = printer.lookup::<PinsObject>("pins")?;
        // Determine pin type
        let pin_desc = config.get("pin", Dflt::Required)?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(&pin_desc, true, false, None).map_err(|e| KError::Pins(e.0))?;
        let (chip_name, pin, invert) = {
            let p = pin_params.borrow();
            (p.chip_name.clone(), p.pin.clone(), p.invert != 0)
        };
        let mcu = get_printer_mcu(&printer, &chip_name)?;
        let mcu_pin = McuQueuedPwm::new(config, &mcu, pin, invert)?;
        let max_duration = mcu_pin.get_mcu().max_nominal_duration();
        let cycle_time = config.getfloat("cycle_time", 0.100, Lim::new().above(0.0).max(max_duration))?;
        let hardware_pwm = config.getboolean("hardware_pwm", false)?;
        mcu_pin.setup_cycle_time(cycle_time, hardware_pwm);
        let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
        // Support mcu checking for maximum duration
        let max_mcu_duration = config.getfloat("maximum_mcu_duration", 0.0, Lim::new().min(0.500).max(max_duration))?;
        mcu_pin.setup_max_duration(max_mcu_duration);
        // Determine start and shutdown values
        let last_value = config.getfloat("value", 0.0, Lim::new().min(0.0).max(scale))? / scale;
        let shutdown_value = config.getfloat("shutdown_value", 0.0, Lim::new().min(0.0).max(scale))? / scale;
        mcu_pin.setup_start_value(last_value, shutdown_value);
        let o = Rc::new(PrinterOutputPin { printer: printer.clone(), mcu_pin, scale, last_print_time: Cell::new(0.0), last_value: Cell::new(last_value) });
        // Register commands
        let pin_name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let oo = o.clone();
        gcode.register_mux_command("SET_PIN", "PIN", Some(&pin_name), Rc::new(move |g| oo.cmd_set_pin(g)), Some(CMD_SET_PIN_HELP))?;
        Ok(o)
    }

    fn set_pin(&self, print_time: f64, value: f64) {
        if value == self.last_value.get() {
            return;
        }
        let print_time = py_max(print_time, self.last_print_time.get());
        self.mcu_pin.set_pwm(print_time, value);
        self.last_value.set(value);
        self.last_print_time.set(print_time);
    }

    fn cmd_set_pin(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        // Read requested value
        let value = gcmd.get_float("VALUE", None, Lim::new().min(0.0).max(self.scale))? / self.scale;
        // Obtain print_time and apply requested settings
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let s = self.clone();
        toolhead.register_lookahead_callback(Rc::new(move |print_time| {
            s.set_pin(print_time, value);
            Ok(())
        }))
    }
}

impl PrinterObject for PrinterOutputPin {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("value".to_string(), PyVal::Float(self.last_value.get()));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterOutputPin::new(config)?)
}
