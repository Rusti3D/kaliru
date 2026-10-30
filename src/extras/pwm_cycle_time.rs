// Handle pwm output pins with variable frequency
// (port of klippy/extras/pwm_cycle_time.py)
//
// Copyright (C) 2017-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::{get_printer_mcu, CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::pins::PinsObject;
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub struct McuPwmCycle {
    pub mcu: Rc<Mcu>,
    pub cycle_time: f64,
    pub oid: Cell<Option<i64>>,
    pub pin: String,
    pub invert: bool,
    pub start_value: f64,
    pub shutdown_value: f64,
    pub last_clock: Cell<i64>,
    pub cycle_ticks: Cell<i64>,
    pub set_cmd: RefCell<Option<CommandWrapper>>,
    pub set_cycle_ticks: RefCell<Option<CommandWrapper>>,
}

impl McuPwmCycle {
    fn new(mcu: &Rc<Mcu>, pin: String, invert: bool, cycle_time: f64, start_value: f64, shutdown_value: f64) -> Rc<Self> {
        let (mut start_value, mut shutdown_value) = (start_value, shutdown_value);
        if invert {
            start_value = 1.0 - start_value;
            shutdown_value = 1.0 - shutdown_value;
        }
        let p = Rc::new(McuPwmCycle {
            mcu: mcu.clone(),
            cycle_time,
            oid: Cell::new(None),
            pin,
            invert,
            start_value: py_max(0.0, py_min(1.0, start_value)),
            shutdown_value: py_max(0.0, py_min(1.0, shutdown_value)),
            last_clock: Cell::new(0),
            cycle_ticks: Cell::new(0),
            set_cmd: RefCell::new(None),
            set_cycle_ticks: RefCell::new(None),
        });
        let pp = p.clone();
        mcu.register_config_callback(Rc::new(move || pp.build_config()));
        p
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn build_config(&self) -> KResult<()> {
        let mcu = &self.mcu;
        let cmd_queue = mcu.alloc_command_queue();
        let curtime = mcu.get_printer().get_reactor().monotonic();
        let printtime = mcu.estimated_print_time(curtime);
        self.last_clock.set(mcu.print_time_to_clock(printtime + 0.200));
        let cycle_ticks = mcu.seconds_to_clock(self.cycle_time);
        if self.shutdown_value != 0.0 && self.shutdown_value != 1.0 {
            return Err(KError::config("shutdown value must be 0.0 or 1.0 on soft pwm"));
        }
        if cycle_ticks >= 1 << 31 {
            return Err(KError::config("PWM pin cycle time too large"));
        }
        mcu.request_move_queue_slot();
        let oid = mcu.create_oid();
        self.oid.set(Some(oid));
        mcu.add_config_cmd(
            &format!(
                "config_digital_out oid={} pin={} value={} default_value={} max_duration={}",
                oid,
                self.pin,
                (self.start_value >= 1.0) as i64,
                (self.shutdown_value >= 0.5) as i64,
                0
            ),
            false,
            false,
        );
        mcu.add_config_cmd(&format!("set_digital_out_pwm_cycle oid={} cycle_ticks={}", oid, cycle_ticks), false, false);
        self.cycle_ticks.set(cycle_ticks);
        let svalue = (self.start_value * cycle_ticks as f64 + 0.5).trunc() as i64;
        mcu.add_config_cmd(&format!("queue_digital_out oid={} clock={} on_ticks={}", oid, self.last_clock.get(), svalue), true, false);
        *self.set_cmd.borrow_mut() = Some(mcu.lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue.clone()))?);
        *self.set_cycle_ticks.borrow_mut() = Some(mcu.lookup_command("set_digital_out_pwm_cycle oid=%c cycle_ticks=%u", Some(cmd_queue))?);
        Ok(())
    }

    pub fn set_pwm_cycle(&self, print_time: f64, value: f64, cycle_time: f64) -> KResult<()> {
        let oid = self.oid.get().unwrap_or(0);
        let clock = self.mcu.print_time_to_clock(print_time);
        let minclock = self.last_clock.get();
        // Send updated cycle_time if necessary
        let cycle_ticks = self.mcu.seconds_to_clock(cycle_time);
        if cycle_ticks != self.cycle_ticks.get() {
            if cycle_ticks >= 1 << 31 {
                return Err(KError::command("PWM cycle time too large"));
            }
            if let Some(cmd) = self.set_cycle_ticks.borrow().as_ref() {
                cmd.send(&[Val::Int(oid), Val::Int(cycle_ticks)], minclock as u64, clock as u64)?;
            }
            self.cycle_ticks.set(cycle_ticks);
        }
        // Send pwm update
        let value = if self.invert { 1.0 - value } else { value };
        let v = (py_max(0.0, py_min(1.0, value)) * self.cycle_ticks.get() as f64 + 0.5).trunc() as i64;
        if let Some(cmd) = self.set_cmd.borrow().as_ref() {
            cmd.send(&[Val::Int(oid), Val::Int(clock), Val::Int(v)], self.last_clock.get() as u64, clock as u64)?;
        }
        self.last_clock.set(clock);
        Ok(())
    }
}

const CMD_SET_PIN_HELP: &str = "Set the value of an output pin";

pub struct PrinterOutputPWMCycle {
    printer: Rc<crate::printer::Printer>,
    pub last_print_time: Cell<f64>,
    pub scale: f64,
    pub last_value: Cell<f64>,
    pub mcu_pin: Rc<McuPwmCycle>,
    pub last_cycle_time: Cell<f64>,
    pub default_cycle_time: f64,
    pub shutdown_value: f64,
}

impl PrinterOutputPWMCycle {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Determine start and shutdown values
        let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
        let last_value = config.getfloat("value", 0.0, Lim::new().min(0.0).max(scale))? / scale;
        let shutdown_value = config.getfloat("shutdown_value", 0.0, Lim::new().min(0.0).max(scale))? / scale;
        // Create pwm pin object
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin_desc = config.get("pin", Dflt::Required)?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(&pin_desc, true, false, None).map_err(|e| KError::Pins(e.0))?;
        let (chip_name, pin, invert) = {
            let p = pin_params.borrow();
            (p.chip_name.clone(), p.pin.clone(), p.invert != 0)
        };
        let mcu = get_printer_mcu(&printer, &chip_name)?;
        let max_duration = mcu.max_nominal_duration();
        let cycle_time = config.getfloat("cycle_time", 0.100, Lim::new().above(0.0).max(max_duration))?;
        let mcu_pin = McuPwmCycle::new(&mcu, pin, invert, cycle_time, last_value, shutdown_value);
        let o = Rc::new(PrinterOutputPWMCycle {
            printer: printer.clone(),
            last_print_time: Cell::new(0.0),
            scale,
            last_value: Cell::new(last_value),
            mcu_pin,
            last_cycle_time: Cell::new(cycle_time),
            default_cycle_time: cycle_time,
            shutdown_value,
        });
        // Register commands
        let pin_name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let oo = o.clone();
        gcode.register_mux_command("SET_PIN", "PIN", Some(&pin_name), Rc::new(move |g| oo.cmd_set_pin(g)), Some(CMD_SET_PIN_HELP))?;
        Ok(o)
    }

    pub fn set_pin(&self, print_time: f64, value: f64, cycle_time: f64) -> KResult<()> {
        if value == self.last_value.get() && cycle_time == self.last_cycle_time.get() {
            return Ok(());
        }
        let min_sched_time = self.mcu_pin.get_mcu().min_schedule_time();
        let print_time = py_max(print_time, self.last_print_time.get() + min_sched_time);
        self.mcu_pin.set_pwm_cycle(print_time, value, cycle_time)?;
        self.last_value.set(value);
        self.last_cycle_time.set(cycle_time);
        self.last_print_time.set(print_time);
        Ok(())
    }

    pub fn cmd_set_pin(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        // Read requested value
        let value = gcmd.get_float("VALUE", None, Lim::new().min(0.0).max(self.scale))? / self.scale;
        let max_duration = self.mcu_pin.get_mcu().max_nominal_duration();
        let cycle_time = gcmd.get_float("CYCLE_TIME", Some(self.default_cycle_time), Lim::new().above(0.0).max(max_duration))?;
        // Obtain print_time and apply requested settings
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let s = self.clone();
        toolhead.register_lookahead_callback(Rc::new(move |print_time| s.set_pin(print_time, value, cycle_time)))
    }
}

impl PrinterObject for PrinterOutputPWMCycle {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::default();
        d.insert("value".into(), PyVal::Float(self.last_value.get()));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterOutputPWMCycle::new(config)?)
}
