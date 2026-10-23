// Printer cooling fan (port of klippy/extras/fan.py)
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::output_pin::{Action, GCodeRequestQueue};
use crate::extras::pulse_counter::FrequencyCounter;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::{DigitalOutPin, Mcu, PwmPin};
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub struct Fan {
    last_fan_value: Cell<f64>,
    last_req_value: Cell<f64>,
    last_pwm_value: Cell<f64>,
    kick_start_time: f64,
    min_power: f64,
    pub max_power: f64,
    initial_speed: Option<f64>,
    mcu_fan: Rc<dyn PwmPin>,
    enable_pin: Option<Rc<dyn DigitalOutPin>>,
    gcrq: RefCell<Option<Rc<GCodeRequestQueue>>>,
    tachometer: FanTachometer,
}

impl Fan {
    pub fn new(config: &ConfigWrapper, default_shutdown_speed: f64) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Read config
        let kick_start_time = config.getfloat("kick_start_time", 0.1, Lim::new().min(0.0))?;
        let min_power = config.getfloat_opt("min_power", Lim::new().min(0.0).max(1.0))?;
        let off_below = config.getfloat_opt("off_below", Lim::new().min(0.0).max(1.0))?;
        if off_below.is_some() {
            config.deprecate("off_below", None);
        }
        let initial_speed = config.getfloat_opt("initial_speed", Lim::new().min(0.0).max(1.0))?;

        // handles switchover of variable
        // if new var is not set, and old var is, set new var to old var
        // if new var is not set and neither is old var, set new var to default of 0.0
        // if new var is set, use new var
        if min_power.is_some() && off_below.is_some() {
            return Err(KError::config("min_power and off_below are both set. Remove one!"));
        }
        let min_power = min_power.or(off_below).unwrap_or(0.0);

        let max_power = config.getfloat("max_power", 1.0, Lim::new().above(0.0).max(1.0))?;
        if min_power > max_power {
            return Err(KError::config(format!(
                "min_power={:.6} can't be larger than max_power={:.6}",
                min_power, max_power
            )));
        }

        let cycle_time = config.getfloat("cycle_time", 0.010, Lim::new().above(0.0))?;
        let hardware_pwm = config.getboolean("hardware_pwm", false)?;
        let shutdown_speed = config.getfloat("shutdown_speed", default_shutdown_speed, Lim::new().min(0.0).max(1.0))?;
        // Setup pwm object
        let mcu_fan = crate::pins::setup_pwm_pin(&printer, &config.get("pin", Dflt::Required)?)?;
        mcu_fan.setup_max_duration(0.0);
        mcu_fan.setup_cycle_time(cycle_time, hardware_pwm)?;

        let shutdown_power = if hardware_pwm {
            py_max(0.0, py_min(max_power, shutdown_speed))
        } else {
            // the config allows shutdown_power to be > 0 and < 1, but it is validated
            // in MCU_pwm._build_config().
            py_max(0.0, shutdown_speed)
        };

        mcu_fan.setup_start_value(0.0, shutdown_power);
        let enable_pin = match config.get_opt("enable_pin")? {
            None => None,
            Some(p) => {
                let pin = crate::pins::setup_digital_out_pin(&printer, &p)?;
                pin.setup_max_duration(0.0);
                Some(pin)
            }
        };

        let fan = Rc::new(Fan {
            last_fan_value: Cell::new(0.0),
            last_req_value: Cell::new(0.0),
            last_pwm_value: Cell::new(0.0),
            kick_start_time,
            min_power,
            max_power,
            initial_speed,
            mcu_fan: mcu_fan.clone(),
            enable_pin,
            gcrq: RefCell::new(None),
            tachometer: FanTachometer::new(config)?,
        });
        // Create gcode request queue
        let w = Rc::downgrade(&fan);
        let gcrq = GCodeRequestQueue::new(
            config,
            &mcu_fan.get_mcu(),
            Rc::new(move |print_time, value| match w.upgrade() {
                Some(f) => f.apply_speed(print_time, value),
                None => Ok(None),
            }),
        )?;
        *fan.gcrq.borrow_mut() = Some(gcrq);

        // Register callbacks
        let f = fan.clone();
        printer.register_event_handler(
            "gcode:request_restart",
            Rc::new(move |args| {
                let print_time = args.first().and_then(|a| a.downcast_ref::<f64>()).copied();
                f.set_speed(0.0, print_time)
            }),
        );
        let f = fan.clone();
        printer.register_event_handler("klippy:ready", Rc::new(move |_| f.handle_ready()));
        Ok(fan)
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_fan.get_mcu()
    }

    fn gcrq(&self) -> Rc<GCodeRequestQueue> {
        self.gcrq.borrow().clone().expect("gcrq")
    }

    fn apply_speed(&self, print_time: f64, mut value: f64) -> KResult<Option<(Action, f64)>> {
        if value == self.last_fan_value.get() {
            return Ok(Some((Action::Discard, 0.0)));
        }
        let pwm_value = if value > 0.0 {
            // Scale value between min_power and max_power
            value = py_min(value, 1.0);
            value * (self.max_power - self.min_power) + self.min_power
        } else {
            0.0
        };
        if let Some(enable_pin) = &self.enable_pin {
            if value > 0.0 && self.last_fan_value.get() == 0.0 {
                enable_pin.set_digital(print_time, true)?;
            } else if value == 0.0 && self.last_fan_value.get() > 0.0 {
                enable_pin.set_digital(print_time, false)?;
            }
        }
        if value != 0.0
            && value < 1.0
            && self.kick_start_time != 0.0
            && (self.last_fan_value.get() == 0.0 || value - self.last_fan_value.get() > 0.5)
        {
            // Run fan at full speed for specified kick_start_time
            self.last_req_value.set(value);

            self.last_fan_value.set(1.0);
            self.last_pwm_value.set(self.max_power);

            self.mcu_fan.set_pwm(print_time, self.max_power)?;

            return Ok(Some((Action::Delay, self.kick_start_time)));
        }
        self.last_fan_value.set(value);
        self.last_req_value.set(value);
        self.last_pwm_value.set(pwm_value);
        self.mcu_fan.set_pwm(print_time, pwm_value)?;
        Ok(None)
    }

    pub fn set_speed(&self, value: f64, print_time: Option<f64>) -> KResult<()> {
        self.gcrq().send_async_request(value, print_time)
    }

    pub fn set_speed_from_command(&self, value: f64) -> KResult<()> {
        self.gcrq().queue_gcode_request(value)
    }

    fn handle_ready(&self) -> KResult<()> {
        if let Some(s) = self.initial_speed.filter(|s| *s != 0.0) {
            self.set_speed_from_command(s)?;
        }
        Ok(())
    }

    pub fn get_status(&self, eventtime: f64) -> PyVal {
        let tachometer_status = self.tachometer.get_status(eventtime);
        let mut d = IndexMap::new();
        d.insert("power".to_string(), PyVal::Float(self.last_pwm_value.get()));
        d.insert("value".to_string(), PyVal::Float(self.last_req_value.get()));
        d.insert("speed".to_string(), PyVal::Float(self.last_req_value.get() * self.max_power));
        d.insert("rpm".to_string(), tachometer_status);
        PyVal::Dict(d)
    }

    /// not in python: drop the request queue (it refers back to the fan)
    pub fn teardown(&self) {
        *self.gcrq.borrow_mut() = None;
    }
}

impl PrinterObject for Fan {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(Fan::get_status(self, eventtime))
    }
}

pub struct FanTachometer {
    freq_counter: Option<(Rc<FrequencyCounter>, i64)>,
}

impl FanTachometer {
    pub fn new(config: &ConfigWrapper) -> KResult<Self> {
        let printer = config.get_printer();
        let mut freq_counter = None;
        if let Some(pin) = config.get_opt("tachometer_pin")? {
            let ppr = config.getint("tachometer_ppr", 2, Lim::new().min(1))?;
            let poll_time = config.getfloat("tachometer_poll_interval", 0.0015, Lim::new().above(0.0))?;
            let sample_time = 1.0;
            freq_counter = Some((FrequencyCounter::new(&printer, &pin, sample_time, poll_time)?, ppr));
        }
        Ok(FanTachometer { freq_counter })
    }

    /// get_status(eventtime)["rpm"]
    pub fn get_status(&self, _eventtime: f64) -> PyVal {
        match &self.freq_counter {
            Some((fc, ppr)) => PyVal::Float(fc.get_frequency() * 30.0 / *ppr as f64),
            None => PyVal::None,
        }
    }
}

pub struct PrinterFan {
    pub fan: Rc<Fan>,
}

impl PrinterFan {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let pf = Rc::new(PrinterFan { fan: Fan::new(config, 0.0)? });
        // Register commands
        let gcode = config.get_printer().lookup::<GCodeDispatch>("gcode")?;
        let p = pf.clone();
        gcode.register_command("M106", Some(Rc::new(move |g| p.cmd_m106(g))), false, None)?;
        let p = pf.clone();
        gcode.register_command("M107", Some(Rc::new(move |_| p.fan.set_speed_from_command(0.0))), false, None)?;
        Ok(pf)
    }

    fn cmd_m106(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Set fan speed
        let value = gcmd.get_float("S", Some(255.0), Lim::new().min(0.0))? / 255.0;
        self.fan.set_speed_from_command(value)
    }
}

impl PrinterObject for PrinterFan {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.fan.get_status(eventtime))
    }
    fn fan(&self) -> Option<Rc<Fan>> {
        Some(self.fan.clone())
    }
    fn teardown(&self) {
        self.fan.teardown();
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterFan::new(config)?)
}
