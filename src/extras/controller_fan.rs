// Support a fan for cooling the MCU whenever a stepper or heater is on
// (port of klippy/extras/controller_fan.py)
//
// Copyright (C) 2019  Nils Friedchen <nils.friedchen@googlemail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::fan::Fan;
use crate::extras::heaters::{load_heaters, Heater, PrinterHeaters};
use crate::extras::stepper_enable::PrinterStepperEnable;
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::py_str_repr;
use crate::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const PIN_MIN_TIME: f64 = 0.100;

pub struct ControllerFan {
    printer: Rc<Printer>,
    stepper_names: RefCell<Option<Vec<String>>>,
    stepper_enable: Rc<PrinterStepperEnable>,
    heaters: RefCell<Vec<Rc<Heater>>>,
    pub fan: Rc<Fan>,
    fan_speed: f64,
    idle_speed: f64,
    standby_speed: f64,
    idle_timeout: i64,
    heater_names: Vec<String>,
    last_on: Cell<i64>,
    last_speed: Cell<f64>,
}

impl ControllerFan {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // klippy:ready and klippy:connect are registered before Fan() registers its handlers
        let slot: Rc<RefCell<Weak<ControllerFan>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                if let Some(cf) = sl.borrow().upgrade() {
                    cf.handle_ready();
                }
                Ok(())
            }),
        );
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match sl.borrow().upgrade() {
                Some(cf) => cf.handle_connect(),
                None => Ok(()),
            }),
        );
        let stepper_names = config.getlist("stepper", Dflt::None, ",", None)?;
        let stepper_enable = printer.load_as::<PrinterStepperEnable>(config, "stepper_enable")?;
        load_heaters(config)?;
        let fan = Fan::new(config, 0.0)?;
        let fan_speed = config.getfloat("fan_speed", 1.0, Lim::new().min(0.0).max(1.0))?;
        let idle_speed = config.getfloat("idle_speed", fan_speed, Lim::new().min(0.0).max(1.0))?;
        let standby_speed = config.getfloat("standby_speed", 0.0, Lim::new().min(0.0).max(1.0))?;
        let idle_timeout = config.getint("idle_timeout", 30, Lim::new().min(0))?;
        let heater_names = config.getlist("heater", PyVal::Tuple(vec![PyVal::from("extruder")]), ",", None)?.unwrap_or_default();
        let cf = Rc::new(ControllerFan {
            printer: printer.clone(),
            stepper_names: RefCell::new(stepper_names),
            stepper_enable,
            heaters: RefCell::new(vec![]),
            fan,
            fan_speed,
            idle_speed,
            standby_speed,
            idle_timeout,
            heater_names,
            last_on: Cell::new(idle_timeout),
            last_speed: Cell::new(0.0),
        });
        *slot.borrow_mut() = Rc::downgrade(&cf);
        Ok(cf)
    }

    fn handle_connect(&self) -> KResult<()> {
        // Heater lookup
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let mut heaters = vec![];
        for n in &self.heater_names {
            heaters.push(pheaters.lookup_heater(n)?);
        }
        *self.heaters.borrow_mut() = heaters;
        // Stepper lookup
        let all_steppers = self.stepper_enable.get_steppers();
        let names = self.stepper_names.borrow().clone();
        let Some(names) = names else {
            *self.stepper_names.borrow_mut() = Some(all_steppers);
            return Ok(());
        };
        if !names.iter().all(|x| all_steppers.contains(x)) {
            let repr: Vec<String> = names.iter().map(|n| py_str_repr(n)).collect();
            return Err(KError::config(format!(
                "One or more of these steppers are unknown: [{}] (valid steppers are: {})",
                repr.join(", "),
                all_steppers.join(", ")
            )));
        }
        Ok(())
    }

    fn handle_ready(self: &Rc<Self>) {
        let reactor = self.printer.get_reactor();
        let w = Rc::downgrade(self);
        reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(cf) => cf.callback(eventtime),
                None => crate::reactor::NEVER,
            }),
            reactor.monotonic() + PIN_MIN_TIME,
        );
    }

    fn callback(&self, eventtime: f64) -> f64 {
        let mut speed = self.standby_speed;
        let mut active = false;
        for name in self.stepper_names.borrow().iter().flatten() {
            if let Ok(e) = self.stepper_enable.lookup_enable(name) {
                active |= e.is_motor_enabled();
            }
        }
        for heater in self.heaters.borrow().iter() {
            let (_, target_temp) = heater.get_temp(eventtime);
            if target_temp != 0.0 {
                active = true;
            }
        }
        if active {
            self.last_on.set(0);
            speed = self.fan_speed;
        } else if self.last_on.get() < self.idle_timeout {
            speed = self.idle_speed;
            self.last_on.set(self.last_on.get() + 1);
        }
        if speed != self.last_speed.get() {
            self.last_speed.set(speed);
            if let Err(e) = self.fan.set_speed(speed, None) {
                logging::exception_err("controller_fan set_speed error", &e);
            }
        }
        eventtime + 1.0
    }
}

impl PrinterObject for ControllerFan {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.fan.get_status(eventtime))
    }
    fn teardown(&self) {
        self.heaters.borrow_mut().clear();
        self.fan.teardown();
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ControllerFan::new(config)?)
}
