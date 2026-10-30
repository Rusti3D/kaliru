// Support fans that are enabled when a heater is on
// (port of klippy/extras/heater_fan.py)
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::fan::Fan;
use crate::extras::heaters::{load_heaters, Heater, PrinterHeaters};
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const PIN_MIN_TIME: f64 = 0.100;

pub struct PrinterHeaterFan {
    pub printer: Rc<Printer>,
    pub heater_names: Vec<String>,
    pub heater_temp: f64,
    pub heaters: RefCell<Vec<Rc<Heater>>>,
    pub fan: Rc<Fan>,
    pub fan_speed: f64,
    pub last_speed: Cell<f64>,
}

impl PrinterHeaterFan {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        load_heaters(config)?;
        // klippy:ready is registered before Fan() registers its handlers
        let slot: Rc<RefCell<std::rc::Weak<PrinterHeaterFan>>> = Rc::new(RefCell::new(std::rc::Weak::new()));
        let sl = slot.clone();
        crate::events::KlippyReady::on(&printer, move || match sl.borrow().upgrade() {
            Some(hf) => hf.handle_ready(),
            None => Ok(()),
        });
        let heater_names = config.getlist("heater", PyVal::Tuple(vec![PyVal::from("extruder")]), ",", None)?.unwrap_or_default();
        let heater_temp = config.getfloat("heater_temp", 50.0, Lim::new())?;
        let fan = Fan::new(config, 1.0)?;
        let fan_speed = config.getfloat("fan_speed", 1.0, Lim::new().min(0.0).max(1.0))?;
        let hf = Rc::new(PrinterHeaterFan {
            printer: printer.clone(),
            heater_names,
            heater_temp,
            heaters: RefCell::new(vec![]),
            fan,
            fan_speed,
            last_speed: Cell::new(0.0),
        });
        *slot.borrow_mut() = Rc::downgrade(&hf);
        Ok(hf)
    }

    pub fn handle_ready(self: &Rc<Self>) -> KResult<()> {
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let mut heaters = vec![];
        for n in &self.heater_names {
            heaters.push(pheaters.lookup_heater(n)?);
        }
        *self.heaters.borrow_mut() = heaters;
        let reactor = self.printer.get_reactor();
        let w = Rc::downgrade(self);
        reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(hf) => hf.callback(eventtime),
                None => crate::reactor::NEVER,
            }),
            reactor.monotonic() + PIN_MIN_TIME,
        );
        Ok(())
    }

    pub fn callback(&self, eventtime: f64) -> f64 {
        let mut speed = 0.0;
        for heater in self.heaters.borrow().iter() {
            let (current_temp, target_temp) = heater.get_temp(eventtime);
            if target_temp != 0.0 || current_temp > self.heater_temp {
                speed = self.fan_speed;
            }
        }
        if speed != self.last_speed.get() {
            self.last_speed.set(speed);
            if let Err(e) = self.fan.set_speed(speed, None) {
                logging::exception_err("heater_fan set_speed error", &e);
            }
        }
        eventtime + 1.0
    }
}

impl PrinterObject for PrinterHeaterFan {
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
        self.heaters.borrow_mut().clear();
        self.fan.teardown();
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let hf = PrinterHeaterFan::new(config)?;
    Ok(hf)
}
