// Support for a heated fan (port of klippy/extras/heated_fan.py)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::fan::Fan;
use crate::extras::heaters::{load_heaters, Heater};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER};
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const PIN_MIN_TIME: f64 = 0.100;

pub struct HeatedFan {
    me: Weak<HeatedFan>,
    printer: Rc<Printer>,
    heater: Rc<Heater>,
    heater_temp: Cell<f64>,
    pub fan: Rc<Fan>,
    min_speed: f64,
    idle_timeout: f64,
    set_speed: Cell<f64>,
    timeout_timer: RefCell<Option<Timer>>,
}

impl HeatedFan {
    /// HeatedFan(config): the printer object, added as "fan" and stored
    /// as "heated_fan" (one python instance)
    fn new(config: &ConfigWrapper) -> KResult<PObj> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        if printer.lookup_object_opt("fan").is_some() {
            return Err(KError::config("Can't setup fan and heated_fan together"));
        }
        // the object is added before the heater and fan exist: it is
        // filled in below
        let slot: Rc<RefCell<Weak<HeatedFan>>> = Rc::new(RefCell::new(Weak::new()));
        let obj: PObj = Rc::new(HeatedFanObject(slot.clone(), RefCell::new(None)));
        printer.add_object("fan", obj.clone())?;
        // setup heater
        let pheaters = load_heaters(config)?;
        let heater = pheaters.setup_heater(config, Some("F"))?;
        let heater_temp = config.getfloat("heater_temp", 50.0, Lim::new())?;
        // setup fan
        let fan = Fan::new(config, 1.0)?;
        let min_speed = config.getfloat("min_speed", 1.0, Lim::new().min(0.0).max(1.0))?;
        let idle_timeout = config.getfloat("idle_timeout", 60.0, Lim::new().min(0.0))?;
        let hf = Rc::new_cyclic(|me| HeatedFan {
            me: me.clone(),
            printer: printer.clone(),
            heater,
            heater_temp: Cell::new(heater_temp),
            fan,
            min_speed,
            idle_timeout,
            set_speed: Cell::new(0.0),
            timeout_timer: RefCell::new(None),
        });
        *slot.borrow_mut() = Rc::downgrade(&hf);
        if let Some(o) = obj.as_any().downcast_ref::<HeatedFanObject>() {
            *o.1.borrow_mut() = Some(hf.clone());
        }
        type HCmd = fn(&HeatedFan, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, HCmd); 3] = [
            ("SET_HEATED_FAN_TARGET", HeatedFan::cmd_set_heated_fan_target),
            ("M106", HeatedFan::cmd_m106),
            ("M107", HeatedFan::cmd_m107),
        ];
        for (name, f) in cmds {
            let w = hf.me.clone();
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(h) => f(&h, g),
                    None => Ok(()),
                })),
                false,
                None,
            )?;
        }
        let w = hf.me.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                if let Some(h) = w.upgrade() {
                    h.handle_ready();
                }
                Ok(())
            }),
        );
        Ok(obj)
    }

    fn cmd_set_heated_fan_target(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.heater_temp.set(gcmd.get_float("TARGET", Some(0.0), Lim::new())?);
        Ok(())
    }

    fn cmd_m106(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.set_speed.set(gcmd.get_float("S", Some(255.0), Lim::new().min(0.0))? / 255.0);
        if self.set_speed.get() == 0.0 {
            return self.cmd_m107(gcmd);
        }
        if self.set_speed.get() < self.min_speed {
            self.set_speed.set(self.min_speed);
        }
        self.fan.set_speed_from_command(self.set_speed.get())?;
        let (_, target_temp) = self.heater.get_temp(self.printer.get_reactor().monotonic());
        // if the heater is off or has a different target temp than configured, set it
        if target_temp == 0.0 || target_temp != self.heater_temp.get() {
            self.heater.set_temp(self.heater_temp.get())?;
        }
        Ok(())
    }

    fn cmd_m107(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.set_speed.set(0.0);
        self.heater.set_temp(0.0)?;
        self.start_timeout_timer()
    }

    fn start_timeout_timer(&self) -> KResult<()> {
        self.fan.set_speed_from_command(self.min_speed)?;
        let reactor = self.printer.get_reactor();
        let w = self.me.clone();
        let timer = reactor.register_timer(
            Box::new(move |_eventtime| {
                if let Some(h) = w.upgrade() {
                    if let Err(e) = h.fan.set_speed_from_command(h.set_speed.get()) {
                        logging::exception_err("heated_fan timeout", &e);
                    }
                }
                NEVER
            }),
            reactor.monotonic() + self.idle_timeout,
        );
        *self.timeout_timer.borrow_mut() = Some(timer);
        Ok(())
    }

    fn handle_ready(&self) {
        let reactor = self.printer.get_reactor();
        let w = self.me.clone();
        reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(h) => h.callback(eventtime),
                None => NEVER,
            }),
            reactor.monotonic() + PIN_MIN_TIME,
        );
    }

    fn callback(&self, eventtime: f64) -> f64 {
        let (_, target_temp) = self.heater.get_temp(eventtime);
        // if the heater has a target and the fan is off, turn it on to min fan speed
        if target_temp != 0.0 && self.set_speed.get() == 0.0 {
            if let Err(e) = self.fan.set_speed_from_command(self.min_speed) {
                logging::exception_err("heated_fan callback", &e);
            }
        }
        eventtime + 1.0
    }

    fn status(&self, eventtime: f64) -> PyVal {
        let mut status = match self.fan.get_status(eventtime) {
            PyVal::Dict(d) => d,
            _ => Default::default(),
        };
        if let PyVal::Dict(h) = self.heater.get_status(eventtime) {
            status.extend(h);
        }
        PyVal::Dict(status)
    }
}

/// the HeatedFan printer object; python adds it as "fan" before its heater
/// and fan exist (the second field keeps the HeatedFan alive)
struct HeatedFanObject(Rc<RefCell<Weak<HeatedFan>>>, RefCell<Option<Rc<HeatedFan>>>);

impl HeatedFanObject {
    fn get(&self) -> Option<Rc<HeatedFan>> {
        self.0.borrow().upgrade()
    }
}

impl PrinterObject for HeatedFanObject {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        self.get().map(|h| h.status(eventtime))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        self.get().map(|h| h.heater.stats(eventtime))
    }
    fn fan(&self) -> Option<Rc<Fan>> {
        self.get().map(|h| h.fan.clone())
    }
    fn teardown(&self) {
        if let Some(h) = self.1.borrow_mut().take() {
            h.fan.teardown();
        }
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    HeatedFan::new(config)
}
