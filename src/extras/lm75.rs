// Support for I2C based LM75/LM75A temperature sensors
// (port of klippy/extras/lm75.py)
//
// Copyright (C) 2020  Boleslaw Ciesielski <combolek@users.noreply.github.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{load_heaters, TempCallback, TempSensor};
use crate::logging;
use crate::mcu::Mcu;
use crate::msgproto::Val;
use crate::printer::{none_object, PObj, Printer, PrinterObject};
use crate::pyfmt::py_round;
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER, NOW};
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const LM75_CHIP_ADDR: i64 = 0x48;
const LM75_I2C_SPEED: i64 = 100000;
/// LM75_REGS
fn lm75_reg(name: &str) -> u8 {
    match name {
        "TEMP" => 0x00,
        "CONF" => 0x01,
        "THYST" => 0x02,
        "TOS" => 0x03,
        _ => 0x07, // PRODID: TI LM75A chips only?
    }
}
const LM75_REPORT_TIME: f64 = 0.8;
// Temperature can be sampled at any time but the read aborts
// the current conversion. Conversion time is 300ms so make
// sure not to read too often.
const LM75_MIN_REPORT_TIME: f64 = 0.5;

pub struct LM75 {
    pub printer: Rc<Printer>,
    pub i2c: Rc<McuI2c>,
    pub mcu: Rc<Mcu>,
    pub report_time: f64,
    pub temp: Cell<f64>,
    pub min_temp: Cell<f64>,
    pub max_temp: Cell<f64>,
    pub sample_timer: RefCell<Option<Timer>>,
    pub callback: RefCell<Option<TempCallback>>,
}

impl LM75 {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let i2c = mcu_i2c_from_config(config, Some(LM75_CHIP_ADDR), LM75_I2C_SPEED)?;
        let mcu = i2c.get_mcu();
        let report_time = config.getfloat("lm75_report_time", LM75_REPORT_TIME, Lim::new().min(LM75_MIN_REPORT_TIME))?;
        let s = Rc::new(LM75 {
            printer: printer.clone(),
            i2c,
            mcu: mcu.clone(),
            report_time,
            temp: Cell::new(0.0),
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            sample_timer: RefCell::new(None),
            callback: RefCell::new(None),
        });
        let w: Weak<LM75> = Rc::downgrade(&s);
        *s.sample_timer.borrow_mut() = Some(printer.get_reactor().register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.sample_lm75(eventtime),
                None => NEVER,
            }),
            NEVER,
        ));
        printer.add_object(&format!("lm75 {}", name), s.clone())?;
        let ss = s.clone();
        crate::events::KlippyConnect::on(&printer, move || ss.handle_connect());
        let ss = s.clone();
        printer.register_event_handler(mcu.get_non_critical_reconnect_event_name(), Rc::new(move |_| ss.handle_connect()));
        Ok(s)
    }

    pub fn handle_connect(&self) -> KResult<()> {
        self.init_lm75();
        if let Some(t) = self.sample_timer.borrow().as_ref() {
            self.printer.get_reactor().update_timer(t, NOW);
        }
        Ok(())
    }

    fn degrees_from_sample(x: &[u8]) -> f64 {
        // The temp sample is encoded in the top 9 bits of a 16-bit
        // value. Resolution is 0.5 degrees C.
        x[0] as f64 + (x[1] >> 7) as f64 * 0.5
    }

    pub fn init_lm75(&self) {
        // Check and report the chip ID but ignore errors since many
        // chips don't have it
        if let Ok(prodid) = self.read_register("PRODID", 1) {
            if let Some(p) = prodid.first() {
                logging::info(&format!("lm75: Chip ID {:#x}", p));
            }
        }
    }

    pub fn sample_lm75(&self, _eventtime: f64) -> f64 {
        let sample = self.read_register("TEMP", 2).and_then(|s| {
            if s.len() < 2 {
                return Err(KError::Py("IndexError".into(), "bytearray index out of range".into()));
            }
            Ok(s)
        });
        match sample {
            Ok(sample) => self.temp.set(Self::degrees_from_sample(&sample)),
            Err(e) => {
                logging::exception_err("lm75: Error reading data", &e);
                self.temp.set(0.0);
                return NEVER;
            }
        }

        let temp = self.temp.get();
        if (temp < self.min_temp.get() || temp > self.max_temp.get()) && !get_danger_options().temp_ignore_limits {
            self.printer.invoke_shutdown(
                &format!("LM75 temperature {:.1} outside range of {:.1}:{:.1}", temp, self.min_temp.get(), self.max_temp.get()),
                None,
            );
        }

        let measured_time = self.printer.get_reactor().monotonic();
        let cb = self.callback.borrow().clone();
        let res = match cb {
            Some(cb) => cb(self.mcu.estimated_print_time(measured_time), temp),
            None => Err(KError::Py("AttributeError".into(), "'LM75' object has no attribute '_callback'".into())),
        };
        if let Err(e) = res {
            self.printer.unhandled_exception(&e);
            return NEVER;
        }
        measured_time + self.report_time
    }

    pub fn read_register(&self, reg_name: &str, read_len: i64) -> KResult<Vec<u8>> {
        // read a single register
        let regs = [lm75_reg(reg_name)];
        let params = self.i2c.i2c_read(&regs, read_len, true)?;
        match params.get("response") {
            Some(Val::Bytes(b)) => Ok(b.clone()),
            _ => Ok(vec![]),
        }
    }
}

impl TempSensor for LM75 {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
        Ok(())
    }
    fn setup_callback(&self, cb: TempCallback) -> KResult<()> {
        *self.callback.borrow_mut() = Some(cb);
        Ok(())
    }
    fn get_report_time_delta(&self) -> KResult<f64> {
        Ok(self.report_time)
    }
}

impl PrinterObject for LM75 {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::default();
        d.insert("temperature".into(), PyVal::Float(py_round(self.temp.get(), 2)));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory("LM75", Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(LM75::new(config)?) }));
    Ok(none_object())
}
