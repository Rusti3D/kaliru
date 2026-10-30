// Support generic temperature sensors (port of klippy/extras/temperature_sensor.py)
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::heaters::{load_heaters, TempSensor};
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_round};
use crate::pyval::PyVal;
use std::cell::Cell;
use std::rc::Rc;

pub const KELVIN_TO_CELSIUS: f64 = -273.15;

pub struct PrinterSensorGeneric {
    pub name: String,
    _sensor: Rc<dyn TempSensor>,
    pub min_temp: f64,
    pub max_temp: f64,
    pub last_temp: Cell<f64>,
    pub measured_min: Cell<f64>,
    pub measured_max: Cell<f64>,
}

impl PrinterSensorGeneric {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let pheaters = load_heaters(config)?;
        let sensor = pheaters.setup_sensor(config)?;
        let min_temp = config.getfloat("min_temp", KELVIN_TO_CELSIUS, Lim::new().min(KELVIN_TO_CELSIUS))?;
        let max_temp = config.getfloat("max_temp", 99999999.9, Lim::new().above(min_temp))?;
        sensor.setup_minmax(min_temp, max_temp)?;
        let s = Rc::new(PrinterSensorGeneric {
            name,
            _sensor: sensor.clone(),
            min_temp,
            max_temp,
            last_temp: Cell::new(0.0),
            measured_min: Cell::new(99999999.0),
            measured_max: Cell::new(0.0),
        });
        let w = Rc::downgrade(&s);
        sensor.setup_callback(Rc::new(move |read_time, temp| {
            if let Some(s) = w.upgrade() {
                s.temperature_callback(read_time, temp);
            }
            Ok(())
        }))?;
        pheaters.register_sensor(config, s.clone(), None)?;
        Ok(s)
    }

    pub fn temperature_callback(&self, _read_time: f64, temp: f64) {
        self.last_temp.set(temp);
        if temp != 0.0 {
            self.measured_min.set(py_min(self.measured_min.get(), temp));
            self.measured_max.set(py_max(self.measured_max.get(), temp));
        }
    }

    pub fn is_adc_faulty(&self) -> bool {
        self.last_temp.get() > self.max_temp || self.last_temp.get() < self.min_temp
    }
}

impl PrinterObject for PrinterSensorGeneric {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let d = crate::pydict! {
            "temperature" => PyVal::Float(py_round(self.last_temp.get(), 2)),
            "measured_min_temp" => PyVal::Float(py_round(self.measured_min.get(), 2)),
            "measured_max_temp" => PyVal::Float(py_round(self.measured_max.get(), 2)),
        };
        Some(PyVal::Dict(d))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        Some((false, format!("{}: temp={:.1}", self.name, self.last_temp.get())))
    }
    fn get_temp(&self, _eventtime: f64) -> Option<(f64, f64)> {
        Some((self.last_temp.get(), 0.0))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterSensorGeneric::new(config)?)
}
