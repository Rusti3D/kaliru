// Support for combination of temperature sensors
// (port of klippy/extras/temperature_combined.py)
//
// Copyright (C) 2023  Michael Jäger <michael@mjaeger.eu>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{load_heaters, TempCallback, TempSensor};
use crate::mcu::Mcu;
use crate::printer::{none_object, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_round, py_sum};
use crate::pyval::{PyDict, PyVal};
use crate::reactor::{Timer, NEVER};
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const REPORT_TIME: f64 = 0.300;

#[derive(Clone, Copy)]
enum Mode {
    Min,
    Max,
    Mean,
}

impl Mode {
    /// apply_mode(values); None is python's mean() of an empty list
    fn apply(self, values: &[f64]) -> Option<f64> {
        match self {
            Mode::Min => values.iter().copied().reduce(py_min),
            Mode::Max => values.iter().copied().reduce(py_max),
            Mode::Mean if values.is_empty() => None,
            Mode::Mean => Some(py_sum(values.iter().copied()) / values.len() as f64),
        }
    }
}

pub struct PrinterSensorCombined {
    printer: Rc<Printer>,
    sensor_names: Vec<String>,
    max_deviation: f64,
    sensors: RefCell<Vec<PObj>>,
    apply_mode: Mode,
    last_temp: Cell<f64>,
    min_temp: Cell<f64>,
    max_temp: Cell<f64>,
    humidity: Cell<Option<f64>>,
    pressure: Cell<Option<f64>>,
    gas: Cell<Option<f64>>,
    temperature_update_timer: RefCell<Option<Timer>>,
    temperature_callback: RefCell<Option<TempCallback>>,
}

fn status_of(sensor: &PObj, eventtime: f64) -> PyDict {
    match sensor.get_status(eventtime) {
        Some(PyVal::Dict(d)) => d,
        _ => IndexMap::default(),
    }
}

impl PrinterSensorCombined {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        // get sensor names
        let sensor_names = config.getlist("sensor_list", Dflt::Required, ",", None)?.unwrap_or_default();
        // get maximum_deviation parameter from config
        let max_deviation = config.getfloat("maximum_deviation", Dflt::Required, Lim::new().above(0.0))?;
        // get combination method to handle the different sensor values with
        let algos = [("min", Mode::Min), ("max", Mode::Max), ("mean", Mode::Mean)];
        let apply_mode = config.getchoice("combination_method", &algos, Dflt::Required)?;
        let s = Rc::new(PrinterSensorCombined {
            printer: printer.clone(),
            sensor_names,
            max_deviation,
            sensors: RefCell::new(vec![]),
            apply_mode,
            // set default values
            last_temp: Cell::new(0.0),
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            humidity: Cell::new(None),
            pressure: Cell::new(None),
            gas: Cell::new(None),
            temperature_update_timer: RefCell::new(None),
            temperature_callback: RefCell::new(None),
        });
        // add object
        printer.add_object(&format!("temperature_combined {}", name), s.clone())?;
        // time-controlled sensor update
        let w: Weak<Self> = Rc::downgrade(&s);
        let timer = printer.get_reactor().register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.temperature_update_event(eventtime),
                None => NEVER,
            }),
            NEVER,
        );
        *s.temperature_update_timer.borrow_mut() = Some(timer);
        let ss = s.clone();
        crate::events::KlippyConnect::on(&printer, move || ss.handle_connect());
        let ss = s.clone();
        crate::events::KlippyReady::on(&printer, move || {
            ss.handle_ready();
            Ok(())
        });
        Ok(s)
    }

    fn handle_connect(&self) -> KResult<()> {
        let reactor = self.printer.get_reactor();
        for sensor_name in &self.sensor_names {
            let sensor = self.printer.lookup_object(sensor_name)?;
            // check if sensor has get_status function and
            // get_status has a 'temperature' value
            if !sensor.has_status() {
                return Err(KError::config(format!("'{}' does not have a status.", sensor_name)));
            }
            let status = status_of(&sensor, reactor.monotonic());
            match status.get("temperature") {
                None => return Err(KError::config(format!("'{}' does not report a temperature.", sensor_name))),
                // Handle temperature monitors
                Some(PyVal::None) => return Err(KError::config(format!("Temperature monitor '{}' is not supported", sensor_name))),
                Some(_) => {}
            }
            self.sensors.borrow_mut().push(sensor);
        }
        Ok(())
    }

    fn handle_ready(&self) {
        // Start temperature update timer
        // There is a race condition with sensors where they can be not ready,
        // and return 0 or None - initialize a little bit later.
        let reactor = self.printer.get_reactor();
        if let Some(t) = self.temperature_update_timer.borrow().as_ref() {
            reactor.update_timer(t, reactor.monotonic() + 1.0);
        }
    }

    fn update_temp(&self, eventtime: f64) {
        // (a None reading would raise in python's max(); NaN here)
        let values: Vec<f64> = self
            .sensors
            .borrow()
            .iter()
            .map(|s| status_of(s, eventtime).get("temperature").and_then(|v| v.as_f64()).unwrap_or(f64::NAN))
            .collect();
        // check if values are out of max_deviation range
        let (vmax, vmin) = (Mode::Max.apply(&values).unwrap_or(f64::NAN), Mode::Min.apply(&values).unwrap_or(f64::NAN));
        if (vmax - vmin) > self.max_deviation && !get_danger_options().temp_ignore_limits {
            self.printer.invoke_shutdown(
                &format!(
                    "COMBINED SENSOR maximum deviation exceeded limit of {:.1}, max sensor value {:.1}, min sensor value {:.1}.",
                    self.max_deviation, vmax, vmin
                ),
                None,
            );
        }
        if let Some(temp) = self.apply_mode.apply(&values) {
            if temp != 0.0 {
                self.last_temp.set(temp);
            }
        }
    }

    fn update_additional(&self, eventtime: f64) {
        let mut values: [Vec<f64>; 3] = [vec![], vec![], vec![]];
        for sensor in self.sensors.borrow().iter() {
            let sensor_status = status_of(sensor, eventtime);
            for (i, key) in ["humidity", "pressure", "gas"].iter().enumerate() {
                if let Some(v) = sensor_status.get(*key).and_then(|v| v.as_f64()) {
                    values[i].push(v);
                }
            }
        }
        for (vals, cell) in values.iter().zip([&self.humidity, &self.pressure, &self.gas]) {
            if !vals.is_empty() {
                if let Some(v) = self.apply_mode.apply(vals) {
                    if v != 0.0 {
                        cell.set(Some(v));
                    }
                }
            }
        }
    }

    fn temperature_update_event(&self, eventtime: f64) -> f64 {
        // update sensor value
        self.update_temp(eventtime);
        self.update_additional(eventtime);

        // check min / max temp values
        let last_temp = self.last_temp.get();
        if last_temp < self.min_temp.get() {
            self.printer.invoke_shutdown(
                &format!("COMBINED SENSOR temperature {:.1} below minimum temperature of {:.1}.", last_temp, self.min_temp.get()),
                None,
            );
        }
        if last_temp > self.max_temp.get() {
            self.printer.invoke_shutdown(
                &format!("COMBINED SENSOR temperature {:.1} above maximum temperature of {:.1}.", last_temp, self.max_temp.get()),
                None,
            );
        }

        // this is copied from temperature_host to enable time triggered updates
        // get mcu and measured / current(?) time
        let measured_time = self.printer.get_reactor().monotonic();
        if let (Ok(mcu), Some(cb)) = (self.printer.lookup::<Mcu>("mcu"), self.temperature_callback.borrow().clone()) {
            // convert to print time?! for the callback???
            if let Err(e) = cb(mcu.estimated_print_time(measured_time), self.last_temp.get()) {
                crate::logging::exception_err("temperature_combined callback", &e);
            }
        }
        // set next update time
        measured_time + REPORT_TIME
    }
}

impl TempSensor for PrinterSensorCombined {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
        Ok(())
    }

    fn setup_callback(&self, cb: TempCallback) {
        *self.temperature_callback.borrow_mut() = Some(cb);
    }

    fn get_report_time_delta(&self) -> f64 {
        REPORT_TIME
    }

}

impl PrinterObject for PrinterSensorCombined {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut data = IndexMap::default();
        data.insert("temperature".into(), PyVal::Float(py_round(self.last_temp.get(), 2)));
        for (key, cell) in [("humidity", &self.humidity), ("pressure", &self.pressure), ("gas", &self.gas)] {
            if let Some(v) = cell.get() {
                data.insert(key.to_string().into(), PyVal::Float(v));
            }
        }
        Some(PyVal::Dict(data))
    }
    fn get_temp(&self, _eventtime: f64) -> Option<(f64, f64)> {
        Some((self.last_temp.get(), 0.0))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(
        "temperature_combined",
        Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(PrinterSensorCombined::new(config)?) }),
    );
    Ok(none_object())
}
