// Support for Raspberry Pi temperature sensor
// (port of klippy/extras/temperature_host.py)
//
// Copyright (C) 2020  Al Crate <al3ph@users.noreply.github.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{load_heaters, TempCallback, TempSensor};
use crate::logging;
use crate::mcu::Mcu;
use crate::printer::{none_object, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_float_error, py_round};
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::io::{Read, Seek};
use std::rc::{Rc, Weak};

const HOST_REPORT_TIME: f64 = 1.0;
const RPI_PROC_TEMP_FILE: &str = "/sys/class/thermal/thermal_zone0/temp";

pub struct TemperatureHost {
    printer: Rc<Printer>,
    temp: Cell<f64>,
    min_temp: Cell<f64>,
    max_temp: Cell<f64>,
    sample_timer: RefCell<Option<Timer>>,
    file_handle: RefCell<Option<std::fs::File>>,
    callback: RefCell<Option<TempCallback>>,
}

impl TemperatureHost {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let path = config.get("sensor_path", RPI_PROC_TEMP_FILE)?;
        let s = Rc::new(TemperatureHost {
            printer: printer.clone(),
            temp: Cell::new(0.0),
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            sample_timer: RefCell::new(None),
            file_handle: RefCell::new(None),
            callback: RefCell::new(None),
        });
        printer.add_object(&format!("temperature_host {}", name), s.clone())?;
        if printer.start_arg("debugoutput").is_some() {
            return Ok(s);
        }
        let w: Weak<TemperatureHost> = Rc::downgrade(&s);
        *s.sample_timer.borrow_mut() = Some(printer.get_reactor().register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.sample_pi_temperature(eventtime),
                None => NEVER,
            }),
            NEVER,
        ));
        match std::fs::File::open(&path) {
            Ok(f) => *s.file_handle.borrow_mut() = Some(f),
            Err(e) => {
                return Err(KError::config(format!("Unable to open temperature file '{}'", path)).with_context(KError::os(&e, &path)));
            }
        }

        let ss = s.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| ss.handle_connect()));
        Ok(s)
    }

    fn handle_connect(&self) -> KResult<()> {
        if let Some(t) = self.sample_timer.borrow().as_ref() {
            self.printer.get_reactor().update_timer(t, NOW);
        }
        Ok(())
    }

    /// float(file_handle.read()) / 1000.0 after seeking to the start
    fn read_temp(&self) -> KResult<f64> {
        let mut fh = self.file_handle.borrow_mut();
        let f = fh.as_mut().ok_or_else(|| KError::Other("no temperature file".into()))?;
        let mut data = String::new();
        f.seek(std::io::SeekFrom::Start(0)).and_then(|_| f.read_to_string(&mut data)).map_err(|e| KError::os(&e, ""))?;
        let v = py_float(&data).ok_or_else(|| KError::Py("ValueError".into(), py_float_error(&data)))?;
        Ok(v / 1000.0)
    }

    fn sample_pi_temperature(&self, _eventtime: f64) -> f64 {
        match self.read_temp() {
            Ok(t) => self.temp.set(t),
            Err(e) => {
                logging::exception_err("temperature_host: Error reading data", &e);
                self.temp.set(0.0);
                return NEVER;
            }
        }

        let temp = self.temp.get();
        if (temp < self.min_temp.get() || temp > self.max_temp.get()) && !get_danger_options().temp_ignore_limits {
            self.printer.invoke_shutdown(
                &format!("HOST temperature {:.1} outside range of {:.1}:{:.1}", temp, self.min_temp.get(), self.max_temp.get()),
                None,
            );
        }

        let measured_time = self.printer.get_reactor().monotonic();
        let res = self.printer.lookup::<Mcu>("mcu").and_then(|mcu| {
            let cb = self.callback.borrow().clone();
            match cb {
                Some(cb) => cb(mcu.estimated_print_time(measured_time), temp),
                None => Err(KError::Py("AttributeError".into(), "'Temperature_HOST' object has no attribute '_callback'".into())),
            }
        });
        if let Err(e) = res {
            self.printer.unhandled_exception(&e);
            return NEVER;
        }
        measured_time + HOST_REPORT_TIME
    }
}

impl TempSensor for TemperatureHost {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
        Ok(())
    }
    fn setup_callback(&self, cb: TempCallback) {
        *self.callback.borrow_mut() = Some(cb);
    }
    fn get_report_time_delta(&self) -> f64 {
        HOST_REPORT_TIME
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl PrinterObject for TemperatureHost {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("temperature".to_string(), PyVal::Float(py_round(self.temp.get(), 2)));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(
        "temperature_host",
        Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(TemperatureHost::new(config)?) }),
    );
    Ok(none_object())
}
