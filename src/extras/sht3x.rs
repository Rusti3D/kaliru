// SHT3X i2c based temperature sensors support
// (port of klippy/extras/sht3x.py)
//
// Copyright (C) 2024  Timofey Titovets <nefelim4ag@gmail.com>
// Based on htu21d.py code
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{lookup_heaters, TempCallback, TempSensor};
use crate::logging;
use crate::msgproto::{Params, Val};
use crate::printer::{none_object, PObj, Printer, PrinterObject};
use crate::pyfmt::py_round;
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

//######################################################################
// Compatible Sensors:
//       SHT31  - Tested on octopus pro and Linux MCU
//
//######################################################################
const SHT3X_I2C_ADDR: i64 = 0x44;

// SHT3X_CMD (the commands used)
const CMD_PERIODIC_2HZ_HIGH_REP: [u8; 2] = [0x22, 0x36];
const CMD_STATUS_READ: [u8; 2] = [0xF3, 0x2D];
const CMD_SOFTRESET: [u8; 2] = [0x30, 0xA2]; // Soft reset
const CMD_FETCH: [u8; 2] = [0xE0, 0x00];
const CMD_BREAK: [u8; 2] = [0x30, 0x93];

fn response(params: &Params) -> Vec<u8> {
    match params.get("response") {
        Some(Val::Bytes(b)) => b.clone(),
        _ => vec![],
    }
}

fn index_error() -> KError {
    KError::Py("IndexError".into(), "bytearray index out of range".into())
}

pub struct SHT3X {
    printer: Rc<Printer>,
    i2c: Rc<McuI2c>,
    report_time: i64,
    temp: Cell<f64>,
    min_temp: Cell<f64>,
    max_temp: Cell<f64>,
    humidity: Cell<f64>,
    sample_timer: RefCell<Option<Timer>>,
    callback: RefCell<Option<TempCallback>>,
}

impl SHT3X {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let i2c = mcu_i2c_from_config(config, Some(SHT3X_I2C_ADDR), 100000)?;
        let mcu = i2c.get_mcu();
        let report_time = config.getint("sht3x_report_time", 1, Lim::new().min(1))?;
        config.get("sensor_type", Dflt::Required)?;
        let s = Rc::new(SHT3X {
            printer: printer.clone(),
            i2c,
            report_time,
            temp: Cell::new(0.0),
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            humidity: Cell::new(0.0),
            sample_timer: RefCell::new(None),
            callback: RefCell::new(None),
        });

        let w: Weak<SHT3X> = Rc::downgrade(&s);
        *s.sample_timer.borrow_mut() = Some(printer.get_reactor().register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.sample_sht3x(eventtime),
                None => NEVER,
            }),
            NEVER,
        ));
        printer.add_object(&format!("sht3x {}", name), s.clone())?;
        let ss = s.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| ss.handle_connect()));
        let ss = s.clone();
        printer.register_event_handler(mcu.get_non_critical_reconnect_event_name(), Rc::new(move |_| ss.handle_connect()));
        Ok(s)
    }

    fn handle_connect(&self) -> KResult<()> {
        self.init_sht3x()?;
        if let Some(t) = self.sample_timer.borrow().as_ref() {
            self.printer.get_reactor().update_timer(t, NOW);
        }
        Ok(())
    }

    fn pause(&self, delay: f64) -> KResult<()> {
        let reactor = self.printer.get_reactor();
        reactor.pause(reactor.monotonic() + delay)?;
        Ok(())
    }

    fn init_sht3x(&self) -> KResult<()> {
        // Device Soft Reset
        self.i2c.i2c_write(&CMD_BREAK, 0, 0)?;
        // Break takes ~ 1ms
        self.pause(0.0015)?;
        self.i2c.i2c_write(&CMD_SOFTRESET, 0, 0)?;
        // Wait <=1.5ms after reset
        self.pause(0.0015)?;

        let response = response(&self.i2c.i2c_read(&CMD_STATUS_READ, 3, true)?);
        if response.len() < 3 {
            return Err(index_error());
        }
        let status = ((response[0] as i64) << 8) | response[1] as i64;
        let checksum = response[2] as i64;

        if Self::crc8(status) != checksum {
            logging::warning("sht3x: Reading status - checksum error!");
        }

        // Enable periodic mode
        self.i2c.i2c_write(&CMD_PERIODIC_2HZ_HIGH_REP, 0, 0)?;
        // Wait <=15.5ms for first measurement
        self.pause(0.0155)
    }

    fn read_measurement(&self) -> KResult<()> {
        // Read measurment
        let mut retries = 5;
        let mut params = None;
        let mut error = None;
        while retries > 0 && params.is_none() {
            match self.i2c.i2c_read(&CMD_FETCH, 6, false) {
                Ok(p) => params = Some(p),
                Err(e) if e.is_command_error() => {
                    error = Some(e);
                    self.pause(0.5)?;
                    retries -= 1;
                }
                Err(e) => return Err(e),
            }
        }
        let Some(params) = params else {
            return Err(error.unwrap_or_else(|| KError::Py("TypeError".into(), "exceptions must derive from BaseException".into())));
        };

        let response = response(&params);
        if response.len() < 6 {
            return Err(index_error());
        }
        let rtemp = ((response[0] as i64) << 8) | response[1] as i64;
        if Self::crc8(rtemp) != response[2] as i64 {
            logging::warning("sht3x: Checksum error on Temperature reading!");
        } else {
            self.temp.set(-45.0 + ((175 * rtemp) as f64 / 65535.0));
            logging::debug(&format!("sht3x: Temperature {:.2} ", self.temp.get()));
        }

        let rhumid = ((response[3] as i64) << 8) | response[4] as i64;
        if Self::crc8(rhumid) != response[5] as i64 {
            logging::warning("sht3x: Checksum error on Humidity reading!");
        } else {
            self.humidity.set((100 * rhumid) as f64 / 65535.0);
            logging::debug(&format!("sht3x: Humidity {:.2} ", self.humidity.get()));
        }
        Ok(())
    }

    fn sample_sht3x(&self, _eventtime: f64) -> f64 {
        if let Err(e) = self.read_measurement() {
            logging::exception_err("sht3x: Error reading data", &e);
            self.temp.set(0.0);
            self.humidity.set(0.0);
            return NEVER;
        }

        let temp = self.temp.get();
        if (temp < self.min_temp.get() || temp > self.max_temp.get()) && !get_danger_options().temp_ignore_limits {
            self.printer.invoke_shutdown(
                &format!("sht3x: temperature {:.1} outside range of {:.1}:{:.1}", temp, self.min_temp.get(), self.max_temp.get()),
                None,
            );
        }

        let measured_time = self.printer.get_reactor().monotonic();
        let print_time = self.i2c.get_mcu().estimated_print_time(measured_time);
        let cb = self.callback.borrow().clone();
        let res = match cb {
            Some(cb) => cb(print_time, temp),
            None => Err(KError::Py("AttributeError".into(), "'SHT3X' object has no attribute '_callback'".into())),
        };
        if let Err(e) = res {
            self.printer.unhandled_exception(&e);
            return NEVER;
        }
        measured_time + self.report_time as f64
    }

    fn crc8(data: i64) -> i64 {
        // crc8 polynomial for 16bit value, CRC8 -> x^8 + x^5 + x^4 + 1
        const SHT3X_CRC8_POLYNOMINAL: i64 = 0x31;
        let mut crc: i64 = 0xFF;
        let data_bytes = [data >> 8 & 0xFF, data & 0xFF];
        for byte in data_bytes {
            crc ^= byte;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ SHT3X_CRC8_POLYNOMINAL;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc & 0xFF
    }
}

impl TempSensor for SHT3X {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
        Ok(())
    }
    fn setup_callback(&self, cb: TempCallback) {
        *self.callback.borrow_mut() = Some(cb);
    }
    fn get_report_time_delta(&self) -> f64 {
        self.report_time as f64
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl PrinterObject for SHT3X {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("temperature".to_string(), PyVal::Float(py_round(self.temp.get(), 2)));
        d.insert("humidity".to_string(), PyVal::Float(py_round(self.humidity.get(), 1)));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheater = lookup_heaters(config)?;
    pheater.add_sensor_factory("SHT3X", Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(SHT3X::new(config)?) }));
    Ok(none_object())
}
