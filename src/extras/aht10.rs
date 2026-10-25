// Support for AHTxx family I2C temperature and humidity sensors
// (port of klippy/extras/aht10.py)
//
// Copyright (C) 2023 Scott Mudge <mail@scottmudge.com>
// Copyright (C) 2025 Lev Voronov <minicx@disroot.org>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{lookup_heaters, TempCallback, TempSensor};
use crate::logging;
use crate::msgproto::Val;
use crate::printer::{none_object, PObj, Printer, PrinterObject};
use crate::pyfmt::py_round;
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER, NOW};
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

//######################################################################
// Compatible Sensors:
//       AHT10      -    Tested w/ BTT GTR 1.0 MCU on i2c3
//       AHT20      -    Tested w/ N32G455 on i2c2
//       AHT21      -    Tested w/ BTT GTR 1.0 MCU on i2c3
//       AHT30      -    Untested, but should work
//######################################################################

const I2C_ADDR: i64 = 0x38;

const CMD_MEASURE: [u8; 3] = [0xAC, 0x33, 0x00];
const CMD_RESET: [u8; 1] = [0xBA];
const CMD_INIT_AHT1X: [u8; 3] = [0xE1, 0x08, 0x00];
const CMD_INIT_AHT2X: [u8; 3] = [0xBE, 0x08, 0x00];

// Status bits
const STATUS_BUSY: u8 = 0x80;
const STATUS_CALIBRATED: u8 = 0x08;

const MAX_BUSY_CYCLES: i64 = 5;

/// the AHTBase subclasses
#[derive(Clone, Copy)]
enum Model {
    Aht1x,
    Aht2x,
    Aht3x,
}

impl Model {
    fn name(self) -> &'static str {
        match self {
            Model::Aht1x => "aht1x",
            Model::Aht2x => "aht2x",
            Model::Aht3x => "aht3x",
        }
    }
}

pub struct AHTBase {
    model: Model,
    printer: Rc<Printer>,
    name: String,
    i2c: Rc<McuI2c>,
    report_time: i64,
    temp: Cell<f64>,
    min_temp: Cell<f64>,
    max_temp: Cell<f64>,
    /// a float until the first measurement, then python's int
    humidity: Cell<f64>,
    humidity_int: Cell<bool>,
    sample_timer: RefCell<Option<Timer>>,
    is_calibrated: Cell<bool>,
    init_sent: Cell<bool>,
    callback: RefCell<Option<TempCallback>>,
}

impl AHTBase {
    fn new(config: &ConfigWrapper, model: Model) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let i2c = mcu_i2c_from_config(config, Some(I2C_ADDR), 100000)?;
        let mcu = i2c.get_mcu();
        let report_time = config.getint("aht10_report_time", 30, Lim::new().min(5))?;
        let s = Rc::new(AHTBase {
            model,
            printer: printer.clone(),
            name: name.clone(),
            i2c,
            report_time,
            temp: Cell::new(0.0),
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            humidity: Cell::new(0.0),
            humidity_int: Cell::new(false),
            sample_timer: RefCell::new(None),
            is_calibrated: Cell::new(false),
            init_sent: Cell::new(false),
            callback: RefCell::new(None),
        });
        let w: Weak<AHTBase> = Rc::downgrade(&s);
        *s.sample_timer.borrow_mut() = Some(printer.get_reactor().register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.sample_aht(eventtime),
                None => NEVER,
            }),
            NEVER,
        ));

        printer.add_object(&format!("aht10 {}", name), s.clone())?;
        let ss = s.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| ss.handle_connect()));
        let ss = s.clone();
        printer.register_event_handler(mcu.get_non_critical_reconnect_event_name(), Rc::new(move |_| ss.handle_connect()));
        Ok(s)
    }

    fn handle_connect(&self) -> KResult<()> {
        self.init_sensor()?;
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

    fn send_init(&self) -> KResult<()> {
        match self.model {
            Model::Aht1x => {
                self.i2c.i2c_write(&CMD_INIT_AHT1X, 0, 0)?;
                self.pause(0.040)
            }
            Model::Aht2x => {
                self.i2c.i2c_write(&CMD_INIT_AHT2X, 0, 0)?;
                self.pause(0.100)
            }
            // Wait for auto-calibration at power-on
            Model::Aht3x => self.pause(0.100),
        }
    }

    fn init_sensor(&self) -> KResult<()> {
        self.send_init()?;

        self.init_sent.set(true);

        if self.make_measurement() {
            if !self.is_calibrated.get() {
                logging::warning(&format!("{} {}: not calibrated, possible OTP fault", self.model.name(), self.name));
            }
            logging::info(&format!(
                "{} {}: successfully initialized, initial temp: {:.3}, humidity: {:.3}",
                self.model.name(),
                self.name,
                self.temp.get(),
                self.humidity.get()
            ));
        }
        Ok(())
    }

    fn soft_reset(&self) -> KResult<()> {
        logging::info(&format!("{} {}: performing soft reset", self.model.name(), self.name));
        self.i2c.i2c_write(&CMD_RESET, 0, 0)?;
        self.pause(0.020)
    }

    /// the busy loop of _make_measurement: the last 6 data bytes, None
    /// when the device stays busy
    fn read_data(&self) -> KResult<Option<Vec<u8>>> {
        let mut data = None;
        let mut is_busy = true;
        let mut cycles = 0;
        while is_busy {
            // Check if we're constantly busy. If so, send soft-reset
            // and issue warning.
            if is_busy && cycles > MAX_BUSY_CYCLES {
                logging::warning(&format!(
                    "{} {}: device reported busy after {} cycles, resetting device",
                    self.model.name(),
                    self.name,
                    MAX_BUSY_CYCLES
                ));
                self.soft_reset()?;
                data = None;
                break;
            }

            cycles += 1;
            // Write command for updating temperature+status bit
            self.i2c.i2c_write(&CMD_MEASURE, 0, 0)?;
            // Wait 110ms after first read, 75ms minimum
            self.pause(0.110)?;

            // Read 6 bytes of data
            let read = self.i2c.i2c_read(&[], 6, true)?;
            let d = match read.get("response") {
                Some(Val::Bytes(b)) => b.clone(),
                _ => vec![],
            };
            if d.len() < 6 {
                logging::warning(&format!("{} {}: received bytes less than expected: got {}, need 6", self.model.name(), self.name, d.len()));
                data = Some(d);
                continue;
            }

            self.is_calibrated.set(d[0] & STATUS_CALIBRATED != 0);
            is_busy = d[0] & STATUS_BUSY != 0;
            data = Some(d);
        }

        if is_busy {
            return Ok(None);
        }
        Ok(data)
    }

    fn make_measurement(&self) -> bool {
        if !self.init_sent.get() {
            return false;
        }

        let data = match self.read_data() {
            Ok(Some(d)) => d,
            Ok(None) => return false,
            Err(e) => {
                logging::exception_err(&format!("{} {}: exception encountered reading data: {}", self.model.name(), self.name, e), &e);
                return false;
            }
        };

        // Parse temperature: 20 bits starting at data[3] (low nibble)
        let temp_raw = (((data[3] & 0x0F) as i64) << 16) | ((data[4] as i64) << 8) | data[5] as i64;
        self.temp.set(((temp_raw as f64 * 200.0) / 1048576.0) - 50.0);

        // Parse humidity: 20 bits starting at data[1]
        let hum_raw = (((data[1] as i64) << 16) | ((data[2] as i64) << 8) | data[3] as i64) >> 4;
        let humidity = ((hum_raw * 100) as f64 / 1048576.0).trunc() as i64;

        // Clamp humidity
        self.humidity.set(humidity.clamp(0, 100) as f64);
        self.humidity_int.set(true);

        true
    }

    fn sample_aht(&self, _eventtime: f64) -> f64 {
        if !self.make_measurement() {
            self.temp.set(0.0);
            self.humidity.set(0.0);
            self.humidity_int.set(false);
            return NEVER;
        }

        let temp = self.temp.get();
        // (python's precedence: the limits override only the upper bound)
        if temp < self.min_temp.get() || temp > self.max_temp.get() && !get_danger_options().temp_ignore_limits {
            self.printer.invoke_shutdown(
                &format!(
                    "{} temperature {:.1} outside range of {:.1}:{:.1}",
                    self.model.name().to_uppercase(),
                    temp,
                    self.min_temp.get(),
                    self.max_temp.get()
                ),
                None,
            );
        }

        let measured_time = self.printer.get_reactor().monotonic();
        let print_time = self.i2c.get_mcu().estimated_print_time(measured_time);
        let cb = self.callback.borrow().clone();
        let res = match cb {
            Some(cb) => cb(print_time, temp),
            None => Err(KError::Py("TypeError".into(), "'NoneType' object is not callable".into())),
        };
        if let Err(e) = res {
            self.printer.unhandled_exception(&e);
            return NEVER;
        }
        measured_time + self.report_time as f64
    }
}

impl TempSensor for AHTBase {
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
}

impl PrinterObject for AHTBase {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let humidity = match self.humidity_int.get() {
            true => PyVal::Int(self.humidity.get() as i64),
            false => PyVal::Float(self.humidity.get()),
        };
        let mut d = IndexMap::new();
        d.insert("temperature".to_string(), PyVal::Float(py_round(self.temp.get(), 2)));
        d.insert("humidity".to_string(), humidity);
        Some(PyVal::Dict(d))
    }
}

fn factory(model: Model) -> crate::extras::heaters::SensorFactory {
    Rc::new(move |config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(AHTBase::new(config, model)?) })
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheater = lookup_heaters(config)?;

    // Backwards compatibility alias
    pheater.add_sensor_factory("AHT10", factory(Model::Aht1x));

    pheater.add_sensor_factory("AHT1X", factory(Model::Aht1x));
    pheater.add_sensor_factory("AHT2X", factory(Model::Aht2x));
    pheater.add_sensor_factory("AHT3X", factory(Model::Aht3x));
    Ok(none_object())
}
