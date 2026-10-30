// HTU21D(F)/Si7013/Si7020/Si7021/SHT21 i2c based temperature sensors support
// (port of klippy/extras/htu21d.py)
//
// Copyright (C) 2020  Lucio Tarantino <lucio.tarantino@gmail.com>
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
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

//######################################################################
// NOTE: The implementation requires write support of length 0
//       before reading on the i2c bus of the mcu.
//
// Compatible Sensors:
//       HTU21D - Tested on Linux MCU.
//       Si7013 - Untested
//       Si7020 - Untested
//       Si7021 - Tested on Pico MCU
//       SHT21  - Tested on Linux MCU.
//
//######################################################################

const HTU21D_I2C_ADDR: i64 = 0x40;

// HTU21D_COMMANDS
const HTU21D_TEMP: u8 = 0xE3;
const HTU21D_HUMID: u8 = 0xE5;
const HTU21D_TEMP_NH: u8 = 0xF3;
const HTU21D_HUMID_NH: u8 = 0xF5;
const CMD_WRITE: u8 = 0xE6;
const CMD_READ: u8 = 0xE7;
const CMD_RESET: u8 = 0xFE;
const CMD_SERIAL: [u8; 4] = [0xFA, 0x0F, 0xFC, 0xC9];

const HTU21D_RESOLUTION_MASK: i64 = 0x7E;
const HTU21D_RESOLUTIONS: [(&str, i64); 4] = [("TEMP14_HUM12", 0b00000000), ("TEMP13_HUM10", 0b10000000), ("TEMP12_HUM08", 0b00000001), ("TEMP11_HUM11", 0b10000001)];

/// ID_MAP
fn id_map(id: i64) -> &'static str {
    match id {
        0x0D => "SI7013",
        0x14 => "SI7020",
        0x15 => "SI7021",
        0x31 | 0x01 => "SHT21",
        0x32 => "HTU21D",
        _ => "Unknown",
    }
}

// Device with conversion time for tmp/resolution bit
// The format is:
//  <CHIPNAME>:{id:<ID>, ..<RESOlUTION>:[<temp time>,<humidity time>].. }
const SI70XX_TIMES: [[f64; 2]; 4] = [[0.11, 0.12], [0.7, 0.5], [0.4, 0.4], [0.3, 0.7]];
const HTU21D_DEVICES: [(&str, [[f64; 2]; 4]); 5] = [
    ("SI7013", SI70XX_TIMES),
    ("SI7020", SI70XX_TIMES),
    ("SI7021", SI70XX_TIMES),
    ("SHT21", [[0.85, 0.29], [0.43, 0.9], [0.22, 0.4], [0.11, 0.15]]),
    ("HTU21D", [[0.50, 0.16], [0.25, 0.5], [0.13, 0.3], [0.12, 0.8]]),
];
// temperature coefficient for RH compensation at range 0C..80C,
//  for HTU21D & SHT21 only (python computes but never applies it)
// crc8 polynomial for 16bit value, CRC8 -> x^8 + x^5 + x^4 + 1
const HTU21D_CRC8_POLYNOMINAL: i64 = 0x13100;

fn response_bytes(params: &Params, n: usize) -> KResult<Vec<u8>> {
    let r = match params.get("response") {
        Some(Val::Bytes(b)) => b.clone(),
        _ => vec![],
    };
    if r.len() < n {
        return Err(KError::Py("IndexError".into(), "bytearray index out of range".into()));
    }
    Ok(r)
}

pub struct HTU21D {
    pub printer: Rc<Printer>,
    pub i2c: Rc<McuI2c>,
    pub hold_master_mode: bool,
    pub resolution: String,
    pub report_time: i64,
    pub device_id: String,
    pub name: String,
    pub temp: Cell<f64>,
    pub min_temp: Cell<f64>,
    pub max_temp: Cell<f64>,
    /// python's humidity: a float, or the int 0 / 100 when clamped
    pub humidity: Cell<f64>,
    humidity_int: Cell<bool>,
    pub sample_timer: RefCell<Option<Timer>>,
    pub callback: RefCell<Option<TempCallback>>,
}

impl HTU21D {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let i2c = mcu_i2c_from_config(config, Some(HTU21D_I2C_ADDR), 100000)?;
        let mcu = i2c.get_mcu();
        let hold_master_mode = config.getboolean("htu21d_hold_master", false)?;
        let resolution = config.get("htu21d_resolution", "TEMP12_HUM08")?;
        let report_time = config.getint("htu21d_report_time", 30, Lim::new().min(5))?;
        if !HTU21D_RESOLUTIONS.iter().any(|(r, _)| *r == resolution) {
            let valid: Vec<&str> = HTU21D_RESOLUTIONS.iter().map(|(r, _)| *r).collect();
            return Err(KError::config(format!("Invalid HTU21D Resolution. Valid are {}", valid.join("|"))));
        }
        let device_id = config.get("sensor_type", Dflt::Required)?;
        let s = Rc::new(HTU21D {
            printer: printer.clone(),
            i2c,
            hold_master_mode,
            resolution,
            report_time,
            device_id,
            name: name.clone(),
            temp: Cell::new(0.0),
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            humidity: Cell::new(0.0),
            humidity_int: Cell::new(false),
            sample_timer: RefCell::new(None),
            callback: RefCell::new(None),
        });
        let w: Weak<HTU21D> = Rc::downgrade(&s);
        *s.sample_timer.borrow_mut() = Some(printer.get_reactor().register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.sample_htu21d(eventtime),
                None => NEVER,
            }),
            NEVER,
        ));
        printer.add_object(&format!("htu21d {}", name), s.clone())?;
        let ss = s.clone();
        crate::events::KlippyConnect::on(&printer, move || ss.handle_connect());
        let ss = s.clone();
        printer.register_event_handler(mcu.get_non_critical_reconnect_event_name(), Rc::new(move |_| ss.handle_connect()));
        Ok(s)
    }

    pub fn handle_connect(&self) -> KResult<()> {
        self.init_htu21d()?;
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

    /// HTU21D_DEVICES[deviceId][resolution]
    fn times(&self) -> KResult<[f64; 2]> {
        let dev = HTU21D_DEVICES.iter().find(|(d, _)| *d == self.device_id).ok_or_else(|| KError::Py("KeyError".into(), crate::pyfmt::py_str_repr(&self.device_id)))?;
        let idx = HTU21D_RESOLUTIONS.iter().position(|(r, _)| *r == self.resolution).unwrap_or(0);
        Ok(dev.1[idx])
    }

    pub fn init_htu21d(&self) -> KResult<()> {
        // Device Soft Reset
        self.i2c.i2c_write(&[CMD_RESET], 0, 0)?;
        // Wait 15ms after reset
        self.pause(0.15)?;

        // Read ChipId
        let params = self.i2c.i2c_read(&[CMD_SERIAL[2], CMD_SERIAL[3]], 3, true)?;
        let response = response_bytes(&params, 3)?;
        let mut rdev_id = ((response[0] as i64) << 8) | response[1] as i64;
        let checksum = response[2] as i64;
        if Self::chek_crc8(rdev_id) != checksum {
            logging::warning("htu21d: Reading deviceId !Checksum error!");
        }
        rdev_id >>= 8;
        let guess_dev = id_map(rdev_id);
        if guess_dev == self.device_id {
            logging::info(&format!("htu21d: Found Device Type {}", guess_dev));
        } else {
            logging::warning(&format!("htu21d: Unknown Device ID {:#x} ", rdev_id));
        }

        if self.device_id != guess_dev {
            logging::warning(&format!("htu21d: Found device {}. Forcing to type {} as config.", guess_dev, self.device_id));
        }

        // Set Resolution
        let params = self.i2c.i2c_read(&[CMD_READ], 1, true)?;
        let response = response_bytes(&params, 1)?;
        let mut register_data = response[0] as i64 & HTU21D_RESOLUTION_MASK;
        register_data |= HTU21D_RESOLUTIONS.iter().find(|(r, _)| *r == self.resolution).map(|(_, v)| *v).unwrap_or(0);
        // (python passes the register value as i2c_write's minclock)
        self.i2c.i2c_write(&[CMD_WRITE], register_data as u64, 0)?;
        logging::info(&format!("htu21d: Setting resolution to {} ", self.resolution));
        Ok(())
    }

    fn read_sensor(&self) -> KResult<()> {
        // Read Temeprature
        if self.hold_master_mode {
            self.i2c.i2c_write(&[HTU21D_TEMP], 0, 0)?;
        } else {
            self.i2c.i2c_write(&[HTU21D_TEMP_NH], 0, 0)?;
        }

        // Wait
        self.pause(self.times()?[0])?;

        let params = self.i2c.i2c_read(&[], 3, true)?;

        let response = response_bytes(&params, 3)?;
        let rtemp = ((response[0] as i64) << 8) | response[1] as i64;
        if Self::chek_crc8(rtemp) != response[2] as i64 {
            logging::warning("htu21d: Checksum error on Temperature reading!");
        } else {
            self.temp.set(0.002681 * rtemp as f64 - 46.85);
            logging::debug(&format!("htu21d: Temperature {:.2} ", self.temp.get()));
        }

        // Read Humidity
        if self.hold_master_mode {
            self.i2c.i2c_write(&[HTU21D_HUMID], 0, 0)?;
        } else {
            self.i2c.i2c_write(&[HTU21D_HUMID_NH], 0, 0)?;
        }

        // Wait
        self.pause(self.times()?[1])?;

        let params = self.i2c.i2c_read(&[], 3, true)?;

        let response = response_bytes(&params, 3)?;
        let mut rhumid = ((response[0] as i64) << 8) | response[1] as i64;
        if Self::chek_crc8(rhumid) != response[2] as i64 {
            logging::warning("htu21d: Checksum error on Humidity reading!");
        } else {
            // clear status bits,
            // humidity always returns xxxxxx10 in the LSB field
            rhumid ^= 0x02;
            let mut humidity = 0.001907 * rhumid as f64 - 6.0;
            let mut is_int = false;
            if humidity < 0.0 {
                // due to RH accuracy, measured value might be
                // slightly less than 0 or more 100
                humidity = 0.0;
                is_int = true;
            } else if humidity > 100.0 {
                humidity = 100.0;
                is_int = true;
            }
            self.humidity.set(humidity);
            self.humidity_int.set(is_int);
            // Only for HTU21D & SHT21.
            // Calculates temperature compensated Humidity, %RH
            let temp = self.temp.get();
            if (self.device_id == "SHT21" || self.device_id == "HTU21D") && temp > 0.0 && temp < 80.0 {
                logging::debug("htu21d: Do temp compensation..");
            }
            logging::debug(&format!("htu21d: Humidity {:.2} ", self.humidity.get()));
        }
        Ok(())
    }

    pub fn sample_htu21d(&self, _eventtime: f64) -> f64 {
        if let Err(e) = self.read_sensor() {
            logging::exception_err("htu21d: Error reading data", &e);
            self.temp.set(0.0);
            self.humidity.set(0.0);
            self.humidity_int.set(false);
            return NEVER;
        }

        let temp = self.temp.get();
        if (temp < self.min_temp.get() || temp > self.max_temp.get()) && !get_danger_options().temp_ignore_limits {
            self.printer.invoke_shutdown(
                &format!("HTU21D temperature {:.1} outside range of {:.1}:{:.1}", temp, self.min_temp.get(), self.max_temp.get()),
                None,
            );
        }

        let measured_time = self.printer.get_reactor().monotonic();
        let print_time = self.i2c.get_mcu().estimated_print_time(measured_time);
        let cb = self.callback.borrow().clone();
        let res = match cb {
            Some(cb) => cb(print_time, temp),
            None => Err(KError::Py("AttributeError".into(), "'HTU21D' object has no attribute '_callback'".into())),
        };
        if let Err(e) = res {
            self.printer.unhandled_exception(&e);
            return NEVER;
        }
        measured_time + self.report_time as f64
    }

    fn chek_crc8(data: i64) -> i64 {
        let mut data = data;
        for _bit in 0..16 {
            if data & 0x8000 != 0 {
                data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
            } else {
                data <<= 1;
            }
        }
        data >> 8
    }
}

impl TempSensor for HTU21D {
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
        Ok(self.report_time as f64)
    }
}

impl PrinterObject for HTU21D {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let humidity = match self.humidity_int.get() {
            true => PyVal::Int(self.humidity.get() as i64),
            false => PyVal::Float(self.humidity.get()),
        };
        let d = crate::pydict! {
            "temperature" => PyVal::Float(py_round(self.temp.get(), 2)),
            "humidity" => humidity,
        };
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheater = lookup_heaters(config)?;
    for (stype, _) in HTU21D_DEVICES {
        pheater.add_sensor_factory(stype, Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(HTU21D::new(config)?) }));
    }
    Ok(none_object())
}
