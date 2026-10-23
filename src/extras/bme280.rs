// Support for i2c based temperature sensors
// (port of klippy/extras/bme280.py)
//
// Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{load_heaters, TempCallback, TempSensor};
use crate::logging;
use crate::mcu::Mcu;
use crate::msgproto::Val;
use crate::printer::{none_object, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_round};
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const REPORT_TIME: f64 = 0.8;
const BME280_CHIP_ADDR: i64 = 0x76;

const BME280_REGS: &[(&str, u8)] = &[
    ("RESET", 0xE0),
    ("CTRL_HUM", 0xF2),
    ("STATUS", 0xF3),
    ("CTRL_MEAS", 0xF4),
    ("CONFIG", 0xF5),
    ("PRESSURE_MSB", 0xF7),
    ("PRESSURE_LSB", 0xF8),
    ("PRESSURE_XLSB", 0xF9),
    ("TEMP_MSB", 0xFA),
    ("TEMP_LSB", 0xFB),
    ("TEMP_XLSB", 0xFC),
    ("HUM_MSB", 0xFD),
    ("HUM_LSB", 0xFE),
    ("CAL_1", 0x88),
    ("CAL_2", 0xE1),
];

const BMP388_REGS: &[(&str, u8)] = &[
    ("CMD", 0x7E),
    ("STATUS", 0x03),
    ("PWR_CTRL", 0x1B),
    ("OSR", 0x1C),
    ("ORD", 0x1D),
    ("INT_CTRL", 0x19),
    ("CAL_1", 0x31),
    ("TEMP_MSB", 0x09),
    ("TEMP_LSB", 0x08),
    ("TEMP_XLSB", 0x07),
    ("PRESS_MSB", 0x06),
    ("PRESS_LSB", 0x05),
    ("PRESS_XLSB", 0x04),
];
const BMP388_REG_VAL_PRESS_EN: i64 = 0x01;
const BMP388_REG_VAL_TEMP_EN: i64 = 0x02;
const BMP388_REG_VAL_PRESS_OS_NO: i64 = 0b000;
const BMP388_REG_VAL_TEMP_OS_NO: i64 = 0b000000;
const BMP388_REG_VAL_ODR_50_HZ: i64 = 0x02;
const BMP388_REG_VAL_DRDY_EN: i64 = 0b100000;
const BMP388_REG_VAL_NORMAL_MODE: i64 = 0x30;

const BME680_REGS: &[(&str, u8)] = &[
    ("RESET", 0xE0),
    ("CTRL_HUM", 0x72),
    ("CTRL_GAS_1", 0x71),
    ("CTRL_GAS_0", 0x70),
    ("GAS_WAIT_0", 0x64),
    ("RES_HEAT_0", 0x5A),
    ("IDAC_HEAT_0", 0x50),
    ("STATUS", 0x73),
    ("EAS_STATUS_0", 0x1D),
    ("CTRL_MEAS", 0x74),
    ("CONFIG", 0x75),
    ("GAS_R_LSB", 0x2B),
    ("GAS_R_MSB", 0x2A),
    ("PRESSURE_MSB", 0x1F),
    ("PRESSURE_LSB", 0x20),
    ("PRESSURE_XLSB", 0x21),
    ("TEMP_MSB", 0x22),
    ("TEMP_LSB", 0x23),
    ("TEMP_XLSB", 0x24),
    ("HUM_MSB", 0x25),
    ("HUM_LSB", 0x26),
    ("CAL_1", 0x88),
    ("CAL_2", 0xE1),
    ("RES_HEAT_VAL", 0x00),
    ("RES_HEAT_RANGE", 0x02),
    ("RANGE_SWITCHING_ERROR", 0x04),
];

const BME680_GAS_CONSTANTS: [(f64, f64); 16] = [
    (1.0, 8000000.0),
    (1.0, 4000000.0),
    (1.0, 2000000.0),
    (1.0, 1000000.0),
    (1.0, 499500.4995),
    (0.99, 248262.1648),
    (1.0, 125000.0),
    (0.992, 63004.03226),
    (1.0, 31281.28128),
    (1.0, 15625.0),
    (0.998, 7812.5),
    (0.995, 3906.25),
    (1.0, 1953.125),
    (0.99, 976.5625),
    (1.0, 488.28125),
    (1.0, 244.140625),
];

const BMP180_REGS: &[(&str, u8)] = &[("RESET", 0xE0), ("CAL_1", 0xAA), ("CTRL_MEAS", 0xF4), ("REG_MSB", 0xF6), ("REG_LSB", 0xF7), ("CRV_TEMP", 0x2E), ("CRV_PRES", 0x34)];

#[allow(dead_code)] // unused in python too
const STATUS_MEASURING: i64 = 1 << 3;
const STATUS_IM_UPDATE: i64 = 1;
const MODE: i64 = 1;
const MODE_PERIODIC: i64 = 3;
const RUN_GAS: i64 = 1 << 4;
const NB_CONV_0: i64 = 0;
const EAS_NEW_DATA: i64 = 1 << 7;
#[allow(dead_code)] // unused in python too
const GAS_IN_PROGRESS: i64 = 1 << 6;
const MEASURE_IN_PROGRESS: i64 = 1 << 5;
const RESET_CHIP_VALUE: i64 = 0xB6;

/// BME_CHIPS
fn bme_chip(id: i64) -> Option<&'static str> {
    match id {
        0x58 => Some("BMP280"),
        0x60 => Some("BME280"),
        0x61 => Some("BME680"),
        0x55 => Some("BMP180"),
        0x50 => Some("BMP388"),
        _ => None,
    }
}
const BME_CHIP_ID_REG: u8 = 0xD0;
const BMP3_CHIP_ID_REG: u8 = 0x00;

fn get_twos_complement(val: i64, bit_size: u32) -> i64 {
    let mut val = val;
    if val & (1 << (bit_size - 1)) != 0 {
        val -= 1 << bit_size;
    }
    val
}

fn get_unsigned_short(bits: &[u8]) -> i64 {
    (bits[1] as i64) << 8 | bits[0] as i64
}

fn get_signed_short(bits: &[u8]) -> i64 {
    get_twos_complement(get_unsigned_short(bits), 16)
}

fn get_signed_byte(bits: u8) -> i64 {
    get_twos_complement(bits as i64, 8)
}

fn get_unsigned_short_msb(bits: &[u8]) -> i64 {
    (bits[0] as i64) << 8 | bits[1] as i64
}

fn get_signed_short_msb(bits: &[u8]) -> i64 {
    get_twos_complement(get_unsigned_short_msb(bits), 16)
}

/// a register read shorter than python indexes into
fn short_read() -> KError {
    KError::Py("IndexError".into(), "bytearray index out of range".into())
}

/// the calibration values (python ints, or floats for the BMP388)
type Dig = IndexMap<&'static str, f64>;

fn dig_of(pairs: &[(&'static str, f64)]) -> Dig {
    pairs.iter().copied().collect()
}

fn read_calibration_data_bmp280(c1: &[u8]) -> Dig {
    let s = |a: usize| get_signed_short(&c1[a..a + 2]) as f64;
    let u = |a: usize| get_unsigned_short(&c1[a..a + 2]) as f64;
    dig_of(&[
        ("T1", u(0)),
        ("T2", s(2)),
        ("T3", s(4)),
        ("P1", u(6)),
        ("P2", s(8)),
        ("P3", s(10)),
        ("P4", s(12)),
        ("P5", s(14)),
        ("P6", s(16)),
        ("P7", s(18)),
        ("P8", s(20)),
        ("P9", s(22)),
    ])
}

fn read_calibration_data_bmp388(c1: &[u8]) -> Dig {
    let s = |a: usize| get_signed_short(&c1[a..a + 2]) as f64;
    let u = |a: usize| get_unsigned_short(&c1[a..a + 2]) as f64;
    let b = |a: usize| get_signed_byte(c1[a]) as f64;
    dig_of(&[
        ("T1", u(0) / 0.00390625),
        ("T2", u(2) / 1073741824.0),
        ("T3", b(4) / 281474976710656.0),
        ("P1", (s(5) - 16384.0) / 1048576.0),
        ("P2", (s(7) - 16384.0) / 536870912.0),
        ("P3", b(9) / 4294967296.0),
        ("P4", b(10) / 137438953472.0),
        ("P5", u(11) / 0.125),
        ("P6", u(13) / 64.0),
        ("P7", b(15) / 256.0),
        ("P8", b(16) / 32768.0),
        ("P9", s(17) / 281474976710656.0),
        ("P10", b(19) / 281474976710656.0),
        ("P11", b(20) / 36893488147419103232.0),
    ])
}

fn read_calibration_data_bme280(c1: &[u8], c2: &[u8]) -> Dig {
    let mut dig = read_calibration_data_bmp280(c1);
    dig.insert("H1", (c1[25] & 0xFF) as f64);
    dig.insert("H2", get_signed_short(&c2[0..2]) as f64);
    dig.insert("H3", (c2[2] & 0xFF) as f64);
    dig.insert("H4", get_twos_complement(((c2[3] as i64) << 4) | (c2[4] & 0x0F) as i64, 12) as f64);
    dig.insert("H5", get_twos_complement(((c2[5] as i64) << 4) | ((c2[4] & 0xF0) >> 4) as i64, 12) as f64);
    dig.insert("H6", get_twos_complement(c2[6] as i64, 8) as f64);
    dig
}

fn read_calibration_data_bme680(c1: &[u8], c2: &[u8]) -> Dig {
    let s1 = |a: usize| get_signed_short(&c1[a..a + 2]) as f64;
    let b1 = |a: usize| get_signed_byte(c1[a]) as f64;
    let b2 = |a: usize| get_signed_byte(c2[a]) as f64;
    dig_of(&[
        ("T1", get_unsigned_short(&c2[8..10]) as f64),
        ("T2", s1(2)),
        ("T3", b1(4)),
        ("P1", get_unsigned_short(&c1[6..8]) as f64),
        ("P2", s1(8)),
        ("P3", c1[10] as f64),
        ("P4", s1(12)),
        ("P5", s1(14)),
        ("P6", b1(17)),
        ("P7", b1(16)),
        ("P8", s1(20)),
        ("P9", s1(22)),
        ("P10", c1[24] as f64),
        ("H1", get_twos_complement(((c2[2] as i64) << 4) | (c2[1] & 0x0F) as i64, 12) as f64),
        ("H2", get_twos_complement(((c2[0] as i64) << 4) | ((c2[1] & 0xF0) >> 4) as i64, 12) as f64),
        ("H3", b2(3)),
        ("H4", b2(4)),
        ("H5", b2(5)),
        ("H6", c2[6] as f64),
        ("H7", b2(7)),
        ("G1", b2(12)),
        ("G2", get_signed_short(&c2[10..12]) as f64),
        ("G3", b2(13)),
    ])
}

fn read_calibration_data_bmp180(c1: &[u8]) -> Dig {
    let s = |a: usize| get_signed_short_msb(&c1[a..a + 2]) as f64;
    let u = |a: usize| get_unsigned_short_msb(&c1[a..a + 2]) as f64;
    dig_of(&[
        ("AC1", s(0)),
        ("AC2", s(2)),
        ("AC3", s(4)),
        ("AC4", u(6)),
        ("AC5", u(8)),
        ("AC6", u(10)),
        ("B1", s(12)),
        ("B2", s(14)),
        ("MB", s(16)),
        ("MC", s(18)),
        ("MD", s(20)),
    ])
}

/// the sample timer callback of the found chip
#[derive(Clone, Copy)]
enum Sampler {
    Bme280,
    Bme680,
    Bmp180,
    Bmp388,
}

pub struct BME280 {
    me: Weak<BME280>,
    printer: Rc<Printer>,
    name: String,
    i2c: Rc<McuI2c>,
    mcu: Rc<Mcu>,
    iir_filter: i64,
    os_temp: i64,
    os_hum: i64,
    os_pres: i64,
    gas_heat_temp: i64,
    gas_heat_duration: i64,
    temp: Cell<f64>,
    pressure: Cell<f64>,
    humidity: Cell<f64>,
    gas: Cell<f64>,
    t_fine: Cell<f64>,
    min_temp: Cell<f64>,
    max_temp: Cell<f64>,
    max_sample_time: Cell<Option<f64>>,
    dig: RefCell<Dig>,
    sample_timer: RefCell<Option<Timer>>,
    chip_type: RefCell<&'static str>,
    chip_registers: Cell<&'static [(&'static str, u8)]>,
    last_gas_time: Cell<f64>,
    callback: RefCell<Option<TempCallback>>,
}

impl BME280 {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let i2c = mcu_i2c_from_config(config, Some(BME280_CHIP_ADDR), 100000)?;
        let mcu = i2c.get_mcu();
        let iir_filter = config.getint("bme280_iir_filter", 1, crate::configfile::Lim::new())?;
        let os_temp = config.getint("bme280_oversample_temp", 2, crate::configfile::Lim::new())?;
        let os_hum = config.getint("bme280_oversample_hum", 2, crate::configfile::Lim::new())?;
        let os_pres = config.getint("bme280_oversample_pressure", 2, crate::configfile::Lim::new())?;
        let gas_heat_temp = config.getint("bme280_gas_target_temp", 320, crate::configfile::Lim::new())?;
        let gas_heat_duration = config.getint("bme280_gas_heat_duration", 150, crate::configfile::Lim::new())?;
        // "%d" % pow(2, n - 1): a float for n < 1
        let pow2 = |n: i64| 2f64.powi(n as i32).trunc() as i64;
        logging::info(&format!(
            "BMxx80 {}: Oversampling: Temp {}x Humid {}x Pressure {}x",
            name,
            pow2(os_temp - 1),
            pow2(os_hum - 1),
            pow2(os_pres - 1)
        ));
        logging::info(&format!("BMxx80 {}: IIR: {}x", name, pow2(iir_filter) - 1));
        let s = Rc::new_cyclic(|me| BME280 {
            me: me.clone(),
            printer: printer.clone(),
            name: name.clone(),
            i2c,
            mcu: mcu.clone(),
            iir_filter: iir_filter & 0x07,
            os_temp,
            os_hum,
            os_pres,
            gas_heat_temp,
            gas_heat_duration,
            temp: Cell::new(0.0),
            pressure: Cell::new(0.0),
            humidity: Cell::new(0.0),
            gas: Cell::new(0.0),
            t_fine: Cell::new(0.0),
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            max_sample_time: Cell::new(None),
            dig: RefCell::new(IndexMap::new()),
            sample_timer: RefCell::new(None),
            chip_type: RefCell::new("BMP280"),
            chip_registers: Cell::new(BME280_REGS),
            last_gas_time: Cell::new(0.0),
            callback: RefCell::new(None),
        });
        printer.add_object(&format!("bme280 {}", name), s.clone())?;
        if printer.start_arg("debugoutput").is_some() {
            return Ok(s);
        }
        let ss = s.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| ss.handle_connect()));
        let ss = s.clone();
        printer.register_event_handler(mcu.get_non_critical_reconnect_event_name(), Rc::new(move |_| ss.handle_connect()));
        Ok(s)
    }

    fn handle_connect(&self) -> KResult<()> {
        self.init_bmxx80()?;
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

    fn d(&self, k: &str) -> f64 {
        self.dig.borrow().get(k).copied().unwrap_or(0.0)
    }

    fn register_sampler(&self, sampler: Sampler) {
        let w = self.me.clone();
        let t = self.printer.get_reactor().register_timer(
            Box::new(move |eventtime| {
                let Some(s) = w.upgrade() else { return NEVER };
                let res = match sampler {
                    Sampler::Bme280 => Ok(s.sample_bme280(eventtime)),
                    Sampler::Bme680 => s.sample_bme680(eventtime),
                    Sampler::Bmp180 => Ok(s.sample_bmp180(eventtime)),
                    Sampler::Bmp388 => s.sample_bmp388(eventtime),
                };
                res.unwrap_or_else(|e| {
                    s.printer.unhandled_exception(&e);
                    NEVER
                })
            }),
            NEVER,
        );
        *self.sample_timer.borrow_mut() = Some(t);
    }

    fn init_bmxx80(&self) -> KResult<()> {
        let mut chip_id = self.read_id()?;
        if chip_id == 0 {
            chip_id = self.read_bmp3_id()?;
        }
        match bme_chip(chip_id) {
            None => logging::info(&format!("bme280 {}: Unknown Chip ID received {:#x}", self.name, chip_id)),
            Some(chip) => {
                *self.chip_type.borrow_mut() = chip;
                logging::info(&format!("bme280 {}: Found Chip {} at {:#x}", self.name, chip, self.i2c.get_i2c_address()));
            }
        }
        let chip_type = *self.chip_type.borrow();

        // Reset chip
        self.write_register("RESET", &[RESET_CHIP_VALUE])?;
        self.pause(0.5)?;

        // Make sure non-volatile memory has been copied to registers
        if chip_type != "BMP180" {
            // BMP180 has no status register available
            let mut status = *self.read_register("STATUS", 1)?.first().ok_or_else(short_read)? as i64;
            while status & STATUS_IM_UPDATE != 0 {
                self.pause(0.01)?;
                status = *self.read_register("STATUS", 1)?.first().ok_or_else(short_read)? as i64;
            }
        }

        let (ot, op, oh) = (self.os_temp as f64, self.os_pres as f64, self.os_hum as f64);
        match chip_type {
            "BME680" => {
                self.max_sample_time.set(Some((1.25 + (2.3 * ot) + ((2.3 * op) + 0.575) + ((2.3 * oh) + 0.575)) / 1000.0));
                self.register_sampler(Sampler::Bme680);
                self.chip_registers.set(BME680_REGS);
            }
            "BMP180" => {
                self.register_sampler(Sampler::Bmp180);
                self.chip_registers.set(BMP180_REGS);
            }
            "BMP388" => {
                self.chip_registers.set(BMP388_REGS);
                self.write_register("PWR_CTRL", &[BMP388_REG_VAL_PRESS_EN | BMP388_REG_VAL_TEMP_EN | BMP388_REG_VAL_NORMAL_MODE])?;
                self.write_register("OSR", &[BMP388_REG_VAL_PRESS_OS_NO | BMP388_REG_VAL_TEMP_OS_NO])?;
                self.write_register("ORD", &[BMP388_REG_VAL_ODR_50_HZ])?;
                self.write_register("INT_CTRL", &[BMP388_REG_VAL_DRDY_EN])?;

                self.register_sampler(Sampler::Bmp388);
            }
            "BME280" => {
                self.max_sample_time.set(Some((1.25 + (2.3 * ot) + ((2.3 * op) + 0.575) + ((2.3 * oh) + 0.575)) / 1000.0));
                self.register_sampler(Sampler::Bme280);
                self.chip_registers.set(BME280_REGS);
            }
            _ => {
                self.max_sample_time.set(Some((1.25 + (2.3 * ot) + ((2.3 * op) + 0.575)) / 1000.0));
                self.register_sampler(Sampler::Bme280);
                self.chip_registers.set(BME280_REGS);
            }
        }

        // Read out and calculate the trimming parameters
        let need = |v: Vec<u8>, n: usize| if v.len() < n { Err(short_read()) } else { Ok(v) };
        let (cal_1, cal_2) = match chip_type {
            "BMP180" => (need(self.read_register("CAL_1", 22)?, 22)?, vec![]),
            "BMP388" => (need(self.read_register("CAL_1", 21)?, 21)?, vec![]),
            _ => (need(self.read_register("CAL_1", 26)?, 24)?, self.read_register("CAL_2", 16)?),
        };
        let dig = match chip_type {
            "BME280" => Some(read_calibration_data_bme280(&need(cal_1, 26)?, &need(cal_2, 7)?)),
            "BMP280" => Some(read_calibration_data_bmp280(&cal_1)),
            "BME680" => Some(read_calibration_data_bme680(&need(cal_1, 25)?, &need(cal_2, 14)?)),
            "BMP180" => Some(read_calibration_data_bmp180(&cal_1)),
            "BMP388" => Some(read_calibration_data_bmp388(&cal_1)),
            _ => None,
        };
        if let Some(dig) = dig {
            *self.dig.borrow_mut() = dig;
        }

        if chip_type == "BME280" || chip_type == "BMP280" {
            let max_standby_time = REPORT_TIME - self.max_sample_time.get().unwrap_or(0.0);
            // 0.5 ms
            let mut t_sb = 0;
            if chip_type == "BME280" {
                for (limit, v) in [(1.0, 5), (0.5, 4), (0.25, 3), (0.125, 2), (0.0625, 1), (0.020, 7), (0.010, 6)] {
                    if max_standby_time > limit {
                        t_sb = v;
                        break;
                    }
                }
            } else {
                for (limit, v) in [(4.0, 7), (2.0, 6), (1.0, 5), (0.5, 4), (0.25, 3), (0.125, 2), (0.0625, 1)] {
                    if max_standby_time > limit {
                        t_sb = v;
                        break;
                    }
                }
            }

            let cfg = t_sb << 5 | self.iir_filter << 2;
            self.write_register("CONFIG", &[cfg])?;
            if chip_type == "BME280" {
                self.write_register("CTRL_HUM", &[self.os_hum])?;
            }
            // Enter normal (periodic) mode
            let meas = self.os_temp << 5 | self.os_pres << 2 | MODE_PERIODIC;
            self.write_register("CTRL_MEAS", &[meas])?;
        }

        if chip_type == "BME680" {
            self.write_register("CONFIG", &[self.iir_filter << 2])?;
            // Should be set once and reused on every mode register write
            self.write_register("CTRL_HUM", &[self.os_hum & 0x07])?;
            let gas_wait_0 = Self::calc_gas_heater_duration(self.gas_heat_duration);
            self.write_register("GAS_WAIT_0", &[gas_wait_0])?;
            let res_heat_0 = self.calc_gas_heater_resistance(self.gas_heat_temp as f64)?;
            self.write_register("RES_HEAT_0", &[res_heat_0])?;
            // Set initial heater current to reach Gas heater target on start
            self.write_register("IDAC_HEAT_0", &[96])?;
        }
        Ok(())
    }

    fn report(&self, check_ignore: bool, what: &str) -> KResult<f64> {
        let temp = self.temp.get();
        let out_of_range = temp < self.min_temp.get() || temp > self.max_temp.get();
        if out_of_range && !(check_ignore && get_danger_options().temp_ignore_limits) {
            self.printer.invoke_shutdown(
                &format!("{} temperature {:.1} outside range of {:.1}:{:.1}", what, temp, self.min_temp.get(), self.max_temp.get()),
                None,
            );
        }
        let measured_time = self.printer.get_reactor().monotonic();
        let cb = self.callback.borrow().clone();
        match cb {
            Some(cb) => cb(self.mcu.estimated_print_time(measured_time), temp)?,
            None => return Err(KError::Py("AttributeError".into(), "'BME280' object has no attribute '_callback'".into())),
        }
        Ok(measured_time + REPORT_TIME)
    }

    fn sample_bme280(&self, _eventtime: f64) -> f64 {
        // In normal mode data shadowing is performed
        // So reading can be done while measurements are in process
        let chip_type = *self.chip_type.borrow();
        let data = match chip_type {
            "BME280" => self.read_register("PRESSURE_MSB", 8),
            "BMP280" => self.read_register("PRESSURE_MSB", 6),
            _ => return NEVER,
        };
        let need = if chip_type == "BME280" { 8 } else { 6 };
        let data = match data.and_then(|d| if d.len() < need { Err(short_read()) } else { Ok(d) }) {
            Ok(d) => d,
            Err(e) => {
                logging::exception_err(&format!("BME280 {}: Error reading data", self.name), &e);
                self.temp.set(0.0);
                self.pressure.set(0.0);
                self.humidity.set(0.0);
                return NEVER;
            }
        };

        let temp_raw = ((data[3] as i64) << 12) | ((data[4] as i64) << 4) | (data[5] >> 4) as i64;
        self.temp.set(self.compensate_temp(temp_raw as f64));
        let pressure_raw = ((data[0] as i64) << 12) | ((data[1] as i64) << 4) | (data[2] >> 4) as i64;
        self.pressure.set(self.compensate_pressure_bme280(pressure_raw as f64) / 100.0);
        if chip_type == "BME280" {
            let humid_raw = ((data[6] as i64) << 8) | data[7] as i64;
            self.humidity.set(self.compensate_humidity_bme280(humid_raw as f64));
        }
        self.report(true, "BME280").unwrap_or_else(|e| {
            self.printer.unhandled_exception(&e);
            NEVER
        })
    }

    fn sample_bmp388(&self, _eventtime: f64) -> KResult<f64> {
        let status = *self.read_register("STATUS", 1)?.first().ok_or_else(short_read)? as i64;
        if status & 0b100000 != 0 {
            self.temp.set(self.sample_bmp388_temp()?);
            let temp = self.temp.get();
            if temp < self.min_temp.get() || temp > self.max_temp.get() {
                self.printer.invoke_shutdown(
                    &format!("BME280 temperature {:.1} outside range of {:.1}:{:.1}", temp, self.min_temp.get(), self.max_temp.get()),
                    None,
                );
            }
        }

        if status & 0b010000 != 0 {
            self.pressure.set(self.sample_bmp388_press()? / 100.0);
        }

        let measured_time = self.printer.get_reactor().monotonic();
        let cb = self.callback.borrow().clone();
        match cb {
            Some(cb) => cb(self.mcu.estimated_print_time(measured_time), self.temp.get())?,
            None => return Err(KError::Py("AttributeError".into(), "'BME280' object has no attribute '_callback'".into())),
        }
        Ok(measured_time + REPORT_TIME)
    }

    fn read3(&self, x: &str, l: &str, m: &str) -> KResult<f64> {
        let byte = |r: &str| -> KResult<i64> { Ok(*self.read_register(r, 1)?.first().ok_or_else(short_read)? as i64) };
        let xlsb = byte(x)?;
        let lsb = byte(l)?;
        let msb = byte(m)?;
        Ok(((msb << 16) + (lsb << 8) + xlsb) as f64)
    }

    fn sample_bmp388_temp(&self) -> KResult<f64> {
        let adc_t = self.read3("TEMP_XLSB", "TEMP_LSB", "TEMP_MSB")?;

        let partial_data1 = adc_t - self.d("T1");
        let partial_data2 = self.d("T2") * partial_data1;

        let mut t_fine = partial_data2;
        t_fine += (partial_data1 * partial_data1) * self.d("T3");

        if t_fine < -40.0 {
            t_fine = -40.0;
        }

        if t_fine > 85.0 {
            t_fine = 85.0;
        }

        self.t_fine.set(t_fine);
        Ok(t_fine)
    }

    fn sample_bmp388_press(&self) -> KResult<f64> {
        let adc_p = self.read3("PRESS_XLSB", "PRESS_LSB", "PRESS_MSB")?;
        let t_fine = self.t_fine.get();
        let d = |k: &str| self.d(k);

        let partial_data1 = d("P6") * t_fine;
        let partial_data2 = d("P7") * (t_fine * t_fine);
        let mut partial_data3 = d("P8");
        partial_data3 *= t_fine * t_fine * t_fine;
        let mut partial_out1 = d("P5");
        partial_out1 += partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = d("P2") * t_fine;
        let partial_data2 = d("P3") * (t_fine * t_fine);
        let mut partial_data3 = d("P4");
        partial_data3 *= t_fine * t_fine * t_fine;
        let partial_out2 = adc_p * (d("P1") + partial_data1 + partial_data2 + partial_data3);

        let partial_data1 = adc_p * adc_p;
        let partial_data2 = d("P9") + (d("P10") * t_fine);
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + adc_p * adc_p * adc_p * d("P11");

        let mut comp_press = partial_out1 + partial_out2 + partial_data4;

        if comp_press < 30000.0 {
            comp_press = 30000.0;
        }

        if comp_press > 125000.0 {
            comp_press = 125000.0;
        }

        Ok(comp_press)
    }

    fn sample_bme680(&self, _eventtime: f64) -> KResult<f64> {
        let reactor = self.printer.get_reactor();
        let mut run_gas = false;
        // Check VOC once a while
        if reactor.monotonic() - self.last_gas_time.get() > 3.0 {
            let gas_config = RUN_GAS | NB_CONV_0;
            self.write_register("CTRL_GAS_1", &[gas_config])?;
            run_gas = true;
        }

        // Enter forced mode
        let meas = self.os_temp << 5 | self.os_pres << 2 | MODE;
        self.write_register("CTRL_MEAS", &[meas])?;
        let mut max_sample_time = self.max_sample_time.get().unwrap_or(0.0);
        if run_gas {
            max_sample_time += self.gas_heat_duration as f64 / 1000.0;
        }
        self.pause(max_sample_time)?;
        let read = || -> KResult<Option<(Vec<u8>, Vec<u8>)>> {
            // wait until results are ready
            let byte = |r: &str| -> KResult<i64> { Ok(*self.read_register(r, 1)?.first().ok_or_else(short_read)? as i64) };
            let mut status = byte("EAS_STATUS_0")?;
            while status & MEASURE_IN_PROGRESS != 0 {
                self.pause(self.max_sample_time.get().unwrap_or(0.0))?;
                status = byte("EAS_STATUS_0")?;
            }

            // Nothing in progress and no new data
            if status & EAS_NEW_DATA == 0 {
                return Ok(None);
            }
            let data = self.read_register("PRESSURE_MSB", 8)?;
            if data.len() < 8 {
                return Err(short_read());
            }
            let mut gas_data = vec![0, 0];
            if run_gas {
                gas_data = self.read_register("GAS_R_MSB", 2)?;
                if gas_data.len() < 2 {
                    return Err(short_read());
                }
            }
            Ok(Some((data, gas_data)))
        };
        let (data, gas_data) = match read() {
            Ok(Some(r)) => r,
            Ok(None) => return Ok(reactor.monotonic() + REPORT_TIME),
            Err(e) => {
                logging::exception_err(&format!("BME680 {}: Error reading data", self.name), &e);
                self.temp.set(0.0);
                self.pressure.set(0.0);
                self.humidity.set(0.0);
                self.gas.set(0.0);
                return Ok(NEVER);
            }
        };

        let temp_raw = ((data[3] as i64) << 12) | ((data[4] as i64) << 4) | (data[5] >> 4) as i64;
        if temp_raw != 0x80000 {
            self.temp.set(self.compensate_temp(temp_raw as f64));
        }
        let pressure_raw = ((data[0] as i64) << 12) | ((data[1] as i64) << 4) | (data[2] >> 4) as i64;
        if pressure_raw != 0x80000 {
            self.pressure.set(self.compensate_pressure_bme680(pressure_raw as f64) / 100.0);
        }
        let humid_raw = ((data[6] as i64) << 8) | data[7] as i64;
        self.humidity.set(self.compensate_humidity_bme680(humid_raw as f64));

        let gas_valid = (gas_data[1] & 0x20) == 0x20;
        if gas_valid {
            let gas_heater_stable = (gas_data[1] & 0x10) == 0x10;
            if !gas_heater_stable {
                logging::warning(&format!("BME680 {}: Gas heater didn't reach target", self.name));
            }
            let gas_raw = ((gas_data[0] as i64) << 2) | ((gas_data[1] & 0xC0) >> 6) as i64;
            let gas_range = (gas_data[1] & 0x0F) as usize;
            self.gas.set(self.compensate_gas(gas_raw as f64, gas_range)?);
            // Disable gas measurement on success
            let gas_config = NB_CONV_0;
            self.write_register("CTRL_GAS_1", &[gas_config])?;
            self.last_gas_time.set(reactor.monotonic());
        }

        self.report(false, "BME680")
    }

    fn sample_bmp180(&self, _eventtime: f64) -> f64 {
        let res = (|| -> KResult<f64> {
            let reg = |k: &str| self.chip_registers.get().iter().find(|(n, _)| *n == k).map(|(_, r)| *r as i64).unwrap_or(0);
            let meas = reg("CRV_TEMP");
            self.write_register("CTRL_MEAS", &[meas])?;

            let temp_raw = match (|| -> KResult<i64> {
                self.pause(0.01)?;
                let data = self.read_register("REG_MSB", 2)?;
                if data.len() < 2 {
                    return Err(short_read());
                }
                Ok(((data[0] as i64) << 8) | data[1] as i64)
            })() {
                Ok(t) => t,
                Err(e) => {
                    logging::exception_err(&format!("BMP180 {}: Error reading temperature", self.name), &e);
                    self.temp.set(0.0);
                    self.pressure.set(0.0);
                    return Ok(NEVER);
                }
            };

            let meas = reg("CRV_PRES") | (self.os_pres << 6);
            self.write_register("CTRL_MEAS", &[meas])?;

            let pressure_raw = match (|| -> KResult<i64> {
                self.pause(0.01)?;
                let data = self.read_register("REG_MSB", 3)?;
                if data.len() < 3 {
                    return Err(short_read());
                }
                let v = ((data[0] as i64) << 16) | ((data[1] as i64) << 8) | data[2] as i64;
                let shift = 8 - self.os_pres;
                if shift < 0 {
                    return Err(KError::Py("ValueError".into(), "negative shift count".into()));
                }
                Ok(v >> shift)
            })() {
                Ok(p) => p,
                Err(e) => {
                    logging::exception_err(&format!("BMP180 {}: Error reading pressure", self.name), &e);
                    self.temp.set(0.0);
                    self.pressure.set(0.0);
                    return Ok(NEVER);
                }
            };

            self.temp.set(self.compensate_temp_bmp180(temp_raw as f64));
            self.pressure.set(self.compensate_pressure_bmp180(pressure_raw as f64)? / 100.0);
            self.report(false, "BMP180")
        })();
        res.unwrap_or_else(|e| {
            self.printer.unhandled_exception(&e);
            NEVER
        })
    }

    fn compensate_temp(&self, raw_temp: f64) -> f64 {
        let d = |k: &str| self.d(k);
        let var1 = (raw_temp / 16384.0 - (d("T1") / 1024.0)) * d("T2");
        let var2 = ((raw_temp / 131072.0) - (d("T1") / 8192.0)) * ((raw_temp / 131072.0) - (d("T1") / 8192.0)) * d("T3");
        self.t_fine.set(var1 + var2);
        self.t_fine.get() / 5120.0
    }

    fn compensate_pressure_bme280(&self, raw_pressure: f64) -> f64 {
        let d = |k: &str| self.d(k);
        let t_fine = self.t_fine.get();
        let mut var1 = t_fine / 2.0 - 64000.0;
        let mut var2 = var1 * var1 * d("P6") / 32768.0;
        var2 = var2 + var1 * d("P5") * 2.0;
        var2 = var2 / 4.0 + (d("P4") * 65536.0);
        var1 = (d("P3") * var1 * var1 / 524288.0 + d("P2") * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * d("P1");
        if var1 == 0.0 {
            0.0
        } else {
            let mut pressure = 1048576.0 - raw_pressure;
            pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1;
            let var1 = d("P9") * pressure * pressure / 2147483648.0;
            let var2 = pressure * d("P8") / 32768.0;
            pressure + (var1 + var2 + d("P7")) / 16.0
        }
    }

    fn compensate_pressure_bme680(&self, raw_pressure: f64) -> f64 {
        let d = |k: &str| self.d(k);
        let t_fine = self.t_fine.get();
        let mut var1 = t_fine / 2.0 - 64000.0;
        let mut var2 = var1 * var1 * d("P6") / 131072.0;
        var2 = var2 + var1 * d("P5") * 2.0;
        var2 = var2 / 4.0 + (d("P4") * 65536.0);
        var1 = (d("P3") * var1 * var1 / 16384.0 + d("P2") * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * d("P1");
        if var1 == 0.0 {
            0.0
        } else {
            let mut pressure = 1048576.0 - raw_pressure;
            pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1;
            let var1 = d("P9") * pressure * pressure / 2147483648.0;
            let var2 = pressure * d("P8") / 32768.0;
            let var3 = (pressure / 256.0) * (pressure / 256.0) * (pressure / 256.0) * (d("P10") / 131072.0);
            pressure + (var1 + var2 + var3 + (d("P7") * 128.0)) / 16.0
        }
    }

    fn compensate_humidity_bme280(&self, raw_humidity: f64) -> f64 {
        let d = |k: &str| self.d(k);
        let t_fine = self.t_fine.get();
        let mut humidity = t_fine - 76800.0;
        let h1 = raw_humidity - (d("H4") * 64.0 + d("H5") / 16384.0 * humidity);
        let h2 = d("H2") / 65536.0 * (1.0 + d("H6") / 67108864.0 * humidity * (1.0 + d("H3") / 67108864.0 * humidity));
        humidity = h1 * h2;
        humidity = humidity * (1.0 - d("H1") * humidity / 524288.0);
        py_min(100.0, py_max(0.0, humidity))
    }

    fn compensate_humidity_bme680(&self, raw_humidity: f64) -> f64 {
        let d = |k: &str| self.d(k);
        let temp_comp = self.temp.get();

        let var1 = raw_humidity - ((d("H1") * 16.0) + ((d("H3") / 2.0) * temp_comp));
        let var2 = var1 * ((d("H2") / 262144.0) * (1.0 + ((d("H4") / 16384.0) * temp_comp) + ((d("H5") / 1048576.0) * temp_comp * temp_comp)));
        let var3 = d("H6") / 16384.0;
        let var4 = d("H7") / 2097152.0;
        let humidity = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
        py_min(100.0, py_max(0.0, humidity))
    }

    fn compensate_gas(&self, gas_raw: f64, gas_range: usize) -> KResult<f64> {
        let gas_switching_error = *self.read_register("RANGE_SWITCHING_ERROR", 1)?.first().ok_or_else(short_read)? as f64;
        let (k0, k1) = BME680_GAS_CONSTANTS[gas_range];
        let var1 = (1340.0 + 5.0 * gas_switching_error) * k0;
        let gas = var1 * k1 / (gas_raw - 512.0 + var1);
        Ok(gas)
    }

    fn calc_gas_heater_resistance(&self, target_temp: f64) -> KResult<i64> {
        let amb_temp = self.temp.get();
        let heater_data = self.read_register("RES_HEAT_VAL", 3)?;
        if heater_data.len() < 3 {
            return Err(short_read());
        }
        let res_heat_val = get_signed_byte(heater_data[0]) as f64;
        let res_heat_range = ((heater_data[2] & 0x30) >> 4) as f64;
        let d = |k: &str| self.d(k);
        let var1 = (d("G1") / 16.0) + 49.0;
        let var2 = ((d("G2") / 32768.0) * 0.0005) + 0.00235;
        let var3 = d("G3") / 1024.0;
        let var4 = var1 * (1.0 + (var2 * target_temp));
        let var5 = var4 + (var3 * amb_temp);
        let res_heat = 3.4 * ((var5 * (4.0 / (4.0 + res_heat_range)) * (1.0 / (1.0 + (res_heat_val * 0.002)))) - 25.0);
        Ok(res_heat.trunc() as i64)
    }

    fn calc_gas_heater_duration(duration_ms: i64) -> i64 {
        if duration_ms >= 4032 {
            0xFF
        } else {
            let mut duration_ms = duration_ms;
            let mut factor = 0;
            while duration_ms > 0x3F {
                duration_ms = duration_ms.div_euclid(4);
                factor += 1;
            }
            duration_ms + (factor * 64)
        }
    }

    fn compensate_temp_bmp180(&self, raw_temp: f64) -> f64 {
        let d = |k: &str| self.d(k);
        let x1 = (raw_temp - d("AC6")) * d("AC5") / 32768.0;
        let x2 = d("MC") * 2048.0 / (x1 + d("MD"));
        let b5 = x1 + x2;
        self.t_fine.set(b5);
        (b5 + 8.0) / 16.0 / 10.0
    }

    fn compensate_pressure_bmp180(&self, raw_pressure: f64) -> KResult<f64> {
        let d = |k: &str| self.d(k);
        let b5 = self.t_fine.get();
        let b6 = b5 - 4000.0;
        let x1 = (d("B2") * (b6 * b6 / 4096.0)) / 2048.0;
        let x2 = d("AC2") * b6 / 2048.0;
        let x3 = x1 + x2;
        let ac = (d("AC1") * 4.0 + x3).trunc() as i64;
        if self.os_pres < 0 {
            return Err(KError::Py("ValueError".into(), "negative shift count".into()));
        }
        let b3 = ((ac << self.os_pres) + 2) as f64 / 4.0;
        let x1 = d("AC3") * b6 / 8192.0;
        let x2 = (d("B1") * (b6 * b6 / 4096.0)) / 65536.0;
        let x3 = ((x1 + x2) + 2.0) / 4.0;
        let b4 = d("AC4") * (x3 + 32768.0) / 32768.0;
        let b7 = (raw_pressure - b3) * (50000i64 >> self.os_pres) as f64;
        let mut p = if b7 < 0x80000000u32 as f64 { (b7 * 2.0) / b4 } else { (b7 / b4) * 2.0 };
        let x1 = (p / 256.0) * (p / 256.0);
        let x1 = (x1 * 3038.0) / 65536.0;
        let x2 = (-7357.0 * p) / 65536.0;
        p = p + (x1 + x2 + 3791.0) / 16.0;
        Ok(p)
    }

    fn read_id(&self) -> KResult<i64> {
        // read chip id register
        let params = self.i2c.i2c_read(&[BME_CHIP_ID_REG], 1, true)?;
        match params.get("response") {
            Some(Val::Bytes(b)) if !b.is_empty() => Ok(b[0] as i64),
            _ => Err(short_read()),
        }
    }

    fn read_bmp3_id(&self) -> KResult<i64> {
        // read chip id register
        let params = self.i2c.i2c_read(&[BMP3_CHIP_ID_REG], 1, true)?;
        match params.get("response") {
            Some(Val::Bytes(b)) if !b.is_empty() => Ok(b[0] as i64),
            _ => Err(short_read()),
        }
    }

    fn register(&self, reg_name: &str) -> KResult<u8> {
        self.chip_registers.get().iter().find(|(n, _)| *n == reg_name).map(|(_, r)| *r).ok_or_else(|| KError::Py("KeyError".into(), crate::pyfmt::py_str_repr(reg_name)))
    }

    fn read_register(&self, reg_name: &str, read_len: i64) -> KResult<Vec<u8>> {
        // read a single register
        let regs = [self.register(reg_name)?];
        let params = self.i2c.i2c_read(&regs, read_len, true)?;
        match params.get("response") {
            Some(Val::Bytes(b)) => Ok(b.clone()),
            _ => Ok(vec![]),
        }
    }

    fn write_register(&self, reg_name: &str, data: &[i64]) -> KResult<()> {
        let reg = self.register(reg_name)? as i64;
        let mut out = vec![];
        for v in std::iter::once(&reg).chain(data) {
            // the i2c_write message's bytearray(data)
            if !(0..256).contains(v) {
                return Err(KError::Py("ValueError".into(), "byte must be in range(0, 256)".into()));
            }
            out.push(*v as u8);
        }
        self.i2c.i2c_write(&out, 0, 0)
    }
}

impl TempSensor for BME280 {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
        Ok(())
    }
    fn setup_callback(&self, cb: TempCallback) {
        *self.callback.borrow_mut() = Some(cb);
    }
    fn get_report_time_delta(&self) -> f64 {
        REPORT_TIME
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl PrinterObject for BME280 {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let chip_type = *self.chip_type.borrow();
        let mut data = IndexMap::new();
        data.insert("temperature".to_string(), PyVal::Float(py_round(self.temp.get(), 2)));
        data.insert("pressure".to_string(), PyVal::Float(self.pressure.get()));
        if chip_type == "BME280" || chip_type == "BME680" {
            data.insert("humidity".to_string(), PyVal::Float(self.humidity.get()));
        }
        if chip_type == "BME680" {
            data.insert("gas".to_string(), PyVal::Float(self.gas.get()));
        }
        Some(PyVal::Dict(data))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory("BME280", Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(BME280::new(config)?) }));
    Ok(none_object())
}
