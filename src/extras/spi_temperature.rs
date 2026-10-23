// Support for common SPI based thermocouple and RTD temperature sensors
// (port of klippy/extras/spi_temperature.py)
//
// Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::extras::danger_options::get_danger_options;
use crate::extras::error_mcu::PrinterMcuError;
use crate::extras::heaters::{load_heaters, SensorFactory, TempCallback, TempSensor};
use crate::logging;
use crate::mcu::Mcu;
use crate::msgproto::{Params, Val};
use crate::printer::{none_object, PObj};
use crate::pyfmt::py_min;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

// ----------------------------------------------------------------------
// SensorBase
// ----------------------------------------------------------------------

const REPORT_TIME: f64 = 0.300;
const MAX_INVALID_COUNT: i64 = 3;

#[derive(Clone, Copy)]
enum Chip {
    Max6675,
    Max31855,
    Max31856,
    Max31865 { adc_to_resist_div_nominal: f64 },
}

pub struct SensorBase {
    name: String,
    chip: Chip,
    chip_type: &'static str,
    callback: RefCell<Option<TempCallback>>,
    min_sample_value: Cell<i64>,
    max_sample_value: Cell<i64>,
    minmax_temp: Cell<Option<(f64, f64)>>,
    last: Cell<Option<(i64, i64)>>,
    report_clock: Cell<i64>,
    spi: Rc<McuSpi>,
    mcu: Rc<Mcu>,
    oid: i64,
    is_connected: Cell<bool>,
    config_reg: Vec<u8>,
}

fn int_of(p: &Params, k: &str) -> i64 {
    match p.get(k) {
        Some(Val::Int(v)) => *v,
        _ => 0,
    }
}

impl SensorBase {
    fn new(config: &ConfigWrapper, chip: Chip, chip_type: &'static str, config_cmd: Option<Vec<u8>>, spi_mode: i64) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let spi = mcu_spi_from_config(config, spi_mode, "sensor_pin", 4000000, None, false)?;
        if let Some(cmd) = &config_cmd {
            spi.spi_send(cmd, 0, 0)?;
        }
        let mcu = spi.get_mcu();
        // Reader chip configuration
        let oid = mcu.create_oid();
        let s = Rc::new(SensorBase {
            name: config.get_name().to_string(),
            chip,
            chip_type,
            callback: RefCell::new(None),
            min_sample_value: Cell::new(0),
            max_sample_value: Cell::new(0),
            minmax_temp: Cell::new(None),
            last: Cell::new(None),
            report_clock: Cell::new(0),
            spi,
            mcu: mcu.clone(),
            oid,
            is_connected: Cell::new(false),
            config_reg: config_cmd.unwrap_or_default(),
        });
        let ss = s.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| {
                ss.is_connected.set(true);
                Ok(())
            }),
        );
        let ss = s.clone();
        mcu.register_response(Some(Rc::new(move |params: &Params| ss.handle_spi_response(params))), "thermocouple_result", Some(oid));
        let ss = s.clone();
        mcu.register_config_callback(Rc::new(move || ss.build_config()));
        let error_mcu = printer.load_as::<PrinterMcuError>(config, "error_mcu")?;
        let ss = s.clone();
        error_mcu.add_clarify("Thermocouple reader fault", Rc::new(move |msg, details| ss.clarify_fault(msg, details)));
        Ok(s)
    }

    fn build_config(&self) -> KResult<()> {
        self.mcu.add_config_cmd(
            &format!("config_thermocouple oid={} spi_oid={} thermocouple_type={}", self.oid, self.spi.get_oid(), self.chip_type),
            false,
            false,
        );
        let clock = self.mcu.get_query_slot(self.oid);
        self.report_clock.set(self.mcu.seconds_to_clock(REPORT_TIME));
        let danger_check_count = if get_danger_options().temp_ignore_limits { 0 } else { MAX_INVALID_COUNT };
        self.mcu.add_config_cmd(
            &format!(
                "query_thermocouple oid={} clock={} rest_ticks={} min_value={} max_value={} max_invalid_count={}",
                self.oid,
                clock,
                self.report_clock.get(),
                self.min_sample_value.get(),
                self.max_sample_value.get(),
                danger_check_count
            ),
            true,
            false,
        );
        Ok(())
    }

    fn handle_spi_response(&self, params: &Params) -> KResult<()> {
        let (value, fault) = (int_of(params, "value"), int_of(params, "fault"));
        self.last.set(Some((value, fault)));
        if !self.is_connected.get() {
            return Ok(());
        }
        if fault != 0 {
            return self.handle_fault(fault);
        }
        let temp = self.calc_temp(value).ok_or_else(|| KError::Other("math domain error".into()))?;
        let next_clock = self.mcu.clock32_to_clock64(int_of(params, "next_clock"));
        let last_read_clock = next_clock - self.report_clock.get();
        let last_read_time = self.mcu.clock_to_print_time(last_read_clock as f64);
        let cb = self.callback.borrow().clone();
        match cb {
            Some(cb) => cb(last_read_time, temp),
            None => Err(KError::Other("'NoneType' object is not callable".into())),
        }
    }

    fn clarify_fault(&self, _msg: &str, _details: &crate::pyval::PyVal) -> Option<String> {
        // Report which sensor caused a "Thermocouple reader fault" shutdown
        let (min_temp, max_temp) = self.minmax_temp.get()?;
        let (value, fault) = self.last.get()?;
        if fault != 0 {
            return Some(format!("Sensor '{}' reported fault {:#x}", self.name, fault));
        }
        if self.min_sample_value.get() <= value && value <= self.max_sample_value.get() {
            return None;
        }
        let tempstr = match self.calc_temp(value) {
            Some(temp) => format!("{:.3}", temp),
            None => {
                logging::exception_tb("Error in calc_temp callback", "ValueError", "math domain error");
                "?".to_string()
            }
        };
        Some(format!("Sensor '{}' temperature {} not in range {:.3}:{:.3}", self.name, tempstr, min_temp, max_temp))
    }

    fn report_fault(&self, msg: &str) {
        logging::warning(msg);
    }

    fn handle_fault(&self, fault: i64) -> KResult<()> {
        let faults: &[(i64, &str)] = match self.chip {
            Chip::Max31856 => &[
                (0x80, "Max31856: Cold Junction Range Fault"),
                (0x40, "Max31856: Thermocouple Range Fault"),
                (0x20, "Max31856: Cold Junction High Fault"),
                (0x10, "Max31856: Cold Junction Low Fault"),
                (0x08, "Max31856: Thermocouple High Fault"),
                (0x04, "Max31856: Thermocouple Low Fault"),
                (0x02, "Max31856: Over/Under Voltage Fault"),
                (0x01, "Max31856: Thermocouple Open Fault"),
            ],
            Chip::Max31855 => &[(0x1, "MAX31855 : Open Circuit"), (0x2, "MAX31855 : Short to GND"), (0x4, "MAX31855 : Short to Vcc")],
            Chip::Max6675 => &[(0x02, "Max6675 : Device ID error"), (0x04, "Max6675 : Thermocouple Open Fault")],
            Chip::Max31865 { .. } => &[
                (0x80, "Max31865 RTD input is disconnected"),
                (0x40, "Max31865 RTD input is shorted"),
                (0x20, "Max31865 VREF- is greater than 0.85 * VBIAS, FORCE- open"),
                (0x10, "Max31865 VREF- is less than 0.85 * VBIAS, FORCE- open"),
                (0x08, "Max31865 VRTD- is less than 0.85 * VBIAS, FORCE- open"),
                (0x04, "Max31865 Overvoltage or undervoltage fault"),
            ],
        };
        for (bit, msg) in faults {
            if fault & bit != 0 {
                self.report_fault(msg);
            }
        }
        if let Chip::Max31865 { .. } = self.chip {
            if fault & 0xFC == 0 {
                self.report_fault("Max31865 Unspecified error");
            }
            // Attempt to clear the fault
            self.spi.spi_send(&self.config_reg, 0, 0)?;
        }
        Ok(())
    }

    /// calc_temp(adc); None where python's math.sqrt raises ValueError
    fn calc_temp(&self, adc: i64) -> Option<f64> {
        // (scale, sign bit, value mask, mult)
        let thermocouple = |scale: u32, sign: i64, mask: i64, mult: f64| {
            let mut adc = adc >> scale;
            // Fix sign bit:
            if adc & sign != 0 {
                adc = ((adc & mask) + 1) * -1;
            }
            mult * adc as f64
        };
        match self.chip {
            Chip::Max31856 => Some(thermocouple(MAX31856_SCALE, 0x40000, 0x3FFFF, MAX31856_MULT)),
            Chip::Max31855 => Some(thermocouple(18, 0x2000, 0x1FFF, 0.25)),
            Chip::Max6675 => Some(thermocouple(3, 0x2000, 0x1FFF, 0.25)),
            Chip::Max31865 { adc_to_resist_div_nominal } => {
                let adc = adc >> 1; // remove fault bit
                let r_div_nominal = adc as f64 * adc_to_resist_div_nominal;
                // Resistance (relative to rtd_nominal_r) is calculated using:
                //  R_div_nominal = 1. + CVD_A * temp + CVD_B * temp**2
                // Solve for temp using quadratic equation:
                //  temp = (-b +- sqrt(b**2 - 4ac)) / 2a
                let d2 = CVD_A.powf(2.0) - 4.0 * CVD_B * (1.0 - r_div_nominal);
                if d2 < 0.0 {
                    return None;
                }
                let discriminant = d2.sqrt();
                Some((-CVD_A + discriminant) / (2.0 * CVD_B))
            }
        }
    }

    fn calc_adc(&self, temp: f64) -> i64 {
        let thermocouple = |mult: f64, max: i64, scale: u32| 0.max(max.min(((temp / mult) + 0.5).trunc() as i64)) << scale;
        match self.chip {
            Chip::Max31856 => thermocouple(MAX31856_MULT, 0x3FFFF, MAX31856_SCALE),
            Chip::Max31855 => thermocouple(0.25, 0x1FFF, 18),
            Chip::Max6675 => thermocouple(0.25, 0x1FFF, 3),
            Chip::Max31865 { adc_to_resist_div_nominal } => {
                // Calculate relative resistance via Callendar-Van Dusen formula:
                //  resistance = rtd_nominal_r * (1 + CVD_A * temp + CVD_B * temp**2)
                let temp = py_min(temp, 1768.3); // Melting point of platinum
                let r_div_nominal = 1.0 + CVD_A * temp + CVD_B * temp * temp;
                let adc = (r_div_nominal / adc_to_resist_div_nominal + 0.5).trunc() as i64;
                let adc = 0.max((MAX31865_ADC_MAX - 1).min(adc));
                adc << 1 // Add fault bit
            }
        }
    }
}

impl TempSensor for SensorBase {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.minmax_temp.set(Some((min_temp, max_temp)));
        let adc_range = [self.calc_adc(min_temp), self.calc_adc(max_temp)];
        self.min_sample_value.set(adc_range[0].min(adc_range[1]));
        self.max_sample_value.set(adc_range[0].max(adc_range[1]));
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

// ----------------------------------------------------------------------
// MAX31856 thermocouple
// ----------------------------------------------------------------------

const MAX31856_CR0_REG: u8 = 0x00;
const MAX31856_CR0_AUTOCONVERT: u8 = 0x80;
const MAX31856_CR0_FILT50HZ: u8 = 0x01;
const MAX31856_CR1_AVGSEL1: u8 = 0x00;
const MAX31856_CR1_AVGSEL2: u8 = 0x10;
const MAX31856_CR1_AVGSEL4: u8 = 0x20;
const MAX31856_CR1_AVGSEL8: u8 = 0x30;
const MAX31856_CR1_AVGSEL16: u8 = 0x70;
const MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT: u8 = 0x02;
const MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT: u8 = 0x01;
const MAX31856_SCALE: u32 = 5;
const MAX31856_MULT: f64 = 0.0078125;

fn max31856_spi_init(config: &ConfigWrapper) -> KResult<Vec<u8>> {
    let mut cmds = vec![];
    let mut value = MAX31856_CR0_AUTOCONVERT;
    if config.getboolean("tc_use_50Hz_filter", false)? {
        value |= MAX31856_CR0_FILT50HZ;
    }
    cmds.push(0x80 + MAX31856_CR0_REG);
    cmds.push(value);

    let types: [(&str, u8); 8] =
        [("B", 0b0000), ("E", 0b0001), ("J", 0b0010), ("K", 0b0011), ("N", 0b0100), ("R", 0b0101), ("S", 0b0110), ("T", 0b0111)];
    let mut value = config.getchoice("tc_type", &types, "K")?;
    let averages: [(i64, u8); 5] = [
        (1, MAX31856_CR1_AVGSEL1),
        (2, MAX31856_CR1_AVGSEL2),
        (4, MAX31856_CR1_AVGSEL4),
        (8, MAX31856_CR1_AVGSEL8),
        (16, MAX31856_CR1_AVGSEL16),
    ];
    value |= config.getchoice_int("tc_averaging_count", &averages, 1)?;
    cmds.push(value);

    cmds.push(MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT | MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT);
    Ok(cmds)
}

// ----------------------------------------------------------------------
// MAX31865 (RTD sensor)
// ----------------------------------------------------------------------

const MAX31865_CONFIG_REG: u8 = 0x00;
const MAX31865_CONFIG_BIAS: u8 = 0x80;
const MAX31865_CONFIG_MODEAUTO: u8 = 0x40;
const MAX31865_CONFIG_3WIRE: u8 = 0x10;
const MAX31865_CONFIG_FAULTCLEAR: u8 = 0x02;
const MAX31865_CONFIG_FILT50HZ: u8 = 0x01;
const MAX31865_ADC_MAX: i64 = 1 << 15;

// Callendar-Van Dusen constants for platinum resistance thermometers (RTD)
const CVD_A: f64 = 3.9083e-3;
const CVD_B: f64 = -5.775e-7;

fn max31865_spi_init(config: &ConfigWrapper) -> KResult<Vec<u8>> {
    let mut value = MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_FAULTCLEAR;
    if config.getboolean("rtd_use_50Hz_filter", false)? {
        value |= MAX31865_CONFIG_FILT50HZ;
    }
    if config.getint("rtd_num_of_wires", 2, Lim::new())? == 3 {
        value |= MAX31865_CONFIG_3WIRE;
    }
    let cmd = 0x80 + MAX31865_CONFIG_REG;
    Ok(vec![cmd, value])
}

// ----------------------------------------------------------------------
// Sensor registration
// ----------------------------------------------------------------------

fn max6675(config: &ConfigWrapper) -> KResult<Rc<dyn TempSensor>> {
    Ok(SensorBase::new(config, Chip::Max6675, "MAX6675", None, 0)?)
}

fn max31855(config: &ConfigWrapper) -> KResult<Rc<dyn TempSensor>> {
    Ok(SensorBase::new(config, Chip::Max31855, "MAX31855", None, 0)?)
}

fn max31856(config: &ConfigWrapper) -> KResult<Rc<dyn TempSensor>> {
    let init = max31856_spi_init(config)?;
    Ok(SensorBase::new(config, Chip::Max31856, "MAX31856", Some(init), 1)?)
}

fn max31865(config: &ConfigWrapper) -> KResult<Rc<dyn TempSensor>> {
    let rtd_nominal_r = config.getfloat("rtd_nominal_r", 100.0, Lim::new().above(0.0))?;
    let rtd_reference_r = config.getfloat("rtd_reference_r", 430.0, Lim::new().above(0.0))?;
    let adc_to_resist = rtd_reference_r / MAX31865_ADC_MAX as f64;
    let adc_to_resist_div_nominal = adc_to_resist / rtd_nominal_r;
    let config_reg = max31865_spi_init(config)?;
    Ok(SensorBase::new(config, Chip::Max31865 { adc_to_resist_div_nominal }, "MAX31865", Some(config_reg), 1)?)
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensors
    let pheaters = load_heaters(config)?;
    let sensors: [(&str, SensorFactory); 4] = [
        ("MAX6675", Rc::new(max6675)),
        ("MAX31855", Rc::new(max31855)),
        ("MAX31856", Rc::new(max31856)),
        ("MAX31865", Rc::new(max31865)),
    ];
    for (name, klass) in sensors {
        pheaters.add_sensor_factory(name, klass);
    }
    Ok(none_object())
}
