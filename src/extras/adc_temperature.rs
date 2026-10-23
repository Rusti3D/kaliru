// Obtain temperature using linear interpolation of ADC values
// (port of klippy/extras/adc_temperature.py)
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::extras::error_mcu::PrinterMcuError;
use crate::extras::heaters::{load_heaters, TempCallback, TempSensor};
use crate::extras::query_adc::QueryADC;
use crate::logging;
use crate::mcu::McuAdc;
use crate::printer::{downcast_rc, none_object, PObj, Printer};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

// ----------------------------------------------------------------------
// Interface between MCU adc and heater temperature callbacks
// ----------------------------------------------------------------------

pub const SAMPLE_TIME: f64 = 0.001;
pub const SAMPLE_COUNT: i64 = 8;
pub const REPORT_TIME: f64 = 0.300;
pub const RANGE_CHECK_COUNT: i64 = 4;

/// an adc value <-> temperature converter (thermistor, linear tables)
pub trait AdcConvert {
    fn calc_temp(&self, adc: f64) -> KResult<f64>;
    fn calc_adc(&self, temp: f64) -> KResult<f64>;
}

/// python's ValueError("math domain error")
pub fn math_domain_error() -> KError {
    KError::Other("math domain error".into())
}

// Interface between ADC and heater temperature callbacks
pub struct PrinterADCtoTemperature {
    adc_convert: Rc<dyn AdcConvert>,
    pub mcu_adc: Rc<McuAdc>,
    diag_helper: Rc<HelperTemperatureDiagnostics>,
    temperature_callback: RefCell<Option<TempCallback>>,
}

impl PrinterADCtoTemperature {
    pub fn new(config: &ConfigWrapper, adc_convert: Rc<dyn AdcConvert>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mcu_adc = crate::pins::setup_pin_as::<McuAdc>(&printer, "adc", &config.get("sensor_pin", Dflt::Required)?)?;
        let ac = adc_convert.clone();
        let diag_helper = HelperTemperatureDiagnostics::new(config, &mcu_adc, Rc::new(move |v| ac.calc_temp(v)))?;
        let s = Rc::new(PrinterADCtoTemperature { adc_convert, mcu_adc: mcu_adc.clone(), diag_helper, temperature_callback: RefCell::new(None) });
        let w = Rc::downgrade(&s);
        mcu_adc.setup_adc_callback(
            REPORT_TIME,
            Rc::new(move |read_time, read_value| match w.upgrade() {
                Some(s) => s.adc_callback(read_time, read_value),
                None => Ok(()),
            }),
        );
        Ok(s)
    }

    fn adc_callback(&self, read_time: f64, read_value: f64) -> KResult<()> {
        let temp = self.adc_convert.calc_temp(read_value)?;
        let cb = self.temperature_callback.borrow().clone();
        match cb {
            Some(cb) => cb(read_time + SAMPLE_COUNT as f64 * SAMPLE_TIME, temp),
            None => Err(KError::Other("'PrinterADCtoTemperature' object has no attribute 'temperature_callback'".into())),
        }
    }
}

impl TempSensor for PrinterADCtoTemperature {
    fn setup_callback(&self, cb: TempCallback) {
        *self.temperature_callback.borrow_mut() = Some(cb);
    }

    fn get_report_time_delta(&self) -> f64 {
        REPORT_TIME
    }

    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        let danger_check_count = if get_danger_options().temp_ignore_limits { 0 } else { RANGE_CHECK_COUNT };

        let a = self.adc_convert.calc_adc(min_temp)?;
        let b = self.adc_convert.calc_adc(max_temp)?;
        let (min_adc, max_adc) = if b < a { (b, a) } else { (a, b) };
        self.mcu_adc.setup_adc_sample(SAMPLE_TIME, SAMPLE_COUNT, min_adc, max_adc, danger_check_count);
        self.diag_helper.setup_diag_minmax(min_temp, max_temp, min_adc, max_adc);
        Ok(())
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub type CalcTempCb = Rc<dyn Fn(f64) -> KResult<f64>>;

// Tool to register with query_adc and report extra info on ADC range errors
pub struct HelperTemperatureDiagnostics {
    name: String,
    mcu_adc: Rc<McuAdc>,
    calc_temp_cb: CalcTempCb,
    /// (min_temp, max_temp, min_adc, max_adc) once setup_diag_minmax ran
    minmax: Cell<Option<(f64, f64, f64, f64)>>,
}

impl HelperTemperatureDiagnostics {
    pub fn new(config: &ConfigWrapper, mcu_adc: &Rc<McuAdc>, calc_temp_cb: CalcTempCb) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let h = Rc::new(HelperTemperatureDiagnostics {
            name: config.get_name().to_string(),
            mcu_adc: mcu_adc.clone(),
            calc_temp_cb,
            minmax: Cell::new(None),
        });
        let query_adc = load_as::<QueryADC>(&printer, config, "query_adc")?;
        query_adc.register_adc(&h.name, mcu_adc.clone());
        let error_mcu = load_as::<PrinterMcuError>(&printer, config, "error_mcu")?;
        let hh = h.clone();
        error_mcu.add_clarify("ADC out of range", Rc::new(move |msg, details| hh.clarify_adc_range(msg, details)));
        Ok(h)
    }

    pub fn setup_diag_minmax(&self, min_temp: f64, max_temp: f64, min_adc: f64, max_adc: f64) {
        self.minmax.set(Some((min_temp, max_temp, min_adc, max_adc)));
    }

    fn clarify_adc_range(&self, _msg: &str, _details: &PyVal) -> Option<String> {
        let (min_temp, max_temp, min_adc, max_adc) = self.minmax.get()?;
        let (last_value, last_read_time) = self.mcu_adc.get_last_value();
        if last_read_time == 0.0 {
            return None;
        }
        if last_value >= min_adc && last_value <= max_adc {
            return None;
        }
        let tempstr = match (self.calc_temp_cb)(last_value) {
            Ok(last_temp) => format!("{:.3}", last_temp),
            Err(e) => {
                logging::exception_err("Error in calc_temp callback", &e);
                "?".to_string()
            }
        };
        Some(format!("Sensor '{}' temperature {} not in range {:.3}:{:.3}", self.name, tempstr, min_temp, max_temp))
    }
}

/// printer.load_object(config, name) as a concrete type
fn load_as<T: 'static>(printer: &Rc<Printer>, config: &ConfigWrapper, name: &str) -> KResult<Rc<T>> {
    printer
        .load_object(config, name, true)?
        .and_then(downcast_rc::<T>)
        .ok_or_else(|| KError::Other(format!("{} has an unexpected type", name)))
}

// ----------------------------------------------------------------------
// Linear interpolation
// ----------------------------------------------------------------------

// Helper code to perform linear interpolation
pub struct LinearInterpolate {
    keys: Vec<f64>,
    slopes: Vec<(f64, f64)>,
}

impl LinearInterpolate {
    /// Err is the ValueError message
    pub fn new(samples: &[(f64, f64)]) -> Result<Self, String> {
        let mut keys = vec![];
        let mut slopes: Vec<(f64, f64)> = vec![];
        let mut sorted = samples.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut last: Option<(f64, f64)> = None;
        for (key, value) in sorted {
            let Some((last_key, last_value)) = last else {
                last = Some((key, value));
                continue;
            };
            if key <= last_key {
                return Err("duplicate value".into());
            }
            let gain = (value - last_value) / (key - last_key);
            let offset = last_value - last_key * gain;
            if slopes.last() == Some(&(gain, offset)) {
                continue;
            }
            last = Some((key, value));
            keys.push(key);
            slopes.push((gain, offset));
        }
        if keys.is_empty() {
            return Err("need at least two samples".into());
        }
        keys.push(9999999999999.0);
        slopes.push(*slopes.last().unwrap());
        Ok(LinearInterpolate { keys, slopes })
    }

    pub fn interpolate(&self, key: f64) -> KResult<f64> {
        // bisect.bisect (bisect_right)
        let pos = self.keys.partition_point(|k| !(key < *k));
        let (gain, offset) = *self.slopes.get(pos).ok_or_else(|| KError::Other("list index out of range".into()))?;
        Ok(key * gain + offset)
    }

    pub fn reverse_interpolate(&self, value: f64) -> f64 {
        let values: Vec<f64> = self.keys.iter().zip(&self.slopes).map(|(key, (gain, offset))| key * gain + offset).collect();
        let n = values.len();
        let valid: Vec<usize> = if values[0] < values[n - 2] {
            (0..n).filter(|&i| values[i] >= value).collect()
        } else {
            (0..n).filter(|&i| values[i] <= value).collect()
        };
        let idx = valid.into_iter().chain([n - 1]).min().unwrap();
        let (gain, offset) = self.slopes[idx];
        (value - offset) / gain
    }
}

// ----------------------------------------------------------------------
// Linear voltage to temperature converter
// ----------------------------------------------------------------------

// Linear style conversion chips calibrated from temperature measurements
pub struct LinearVoltage {
    li: LinearInterpolate,
}

impl LinearVoltage {
    pub fn new(config: &ConfigWrapper, params: &[(f64, f64)]) -> KResult<Self> {
        let adc_voltage = config.getfloat("adc_voltage", 5.0, Lim::new().above(0.0))?;
        let voltage_offset = config.getfloat("voltage_offset", 0.0, Lim::new())?;
        let mut samples = vec![];
        for &(temp, volt) in params {
            let adc = (volt - voltage_offset) / adc_voltage;
            if adc < 0.0 || adc > 1.0 {
                logging::warning(&format!("Ignoring adc sample {:.3}/{:.3} in heater {}", temp, volt, config.get_name()));
                continue;
            }
            samples.push((adc, temp));
        }
        let li = LinearInterpolate::new(&samples)
            .map_err(|e| KError::config(format!("adc_temperature {} in heater {}", e, config.get_name())))?;
        Ok(LinearVoltage { li })
    }
}

impl AdcConvert for LinearVoltage {
    fn calc_temp(&self, adc: f64) -> KResult<f64> {
        self.li.interpolate(adc)
    }
    fn calc_adc(&self, temp: f64) -> KResult<f64> {
        Ok(self.li.reverse_interpolate(temp))
    }
}

/// the temperatureN / <value>N pairs of a custom sensor section
fn read_pairs(config: &ConfigWrapper, value_name: &str) -> KResult<Vec<(f64, f64)>> {
    let mut params = vec![];
    for i in 1..1000 {
        let Some(t) = config.getfloat_opt(&format!("temperature{}", i), Lim::new())? else {
            break;
        };
        let v = config.getfloat(&format!("{}{}", value_name, i), Dflt::Required, Lim::new())?;
        params.push((t, v));
    }
    Ok(params)
}

/// " ".join(config.get_name().split()[1:])
pub fn custom_name(config: &ConfigWrapper) -> String {
    config.get_name().split_whitespace().skip(1).collect::<Vec<_>>().join(" ")
}

// ----------------------------------------------------------------------
// Linear resistance to temperature converter
// ----------------------------------------------------------------------

// Linear resistance calibrated from temperature measurements
pub struct LinearResistance {
    pullup: f64,
    inline_resistor: f64,
    li: LinearInterpolate,
}

impl LinearResistance {
    pub fn new(config: &ConfigWrapper, samples: &[(f64, f64)]) -> KResult<Self> {
        let pullup = config.getfloat("pullup_resistor", 4700.0, Lim::new().above(0.0))?;
        let inline_resistor = config.getfloat("inline_resistor", 0.0, Lim::new().min(0.0))?;
        let rt: Vec<(f64, f64)> = samples.iter().map(|&(t, r)| (r, t)).collect();
        let li = LinearInterpolate::new(&rt)
            .map_err(|e| KError::config(format!("adc_temperature {} in heater {}", e, config.get_name())))?;
        Ok(LinearResistance { pullup, inline_resistor, li })
    }
}

impl AdcConvert for LinearResistance {
    fn calc_temp(&self, adc: f64) -> KResult<f64> {
        // Calculate temperature from adc
        let adc = py_max(0.00001, py_min(0.99999, adc));
        let r = self.pullup * adc / (1.0 - adc) - self.inline_resistor;
        self.li.interpolate(r)
    }
    fn calc_adc(&self, temp: f64) -> KResult<f64> {
        // Calculate adc reading from a temperature
        let r = self.li.reverse_interpolate(temp) + self.inline_resistor;
        Ok(r / (self.pullup + r))
    }
}

// ----------------------------------------------------------------------
// Default sensors
// ----------------------------------------------------------------------

include!("adc_temperature_tables.rs");

pub fn calc_pt100(base: f64) -> Vec<(f64, f64)> {
    // Calc PT100/PT1000 resistances using Callendar-Van Dusen formula
    let (a, b) = (3.9083e-3, -5.775e-7);
    (0..500).step_by(10).map(|t| (t as f64, base * (1.0 + a * t as f64 + b * t as f64 * t as f64))).collect()
}

pub fn calc_ina826_pt100() -> Vec<(f64, f64)> {
    // Standard circuit is 4400ohm pullup with 10x gain to 5V
    calc_pt100(100.0).into_iter().map(|(t, r)| (t, 10.0 * 5.0 * r / (4400.0 + r))).collect()
}

fn default_voltage_sensors() -> Vec<(&'static str, Vec<(f64, f64)>)> {
    vec![
        ("AD595", AD595.to_vec()),
        ("AD597", AD597.to_vec()),
        ("AD8494", AD8494.to_vec()),
        ("AD8495", AD8495.to_vec()),
        ("AD8496", AD8496.to_vec()),
        ("AD8497", AD8497.to_vec()),
        ("PT100 INA826", calc_ina826_pt100()),
    ]
}

fn voltage_factory(params: Vec<(f64, f64)>) -> crate::extras::heaters::SensorFactory {
    Rc::new(move |config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> {
        Ok(PrinterADCtoTemperature::new(config, Rc::new(LinearVoltage::new(config, &params)?))?)
    })
}

fn resistance_factory(samples: Vec<(f64, f64)>) -> crate::extras::heaters::SensorFactory {
    Rc::new(move |config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> {
        Ok(PrinterADCtoTemperature::new(config, Rc::new(LinearResistance::new(config, &samples)?))?)
    })
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register default sensors
    let pheaters = load_heaters(config)?;
    for (sensor_type, params) in default_voltage_sensors() {
        pheaters.add_sensor_factory(sensor_type, voltage_factory(params));
    }
    pheaters.add_sensor_factory("PT1000", resistance_factory(calc_pt100(1000.0)));
    Ok(none_object())
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let name = custom_name(config);
    let factory = if config.get_opt("resistance1")?.is_none() {
        // CustomLinearVoltage
        voltage_factory(read_pairs(config, "voltage")?)
    } else {
        // CustomLinearResistance
        resistance_factory(read_pairs(config, "resistance")?)
    };
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(&name, factory);
    Ok(none_object())
}
