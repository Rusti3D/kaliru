// Temperature measurements with thermistors (port of klippy/extras/thermistor.py)
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::extras::adc_temperature::{custom_name, math_domain_error, AdcConvert, PrinterADCtoTemperature};
use crate::extras::heaters::{load_heaters, TempSensor};
use crate::logging;
use crate::printer::{none_object, PObj};
use crate::pyfmt::{py_max, py_min};
use std::cell::Cell;
use std::rc::Rc;

pub const KELVIN_TO_CELSIUS: f64 = -273.15;

/// math.log(x)
fn py_log(x: f64) -> KResult<f64> {
    if x <= 0.0 {
        return Err(math_domain_error());
    }
    Ok(x.ln())
}

// Analog voltage to temperature converter for thermistors
pub struct Thermistor {
    pullup: f64,
    inline_resistor: f64,
    c1: Cell<f64>,
    c2: Cell<f64>,
    c3: Cell<f64>,
}

impl Thermistor {
    pub fn new(pullup: f64, inline_resistor: f64) -> Self {
        Thermistor { pullup, inline_resistor, c1: Cell::new(0.0), c2: Cell::new(0.0), c3: Cell::new(0.0) }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn setup_coefficients(&self, t1: f64, r1: f64, t2: f64, r2: f64, t3: f64, r3: f64, name: &str) -> KResult<()> {
        // Calculate Steinhart-Hart coefficents from temp measurements.
        // Arrange samples as 3 linear equations and solve for c1, c2, and c3.
        let inv_t1 = 1.0 / (t1 - KELVIN_TO_CELSIUS);
        let inv_t2 = 1.0 / (t2 - KELVIN_TO_CELSIUS);
        let inv_t3 = 1.0 / (t3 - KELVIN_TO_CELSIUS);
        let ln_r1 = py_log(r1)?;
        let ln_r2 = py_log(r2)?;
        let ln_r3 = py_log(r3)?;
        let (ln3_r1, ln3_r2, ln3_r3) = (ln_r1.powf(3.0), ln_r2.powf(3.0), ln_r3.powf(3.0));

        let (inv_t12, inv_t13) = (inv_t1 - inv_t2, inv_t1 - inv_t3);
        let (ln_r12, ln_r13) = (ln_r1 - ln_r2, ln_r1 - ln_r3);
        let (ln3_r12, ln3_r13) = (ln3_r1 - ln3_r2, ln3_r1 - ln3_r3);

        self.c3.set((inv_t12 - inv_t13 * ln_r12 / ln_r13) / (ln3_r12 - ln3_r13 * ln_r12 / ln_r13));
        if self.c3.get() <= 0.0 {
            let beta = ln_r13 / inv_t13;
            logging::warning(&format!("Using thermistor beta {:.3} in heater {}", beta, name));
            return self.setup_coefficients_beta(t1, r1, beta);
        }
        self.c2.set((inv_t12 - self.c3.get() * ln3_r12) / ln_r12);
        self.c1.set(inv_t1 - self.c2.get() * ln_r1 - self.c3.get() * ln3_r1);
        Ok(())
    }

    pub fn setup_coefficients_beta(&self, t1: f64, r1: f64, beta: f64) -> KResult<()> {
        // Calculate equivalent Steinhart-Hart coefficents from beta
        let inv_t1 = 1.0 / (t1 - KELVIN_TO_CELSIUS);
        let ln_r1 = py_log(r1)?;
        self.c3.set(0.0);
        self.c2.set(1.0 / beta);
        self.c1.set(inv_t1 - self.c2.get() * ln_r1);
        Ok(())
    }
}

impl AdcConvert for Thermistor {
    fn calc_temp(&self, adc: f64) -> KResult<f64> {
        // Calculate temperature from adc
        let adc = py_max(0.00001, py_min(0.99999, adc));
        let r = self.pullup * adc / (1.0 - adc);
        let ln_r = py_log(r - self.inline_resistor)?;
        let inv_t = self.c1.get() + self.c2.get() * ln_r + self.c3.get() * ln_r.powf(3.0);
        Ok(1.0 / inv_t + KELVIN_TO_CELSIUS)
    }

    fn calc_adc(&self, temp: f64) -> KResult<f64> {
        // Calculate adc reading from a temperature
        if temp <= KELVIN_TO_CELSIUS {
            return Ok(1.0);
        }
        let inv_t = 1.0 / (temp - KELVIN_TO_CELSIUS);
        let ln_r = if self.c3.get() != 0.0 {
            // Solve for ln_r using Cardano's formula
            let y = (self.c1.get() - inv_t) / (2.0 * self.c3.get());
            let x2 = (self.c2.get() / (3.0 * self.c3.get())).powf(3.0) + y.powf(2.0);
            if x2 < 0.0 {
                return Err(math_domain_error());
            }
            let x = x2.sqrt();
            let pow13 = |v: f64| if v < 0.0 { Err(math_domain_error()) } else { Ok(v.powf(1.0 / 3.0)) };
            pow13(x - y)? - pow13(x + y)?
        } else {
            (inv_t - self.c1.get()) / self.c2.get()
        };
        let r = ln_r.exp() + self.inline_resistor;
        Ok(r / (self.pullup + r))
    }
}

/// thermistor parameters: (t1, r1, beta) or the three points
#[derive(Clone)]
pub enum ThermistorParams {
    Beta { t1: f64, r1: f64, beta: f64 },
    Points { t1: f64, r1: f64, t2: f64, r2: f64, t3: f64, r3: f64 },
}

// Create an ADC converter with a thermistor
pub fn printer_thermistor(config: &ConfigWrapper, params: &ThermistorParams) -> KResult<Rc<dyn TempSensor>> {
    let pullup = config.getfloat("pullup_resistor", 4700.0, Lim::new().above(0.0))?;
    let inline_resistor = config.getfloat("inline_resistor", 0.0, Lim::new().min(0.0))?;
    let thermistor = Thermistor::new(pullup, inline_resistor);
    match *params {
        ThermistorParams::Beta { t1, r1, beta } => thermistor.setup_coefficients_beta(t1, r1, beta)?,
        ThermistorParams::Points { t1, r1, t2, r2, t3, r3 } => {
            thermistor.setup_coefficients(t1, r1, t2, r2, t3, r3, config.get_name())?
        }
    }
    Ok(PrinterADCtoTemperature::new(config, Rc::new(thermistor))?)
}

// Custom defined thermistors from the config file
fn custom_thermistor(config: &ConfigWrapper) -> KResult<ThermistorParams> {
    let t1 = config.getfloat("temperature1", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
    let r1 = config.getfloat("resistance1", Dflt::Required, Lim::new().min(0.0))?;
    if let Some(beta) = config.getfloat_opt("beta", Lim::new().above(0.0))? {
        return Ok(ThermistorParams::Beta { t1, r1, beta });
    }
    let t2 = config.getfloat("temperature2", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
    let r2 = config.getfloat("resistance2", Dflt::Required, Lim::new().min(0.0))?;
    let t3 = config.getfloat("temperature3", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
    let r3 = config.getfloat("resistance3", Dflt::Required, Lim::new().min(0.0))?;
    let mut pts = [(t1, r1), (t2, r2), (t3, r3)];
    pts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let [(t1, r1), (t2, r2), (t3, r3)] = pts;
    Ok(ThermistorParams::Points { t1, r1, t2, r2, t3, r3 })
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let name = custom_name(config);
    let params = custom_thermistor(config)?;
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(&name, Rc::new(move |config: &ConfigWrapper| printer_thermistor(config, &params)));
    Ok(none_object())
}
