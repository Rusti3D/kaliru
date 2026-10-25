// INDX temperature sensors (port of klippy/extras/indx/sensors.py)
use super::heater::NozzleTemperature;
use super::toolboard::IndxToolboard;
use super::{compat_config, CVal};
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::heaters::{TempCallback, TempSensor};
use crate::extras::thermistor::{custom_thermistor, printer_thermistor};
use std::cell::RefCell;
use std::rc::Rc;

pub const TEMP_REPORT_TIME: f64 = 0.3;

pub fn indx_temp_sensor_factory(config: &ConfigWrapper) -> KResult<Rc<dyn TempSensor>> {
    let kinds = ["heater", "board", "ldc_coil", "bracket", "sensor", "check_model", "check_model_delta"];
    let choices: Vec<(&str, &str)> = kinds.iter().map(|k| (*k, *k)).collect();
    let kind = config.getchoice("indx_sensor", &choices, "heater")?;
    let toolboard_name = config.get_opt("toolboard")?;
    let name = match &toolboard_name {
        None => "indx".to_string(),
        Some(n) => format!("indx {}", n),
    };
    let toolboard = config.get_printer().lookup::<IndxToolboard>(&name)?;

    Ok(match kind {
        "heater" => IndxHeaterTemperatureSensor::new(&toolboard, Source::Heater)?,
        "board" => Rc::new(IndxTempSensorWrapper { toolboard, sensor: "board" }),
        "ldc_coil" => ldc_coil_sensor(&toolboard, config)?,
        "bracket" => Rc::new(IndxTempSensorWrapper { toolboard, sensor: "bracket" }),
        "sensor" => IndxHeaterTemperatureSensor::new(&toolboard, Source::Sensor)?,
        "check_model" => IndxHeaterTemperatureSensor::new(&toolboard, Source::Model)?,
        _ => IndxHeaterTemperatureSensor::new(&toolboard, Source::ModelDelta)?,
    })
}

#[derive(Clone, Copy)]
enum Source {
    Heater,
    Sensor,
    Model,
    ModelDelta,
}

pub struct IndxHeaterTemperatureSensor {
    toolboard: Rc<IndxToolboard>,
    source: Source,
    temperature_callback: RefCell<Option<TempCallback>>,
}

impl IndxHeaterTemperatureSensor {
    fn new(toolboard: &Rc<IndxToolboard>, source: Source) -> KResult<Rc<dyn TempSensor>> {
        let s = Rc::new(IndxHeaterTemperatureSensor { toolboard: toolboard.clone(), source, temperature_callback: RefCell::new(None) });
        let w = Rc::downgrade(&s);
        toolboard.heater().add_temperature_callback(Rc::new(move |reading| match w.upgrade() {
            Some(s) => s.handle_reading(reading),
            None => Ok(()),
        }));
        Ok(s)
    }

    fn handle_reading(&self, reading: &NozzleTemperature) -> KResult<()> {
        let value = match self.source {
            Source::Heater => reading.nozzle_temperature,
            Source::Sensor => reading.sensor_temperature,
            Source::Model => self.toolboard.heater().model_temperature(),
            Source::ModelDelta => self.toolboard.heater().model_temperature() - reading.nozzle_temperature,
        };
        // (the default callback is lambda time, value: 0)
        let cb = self.temperature_callback.borrow().clone();
        match cb {
            Some(cb) => cb(reading.time, value),
            None => Ok(()),
        }
    }
}

impl TempSensor for IndxHeaterTemperatureSensor {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) -> KResult<()> {
        Ok(())
    }
    fn setup_callback(&self, cb: TempCallback) {
        *self.temperature_callback.borrow_mut() = Some(cb);
    }
    fn get_report_time_delta(&self) -> f64 {
        TEMP_REPORT_TIME
    }
}

fn ldc_coil_sensor(toolboard: &Rc<IndxToolboard>, config: &ConfigWrapper) -> KResult<Rc<dyn TempSensor>> {
    let config_thermistor = compat_config(
        config,
        config.get_name(),
        &[
            // 10K Murata NCU15XH103J6SRC
            ("beta", CVal::F(3425.0)),
            ("temperature1", CVal::F(25.0)),
            ("resistance1", CVal::F(10000.0)),
        ],
    );
    let config_sensor = compat_config(
        config,
        config.get_name(),
        &[("sensor_pin", CVal::S(format!("{}:temp_ldc", toolboard.mcu.get_name()))), ("pullup_resistor", CVal::I(3900))],
    );
    let params = custom_thermistor(&config_thermistor)?;
    printer_thermistor(&config_sensor, &params)
}

pub struct IndxTempSensorWrapper {
    toolboard: Rc<IndxToolboard>,
    sensor: &'static str,
}

impl TempSensor for IndxTempSensorWrapper {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) -> KResult<()> {
        Ok(())
    }
    fn setup_callback(&self, cb: TempCallback) {
        self.toolboard.heater().aux_temp_sensor(self.sensor).add_callback(cb);
    }
    // ponytail: python's IndxTempSensorWrapper has no get_report_time_delta
    // (a heater using it fails with AttributeError); temperature_sensor
    // sections never ask for it
    fn get_report_time_delta(&self) -> f64 {
        TEMP_REPORT_TIME
    }
}
