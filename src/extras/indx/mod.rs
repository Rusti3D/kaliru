// Bondtech INDX toolboard support (port of klippy/extras/indx/__init__.py
// and compat.py)
pub mod calibration;
pub mod heater;
pub mod heatsink_fan;
pub mod sensors;
pub mod toolboard;

use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::heaters::PrinterHeaters;
use crate::mcu::{AdcCallback, AdcPin, Mcu};
use crate::msgproto::Params;
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::py_float_repr;
use std::rc::Rc;

/// a python value of a compat.ConfigWrapper contents dict, as its str()
pub(crate) enum CVal {
    S(String),
    F(f64),
    I(i64),
}

impl CVal {
    fn to_str(&self) -> String {
        match self {
            CVal::S(s) => s.clone(),
            CVal::F(f) => py_float_repr(*f),
            CVal::I(i) => i.to_string(),
        }
    }
}

/// compat.ConfigWrapper(printer, name, contents): a config section of just
/// `contents` (its fallback is unreachable: has_option() checks contents)
pub(crate) fn compat_config(config: &ConfigWrapper, name: &str, contents: &[(&str, CVal)]) -> ConfigWrapper {
    let contents: Vec<(&str, String)> = contents.iter().map(|(k, v)| (*k, v.to_str())).collect();
    config.synthetic(name, &contents)
}

/// compat.register_response(mcu, callback, msgformat): registers by the
/// message name; the returned handle unregisters it
pub(crate) fn register_response(mcu: &Mcu, callback: Rc<dyn Fn(&Params) -> KResult<()>>, msgformat: &str) -> ResponseHandle {
    let name = msgformat.split_whitespace().next().unwrap_or("").to_string();
    mcu.register_response(Some(callback), &name, None);
    ResponseHandle { name }
}

pub(crate) struct ResponseHandle {
    name: String,
}

impl ResponseHandle {
    pub(crate) fn unregister(&self, mcu: &Mcu) {
        mcu.register_response(None, &self.name, None);
    }
}

/// compat.register_adc_callback(adc, callback)
pub(crate) fn register_adc_callback(adc: &Rc<dyn AdcPin>, callback: AdcCallback) {
    // Old klipper and Kalico ADC callback style
    adc.setup_adc_callback(0.3, callback);
    adc.setup_adc_sample(0.001, 8, 0.0, 1.0, 0);
}

/// the IndxFactories object (registers the "indx" temperature sensor)
pub struct IndxFactories;

impl PrinterObject for IndxFactories {
}

pub fn register_factories(config: &ConfigWrapper) -> KResult<()> {
    let printer = config.get_printer();
    if printer.lookup_object_opt("indx_factories").is_some() {
        return Ok(());
    }
    let pheaters = printer.load_as::<PrinterHeaters>(config, "heaters")?;
    pheaters.add_sensor_factory("indx", Rc::new(sensors::indx_temp_sensor_factory));
    printer.add_object("indx_factories", Rc::new(IndxFactories))
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    register_factories(config)?;
    Ok(toolboard::IndxToolboard::new(config)?)
}
