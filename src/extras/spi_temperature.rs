// MAX6675/MAX31855/MAX31856/MAX31865 temperature sensors (port of klippy/extras/spi_temperature.py)
//
// Ported so far: load_config (the sensor factory registration). The sensor
// class is not ported yet: configuring one fails with a config error.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::heaters::{load_heaters, unported_factory};
use crate::printer::{none_object, PObj};

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory("MAX6675", unported_factory("MAX6675"));
    pheaters.add_sensor_factory("MAX31855", unported_factory("MAX31855"));
    pheaters.add_sensor_factory("MAX31856", unported_factory("MAX31856"));
    pheaters.add_sensor_factory("MAX31865", unported_factory("MAX31865"));
    Ok(none_object())
}
