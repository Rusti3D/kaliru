// AHT10/AHT1X/AHT2X/AHT3X temperature sensors (port of klippy/extras/aht10.py)
//
// Ported so far: load_config (the sensor factory registration). The sensor
// class is not ported yet: configuring one fails with a config error.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::heaters::{lookup_heaters, unported_factory};
use crate::printer::{none_object, PObj};

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheaters = lookup_heaters(config)?;
    pheaters.add_sensor_factory("AHT10", unported_factory("AHT10"));
    pheaters.add_sensor_factory("AHT1X", unported_factory("AHT1X"));
    pheaters.add_sensor_factory("AHT2X", unported_factory("AHT2X"));
    pheaters.add_sensor_factory("AHT3X", unported_factory("AHT3X"));
    Ok(none_object())
}
