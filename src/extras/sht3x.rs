// SHT3X temperature sensor (port of klippy/extras/sht3x.py)
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
    pheaters.add_sensor_factory("SHT3X", unported_factory("SHT3X"));
    Ok(none_object())
}
