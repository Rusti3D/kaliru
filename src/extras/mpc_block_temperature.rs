// MPC block temperature sensor (port of klippy/extras/mpc_block_temperature.py)
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
    pheaters.add_sensor_factory("mpc_block_temperature", unported_factory("mpc_block_temperature"));
    Ok(none_object())
}
