// HTU21D(F)/Sht21/Si7013/Si7020/Si7021 temperature sensors (port of klippy/extras/htu21d.py)
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
    pheaters.add_sensor_factory("SI7013", unported_factory("SI7013"));
    pheaters.add_sensor_factory("SI7020", unported_factory("SI7020"));
    pheaters.add_sensor_factory("SI7021", unported_factory("SI7021"));
    pheaters.add_sensor_factory("SHT21", unported_factory("SHT21"));
    pheaters.add_sensor_factory("HTU21D", unported_factory("HTU21D"));
    Ok(none_object())
}
