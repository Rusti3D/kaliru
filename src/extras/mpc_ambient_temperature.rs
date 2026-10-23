// MPC ambient temperature sensor (port of klippy/extras/mpc_ambient_temperature.py)
//
// The sensor is mpc_block_temperature's wrapper reporting the ambient state.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::heaters::{load_heaters, TempSensor};
use crate::extras::mpc_block_temperature::{Kind, MpcTempWrapper};
use crate::printer::{none_object, PObj};
use std::rc::Rc;

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(
        "mpc_ambient_temperature",
        Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(MpcTempWrapper::new(config, Kind::Ambient)?) }),
    );
    Ok(none_object())
}
