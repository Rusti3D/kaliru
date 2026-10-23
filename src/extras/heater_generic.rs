// Support for a generic heater (port of klippy/extras/heater_generic.py)
//
// Copyright (C) 2019  John Jardine <john@gprime.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::heaters::load_heaters;
use crate::printer::PObj;

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let pheaters = load_heaters(config)?;
    Ok(pheaters.setup_heater(config, None)?)
}
