// Support for reading acceleration data from an LIS3DH chip
// (port of klippy/extras/lis3dh.py)
//
// Copyright (C) 2024 Luke Vuksta <wulfstawulfsta@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::lis2dw::{LIS2DW, LIS3DH_TYPE};
use crate::printer::PObj;

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(LIS2DW::new(config, LIS3DH_TYPE)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(LIS2DW::new(config, LIS3DH_TYPE)?)
}
