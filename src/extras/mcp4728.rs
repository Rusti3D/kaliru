// MCP4728 dac code (port of klippy/extras/mcp4728.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::printer::{PObj, PrinterObject};
use std::any::Any;
use std::rc::Rc;

pub struct Mcp4728 {
    _i2c: Rc<McuI2c>,
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let i2c = mcu_i2c_from_config(config, Some(0x60), 100000)?;
    let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
    // Configure registers
    for (i, name) in "abcd".chars().enumerate() {
        let val = config.getfloat_opt(&format!("channel_{}", name), Lim::new().min(0.0).max(scale))?;
        if let Some(val) = val {
            // set_dac(dac, value)
            let value = (val * 4095.0 / scale + 0.5) as i64;
            i2c.i2c_write(&[0x40 | ((i as u8) << 1), (((value >> 8) & 0x0F) | 0x80) as u8, (value & 0xFF) as u8], 0, 0)?;
        }
    }
    Ok(Rc::new(Mcp4728 { _i2c: i2c }))
}

impl PrinterObject for Mcp4728 {
    fn as_any(&self) -> &dyn Any {
        self
    }
}
