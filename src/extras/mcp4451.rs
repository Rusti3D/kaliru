// MCP4451 digipot code (port of klippy/extras/mcp4451.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::ad5206::byte;
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::printer::{PObj, PrinterObject};
use std::rc::Rc;

const WIPER_REGISTERS: [i64; 4] = [0x00, 0x01, 0x06, 0x07];

pub struct Mcp4451 {
    _i2c: Rc<McuI2c>,
}

fn set_register(i2c: &McuI2c, reg: i64, value: i64) -> KResult<()> {
    i2c.i2c_write(&[byte((reg << 4) | ((value >> 8) & 0x03))?, byte(value)?], 0, 0)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let i2c = mcu_i2c_from_config(config, None, 100000)?;
    let i2c_addr = i2c.get_i2c_address();
    if !(44..=47).contains(&i2c_addr) {
        return Err(KError::config("mcp4451 address must be between 44 and 47"));
    }
    let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
    // Configure registers
    set_register(&i2c, 0x04, 0xFF)?;
    set_register(&i2c, 0x0A, 0xFF)?;
    for (i, reg) in WIPER_REGISTERS.iter().enumerate() {
        let val = config.getfloat_opt(&format!("wiper_{}", i), Lim::new().min(0.0).max(scale))?;
        if let Some(val) = val {
            set_register(&i2c, *reg, (val * 255.0 / scale + 0.5) as i64)?;
        }
    }
    Ok(Rc::new(Mcp4451 { _i2c: i2c }))
}

impl PrinterObject for Mcp4451 {
}
