// AD5206 digipot code (port of klippy/extras/ad5206.py)
//
// Copyright (C) 2017,2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::printer::{PObj, PrinterObject};
use std::rc::Rc;

pub struct Ad5206 {
    pub spi: Rc<McuSpi>,
}

/// a byte of a command buffer (bytearray() rejects values above 255)
pub(crate) fn byte(v: i64) -> KResult<u8> {
    u8::try_from(v).map_err(|_| KError::Other("byte must be in range(0, 256)".into()))
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let spi = mcu_spi_from_config(config, 0, "enable_pin", 25000000, None, false)?;
    let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
    for i in 0..6 {
        let val = config.getfloat_opt(&format!("channel_{}", i + 1), Lim::new().min(0.0).max(scale))?;
        if let Some(val) = val {
            // set_register(reg, value)
            spi.spi_send(&[i as u8, byte((val * 256.0 / scale + 0.5) as i64)?], 0, 0)?;
        }
    }
    Ok(Rc::new(Ad5206 { spi }))
}

impl PrinterObject for Ad5206 {
}
