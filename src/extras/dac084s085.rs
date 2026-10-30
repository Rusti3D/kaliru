// SPI DAC DAC084S085 implementation
// (port of klippy/extras/dac084S085.py)
//
// Copyright (C) 2021  Lorenzo Franco <lorenzo.franco@lorenzing.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::printer::{PObj, PrinterObject};
use std::rc::Rc;

pub struct Dac084S085 {
    pub spi: Rc<McuSpi>,
}

impl Dac084S085 {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let spi = mcu_spi_from_config(config, 1, "enable_pin", 10000000, None, false)?;
        let d = Dac084S085 { spi };
        let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
        for (chan, name) in "ABCD".chars().enumerate() {
            let val = config.getfloat_opt(&format!("channel_{}", name), Lim::new().min(0.0).max(scale))?;
            if let Some(val) = val {
                d.set_register(chan as i64, (val * 255.0 / scale).trunc() as i64)?;
            }
        }
        Ok(Rc::new(d))
    }

    pub fn set_register(&self, chan: i64, value: i64) -> KResult<()> {
        let b1 = (chan << 6) | (1 << 4) | ((value >> 4) & 0x0F);
        let b2 = (value << 4) & 0xF0;
        self.spi.spi_send(&[b1 as u8, b2 as u8], 0, 0)
    }
}

impl PrinterObject for Dac084S085 {
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(Dac084S085::new(config)?)
}
