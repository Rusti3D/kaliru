// Tool to disable config checks for duplicate pins
// (port of klippy/extras/duplicate_pin_override.py)
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::pins::PinsObject;
use crate::printer::{PObj, PrinterObject};
use std::any::Any;
use std::rc::Rc;

pub struct PrinterDupPinOverride;

impl PrinterObject for PrinterDupPinOverride {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let ppins = printer.lookup::<PinsObject>("pins")?;
    for pin_desc in config.getlist("pins", Dflt::Required, ",", None)?.unwrap_or_default() {
        ppins.0.borrow_mut().allow_multi_use_pin(&pin_desc).map_err(|e| KError::Pins(e.0))?;
    }
    Ok(Rc::new(PrinterDupPinOverride))
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    load_config(config)
}
