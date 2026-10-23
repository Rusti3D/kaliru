// Set the state of a list of digital output pins
// (port of klippy/extras/static_digital_output.py)
//
// Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::mcu::get_printer_mcu;
use crate::pins::PinsObject;
use crate::printer::{PObj, PrinterObject};
use std::any::Any;
use std::rc::Rc;

pub struct PrinterStaticDigitalOut;

impl PrinterObject for PrinterStaticDigitalOut {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let pin_list = config.getlist("pins", Dflt::Required, ",", None)?.unwrap_or_default();
    for pin_desc in pin_list {
        let pin_params = ppins.0.borrow_mut().lookup_pin(&pin_desc, true, false, None).map_err(|e| KError::Pins(e.0))?;
        let (chip_name, pin, invert) = {
            let p = pin_params.borrow();
            (p.chip_name.clone(), p.pin.clone(), p.invert)
        };
        let mcu = get_printer_mcu(&printer, &chip_name)?;
        mcu.add_config_cmd(&format!("set_digital_out pin={} value={}", pin, (invert == 0) as i64), false, false);
    }
    Ok(Rc::new(PrinterStaticDigitalOut))
}
