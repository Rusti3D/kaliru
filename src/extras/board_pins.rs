// Support for custom board pin aliases (port of klippy/extras/board_pins.py)
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, ListParser};
use crate::error::{KError, KResult};
use crate::pins::PinsObject;
use crate::printer::{PObj, PrinterObject};
use crate::pyval::PyVal;
use std::rc::Rc;

pub struct PrinterBoardAliases;

impl PrinterBoardAliases {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let ppins = config.get_printer().lookup::<PinsObject>("pins")?;
        let mcu_names = config
            .getlist("mcu", PyVal::Tuple(vec![PyVal::from("mcu")]), ",", None)?
            .unwrap_or_default();
        let mut pins = ppins.0.borrow_mut();
        for n in &mcu_names {
            pins.get_pin_resolver(n).map_err(|e| KError::Pins(e.0))?;
        }
        let mut options = vec!["aliases".to_string()];
        options.extend(config.get_prefix_options("aliases_")?);
        for opt in options {
            let aliases = config.getlists(&opt, Dflt::Required, &["=", ","], Some(2), ListParser::Str, true)?;
            for pair in aliases.as_ref().and_then(|v| v.items()).unwrap_or(&[]) {
                let items = pair.items().unwrap_or(&[]);
                let (name, value) = (items[0].str(), items[1].str());
                for n in &mcu_names {
                    let pin_resolver = pins.get_pin_resolver(n).map_err(|e| KError::Pins(e.0))?;
                    let r = if value.starts_with('<') && value.ends_with('>') {
                        pin_resolver.reserve_pin(&name, &value)
                    } else {
                        pin_resolver.alias_pin(&name, &value)
                    };
                    r.map_err(|e| KError::Pins(e.0))?;
                }
            }
        }
        Ok(Rc::new(PrinterBoardAliases))
    }
}

impl PrinterObject for PrinterBoardAliases {
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterBoardAliases::new(config)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterBoardAliases::new(config)?)
}
