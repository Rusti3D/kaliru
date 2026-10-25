// SAMD Sercom configuration (port of klippy/extras/samd_sercom.py)
//
// Copyright (C) 2019  Florian Heilmann <Florian.Heilmann@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::extras::bus::{pin_mcu, same_rc};
use crate::pins::{PinsObject, SharedPinParams};
use crate::printer::{PObj, PrinterObject};
use std::rc::Rc;

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();

    let sercom = config.get("sercom", Dflt::Required)?;
    let tx_pin = config.get("tx_pin", Dflt::Required)?;
    let rx_pin = config.get_opt("rx_pin")?;
    let clk_pin = config.get("clk_pin", Dflt::Required)?;

    let ppins = printer.lookup::<PinsObject>("pins")?;
    let lookup = |pin: &str| -> KResult<SharedPinParams> {
        ppins.0.borrow_mut().lookup_pin(pin, false, false, None).map_err(|e| KError::Pins(e.0))
    };
    let tx_pin_params = lookup(&tx_pin)?;
    let mcu = pin_mcu(config, &tx_pin_params)?;
    let pin = tx_pin_params.borrow().pin.clone();
    mcu.add_config_cmd(&format!("set_sercom_pin bus={} sercom_pin_type=tx pin={}", sercom, pin), false, false);

    let same_mcu_error = || KError::Pins(format!("{}: SERCOM pins must be on same mcu", config.get_name()));
    let clk_pin_params = lookup(&clk_pin)?;
    if !same_rc(&clk_pin_params.borrow().chip, &mcu) {
        return Err(same_mcu_error());
    }
    let pin = clk_pin_params.borrow().pin.clone();
    mcu.add_config_cmd(&format!("set_sercom_pin bus={} sercom_pin_type=clk pin={}", sercom, pin), false, false);

    if let Some(rx_pin) = rx_pin.filter(|p| !p.is_empty()) {
        let rx_pin_params = lookup(&rx_pin)?;
        if !same_rc(&rx_pin_params.borrow().chip, &mcu) {
            return Err(same_mcu_error());
        }
        let pin = rx_pin_params.borrow().pin.clone();
        mcu.add_config_cmd(&format!("set_sercom_pin bus={} sercom_pin_type=rx pin={}", sercom, pin), false, false);
    }
    Ok(Rc::new(SamdSERCOM))
}

/// the SamdSERCOM object (it only keeps its config values)
pub struct SamdSERCOM;

impl PrinterObject for SamdSERCOM {
}
