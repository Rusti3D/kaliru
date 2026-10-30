// TMC2209 configuration (port of klippy/extras/tmc2209.py)
//
// Copyright (C) 2019  Stephan Oelze <stephan.oelze@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::tmc::{
    fields_from_table, tmc_stealthchop_helper, tmc_vcoolthrs_helper, FieldHelper, TMCCommandHelper, TMCVirtualPinHelper, TmcDriver,
    TmcMcuRef,
};
use crate::extras::tmc2130::TMC2130CurrentHelper;
use crate::extras::tmc2208;
use crate::extras::tmc_uart::McuTmcUart;
use crate::printer::PObj;
use std::rc::Rc;

pub use crate::extras::tmc_tables::tmc2209::*;

// ----------------------------------------------------------------------
// TMC2209 printer object
// ----------------------------------------------------------------------

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    // Setup mcu communication
    let fields = Rc::new(FieldHelper::new(fields_from_table(FIELDS), SIGNED_FIELDS, Some(tmc2208::field_formatters)));
    let mcu_tmc: TmcMcuRef = McuTmcUart::new(config, REGISTERS, fields.clone(), 3, TMC_FREQUENCY)?;
    // Setup fields for UART
    fields.set_field("pdn_disable", 1)?;
    fields.set_field("senddelay", 2)?; // Avoid tx errors on shared uart
    // Allow virtual pins to be created
    TMCVirtualPinHelper::new(config, &mcu_tmc)?;
    // Register commands
    let current_helper = Rc::new(TMC2130CurrentHelper::new(config, &mcu_tmc)?);
    let cmdhelper = TMCCommandHelper::new(config, &mcu_tmc, current_helper, None, None)?;
    cmdhelper.setup_register_dump(READ_REGISTERS, None)?;
    // Setup basic register values
    fields.set_field("mstep_reg_select", 1)?;
    tmc_stealthchop_helper(config, &mcu_tmc)?;
    tmc_vcoolthrs_helper(config, &mcu_tmc)?;
    // Allow other registers to be set from the config
    tmc2208::set_uart_config_fields(&fields, config, true)?;
    // SGTHRS
    fields.set_config_field(config, "sgthrs", 0)?;
    Ok(Rc::new(TmcDriver { fields, mcu_tmc, cmdhelper, class: "TMC2209" }))
}
