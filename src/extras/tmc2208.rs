// TMC2208 UART communication and configuration (port of klippy/extras/tmc2208.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::tmc::{fields_from_table, tmc_stealthchop_helper, FieldHelper, TMCCommandHelper, TmcDriver, TmcMcuRef};
use crate::extras::tmc2130::{self, TMC2130CurrentHelper};
use crate::extras::tmc_uart::McuTmcUart;
use crate::printer::PObj;
use std::rc::Rc;

pub use crate::extras::tmc_tables::tmc2208::*;

pub fn field_formatters(field: &str, v: i64) -> Option<String> {
    let flag = |s: &str| Some(if v != 0 { s.to_string() } else { String::new() });
    match field {
        "sel_a" => Some(format!("{}({})", v, ["TMC222x", "TMC220x"][v as usize])),
        "s2vsa" => flag("1(ShortToSupply_A!)"),
        "s2vsb" => flag("1(ShortToSupply_B!)"),
        _ => tmc2130::field_formatters(field, v),
    }
}

/// the GCONF..TPOWERDOWN config fields set by both TMC2208 and TMC2209
pub fn set_uart_config_fields(fields: &FieldHelper, config: &ConfigWrapper, coolconf: bool) -> KResult<()> {
    // GCONF
    fields.set_config_field(config, "multistep_filt", true)?;
    // CHOPCONF
    fields.set_config_field(config, "toff", 3)?;
    fields.set_config_field(config, "hstrt", 5)?;
    fields.set_config_field(config, "hend", 0)?;
    fields.set_config_field(config, "tbl", 2)?;
    if coolconf {
        // COOLCONF
        for f in ["semin", "seup", "semax", "sedn", "seimin"] {
            fields.set_config_field(config, f, 0)?;
        }
    }
    // IHOLDIRUN
    fields.set_config_field(config, "iholddelay", 8)?;
    // PWMCONF
    fields.set_config_field(config, "pwm_ofs", 36)?;
    fields.set_config_field(config, "pwm_grad", 14)?;
    fields.set_config_field(config, "pwm_freq", 1)?;
    fields.set_config_field(config, "pwm_autoscale", true)?;
    fields.set_config_field(config, "pwm_autograd", true)?;
    fields.set_config_field(config, "freewheel", 0)?;
    fields.set_config_field(config, "pwm_reg", 8)?;
    fields.set_config_field(config, "pwm_lim", 12)?;
    // TPOWERDOWN
    fields.set_config_field(config, "tpowerdown", 20)?;
    Ok(())
}

// ----------------------------------------------------------------------
// TMC2208 printer object
// ----------------------------------------------------------------------

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    // Setup mcu communication
    let fields = Rc::new(FieldHelper::new(fields_from_table(FIELDS), SIGNED_FIELDS, Some(field_formatters)));
    let mcu_tmc: TmcMcuRef = McuTmcUart::new(config, REGISTERS, fields.clone(), 0, TMC_FREQUENCY)?;
    fields.set_field("pdn_disable", 1)?;
    // Register commands
    let current_helper = Rc::new(TMC2130CurrentHelper::new(config, &mcu_tmc)?);
    let cmdhelper = TMCCommandHelper::new(config, &mcu_tmc, current_helper, None, None)?;
    let f = fields.clone();
    cmdhelper.setup_register_dump(
        READ_REGISTERS,
        Some(Rc::new(move |reg_name: &str, val: i64| {
            if reg_name == "IOIN" {
                let drv_type = f.get_field_of("sel_a", val);
                let reg_name = if drv_type != 0 { "IOIN@TMC220x" } else { "IOIN@TMC222x" };
                return (reg_name.to_string(), val);
            }
            (reg_name.to_string(), val)
        })),
    )?;
    // Setup basic register values
    fields.set_field("mstep_reg_select", 1)?;
    tmc_stealthchop_helper(config, &mcu_tmc)?;
    // Allow other registers to be set from the config
    set_uart_config_fields(&fields, config, false)?;
    Ok(Rc::new(TmcDriver { fields, mcu_tmc, cmdhelper }))
}
