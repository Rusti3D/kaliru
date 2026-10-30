// MCP4018 digipot support (port of klippy/extras/mcp4018.py)
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::extras::ad5206::byte;
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, PrinterObject};
use std::rc::Rc;

const CMD_SET_DIGIPOT_HELP: &str = "Set digipot value";

pub struct Mcp4018 {
    pub i2c: Rc<McuI2c>,
    pub scale: f64,
    pub start_value: f64,
    pub name: String,
}

impl Mcp4018 {
    pub fn set_dac(&self, value: f64) -> KResult<()> {
        let val = (value * 127.0 / self.scale + 0.5) as i64;
        self.i2c.i2c_write(&[byte(val)?], 0, 0)
    }

    pub fn cmd_set_digipot(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let wiper = gcmd.get_float("WIPER", None, Lim::new().min(0.0).max(self.scale))?;
        self.set_dac(wiper)?;
        gcmd.respond_info(&format!("New value for DIGIPOT = {}, wiper = {:.2}", self.name, wiper), true);
        Ok(())
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let i2c = mcu_i2c_from_config(config, Some(0x2F), 100000)?;
    let scale = config.getfloat("scale", 1.0, Lim::new().above(0.0))?;
    let start_value = config.getfloat("wiper", Dflt::Required, Lim::new().min(0.0).max(scale))?;
    let name = config.get_name().split_whitespace().nth(1).unwrap_or_default().to_string();
    let m = Rc::new(Mcp4018 { i2c, scale, start_value, name: name.clone() });
    let mm = m.clone();
    crate::events::KlippyConnect::on(&printer, move || mm.set_dac(mm.start_value));
    // Register commands
    let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
    let mm = m.clone();
    gcode.register_mux_command("SET_DIGIPOT", "DIGIPOT", Some(&name), Rc::new(move |g| mm.cmd_set_digipot(g)), Some(CMD_SET_DIGIPOT_HELP))?;
    Ok(m)
}

impl PrinterObject for Mcp4018 {
}
