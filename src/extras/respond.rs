// Add 'RESPOND' and 'M118' commands for sending messages to the host
// (port of klippy/extras/respond.py)
//
// Copyright (C) 2018  Alec Plumb <alec@etherwalker.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, PrinterObject};
use std::any::Any;
use std::rc::Rc;

const RESPOND_TYPES: &[(&str, &str)] = &[("echo", "echo:"), ("command", "//"), ("error", "!!")];

const RESPOND_TYPES_NO_SPACE: &[(&str, &str)] = &[("echo_no_space", "echo:")];

const CMD_RESPOND_HELP: &str = "Echo the message prepended with a prefix";

pub struct HostResponder {
    default_prefix: String,
}

impl HostResponder {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let default_prefix = config.getchoice("default_type", RESPOND_TYPES, "echo")?;
        let default_prefix = config.get("default_prefix", default_prefix)?;
        let hr = Rc::new(HostResponder { default_prefix });

        if !config.getboolean("enable_respond", true)? {
            return Ok(hr);
        }

        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let h = hr.clone();
        gcode.register_command("M118", Some(Rc::new(move |g| h.cmd_m118(g))), true, None)?;
        let h = hr.clone();
        gcode.register_command("RESPOND", Some(Rc::new(move |g| h.cmd_respond(g))), true, Some(CMD_RESPOND_HELP))?;
        Ok(hr)
    }

    fn cmd_m118(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let msg = gcmd.get_raw_command_parameters();
        gcmd.respond_raw(&format!("{} {}", self.default_prefix, msg));
        Ok(())
    }

    fn cmd_respond(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut no_space = false;
        let respond_type = gcmd.get_opt("TYPE");
        let mut prefix = self.default_prefix.clone();
        if let Some(respond_type) = respond_type {
            let respond_type = respond_type.to_lowercase();
            if let Some((_, p)) = RESPOND_TYPES.iter().find(|(k, _)| *k == respond_type) {
                prefix = p.to_string();
            } else if let Some((_, p)) = RESPOND_TYPES_NO_SPACE.iter().find(|(k, _)| *k == respond_type) {
                prefix = p.to_string();
                no_space = true;
            } else {
                return Err(gcmd.error(format!(
                    "RESPOND TYPE '{}' is invalid. Must be one of 'echo', 'command', or 'error'",
                    respond_type
                )));
            }
        }
        let prefix = gcmd.get_or("PREFIX", &prefix);
        let msg = gcmd.get_or("MSG", "");
        if no_space {
            gcmd.respond_raw(&format!("{}{}", prefix, msg));
        } else {
            gcmd.respond_raw(&format!("{} {}", prefix, msg));
        }
        Ok(())
    }
}

impl PrinterObject for HostResponder {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(HostResponder::new(config)?)
}
