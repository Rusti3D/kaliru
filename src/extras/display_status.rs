// Module to handle M73 and M117 display status commands
// (port of klippy/extras/display_status.py)
//
// Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const M73_TIMEOUT: f64 = 5.0;

const CMD_SET_DISPLAY_TEXT_HELP: &str = "Set or clear the display message";

pub struct DisplayStatus {
    printer: Rc<Printer>,
    expire_progress: Cell<f64>,
    progress: Cell<Option<f64>>,
    message: RefCell<Option<String>>,
}

impl DisplayStatus {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let ds = Rc::new(DisplayStatus { printer: printer.clone(), expire_progress: Cell::new(0.0), progress: Cell::new(None), message: RefCell::new(None) });
        // Register commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let d = ds.clone();
        gcode.register_command("M73", Some(Rc::new(move |g| d.cmd_m73(g))), false, None)?;
        let d = ds.clone();
        gcode.register_command(
            "M117",
            Some(Rc::new(move |g: &GCodeCommand| {
                let msg = g.get_raw_command_parameters();
                *d.message.borrow_mut() = if msg.is_empty() { None } else { Some(msg) };
                Ok(())
            })),
            false,
            None,
        )?;
        let d = ds.clone();
        gcode.register_command(
            "SET_DISPLAY_TEXT",
            Some(Rc::new(move |g: &GCodeCommand| {
                *d.message.borrow_mut() = g.get_opt("MSG");
                Ok(())
            })),
            false,
            Some(CMD_SET_DISPLAY_TEXT_HELP),
        )?;
        Ok(ds)
    }

    fn cmd_m73(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if let Some(progress) = gcmd.get_float_opt("P", Lim::new())? {
            let progress = progress / 100.0;
            self.progress.set(Some(py_min(1.0, py_max(0.0, progress))));
            let curtime = self.printer.get_reactor().monotonic();
            self.expire_progress.set(curtime + M73_TIMEOUT);
        }
        Ok(())
    }

    fn status(&self, eventtime: f64) -> KResult<PyVal> {
        let mut progress = self.progress.get();
        if progress.is_some() && eventtime > self.expire_progress.get() {
            let idle_timeout = self.printer.lookup_object("idle_timeout")?;
            let state = match idle_timeout.get_status(eventtime) {
                Some(PyVal::Dict(d)) => d.get("state").map(|v| v.str()).unwrap_or_default(),
                _ => String::new(),
            };
            if state != "Printing" {
                self.progress.set(None);
                progress = None;
            }
        }
        let progress = match progress {
            Some(p) => p,
            None => match self.printer.lookup_object_opt("virtual_sdcard") {
                Some(sd) => match sd.get_status(eventtime) {
                    Some(PyVal::Dict(d)) => d.get("progress").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    _ => 0.0,
                },
                None => 0.0,
            },
        };
        let mut d = IndexMap::new();
        d.insert("progress".to_string(), PyVal::Float(progress));
        d.insert("message".to_string(), self.message.borrow().clone().map(PyVal::Str).unwrap_or(PyVal::None));
        Ok(PyVal::Dict(d))
    }
}

impl PrinterObject for DisplayStatus {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        self.status(eventtime).ok()
    }
    fn try_get_status(&self, eventtime: f64) -> KResult<Option<PyVal>> {
        self.status(eventtime).map(Some)
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(DisplayStatus::new(config)?)
}
