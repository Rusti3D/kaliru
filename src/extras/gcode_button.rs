// Support for executing gcode when a hardware button is pressed or released.
// (port of klippy/extras/gcode_button.py)
//
// Copyright (C) 2019 Alec Plumb <alec@etherwalker.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::extras::buttons::PrinterButtons;
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, PrinterObject};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const CMD_QUERY_BUTTON_HELP: &str = "Report on the state of a button";

pub struct GCodeButton {
    name: String,
    last_state: Cell<i64>,
    press_template: RefCell<Option<Rc<Template>>>,
    release_template: RefCell<Option<Rc<Template>>>,
    gcode: Rc<GCodeDispatch>,
}

impl GCodeButton {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split(' ').last().unwrap_or_default().to_string();
        let pin = config.get("pin", Dflt::Required)?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let b = Rc::new(GCodeButton {
            name: name.clone(),
            last_state: Cell::new(0),
            press_template: RefCell::new(None),
            release_template: RefCell::new(None),
            gcode: gcode.clone(),
        });
        let buttons = printer.load_as::<PrinterButtons>(config, "buttons")?;
        let w: Weak<GCodeButton> = Rc::downgrade(&b);
        let callback: crate::extras::buttons::ButtonCallback = Rc::new(move |eventtime, state| {
            if let Some(b) = w.upgrade() {
                b.button_callback(eventtime, state);
            }
            Ok(())
        });
        if config.get_opt("analog_range")?.is_none() {
            buttons.register_debounce_button(&pin, callback, config)?;
        } else {
            let range = config.getfloatlist("analog_range", Dflt::Required, ",", Some(2))?.unwrap_or_default();
            let pullup = config.getfloat("analog_pullup_resistor", 4700.0, Lim::new().above(0.0))?;
            buttons.register_debounce_adc_button(&pin, range[0], range[1], pullup, callback, config)?;
        }
        let gcode_macro = load_gcode_macro(config)?;
        *b.press_template.borrow_mut() = Some(gcode_macro.load_template(config, "press_gcode", None)?);
        *b.release_template.borrow_mut() = Some(gcode_macro.load_template(config, "release_gcode", Some(""))?);
        let w = Rc::downgrade(&b);
        gcode.register_mux_command(
            "QUERY_BUTTON",
            "BUTTON",
            Some(&name),
            Rc::new(move |g| match w.upgrade() {
                Some(b) => b.cmd_query_button(g),
                None => Ok(()),
            }),
            Some(CMD_QUERY_BUTTON_HELP),
        )?;
        Ok(b)
    }

    fn state(&self) -> &'static str {
        if self.last_state.get() != 0 {
            "PRESSED"
        } else {
            "RELEASED"
        }
    }

    fn cmd_query_button(&self, gcmd: &GCodeCommand) -> KResult<()> {
        gcmd.respond_info(&format!("{}: {}", self.name, self.state()), true);
        Ok(())
    }

    fn button_callback(&self, _eventtime: f64, state: i64) {
        self.last_state.set(state);
        let template = if state != 0 { self.press_template.borrow().clone() } else { self.release_template.borrow().clone() };
        let Some(template) = template else { return };
        let r = (|| -> KResult<()> {
            let commands = template.render(None)?;
            if commands.trim().is_empty() {
                return Ok(());
            }
            self.gcode.run_script(&commands)
        })();
        if let Err(e) = r {
            logging::exception_err("Script running error", &e);
        }
    }
}

impl PrinterObject for GCodeButton {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("state".to_string(), PyVal::Str(self.state().into()));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(GCodeButton::new(config)?)
}
