// Support fans that are controlled by gcode (port of klippy/extras/fan_generic.py)
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::fan::Fan;
use crate::extras::output_pin::{lookup_template_eval, PrinterTemplateEvaluator, TemplateCallback};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, PrinterObject};
use crate::pyval::PyVal;
use std::cell::RefCell;
use std::rc::Rc;

const CMD_SET_FAN_SPEED_HELP: &str = "Sets the speed of a fan";

pub struct PrinterFanGeneric {
    pub fan: Rc<Fan>,
    template_eval: Rc<PrinterTemplateEvaluator>,
    template_cb: RefCell<Option<TemplateCallback>>,
}

impl PrinterFanGeneric {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let fan = Fan::new(config, 0.0)?;
        let fan_name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        // Template handling
        let template_eval = lookup_template_eval(config)?;
        let f = Rc::new(PrinterFanGeneric { fan, template_eval, template_cb: RefCell::new(None) });
        let w = Rc::downgrade(&f);
        *f.template_cb.borrow_mut() = Some(Rc::new(move |text: &str| match w.upgrade() {
            Some(f) => f.template_update(text),
            None => Ok(()),
        }));
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let ff = f.clone();
        gcode.register_mux_command(
            "SET_FAN_SPEED",
            "FAN",
            Some(&fan_name),
            Rc::new(move |g| ff.cmd_set_fan_speed(g)),
            Some(CMD_SET_FAN_SPEED_HELP),
        )?;
        Ok(f)
    }

    fn template_update(&self, text: &str) -> KResult<()> {
        let value = match crate::pyfmt::py_float(text) {
            Some(v) => v,
            None => {
                logging::exception_tb(
                    "fan_generic template render error",
                    "ValueError",
                    &format!("could not convert string to float: {}", crate::pyfmt::py_str_repr(text)),
                );
                0.0
            }
        };
        self.fan.set_speed(value, None)
    }

    fn cmd_set_fan_speed(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let speed = gcmd.get_float_opt("SPEED", Lim::new().min(0.0))?;
        let template = gcmd.get_opt("TEMPLATE");
        if speed.is_none() == template.is_none() {
            return Err(gcmd.error("SET_FAN_SPEED must specify SPEED or TEMPLATE"));
        }
        // Check for template setting
        if template.is_some() {
            let cb = self.template_cb.borrow().clone().expect("template callback");
            return self.template_eval.set_template(gcmd, cb, None);
        }
        self.fan.set_speed_from_command(speed.unwrap())
    }
}

impl PrinterObject for PrinterFanGeneric {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.fan.get_status(eventtime))
    }
    fn fan(&self) -> Option<Rc<Fan>> {
        Some(self.fan.clone())
    }
    fn teardown(&self) {
        *self.template_cb.borrow_mut() = None;
        self.fan.teardown();
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterFanGeneric::new(config)?)
}
