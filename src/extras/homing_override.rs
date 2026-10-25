// Run user defined actions in place of a normal G28 homing command
// (port of klippy/extras/homing_override.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::gcode::{GCodeCommand, GCodeDispatch, Handler};
use crate::jinja::value::Value;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub struct HomingOverride {
    printer: Rc<Printer>,
    start_pos: [Option<f64>; 3],
    axes: String,
    template: Rc<Template>,
    in_script: Cell<bool>,
    prev_g28: RefCell<Option<Handler>>,
}

impl HomingOverride {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mut start_pos = [None; 3];
        for (i, a) in ["x", "y", "z"].iter().enumerate() {
            start_pos[i] = config.getfloat_opt(&format!("set_position_{}", a), Lim::new())?;
        }
        let axes = config.get("axes", "XYZ")?.to_uppercase();
        let gcode_macro = load_gcode_macro(config)?;
        let template = gcode_macro.load_template(config, "gcode", None)?;
        printer.load_object(config, "homing", true)?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let prev_g28 = gcode.register_command("G28", None, false, None)?;
        let ho = Rc::new(HomingOverride {
            printer: printer.clone(),
            start_pos,
            axes,
            template,
            in_script: Cell::new(false),
            prev_g28: RefCell::new(prev_g28),
        });
        let h = ho.clone();
        gcode.register_command("G28", Some(Rc::new(move |g| h.cmd_g28(g))), false, None)?;
        Ok(ho)
    }

    fn prev_g28(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let prev = self.prev_g28.borrow().clone();
        match prev {
            Some(f) => f(gcmd),
            None => Err(KError::Other("'NoneType' object is not callable".into())),
        }
    }

    fn cmd_g28(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.in_script.get() {
            // Was called recursively - invoke the real G28 command
            return self.prev_g28(gcmd);
        }

        // if no axis is given as parameter we assume the override
        let no_axis = !["X", "Y", "Z"].iter().any(|a| gcmd.get_opt(a).is_some());
        let override_ = if no_axis {
            true
        } else {
            // check if we home an axis which needs the override
            self.axes.chars().any(|a| gcmd.get_opt(&a.to_string()).is_some())
        };

        if !override_ {
            return self.prev_g28(gcmd);
        }

        // Calculate forced position (if configured)
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let mut pos = toolhead.get_position();
        let mut homing_axes = String::new();
        for (axis, loc) in self.start_pos.iter().enumerate() {
            if let Some(loc) = loc {
                pos[axis] = *loc;
                homing_axes.push(['x', 'y', 'z'][axis]);
            }
        }
        toolhead.set_position(&pos, &homing_axes)?;
        // Perform homing
        let mut context = self.template.create_template_context();
        let params: indexmap::IndexMap<String, PyVal> =
            gcmd.get_command_parameters().into_iter().map(|(k, v)| (k, PyVal::Str(v))).collect();
        context.insert("params".to_string(), Value::from_pyval(&PyVal::Dict(params)));
        context.insert("rawparams".to_string(), Value::from(gcmd.get_raw_command_parameters()));
        self.in_script.set(true);
        let r = self.template.run_gcode_from_command(Some(context));
        self.in_script.set(false);
        r
    }
}

impl PrinterObject for HomingOverride {
    fn teardown(&self) {
        *self.prev_g28.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(HomingOverride::new(config)?)
}
