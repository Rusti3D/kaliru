// Code for supporting multiple steppers in single filament extruder
// (port of klippy/extras/extruder_stepper.py)
//
// Copyright (C) 2019 Simo Apell <simo.apell@live.fi>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::KResult;
use crate::kinematics::extruder::ExtruderStepper;
use crate::printer::{PObj, PrinterObject};
use crate::pyval::PyVal;
use std::rc::Rc;

pub struct PrinterExtruderStepper {
    pub extruder_stepper: Rc<ExtruderStepper>,
    extruder_name: String,
}

impl PrinterExtruderStepper {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let extruder_stepper = ExtruderStepper::new(config)?;
        let extruder_name = config.get("extruder", Dflt::Required)?;
        let pes = Rc::new(PrinterExtruderStepper { extruder_stepper, extruder_name });
        let p = pes.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| p.extruder_stepper.sync_to_extruder(&p.extruder_name)));
        Ok(pes)
    }

    pub fn find_past_position(&self, print_time: f64) -> f64 {
        self.extruder_stepper.find_past_position(print_time)
    }
}

impl PrinterObject for PrinterExtruderStepper {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(PyVal::Dict(self.extruder_stepper.get_status(eventtime)))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterExtruderStepper::new(config)?)
}
