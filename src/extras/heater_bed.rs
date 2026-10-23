// Support for a heated bed (port of klippy/extras/heater_bed.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::heaters::{load_heaters, Heater, PrinterHeaters};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use std::any::Any;
use std::rc::Rc;

pub struct PrinterHeaterBed {
    printer: Rc<Printer>,
    pub heater: Rc<Heater>,
}

impl PrinterHeaterBed {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let pheaters = load_heaters(config)?;
        let heater = pheaters.setup_heater(config, Some("B"))?;
        let hb = Rc::new(PrinterHeaterBed { printer: printer.clone(), heater });
        // Register commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let h = hb.clone();
        gcode.register_command("M140", Some(Rc::new(move |g| h.cmd_m140(g, false))), false, None)?;
        let h = hb.clone();
        gcode.register_command("M190", Some(Rc::new(move |g| h.cmd_m140(g, true))), false, None)?;
        Ok(hb)
    }

    fn cmd_m140(&self, gcmd: &GCodeCommand, wait: bool) -> KResult<()> {
        // Set Bed Temperature
        let temp = gcmd.get_float("S", Some(0.0), Lim::new())?;
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        pheaters.set_temperature(&self.heater, temp, wait)
    }
}

impl PrinterObject for PrinterHeaterBed {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.heater.get_status(eventtime))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(self.heater.stats(eventtime))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterHeaterBed::new(config)?)
}
