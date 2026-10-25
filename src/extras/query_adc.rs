// Utility for querying the current state of adc pins
// (port of klippy/extras/query_adc.py)
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::AdcPin;
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

const CMD_QUERY_ADC_HELP: &str = "Report the last value of an analog pin";

pub struct QueryADC {
    adc: RefCell<IndexMap<String, Rc<dyn AdcPin>>>,
}

impl QueryADC {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let q = Rc::new(QueryADC { adc: RefCell::new(IndexMap::default()) });
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let qq = q.clone();
        gcode.register_command("QUERY_ADC", Some(Rc::new(move |g| qq.cmd_query_adc(g))), false, Some(CMD_QUERY_ADC_HELP))?;
        Ok(q)
    }

    pub fn register_adc(&self, name: &str, mcu_adc: Rc<dyn AdcPin>) {
        self.adc.borrow_mut().insert(name.to_string(), mcu_adc);
    }

    fn cmd_query_adc(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let name = gcmd.get_opt("NAME");
        let adc = name.as_ref().and_then(|n| self.adc.borrow().get(n).cloned());
        let Some(adc) = adc else {
            let mut names: Vec<String> = self.adc.borrow().keys().cloned().collect();
            names.sort();
            let objs: Vec<String> = names.iter().map(|n| format!("\"{}\"", n)).collect();
            gcmd.respond_info(&format!("Available ADC objects: {}", objs.join(", ")), true);
            return Ok(());
        };
        let (value, timestamp) = adc.get_last_value();
        let mut msg = format!("ADC object \"{}\" has value {:.6} (timestamp {:.3})", name.unwrap_or_default(), value, timestamp);
        if let Some(pullup) = gcmd.get_float_opt("PULLUP", Lim::new().above(0.0))? {
            let v = py_max(0.00001, py_min(0.99999, value));
            let r = pullup * v / (1.0 - v);
            msg += &format!("\n resistance {:.3} (with {:.0} pullup)", r, pullup);
        }
        gcmd.respond_info(&msg, true);
        Ok(())
    }
}

impl PrinterObject for QueryADC {
    fn teardown(&self) {
        self.adc.borrow_mut().clear();
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(QueryADC::new(config)?)
}
