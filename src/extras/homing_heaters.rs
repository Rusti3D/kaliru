// Heater handling on homing moves (port of klippy/extras/homing_heaters.py)
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::extras::heaters::PrinterHeaters;
use crate::extras::homing::HomingMove;
use crate::mcu::EndstopRef;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

pub struct HomingHeaters {
    printer: Rc<Printer>,
    disable_heaters: RefCell<Option<Vec<String>>>,
    flaky_steppers: Option<Vec<String>>,
    pheaters: Rc<PrinterHeaters>,
    target_save: RefCell<IndexMap<String, f64>>,
}

/// repr() of the getlist() tuple of names
fn names_repr(v: &[String]) -> String {
    PyVal::Tuple(v.iter().map(|s| PyVal::Str(s.clone())).collect()).repr()
}

impl HomingHeaters {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let disable_heaters = config.getlist("heaters", Dflt::None, ",", None)?;
        let flaky_steppers = config.getlist("steppers", Dflt::None, ",", None)?;
        let pheaters = printer.load_as::<PrinterHeaters>(config, "heaters")?;
        let h = Rc::new(HomingHeaters {
            printer: printer.clone(),
            disable_heaters: RefCell::new(disable_heaters),
            flaky_steppers,
            pheaters,
            target_save: RefCell::new(IndexMap::new()),
        });
        let w: Weak<HomingHeaters> = Rc::downgrade(&h);
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match w.upgrade() {
                Some(h) => h.handle_connect(),
                None => Ok(()),
            }),
        );
        for (event, begin) in [("homing:homing_move_begin", true), ("homing:homing_move_end", false)] {
            let w = Rc::downgrade(&h);
            printer.register_event_handler(
                event,
                Rc::new(move |args| {
                    let (Some(h), Some(hmove)) = (w.upgrade(), args[0].downcast_ref::<Rc<HomingMove>>()) else { return Ok(()) };
                    if begin {
                        h.handle_homing_move_begin(hmove)
                    } else {
                        h.handle_homing_move_end(hmove)
                    }
                }),
            );
        }
        Ok(h)
    }

    fn handle_connect(&self) -> KResult<()> {
        // heaters to disable
        let all_heaters = self.pheaters.get_all_heaters();
        let disable = self.disable_heaters.borrow().clone();
        match disable {
            None => *self.disable_heaters.borrow_mut() = Some(all_heaters),
            Some(d) => {
                if !d.iter().all(|x| all_heaters.contains(x)) {
                    return Err(KError::config(format!("One or more of these heaters are unknown: {}", names_repr(&d))));
                }
            }
        }
        // steppers valid?
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        let all_steppers: Vec<String> = kin.get_steppers().iter().map(|s| s.get_name(false)).collect();
        let Some(flaky) = &self.flaky_steppers else { return Ok(()) };
        if !flaky.iter().all(|x| all_steppers.contains(x)) {
            return Err(KError::config(format!("One or more of these steppers are unknown: {}", names_repr(flaky))));
        }
        Ok(())
    }

    fn check_eligible(&self, endstops: &[EndstopRef]) -> bool {
        let Some(flaky) = &self.flaky_steppers else { return true };
        let steppers_being_homed: Vec<String> = endstops.iter().flat_map(|es| es.get_steppers()).map(|s| s.get_name(false)).collect();
        steppers_being_homed.iter().any(|x| flaky.contains(x))
    }

    fn handle_homing_move_begin(&self, hmove: &HomingMove) -> KResult<()> {
        if !self.check_eligible(&hmove.get_mcu_endstops()) {
            return Ok(());
        }
        let heaters = self.disable_heaters.borrow().clone().unwrap_or_default();
        for heater_name in heaters {
            let heater = self.pheaters.lookup_heater(&heater_name)?;
            self.target_save.borrow_mut().insert(heater_name, heater.get_temp(0.0).1);
            heater.set_temp(0.0)?;
        }
        Ok(())
    }

    fn handle_homing_move_end(&self, hmove: &HomingMove) -> KResult<()> {
        if !self.check_eligible(&hmove.get_mcu_endstops()) {
            return Ok(());
        }
        let heaters = self.disable_heaters.borrow().clone().unwrap_or_default();
        for heater_name in heaters {
            let heater = self.pheaters.lookup_heater(&heater_name)?;
            let target = self.target_save.borrow().get(&heater_name).copied().ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(&heater_name)))?;
            heater.set_temp(target)?;
        }
        Ok(())
    }
}

impl PrinterObject for HomingHeaters {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(HomingHeaters::new(config)?)
}
