// Endstop accuracy improvement via stepper phase tracking
// (port of klippy/extras/endstop_phase.py)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::Homing;
use crate::extras::tmc::TmcDriver;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{downcast_rc, ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::stepper::{parse_step_distance, RailRef};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const TRINAMIC_DRIVERS: [&str; 7] = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160", "tmc5262"];

// Calculate the trigger phase of a stepper motor
pub struct PhaseCalc {
    printer: Rc<Printer>,
    name: String,
    phases: Cell<Option<i64>>,
    tmc_module: RefCell<Option<Rc<TmcDriver>>>,
    // Statistics tracking for ENDSTOP_PHASE_CALIBRATE
    phase_history: RefCell<Option<Vec<i64>>>,
    last_phase: Cell<Option<i64>>,
    last_mcu_position: Cell<Option<i64>>,
    is_primary: Cell<bool>,
    stats_only: Cell<bool>,
}

impl PhaseCalc {
    fn new(printer: &Rc<Printer>, name: &str, phases: Option<i64>) -> Rc<Self> {
        Rc::new(PhaseCalc {
            printer: printer.clone(),
            name: name.to_string(),
            phases: Cell::new(phases),
            tmc_module: RefCell::new(None),
            phase_history: RefCell::new(None),
            last_phase: Cell::new(None),
            last_mcu_position: Cell::new(None),
            is_primary: Cell::new(false),
            stats_only: Cell::new(false),
        })
    }

    fn lookup_tmc(&self) {
        for driver in TRINAMIC_DRIVERS {
            let driver_name = format!("{} {}", driver, self.name);
            if let Some(module) = self.printer.lookup_object_opt(&driver_name).and_then(downcast_rc::<TmcDriver>) {
                if self.phases.get().is_none() {
                    let (_phase_offset, phases) = module.get_phase_offset();
                    self.phases.set(Some(phases));
                }
                *self.tmc_module.borrow_mut() = Some(module);
                break;
            }
        }
        if let Some(phases) = self.phases.get() {
            *self.phase_history.borrow_mut() = Some(vec![0; phases as usize]);
        }
    }

    fn convert_phase(&self, driver_phase: i64, driver_phases: i64) -> i64 {
        let phases = self.phases.get().unwrap_or(0);
        ((driver_phase as f64 / driver_phases as f64 * phases as f64 + 0.5).trunc() as i64).rem_euclid(phases)
    }

    fn calc_phase(&self, trig_mcu_pos: i64) -> KResult<i64> {
        let mut mcu_phase_offset = 0;
        if let Some(tmc) = self.tmc_module.borrow().as_ref() {
            match tmc.get_phase_offset().0 {
                Some(o) => mcu_phase_offset = o,
                None => {
                    if self.printer.start_arg("debugoutput").is_none() {
                        return Err(KError::command(format!("Stepper {} phase unknown", self.name)));
                    }
                }
            }
        }
        let phases = self.phases.get().unwrap_or(0);
        let phase = (trig_mcu_pos + mcu_phase_offset).rem_euclid(phases);
        if let Some(h) = self.phase_history.borrow_mut().as_mut() {
            h[phase as usize] += 1;
        }
        self.last_phase.set(Some(phase));
        self.last_mcu_position.set(Some(trig_mcu_pos));
        Ok(phase)
    }
}

fn rails_arg(args: &[&dyn Any]) -> (Rc<Homing>, Vec<RailRef>) {
    let homing_state = args[0].downcast_ref::<Rc<Homing>>().expect("homing state").clone();
    let rails = args[1].downcast_ref::<Vec<RailRef>>().expect("rails").clone();
    (homing_state, rails)
}

// Adjusted endstop trigger positions
pub struct EndstopPhase {
    name: String,
    step_dist: f64,
    phases: i64,
    phase_calc: Rc<PhaseCalc>,
    endstop_phase: Cell<Option<i64>>,
    endstop_align_zero: bool,
    endstop_phase_accuracy: i64,
}

impl EndstopPhase {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        // Obtain step_distance and microsteps from stepper config section
        let sconfig = config.getsection(&name);
        let (rotation_dist, steps_per_rotation) = parse_step_distance(&sconfig, None, false)?;
        let step_dist = rotation_dist / steps_per_rotation;
        let phases = sconfig.getint("microsteps", Dflt::Required, Lim::new().no_note())? * 4;
        let phase_calc = PhaseCalc::new(&printer, &name, Some(phases));
        // Register event handlers
        let pc = phase_calc.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| {
                pc.lookup_tmc();
                Ok(())
            }),
        );
        let slot: Rc<RefCell<std::rc::Weak<EndstopPhase>>> = Rc::new(RefCell::new(std::rc::Weak::new()));
        let s = slot.clone();
        printer.register_event_handler(
            "homing:home_rails_end",
            Rc::new(move |args| match s.borrow().upgrade() {
                Some(e) => {
                    let (homing_state, rails) = rails_arg(args);
                    e.handle_home_rails_end(&homing_state, &rails)
                }
                None => Ok(()),
            }),
        );
        printer.load_object(config, "endstop_phase", true)?;
        // Read config
        let mut endstop_phase = None;
        if let Some(trigger_phase) = config.get_opt("trigger_phase")? {
            let v = config.getintlist("trigger_phase", Dflt::Required, "/", Some(2))?.unwrap_or_default();
            let (p, ps) = (v[0], v[1]);
            if p >= ps {
                return Err(KError::config(format!("Invalid trigger_phase '{}'", trigger_phase)));
            }
            endstop_phase = Some(phase_calc.convert_phase(p, ps));
        }
        let endstop_align_zero = config.getboolean("endstop_align_zero", false)?;
        let endstop_accuracy = config.getfloat_opt("endstop_accuracy", Lim::new().above(0.0))?;
        // Determine endstop accuracy
        let mut endstop_phase_accuracy = match endstop_accuracy {
            None => phases.div_euclid(2) - 1,
            Some(a) if endstop_phase.is_some() => (a * 0.5 / step_dist).ceil() as i64,
            Some(a) => (a / step_dist).ceil() as i64,
        };
        if endstop_phase_accuracy >= phases.div_euclid(2) {
            return Err(KError::config(format!("Endstop for {} is not accurate enough for stepper phase adjustment", name)));
        }
        if printer.start_arg("debugoutput").is_some() {
            endstop_phase_accuracy = phases;
        }
        let e = Rc::new(EndstopPhase {
            name,
            step_dist,
            phases,
            phase_calc,
            endstop_phase: Cell::new(endstop_phase),
            endstop_align_zero,
            endstop_phase_accuracy,
        });
        *slot.borrow_mut() = Rc::downgrade(&e);
        Ok(e)
    }

    fn align_endstop(&self, rail: &RailRef) -> f64 {
        let Some(endstop_phase) = self.endstop_phase.get() else { return 0.0 };
        if !self.endstop_align_zero {
            return 0.0;
        }
        // Adjust the endstop position so 0.0 is always at a full step
        let microsteps = self.phases.div_euclid(4);
        let half_microsteps = microsteps.div_euclid(2);
        let phase_offset = ((endstop_phase + half_microsteps).rem_euclid(microsteps) - half_microsteps) as f64 * self.step_dist;
        let full_step = microsteps as f64 * self.step_dist;
        let pe = rail.get_homing_info().position_endstop;
        (pe / full_step + 0.5).trunc() * full_step - pe + phase_offset
    }

    fn get_homed_offset(&self, trig_mcu_pos: i64) -> KResult<f64> {
        let phase = self.phase_calc.calc_phase(trig_mcu_pos)?;
        let Some(endstop_phase) = self.endstop_phase.get() else {
            logging::info(&format!("Setting {} endstop phase to {}", self.name, phase));
            self.endstop_phase.set(Some(phase));
            return Ok(0.0);
        };
        let mut delta = (phase - endstop_phase).rem_euclid(self.phases);
        if delta >= self.phases - self.endstop_phase_accuracy {
            delta -= self.phases;
        } else if delta > self.endstop_phase_accuracy {
            return Err(KError::command(format!("Endstop {} incorrect phase (got {} vs {})", self.name, phase, endstop_phase)));
        }
        Ok(delta as f64 * self.step_dist)
    }

    fn handle_home_rails_end(&self, homing_state: &Rc<Homing>, rails: &[RailRef]) -> KResult<()> {
        for rail in rails {
            let stepper = rail.get_steppers()[0].clone();
            if stepper.get_name(false) == self.name {
                let trig_mcu_pos = homing_state.get_trigger_position(&self.name)?;
                let align = self.align_endstop(rail);
                let offset = self.get_homed_offset(trig_mcu_pos)?;
                homing_state.set_stepper_adjustment(&self.name, align + offset);
                return Ok(());
            }
        }
        Ok(())
    }
}

impl PrinterObject for EndstopPhase {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

// Support for ENDSTOP_PHASE_CALIBRATE command
pub struct EndstopPhases {
    printer: Rc<Printer>,
    tracking: RefCell<IndexMap<String, Rc<PhaseCalc>>>,
    gcode: Rc<GCodeDispatch>,
}

const CMD_ENDSTOP_PHASE_CALIBRATE_HELP: &str = "Calibrate stepper phase";

impl EndstopPhases {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let e = Rc::new(EndstopPhases { printer: printer.clone(), tracking: RefCell::new(IndexMap::new()), gcode: gcode.clone() });
        // Register handlers
        let ee = Rc::downgrade(&e);
        printer.register_event_handler(
            "homing:home_rails_end",
            Rc::new(move |args| match ee.upgrade() {
                Some(e) => {
                    let (homing_state, rails) = rails_arg(args);
                    e.handle_home_rails_end(&homing_state, &rails)
                }
                None => Ok(()),
            }),
        );
        let ee = Rc::downgrade(&e);
        gcode.register_command(
            "ENDSTOP_PHASE_CALIBRATE",
            Some(Rc::new(move |g| match ee.upgrade() {
                Some(e) => e.cmd_endstop_phase_calibrate(g),
                None => Ok(()),
            })),
            false,
            Some(CMD_ENDSTOP_PHASE_CALIBRATE_HELP),
        )?;
        Ok(e)
    }

    fn update_stepper(&self, stepper_name: &str, trig_mcu_pos: i64, is_primary: bool) -> KResult<()> {
        let existing = self.tracking.borrow().get(stepper_name).cloned();
        let phase_calc = match existing {
            Some(pc) => pc,
            None => {
                // Check if stepper has an endstop_phase config section defined
                let mod_name = format!("endstop_phase {}", stepper_name);
                let pc = match self.printer.lookup_object_opt(&mod_name).and_then(downcast_rc::<EndstopPhase>) {
                    Some(m) => m.phase_calc.clone(),
                    None => {
                        // Create new PhaseCalc tracker
                        let pc = PhaseCalc::new(&self.printer, stepper_name, None);
                        pc.stats_only.set(true);
                        pc.lookup_tmc();
                        pc
                    }
                };
                self.tracking.borrow_mut().insert(stepper_name.to_string(), pc.clone());
                pc
            }
        };
        if phase_calc.phase_history.borrow().is_none() {
            return Ok(());
        }
        if is_primary {
            phase_calc.is_primary.set(true);
        }
        if phase_calc.stats_only.get() {
            phase_calc.calc_phase(trig_mcu_pos)?;
        }
        Ok(())
    }

    fn handle_home_rails_end(&self, homing_state: &Rc<Homing>, rails: &[RailRef]) -> KResult<()> {
        for rail in rails {
            let mut is_primary = true;
            for stepper in rail.get_steppers() {
                let sname = stepper.get_name(false);
                let trig_mcu_pos = homing_state.get_trigger_position(&sname)?;
                self.update_stepper(&sname, trig_mcu_pos, is_primary)?;
                is_primary = false;
            }
        }
        Ok(())
    }

    fn cmd_endstop_phase_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let Some(stepper_name) = gcmd.get_opt("STEPPER") else {
            self.report_stats();
            return Ok(());
        };
        let phase_calc = self.tracking.borrow().get(&stepper_name).cloned();
        let Some(phase_calc) = phase_calc.filter(|pc| pc.phase_history.borrow().is_some()) else {
            return Err(gcmd.error(format!("Stats not available for stepper {}", stepper_name)));
        };
        let (endstop_phase, phases) = self.generate_stats(&stepper_name, &phase_calc);
        if !phase_calc.is_primary.get() {
            return Ok(());
        }
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let section = format!("endstop_phase {}", stepper_name);
        {
            let mut cf = configfile.pconfig.borrow_mut();
            cf.remove_section(&section);
            cf.set(&section, "trigger_phase", &format!("{}/{}", endstop_phase, phases))?;
        }
        gcmd.respond_info(
            "The SAVE_CONFIG command will update the printer config\nfile with these parameters and restart the printer.",
            true,
        );
        Ok(())
    }

    fn generate_stats(&self, stepper_name: &str, phase_calc: &PhaseCalc) -> (i64, i64) {
        let phase_history = phase_calc.phase_history.borrow().clone().unwrap_or_default();
        let wph: Vec<i64> = phase_history.iter().chain(phase_history.iter()).copied().collect();
        let phases = phase_history.len() as i64;
        let half_phases = phases.div_euclid(2);
        let mut res = vec![];
        for i in 0..phases {
            let phase = i + half_phases;
            let cost: i64 = (i..i + phases).map(|j| wph[j as usize] * (j - phase).abs()).sum();
            res.push((cost, phase));
        }
        res.sort();
        let best = res[0].1;
        // wph[j] with python negative indexing
        let at = |j: i64| wph[j.rem_euclid(wph.len() as i64) as usize];
        let found: Vec<i64> = (best - half_phases..best + half_phases).filter(|&j| at(j) != 0).collect();
        let best_phase = best.rem_euclid(phases);
        let (lo, hi) = (found[0].rem_euclid(phases), found[found.len() - 1].rem_euclid(phases));
        self.gcode.respond_info(
            &format!("{}: trigger_phase={}/{} (range {} to {})", stepper_name, best_phase, phases, lo, hi),
            true,
        );
        (best_phase, phases)
    }

    fn report_stats(&self) {
        if self.tracking.borrow().is_empty() {
            self.gcode.respond_info("No steppers found. (Be sure to home at least once.)", true);
            return;
        }
        let mut names: Vec<String> = self.tracking.borrow().keys().cloned().collect();
        names.sort();
        for stepper_name in names {
            let phase_calc = self.tracking.borrow()[&stepper_name].clone();
            if !phase_calc.is_primary.get() {
                continue;
            }
            self.generate_stats(&stepper_name, &phase_calc);
        }
    }
}

impl PrinterObject for EndstopPhases {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let opt = |v: Option<i64>| v.map(PyVal::Int).unwrap_or(PyVal::None);
        let mut lh = IndexMap::new();
        for (name, pc) in self.tracking.borrow().iter() {
            if pc.phase_history.borrow().is_none() {
                continue;
            }
            let mut d = IndexMap::new();
            d.insert("phase".to_string(), opt(pc.last_phase.get()));
            d.insert("phases".to_string(), opt(pc.phases.get()));
            d.insert("mcu_position".to_string(), opt(pc.last_mcu_position.get()));
            lh.insert(name.clone(), PyVal::Dict(d));
        }
        let mut res = IndexMap::new();
        res.insert("last_home".to_string(), PyVal::Dict(lh));
        Some(PyVal::Dict(res))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(EndstopPhase::new(config)?)
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(EndstopPhases::new(config)?)
}
