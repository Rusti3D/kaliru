// Helper script to adjust bed screws (port of klippy/extras/bed_screws.py)
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const CMD_BED_SCREWS_ADJUST_HELP: &str = "Tool to help adjust bed leveling screws";
const CMD_ACCEPT_HELP: &str = "Accept bed screw position";
const CMD_ADJUSTED_HELP: &str = "Accept bed screw position after notable adjustment";
const CMD_ABORT_HELP: &str = "Abort bed screws tool";

type Screw = ((f64, f64), String);

pub struct BedScrews {
    me: Weak<BedScrews>,
    pub printer: Rc<Printer>,
    pub state: RefCell<Option<&'static str>>,
    pub current_screw: Cell<usize>,
    pub accepted_screws: Cell<i64>,
    pub number_of_screws: i64,
    adjust: Vec<Screw>,
    fine: Vec<Screw>,
    pub speed: f64,
    pub lift_speed: f64,
    pub horizontal_move_z: f64,
    pub probe_z: f64,
    pub gcode: Rc<GCodeDispatch>,
}

impl BedScrews {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Read config
        let mut screws = vec![];
        let mut fine_adjust = vec![];
        for i in 0..99 {
            let prefix = format!("screw{}", i + 1);
            if config.get_opt(&prefix)?.is_none() {
                break;
            }
            let c = config.getfloatlist(&prefix, Dflt::Required, ",", Some(2))?.unwrap_or_default();
            let screw_coord = (c[0], c[1]);
            let screw_name = format!("screw at {:.3},{:.3}", c[0], c[1]);
            let screw_name = config.get(&format!("{}_name", prefix), screw_name)?;
            screws.push((screw_coord, screw_name.clone()));
            let pfa = format!("{}_fine_adjust", prefix);
            if config.get_opt(&pfa)?.is_some() {
                let f = config.getfloatlist(&pfa, Dflt::Required, ",", Some(2))?.unwrap_or_default();
                fine_adjust.push(((f[0], f[1]), screw_name));
            }
        }
        if screws.len() < 3 {
            return Err(KError::config("bed_screws: Must have at least three screws"));
        }
        let speed = config.getfloat("speed", 50.0, Lim::new().above(0.0))?;
        let lift_speed = config.getfloat("probe_speed", 5.0, Lim::new().above(0.0))?;
        let horizontal_move_z = config.getfloat("horizontal_move_z", 5.0, Lim::new())?;
        let probe_z = config.getfloat("probe_height", 0.0, Lim::new())?;
        // Register command
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let bs = Rc::new_cyclic(|me| BedScrews {
            me: me.clone(),
            printer: printer.clone(),
            state: RefCell::new(None),
            current_screw: Cell::new(0),
            accepted_screws: Cell::new(0),
            number_of_screws: screws.len() as i64,
            adjust: screws,
            fine: fine_adjust,
            speed,
            lift_speed,
            horizontal_move_z,
            probe_z,
            gcode: gcode.clone(),
        });
        let b = bs.clone();
        gcode.register_command(
            "BED_SCREWS_ADJUST",
            Some(Rc::new(move |g| b.cmd_bed_screws_adjust(g))),
            false,
            Some(CMD_BED_SCREWS_ADJUST_HELP),
        )?;
        Ok(bs)
    }

    pub fn reset(&self) {
        *self.state.borrow_mut() = None;
        self.current_screw.set(0);
        self.accepted_screws.set(0);
    }

    fn screws(&self, state: &str) -> &Vec<Screw> {
        if state == "fine" {
            &self.fine
        } else {
            &self.adjust
        }
    }

    fn move_to(&self, coord: &[Option<f64>], speed: f64) -> KResult<()> {
        let r = self.printer.lookup::<ToolHead>("toolhead")?.manual_move(coord, speed);
        if let Err(e) = &r {
            if e.is_command_error() {
                self.unregister_commands()?;
                self.reset();
            }
        }
        r
    }

    fn move_to_screw(&self, state: &'static str, screw: usize) -> KResult<()> {
        // Move up, over, and then down
        self.move_to(&[None, None, Some(self.horizontal_move_z)], self.lift_speed)?;
        let ((x, y), name) = self.screws(state)[screw].clone();
        self.move_to(&[Some(x), Some(y), Some(self.horizontal_move_z)], self.speed)?;
        self.move_to(&[Some(x), Some(y), Some(self.probe_z)], self.lift_speed)?;
        // Update state
        *self.state.borrow_mut() = Some(state);
        self.current_screw.set(screw);
        // Register commands
        self.gcode.respond_info(
            &format!("Adjust {}. Then run ACCEPT, ADJUSTED, or ABORT\nUse ADJUSTED if a significant screw adjustment is made", name),
            true,
        );
        let me = self.me.upgrade().expect("bed screws");
        let b = me.clone();
        self.gcode.register_command("ACCEPT", Some(Rc::new(move |g| b.cmd_accept(g))), false, Some(CMD_ACCEPT_HELP))?;
        let b = me.clone();
        self.gcode.register_command("ADJUSTED", Some(Rc::new(move |g| b.cmd_adjusted(g))), false, Some(CMD_ADJUSTED_HELP))?;
        let b = me;
        self.gcode.register_command(
            "ABORT",
            Some(Rc::new(move |_g: &GCodeCommand| {
                b.unregister_commands()?;
                b.reset();
                Ok(())
            })),
            false,
            Some(CMD_ABORT_HELP),
        )?;
        Ok(())
    }

    pub fn unregister_commands(&self) -> KResult<()> {
        for cmd in ["ACCEPT", "ADJUSTED", "ABORT"] {
            self.gcode.register_command(cmd, None, false, None)?;
        }
        Ok(())
    }

    pub fn cmd_bed_screws_adjust(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.state.borrow().is_some() {
            return Err(gcmd.error("Already in bed_screws helper; use ABORT to exit"));
        }
        // reset accepted screws
        self.accepted_screws.set(0);
        self.move_to(&[None, None, Some(self.horizontal_move_z)], self.speed)?;
        self.move_to_screw("adjust", 0)
    }

    pub fn cmd_accept(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.unregister_commands()?;
        self.accepted_screws.set(self.accepted_screws.get() + 1);
        let state = self.state.borrow().unwrap_or("adjust");
        if self.current_screw.get() + 1 < self.screws(state).len() && self.accepted_screws.get() < self.number_of_screws {
            // Continue with next screw
            return self.move_to_screw(state, self.current_screw.get() + 1);
        }
        if self.accepted_screws.get() < self.number_of_screws {
            // Retry coarse adjustments
            return self.move_to_screw("adjust", 0);
        }
        if state == "adjust" && !self.fine.is_empty() {
            // Reset accepted screws for fine adjustment
            self.accepted_screws.set(0);
            // Perform fine screw adjustments
            return self.move_to_screw("fine", 0);
        }
        // Done
        self.reset();
        self.move_to(&[None, None, Some(self.horizontal_move_z)], self.lift_speed)?;
        gcmd.respond_info("Bed screws tool completed successfully", true);
        Ok(())
    }

    pub fn cmd_adjusted(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.unregister_commands()?;
        self.accepted_screws.set(-1);
        self.cmd_accept(gcmd)
    }
}

impl PrinterObject for BedScrews {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let state = *self.state.borrow();
        let d = crate::pydict! {
            "is_active" => PyVal::Bool(state.is_some()),
            "state" => state.map(PyVal::from).unwrap_or(PyVal::None),
            "current_screw" => PyVal::Int(self.current_screw.get() as i64),
            "accepted_screws" => PyVal::Int(self.accepted_screws.get()),
        };
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(BedScrews::new(config)?)
}
