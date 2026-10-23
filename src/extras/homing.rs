// Helper code for implementing homing operations
// (port of klippy/extras/homing.py)
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::EndstopRef;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::reactor::{ReactorCompletion, NEVER, NOW};
use crate::stepper::{McuStepper, RailRef};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

thread_local! {
    // module globals, set by load_config() from danger_options
    static HOMING_START_DELAY: Cell<f64> = const { Cell::new(0.001) };
    static ENDSTOP_SAMPLE_TIME: Cell<f64> = const { Cell::new(0.000015) };
    static ENDSTOP_SAMPLE_COUNT: Cell<i64> = const { Cell::new(4) };
}

fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

// Return a completion that completes when all completions in a list complete
pub fn multi_complete(printer: &Printer, completions: &[Rc<ReactorCompletion<bool>>]) -> Rc<ReactorCompletion<bool>> {
    if completions.len() == 1 {
        return completions[0].clone();
    }
    // Build completion that waits for all completions
    let reactor = printer.get_reactor();
    let cs = completions.to_vec();
    let cp = reactor.register_callback(
        move |_| {
            for c in &cs {
                let _ = c.wait(NEVER, None);
            }
            // the list of results is truthy unless empty
            !cs.is_empty()
        },
        NOW,
    );
    // If any completion indicates an error, then exit main completion early
    for c in completions {
        let (c, cp) = (c.clone(), cp.clone());
        reactor.register_callback(
            move |_| {
                if let Ok(Some(true)) = c.wait(NEVER, None) {
                    cp.complete(true);
                }
            },
            NOW,
        );
    }
    cp
}

// Tracking of stepper positions during a homing/probing move
pub struct StepperPosition {
    pub stepper: Rc<McuStepper>,
    pub endstop_name: String,
    pub stepper_name: String,
    pub start_pos: i64,
    pub start_cmd_pos: f64,
    pub halt_pos: Option<i64>,
    pub trig_pos: Option<i64>,
}

impl StepperPosition {
    fn new(stepper: &Rc<McuStepper>, endstop_name: &str) -> Self {
        let start_pos = stepper.get_mcu_position(None);
        StepperPosition {
            stepper: stepper.clone(),
            endstop_name: endstop_name.to_string(),
            stepper_name: stepper.get_name(false),
            start_pos,
            start_cmd_pos: stepper.mcu_to_commanded_position(start_pos),
            halt_pos: None,
            trig_pos: None,
        }
    }

    fn note_home_end(&mut self, trigger_time: f64) {
        self.halt_pos = Some(self.stepper.get_mcu_position(None));
        self.trig_pos = Some(self.stepper.get_past_mcu_position(trigger_time));
    }

    fn verify_no_probe_skew(&self) {
        let new_start_pos = self.stepper.get_mcu_position(Some(self.start_cmd_pos));
        if new_start_pos != self.start_pos {
            logging::warning(&format!(
                "Stepper '{}' position skew after probe: pos {} now {}",
                self.stepper.get_name(false),
                self.start_pos,
                new_start_pos
            ));
        }
    }

    fn halt(&self) -> i64 {
        self.halt_pos.unwrap_or(0)
    }

    pub fn trig(&self) -> i64 {
        self.trig_pos.unwrap_or(0)
    }
}

// Implementation of homing/probing moves
/// the toolhead a homing move drives: the toolhead, or a manual_stepper
/// (python duck types toolhead.get_kinematics() on both)
pub trait HomingToolhead {
    fn get_position(&self) -> Vec<f64>;
    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()>;
    fn flush_step_generation(&self) -> KResult<()>;
    fn get_last_move_time(&self) -> KResult<f64>;
    fn dwell(&self, delay: f64) -> KResult<()>;
    fn drip_move(&self, newpos: &[f64], speed: f64, drip_completion: &Rc<crate::reactor::ReactorCompletion<bool>>) -> KResult<()>;
    /// get_kinematics().get_steppers()
    fn kin_steppers(&self) -> Vec<Rc<McuStepper>>;
    /// get_kinematics().calc_position(stepper_positions)
    fn kin_calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64>;
}

impl HomingToolhead for ToolHead {
    fn get_position(&self) -> Vec<f64> {
        ToolHead::get_position(self)
    }
    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        ToolHead::set_position(self, newpos, homing_axes)
    }
    fn flush_step_generation(&self) -> KResult<()> {
        ToolHead::flush_step_generation(self)
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        ToolHead::get_last_move_time(self)
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        ToolHead::dwell(self, delay)
    }
    fn drip_move(&self, newpos: &[f64], speed: f64, drip_completion: &Rc<crate::reactor::ReactorCompletion<bool>>) -> KResult<()> {
        ToolHead::drip_move(self, newpos, speed, drip_completion)
    }
    fn kin_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.get_kinematics().get_steppers()
    }
    fn kin_calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        self.get_kinematics().calc_position(stepper_positions)
    }
}

pub struct HomingMove {
    printer: Rc<Printer>,
    endstops: Vec<(EndstopRef, String)>,
    toolhead: Rc<dyn HomingToolhead>,
    pub stepper_positions: RefCell<Vec<StepperPosition>>,
    pub distance_elapsed: RefCell<Vec<f64>>,
}

impl HomingMove {
    pub fn new(printer: &Rc<Printer>, endstops: Vec<(EndstopRef, String)>, toolhead: Option<Rc<dyn HomingToolhead>>) -> KResult<Rc<Self>> {
        let toolhead = match toolhead {
            Some(t) => t,
            None => printer.lookup::<ToolHead>("toolhead")? as Rc<dyn HomingToolhead>,
        };
        Ok(Rc::new(HomingMove {
            printer: printer.clone(),
            endstops,
            toolhead,
            stepper_positions: RefCell::new(vec![]),
            distance_elapsed: RefCell::new(vec![]),
        }))
    }

    pub fn get_mcu_endstops(&self) -> Vec<EndstopRef> {
        self.endstops.iter().map(|(es, _)| es.clone()).collect()
    }

    fn calc_endstop_rate(&self, mcu_endstop: &EndstopRef, movepos: &[f64], speed: f64) -> KResult<f64> {
        let startpos = self.toolhead.get_position();
        let axes_d: Vec<f64> = movepos.iter().zip(&startpos).map(|(mp, sp)| mp - sp).collect();
        let move_d = crate::pyfmt::py_sum(axes_d[..3].iter().map(|d| d * d)).sqrt();
        let move_t = move_d / speed;
        let steps: Vec<f64> = mcu_endstop
            .get_steppers()
            .iter()
            .map(|s| (s.calc_position_from_coord(&startpos) - s.calc_position_from_coord(movepos)).abs() / s.get_step_dist())
            .collect();
        if steps.is_empty() {
            return Err(KError::Other("max() iterable argument is empty".into()));
        }
        let max_steps = steps[1..].iter().fold(steps[0], |a, &b| if b > a { b } else { a });
        if max_steps <= 0.0 {
            return Ok(0.001);
        }
        Ok(move_t / max_steps)
    }

    pub fn calc_toolhead_pos(&self, kin_spos: &IndexMap<String, f64>, offsets: &IndexMap<String, i64>) -> KResult<Vec<f64>> {
        let mut kin_spos = kin_spos.clone();
        for stepper in self.toolhead.kin_steppers() {
            let sname = stepper.get_name(false);
            let off = offsets.get(&sname).copied().unwrap_or(0) as f64 * stepper.get_step_dist();
            match kin_spos.get_mut(&sname) {
                Some(v) => *v += off,
                None => return Err(KError::Other(crate::pyfmt::py_str_repr(&sname))),
            }
        }
        let thpos = self.toolhead.get_position();
        let mut res: Vec<f64> = self.toolhead.kin_calc_position(&kin_spos).into_iter().take(3).collect();
        res.extend_from_slice(&thpos[3..]);
        Ok(res)
    }

    fn kin_spos(&self) -> IndexMap<String, f64> {
        self.toolhead.kin_steppers().iter().map(|s| (s.get_name(false), s.get_commanded_position())).collect()
    }

    pub fn homing_move(
        self: &Rc<Self>,
        movepos: &[f64],
        speed: f64,
        probe_pos: bool,
        triggered: bool,
        check_triggered: bool,
    ) -> KResult<Vec<f64>> {
        // Notify start of homing/probing move
        self.printer.send_event("homing:homing_move_begin", &[self])?;
        // Note start location
        self.toolhead.flush_step_generation()?;
        let kin_spos = self.kin_spos();
        *self.stepper_positions.borrow_mut() = self
            .endstops
            .iter()
            .flat_map(|(es, name)| es.get_steppers().into_iter().map(move |s| StepperPosition::new(&s, name)))
            .collect();
        // Start endstop checking
        let print_time = self.toolhead.get_last_move_time()?;
        let mut endstop_triggers = vec![];
        for (mcu_endstop, _name) in &self.endstops {
            let rest_time = self.calc_endstop_rate(mcu_endstop, movepos, speed)?;
            let wait = mcu_endstop.home_start(
                print_time,
                ENDSTOP_SAMPLE_TIME.with(|v| v.get()),
                ENDSTOP_SAMPLE_COUNT.with(|v| v.get()),
                rest_time,
                triggered,
            )?;
            endstop_triggers.push(wait);
        }
        let all_endstop_trigger = multi_complete(&self.printer, &endstop_triggers);

        self.toolhead.dwell(HOMING_START_DELAY.with(|v| v.get()))?;
        // Issue move
        let mut error = None;
        match self.toolhead.drip_move(movepos, speed, &all_endstop_trigger) {
            Err(e) if e.is_command_error() => error = Some(format!("Error during homing move: {}", e)),
            Err(e) => return Err(e),
            Ok(()) => {}
        }
        // Wait for endstops to trigger
        let mut trigger_times = IndexMap::new();
        let move_end_print_time = self.toolhead.get_last_move_time()?;
        for (mcu_endstop, name) in &self.endstops {
            let trigger_time = match mcu_endstop.home_wait(move_end_print_time) {
                Err(e) if e.is_command_error() => {
                    if error.is_none() {
                        error = Some(format!("Error during homing {}: {}", name, e));
                    }
                    continue;
                }
                Err(e) => return Err(e),
                Ok(t) => t,
            };
            if trigger_time > 0.0 {
                trigger_times.insert(name.clone(), trigger_time);
            } else if check_triggered && error.is_none() {
                error = Some(format!("No trigger on {} after full movement", name));
            }
        }
        // Determine stepper halt positions
        self.toolhead.flush_step_generation()?;
        for sp in self.stepper_positions.borrow_mut().iter_mut() {
            let tt = trigger_times.get(&sp.endstop_name).copied().unwrap_or(move_end_print_time);
            sp.note_home_end(tt);
        }
        let trigpos;
        if probe_pos {
            let (halt_steps, trig_steps): (IndexMap<String, i64>, IndexMap<String, i64>) = {
                let sps = self.stepper_positions.borrow();
                (
                    sps.iter().map(|sp| (sp.stepper_name.clone(), sp.halt() - sp.start_pos)).collect(),
                    sps.iter().map(|sp| (sp.stepper_name.clone(), sp.trig() - sp.start_pos)).collect(),
                )
            };
            trigpos = self.calc_toolhead_pos(&kin_spos, &trig_steps)?;
            let mut haltpos = trigpos.clone();
            if !dict_eq(&trig_steps, &halt_steps) {
                haltpos = self.calc_toolhead_pos(&kin_spos, &halt_steps)?;
            }
            self.toolhead.set_position(&haltpos, "")?;
            for sp in self.stepper_positions.borrow().iter() {
                sp.verify_no_probe_skew();
            }
        } else {
            trigpos = movepos.to_vec();
            let mut haltpos = movepos.to_vec();
            let (over_steps, steps_moved): (IndexMap<String, i64>, IndexMap<String, f64>) = {
                let sps = self.stepper_positions.borrow();
                (
                    sps.iter().map(|sp| (sp.stepper_name.clone(), sp.halt() - sp.trig())).collect(),
                    sps.iter()
                        .map(|sp| (sp.stepper_name.clone(), (sp.halt() - sp.start_pos) as f64 * sp.stepper.get_step_dist()))
                        .collect(),
                )
            };
            let filled_steps_moved: IndexMap<String, f64> = self
                .toolhead
                .kin_steppers()
                .iter()
                .map(|s| {
                    let sname = s.get_name(false);
                    let v = steps_moved.get(&sname).copied().unwrap_or(0.0);
                    (sname, v)
                })
                .collect();
            *self.distance_elapsed.borrow_mut() = self.toolhead.kin_calc_position(&filled_steps_moved);
            if over_steps.values().any(|v| *v != 0) {
                self.toolhead.set_position(movepos, "")?;
                let halt_kin_spos = self.kin_spos();
                haltpos = self.calc_toolhead_pos(&halt_kin_spos, &over_steps)?;
            }
            self.toolhead.set_position(&haltpos, "")?;
        }
        // Signal homing/probing move complete
        match self.printer.send_event("homing:homing_move_end", &[self]) {
            Err(e) if e.is_command_error() => {
                if error.is_none() {
                    error = Some(e.to_string());
                }
            }
            Err(e) => return Err(e),
            Ok(()) => {}
        }
        if let Some(error) = error {
            return Err(KError::command(error));
        }
        Ok(trigpos)
    }

    pub fn check_no_movement(&self) -> Option<String> {
        if self.printer.start_arg("debuginput").is_some() {
            return None;
        }
        for sp in self.stepper_positions.borrow().iter() {
            if Some(sp.start_pos) == sp.trig_pos {
                return Some(sp.endstop_name.clone());
            }
        }
        None
    }

    pub fn moved_less_than_dist(&self, min_dist: f64, homing_axes: &[usize]) -> bool {
        let distance_tolerance = get_danger_options().homing_elapsed_distance_tolerance;
        self.distance_elapsed
            .borrow()
            .iter()
            .enumerate()
            .filter(|(i, _)| homing_axes.contains(i))
            .any(|(_, dist)| dist.abs() < min_dist && min_dist - dist.abs() >= distance_tolerance)
    }
}

/// python dict == for the step maps (same keys and values, any order)
fn dict_eq(a: &IndexMap<String, i64>, b: &IndexMap<String, i64>) -> bool {
    a.len() == b.len() && a.iter().all(|(k, v)| b.get(k) == Some(v))
}

// State tracking of homing requests
pub struct Homing {
    printer: Rc<Printer>,
    toolhead: Rc<ToolHead>,
    changed_axes: RefCell<Vec<usize>>,
    trigger_mcu_pos: RefCell<IndexMap<String, i64>>,
    adjust_pos: RefCell<IndexMap<String, f64>>,
    saved_accel: Cell<Option<f64>>,
}

impl Homing {
    pub fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        Ok(Rc::new(Homing {
            printer: printer.clone(),
            toolhead: printer.lookup::<ToolHead>("toolhead")?,
            changed_axes: RefCell::new(vec![]),
            trigger_mcu_pos: RefCell::new(IndexMap::new()),
            adjust_pos: RefCell::new(IndexMap::new()),
            saved_accel: Cell::new(None),
        }))
    }

    /// Homing(printer) driving another toolhead (trad_rack's TradRackHoming
    /// sets self.toolhead after Homing.__init__)
    pub fn with_toolhead(printer: &Rc<Printer>, toolhead: Rc<ToolHead>) -> Rc<Self> {
        Rc::new(Homing {
            printer: printer.clone(),
            toolhead,
            changed_axes: RefCell::new(vec![]),
            trigger_mcu_pos: RefCell::new(IndexMap::new()),
            adjust_pos: RefCell::new(IndexMap::new()),
            saved_accel: Cell::new(None),
        })
    }

    pub fn toolhead(&self) -> Rc<ToolHead> {
        self.toolhead.clone()
    }

    pub fn printer(&self) -> Rc<Printer> {
        self.printer.clone()
    }

    /// self.trigger_mcu_pos = ...
    pub fn set_trigger_positions(&self, trigger_mcu_pos: IndexMap<String, i64>) {
        *self.trigger_mcu_pos.borrow_mut() = trigger_mcu_pos;
    }

    /// self.adjust_pos
    pub fn adjust_pos(&self) -> IndexMap<String, f64> {
        self.adjust_pos.borrow().clone()
    }

    /// self.adjust_pos = {}
    pub fn clear_adjust_pos(&self) {
        self.adjust_pos.borrow_mut().clear();
    }

    pub fn set_axes(&self, axes: Vec<usize>) {
        *self.changed_axes.borrow_mut() = axes;
    }

    pub fn get_axes(&self) -> Vec<usize> {
        self.changed_axes.borrow().clone()
    }

    pub fn get_trigger_position(&self, stepper_name: &str) -> KResult<i64> {
        self.trigger_mcu_pos
            .borrow()
            .get(stepper_name)
            .copied()
            .ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(stepper_name)))
    }

    pub fn set_stepper_adjustment(&self, stepper_name: &str, adjustment: f64) {
        self.adjust_pos.borrow_mut().insert(stepper_name.to_string(), adjustment);
    }

    pub fn fill_coord(&self, coord: &[Option<f64>]) -> Vec<f64> {
        // Fill in any None entries in 'coord' with current toolhead position
        let mut thcoord = self.toolhead.get_position();
        for (i, c) in coord.iter().enumerate() {
            if let Some(c) = c {
                thcoord[i] = *c;
            }
        }
        thcoord
    }

    pub fn set_homed_position(&self, pos: &[Option<f64>]) -> KResult<()> {
        self.toolhead.set_position(&self.fill_coord(pos), "")
    }

    fn set_homing_accel(&self, accel: Option<f64>, pre_homing: bool) {
        let Some(accel) = accel else { return };
        if pre_homing {
            if self.saved_accel.get().is_none() {
                self.saved_accel.set(Some(self.toolhead.get_max_velocity().1));
            }
            self.toolhead.set_accel(accel);
        } else if let Some(saved) = self.saved_accel.take() {
            self.toolhead.set_accel(saved);
        }
    }

    fn set_homing_current(&self, homing_axes: &str, pre_homing: bool) -> KResult<()> {
        let print_time = self.toolhead.get_last_move_time()?;
        // python collects a set(); insertion order here
        let mut affected_rails: Vec<RailRef> = vec![];
        for axis in homing_axes.chars() {
            for rail in self.toolhead.get_active_rails_for_axis(axis) {
                if !affected_rails.iter().any(|r| Rc::ptr_eq(r, &rail)) {
                    affected_rails.push(rail);
                }
            }
        }

        let mut dwell_time: f64 = 0.0;
        for rail in &affected_rails {
            for ch in rail.get_tmc_current_helpers().into_iter().flatten() {
                let current_dwell_time = ch.set_current_for_homing(print_time, pre_homing)?;
                if current_dwell_time > dwell_time {
                    dwell_time = current_dwell_time;
                }
            }
        }

        if dwell_time != 0.0 {
            self.toolhead.dwell(dwell_time)?;
        }
        Ok(())
    }

    fn reset_endstop_states(&self, endstops: &[(EndstopRef, String)]) -> KResult<()> {
        // re-querying a tmc endstop seems to reset the state
        // otherwise it triggers almost immediately upon second home
        // this seems to be an adequate substitute for a 2 second dwell.
        let print_time = self.toolhead.get_last_move_time()?;
        for endstop in endstops {
            endstop.0.query_endstop(print_time)?;
        }
        Ok(())
    }

    /// the retract position for home_rails(): (retractpos, axes_d, retract_r)
    fn retract(&self, forcepos: &[Option<f64>], movepos: &[Option<f64>], retract_dist: f64) -> (Vec<f64>, Vec<f64>, f64) {
        let startpos = self.fill_coord(forcepos);
        let homepos = self.fill_coord(movepos);
        let axes_d: Vec<f64> = homepos.iter().zip(&startpos).map(|(hp, sp)| hp - sp).collect();
        let move_d = crate::pyfmt::py_sum(axes_d[..3].iter().map(|d| d * d)).sqrt();
        let retract_r = py_min(1.0, retract_dist / move_d);
        let retractpos = homepos.iter().zip(&axes_d).map(|(hp, ad)| hp - ad * retract_r).collect();
        (retractpos, axes_d, retract_r)
    }

    pub fn home_rails(self: &Rc<Self>, rails: &[RailRef], forcepos: &[Option<f64>], movepos: &[Option<f64>]) -> KResult<()> {
        // Notify of upcoming homing operation
        let rails_v = rails.to_vec();
        self.printer.send_event("homing:home_rails_begin", &[self, &rails_v])?;
        // Alter kinematics class to think printer is at forcepos
        let force_axes: Vec<usize> = (0..3).filter(|&axis| forcepos[axis].is_some()).collect();
        let homing_axes: String = force_axes.iter().map(|&i| ['x', 'y', 'z'][i]).collect();
        let startpos = self.fill_coord(forcepos);
        let homepos = self.fill_coord(movepos);
        self.toolhead.set_position(&startpos, &homing_axes)?;
        // Perform first home
        let endstops: Vec<(EndstopRef, String)> = rails.iter().flat_map(|r| r.get_endstops()).collect();
        let hi = rails[0].get_homing_info();
        let mut hmove = HomingMove::new(&self.printer, endstops.clone(), None)?;

        let res = (|| -> KResult<()> {
            self.set_homing_accel(hi.accel, true);
            self.set_homing_current(&homing_axes, true)?;
            self.reset_endstop_states(&endstops)?;
            hmove.homing_move(&homepos, hi.speed, false, true, true)?;

            let mut needs_rehome = false;
            let mut retract_dist = hi.retract_dist;
            if hmove.moved_less_than_dist(hi.min_home_dist, &force_axes) {
                needs_rehome = true;
                retract_dist = hi.min_home_dist;
            }

            // Perform second home
            if retract_dist != 0.0 {
                logging::info(&format!("homing: needs rehome: {}", crate::pyfmt::py_bool(needs_rehome)));
                // Retract
                let (retractpos, axes_d, retract_r) = self.retract(forcepos, movepos, retract_dist);
                self.toolhead.move_to(&retractpos, hi.retract_speed)?;
                if !hi.use_sensorless_homing || needs_rehome {
                    // Home again
                    let startpos: Vec<f64> = retractpos.iter().zip(&axes_d).map(|(rp, ad)| rp - ad * retract_r).collect();
                    self.toolhead.set_position(&startpos, "")?;
                    self.reset_endstop_states(&endstops)?;

                    hmove = HomingMove::new(&self.printer, endstops.clone(), None)?;
                    let homepos = self.fill_coord(movepos);
                    hmove.homing_move(&homepos, hi.second_homing_speed, false, true, true)?;

                    if let Some(name) = hmove.check_no_movement() {
                        return Err(KError::command(format!("Endstop {} still triggered after retract", name)));
                    }
                    if hi.use_sensorless_homing && needs_rehome && hmove.moved_less_than_dist(hi.min_home_dist, &force_axes) {
                        return Err(KError::command("Early homing trigger on second home!"));
                    }

                    if hi.use_sensorless_homing && hi.retract_dist != 0.0 {
                        // Retract (again)
                        let (retractpos, _, _) = self.retract(forcepos, movepos, hi.retract_dist);
                        self.toolhead.move_to(&retractpos, hi.retract_speed)?;
                    }
                }
            }
            Ok(())
        })();
        // finally
        self.set_homing_accel(hi.accel, false);
        let fin = self.set_homing_current(&homing_axes, false);
        res?;
        fin?;

        // Signal home operation complete
        self.toolhead.flush_step_generation()?;
        *self.trigger_mcu_pos.borrow_mut() =
            hmove.stepper_positions.borrow().iter().map(|sp| (sp.stepper_name.clone(), sp.trig())).collect();
        self.adjust_pos.borrow_mut().clear();
        self.printer.send_event("homing:home_rails_end", &[self, &rails_v])?;
        if self.adjust_pos.borrow().values().any(|v| *v != 0.0) {
            // Apply any homing offsets
            let kin = self.toolhead.get_kinematics();
            let mut homepos = self.toolhead.get_position();
            let adjust_pos = self.adjust_pos.borrow().clone();
            let kin_spos: IndexMap<String, f64> = kin
                .get_steppers()
                .iter()
                .map(|s| {
                    let sname = s.get_name(false);
                    let v = s.get_commanded_position() + adjust_pos.get(&sname).copied().unwrap_or(0.0);
                    (sname, v)
                })
                .collect();
            let newpos = kin.calc_position(&kin_spos);
            for &axis in &force_axes {
                homepos[axis] = newpos[axis];
            }
            self.toolhead.set_position(&homepos, "")?;
        }
        Ok(())
    }
}

pub struct PrinterHoming {
    printer: Rc<Printer>,
}

impl PrinterHoming {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let ph = Rc::new(PrinterHoming { printer: printer.clone() });
        // Register g-code commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let p = ph.clone();
        gcode.register_command("G28", Some(Rc::new(move |g| p.cmd_g28(g))), false, None)?;
        Ok(ph)
    }

    pub fn manual_home(
        &self,
        toolhead: &Rc<dyn HomingToolhead>,
        endstops: Vec<(EndstopRef, String)>,
        pos: &[f64],
        speed: f64,
        triggered: bool,
        check_triggered: bool,
    ) -> KResult<()> {
        let hmove = HomingMove::new(&self.printer, endstops, Some(toolhead.clone()))?;
        match hmove.homing_move(pos, speed, false, triggered, check_triggered) {
            Err(e) if e.is_command_error() => {
                if self.printer.is_shutdown() {
                    return Err(KError::command("Homing failed due to printer shutdown"));
                }
                Err(e)
            }
            r => r.map(|_| ()),
        }
    }

    pub fn probing_move(&self, mcu_probe: EndstopRef, pos: &[f64], speed: f64) -> KResult<Vec<f64>> {
        let endstops = vec![(mcu_probe, "probe".to_string())];
        let hmove = HomingMove::new(&self.printer, endstops, None)?;
        let epos = match hmove.homing_move(pos, speed, true, true, true) {
            Err(e) if e.is_command_error() => {
                if self.printer.is_shutdown() {
                    return Err(KError::command("Probing failed due to printer shutdown"));
                }
                return Err(e);
            }
            r => r?,
        };
        if hmove.check_no_movement().is_some() {
            return Err(KError::command("Probe triggered prior to movement"));
        }
        Ok(epos)
    }

    fn cmd_g28(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Move to origin
        let mut axes = vec![];
        for (pos, axis) in ["X", "Y", "Z"].iter().enumerate() {
            if gcmd.get_opt(axis).is_some() {
                axes.push(pos);
            }
        }
        if axes.is_empty() {
            axes = vec![0, 1, 2];
        }
        let homing_state = Homing::new(&self.printer)?;
        homing_state.set_axes(axes);
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        match kin.home(&homing_state) {
            Err(e) if e.is_command_error() => {
                if self.printer.is_shutdown() {
                    return Err(KError::command("Homing failed due to printer shutdown"));
                }
                crate::extras::stepper_enable::motor_off(&self.printer)?;
                Err(e)
            }
            r => r,
        }
    }
}

impl PrinterObject for PrinterHoming {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let dopts = get_danger_options();
    HOMING_START_DELAY.with(|v| v.set(dopts.homing_start_delay));
    ENDSTOP_SAMPLE_TIME.with(|v| v.set(dopts.endstop_sample_time));
    ENDSTOP_SAMPLE_COUNT.with(|v| v.set(dopts.endstop_sample_count));
    Ok(PrinterHoming::new(config)?)
}
