// Support for duplication and mirroring modes for IDEX printers
// (port of klippy/kinematics/idex_modes.py)
//
// Copyright (C) 2021  Fabrice Gallet <tircown@gmail.com>
// Copyright (C) 2023  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::itersolve::SkRef;
use crate::chelper::kin_idex::{dual_carriage_alloc, dual_carriage_set_sk, dual_carriage_set_transform};
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::Homing;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_sum};
use crate::pyval::{PyDict, PyVal};
use crate::stepper::{PrinterRail, Rail};
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const INACTIVE: &str = "INACTIVE";
pub const PRIMARY: &str = "PRIMARY";
pub const COPY: &str = "COPY";
pub const MIRROR: &str = "MIRROR";

const VALID_MODES: [&str; 3] = [PRIMARY, COPY, MIRROR];

struct SavedState {
    carriage_modes: Vec<&'static str>,
    axes_positions: Vec<f64>,
}

pub struct DualCarriages {
    pub printer: Rc<Printer>,
    pub axis: usize,
    pub dc: [Rc<DualCarriagesRail>; 2],
    saved_states: RefCell<IndexMap<String, SavedState>>,
    pub safe_dist: f64,
}

fn mode_of(s: &str) -> &'static str {
    match s {
        PRIMARY => PRIMARY,
        COPY => COPY,
        MIRROR => MIRROR,
        _ => INACTIVE,
    }
}

impl DualCarriages {
    pub fn new(dc_config: &ConfigWrapper, rail_0: Rc<DualCarriagesRail>, rail_1: Rc<DualCarriagesRail>, axis: usize) -> KResult<Rc<Self>> {
        let printer = dc_config.get_printer();
        let safe_dist = match dc_config.getfloat_opt("safe_distance", Lim::new().min(0.0))? {
            Some(d) => d,
            None => {
                let (dc0_rail, dc1_rail) = (rail_0.get_rail(), rail_1.get_rail());
                py_min(
                    (dc0_rail.position_min.get() - dc1_rail.position_min.get()).abs(),
                    (dc0_rail.position_max.get() - dc1_rail.position_max.get()).abs(),
                )
            }
        };
        let d = Rc::new(DualCarriages {
            printer: printer.clone(),
            axis,
            dc: [rail_0, rail_1],
            saved_states: RefCell::new(IndexMap::default()),
            safe_dist,
        });
        printer.add_object("dual_carriage", d.clone())?;
        let w = Rc::downgrade(&d);
        crate::events::KlippyReady::on(&printer, move || {
            if let Some(d) = w.upgrade() {
                d.handle_ready();
            }
            Ok(())
        });
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        type DCmd = fn(&DualCarriages, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, DCmd, &str); 3] = [
            ("SET_DUAL_CARRIAGE", DualCarriages::cmd_set_dual_carriage, "Configure the dual carriages mode"),
            ("SAVE_DUAL_CARRIAGE_STATE", DualCarriages::cmd_save_dual_carriage_state, "Save dual carriages modes and positions"),
            (
                "RESTORE_DUAL_CARRIAGE_STATE",
                DualCarriages::cmd_restore_dual_carriage_state,
                "Restore dual carriages modes and positions",
            ),
        ];
        for (name, f, desc) in cmds {
            let w = Rc::downgrade(&d);
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(d) => f(&d, g),
                    None => Ok(()),
                })),
                false,
                Some(desc),
            )?;
        }
        Ok(d)
    }

    pub fn get_rails(&self) -> &[Rc<DualCarriagesRail>; 2] {
        &self.dc
    }

    pub fn get_primary_rail(&self) -> Option<Rc<DualCarriagesRail>> {
        self.dc.iter().find(|r| r.mode.get() == PRIMARY).cloned()
    }

    /// get_primary_rail().get_rail() (python raises AttributeError on None)
    pub fn primary_printer_rail(&self) -> KResult<Rc<PrinterRail>> {
        self.get_primary_rail()
            .map(|r| r.get_rail())
            .ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_rail'".into()))
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.printer.lookup::<ToolHead>("toolhead")
    }

    pub fn toggle_active_dc_rail(&self, index: usize) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let pos = toolhead.get_position();
        let kin = toolhead.get_kinematics();
        for (i, dc) in self.dc.iter().enumerate() {
            if i != index && dc.is_active() {
                dc.inactivate(&pos);
            }
        }
        let target_dc = &self.dc[index];
        if target_dc.mode.get() != PRIMARY {
            let mut newpos = pos.clone();
            newpos[self.axis] = target_dc.get_axis_position(&pos);
            target_dc.activate(PRIMARY, &newpos, Some(&pos));
            toolhead.set_position(&newpos, "")?;
        }
        kin.update_limits(self.axis, target_dc.get_rail().get_range());
        Ok(())
    }

    pub fn home(&self, homing_state: &Rc<Homing>) -> KResult<()> {
        let kin = self.toolhead()?.get_kinematics();
        let mut enumerated_dcs: Vec<(usize, Rc<DualCarriagesRail>)> = self.dc.iter().cloned().enumerate().collect();
        if (self.get_dc_order(0, 1) > 0) != self.dc[0].get_rail().get_homing_info().positive_dir {
            // The second carriage must home first, because the carriages home in
            // the same direction and the first carriage homes on the second one
            enumerated_dcs.reverse();
        }
        for (i, dc_rail) in enumerated_dcs {
            self.toggle_active_dc_rail(i)?;
            kin.home_axis(homing_state, self.axis, &dc_rail.get_rail())?;
        }
        // Restore the original rails ordering
        self.toggle_active_dc_rail(0)
    }

    pub fn get_kin_range(&self, toolhead: &ToolHead, mode: &str) -> (f64, f64) {
        let pos = toolhead.get_position();
        let axes_pos: Vec<f64> = self.dc.iter().map(|dc| dc.get_axis_position(&pos)).collect();
        let (dc0_rail, dc1_rail) = (self.dc[0].get_rail(), self.dc[1].get_rail());
        let (mut range_min, mut range_max) = if mode != PRIMARY || self.dc[0].is_active() {
            (dc0_rail.position_min.get(), dc0_rail.position_max.get())
        } else {
            (dc1_rail.position_min.get(), dc1_rail.position_max.get())
        };
        let safe_dist = self.safe_dist;
        if safe_dist == 0.0 {
            return (range_min, range_max);
        }
        let sum_pos = py_sum(axes_pos.iter().copied());
        if mode == COPY {
            range_min = py_max(range_min, axes_pos[0] - axes_pos[1] + dc1_rail.position_min.get());
            range_max = py_min(range_max, axes_pos[0] - axes_pos[1] + dc1_rail.position_max.get());
        } else if mode == MIRROR {
            if self.get_dc_order(0, 1) > 0 {
                range_min = py_max(range_min, 0.5 * (sum_pos + safe_dist));
                range_max = py_min(range_max, sum_pos - dc1_rail.position_min.get());
            } else {
                range_max = py_min(range_max, 0.5 * (sum_pos - safe_dist));
                range_min = py_max(range_min, sum_pos - dc1_rail.position_max.get());
            }
        } else {
            // mode == PRIMARY
            let active_idx = if self.dc[1].is_active() { 1 } else { 0 };
            let inactive_idx = 1 - active_idx;
            if self.get_dc_order(active_idx, inactive_idx) > 0 {
                range_min = py_max(range_min, axes_pos[inactive_idx] + safe_dist);
            } else {
                range_max = py_min(range_max, axes_pos[inactive_idx] - safe_dist);
            }
        }
        if range_min > range_max {
            // During multi-MCU homing it is possible that the carriage
            // position will end up below position_min or above position_max
            // if position_endstop is too close to the rail motion ends due
            // to inherent latencies of the data transmission between MCUs.
            // This can result in an invalid range_min > range_max range
            // in certain modes, which may confuse the kinematics code.
            // So, return an empty range instead, which will correctly
            // block the carriage motion until a different mode is selected
            // which actually permits carriage motion.
            return (range_min, range_min);
        }
        (range_min, range_max)
    }

    pub fn get_dc_order(&self, first: usize, second: usize) -> i64 {
        if first == second {
            return 0;
        }
        // Check the relative order of the first and second carriages and
        // return -1 if the first carriage position is always smaller
        // than the second one and 1 otherwise
        let first_rail = self.dc[first].get_rail();
        let second_rail = self.dc[second].get_rail();
        let first_homing_info = first_rail.get_homing_info();
        let second_homing_info = second_rail.get_homing_info();
        if first_homing_info.positive_dir != second_homing_info.positive_dir {
            // Carriages home away from each other
            return if first_homing_info.positive_dir { 1 } else { -1 };
        }
        // Carriages home in the same direction
        if first_rail.position_endstop.get() > second_rail.position_endstop.get() {
            return 1;
        }
        -1
    }

    fn activate_dc_mode(&self, index: usize, mode: &'static str) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let kin = toolhead.get_kinematics();
        if mode == INACTIVE {
            self.dc[index].inactivate(&toolhead.get_position());
        } else if mode == PRIMARY {
            self.toggle_active_dc_rail(index)?;
        } else {
            self.toggle_active_dc_rail(0)?;
            self.dc[index].activate(mode, &toolhead.get_position(), None);
        }
        kin.update_limits(self.axis, self.get_kin_range(&toolhead, mode));
        Ok(())
    }

    pub fn handle_ready(&self) {
        // Apply the transform later during Klipper initialization to make sure
        // that input shaping can pick up the correct stepper kinematic flags.
        for dc in &self.dc {
            dc.apply_transform();
        }
    }

    pub fn cmd_set_dual_carriage(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let index = gcmd.get_int("CARRIAGE", None, Lim::new().min(0).max(1))? as usize;
        let mode = gcmd.get_or("MODE", PRIMARY).to_uppercase();
        if !VALID_MODES.contains(&mode.as_str()) {
            return Err(gcmd.error(format!("Invalid mode={} specified", mode)));
        }
        let mode = mode_of(&mode);
        if mode == COPY || mode == MIRROR {
            if index == 0 {
                return Err(gcmd.error(format!("Mode={} is not supported for carriage=0", mode)));
            }
            let curtime = self.printer.get_reactor().monotonic();
            let kin = self.toolhead()?.get_kinematics();
            let axis = &"xyz"[self.axis..self.axis + 1];
            let homed = match kin.get_status(curtime).get("homed_axes") {
                Some(PyVal::Str(s)) => s.clone(),
                _ => String::new(),
            };
            if !homed.contains(axis) {
                return Err(gcmd.error(format!("Axis {} must be homed prior to enabling mode={}", axis, mode)));
            }
        }
        self.activate_dc_mode(index, mode)
    }

    pub fn cmd_save_dual_carriage_state(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let state_name = gcmd.get_or("NAME", "default");
        let pos = self.toolhead()?.get_position();
        let state = SavedState {
            carriage_modes: self.dc.iter().map(|dc| dc.mode.get()).collect(),
            axes_positions: self.dc.iter().map(|dc| dc.get_axis_position(&pos)).collect(),
        };
        self.saved_states.borrow_mut().insert(state_name, state);
        Ok(())
    }

    pub fn cmd_restore_dual_carriage_state(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let state_name = gcmd.get_or("NAME", "default");
        let (saved_modes, saved_positions) = match self.saved_states.borrow().get(&state_name) {
            Some(s) => (s.carriage_modes.clone(), s.axes_positions.clone()),
            None => return Err(gcmd.error(format!("Unknown DUAL_CARRIAGE state: {}", state_name))),
        };
        let move_speed = gcmd.get_float("MOVE_SPEED", Some(0.0), Lim::new().above(0.0))?;
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        if gcmd.get_int("MOVE", Some(1), Lim::new())? != 0 {
            let mut homing_speed = 99999999.0;
            let mut cur_pos = vec![];
            for (i, dc) in self.dc.iter().enumerate() {
                self.toggle_active_dc_rail(i)?;
                homing_speed = py_min(homing_speed, dc.get_rail().get_homing_info().speed);
                cur_pos.push(toolhead.get_position());
            }
            let mut move_pos = cur_pos[0].clone();
            let dl: Vec<f64> = (0..2).map(|i| saved_positions[i] - cur_pos[i][self.axis]).collect();
            let primary_ind = if dl[0].abs() >= dl[1].abs() { 0 } else { 1 };
            self.toggle_active_dc_rail(primary_ind)?;
            move_pos[self.axis] = saved_positions[primary_ind];
            let dc_mode = if py_min(dl[0].abs(), dl[1].abs()) < 0.000000001 {
                INACTIVE
            } else if dl[0] * dl[1] > 0.0 {
                COPY
            } else {
                MIRROR
            };
            if dc_mode != INACTIVE {
                let other = &self.dc[1 - primary_ind];
                other.activate(dc_mode, &cur_pos[primary_ind], None);
                if dl[primary_ind] == 0.0 {
                    return Err(KError::Other("float division by zero".into()));
                }
                other.override_axis_scaling((dl[1 - primary_ind] / dl[primary_ind]).abs(), &cur_pos[primary_ind]);
            }
            let speed = if move_speed != 0.0 { move_speed } else { homing_speed };
            let coord: Vec<Option<f64>> = move_pos.iter().map(|v| Some(*v)).collect();
            toolhead.manual_move(&coord, speed)?;
            toolhead.flush_step_generation()?;
            // Make sure the scaling coefficients are restored with the mode
            self.dc[0].inactivate(&move_pos);
            self.dc[1].inactivate(&move_pos);
        }
        for (i, saved_mode) in saved_modes.into_iter().enumerate() {
            self.activate_dc_mode(i, saved_mode)?;
        }
        Ok(())
    }
}

impl PrinterObject for DualCarriages {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let d: PyDict =
            self.dc.iter().enumerate().map(|(i, dc)| (format!("carriage_{}", i).into(), PyVal::from(dc.mode.get()))).collect();
        Some(PyVal::Dict(d))
    }
}

const ENC_AXES: [char; 2] = ['x', 'y'];

pub struct DualCarriagesRail {
    pub rail: Rc<PrinterRail>,
    pub axis: usize,
    pub mode: Cell<&'static str>,
    pub offset: Cell<f64>,
    pub scale: Cell<f64>,
    pub dc_stepper_kinematics: Vec<SkRef>,
}

impl DualCarriagesRail {
    pub fn new(rail: &Rc<PrinterRail>, axis: usize, active: bool) -> Rc<Self> {
        let mut dc_sks = vec![];
        for s in rail.get_steppers() {
            let sk = dual_carriage_alloc();
            let orig_sk = s.get_stepper_kinematics().expect("stepper kinematics");
            dual_carriage_set_sk(&sk, &orig_sk);
            // Set the default transform for the other axis
            dual_carriage_set_transform(&sk, ENC_AXES[1 - axis], 1.0, 0.0);
            dc_sks.push(sk.clone());
            s.set_stepper_kinematics(Some(sk));
        }
        Rc::new(DualCarriagesRail {
            rail: rail.clone(),
            axis,
            mode: Cell::new(if active { PRIMARY } else { INACTIVE }),
            offset: Cell::new(0.0),
            scale: Cell::new(if active { 1.0 } else { 0.0 }),
            dc_stepper_kinematics: dc_sks,
        })
    }

    pub fn get_rail(&self) -> Rc<PrinterRail> {
        self.rail.clone()
    }

    pub fn is_active(&self) -> bool {
        self.mode.get() != INACTIVE
    }

    pub fn get_axis_position(&self, position: &[f64]) -> f64 {
        position[self.axis] * self.scale.get() + self.offset.get()
    }

    pub fn apply_transform(&self) {
        for sk in &self.dc_stepper_kinematics {
            dual_carriage_set_transform(sk, ENC_AXES[self.axis], self.scale.get(), self.offset.get());
        }
    }

    pub fn activate(&self, mode: &'static str, position: &[f64], old_position: Option<&[f64]>) {
        let old_axis_position = self.get_axis_position(old_position.unwrap_or(position));
        self.scale.set(if mode == MIRROR { -1.0 } else { 1.0 });
        self.offset.set(old_axis_position - position[self.axis] * self.scale.get());
        self.apply_transform();
        self.mode.set(mode);
    }

    pub fn inactivate(&self, position: &[f64]) {
        self.offset.set(self.get_axis_position(position));
        self.scale.set(0.0);
        self.apply_transform();
        self.mode.set(INACTIVE);
    }

    pub fn override_axis_scaling(&self, new_scale: f64, position: &[f64]) {
        let old_axis_position = self.get_axis_position(position);
        self.scale.set(new_scale.copysign(self.scale.get()));
        self.offset.set(old_axis_position - position[self.axis] * self.scale.get());
        self.apply_transform();
    }
}
