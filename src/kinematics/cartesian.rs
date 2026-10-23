// Code for handling the kinematics of cartesian robots
// (port of klippy/kinematics/cartesian.py)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Ported so far: everything except [dual_carriage] (idex_modes.py), which
// raises a config error.
use super::Kinematics;
use crate::chelper::kin_cartesian::cartesian_stepper_alloc;
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::Homing;
use crate::gcode::coord_pyval;
use crate::pyval::PyVal;
use crate::stepper::{lookup_multi_rail, McuStepper, PrinterRail, Rail, RailRef};
use crate::toolhead::{Move, ToolHead};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub struct CartKinematics {
    rails: Vec<Rc<PrinterRail>>,
    axes_min: [f64; 4],
    axes_max: [f64; 4],
    max_z_velocity: Cell<f64>,
    max_z_accel: Cell<f64>,
    limits: RefCell<[(f64, f64); 3]>,
}

impl CartKinematics {
    pub fn new(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Self> {
        // Setup axis rails
        let mut rails = vec![];
        for n in ["x", "y", "z"] {
            rails.push(lookup_multi_rail(&config.getsection(&format!("stepper_{}", n)), true, None, false)?);
        }
        for (rail, axis) in rails.iter().zip(['x', 'y', 'z']) {
            rail.setup_itersolve(|| cartesian_stepper_alloc(axis).expect("axis"));
        }
        let ranges: Vec<(f64, f64)> = rails.iter().map(|r| r.get_range()).collect();
        let axes_min = [ranges[0].0, ranges[1].0, ranges[2].0, 0.0];
        let axes_max = [ranges[0].1, ranges[1].1, ranges[2].1, 0.0];
        if config.has_section("dual_carriage") {
            return Err(KError::config("[dual_carriage] (idex_modes) is not ported to Kaliru yet"));
        }
        let kin = CartKinematics {
            rails,
            axes_min,
            axes_max,
            max_z_velocity: Cell::new(0.0),
            max_z_accel: Cell::new(0.0),
            limits: RefCell::new([(1.0, -1.0); 3]),
        };
        for s in kin.get_steppers() {
            s.set_trapq(Some(toolhead.get_trapq()));
        }
        // Setup boundary checks
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        kin.max_z_velocity.set(config.getfloat("max_z_velocity", max_velocity, Lim::new().above(0.0).max(max_velocity))?);
        kin.max_z_accel.set(config.getfloat("max_z_accel", max_accel, Lim::new().above(0.0).max(max_accel))?);
        Ok(kin)
    }

    fn home_axis(&self, homing_state: &Rc<Homing>, axis: usize, rail: &Rc<PrinterRail>) -> KResult<()> {
        // Determine movement
        let (position_min, position_max) = rail.get_range();
        let hi = rail.get_homing_info();
        let mut homepos = [None, None, None, None];
        homepos[axis] = Some(hi.position_endstop);
        let mut forcepos = homepos;
        if hi.positive_dir {
            forcepos[axis] = Some(hi.position_endstop - 1.5 * (hi.position_endstop - position_min));
        } else {
            forcepos[axis] = Some(hi.position_endstop + 1.5 * (position_max - hi.position_endstop));
        }
        // Perform homing
        let r: RailRef = rail.clone();
        homing_state.home_rails(&[r], &forcepos, &homepos)
    }

    fn check_endstops(&self, mv: &Move) -> KResult<()> {
        let end_pos = &mv.end_pos;
        let limits = self.limits.borrow();
        for i in 0..3 {
            if mv.axes_d[i] != 0.0 && (end_pos[i] < limits[i].0 || end_pos[i] > limits[i].1) {
                if limits[i].0 > limits[i].1 {
                    return Err(mv.move_error("Must home axis first"));
                }
                return Err(mv.move_error("Move out of range"));
            }
        }
        Ok(())
    }
}

impl Kinematics for CartKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.rails.iter().flat_map(|r| r.get_steppers()).collect()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        self.rails.iter().map(|r| stepper_positions[&r.get_name(false)]).collect()
    }

    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        for rail in &self.rails {
            rail.set_position(newpos);
        }
        for axis_name in homing_axes.chars() {
            let axis = "xyz".find(axis_name).ok_or_else(|| KError::Other("substring not found".into()))?;
            self.limits.borrow_mut()[axis] = self.rails[axis].get_range();
        }
        Ok(())
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        for (axis, axis_name) in "xyz".chars().enumerate() {
            if clear_axes.contains(axis_name) {
                self.limits.borrow_mut()[axis] = (1.0, -1.0);
            }
        }
    }

    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()> {
        // Each axis is homed independently and in order
        for axis in homing_state.get_axes() {
            self.home_axis(homing_state, axis, &self.rails[axis])?;
        }
        Ok(())
    }

    fn check_move(&self, mv: &mut Move) -> KResult<()> {
        let (xpos, ypos) = (mv.end_pos[0], mv.end_pos[1]);
        {
            let limits = self.limits.borrow();
            if xpos < limits[0].0 || xpos > limits[0].1 || ypos < limits[1].0 || ypos > limits[1].1 {
                drop(limits);
                self.check_endstops(mv)?;
            }
        }
        if mv.axes_d[2] == 0.0 {
            // Normal XY move - use defaults
            return Ok(());
        }
        // Move with Z - update velocity and accel for slower Z axis
        self.check_endstops(mv)?;
        let z_ratio = mv.move_d / mv.axes_d[2].abs();
        mv.limit_speed(self.max_z_velocity.get() * z_ratio, self.max_z_accel.get() * z_ratio);
        Ok(())
    }

    fn rails(&self) -> Vec<RailRef> {
        self.rails.iter().map(|r| r.clone() as RailRef).collect()
    }

    fn get_status(&self, _eventtime: f64) -> IndexMap<String, PyVal> {
        let limits = self.limits.borrow();
        let axes: String = "xyz".chars().zip(limits.iter()).filter(|(_, (l, h))| l <= h).map(|(a, _)| a).collect();
        let mut d = IndexMap::new();
        d.insert("homed_axes".to_string(), PyVal::Str(axes));
        d.insert("axis_minimum".to_string(), coord_pyval(&self.axes_min));
        d.insert("axis_maximum".to_string(), coord_pyval(&self.axes_max));
        d
    }

    fn supports_dual_carriage(&self) -> bool {
        true
    }

    fn limit(&self, name: &str) -> Option<&Cell<f64>> {
        match name {
            "max_z_velocity" => Some(&self.max_z_velocity),
            "max_z_accel" => Some(&self.max_z_accel),
            _ => None,
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_kinematics(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(CartKinematics::new(toolhead, config)?))
}
