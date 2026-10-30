// Code for handling the kinematics of polar robots
// (port of klippy/kinematics/polar.py)
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::Kinematics;
use crate::chelper::kin_cartesian::cartesian_stepper_alloc;
use crate::chelper::kin_polar::polar_stepper_alloc;
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::homing::Homing;
use crate::gcode::coord_pyval;
use crate::pyval::{PyDict, PyVal};
use crate::stepper::{lookup_multi_rail, printer_stepper, McuStepper, PrinterRail, Rail, RailRef};
use crate::toolhead::{Move, ToolHead};
use crate::IndexMap;
use std::cell::Cell;
use std::rc::Rc;

pub struct PolarKinematics {
    pub rails: [Rc<PrinterRail>; 2],
    pub steppers: Vec<Rc<McuStepper>>,
    pub max_z_velocity: Cell<f64>,
    pub max_z_accel: Cell<f64>,
    pub limit_z: Cell<(f64, f64)>,
    pub limit_xy2: Cell<f64>,
    pub axes_min: [f64; 4],
    pub axes_max: [f64; 4],
}

impl PolarKinematics {
    pub fn new(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Self> {
        // Setup axis steppers
        let stepper_bed = printer_stepper(&config.getsection("stepper_bed"), true)?;
        let rail_arm = PrinterRail::new(&config.getsection("stepper_arm"), true, None, false)?;
        let rail_z = lookup_multi_rail(&config.getsection("stepper_z"), true, None, false)?;
        stepper_bed.setup_itersolve(polar_stepper_alloc('a').expect("polar"));
        rail_arm.setup_itersolve(|| polar_stepper_alloc('r').expect("polar"));
        rail_z.setup_itersolve(|| cartesian_stepper_alloc('z').expect("axis"));
        let rails = [rail_arm, rail_z];
        let mut steppers = vec![stepper_bed];
        steppers.extend(rails.iter().flat_map(|r| r.get_steppers()));
        for s in &steppers {
            s.set_trapq(Some(toolhead.get_trapq()));
        }
        // Setup boundary checks
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let max_z_velocity = config.getfloat("max_z_velocity", max_velocity, Lim::new().above(0.0).max(max_velocity))?;
        let max_z_accel = config.getfloat("max_z_accel", max_accel, Lim::new().above(0.0).max(max_accel))?;
        let max_xy = rails[0].get_range().1;
        let (min_z, max_z) = rails[1].get_range();
        Ok(PolarKinematics {
            rails,
            steppers,
            max_z_velocity: Cell::new(max_z_velocity),
            max_z_accel: Cell::new(max_z_accel),
            limit_z: Cell::new((1.0, -1.0)),
            limit_xy2: Cell::new(-1.0),
            axes_min: [-max_xy, -max_xy, min_z, 0.0],
            axes_max: [max_xy, max_xy, max_z, 0.0],
        })
    }

    pub fn home_axis(&self, homing_state: &Rc<Homing>, axis: usize, rail: &Rc<PrinterRail>) -> KResult<()> {
        // Determine movement
        let (position_min, position_max) = rail.get_range();
        let hi = rail.get_homing_info();
        let mut homepos = [None, None, None, None];
        homepos[axis] = Some(hi.position_endstop);
        if axis == 0 {
            homepos[1] = Some(0.0);
        }
        let mut forcepos = homepos;
        if hi.positive_dir {
            forcepos[axis] = Some(hi.position_endstop - (hi.position_endstop - position_min));
        } else {
            forcepos[axis] = Some(hi.position_endstop + (position_max - hi.position_endstop));
        }
        // Perform homing
        let r: RailRef = rail.clone();
        homing_state.home_rails(&[r], &forcepos, &homepos)
    }
}

impl Kinematics for PolarKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.steppers.clone()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        let bed_angle = stepper_positions[&self.steppers[0].get_name(false)];
        let arm_pos = stepper_positions[&self.rails[0].get_name(false)];
        let z_pos = stepper_positions[&self.rails[1].get_name(false)];
        vec![bed_angle.cos() * arm_pos, bed_angle.sin() * arm_pos, z_pos]
    }

    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        for s in &self.steppers {
            s.set_position(newpos);
        }
        if homing_axes.contains('z') {
            self.limit_z.set(self.rails[1].get_range());
        }
        if homing_axes.contains('x') && homing_axes.contains('y') {
            let r = self.rails[0].get_range().1;
            self.limit_xy2.set(r * r);
        }
        Ok(())
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        if clear_axes.contains('x') || clear_axes.contains('y') {
            // X and Y cannot be cleared separately
            self.limit_xy2.set(-1.0);
        }
        if clear_axes.contains('z') {
            self.limit_z.set((1.0, -1.0));
        }
    }

    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()> {
        // Always home XY together
        let homing_axes = homing_state.get_axes();
        let home_xy = homing_axes.contains(&0) || homing_axes.contains(&1);
        let home_z = homing_axes.contains(&2);
        let mut updated_axes = vec![];
        if home_xy {
            updated_axes = vec![0, 1];
        }
        if home_z {
            updated_axes.push(2);
        }
        homing_state.set_axes(updated_axes);
        // Do actual homing
        if home_xy {
            self.home_axis(homing_state, 0, &self.rails[0])?;
        }
        if home_z {
            self.home_axis(homing_state, 2, &self.rails[1])?;
        }
        Ok(())
    }

    fn check_move(&self, mv: &mut Move) -> KResult<()> {
        let end_pos = &mv.end_pos;
        let xy2 = end_pos[0] * end_pos[0] + end_pos[1] * end_pos[1];
        let limit_xy2 = self.limit_xy2.get();
        if xy2 > limit_xy2 {
            if limit_xy2 < 0.0 {
                return Err(mv.move_error("Must home axis first"));
            }
            return Err(mv.move_error("Move out of range"));
        }
        if mv.axes_d[2] != 0.0 {
            let limit_z = self.limit_z.get();
            if end_pos[2] < limit_z.0 || end_pos[2] > limit_z.1 {
                if limit_z.0 > limit_z.1 {
                    return Err(mv.move_error("Must home axis first"));
                }
                return Err(mv.move_error("Move out of range"));
            }
            // Move with Z - update velocity and accel for slower Z axis
            let z_ratio = mv.move_d / mv.axes_d[2].abs();
            mv.limit_speed(self.max_z_velocity.get() * z_ratio, self.max_z_accel.get() * z_ratio);
        }
        Ok(())
    }

    fn rails(&self) -> Vec<RailRef> {
        self.rails.iter().map(|r| r.clone() as RailRef).collect()
    }

    fn get_status(&self, _eventtime: f64) -> PyDict {
        let xy_home = if self.limit_xy2.get() >= 0.0 { "xy" } else { "" };
        let limit_z = self.limit_z.get();
        let z_home = if limit_z.0 <= limit_z.1 { "z" } else { "" };
        let d = crate::pydict! {
            "homed_axes" => PyVal::Str(format!("{}{}", xy_home, z_home)),
            "axis_minimum" => coord_pyval(&self.axes_min),
            "axis_maximum" => coord_pyval(&self.axes_max),
        };
        d
    }

    fn limit(&self, name: &str) -> Option<&Cell<f64>> {
        match name {
            "max_z_velocity" => Some(&self.max_z_velocity),
            "max_z_accel" => Some(&self.max_z_accel),
            _ => None,
        }
    }

}

pub fn load_kinematics(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(PolarKinematics::new(toolhead, config)?))
}
