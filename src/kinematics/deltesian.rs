// Code for handling the kinematics of deltesian robots
// (port of klippy/kinematics/deltesian.py)
//
// Copyright (C) 2022  Fabrice Gallet <tircown@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::Kinematics;
use crate::chelper::kin_cartesian::cartesian_stepper_alloc;
use crate::chelper::kin_deltesian::deltesian_stepper_alloc;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::Homing;
use crate::logging;
use crate::pyfmt::{py_max, py_min, py_sum};
use crate::pyval::PyVal;
use crate::stepper::{lookup_multi_rail, McuStepper, PrinterRail, Rail, RailRef};
use crate::toolhead::{Move, ToolHead};
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

// Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
const SLOW_RATIO: f64 = 3.0;

// Minimum angle with the horizontal for the arm to not exceed - in degrees
const MIN_ANGLE: f64 = 5.0;

fn min_of(v: &[f64]) -> f64 {
    v.iter().copied().reduce(py_min).unwrap_or(f64::NAN)
}

pub struct DeltesianKinematics {
    rails: Vec<Rc<PrinterRail>>,
    arm_x: [f64; 2],
    arm2: [f64; 2],
    limits: [(f64, f64); 3],
    abs_endstop: [f64; 2],
    home_z: f64,
    slow_x2: Option<f64>,
    very_slow_x2: Option<f64>,
    max_z_velocity: Cell<f64>,
    max_z_accel: Cell<f64>,
    axes_min: PyVal,
    axes_max: PyVal,
    homed_axis: RefCell<[bool; 3]>,
}

/// a python number: an int where python computed one
fn num(v: f64, is_int: bool) -> PyVal {
    if is_int {
        PyVal::Int(v as i64)
    } else {
        PyVal::Float(v)
    }
}

impl DeltesianKinematics {
    fn new(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Self> {
        let stepper_configs: Vec<ConfigWrapper> =
            ["left", "right", "y"].iter().map(|s| config.getsection(&format!("stepper_{}", s))).collect();
        let rail0 = PrinterRail::new(&stepper_configs[0], false, None, false)?;
        let def_pos_es = rail0.get_homing_info().position_endstop;
        let rail1 = PrinterRail::new(&stepper_configs[1], false, Some(def_pos_es), false)?;
        let rail2 = lookup_multi_rail(&stepper_configs[2], true, None, false)?;
        let rails = vec![rail0, rail1, rail2];
        let ax0 = stepper_configs[0].getfloat("arm_x_length", Dflt::Required, Lim::new().above(0.0))?;
        let ax1 = stepper_configs[1].getfloat("arm_x_length", ax0, Lim::new().above(0.0))?;
        let arm_x = [ax0, ax1];
        let a0 = stepper_configs[0].getfloat("arm_length", Dflt::Required, Lim::new().above(arm_x[0]))?;
        let a1 = stepper_configs[1].getfloat("arm_length", a0, Lim::new().above(arm_x[1]))?;
        let arm = [a0, a1];
        let arm2 = [arm[0].powf(2.0), arm[1].powf(2.0)];
        let (a2_0, a2_1) = (arm2[0], arm2[1]);
        rails[0].setup_itersolve(|| deltesian_stepper_alloc(a2_0, -ax0));
        rails[1].setup_itersolve(|| deltesian_stepper_alloc(a2_1, ax1));
        rails[2].setup_itersolve(|| cartesian_stepper_alloc('y').expect("axis"));
        for rail in &rails {
            for s in rail.get_steppers() {
                s.set_trapq(Some(toolhead.get_trapq()));
            }
        }
        let mut limits = [(1.0, -1.0); 3];
        // X axis limits
        let min_angle = config.getfloat("min_angle", MIN_ANGLE, Lim::new().min(0.0).max(90.0))?;
        let cos_angle = min_angle.to_radians().cos();
        let x_kin_min = (-py_min(arm_x[0], cos_angle * arm[1] - arm_x[1])).ceil();
        let x_kin_max = py_min(arm_x[1], cos_angle * arm[0] - arm_x[0]).floor();
        let x_kin_range = (x_kin_max - x_kin_min).min(x_kin_max * 2.0).min(-x_kin_min * 2.0);
        let print_width = config.getfloat_opt("print_width", Lim::new().min(0.0).max(x_kin_range as i64))?;
        let x_is_int = match print_width {
            Some(pw) if pw != 0.0 => {
                limits[0] = (-pw * 0.5, pw * 0.5);
                false
            }
            _ => {
                limits[0] = (x_kin_min, x_kin_max);
                true
            }
        };
        // Y axis limits
        limits[1] = rails[2].get_range();
        // Z axis limits
        let pmax: Vec<f64> = rails[..2].iter().map(|r| r.get_homing_info().position_endstop).collect();
        let abs_endstop = [pmax[0] + (arm2[0] - arm_x[0].powf(2.0)).sqrt(), pmax[1] + (arm2[1] - arm_x[1].powf(2.0)).sqrt()];
        let mut kin = DeltesianKinematics {
            rails,
            arm_x,
            arm2,
            limits,
            abs_endstop,
            home_z: 0.0,
            slow_x2: None,
            very_slow_x2: None,
            max_z_velocity: Cell::new(0.0),
            max_z_accel: Cell::new(0.0),
            axes_min: PyVal::None,
            axes_max: PyVal::None,
            homed_axis: RefCell::new([false; 3]),
        };
        kin.home_z = kin.actuator_to_cartesian(&kin.abs_endstop)[1];
        let z_max = py_min(kin.pillars_z_max(kin.limits[0].0), kin.pillars_z_max(kin.limits[0].1));
        // the default is python's int 0
        let z_min_value = config.getfloat_value("minimum_z_position", &Dflt::from(PyVal::Int(0)), &Lim::new().max(z_max))?.unwrap_or(PyVal::Int(0));
        let z_min = z_min_value.as_f64().unwrap_or(0.0);
        kin.limits[2] = (z_min, z_max);
        // Limit the speed/accel if is is at the extreme values of the x axis
        let slow_ratio = config.getfloat("slow_ratio", SLOW_RATIO, Lim::new().min(0.0))?;
        if slow_ratio > 0.0 {
            let sr2 = slow_ratio.powf(2.0);
            let s: Vec<f64> = arm2.iter().zip(&arm_x).map(|(a2, axl)| ((sr2 * a2) / (sr2 + 1.0)).sqrt() - axl).collect();
            let vs: Vec<f64> = arm2.iter().zip(&arm_x).map(|(a2, axl)| ((2.0 * sr2 * a2) / (2.0 * sr2 + 1.0)).sqrt() - axl).collect();
            let (slow_x2, very_slow_x2) = (min_of(&s).powf(2.0), min_of(&vs).powf(2.0));
            kin.slow_x2 = Some(slow_x2);
            kin.very_slow_x2 = Some(very_slow_x2);
            logging::info(&format!(
                "Deltesian kinematics: moves slowed past {:.2}mm and {:.2}mm",
                slow_x2.sqrt(),
                very_slow_x2.sqrt()
            ));
        }
        // Setup boundary checks
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        kin.max_z_velocity.set(config.getfloat("max_z_velocity", max_velocity, Lim::new().above(0.0).max(max_velocity))?);
        kin.max_z_accel.set(config.getfloat("max_z_accel", max_accel, Lim::new().above(0.0).max(max_accel))?);
        let l = kin.limits;
        let coord = |v: Vec<PyVal>| PyVal::NamedTuple("Coord", &["x", "y", "z", "e"], v);
        kin.axes_min = coord(vec![num(l[0].0, x_is_int), PyVal::Float(l[1].0), z_min_value, PyVal::Float(0.0)]);
        kin.axes_max = coord(vec![num(l[0].1, x_is_int), PyVal::Float(l[1].1), PyVal::Float(l[2].1), PyVal::Float(0.0)]);
        kin.set_position(&[0.0, 0.0, 0.0], "")?;
        Ok(kin)
    }

    fn actuator_to_cartesian(&self, sp: &[f64]) -> [f64; 2] {
        let (arm_x, arm2) = (self.arm_x, self.arm2);
        let (dx, dz) = (py_sum(arm_x.iter().copied()), sp[1] - sp[0]);
        let pivots = (dx.powf(2.0) + dz.powf(2.0)).sqrt();
        // Trilateration w/ reference frame along left to right pivots
        let xt = (pivots.powf(2.0) + arm2[0] - arm2[1]) / (2.0 * pivots);
        let zt = (arm2[0] - xt.powf(2.0)).sqrt();
        // Rotation and translation of the reference frame
        let x = xt * dx / pivots + zt * dz / pivots - arm_x[0];
        let z = xt * dz / pivots - zt * dx / pivots + sp[0];
        [x, z]
    }

    fn pillars_z_max(&self, x: f64) -> f64 {
        let (arm_x, arm2) = (self.arm_x, self.arm2);
        let dz = [(arm2[0] - (arm_x[0] + x).powf(2.0)).sqrt(), (arm2[1] - (arm_x[1] - x).powf(2.0)).sqrt()];
        py_min(self.abs_endstop[0] - dz[0], self.abs_endstop[1] - dz[1])
    }
}

impl Kinematics for DeltesianKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.rails.iter().flat_map(|r| r.get_steppers()).collect()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        let sp: Vec<f64> = self.rails.iter().map(|r| stepper_positions[&r.get_name(false)]).collect();
        let [x, z] = self.actuator_to_cartesian(&sp[..2]);
        vec![x, sp[2], z]
    }

    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        for rail in &self.rails {
            rail.set_position(newpos);
        }
        for axis_name in homing_axes.chars() {
            let axis = "xyz".find(axis_name).ok_or_else(|| KError::Other("substring not found".into()))?;
            self.homed_axis.borrow_mut()[axis] = true;
        }
        Ok(())
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        for (axis, axis_name) in "xyz".chars().enumerate() {
            if clear_axes.contains(axis_name) {
                self.homed_axis.borrow_mut()[axis] = false;
            }
        }
    }

    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()> {
        let homing_axes = homing_state.get_axes();
        let home_xz = homing_axes.contains(&0) || homing_axes.contains(&2);
        let home_y = homing_axes.contains(&1);
        let forceaxes = match (home_xz, home_y) {
            (true, true) => vec![0, 1, 2],
            (true, false) => vec![0, 2],
            (false, true) => vec![1],
            (false, false) => vec![],
        };
        homing_state.set_axes(forceaxes);
        let mut homepos = [None; 4];
        if home_xz {
            homing_state.set_axes(if home_y { vec![0, 1, 2] } else { vec![0, 2] });
            homepos[0] = Some(0.0);
            homepos[2] = Some(self.home_z);
            let mut forcepos = homepos;
            let dz2: Vec<f64> = self.arm2.iter().zip(&self.arm_x).map(|(a2, ax)| a2 - ax.powf(2.0)).collect();
            forcepos[2] = Some(-1.5 * py_max(dz2[0], dz2[1]).sqrt());
            let rails: Vec<RailRef> = self.rails[..2].iter().map(|r| r.clone() as RailRef).collect();
            homing_state.home_rails(&rails, &forcepos, &homepos)?;
        }
        if home_y {
            let (position_min, position_max) = self.rails[2].get_range();
            let hi = self.rails[2].get_homing_info();
            homepos[1] = Some(hi.position_endstop);
            let mut forcepos = homepos;
            if hi.positive_dir {
                forcepos[1] = Some(hi.position_endstop - 1.5 * (hi.position_endstop - position_min));
            } else {
                forcepos[1] = Some(hi.position_endstop + 1.5 * (position_max - hi.position_endstop));
            }
            let r: RailRef = self.rails[2].clone();
            homing_state.home_rails(&[r], &forcepos, &homepos)?;
        }
        Ok(())
    }

    fn check_move(&self, mv: &mut Move) -> KResult<()> {
        let mut limits = self.limits;
        let (spos, epos) = (mv.start_pos.clone(), mv.end_pos.clone());
        let mut homing_move = false;
        if epos[0] == 0.0 && epos[2] == self.home_z && mv.axes_d[1] == 0.0 {
            // Identify a homing move
            homing_move = true;
        } else if epos[2] > limits[2].1 {
            // Moves at the very top - adapt limits depending on the X position
            limits[2].1 = self.pillars_z_max(epos[0]);
        }
        for i in 0..3 {
            if mv.axes_d[i] == 0.0 {
                continue;
            }
            if !self.homed_axis.borrow()[i] {
                return Err(mv.move_error("Must home axis first"));
            }
            // Move out of range - verify not a homing move
            if (epos[i] < limits[i].0 || epos[i] > limits[i].1) && !homing_move {
                return Err(mv.move_error("Move out of range"));
            }
        }
        if mv.axes_d[2] != 0.0 {
            let z_ratio = mv.move_d / mv.axes_d[2].abs();
            mv.limit_speed(self.max_z_velocity.get() * z_ratio, self.max_z_accel.get() * z_ratio);
        }
        // Limit the speed/accel if is is at the extreme values of the x axis
        if let (Some(slow_x2), Some(very_slow_x2)) = (self.slow_x2, self.very_slow_x2) {
            if mv.axes_d[0] != 0.0 && slow_x2 != 0.0 && very_slow_x2 != 0.0 {
                let move_x2 = py_max(spos[0].powf(2.0), epos[0].powf(2.0));
                if move_x2 > very_slow_x2 || move_x2 > slow_x2 {
                    // python reads the unset self.max_velocity here
                    return Err(KError::Other("'DeltesianKinematics' object has no attribute 'max_velocity'".into()));
                }
            }
        }
        Ok(())
    }

    fn rails(&self) -> Vec<RailRef> {
        self.rails.iter().map(|r| r.clone() as RailRef).collect()
    }

    fn get_status(&self, _eventtime: f64) -> IndexMap<String, PyVal> {
        let axes: String = "xyz".chars().zip(self.homed_axis.borrow().iter()).filter(|(_, b)| **b).map(|(a, _)| a).collect();
        let mut d = IndexMap::new();
        d.insert("homed_axes".to_string(), PyVal::Str(axes));
        d.insert("axis_minimum".to_string(), self.axes_min.clone());
        d.insert("axis_maximum".to_string(), self.axes_max.clone());
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
    Ok(Rc::new(DeltesianKinematics::new(toolhead, config)?))
}
