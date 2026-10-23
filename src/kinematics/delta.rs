// Code for handling the kinematics of linear delta robots
// (port of klippy/kinematics/delta.py)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::Kinematics;
use crate::chelper::kin_delta::delta_stepper_alloc;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::Homing;
use crate::gcode::{coord_pyval, GCodeDispatch};
use crate::logging;
use crate::mathutil::{trilateration, MathError};
use crate::printer::{ConfigFileObject, Printer};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use crate::stepper::{lookup_multi_rail, McuStepper, PrinterRail, Rail, RailRef};
use crate::toolhead::{Move, ToolHead};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::Cell;
use std::rc::Rc;

// Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
pub const SLOW_RATIO: f64 = 3.0;

fn min_of(v: &[f64]) -> f64 {
    v[1..].iter().fold(v[0], |a, &b| py_min(a, b))
}

fn max_of(v: &[f64]) -> f64 {
    v[1..].iter().fold(v[0], |a, &b| py_max(a, b))
}

/// a math error as the KError python raises
pub fn math_kerror(e: MathError) -> KError {
    KError::Other(e.1.into())
}

pub struct DeltaKinematics {
    rails: Vec<Rc<PrinterRail>>,
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: Cell<f64>,
    max_z_accel: Cell<f64>,
    radius: f64,
    arm_lengths: Vec<f64>,
    arm2: Vec<f64>,
    angles: Vec<f64>,
    towers: Vec<(f64, f64)>,
    need_home: Cell<bool>,
    limit_xy2: Cell<f64>,
    home_position: [f64; 3],
    max_z: f64,
    min_z: f64,
    limit_z: f64,
    min_arm_length: f64,
    min_arm2: f64,
    slow_xy2: f64,
    very_slow_xy2: f64,
    max_xy2: f64,
    axes_min: PyVal,
    axes_max: PyVal,
}

fn actuator_to_cartesian(towers: &[(f64, f64)], arm2: &[f64], spos: &[f64]) -> Result<[f64; 3], MathError> {
    let sphere_coords = [
        [towers[0].0, towers[0].1, spos[0]],
        [towers[1].0, towers[1].1, spos[1]],
        [towers[2].0, towers[2].1, spos[2]],
    ];
    trilateration(&sphere_coords, &[arm2[0], arm2[1], arm2[2]])
}

impl DeltaKinematics {
    pub fn new(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Self> {
        // Setup tower rails
        let stepper_configs: Vec<ConfigWrapper> = ["a", "b", "c"].iter().map(|a| config.getsection(&format!("stepper_{}", a))).collect();
        let rail_a = lookup_multi_rail(&stepper_configs[0], false, None, false)?;
        let a_endstop = rail_a.get_homing_info().position_endstop;
        let rail_b = lookup_multi_rail(&stepper_configs[1], false, Some(a_endstop), false)?;
        let rail_c = lookup_multi_rail(&stepper_configs[2], false, Some(a_endstop), false)?;
        let rails = vec![rail_a, rail_b, rail_c];
        // Setup max velocity
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let max_z_velocity = config.getfloat("max_z_velocity", max_velocity, Lim::new().above(0.0).max(max_velocity))?;
        let max_z_accel = config.getfloat("max_z_accel", max_accel, Lim::new().above(0.0).max(max_accel))?;
        // Read radius and arm lengths
        let radius = config.getfloat("delta_radius", Dflt::Required, Lim::new().above(0.0))?;
        let print_radius = config.getfloat("print_radius", radius, Lim::new().above(0.0))?;
        let arm_length_a = stepper_configs[0].getfloat("arm_length", Dflt::Required, Lim::new().above(radius))?;
        let mut arm_lengths = vec![];
        for sconfig in &stepper_configs {
            arm_lengths.push(sconfig.getfloat("arm_length", arm_length_a, Lim::new().above(radius))?);
        }
        let arm2: Vec<f64> = arm_lengths.iter().map(|arm| arm.powf(2.0)).collect();
        let abs_endstops: Vec<f64> =
            rails.iter().zip(&arm2).map(|(rail, a2)| rail.get_homing_info().position_endstop + (a2 - radius.powf(2.0)).sqrt()).collect();
        // Determine tower locations in cartesian space
        let mut angles = vec![];
        for (sconfig, angle) in stepper_configs.iter().zip([210.0, 330.0, 90.0]) {
            angles.push(sconfig.getfloat("angle", angle, Lim::new())?);
        }
        let towers: Vec<(f64, f64)> = angles.iter().map(|a| (a.to_radians().cos() * radius, a.to_radians().sin() * radius)).collect();
        for ((r, a), t) in rails.iter().zip(&arm2).zip(&towers) {
            let (a, tx, ty) = (*a, t.0, t.1);
            r.setup_itersolve(|| delta_stepper_alloc(a, tx, ty));
        }
        for rail in &rails {
            for s in rail.get_steppers() {
                s.set_trapq(Some(toolhead.get_trapq()));
            }
        }
        // Setup boundary checks
        let home_position = actuator_to_cartesian(&towers, &arm2, &abs_endstops).map_err(math_kerror)?;
        let endstops: Vec<f64> = rails.iter().map(|r| r.get_homing_info().position_endstop).collect();
        let max_z = min_of(&endstops);
        // the default is python's int 0 (shown as such in axis_minimum)
        let min_z_value = config.getfloat_value("minimum_z_position", &Dflt::from(PyVal::Int(0)), &Lim::new().max(max_z))?.unwrap_or(PyVal::Int(0));
        let min_z = min_z_value.as_f64().unwrap_or(0.0);
        let lz: Vec<f64> = abs_endstops.iter().zip(&arm_lengths).map(|(ep, arm)| ep - arm).collect();
        let limit_z = min_of(&lz);
        let min_arm_length = min_of(&arm_lengths);
        let min_arm2 = min_arm_length.powf(2.0);
        logging::info(&format!("Delta max build height {:.2}mm (radius tapered above {:.2}mm)", max_z, limit_z));
        // Find the point where an XY move could result in excessive
        // tower movement
        let sd: Vec<f64> = rails.iter().map(|r| r.get_steppers()[0].get_step_dist()).collect();
        let half_min_step_dist = min_of(&sd) * 0.5;
        let ratio_to_xy = |ratio: f64| {
            ratio * (min_arm_length.powf(2.0) / (ratio.powf(2.0) + 1.0) - half_min_step_dist.powf(2.0)).sqrt() + half_min_step_dist - radius
        };
        let slow_xy2 = ratio_to_xy(SLOW_RATIO).powf(2.0);
        let very_slow_xy2 = ratio_to_xy(2.0 * SLOW_RATIO).powf(2.0);
        let max_xy2 = print_radius.powf(2.0);
        let max_xy = max_xy2.sqrt();
        logging::info(&format!(
            "Delta max build radius {:.2}mm (moves slowed past {:.2}mm and {:.2}mm)",
            max_xy,
            slow_xy2.sqrt(),
            very_slow_xy2.sqrt()
        ));
        let kin = DeltaKinematics {
            rails,
            max_velocity,
            max_accel,
            max_z_velocity: Cell::new(max_z_velocity),
            max_z_accel: Cell::new(max_z_accel),
            radius,
            arm_lengths,
            arm2,
            angles,
            towers,
            need_home: Cell::new(true),
            limit_xy2: Cell::new(-1.0),
            home_position,
            max_z,
            min_z,
            limit_z,
            min_arm_length,
            min_arm2,
            slow_xy2,
            very_slow_xy2,
            max_xy2,
            axes_min: PyVal::NamedTuple("Coord", &["x", "y", "z", "e"], vec![PyVal::Float(-max_xy), PyVal::Float(-max_xy), min_z_value, PyVal::Float(0.0)]),
            axes_max: coord_pyval(&[max_xy, max_xy, max_z, 0.0]),
        };
        kin.set_position(&[0.0, 0.0, 0.0], "")?;
        Ok(kin)
    }

    pub fn get_calibration(&self) -> DeltaCalibration {
        let endstops = self.rails.iter().map(|r| r.get_homing_info().position_endstop).collect();
        let stepdists = self.rails.iter().map(|r| r.get_steppers()[0].get_step_dist()).collect();
        DeltaCalibration::new(self.radius, self.angles.clone(), self.arm_lengths.clone(), endstops, stepdists)
    }
}

impl Kinematics for DeltaKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.rails.iter().flat_map(|r| r.get_steppers()).collect()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        let spos: Vec<f64> = self.rails.iter().map(|r| stepper_positions[&r.get_name(false)]).collect();
        // (python raises from a failed trilateration; NaNs here)
        actuator_to_cartesian(&self.towers, &self.arm2, &spos).map(|p| p.to_vec()).unwrap_or(vec![f64::NAN; 3])
    }

    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        for rail in &self.rails {
            rail.set_position(newpos);
        }
        self.limit_xy2.set(-1.0);
        if homing_axes == "xyz" {
            self.need_home.set(false);
        }
        Ok(())
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        // Clearing homing state for each axis individually is not implemented
        if !clear_axes.is_empty() {
            self.limit_xy2.set(-1.0);
            self.need_home.set(true);
        }
    }

    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()> {
        // All axes are homed simultaneously
        homing_state.set_axes(vec![0, 1, 2]);
        let mut forcepos: Vec<Option<f64>> = self.home_position.iter().map(|v| Some(*v)).collect();
        forcepos[2] = Some(-1.5 * (max_of(&self.arm2) - self.max_xy2).sqrt());
        forcepos.push(None);
        let mut homepos: Vec<Option<f64>> = self.home_position.iter().map(|v| Some(*v)).collect();
        homepos.push(None);
        let rails: Vec<RailRef> = self.rails.iter().map(|r| r.clone() as RailRef).collect();
        homing_state.home_rails(&rails, &forcepos, &homepos)
    }

    fn check_move(&self, mv: &mut Move) -> KResult<()> {
        let end_pos = mv.end_pos.clone();
        let end_xy2 = end_pos[0].powf(2.0) + end_pos[1].powf(2.0);
        if end_xy2 <= self.limit_xy2.get() && mv.axes_d[2] == 0.0 {
            // Normal XY move
            return Ok(());
        }
        if self.need_home.get() {
            return Err(mv.move_error("Must home first"));
        }
        let end_z = end_pos[2];
        let mut limit_xy2 = self.max_xy2;
        if end_z > self.limit_z {
            let above_z_limit = end_z - self.limit_z;
            let allowed_radius = self.radius - (self.min_arm2 - (self.min_arm_length - above_z_limit).powf(2.0)).sqrt();
            limit_xy2 = py_min(limit_xy2, allowed_radius.powf(2.0));
        }
        if end_xy2 > limit_xy2 || end_z > self.max_z || end_z < self.min_z {
            // Move out of range - verify not a homing move
            if end_pos[..2] != self.home_position[..2] || end_z < self.min_z || end_z > self.home_position[2] {
                return Err(mv.move_error("Move out of range"));
            }
            limit_xy2 = -1.0;
        }
        if mv.axes_d[2] != 0.0 {
            let z_ratio = mv.move_d / mv.axes_d[2].abs();
            mv.limit_speed(self.max_z_velocity.get() * z_ratio, self.max_z_accel.get() * z_ratio);
            limit_xy2 = -1.0;
        }
        // Limit the speed/accel of this move if is is at the extreme
        // end of the build envelope
        let extreme_xy2 = py_max(end_xy2, mv.start_pos[0].powf(2.0) + mv.start_pos[1].powf(2.0));
        if extreme_xy2 > self.slow_xy2 {
            let mut r = 0.5;
            if extreme_xy2 > self.very_slow_xy2 {
                r = 0.25;
            }
            mv.limit_speed(self.max_velocity * r, self.max_accel * r);
            limit_xy2 = -1.0;
        }
        self.limit_xy2.set(py_min(limit_xy2, self.slow_xy2));
        Ok(())
    }

    fn rails(&self) -> Vec<RailRef> {
        self.rails.iter().map(|r| r.clone() as RailRef).collect()
    }

    fn get_status(&self, _eventtime: f64) -> IndexMap<String, PyVal> {
        let mut d = IndexMap::new();
        d.insert("homed_axes".to_string(), PyVal::from(if self.need_home.get() { "" } else { "xyz" }));
        d.insert("axis_minimum".to_string(), self.axes_min.clone());
        d.insert("axis_maximum".to_string(), self.axes_max.clone());
        d.insert("cone_start_z".to_string(), PyVal::Float(self.limit_z));
        d
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

// Delta parameter calibration for DELTA_CALIBRATE tool
#[derive(Clone)]
pub struct DeltaCalibration {
    pub radius: f64,
    pub angles: Vec<f64>,
    pub arms: Vec<f64>,
    pub endstops: Vec<f64>,
    pub stepdists: Vec<f64>,
    towers: Vec<(f64, f64)>,
    abs_endstops: Vec<f64>,
}

impl DeltaCalibration {
    pub fn new(radius: f64, angles: Vec<f64>, arms: Vec<f64>, endstops: Vec<f64>, stepdists: Vec<f64>) -> Self {
        // Calculate the XY cartesian coordinates of the delta towers
        let towers = angles.iter().map(|a| a.to_radians()).map(|a| (a.cos() * radius, a.sin() * radius)).collect();
        // Calculate the absolute Z height of each tower endstop
        let radius2 = radius.powf(2.0);
        let abs_endstops = endstops.iter().zip(&arms).map(|(e, a)| e + (a.powf(2.0) - radius2).sqrt()).collect();
        DeltaCalibration { radius, angles, arms, endstops, stepdists, towers, abs_endstops }
    }

    /// Determine adjustment parameters (for use with coordinate_descent)
    pub fn coordinate_descent_params(&self, is_extended: bool) -> (Vec<String>, IndexMap<String, f64>) {
        let mut adj_params: Vec<String> =
            ["radius", "angle_a", "angle_b", "endstop_a", "endstop_b", "endstop_c"].iter().map(|s| s.to_string()).collect();
        if is_extended {
            adj_params.extend(["arm_a", "arm_b", "arm_c"].iter().map(|s| s.to_string()));
        }
        let mut params = IndexMap::new();
        params.insert("radius".to_string(), self.radius);
        for (i, axis) in ["a", "b", "c"].iter().enumerate() {
            params.insert(format!("angle_{}", axis), self.angles[i]);
            params.insert(format!("arm_{}", axis), self.arms[i]);
            params.insert(format!("endstop_{}", axis), self.endstops[i]);
            params.insert(format!("stepdist_{}", axis), self.stepdists[i]);
        }
        (adj_params, params)
    }

    /// Create a new calibration object from coordinate_descent params
    pub fn new_calibration(&self, params: &IndexMap<String, f64>) -> Self {
        let g = |p: &str| ["a", "b", "c"].iter().map(|a| params[&format!("{}_{}", p, a)]).collect::<Vec<f64>>();
        DeltaCalibration::new(params["radius"], g("angle"), g("arm"), g("endstop"), g("stepdist"))
    }

    /// Return cartesian coordinates for the given stable_position
    pub fn get_position_from_stable(&self, stable_position: &[f64]) -> Result<[f64; 3], MathError> {
        let mut sphere_coords = [[0.0; 3]; 3];
        for i in 0..3 {
            let (sd, t, es, sp) = (self.stepdists[i], self.towers[i], self.abs_endstops[i], stable_position[i]);
            sphere_coords[i] = [t.0, t.1, es - sp * sd];
        }
        let arms2: Vec<f64> = self.arms.iter().map(|a| a.powf(2.0)).collect();
        trilateration(&sphere_coords, &[arms2[0], arms2[1], arms2[2]])
    }

    /// Return a stable_position from a cartesian coordinate
    pub fn calc_stable_position(&self, coord: &[f64]) -> Result<Vec<f64>, MathError> {
        let mut out = vec![];
        for i in 0..3 {
            let (t, a) = (self.towers[i], self.arms[i]);
            let v = a.powf(2.0) - (t.0 - coord[0]).powf(2.0) - (t.1 - coord[1]).powf(2.0);
            if v < 0.0 {
                return Err(("ValueError", "math domain error"));
            }
            let sp = v.sqrt() + coord[2];
            out.push((self.abs_endstops[i] - sp) / self.stepdists[i]);
        }
        Ok(out)
    }

    /// Save the current parameters (for use with SAVE_CONFIG)
    pub fn save_state(&self, printer: &Printer) -> KResult<()> {
        let configfile = printer.lookup::<ConfigFileObject>("configfile")?;
        {
            let mut cf = configfile.pconfig.borrow_mut();
            cf.set("printer", "delta_radius", &format!("{:.6}", self.radius))?;
            for (i, axis) in ["a", "b", "c"].iter().enumerate() {
                let section = format!("stepper_{}", axis);
                cf.set(&section, "angle", &format!("{:.6}", self.angles[i]))?;
                cf.set(&section, "arm_length", &format!("{:.6}", self.arms[i]))?;
                cf.set(&section, "position_endstop", &format!("{:.6}", self.endstops[i]))?;
            }
        }
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        gcode.respond_info(
            &format!(
                "stepper_a: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\nstepper_b: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\nstepper_c: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\ndelta_radius: {:.6}",
                self.endstops[0],
                self.angles[0],
                self.arms[0],
                self.endstops[1],
                self.angles[1],
                self.arms[1],
                self.endstops[2],
                self.angles[2],
                self.arms[2],
                self.radius
            ),
            true,
        );
        Ok(())
    }
}

pub fn load_kinematics(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(DeltaKinematics::new(toolhead, config)?))
}
