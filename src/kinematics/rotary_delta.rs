// Code for handling the kinematics of rotary delta robots
// (port of klippy/kinematics/rotary_delta.py)
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::delta::{math_kerror, Calibration};
use super::Kinematics;
use crate::chelper::itersolve::{calc_position_from_coord, SkRef};
use crate::chelper::kin_rotary_delta::rotary_delta_stepper_alloc;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::extras::homing::Homing;
use crate::gcode::{coord_pyval, GCodeDispatch};
use crate::logging;
use crate::mathutil::{trilateration, MathError};
use crate::printer::{ConfigFileObject, Printer};
use crate::pyfmt::py_min;
use crate::pyval::PyVal;
use crate::stepper::{McuStepper, PrinterRail, Rail, RailRef};
use crate::toolhead::{Move, ToolHead};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::Cell;
use std::rc::Rc;

fn min_of(v: &[f64]) -> f64 {
    v.iter().copied().reduce(py_min).unwrap_or(f64::NAN)
}

pub struct RotaryDeltaKinematics {
    rails: Vec<Rc<PrinterRail>>,
    max_z_velocity: Cell<f64>,
    calibration: Rc<RotaryDeltaCalibration>,
    need_home: Cell<bool>,
    limit_xy2: Cell<f64>,
    home_position: [f64; 3],
    max_z: f64,
    min_z: f64,
    max_xy2: f64,
    limit_z: f64,
    axes_min: PyVal,
    axes_max: PyVal,
}

impl RotaryDeltaKinematics {
    fn new(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Self> {
        // Setup tower rails
        let stepper_configs: Vec<ConfigWrapper> = ["a", "b", "c"].iter().map(|a| config.getsection(&format!("stepper_{}", a))).collect();
        let rail_a = PrinterRail::new(&stepper_configs[0], false, None, true)?;
        let a_endstop = rail_a.get_homing_info().position_endstop;
        let rail_b = PrinterRail::new(&stepper_configs[1], false, Some(a_endstop), true)?;
        let rail_c = PrinterRail::new(&stepper_configs[2], false, Some(a_endstop), true)?;
        let rails = vec![rail_a, rail_b, rail_c];
        // Read config
        let (max_velocity, _max_accel) = toolhead.get_max_velocity();
        let max_z_velocity = config.getfloat("max_z_velocity", max_velocity, Lim::new().above(0.0).max(max_velocity))?;
        let shoulder_radius = config.getfloat("shoulder_radius", Dflt::Required, Lim::new().above(0.0))?;
        let shoulder_height = config.getfloat("shoulder_height", Dflt::Required, Lim::new().above(0.0))?;
        let a_upper_arm = stepper_configs[0].getfloat("upper_arm_length", Dflt::Required, Lim::new().above(0.0))?;
        let mut upper_arms = vec![];
        for sconfig in &stepper_configs {
            upper_arms.push(sconfig.getfloat("upper_arm_length", a_upper_arm, Lim::new().above(0.0))?);
        }
        let a_lower_arm = stepper_configs[0].getfloat("lower_arm_length", Dflt::Required, Lim::new().above(0.0))?;
        let mut lower_arms = vec![];
        for sconfig in &stepper_configs {
            lower_arms.push(sconfig.getfloat("lower_arm_length", a_lower_arm, Lim::new().above(0.0))?);
        }
        let mut angles = vec![];
        for (sconfig, angle) in stepper_configs.iter().zip([30.0, 150.0, 270.0]) {
            angles.push(sconfig.getfloat("angle", angle, Lim::new())?);
        }
        // Setup rotary delta calibration helper
        let endstops: Vec<f64> = rails.iter().map(|r| r.get_homing_info().position_endstop).collect();
        let stepdists: Vec<f64> = rails.iter().map(|r| r.get_steppers()[0].get_step_dist()).collect();
        let calibration = RotaryDeltaCalibration::new(
            shoulder_radius,
            shoulder_height,
            angles.clone(),
            upper_arms.clone(),
            lower_arms.clone(),
            endstops.clone(),
            stepdists,
        );
        // Setup iterative solver
        for (((r, a), ua), la) in rails.iter().zip(&angles).zip(&upper_arms).zip(&lower_arms) {
            let (a, ua, la) = (a.to_radians(), *ua, *la);
            r.setup_itersolve(|| rotary_delta_stepper_alloc(shoulder_radius, shoulder_height, a, ua, la));
        }
        for rail in &rails {
            for s in rail.get_steppers() {
                s.set_trapq(Some(toolhead.get_trapq()));
            }
        }
        // Setup boundary checks
        let eangles: Vec<f64> = rails.iter().zip(&endstops).map(|(r, ep)| r.calc_position_from_coord(&[0.0, 0.0, *ep])).collect();
        let home_position = calibration.actuator_to_cartesian(&eangles).map_err(math_kerror)?;
        let max_z = min_of(&endstops);
        // the default is python's int 0 (shown as such in axis_minimum)
        let min_z_value =
            config.getfloat_value("minimum_z_position", &Dflt::from(PyVal::Int(0)), &Lim::new().max(max_z))?.unwrap_or(PyVal::Int(0));
        let min_z = min_z_value.as_f64().unwrap_or(0.0);
        let min_ua = min_of(&upper_arms.iter().map(|ua| shoulder_radius + ua).collect::<Vec<_>>());
        let min_la = min_of(&lower_arms.iter().map(|la| la - shoulder_radius).collect::<Vec<_>>());
        let max_xy2 = py_min(min_ua, min_la).powf(2.0);
        let arm_z: Vec<f64> = eangles.iter().enumerate().map(|(i, ea)| calibration.elbow_coord(i, *ea)[2]).collect();
        let limit_z = min_of(&arm_z.iter().zip(&lower_arms).map(|(az, la)| az - la).collect::<Vec<_>>());
        logging::info(&format!("Delta max build height {:.2}mm (radius tapered above {:.2}mm)", max_z, limit_z));
        let max_xy = max_xy2.sqrt();
        let kin = RotaryDeltaKinematics {
            rails,
            max_z_velocity: Cell::new(max_z_velocity),
            calibration: Rc::new(calibration),
            need_home: Cell::new(true),
            limit_xy2: Cell::new(-1.0),
            home_position,
            max_z,
            min_z,
            max_xy2,
            limit_z,
            axes_min: PyVal::NamedTuple("Coord", &["x", "y", "z", "e"], vec![PyVal::Float(-max_xy), PyVal::Float(-max_xy), min_z_value, PyVal::Float(0.0)]),
            axes_max: coord_pyval(&[max_xy, max_xy, max_z, 0.0]),
        };
        kin.set_position(&[0.0, 0.0, 0.0], "")?;
        Ok(kin)
    }
}

impl Kinematics for RotaryDeltaKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.rails.iter().flat_map(|r| r.get_steppers()).collect()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        let spos: Vec<f64> = self.rails.iter().map(|r| stepper_positions[&r.get_name(false)]).collect();
        // python raises on a degenerate position; NaN here
        self.calibration.actuator_to_cartesian(&spos).map(|p| p.to_vec()).unwrap_or_else(|_| vec![f64::NAN; 3])
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
        // min_angles = [-.5 * math.pi] * 3
        // forcepos[2] = self.calibration.actuator_to_cartesian(min_angles)[2]
        forcepos[2] = Some(-1.0);
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
            limit_xy2 = py_min(limit_xy2, (self.max_z - end_z).powf(2.0));
        }
        if end_xy2 > limit_xy2 || end_z > self.max_z || end_z < self.min_z {
            // Move out of range - verify not a homing move
            if end_pos[..2] != self.home_position[..2] || end_z < self.min_z || end_z > self.home_position[2] {
                return Err(mv.move_error("Move out of range"));
            }
            limit_xy2 = -1.0;
        }
        if mv.axes_d[2] != 0.0 {
            let accel = mv.accel;
            mv.limit_speed(self.max_z_velocity.get(), accel);
            limit_xy2 = -1.0;
        }
        self.limit_xy2.set(limit_xy2);
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
        d
    }

    fn get_calibration(&self) -> Option<Rc<dyn Calibration>> {
        Some(self.calibration.clone())
    }

    fn limit(&self, name: &str) -> Option<&Cell<f64>> {
        match name {
            "max_z_velocity" => Some(&self.max_z_velocity),
            _ => None,
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

// Rotary delta parameter calibration for DELTA_CALIBRATE tool
pub struct RotaryDeltaCalibration {
    shoulder_radius: f64,
    shoulder_height: f64,
    angles: Vec<f64>,
    upper_arms: Vec<f64>,
    lower_arms: Vec<f64>,
    endstops: Vec<f64>,
    stepdists: Vec<f64>,
    sks: Vec<SkRef>,
    abs_endstops: Vec<f64>,
}

impl RotaryDeltaCalibration {
    #[allow(clippy::too_many_arguments)]
    fn new(
        shoulder_radius: f64,
        shoulder_height: f64,
        angles: Vec<f64>,
        upper_arms: Vec<f64>,
        lower_arms: Vec<f64>,
        endstops: Vec<f64>,
        stepdists: Vec<f64>,
    ) -> Self {
        // Calculate the absolute angle of each endstop
        let sks: Vec<SkRef> = angles
            .iter()
            .zip(&upper_arms)
            .zip(&lower_arms)
            .map(|((a, ua), la)| rotary_delta_stepper_alloc(shoulder_radius, shoulder_height, a.to_radians(), *ua, *la))
            .collect();
        let abs_endstops = sks.iter().zip(&endstops).map(|(sk, es)| calc_position_from_coord(&sk.borrow(), 0.0, 0.0, *es)).collect();
        RotaryDeltaCalibration { shoulder_radius, shoulder_height, angles, upper_arms, lower_arms, endstops, stepdists, sks, abs_endstops }
    }

    fn elbow_coord(&self, elbow_id: usize, spos: f64) -> [f64; 3] {
        // Calculate elbow position in coordinate system at shoulder joint
        let sj_elbow_x = self.upper_arms[elbow_id] * spos.cos();
        let sj_elbow_y = self.upper_arms[elbow_id] * spos.sin();
        // Shift and rotate to main cartesian coordinate system
        let angle = self.angles[elbow_id].to_radians();
        let x = (sj_elbow_x + self.shoulder_radius) * angle.cos();
        let y = (sj_elbow_x + self.shoulder_radius) * angle.sin();
        let z = sj_elbow_y + self.shoulder_height;
        [x, y, z]
    }

    fn actuator_to_cartesian(&self, spos: &[f64]) -> Result<[f64; 3], MathError> {
        let sphere_coords = [self.elbow_coord(0, spos[0]), self.elbow_coord(1, spos[1]), self.elbow_coord(2, spos[2])];
        let la2: Vec<f64> = self.lower_arms.iter().map(|la| la.powf(2.0)).collect();
        trilateration(&sphere_coords, &[la2[0], la2[1], la2[2]])
    }
}

impl Calibration for RotaryDeltaCalibration {
    fn coordinate_descent_params(&self, is_extended: bool) -> (Vec<String>, IndexMap<String, f64>) {
        // Determine adjustment parameters (for use with coordinate_descent)
        let mut adj_params: Vec<String> = ["shoulder_height", "endstop_a", "endstop_b", "endstop_c"].iter().map(|s| s.to_string()).collect();
        if is_extended {
            adj_params.extend(["shoulder_radius", "angle_a", "angle_b"].iter().map(|s| s.to_string()));
        }
        let mut params = IndexMap::new();
        params.insert("shoulder_radius".to_string(), self.shoulder_radius);
        params.insert("shoulder_height".to_string(), self.shoulder_height);
        for (i, axis) in ["a", "b", "c"].iter().enumerate() {
            params.insert(format!("angle_{}", axis), self.angles[i]);
            params.insert(format!("upper_arm_{}", axis), self.upper_arms[i]);
            params.insert(format!("lower_arm_{}", axis), self.lower_arms[i]);
            params.insert(format!("endstop_{}", axis), self.endstops[i]);
            params.insert(format!("stepdist_{}", axis), self.stepdists[i]);
        }
        (adj_params, params)
    }

    fn new_calibration(&self, params: &IndexMap<String, f64>) -> Rc<dyn Calibration> {
        // Create a new calibration object from coordinate_descent params
        let g = |p: &str| ["a", "b", "c"].iter().map(|a| params[&format!("{}_{}", p, a)]).collect::<Vec<f64>>();
        Rc::new(RotaryDeltaCalibration::new(
            params["shoulder_radius"],
            params["shoulder_height"],
            g("angle"),
            g("upper_arm"),
            g("lower_arm"),
            g("endstop"),
            g("stepdist"),
        ))
    }

    fn get_position_from_stable(&self, stable_position: &[f64]) -> Result<[f64; 3], MathError> {
        // Return cartesian coordinates for the given stable_position
        let spos: Vec<f64> = (0..3).map(|i| self.abs_endstops[i] - stable_position[i] * self.stepdists[i]).collect();
        self.actuator_to_cartesian(&spos)
    }

    fn calc_stable_position(&self, coord: &[f64]) -> Result<Vec<f64>, MathError> {
        // Return a stable_position from a cartesian coordinate
        let pos: Vec<f64> = self.sks.iter().map(|sk| calc_position_from_coord(&sk.borrow(), coord[0], coord[1], coord[2])).collect();
        Ok((0..3).map(|i| (self.abs_endstops[i] - pos[i]) / self.stepdists[i]).collect())
    }

    fn save_state(&self, printer: &Printer) -> KResult<()> {
        // Save the current parameters (for use with SAVE_CONFIG)
        let configfile = printer.lookup::<ConfigFileObject>("configfile")?;
        {
            let mut cf = configfile.pconfig.borrow_mut();
            cf.set("printer", "shoulder_radius", &format!("{:.6}", self.shoulder_radius))?;
            cf.set("printer", "shoulder_height", &format!("{:.6}", self.shoulder_height))?;
            for (i, axis) in ["a", "b", "c"].iter().enumerate() {
                let section = format!("stepper_{}", axis);
                cf.set(&section, "angle", &format!("{:.6}", self.angles[i]))?;
                cf.set(&section, "position_endstop", &format!("{:.6}", self.endstops[i]))?;
            }
        }
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        gcode.respond_info(
            &format!(
                "stepper_a: position_endstop: {:.6} angle: {:.6}\nstepper_b: position_endstop: {:.6} angle: {:.6}\nstepper_c: position_endstop: {:.6} angle: {:.6}\nshoulder_radius: {:.6} shoulder_height: {:.6}",
                self.endstops[0],
                self.angles[0],
                self.endstops[1],
                self.angles[1],
                self.endstops[2],
                self.angles[2],
                self.shoulder_radius,
                self.shoulder_height
            ),
            true,
        );
        Ok(())
    }
}

pub fn load_kinematics(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(RotaryDeltaKinematics::new(toolhead, config)?))
}
