// Code for handling the kinematics of cable winch robots
// (port of klippy/kinematics/winch.py)
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::Kinematics;
use crate::chelper::kin_winch::winch_stepper_alloc;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::extras::homing::Homing;
use crate::gcode::coord_pyval;
use crate::mathutil::trilateration;
use crate::pyfmt::{py_max, py_min};
use crate::pyval::{PyDict, PyVal};
use crate::stepper::{printer_stepper, McuStepper};
use crate::toolhead::{Move, ToolHead};
use crate::IndexMap;
use std::rc::Rc;

pub struct WinchKinematics {
    steppers: Vec<Rc<McuStepper>>,
    anchors: Vec<[f64; 3]>,
    axes_min: [f64; 4],
    axes_max: [f64; 4],
}

impl WinchKinematics {
    fn new(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Self> {
        // Setup steppers at each anchor
        let mut steppers = vec![];
        let mut anchors = vec![];
        for i in 0..26u8 {
            let name = format!("stepper_{}", (b'a' + i) as char);
            if i >= 3 && !config.has_section(&name) {
                break;
            }
            let stepper_config = config.getsection(&name);
            let s = printer_stepper(&stepper_config, false)?;
            steppers.push(s.clone());
            let mut a = [0.0; 3];
            for (j, n) in "xyz".chars().enumerate() {
                a[j] = stepper_config.getfloat(&format!("anchor_{}", n), Dflt::Required, Lim::new())?;
            }
            anchors.push(a);
            s.setup_itersolve(winch_stepper_alloc(a[0], a[1], a[2]));
            s.set_trapq(Some(toolhead.get_trapq()));
        }
        // Setup boundary checks
        let mut axes_min = [0.0; 4];
        let mut axes_max = [0.0; 4];
        for j in 0..3 {
            axes_min[j] = anchors.iter().map(|a| a[j]).reduce(py_min).unwrap_or(0.0);
            axes_max[j] = anchors.iter().map(|a| a[j]).reduce(py_max).unwrap_or(0.0);
        }
        let kin = WinchKinematics { steppers, anchors, axes_min, axes_max };
        kin.set_position(&[0.0, 0.0, 0.0], "")?;
        Ok(kin)
    }
}

impl Kinematics for WinchKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.steppers.clone()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        // Use only first three steppers to calculate cartesian position
        let pos: Vec<f64> = self.steppers[..3].iter().map(|s| stepper_positions[&s.get_name(false)]).collect();
        let anchors = [self.anchors[0], self.anchors[1], self.anchors[2]];
        let radius2 = [pos[0] * pos[0], pos[1] * pos[1], pos[2] * pos[2]];
        // python raises on a degenerate anchor layout; NaN here
        trilateration(&anchors, &radius2).map(|p| p.to_vec()).unwrap_or_else(|_| vec![f64::NAN; 3])
    }

    fn set_position(&self, newpos: &[f64], _homing_axes: &str) -> KResult<()> {
        for s in &self.steppers {
            s.set_position(newpos);
        }
        Ok(())
    }

    fn clear_homing_state(&self, _clear_axes: &str) {
        // XXX - homing not implemented
    }

    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()> {
        // XXX - homing not implemented
        homing_state.set_axes(vec![0, 1, 2]);
        homing_state.set_homed_position(&[Some(0.0), Some(0.0), Some(0.0)])
    }

    fn check_move(&self, _mv: &mut Move) -> KResult<()> {
        // XXX - boundary checks and speed limits not implemented
        Ok(())
    }

    fn get_status(&self, _eventtime: f64) -> PyDict {
        // XXX - homed_checks and rail limits not implemented
        let d = crate::pydict! {
            "homed_axes" => PyVal::from("xyz"),
            "axis_minimum" => coord_pyval(&self.axes_min),
            "axis_maximum" => coord_pyval(&self.axes_max),
        };
        d
    }

}

pub fn load_kinematics(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(WinchKinematics::new(toolhead, config)?))
}
