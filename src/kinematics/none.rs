// Dummy "none" kinematics support (for developer testing)
// (port of klippy/kinematics/none.py)
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::Kinematics;
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::gcode::Coord;
use crate::extras::homing::Homing;
use crate::pyval::{PyDict, PyVal};
use crate::stepper::McuStepper;
use crate::toolhead::{Move, ToolHead};
use crate::IndexMap;
use std::rc::Rc;

pub struct NoneKinematics {
    axes_minmax: Coord,
}

impl Kinematics for NoneKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        vec![]
    }

    fn calc_position(&self, _stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        vec![0.0, 0.0, 0.0]
    }

    fn set_position(&self, _newpos: &[f64], _homing_axes: &str) -> KResult<()> {
        Ok(())
    }

    fn clear_homing_state(&self, _clear_axes: &str) {}

    fn home(&self, _homing_state: &Rc<Homing>) -> KResult<()> {
        Ok(())
    }

    fn check_move(&self, _mv: &mut Move) -> KResult<()> {
        Ok(())
    }

    fn get_status(&self, _eventtime: f64) -> PyDict {
        let d = crate::pydict! {
            "homed_axes" => PyVal::from(""),
            "axis_minimum" => self.axes_minmax.to_pyval(),
            "axis_maximum" => self.axes_minmax.to_pyval(),
        };
        d
    }

}

pub fn load_kinematics(_toolhead: &Rc<ToolHead>, _config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(NoneKinematics { axes_minmax: Coord { x: 0.0, y: 0.0, z: 0.0, e: 0.0 } }))
}
