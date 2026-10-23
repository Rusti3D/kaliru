// Cartesian kinematics stepper pulse time generation
// (port of klippy/chelper/kin_cartesian.c)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Y, AF_Z};
use super::trapq::MoveRef;
use std::any::Any;

pub struct CartesianStepper {
    axis: usize,
}

impl Kinematic for CartesianStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        m.get_coord(move_time).axis(self.axis)
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// cartesian_stepper_alloc(axis); C leaves calc_position_cb NULL for an
/// axis other than 'x', 'y' or 'z' (a crash when used): None here
pub fn cartesian_stepper_alloc(axis: char) -> Option<SkRef> {
    let (axis, flags) = match axis {
        'x' => (0, AF_X),
        'y' => (1, AF_Y),
        'z' => (2, AF_Z),
        _ => return None,
    };
    Some(StepperKinematics::new(Box::new(CartesianStepper { axis }), flags))
}
