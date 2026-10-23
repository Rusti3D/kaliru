// Deltesian kinematics stepper pulse time generation
// (port of klippy/chelper/kin_deltesian.c)
//
// Copyright (C) 2022  Fabrice Gallet <tircown@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Z};
use super::trapq::MoveRef;
use std::any::Any;

pub struct DeltesianStepper {
    arm2: f64,
    arm_x: f64,
}

impl Kinematic for DeltesianStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let c = m.get_coord(move_time);
        let dx = c.x - self.arm_x;
        (self.arm2 - dx * dx).sqrt() + c.z
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

pub fn deltesian_stepper_alloc(arm2: f64, arm_x: f64) -> SkRef {
    StepperKinematics::new(Box::new(DeltesianStepper { arm2, arm_x }), AF_X | AF_Z)
}
