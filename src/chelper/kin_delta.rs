// Delta kinematics stepper pulse time generation
// (port of klippy/chelper/kin_delta.c)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Y, AF_Z};
use super::trapq::MoveRef;
use std::any::Any;

pub struct DeltaStepper {
    arm2: f64,
    tower_x: f64,
    tower_y: f64,
}

impl Kinematic for DeltaStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let c = m.get_coord(move_time);
        let dx = self.tower_x - c.x;
        let dy = self.tower_y - c.y;
        (self.arm2 - dx * dx - dy * dy).sqrt() + c.z
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

pub fn delta_stepper_alloc(arm2: f64, tower_x: f64, tower_y: f64) -> SkRef {
    StepperKinematics::new(Box::new(DeltaStepper { arm2, tower_x, tower_y }), AF_X | AF_Y | AF_Z)
}
