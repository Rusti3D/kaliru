// CoreXZ kinematics stepper pulse time generation
// (port of klippy/chelper/kin_corexz.c)
//
// Copyright (C) 2020  Maks Zolin <mzolin@vorondesign.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Z};
use super::trapq::MoveRef;

pub struct CorexzStepper {
    plus: bool,
}

impl Kinematic for CorexzStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let c = m.get_coord(move_time);
        if self.plus {
            c.x + c.z
        } else {
            c.x - c.z
        }
    }
}

/// corexz_stepper_alloc(type); None for a type other than '+' or '-'
pub fn corexz_stepper_alloc(type_: char) -> Option<SkRef> {
    let plus = match type_ {
        '+' => true,
        '-' => false,
        _ => return None,
    };
    Some(StepperKinematics::new(Box::new(CorexzStepper { plus }), AF_X | AF_Z))
}
