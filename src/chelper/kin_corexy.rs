// CoreXY kinematics stepper pulse time generation
// (port of klippy/chelper/kin_corexy.c)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Y};
use super::trapq::MoveRef;

pub struct CorexyStepper {
    plus: bool,
}

impl Kinematic for CorexyStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let c = m.get_coord(move_time);
        if self.plus {
            c.x + c.y
        } else {
            c.x - c.y
        }
    }
}

/// corexy_stepper_alloc(type); None for a type other than '+' or '-' (C
/// leaves calc_position_cb NULL)
pub fn corexy_stepper_alloc(type_: char) -> Option<SkRef> {
    let plus = match type_ {
        '+' => true,
        '-' => false,
        _ => return None,
    };
    Some(StepperKinematics::new(Box::new(CorexyStepper { plus }), AF_X | AF_Y))
}
