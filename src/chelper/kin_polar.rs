// Polar kinematics stepper pulse time generation
// (port of klippy/chelper/kin_polar.c)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Y};
use super::trapq::MoveRef;
use std::any::Any;
use std::f64::consts::PI;

pub struct PolarStepper {
    angle: bool,
}

impl Kinematic for PolarStepper {
    fn calc_position(&self, sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let c = m.get_coord(move_time);
        if !self.angle {
            return (c.x * c.x + c.y * c.y).sqrt();
        }
        // XXX - handle x==y==0
        let mut angle = c.y.atan2(c.x);
        if angle - sk.commanded_pos > PI {
            angle -= 2. * PI;
        } else if angle - sk.commanded_pos < -PI {
            angle += 2. * PI;
        }
        angle
    }
    fn has_post(&self) -> bool {
        self.angle
    }
    fn post(&self, sk: &mut SkState) {
        // Normalize the stepper_bed angle
        if sk.commanded_pos < -PI {
            sk.commanded_pos += 2. * PI;
        } else if sk.commanded_pos > PI {
            sk.commanded_pos -= 2. * PI;
        }
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// polar_stepper_alloc(type); None for a type other than 'r' or 'a'
pub fn polar_stepper_alloc(type_: char) -> Option<SkRef> {
    let angle = match type_ {
        'r' => false,
        'a' => true,
        _ => return None,
    };
    Some(StepperKinematics::new(Box::new(PolarStepper { angle }), AF_X | AF_Y))
}
