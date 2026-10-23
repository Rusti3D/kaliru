// Rotary delta kinematics stepper pulse time generation
// (port of klippy/chelper/kin_rotary_delta.c)
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Y, AF_Z};
use super::trapq::MoveRef;
use std::any::Any;

// The arm angle calculation is based on the following two formulas:
//   elbow_x**2 + elbow_y**2 = upper_arm**2
//   (effector_x - elbow_x)**2 + (effector_y - elbow_y)**2 = lower_arm**2

// Calculate upper arm angle given xy position of effector joint
// (relative to shoulder joint), upper arm length, and lower arm length.
#[inline]
fn rotary_two_arm_calc(dx: f64, dy: f64, upper_arm2: f64, lower_arm2: f64) -> f64 {
    // Determine constants such that: elbow_y = c1 - c2*elbow_x
    let inv_dy = 1. / dy;
    let c1 = 0.5 * inv_dy * (dx * dx + dy * dy + upper_arm2 - lower_arm2);
    let c2 = dx * inv_dy;
    // Calculate scaled elbow coordinates via quadratic equation.
    let scale = c2 * c2 + 1.0;
    let scaled_elbow_x = c1 * c2 + (scale * upper_arm2 - c1 * c1).sqrt();
    let scaled_elbow_y = c1 * scale - c2 * scaled_elbow_x;
    // Calculate angle in radians
    scaled_elbow_y.atan2(scaled_elbow_x)
}

pub struct RotaryStepper {
    cos: f64,
    sin: f64,
    shoulder_radius: f64,
    shoulder_height: f64,
    upper_arm2: f64,
    lower_arm2: f64,
}

impl Kinematic for RotaryStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let c = m.get_coord(move_time);
        // Rotate and shift axes to an origin at shoulder joint with upper
        // arm constrained to xy plane and x aligned to shoulder platform.
        let sjz = c.y * self.cos - c.x * self.sin;
        let sjx = c.x * self.cos + c.y * self.sin - self.shoulder_radius;
        let sjy = c.z - self.shoulder_height;
        // Calculate angle in radians
        rotary_two_arm_calc(sjx, sjy, self.upper_arm2, self.lower_arm2 - sjz * sjz)
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

pub fn rotary_delta_stepper_alloc(shoulder_radius: f64, shoulder_height: f64, angle: f64, upper_arm: f64, lower_arm: f64) -> SkRef {
    let rs = RotaryStepper {
        cos: angle.cos(),
        sin: angle.sin(),
        shoulder_radius,
        shoulder_height,
        upper_arm2: upper_arm * upper_arm,
        lower_arm2: lower_arm * lower_arm,
    };
    StepperKinematics::new(Box::new(rs), AF_X | AF_Y | AF_Z)
}
