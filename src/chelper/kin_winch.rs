// Cable winch stepper kinematics (port of klippy/chelper/kin_winch.c)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Y, AF_Z};
use super::trapq::{Coord, MoveRef};
use std::any::Any;

pub struct WinchStepper {
    anchor: Coord,
}

impl Kinematic for WinchStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let c = m.get_coord(move_time);
        let (dx, dy) = (self.anchor.x - c.x, self.anchor.y - c.y);
        let dz = self.anchor.z - c.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

pub fn winch_stepper_alloc(anchor_x: f64, anchor_y: f64, anchor_z: f64) -> SkRef {
    let anchor = Coord { x: anchor_x, y: anchor_y, z: anchor_z };
    StepperKinematics::new(Box::new(WinchStepper { anchor }), AF_X | AF_Y | AF_Z)
}
