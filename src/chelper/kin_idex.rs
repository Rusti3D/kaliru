// Dual carriage kinematics stepper pulse time generation
// (port of klippy/chelper/kin_idex.c)
//
// Copyright (C) 2023  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Y};
use super::trapq::{Coord, Move, MoveRef};
use std::any::Any;

const DUMMY_T: f64 = 500.0;

pub struct DualCarriageStepper {
    orig_sk: Option<SkRef>,
    x_scale: f64,
    x_offs: f64,
    y_scale: f64,
    y_offs: f64,
}

impl Kinematic for DualCarriageStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let pos = m.get_coord(move_time);
        let start_pos = Coord { x: pos.x * self.x_scale + self.x_offs, y: pos.y * self.y_scale + self.y_offs, z: pos.z };
        let dm = Move { print_time: 0., move_t: 2. * DUMMY_T, start_v: 0., half_accel: 0., start_pos, axes_r: Coord { x: 0., y: 0., z: 0. } };
        let orig = self.orig_sk.as_ref().expect("orig sk").borrow();
        orig.kin.calc_position(&orig.st, MoveRef::standalone(&dm), DUMMY_T)
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

fn with_dc<R>(sk: &SkRef, f: impl FnOnce(&mut SkState, &mut DualCarriageStepper) -> R) -> R {
    let mut s = sk.borrow_mut();
    let s = &mut *s;
    let dc = s.kin.as_any_mut().downcast_mut::<DualCarriageStepper>().expect("dual carriage sk");
    f(&mut s.st, dc)
}

pub fn dual_carriage_set_sk(sk: &SkRef, orig_sk: &SkRef) {
    let flags = orig_sk.borrow().st.active_flags;
    with_dc(sk, |st, dc| {
        st.active_flags = flags;
        dc.orig_sk = Some(orig_sk.clone());
    })
}

pub fn dual_carriage_set_transform(sk: &SkRef, axis: char, scale: f64, offs: f64) -> i32 {
    with_dc(sk, |st, dc| {
        let orig_flags = dc.orig_sk.as_ref().map(|o| o.borrow().st.active_flags).unwrap_or(0);
        let flag = match axis {
            'x' => {
                dc.x_scale = scale;
                dc.x_offs = offs;
                AF_X
            }
            'y' => {
                dc.y_scale = scale;
                dc.y_offs = offs;
                AF_Y
            }
            _ => return -1,
        };
        if scale == 0. {
            st.active_flags &= !flag;
        } else if orig_flags & flag != 0 {
            st.active_flags |= flag;
        }
        0
    })
}

pub fn dual_carriage_alloc() -> SkRef {
    let dc = DualCarriageStepper { orig_sk: None, x_scale: 0., x_offs: 0., y_scale: 0., y_offs: 0. };
    StepperKinematics::new(Box::new(dc), 0)
}
