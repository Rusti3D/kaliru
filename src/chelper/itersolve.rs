// Iterative solver for kinematic moves (port of klippy/chelper/itersolve.c)
//
// Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// struct stepper_kinematics is split in two: SkState holds the common
// fields and a Kinematic implementation holds the kinematic specific ones
// plus calc_position_cb / post_cb.
use super::stepcompress::StepCompress;
use super::trapq::{Move, MoveRef, TrapQRef};
use std::cell::RefCell;
use std::rc::Rc;

pub const AF_X: i32 = 1 << 0;
pub const AF_Y: i32 = 1 << 1;
pub const AF_Z: i32 = 1 << 2;

/// the common fields of struct stepper_kinematics
#[derive(Default)]
pub struct SkState {
    pub step_dist: f64,
    pub commanded_pos: f64,
    pub last_flush_time: f64,
    pub last_move_time: f64,
    pub tq: Option<TrapQRef>,
    pub active_flags: i32,
    pub gen_steps_pre_active: f64,
    pub gen_steps_post_active: f64,
}

crate::impl_downcast!(dyn Kinematic);
/// calc_position_cb / post_cb of a kinematic
pub trait Kinematic: std::any::Any {
    fn calc_position(&self, sk: &SkState, m: MoveRef, move_time: f64) -> f64;
    fn has_post(&self) -> bool {
        false
    }
    fn post(&self, _sk: &mut SkState) {}
}

pub struct StepperKinematics {
    pub st: SkState,
    pub kin: Box<dyn Kinematic>,
}

pub type SkRef = Rc<RefCell<StepperKinematics>>;

impl StepperKinematics {
    pub fn new(kin: Box<dyn Kinematic>, active_flags: i32) -> SkRef {
        Rc::new(RefCell::new(StepperKinematics { st: SkState { active_flags, ..SkState::default() }, kin }))
    }
}

/****************************************************************
 * Main iterative solver
 ****************************************************************/

#[derive(Clone, Copy)]
struct TimePos {
    time: f64,
    position: f64,
}

const SEEK_TIME_RESET: f64 = 0.000100;

// Generate step times for a portion of a move
fn gen_steps_range(sk: &mut StepperKinematics, sc: &mut StepCompress, m: MoveRef, abs_start: f64, abs_end: f64) -> i32 {
    let half_step = 0.5 * sk.st.step_dist;
    let mut start = abs_start - m.print_time;
    let mut end = abs_end - m.print_time;
    if start < 0. {
        start = 0.;
    }
    if end > m.move_t {
        end = m.move_t;
    }
    let mut old_guess = TimePos { time: start, position: sk.st.commanded_pos };
    let mut guess = old_guess;
    let mut sdir = sc.get_step_dir();
    let (mut is_dir_change, mut have_bracket, mut check_oscillate) = (false, false, false);
    let mut target = sk.st.commanded_pos + if sdir != 0 { half_step } else { -half_step };
    let (mut last_time, mut low_time, mut high_time) = (start, start, start + SEEK_TIME_RESET);
    if high_time > end {
        high_time = end;
    }
    loop {
        // Use the "secant method" to guess a new time from previous guesses
        let guess_dist = guess.position - target;
        let og_dist = old_guess.position - target;
        let mut next_time = (old_guess.time * guess_dist - guess.time * og_dist) / (guess_dist - og_dist);
        if !(next_time > low_time && next_time < high_time) {
            // or NaN
            // Next guess is outside bounds checks - validate it
            if have_bracket {
                // A poor guess - fall back to bisection
                next_time = (low_time + high_time) * 0.5;
                check_oscillate = false;
            } else if guess.time >= end {
                // No more steps present in requested time range
                break;
            } else {
                // Might be a poor guess - limit to exponential search
                next_time = high_time;
                high_time = 2. * high_time - last_time;
                if high_time > end {
                    high_time = end;
                }
            }
        }
        // Calculate position at next_time guess
        old_guess = guess;
        guess.time = next_time;
        guess.position = sk.kin.calc_position(&sk.st, m, next_time);
        let guess_dist = guess.position - target;
        if guess_dist.abs() > 0.000000001 {
            // Guess does not look close enough - update bounds
            let rel_dist = if sdir != 0 { guess_dist } else { -guess_dist };
            if rel_dist > 0. {
                // Found position past target, so step is definitely present
                if have_bracket && old_guess.time <= low_time {
                    if check_oscillate {
                        // Force bisect next to avoid persistent oscillations
                        old_guess = guess;
                    }
                    check_oscillate = true;
                }
                high_time = guess.time;
                have_bracket = true;
            } else if rel_dist < -(half_step + half_step + 0.000000010) {
                // Found direction change
                sdir = (sdir == 0) as i32;
                target = if sdir != 0 { target + half_step + half_step } else { target - half_step - half_step };
                low_time = last_time;
                high_time = guess.time;
                is_dir_change = true;
                have_bracket = true;
                check_oscillate = false;
            } else {
                low_time = guess.time;
            }
            if !have_bracket || high_time - low_time > 0.000000001 {
                if !is_dir_change && rel_dist >= -half_step {
                    // Avoid rollback if stepper fully reaches step position
                    sc.commit();
                }
                // Guess is not close enough - guess again with new time
                continue;
            }
        }
        // Found next step - submit it
        let ret = sc.append(sdir, m.print_time, guess.time);
        if ret != 0 {
            return ret;
        }
        target = if sdir != 0 { target + half_step + half_step } else { target - half_step - half_step };
        // Reset bounds checking
        let mut seek_time_delta = 1.5 * (guess.time - last_time);
        if seek_time_delta < 0.000000001 {
            seek_time_delta = 0.000000001;
        }
        if is_dir_change && seek_time_delta > SEEK_TIME_RESET {
            seek_time_delta = SEEK_TIME_RESET;
        }
        last_time = guess.time;
        low_time = guess.time;
        high_time = guess.time + seek_time_delta;
        if high_time > end {
            high_time = end;
        }
        is_dir_change = false;
        have_bracket = false;
        check_oscillate = false;
    }
    sk.st.commanded_pos = target - if sdir != 0 { half_step } else { -half_step };
    if sk.kin.has_post() {
        sk.kin.post(&mut sk.st);
    }
    0
}

/****************************************************************
 * Interface functions
 ****************************************************************/

// Check if a move is likely to cause movement on a stepper
#[inline]
fn check_active(st: &SkState, m: &Move) -> bool {
    let af = st.active_flags;
    (af & AF_X != 0 && m.axes_r.x != 0.) || (af & AF_Y != 0 && m.axes_r.y != 0.) || (af & AF_Z != 0 && m.axes_r.z != 0.)
}

// Generate step times for a range of moves on the trapq
pub fn generate_steps(sk: &mut StepperKinematics, sc: &mut StepCompress, flush_time: f64) -> i32 {
    let last_flush_time = sk.st.last_flush_time;
    sk.st.last_flush_time = flush_time;
    let Some(tq) = sk.st.tq.clone() else { return 0 };
    tq.borrow_mut().check_sentinels();
    let tq = tq.borrow();
    let moves = &tq.moves;
    let mut mi = 0;
    while last_flush_time >= moves[mi].print_time + moves[mi].move_t {
        mi += 1;
    }
    let mut force_steps_time = sk.st.last_move_time + sk.st.gen_steps_post_active;
    let mut skip_count = 0;
    loop {
        let m = MoveRef::new(moves, mi);
        let move_start = m.print_time;
        let move_end = move_start + m.move_t;
        if check_active(&sk.st, m.m) {
            if skip_count != 0 && sk.st.gen_steps_pre_active != 0. {
                // Must generate steps leading up to stepper activity
                let mut abs_start = move_start - sk.st.gen_steps_pre_active;
                if abs_start < last_flush_time {
                    abs_start = last_flush_time;
                }
                if abs_start < force_steps_time {
                    abs_start = force_steps_time;
                }
                let mut pm = mi - 1;
                loop {
                    skip_count -= 1;
                    if skip_count == 0 || moves[pm].print_time <= abs_start {
                        break;
                    }
                    pm -= 1;
                }
                loop {
                    let ret = gen_steps_range(sk, sc, MoveRef::new(moves, pm), abs_start, flush_time);
                    if ret != 0 {
                        return ret;
                    }
                    pm += 1;
                    if pm == mi {
                        break;
                    }
                }
            }
            // Generate steps for this move
            let ret = gen_steps_range(sk, sc, m, last_flush_time, flush_time);
            if ret != 0 {
                return ret;
            }
            if move_end >= flush_time {
                sk.st.last_move_time = flush_time;
                return 0;
            }
            skip_count = 0;
            sk.st.last_move_time = move_end;
            force_steps_time = sk.st.last_move_time + sk.st.gen_steps_post_active;
        } else {
            if move_start < force_steps_time {
                // Must generates steps just past stepper activity
                let mut abs_end = force_steps_time;
                if abs_end > flush_time {
                    abs_end = flush_time;
                }
                let ret = gen_steps_range(sk, sc, m, last_flush_time, abs_end);
                if ret != 0 {
                    return ret;
                }
                skip_count = 1;
            } else {
                // This move doesn't impact this stepper - skip it
                skip_count += 1;
            }
            if flush_time + sk.st.gen_steps_pre_active <= move_end {
                return 0;
            }
        }
        mi += 1;
    }
}

// Check if the given stepper is likely to be active in the given time range
pub fn check_active_time(sk: &StepperKinematics, flush_time: f64) -> f64 {
    let Some(tq) = sk.st.tq.clone() else { return 0. };
    tq.borrow_mut().check_sentinels();
    let tq = tq.borrow();
    let moves = &tq.moves;
    let mut mi = 0;
    while sk.st.last_flush_time >= moves[mi].print_time + moves[mi].move_t {
        mi += 1;
    }
    loop {
        let m = &moves[mi];
        if check_active(&sk.st, m) {
            return m.print_time;
        }
        if flush_time <= m.print_time + m.move_t {
            return 0.;
        }
        mi += 1;
    }
}

// Report if the given stepper is registered for the given axis
pub fn is_active_axis(sk: &StepperKinematics, axis: u8) -> i32 {
    if !(b'x'..=b'z').contains(&axis) {
        return 0;
    }
    (sk.st.active_flags & (AF_X << (axis - b'x')) != 0) as i32
}

pub fn set_trapq(sk: &mut StepperKinematics, tq: Option<TrapQRef>, step_dist: f64) {
    sk.st.tq = tq;
    sk.st.step_dist = step_dist;
}

pub fn get_trapq(sk: &StepperKinematics) -> Option<TrapQRef> {
    sk.st.tq.clone()
}

pub fn calc_position_from_coord(sk: &StepperKinematics, x: f64, y: f64, z: f64) -> f64 {
    let mut m = Move::default();
    m.start_pos.x = x;
    m.start_pos.y = y;
    m.start_pos.z = z;
    m.move_t = 1000.;
    sk.kin.calc_position(&sk.st, MoveRef::standalone(&m), 500.)
}

pub fn set_position(sk: &mut StepperKinematics, x: f64, y: f64, z: f64) {
    sk.st.commanded_pos = calc_position_from_coord(sk, x, y, z);
}

pub fn get_commanded_pos(sk: &StepperKinematics) -> f64 {
    sk.st.commanded_pos
}

pub fn get_gen_steps_pre_active(sk: &StepperKinematics) -> f64 {
    sk.st.gen_steps_pre_active
}

pub fn get_gen_steps_post_active(sk: &StepperKinematics) -> f64 {
    sk.st.gen_steps_post_active
}
