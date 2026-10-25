// Kinematic input shapers to minimize motion vibrations in XY plane
// (port of klippy/chelper/kin_shaper.c)
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X, AF_Y};
use super::trapq::{Coord, Move, MoveRef};

/****************************************************************
 * Shaper initialization
 ****************************************************************/

#[derive(Clone, Copy, Default)]
struct Pulse {
    t: f64,
    a: f64,
}

#[derive(Clone, Default)]
struct ShaperPulses {
    num_pulses: usize,
    pulses: [Pulse; 5],
}

// Shift pulses around 'mid-point' t=0 so that the input shaper is an identity
// transformation for constant-speed motion (i.e. input_shaper(v * T) = v * T)
fn shift_pulses(sp: &mut ShaperPulses) {
    let mut ts = 0.;
    for i in 0..sp.num_pulses {
        ts += sp.pulses[i].a * sp.pulses[i].t;
    }
    for i in 0..sp.num_pulses {
        sp.pulses[i].t -= ts;
    }
}

fn init_shaper(n: i32, a: &[f64], t: &[f64], sp: &mut ShaperPulses) -> i32 {
    if n < 0 || n as usize > sp.pulses.len() {
        sp.num_pulses = 0;
        return -1;
    }
    let n = n as usize;
    let mut sum_a = 0.;
    for v in &a[..n] {
        sum_a += v;
    }
    let inv_a = 1. / sum_a;
    // Reverse pulses vs their traditional definition
    for i in 0..n {
        sp.pulses[n - i - 1].a = a[i] * inv_a;
        sp.pulses[n - i - 1].t = -t[i];
    }
    sp.num_pulses = n;
    shift_pulses(sp);
    0
}

/****************************************************************
 * Generic position calculation via shaper convolution
 ****************************************************************/

#[inline]
fn get_axis_position(m: &MoveRef, axis: usize, move_time: f64) -> f64 {
    let axis_r = m.axes_r.axis(axis);
    let start_pos = m.start_pos.axis(axis);
    let move_dist = m.get_distance(move_time);
    start_pos + axis_r * move_dist
}

#[inline]
fn get_axis_position_across_moves(mut m: MoveRef, axis: usize, mut time: f64) -> f64 {
    while time < 0. {
        m = m.prev();
        time += m.move_t;
    }
    while time > m.move_t {
        time -= m.move_t;
        m = m.next();
    }
    get_axis_position(&m, axis, time)
}

// Calculate the position from the convolution of the shaper with input signal
#[inline]
fn calc_position(m: MoveRef, axis: usize, move_time: f64, sp: &ShaperPulses) -> f64 {
    let mut res = 0.;
    for i in 0..sp.num_pulses {
        let (t, a) = (sp.pulses[i].t, sp.pulses[i].a);
        res += a * get_axis_position_across_moves(m, axis, move_time + t);
    }
    res
}

/****************************************************************
 * Kinematics-related shaper code
 ****************************************************************/

pub const DUMMY_T: f64 = 500.0;

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    None,
    X,
    Y,
    Xy,
}

pub struct InputShaper {
    orig_sk: Option<SkRef>,
    mode: Mode,
    has_post: bool,
    sx: ShaperPulses,
    sy: ShaperPulses,
}

impl InputShaper {
    /// orig_sk->calc_position_cb(orig_sk, m, t)
    fn orig_calc(&self, m: MoveRef, t: f64) -> f64 {
        let orig = self.orig_sk.as_ref().expect("orig sk").borrow();
        orig.kin.calc_position(&orig.st, m, t)
    }

    /// the "struct move m" of the input shaper with start_pos set
    fn dummy(start_pos: Coord) -> Move {
        Move { print_time: 0., move_t: 2. * DUMMY_T, start_v: 0., half_accel: 0., start_pos, axes_r: Coord { x: 0., y: 0., z: 0. } }
    }
}

impl Kinematic for InputShaper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        match self.mode {
            // Optimized calc_position when only x axis is needed
            Mode::X => {
                if self.sx.num_pulses == 0 {
                    return self.orig_calc(m, move_time);
                }
                let mut sp = Coord { x: 0., y: 0., z: 0. };
                sp.x = calc_position(m, 0, move_time, &self.sx);
                let d = Self::dummy(sp);
                self.orig_calc(MoveRef::standalone(&d), DUMMY_T)
            }
            // Optimized calc_position when only y axis is needed
            Mode::Y => {
                if self.sy.num_pulses == 0 {
                    return self.orig_calc(m, move_time);
                }
                let mut sp = Coord { x: 0., y: 0., z: 0. };
                sp.y = calc_position(m, 1, move_time, &self.sy);
                let d = Self::dummy(sp);
                self.orig_calc(MoveRef::standalone(&d), DUMMY_T)
            }
            // General calc_position for both x and y axes
            Mode::Xy => {
                if self.sx.num_pulses == 0 && self.sy.num_pulses == 0 {
                    return self.orig_calc(m, move_time);
                }
                let mut sp = m.get_coord(move_time);
                if self.sx.num_pulses != 0 {
                    sp.x = calc_position(m, 0, move_time, &self.sx);
                }
                if self.sy.num_pulses != 0 {
                    sp.y = calc_position(m, 1, move_time, &self.sy);
                }
                let d = Self::dummy(sp);
                self.orig_calc(MoveRef::standalone(&d), DUMMY_T)
            }
            Mode::None => 0.,
        }
    }

    fn has_post(&self) -> bool {
        self.has_post
    }

    // A callback that forwards post_cb call to the original kinematics
    fn post(&self, sk: &mut SkState) {
        let mut orig = self.orig_sk.as_ref().expect("orig sk").borrow_mut();
        let orig = &mut *orig;
        orig.st.commanded_pos = sk.commanded_pos;
        orig.kin.post(&mut orig.st);
        sk.commanded_pos = orig.st.commanded_pos;
    }


}

fn shaper_note_generation_time(st: &mut SkState, is: &InputShaper) {
    let (mut pre_active, mut post_active) = (0., 0.);
    if (st.active_flags & AF_X) != 0 && is.sx.num_pulses != 0 {
        pre_active = is.sx.pulses[is.sx.num_pulses - 1].t;
        post_active = -is.sx.pulses[0].t;
    }
    if (st.active_flags & AF_Y) != 0 && is.sy.num_pulses != 0 {
        let t = is.sy.pulses[is.sy.num_pulses - 1].t;
        pre_active = if t > pre_active { t } else { pre_active };
        let t = -is.sy.pulses[0].t;
        post_active = if t > post_active { t } else { post_active };
    }
    st.gen_steps_pre_active = pre_active;
    st.gen_steps_post_active = post_active;
}

fn with_shaper<R>(sk: &SkRef, f: impl FnOnce(&mut SkState, &mut InputShaper) -> R) -> R {
    let mut s = sk.borrow_mut();
    let s = &mut *s;
    let is = s.kin.downcast_mut::<InputShaper>().expect("input shaper sk");
    f(&mut s.st, is)
}

pub fn input_shaper_update_sk(sk: &SkRef) {
    with_shaper(sk, |st, is| {
        let orig_flags = is.orig_sk.as_ref().expect("orig sk").borrow().st.active_flags;
        if (orig_flags & (AF_X | AF_Y)) == (AF_X | AF_Y) {
            is.mode = Mode::Xy;
        } else if orig_flags & AF_X != 0 {
            is.mode = Mode::X;
        } else if orig_flags & AF_Y != 0 {
            is.mode = Mode::Y;
        }
        st.active_flags = orig_flags;
        shaper_note_generation_time(st, is);
    })
}

pub fn input_shaper_set_sk(sk: &SkRef, orig_sk: &SkRef) -> i32 {
    let (flags, commanded_pos, last_flush_time, last_move_time, orig_has_post) = {
        let o = orig_sk.borrow();
        (o.st.active_flags, o.st.commanded_pos, o.st.last_flush_time, o.st.last_move_time, o.kin.has_post())
    };
    with_shaper(sk, |st, is| {
        if flags == AF_X {
            is.mode = Mode::X;
        } else if flags == AF_Y {
            is.mode = Mode::Y;
        } else if flags & (AF_X | AF_Y) != 0 {
            is.mode = Mode::Xy;
        } else {
            return -1;
        }
        st.active_flags = flags;
        is.orig_sk = Some(orig_sk.clone());
        st.commanded_pos = commanded_pos;
        st.last_flush_time = last_flush_time;
        st.last_move_time = last_move_time;
        if orig_has_post {
            is.has_post = true;
        }
        0
    })
}

pub fn input_shaper_set_shaper_params(sk: &SkRef, axis: char, n: i32, a: &[f64], t: &[f64]) -> i32 {
    if axis != 'x' && axis != 'y' {
        return -1;
    }
    with_shaper(sk, |st, is| {
        let orig_flags = is.orig_sk.as_ref().expect("orig sk").borrow().st.active_flags;
        let mut status = 0;
        // Ignore input shaper update if the axis is not active
        if orig_flags & (if axis == 'x' { AF_X } else { AF_Y }) != 0 {
            let sp = if axis == 'x' { &mut is.sx } else { &mut is.sy };
            status = init_shaper(n, a, t, sp);
            shaper_note_generation_time(st, is);
        }
        status
    })
}

pub fn input_shaper_alloc() -> SkRef {
    let is = InputShaper { orig_sk: None, mode: Mode::None, has_post: false, sx: ShaperPulses::default(), sy: ShaperPulses::default() };
    StepperKinematics::new(Box::new(is), 0)
}
