// Extruder stepper pulse time generation (port of klippy/chelper/kin_extruder.c)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::itersolve::{Kinematic, SkRef, SkState, StepperKinematics, AF_X};
use super::trapq::MoveRef;
use std::any::Any;

#[derive(Clone, Copy, Default)]
struct PaParams {
    pressure_advance: f64,
    active_print_time: f64,
}

// Without pressure advance, the extruder stepper position is:
//     extruder_position(t) = nominal_position(t)
// When pressure advance is enabled, additional filament is pushed
// into the extruder during acceleration (and retracted during
// deceleration). The formula is:
//     pa_position(t) = (nominal_position(t)
//                       + pressure_advance * nominal_velocity(t))
// Which is then "smoothed" using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(pa_position(x) * (smooth_time/2 - abs(t-x)) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//         / ((smooth_time/2)**2))

// Calculate the definitive integral of the motion formula:
//   position(t) = base + t * (start_v + t * half_accel)
fn extruder_integrate(base: f64, start_v: f64, half_accel: f64, start: f64, end: f64) -> f64 {
    let half_v = 0.5 * start_v;
    let sixth_a = (1. / 3.) * half_accel;
    let si = start * (base + start * (half_v + start * sixth_a));
    let ei = end * (base + end * (half_v + end * sixth_a));
    ei - si
}

// Calculate the definitive integral of time weighted position:
//   weighted_position(t) = t * (base + t * (start_v + t * half_accel))
fn extruder_integrate_time(base: f64, start_v: f64, half_accel: f64, start: f64, end: f64) -> f64 {
    let half_b = 0.5 * base;
    let third_v = (1. / 3.) * start_v;
    let eighth_a = 0.25 * half_accel;
    let si = start * start * (half_b + start * (third_v + start * eighth_a));
    let ei = end * end * (half_b + end * (third_v + end * eighth_a));
    ei - si
}

// Calculate the definitive integral of extruder for a given move
fn pa_move_integrate(m: &MoveRef, pa_list: &[PaParams], mut base: f64, mut start: f64, mut end: f64, time_offset: f64) -> f64 {
    if start < 0. {
        start = 0.;
    }
    if end > m.move_t {
        end = m.move_t;
    }
    // Determine pressure_advance value
    let can_pressure_advance = m.axes_r.y != 0.;
    let mut pressure_advance = 0.;
    if can_pressure_advance {
        let mut i = pa_list.len() - 1;
        while pa_list[i].active_print_time > m.print_time && i != 0 {
            i -= 1;
        }
        pressure_advance = pa_list[i].pressure_advance;
    }
    // Calculate base position and velocity with pressure advance
    base += pressure_advance * m.start_v;
    let start_v = m.start_v + pressure_advance * 2. * m.half_accel;
    // Calculate definitive integral
    let ha = m.half_accel;
    let iext = extruder_integrate(base, start_v, ha, start, end);
    let wgt_ext = extruder_integrate_time(base, start_v, ha, start, end);
    wgt_ext - time_offset * iext
}

// Calculate the definitive integral of the extruder over a range of moves
fn pa_range_integrate(m: MoveRef, move_time: f64, pa_list: &[PaParams], hst: f64) -> f64 {
    // Calculate integral for the current move
    let mut res = 0.;
    let mut start = move_time - hst;
    let mut end = move_time + hst;
    let start_base = m.start_pos.x;
    res += pa_move_integrate(&m, pa_list, 0., start, move_time, start);
    res -= pa_move_integrate(&m, pa_list, 0., move_time, end, end);
    // Integrate over previous moves
    let mut prev = m;
    while start < 0. {
        prev = prev.prev();
        start += prev.move_t;
        let base = prev.start_pos.x - start_base;
        res += pa_move_integrate(&prev, pa_list, base, start, prev.move_t, start);
    }
    // Integrate over future moves
    let mut m = m;
    while end > m.move_t {
        end -= m.move_t;
        m = m.next();
        let base = m.start_pos.x - start_base;
        res -= pa_move_integrate(&m, pa_list, base, 0., end, end);
    }
    res
}

pub struct ExtruderStepper {
    pa_list: Vec<PaParams>,
    half_smooth_time: f64,
    inv_half_smooth_time2: f64,
}

impl Kinematic for ExtruderStepper {
    fn calc_position(&self, _sk: &SkState, m: MoveRef, move_time: f64) -> f64 {
        let hst = self.half_smooth_time;
        if hst == 0. {
            // Pressure advance not enabled
            return m.start_pos.x + m.get_distance(move_time);
        }
        // Apply pressure advance and average over smooth_time
        let start_x = m.start_pos.x;
        let area = pa_range_integrate(m, move_time, &self.pa_list, hst);
        start_x + area * self.inv_half_smooth_time2
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

pub fn extruder_set_pressure_advance(sk: &SkRef, print_time: f64, pressure_advance: f64, smooth_time: f64) {
    let mut sk = sk.borrow_mut();
    let sk = &mut *sk;
    let es = sk.kin.as_any_mut().downcast_mut::<ExtruderStepper>().expect("extruder stepper");
    let hst = smooth_time * 0.5;
    let old_hst = es.half_smooth_time;
    es.half_smooth_time = hst;
    sk.st.gen_steps_pre_active = hst;
    sk.st.gen_steps_post_active = hst;

    // Cleanup old pressure advance parameters
    let cleanup_time = sk.st.last_flush_time - if old_hst > hst { old_hst } else { hst };
    while es.pa_list.len() > 1 {
        if es.pa_list[1].active_print_time >= cleanup_time {
            break;
        }
        es.pa_list.remove(0);
    }

    if hst == 0. {
        return;
    }
    es.inv_half_smooth_time2 = 1. / (hst * hst);

    if es.pa_list.last().unwrap().pressure_advance == pressure_advance {
        // Retain old pa_params
        return;
    }
    // Add new pressure advance parameters
    es.pa_list.push(PaParams { pressure_advance, active_print_time: print_time });
}

pub fn extruder_stepper_alloc() -> SkRef {
    let es = ExtruderStepper { pa_list: vec![PaParams::default()], half_smooth_time: 0., inv_half_smooth_time2: 0. };
    StepperKinematics::new(Box::new(es), AF_X)
}
