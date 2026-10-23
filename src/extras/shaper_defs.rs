// Definitions of the supported input shapers (port of klippy/extras/shaper_defs.py)
//
// Copyright (C) 2020-2021  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use std::f64::consts::PI;

pub const SHAPER_VIBRATION_REDUCTION: f64 = 20.0;
pub const DEFAULT_DAMPING_RATIO: f64 = 0.1;

/// a shaper's (A, T) pulse amplitudes and times
pub type Shaper = (Vec<f64>, Vec<f64>);

pub struct InputShaperCfg {
    pub name: &'static str,
    pub init_func: fn(f64, f64) -> Shaper,
    pub min_freq: f64,
}

pub fn get_none_shaper() -> Shaper {
    (vec![], vec![])
}

fn common(shaper_freq: f64, damping_ratio: f64, k_mult: f64) -> (f64, f64) {
    let df = (1.0 - damping_ratio.powf(2.0)).sqrt();
    let k = (-k_mult * damping_ratio * PI / df).exp();
    let t_d = 1.0 / (shaper_freq * df);
    (k, t_d)
}

pub fn get_zv_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let (k, t_d) = common(shaper_freq, damping_ratio, 1.0);
    (vec![1.0, k], vec![0.0, 0.5 * t_d])
}

pub fn get_zvd_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let (k, t_d) = common(shaper_freq, damping_ratio, 1.0);
    (vec![1.0, 2.0 * k, k.powf(2.0)], vec![0.0, 0.5 * t_d, t_d])
}

pub fn get_mzv_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let df = (1.0 - damping_ratio.powf(2.0)).sqrt();
    let k = (-0.75 * damping_ratio * PI / df).exp();
    let t_d = 1.0 / (shaper_freq * df);

    let a1 = 1.0 - 1.0 / 2f64.sqrt();
    let a2 = (2f64.sqrt() - 1.0) * k;
    let a3 = a1 * k * k;

    (vec![a1, a2, a3], vec![0.0, 0.375 * t_d, 0.75 * t_d])
}

pub fn get_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION; // vibration tolerance
    let (k, t_d) = common(shaper_freq, damping_ratio, 1.0);

    let a1 = 0.25 * (1.0 + v_tol);
    let a2 = 0.5 * (1.0 - v_tol) * k;
    let a3 = a1 * k * k;

    (vec![a1, a2, a3], vec![0.0, 0.5 * t_d, t_d])
}

pub fn get_2hump_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION; // vibration tolerance
    let (k, t_d) = common(shaper_freq, damping_ratio, 1.0);

    let v2 = v_tol.powf(2.0);
    let x = (v2 * ((1.0 - v2).sqrt() + 1.0)).powf(1.0 / 3.0);
    let a1 = (3.0 * x * x + 2.0 * x + 3.0 * v2) / (16.0 * x);
    let a2 = (0.5 - a1) * k;
    let a3 = a2 * k;
    let a4 = a1 * k * k * k;

    (vec![a1, a2, a3, a4], vec![0.0, 0.5 * t_d, t_d, 1.5 * t_d])
}

pub fn get_3hump_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION; // vibration tolerance
    let (k, t_d) = common(shaper_freq, damping_ratio, 1.0);

    let k2 = k * k;
    let a1 = 0.0625 * (1.0 + 3.0 * v_tol + 2.0 * (2.0 * (v_tol + 1.0) * v_tol).sqrt());
    let a2 = 0.25 * (1.0 - v_tol) * k;
    let a3 = (0.5 * (1.0 + v_tol) - 2.0 * a1) * k2;
    let a4 = a2 * k2;
    let a5 = a1 * k2 * k2;

    (vec![a1, a2, a3, a4, a5], vec![0.0, 0.5 * t_d, t_d, 1.5 * t_d, 2.0 * t_d])
}

// min_freq for each shaper is chosen to have projected max_accel ~= 1500
pub const INPUT_SHAPERS: &[InputShaperCfg] = &[
    InputShaperCfg { name: "zv", init_func: get_zv_shaper, min_freq: 21.0 },
    InputShaperCfg { name: "mzv", init_func: get_mzv_shaper, min_freq: 23.0 },
    InputShaperCfg { name: "zvd", init_func: get_zvd_shaper, min_freq: 29.0 },
    InputShaperCfg { name: "ei", init_func: get_ei_shaper, min_freq: 29.0 },
    InputShaperCfg { name: "2hump_ei", init_func: get_2hump_ei_shaper, min_freq: 39.0 },
    InputShaperCfg { name: "3hump_ei", init_func: get_3hump_ei_shaper, min_freq: 48.0 },
];
