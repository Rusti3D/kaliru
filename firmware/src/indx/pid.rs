// INDX heater PID controller (pid.h, pid.c++)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::libm::fabsf;

#[derive(Clone, Copy)]
pub struct PidParams {
    pub kp: f32,
    pub ti: f32,
    pub td: f32,
    pub b: f32,
    pub tt: f32,
    pub i_window: f32,
    pub i_limit: f32,
}

impl PidParams {
    pub const fn zero() -> PidParams {
        PidParams { kp: 0.0, ti: 0.0, td: 0.0, b: 0.0, tt: 0.0, i_window: 0.0, i_limit: 0.0 }
    }
}

// std::min / std::max of floats (b < a ? b : a, a < b ? b : a)
fn min(a: f32, b: f32) -> f32 {
    if b < a {
        b
    } else {
        a
    }
}
fn max(a: f32, b: f32) -> f32 {
    if a < b {
        b
    } else {
        a
    }
}

pub struct PidController {
    params: PidParams,
    set_point: Option<f32>,
    output: f32,
    integrator: f32,
}

impl PidController {
    pub const fn new(params: PidParams) -> PidController {
        PidController { params, set_point: None, output: 0.0, integrator: 0.0 }
    }

    pub fn update_params(&mut self, params: PidParams) {
        self.params = params;
        self.integrator = 0.0;
    }

    pub fn update_set_point(&mut self, set_point: Option<f32>) {
        if self.set_point != set_point {
            self.integrator = 0.0;
        }
        self.set_point = set_point;
    }

    pub fn set_point(&self) -> Option<f32> {
        self.set_point
    }

    pub fn step(&mut self, current_temperature: f32, dt: f32) -> f32 {
        let Some(sp) = self.set_point else {
            self.output = 0.0;
            self.integrator = 0.0;
            return self.output;
        };

        let params = &self.params;

        let p = params.kp * (params.b * sp - current_temperature);

        let error = sp - current_temperature;
        // (C: the ti test in double)
        if (params.ti as f64) < 0.000001 || fabsf(error) > params.i_window {
            self.integrator = 0.0;
        } else {
            self.integrator = max(0.0, min(params.i_limit, self.integrator + params.kp * error / params.ti * dt));
        }

        let d: f32 = 0.0;

        self.output = p + self.integrator + d;

        max(0.0, min(1.0, self.output))
    }
}
