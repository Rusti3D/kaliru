// INDX thermal model / fan calibration fits
// (port of klippy/extras/indx/calibration.py)
use crate::error::{KError, KResult};
use crate::gcode::GCodeDispatch;
use crate::printer::Printer;
use crate::pyfmt::py_sum;
use std::rc::Rc;

// Thermal-model calibration (INDX_CALIBRATE) defaults / limits.
pub const MODEL_CAL_MIN_TEMP_DEFAULT: f64 = 60.0; // start heatup from below this (deg C)
pub const MODEL_CAL_MIN_TEMP_CEILING: f64 = 100.0; // MIN_TEMP must stay below this
pub const MODEL_CAL_MAX_TEMP_DEFAULT: f64 = 200.0; // heat up to this (deg C)
pub const MODEL_CAL_HOLD_TIME_DEFAULT: f64 = 10.0; // dwell at MAX_TEMP (s); 0 disables
pub const MODEL_CAL_COOLDOWN_TIME_DEFAULT: f64 = 15.0; // measure cooldown for this long (s)
pub const MODEL_CAL_COOLDOWN_TIME_MAX: f64 = 20.0; // cap on the cooldown window (s)
pub const MODEL_CAL_BASELINE_TIME: f64 = 4.0; // heater-off baseline power window (s)
pub const MODEL_CAL_STEPPER_DWELL: f64 = 0.1; // dwell around stepper enable toggles (s)
pub const MODEL_CAL_POLL_INTERVAL: f64 = 0.2; // reactor pause granularity in waits (s)
pub const MODEL_CAL_FIRST_READ_TIMEOUT: f64 = 5.0; // wait for the first temperature report
pub const MODEL_CAL_PRECOOL_TIMEOUT: f64 = 600.0; // wait-to-drop-below-MIN_TEMP cap (s)
pub const MODEL_CAL_HEATUP_TIMEOUT: f64 = 600.0; // heatup-to-MAX_TEMP cap (s)
pub const MODEL_CAL_MIN_SAMPLES: usize = 20; // refuse to fit with fewer samples
// Reference temperature at which the stored max_power is defined; the fitted
// temperature coefficient corrects max_power away from this point:
//   max_power(T) = max_power + max_power_temp_coeff * (T - ref)
pub const MODEL_MAX_POWER_REF_TEMP: f64 = 200.0;

pub const FAN_CAL_BREAKS_DEFAULT: i64 = 6;
pub const FAN_CAL_HOLD_TIME_DEFAULT: f64 = 5.0;
pub const FAN_CAL_MIN_SPEED_DEFAULT: f64 = 0.20;
pub const FAN_CAL_TEMP_DEFAULT: f64 = 200.0;
pub const FAN_CAL_SETTLE_TIME_DEFAULT: f64 = 10.0;
pub const FAN_CAL_HEATUP_TIMEOUT: f64 = 600.0;
pub const FAN_CAL_MIN_SAMPLES: usize = 2;

/// a calibration sample: (temp, power, pwm, dt, ambient, phase)
#[derive(Clone, Copy, Debug)]
pub struct CalSample {
    pub temp: f64,
    pub power: f64,
    pub pwm: f64,
    pub dt: f64,
    pub ambient: f64,
    pub phase: &'static str,
}

/// the fit_max_power() result
#[derive(Clone, Debug)]
pub struct MaxPowerFit {
    pub max_power: f64,
    pub max_power_se: Option<f64>,
    pub max_power_ref_temp: f64,
    pub max_power_temp_coeff: f64,
    pub max_power_temp_coeff_se: Option<f64>,
    pub max_power_n: usize,
    pub fit_t_min: Option<f64>,
    pub fit_t_max: Option<f64>,
}

pub fn fit_max_power(samples: &[CalSample], baseline_power: f64) -> Result<MaxPowerFit, String> {
    // Bilinear least squares over the heat/hold phases:
    //   power - baseline = P0*pwm + b*pwm*(T - Tref)
    // P0 is max_power at the reference temperature; b (W/K) captures how the
    // delivered power changes as the nozzle heats. Fitted through the origin
    // (P=0 at pwm=0 post-baseline); the pwm factor weights the high-duty
    // heatup, and the wide temperature span there resolves the slope. No
    // external/voltage reference is used.
    let reference = MODEL_MAX_POWER_REF_TEMP;
    let (mut spp, mut spq, mut sqq, mut spy, mut sqy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let mut n = 0;
    let (mut t_min, mut t_max): (Option<f64>, Option<f64>) = (None, None);
    let mut rows = vec![];
    for s in samples {
        if s.phase != "heat" && s.phase != "hold" {
            continue;
        }
        let pwm = s.pwm;
        let temp = s.temp;
        let p1 = pwm;
        let p2 = pwm * (temp - reference);
        let y = s.power - baseline_power;
        spp += p1 * p1;
        spq += p1 * p2;
        sqq += p2 * p2;
        spy += p1 * y;
        sqy += p2 * y;
        rows.push((p1, p2, y));
        n += 1;
        if t_min.map(|t| temp < t).unwrap_or(true) {
            t_min = Some(temp);
        }
        if t_max.map(|t| temp > t).unwrap_or(true) {
            t_max = Some(temp);
        }
    }
    if n < MODEL_CAL_MIN_SAMPLES {
        return Err(format!("not enough heat/hold samples for max_power ({})", n));
    }
    let det = spp * sqq - spq * spq;
    if det == 0.0 {
        return Err("degenerate max_power fit (too little temperature range at full drive)".into());
    }
    let p0 = (spy * sqq - sqy * spq) / det;
    let b = (spp * sqy - spq * spy) / det;
    if !(p0 > 0.0) || !p0.is_finite() || !b.is_finite() {
        return Err(format!("implausible max_power fit (P0={}, coeff={})", fmt_g4(p0), fmt_g4(b)));
    }

    let (mut p0_se, mut b_se) = (None, None);
    let ssr = py_sum(rows.iter().map(|(p1, p2, y)| (y - (p0 * p1 + b * p2)).powf(2.0)));
    if n > 2 {
        let sigma2 = ssr / (n - 2) as f64;
        let var_p0 = sigma2 * sqq / det;
        let var_b = sigma2 * spp / det;
        if var_p0 >= 0.0 {
            p0_se = Some(var_p0.sqrt());
        }
        if var_b >= 0.0 {
            b_se = Some(var_b.sqrt());
        }
    }

    Ok(MaxPowerFit {
        max_power: p0,
        max_power_se: p0_se,
        max_power_ref_temp: reference,
        max_power_temp_coeff: b,
        max_power_temp_coeff_se: b_se,
        max_power_n: n,
        fit_t_min: t_min,
        fit_t_max: t_max,
    })
}

/// the fit_thermal_rc() result
#[derive(Clone, Debug)]
pub struct ThermalFit {
    pub thermal_capacity: f64,
    pub thermal_capacity_ci: Option<f64>,
    pub to_ambient_r: f64,
    pub to_ambient_r_ci: Option<f64>,
    pub rms: f64,
    pub n_fit: usize,
}

pub fn fit_thermal_rc(samples: &[CalSample], baseline_power: f64) -> Result<ThermalFit, String> {
    // Integral-form linear least squares. Integrating C*dT/dt = P - (T-Ta)/R
    // gives  T_i - T_0 = (1/C)*E_i - (1/(RC))*G_i,  with cumulative trapezoids
    //   E_i = integral of (P - baseline), G_i = integral of (T - ambient).
    // Fit y = a*E + d*G through the origin (a=1/C, d=-1/(RC)); never
    // differentiates the noisy temperature signal.
    let fitset: Vec<&CalSample> = samples.iter().filter(|s| s.phase == "heat" || s.phase == "hold" || s.phase == "cooldown").collect();
    if fitset.len() < MODEL_CAL_MIN_SAMPLES {
        return Err(format!("not enough samples for thermal fit ({})", fitset.len()));
    }
    let mut rows = vec![];
    let (mut e_cum, mut g_cum) = (0.0, 0.0);
    let (mut prev_p, mut prev_e) = (0.0, 0.0);
    let t0 = fitset[0].temp;
    for (idx, s) in fitset.iter().enumerate() {
        let p = s.power - baseline_power;
        let e = s.temp - s.ambient;
        if idx > 0 {
            let dt = s.dt;
            e_cum += 0.5 * (p + prev_p) * dt;
            g_cum += 0.5 * (e + prev_e) * dt;
        }
        rows.push((e_cum, g_cum, s.temp - t0));
        prev_p = p;
        prev_e = e;
    }

    let saa = py_sum(rows.iter().map(|(e, _, _)| e * e));
    let sag = py_sum(rows.iter().map(|(e, g, _)| e * g));
    let sgg = py_sum(rows.iter().map(|(_, g, _)| g * g));
    let say = py_sum(rows.iter().map(|(e, _, y)| e * y));
    let sgy = py_sum(rows.iter().map(|(_, g, y)| g * y));
    let det = saa * sgg - sag * sag;
    if det == 0.0 {
        return Err("degenerate thermal fit (singular normal equations)".into());
    }
    let a = (say * sgg - sgy * sag) / det;
    let d = (saa * sgy - sag * say) / det;
    if a <= 0.0 || d >= 0.0 {
        return Err(format!("unphysical thermal fit (C={}, slope d={})", fmt_g4(a), fmt_g4(d)));
    }
    let capacity = 1.0 / a;
    let to_ambient_r = -a / d;

    // Confidence intervals via covariance sigma^2 (X^T X)^-1 + delta method.
    let (mut capacity_ci, mut to_ambient_r_ci) = (None, None);
    let n = rows.len();
    let ssr = py_sum(rows.iter().map(|(e, g, y)| (y - (a * e + d * g)).powf(2.0)));
    if n > 2 {
        let sigma2 = ssr / (n - 2) as f64;
        let var_a = sigma2 * sgg / det;
        let var_d = sigma2 * saa / det;
        let cov_ad = -sigma2 * sag / det;
        let var_c = var_a / a.powf(4.0);
        // R = -a/d : dR/da=-1/d, dR/dd=a/d^2
        let var_r = var_a / d.powf(2.0) + a * a * var_d / d.powf(4.0) - 2.0 * a * cov_ad / d.powf(3.0);
        if var_c >= 0.0 {
            capacity_ci = Some(1.96 * var_c.sqrt());
        }
        if var_r >= 0.0 {
            to_ambient_r_ci = Some(1.96 * var_r.sqrt());
        }
    }

    // Goodness of fit: forward-simulate (same Euler step as the model) and
    // report the RMS temperature error in deg C.
    let mut sq = 0.0;
    let mut t_sim = t0;
    for (idx, s) in fitset.iter().enumerate() {
        if idx > 0 {
            let p = s.power - baseline_power;
            let loss = (t_sim - s.ambient) / to_ambient_r;
            t_sim += (p - loss) / capacity * s.dt;
        }
        sq += (t_sim - s.temp).powf(2.0);
    }
    let rms = (sq / fitset.len() as f64).sqrt();

    Ok(ThermalFit { thermal_capacity: capacity, thermal_capacity_ci: capacity_ci, to_ambient_r, to_ambient_r_ci, rms, n_fit: fitset.len() })
}

/// thermal_model_fit(): both fits merged
pub fn thermal_model_fit(samples: &[CalSample], baseline_power: f64) -> Result<(ThermalFit, MaxPowerFit), String> {
    // Pure function (plain data in/out) so it can run in a forked subprocess.
    let result = fit_thermal_rc(samples, baseline_power)?;
    let max_power = fit_max_power(samples, baseline_power)?;
    Ok((result, max_power))
}

pub fn mean_phase_power(samples: &[CalSample], phase_name: &str) -> Option<f64> {
    let vals: Vec<f64> = samples.iter().filter(|s| s.phase == phase_name).map(|s| s.power).collect();
    if vals.is_empty() {
        return None;
    }
    Some(py_sum(vals.iter().copied()) / vals.len() as f64)
}

pub fn fan_calibration_breakpoints(count: i64, min_speed: f64) -> KResult<Vec<f64>> {
    if count < 2 || !(0.0 < min_speed && min_speed < 1.0) {
        return Err(KError::Other("invalid fan calibration breakpoints".into()));
    }
    let step = (1.0 - min_speed) / (count - 1) as f64;
    Ok((0..count).map(|i| min_speed + step * i as f64).collect())
}

/// a fan calibration sample: (temp, ambient, dt, pwm_energy)
pub type FanSample = (f64, f64, f64, f64);

/// the fit_fan_model() result
#[derive(Clone, Debug)]
pub struct FanFit {
    pub part_cooling_fan_a: f64,
    pub part_cooling_fan_a_ci: Option<f64>,
    pub part_cooling_fan_k: f64,
    pub part_cooling_fan_k_ci: Option<f64>,
    pub rms_power: f64,
    pub r_squared: Option<f64>,
    /// (speed, fan_power, mean_delta, conductance, duration)
    pub points: Vec<(f64, f64, f64, f64, f64)>,
}

pub fn fit_fan_model(samples: &[(f64, Vec<FanSample>)], thermal_capacity: f64, to_ambient_r: f64) -> Result<FanFit, String> {
    let mut points = vec![];
    for (speed, phase_samples) in samples {
        if phase_samples.len() < FAN_CAL_MIN_SAMPLES {
            continue;
        }
        let (mut duration, mut input_energy, mut ambient_energy) = (0.0, 0.0, 0.0);
        let first_temp = phase_samples[0].0;
        let mut last_temp = first_temp;
        let mut last_ambient = phase_samples[0].1;
        for &(temp, ambient, dt, pwm_energy) in &phase_samples[1..] {
            if dt <= 0.0 {
                continue;
            }
            duration += dt;
            input_energy += pwm_energy;
            ambient_energy += 0.5 * ((last_temp - last_ambient) + (temp - ambient)) / to_ambient_r * dt;
            last_temp = temp;
            last_ambient = ambient;
        }
        if duration <= 0.0 {
            continue;
        }
        let fan_power = (input_energy - ambient_energy - thermal_capacity * (last_temp - first_temp)) / duration;
        let mean_delta = py_sum(phase_samples.iter().map(|(t, a, _, _)| t - a)) / phase_samples.len() as f64;
        let conductance = if mean_delta > 0.0 { fan_power / mean_delta } else { 0.0 };
        if fan_power > 0.0 && conductance > 0.0 {
            points.push((*speed, fan_power, mean_delta, conductance, duration));
        }
    }

    if points.len() < 3 {
        return Err(format!("not enough usable fan breakpoints ({})", points.len()));
    }

    let rows: Vec<(f64, f64)> = points.iter().map(|p| (p.0.ln(), p.3.ln())).collect();
    let n = rows.len() as f64;
    let mean_x = py_sum(rows.iter().map(|(x, _)| *x)) / n;
    let mean_y = py_sum(rows.iter().map(|(_, y)| *y)) / n;
    let sxx = py_sum(rows.iter().map(|(x, _)| (x - mean_x).powf(2.0)));
    if sxx == 0.0 {
        return Err("degenerate fan fit".into());
    }
    let k = py_sum(rows.iter().map(|(x, y)| (x - mean_x) * (y - mean_y))) / sxx;
    let intercept = mean_y - k * mean_x;
    let a = (-intercept).exp();
    if !(a > 0.0 && k > 0.0 && a.is_finite() && k.is_finite()) {
        return Err(format!("unphysical fan fit (a={}, k={})", fmt_g4(a), fmt_g4(k)));
    }

    let mut residuals = vec![];
    let mut actual = vec![];
    for &(speed, power, delta, _, _) in &points {
        let predicted = delta / a * speed.powf(k);
        residuals.push(power - predicted);
        actual.push(power);
    }
    let rms = (py_sum(residuals.iter().map(|v| v * v)) / n).sqrt();
    let mean_power = py_sum(actual.iter().copied()) / n;
    let total_sq = py_sum(actual.iter().map(|v| (v - mean_power).powf(2.0)));
    let residual_sq = py_sum(residuals.iter().map(|v| v * v));
    let r_squared = if total_sq > 0.0 { Some(1.0 - residual_sq / total_sq) } else { None };

    let (mut a_ci, mut k_ci) = (None, None);
    if rows.len() > 2 {
        let log_ssr = py_sum(rows.iter().map(|(x, y)| (y - (intercept + k * x)).powf(2.0)));
        let sigma2 = log_ssr / (n - 2.0);
        let k_se = (sigma2 / sxx).sqrt();
        let intercept_se = (sigma2 * (1.0 / n + mean_x * mean_x / sxx)).sqrt();
        k_ci = Some(1.96 * k_se);
        a_ci = Some(1.96 * a * intercept_se);
    }

    Ok(FanFit {
        part_cooling_fan_a: a,
        part_cooling_fan_a_ci: a_ci,
        part_cooling_fan_k: k,
        part_cooling_fan_k_ci: k_ci,
        rms_power: rms,
        r_squared,
        points,
    })
}

/// python's "%.4g"
fn fmt_g4(v: f64) -> String {
    crate::pyformat::percent_format("%.4g", &crate::pyval::PyVal::Tuple(vec![crate::pyval::PyVal::Float(v)])).unwrap_or_default()
}

/// run_fit_in_background(printer, func, *fit_args): the fit runs on a
/// thread (python: a process) while the reactor keeps running
pub fn run_fit_in_background<T: Send + 'static>(printer: &Rc<Printer>, func: impl FnOnce() -> T + Send + 'static) -> KResult<T> {
    let handle = std::thread::spawn(func);
    let reactor = printer.get_reactor();
    let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
    let mut eventtime = reactor.monotonic();
    let mut last_report = eventtime;
    while !handle.is_finished() {
        if eventtime > last_report + 5.0 {
            last_report = eventtime;
            gcode.respond_info("INDX: fitting thermal model...", false);
        }
        eventtime = reactor.pause(eventtime + 0.1)?;
    }
    handle.join().map_err(|_| KError::Other("Error in thermal model fit: fit process died unexpectedly".into()))
}
