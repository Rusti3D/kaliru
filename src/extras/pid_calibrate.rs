// Calibration of heater PID settings (port of klippy/extras/pid_calibrate.py)
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::heaters::{
    prof_set, Control, ControlDualLoopPID, ControlRef, Heater, PrinterHeaters, ProfVal, Profile, PID_PARAM_BASE,
};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyglob::gettempdir;
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::rc::Rc;

const CMD_PID_CALIBRATE_HELP: &str = "Run PID calibration test";

pub struct PIDCalibrate {
    printer: Rc<Printer>,
}

impl PIDCalibrate {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let pc = Rc::new(PIDCalibrate { printer: printer.clone() });
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let p = pc.clone();
        gcode.register_command(
            "PID_CALIBRATE",
            Some(Rc::new(move |g| p.cmd_pid_calibrate(g))),
            false,
            Some(CMD_PID_CALIBRATE_HELP),
        )?;
        Ok(pc)
    }

    #[allow(clippy::too_many_arguments)]
    fn calibrate(
        &self,
        pheaters: &PrinterHeaters,
        heater: &Rc<Heater>,
        target: f64,
        tolerance: f64,
        write_file: i64,
        gcmd: &GCodeCommand,
        calibrate_secondary: bool,
    ) -> KResult<(f64, f64, f64, ControlRef)> {
        let control = heater.get_control();
        let calibrate = if let Some(dual) = control.downcast_ref::<ControlDualLoopPID>() {
            if calibrate_secondary {
                let delta = dual.inner_target_temp - target;
                if delta <= 15.0 {
                    gcmd.respond_raw(&format!(
                        "!! Target calibration temperature of {}°C is <= 15°C below `inner_target_temp`.",
                        target.trunc() as i64
                    ));
                    gcmd.respond_raw("!! Depending on the thermal mass this might prevent successful calibration.");
                }

                gcmd.respond_info(
                    &format!(
                        "Calibrating secondary pid loop around {:.1}C. The inner sensor temperature will oscillate above and below this target during calibration; this is expected.",
                        dual.inner_target_temp
                    ),
                    true,
                );
                ControlAutoTune::new(heater, dual.inner_target_temp, tolerance, calibrate_secondary)?
            } else {
                gcmd.respond_info(&format!("Calibrating primary pid loop (target={:.1})", target), true);
                let calibrate = ControlAutoTune::new(heater, target, tolerance, calibrate_secondary)?;
                let t = calibrate.temp_low - 5.0;
                let max_temp = if t > 0.0 { t } else { 0.0 };
                gcmd.respond_info(
                    &format!("Waiting for heater {} to cool down to {:.1} for calibration.", heater.get_name(), max_temp),
                    true,
                );
                pheaters.set_temperature(heater, max_temp, true)?;
                calibrate
            }
        } else {
            ControlAutoTune::new(heater, target, tolerance, false)?
        };
        let calibrate = Rc::new(calibrate);

        let old_control = heater.set_control(calibrate.clone(), true);
        if let Err(e) = pheaters.set_temperature(heater, target, true) {
            if e.is_command_error() {
                heater.set_control(old_control, false);
            }
            return Err(e);
        }
        heater.set_control(old_control.clone(), false);

        if write_file != 0 {
            let tmpdir = gettempdir();
            let mut fname = format!("{}/heattest.csv", tmpdir);
            if calibrate_secondary {
                fname = format!("{}/heattest_secondary.csv", tmpdir);
            }
            calibrate.write_file(&fname)?;
        }

        if calibrate.check_busy(0.0, 0.0, 0.0) {
            return Err(gcmd.error("pid_calibrate interrupted"));
        }
        // Log and report results
        let (kp, ki, kd) = calibrate.calc_pid()?;
        logging::info(&format!(
            "Autotune: final: Kp={:.6} Ki={:.6} Kd={:.6}, secondary={}",
            kp,
            ki,
            kd,
            crate::pyfmt::py_bool(calibrate_secondary)
        ));
        Ok((kp, ki, kd, old_control))
    }

    fn cmd_pid_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let heater_name = gcmd.get("HEATER")?;
        let target = gcmd.get_float("TARGET", None, Lim::new())?;
        let write_file = gcmd.get_int("WRITE_FILE", Some(0), Lim::new())?;
        let tolerance = gcmd.get_float("TOLERANCE", Some(TUNE_PID_TOL), Lim::new().above(0.0))?;
        let profile_name = gcmd.get_or("PROFILE", "default");
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let heater = match pheaters.lookup_heater(&heater_name) {
            Ok(h) => h,
            Err(KError::Config(m)) => return Err(gcmd.error(m)),
            Err(e) => return Err(e),
        };
        self.printer.lookup::<ToolHead>("toolhead")?.get_last_move_time()?;

        let profile: Profile = Rc::new(RefCell::new(IndexMap::new()));
        let f = PyVal::Float;
        if heater.get_control().is::<ControlDualLoopPID>() {
            // Calibrate the inner (secondary) loop
            let (kp_s, ki_s, kd_s, _) = self.calibrate(&pheaters, &heater, target, tolerance, write_file, gcmd, true)?;

            logging::info(&format!("PID_CALIBRATE (primary) target={:.3}", target));
            // Calibrate the outer (primary) loop1
            let (kp_p, ki_p, kd_p, _) = self.calibrate(&pheaters, &heater, target, tolerance, write_file, gcmd, false)?;

            gcmd.respond_info(
                &format!(
                    "PID parameters for {:.2}\u{b0}C: Primary: pid_Kp={:.3} pid_Ki={:.3} pid_Kd={:.3}\nSecondary: inner_pid_Kp={:.3} inner_pid_Ki={:.3} inner_pid_Kd={:.3}\nHeater: {}\nTolerance: {:.4}\nProfile: {}\nThe SAVE_CONFIG command will update the printer config file\nwith these parameters and restart the printer.",
                    target, kp_p, ki_p, kd_p, kp_s, ki_s, kd_s, heater_name, tolerance, profile_name
                ),
                true,
            );

            for (k, v) in [
                ("pid_target", f(target)),
                ("pid_tolerance", f(tolerance)),
                ("control", PyVal::from("dual_loop_pid")),
                ("pid_kp", f(kp_p)),
                ("pid_ki", f(ki_p)),
                ("pid_kd", f(kd_p)),
                ("inner_pid_kp", f(kp_s)),
                ("inner_pid_ki", f(ki_s)),
                ("inner_pid_kd", f(kd_s)),
                ("smooth_time", PyVal::None),
                ("name", PyVal::from(profile_name.as_str())),
            ] {
                prof_set(&profile, k, v);
            }
        } else {
            let (kp, ki, kd, old_control) = self.calibrate(&pheaters, &heater, target, tolerance, write_file, gcmd, false)?;

            logging::info(&format!("Autotune: final: Kp={:.6} Ki={:.6} Kd={:.6}", kp, ki, kd));
            gcmd.respond_info(
                &format!(
                    "PID parameters for {:.2}\u{b0}C: pid_Kp={:.3} pid_Ki={:.3} pid_Kd={:.3}\nHeater: {}\nTolerance: {:.4}\nProfile: {}\nThe SAVE_CONFIG command will update the printer config file\nwith these parameters and restart the printer.",
                    target, kp, ki, kd, heater_name, tolerance, profile_name
                ),
                true,
            );
            let control = if old_control.get_type() == "pid_v" { "pid_v" } else { "pid" };

            for (k, v) in [
                ("pid_target", f(target)),
                ("pid_tolerance", f(tolerance)),
                ("control", PyVal::from(control)),
                ("pid_kp", f(kp)),
                ("pid_ki", f(ki)),
                ("pid_kd", f(kd)),
                ("smooth_time", PyVal::None),
                ("name", PyVal::from(profile_name.as_str())),
            ] {
                prof_set(&profile, k, v);
            }
        }

        heater.set_control(heater.lookup_control(&profile, true)?, false);
        heater.pm_save_profile(Some(&profile_name), false)
    }
}

impl PrinterObject for PIDCalibrate {
}

pub const TUNE_PID_DELTA: f64 = 5.0;
pub const TUNE_PID_TOL: f64 = 0.02;
pub const TUNE_PID_SAMPLES: usize = 3;
pub const TUNE_PID_MAX_PEAKS: usize = 60;

pub struct ControlAutoTune {
    heater: Rc<Heater>,
    heater_max_power: f64,
    gcode: Rc<GCodeDispatch>,
    powers: RefCell<Vec<f64>>,
    times: RefCell<Vec<f64>>,
    target: f64,
    tolerance: f64,
    temp_high: f64,
    pub temp_low: f64,
    heating: Cell<bool>,
    peak: Cell<f64>,
    peak_times: RefCell<Vec<f64>>,
    peaks: RefCell<Vec<(f64, f64)>>,
    target_crossed: Cell<bool>,
    done: Cell<bool>,
    started: Cell<bool>,
    errored: Cell<bool>,
    data: RefCell<Vec<(f64, f64, f64, f64)>>,
    control: ControlRef,
    calibrate_secondary: bool,
}

fn index_error() -> KError {
    KError::Other("list index out of range".into())
}

impl ControlAutoTune {
    fn new(heater: &Rc<Heater>, target: f64, tolerance: f64, calibrate_secondary: bool) -> KResult<Self> {
        let heater_max_power = heater.get_max_power();
        Ok(ControlAutoTune {
            heater: heater.clone(),
            heater_max_power,
            gcode: heater.printer.lookup::<GCodeDispatch>("gcode")?,
            powers: RefCell::new(vec![heater_max_power]),
            times: RefCell::new(vec![]),
            target,
            tolerance,
            temp_high: target + TUNE_PID_DELTA / 2.0,
            temp_low: target - TUNE_PID_DELTA / 2.0,
            heating: Cell::new(false),
            peak: Cell::new(target),
            peak_times: RefCell::new(vec![]),
            peaks: RefCell::new(vec![]),
            target_crossed: Cell::new(false),
            done: Cell::new(false),
            started: Cell::new(false),
            errored: Cell::new(false),
            data: RefCell::new(vec![]),
            control: heater.get_control(),
            calibrate_secondary,
        })
    }

    fn check_peak(&self, time: f64, temp: f64) {
        // deal with duplicate temps
        if temp == self.peak.get() {
            self.peak_times.borrow_mut().push(time);
        }
        // deal with storing high peak values
        if temp > self.target && temp > self.peak.get() {
            self.peak.set(temp);
            *self.peak_times.borrow_mut() = vec![time];
        }
        // deal with storing low peak values
        if temp < self.target && temp < self.peak.get() {
            self.peak.set(temp);
            *self.peak_times.borrow_mut() = vec![time];
        }
    }

    fn store_peak(&self) -> KResult<()> {
        let pt = std::mem::take(&mut *self.peak_times.borrow_mut());
        if pt.is_empty() {
            return Err(KError::Other("float division by zero".into()));
        }
        let time = crate::pyfmt::py_sum(pt.iter().copied()) / pt.len() as f64;
        self.peaks.borrow_mut().push((time, self.peak.get()));
        self.peak.set(self.target);
        Ok(())
    }

    fn log_info(&self) -> KResult<()> {
        // provide some useful info to the user
        let sample = self.powers.borrow().len();
        let pwm = *self.powers.borrow().last().unwrap();
        let peaks = self.peaks.borrow();
        if peaks.len() < 2 {
            return Err(index_error());
        }
        let asymmetry = (peaks[peaks.len() - 2].1 + peaks[peaks.len() - 1].1) / 2.0 - self.target;
        match self.get_sample_tolerance() {
            None => self.gcode.respond_info(&format!("sample:{} pwm:{:.4} asymmetry:{:.4} tolerance:n/a\n", sample, pwm, asymmetry), true),
            Some(tolerance) => self.gcode.respond_info(
                &format!("sample:{} pwm:{:.4} asymmetry:{:.4} tolerance:{:.4}\n", sample, pwm, asymmetry, tolerance),
                true,
            ),
        }
        Ok(())
    }

    /// get_sample_tolerance(); None is python's False
    fn get_sample_tolerance(&self) -> Option<f64> {
        let powers = self.powers.borrow();
        if powers.len() < TUNE_PID_SAMPLES + 1 {
            return None;
        }
        let p = &powers[powers.len() - (TUNE_PID_SAMPLES + 1)..];
        let max = p[1..].iter().fold(p[0], |a, &b| if b > a { b } else { a });
        let min = p[1..].iter().fold(p[0], |a, &b| if b < a { b } else { a });
        Some(max - min)
    }

    fn converged(&self) -> bool {
        match self.get_sample_tolerance() {
            None => false,
            Some(t) => t <= self.tolerance,
        }
    }

    fn set_power(&self) {
        let peaks = self.peaks.borrow();
        let peak_low = peaks[peaks.len() - 2].1;
        let peak_high = peaks[peaks.len() - 1].1;
        let power = *self.powers.borrow().last().unwrap();
        let mid = power * ((self.target - peak_low) / (peak_high - peak_low));
        if mid * 2.0 > self.heater_max_power {
            // the new power is to high so just return max power
            self.powers.borrow_mut().push(self.heater_max_power);
            return;
        }
        self.powers.borrow_mut().push(mid * 2.0);
    }

    fn finish(&self, time: f64) -> KResult<()> {
        self.heater.set_pwm(time, 0.0)?;
        self.heater.alter_target(0.0);
        self.done.set(true);
        self.heating.set(false);
        Ok(())
    }

    fn write_file(&self, filename: &str) -> KResult<()> {
        let mut out = String::from("time, temp, pwm, target\n");
        let data: Vec<String> = self
            .data
            .borrow()
            .iter()
            .map(|(time, temp, pwm, target)| format!("{:.5}, {:.5}, {:.5}, {:.5}", time, temp, pwm, target))
            .collect();
        out += &data.join("\n");
        let mut peaks = self.peaks.borrow().clone();
        let powers = self.powers.borrow().clone();
        // pop off the
        if peaks.is_empty() {
            return Err(KError::Other("pop from empty list".into()));
        }
        peaks.remove(0);
        let mut samples = vec![];
        for (i, power) in powers.iter().enumerate() {
            let (a, b) = (peaks.get(i * 2).ok_or_else(index_error)?, peaks.get(i * 2 + 1).ok_or_else(index_error)?);
            samples.push((i as f64, a.0, a.1, b.0, b.1, *power));
        }
        out += "\nsample, low time, low, high time, high, max power\n";
        let data: Vec<String> = samples
            .iter()
            .map(|(sample, low_time, low, high_time, high, max_power)| {
                format!("{:.5}, {:.5}, {:.5}, {:.5}, {:.5}, {:.5}", sample, low_time, low, high_time, high, max_power)
            })
            .collect();
        out += &data.join("\n");
        std::fs::write(filename, out).map_err(|e| KError::os(&e, filename))
    }

    fn calc_pid(&self) -> KResult<(f64, f64, f64)> {
        let peaks = self.peaks.borrow();
        let times = self.times.borrow();
        let neg = |v: usize, len: usize| len.checked_sub(v).ok_or_else(index_error);
        let mut temp_diff = 0.0;
        let mut time_diff = 0.0;
        let mut theta = 0.0;
        for i in (1..TUNE_PID_SAMPLES * 2).step_by(2) {
            let (a, b, c) = (neg(i, peaks.len())?, neg(i + 1, peaks.len())?, neg(i + 2, peaks.len())?);
            temp_diff = temp_diff + peaks[a].1 - peaks[b].1;
            time_diff = time_diff + peaks[a].0 - peaks[c].0;
            theta = theta + peaks[a].0 - times[neg(i, times.len())?];
        }
        temp_diff /= TUNE_PID_SAMPLES as f64;
        time_diff /= TUNE_PID_SAMPLES as f64;
        theta /= TUNE_PID_SAMPLES as f64;
        let amplitude = 0.5 * temp_diff.abs();
        let powers = self.powers.borrow();
        let p = &powers[powers.len().saturating_sub(TUNE_PID_SAMPLES)..];
        let power = crate::pyfmt::py_sum(p.iter().copied()) / p.len() as f64;
        // calculate the various parameters
        let ku = 4.0 * power / (PI * amplitude);
        let tu = time_diff;
        let wu = (2.0 * PI) / tu;
        let tau = (PI - theta * wu).tan() / wu;
        let km = -(tau.powi(2) * wu.powi(2) + 1.0).sqrt() / ku;
        // log the extra details
        logging::info(&format!("Ziegler-Nichols constants: Ku={:.6} Tu={:.6}", ku, tu));
        logging::info(&format!("Cohen-Coon constants: Km={:.6} Theta={:.6} Tau={:.6}", km, theta, tau));
        // Use Ziegler-Nichols method to generate PID parameters
        let ti = 0.5 * tu;
        let td = 0.125 * tu;
        let kp = 0.6 * ku * PID_PARAM_BASE;
        let ki = kp / ti;
        let kd = kp * td;
        Ok((kp, ki, kd))
    }
}

impl Control for ControlAutoTune {
    fn temperature_update(&self, read_time: f64, primary_temp: f64, _target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        let temp = if self.calibrate_secondary {
            secondary_temp.ok_or_else(|| KError::Other("'>=' not supported between instances of 'NoneType' and 'float'".into()))?
        } else {
            primary_temp
        };

        // tuning is done, so don't do any more calculations
        if self.done.get() {
            return Ok(());
        }
        // store test data
        self.data.borrow_mut().push((read_time, temp, self.heater.last_pwm_value.get(), self.target));
        // ensure the starting temp is low enough to run the test.
        if !self.started.get() && temp >= self.temp_low {
            self.errored.set(true);
            self.finish(read_time)?;
            self.gcode.respond_info("temperature is too high to start calibration", true);
            return Ok(());
        } else {
            self.started.set(true);
        }
        // ensure the test doesn't run too long
        if self.peaks.borrow().len() > TUNE_PID_MAX_PEAKS {
            self.errored.set(true);
            self.finish(read_time)?;
            self.gcode.respond_info("calibration did not finish in time", true);
            return Ok(());
        }
        // indicate that the target temp has been crossed at-least once
        if temp > self.target && !self.target_crossed.get() {
            self.target_crossed.set(true);
        }
        // only do work if the target temp has been crossed at-least once
        if self.target_crossed.get() {
            // check for a new peak value
            if temp > self.temp_high || temp < self.temp_low {
                self.check_peak(read_time, temp);
            }
            // it's time to calculate and store a high peak
            if self.peak.get() > self.temp_high && temp < self.target {
                self.store_peak()?;
            }
            // it's time to calculate and store a low peak
            if self.peak.get() < self.temp_low && temp > self.target {
                self.store_peak()?;
            }
            // check if the conditions are right to evaluate a new sample
            let peaks = self.peaks.borrow().len() as f64 - 1.0;
            let powers = self.powers.borrow().len() as f64;
            if peaks.rem_euclid(2.0) == 0.0 && (powers * 2.0) == peaks {
                self.log_info()?;
                // check for convergence
                if self.converged() {
                    self.finish(read_time)?;
                    return Ok(());
                }
                self.set_power();
            }
        }
        // turn the heater off
        if self.heating.get() && temp >= self.temp_high {
            self.heating.set(false);
            self.times.borrow_mut().push(read_time);
            self.heater.alter_target(self.temp_low);
        }
        // turn the heater on
        if !self.heating.get() && temp <= self.temp_low {
            self.heating.set(true);
            self.times.borrow_mut().push(read_time);
            self.heater.alter_target(self.temp_high);
        }
        // set the pwm output based on the heater state
        let last_power = *self.powers.borrow().last().unwrap();
        if self.heating.get() {
            match self.control.downcast_ref::<ControlDualLoopPID>() {
                Some(dual) if !self.calibrate_secondary => {
                    let pid = &dual.secondary_pid;
                    let secondary_target_temp = dual.inner_target_temp;
                    let secondary_temp = secondary_temp.ok_or_else(|| {
                        KError::Other("unsupported operand type(s) for -: 'NoneType' and 'float'".into())
                    })?;
                    let (_, bounded_co) = pid.calculate_output(read_time, secondary_temp, secondary_target_temp);
                    let bounded_co = if last_power < bounded_co { last_power } else { bounded_co };
                    self.heater.set_pwm(read_time, bounded_co)
                }
                _ => self.heater.set_pwm(read_time, last_power),
            }
        } else {
            self.heater.set_pwm(read_time, 0.0)
        }
    }

    fn check_busy(&self, eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        if eventtime == 0.0 && smoothed_temp == 0.0 && target_temp == 0.0 {
            return self.errored.get();
        }
        !self.done.get()
    }

    fn update_smooth_time(&self) -> KResult<()> {
        Err(KError::Other("ControlAutoTune.update_smooth_time() missing 1 required positional argument: 'write_to_profile'".into()))
    }

    fn get_profile(&self) -> Profile {
        let p: Profile = Rc::new(RefCell::new(IndexMap::new()));
        p.borrow_mut().insert("name".into(), ProfVal::V(PyVal::from("autotune")));
        p
    }

    fn get_type(&self) -> &'static str {
        "autotune"
    }

}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PIDCalibrate::new(config)?)
}
