// Support fans that are enabled when temperature exceeds a set threshold
// (port of klippy/extras/temperature_fan.py)
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim, ListParser};
use crate::error::{KError, KResult};
use crate::extras::fan::Fan;
use crate::extras::heaters::{load_heaters, TempSensor};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mathutil::np_interp;
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_round};
use crate::pyval::PyVal;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const KELVIN_TO_CELSIUS: f64 = -273.15;
pub const MAX_FAN_TIME: f64 = 5.0;
pub const AMBIENT_TEMP: f64 = 25.0;
pub const PID_PARAM_BASE: f64 = 255.0;

const CMD_SET_TEMPERATURE_FAN_TARGET_HELP: &str = "Sets a temperature fan target and fan speed limits";

/// the control algorithm of a temperature fan
trait TfControl {
    fn temperature_callback(&self, tf: &TemperatureFan, read_time: f64, temp: f64) -> KResult<()>;
    fn get_type(&self) -> &'static str;
}

pub struct TemperatureFan {
    pub fan: Rc<Fan>,
    min_temp: f64,
    max_temp: f64,
    _sensor: Rc<dyn TempSensor>,
    speed_delay: Cell<f64>,
    max_speed: Cell<f64>,
    min_speed: Cell<f64>,
    last_temp: Cell<f64>,
    target_temp_conf: Cell<f64>,
    target_temp: Cell<f64>,
    control: RefCell<Option<Box<dyn TfControl>>>,
    next_speed_time: Cell<f64>,
    last_speed_value: Cell<f64>,
}

impl TemperatureFan {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let printer = config.get_printer();
        let fan = Fan::new(config, 1.0)?;
        let min_temp = config.getfloat("min_temp", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
        let max_temp = config.getfloat("max_temp", Dflt::Required, Lim::new().above(min_temp))?;
        let pheaters = load_heaters(config)?;
        let sensor = pheaters.setup_sensor(config)?;
        sensor.setup_minmax(min_temp, max_temp)?;
        let tf = Rc::new(TemperatureFan {
            fan,
            min_temp,
            max_temp,
            _sensor: sensor.clone(),
            speed_delay: Cell::new(0.0),
            max_speed: Cell::new(0.0),
            min_speed: Cell::new(0.0),
            last_temp: Cell::new(0.0),
            target_temp_conf: Cell::new(0.0),
            target_temp: Cell::new(0.0),
            control: RefCell::new(None),
            next_speed_time: Cell::new(0.0),
            last_speed_value: Cell::new(0.0),
        });
        let w = Rc::downgrade(&tf);
        sensor.setup_callback(Rc::new(move |read_time, temp| match w.upgrade() {
            Some(tf) => tf.temperature_callback(read_time, temp),
            None => Ok(()),
        }));
        pheaters.register_sensor(config, tf.clone(), None)?;
        tf.speed_delay.set(sensor.get_report_time_delta());
        let max_speed_conf = config.getfloat("max_speed", 1.0, Lim::new().above(0.0).max(1.0))?;
        tf.max_speed.set(max_speed_conf);
        let min_speed_conf = config.getfloat("min_speed", 0.3, Lim::new().min(0.0).max(1.0))?;
        tf.min_speed.set(min_speed_conf);
        let target_temp_conf = config.getfloat(
            "target_temp",
            if max_temp > 40.0 { 40.0 } else { max_temp },
            Lim::new().min(min_temp).max(max_temp),
        )?;
        tf.target_temp_conf.set(target_temp_conf);
        tf.target_temp.set(target_temp_conf);
        let algo = config.getchoice("control", &[("watermark", 0), ("pid", 1), ("curve", 2)], Dflt::Required)?;
        let control: Box<dyn TfControl> = match algo {
            0 => Box::new(ControlBangBang::new(config)?),
            1 => Box::new(ControlPID::new(&tf, config)?),
            _ => Box::new(ControlCurve::new(&tf, config)?),
        };
        *tf.control.borrow_mut() = Some(control);
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let t = tf.clone();
        gcode.register_mux_command(
            "SET_TEMPERATURE_FAN_TARGET",
            "TEMPERATURE_FAN",
            Some(&name),
            Rc::new(move |g| t.cmd_set_temperature_fan_target(g)),
            Some(CMD_SET_TEMPERATURE_FAN_TARGET_HELP),
        )?;
        Ok(tf)
    }

    pub fn set_tf_speed(&self, read_time: f64, mut value: f64) -> KResult<()> {
        if value <= 0.0 {
            value = 0.0;
        } else if value < self.min_speed.get() {
            value = self.min_speed.get();
        }
        if self.target_temp.get() <= 0.0 {
            value = 0.0;
        }
        if (read_time < self.next_speed_time.get() || self.last_speed_value.get() == 0.0)
            && (value - self.last_speed_value.get()).abs() < 0.05
        {
            // No significant change in value - can suppress update
            return Ok(());
        }
        let speed_time = read_time + self.speed_delay.get();
        self.next_speed_time.set(speed_time + 0.75 * MAX_FAN_TIME);
        self.last_speed_value.set(value);
        self.fan.set_speed(value, Some(speed_time))
    }

    fn temperature_callback(&self, read_time: f64, temp: f64) -> KResult<()> {
        self.last_temp.set(temp);
        let control = self.control.borrow();
        match control.as_ref() {
            Some(c) => c.temperature_callback(self, read_time, temp),
            None => Err(KError::Other("'TemperatureFan' object has no attribute 'control'".into())),
        }
    }

    pub fn get_min_speed(&self) -> f64 {
        self.min_speed.get()
    }

    pub fn get_max_speed(&self) -> f64 {
        self.max_speed.get()
    }

    pub fn is_adc_faulty(&self) -> bool {
        self.last_temp.get() > self.max_temp || self.last_temp.get() < self.min_temp
    }

    fn control_type(&self) -> &'static str {
        self.control.borrow().as_ref().map(|c| c.get_type()).unwrap_or("")
    }

    fn cmd_set_temperature_fan_target(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let temp = gcmd.get_float_opt("TARGET", Lim::new())?;
        if temp.is_some() && self.control_type() == "curve" {
            return Err(gcmd.error("Setting Target not supported for control curve"));
        }
        let min_speed = gcmd.get_float("MIN_SPEED", Some(self.min_speed.get()), Lim::new())?;
        let max_speed = gcmd.get_float("MAX_SPEED", Some(self.max_speed.get()), Lim::new())?;
        if min_speed > max_speed {
            return Err(KError::command(format!(
                "Requested min speed ({:.1}) is greater than max speed ({:.1})",
                min_speed, max_speed
            )));
        }
        self.set_min_speed(min_speed)?;
        self.set_max_speed(max_speed)?;
        self.set_temp(temp.unwrap_or(self.target_temp_conf.get()))
    }

    pub fn set_temp(&self, degrees: f64) -> KResult<()> {
        if degrees != 0.0 && (degrees < self.min_temp || degrees > self.max_temp) {
            return Err(KError::command(format!(
                "Requested temperature ({:.1}) out of range ({:.1}:{:.1})",
                degrees, self.min_temp, self.max_temp
            )));
        }
        self.target_temp.set(degrees);
        Ok(())
    }

    pub fn set_min_speed(&self, speed: f64) -> KResult<()> {
        if speed != 0.0 && (speed < 0.0 || speed > 1.0) {
            return Err(KError::command(format!("Requested min speed ({:.1}) out of range (0.0 : 1.0)", speed)));
        }
        self.min_speed.set(speed);
        Ok(())
    }

    pub fn set_max_speed(&self, speed: f64) -> KResult<()> {
        if speed != 0.0 && (speed < 0.0 || speed > 1.0) {
            return Err(KError::command(format!("Requested max speed ({:.1}) out of range (0.0 : 1.0)", speed)));
        }
        self.max_speed.set(speed);
        Ok(())
    }
}

impl PrinterObject for TemperatureFan {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        let mut status = match self.fan.get_status(eventtime) {
            PyVal::Dict(d) => d,
            _ => Default::default(),
        };
        status.insert("temperature".to_string(), PyVal::Float(py_round(self.last_temp.get(), 2)));
        status.insert("target".to_string(), PyVal::Float(self.target_temp.get()));
        status.insert("control".to_string(), PyVal::from(self.control_type()));
        Some(PyVal::Dict(status))
    }
    fn get_temp(&self, _eventtime: f64) -> Option<(f64, f64)> {
        Some((self.last_temp.get(), self.target_temp.get()))
    }
    fn fan(&self) -> Option<Rc<Fan>> {
        Some(self.fan.clone())
    }
    fn teardown(&self) {
        *self.control.borrow_mut() = None;
        self.fan.teardown();
    }
}

// ----------------------------------------------------------------------
// Bang-bang control algo
// ----------------------------------------------------------------------

struct ControlBangBang {
    reverse: bool,
    max_delta: f64,
    heating: Cell<bool>,
}

impl ControlBangBang {
    fn new(config: &ConfigWrapper) -> KResult<Self> {
        let reverse = config.getboolean("reverse", false)?;
        let max_delta = config.getfloat("max_delta", 2.0, Lim::new().above(0.0))?;
        Ok(ControlBangBang { reverse, max_delta, heating: Cell::new(false) })
    }
}

impl TfControl for ControlBangBang {
    fn temperature_callback(&self, tf: &TemperatureFan, read_time: f64, temp: f64) -> KResult<()> {
        let target_temp = tf.target_temp.get();
        if self.heating.get() != self.reverse && temp >= target_temp + self.max_delta {
            self.heating.set(self.reverse);
        } else if self.heating.get() == self.reverse && temp <= target_temp - self.max_delta {
            self.heating.set(!self.reverse);
        }
        if self.heating.get() {
            tf.set_tf_speed(read_time, 0.0)
        } else {
            tf.set_tf_speed(read_time, tf.get_max_speed())
        }
    }
    fn get_type(&self) -> &'static str {
        "watermark"
    }
}

// ----------------------------------------------------------------------
// Proportional Integral Derivative (PID) control algo
// ----------------------------------------------------------------------

pub const PID_SETTLE_DELTA: f64 = 1.0;
pub const PID_SETTLE_SLOPE: f64 = 0.1;

struct ControlPID {
    reverse: bool,
    kp: f64,
    ki: f64,
    kd: f64,
    min_deriv_time: f64,
    temp_integ_max: f64,
    prev_temp: Cell<f64>,
    prev_temp_time: Cell<f64>,
    prev_temp_deriv: Cell<f64>,
    prev_temp_integ: Cell<f64>,
}

impl ControlPID {
    fn new(tf: &TemperatureFan, config: &ConfigWrapper) -> KResult<Self> {
        let reverse = config.getboolean("reverse", false)?;
        let kp = config.getfloat("pid_Kp", Dflt::Required, Lim::new())? / PID_PARAM_BASE;
        let ki = config.getfloat("pid_Ki", Dflt::Required, Lim::new())? / PID_PARAM_BASE;
        let kd = config.getfloat("pid_Kd", Dflt::Required, Lim::new())? / PID_PARAM_BASE;
        let min_deriv_time = config.getfloat("pid_deriv_time", 2.0, Lim::new().above(0.0))?;
        let imax = config.getfloat("pid_integral_max", tf.get_max_speed(), Lim::new().min(0.0))?;
        if ki == 0.0 {
            return Err(KError::Other("float division by zero".into()));
        }
        Ok(ControlPID {
            reverse,
            kp,
            ki,
            kd,
            min_deriv_time,
            temp_integ_max: imax / ki,
            prev_temp: Cell::new(AMBIENT_TEMP),
            prev_temp_time: Cell::new(0.0),
            prev_temp_deriv: Cell::new(0.0),
            prev_temp_integ: Cell::new(0.0),
        })
    }
}

impl TfControl for ControlPID {
    fn temperature_callback(&self, tf: &TemperatureFan, read_time: f64, temp: f64) -> KResult<()> {
        let target_temp = tf.target_temp.get();
        let time_diff = read_time - self.prev_temp_time.get();
        // Calculate change of temperature
        let temp_diff = temp - self.prev_temp.get();
        let temp_deriv = if time_diff >= self.min_deriv_time {
            temp_diff / time_diff
        } else {
            (self.prev_temp_deriv.get() * (self.min_deriv_time - time_diff) + temp_diff) / self.min_deriv_time
        };
        // Calculate accumulated temperature "error"
        let temp_err = target_temp - temp;
        let mut temp_integ = self.prev_temp_integ.get() + temp_err * time_diff;
        temp_integ = py_max(0.0, py_min(self.temp_integ_max, temp_integ));
        // Calculate output
        let co = self.kp * temp_err + self.ki * temp_integ - self.kd * temp_deriv;
        let bounded_co = py_max(0.0, py_min(tf.get_max_speed(), co));
        if !self.reverse {
            tf.set_tf_speed(read_time, py_max(tf.get_min_speed(), tf.get_max_speed() - bounded_co))?;
        } else {
            tf.set_tf_speed(read_time, py_max(tf.get_min_speed(), bounded_co))?;
        }
        // Store state for next measurement
        self.prev_temp.set(temp);
        self.prev_temp_time.set(read_time);
        self.prev_temp_deriv.set(temp_deriv);
        if co == bounded_co {
            self.prev_temp_integ.set(temp_integ);
        }
        Ok(())
    }
    fn get_type(&self) -> &'static str {
        "pid"
    }
}

// ----------------------------------------------------------------------
// Curve control algo
// ----------------------------------------------------------------------

struct ControlCurve {
    /// [temps, speeds] with the heating / cooling hysteresis applied
    curve_heating: [Vec<f64>; 2],
    curve_cooling: [Vec<f64>; 2],
}

impl ControlCurve {
    fn new(tf: &TemperatureFan, config: &ConfigWrapper) -> KResult<Self> {
        let raw = config.getlists("points", Dflt::Required, &[",", "\n"], Some(2), ListParser::Float, true)?;
        let mut points: Vec<(f64, f64)> = raw
            .as_ref()
            .and_then(|v| v.items())
            .unwrap_or(&[])
            .iter()
            .map(|p| {
                let i = p.items().unwrap_or(&[]);
                (i[0].as_f64().unwrap_or(0.0), i[1].as_f64().unwrap_or(0.0))
            })
            .collect();
        // list.sort(key=...) is stable
        points.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        if points.len() < 2 {
            return Err(KError::config("At least two points must be defined for curve in temperature_fan."));
        }
        // (every point has two values: getlists(count=2) checked it)
        let tuple_le = |a: &(f64, f64), b: &(f64, f64)| a.0 < b.0 || (a.0 == b.0 && a.1 <= b.1);
        if !(0..points.len() - 1).all(|i| tuple_le(&points[i], &points[i + 1])) {
            return Err(KError::config("The fan curve must be monotonically increasing."));
        }

        let temp_values: Vec<f64> = points.iter().map(|p| p.0).collect();
        let pwm_values: Vec<f64> = points.iter().map(|p| p.1).collect();

        let mut distinct = temp_values.clone();
        distinct.dedup();
        if temp_values.len() > distinct.len() {
            return Err(KError::config("Temperature may not exist twice in curve table."));
        }
        if temp_values[temp_values.len() - 1] > tf.target_temp.get() {
            return Err(KError::config("Temperature in point may not exceed target_temp."));
        }
        if temp_values[0] < tf.min_temp {
            return Err(KError::config("Temperature in point may not fall below min_temp."));
        }
        if pwm_values[pwm_values.len() - 1] > tf.get_max_speed() {
            return Err(KError::config("Speed in point may not exceed max_speed."));
        }
        if pwm_values[0] < tf.get_min_speed() {
            return Err(KError::config("Speed in point may not fall below min_speed."));
        }

        if points[0].0 > tf.min_temp {
            points.insert(0, (tf.min_temp, points[0].1));
        }
        if points[points.len() - 1].0 < tf.max_temp {
            let last = points[points.len() - 1].1;
            points.push((tf.max_temp, last));
        }

        let smooth_readings = config.getint_opt("smooth_readings", Lim::new().min(0))?;
        if smooth_readings.is_some() {
            config.deprecate("smooth_readings", None);
        }

        let cooling_hysteresis = config.getfloat("cooling_hysteresis", 0.0, Lim::new())?;
        let heating_hysteresis = config.getfloat("heating_hysteresis", 0.0, Lim::new())?;
        let temps: Vec<f64> = points.iter().map(|p| p.0).collect();
        let speeds: Vec<f64> = points.iter().map(|p| p.1).collect();
        let curve_heating = [temps.iter().map(|t| t + heating_hysteresis).collect(), speeds.clone()];
        let curve_cooling = [temps.iter().map(|t| t - cooling_hysteresis).collect(), speeds];
        Ok(ControlCurve { curve_heating, curve_cooling })
    }
}

impl TfControl for ControlCurve {
    fn temperature_callback(&self, tf: &TemperatureFan, read_time: f64, temp: f64) -> KResult<()> {
        let current_speed = tf.last_speed_value.get();
        let upper_temp = np_interp(current_speed, &self.curve_heating[1], &self.curve_heating[0]);
        let lower_temp = np_interp(current_speed, &self.curve_cooling[1], &self.curve_cooling[0]);

        let next_speed = if temp < lower_temp {
            np_interp(temp, &self.curve_cooling[0], &self.curve_cooling[1])
        } else if temp > upper_temp {
            np_interp(temp, &self.curve_heating[0], &self.curve_heating[1])
        } else {
            current_speed
        };

        tf.set_tf_speed(read_time, next_speed)
    }
    fn get_type(&self) -> &'static str {
        "curve"
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(TemperatureFan::new(config)?)
}
