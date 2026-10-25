// Support for servos (port of klippy/extras/servo.py)
//
// Copyright (C) 2017-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::extras::output_pin::{Action, GCodeRequestQueue};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::PwmPin;
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const SERVO_SIGNAL_PERIOD: f64 = 0.020;

const CMD_SET_SERVO_HELP: &str = "Set servo angle";

/// the pulse width conversions of a servo
struct PwmCalc {
    min_width: f64,
    max_width: f64,
    max_angle: f64,
    angle_to_width: f64,
    width_to_value: f64,
}

impl PwmCalc {
    fn from_angle(&self, angle: f64) -> f64 {
        let angle = py_max(0.0, py_min(self.max_angle, angle));
        let width = self.min_width + angle * self.angle_to_width;
        width * self.width_to_value
    }

    fn from_pulse_width(&self, mut width: f64) -> f64 {
        if width != 0.0 {
            width = py_max(self.min_width, py_min(self.max_width, width));
        }
        width * self.width_to_value
    }
}

pub struct PrinterServo {
    calc: PwmCalc,
    last_value: Cell<f64>,
    mcu_servo: Rc<dyn PwmPin>,
    gcrq: RefCell<Option<Rc<GCodeRequestQueue>>>,
}

impl PrinterServo {
    /// _get_pwm_from_angle(angle)
    pub fn get_pwm_from_angle(&self, angle: f64) -> f64 {
        self.calc.from_angle(angle)
    }

    /// _get_pwm_from_pulse_width(width)
    pub fn get_pwm_from_pulse_width(&self, width: f64) -> f64 {
        self.calc.from_pulse_width(width)
    }

    /// self.max_angle
    pub fn max_angle(&self) -> f64 {
        self.calc.max_angle
    }

    /// self.gcrq.send_async_request(value, print_time)
    pub fn send_async_request(&self, value: f64, print_time: Option<f64>) -> KResult<()> {
        let gcrq = self.gcrq.borrow().clone().ok_or_else(|| crate::error::KError::Other("servo request queue".into()))?;
        gcrq.send_async_request(value, print_time)
    }
}

impl PrinterServo {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let min_width = config.getfloat("minimum_pulse_width", 0.001, Lim::new().above(0.0).below(SERVO_SIGNAL_PERIOD))?;
        let max_width = config.getfloat("maximum_pulse_width", 0.002, Lim::new().above(min_width).below(SERVO_SIGNAL_PERIOD))?;
        let max_angle = config.getfloat("maximum_servo_angle", 180.0, Lim::new())?;
        let angle_to_width = (max_width - min_width) / max_angle;
        let width_to_value = 1.0 / SERVO_SIGNAL_PERIOD;
        let iangle = config.getfloat_opt("initial_angle", Lim::new().min(0.0).max(360.0))?;
        let calc = PwmCalc { min_width, max_width, max_angle, angle_to_width, width_to_value };
        let initial_pwm = match iangle {
            Some(a) => calc.from_angle(a),
            None => {
                let iwidth = config.getfloat("initial_pulse_width", 0.0, Lim::new().min(0.0).max(max_width))?;
                calc.from_pulse_width(iwidth)
            }
        };
        // Setup mcu_servo pin
        let mcu_servo = crate::pins::setup_pwm_pin(&printer, &config.get("pin", Dflt::Required)?)?;
        mcu_servo.setup_max_duration(0.0);
        mcu_servo.setup_cycle_time(SERVO_SIGNAL_PERIOD, false)?;
        mcu_servo.setup_start_value(initial_pwm, 0.0);
        let servo = Rc::new(PrinterServo { calc, last_value: Cell::new(0.0), mcu_servo: mcu_servo.clone(), gcrq: RefCell::new(None) });
        // Create gcode request queue
        let w = Rc::downgrade(&servo);
        let gcrq = GCodeRequestQueue::new(
            config,
            &mcu_servo.get_mcu(),
            Rc::new(move |print_time, value| match w.upgrade() {
                Some(s) => s.set_pwm(print_time, value),
                None => Ok(None),
            }),
        )?;
        *servo.gcrq.borrow_mut() = Some(gcrq);
        // Register commands
        let servo_name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let ss = servo.clone();
        gcode.register_mux_command("SET_SERVO", "SERVO", Some(&servo_name), Rc::new(move |g| ss.cmd_set_servo(g)), Some(CMD_SET_SERVO_HELP))?;
        Ok(servo)
    }

    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<Option<(Action, f64)>> {
        if value == self.last_value.get() {
            return Ok(Some((Action::Discard, 0.0)));
        }
        self.last_value.set(value);
        self.mcu_servo.set_pwm(print_time, value)?;
        Ok(None)
    }

    fn cmd_set_servo(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let value = match gcmd.get_float_opt("WIDTH", Lim::new())? {
            Some(width) => self.calc.from_pulse_width(width),
            None => {
                let angle = gcmd.get_float("ANGLE", None, Lim::new())?;
                self.calc.from_angle(angle)
            }
        };
        self.gcrq.borrow().clone().expect("gcrq").queue_gcode_request(value)
    }
}

impl PrinterObject for PrinterServo {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::default();
        d.insert("value".into(), PyVal::Float(self.last_value.get()));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.gcrq.borrow_mut() = None;
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterServo::new(config)?)
}
