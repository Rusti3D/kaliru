// Support for PWM driven LEDs (port of klippy/extras/led.py)
//
// Copyright (C) 2019-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::output_pin::{lookup_template_eval, PrinterTemplateEvaluator, TemplateCallback, TemplateFlush};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::PwmPin;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use crate::reactor::{ReactorMutex, NOW};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// an LED color; python keeps tuples (a template may give 5 values)
pub type Color = Vec<f64>;

/// update_func(led_state, print_time)
pub type UpdateFunc = Rc<dyn Fn(&[Color], Option<f64>) -> KResult<()>>;

const CMD_SET_LED_HELP: &str = "Set the color of an LED";
const CMD_SET_LED_TEMPLATE_HELP: &str = "Assign a display_template to an LED";

// Helper code for common LED initialization and control
pub struct LEDHelper {
    me: Weak<LEDHelper>,
    printer: Rc<Printer>,
    mutex: Rc<ReactorMutex>,
    update_func: UpdateFunc,
    led_count: i64,
    need_transmit: Cell<bool>,
    led_state: RefCell<Vec<Color>>,
    template_eval: Rc<PrinterTemplateEvaluator>,
    tcallbacks: RefCell<Vec<TemplateCallback>>,
    check_transmit_cb: RefCell<Option<TemplateFlush>>,
}

impl LEDHelper {
    pub fn new(config: &ConfigWrapper, update_func: UpdateFunc, led_count: i64) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Initial color
        let lim = || Lim::new().min(0.0).max(1.0);
        let red = config.getfloat("initial_RED", 0.0, lim())?;
        let green = config.getfloat("initial_GREEN", 0.0, lim())?;
        let blue = config.getfloat("initial_BLUE", 0.0, lim())?;
        let white = config.getfloat("initial_WHITE", 0.0, lim())?;
        let led_state = vec![vec![red, green, blue, white]; led_count as usize];
        // Support setting an led template
        let template_eval = lookup_template_eval(config)?;
        let h = Rc::new_cyclic(|me| LEDHelper {
            me: me.clone(),
            printer: printer.clone(),
            mutex: printer.get_reactor().mutex(false),
            update_func,
            led_count,
            need_transmit: Cell::new(false),
            led_state: RefCell::new(led_state),
            template_eval,
            tcallbacks: RefCell::new(vec![]),
            check_transmit_cb: RefCell::new(None),
        });
        let mut tcallbacks: Vec<TemplateCallback> = vec![];
        for i in 0..led_count {
            let w = h.me.clone();
            tcallbacks.push(Rc::new(move |text: &str| match w.upgrade() {
                Some(h) => h.template_update(i + 1, text),
                None => Ok(()),
            }));
        }
        *h.tcallbacks.borrow_mut() = tcallbacks;
        let w = h.me.clone();
        *h.check_transmit_cb.borrow_mut() = Some(Rc::new(move || match w.upgrade() {
            Some(h) => h.check_transmit(None),
            None => Ok(()),
        }));
        // Register commands
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let hh = h.clone();
        gcode.register_mux_command("SET_LED", "LED", Some(&name), Rc::new(move |g| hh.cmd_set_led(g)), Some(CMD_SET_LED_HELP))?;
        let hh = h.clone();
        gcode.register_mux_command(
            "SET_LED_TEMPLATE",
            "LED",
            Some(&name),
            Rc::new(move |g| hh.cmd_set_led_template(g)),
            Some(CMD_SET_LED_TEMPLATE_HELP),
        )?;
        Ok(h)
    }

    pub fn get_status(&self) -> PyVal {
        let cd: Vec<PyVal> = self.led_state.borrow().iter().map(|c| PyVal::Tuple(c.iter().map(|v| PyVal::Float(*v)).collect())).collect();
        let mut d = IndexMap::new();
        d.insert("color_data".to_string(), PyVal::List(cd));
        PyVal::Dict(d)
    }

    pub fn led_state(&self) -> Vec<Color> {
        self.led_state.borrow().clone()
    }

    fn set_color(&self, index: Option<i64>, color: &Color) {
        let mut state = self.led_state.borrow_mut();
        match index {
            None => {
                let new_led_state = vec![color.clone(); self.led_count as usize];
                if *state == new_led_state {
                    return;
                }
                *state = new_led_state;
            }
            Some(index) => {
                let i = (index - 1) as usize;
                if state[i] == *color {
                    return;
                }
                state[i] = color.clone();
            }
        }
        self.need_transmit.set(true);
    }

    fn template_update(&self, index: i64, text: &str) -> KResult<()> {
        // text.split(",", 4): at most 5 parts
        let pieces: Vec<&str> = text.splitn(5, ',').collect();
        let parsed: Option<Vec<f64>> = pieces.iter().map(|f| crate::pyfmt::py_float(f).map(|v| py_max(0.0, py_min(1.0, v)))).collect();
        let mut parts = match parsed {
            Some(p) => p,
            None => {
                let bad = pieces.iter().find(|f| crate::pyfmt::py_float(f).is_none()).copied().unwrap_or("");
                logging::exception_tb(
                    "led template render error",
                    "ValueError",
                    &format!("could not convert string to float: {}", crate::pyfmt::py_str_repr(bad)),
                );
                vec![]
            }
        };
        if parts.len() < 4 {
            parts.resize(4, 0.0);
        }
        self.set_color(Some(index), &parts);
        Ok(())
    }

    fn check_transmit(&self, print_time: Option<f64>) -> KResult<()> {
        if !self.need_transmit.get() {
            return Ok(());
        }
        // Just avoid any race conditions
        let led_state = self.led_state.borrow().clone();
        self.need_transmit.set(false);
        let me = self.me.upgrade().expect("led helper");
        self.printer.get_reactor().register_callback(
            move |_| {
                let r = me.mutex.lock().map_err(KError::from).and_then(|_| {
                    let r = (me.update_func)(&led_state, print_time);
                    me.mutex.unlock();
                    r
                });
                if let Err(e) = r {
                    if e.is_command_error() {
                        logging::exception_err("led update transmit error", &e);
                    } else {
                        logging::exception_err("Unhandled exception in led update", &e);
                    }
                }
            },
            NOW,
        );
        Ok(())
    }

    fn cmd_set_led(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Parse parameters
        let lim = || Lim::new().min(0.0).max(1.0);
        let red = gcmd.get_float("RED", Some(0.0), lim())?;
        let green = gcmd.get_float("GREEN", Some(0.0), lim())?;
        let blue = gcmd.get_float("BLUE", Some(0.0), lim())?;
        let white = gcmd.get_float("WHITE", Some(0.0), lim())?;
        let index = gcmd.get_int_opt("INDEX", Lim::new().min(1).max(self.led_count))?;
        let transmit = gcmd.get_int("TRANSMIT", Some(1), Lim::new())?;
        let sync = gcmd.get_int("SYNC", Some(1), Lim::new())?;
        let color = vec![red, green, blue, white];

        // Update and transmit data
        let me = self.me.upgrade().expect("led helper");
        let lookahead_bgfunc = move |print_time: Option<f64>| -> KResult<()> {
            me.set_color(index, &color);
            if transmit != 0 {
                me.check_transmit(print_time)?;
            }
            Ok(())
        };
        if sync != 0 {
            // Sync LED Update with print time and send
            let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
            toolhead.register_lookahead_callback(Rc::new(move |pt| lookahead_bgfunc(Some(pt))))
        } else {
            // Send update now (so as not to wake toolhead and reset idle_timeout)
            lookahead_bgfunc(None)
        }
    }

    fn cmd_set_led_template(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let index = gcmd.get_int_opt("INDEX", Lim::new().min(1).max(self.led_count))?;
        let flush = self.check_transmit_cb.borrow().clone();
        let tcallbacks = self.tcallbacks.borrow().clone();
        match index {
            Some(i) => self.template_eval.set_template(gcmd, tcallbacks[(i - 1) as usize].clone(), flush),
            None => {
                for cb in tcallbacks {
                    self.template_eval.set_template(gcmd, cb, flush.clone())?;
                }
                Ok(())
            }
        }
    }

    /// not in python: break the callback cycles
    pub fn teardown(&self) {
        self.tcallbacks.borrow_mut().clear();
        *self.check_transmit_cb.borrow_mut() = None;
    }
}

pub const PIN_MIN_TIME: f64 = 0.100;
pub const MAX_SCHEDULE_TIME: f64 = 5.0;

// Handler for PWM controlled LEDs
pub struct PrinterPWMLED {
    printer: Rc<Printer>,
    pins: Vec<(usize, Rc<dyn PwmPin>)>,
    last_print_time: Cell<f64>,
    led_helper: RefCell<Option<Rc<LEDHelper>>>,
    prev_color: RefCell<Color>,
}

impl PrinterPWMLED {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Configure pwm pins
        let cycle_time = config.getfloat("cycle_time", 0.010, Lim::new().above(0.0).max(MAX_SCHEDULE_TIME))?;
        let hardware_pwm = config.getboolean("hardware_pwm", false)?;
        let mut pins = vec![];
        for (i, name) in ["red", "green", "blue", "white"].iter().enumerate() {
            let Some(pin_name) = config.get_opt(&format!("{}_pin", name))? else { continue };
            let mcu_pin = crate::pins::setup_pwm_pin(&printer, &pin_name)?;
            mcu_pin.setup_max_duration(0.0);
            mcu_pin.setup_cycle_time(cycle_time, hardware_pwm);
            pins.push((i, mcu_pin));
        }
        if pins.is_empty() {
            return Err(KError::config(format!("No LED pin definitions found in '{}'", config.get_name())));
        }
        let led = Rc::new(PrinterPWMLED {
            printer,
            pins,
            last_print_time: Cell::new(0.0),
            led_helper: RefCell::new(None),
            prev_color: RefCell::new(vec![]),
        });
        // Initialize color data
        let w = Rc::downgrade(&led);
        let helper = LEDHelper::new(
            config,
            Rc::new(move |state, print_time| match w.upgrade() {
                Some(l) => l.update_leds(state, print_time),
                None => Ok(()),
            }),
            1,
        )?;
        let color = helper.led_state()[0].clone();
        *led.prev_color.borrow_mut() = color.clone();
        for (idx, mcu_pin) in &led.pins {
            mcu_pin.setup_start_value(color[*idx], 0.0);
        }
        *led.led_helper.borrow_mut() = Some(helper);
        Ok(led)
    }

    fn update_leds(&self, led_state: &[Color], print_time: Option<f64>) -> KResult<()> {
        let print_time = match print_time {
            Some(pt) => pt,
            None => {
                let eventtime = self.printer.get_reactor().monotonic();
                let mcu = self.pins[0].1.get_mcu();
                mcu.estimated_print_time(eventtime) + PIN_MIN_TIME
            }
        };
        let print_time = py_max(print_time, self.last_print_time.get() + PIN_MIN_TIME);
        let color = led_state[0].clone();
        let prev = self.prev_color.borrow().clone();
        for (idx, mcu_pin) in &self.pins {
            if prev[*idx] != color[*idx] {
                mcu_pin.set_pwm(print_time, color[*idx])?;
                self.last_print_time.set(print_time);
            }
        }
        *self.prev_color.borrow_mut() = color;
        Ok(())
    }
}

impl PrinterObject for PrinterPWMLED {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        self.led_helper.borrow().as_ref().map(|h| h.get_status())
    }
    fn teardown(&self) {
        if let Some(h) = self.led_helper.borrow_mut().take() {
            h.teardown();
        }
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterPWMLED::new(config)?)
}
