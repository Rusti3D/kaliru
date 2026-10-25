// Support for filament width sensor
// (port of klippy/extras/hall_filament_width_sensor.py)
//
// Copyright (C) 2019  Mustafa YILDIZ <mydiz@hotmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::filament_switch_sensor::{DefinedSensor, RunoutHelper};
use crate::extras::gcode_macro::load_gcode_macro;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::{AdcPin, Mcu};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float_repr, py_round};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, Timer, NEVER, NOW};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const ADC_REPORT_TIME: f64 = 0.500;
const ADC_SAMPLE_TIME: f64 = 0.03;
const ADC_SAMPLE_COUNT: i64 = 15;

pub struct HallFilamentWidthSensor {
    me: Weak<HallFilamentWidthSensor>,
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    mcu: RefCell<Option<Rc<Mcu>>>,
    dia1: f64,
    dia2: f64,
    rawdia1: i64,
    rawdia2: i64,
    measurement_interval_mm: i64,
    nominal_filament_dia: f64,
    measurement_delay: f64,
    max_diameter: f64,
    min_diameter: f64,
    diameter: Cell<f64>,
    is_active: Cell<bool>,
    runout_dia_min: f64,
    runout_dia_max: f64,
    is_log: Cell<bool>,
    use_current_dia_while_delay: bool,
    check_on_print_start: Cell<bool>,
    // filament array [position, filamentWidth]
    filament_array: RefCell<Vec<(f64, f64)>>,
    last_reading: Cell<i64>,
    last_reading2: Cell<i64>,
    first_extruder_update_position: Cell<f64>,
    filament_width: Cell<f64>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    _mcu_adc: Rc<dyn AdcPin>,
    _mcu_adc2: Rc<dyn AdcPin>,
    extrude_factor_update_timer: RefCell<Option<Timer>>,
    gcode: Rc<GCodeDispatch>,
    runout_helper: RefCell<Option<Rc<RunoutHelper>>>,
}

impl HallFilamentWidthSensor {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode_macro = load_gcode_macro(config)?;
        let reactor = printer.get_reactor();
        let pin1 = config.get("adc1", Dflt::Required)?;
        let pin2 = config.get("adc2", Dflt::Required)?;
        let dia1 = config.getfloat("Cal_dia1", 1.5, Lim::new())?;
        let dia2 = config.getfloat("Cal_dia2", 2.0, Lim::new())?;
        let rawdia1 = config.getint("Raw_dia1", 9500, Lim::new())?;
        let rawdia2 = config.getint("Raw_dia2", 10500, Lim::new())?;
        let measurement_interval_mm = config.getint("measurement_interval", 10, Lim::new())?;
        let nominal_filament_dia = config.getfloat("default_nominal_filament_diameter", Dflt::Required, Lim::new().above(1))?;
        let measurement_delay = config.getfloat("measurement_delay", Dflt::Required, Lim::new().above(0.0))?;
        let measurement_max_difference = config.getfloat("max_difference", 0.2, Lim::new())?;
        let max_diameter = nominal_filament_dia + measurement_max_difference;
        let min_diameter = nominal_filament_dia - measurement_max_difference;
        let is_active = config.getboolean("enable", false)?;
        let runout_dia_min = config.getfloat("min_diameter", 1.0, Lim::new())?;
        let runout_dia_max = config.getfloat("max_diameter", max_diameter, Lim::new())?;
        let is_log = config.getboolean("logging", false)?;
        // Use the current diameter instead of nominal while the first
        // measurement isn't in place
        let use_current_dia_while_delay = config.getboolean("use_current_dia_while_delay", false)?;
        let runout_distance = config.getfloat("runout_distance", 0.0, Lim::new().min(0.0))?;
        let check_on_print_start = config.getboolean("check_on_print_start", false)?;
        // klippy:ready is registered before the adc pins are set up
        let slot: Rc<RefCell<Weak<HallFilamentWidthSensor>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| match sl.borrow().upgrade() {
                Some(s) => s.handle_ready(),
                None => Ok(()),
            }),
        );
        // Start adc
        let mcu_adc = crate::pins::setup_adc_pin(&printer, &pin1)?;
        mcu_adc.setup_adc_sample(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT, 0.0, 1.0, 0);
        let sl = slot.clone();
        mcu_adc.setup_adc_callback(
            ADC_REPORT_TIME,
            Rc::new(move |_read_time, read_value| {
                if let Some(s) = sl.borrow().upgrade() {
                    // read sensor value
                    s.last_reading.set((read_value * 10000.0).round_ties_even() as i64);
                }
                Ok(())
            }),
        );
        let mcu_adc2 = crate::pins::setup_adc_pin(&printer, &pin2)?;
        mcu_adc2.setup_adc_sample(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT, 0.0, 1.0, 0);
        let sl = slot.clone();
        mcu_adc2.setup_adc_callback(
            ADC_REPORT_TIME,
            Rc::new(move |_read_time, read_value| {
                if let Some(s) = sl.borrow().upgrade() {
                    s.adc2_callback(read_value);
                }
                Ok(())
            }),
        );
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let s = Rc::new_cyclic(|me| HallFilamentWidthSensor {
            me: me.clone(),
            printer: printer.clone(),
            reactor: reactor.clone(),
            mcu: RefCell::new(None),
            dia1,
            dia2,
            rawdia1,
            rawdia2,
            measurement_interval_mm,
            nominal_filament_dia,
            measurement_delay,
            max_diameter,
            min_diameter,
            diameter: Cell::new(nominal_filament_dia),
            is_active: Cell::new(is_active),
            runout_dia_min,
            runout_dia_max,
            is_log: Cell::new(is_log),
            use_current_dia_while_delay,
            check_on_print_start: Cell::new(check_on_print_start),
            filament_array: RefCell::new(vec![]),
            last_reading: Cell::new(0),
            last_reading2: Cell::new(0),
            first_extruder_update_position: Cell::new(0.0),
            filament_width: Cell::new(nominal_filament_dia),
            toolhead: RefCell::new(None),
            _mcu_adc: mcu_adc,
            _mcu_adc2: mcu_adc2,
            extrude_factor_update_timer: RefCell::new(None),
            gcode: gcode.clone(),
            runout_helper: RefCell::new(None),
        });
        *slot.borrow_mut() = Rc::downgrade(&s);
        // extrude factor updating
        let w = s.me.clone();
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.extrude_factor_update_event(eventtime),
                None => NEVER,
            }),
            NEVER,
        );
        *s.extrude_factor_update_timer.borrow_mut() = Some(timer);
        // Register commands
        type HCmd = fn(&HallFilamentWidthSensor, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, HCmd); 7] = [
            ("QUERY_FILAMENT_WIDTH", HallFilamentWidthSensor::cmd_m407),
            ("RESET_FILAMENT_WIDTH_SENSOR", HallFilamentWidthSensor::cmd_clear_filament_array),
            ("DISABLE_FILAMENT_WIDTH_SENSOR", HallFilamentWidthSensor::cmd_m406),
            ("ENABLE_FILAMENT_WIDTH_SENSOR", HallFilamentWidthSensor::cmd_m405),
            ("QUERY_RAW_FILAMENT_WIDTH", HallFilamentWidthSensor::cmd_get_raw_values),
            ("ENABLE_FILAMENT_WIDTH_LOG", HallFilamentWidthSensor::cmd_log_enable),
            ("DISABLE_FILAMENT_WIDTH_LOG", HallFilamentWidthSensor::cmd_log_disable),
        ];
        for (name, f) in cmds {
            let w = s.me.clone();
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(s) => f(&s, g),
                    None => Ok(()),
                })),
                false,
                None,
            )?;
        }
        let runout_helper = RunoutHelper::new(config, runout_distance)?;
        let ds: Rc<dyn DefinedSensor> = s.clone();
        runout_helper.set_defined_sensor(Rc::downgrade(&ds));
        if config.get_opt("immediate_runout_gcode")?.is_some() {
            *runout_helper.immediate_runout_gcode.borrow_mut() = Some(gcode_macro.load_template(config, "immediate_runout_gcode", Some(""))?);
        }
        *s.runout_helper.borrow_mut() = Some(runout_helper);
        for (ev, smart) in [("print_stats:start_printing", true), ("idle_timeout:printing", false)] {
            let w = s.me.clone();
            printer.register_event_handler(
                ev,
                Rc::new(move |_| match w.upgrade() {
                    Some(s) => s.handle_printing(smart),
                    None => Ok(()),
                }),
            );
        }
        Ok(s)
    }

    fn runout_helper(&self) -> Rc<RunoutHelper> {
        self.runout_helper.borrow().clone().expect("runout helper")
    }

    // Initialization
    fn handle_ready(&self) -> KResult<()> {
        // Load printer objects
        *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);
        *self.mcu.borrow_mut() = Some(self.printer.lookup::<Mcu>("mcu")?);
        // Start extrude factor update timer
        if let Some(t) = self.extrude_factor_update_timer.borrow().as_ref() {
            self.reactor.update_timer(t, NOW);
        }
        Ok(())
    }

    /// _handle_printing (smart false) / _handle_printing_smart (smart true)
    fn handle_printing(&self, smart: bool) -> KResult<()> {
        let rh = self.runout_helper();
        if rh.smart.get() == smart && self.check_on_print_start.get() {
            self.reset()?;
            let d = self.diameter.get();
            rh.note_filament_present(self.reactor.monotonic(), Some(self.runout_dia_min <= d && d <= self.runout_dia_max), true, true)?;
        }
        Ok(())
    }

    fn adc2_callback(&self, read_value: f64) {
        // read sensor value
        self.last_reading2.set((read_value * 10000.0).round_ties_even() as i64);
        // calculate diameter
        let raw = (self.last_reading.get() + self.last_reading2.get() - self.rawdia1) as f64;
        let diameter_new =
            py_round((self.dia2 - self.dia1) / (self.rawdia2 - self.rawdia1) as f64 * raw + self.dia1, 2);
        self.diameter.set((5.0 * self.diameter.get() + diameter_new) / 6.0);
    }

    fn update_filament_array(&self, last_epos: f64) {
        let mut fa = self.filament_array.borrow_mut();
        // Fill array
        if let Some(last) = fa.last() {
            // Get last reading position in array & calculate next
            // reading position
            let next_reading_position = last.0 + self.measurement_interval_mm as f64;
            if next_reading_position <= last_epos + self.measurement_delay {
                fa.push((last_epos + self.measurement_delay, self.diameter.get()));
                if self.is_log.get() {
                    self.gcode.respond_info(&format!("Filament width:{:.3}", self.diameter.get()), true);
                }
            }
        } else {
            // add first item to array
            fa.push((self.measurement_delay + last_epos, self.diameter.get()));
            self.first_extruder_update_position.set(self.measurement_delay + last_epos);
        }
    }

    fn extrude_factor_update_event(&self, eventtime: f64) -> f64 {
        if let Err(e) = self.extrude_factor_update(eventtime) {
            logging::exception_err("filament width update", &e);
        }
        if self.is_active.get() {
            eventtime + 1.0
        } else {
            NEVER
        }
    }

    fn extrude_factor_update(&self, eventtime: f64) -> KResult<()> {
        // Update extrude factor
        let toolhead = self.toolhead.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_position'".into()))?;
        let pos = toolhead.get_position();
        let last_epos = pos[3];
        // Update filament array for lastFilamentWidthReading
        self.update_filament_array(last_epos);
        // Check runout
        let d = self.diameter.get();
        self.runout_helper().note_filament_present(eventtime, Some(self.runout_dia_min <= d && d <= self.runout_dia_max), false, false)?;
        // Does filament exists
        if d > 0.5 {
            let first = self.filament_array.borrow().first().copied();
            if let Some((pending_position, width)) = first {
                // Get first position in filament array
                if pending_position <= last_epos {
                    // Get first item in filament_array queue
                    self.filament_array.borrow_mut().remove(0);
                    self.filament_width.set(width);
                } else if self.use_current_dia_while_delay && self.first_extruder_update_position.get() == pending_position {
                    self.filament_width.set(d);
                } else if self.first_extruder_update_position.get() == pending_position {
                    self.filament_width.set(self.nominal_filament_dia);
                }
                let fw = self.filament_width.get();
                if fw <= self.max_diameter && fw >= self.min_diameter {
                    let percentage = (self.nominal_filament_dia.powf(2.0) / fw.powf(2.0) * 100.0).round_ties_even() as i64;
                    self.gcode.run_script(&format!("M221 S{}", percentage))?;
                } else {
                    self.gcode.run_script("M221 S100")?;
                }
            }
        } else {
            self.gcode.run_script("M221 S100")?;
            self.filament_array.borrow_mut().clear();
        }
        Ok(())
    }

    fn cmd_m407(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let d = self.diameter.get();
        let response = if d > 0.0 { format!("Filament dia (measured mm): {}", py_float_repr(d)) } else { "Filament NOT present".to_string() };
        gcmd.respond_info(&response, true);
        Ok(())
    }

    fn cmd_clear_filament_array(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.filament_array.borrow_mut().clear();
        gcmd.respond_info("Filament width measurements cleared!", true);
        // Set extrude multiplier to 100%
        self.gcode.run_script_from_command("M221 S100")
    }

    fn cmd_m405(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut response = "Filament width sensor Turned On";
        if self.is_active.get() {
            response = "Filament width sensor is already On";
        } else {
            self.is_active.set(true);
            // Start extrude factor update timer
            if let Some(t) = self.extrude_factor_update_timer.borrow().as_ref() {
                self.reactor.update_timer(t, NOW);
            }
        }
        gcmd.respond_info(response, true);
        Ok(())
    }

    fn cmd_m406(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut response = "Filament width sensor Turned Off";
        if !self.is_active.get() {
            response = "Filament width sensor is already Off";
        } else {
            self.is_active.set(false);
            // Stop extrude factor update timer
            if let Some(t) = self.extrude_factor_update_timer.borrow().as_ref() {
                self.reactor.update_timer(t, NEVER);
            }
            // Clear filament array
            self.filament_array.borrow_mut().clear();
            // Set extrude multiplier to 100%
            self.gcode.run_script_from_command("M221 S100")?;
        }
        gcmd.respond_info(response, true);
        Ok(())
    }

    fn cmd_get_raw_values(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let (a, b) = (self.last_reading.get(), self.last_reading2.get());
        gcmd.respond_info(&format!("ADC1= {} ADC2={} RAW={}", a, b, a + b), true);
        Ok(())
    }

    fn cmd_log_enable(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.is_log.set(true);
        gcmd.respond_info("Filament width logging Turned On", true);
        Ok(())
    }

    fn cmd_log_disable(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.is_log.set(false);
        gcmd.respond_info("Filament width logging Turned Off", true);
        Ok(())
    }

    fn get_sensor_status(&self) -> String {
        let rh = self.runout_helper();
        let tf = |b: bool| if b { "true" } else { "false" };
        format!(
            "Filament Width Sensor {}: {}\nFilament Detected: {}\nSmart: {}\nAlways Fire Events: {}",
            rh.name,
            if rh.sensor_enabled.get() { "enabled" } else { "disabled" },
            tf(rh.filament_present.get()),
            tf(rh.smart.get()),
            tf(rh.always_fire_events.get())
        )
    }
}

impl DefinedSensor for HallFilamentWidthSensor {
    fn get_extruder_pos(&self, eventtime: Option<f64>) -> KResult<f64> {
        let eventtime = eventtime.unwrap_or_else(|| self.reactor.monotonic());
        let mcu = self.mcu.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object is not callable".into()))?;
        let print_time = mcu.estimated_print_time(eventtime);
        let extruder = self.printer.lookup::<ToolHead>("toolhead")?.get_extruder();
        Ok(extruder.find_past_position(print_time))
    }

    fn sensor_get_status(&self, _eventtime: f64) -> IndexMap<String, PyVal> {
        let mut d = IndexMap::new();
        d.insert("Diameter".to_string(), PyVal::Float(self.diameter.get()));
        d.insert("Raw".to_string(), PyVal::Int(self.last_reading.get() + self.last_reading2.get()));
        d.insert("is_active".to_string(), PyVal::Bool(self.is_active.get()));
        d.insert("check_on_print_start".to_string(), PyVal::Bool(self.check_on_print_start.get()));
        d
    }

    fn get_info(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        let check_on_print_start = gcmd.get_int_opt("CHECK_ON_PRINT_START", Lim::new().min(0).max(1))?;
        if check_on_print_start.is_none() {
            gcmd.respond_info(&self.get_sensor_status(), true);
            return Ok(true);
        }
        Ok(false)
    }

    fn reset_needed(&self, enable: Option<i64>, always_fire_events: Option<i64>) -> bool {
        let rh = self.runout_helper();
        if let Some(e) = enable {
            if (e != 0) != rh.sensor_enabled.get() || (e != 0 && e != 1) {
                return true;
            }
        }
        always_fire_events.map(|a| a != 0).unwrap_or(false)
    }

    fn set_filament_sensor(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        if let Some(c) = gcmd.get_int_opt("CHECK_ON_PRINT_START", Lim::new().min(0).max(1))? {
            self.check_on_print_start.set(c != 0);
        }
        // No reset is needed when changing check_on_print_start, so we always
        // return False
        Ok(false)
    }

    fn reset(&self) -> KResult<()> {
        let rh = self.runout_helper();
        rh.reset_runout_distance_info();
        rh.note_filament_present(self.reactor.monotonic(), Some(rh.filament_present.get()), true, false)
    }
}

impl PrinterObject for HallFilamentWidthSensor {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        Some(self.runout_helper().get_status(eventtime))
    }
    fn teardown(&self) {
        *self.runout_helper.borrow_mut() = None;
        *self.toolhead.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(HallFilamentWidthSensor::new(config)?)
}
