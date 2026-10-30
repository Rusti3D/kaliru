// Z Thermal Adjust (port of klippy/extras/z_thermal_adjust.py)
//
// Copyright (C) 2022  Robert Pazdzior <robertp@norbital.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Adjusts Z position in real-time using a thermal probe to e.g. compensate
// for thermal expansion of the printer frame.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_move::{GCodeMove, MoveTransform, MoveTransformRef};
use crate::extras::heaters::{PrinterHeaters, TempSensor};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_round};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const KELVIN_TO_CELSIUS: f64 = -273.15;

const CMD_SET_Z_THERMAL_ADJUST_HELP: &str = "Set/query Z Thermal Adjust parameters.";

pub struct ZThermalAdjuster {
    me: Weak<ZThermalAdjuster>,
    pub printer: Rc<Printer>,
    pub temp_coeff: Cell<f64>,
    pub off_above_z: f64,
    pub max_z_adjust_mm: f64,
    pub smooth_time: f64,
    pub inv_smooth_time: f64,
    pub min_temp: f64,
    pub max_temp: f64,
    pub sensor: Rc<dyn TempSensor>,
    pub last_temp: Cell<f64>,
    pub measured_min: Cell<f64>,
    pub measured_max: Cell<f64>,
    pub smoothed_temp: Cell<f64>,
    pub last_temp_time: Cell<f64>,
    pub ref_temperature: Cell<f64>,
    pub ref_temp_override: Cell<bool>,
    // Z transformation
    pub z_adjust_mm: Cell<f64>,
    pub adjust_enable: Cell<bool>,
    pub last_position: RefCell<Vec<f64>>,
    pub next_transform: RefCell<Option<MoveTransformRef>>,
    pub z_step_dist: Cell<f64>,
    pub component: Option<String>,
}

impl ZThermalAdjuster {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;

        // Get config parameters, convert to SI units where necessary
        let temp_coeff = config.getfloat("temp_coeff", Dflt::from(PyVal::Int(0)), Lim::new().min(-1.0).max(1.0))?;
        let off_above_z = config.getfloat("z_adjust_off_above", 99999999.0, Lim::new())?;
        let max_z_adjust_mm = config.getfloat("max_z_adjustment", 99999999.0, Lim::new())?;

        // Register printer events
        let slot: Rc<RefCell<Weak<ZThermalAdjuster>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        crate::events::KlippyConnect::on(&printer, move || match sl.borrow().upgrade() {
            Some(z) => z.handle_connect(),
            None => Ok(()),
        });
        let sl = slot.clone();
        crate::events::HomingHomeRailsEnd::on(&printer, move |homing_state, _rails| {
            if let Some(z) = sl.borrow().upgrade() {
                // Set reference temperature after Z homing.
                if homing_state.get_axes().contains(&2) {
                    z.ref_temperature.set(z.smoothed_temp.get());
                    z.ref_temp_override.set(false);
                    z.z_adjust_mm.set(0.0);
                }
            }
            Ok(())
        });

        // Setup temperature sensor
        let smooth_time = config.getfloat("smooth_time", 2.0, Lim::new().above(0.0))?;
        let min_temp = config.getfloat("min_temp", Dflt::Required, Lim::new().min(KELVIN_TO_CELSIUS))?;
        let max_temp = config.getfloat("max_temp", Dflt::Required, Lim::new().above(min_temp))?;
        let pheaters = printer.load_as::<PrinterHeaters>(config, "heaters")?;
        let sensor = pheaters.setup_sensor(config)?;
        sensor.setup_minmax(min_temp, max_temp)?;
        let sl = slot.clone();
        sensor.setup_callback(Rc::new(move |read_time, temp| match sl.borrow().upgrade() {
            Some(z) => z.init_temperature_callback(read_time, temp),
            None => Ok(()),
        }))?;

        // Register gcode commands
        let full_name = config.get_name().to_string();
        let component = if full_name != "z_thermal_adjust" {
            Some(full_name.split_once(char::is_whitespace).map(|(_, c)| c.trim_start().to_string()).unwrap_or(full_name.clone()))
        } else {
            None
        };
        let z = Rc::new_cyclic(|me| ZThermalAdjuster {
            me: me.clone(),
            printer: printer.clone(),
            temp_coeff: Cell::new(temp_coeff),
            off_above_z,
            max_z_adjust_mm,
            smooth_time,
            inv_smooth_time: 1.0 / smooth_time,
            min_temp,
            max_temp,
            sensor,
            last_temp: Cell::new(0.0),
            measured_min: Cell::new(0.0),
            measured_max: Cell::new(0.0),
            smoothed_temp: Cell::new(0.0),
            last_temp_time: Cell::new(0.0),
            ref_temperature: Cell::new(0.0),
            ref_temp_override: Cell::new(false),
            z_adjust_mm: Cell::new(0.0),
            adjust_enable: Cell::new(true),
            last_position: RefCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            next_transform: RefCell::new(None),
            z_step_dist: Cell::new(0.0),
            component: component.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&z);
        pheaters.register_sensor(config, z.clone(), None)?;
        let w = z.me.clone();
        gcode.register_mux_command(
            "SET_Z_THERMAL_ADJUST",
            "COMPONENT",
            component.as_deref(),
            Rc::new(move |g| match w.upgrade() {
                Some(z) => z.cmd_set_z_thermal_adjust(g),
                None => Ok(()),
            }),
            Some(CMD_SET_Z_THERMAL_ADJUST_HELP),
        )?;
        Ok(z)
    }

    // Called after all printer objects are instantiated
    pub fn handle_connect(&self) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let gcode_move = self.printer.lookup::<GCodeMove>("gcode_move")?;

        // Register move transformation
        let me: MoveTransformRef = self.me.upgrade().expect("z_thermal_adjust");
        *self.next_transform.borrow_mut() = gcode_move.set_move_transform(me, true)?;

        // Pull Z step distance for minimum adjustment increment
        let kin = toolhead.get_kinematics();
        let steppers = kin.get_steppers();
        let z_stepper = steppers
            .iter()
            .find(|s| s.get_name(false) == "stepper_z")
            .ok_or_else(|| KError::Other("'stepper_z' is not in list".into()))?;
        self.z_step_dist.set(z_stepper.get_step_dist());
        Ok(())
    }

    // Z adjustment calculation
    pub fn calc_adjust(&self, pos: &[f64]) -> Vec<f64> {
        if pos[2] < self.off_above_z {
            let delta_t = self.smoothed_temp.get() - self.ref_temperature.get();

            // Calculate Z adjustment
            let adjust = -self.temp_coeff.get() * delta_t;

            // compute sign (+1 or -1) for maximum offset setting
            let sign = 1.0 - ((adjust <= 0.0) as i64 * 2) as f64;

            // Don't apply adjustments smaller than step distance
            if (adjust - self.z_adjust_mm.get()).abs() > self.z_step_dist.get() {
                // min([max * sign, adjust], key=abs)
                let a = self.max_z_adjust_mm * sign;
                self.z_adjust_mm.set(if adjust.abs() < a.abs() { adjust } else { a });
            }
        }

        // Apply Z adjustment
        let new_z = pos[2] + self.z_adjust_mm.get();
        let mut out = vec![pos[0], pos[1], new_z];
        out.extend_from_slice(&pos[3..]);
        out
    }

    // Remove Z adjustment
    pub fn calc_unadjust(&self, pos: &[f64]) -> Vec<f64> {
        let unadjusted_z = pos[2] - self.z_adjust_mm.get();
        let mut out = vec![pos[0], pos[1], unadjusted_z];
        out.extend_from_slice(&pos[3..]);
        out
    }

    fn next(&self) -> KResult<MoveTransformRef> {
        self.next_transform.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'move'".into()))
    }

    // Initialize Z adjust thermistor ref temp
    pub fn init_temperature_callback(&self, _read_time: f64, temp: f64) -> KResult<()> {
        self.ref_temperature.set(temp);
        let w = self.me.clone();
        self.sensor.setup_callback(Rc::new(move |read_time, temp| {
            if let Some(z) = w.upgrade() {
                z.temperature_callback(read_time, temp);
            }
            Ok(())
        }))?;
        Ok(())
    }

    // Called everytime the Z adjust thermistor is read
    pub fn temperature_callback(&self, read_time: f64, temp: f64) {
        let time_diff = read_time - self.last_temp_time.get();
        self.last_temp.set(temp);
        self.last_temp_time.set(read_time);
        let temp_diff = temp - self.smoothed_temp.get();
        let adj_time = py_min(time_diff * self.inv_smooth_time, 1.0);
        self.smoothed_temp.set(self.smoothed_temp.get() + temp_diff * adj_time);
        self.measured_min.set(py_min(self.measured_min.get(), self.smoothed_temp.get()));
        self.measured_max.set(py_max(self.measured_max.get(), self.smoothed_temp.get()));
    }

    pub fn cmd_set_z_thermal_adjust(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let enable = gcmd.get_int_opt("ENABLE", Lim::new().min(0.0).max(1.0))?;
        let coeff = gcmd.get_float_opt("TEMP_COEFF", Lim::new().min(-1.0).max(1.0))?;
        let ref_temp = gcmd.get_float_opt("REF_TEMP", Lim::new().min(KELVIN_TO_CELSIUS))?;

        if let Some(r) = ref_temp {
            self.ref_temperature.set(r);
            self.ref_temp_override.set(true);
        }
        if let Some(c) = coeff {
            self.temp_coeff.set(c);
        }
        if let Some(e) = enable {
            if (e != 0) != self.adjust_enable.get() {
                self.adjust_enable.set(e != 0);
                self.printer.lookup::<GCodeMove>("gcode_move")?.reset_last_position()?;
            }
        }

        let state = if self.adjust_enable.get() { "1 (enabled)" } else { "0 (disabled)" };
        let overridden = if self.ref_temp_override.get() { " (manual)" } else { "" };
        let component = match &self.component {
            Some(c) => format!("component: {}\n", c),
            None => String::new(),
        };
        let msg = format!(
            "{}enable: {}\ntemp_coeff: {:.6} mm/degC\nref_temp: {:.2} degC{}\n-------------------\nCurrent Z temp: {:.2} degC\nApplied Z adjustment: {:.4} mm",
            component,
            state,
            self.temp_coeff.get(),
            self.ref_temperature.get(),
            overridden,
            self.smoothed_temp.get(),
            self.z_adjust_mm.get()
        );
        gcmd.respond_info(&msg, true);
        Ok(())
    }
}

impl MoveTransform for ZThermalAdjuster {
    fn get_position(&self) -> Vec<f64> {
        let Ok(next) = self.next() else { return vec![] };
        let position = self.calc_unadjust(&next.get_position());
        *self.last_position.borrow_mut() = self.calc_adjust(&position);
        position
    }

    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        // don't apply to extrude only moves or when disabled
        let same = newpos[..3] == self.last_position.borrow()[..3];
        let adjusted_pos = if same || !self.adjust_enable.get() {
            let z = newpos[2] + self.z_adjust_mm.get();
            vec![newpos[0], newpos[1], z, newpos[3]]
        } else {
            self.calc_adjust(newpos)
        };
        self.next()?.move_to(&adjusted_pos, speed)?;
        *self.last_position.borrow_mut() = newpos.to_vec();
        Ok(())
    }
}

impl PrinterObject for ZThermalAdjuster {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let d = crate::pydict! {
            "temperature" => PyVal::Float(self.smoothed_temp.get()),
            "measured_min_temp" => PyVal::Float(py_round(self.measured_min.get(), 2)),
            "measured_max_temp" => PyVal::Float(py_round(self.measured_max.get(), 2)),
            "current_z_adjust" => PyVal::Float(self.z_adjust_mm.get()),
            "z_adjust_ref_temperature" => PyVal::Float(self.ref_temperature.get()),
            "enabled" => PyVal::Bool(self.adjust_enable.get()),
        };
        Some(PyVal::Dict(d))
    }
    fn get_temp(&self, _eventtime: f64) -> Option<(f64, f64)> {
        Some((self.smoothed_temp.get(), 0.0))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        Some((false, format!("z_thermal_adjust: temp={:.1}", self.smoothed_temp.get())))
    }
    fn teardown(&self) {
        *self.next_transform.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ZThermalAdjuster::new(config)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ZThermalAdjuster::new(config)?)
}
