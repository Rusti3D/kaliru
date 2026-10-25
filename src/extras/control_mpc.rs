// Model predictive temperature control (port of klippy/extras/control_mpc.py)
use crate::configfile::Lim;
use crate::error::{KError, KResult};
use crate::extras::heaters::{prof_get, prof_obj, Control, Heater, ProfVal, Profile};
use crate::extras::fan::Fan;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{ConfigFileObject, PObj, Printer};
use crate::pyformat::percent_format;
use crate::pyval::{PyDict, PyVal};
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::rc::Rc;

pub const AMBIENT_TEMP: f64 = 25.0;
pub const PIN_MIN_TIME: f64 = 0.100;

pub const FILAMENT_TEMP_SRC_AMBIENT: &str = "ambient";
pub const FILAMENT_TEMP_SRC_FIXED: &str = "fixed";
pub const FILAMENT_TEMP_SRC_SENSOR: &str = "sensor";

use crate::pyfmt::{py_max, py_min};

fn fmt(f: &str, v: f64) -> String {
    percent_format(f, &PyVal::Float(v)).unwrap_or_default()
}

/// obj.get_temp(eventtime)[0] of an ambient sensor object
fn obj_temp(obj: &PObj, eventtime: f64) -> KResult<f64> {
    obj.get_temp(eventtime)
        .map(|t| t.0)
        .ok_or_else(|| KError::Other("object has no attribute 'get_temp'".into()))
}

const CMD_MPC_SET_HELP: &str = "Set MPC parameter";
const CMD_MPC_CALIBRATE_HELP: &str = "Run MPC calibration";

pub struct ControlMPC {
    pub profile: Profile,
    heater: Rc<Heater>,
    pub heater_max_power: f64,
    const_block_heat_capacity: Cell<Option<f64>>,
    const_ambient_transfer: Cell<Option<f64>>,
    const_target_reach_time: f64,
    const_heater_power: f64,
    const_smoothing: f64,
    const_sensor_responsiveness: Cell<Option<f64>>,
    const_min_ambient_change: f64,
    const_steady_state_rate: f64,
    const_filament_diameter: Cell<f64>,
    const_filament_density: Cell<f64>,
    const_filament_heat_capacity: Cell<f64>,
    const_maximum_retract: f64,
    const_filament_cross_section_heat_capacity: Cell<f64>,
    /// ("ambient",) / ("sensor",) / ("fixed", value)
    filament_temp_src: RefCell<PyVal>,
    pub ambient_sensor: Option<PObj>,
    pub cooling_fan: Option<Rc<Fan>>,
    const_fan_ambient_transfer: RefCell<Vec<f64>>,
    want_ambient_refresh: Cell<bool>,
    pub state_block_temp: Cell<f64>,
    pub state_sensor_temp: Cell<f64>,
    pub state_ambient_temp: Cell<f64>,
    last_power: Cell<f64>,
    last_loss_ambient: Cell<f64>,
    last_loss_filament: Cell<f64>,
    last_temp_time: Cell<f64>,
    printer: Rc<Printer>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
}

impl ControlMPC {
    pub fn new(profile: &Profile, heater: &Rc<Heater>, load_clean: bool, register: bool) -> KResult<Rc<ControlMPC>> {
        // _load_profile(): every key must be present (python KeyError)
        let need = |k: &str| -> KResult<PyVal> {
            if !profile.borrow().contains_key(k) {
                return Err(KError::Other(crate::pyfmt::py_str_repr(k)));
            }
            Ok(prof_get(profile, k))
        };
        let need_f = |k: &str| -> KResult<f64> {
            need(k)?.as_f64().ok_or_else(|| KError::Other("unsupported operand type(s): 'NoneType'".into()))
        };
        let const_block_heat_capacity = need("block_heat_capacity")?.as_f64();
        let const_ambient_transfer = need("ambient_transfer")?.as_f64();
        let const_target_reach_time = need_f("target_reach_time")?;
        let const_heater_power = need_f("heater_power")?;
        let const_smoothing = need_f("smoothing")?;
        let const_sensor_responsiveness = need("sensor_responsiveness")?.as_f64();
        let const_min_ambient_change = need_f("min_ambient_change")?;
        let const_steady_state_rate = need_f("steady_state_rate")?;
        let const_filament_diameter = need_f("filament_diameter")?;
        let const_filament_density = need_f("filament_density")?;
        let const_filament_heat_capacity = need_f("filament_heat_capacity")?;
        let const_maximum_retract = need_f("maximum_retract")?;
        let filament_temp_src = need("filament_temp_src")?;
        let ambient_sensor = prof_obj(profile, "ambient_temp_sensor");
        let cooling_fan = prof_obj(profile, "cooling_fan").and_then(crate::printer::downcast_rc::<Fan>);
        let fan_ambient_transfer = match need("fan_ambient_transfer")? {
            PyVal::List(l) => l.iter().filter_map(|v| v.as_f64()).collect(),
            _ => vec![],
        };
        let state_block_temp = if load_clean { AMBIENT_TEMP } else { heater.get_temp(heater.reactor.monotonic()).0 };
        let c = Rc::new(ControlMPC {
            profile: profile.clone(),
            heater: heater.clone(),
            heater_max_power: heater.get_max_power() * const_heater_power,
            const_block_heat_capacity: Cell::new(const_block_heat_capacity),
            const_ambient_transfer: Cell::new(const_ambient_transfer),
            const_target_reach_time,
            const_heater_power,
            const_smoothing,
            const_sensor_responsiveness: Cell::new(const_sensor_responsiveness),
            const_min_ambient_change,
            const_steady_state_rate,
            const_filament_diameter: Cell::new(const_filament_diameter),
            const_filament_density: Cell::new(const_filament_density),
            const_filament_heat_capacity: Cell::new(const_filament_heat_capacity),
            const_maximum_retract,
            const_filament_cross_section_heat_capacity: Cell::new(0.0),
            filament_temp_src: RefCell::new(filament_temp_src),
            want_ambient_refresh: Cell::new(ambient_sensor.is_some()),
            ambient_sensor,
            cooling_fan,
            const_fan_ambient_transfer: RefCell::new(fan_ambient_transfer),
            state_block_temp: Cell::new(state_block_temp),
            state_sensor_temp: Cell::new(state_block_temp),
            state_ambient_temp: Cell::new(AMBIENT_TEMP),
            last_power: Cell::new(0.0),
            last_loss_ambient: Cell::new(0.0),
            last_loss_filament: Cell::new(0.0),
            last_temp_time: Cell::new(0.0),
            printer: heater.printer.clone(),
            toolhead: RefCell::new(None),
        });
        c.update_filament_const();
        if !register {
            return Ok(c);
        }
        let gcode = c.printer.lookup::<GCodeDispatch>("gcode")?;
        let cc = c.clone();
        gcode.register_mux_command(
            "MPC_CALIBRATE",
            "HEATER",
            Some(heater.get_name()),
            Rc::new(move |g| cc.cmd_mpc_calibrate(g)),
            Some(CMD_MPC_CALIBRATE_HELP),
        )?;
        let cc = c.clone();
        gcode.register_mux_command("MPC_SET", "HEATER", Some(heater.get_name()), Rc::new(move |g| cc.cmd_mpc_set(g)), Some(CMD_MPC_SET_HELP))?;
        Ok(c)
    }

    fn cmd_mpc_set(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let f = |name: &str, cell: &Cell<f64>| -> KResult<()> {
            cell.set(gcmd.get_float(name, Some(cell.get()), Lim::new())?);
            Ok(())
        };
        f("FILAMENT_DIAMETER", &self.const_filament_diameter)?;
        f("FILAMENT_DENSITY", &self.const_filament_density)?;
        f("FILAMENT_HEAT_CAPACITY", &self.const_filament_heat_capacity)?;
        // the defaults may be None: get_float hands a default back as given
        let fo = |name: &str, cell: &Cell<Option<f64>>| -> KResult<()> {
            if let Some(v) = gcmd.get_float_opt(name, Lim::new())? {
                cell.set(Some(v));
            }
            Ok(())
        };
        fo("BLOCK_HEAT_CAPACITY", &self.const_block_heat_capacity)?;
        fo("SENSOR_RESPONSIVENESS", &self.const_sensor_responsiveness)?;
        fo("AMBIENT_TRANSFER", &self.const_ambient_transfer)?;

        if let Some(s) = gcmd.get_opt("FAN_AMBIENT_TRANSFER").filter(|s| !s.is_empty()) {
            let parsed: Option<Vec<f64>> = s.split(',').map(crate::pyfmt::py_float).collect();
            match parsed {
                Some(v) => *self.const_fan_ambient_transfer.borrow_mut() = v,
                None => {
                    return Err(gcmd.error(format!(
                        "Error on '{}': unable to parse FAN_AMBIENT_TRANSFER\nMust be a comma-separated list of values ('0.05,0.07,0.08')",
                        gcmd.get_commandline()
                    )))
                }
            }
        }

        if let Some(temp) = gcmd.get_opt("FILAMENT_TEMP") {
            let temp = crate::pyfmt::py_strip(&temp.to_lowercase()).to_string();
            let src = if temp == "sensor" {
                PyVal::Tuple(vec![PyVal::from(FILAMENT_TEMP_SRC_SENSOR)])
            } else if temp == "ambient" {
                PyVal::Tuple(vec![PyVal::from(FILAMENT_TEMP_SRC_AMBIENT)])
            } else {
                let value = crate::pyfmt::py_float(&temp).ok_or_else(|| {
                    gcmd.error(format!(
                        "Error on '{}': unable to parse FILAMENT_TEMP\nValid options are 'sensor', 'ambient', or number.",
                        gcmd.get_commandline()
                    ))
                })?;
                PyVal::Tuple(vec![PyVal::from(FILAMENT_TEMP_SRC_FIXED), PyVal::Float(value)])
            };
            *self.filament_temp_src.borrow_mut() = src;
        }

        self.update_filament_const();
        Ok(())
    }

    fn cmd_mpc_calibrate(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let cal = MpcCalibrate { printer: self.printer.clone(), heater: self.heater.clone(), orig_control: self.clone() };
        cal.run(gcmd)
    }

    // Helpers

    pub fn is_valid(&self) -> bool {
        self.const_block_heat_capacity.get().is_some()
            && self.const_ambient_transfer.get().is_some()
            && self.const_sensor_responsiveness.get().is_some()
    }

    fn update_filament_const(&self) {
        let radius = self.const_filament_diameter.get() / 2.0;
        self.const_filament_cross_section_heat_capacity.set(
            (radius * radius) // mm^2
                * PI // 1
                / 1000.0 // mm^3 => cm^3
                * self.const_filament_density.get() // g/cm^3
                * self.const_filament_heat_capacity.get(), // J/g/K
        );
    }

    fn filament_temp(&self, read_time: f64, ambient_temp: f64) -> KResult<f64> {
        let src = self.filament_temp_src.borrow().clone();
        let items = src.items().unwrap_or(&[]).to_vec();
        let kind = items.first().map(|v| v.str()).unwrap_or_default();
        if kind == FILAMENT_TEMP_SRC_FIXED {
            return Ok(items.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0));
        } else if kind == FILAMENT_TEMP_SRC_SENSOR {
            if let Some(s) = &self.ambient_sensor {
                return obj_temp(s, read_time);
            }
        }
        Ok(ambient_temp)
    }
}

impl Control for ControlMPC {
    fn temperature_update(&self, read_time: f64, mut temp: f64, target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        if secondary_temp.is_some() {
            return Err(KError::Other("ControlMPC.temperature_update() takes 4 positional arguments but 5 were given".into()));
        }
        if !self.is_valid() {
            return self.heater.set_pwm(read_time, 0.0);
        }
        let block_heat_capacity = self.const_block_heat_capacity.get().unwrap();
        let sensor_responsiveness = self.const_sensor_responsiveness.get().unwrap();

        let mut dt = read_time - self.last_temp_time.get();
        if self.last_temp_time.get() == 0.0 || dt < 0.0 || dt > 1.0 {
            dt = 0.1;
        }

        // Extruder position
        let mut extrude_speed_prev = 0.0;
        let mut extrude_speed_next = 0.0;
        if target_temp != 0.0 {
            if self.toolhead.borrow().is_none() {
                *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);
            }
            let toolhead = self.toolhead.borrow().clone().unwrap();
            let extruder = toolhead.get_extruder();
            if Rc::ptr_eq(&extruder.get_heater()?, &self.heater) {
                let pos = extruder.find_past_position(read_time);

                let pos_prev = extruder.find_past_position(read_time - dt);
                let pos_moved = py_max(-self.const_maximum_retract, pos - pos_prev);
                extrude_speed_prev = pos_moved / dt;

                let pos_next = extruder.find_past_position(read_time + dt);
                let pos_move = py_max(-self.const_maximum_retract, pos_next - pos);
                extrude_speed_next = pos_move / dt;
            }
        }

        // Modulate ambient transfer coefficient with fan speed
        let mut ambient_transfer = self.const_ambient_transfer.get().unwrap();
        let fat = self.const_fan_ambient_transfer.borrow().clone();
        if let Some(fan) = &self.cooling_fan {
            if fat.len() > 1 {
                let speed = match fan.get_status(read_time) {
                    PyVal::Dict(d) => d.get("speed").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    _ => 0.0,
                };
                let fan_speed = py_max(0.0, py_min(1.0, speed));
                let fan_break = fan_speed * (fat.len() - 1) as f64;
                let below = fat[fan_break.floor() as usize];
                let above = fat[fan_break.ceil() as usize];
                if below != above {
                    let frac = fan_break.rem_euclid(1.0);
                    ambient_transfer = below * (1.0 - frac) + frac * above;
                } else {
                    ambient_transfer = below;
                }
            }
        }

        // Simulate

        // Expected power by heating at last power setting
        let expected_heating = self.last_power.get();
        // Expected power from block to ambient
        let block_ambient_delta = self.state_block_temp.get() - self.state_ambient_temp.get();
        let expected_ambient_transfer = block_ambient_delta * ambient_transfer;
        let expected_filament_transfer =
            block_ambient_delta * extrude_speed_prev * self.const_filament_cross_section_heat_capacity.get();

        // Expected block dT since last period
        let expected_block_dt =
            (expected_heating - expected_ambient_transfer - expected_filament_transfer) * dt / block_heat_capacity;
        self.state_block_temp.set(self.state_block_temp.get() + expected_block_dt);

        // Expected sensor dT since last period
        let expected_sensor_dt = (self.state_block_temp.get() - self.state_sensor_temp.get()) * sensor_responsiveness * dt;
        self.state_sensor_temp.set(self.state_sensor_temp.get() + expected_sensor_dt);

        // Correct

        let smoothing = 1.0 - (1.0 - self.const_smoothing).powf(dt);
        let adjustment_dt = (temp - self.state_sensor_temp.get()) * smoothing;
        self.state_block_temp.set(self.state_block_temp.get() + adjustment_dt);
        self.state_sensor_temp.set(self.state_sensor_temp.get() + adjustment_dt);

        if self.want_ambient_refresh.get() {
            temp = obj_temp(self.ambient_sensor.as_ref().unwrap(), read_time)?;
            if temp != 0.0 {
                self.state_ambient_temp.set(temp);
                self.want_ambient_refresh.set(false);
            }
        }
        let last_power = self.last_power.get();
        if (last_power > 0.0 && last_power < 1.0)
            || (expected_block_dt + adjustment_dt).abs() < self.const_steady_state_rate * dt
        {
            let ambient_delta = if adjustment_dt > 0.0 {
                py_max(adjustment_dt, self.const_min_ambient_change * dt)
            } else {
                py_min(adjustment_dt, -self.const_min_ambient_change * dt)
            };
            self.state_ambient_temp.set(self.state_ambient_temp.get() + ambient_delta);
        }

        // Output

        // Amount of power needed to reach the target temperature in the desired time

        let heating_power = (target_temp - self.state_block_temp.get()) * block_heat_capacity / self.const_target_reach_time;
        // Losses (+ = lost from block, - = gained to block)
        let block_ambient_delta = self.state_block_temp.get() - self.state_ambient_temp.get();
        let loss_ambient = block_ambient_delta * ambient_transfer;
        let block_filament_delta = self.state_block_temp.get() - self.filament_temp(read_time, self.state_ambient_temp.get())?;
        let loss_filament = block_filament_delta * extrude_speed_next * self.const_filament_cross_section_heat_capacity.get();

        let power = if target_temp != 0.0 {
            // The required power is the desired heating power + compensation for all the losses
            py_max(0.0, py_min(self.heater_max_power, heating_power + loss_ambient + loss_filament))
        } else {
            0.0
        };

        let duty = power / self.const_heater_power;

        self.last_power.set(power);
        self.last_loss_ambient.set(loss_ambient);
        self.last_loss_filament.set(loss_filament);
        self.last_temp_time.set(read_time);
        self.heater.set_pwm(read_time, duty)
    }

    fn check_busy(&self, _eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        (target_temp - smoothed_temp).abs() > 1.0
    }

    fn get_profile(&self) -> Profile {
        self.profile.clone()
    }

    fn get_type(&self) -> &'static str {
        "mpc"
    }

    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::default();
        let f = |v: f64| PyVal::Float(v);
        d.insert("temp_block".into(), f(self.state_block_temp.get()));
        d.insert("temp_sensor".into(), f(self.state_sensor_temp.get()));
        d.insert("temp_ambient".into(), f(self.state_ambient_temp.get()));
        d.insert("power".into(), f(self.last_power.get()));
        d.insert("loss_ambient".into(), f(self.last_loss_ambient.get()));
        d.insert("loss_filament".into(), f(self.last_loss_filament.get()));
        d.insert("filament_temp".into(), self.filament_temp_src.borrow().clone());
        d.insert("filament_heat_capacity".into(), f(self.const_filament_heat_capacity.get()));
        d.insert("filament_density".into(), f(self.const_filament_density.get()));
        Some(PyVal::Dict(d))
    }

    fn check_valid(&self) -> KResult<()> {
        if self.is_valid() {
            return Ok(());
        }
        let name = self.heater.get_name();
        Err(KError::command(format!(
            "Cannot activate '{name}' as MPC control is not fully configured.\n\nRun 'MPC_CALIBRATE' or ensure 'block_heat_capacity', 'sensor_responsiveness', and 'ambient_transfer' settings are defined for '{name}'."
        )))
    }

}

/// a dict of floats as the calibration passes build them
type Res = IndexMap<&'static str, f64>;

fn res_repr(r: &Res) -> String {
    let d: PyDict = r.iter().map(|(k, v)| (k.to_string().into(), PyVal::Float(*v))).collect();
    PyVal::Dict(d).repr()
}

struct TransferRes {
    target_temp: f64,
    base_power: f64,
    fan_powers: Vec<(f64, f64)>,
}

pub struct MpcCalibrate {
    printer: Rc<Printer>,
    heater: Rc<Heater>,
    orig_control: Rc<ControlMPC>,
}

impl MpcCalibrate {
    fn run(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let use_analytic = gcmd.get_opt("USE_DELTA").is_some();
        let ambient_max_measure_time = gcmd.get_float("AMBIENT_MAX_MEASURE_TIME", Some(20.0), Lim::new().above(0.0))?;
        let ambient_measure_sample_time =
            gcmd.get_float("AMBIENT_MEASURE_SAMPLE_TIME", Some(5.0), Lim::new().below(ambient_max_measure_time))?;
        let fan_breakpoints = gcmd.get_int("FAN_BREAKPOINTS", Some(3), Lim::new().min(2))?;
        let default_target_temp = if self.heater.get_name() == "heater_bed" { 90.0 } else { 200.0 };
        let target_temp = gcmd.get_float("TARGET", Some(default_target_temp), Lim::new().min(60.0))?;
        let threshold_temp = gcmd.get_float("THRESHOLD", Some(py_max(50.0, py_min(100.0, target_temp - 100.0))), Lim::new())?;

        let control = Rc::new(TuningControl::new(&self.heater));
        let old_control = self.heater.set_control(control.clone(), true);
        let result = (|| -> KResult<()> {
            let ambient_temp = self.await_ambient(gcmd, &control, threshold_temp)?;
            let samples = self.heatup_test(gcmd, target_temp, &control)?;
            let first_res =
                self.process_first_pass(&samples, self.orig_control.heater_max_power, ambient_temp, threshold_temp, use_analytic);
            logging::info(&format!("First pass: {}", res_repr(&first_res)));

            let profile: Profile = Rc::new(RefCell::new(self.orig_control.profile.borrow().clone()));
            for key in ["block_heat_capacity", "ambient_transfer", "sensor_responsiveness"] {
                profile.borrow_mut().insert(key.to_string(), ProfVal::V(PyVal::Float(first_res[key])));
            }
            let new_control = ControlMPC::new(&profile, &self.heater, false, false)?;
            new_control.state_block_temp.set(first_res["post_block_temp"]);
            new_control.state_sensor_temp.set(first_res["post_sensor_temp"]);
            new_control.state_ambient_temp.set(ambient_temp);
            self.heater.set_control(new_control, true);

            let transfer_res =
                self.transfer_test(gcmd, ambient_max_measure_time, ambient_measure_sample_time, fan_breakpoints, &first_res)?;
            let (second_res, fan_ambient_transfer) =
                self.process_second_pass(&first_res, &transfer_res, ambient_temp, self.orig_control.heater_max_power);
            let mut repr = res_repr(&second_res);
            repr.truncate(repr.len() - 1);
            let fat = PyVal::List(fan_ambient_transfer.iter().map(|v| PyVal::Float(*v)).collect());
            logging::info(&format!("Second pass: {}, 'fan_ambient_transfer': {}}}", repr, fat.repr()));

            let block_heat_capacity =
                if use_analytic { second_res["block_heat_capacity"] } else { first_res["block_heat_capacity"] };
            let sensor_responsiveness =
                if use_analytic { second_res["sensor_responsiveness"] } else { first_res["sensor_responsiveness"] };
            let ambient_transfer = second_res["ambient_transfer"];
            let fan_ambient_transfer: Vec<String> = fan_ambient_transfer.iter().map(|p| fmt("%.6g", *p)).collect();
            let fan_ambient_transfer = fan_ambient_transfer.join(", ");

            let cfgname = self.heater.get_name().to_string();
            gcmd.respond_info(
                &format!(
                    "Finished MPC calibration of heater '{}'\nMeasured:\n   block_heat_capacity={} [J/K]\n  sensor_responsiveness={} [K/s/K]\n  ambient_transfer={} [W/K]\n  fan_ambient_transfer={} [W/K]\n",
                    cfgname,
                    fmt("%#.6g", block_heat_capacity),
                    fmt("%#.6g", sensor_responsiveness),
                    fmt("%#.6g", ambient_transfer),
                    fan_ambient_transfer
                ),
                true,
            );

            let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
            let mut cf = configfile.pconfig.borrow_mut();
            cf.set(&cfgname, "control", "mpc")?;
            cf.set(&cfgname, "block_heat_capacity", &fmt("%#.6g", block_heat_capacity))?;
            cf.set(&cfgname, "sensor_responsiveness", &fmt("%#.6g", sensor_responsiveness))?;
            cf.set(&cfgname, "ambient_transfer", &fmt("%#.6g", ambient_transfer))?;
            cf.set(&cfgname, "fan_ambient_transfer", &fan_ambient_transfer)?;
            Ok(())
        })();
        self.heater.set_control(old_control, true);
        self.heater.alter_target(0.0);
        match result {
            Err(e) if e.is_command_error() => Err(gcmd.error(format!("{} failed: {}", gcmd.get_command(), e.message()))),
            r => r,
        }
    }

    fn wait_stable(&self, cycles: i64) -> KResult<()> {
        let mut below_target = true;
        let mut above_target = 0;
        let mut on_target = 0;
        let starttime = self.printer.reactor.monotonic();
        self.printer.wait_while(
            |eventtime| {
                let (temp, target) = self.heater.get_temp(eventtime);
                if below_target && temp > target + 0.015 {
                    above_target += 1;
                    below_target = false;
                } else if !below_target && temp < target - 0.015 {
                    below_target = true;
                }
                if above_target >= cycles && (self.printer.reactor.monotonic() - starttime) > 30.0 {
                    return Ok(false);
                }
                if above_target > 0 && (target - temp).abs() < 0.1 {
                    on_target += 1;
                } else {
                    on_target = 0;
                }
                if on_target >= 150 {
                    // in case the heating is super consistent
                    return Ok(false);
                }
                Ok(true)
            },
            true,
            0.2,
        )
    }

    fn wait_settle(&self, max_rate: f64) -> KResult<f64> {
        let mut samples: Vec<(f64, f64)> = vec![];
        self.printer.wait_while(
            |eventtime| {
                let (temp, _) = self.heater.get_temp(eventtime);
                samples.push((eventtime, temp));
                while samples[0].0 < eventtime - 10.0 {
                    samples.remove(0);
                }
                let dtemp = samples[samples.len() - 1].1 - samples[0].1;
                let dt = samples[samples.len() - 1].0 - samples[0].0;
                if dt < 8.0 {
                    return Ok(true);
                }
                let rate = (dtemp / dt).abs();
                Ok(!(rate < max_rate))
            },
            true,
            1.0,
        )?;
        Ok(samples[samples.len() - 1].1)
    }

    fn await_ambient(&self, gcmd: &GCodeCommand, _control: &Rc<TuningControl>, minimum_temp: f64) -> KResult<f64> {
        self.heater.alter_target(1.0); // Turn on fan to increase settling speed
        if let Some(ambient_sensor) = &self.orig_control.ambient_sensor {
            // If we have an ambient sensor we won't waste time waiting for ambient.
            // We do however need to wait for sub minimum_temp(we pick -5 C relative).
            let mut reported = false;
            let target = minimum_temp - 5.0;
            self.printer.wait_while(
                |eventtime| {
                    let (temp, _) = self.heater.get_temp(eventtime);
                    let ret = temp > target;
                    if ret && !reported {
                        gcmd.respond_info(
                            &format!("Waiting for heater to drop below {} degrees Celsius", crate::pyfmt::py_float_repr(target)),
                            true,
                        );
                        reported = true;
                    }
                    Ok(ret)
                },
                true,
                1.0,
            )?;
            self.heater.alter_target(0.0);
            return obj_temp(ambient_sensor, self.heater.reactor.monotonic());
        }

        gcmd.respond_info("Waiting for heater to settle at ambient temperature", true);
        let ambient_temp = self.wait_settle(0.01)?;
        self.heater.alter_target(0.0);
        Ok(ambient_temp)
    }

    fn heatup_test(&self, gcmd: &GCodeCommand, target_temp: f64, control: &Rc<TuningControl>) -> KResult<Vec<(f64, f64)>> {
        gcmd.respond_info(&format!("Performing heatup test, target is {:.1} degrees", target_temp), true);
        control.set_output(self.heater.get_max_power(), target_temp)?;

        control.logging.set(true);

        self.printer.wait_while(
            |eventtime| {
                let (temp, _) = self.heater.get_temp(eventtime);
                Ok(temp < target_temp)
            },
            true,
            1.0,
        )?;
        control.logging.set(false);
        self.heater.alter_target(0.0);

        Ok(std::mem::take(&mut *control.log.borrow_mut()))
    }

    fn transfer_test(
        &self,
        gcmd: &GCodeCommand,
        ambient_max_measure_time: f64,
        ambient_measure_sample_time: f64,
        fan_breakpoints: i64,
        first_pass_results: &Res,
    ) -> KResult<TransferRes> {
        let target_temp = first_pass_results["post_block_temp"].round_ties_even();
        self.heater.set_temp(target_temp)?;
        gcmd.respond_info(&format!("Performing ambient transfer tests, target is {:.1} degrees", target_temp), true);

        self.wait_stable(5)?;

        let mut fan_powers = vec![];
        let power_base = match &self.orig_control.cooling_fan {
            None => {
                let power_base = self.measure_power(ambient_max_measure_time, ambient_measure_sample_time)?;
                gcmd.respond_info(&format!("Average stable power: {} W", crate::pyfmt::py_float_repr(power_base)), true);
                power_base
            }
            Some(fan) => {
                for idx in 0..fan_breakpoints {
                    let speed = idx as f64 / (fan_breakpoints - 1) as f64;
                    fan.set_speed(speed, None)?;
                    gcmd.respond_info("Waiting for temperature to stabilize", true);
                    self.wait_stable(3)?;
                    gcmd.respond_info(
                        &format!("Temperature stable, measuring power usage with {:.0}% fan speed", speed * 100.0),
                        true,
                    );
                    let power = self.measure_power(ambient_max_measure_time, ambient_measure_sample_time)?;
                    gcmd.respond_info(&format!("{:.0}% fan average power: {:.2} W", speed * 100.0, power), true);
                    fan_powers.push((speed, power));
                }
                fan.set_speed(0.0, None)?;
                fan_powers[0].1
            }
        };
        // python returns round() of a float: an int
        Ok(TransferRes { target_temp, base_power: power_base, fan_powers })
    }

    fn measure_power(&self, max_time: f64, sample_time: f64) -> KResult<f64> {
        let mut samples: Vec<(f64, f64)> = vec![];
        let mut time = 0.0;
        let mut last_time: Option<f64> = None;
        self.printer.wait_while(
            |eventtime| {
                let dt = eventtime - last_time.unwrap_or(eventtime);
                last_time = Some(eventtime);
                let status = self.heater.get_status(eventtime);
                let power = match &status {
                    PyVal::Dict(d) => match d.get("control_stats") {
                        Some(PyVal::Dict(cs)) => cs.get("power").and_then(|v| v.as_f64()),
                        _ => None,
                    },
                    _ => None,
                };
                let power = power.ok_or_else(|| KError::Other("'control_stats'".into()))?;
                samples.push((dt, power * dt));
                time += dt;
                Ok(time < max_time)
            },
            true,
            1.0,
        )?;

        let mut total_energy = 0.0;
        let mut total_time = 0.0;
        for (dt, energy) in samples.iter().rev() {
            total_energy += energy;
            total_time += dt;
            if total_time > sample_time {
                break;
            }
        }
        if total_time == 0.0 {
            return Err(KError::Other("float division by zero".into()));
        }
        Ok(total_energy / total_time)
    }

    fn fastest_rate(&self, samples: &[(f64, f64)]) -> [f64; 3] {
        let mut best = [-1.0, 0.0, 0.0];
        let base_t = samples[0].0;
        for idx in 2..samples.len() {
            let dtemp = samples[idx].1 - samples[idx - 2].1;
            let dt = samples[idx].0 - samples[idx - 2].0;
            let rate = dtemp / dt;
            if rate > best[0] {
                let sample = samples[idx - 1];
                best = [sample.0 - base_t, sample.1, rate];
            }
        }
        best
    }

    fn process_first_pass(
        &self,
        all_samples: &[(f64, f64)],
        heater_power: f64,
        ambient_temp: f64,
        threshold_temp: f64,
        use_analytic: bool,
    ) -> Res {
        // Find a continous segment of samples that all lie in the threshold.. range
        let mut best_lower: Option<usize> = None;
        for idx in 0..all_samples.len() {
            if all_samples[idx].1 > threshold_temp && best_lower.is_none() {
                best_lower = Some(idx);
            } else if all_samples[idx].1 < threshold_temp {
                best_lower = None;
            }
        }
        let best_lower = best_lower.unwrap_or(0);

        let t1_time = all_samples[best_lower].0 - all_samples[0].0;

        let samples = &all_samples[best_lower..];
        let pitch = (samples.len() - 1) / 2;
        // We pick samples 0, pitch, and 2pitch, ensuring matching time spacing
        let dt = samples[pitch].0 - samples[0].0;
        let t1 = samples[0].1;
        let t2 = samples[pitch].1;
        let t3 = samples[2 * pitch].1;

        let asymp_t = (t2 * t2 - t1 * t3) / (2.0 * t2 - t1 - t3);
        let block_responsiveness = -((t2 - asymp_t) / (t1 - asymp_t)).ln() / dt;
        let ambient_transfer = heater_power / (asymp_t - ambient_temp);

        let mut block_heat_capacity = -1.0;
        let mut sensor_responsiveness = -1.0;
        let start_temp = all_samples[0].1;

        // Asymptotic method
        if use_analytic {
            block_heat_capacity = ambient_transfer / block_responsiveness;
            sensor_responsiveness = block_responsiveness
                / (1.0 - (start_temp - asymp_t) * (-block_responsiveness * t1_time).exp() / (t1 - asymp_t));
        }

        // Differential method
        if !use_analytic || block_heat_capacity < 0.0 || sensor_responsiveness < 0.0 {
            let fastest_rate = self.fastest_rate(samples);
            block_heat_capacity = heater_power / fastest_rate[2];
            sensor_responsiveness = fastest_rate[2] / (fastest_rate[2] * fastest_rate[0] + ambient_temp - fastest_rate[0]);
        }

        let heat_time = all_samples[all_samples.len() - 1].0 - all_samples[0].0;
        let post_block_temp = asymp_t + (start_temp - asymp_t) * (-block_responsiveness * heat_time).exp();
        let post_sensor_temp = all_samples[all_samples.len() - 1].1;

        let mut r = Res::default();
        r.insert("post_block_temp", post_block_temp);
        r.insert("post_sensor_temp", post_sensor_temp);
        r.insert("block_responsiveness", block_responsiveness);
        r.insert("ambient_transfer", ambient_transfer);
        r.insert("block_heat_capacity", block_heat_capacity);
        r.insert("sensor_responsiveness", sensor_responsiveness);
        r.insert("asymp_temp", asymp_t);
        r.insert("t1", t1);
        r.insert("t1_time", t1_time);
        r.insert("t2", t2);
        r.insert("start_temp", start_temp);
        r.insert("dt", dt);
        r
    }

    fn process_second_pass(&self, first_res: &Res, transfer_res: &TransferRes, ambient_temp: f64, heater_power: f64) -> (Res, Vec<f64>) {
        let target_ambient_temp = transfer_res.target_temp - ambient_temp;
        let ambient_transfer = transfer_res.base_power / target_ambient_temp;
        let asymp_t = ambient_temp + heater_power / ambient_transfer;
        let block_responsiveness = -((first_res["t2"] - asymp_t) / (first_res["t1"] - asymp_t)).ln() / first_res["dt"];
        let block_heat_capacity = ambient_transfer / block_responsiveness;
        let sensor_responsiveness = block_responsiveness
            / (1.0
                - (first_res["start_temp"] - asymp_t) * (-block_responsiveness * first_res["t1_time"]).exp()
                    / (first_res["t1"] - asymp_t));

        let fan_ambient_transfer = transfer_res.fan_powers.iter().map(|(_speed, power)| power / target_ambient_temp).collect();

        let mut r = Res::default();
        r.insert("ambient_transfer", ambient_transfer);
        r.insert("block_responsiveness", block_responsiveness);
        r.insert("block_heat_capacity", block_heat_capacity);
        r.insert("sensor_responsiveness", sensor_responsiveness);
        r.insert("asymp_temp", asymp_t);
        (r, fan_ambient_transfer)
    }
}

pub struct TuningControl {
    value: Cell<f64>,
    target: Cell<Option<f64>>,
    heater: Rc<Heater>,
    log: RefCell<Vec<(f64, f64)>>,
    logging: Cell<bool>,
}

impl TuningControl {
    fn new(heater: &Rc<Heater>) -> Self {
        TuningControl { value: Cell::new(0.0), target: Cell::new(None), heater: heater.clone(), log: RefCell::new(vec![]), logging: Cell::new(false) }
    }

    fn set_output(&self, value: f64, target: f64) -> KResult<()> {
        self.value.set(value);
        self.target.set(Some(target));
        self.heater.set_temp(target)
    }
}

impl Control for TuningControl {
    fn temperature_update(&self, read_time: f64, temp: f64, _target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        if secondary_temp.is_some() {
            return Err(KError::Other("TuningControl.temperature_update() takes 4 positional arguments but 5 were given".into()));
        }
        if self.logging.get() {
            self.log.borrow_mut().push((read_time, temp));
        }
        self.heater.set_pwm(read_time, self.value.get())
    }
    fn check_busy(&self, _eventtime: f64, _smoothed_temp: f64, _target_temp: f64) -> bool {
        self.value.get() != 0.0 || self.target.get() != Some(0.0)
    }
    fn get_profile(&self) -> Profile {
        let p: Profile = Rc::new(RefCell::new(IndexMap::default()));
        p.borrow_mut().insert("name".into(), ProfVal::V(PyVal::from("tuning")));
        p
    }
    fn get_type(&self) -> &'static str {
        "tuning"
    }
    fn update_smooth_time(&self) -> KResult<()> {
        Err(KError::Other("'TuningControl' object has no attribute 'update_smooth_time'".into()))
    }
}
