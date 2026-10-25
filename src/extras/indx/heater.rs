// INDX induction heater (port of klippy/extras/indx/heater.py)
use super::calibration::{self as cal, CalSample, FanSample};
use super::heatsink_fan::IndxHeatsinkFan;
use super::toolboard::IndxToolboard;
use super::{compat_config, register_adc_callback, register_response, CVal};
use crate::chelper::serialqueue::CommandQueue;
use crate::configfile::{strftime_local, ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::fan::Fan;
use crate::extras::heaters::{no_secondary, Control, ControlRef, Heater, PrinterHeaters, Profile, TempCallback, TempSensor};
use crate::extras::query_adc::QueryADC;
use crate::extras::stepper_enable::EnableTracking;
use crate::extras::thermistor::{custom_thermistor, printer_thermistor};
use crate::extras::tmc::StepperCurrentHelper;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::kinematics::extruder::PrinterExtruder;
use crate::logging;
use crate::mcu::{AdcPin, CommandQueryWrapper, CommandWrapper, Mcu, McuAdc, PwmPin};
use crate::msgproto::{Params, Val};
use crate::pins::{PinChip, PinParams};
use crate::printer::{ConfigFileObject, PObj, Printer};
use crate::pyfmt::{py_float_repr, py_max, py_min, py_sum};
use crate::pyval::PyVal;
use crate::reactor::Reactor;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::io::Write as _;
use std::rc::{Rc, Weak};

const KELVIN_OFFSET: f64 = 273.15;

// Mirrors the MCU coil-tuner enums (coil_driver.c++ tune_status / tune_error).
fn coil_tune_status(v: i64) -> Option<&'static str> {
    match v {
        0 => Some("idle"),
        1 => Some("running"),
        2 => Some("success"),
        3 => Some("failed"),
        _ => None,
    }
}
const COIL_TUNE_STATUS_SUCCESS: i64 = 2;
const COIL_TUNE_STATUS_FAILED: i64 = 3;
fn coil_tune_error(v: i64) -> Option<&'static str> {
    match v {
        0 => Some("none"),
        1 => Some("overvoltage triggered during the tune (target removed?)"),
        2 => Some("ON-time tuning hit maximum time without reaching the target peak"),
        3 => Some("the resonance never returned to zero within the sweep window"),
        4 => Some("steady ON hit the ceiling without reaching the target"),
        5 => Some("timed out"),
        _ => None,
    }
}
const COIL_TUNE_STATUS_RUNNING: i64 = 1;
const COIL_TUNE_TIMEOUT: f64 = 15.0; // seconds. The MCU self-aborts a stuck tune at 10 s
const COIL_TUNE_POLL_INTERVAL: f64 = 0.2; // seconds between latched-status polls
const COIL_TUNE_WAVE_PATH: &str = "/tmp/indx_coil_waveform.csv";
const FILAMENT_LOAD_THRESHOLD: f64 = 10.0;
const FILAMENT_LOAD_PRIME_TIME: f64 = 5.0;
const FILAMENT_LOAD_MAX_LENGTH: f64 = 120.0;
const FILAMENT_LOAD_SPEED: f64 = 10.0;
const FILAMENT_LOAD_SEGMENT_TIME: f64 = 1.0;

/// NozzleTemperature(time, nozzle_temperature, sensor_temperature,
/// delta_time, delta_energy, pwm_avg, delta_pwm_energy)
#[derive(Clone, Debug)]
pub struct NozzleTemperature {
    pub time: f64,
    pub nozzle_temperature: f64,
    pub sensor_temperature: f64,
    pub delta_time: Option<f64>,
    pub delta_energy: Option<f64>,
    pub pwm_avg: f64,
    pub delta_pwm_energy: f64,
}

pub type NozzleCallback = Rc<dyn Fn(&NozzleTemperature) -> KResult<()>>;

/// int(struct.unpack("!i", struct.pack("!f", val))[0])
pub fn float_to_u32(val: f64) -> i64 {
    (val as f32).to_bits() as i32 as i64
}

fn param(params: &Params, key: &str) -> i64 {
    params.get(key).and_then(|v| v.as_int()).unwrap_or(0)
}

/// gcmd.get_float(name, default=value) where the default may be None
fn get_float_or(gcmd: &GCodeCommand, name: &str, default: Option<f64>, lim: Lim) -> KResult<Option<f64>> {
    Ok(gcmd.get_float_opt(name, lim)?.or(default))
}

/// str() of an optional float
fn opt_str(v: Option<f64>) -> String {
    v.map(py_float_repr).unwrap_or_else(|| "None".into())
}

pub struct IndxThermistorWrapper {
    /// keeps the sensor alive (its adc callback holds this weakly)
    _sensor: Rc<dyn TempSensor>,
    last_temp: Cell<Option<f64>>,
    temperature_callbacks: RefCell<Vec<TempCallback>>,
}

impl IndxThermistorWrapper {
    fn new(
        config: &ConfigWrapper,
        mcu: &Mcu,
        toolboard_name: &str,
        sensor: &str,
        def_max_temp: f64,
        thermistor_config: &[(&str, CVal)],
    ) -> KResult<Rc<Self>> {
        let mcu_name = mcu.get_name();
        let config_thermistor = compat_config(config, &format!("indx_{}_thermistor", sensor), thermistor_config);
        let config_sensor = compat_config(
            config,
            &format!("temperature_sensor {}_{}", toolboard_name, sensor),
            &[("sensor_pin", CVal::S(format!("{}:temp_{}", mcu_name, sensor))), ("pullup_resistor", CVal::I(3900))],
        );
        let params = custom_thermistor(&config_thermistor)?;
        let temp_sensor = printer_thermistor(&config_sensor, &params)?;
        let max_temp = config.getfloat(&format!("max_temp_{}", sensor), def_max_temp, Lim::new().min(0.0))?;
        temp_sensor.setup_minmax(-KELVIN_OFFSET, max_temp)?;
        let w = Rc::new(IndxThermistorWrapper { _sensor: temp_sensor.clone(), last_temp: Cell::new(None), temperature_callbacks: RefCell::new(vec![]) });
        let ww = Rc::downgrade(&w);
        temp_sensor.setup_callback(Rc::new(move |read_time, value| match ww.upgrade() {
            Some(w) => w.callback(read_time, value),
            None => Ok(()),
        }));
        Ok(w)
    }

    pub fn get_last(&self) -> Option<f64> {
        self.last_temp.get()
    }

    pub fn add_callback(&self, callback: TempCallback) {
        self.temperature_callbacks.borrow_mut().push(callback);
    }

    fn callback(&self, read_time: f64, value: f64) -> KResult<()> {
        self.last_temp.set(Some(value));
        let cbs = self.temperature_callbacks.borrow().clone();
        for cb in cbs {
            cb(read_time, value)?;
        }
        Ok(())
    }
}

struct IndxBracketTempSensor {
    mcu: Rc<Mcu>,
    /// (set once the heater exists; python reads heater.cmd_queue at connect)
    heater: RefCell<Weak<IndxToolboardHeater>>,
    cmd_set_ambient_temp: RefCell<Option<CommandWrapper>>,
    last_temperature: Cell<Option<f64>>,
    force_temp: Cell<Option<f64>>,
}

impl IndxBracketTempSensor {
    fn new(printer: &Rc<Printer>, mcu: &Rc<Mcu>, sensor: &IndxThermistorWrapper) -> KResult<Rc<Self>> {
        let s = Rc::new(IndxBracketTempSensor {
            mcu: mcu.clone(),
            heater: RefCell::new(Weak::new()),
            cmd_set_ambient_temp: RefCell::new(None),
            last_temperature: Cell::new(None),
            force_temp: Cell::new(None),
        });
        let w = Rc::downgrade(&s);
        sensor.add_callback(Rc::new(move |read_time, value| match w.upgrade() {
            Some(s) => s.callback(read_time, value),
            None => Ok(()),
        }));
        let w = Rc::downgrade(&s);
        mcu.register_config_callback(Rc::new(move || match w.upgrade() {
            Some(s) => s.build_config(),
            None => Ok(()),
        }));
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let w = Rc::downgrade(&s);
        gcode.register_command(
            "INDX_FORCE_BRACKET_TEMP",
            Some(Rc::new(move |g| match w.upgrade() {
                Some(s) => s.cmd_force_bracket_temp(g),
                None => Ok(()),
            })),
            false,
            None,
        )?;
        Ok(s)
    }

    fn build_config(&self) -> KResult<()> {
        let cq = self.heater.borrow().upgrade().map(|h| h.cmd_queue.clone());
        *self.cmd_set_ambient_temp.borrow_mut() = Some(self.mcu.lookup_command("indx_set_bracket_temp temp=%u", cq)?);
        Ok(())
    }

    fn callback(&self, _read_time: f64, value: f64) -> KResult<()> {
        let value = if value < 0.0 { None } else { Some(value) };
        self.last_temperature.set(value);
        let value = self.force_temp.get().or(value);
        let Some(value) = value else { return Ok(()) };
        let cmd = self.cmd_set_ambient_temp.borrow();
        let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'IndxBracketTempSensor' object has no attribute 'cmd_set_ambient_temp'".into()))?;
        cmd.send(&[Val::Int(float_to_u32(value + KELVIN_OFFSET))], 0, 0)
    }

    fn cmd_force_bracket_temp(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.force_temp.set(gcmd.get_float_opt("TEMP", Lim::new())?);
        Ok(())
    }
}

/// ThermalModelParams namedtuple
#[derive(Clone, Debug)]
pub struct ThermalModelParams {
    pub max_power: Option<f64>,
    pub max_power_temp_coeff: Option<f64>,
    pub thermal_capacity: Option<f64>,
    pub to_ambient_r: Option<f64>,
    pub filament_radius: f64,
    pub filament_density: f64,
    pub filament_heat_capacity: f64,
    pub part_cooling_fan_a: f64,
    pub part_cooling_fan_k: f64,
    pub ambient_blend_bracket: f64,
    pub ambient_blend_board: f64,
    pub ambient_blend_sensor: f64,
    pub error_application: f64,
}

impl ThermalModelParams {
    /// "\n".join(f"{k}: {v}" for k, v in params._asdict().items())
    fn describe(&self) -> String {
        let f = py_float_repr;
        [
            format!("max_power: {}", opt_str(self.max_power)),
            format!("max_power_temp_coeff: {}", opt_str(self.max_power_temp_coeff)),
            format!("thermal_capacity: {}", opt_str(self.thermal_capacity)),
            format!("to_ambient_r: {}", opt_str(self.to_ambient_r)),
            format!("filament_radius: {}", f(self.filament_radius)),
            format!("filament_density: {}", f(self.filament_density)),
            format!("filament_heat_capacity: {}", f(self.filament_heat_capacity)),
            format!("part_cooling_fan_a: {}", f(self.part_cooling_fan_a)),
            format!("part_cooling_fan_k: {}", f(self.part_cooling_fan_k)),
            format!("ambient_blend_bracket: {}", f(self.ambient_blend_bracket)),
            format!("ambient_blend_board: {}", f(self.ambient_blend_board)),
            format!("ambient_blend_sensor: {}", f(self.ambient_blend_sensor)),
            format!("error_application: {}", f(self.error_application)),
        ]
        .join("\n")
    }
}

/// ThermalModelStep namedtuple
#[derive(Clone, Debug)]
pub struct ThermalModelStep {
    current_temp: f64,
    avg_input_power: f64,
    ambient_temp: f64,
    filament_temp: f64,
    loss_ambient: f64,
    loss_filament: f64,
    loss_part_cooling: f64,
    delta_temp_rate: f64,
    dt: f64,
    new_temp: f64,
}

impl ThermalModelStep {
    fn describe(&self) -> String {
        let f = py_float_repr;
        [
            format!("current_temp: {}", f(self.current_temp)),
            format!("avg_input_power: {}", f(self.avg_input_power)),
            format!("ambient_temp: {}", f(self.ambient_temp)),
            format!("filament_temp: {}", f(self.filament_temp)),
            format!("loss_ambient: {}", f(self.loss_ambient)),
            format!("loss_filament: {}", f(self.loss_filament)),
            format!("loss_part_cooling: {}", f(self.loss_part_cooling)),
            format!("delta_temp_rate: {}", f(self.delta_temp_rate)),
            format!("dt: {}", f(self.dt)),
            format!("new_temp: {}", f(self.new_temp)),
        ]
        .join("\n")
    }
}

pub struct ThermalModel {
    pub temperature: f64,
    pub params: ThermalModelParams,
    last_step: Option<ThermalModelStep>,
}

impl ThermalModel {
    fn new(params: ThermalModelParams) -> Self {
        ThermalModel { temperature: 0.0, params, last_step: None }
    }

    pub fn is_valid(&self) -> bool {
        self.params.max_power.is_some()
            && self.params.max_power_temp_coeff.is_some()
            && self.params.thermal_capacity.is_some()
            && self.params.to_ambient_r.is_some()
    }

    fn force_temp(&mut self, temperature: f64) {
        self.temperature = temperature;
    }

    fn calc_step(&self, dt: f64, avg_input_power: f64, ambient_temp: f64, filament_temp: f64, filament_distance: f64, fan_speed: f64) -> ThermalModelStep {
        let p = &self.params;
        let filament_heat_capacity_mm = (p.filament_radius * p.filament_radius) * PI / 1000.0 * p.filament_density * p.filament_heat_capacity;

        let mut loss_part_cooling = 0.0;
        if fan_speed > 0.0 {
            let pcf_ambient_r = p.part_cooling_fan_a * fan_speed.powf(-p.part_cooling_fan_k);
            if pcf_ambient_r > 0.0 {
                loss_part_cooling = py_max(0.0, (self.temperature - ambient_temp) / pcf_ambient_r);
            }
        }

        let to_ambient_r = p.to_ambient_r.unwrap_or(f64::NAN);
        let loss_ambient = (self.temperature - ambient_temp) / to_ambient_r;
        let loss_filament = filament_distance * filament_heat_capacity_mm * (self.temperature - filament_temp) / dt;
        let total_power = avg_input_power - loss_ambient - loss_filament - loss_part_cooling;
        let delta_temp_rate = total_power / p.thermal_capacity.unwrap_or(f64::NAN);
        let new_temp = self.temperature + delta_temp_rate * dt;

        ThermalModelStep {
            current_temp: self.temperature,
            avg_input_power,
            ambient_temp,
            filament_temp,
            loss_ambient,
            loss_filament,
            loss_part_cooling,
            dt,
            delta_temp_rate,
            new_temp,
        }
    }

    fn step(&mut self, dt: f64, avg_input_power: f64, ambient_temp: f64, filament_temp: f64, filament_distance: f64, fan_speed: f64) {
        let info = self.calc_step(dt, avg_input_power, ambient_temp, filament_temp, filament_distance, fan_speed);
        self.temperature = info.new_temp;
        self.last_step = Some(info);
    }
}

/// the last nozzle report: (time, charge, power, extruder_pos)
type LastReport = (f64, i64, f64, f64);

/// the commands looked up at build_config
#[derive(Default)]
struct HeaterCmds {
    debug_stream_raw_ir: Option<CommandWrapper>,
    tune_coil: Option<CommandWrapper>,
    query_coil_tune: Option<CommandQueryWrapper>,
    set_control_params: Option<CommandWrapper>,
    set_target: Option<CommandWrapper>,
}

pub struct IndxToolboardHeater {
    me: Weak<IndxToolboardHeater>,
    printer: Rc<Printer>,
    mcu: Rc<Mcu>,
    toolboard_name: String,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    heater: RefCell<Option<Rc<Heater>>>,
    fan: Rc<IndxHeatsinkFan>,
    aux_temp_sensors: Vec<(&'static str, Rc<IndxThermistorWrapper>)>,
    _bracket_temp_update: RefCell<Option<Rc<IndxBracketTempSensor>>>,
    /// keeps the vin_mon adc alive
    _vin_adc: Rc<dyn AdcPin>,
    supply_voltage: Cell<f64>,
    thermal_model: RefCell<ThermalModel>,
    max_model_error: Cell<f64>,
    pid: RefCell<[f64; 6]>,
    max_temp_nozzle: f64,
    max_temp_sensor: f64,
    ir_sensor_tuning: Option<(f64, f64, f64)>,
    coil_timings: Cell<Option<(f64, f64, f64)>>,
    part_cooling_fan: Option<Rc<Fan>>,
    temperature_callbacks: RefCell<Vec<NozzleCallback>>,
    pub cmd_queue: CommandQueue,
    last_report: Cell<Option<LastReport>>,
    power_scale: Cell<f64>,
    heaters: RefCell<Vec<Rc<Heater>>>,
    cmds: RefCell<HeaterCmds>,
    log_file: RefCell<Option<std::fs::File>>,
    raw_ir_log_file: RefCell<Option<std::fs::File>>,
    raw_ir_log_sensors: RefCell<Vec<String>>,
}

/// the fan of a fan-like printer object (python: hasattr(obj, "fan"))
fn fan_of(obj: &PObj) -> Option<Rc<Fan>> {
    let any: &dyn std::any::Any = &**obj;
    if let Some(f) = any.downcast_ref::<crate::extras::fan::PrinterFan>() {
        return Some(f.fan.clone());
    }
    if let Some(f) = any.downcast_ref::<crate::extras::fan_generic::PrinterFanGeneric>() {
        return Some(f.fan.clone());
    }
    if let Some(f) = any.downcast_ref::<crate::extras::heater_fan::PrinterHeaterFan>() {
        return Some(f.fan.clone());
    }
    if let Some(f) = any.downcast_ref::<crate::extras::controller_fan::ControllerFan>() {
        return Some(f.fan.clone());
    }
    if let Some(f) = any.downcast_ref::<crate::extras::temperature_fan::TemperatureFan>() {
        return Some(f.fan.clone());
    }
    if let Some(f) = any.downcast_ref::<IndxHeatsinkFan>() {
        return Some(f.fan.clone());
    }
    None
}

impl IndxToolboardHeater {
    pub fn new(toolboard: &Rc<IndxToolboard>, config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = toolboard.printer.clone();
        let mcu = toolboard.mcu.clone();
        let name = toolboard.name.clone();
        let fan = IndxHeatsinkFan::new(config, &printer, &mcu, &name)?;
        let bracket = IndxThermistorWrapper::new(
            config,
            &mcu,
            &name,
            "bracket",
            130.0,
            &[
                // Source:
                // https://amphenol-sensors.com/hubfs/Documents/AAS-913-318C-Temperature-resistance-curves-071816-web.pdf
                // Material Type 1, products NK
                ("temperature1", CVal::F(25.0)),
                ("resistance1", CVal::F(10000.0)),
                ("temperature2", CVal::F(65.0)),
                ("resistance2", CVal::F(2080.0)),
                ("temperature3", CVal::F(100.0)),
                ("resistance3", CVal::F(678.0)),
            ],
        )?;
        let board = IndxThermistorWrapper::new(
            config,
            &mcu,
            &name,
            "board",
            100.0,
            &[
                // 10K Murata NCU15XH103J6SRC
                ("beta", CVal::F(3425.0)),
                ("temperature1", CVal::F(25.0)),
                ("resistance1", CVal::F(10000.0)),
            ],
        )?;

        // (the heater is built below; the bracket sensor reaches it weakly)
        let slot: Rc<RefCell<Weak<IndxToolboardHeater>>> = Rc::new(RefCell::new(Weak::new()));
        let bracket_temp_update = IndxBracketTempSensor::new(&printer, &mcu, &bracket)?;

        let pin_chip: Rc<dyn PinChip> = mcu.clone();
        let pin_params = Rc::new(RefCell::new(PinParams {
            chip: pin_chip.clone(),
            chip_name: mcu.get_name().to_string(),
            pin: "vin_mon".into(),
            invert: 0,
            pullup: 0,
            share_type: None,
            class: None,
        }));
        let vin_adc: Rc<dyn AdcPin> = pin_chip
            .setup_pin("adc", &pin_params)
            .map_err(KError::Pins)?
            .downcast::<McuAdc>()
            .map_err(|_| KError::Other("mcu adc pin of an unexpected type".into()))?;
        let sl = slot.clone();
        register_adc_callback(
            &vin_adc,
            Rc::new(move |read_time, read_value| {
                if let Some(h) = sl.borrow().upgrade() {
                    h.handle_vin_mon(read_time, read_value);
                }
                Ok(())
            }),
        );
        let query_adc = printer.load_as::<QueryADC>(config, "query_adc")?;
        query_adc.register_adc(&format!("{}_vin_mon", mcu.get_name()), vin_adc.clone());

        // The autotuned thermal-model parameters have no defaults.
        // The user must run INDX_CALIBRATE to generate them.
        let model_params = ThermalModelParams {
            max_power: config.getfloat_opt("max_power", Lim::new().above(0.0))?,
            max_power_temp_coeff: config.getfloat_opt("model_max_power_temp_coeff", Lim::new())?,
            thermal_capacity: config.getfloat_opt("model_thermal_capacity", Lim::new().above(0.0))?,
            to_ambient_r: config.getfloat_opt("model_to_ambient_r", Lim::new().above(0.0))?,
            filament_radius: config.getfloat("model_filament_diameter", 1.75, Lim::new())? / 2.0,
            filament_density: config.getfloat("model_filament_density", 1.20, Lim::new())?,
            filament_heat_capacity: config.getfloat("model_filament_heat_capacity", 1.8, Lim::new())?,
            part_cooling_fan_a: config.getfloat("model_part_cooling_fan_a", 0.0, Lim::new())?,
            part_cooling_fan_k: config.getfloat("model_part_cooling_fan_k", 0.0, Lim::new())?,
            ambient_blend_board: config.getfloat("model_ambient_blend_board", 0.0, Lim::new())?,
            ambient_blend_bracket: config.getfloat("model_ambient_blend_bracket", 1.0, Lim::new())?,
            ambient_blend_sensor: config.getfloat("model_ambient_blend_sensor", 1.0, Lim::new())?,
            error_application: config.getfloat("model_error_application", 1.0, Lim::new())?,
        };
        let max_model_error = config.getfloat("max_model_error", 50.0, Lim::new())?;

        let pid = [
            config.getfloat("pid_kp", 4.0, Lim::new())?,
            config.getfloat("pid_ti", 0.2, Lim::new().min(0.0))?,
            config.getfloat("pid_td", 0.0, Lim::new().min(0.0))?,
            config.getfloat("pid_b", 1.0, Lim::new().min(0.0).max(1.0))?,
            config.getfloat("pid_i_window", 0.2, Lim::new().above(0.0))?,
            config.getfloat("pid_i_limit", 0.5, Lim::new().above(0.0).max(1.0))?,
        ];
        let max_temp_nozzle = config.getfloat("max_temp_nozzle", 305.0, Lim::new())?;
        let max_temp_sensor = config.getfloat("max_temp_sensor", 130.0, Lim::new())?;

        let tuning = (
            config.getfloat_opt("ir_sensor_exponent", Lim::new())?,
            config.getfloat_opt("ir_sensor_obj_gain", Lim::new())?,
            config.getfloat_opt("ir_sensor_bracket_gain", Lim::new())?,
        );
        let ir_sensor_tuning = match tuning {
            (Some(a), Some(b), Some(c)) => Some((a, b, c)),
            (None, None, None) => None,
            _ => return Err(KError::config("All of ir_sensor_{exponent,obj_gain,bracket_gain} must be given if one is")),
        };

        // Coil-drive timings (microseconds) to apply to the MCU at connect. When not set,
        // heating can't be done, but we allow not setting them so the user can run the
        // tuning routine. These are specified in microseconds by the user.
        let lim = || Lim::new().above(0.0).below(10.0);
        let coil = (
            config.getfloat_opt("coil_time_on", lim())?,
            config.getfloat_opt("coil_time_off", lim())?,
            config.getfloat_opt("coil_time_on_first", lim())?,
        );
        let coil_timings = match coil {
            (Some(t_on), Some(t_off), Some(t_on_first)) => {
                if t_on_first > t_on {
                    return Err(KError::config("coil_time_on_first must not exceed coil_time_on"));
                }
                // microseconds -> seconds for the MCU
                Some((t_on * 1e-6, t_off * 1e-6, t_on_first * 1e-6))
            }
            (None, None, None) => None,
            _ => return Err(KError::config("coil_time_on, coil_time_off, coil_time_on_first must be set together")),
        };

        let fan_name = config.get("part_cooling_fan", "fan")?;
        let mut part_cooling_fan = None;
        if !fan_name.is_empty() {
            let fan_obj = match printer.load_object(config, &fan_name, false)? {
                Some(o) if !o.is_none() => o,
                _ => printer.lookup_object(&fan_name)?,
            };
            let Some(f) = fan_of(&fan_obj) else {
                return Err(KError::config(format!("cooling_fan '{}' is not a valid fan object", fan_name)));
            };
            part_cooling_fan = Some(f);
        }

        let cmd_queue = mcu.alloc_command_queue();
        let h = Rc::new_cyclic(|me| IndxToolboardHeater {
            me: me.clone(),
            printer: printer.clone(),
            mcu: mcu.clone(),
            toolboard_name: name,
            toolhead: RefCell::new(None),
            heater: RefCell::new(None),
            fan,
            aux_temp_sensors: vec![("bracket", bracket), ("board", board)],
            _bracket_temp_update: RefCell::new(None),
            _vin_adc: vin_adc,
            supply_voltage: Cell::new(24.0),
            thermal_model: RefCell::new(ThermalModel::new(model_params)),
            max_model_error: Cell::new(max_model_error),
            pid: RefCell::new(pid),
            max_temp_nozzle,
            max_temp_sensor,
            ir_sensor_tuning,
            coil_timings: Cell::new(coil_timings),
            part_cooling_fan,
            temperature_callbacks: RefCell::new(vec![]),
            cmd_queue,
            last_report: Cell::new(None),
            power_scale: Cell::new(0.0),
            heaters: RefCell::new(vec![]),
            cmds: RefCell::new(HeaterCmds::default()),
            log_file: RefCell::new(None),
            raw_ir_log_file: RefCell::new(None),
            raw_ir_log_sensors: RefCell::new(vec![]),
        });
        *slot.borrow_mut() = Rc::downgrade(&h);
        *bracket_temp_update.heater.borrow_mut() = Rc::downgrade(&h);
        *h._bracket_temp_update.borrow_mut() = Some(bracket_temp_update);

        let w = Rc::downgrade(&h);
        mcu.register_config_callback(Rc::new(move || match w.upgrade() {
            Some(h) => h.build_config(),
            None => Ok(()),
        }));
        let w = Rc::downgrade(&h);
        crate::events::KlippyConnect::on(&printer, move || match w.upgrade() {
            Some(h) => h.handle_connect(),
            None => Ok(()),
        });

        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        type HCmd = fn(&IndxToolboardHeater, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, HCmd); 14] = [
            ("INDX_SET_PID", IndxToolboardHeater::cmd_set_pid),
            ("INDX_SET_CYCLE_LIMIT", IndxToolboardHeater::cmd_set_cycle_limit),
            ("INDX_SET_IR_SENSOR_PARAMS", IndxToolboardHeater::cmd_set_ir_sensor_params),
            ("INDX_CALIBRATE", IndxToolboardHeater::cmd_calibrate),
            ("INDX_FAN_CALIBRATE", IndxToolboardHeater::cmd_fan_calibrate),
            ("INDX_MEASURE_POWER", IndxToolboardHeater::cmd_measure_power),
            ("INDX_LOAD_FILAMENT", IndxToolboardHeater::cmd_load_filament),
            ("INDX_CLEAR_FILAMENT", IndxToolboardHeater::cmd_clear_filament),
            ("INDX_EXTRUDER_MOVE", IndxToolboardHeater::cmd_extruder_move),
            ("INDX_SET_MODEL_PARAMS", IndxToolboardHeater::cmd_set_model_params),
            ("INDX_DUMP_MODEL_UPDATE", IndxToolboardHeater::cmd_dump_model_update),
            ("INDX_LOG", IndxToolboardHeater::cmd_log),
            ("INDX_DEBUG_IR_SENSOR_EEPROM", IndxToolboardHeater::cmd_debug_ir_sensor_eeprom),
            ("INDX_DEBUG_STREAM_RAW_IR_SENSOR", IndxToolboardHeater::cmd_debug_stream_raw_ir_sensor),
        ];
        for (cmd, f) in cmds {
            let w = Rc::downgrade(&h);
            gcode.register_command(
                cmd,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(h) => f(&h, g),
                    None => Ok(()),
                })),
                false,
                None,
            )?;
        }
        Ok(h)
    }

    pub fn teardown(&self) {
        if let Some(heater) = self.heater.borrow_mut().take() {
            heater.set_set_temp_hook(None);
        }
        self.heaters.borrow_mut().clear();
        self.temperature_callbacks.borrow_mut().clear();
        *self._bracket_temp_update.borrow_mut() = None;
    }

    pub fn add_temperature_callback(&self, cb: NozzleCallback) {
        self.temperature_callbacks.borrow_mut().push(cb);
    }

    fn remove_temperature_callback(&self, cb: &NozzleCallback) {
        let mut cbs = self.temperature_callbacks.borrow_mut();
        if let Some(pos) = cbs.iter().position(|c| Rc::ptr_eq(c, cb)) {
            cbs.remove(pos);
        }
    }

    pub fn model_temperature(&self) -> f64 {
        self.thermal_model.borrow().temperature
    }

    pub fn aux_temp_sensor(&self, name: &str) -> Rc<IndxThermistorWrapper> {
        self.aux_temp_sensors.iter().find(|(n, _)| *n == name).map(|(_, s)| s.clone()).expect("aux sensor")
    }

    fn cmd<T>(&self, f: impl FnOnce(&HeaterCmds) -> Option<&T>, use_it: impl FnOnce(&T) -> KResult<()>) -> KResult<()> {
        let cmds = self.cmds.borrow();
        let c = f(&cmds).ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        use_it(c)
    }

    fn build_config(&self) -> KResult<()> {
        let mcu = &self.mcu;

        let w = self.me.clone();
        register_response(
            mcu,
            Rc::new(move |p| match w.upgrade() {
                Some(h) => h.handle_report_nozzle_temp(p),
                None => Ok(()),
            }),
            "indx_nozzle_temp clock=%u nozzle=%u sensor=%u power=%u charge=%u overvoltage=%u",
        );

        let w = self.me.clone();
        register_response(
            mcu,
            Rc::new(move |p| match w.upgrade() {
                Some(h) => h.handle_debug_raw_ir(p),
                None => Ok(()),
            }),
            "indx_debug_raw_ir object=%u ambient=%u",
        );
        let cq = Some(self.cmd_queue.clone());
        let mut cmds = HeaterCmds {
            debug_stream_raw_ir: Some(mcu.lookup_command("indx_debug_stream_raw_ir_sensor enable=%c", cq.clone())?),
            tune_coil: Some(mcu.lookup_command("indx_tune_coil debug=%c", cq.clone())?),
            query_coil_tune: Some(mcu.lookup_query_command(
                "indx_query_coil_tune",
                "indx_coil_tune_result status=%c error=%c on_first=%u off=%u on=%u",
                None,
                cq.clone(),
                false,
            )?),
            ..Default::default()
        };
        let amp_per_count = mcu.get_constant_float("INDX_CURRENT_SENSE_AMP_PER_COUNT")?;
        let current_sense_rate = mcu.get_constant_float("INDX_CURRENT_SENSE_RATE")?;
        self.power_scale.set(amp_per_count / current_sense_rate);

        let p = *self.pid.borrow();
        mcu.add_config_cmd(
            &format!(
                "indx_set_control_params kp={} ti={} td={} b={} i_window={} i_limit={}",
                float_to_u32(p[0]),
                float_to_u32(p[1]),
                float_to_u32(p[2]),
                float_to_u32(p[3]),
                float_to_u32(p[4]),
                float_to_u32(p[5])
            ),
            false,
            false,
        );
        cmds.set_control_params = Some(mcu.lookup_command("indx_set_control_params kp=%u ti=%u td=%u b=%u i_window=%u i_limit=%u", cq.clone())?);
        cmds.set_target = Some(mcu.lookup_command("indx_set_target target=%u", cq)?);
        *self.cmds.borrow_mut() = cmds;

        if let Some((exponent, obj_gain, bracket_gain)) = self.ir_sensor_tuning {
            mcu.add_config_cmd(
                &format!(
                    "indx_set_ir_sensor_params exponent={} obj_gain={} bracket_gain={}",
                    float_to_u32(exponent),
                    float_to_u32(obj_gain),
                    float_to_u32(bracket_gain)
                ),
                false,
                false,
            );
        }

        if let Some((t_on, t_off, t_on_first)) = self.coil_timings.get() {
            mcu.add_config_cmd(
                &format!(
                    "indx_set_coil_driver_params time_on={} time_off={} time_on_first={}",
                    float_to_u32(t_on),
                    float_to_u32(t_off),
                    float_to_u32(t_on_first)
                ),
                false,
                false,
            );
        }
        Ok(())
    }

    fn handle_connect(&self) -> KResult<()> {
        // Look for heaters that have us as their PWM output
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let me_ptr = self as *const IndxToolboardHeater as *const ();
        for heater_name in pheaters.get_all_heaters() {
            let heater = pheaters.lookup_heater(&heater_name)?;
            if Rc::as_ptr(&heater.mcu_pwm()) as *const () != me_ptr {
                continue;
            }
            *self.heater.borrow_mut() = Some(heater.clone());
            let control = Rc::new(HeaterControlIndx { heater: Rc::downgrade(&heater), indx_heater: self.me.clone(), profile: RefCell::new(None) });
            let c: ControlRef = control.clone();
            heater.set_control(c, true);

            // Refuse to heat until the coil driver and thermal model are tuned
            // (uncalibrated coil timings also make the MCU shut down). Monkey-
            // patch set_temp so we can give a clear command error to the user.
            let w = self.me.clone();
            let hw = Rc::downgrade(&heater);
            heater.set_set_temp_hook(Some(Rc::new(move |degrees| {
                if let Some(h) = w.upgrade() {
                    if degrees != 0.0 && !h.heater_config_valid() {
                        return Err(KError::command("INDX heater has not been calibrated yet. Run INDX_CALIBRATE to perform calibration."));
                    }
                }
                match hw.upgrade() {
                    Some(heater) => heater.raw_set_temp(degrees),
                    None => Ok(()),
                }
            })));

            self.fan.add_heater(heater.clone());
            self.heaters.borrow_mut().push(heater.clone());

            // Support for Kalico profile system
            *control.profile.borrow_mut() = Some(heater.pm_init_default_profile()?);

            break;
        }
        Ok(())
    }

    fn apply_pid_params(&self) -> KResult<()> {
        let p = *self.pid.borrow();
        self.cmd(|c| c.set_control_params.as_ref(), |cmd| cmd.send(&p.map(|v| Val::Int(float_to_u32(v))), 0, 0))
    }

    fn get_max_power(&self, temp: Option<f64>) -> f64 {
        let model = self.thermal_model.borrow();
        let params = &model.params;
        let base = params.max_power.unwrap_or(0.0);
        if base == 0.0 {
            // If the model isn't loaded, just return 0 for now - the user won't
            // be allowed to heat regardless.
            return 0.0;
        }
        let Some(temp) = temp else { return base };
        // Correct for decrease in delivered power as the nozzle heats.
        let coeff = params.max_power_temp_coeff.unwrap_or(0.0);
        let adjusted = base + coeff * (temp - cal::MODEL_MAX_POWER_REF_TEMP);
        py_max(0.0, adjusted)
    }

    fn heater_config_valid(&self) -> bool {
        // Heating is allowed only once with fully populated coil and thermal models.
        self.coil_timings.get().is_some() && self.thermal_model.borrow().is_valid()
    }

    fn cur_target_temp(&self) -> Option<f64> {
        let time = self.printer.get_reactor().monotonic();
        self.heaters.borrow().iter().map(|h| h.get_temp(time).1).find(|t| *t != 0.0)
    }

    fn extruder_position(&self, time: f64) -> KResult<f64> {
        if self.toolhead.borrow().is_none() {
            *self.toolhead.borrow_mut() = self.printer.lookup_object_opt("toolhead").and_then(crate::printer::downcast_rc::<ToolHead>);
        }
        let Some(toolhead) = self.toolhead.borrow().clone() else { return Ok(0.0) };
        let extruder = toolhead.get_extruder();
        let eh = extruder.get_heater()?;
        let same = self.heater.borrow().as_ref().map(|h| Rc::ptr_eq(h, &eh)).unwrap_or(false);
        if !same {
            return Ok(0.0);
        }
        Ok(extruder.find_past_position(time))
    }

    fn fan_speed(&self, time: f64) -> f64 {
        let Some(fan) = &self.part_cooling_fan else { return 0.0 };
        let speed = match fan.get_status(time) {
            PyVal::Dict(d) => d.get("speed").and_then(|v| v.as_f64()).unwrap_or(0.0),
            _ => 0.0,
        };
        py_max(0.0, py_min(1.0, speed))
    }

    fn blend_ambient_temp(&self, sensor_temp: f64) -> f64 {
        // Weighted blend of the board/bracket thermistors and the IR sensor's
        // ambient reading, using the model's ambient_blend_* weights. This is
        // the same ambient the model is fed at runtime, reused by calibration.
        let model = self.thermal_model.borrow();
        let params = &model.params;
        // (python: get_last() or 25.0, so a 0.0 reading counts as missing)
        let last = |n: &str| self.aux_temp_sensor(n).get_last().filter(|v| *v != 0.0).unwrap_or(25.0);
        let components = [
            (last("board"), params.ambient_blend_board),
            (last("bracket"), params.ambient_blend_bracket),
            (sensor_temp, params.ambient_blend_sensor),
        ];
        let w_total = py_sum(components.iter().map(|(_, w)| *w));
        if w_total == 0.0 {
            return 25.0;
        }
        py_sum(components.iter().map(|(t, w)| t * w)) / w_total
    }

    fn handle_report_nozzle_temp(&self, params: &Params) -> KResult<()> {
        let clock = self.mcu.clock32_to_clock64(param(params, "clock"));
        let time = self.mcu.clock_to_print_time(clock as f64);
        let nozzle_temp = param(params, "nozzle") as f64 / 100.0 - KELVIN_OFFSET;
        let sensor_temp = param(params, "sensor") as f64 / 100.0 - KELVIN_OFFSET;
        let extruder_pos = self.extruder_position(time)?;

        let current_target_temp = self.cur_target_temp();
        let is_active = current_target_temp.is_some();

        let power = param(params, "power") as f64 / 1000000.0;
        if !is_active {
            self.thermal_model.borrow_mut().force_temp(nozzle_temp);
        }

        let charge = param(params, "charge");
        let (delta_time, delta_energy) = if let Some(last_report) = self.last_report.get() {
            let delta_time = time - last_report.0;
            let mut delta_charge = charge - last_report.1;
            if delta_charge < 0 {
                delta_charge += 1 << 32;
            }
            let delta_charge = delta_charge as f64 * self.power_scale.get();
            let delta_energy = delta_charge * self.supply_voltage.get();
            let input_power = delta_energy / delta_time;

            if let Some(f) = self.log_file.borrow_mut().as_mut() {
                let _ = f.write_all(
                    format!(
                        "{},{},{},{},{}\n",
                        py_float_repr(time),
                        py_float_repr(nozzle_temp),
                        py_float_repr(self.thermal_model.borrow().temperature),
                        py_float_repr(delta_energy),
                        py_float_repr(input_power)
                    )
                    .as_bytes(),
                );
            }

            if let Some(target) = current_target_temp {
                let is_cooling = nozzle_temp > (target + 5.0);
                let error = nozzle_temp - self.thermal_model.borrow().temperature;
                let mut force_adjust = py_max(0.0, error * self.thermal_model.borrow().params.error_application);
                if is_cooling || !self.thermal_model.borrow().is_valid() {
                    force_adjust = error;
                }

                if force_adjust != 0.0 {
                    let t = self.thermal_model.borrow().temperature;
                    self.thermal_model.borrow_mut().force_temp(t + force_adjust);
                }
                // When thermal model isn't valid, we won't run it. This is okay,
                // because the user isn't allowed to start heating when no thermal
                // model is set. We need this check to allow the tuner to work.
                if self.thermal_model.borrow().is_valid() {
                    let filament_distance = py_max(0.0, extruder_pos - last_report.3);
                    let fan_speed = self.fan_speed(time);

                    let ambient_temp = self.blend_ambient_temp(sensor_temp);
                    let filament_temp = ambient_temp; // TODO

                    let input_power = power * self.get_max_power(Some(nozzle_temp));
                    self.thermal_model.borrow_mut().step(delta_time, input_power, ambient_temp, filament_temp, filament_distance, fan_speed);
                    let model_temp = self.thermal_model.borrow().temperature;
                    let error = model_temp - nozzle_temp;
                    if error > self.max_model_error.get() {
                        self.printer.invoke_shutdown(
                            &format!(
                                "Modelled nozzle temperature {:.1} exceeds measured temperature of {:.1} by more than {:.1} deg C",
                                model_temp,
                                nozzle_temp,
                                self.max_model_error.get()
                            ),
                            None,
                        );
                    }
                }
            }
            (Some(delta_time), Some(delta_energy))
        } else {
            (None, None)
        };

        self.last_report.set(Some((time, charge, power, extruder_pos)));
        if nozzle_temp >= self.max_temp_nozzle {
            self.printer.invoke_shutdown(
                &format!("Maximum nozzle temperature of {} exceeded, saw {}", py_float_repr(self.max_temp_nozzle), py_float_repr(nozzle_temp)),
                None,
            );
        } else if sensor_temp >= self.max_temp_sensor {
            self.printer.invoke_shutdown(
                &format!("Maximum IR sensor temperature of {} exceeded, saw {}", py_float_repr(self.max_temp_sensor), py_float_repr(sensor_temp)),
                None,
            );
        }
        let reading = NozzleTemperature {
            time,
            nozzle_temperature: nozzle_temp,
            sensor_temperature: sensor_temp,
            delta_time,
            delta_energy,
            pwm_avg: power,
            delta_pwm_energy: power * self.get_max_power(Some(nozzle_temp)) * delta_time.unwrap_or(0.0),
        };
        let cbs = self.temperature_callbacks.borrow().clone();
        for cb in cbs {
            cb(&reading)?;
        }
        Ok(())
    }

    fn handle_vin_mon(&self, _read_time: f64, read_value: f64) {
        let vin = read_value * 3.3 * (4700.0 + 60400.0) / 4700.0;
        self.supply_voltage.set(vin);
    }

    fn set_target_temp(&self, target_temp: f64) -> KResult<()> {
        let target_temp = if target_temp == 0.0 { f64::NAN } else { target_temp };
        self.cmd(|c| c.set_target.as_ref(), |cmd| cmd.send(&[Val::Int(float_to_u32(target_temp + KELVIN_OFFSET))], 0, 0))
    }

    fn cmd_set_pid(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let p = *self.pid.borrow();
        let new = [
            gcmd.get_float("KP", Some(p[0]), Lim::new())?,
            gcmd.get_float("TI", Some(p[1]), Lim::new().min(0.0))?,
            gcmd.get_float("TD", Some(p[2]), Lim::new().min(0.0))?,
            gcmd.get_float("B", Some(p[3]), Lim::new().min(0.0))?,
            gcmd.get_float("I_WINDOW", Some(p[4]), Lim::new().above(0.0))?,
            gcmd.get_float("I_LIMIT", Some(p[5]), Lim::new().above(0.0).max(1.0))?,
        ];
        *self.pid.borrow_mut() = new;
        self.apply_pid_params()
    }

    fn cmd_set_cycle_limit(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cmd = self.mcu.lookup_command("indx_cycle_limit limit=%u", Some(self.cmd_queue.clone()))?;
        cmd.send(&[Val::Int(gcmd.get_int("LIMIT", None, Lim::new())?)], 0, 0)
    }

    fn cmd_set_ir_sensor_params(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cmd = self.mcu.lookup_command("indx_set_ir_sensor_params exponent=%u obj_gain=%u bracket_gain=%u", Some(self.cmd_queue.clone()))?;
        let exponent = gcmd.get_float("EXPONENT", None, Lim::new())?;
        let obj_gain = gcmd.get_float("OBJ_GAIN", None, Lim::new())?;
        let bracket_gain = gcmd.get_float("BRACKET_GAIN", None, Lim::new())?;
        cmd.send(&[Val::Int(float_to_u32(exponent)), Val::Int(float_to_u32(obj_gain)), Val::Int(float_to_u32(bracket_gain))], 0, 0)
    }

    fn cmd_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.cur_target_temp().is_some() {
            return Err(gcmd.error("Cannot calibrate while the heater is active. Turn the heater off first."));
        }
        {
            let mut m = self.thermal_model.borrow_mut();
            m.params.max_power = None;
            m.params.max_power_temp_coeff = None;
            m.params.thermal_capacity = None;
            m.params.to_ambient_r = None;
        }
        self.calibrate_coil_driver(gcmd)?;
        self.calibrate_thermal_model(gcmd)?;
        gcmd.respond_info("INDX calibration complete. Run SAVE_CONFIG to save the new parameters and restart.", true);
        Ok(())
    }

    fn calibrate_coil_driver(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mcu = &self.mcu;
        let capture = gcmd.get_opt("SAVE_WAVEFORM").is_some();

        // Tune at a consistent (cold) temperature: wait until the nozzle is
        // below the same MIN_TEMP used for thermal-model calibration, so the
        // coil timings are always measured at (almost) the same temperature.
        let min_temp = gcmd.get_float(
            "MIN_TEMP",
            Some(cal::MODEL_CAL_MIN_TEMP_DEFAULT),
            Lim::new().above(0.0).below(cal::MODEL_CAL_MIN_TEMP_CEILING),
        )?;
        self.wait_cooldown(gcmd, min_temp)?;

        gcmd.respond_info("INDX: tuning coil driver, please wait...", true);

        // Consume both tune streams so they don't log as unhandled; we poll the
        // latched status, so the notification is a no-op.
        let samples: Rc<RefCell<Vec<(i64, i64, i64)>>> = Rc::new(RefCell::new(vec![]));
        let reactor = self.printer.get_reactor();
        let finished = register_response(mcu, Rc::new(|_| Ok(())), "indx_coil_tune_finished status=%c error=%c");
        let s = samples.clone();
        let wave = register_response(
            mcu,
            Rc::new(move |p| {
                s.borrow_mut().push((param(p, "index"), param(p, "ns"), param(p, "mv")));
                Ok(())
            }),
            "indx_coil_tune_wave index=%u ns=%u mv=%u",
        );
        let res = (|| -> KResult<Params> {
            self.cmd(|c| c.tune_coil.as_ref(), |cmd| cmd.send(&[Val::Int(capture as i64)], 0, 0))?;
            let deadline = reactor.monotonic() + COIL_TUNE_TIMEOUT;
            loop {
                let result = {
                    let cmds = self.cmds.borrow();
                    let q = cmds.query_coil_tune.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
                    q.send(&[], 0, 0, true)?
                };
                if param(&result, "status") != COIL_TUNE_STATUS_RUNNING {
                    return Ok(result);
                }
                if reactor.monotonic() > deadline {
                    self.coil_timings.set(None);
                    return Err(gcmd.error("INDX coil driver tuning timed out"));
                }
                reactor.pause(reactor.monotonic() + COIL_TUNE_POLL_INTERVAL)?;
            }
        })();
        // finally:
        wave.unregister(mcu);
        finished.unregister(mcu);
        if capture {
            let samples = samples.borrow();
            let mut out = String::from("index,ns,mv\n");
            for (index, ns, mv) in samples.iter() {
                out += &format!("{},{},{}\n", index, ns, mv);
            }
            std::fs::write(COIL_TUNE_WAVE_PATH, out).map_err(|e| KError::Other(e.to_string()))?;
            gcmd.respond_info(&format!("INDX coil tune waveform ({} samples) saved to {}", samples.len(), COIL_TUNE_WAVE_PATH), true);
        }
        let result = res?;

        let status = param(&result, "status");
        if status != COIL_TUNE_STATUS_SUCCESS {
            // MCU no longer holds a valid model; re-gate host heating.
            self.coil_timings.set(None);
            let reason = if status == COIL_TUNE_STATUS_FAILED {
                let error = param(&result, "error");
                coil_tune_error(error).map(|s| s.to_string()).unwrap_or_else(|| error.to_string())
            } else {
                coil_tune_status(status).map(|s| s.to_string()).unwrap_or_else(|| status.to_string())
            };
            return Err(gcmd.error(format!("INDX coil driver tuning failed: {}", reason)));
        }

        // Cache in seconds (opens the heating gate for this session); ns -> s.
        let (on, off, on_first) = (param(&result, "on") as f64, param(&result, "off") as f64, param(&result, "on_first") as f64);
        self.coil_timings.set(Some((on * 1e-9, off * 1e-9, on_first * 1e-9)));
        self.save_coil_config()?;
        gcmd.respond_info(
            &format!("INDX coil driver tuned: TIME_ON={:.3} us TIME_OFF={:.3} us TIME_ON_FIRST={:.3} us", on / 1000.0, off / 1000.0, on_first / 1000.0),
            true,
        );
        Ok(())
    }

    fn configfile_set(&self, option: &str, value: &str) -> KResult<()> {
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        configfile.pconfig.borrow_mut().set(&self.toolboard_name, option, value)?;
        Ok(())
    }

    fn save_coil_config(&self) -> KResult<()> {
        let Some((t_on, t_off, t_on_first)) = self.coil_timings.get() else {
            return Err(KError::Other("cannot unpack non-iterable NoneType object".into()));
        };
        self.configfile_set("coil_time_on", &format!("{:.3}", t_on * 1e6))?;
        self.configfile_set("coil_time_off", &format!("{:.3}", t_off * 1e6))?;
        self.configfile_set("coil_time_on_first", &format!("{:.3}", t_on_first * 1e6))
    }

    fn toggle_steppers(&self, toolhead: &ToolHead, ets: &[Rc<EnableTracking>], enable: bool) -> KResult<()> {
        // Enable/disable a set of EnableTracking lines with proper toolhead
        // sync (mirrors stepper_enable's motor_debug_enable; portable both forks).
        if ets.is_empty() {
            return Ok(());
        }
        toolhead.dwell(cal::MODEL_CAL_STEPPER_DWELL)?;
        let print_time = toolhead.get_last_move_time()?;
        for et in ets {
            if enable {
                et.motor_enable(print_time)?;
            } else {
                et.motor_disable(print_time)?;
            }
        }
        toolhead.dwell(cal::MODEL_CAL_STEPPER_DWELL)
    }

    fn model_cal_wait_temp(
        &self,
        reactor: &Reactor,
        gcmd: &GCodeCommand,
        predicate: impl Fn(f64) -> bool,
        last_temp: &Cell<Option<f64>>,
        timeout: f64,
        timeout_msg: &str,
    ) -> KResult<()> {
        let deadline = reactor.monotonic() + timeout;
        loop {
            if let Some(t) = last_temp.get() {
                if predicate(t) {
                    return Ok(());
                }
            }
            if reactor.monotonic() > deadline {
                return Err(gcmd.error(timeout_msg));
            }
            reactor.pause(reactor.monotonic() + cal::MODEL_CAL_POLL_INTERVAL)?;
        }
    }

    fn heater_or_err(&self) -> KResult<Rc<Heater>> {
        self.heater.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'set_temp'".into()))
    }

    fn wait_cooldown(&self, gcmd: &GCodeCommand, min_temp: f64) -> KResult<()> {
        // Block until the nozzle has cooled below min_temp (the heater must
        // already be off) so the following operation runs at a consistent,
        // repeatable temperature.
        let reactor = self.printer.get_reactor();
        let last_temp: Rc<Cell<Option<f64>>> = Rc::new(Cell::new(None));
        let lt = last_temp.clone();
        let temp_cb: NozzleCallback = Rc::new(move |reading| {
            lt.set(Some(reading.nozzle_temperature));
            Ok(())
        });

        if let Some(heater) = self.heater.borrow().clone() {
            heater.set_temp(0.0)?;
        }
        self.add_temperature_callback(temp_cb.clone());
        let r = (|| -> KResult<()> {
            self.model_cal_wait_temp(&reactor, gcmd, |_| true, &last_temp, cal::MODEL_CAL_FIRST_READ_TIMEOUT, "INDX: no temperature reports received")?;
            if last_temp.get().unwrap_or(0.0) > min_temp {
                gcmd.respond_info(&format!("INDX: cooling below {:.0} C before tuning", min_temp), true);
                self.model_cal_wait_temp(&reactor, gcmd, |t| t < min_temp, &last_temp, cal::MODEL_CAL_PRECOOL_TIMEOUT, "INDX: timed out cooling to MIN_TEMP")?;
            }
            Ok(())
        })();
        self.remove_temperature_callback(&temp_cb);
        r
    }

    fn calibrate_thermal_model(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let Some(heater) = self.heater.borrow().clone() else {
            return Err(gcmd.error("INDX heater is not bound to an extruder/heater"));
        };
        if self.coil_timings.get().is_none() {
            return Err(gcmd.error("INDX heater has not been calibrated yet. Run INDX_CALIBRATE to perform calibration."));
        }
        if self.cur_target_temp().is_some() {
            return Err(gcmd.error("Cannot calibrate while the heater is active. Turn the heater off first."));
        }

        let min_temp = gcmd.get_float(
            "MIN_TEMP",
            Some(cal::MODEL_CAL_MIN_TEMP_DEFAULT),
            Lim::new().above(0.0).below(cal::MODEL_CAL_MIN_TEMP_CEILING),
        )?;
        let heater_max = heater.max_temp;
        let max_temp = gcmd.get_float("MAX_TEMP", Some(cal::MODEL_CAL_MAX_TEMP_DEFAULT), Lim::new().min(min_temp + 1.0).max(heater_max))?;
        let hold_time = gcmd.get_float("HOLD_TIME", Some(cal::MODEL_CAL_HOLD_TIME_DEFAULT), Lim::new().min(0.0))?;
        let cooldown_time = gcmd.get_float(
            "COOLDOWN_TIME",
            Some(cal::MODEL_CAL_COOLDOWN_TIME_DEFAULT),
            Lim::new().min(0.0).max(cal::MODEL_CAL_COOLDOWN_TIME_MAX),
        )?;

        let printer = &self.printer;
        let reactor = printer.get_reactor();
        let toolhead = printer.lookup::<ToolHead>("toolhead")?;

        let samples: Rc<RefCell<Vec<CalSample>>> = Rc::new(RefCell::new(vec![]));
        let phase: Rc<Cell<&'static str>> = Rc::new(Cell::new("precool"));
        let last_temp: Rc<Cell<Option<f64>>> = Rc::new(Cell::new(None));

        let (s, ph, lt, w) = (samples.clone(), phase.clone(), last_temp.clone(), self.me.clone());
        let sampler: NozzleCallback = Rc::new(move |reading| {
            lt.set(Some(reading.nozzle_temperature));
            let (Some(dt), Some(de)) = (reading.delta_time, reading.delta_energy) else { return Ok(()) };
            if dt <= 0.0 {
                return Ok(());
            }
            let ambient = w.upgrade().map(|h| h.blend_ambient_temp(reading.sensor_temperature)).unwrap_or(25.0);
            s.borrow_mut().push(CalSample { temp: reading.nozzle_temperature, power: de / dt, pwm: reading.pwm_avg, dt, ambient, phase: ph.get() });
            Ok(())
        });

        let old_max_model_error = self.max_model_error.get();
        let mut reenable: Vec<Rc<EnableTracking>> = vec![];
        let installed = Cell::new(false);
        let r = (|| -> KResult<(cal::ThermalFit, cal::MaxPowerFit)> {
            // Don't let the un-calibrated production model trip the
            // model-vs-measured divergence guard during the heatup. The hard
            // over-temperature interlocks stay active.
            self.max_model_error.set(f64::INFINITY);

            // Heater off and disable the toolboard steppers (extruder motor) so
            // they can't perturb the power/baseline measurement. Reuse the
            // enable lines the heatsink fan already collected for this MCU.
            heater.set_temp(0.0)?;
            reenable = self.fan.steppers.borrow().iter().filter(|et| et.is_motor_enabled()).cloned().collect();
            self.toggle_steppers(&toolhead, &reenable, false)?;

            self.add_temperature_callback(sampler.clone());
            installed.set(true);

            self.model_cal_wait_temp(&reactor, gcmd, |_| true, &last_temp, cal::MODEL_CAL_FIRST_READ_TIMEOUT, "INDX: no temperature reports received")?;
            if last_temp.get().unwrap_or(0.0) > min_temp {
                gcmd.respond_info(&format!("INDX: cooling below {:.0} C before calibration", min_temp), true);
                self.model_cal_wait_temp(&reactor, gcmd, |t| t < min_temp, &last_temp, cal::MODEL_CAL_PRECOOL_TIMEOUT, "INDX: timed out cooling to MIN_TEMP")?;
            }

            // Baseline (heater off) draw to subtract from measured power.
            phase.set("baseline");
            gcmd.respond_info("INDX: measuring baseline power", true);
            reactor.pause(reactor.monotonic() + cal::MODEL_CAL_BASELINE_TIME)?;
            let Some(baseline_power) = cal::mean_phase_power(&samples.borrow(), "baseline") else {
                return Err(gcmd.error("INDX: no baseline power samples received"));
            };

            // Heatup to MAX_TEMP.
            phase.set("heat");
            gcmd.respond_info(&format!("INDX: heating to {:.0} C (baseline {:.2} W)", max_temp, baseline_power), true);
            heater.raw_set_temp(max_temp)?;
            self.model_cal_wait_temp(&reactor, gcmd, |t| t >= max_temp, &last_temp, cal::MODEL_CAL_HEATUP_TIMEOUT, "INDX: timed out heating to MAX_TEMP")?;

            // Hold at temperature (strong constraint on to_ambient_r).
            if hold_time > 0.0 {
                phase.set("hold");
                gcmd.respond_info(&format!("INDX: holding for {:.0} s", hold_time), true);
                reactor.pause(reactor.monotonic() + hold_time)?;
            }

            // Cooldown (constrains R*C).
            heater.set_temp(0.0)?;
            if cooldown_time > 0.0 {
                phase.set("cooldown");
                gcmd.respond_info(&format!("INDX: measuring cooldown for {:.0} s", cooldown_time), true);
                reactor.pause(reactor.monotonic() + cooldown_time)?;
            }

            self.remove_temperature_callback(&sampler);
            installed.set(false);
            heater.set_temp(0.0)?;

            let data = samples.borrow().clone();
            gcmd.respond_info(&format!("INDX: collected {} samples, fitting model...", data.len()), true);
            let res = cal::run_fit_in_background(printer, move || cal::thermal_model_fit(&data, baseline_power))?;
            res.map_err(|e| gcmd.error(format!("INDX thermal model fit failed: {}", e)))
        })();
        // finally:
        if installed.get() {
            self.remove_temperature_callback(&sampler);
        }
        let fin = heater.set_temp(0.0);
        self.max_model_error.set(old_max_model_error);
        let fin2 = self.toggle_steppers(&toolhead, &reenable, true);
        let result = r?;
        fin?;
        fin2?;
        self.apply_model_calibration(gcmd, &result)
    }

    fn apply_model_calibration(&self, gcmd: &GCodeCommand, result: &(cal::ThermalFit, cal::MaxPowerFit)) -> KResult<()> {
        let (thermal, mp) = result;
        let max_power = mp.max_power;
        let coeff = mp.max_power_temp_coeff;
        let reference = mp.max_power_ref_temp;
        let capacity = thermal.thermal_capacity;
        let to_ambient_r = thermal.to_ambient_r;

        {
            let mut m = self.thermal_model.borrow_mut();
            m.params.max_power = Some(max_power);
            m.params.max_power_temp_coeff = Some(coeff);
            m.params.thermal_capacity = Some(capacity);
            m.params.to_ambient_r = Some(to_ambient_r);
        }
        self.configfile_set("max_power", &format!("{:.3}", max_power))?;
        self.configfile_set("model_max_power_temp_coeff", &format!("{:.5}", coeff))?;
        self.configfile_set("model_thermal_capacity", &format!("{:.4}", capacity))?;
        self.configfile_set("model_to_ambient_r", &format!("{:.4}", to_ambient_r))?;

        let fmt = |val: f64, ci: Option<f64>, unit: &str, prec: usize| match ci {
            None => format!("{:.*} {}", prec, val, unit),
            Some(ci) => format!("{:.*} +/- {:.*} {} (95%)", prec, val, prec, ci, unit),
        };

        let mut lines = vec![];
        let mp_se = mp.max_power_se.filter(|v| *v != 0.0);
        lines.push(format!("max_power: {}", fmt(max_power, mp_se.map(|se| 1.96 * se), &format!("W (at {:.0} C)", reference), 2)));
        let c_se = mp.max_power_temp_coeff_se.filter(|v| *v != 0.0);
        lines.push(format!("max_power_temp_coeff: {}", fmt(coeff, c_se.map(|se| 1.96 * se), "W/K", 4)));
        // Illustrate the power change across the measured heatup range.
        if let (Some(t_min), Some(t_max)) = (mp.fit_t_min, mp.fit_t_max) {
            let p_lo = max_power + coeff * (t_min - reference);
            let p_hi = max_power + coeff * (t_max - reference);
            let drop_pct = if p_lo != 0.0 { 100.0 * (p_lo - p_hi) / p_lo } else { 0.0 };
            lines.push(format!("  power {:.1} W @ {:.0} C -> {:.1} W @ {:.0} C ({:+.1}%)", p_lo, t_min, p_hi, t_max, -drop_pct));
        }
        lines.push(format!("thermal_capacity: {}", fmt(capacity, thermal.thermal_capacity_ci, "J/K", 4)));
        lines.push(format!("to_ambient_r: {}", fmt(to_ambient_r, thermal.to_ambient_r_ci, "K/W", 4)));
        lines.push(format!("fit RMS error: {:.2} C over {} samples", thermal.rms, thermal.n_fit));
        gcmd.respond_info(&format!("INDX thermal model calibrated:\n{}", lines.join("\n")), true);
        Ok(())
    }

    fn cmd_fan_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let heater = match self.heater.borrow().clone() {
            Some(h) if self.heater_config_valid() => h,
            _ => return Err(gcmd.error("INDX thermal model is not calibrated")),
        };
        let Some(fan) = self.part_cooling_fan.clone() else {
            return Err(gcmd.error("INDX part cooling fan is not configured"));
        };

        let breaks = gcmd.get_int("BREAKS", Some(cal::FAN_CAL_BREAKS_DEFAULT), Lim::new().min(3.0))?;
        let hold_time = gcmd.get_float("HOLD_TIME", Some(cal::FAN_CAL_HOLD_TIME_DEFAULT), Lim::new().above(0.0))?;
        let min_speed = gcmd.get_float("MIN_SPEED", Some(cal::FAN_CAL_MIN_SPEED_DEFAULT), Lim::new().above(0.0).below(1.0))?;
        let heater_max = heater.max_temp;
        let target = gcmd.get_float("TEMP", Some(cal::FAN_CAL_TEMP_DEFAULT), Lim::new().above(0.0).max(heater_max))?;
        let settle_time = gcmd.get_float("SETTLE_TIME", Some(cal::FAN_CAL_SETTLE_TIME_DEFAULT), Lim::new().min(0.0))?;
        let speeds = cal::fan_calibration_breakpoints(breaks, min_speed)?;
        let printer = &self.printer;
        let reactor = printer.get_reactor();
        let last_temp: Rc<Cell<Option<f64>>> = Rc::new(Cell::new(None));
        let active_samples: Rc<RefCell<Option<Vec<FanSample>>>> = Rc::new(RefCell::new(None));

        let (lt, act, w) = (last_temp.clone(), active_samples.clone(), self.me.clone());
        let sampler: NozzleCallback = Rc::new(move |reading| {
            lt.set(Some(reading.nozzle_temperature));
            let Some(dt) = reading.delta_time else { return Ok(()) };
            if act.borrow().is_none() || dt <= 0.0 {
                return Ok(());
            }
            let ambient = w.upgrade().map(|h| h.blend_ambient_temp(reading.sensor_temperature)).unwrap_or(25.0);
            if let Some(v) = act.borrow_mut().as_mut() {
                v.push((reading.nozzle_temperature, ambient, dt, reading.delta_pwm_energy));
            }
            Ok(())
        });

        let old_max_model_error = self.max_model_error.get();
        let mut samples: Vec<(f64, Vec<FanSample>)> = vec![];
        let installed = Cell::new(false);
        let r = (|| -> KResult<()> {
            self.max_model_error.set(f64::INFINITY);
            fan.set_speed(0.0, None)?;
            self.add_temperature_callback(sampler.clone());
            installed.set(true);
            heater.raw_set_temp(target)?;
            gcmd.respond_info(&format!("INDX: waiting for {:.0} C for fan calibration", target), true);
            self.model_cal_wait_temp(
                &reactor,
                gcmd,
                |t| (t - target).abs() <= 1.0,
                &last_temp,
                cal::FAN_CAL_HEATUP_TIMEOUT,
                "INDX: timed out reaching fan calibration temperature",
            )?;
            if settle_time != 0.0 {
                gcmd.respond_info(&format!("INDX: settling for {:.0} s", settle_time), true);
                reactor.pause(reactor.monotonic() + settle_time)?;
            }

            for &speed in &speeds {
                *active_samples.borrow_mut() = Some(vec![]);
                fan.set_speed(speed, None)?;
                gcmd.respond_info(&format!("INDX: fan {:.0}% for {:.1} s", speed * 100.0, hold_time), true);
                reactor.pause(reactor.monotonic() + hold_time)?;
                let phase_samples = active_samples.borrow_mut().take().unwrap_or_default();
                samples.push((speed, phase_samples));
            }
            Ok(())
        })();
        // finally:
        *active_samples.borrow_mut() = None;
        if installed.get() {
            self.remove_temperature_callback(&sampler);
        }
        let fin = fan.set_speed(0.0, None).and_then(|_| heater.raw_set_temp(0.0));
        self.max_model_error.set(old_max_model_error);
        r?;
        fin?;

        let (thermal_capacity, to_ambient_r) = {
            let m = self.thermal_model.borrow();
            (m.params.thermal_capacity.unwrap_or(f64::NAN), m.params.to_ambient_r.unwrap_or(f64::NAN))
        };
        let result = cal::fit_fan_model(&samples, thermal_capacity, to_ambient_r).map_err(|e| gcmd.error(format!("INDX fan model fit failed: {}", e)))?;

        let a = result.part_cooling_fan_a;
        let k = result.part_cooling_fan_k;
        {
            let mut m = self.thermal_model.borrow_mut();
            m.params.part_cooling_fan_a = a;
            m.params.part_cooling_fan_k = k;
        }
        self.configfile_set("model_part_cooling_fan_a", &format!("{:.5}", a))?;
        self.configfile_set("model_part_cooling_fan_k", &format!("{:.5}", k))?;

        let fit_value = |value: f64, ci: Option<f64>| format!("{:.5}{}", value, ci.map(|ci| format!(" +/- {:.5} (95%)", ci)).unwrap_or_default());

        let point_lines: Vec<String> = result
            .points
            .iter()
            .map(|(speed, power, _, _, duration)| format!("  {:.0}%: {:.2} W over {:.1} s", speed * 100.0, power, duration))
            .collect();
        gcmd.respond_info(
            &format!(
                "INDX fan model calibrated:\npart_cooling_fan_a: {} K/W\npart_cooling_fan_k: {}\nfit RMS power error: {:.2} W{}\n{}\nRun SAVE_CONFIG to save the new fan parameters.",
                fit_value(a, result.part_cooling_fan_a_ci),
                fit_value(k, result.part_cooling_fan_k_ci),
                result.rms_power,
                result.r_squared.map(|r| format!("; R^2: {:.4}", r)).unwrap_or_default(),
                point_lines.join("\n")
            ),
            true,
        );
        Ok(())
    }

    fn cmd_measure_power(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let duration = gcmd.get_float("DURATION", Some(5.0), Lim::new().above(0.0))?;
        gcmd.respond_info(&format!("Measuring power for {:.1} seconds", duration), true);

        let reactor = self.printer.get_reactor();

        let totals: Rc<Cell<[f64; 3]>> = Rc::new(Cell::new([0.0; 3]));
        let t = totals.clone();
        let cb: NozzleCallback = Rc::new(move |reading| {
            let mut v = t.get();
            v[0] += reading.delta_time.unwrap_or(0.0);
            v[1] += reading.delta_energy.unwrap_or(0.0);
            v[2] += reading.delta_pwm_energy;
            t.set(v);
            Ok(())
        });

        self.add_temperature_callback(cb.clone());
        let begin_time = reactor.monotonic();
        let r = reactor.pause(begin_time + duration);
        self.remove_temperature_callback(&cb);
        r?;

        let totals = totals.get();
        if totals[0] == 0.0 {
            return Err(gcmd.error("No power meansurements received over entire duration"));
        }

        let power = totals[1] / totals[0];
        gcmd.respond_info(
            &format!(
                "Measured power for {:.1} seconds.\nTotal energy {:.1} J.\nAverage power {:.2} W.\nPWM modelled: {:.1} J / {:.2} W",
                totals[0],
                totals[1],
                power,
                totals[2],
                totals[2] / totals[0]
            ),
            true,
        );
        Ok(())
    }

    /// the active extruder as a PrinterExtruder using this heater
    fn indx_extruder(&self, gcmd: &GCodeCommand, toolhead: &ToolHead) -> KResult<Rc<PrinterExtruder>> {
        let extruder = toolhead.get_extruder();
        let eh = extruder.get_heater().ok();
        let heater = self.heater.borrow().clone();
        let same = match (&heater, &eh) {
            (Some(h), Some(eh)) => Rc::ptr_eq(h, eh),
            _ => false,
        };
        if !same {
            return Err(gcmd.error("Active extruder is not using the INDX heater"));
        }
        let obj = self.printer.lookup_object(&extruder.get_name())?;
        crate::printer::downcast_rc::<PrinterExtruder>(obj).ok_or_else(|| KError::Other("extruder has an unexpected type".into()))
    }

    fn cmd_load_filament(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if !self.thermal_model.borrow().is_valid() {
            return Err(gcmd.error("INDX thermal model is not calibrated"));
        }

        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let extruder = self.indx_extruder(gcmd, &toolhead)?;
        let heater = self.heater_or_err()?;
        if !heater.can_extrude.get() {
            return Err(gcmd.error("INDX_LOAD_FILAMENT requires the active extruder to be above min_extrude_temp"));
        }
        let target_temp = self.cur_target_temp();
        if target_temp.map(|t| t < heater.min_extrude_temp.get()).unwrap_or(true) {
            return Err(gcmd.error("INDX_LOAD_FILAMENT requires the active extruder target temperature to be set to at least min_extrude_temp"));
        }

        let speed = gcmd.get_float("SPEED", Some(FILAMENT_LOAD_SPEED), Lim::new().above(0.0))?;
        let max_length = gcmd.get_float("MAX_LENGTH", Some(FILAMENT_LOAD_MAX_LENGTH), Lim::new().above(0.0))?;
        let prime_time = gcmd.get_float("PRIME_TIME", Some(FILAMENT_LOAD_PRIME_TIME), Lim::new().above(0.0))?;
        let prime_length = gcmd.get_float_opt("PRIME_LENGTH", Lim::new().above(0.0))?.unwrap_or(speed * prime_time);
        let threshold = gcmd.get_float("THRESHOLD", Some(FILAMENT_LOAD_THRESHOLD), Lim::new().above(0.0))?;
        let segment_time = gcmd.get_float("SEGMENT_TIME", Some(FILAMENT_LOAD_SEGMENT_TIME), Lim::new().above(0.0))?;
        let apply_result = gcmd.get_int("APPLY", Some(1), Lim::new().min(0.0).max(1.0))?;

        let params = self.thermal_model.borrow().params.clone();
        let filament_area = params.filament_radius * params.filament_radius * PI;

        #[derive(Default)]
        struct LoadState {
            last_temp: Option<f64>,
            last_pos: f64,
            start_pos: Option<f64>,
            loaded_at_pos: f64,
            load_energy: f64,
            loaded_energy: f64,
            loaded_denominator: f64,
            loaded: bool,
            stop_requested: bool,
        }
        let st: Rc<RefCell<LoadState>> = Rc::new(RefCell::new(LoadState::default()));
        let reactor = self.printer.get_reactor();

        let (s, w, p) = (st.clone(), self.me.clone(), params.clone());
        let cb: NozzleCallback = Rc::new(move |reading| {
            let Some(h) = w.upgrade() else { return Ok(()) };
            let Some(dt) = reading.delta_time.filter(|dt| *dt > 0.0) else { return Ok(()) };
            let pos = h.extruder_position(reading.time)?;
            let mut s = s.borrow_mut();
            let Some(last_temp) = s.last_temp else {
                s.last_temp = Some(reading.nozzle_temperature);
                s.last_pos = pos;
                s.start_pos = Some(pos);
                return Ok(());
            };
            let ambient = h.blend_ambient_temp(reading.sensor_temperature);
            let loss_ambient = (reading.nozzle_temperature - ambient) / p.to_ambient_r.unwrap_or(f64::NAN);
            let mut loss_part_cooling = 0.0;
            let fan_speed = h.fan_speed(reading.time);
            if fan_speed > 0.0 {
                let pcf_ambient_r = p.part_cooling_fan_a * fan_speed.powf(-p.part_cooling_fan_k);
                if pcf_ambient_r > 0.0 {
                    loss_part_cooling = py_max(0.0, (reading.nozzle_temperature - ambient) / pcf_ambient_r);
                }
            }
            let stored = p.thermal_capacity.unwrap_or(f64::NAN) * (reading.nozzle_temperature - last_temp);
            let loss = (loss_ambient + loss_part_cooling) * dt;
            let residual = reading.delta_pwm_energy - loss - stored;
            s.load_energy = py_max(0.0, s.load_energy + residual);
            let pos_delta = py_max(0.0, pos - s.last_pos);
            let temp_delta = py_max(0.0, reading.nozzle_temperature - ambient);
            if s.loaded {
                s.loaded_energy += py_max(0.0, residual);
                s.loaded_denominator += pos_delta * filament_area / 1000.0 * temp_delta;
            } else if s.load_energy >= threshold {
                s.loaded = true;
                s.loaded_at_pos = pos;
                s.last_pos = pos;
            }
            s.last_temp = Some(reading.nozzle_temperature);
            s.last_pos = pos;
            if !s.stop_requested {
                let start_pos = s.start_pos.unwrap_or(0.0);
                if (!s.loaded && pos - start_pos >= max_length) || (s.loaded && pos - s.loaded_at_pos >= prime_length) {
                    s.stop_requested = true;
                }
            }
            Ok(())
        });

        gcmd.respond_info("INDX: loading filament", true);
        let accel_dist = speed * speed / extruder.max_e_accel;
        let move_len = py_max(speed * segment_time, accel_dist * 1.1);
        let move_len = py_min(move_len, extruder.max_e_dist);
        if move_len <= 0.0 {
            return Err(gcmd.error("INDX: invalid extruder segment length"));
        }
        let mut queued = 0.0;
        self.add_temperature_callback(cb.clone());
        let r = (|| -> KResult<()> {
            while !st.borrow().stop_requested && queued < max_length + prime_length {
                let mut pos = toolhead.get_position();
                let segment = py_min(move_len, max_length + prime_length - queued);
                pos[3] += segment;
                let coord: Vec<Option<f64>> = pos.iter().map(|v| Some(*v)).collect();
                toolhead.manual_move(&coord, speed)?;
                queued += segment;
                reactor.pause(reactor.monotonic() + 0.01)?;
            }
            toolhead.wait_moves()
        })();
        self.remove_temperature_callback(&cb);
        r?;

        let s = st.borrow();
        let moved = match s.start_pos {
            None => 0.0,
            Some(sp) => s.last_pos - sp,
        };
        if !s.loaded {
            return Err(gcmd.error(format!("INDX: filament load was not detected after {:.1} mm ({:.1} J load energy)", moved, s.load_energy)));
        }
        if s.loaded_denominator <= 0.0 {
            return Err(gcmd.error("INDX: insufficient prime data for heat capacity"));
        }

        let heat_capacity = s.loaded_energy / s.loaded_denominator;
        if apply_result != 0 {
            {
                let mut m = self.thermal_model.borrow_mut();
                m.params = ThermalModelParams { filament_density: 1.0, filament_heat_capacity: heat_capacity, ..params };
            }
            self.configfile_set("model_filament_density", "1.0000")?;
            self.configfile_set("model_filament_heat_capacity", &format!("{:.4}", heat_capacity))?;
        }
        gcmd.respond_info(
            &format!(
                "INDX filament loaded after {:.1} mm; primed {:.1} mm.\nfilament_density: 1.0000 g/cm^3\nfilament_heat_capacity: {:.4} J/g/K\n{}",
                s.loaded_at_pos - s.start_pos.unwrap_or(0.0),
                s.last_pos - s.loaded_at_pos,
                heat_capacity,
                if apply_result != 0 {
                    "Run SAVE_CONFIG to save the new filament parameters."
                } else {
                    "Use APPLY=1 to apply the measured filament parameters."
                }
            ),
            true,
        );
        Ok(())
    }

    fn cmd_clear_filament(&self, gcmd: &GCodeCommand) -> KResult<()> {
        {
            let mut m = self.thermal_model.borrow_mut();
            m.params.filament_density = 0.0;
            m.params.filament_heat_capacity = 0.0;
        }
        self.configfile_set("model_filament_density", "0.0000")?;
        self.configfile_set("model_filament_heat_capacity", "0.0000")?;
        gcmd.respond_info("INDX filament model cleared. Run SAVE_CONFIG to save.", true);
        Ok(())
    }

    fn cmd_extruder_move(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let distance = gcmd.get_float("DISTANCE", None, Lim::new())?;
        let speed = gcmd.get_float("SPEED", None, Lim::new().above(0.0))?;
        let current = gcmd.get_float("CURRENT", None, Lim::new().above(0.0))?;

        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let extruder = self.indx_extruder(gcmd, &toolhead)?;
        let Some(es) = extruder.extruder_stepper.clone() else {
            return Err(gcmd.error("Active extruder does not have a stepper"));
        };
        let stepper = es.stepper.clone();
        let current_helper = stepper
            .get_tmc_current_helper()
            .and_then(|h| h.downcast_ref::<StepperCurrentHelper>().map(|s| s.0.clone()));
        let Some(current_helper) = current_helper else {
            return Err(gcmd.error("Active extruder does not have a TMC current helper"));
        };

        let (run_current, _hold_current, req_hold_current, _, _) = current_helper.get_current();
        let current = py_min(current, run_current);
        let restore_hold_current = req_hold_current;

        toolhead.wait_moves()?;
        let print_time = toolhead.get_last_move_time()?;
        current_helper.set_current(current, restore_hold_current, Some(print_time), false)?;
        let heater = self.heater_or_err()?;
        let r = (|| -> KResult<()> {
            let mut pos = toolhead.get_position();
            pos[3] += distance;
            let can_extrude = heater.can_extrude.get();
            heater.can_extrude.set(true);
            let coord: Vec<Option<f64>> = pos.iter().map(|v| Some(*v)).collect();
            let m = toolhead.manual_move(&coord, speed);
            heater.can_extrude.set(can_extrude);
            m?;
            toolhead.wait_moves()
        })();
        // finally:
        let print_time = toolhead.get_last_move_time()?;
        current_helper.set_current(run_current, restore_hold_current, Some(print_time), false)?;
        r
    }

    fn cmd_set_model_params(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cur = self.thermal_model.borrow().params.clone();

        let new_model = ThermalModelParams {
            max_power: get_float_or(gcmd, "MAX_POWER", cur.max_power, Lim::new().min(0.0))?,
            max_power_temp_coeff: get_float_or(gcmd, "MAX_POWER_TEMP_COEFF", cur.max_power_temp_coeff, Lim::new())?,
            thermal_capacity: get_float_or(gcmd, "THERMAL_CAPACITY", cur.thermal_capacity, Lim::new().above(0.0))?,
            to_ambient_r: get_float_or(gcmd, "TO_AMBIENT_R", cur.to_ambient_r, Lim::new().above(0.0))?,
            filament_radius: gcmd.get_float("FILAMENT_DIAMETER", Some(cur.filament_radius * 2.0), Lim::new().min(0.0))? / 2.0,
            filament_density: gcmd.get_float("FILAMENT_DENSITY", Some(cur.filament_density), Lim::new().min(0.0))?,
            filament_heat_capacity: gcmd.get_float("FILAMENT_HEAT_CAPACITY", Some(cur.filament_heat_capacity), Lim::new().min(0.0))?,
            part_cooling_fan_a: gcmd.get_float("PART_COOLING_FAN_A", Some(cur.part_cooling_fan_a), Lim::new())?,
            part_cooling_fan_k: gcmd.get_float("PART_COOLING_FAN_K", Some(cur.part_cooling_fan_k), Lim::new())?,
            ambient_blend_board: gcmd.get_float("AMBIENT_BLEND_BOARD", Some(cur.ambient_blend_board), Lim::new())?,
            ambient_blend_bracket: gcmd.get_float("AMBIENT_BLEND_BRACKET", Some(cur.ambient_blend_bracket), Lim::new())?,
            ambient_blend_sensor: gcmd.get_float("AMBIENT_BLEND_SENSOR", Some(cur.ambient_blend_sensor), Lim::new())?,
            error_application: gcmd.get_float("ERROR_APPLICATION", Some(cur.error_application), Lim::new())?,
        };
        let params = new_model.describe();
        self.thermal_model.borrow_mut().params = new_model;
        gcmd.respond_info(&format!("New model parameters:\n{}", params), true);
        Ok(())
    }

    fn cmd_dump_model_update(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.cur_target_temp().is_none() {
            return Err(gcmd.error("Heating is not currently active"));
        }
        match self.thermal_model.borrow().last_step.as_ref() {
            Some(state) => gcmd.respond_info(&format!("Last model update:\n{}", state.describe()), true),
            None => gcmd.respond_info("Thermal model has not yet run", true),
        }
        Ok(())
    }

    fn cmd_log(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        if self.log_file.borrow_mut().take().is_some() {
            return Ok(());
        }
        let mut f = std::fs::File::create("/tmp/indx_log.csv").map_err(|e| KError::Other(e.to_string()))?;
        f.write_all(b"time,temp,model,delta_energy,power\n").map_err(|e| KError::Other(e.to_string()))?;
        *self.log_file.borrow_mut() = Some(f);
        Ok(())
    }

    fn cmd_debug_ir_sensor_eeprom(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cmd = self.mcu.lookup_query_command(
            "indx_debug_ir_sensor_eeprom",
            "indx_debug_ir_sensor_eeprom_data data=%*s",
            None,
            Some(self.cmd_queue.clone()),
            false,
        )?;
        let res = cmd.send(&[], 0, 0, true)?;
        println!("{}", crate::msgproto::params_repr(&res));
        let data: &[u8] = match res.get("data") {
            Some(Val::Bytes(b)) => b,
            _ => &[],
        };
        // hex(c)[2:].ljust(2, "0")
        gcmd.respond_info(&data.iter().map(|c| format!("{:0<2}", format!("{:x}", c))).collect::<String>(), true);
        Ok(())
    }

    fn cmd_debug_stream_raw_ir_sensor(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.raw_ir_log_file.borrow().is_some() {
            let r = self.cmd(|c| c.debug_stream_raw_ir.as_ref(), |cmd| cmd.send(&[Val::Int(0)], 0, 0));
            *self.raw_ir_log_file.borrow_mut() = None;
            self.raw_ir_log_sensors.borrow_mut().clear();
            r?;
            gcmd.respond_info("INDX raw IR streaming stopped", true);
            return Ok(());
        }

        let reactor = self.printer.get_reactor();
        let eventtime = reactor.monotonic();
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let sensors: Vec<String> = match crate::printer::PrinterObject::get_status(&*pheaters, eventtime) {
            Some(PyVal::Dict(d)) => match d.get("available_sensors") {
                Some(PyVal::List(v)) | Some(PyVal::Tuple(v)) => v.iter().map(|s| s.str()).collect(),
                _ => vec![],
            },
            _ => vec![],
        };
        println!("{}", PyVal::List(sensors.iter().map(|s| PyVal::Str(s.clone())).collect()).repr());
        *self.raw_ir_log_sensors.borrow_mut() = sensors.clone();
        let path = format!("/tmp/indx_raw_ir_{}.csv", strftime_local("%Y%m%d_%H%M%S"));
        let mut f = std::fs::File::create(&path).map_err(|e| KError::Other(e.to_string()))?;
        let names: Vec<String> =
            sensors.iter().map(|n| n.strip_prefix("temperature_sensor ").map(|s| s.to_string()).unwrap_or_else(|| n.clone())).collect();
        let mut header = vec!["time".to_string(), "raw_object".into(), "raw_ambient".into()];
        header.extend(names);
        f.write_all(format!("{}\n", header.join(",")).as_bytes()).map_err(|e| KError::Other(e.to_string()))?;
        *self.raw_ir_log_file.borrow_mut() = Some(f);
        self.cmd(|c| c.debug_stream_raw_ir.as_ref(), |cmd| cmd.send(&[Val::Int(1)], 0, 0))?;
        gcmd.respond_info(&format!("INDX raw IR streaming to {}", path), true);
        Ok(())
    }

    fn handle_debug_raw_ir(&self, params: &Params) -> KResult<()> {
        if self.raw_ir_log_file.borrow().is_none() {
            return Ok(());
        }
        let eventtime = self.printer.get_reactor().monotonic();
        let mut cols = vec![format!("{:.6}", eventtime), param(params, "object").to_string(), param(params, "ambient").to_string()];
        for name in self.raw_ir_log_sensors.borrow().iter() {
            let t = self.printer.lookup_object(name).ok().and_then(|o| match o.get_status(eventtime) {
                Some(PyVal::Dict(d)) => d.get("temperature").and_then(|v| v.as_f64()),
                _ => None,
            });
            cols.push(t.map(|t| format!("{:.2}", t)).unwrap_or_default());
        }
        if let Some(f) = self.raw_ir_log_file.borrow_mut().as_mut() {
            if let Err(e) = f.write_all(format!("{}\n", cols.join(",")).as_bytes()) {
                logging::error(&format!("Write failed: {}", e));
            }
        }
        Ok(())
    }
}

// PWM pin interface
impl PwmPin for IndxToolboardHeater {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, _max_duration: f64) {
        // No-op for INDX, controlled by hardware
    }
    fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: bool) -> KResult<()> {
        // No-op for INDX, controlled by hardware
        Ok(())
    }
    fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}
    fn set_pwm(&self, _print_time: f64, _value: f64) -> KResult<()> {
        Ok(())
    }
}

pub struct HeaterControlIndx {
    heater: Weak<Heater>,
    indx_heater: Weak<IndxToolboardHeater>,
    profile: RefCell<Option<Profile>>,
}

impl Control for HeaterControlIndx {
    fn temperature_update(&self, read_time: f64, _temp: f64, target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        no_secondary("HeaterControlIndx", secondary_temp)?;
        let (Some(indx), Some(heater)) = (self.indx_heater.upgrade(), self.heater.upgrade()) else { return Ok(()) };
        indx.set_target_temp(target_temp)?;
        let power = indx.last_report.get().map(|r| r.2).ok_or_else(|| KError::Other("'NoneType' object is not subscriptable".into()))?;
        heater.set_pwm(read_time, power)
    }

    fn check_busy(&self, _eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        // TODO: Improve this
        smoothed_temp < target_temp - 1.0
    }

    fn get_profile(&self) -> Profile {
        self.profile.borrow().clone().unwrap_or_default()
    }

    // ponytail: python's HeaterControlIndx has no get_type
    fn get_type(&self) -> &'static str {
        "indx"
    }

}
