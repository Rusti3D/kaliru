// Load Cell Probe
// (port of klippy/extras/load_cell/load_cell_probe.py)
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::extras::homing::{HomingMove as HMove, PrinterHoming};
use crate::extras::load_cell::load_cell::{LoadCell, LoadCellSampleCollector, ZeroReference};
use crate::extras::load_cell::sos_filter::{self, DigitalFilter, FixedPointSosFilter, SosFilter};
use crate::extras::load_cell::tap_analysis::{TapAnalysisHelper, TapClassifier};
use crate::extras::load_cell::BulkAdcSensor;
use crate::extras::probe::{McuProbe, PrinterProbe};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::chelper::serialqueue::CommandQueue;
use crate::mcu::{CommandQueryWrapper, CommandWrapper, Endstop, EndstopRef, Mcu, McuTrsync, StepperRef, TriggerDispatch};
use crate::msgproto::Val;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::ReactorCompletion;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

// constants for fixed point numbers
const Q2_INT_BITS: i64 = 2;
const Q2_FRAC_BITS: i64 = 32 - (1 + Q2_INT_BITS);
const Q16_INT_BITS: i64 = 16;

/// python truthiness of an optional bound (`x or y`)
fn or(a: &Option<PyVal>, b: &Option<PyVal>) -> Option<PyVal> {
    match a {
        Some(v) if v.truthy() => Some(v.clone()),
        _ => b.clone(),
    }
}

fn lim(minval: Option<PyVal>, maxval: Option<PyVal>, above: Option<PyVal>, below: Option<PyVal>) -> Lim {
    Lim { minval, maxval, above, below, note_valid: true }
}

#[derive(Clone, Copy, PartialEq)]
enum ParamType {
    Boolean,
    Int,
    Float,
    FloatList,
}

/// per call bound overrides of ParamHelper.get()
#[derive(Default)]
struct Bounds {
    above: Option<PyVal>,
    below: Option<PyVal>,
}

// Access a parameter from config or GCode command via a consistent interface
// stores name and constraints to keep things DRY
struct ParamHelper {
    config_section: String,
    name: String,
    type_name: ParamType,
    value: PyVal,
    minval: Option<PyVal>,
    maxval: Option<PyVal>,
    above: Option<PyVal>,
    below: Option<PyVal>,
    max_len: Option<usize>,
}

impl ParamHelper {
    #[allow(clippy::too_many_arguments)]
    fn new(
        config: &ConfigWrapper,
        name: &str,
        type_name: ParamType,
        default: PyVal,
        minval: Option<PyVal>,
        maxval: Option<PyVal>,
        above: Option<PyVal>,
        below: Option<PyVal>,
        max_len: Option<usize>,
    ) -> KResult<Self> {
        let mut p = ParamHelper { config_section: config.get_name().to_string(), name: name.to_string(), type_name, value: default, minval, maxval, above, below, max_len };
        // read from config once
        p.value = p.get_config(config)?;
        Ok(p)
    }

    fn gname(&self, gcmd: Option<&GCodeCommand>) -> String {
        if gcmd.is_some() {
            self.name.to_uppercase()
        } else {
            self.name.clone()
        }
    }

    fn validate_float(&self, description: &str, error: &dyn Fn(String) -> KError, value: f64, b: &Bounds) -> KResult<()> {
        if let Some(above) = or(&b.above, &self.above) {
            if value <= above.as_f64().unwrap_or(0.0) {
                return Err(error(format!("{} must be above {}", description, above.str())));
            }
        }
        if let Some(below) = or(&b.below, &self.below) {
            if value >= below.as_f64().unwrap_or(0.0) {
                return Err(error(format!("{} must be below {}", description, below.str())));
            }
        }
        Ok(())
    }

    // support for validating individual options in a list of floats
    fn validate_float_list(&self, gcmd: Option<&GCodeCommand>, values: &[f64], b: &Bounds) -> KResult<()> {
        let (description, error): (String, Box<dyn Fn(String) -> KError>) = match gcmd {
            Some(g) => {
                let g = g.clone();
                (format!("Error on '{}': {}", g.get_commandline(), self.gname(Some(&g))), Box::new(move |m| g.error(m)))
            }
            None => (format!("Option '{}' in section '{}'", self.name, self.config_section), Box::new(KError::config)),
        };
        if let Some(max_len) = self.max_len {
            if values.len() > max_len {
                return Err(error(format!("{} has maximum length {}", description, max_len)));
            }
        }
        for value in values {
            self.validate_float(&description, &*error, *value, b)?;
        }
        Ok(())
    }

    fn get_gcmd_boolean(gcmd: &GCodeCommand, name: &str, default: &PyVal) -> KResult<PyVal> {
        let raw = match gcmd.get_opt(name) {
            Some(r) => r,
            None => {
                if matches!(default, PyVal::None) {
                    return Ok(PyVal::None);
                }
                default.str()
            }
        };
        if raw.to_uppercase() == "FALSE" || raw == "0" {
            Ok(PyVal::Bool(false))
        } else if raw.to_uppercase() == "TRUE" || raw == "1" {
            Ok(PyVal::Bool(true))
        } else {
            Err(gcmd.error(format!("Value `{}` is not a valid boolean for {}", raw, name)))
        }
    }

    fn get_config(&self, config: &ConfigWrapper) -> KResult<PyVal> {
        let name = self.gname(None);
        let dflt = |v: &PyVal| if matches!(v, PyVal::None) { Dflt::None } else { Dflt::Val(v.clone()) };
        Ok(match self.type_name {
            ParamType::Boolean => match dflt(&self.value) {
                Dflt::Val(v) => PyVal::Bool(config.getboolean(&name, v.truthy())?),
                _ => config.getboolean_opt(&name)?.map(PyVal::Bool).unwrap_or(PyVal::None),
            },
            ParamType::Int => {
                let l = lim(self.minval.clone(), self.maxval.clone(), None, None);
                PyVal::Int(config.getint(&name, dflt(&self.value), l)?)
            }
            ParamType::Float => {
                let l = lim(self.minval.clone(), self.maxval.clone(), self.above.clone(), self.below.clone());
                match dflt(&self.value) {
                    Dflt::Val(v) => PyVal::Float(config.getfloat(&name, v, l)?),
                    _ => config.getfloat_opt(&name, l)?.map(PyVal::Float).unwrap_or(PyVal::None),
                }
            }
            ParamType::FloatList => {
                // this code defaults to the empty list, never return None
                let default = match &self.value {
                    PyVal::List(v) => v.clone(),
                    _ => vec![],
                };
                let float_list = config.getfloatlist(&name, PyVal::List(default), ",", None)?.unwrap_or_default();
                if !float_list.is_empty() {
                    self.validate_float_list(None, &float_list, &Bounds::default())?;
                }
                PyVal::List(float_list.into_iter().map(PyVal::Float).collect())
            }
        })
    }

    fn get(&self, gcmd: Option<&GCodeCommand>, b: Bounds) -> KResult<PyVal> {
        let Some(gcmd) = gcmd else { return Ok(self.value.clone()) };
        let name = self.gname(Some(gcmd));
        Ok(match self.type_name {
            ParamType::Boolean => Self::get_gcmd_boolean(gcmd, &name, &self.value)?,
            ParamType::Int => {
                let l = lim(self.minval.clone(), self.maxval.clone(), None, None);
                match &self.value {
                    PyVal::Int(d) => PyVal::Int(gcmd.get_int(&name, Some(*d), l)?),
                    _ => match gcmd.get_opt(&name) {
                        None => PyVal::None,
                        Some(_) => PyVal::Int(gcmd.get_int(&name, None, l)?),
                    },
                }
            }
            ParamType::Float => {
                let l = lim(self.minval.clone(), self.maxval.clone(), or(&b.above, &self.above), or(&b.below, &self.below));
                match self.value.as_f64() {
                    Some(d) => PyVal::Float(gcmd.get_float(&name, Some(d), l)?),
                    None => gcmd.get_float_opt(&name, l)?.map(PyVal::Float).unwrap_or(PyVal::None),
                }
            }
            ParamType::FloatList => {
                let default = self.value.clone();
                // if the parameter isn't part of the command, return the default
                if gcmd.get_opt(&name).is_none() {
                    return Ok(default);
                }
                // parameter exists, always prefer whatever is in the command
                let value = gcmd.get_or(&name, "");
                // Return an empty list for empty value
                if crate::pyfmt::py_strip(&value).is_empty() {
                    return Ok(PyVal::List(vec![]));
                }
                let mut float_list = vec![];
                for p in value.split(',') {
                    match crate::pyfmt::py_float(crate::pyfmt::py_strip(p)) {
                        Some(f) => float_list.push(f),
                        None => return Err(gcmd.error(format!("Error on '{}': unable to parse {}", gcmd.get_commandline(), value))),
                    }
                }
                if !float_list.is_empty() {
                    self.validate_float_list(Some(gcmd), &float_list, &b)?;
                }
                PyVal::List(float_list.into_iter().map(PyVal::Float).collect())
            }
        })
    }

    fn get_f64(&self, gcmd: Option<&GCodeCommand>) -> KResult<Option<f64>> {
        Ok(self.get(gcmd, Bounds::default())?.as_f64())
    }

    fn get_i64(&self, gcmd: Option<&GCodeCommand>) -> KResult<i64> {
        Ok(self.get(gcmd, Bounds::default())?.as_f64().unwrap_or(0.0) as i64)
    }
}

fn float_param(config: &ConfigWrapper, name: &str, default: Option<f64>, minval: Option<f64>, maxval: Option<f64>, above: Option<PyVal>, below: Option<PyVal>) -> KResult<ParamHelper> {
    ParamHelper::new(config, name, ParamType::Float, default.map(PyVal::Float).unwrap_or(PyVal::None), minval.map(PyVal::Float), maxval.map(PyVal::Float), above, below, None)
}

fn int_param(config: &ConfigWrapper, name: &str, default: i64, minval: Option<i64>, maxval: Option<i64>) -> KResult<ParamHelper> {
    ParamHelper::new(config, name, ParamType::Int, PyVal::Int(default), minval.map(PyVal::Int), maxval.map(PyVal::Int), None, None, None)
}

/// the f64 items of a float list value
fn floats(v: &PyVal) -> Vec<f64> {
    v.items().map(|a| a.iter().filter_map(|x| x.as_f64()).collect()).unwrap_or_default()
}

// container for filter parameters
// allows different filter configurations to be compared
#[derive(Clone, PartialEq, Default)]
struct ContinuousTareFilter {
    sps: Option<i64>,
    drift: Option<f64>,
    drift_delay: Option<i64>,
    buzz: Option<f64>,
    buzz_delay: Option<i64>,
    notches: Option<Vec<f64>>,
    notch_quality: Option<f64>,
}

impl ContinuousTareFilter {
    // create a filter design from the parameters
    fn design_filter(&self) -> KResult<FixedPointSosFilter> {
        let design = DigitalFilter::new(
            self.sps.unwrap_or(0) as f64,
            self.drift,
            self.drift_delay.unwrap_or(1),
            self.buzz,
            self.buzz_delay.unwrap_or(1),
            self.notches.as_deref().unwrap_or(&[]),
            self.notch_quality.unwrap_or(2.0),
        )?;
        FixedPointSosFilter::new(design.get_filter_sections(), design.get_initial_state(), Q2_INT_BITS, Q16_INT_BITS)
    }
}

// Combine ContinuousTareFilter and SosFilter into an easy-to-use class
struct ContinuousTareFilterHelper {
    sps: i64,
    drift_param: ParamHelper,
    drift_delay_param: ParamHelper,
    buzz_param: ParamHelper,
    buzz_delay_param: ParamHelper,
    notches_param: ParamHelper,
    notch_quality_param: ParamHelper,
    active_design: ContinuousTareFilter,
    sos_filter: Rc<SosFilter>,
}

impl ContinuousTareFilterHelper {
    fn new(config: &ConfigWrapper, sensor: &Rc<dyn BulkAdcSensor>, cmd_queue: CommandQueue) -> KResult<Self> {
        let sps = sensor.get_samples_per_second();
        let max_filter_frequency = (sps as f64 / 2.0).floor() as i64;
        // setup filter parameters
        let drift_param = float_param(config, "drift_filter_cutoff_frequency", None, Some(0.1), Some(20.0), None, None)?;
        let drift_delay_param = int_param(config, "drift_filter_delay", 2, Some(1), Some(2))?;
        let buzz_param = float_param(
            config,
            "buzz_filter_cutoff_frequency",
            None,
            None,
            None,
            Some(PyVal::Float(crate::pyfmt::py_min(80.0, max_filter_frequency as f64 - 1.0))),
            Some(PyVal::Int(max_filter_frequency)),
        )?;
        let buzz_delay_param = int_param(config, "buzz_filter_delay", 2, Some(1), Some(2))?;
        let notches_param = ParamHelper::new(
            config,
            "notch_filter_frequencies",
            ParamType::FloatList,
            PyVal::List(vec![]),
            None,
            None,
            Some(PyVal::Float(0.0)),
            Some(PyVal::Int(max_filter_frequency)),
            Some(2),
        )?;
        let notch_quality_param = float_param(config, "notch_filter_quality", Some(2.0), Some(0.5), Some(6.0), None, None)?;
        let params = [&drift_param, &drift_delay_param, &buzz_param, &buzz_delay_param, &notches_param, &notch_quality_param];
        // filter design specified in the config file, used for defaults
        // filter design currently inside the MCU
        let active_design = Self::build(sps, params, None)?;
        let sos_filter = SosFilter::new(&sensor.get_mcu(), cmd_queue, active_design.design_filter()?, Some(4));
        Ok(ContinuousTareFilterHelper {
            sps,
            drift_param,
            drift_delay_param,
            buzz_param,
            buzz_delay_param,
            notches_param,
            notch_quality_param,
            active_design,
            sos_filter,
        })
    }

    /// _build_filter(gcmd) from the drift, drift delay, buzz, buzz delay,
    /// notches and notch quality params
    fn build(sps: i64, p: [&ParamHelper; 6], gcmd: Option<&GCodeCommand>) -> KResult<ContinuousTareFilter> {
        let drift = p[0].get_f64(gcmd)?;
        let drift_delay = p[1].get(gcmd, Bounds::default())?.as_f64().map(|v| v as i64);
        let buzz = p[2].get_f64(gcmd)?;
        let buzz_delay = p[3].get(gcmd, Bounds::default())?.as_f64().map(|v| v as i64);
        // notches must be between drift and buzz:
        let notches = floats(&p[4].get(gcmd, Bounds { above: drift.map(PyVal::Float), below: buzz.map(PyVal::Float) })?);
        let notch_quality = p[5].get_f64(gcmd)?;
        Ok(ContinuousTareFilter { sps: Some(sps), drift, drift_delay, buzz, buzz_delay, notches: Some(notches), notch_quality })
    }

    fn build_filter(&self, gcmd: Option<&GCodeCommand>) -> KResult<ContinuousTareFilter> {
        let p = [&self.drift_param, &self.drift_delay_param, &self.buzz_param, &self.buzz_delay_param, &self.notches_param, &self.notch_quality_param];
        Self::build(self.sps, p, gcmd)
    }

    fn update_from_command(&self, gcmd: Option<&GCodeCommand>) -> KResult<()> {
        let gcmd_filter = self.build_filter(gcmd)?;
        // if filters are identical, no change required
        if self.active_design == gcmd_filter {
            return Ok(());
        }
        // update MCU filter from GCode command (python designs the active
        // filter here, not the one from the command)
        self.sos_filter.change_filter(self.active_design.design_filter()?)
    }
}

struct LoadCellProbeConfigHelper {
    load_cell: Rc<LoadCell>,
    rest_time: f64,
    tare_time_param: ParamHelper,
    trigger_force_param: ParamHelper,
    force_safety_limit_param: ParamHelper,
    drift_safety_limit: ParamHelper,
    disable_pullback_move: ParamHelper,
    pullback_distance_param: ParamHelper,
    pullback_speed_param: ParamHelper,
}

impl LoadCellProbeConfigHelper {
    fn new(config: &ConfigWrapper, load_cell: &Rc<LoadCell>) -> KResult<Self> {
        let sensor = load_cell.sensor.clone();
        let rest_time = 1.0 / sensor.get_samples_per_second() as f64;
        // Collect 5 x 50hz power cycles of data to average across power noise
        let tare_time_param = float_param(config, "tare_time", Some(5.0 / 50.0), Some(0.01), Some(1.0), None, None)?;
        // triggering options
        let trigger_force_param = int_param(config, "trigger_force", 75, Some(10), Some(250))?;
        let force_safety_limit_param = int_param(config, "force_safety_limit", 2000, Some(0), None)?;
        let drift_safety_limit = int_param(config, "drift_safety_limit", 1000, Some(0), None)?;
        // pullback move
        let disable_pullback_move = ParamHelper::new(config, "disable_pullback_move", ParamType::Boolean, PyVal::Bool(false), None, None, None, None, None)?;
        let pullback_distance_param = float_param(config, "pullback_distance", Some(0.2), Some(0.01), Some(2.0), None, None)?;
        let sps = sensor.get_samples_per_second();
        let pullback_speed_param = float_param(config, "pullback_speed", Some(sps as f64 * 0.001), Some(0.1), Some(1.0), None, None)?;
        Ok(LoadCellProbeConfigHelper {
            load_cell: load_cell.clone(),
            rest_time,
            tare_time_param,
            trigger_force_param,
            force_safety_limit_param,
            drift_safety_limit,
            disable_pullback_move,
            pullback_distance_param,
            pullback_speed_param,
        })
    }

    fn get_tare_samples(&self, gcmd: Option<&GCodeCommand>) -> KResult<i64> {
        let tare_time = self.tare_time_param.get_f64(gcmd)?.unwrap_or(0.0);
        let sps = self.load_cell.sensor.get_samples_per_second();
        Ok(2.max((tare_time * sps as f64).ceil() as i64))
    }

    fn get_trigger_force_grams(&self, gcmd: Option<&GCodeCommand>) -> KResult<i64> {
        self.trigger_force_param.get_i64(gcmd)
    }

    fn get_safety_limit_grams(&self, gcmd: Option<&GCodeCommand>) -> KResult<i64> {
        self.force_safety_limit_param.get_i64(gcmd)
    }

    fn get_drift_safety_limit(&self, gcmd: Option<&GCodeCommand>) -> KResult<i64> {
        self.drift_safety_limit.get_i64(gcmd)
    }

    fn get_pullback_speed(&self, gcmd: Option<&GCodeCommand>) -> KResult<f64> {
        Ok(self.pullback_speed_param.get_f64(gcmd)?.unwrap_or(0.0))
    }

    fn get_pullback_distance(&self, gcmd: Option<&GCodeCommand>) -> KResult<f64> {
        Ok(self.pullback_distance_param.get_f64(gcmd)?.unwrap_or(0.0))
    }

    fn is_pullback_move_disabled(&self, gcmd: Option<&GCodeCommand>) -> KResult<bool> {
        Ok(self.disable_pullback_move.get(gcmd, Bounds::default())?.truthy())
    }

    fn get_rest_time(&self) -> f64 {
        self.rest_time
    }

    fn counts_per_gram(&self) -> KResult<f64> {
        self.load_cell
            .get_counts_per_gram()
            .ok_or_else(|| KError::Py("TypeError".into(), "unsupported operand type(s) for *: 'NoneType' and 'int'".into()))
    }

    fn get_reference_safety_range(&self, gcmd: Option<&GCodeCommand>) -> KResult<(i64, i64)> {
        let counts_per_gram = self.counts_per_gram()?;
        // calculate the safety band
        let zero = self.load_cell.get_reference_tare_counts().ok_or_else(|| KError::Py("TypeError".into(), "unsupported operand type(s) for -: 'NoneType' and 'int'".into()))?;
        let safety_counts = (counts_per_gram * self.get_safety_limit_grams(gcmd)? as f64) as i64;
        let safety_min = zero - safety_counts;
        let safety_max = zero + safety_counts;
        // don't allow a safety range outside the sensor's real range
        let (sensor_min, sensor_max) = self.load_cell.sensor.get_range();
        if safety_min <= sensor_min || safety_max >= sensor_max {
            return Err(KError::command("Load Cell Probe Error: force_safety_limit exceeds sensor range!"));
        }
        Ok((safety_min, safety_max))
    }

    // check if tare_counts is within the force_safety_limit
    fn assert_force_safety_limit(&self, gcmd: Option<&GCodeCommand>) -> KResult<()> {
        let limit = self.get_safety_limit_grams(gcmd)?;
        // zero limit disables this check
        if limit == 0 {
            return Ok(());
        }
        let (safety_min, safety_max) = self.get_reference_safety_range(gcmd)?;
        let tare_counts = self.load_cell.get_tare_counts().unwrap_or(0);
        if tare_counts <= safety_min || tare_counts >= safety_max {
            let force = self.load_cell.counts_to_grams(tare_counts, ZeroReference::ReferenceTare, None).unwrap_or(0.0);
            return Err(KError::command(format!(
                "Load Cell Probe Error: current absolute force of {:.1}g exceeds force_safety_limit (+/-{:.1}g) before probing!",
                force, limit as f64
            )));
        }
        Ok(())
    }

    fn get_probe_drift_range(&self, tare_counts: i64, gcmd: Option<&GCodeCommand>) -> KResult<(i64, i64)> {
        let counts_per_gram = self.counts_per_gram()?;
        let mut drift_min: i64 = -(1 << 31);
        let mut drift_max: i64 = (1 << 31) - 1;
        let drift_force = self.get_drift_safety_limit(gcmd)?;
        if drift_force > 0 {
            let drift_counts = (counts_per_gram * drift_force as f64) as i64;
            drift_min = tare_counts - drift_counts;
            drift_max = tare_counts + drift_counts;
            let (sensor_min, sensor_max) = self.load_cell.sensor.get_range();
            if drift_min <= sensor_min || drift_max >= sensor_max {
                return Err(KError::command("Load Cell Probe Error: drift_safety_limit exceeds sensor range!"));
            }
        }
        Ok((drift_min, drift_max))
    }

    // calculate 1/counts_per_gram in Q2 fixed point
    fn get_grams_per_count(&self) -> KResult<i64> {
        let counts_per_gram = self.counts_per_gram()?;
        // The counts_per_gram could be so large that it becomes 0.0 when
        // converted to Q2 format. This would mean the ADC range only measures a
        // few grams which seems very unlikely. Treat this as an error:
        if counts_per_gram >= (1i64 << Q2_FRAC_BITS) as f64 {
            return Err(KError::Py("OverflowError".into(), "counts_per_gram value is too large to filter".into()));
        }
        sos_filter::to_fixed_32(1.0 / counts_per_gram, Q2_INT_BITS)
    }
}

// McuLoadCellProbe is the interface to `load_cell_probe` on the MCU
// This also manages the SosFilter so all commands use one command queue
struct McuLoadCellProbe {
    load_cell: Rc<LoadCell>,
    sos_filter: Rc<SosFilter>,
    config_helper: Rc<LoadCellProbeConfigHelper>,
    mcu: Rc<Mcu>,
    dispatch: Rc<TriggerDispatch>,
    cmd_queue: CommandQueue,
    oid: i64,
    home_cmd: RefCell<Option<CommandWrapper>>,
    query_cmd: RefCell<Option<CommandQueryWrapper>>,
    set_range_cmd: RefCell<Option<CommandWrapper>>,
}

impl McuLoadCellProbe {
    const WATCHDOG_MAX: i64 = 3;
    const ERROR_SAFETY_RANGE: i64 = McuTrsync::REASON_COMMS_TIMEOUT + 1;
    const ERROR_OVERFLOW: i64 = McuTrsync::REASON_COMMS_TIMEOUT + 2;
    const ERROR_WATCHDOG: i64 = McuTrsync::REASON_COMMS_TIMEOUT + 3;

    fn new(
        config: &ConfigWrapper,
        load_cell: &Rc<LoadCell>,
        sos_filter: Rc<SosFilter>,
        config_helper: Rc<LoadCellProbeConfigHelper>,
        dispatch: Rc<TriggerDispatch>,
    ) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let sensor = load_cell.sensor.clone();
        let mcu = sensor.get_mcu();
        // configure MCU objects
        let cmd_queue = dispatch.get_command_queue();
        let oid = mcu.create_oid();
        let p = Rc::new(McuLoadCellProbe {
            load_cell: load_cell.clone(),
            sos_filter,
            config_helper,
            mcu: mcu.clone(),
            dispatch,
            cmd_queue,
            oid,
            home_cmd: RefCell::new(None),
            query_cmd: RefCell::new(None),
            set_range_cmd: RefCell::new(None),
        });
        p.config_commands()?;
        let pp = p.clone();
        mcu.register_config_callback(Rc::new(move || pp.build_config()));
        // the sensor data stream is connected on the MCU at the ready event
        let s = sensor.clone();
        crate::events::KlippyConnect::on(&printer, move || s.attach_load_cell_probe(oid));
        Ok(p)
    }

    fn config_commands(&self) -> KResult<()> {
        self.sos_filter.create_filter()?;
        self.mcu.add_config_cmd(&format!("config_load_cell_probe oid={} sos_filter_oid={}", self.oid, self.sos_filter.get_oid()), false, false);
        Ok(())
    }

    fn build_config(&self) -> KResult<()> {
        *self.query_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "load_cell_probe_query_state oid=%c",
            "load_cell_probe_state oid=%c is_homing_trigger=%c trigger_ticks=%u",
            Some(self.oid),
            Some(self.cmd_queue.clone()),
            false,
        )?);
        *self.set_range_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "load_cell_probe_set_range oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i trigger_grams=%u grams_per_count=%i",
            Some(self.cmd_queue.clone()),
        )?);
        *self.home_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "load_cell_probe_home oid=%c trsync_oid=%c trigger_reason=%c error_reason=%c clock=%u rest_ticks=%u timeout=%u",
            Some(self.cmd_queue.clone()),
        )?);
        Ok(())
    }

    fn not_built() -> KError {
        KError::Py("AttributeError".into(), "'NoneType' object has no attribute 'send'".into())
    }

    fn set_endstop_range(&self, gcmd: Option<&GCodeCommand>) -> KResult<()> {
        // update the load cell so it reflects the new tare value
        let tare_counts = self.load_cell.get_tare_counts().unwrap_or(0);
        let (safety_min, safety_max) = self.config_helper.get_probe_drift_range(tare_counts, gcmd)?;
        let args = [self.oid, safety_min, safety_max, tare_counts, self.config_helper.get_trigger_force_grams(gcmd)?, self.config_helper.get_grams_per_count()?];
        {
            let c = self.set_range_cmd.borrow();
            c.as_ref().ok_or_else(Self::not_built)?.send(&args.map(Val::Int), 0, 0)?;
        }
        self.sos_filter.reset_filter()
    }

    fn home_start(&self, print_time: f64) -> KResult<()> {
        let clock = self.mcu.print_time_to_clock(print_time);
        let rest_time = self.config_helper.get_rest_time();
        let rest_ticks = self.mcu.seconds_to_clock(rest_time);
        let args = [self.oid, self.dispatch.get_oid(), McuTrsync::REASON_ENDSTOP_HIT, Self::ERROR_SAFETY_RANGE, clock, rest_ticks, Self::WATCHDOG_MAX];
        let c = self.home_cmd.borrow();
        c.as_ref().ok_or_else(Self::not_built)?.send(&args.map(Val::Int), 0, clock as u64)
    }

    fn clear_home(&self) -> KResult<f64> {
        let params = {
            let q = self.query_cmd.borrow();
            q.as_ref().ok_or_else(Self::not_built)?.send(&[Val::Int(self.oid)], 0, 0, true)?
        };
        // The time of the first sample that triggered is in "trigger_ticks"
        let ticks = match params.get("trigger_ticks") {
            Some(Val::Int(t)) => *t,
            _ => 0,
        };
        let trigger_ticks = self.mcu.clock32_to_clock64(ticks);
        // clear trsync from load_cell_endstop
        {
            let c = self.home_cmd.borrow();
            c.as_ref().ok_or_else(Self::not_built)?.send(&[self.oid, 0, 0, 0, 0, 0, 0, 0].map(Val::Int), 0, 0)?;
        }
        Ok(self.mcu.clock_to_print_time(trigger_ticks as f64))
    }
}

// Execute probing moves using the McuLoadCellProbe
struct LoadCellPrimitives {
    printer: Rc<Printer>,
    mcu_load_cell_probe: Rc<McuLoadCellProbe>,
    continuous_tare_filter_helper: ContinuousTareFilterHelper,
    config_helper: Rc<LoadCellProbeConfigHelper>,
    load_cell: Rc<LoadCell>,
    dispatch: Rc<TriggerDispatch>,
    // internal state tracking
    last_trigger_time: Cell<f64>,
}

impl LoadCellPrimitives {
    fn error_map(res: i64) -> Option<&'static str> {
        match res {
            McuTrsync::REASON_COMMS_TIMEOUT => Some("Communication timeout during homing"),
            McuLoadCellProbe::ERROR_SAFETY_RANGE => Some("Load Cell Probe Error: force exceeded drift_safety_limit before triggering!"),
            McuLoadCellProbe::ERROR_OVERFLOW => Some("Load Cell Probe Error: fixed point math overflow"),
            McuLoadCellProbe::ERROR_WATCHDOG => Some("Load Cell Probe Error: timed out waiting for sensor data"),
            _ => None,
        }
    }

    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_load_cell_probe.mcu.clone()
    }

    fn start_collector(&self) -> KResult<Rc<LoadCellSampleCollector>> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        // homing uses the toolhead last move time which gets special handling
        // to significantly buffer print_time if the move queue has drained
        let print_time = toolhead.get_last_move_time()?;
        let collector = self.load_cell.get_collector();
        collector.start_collecting(Some(print_time));
        Ok(collector)
    }

    // pauses for the last move to complete and then
    // sets the endstop tare value and range
    fn tare(&self, gcmd: Option<&GCodeCommand>) -> KResult<()> {
        let num_samples = self.config_helper.get_tare_samples(gcmd)?;
        let tare_counts_per_channel = self.load_cell.avg_counts(Some(num_samples))?;
        self.load_cell.tare(&tare_counts_per_channel)?;
        self.config_helper.assert_force_safety_limit(gcmd)?;
        // update sos_filter with any gcode parameter changes
        self.continuous_tare_filter_helper.update_from_command(gcmd)?;
        self.mcu_load_cell_probe.set_endstop_range(gcmd)
    }

    fn home_start(&self, print_time: f64) -> KResult<Rc<ReactorCompletion<bool>>> {
        // do not permit homing if the load cell is not calibrated
        if !self.load_cell.is_calibrated() {
            return Err(KError::command("Load Cell not calibrated"));
        }
        // start trsync
        let trigger_completion = self.dispatch.start(print_time)?;
        self.mcu_load_cell_probe.home_start(print_time)?;
        Ok(trigger_completion)
    }

    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.dispatch.wait_end(home_end_time)?;
        // trigger has happened, now to find out why...
        let res = self.dispatch.stop()?;
        // clear the homing state so it stops processing samples
        self.last_trigger_time.set(self.mcu_load_cell_probe.clear_home()?);
        if res >= McuTrsync::REASON_COMMS_TIMEOUT {
            let error = Self::error_map(res).map(|s| s.to_string()).unwrap_or_else(|| format!("Load Cell Probe Error: unknown reason code {}", res));
            return Err(KError::command(error));
        }
        if res != McuTrsync::REASON_ENDSTOP_HIT {
            return Ok(0.0);
        }
        Ok(self.last_trigger_time.get())
    }

    // Probe towards z_min until the load_cell_probe on the MCU triggers
    // returns the running collector instance along with the halt position
    fn probing_move(&self, mcu_probe: EndstopRef, pos: &[f64], speed: f64, gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Rc<LoadCellSampleCollector>)> {
        // tare the sensor just before probing
        self.tare(Some(gcmd))?;
        // start collector after tare samples are consumed
        let collector = self.start_collector()?;
        // do homing move
        let printer_homing = self.printer.lookup::<PrinterHoming>("homing")?;
        match printer_homing.probing_move(mcu_probe, pos, speed) {
            Ok(epos) => Ok((epos, collector)),
            Err(e) => {
                if e.is_command_error() {
                    collector.stop_collecting();
                }
                Err(e)
            }
        }
    }

    // Wait for the MCU to trigger with no movement
    fn probing_test(&self, gcmd: &GCodeCommand, timeout: f64) -> KResult<f64> {
        self.tare(Some(gcmd))?;
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let print_time = toolhead.get_last_move_time()?;
        self.home_start(print_time)?;
        self.home_wait(print_time + timeout)
    }

    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        let mut status = match PrinterObject::get_status(&*self.load_cell, eventtime)? {
            PyVal::Dict(d) => d,
            _ => return None,
        };
        status.insert("last_trigger_time".into(), PyVal::Float(self.last_trigger_time.get()));
        Some(PyVal::Dict(status))
    }
}

// Handle homing the z axis with the load cell probe
struct HomingMove {
    printer: Rc<Printer>,
    primitives: Rc<LoadCellPrimitives>,
}

impl HomingMove {
    // Overrides for the MCU_endstop interface
    fn home_start(&self) -> KResult<Rc<ReactorCompletion<bool>>> {
        self.primitives.tare(None)?;
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        // taring requires time, so print_time must be updated
        let print_time = toolhead.get_last_move_time()?;
        self.primitives.home_start(print_time)
    }
}

// Perform a single complete tap, broadcast results to the socket
struct TappingMove {
    me: Weak<TappingMove>,
    printer: Rc<Printer>,
    primitives: Rc<LoadCellPrimitives>,
    tap_analysis_helper: TapAnalysisHelper,
    config_helper: Rc<LoadCellProbeConfigHelper>,
    // track results of the last tap
    is_last_result_valid: Cell<bool>,
}

impl TappingMove {
    // Perform the pullback move and returns the time when the move will end
    fn pullback_move(&self, gcmd: &GCodeCommand) -> KResult<f64> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let mut pullback_pos = toolhead.get_position();
        pullback_pos[2] += self.config_helper.get_pullback_distance(Some(gcmd))?;
        let pos = [None, None, Some(pullback_pos[2])];
        toolhead.manual_move(&pos, self.config_helper.get_pullback_speed(Some(gcmd))?)?;
        toolhead.flush_step_generation()?;
        toolhead.get_last_move_time()
    }

    // perform a complete tapping cycle
    fn probing_move(&self, pos: &[f64], speed: f64, gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        self.is_last_result_valid.set(false);
        // do the probing/homing move
        let me: EndstopRef = self.me.upgrade().expect("tapping move");
        let (mut epos, collector) = self.primitives.probing_move(me, pos, speed, gcmd)?;
        // when pullback is disabled, skip the pullback move and tap analysis
        if self.config_helper.is_pullback_move_disabled(Some(gcmd))? {
            collector.stop_collecting();
            self.is_last_result_valid.set(true);
            return Ok((epos, Some(true)));
        }
        // do the pullback move
        let pullback_end_time = self.pullback_move(gcmd)?;
        // collect samples from the tap
        let (samples, errors) = collector.collect_until(pullback_end_time)?;
        self.primitives.load_cell.validate_samples(&samples, errors)?;
        // calculate how long we waited to get the data
        let t_end = self.printer.get_reactor().monotonic();
        let t_end = self.primitives.get_mcu().estimated_print_time(t_end);
        let collection_time = t_end - pullback_end_time;
        // check for data errors
        let trigger_force = self.config_helper.get_trigger_force_grams(Some(gcmd))?;
        // Analyze the tap data
        let tap_analysis = self.tap_analysis_helper.analyze(&samples, trigger_force, collection_time, Some(gcmd))?;
        self.is_last_result_valid.set(tap_analysis.is_valid());
        // if the tap is valid, replace the z position with the calculated one
        if self.is_last_result_valid.get() {
            if let Some(p) = tap_analysis.get_tap_pos() {
                epos[2] = p.2;
            }
        }
        Ok((epos, Some(self.is_last_result_valid.get())))
    }

    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        let mut status = match self.primitives.get_status(eventtime)? {
            PyVal::Dict(d) => d,
            _ => return None,
        };
        status.insert("is_last_tap_valid".into(), PyVal::Bool(self.is_last_result_valid.get()));
        Some(PyVal::Dict(status))
    }
}

impl Endstop for TappingMove {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.primitives.get_mcu()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.primitives.dispatch.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        self.primitives.dispatch.get_steppers()
    }
    fn home_start(&self, print_time: f64, _sample_time: f64, _sample_count: i64, _rest_time: f64, _triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        self.primitives.home_start(print_time)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.primitives.home_wait(home_end_time)
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        Ok(0)
    }
}

const CMD_LOAD_CELL_TEST_TAP_HELP: &str = "Tap the load cell probe to verify operation";

fn cmd_load_cell_test_tap(printer: &Printer, primitives: &LoadCellPrimitives, gcmd: &GCodeCommand) -> KResult<()> {
    let taps = gcmd.get_int("TAPS", Some(3), Lim::new().min(1).max(10))?;
    let timeout = gcmd.get_float("TIMEOUT", Some(30.0), Lim::new().min(1.0).max(120.0))?;
    gcmd.respond_info(&format!("Tap the load cell {} times:", taps), true);
    let reactor = printer.get_reactor();
    for _ in 0..taps {
        let result = primitives.probing_test(gcmd, timeout)?;
        if result == 0.0 {
            // notify of error, likely due to timeout
            return Err(gcmd.error("Test timeout out"));
        }
        gcmd.respond_info("Tap Detected!", true);
        // give the user some time for their finger to move away
        reactor.pause(reactor.monotonic() + 0.2)?;
    }
    gcmd.respond_info(&format!("Test complete, {} taps detected", taps), true);
    Ok(())
}

// Probe `activate_gcode` and `deactivate_gcode` support
struct ProbeActivationHelper {
    printer: Rc<Printer>,
    activate_gcode: Rc<Template>,
    deactivate_gcode: Rc<Template>,
    is_multi: Cell<bool>,
    is_active: Cell<bool>,
}

impl ProbeActivationHelper {
    fn new(config: &ConfigWrapper) -> KResult<Self> {
        let gcode_macro = load_gcode_macro(config)?;
        Ok(ProbeActivationHelper {
            printer: config.get_printer(),
            activate_gcode: gcode_macro.load_template(config, "activate_gcode", Some(""))?,
            deactivate_gcode: gcode_macro.load_template(config, "deactivate_gcode", Some(""))?,
            is_multi: Cell::new(false),
            is_active: Cell::new(false),
        })
    }

    fn run(&self, template: &Template, what: &str) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let start_pos = toolhead.get_position();
        template.run_gcode_from_command(None)?;
        if toolhead.get_position()[..3] != start_pos[..3] {
            return Err(KError::command(format!("Toolhead moved during probe {} script", what)));
        }
        Ok(())
    }

    fn activate_probe(&self) -> KResult<()> {
        if self.is_active.get() {
            return Ok(());
        }
        self.is_active.set(true);
        self.run(&self.activate_gcode, "activate_gcode")
    }

    fn deactivate_probe(&self) -> KResult<()> {
        if !self.is_active.get() {
            return Ok(());
        }
        self.is_active.set(false);
        self.run(&self.deactivate_gcode, "deactivate_gcode")
    }
}

struct LoadCellEndstopWrapper {
    printer: Rc<Printer>,
    z_offset: f64,
    homing_move: HomingMove,
    tapping_move: Rc<TappingMove>,
    probe_activation_helper: ProbeActivationHelper,
}

impl LoadCellEndstopWrapper {
    fn handle_mcu_identify(&self) -> KResult<()> {
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        for stepper in kin.get_steppers() {
            if stepper.is_active_axis('z') {
                self.add_stepper(&stepper)?;
            }
        }
        Ok(())
    }
}

impl Endstop for LoadCellEndstopWrapper {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.homing_move.primitives.get_mcu()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.homing_move.primitives.dispatch.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        self.homing_move.primitives.dispatch.get_steppers()
    }
    fn home_start(&self, _print_time: f64, _sample_time: f64, _sample_count: i64, _rest_time: f64, _triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        self.homing_move.home_start()
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.homing_move.primitives.home_wait(home_end_time)
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        Ok(0)
    }
    // Interface for ProbeEndstopWrapper
    fn get_position_endstop(&self) -> Option<f64> {
        Some(self.z_offset)
    }
}

impl McuProbe for LoadCellEndstopWrapper {
    fn multi_probe_begin(&self, _always_restore_toolhead: bool) -> KResult<()> {
        self.probe_activation_helper.is_multi.set(true);
        self.probe_activation_helper.activate_probe()
    }
    fn multi_probe_end(&self) -> KResult<()> {
        self.probe_activation_helper.is_multi.set(false);
        self.probe_activation_helper.deactivate_probe()
    }
    fn probing_move(&self, pos: &[f64], speed: f64, gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        self.tapping_move.probing_move(pos, speed, gcmd)
    }
    fn probe_prepare(&self, _hmove: &Rc<HMove>) -> KResult<()> {
        self.probe_activation_helper.activate_probe()
    }
    fn probe_finish(&self, _hmove: &Rc<HMove>) -> KResult<()> {
        // only deactivate on probe if not doing a multi-probe
        if !self.probe_activation_helper.is_multi.get() {
            return self.probe_activation_helper.deactivate_probe();
        }
        Ok(())
    }
}

pub struct LoadCellPrinterProbe {
    tapping_move: Rc<TappingMove>,
}

impl LoadCellPrinterProbe {
    pub fn new(config: &ConfigWrapper, sensor: Rc<dyn BulkAdcSensor>, tap_classifier: Rc<dyn TapClassifier>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let load_cell = LoadCell::new(config, sensor.clone())?;
        // Read all user configuration and build modules
        let name = config.get_name().to_string();
        let tap_analysis_helper = TapAnalysisHelper::new(&printer, &name, tap_classifier)?;
        let config_helper = Rc::new(LoadCellProbeConfigHelper::new(config, &load_cell)?);
        let mcu = load_cell.sensor.get_mcu();
        let trigger_dispatch = Rc::new(TriggerDispatch::new(&mcu));
        let continuous_tare_filter_helper = ContinuousTareFilterHelper::new(config, &sensor, trigger_dispatch.get_command_queue())?;
        // Probe Interface
        let mcu_load_cell_probe = McuLoadCellProbe::new(
            config,
            &load_cell,
            continuous_tare_filter_helper.sos_filter.clone(),
            config_helper.clone(),
            trigger_dispatch.clone(),
        )?;
        let primitives = Rc::new(LoadCellPrimitives {
            printer: printer.clone(),
            mcu_load_cell_probe,
            continuous_tare_filter_helper,
            config_helper: config_helper.clone(),
            load_cell: load_cell.clone(),
            dispatch: trigger_dispatch,
            last_trigger_time: Cell::new(0.0),
        });
        let homing_move = HomingMove { printer: printer.clone(), primitives: primitives.clone() };
        let tapping_move = Rc::new_cyclic(|me| TappingMove {
            me: me.clone(),
            printer: printer.clone(),
            primitives: primitives.clone(),
            tap_analysis_helper,
            config_helper,
            is_last_result_valid: Cell::new(false),
        });
        // printer integration
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let (p, pr) = (printer.clone(), primitives.clone());
        gcode.register_command(
            "LOAD_CELL_TEST_TAP",
            Some(Rc::new(move |gcmd| cmd_load_cell_test_tap(&p, &pr, gcmd))),
            false,
            Some(CMD_LOAD_CELL_TEST_TAP_HELP),
        )?;
        let z_offset = config.getfloat("z_offset", Dflt::Required, Lim::new())?;
        let wrapper = Rc::new(LoadCellEndstopWrapper {
            printer: printer.clone(),
            z_offset,
            homing_move,
            tapping_move: tapping_move.clone(),
            probe_activation_helper: ProbeActivationHelper::new(config)?,
        });
        // Register for MCU identification to add Z steppers
        let w = wrapper.clone();
        crate::events::KlippyMcuIdentify::on(&printer, move || w.handle_mcu_identify());
        PrinterProbe::new(config, wrapper)?;
        Ok(Rc::new(LoadCellPrinterProbe { tapping_move }))
    }
}

impl PrinterObject for LoadCellPrinterProbe {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        self.tapping_move.get_status(eventtime)
    }
}

/// a tap classifier class of the load_cell_probe_tap_classifiers components
pub type ClassifierFactory = fn(&ConfigWrapper) -> KResult<Rc<dyn TapClassifier>>;

/// TapClassifierModule(config): the base class takes no arguments
pub fn empty_classifier(_config: &ConfigWrapper) -> KResult<Rc<dyn TapClassifier>> {
    Err(KError::Py("TypeError".into(), "TapClassifierModule() takes no arguments".into()))
}

pub fn load(config: &ConfigWrapper) -> KResult<PObj> {
    use crate::extras::load_cell::lookup_sensor_class;
    let sensor_class = lookup_sensor_class::<crate::extras::load_cell::SensorFactory>(config, "load_cell_probe_sensors")?;
    let classifiers = config.get_printer().lookup_components("load_cell_probe_tap_classifiers");
    let choices: Vec<(&str, ClassifierFactory)> =
        classifiers.iter().filter_map(|(name, c)| c.downcast_ref::<ClassifierFactory>().map(|f| (name.as_str(), *f))).collect();
    let tap_classifier = config.getchoice("tap_classifier", &choices, "tap_quality")?;
    let sensor = sensor_class(config)?;
    let classifier = tap_classifier(config)?;
    Ok(LoadCellPrinterProbe::new(config, sensor, classifier)?)
}
