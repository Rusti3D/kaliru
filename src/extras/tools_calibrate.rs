// Nozzle alignment module for 3d kinematic probes.
// (port of klippy/extras/tools_calibrate.py)
//
// This module has been adapted from code written by Kevin O'Connor <kevin@koconnor.net> and Martin Hierholzer <martin@hierholzer.info>
// Originally sourced from https://github.com/ben5459/Klipper_ToolChanger/blob/master/probe_multi_axis.py
// Sourced from https://github.com/viesturz/klipper-toolchanger/blob/main/klipper/extras/tools_calibrate.py
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::PrinterHoming;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::jinja::strformat::str_format;
use crate::jinja::value::Value;
use crate::kinematics::idex_modes::DualCarriages;
use crate::logging;
use crate::mcu::{Endstop, EndstopRef, McuEndstop, Mcu, StepperRef};
use crate::pins::{PinChip, PinsObject, SharedPinParams};
use crate::printer::{ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min, py_str_repr, py_sum};
use crate::pyval::PyVal;
use crate::reactor::ReactorCompletion;
use crate::stepper::Rail;
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::any::Any;
use std::cell::Cell;
use std::rc::Rc;

const X: usize = 0;
const Y: usize = 1;
const Z: usize = 2;

/// a Position(x, y, z) namedtuple
type Position = [f64; 3];

fn position_pyval(p: &Position) -> PyVal {
    PyVal::NamedTuple("Position", &["x", "y", "z"], p.iter().map(|v| PyVal::Float(*v)).collect())
}

fn list_repr(p: &[f64]) -> String {
    PyVal::List(p.iter().map(|v| PyVal::Float(*v)).collect()).repr()
}

/// Directions[direction]: (axis, sense)
fn direction(name: &str) -> Option<(usize, f64)> {
    match name {
        "x+" => Some((X, 1.0)),
        "x-" => Some((X, -1.0)),
        "y+" => Some((Y, 1.0)),
        "y-" => Some((Y, -1.0)),
        "z+" => Some((Z, 1.0)),
        "z-" => Some((Z, -1.0)),
        _ => None,
    }
}

const HINT_TIMEOUT: &str = "
If the probe did not move far enough to trigger, then
consider reducing/increasing the axis minimum/maximum
position so the probe can travel further (the minimum
position can be negative).
";

const CMD_TOOL_LOCATE_SENSOR_HELP: &str = "Locate the tool calibration sensor, use with tool 0.";
const CMD_TOOL_CALIBRATE_TOOL_OFFSET_HELP: &str = "Calibrate current tool offset relative to tool 0";
const CMD_TOOL_CALIBRATE_SAVE_TOOL_OFFSET_HELP: &str = "Save tool offset calibration to config";
const CMD_TOOL_CALIBRATE_QUERY_PROBE_HELP: &str = "Return the state of calibration probe";

/// a probe position argument: a Position, or a plain list once rebuilt
#[derive(Clone, Copy)]
struct TopPos {
    pos: Position,
    named: bool,
}

impl TopPos {
    fn repr(&self) -> String {
        if self.named {
            position_pyval(&self.pos).repr()
        } else {
            list_repr(&self.pos)
        }
    }
}

pub struct ToolsCalibrate {
    pub printer: Rc<Printer>,
    pub probe_multi_axis: Rc<PrinterProbeMultiAxis>,
    pub travel_speed: f64,
    pub spread: [f64; 2],
    pub initial_spread: [f64; 2],
    pub lower_z: f64,
    pub lift_z: f64,
    pub lift_speed: f64,
    pub final_lift_z: f64,
    sensor_location: Cell<Option<Position>>,
    last_result: Cell<Option<Position>>,
    pub last_probe_offset: Cell<f64>,
    pub calibration_probe_inactive: Cell<bool>,
    pub gcode: Rc<GCodeDispatch>,
}

impl ToolsCalibrate {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        printer.load_object(config, "gcode_move", true)?;
        let (px, py, pz) = (ProbeEndstopWrapper::new(config, 'x')?, ProbeEndstopWrapper::new(config, 'y')?, ProbeEndstopWrapper::new(config, 'z')?);
        let probe_multi_axis = PrinterProbeMultiAxis::new(config, px, py, pz)?;
        let travel_speed = config.getfloat("travel_speed", 10.0, Lim::new().above(0.0))?;

        let spread = config.getfloat("spread", 5.0, Lim::new())?;
        let spread = [config.getfloat("spread_x", spread, Lim::new())?, config.getfloat("spread_y", spread, Lim::new())?];
        let initial_spread =
            [config.getfloat("initial_spread_x", spread[X], Lim::new())?, config.getfloat("initial_spread_y", spread[Y], Lim::new())?];

        let lower_z = config.getfloat("lower_z", 0.5, Lim::new())?;
        let lift_z = config.getfloat("lift_z", 1.0, Lim::new())?;
        config.getfloat("trigger_to_bottom_z", 0.0, Lim::new())?;
        let lift_speed = config.getfloat("lift_speed", probe_multi_axis.lift_speed, Lim::new())?;
        let final_lift_z = config.getfloat("final_lift_z", 4.0, Lim::new())?;

        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let t = Rc::new(ToolsCalibrate {
            printer: printer.clone(),
            probe_multi_axis,
            travel_speed,
            spread,
            initial_spread,
            lower_z,
            lift_z,
            lift_speed,
            final_lift_z,
            sensor_location: Cell::new(None),
            last_result: Cell::new(None),
            last_probe_offset: Cell::new(0.0),
            calibration_probe_inactive: Cell::new(true),
            gcode: gcode.clone(),
        });
        t.reset_last_results();

        // Register events
        let tt = t.clone();
        crate::events::StepperEnableMotorOff::on(&printer, move || {
            tt.reset_last_results();
            Ok(())
        });

        // Register commands
        type TCmd = fn(&ToolsCalibrate, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, TCmd, &str); 4] = [
            ("TOOL_LOCATE_SENSOR", Self::cmd_tool_locate_sensor, CMD_TOOL_LOCATE_SENSOR_HELP),
            ("TOOL_CALIBRATE_TOOL_OFFSET", Self::cmd_tool_calibrate_tool_offset, CMD_TOOL_CALIBRATE_TOOL_OFFSET_HELP),
            ("TOOL_CALIBRATE_SAVE_TOOL_OFFSET", Self::cmd_tool_calibrate_save_tool_offset, CMD_TOOL_CALIBRATE_SAVE_TOOL_OFFSET_HELP),
            ("TOOL_CALIBRATE_QUERY_PROBE", Self::cmd_tool_calibrate_query_probe, CMD_TOOL_CALIBRATE_QUERY_PROBE_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let tt = t.clone();
            gcode.register_command(cmd, Some(Rc::new(move |gcmd| f(&tt, gcmd))), false, Some(desc))?;
        }
        Ok(t)
    }

    fn reset_last_results(&self) {
        self.sensor_location.set(None);
        self.last_result.set(None);
        self.last_probe_offset.set(0.0);
        self.calibration_probe_inactive.set(true);
    }

    fn probe_xy(&self, toolhead: &ToolHead, top_pos: TopPos, dir: &str, gcmd: &GCodeCommand, spread: [f64; 2], samples: Option<i64>) -> KResult<f64> {
        let (axis, offset) = direction(dir).expect("direction");
        let mut start_pos = top_pos.pos;
        start_pos[axis] -= offset * spread[axis];
        logging::info(&format!(
            "tools_calibrate: probe_xy(top_pos={}, start_pos={}, direction={})",
            top_pos.repr(),
            list_repr(&start_pos),
            py_str_repr(dir)
        ));
        toolhead.manual_move(&[None, None, Some(top_pos.pos[2] + self.lift_z)], self.lift_speed)?;
        toolhead.manual_move(&[Some(start_pos[0]), Some(start_pos[1]), None], self.travel_speed)?;
        toolhead.manual_move(&[None, None, Some(top_pos.pos[2] - self.lower_z)], self.lift_speed)?;
        Ok(self.probe_multi_axis.run_probe(dir, gcmd, 1.0, samples, spread[axis] * 1.8)?[axis])
    }

    fn calibrate_xy(&self, toolhead: &ToolHead, top_pos: TopPos, gcmd: &GCodeCommand, spread: [f64; 2], samples: Option<i64>) -> KResult<[f64; 2]> {
        let left_x = self.probe_xy(toolhead, top_pos, "x+", gcmd, spread, samples)?;
        let right_x = self.probe_xy(toolhead, top_pos, "x-", gcmd, spread, samples)?;
        let center_x = (left_x + right_x) / 2.0;
        let top_pos = TopPos { pos: [center_x, top_pos.pos[1], top_pos.pos[2]], named: false };
        let near_y = self.probe_xy(toolhead, top_pos, "y+", gcmd, spread, samples)?;
        let far_y = self.probe_xy(toolhead, top_pos, "y-", gcmd, spread, samples)?;
        Ok([center_x, (near_y + far_y) / 2.0])
    }

    fn locate_sensor(&self, gcmd: &GCodeCommand) -> KResult<Position> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let mut position = toolhead.get_position();
        let down_pos = self.probe_multi_axis.run_probe("z-", gcmd, 1.0, Some(1), 100.0)?;
        let [center_x, center_y] = self.calibrate_xy(&toolhead, TopPos { pos: down_pos, named: true }, gcmd, self.initial_spread, Some(1))?;

        toolhead.manual_move(&[None, None, Some(down_pos[2] + self.lift_z)], self.lift_speed)?;
        toolhead.manual_move(&[Some(center_x), Some(center_y), None], self.travel_speed)?;
        let center_z = self.probe_multi_axis.run_probe("z-", gcmd, 0.5, None, 100.0)?[2];
        // Now redo X and Y, since we have a more accurate center.
        let [center_x, center_y] =
            self.calibrate_xy(&toolhead, TopPos { pos: [center_x, center_y, center_z], named: false }, gcmd, self.spread, None)?;

        // rest above center
        position[0] = center_x;
        position[1] = center_y;
        position[2] = center_z + self.final_lift_z;
        toolhead.manual_move(&[None, None, Some(position[2])], self.lift_speed)?;
        toolhead.manual_move(&[Some(position[0]), Some(position[1]), None], self.travel_speed)?;
        toolhead.set_position(&position, "")?;
        Ok([center_x, center_y, center_z])
    }

    pub fn cmd_tool_locate_sensor(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let r = self.locate_sensor(gcmd)?;
        self.last_result.set(Some(r));
        self.sensor_location.set(Some(r));
        self.gcode.respond_info(&format!("Sensor location at {:.6},{:.6},{:.6}", r[0], r[1], r[2]), true);
        Ok(())
    }

    pub fn cmd_tool_calibrate_tool_offset(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let Some(sensor_location) = self.sensor_location.get() else {
            return Err(gcmd.error("No recorded sensor location, please run TOOL_LOCATE_SENSOR first"));
        };
        let location = self.locate_sensor(gcmd)?;
        let r = [location[0] - sensor_location[0], location[1] - sensor_location[1], location[2] - sensor_location[2]];
        self.last_result.set(Some(r));
        self.gcode.respond_info(&format!("Tool offset is {:.6},{:.6},{:.6}", r[0], r[1], r[2]), true);
        Ok(())
    }

    /// template.format(x=..., y=..., z=...)
    fn format_result(template: &str, r: &Position) -> KResult<String> {
        let mut kwargs = IndexMap::default();
        for (k, v) in ["x", "y", "z"].iter().zip(r) {
            kwargs.insert(k.to_string(), Value::Float(*v));
        }
        str_format(template, &[], &kwargs).map_err(|e| KError::Py(e.cls, e.msg))
    }

    pub fn cmd_tool_calibrate_save_tool_offset(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let Some(r) = self.last_result.get() else {
            // python builds the error without raising it
            return Ok(());
        };

        if gcmd.get_opt("SECTION").is_some_and(|s| !s.is_empty()) {
            let section_name = gcmd.get("SECTION")?;
            let param_name = gcmd.get("ATTRIBUTE")?;
            let template = gcmd.get_or("VALUE", "{x:0.6f}, {y:0.6f}, {z:0.6f}");
            let value = Self::format_result(&template, &r)?;

            let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
            configfile.pconfig.borrow_mut().set(&section_name, &param_name, &value)?;
        } else if gcmd.get_opt("MACRO").is_some_and(|s| !s.is_empty()) {
            let macro_name = gcmd.get("MACRO")?;
            let variable_name = gcmd.get("VARIABLE")?;
            let template = gcmd.get_or("VALUE", "({x:0.6f}, {y:0.6f}, {z:0.6f})");
            let value = Self::format_result(&template, &r)?;

            self.gcode.run_script_from_command(&format!(
                "SET_GCODE_VARIABLE MACRO=\"{}\" VARIABLE=\"{}\" VALUE=\"{}\"",
                macro_name, variable_name, value
            ))?;
        }
        Ok(())
    }

    pub fn cmd_tool_calibrate_query_probe(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let print_time = toolhead.get_last_move_time()?;
        let mut endstop_states = vec![];
        for probe in &self.probe_multi_axis.mcu_probe {
            endstop_states.push(probe.query_endstop(print_time)?);
        } // Check the state of each axis probe (x, y, z)
        let any = endstop_states.iter().any(|s| *s != 0);
        self.calibration_probe_inactive.set(any);
        gcmd.respond_info(&format!("Calibration Probe: {}", ["open", "TRIGGERED"][any as usize]), true);
        Ok(())
    }
}

impl PrinterObject for ToolsCalibrate {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let opt = |p: Option<Position>| p.map(|p| position_pyval(&p)).unwrap_or(PyVal::None);
        let d = crate::pydict! {
            "sensor_location" => opt(self.sensor_location.get()),
            "last_result" => opt(self.last_result.get()),
            "calibration_probe_inactive" => PyVal::Bool(self.calibration_probe_inactive.get()),
        };
        Some(PyVal::Dict(d))
    }
}

pub struct PrinterProbeMultiAxis {
    pub printer: Rc<Printer>,
    pub mcu_probe: [Rc<ProbeEndstopWrapper>; 3],
    pub speed: f64,
    pub lift_speed: f64,
    pub gcode: Rc<GCodeDispatch>,
    pub sample_count: i64,
    pub sample_retract_dist: f64,
    pub samples_result: String,
    pub samples_tolerance: f64,
    pub samples_retries: i64,
}

impl PrinterProbeMultiAxis {
    pub fn new(config: &ConfigWrapper, mcu_probe_x: Rc<ProbeEndstopWrapper>, mcu_probe_y: Rc<ProbeEndstopWrapper>, mcu_probe_z: Rc<ProbeEndstopWrapper>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let speed = config.getfloat("speed", 5.0, Lim::new().above(0.0))?;
        let lift_speed = config.getfloat("lift_speed", speed, Lim::new().above(0.0))?;
        config.getfloat("max_travel", 4.0, Lim::new().above(0.0))?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        printer.load_object(config, "gcode_move", true)?;

        // Multi-sample support (for improved accuracy)
        let sample_count = config.getint("samples", 1, Lim::new().min(1))?;
        let sample_retract_dist = config.getfloat("sample_retract_dist", 2.0, Lim::new().above(0.0))?;
        let samples_result = config.getchoice("samples_result", &[("median", "median"), ("average", "average")], "average")?;
        let samples_tolerance = config.getfloat("samples_tolerance", 0.100, Lim::new().min(0.0))?;
        let samples_retries = config.getint("samples_tolerance_retries", 0, Lim::new().min(0))?;
        let p = Rc::new(PrinterProbeMultiAxis {
            printer: printer.clone(),
            mcu_probe: [mcu_probe_x, mcu_probe_y, mcu_probe_z],
            speed,
            lift_speed,
            gcode,
            sample_count,
            sample_retract_dist,
            samples_result: samples_result.to_string(),
            samples_tolerance,
            samples_retries,
        });
        // Register xyz_virtual_endstop pin
        let ppins = printer.lookup::<PinsObject>("pins")?;
        ppins.0.borrow_mut().register_chip("probe_multi_axis", p.clone()).map_err(|e| KError::Pins(e.0))?;
        Ok(p)
    }

    pub fn get_lift_speed(&self, gcmd: Option<&GCodeCommand>) -> KResult<f64> {
        match gcmd {
            Some(gcmd) => gcmd.get_float("LIFT_SPEED", Some(self.lift_speed), Lim::new().above(0.0)),
            None => Ok(self.lift_speed),
        }
    }

    fn probe(&self, speed: f64, axis: usize, sense: f64, max_distance: f64) -> KResult<Position> {
        let phoming = self.printer.lookup::<PrinterHoming>("homing")?;
        let pos = self.get_target_position(axis, sense, max_distance)?;
        let endstop: EndstopRef = self.mcu_probe[axis].clone();
        let epos = match phoming.probing_move(endstop, &pos, speed) {
            Ok(epos) => epos,
            Err(e) if e.is_command_error() => {
                let mut reason = e.to_string();
                if reason.contains("Timeout during endstop homing") {
                    reason += HINT_TIMEOUT;
                }
                return Err(KError::command(reason));
            }
            Err(e) => return Err(e),
        };
        // self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
        self.gcode.respond_info(&format!("Probe made contact at {:.6},{:.6},{:.6}", epos[0], epos[1], epos[2]), true);
        Ok([epos[0], epos[1], epos[2]])
    }

    pub fn get_target_position(&self, axis: usize, sense: f64, max_distance: f64) -> KResult<Vec<f64>> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let curtime = self.printer.get_reactor().monotonic();
        let homed = match PrinterObject::get_status(&*toolhead, curtime) {
            Some(PyVal::Dict(d)) => d.get("homed_axes").map(|v| v.str()).unwrap_or_default(),
            _ => String::new(),
        };
        if !homed.contains('x') || !homed.contains('y') || !homed.contains('z') {
            return Err(KError::command("Must home before probe"));
        }
        let mut pos = toolhead.get_position();
        let kin_status = toolhead.get_kinematics().get_status(curtime);
        let limit = |key: &str| kin_status.get(key).and_then(|v| v.items().and_then(|l| l.get(axis)).and_then(|v| v.as_f64()));
        let (Some(axis_minimum), Some(axis_maximum)) = (limit("axis_minimum"), limit("axis_maximum")) else {
            return Err(KError::command("Tools calibrate only works with cartesian kinematics"));
        };
        if sense > 0.0 {
            pos[axis] = py_min(pos[axis] + max_distance, axis_maximum);
        } else {
            pos[axis] = py_max(pos[axis] - max_distance, axis_minimum);
        }
        Ok(pos)
    }

    fn move_to(&self, coord: &[f64], speed: f64) -> KResult<()> {
        let coord: Vec<Option<f64>> = coord.iter().map(|v| Some(*v)).collect();
        self.printer.lookup::<ToolHead>("toolhead")?.manual_move(&coord, speed)
    }

    fn calc_mean(positions: &[Position]) -> Position {
        let count = positions.len() as f64;
        let m = |i: usize| py_sum(positions.iter().map(|p| p[i])) / count;
        [m(0), m(1), m(2)]
    }

    fn calc_median(positions: &[Position], axis: usize) -> Position {
        let mut axis_sorted = positions.to_vec();
        axis_sorted.sort_by(|a, b| a[axis].partial_cmp(&b[axis]).unwrap_or(std::cmp::Ordering::Equal));
        let middle = positions.len() / 2;
        if (positions.len() & 1) == 1 {
            // odd number of samples
            return axis_sorted[middle];
        }
        // even number of samples
        Self::calc_mean(&axis_sorted[middle - 1..middle + 1])
    }

    fn run_probe(&self, dir: &str, gcmd: &GCodeCommand, speed_ratio: f64, samples: Option<i64>, max_distance: f64) -> KResult<Position> {
        let speed = gcmd.get_float("PROBE_SPEED", Some(self.speed), Lim::new().above(0.0))? * speed_ratio;
        let Some((axis, sense)) = direction(dir) else {
            return Err(KError::command("Wrong value for DIRECTION."));
        };

        logging::info(&format!("tools_calibrate: run_probe direction = {}", dir));

        logging::info(&format!("tools_calibrate: run_probe axis = {}, sense = {}", axis, sense as i64));

        let lift_speed = self.get_lift_speed(Some(gcmd))?;
        let sample_count = gcmd.get_int("SAMPLES", Some(samples.filter(|s| *s != 0).unwrap_or(self.sample_count)), Lim::new().min(1))?;
        let sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST", Some(self.sample_retract_dist), Lim::new().above(0.0))?;
        let samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE", Some(self.samples_tolerance), Lim::new().min(0.0))?;
        let samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES", Some(self.samples_retries), Lim::new().min(0))?;
        let samples_result = gcmd.get_or("SAMPLES_RESULT", &self.samples_result);

        // liftpos is probe_start (the same list), so retracts update it
        let mut probe_start = self.printer.lookup::<ToolHead>("toolhead")?.get_position();
        let mut retries = 0;
        let mut positions: Vec<Position> = vec![];
        while (positions.len() as i64) < sample_count {
            // Probe position
            let pos = self.probe(speed, axis, sense, max_distance)?;
            logging::info(&format!("tools_calibrate: run_probe result probe_start={} pos={}", list_repr(&probe_start), position_pyval(&pos).repr()));
            positions.push(pos);
            // Check samples tolerance
            let axis_positions: Vec<f64> = positions.iter().map(|p| p[axis]).collect();
            let max = axis_positions.iter().copied().fold(axis_positions[0], py_max);
            let min = axis_positions.iter().copied().fold(axis_positions[0], py_min);
            if max - min > samples_tolerance {
                if retries >= samples_retries {
                    return Err(gcmd.error("Probe samples exceed samples_tolerance"));
                }
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...", true);
                retries += 1;
                positions.clear();
            }
            // Retract
            if (positions.len() as i64) < sample_count {
                probe_start[axis] = pos[axis] - sense * sample_retract_dist;
                self.move_to(&probe_start, lift_speed)?;
            }
        }
        // Calculate and return result
        if samples_result == "median" {
            return Ok(Self::calc_median(&positions, axis));
        }
        Ok(Self::calc_mean(&positions))
    }
}

impl PinChip for PrinterProbeMultiAxis {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        let p = pin_params.borrow();
        if pin_type != "endstop" || p.pin != "xy_virtual_endstop" {
            return Err("Probe virtual endstop only useful as endstop pin".into());
        }
        if p.invert != 0 || p.pullup != 0 {
            return Err("Can not pullup/invert probe virtual endstop".into());
        }
        // python returns the list of the three axis probes
        let probes: Vec<EndstopRef> = self.mcu_probe.iter().map(|p| p.clone() as EndstopRef).collect();
        Ok(Rc::new(probes) as Rc<dyn Any>)
    }
}

// Endstop wrapper that enables probe specific features
pub struct ProbeEndstopWrapper {
    pub printer: Rc<Printer>,
    axis: char,
    pub idex: bool,
    pub mcu_endstop: Rc<McuEndstop>,
}

impl ProbeEndstopWrapper {
    fn new(config: &ConfigWrapper, axis: char) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let idex = config.has_section("dual_carriage");
        // Create an "endstop" object to handle the probe pin
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin = config.get("pin", Dflt::Required)?;
        let pin_params = {
            let mut pins = ppins.0.borrow_mut();
            pins.allow_multi_use_pin(&pin.replace(['^', '!'], "")).map_err(|e| KError::Pins(e.0))?;
            pins.lookup_pin(&pin, true, true, None).map_err(|e| KError::Pins(e.0))?
        };
        let chip = pin_params.borrow().chip.clone();
        let mcu_endstop = chip
            .setup_pin("endstop", &pin_params)
            .map_err(KError::Pins)?
            .downcast::<McuEndstop>()
            .map_err(|_| KError::Other("probe pin chip has no mcu endstop".into()))?;
        let w = Rc::new(ProbeEndstopWrapper { printer: printer.clone(), axis, idex, mcu_endstop });
        let ww = w.clone();
        crate::events::KlippyMcuIdentify::on(&printer, move || ww.handle_mcu_identify());
        Ok(w)
    }

    pub fn handle_mcu_identify(&self) -> KResult<()> {
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        for stepper in kin.get_steppers() {
            if stepper.is_active_axis(self.axis) {
                self.add_stepper(&stepper)?;
            }
        }
        Ok(())
    }
}

impl Endstop for ProbeEndstopWrapper {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_endstop.get_mcu()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.mcu_endstop.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        if self.idex && self.axis == 'x' {
            let prime_rail = self.printer.lookup::<DualCarriages>("dual_carriage").ok().and_then(|d| d.get_primary_rail());
            if let Some(prime_rail) = prime_rail {
                return prime_rail.get_rail().get_steppers();
            }
            return vec![];
        }
        Endstop::get_steppers(&*self.mcu_endstop)
    }
    fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        Endstop::home_start(&*self.mcu_endstop, print_time, sample_time, sample_count, rest_time, triggered)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        Endstop::home_wait(&*self.mcu_endstop, home_end_time)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        Endstop::query_endstop(&*self.mcu_endstop, print_time)
    }
    fn get_position_endstop(&self) -> Option<f64> {
        Some(0.0)
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ToolsCalibrate::new(config)?)
}
