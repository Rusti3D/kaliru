// Trad Rack multimaterial system support (port of klippy/extras/trad_rack.py)
//
// Copyright (C) 2022-2026 Ryan Ghosh <rghosh776@gmail.com>
// based on code by Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::itersolve::SkRef;
use crate::chelper::kin_cartesian::cartesian_stepper_alloc;
use crate::chelper::kin_extruder::extruder_stepper_alloc;
use crate::chelper::trapq::TrapQRef;
use crate::configfile::{strftime_local, ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::buttons::PrinterButtons;
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::extras::gcode_move::GCodeMove;
use crate::extras::heaters::PrinterHeaters;
use crate::extras::homing::{Homing, HomingMove, HomingToolhead};
use crate::extras::query_endstops::QueryEndstops;
use crate::extras::save_variables::SaveVariables;
use crate::extras::servo::PrinterServo;
use crate::extras::stepper_enable::PrinterStepperEnable;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::jinja::value::Value;
use crate::kinematics::extruder::PrinterExtruder;
use crate::kinematics::Kinematics;
use crate::logging;
use crate::mcu::{any_to_endstop, EndstopRef};
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float_repr, py_max, py_min, py_round};
use crate::pyglob::expanduser;
use crate::pyval::{PyDict, PyVal};
use crate::reactor::{Reactor, NOW};
use crate::stepper::{lookup_multi_rail, McuStepper, PrinterRail, Rail, RailRef};
use crate::toolhead::{Move, ToolHead, ToolHeadLimits};
use crate::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::rc::{Rc, Weak};

const SERVO_NAME: &str = "servo tr_servo";
const SELECTOR_STEPPER_NAME: &str = "stepper_tr_selector";
const FIL_DRIVER_STEPPER_NAME: &str = "stepper_tr_fil_driver";

// variables saved with save_variables
const VARS_CALIB_BOWDEN_LOAD_LENGTH: &str = "tr_calib_bowden_load_length";
const VARS_CALIB_BOWDEN_UNLOAD_LENGTH: &str = "tr_calib_bowden_unload_length";
const VARS_CONFIG_BOWDEN_LENGTH: &str = "tr_config_bowden_length";
const VARS_HEATER_TARGET: &str = "tr_last_heater_target";
const VARS_ACTIVE_LANE: &str = "tr_active_lane";

// gcode states
const GCODE_STATE_TOOLCHANGE: &str = "TR_TOOLCHANGE_STATE";

const TRADRACK_LOAD_ERROR: &str = "klippy.extras.trad_rack.TradRackLoadError";
const SELECTOR_NOT_HOMED_ERROR: &str = "klippy.extras.trad_rack.SelectorNotHomedError";

fn load_error(msg: impl Into<String>) -> KError {
    KError::CommandSub(TRADRACK_LOAD_ERROR, msg.into())
}

fn not_homed_error(msg: impl Into<String>) -> KError {
    KError::CommandSub(SELECTOR_NOT_HOMED_ERROR, msg.into())
}

fn is_class(e: &KError, class: &str) -> bool {
    matches!(e.base(), KError::CommandSub(c, _) if *c == class)
}

/// str() of an optional int (python's None / 3)
fn opt_str(v: Option<i64>) -> String {
    v.map(|v| v.to_string()).unwrap_or_else(|| "None".into())
}

/// repr() of a list of ints
fn int_list_repr(v: &[i64]) -> String {
    format!("[{}]", v.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", "))
}

/// the homing_axes tuple of ints as the toolhead's axis string
fn axes_str(axes: &[usize]) -> String {
    axes.iter().map(|a| ['x', 'y', 'z'][*a]).collect()
}

/// the fil_homing_lengths keys
#[derive(Clone, Copy)]
enum HomingLen {
    UserLoadLane = 0,
    LoadSelector = 1,
    BowdenLoad = 2,
    BowdenUnload = 3,
    UnloadToolhead = 4,
}

/// a resume callback with its arguments
#[derive(Clone)]
enum Resume {
    LoadToolhead,
    CheckCondition {
        condition: Rc<dyn Fn(&TradRack) -> KResult<bool>>,
        action: Option<Rc<dyn Fn(&TradRack) -> KResult<()>>>,
        resume_msg: String,
        fail_msg: String,
    },
    Runout,
}

/// the state of the _calibrate_selector() generator
#[derive(Clone, Copy)]
enum Calib {
    /// paused at the first yield
    AtLane0,
    /// paused at the second yield, with endstop_to_lane0
    AtLastLane(f64),
    /// exhausted (finished or raised)
    Done,
}

pub struct TradRack {
    me: Weak<TradRack>,
    pub printer: Rc<Printer>,
    pub reactor: Rc<Reactor>,
    pub spool_pull_speed: f64,
    pub buffer_pull_speed: f64,
    pub tr_toolhead: Rc<ToolHead>,
    pub tr_kinematics: Rc<TradRackKinematics>,
    pub sel_max_velocity: f64,
    pub servo: Rc<TradRackServo>,
    pub fil_driver_endstops: Vec<(EndstopRef, String)>,
    pub toolhead_fil_endstops: Vec<(EndstopRef, String)>,
    pub selector_sensor: Rc<TradRackRunoutSensor>,
    pub lane_count: i64,
    pub lane_position_manager: Rc<TradRackLanePositionManager>,
    pub lane_positions: RefCell<Vec<f64>>,
    pub bowden_load_length_filter: Rc<MovingAverageFilter>,
    pub bowden_unload_length_filter: Rc<MovingAverageFilter>,
    pub extruder_sync_manager: Rc<TradRackExtruderSyncManager>,
    pub servo_down_angle: f64,
    pub servo_up_angle: f64,
    pub servo_wait: f64,
    pub selector_unload_length: f64,
    pub selector_unload_length_extra: f64,
    pub eject_length: f64,
    pub config_bowden_length: f64,
    pub bowden_load_length: Cell<f64>,
    pub bowden_unload_length: Cell<f64>,
    pub extruder_load_length: f64,
    pub hotend_load_length: Cell<f64>,
    pub toolhead_unload_length: f64,
    pub selector_sense_speed: f64,
    pub selector_unload_speed: f64,
    pub eject_speed: f64,
    pub toolhead_sense_speed: f64,
    pub extruder_load_speed: f64,
    pub hotend_load_speed: f64,
    pub toolhead_unload_speed: f64,
    pub load_with_toolhead_sensor: bool,
    pub unload_with_toolhead_sensor: bool,
    pub fil_homing_retract_dist: f64,
    pub target_toolhead_homing_dist: f64,
    pub target_selector_homing_dist: f64,
    pub fil_homing_lengths: [f64; 5],
    pub sync_to_extruder: Cell<bool>,
    pub user_wait_time: i64,
    pub save_active_lane: bool,
    pub keep_servo_down_after_lane_load: bool,
    pub log_bowden_lengths: bool,
    // other variables
    pub toolhead: RefCell<Option<Rc<ToolHead>>>,
    pub curr_lane: Cell<Option<i64>>,
    pub active_lane: Cell<Option<i64>>,
    pub retry_lane: Cell<Option<i64>>,
    pub next_lane: Cell<Option<i64>>,
    pub next_tool: Cell<Option<i64>>,
    pub servo_raised: Cell<Option<bool>>,
    pub lanes_buffered: RefCell<Vec<bool>>,
    pub bowden_load_calibrated: Cell<bool>,
    pub bowden_unload_calibrated: Cell<bool>,
    pub bowden_load_lengths_filename: String,
    pub bowden_unload_lengths_filename: String,
    pub ignore_next_unload_length: Cell<bool>,
    pub last_heater_target: Cell<f64>,
    tr_next_generator: Cell<Option<Calib>>,
    pub selector_pos_uncertain: Cell<bool>,
    /// save_variables.allVariables as of klippy:connect (python keeps that
    /// dict; SAVE_VARIABLE replaces save_variables' own)
    pub variables: RefCell<Option<IndexMap<String, Value>>>,
    resume_stack: RefCell<VecDeque<Resume>>,
    // tool mapping
    pub lanes_dead: RefCell<Vec<bool>>,
    pub tool_map: RefCell<Vec<i64>>,
    pub default_lanes: RefCell<Vec<Option<i64>>>,
    // runout variables
    pub runout_lane: Cell<Option<i64>>,
    pub runout_steps_done: Cell<i64>,
    pub replacement_lane: Cell<Option<i64>>,
    // custom user macros
    pub pre_unload_macro: Rc<Template>,
    pub post_unload_macro: Rc<Template>,
    pub pre_load_macro: Rc<Template>,
    pub post_load_macro: Rc<Template>,
    pub pause_macro: Rc<Template>,
    pub resume_macro: Rc<Template>,
    pub gcode: Rc<GCodeDispatch>,
}

macro_rules! help {
    ($name:ident, $text:expr) => {
        const $name: &str = $text;
    };
}

help!(CMD_TR_HOME_HELP, "Home Trad Rack's selector");
help!(CMD_TR_GO_TO_LANE_HELP, "Move Trad Rack's selector to a filament lane");
help!(CMD_TR_LOAD_LANE_HELP, "Load filament from the spool into Trad Rack in the specified lane");
help!(CMD_TR_LOAD_TOOLHEAD_HELP, "Load filament from Trad Rack into the toolhead");
help!(CMD_TR_UNLOAD_TOOLHEAD_HELP, "Unload filament from the toolhead");
help!(CMD_TR_SERVO_DOWN_HELP, "Lower the servo");
help!(CMD_TR_SERVO_UP_HELP, "Raise the servo");
help!(CMD_TR_SERVO_TEST_HELP, "Test an angle for Trad Rack's servo relative to servo_down_angle");
help!(CMD_TR_SET_ACTIVE_LANE_HELP, "Set lane number that is currently loaded in the toolhead");
help!(CMD_TR_RESET_ACTIVE_LANE_HELP, "Resets active lane to None to indicate the toolhead is not loaded");
help!(CMD_TR_RESUME_HELP, "Completes necessary actions for Trad Rack to recover and resumes the print");
help!(CMD_TR_LOCATE_SELECTOR_HELP, "Ensures the position of Trad Rack's selector is known so that it is ready for a print");
help!(CMD_TR_CALIBRATE_SELECTOR_HELP, "Calibrate lane_spacing and the selector's min, endstop, and max positions");
help!(CMD_TR_NEXT_HELP, "You will be prompted to use this command if Trad Rack requires user confirmation");
help!(CMD_TR_SET_HOTEND_LOAD_LENGTH_HELP, "Sets hotend_load_length. Does not persist across restarts.");
help!(CMD_TR_DISCARD_BOWDEN_LENGTHS_HELP, "Discards saved bowden lengths and reverts them to the bowden_length config value");
help!(CMD_TR_SYNC_TO_EXTRUDER_HELP, "Sync Trad Rack's filament driver to the extruder");
help!(CMD_TR_UNSYNC_FROM_EXTRUDER_HELP, "Unsync Trad Rack's filament driver from the extruder");
help!(CMD_TR_ASSIGN_LANE_HELP, "Assign a lane to a tool");
help!(CMD_TR_SET_DEFAULT_LANE_HELP, "Set the default lane for a tool");
help!(CMD_TR_RESET_TOOL_MAP_HELP, "Reset tools assigned to each lane");
help!(CMD_TR_PRINT_TOOL_MAP_HELP, "Print tool assignment for each lane");
help!(CMD_TR_PRINT_TOOL_GROUPS_HELP, "Print lanes assigned to each tool");

impl TradRack {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let slot: Rc<RefCell<Weak<TradRack>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        crate::events::KlippyConnect::on(&printer, move || match sl.borrow().upgrade() {
            Some(t) => t.handle_connect(),
            None => Ok(()),
        });
        let sl = slot.clone();
        crate::events::KlippyReady::on(&printer, move || match sl.borrow().upgrade() {
            Some(t) => t.handle_ready(),
            None => Ok(()),
        });

        // read spool and buffer pull speeds
        let spool_pull_speed = config.getfloat("spool_pull_speed", 100.0, Lim::new().above(0.0))?;
        let buffer_pull_speed = config.getfloat("buffer_pull_speed", spool_pull_speed, Lim::new().above(0.0))?;

        // create toolhead
        let sync_slot: Rc<RefCell<Weak<TradRackExtruderSyncManager>>> = Rc::new(RefCell::new(Weak::new()));
        let ss = sync_slot.clone();
        let is_extruder_synced: Rc<dyn Fn() -> bool> =
            Rc::new(move || ss.borrow().upgrade().map(|m| m.is_extruder_synced()).unwrap_or(false));
        let (tr_toolhead, tr_kinematics) = new_tr_toolhead(config, buffer_pull_speed, is_extruder_synced)?;
        let sel_max_velocity = tr_kinematics.sel_max_velocity;

        // get servo
        let servo_obj = printer.load_as::<PrinterServo>(config, SERVO_NAME)?;
        let servo = Rc::new(TradRackServo { servo: servo_obj, toolhead: tr_toolhead.clone() });

        // get kinematics and filament driver endstops
        let fil_driver_endstops = tr_kinematics.get_fil_driver_rail().get_endstops();

        // set up toolhead filament sensor
        let mut toolhead_fil_endstops = vec![];
        if let Some(pin) = config.get_opt("toolhead_fil_sensor_pin")? {
            // register endstop
            let obj = crate::pins::setup_pin(&printer, "endstop", &pin)?;
            let mcu_endstop = any_to_endstop(obj).ok_or_else(|| KError::Other("endstop pin of an unexpected type".into()))?;
            let name = "toolhead_fil_sensor";
            toolhead_fil_endstops.push((mcu_endstop.clone(), name.to_string()));
            let query_endstops = printer.load_as::<QueryEndstops>(config, "query_endstops")?;
            query_endstops.register_endstop(mcu_endstop.clone(), name);

            // add filament driver stepper to endstop
            for stepper in tr_kinematics.get_fil_driver_rail().get_steppers() {
                mcu_endstop.add_stepper(&stepper)?;
            }
        }

        // set up selector sensor as a runout sensor
        let pin = config.getsection(FIL_DRIVER_STEPPER_NAME).get("endstop_pin", Dflt::Required)?;
        let sl = slot.clone();
        let selector_sensor = TradRackRunoutSensor::new(
            config,
            Rc::new(move |eventtime| match sl.borrow().upgrade() {
                Some(t) => t.handle_runout(eventtime),
                None => Ok(()),
            }),
            &pin,
        )?;

        // read lane count and get lane positions
        let lane_count = config.getint("lane_count", Dflt::Required, Lim::new().min(2))?;
        let lane_position_manager = TradRackLanePositionManager::new(lane_count, config)?;
        let lane_positions = lane_position_manager.get_lane_positions();

        // create bowden length filters
        let bowden_samples = config.getint("bowden_length_samples", 10, Lim::new().min(1))?;

        // create extruder sync manager
        let extruder_sync_manager = TradRackExtruderSyncManager::new(&printer, &tr_toolhead, tr_kinematics.get_fil_driver_rail());
        *sync_slot.borrow_mut() = Rc::downgrade(&extruder_sync_manager);

        // read other values
        let servo_down_angle = config.getfloat("servo_down_angle", Dflt::Required, Lim::new())?;
        let servo_up_angle = config.getfloat("servo_up_angle", Dflt::Required, Lim::new())?;
        let servo_wait = config.getfloat("servo_wait_ms", 500.0, Lim::new().above(0.0))? / 1000.0;
        let selector_unload_length = config.getfloat("selector_unload_length", Dflt::Required, Lim::new().above(0.0))?;
        let selector_unload_length_extra = config.getfloat("selector_unload_length_extra", 0.0, Lim::new().min(0.0))?;
        let eject_length = config.getfloat("eject_length", 30.0, Lim::new().above(0.0))?;
        let config_bowden_length = config.getfloat("bowden_length", Dflt::Required, Lim::new().above(0.0))?;
        let extruder_load_length = config.getfloat("extruder_load_length", Dflt::Required, Lim::new().above(0.0))?;
        let hotend_load_length = config.getfloat("hotend_load_length", Dflt::Required, Lim::new().above(0.0))?;
        let toolhead_unload_length = if !toolhead_fil_endstops.is_empty() {
            config.getfloat("toolhead_unload_length", Dflt::Required, Lim::new().min(0.0))?
        } else {
            config.getfloat("toolhead_unload_length", extruder_load_length + hotend_load_length, Lim::new().above(0.0))?
        };
        let selector_sense_speed = config.getfloat("selector_sense_speed", 40.0, Lim::new().above(0.0))?;
        let selector_unload_speed = config.getfloat("selector_unload_speed", 60.0, Lim::new().above(0.0))?;
        let eject_speed = config.getfloat("eject_speed", 80.0, Lim::new().above(0.0))?;
        let toolhead_sense_speed = config.getfloat("toolhead_sense_speed", selector_sense_speed, Lim::new().above(0.0))?;
        let extruder_load_speed = config.getfloat("extruder_load_speed", 60.0, Lim::new().above(0.0))?;
        let hotend_load_speed = config.getfloat("hotend_load_speed", 7.0, Lim::new().above(0.0))?;
        let toolhead_unload_speed = config.getfloat("toolhead_unload_speed", extruder_load_speed, Lim::new().above(0.0))?;
        let load_with_toolhead_sensor = config.getboolean("load_with_toolhead_sensor", true)?;
        let unload_with_toolhead_sensor = config.getboolean("unload_with_toolhead_sensor", true)?;
        let fil_homing_retract_dist = config.getfloat("fil_homing_retract_dist", 20.0, Lim::new().min(0.0))?;
        let target_toolhead_homing_dist =
            config.getfloat("target_toolhead_homing_dist", py_max(10.0, toolhead_unload_length), Lim::new().above(0.0))?;
        let target_selector_homing_dist = config.getfloat("target_selector_homing_dist", 10.0, Lim::new().above(0.0))?;
        let fil_homing_lengths = [
            config.getint("load_lane_time", 15, Lim::new().min(selector_unload_length / selector_sense_speed))? as f64 * selector_sense_speed,
            config.getfloat("load_selector_homing_dist", selector_unload_length * 2.0, Lim::new().above(selector_unload_length))?,
            config.getfloat("bowden_load_homing_dist", config_bowden_length, Lim::new().above(target_toolhead_homing_dist))?,
            config.getfloat("bowden_unload_homing_dist", config_bowden_length, Lim::new().above(target_selector_homing_dist))?,
            config.getfloat("unload_toolhead_homing_dist", (extruder_load_length + hotend_load_length) * 2.0, Lim::new().above(0.0))?,
        ];
        let sync_to_extruder = config.getboolean("sync_to_extruder", false)?;
        let user_wait_time = config.getint("user_wait_time", 15, Lim::new().min(-1))?;
        let register_toolchange_commands = config.getboolean("register_toolchange_commands", true)?;
        let save_active_lane = config.getboolean("save_active_lane", true)?;
        let keep_servo_down_after_lane_load = config.getboolean("keep_servo_down_after_lane_load", false)?;
        let log_bowden_lengths = config.getboolean("log_bowden_lengths", false)?;

        // custom user macros
        let gcode_macro = load_gcode_macro(config)?;
        let pre_unload_macro = gcode_macro.load_template(config, "pre_unload_gcode", Some(""))?;
        let post_unload_macro = gcode_macro.load_template(config, "post_unload_gcode", Some(""))?;
        let pre_load_macro = gcode_macro.load_template(config, "pre_load_gcode", Some(""))?;
        let post_load_macro = gcode_macro.load_template(config, "post_load_gcode", Some(""))?;
        let pause_macro = gcode_macro.load_template(config, "pause_gcode", Some("PAUSE"))?;
        let resume_macro = gcode_macro.load_template(config, "resume_gcode", Some("RESUME"))?;

        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let n = lane_count.max(0) as usize;
        let tr = Rc::new_cyclic(|me| TradRack {
            me: me.clone(),
            printer: printer.clone(),
            reactor,
            spool_pull_speed,
            buffer_pull_speed,
            tr_toolhead,
            tr_kinematics,
            sel_max_velocity,
            servo,
            fil_driver_endstops,
            toolhead_fil_endstops,
            selector_sensor,
            lane_count,
            lane_position_manager: Rc::new(lane_position_manager),
            lane_positions: RefCell::new(lane_positions),
            bowden_load_length_filter: Rc::new(MovingAverageFilter::new(bowden_samples as usize)),
            bowden_unload_length_filter: Rc::new(MovingAverageFilter::new(bowden_samples as usize)),
            extruder_sync_manager,
            servo_down_angle,
            servo_up_angle,
            servo_wait,
            selector_unload_length,
            selector_unload_length_extra,
            eject_length,
            config_bowden_length,
            bowden_load_length: Cell::new(config_bowden_length),
            bowden_unload_length: Cell::new(config_bowden_length),
            extruder_load_length,
            hotend_load_length: Cell::new(hotend_load_length),
            toolhead_unload_length,
            selector_sense_speed,
            selector_unload_speed,
            eject_speed,
            toolhead_sense_speed,
            extruder_load_speed,
            hotend_load_speed,
            toolhead_unload_speed,
            load_with_toolhead_sensor,
            unload_with_toolhead_sensor,
            fil_homing_retract_dist,
            target_toolhead_homing_dist,
            target_selector_homing_dist,
            fil_homing_lengths,
            sync_to_extruder: Cell::new(sync_to_extruder),
            user_wait_time,
            save_active_lane,
            keep_servo_down_after_lane_load,
            log_bowden_lengths,
            toolhead: RefCell::new(None),
            curr_lane: Cell::new(None),
            active_lane: Cell::new(None),
            retry_lane: Cell::new(None),
            next_lane: Cell::new(None),
            next_tool: Cell::new(None),
            servo_raised: Cell::new(None),
            lanes_buffered: RefCell::new(vec![false; n]),
            bowden_load_calibrated: Cell::new(false),
            bowden_unload_calibrated: Cell::new(false),
            bowden_load_lengths_filename: expanduser("~/bowden_load_lengths.csv"),
            bowden_unload_lengths_filename: expanduser("~/bowden_unload_lengths.csv"),
            ignore_next_unload_length: Cell::new(false),
            last_heater_target: Cell::new(0.0),
            tr_next_generator: Cell::new(None),
            selector_pos_uncertain: Cell::new(false),
            variables: RefCell::new(None),
            resume_stack: RefCell::new(VecDeque::new()),
            lanes_dead: RefCell::new(vec![false; n]),
            tool_map: RefCell::new(vec![]),
            default_lanes: RefCell::new(vec![]),
            runout_lane: Cell::new(None),
            runout_steps_done: Cell::new(0),
            replacement_lane: Cell::new(None),
            pre_unload_macro,
            post_unload_macro,
            pre_load_macro,
            post_load_macro,
            pause_macro,
            resume_macro,
            gcode: gcode.clone(),
        });
        tr.reset_tool_map();
        *slot.borrow_mut() = Rc::downgrade(&tr);

        // register gcode commands
        type TCmd = fn(&TradRack, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, TCmd, &str); 23] = [
            ("TR_HOME", TradRack::cmd_tr_home, CMD_TR_HOME_HELP),
            ("TR_GO_TO_LANE", TradRack::cmd_tr_go_to_lane, CMD_TR_GO_TO_LANE_HELP),
            ("TR_LOAD_LANE", TradRack::cmd_tr_load_lane, CMD_TR_LOAD_LANE_HELP),
            ("TR_LOAD_TOOLHEAD", TradRack::cmd_tr_load_toolhead_default, CMD_TR_LOAD_TOOLHEAD_HELP),
            ("TR_UNLOAD_TOOLHEAD", TradRack::cmd_tr_unload_toolhead, CMD_TR_UNLOAD_TOOLHEAD_HELP),
            ("TR_SERVO_DOWN", TradRack::cmd_tr_servo_down, CMD_TR_SERVO_DOWN_HELP),
            ("TR_SERVO_UP", TradRack::cmd_tr_servo_up, CMD_TR_SERVO_UP_HELP),
            ("TR_SERVO_TEST", TradRack::cmd_tr_servo_test, CMD_TR_SERVO_TEST_HELP),
            ("TR_SET_ACTIVE_LANE", TradRack::cmd_tr_set_active_lane, CMD_TR_SET_ACTIVE_LANE_HELP),
            ("TR_RESET_ACTIVE_LANE", TradRack::cmd_tr_reset_active_lane, CMD_TR_RESET_ACTIVE_LANE_HELP),
            ("TR_RESUME", TradRack::cmd_tr_resume, CMD_TR_RESUME_HELP),
            ("TR_LOCATE_SELECTOR", TradRack::cmd_tr_locate_selector, CMD_TR_LOCATE_SELECTOR_HELP),
            ("TR_CALIBRATE_SELECTOR", TradRack::cmd_tr_calibrate_selector, CMD_TR_CALIBRATE_SELECTOR_HELP),
            ("TR_NEXT", TradRack::cmd_tr_next, CMD_TR_NEXT_HELP),
            ("TR_SET_HOTEND_LOAD_LENGTH", TradRack::cmd_tr_set_hotend_load_length, CMD_TR_SET_HOTEND_LOAD_LENGTH_HELP),
            ("TR_DISCARD_BOWDEN_LENGTHS", TradRack::cmd_tr_discard_bowden_lengths, CMD_TR_DISCARD_BOWDEN_LENGTHS_HELP),
            ("TR_SYNC_TO_EXTRUDER", TradRack::cmd_tr_sync_to_extruder, CMD_TR_SYNC_TO_EXTRUDER_HELP),
            ("TR_UNSYNC_FROM_EXTRUDER", TradRack::cmd_tr_unsync_from_extruder, CMD_TR_UNSYNC_FROM_EXTRUDER_HELP),
            ("TR_ASSIGN_LANE", TradRack::cmd_tr_assign_lane, CMD_TR_ASSIGN_LANE_HELP),
            ("TR_SET_DEFAULT_LANE", TradRack::cmd_tr_set_default_lane, CMD_TR_SET_DEFAULT_LANE_HELP),
            ("TR_RESET_TOOL_MAP", TradRack::cmd_tr_reset_tool_map, CMD_TR_RESET_TOOL_MAP_HELP),
            ("TR_PRINT_TOOL_MAP", TradRack::cmd_tr_print_tool_map, CMD_TR_PRINT_TOOL_MAP_HELP),
            ("TR_PRINT_TOOL_GROUPS", TradRack::cmd_tr_print_tool_groups, CMD_TR_PRINT_TOOL_GROUPS_HELP),
        ];
        for (name, f, desc) in cmds {
            let w = tr.me.clone();
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(t) => f(&t, g),
                    None => Ok(()),
                })),
                false,
                Some(desc),
            )?;
        }
        if register_toolchange_commands {
            for i in 0..lane_count {
                let w = tr.me.clone();
                let desc = format!("Load filament from Trad Rack into the toolhead from tool {}", i);
                gcode.register_command(
                    &format!("T{}", i),
                    Some(Rc::new(move |g| match w.upgrade() {
                        Some(t) => {
                            t.gcode.get_extended_params(g)?;
                            t.cmd_tr_load_toolhead(g, Some(i))
                        }
                        None => Ok(()),
                    })),
                    false,
                    Some(&desc),
                )?;
            }
        }
        Ok(tr)
    }

    fn rc(&self) -> Rc<TradRack> {
        self.me.upgrade().expect("trad_rack")
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.toolhead.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'wait_moves'".into()))
    }

    pub fn handle_connect(&self) -> KResult<()> {
        *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);
        let save_variables = self.printer.lookup_object_opt("save_variables").and_then(crate::printer::downcast_rc::<SaveVariables>);
        let Some(save_variables) = save_variables else {
            return Err(KError::config("[save_variables] is required for trad_rack"));
        };
        *self.variables.borrow_mut() = Some(save_variables.all_variables.borrow().clone());
        Ok(())
    }

    pub fn handle_ready(&self) -> KResult<()> {
        self.load_saved_state()
    }

    fn var(&self, name: &str) -> Option<Value> {
        self.variables.borrow().as_ref().and_then(|v| v.get(name).cloned())
    }

    fn save_variable(&self, name: &str, value: &str) -> KResult<()> {
        self.gcode.run_script_from_command(&format!("SAVE_VARIABLE VARIABLE={} VALUE=\"{}\"", name, value))
    }

    pub fn load_saved_state(&self) -> KResult<()> {
        // load bowden lengths if the user has not changed the config value
        let prev_config_bowden_length = self.var(VARS_CONFIG_BOWDEN_LENGTH);
        let prev_truthy = prev_config_bowden_length.as_ref().map(|v| v.truthy()).unwrap_or(false);
        let prev_equal = prev_config_bowden_length.as_ref().and_then(|v| v.as_f64()).map(|v| v == self.config_bowden_length).unwrap_or(false);
        if prev_truthy && prev_equal {
            // update load length
            for (name, length, filter) in [
                (VARS_CALIB_BOWDEN_LOAD_LENGTH, &self.bowden_load_length, &self.bowden_load_length_filter),
                (VARS_CALIB_BOWDEN_UNLOAD_LENGTH, &self.bowden_unload_length, &self.bowden_unload_length_filter),
            ] {
                let stats = self.var(name);
                if stats.as_ref().map(|v| v.truthy()).unwrap_or(false) {
                    let stats = stats.unwrap();
                    let get = |k: &str| match &stats {
                        Value::Dict(d) => d.borrow().get(&crate::jinja::value::Key::str(k)).cloned().ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(k))),
                        _ => Err(KError::Other(format!("{} indices must be integers", stats.type_name()))),
                    };
                    let new_set_length = get("new_set_length")?.as_f64().unwrap_or(0.0);
                    length.set(new_set_length);
                    let count = get("sample_count")?.as_f64().unwrap_or(0.0) as i64;
                    for _ in 0..count {
                        filter.update(new_set_length);
                    }
                }
            }
        } else {
            // save bowden_length config value
            self.save_variable(VARS_CONFIG_BOWDEN_LENGTH, &py_float_repr(self.config_bowden_length))?;
        }

        // load last heater target
        self.last_heater_target.set(self.var(VARS_HEATER_TARGET).and_then(|v| v.as_f64()).unwrap_or(0.0));
        Ok(())
    }

    pub fn handle_runout(&self, _eventtime: f64) -> KResult<()> {
        // send pause command
        let pause_resume = self.printer.lookup::<crate::extras::pause_resume::PauseResume>("pause_resume")?;
        pause_resume.send_pause_command()?;

        // set up resume callback and run pause gcode
        self.set_up_resume_and_pause(Resume::Runout)?;

        // note runout
        let runout_lane = self.active_lane.get();
        self.runout_lane.set(runout_lane);
        self.set_active_lane(None)?;
        let rl = runout_lane.ok_or_else(|| KError::Other("list indices must be integers or slices, not NoneType".into()))?;
        self.lanes_buffered.borrow_mut()[rl as usize] = false;
        self.lanes_dead.borrow_mut()[rl as usize] = true;
        self.gcode.respond_info(&format!("Runout detected at selector on lane {} (tool {})", rl, self.tool_map.borrow()[rl as usize]), true);

        // unload filament and reload from a new lane
        self.runout_steps_done.set(0);
        let g = self.gcode.create_gcode_command("TR_RESUME", "TR_RESUME", IndexMap::default());
        self.cmd_tr_resume(&g)
    }

    // gcode commands

    pub fn cmd_tr_home(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // check for filament in the selector
        if self.query_selector_sensor()? {
            return Err(KError::command("Cannot home with filament in selector"));
        }

        // reset current lane
        self.curr_lane.set(None);

        // raise servo
        self.raise_servo(false, true, true, None)?;

        // home selector
        let homing_state = Homing::with_toolhead(&self.printer, self.tr_toolhead.clone());
        homing_state.set_axes(vec![0]);
        if let Err(e) = self.tr_kinematics.home(&homing_state) {
            if !e.is_command_error() {
                return Err(e);
            }
            if self.printer.is_shutdown() {
                return Err(KError::command("Homing failed due to printer shutdown"));
            }
            let print_time = self.tr_toolhead.get_last_move_time()?;
            let stepper_enable = self.printer.lookup::<PrinterStepperEnable>("stepper_enable")?;
            stepper_enable.lookup_enable(SELECTOR_STEPPER_NAME)?.motor_disable(print_time)?;
            return Err(e);
        }

        // unmark selector position as uncertain
        self.selector_pos_uncertain.set(false);
        Ok(())
    }

    pub fn cmd_tr_go_to_lane(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.go_to_lane(gcmd.get_int_opt("LANE", Lim::new())?)
    }

    pub fn cmd_tr_load_lane(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let lane = gcmd.get_int_opt("LANE", Lim::new())?;
        self.load_lane(lane, gcmd.get_int("RESET_SPEED", Some(1), Lim::new())? != 0, true)?;
        let lane = lane.ok_or_else(|| KError::Other("list indices must be integers or slices, not NoneType".into()))?;
        self.lanes_dead.borrow_mut()[lane as usize] = false;
        Ok(())
    }

    fn cmd_tr_load_toolhead_default(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.cmd_tr_load_toolhead(gcmd, None)
    }

    pub fn cmd_tr_load_toolhead(&self, gcmd: &GCodeCommand, tool_override: Option<i64>) -> KResult<()> {
        let start_lane = self.active_lane.get();
        let mut lane = gcmd.get_int_opt("LANE", Lim::new())?;
        let tool = match tool_override {
            Some(t) => Some(t),
            None => gcmd.get_int_opt("TOOL", Lim::new())?,
        };

        // select lane
        if lane.is_none() {
            // check tool
            let t = self.check_tool_valid(tool)?;

            // get default lane for the selected tool
            lane = self.default_lanes.borrow()[t as usize];
            if lane.is_none() {
                gcmd.respond_info(
                    &format!(
                        "Tool {tool} has no lanes assigned to it. Use TR_ASSIGN_LANE LANE=&lt;lane index&gt; TOOL={tool} to assign a lane to tool {tool}, then use TR_RESUME to continue.",
                        tool = t
                    ),
                    true,
                );

                // set up resume callback and pause the print
                // (and wait for user to resume)
                let g = gcmd.clone();
                self.set_up_resume_and_pause(Resume::CheckCondition {
                    condition: Rc::new(move |tr: &TradRack| Ok(tr.default_lanes.borrow()[t as usize].is_some())),
                    action: Some(Rc::new(move |tr: &TradRack| tr.cmd_tr_load_toolhead(&g, tool_override))),
                    resume_msg: "Resuming print".into(),
                    fail_msg: format!("Cannot resume. Please use TR_ASSIGN_LANE to assign a lane to tool {}, then use TR_RESUME.", t),
                })?;
                return Ok(());
            }
        }

        // load toolhead (the gcode values fill the positional arguments
        // after exact_temp: selector_already_loaded=BOWDEN_LENGTH,
        // bowden_length=EXTRUDER_LOAD_LENGTH, extruder_load_length=
        // HOTEND_LOAD_LENGTH, as python passes them)
        let min_temp = gcmd.get_float("MIN_TEMP", Some(0.0), Lim::new().min(0.0))?;
        let exact_temp = gcmd.get_float("EXACT_TEMP", Some(0.0), Lim::new().min(0.0))?;
        let bowden = gcmd.get_float_opt("BOWDEN_LENGTH", Lim::new().min(0.0))?;
        let extruder_load = gcmd.get_float_opt("EXTRUDER_LOAD_LENGTH", Lim::new().min(0.0))?;
        let hotend_load = gcmd.get_float_opt("HOTEND_LOAD_LENGTH", Lim::new().min(0.0))?;
        let selector_already_loaded = bowden.map(|b| b != 0.0).unwrap_or(false);
        match self.load_toolhead(lane, tool, min_temp, exact_temp, selector_already_loaded, extruder_load, hotend_load, None) {
            Ok(()) => Ok(()),
            Err(e) if is_class(&e, TRADRACK_LOAD_ERROR) => {
                logging::warning_err(&format!("trad_rack: Toolchange from lane {} to {} failed", opt_str(start_lane), opt_str(lane)), &e);
                // set up resume callback and pause the print
                // (and wait for user to resume)
                self.set_up_resume_and_pause(Resume::LoadToolhead)
            }
            Err(e) if is_class(&e, SELECTOR_NOT_HOMED_ERROR) => {
                gcmd.respond_info(
                    "Selector not homed. Use TR_LOCATE_SELECTOR (or TR_HOME to home the selector directly), then use TR_RESUME to continue.",
                    true,
                );
                // set up resume callback and pause the print
                // (and wait for user to resume)
                let g = gcmd.clone();
                self.set_up_resume_and_pause(Resume::CheckCondition {
                    condition: Rc::new(|tr: &TradRack| tr.is_selector_homed()),
                    action: Some(Rc::new(move |tr: &TradRack| tr.cmd_tr_load_toolhead(&g, tool_override))),
                    resume_msg: "Resuming print".into(),
                    fail_msg: "Cannot resume. Please use either TR_LOCATE_SELECTOR or TR_HOME to home the selector, then use TR_RESUME.".into(),
                })
            }
            Err(e) => Err(e),
        }
    }

    pub fn cmd_tr_unload_toolhead(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // unload toolhead
        let r = (|| {
            let min_temp = gcmd.get_float("MIN_TEMP", Some(0.0), Lim::new().min(0.0))?;
            let exact_temp = gcmd.get_float("EXACT_TEMP", Some(0.0), Lim::new().min(0.0))?;
            self.unload_toolhead(min_temp, exact_temp, false, false, false)
        })();
        // finally: reset lane speed (in case the user is removing/swapping the spool)
        let fin = (|| {
            if gcmd.get_int("RESET_SPEED", Some(1), Lim::new())? != 0 {
                if let Some(cl) = self.curr_lane.get() {
                    self.lanes_buffered.borrow_mut()[cl as usize] = false;
                }
            }
            Ok(())
        })();
        r?;
        fin
    }

    pub fn cmd_tr_servo_down(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // check that the selector is at a lane
        if gcmd.get_int("FORCE", Some(0), Lim::new())? == 0 && !self.can_lower_servo()? {
            return Err(KError::command("Selector must be moved to a lane before lowering the servo"));
        }
        // lower servo
        self.lower_servo(false)
    }

    pub fn cmd_tr_servo_up(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // raise servo
        self.raise_servo(false, true, true, None)
    }

    pub fn cmd_tr_servo_test(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // get commanded and raw angles
        let cmd_angle = gcmd.get_float("ANGLE", Some((self.servo_up_angle - self.servo_down_angle).abs()), Lim::new())?;
        let up = self.servo_up_angle > self.servo_down_angle;
        let raw_angle = if up { self.servo_down_angle + cmd_angle } else { self.servo_down_angle - cmd_angle };
        let raw_to_cmd = |raw: f64| if up { raw - self.servo_down_angle } else { self.servo_down_angle - raw };

        // display angles
        gcmd.respond_info(&format!("commanded angle: {:.3}\nraw angle: {:.3}", cmd_angle, raw_angle), true);

        // check raw angle
        let max_angle = self.servo.get_max_angle();
        if raw_angle > max_angle {
            return Err(KError::command(format!(
                "Raw angle is above the maximum of {:.3} (corresponding to a commanded angle of {:.3}). If the servo is not rotating far enough, try increasing maximum_pulse_width in the [{}] section in the config file.",
                max_angle,
                raw_to_cmd(max_angle),
                SERVO_NAME
            )));
        } else if raw_angle < 0.0 {
            return Err(KError::command(format!(
                "Raw angle is below the minimum of 0.0 (corresponding to a commanded angle of {:.3}). If the servo is not rotating far enough, try decreasing minimum_pulse_width in the [{}] section in the config file.",
                raw_to_cmd(0.0),
                SERVO_NAME
            )));
        }

        // set servo
        self.servo.set_servo(None, Some(raw_angle), None)
    }

    pub fn cmd_tr_set_active_lane(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // get lane
        let lane = gcmd.get_int_opt("LANE", Lim::new())?;

        // check lane
        let lane = self.check_lane_valid(lane)?;

        // check for filament in the selector
        if !self.query_selector_sensor()? {
            return Err(KError::command("Cannot set active lane without filament in selector"));
        }

        // get current selector position and lane position
        let print_time = self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        let lane_pos = self.lane_positions.borrow()[lane as usize];

        // mark selector position as uncertain if not homed or current position
        // doesn't match lane position
        if !(self.is_selector_homed()? && pos[0] == lane_pos) {
            self.selector_pos_uncertain.set(true);
        }

        // set selector position
        pos[0] = lane_pos;
        self.tr_toolhead.set_position(&pos, &axes_str(&[0]))?;
        let stepper_enable = self.printer.lookup::<PrinterStepperEnable>("stepper_enable")?;
        stepper_enable.lookup_enable(SELECTOR_STEPPER_NAME)?.motor_enable(print_time)?;

        // set current lane and active lane
        self.curr_lane.set(Some(lane));
        self.set_active_lane(Some(lane))?;

        // reset next lane and tool if there is no longer a pending toolchange
        if self.is_next_toolchange_done() {
            self.next_lane.set(None);
            self.next_tool.set(None);
        }

        // restore extruder sync
        self.restore_extruder_sync()?;

        // make lane the new default for its assigned tool
        self.make_lane_default(lane);

        // enable runout detection
        self.selector_sensor.set_active(true);

        // notify active lane was set without loading the toolhead
        crate::events::TradRackForcedActiveLane::emit(&self.printer)
    }

    pub fn cmd_tr_reset_active_lane(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.set_active_lane(None)?;
        self.raise_servo(false, true, true, None)?;
        self.extruder_sync_manager.unsync()?;
        self.selector_sensor.set_active(false);
        crate::events::TradRackResetActiveLane::emit(&self.printer)
    }

    pub fn cmd_tr_resume(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let mut resume_msg = None;

        // loop through all resumes in the stack
        loop {
            // pop the last resume
            let Some(resume) = self.resume_stack.borrow_mut().pop_back() else { break };
            let curr_stack_size = self.resume_stack.borrow().len();

            // run the resume callback
            let (retry_resume, msg) = match self.run_resume(&resume) {
                Ok(r) => r,
                Err(e) => {
                    // if the resume callback raised an error, add the resume back to
                    // the stack
                    if e.is_command_error() {
                        self.resume_stack.borrow_mut().push_back(resume);
                    }
                    return Err(e);
                }
            };
            resume_msg = msg;

            // if a new resume was set up by the callback, return since the user
            // needs to handle that and call TR_RESUME again
            if self.resume_stack.borrow().len() > curr_stack_size {
                return Ok(());
            }

            // if the resume needs to be retried, add it back to the stack and
            // wait for the user to call TR_RESUME again
            if retry_resume {
                self.resume_stack.borrow_mut().push_back(resume);
                return Ok(());
            }
        }

        // resume the print
        self.send_resume(resume_msg.as_deref())
    }

    pub fn cmd_tr_locate_selector(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.query_selector_sensor()? {
            if self.active_lane.get().is_none() && self.save_active_lane {
                // set active lane if a valid lane was saved
                let saved = self.var(VARS_ACTIVE_LANE).and_then(|v| match v {
                    Value::Int(i) => Some(i),
                    _ => None,
                });
                if let Ok(l) = self.check_lane_valid(saved) {
                    self.active_lane.set(Some(l));
                }
            }

            match self.active_lane.get() {
                None => {
                    // ask user to set the active lane or unload the lane
                    gcmd.respond_info(
                        "Selector sensor is triggered but no active lane is set. Please use TR_SET_ACTIVE_LANE LANE=&lt;lane index&gt; if the toolhead is already loaded, else use TR_UNLOAD_TOOLHEAD to unload the filament. Then use TR_RESUME to resume the print.",
                        true,
                    );
                    self.ignore_next_unload_length.set(true);

                    // set up callback to run if user takes no action
                    if self.user_wait_time != -1 {
                        gcmd.respond_info(
                            &format!(
                                "If no action is taken within {} seconds, Trad Rack will unload the toolhead and resume automatically.",
                                self.user_wait_time
                            ),
                            true,
                        );
                        let w = self.me.clone();
                        RunIfNoActivity::start(
                            &self.tr_toolhead,
                            &self.reactor,
                            Rc::new(move || match w.upgrade() {
                                Some(t) => t.unload_toolhead_and_resume(),
                                None => Ok(()),
                            }),
                            self.user_wait_time as f64,
                        )?;
                    }

                    // set up resume callback and pause the print
                    // (and wait for user to resume)
                    self.set_up_resume_and_pause(Resume::CheckCondition {
                        condition: Rc::new(|tr: &TradRack| Ok(tr.active_lane.get().is_some() || !tr.query_selector_sensor()?)),
                        action: Some(Rc::new(|tr: &TradRack| tr.resume_act_locate_selector())),
                        resume_msg: "Resuming print".into(),
                        fail_msg: "Cannot resume. Please use either TR_SET_ACTIVE_LANE or TR_UNLOAD_TOOLHEAD, then use TR_RESUME.".into(),
                    })?;
                }
                Some(active) => {
                    // (if the selector is homed, nothing needs to be done)
                    if !self.is_selector_homed()? {
                        // set selector position and enable motor
                        // (this allows printing again without reloading the
                        // toolhead if the motor was disabled after the last print)
                        let mut params = IndexMap::default();
                        params.insert("LANE".into(), active.to_string());
                        let g = self.gcode.create_gcode_command("TR_SET_ACTIVE_LANE", "TR_SET_ACTIVE_LANE", params);
                        self.cmd_tr_set_active_lane(&g)?;
                        gcmd.respond_info(&format!("Set lane {} as the active lane", active), true);
                    }
                }
            }
        } else {
            self.set_active_lane(None)?;
            self.selector_sensor.set_active(false);
            if !self.is_selector_homed()? {
                self.home()?;
            }
        }
        Ok(())
    }

    /// cmd_TR_HOME(create_gcode_command("TR_HOME", "TR_HOME", {}))
    fn home(&self) -> KResult<()> {
        let g = self.gcode.create_gcode_command("TR_HOME", "TR_HOME", IndexMap::default());
        self.cmd_tr_home(&g)
    }

    pub fn cmd_tr_calibrate_selector(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.tr_next_generator.set(Some(Calib::AtLane0));
        // next(generator): runs to the first yield
        if let Err(e) = self.prompt_selector_calibration(0) {
            self.tr_next_generator.set(Some(Calib::Done));
            return Err(e);
        }
        Ok(())
    }

    pub fn cmd_tr_next(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let Some(state) = self.tr_next_generator.get() else {
            return Err(KError::command("TR_NEXT command is inactive"));
        };
        match self.calibrate_selector_step(state) {
            // the generator yielded again
            Ok(Some(next)) => {
                self.tr_next_generator.set(Some(next));
                Ok(())
            }
            // StopIteration
            Ok(None) => {
                self.tr_next_generator.set(None);
                Ok(())
            }
            Err(e) => {
                self.tr_next_generator.set(None);
                Err(e)
            }
        }
    }

    pub fn cmd_tr_set_hotend_load_length(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let value = match gcmd.get_float_opt("VALUE", Lim::new().min(0.0))? {
            Some(v) => v,
            None => {
                let Some(adjust) = gcmd.get_float_opt("ADJUST", Lim::new())? else {
                    return Err(KError::command("VALUE or ADJUST must be specified"));
                };
                py_max(0.0, self.hotend_load_length.get() + adjust)
            }
        };
        self.hotend_load_length.set(value);
        gcmd.respond_info(&format!("hotend_load_length: {:.6}", self.hotend_load_length.get()), true);
        Ok(())
    }

    pub fn cmd_tr_discard_bowden_lengths(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mode = gcmd.get_or("MODE", "ALL").to_uppercase();
        if !["ALL", "LOAD", "UNLOAD"].contains(&mode.as_str()) {
            return Err(gcmd.error(format!("Invalid MODE: {}", mode)));
        }
        // discard load length
        if mode == "ALL" || mode == "LOAD" {
            self.bowden_load_length.set(self.config_bowden_length);
            self.bowden_load_length_filter.reset();
            self.save_variable(VARS_CALIB_BOWDEN_LOAD_LENGTH, "{}")?;
        }
        // discard unload length
        if mode == "ALL" || mode == "UNLOAD" {
            self.bowden_unload_length.set(self.config_bowden_length);
            self.bowden_unload_length_filter.reset();
            self.save_variable(VARS_CALIB_BOWDEN_UNLOAD_LENGTH, "{}")?;
        }
        Ok(())
    }

    pub fn cmd_tr_sync_to_extruder(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.toolhead()?.wait_moves()?;
        self.sync_to_extruder.set(true);
        self.restore_extruder_sync()
    }

    pub fn cmd_tr_unsync_from_extruder(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.toolhead()?.wait_moves()?;
        self.sync_to_extruder.set(false);
        self.restore_extruder_sync()
    }

    pub fn cmd_tr_assign_lane(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let lane = gcmd.get_int_opt("LANE", Lim::new())?;
        let tool = gcmd.get_int_opt("TOOL", Lim::new())?;
        // check lane and tool
        let lane = self.check_lane_valid(lane)?;
        let tool = self.check_tool_valid(tool)?;
        // assign lane
        self.assign_lane(lane, tool);
        // mark lane as not dead
        self.lanes_dead.borrow_mut()[lane as usize] = false;
        // make lane the new default for the tool
        if gcmd.get_int("SET_DEFAULT", Some(0), Lim::new())? != 0 {
            self.default_lanes.borrow_mut()[tool as usize] = Some(lane);
        }
        Ok(())
    }

    pub fn cmd_tr_set_default_lane(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let lane = gcmd.get_int_opt("LANE", Lim::new())?;
        let tool = gcmd.get_int_opt("TOOL", Lim::new())?;
        // check lane
        let lane = self.check_lane_valid(lane)?;
        match tool {
            // set lane as default for its currently-assigned tool
            None => self.make_lane_default(lane),
            Some(_) => {
                // check tool
                let tool = self.check_tool_valid(tool)?;
                // set lane as default for the tool
                self.set_default_lane(tool, Some(lane));
            }
        }
        Ok(())
    }

    pub fn cmd_tr_reset_tool_map(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.reset_tool_map();
        Ok(())
    }

    pub fn cmd_tr_print_tool_map(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let num_chars = (self.lane_count - 1).to_string().len();
        let mut lane_msg = "|Lane: |".to_string();
        let mut tool_msg = "|Tool: |".to_string();
        let tool_map = self.tool_map.borrow();
        for lane in 0..self.lane_count {
            let lane_str = lane.to_string();
            lane_msg += &format!("{}{}|", " ".repeat(num_chars.saturating_sub(lane_str.len())), lane_str);
            let tool_str = tool_map[lane as usize].to_string();
            tool_msg += &format!("{}{}|", " ".repeat(num_chars.saturating_sub(tool_str.len())), tool_str);
        }
        gcmd.respond_info(&format!("{}\n{}", lane_msg, tool_msg), true);
        Ok(())
    }

    pub fn cmd_tr_print_tool_groups(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let n = self.lane_count as usize;
        let mut tool_groups: Vec<Vec<i64>> = vec![vec![]; n];
        let tool_map = self.tool_map.borrow();
        for lane in 0..n {
            tool_groups[tool_map[lane] as usize].push(lane as i64);
        }
        let mut msg = String::new();
        for (tool, group) in tool_groups.iter().enumerate() {
            msg += &format!("Tool {}: {}", tool, int_list_repr(group));
            if group.len() > 1 {
                msg += &format!(" (default: {})", opt_str(self.default_lanes.borrow()[tool]));
            }
            msg += "\n";
        }
        gcmd.respond_info(&msg, true);
        Ok(())
    }

    // helper functions

    pub fn lower_servo(&self, toolhead_dwell: bool) -> KResult<()> {
        self.tr_toolhead.wait_moves()?;
        self.servo.set_servo(None, Some(self.servo_down_angle), None)?;
        if self.servo_raised.get() != Some(false) {
            self.tr_toolhead.dwell(self.servo_wait)?;
            if toolhead_dwell {
                self.toolhead()?.dwell(self.servo_wait)?;
            }
        }
        self.servo_raised.set(Some(false));
        Ok(())
    }

    pub fn raise_servo(&self, toolhead_dwell: bool, tr_toolhead_dwell: bool, wait_moves: bool, print_time: Option<f64>) -> KResult<()> {
        if wait_moves {
            self.tr_toolhead.wait_moves()?;
        }
        self.servo.set_servo(None, Some(self.servo_up_angle), print_time)?;
        if self.servo_raised.get() != Some(true) {
            if tr_toolhead_dwell {
                self.tr_toolhead.dwell(self.servo_wait)?;
            }
            if toolhead_dwell {
                self.toolhead()?.dwell(self.servo_wait)?;
            }
        }
        self.servo_raised.set(Some(true));
        Ok(())
    }

    pub fn is_selector_homed(&self) -> KResult<bool> {
        let status = self.tr_toolhead.get_kinematics().get_status(self.reactor.monotonic());
        Ok(matches!(status.get("homed_axes"), Some(PyVal::Str(s)) if s.contains('x')))
    }

    pub fn query_selector_sensor(&self) -> KResult<bool> {
        let move_time = self.tr_toolhead.get_last_move_time()?;
        Ok(self.fil_driver_endstops[0].0.query_endstop(move_time)? != 0)
    }

    pub fn query_toolhead_sensor(&self) -> KResult<Option<bool>> {
        let Some((es, _)) = self.toolhead_fil_endstops.first() else { return Ok(None) };
        let move_time = self.tr_toolhead.get_last_move_time()?;
        Ok(Some(es.query_endstop(move_time)? != 0))
    }

    pub fn check_lane_valid(&self, lane: Option<i64>) -> KResult<i64> {
        match lane {
            Some(l) if l < self.lane_count && l >= 0 => Ok(l),
            _ => Err(KError::command("Invalid LANE")),
        }
    }

    pub fn check_tool_valid(&self, tool: Option<i64>) -> KResult<i64> {
        self.check_lane_valid(tool).map_err(|_| KError::command("Invalid TOOL"))
    }

    pub fn check_selector_homed(&self) -> KResult<()> {
        if !self.is_selector_homed()? {
            return Err(not_homed_error("Must home selector first"));
        }
        Ok(())
    }

    pub fn can_lower_servo(&self) -> KResult<bool> {
        Ok((self.is_selector_homed()? && self.curr_lane.get().is_some()) || self.query_selector_sensor()?)
    }

    pub fn is_next_toolchange_done(&self) -> bool {
        // return True if there is no pending toolchange
        let Some(next_lane) = self.next_lane.get() else { return true };
        // check if an active lane is set
        let Some(active) = self.active_lane.get() else { return false };
        // check if active lane matches next lane
        if active == next_lane {
            return true;
        }
        // check if active lane is assigned to next tool
        if let Some(next_tool) = self.next_tool.get() {
            if self.tool_map.borrow()[active as usize] == next_tool {
                return true;
            }
        }
        false
    }

    pub fn reset_fil_driver(&self) -> KResult<()> {
        self.extruder_sync_manager.reset_fil_driver()
    }

    pub fn restore_extruder_sync(&self) -> KResult<()> {
        if self.sync_to_extruder.get() && self.active_lane.get().is_some() {
            self.extruder_sync_manager.sync_fil_driver_to_extruder()?;
            self.lower_servo(true)
        } else {
            self.raise_servo(false, true, true, None)?;
            self.tr_toolhead.wait_moves()?;
            self.extruder_sync_manager.unsync()
        }
    }

    pub fn go_to_lane(&self, lane: Option<i64>) -> KResult<()> {
        // check if homed
        self.check_selector_homed()?;

        // check lane
        let lane = self.check_lane_valid(lane)?;
        if Some(lane) == self.curr_lane.get() {
            return Ok(());
        }

        // check for filament in the selector
        if self.query_selector_sensor()? {
            return Err(KError::command("Cannot change lane with filament in selector"));
        }

        // raise servo
        self.raise_servo(false, true, true, None)?;

        // move to lane
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[0] = self.lane_positions.borrow()[lane as usize];
        self.tr_toolhead.move_to(&pos, self.sel_max_velocity)?;

        // set current lane
        self.curr_lane.set(Some(lane));
        Ok(())
    }

    pub fn load_lane(&self, lane: Option<i64>, reset_speed: bool, user_load: bool) -> KResult<()> {
        // check lane
        let lane = self.check_lane_valid(lane)?;

        // reset lane speed
        if reset_speed {
            self.lanes_buffered.borrow_mut()[lane as usize] = false;
        }

        // load filament into the selector
        self.load_selector(lane, None, user_load)?;

        // retract filament into the module
        self.reset_fil_driver()?;
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[1] -= self.selector_unload_length + self.selector_unload_length_extra;
        self.tr_toolhead.move_to(&pos, self.selector_unload_speed)?;

        // undo extra unload length offset
        pos[1] += self.selector_unload_length_extra;
        self.tr_toolhead.move_to(&pos, self.selector_unload_speed)?;

        // reset filament driver position
        self.reset_fil_driver()?;

        if !self.keep_servo_down_after_lane_load {
            // raise servo
            self.raise_servo(false, true, true, None)?;
        }

        if user_load {
            self.gcode.respond_info("Load complete", true);
        }
        Ok(())
    }

    fn extruder_heater(&self) -> KResult<Rc<crate::extras::heaters::Heater>> {
        self.toolhead()?.get_extruder().get_heater()
    }

    pub fn wait_for_heater_temp(&self, min_temp: f64, exact_temp: f64) -> KResult<Option<f64>> {
        // get current and target temps
        let heater = self.extruder_heater()?;
        let (smoothed_temp, target_temp) = heater.get_temp(self.reactor.monotonic());
        let min_extrude_temp = heater.min_extrude_temp.get();

        // raise an error if no valid temp has been set
        let highest = [min_temp, exact_temp, target_temp, self.last_heater_target.get()].into_iter().reduce(py_max).unwrap_or(0.0);
        if highest < min_extrude_temp {
            return Err(KError::command("Extruder temperature must be set above min_extrude_temp"));
        }

        // set temp and wait if below acceptable temp
        let min_usable_temp = py_max(min_temp, min_extrude_temp);
        if exact_temp != 0.0 || smoothed_temp < min_usable_temp {
            let (temp, save_temp) = if exact_temp != 0.0 {
                (exact_temp, Some(exact_temp))
            } else if target_temp > min_usable_temp {
                (target_temp, Some(target_temp))
            } else {
                let temp = py_max(min_temp, self.last_heater_target.get());
                (temp, if min_temp >= min_extrude_temp { Some(min_temp) } else { None })
            };
            let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
            pheaters.set_temperature(&heater, temp, true)?;
            return Ok(save_temp);
        }
        Ok(Some(target_temp))
    }

    pub fn save_heater_target(&self, target_temp: Option<f64>) -> KResult<()> {
        let target_temp = match target_temp {
            Some(t) => t,
            None => self.extruder_heater()?.get_temp(self.reactor.monotonic()).1,
        };
        self.save_variable(VARS_HEATER_TARGET, &py_float_repr(target_temp))?;
        self.last_heater_target.set(target_temp);
        Ok(())
    }

    pub fn note_heater_temps_for_redundant_toolchange(&self, min_temp: f64, exact_temp: f64) -> KResult<()> {
        let min_extrude_temp = self.extruder_heater()?.min_extrude_temp.get();
        if exact_temp >= min_extrude_temp {
            self.save_heater_target(Some(exact_temp))?;
        } else if min_temp >= min_extrude_temp {
            self.save_heater_target(Some(min_temp))?;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn load_toolhead(
        &self,
        lane: Option<i64>,
        tool: Option<i64>,
        min_temp: f64,
        exact_temp: f64,
        selector_already_loaded: bool,
        bowden_length: Option<f64>,
        extruder_load_length: Option<f64>,
        hotend_load_length: Option<f64>,
    ) -> KResult<()> {
        // save gcode state
        self.gcode.run_script_from_command(&format!("SAVE_GCODE_STATE NAME={}", GCODE_STATE_TOOLCHANGE))?;

        // reset retry_lane
        self.retry_lane.set(None);

        // check lane, and update next_lane and next_tool unless lane is already
        // active
        let checked = self.check_lane_valid(lane);
        // finally:
        // skip toolchange if lane is already active (the return in the
        // finally block also drops a failed lane check)
        if lane == self.active_lane.get() {
            // save heater target based on temperature arguments
            return self.note_heater_temps_for_redundant_toolchange(min_temp, exact_temp);
        }
        // keep track of lane in case of an error (and for status)
        self.next_lane.set(lane);
        // keep track of tool for status
        self.next_tool.set(tool);
        let mut lane = checked?;

        // check if homed
        self.check_selector_homed()?;

        // check and set lengths
        let bowden_length = bowden_length.unwrap_or(self.bowden_load_length.get());
        let extruder_load_length = extruder_load_length.unwrap_or(self.extruder_load_length);
        let hotend_load_length = hotend_load_length.unwrap_or(self.hotend_load_length.get());

        // wait for heater temp if needed
        let save_temp = self.wait_for_heater_temp(min_temp, exact_temp)?;

        // disable runout detection
        self.selector_sensor.set_active(false);

        if !(selector_already_loaded && self.curr_lane.get() == Some(lane)) {
            // unload current lane (if filament is detected)
            if let Err(e) = self.unload_toolhead(0.0, 0.0, false, false, false) {
                if !e.is_command_error() {
                    return Err(e);
                }
                self.raise_servo(false, true, true, None)?;
                match self.curr_lane.get() {
                    None => self.gcode.respond_info(
                        "Failed to unload. Please either pull the filament out of the toolhead and selector or retry with TR_UNLOAD_TOOLHEAD, then use TR_RESUME to continue.",
                        true,
                    ),
                    Some(cl) => {
                        self.gcode.respond_info(
                            &format!(
                                "Failed to unload. Please either pull the filament in lane {lane} out of the toolhead and selector or retry with TR_UNLOAD_TOOLHEAD, then use TR_RESUME to reload lane {lane} and continue.",
                                lane = cl
                            ),
                            true,
                        );
                        self.lanes_buffered.borrow_mut()[cl as usize] = false;
                    }
                }
                self.retry_lane.set(self.curr_lane.get());
                logging::warning_err("trad_rack: Failed to unload toolhead", &e);
                return Err(load_error("Failed to load toolhead. Could not unload toolhead before load"));
            }

            // home if selector position is uncertain
            if self.selector_pos_uncertain.get() {
                if let Err(e) = self.home() {
                    logging::warning_err("trad_rack: Failed to home selector", &e);
                    return Err(not_homed_error("Failed to home selector"));
                }
            }
        }

        // notify toolhead load started
        crate::events::TradRackLoadStarted::emit(&self.printer)?;

        // run pre-load custom gcode
        self.pre_load_macro.run_gcode_from_command(None)?;
        self.toolhead()?.wait_moves()?;
        self.tr_toolhead.wait_moves()?;

        // load filament into the selector
        let selected_lane = match self.load_selector(lane, tool, false) {
            Ok(l) => l,
            Err(e) => {
                if !e.is_command_error() {
                    return Err(e);
                }
                self.raise_servo(false, true, true, None)?;
                match tool {
                    None => {
                        self.gcode.respond_info(
                            &format!("Failed to load selector from lane {lane}. Use TR_RESUME to reload lane {lane} and retry.", lane = lane),
                            true,
                        );
                        self.retry_lane.set(Some(lane));
                    }
                    Some(tool) => {
                        let assigned_lanes = self.get_assigned_lanes(tool);
                        self.gcode.respond_info(
                            &format!(
                                "Failed to load selector from any of the lanes assigned to tool {tool}: {lanes}. Use TR_LOAD_LANE LANE=&lt;lane index&gt to reload one of these lanes, then use TR_RESUME to retry. (If you want to use a different lane, use TR_ASSIGN_LANE LANE=&lt;lane index&gt TOOL={tool} beforehand.)",
                                tool = tool,
                                lanes = int_list_repr(&assigned_lanes)
                            ),
                            true,
                        );
                    }
                }
                logging::warning_err("trad_rack: Failed to load selector", &e);
                return Err(load_error(format!("Failed to load toolhead. Could not load selector from lane {}", lane)));
            }
        };

        // update lane and next_lane in case the selector was loaded from a lane
        // other than what was initially specified
        lane = selected_lane;
        self.next_lane.set(Some(lane));

        // move filament through the bowden tube
        self.reset_fil_driver()?;
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        let move_start = pos[1];
        pos[1] += bowden_length;
        let cl = self.curr_lane.get().ok_or_else(|| KError::Other("list indices must be integers or slices, not NoneType".into()))?;
        let speed = if self.lanes_buffered.borrow()[cl as usize] { self.buffer_pull_speed } else { self.spool_pull_speed };
        let mut reached_sensor_early = true;
        if self.load_with_toolhead_sensor && !self.toolhead_fil_endstops.is_empty() {
            let hmove = self.homing_move_obj(&self.toolhead_fil_endstops)?;
            // move and check for early sensor trigger
            match hmove.homing_move(&pos, speed, true, true, true) {
                // if sensor triggered early, retract before next homing move
                Ok(trigpos) => pos[1] = trigpos[1] - self.fil_homing_retract_dist,
                Err(e) if e.is_command_error() => reached_sensor_early = false,
                Err(e) => return Err(e),
            }
        }
        self.tr_toolhead.move_to(&pos, speed)?;
        let base_length = pos[1] - move_start;

        // sync extruder to filament driver
        self.tr_toolhead.wait_moves()?;
        self.extruder_sync_manager.sync_extruder_to_fil_driver()?;

        // move filament until toolhead sensor is triggered
        if self.load_with_toolhead_sensor && !self.toolhead_fil_endstops.is_empty() {
            let mut pos = self.tr_toolhead.get_position();
            let move_start = pos[1];
            pos[1] += self.fil_homing_lengths[HomingLen::BowdenLoad as usize];
            let hmove = self.homing_move_obj(&self.toolhead_fil_endstops)?;
            let trigpos = match hmove.homing_move(&pos, self.toolhead_sense_speed, true, true, true) {
                Ok(t) => t,
                Err(e) if e.is_command_error() => {
                    self.raise_servo(false, true, true, None)?;
                    self.extruder_sync_manager.unsync()?;
                    self.gcode.respond_info(
                        &format!(
                            "Failed to load toolhead from lane {lane} (no trigger on toolhead sensor). Please either pull the filament in lane {lane} out of the toolhead and selector or use TR_UNLOAD_TOOLHEAD. Then use TR_RESUME to reload lane {lane} and retry.",
                            lane = lane
                        ),
                        true,
                    );
                    self.retry_lane.set(Some(lane));
                    logging::warning_err("trad_rack: Toolhead sensor homing move failed", &e);
                    return Err(load_error("Failed to load toolhead. No trigger on toolhead sensor after full movement"));
                }
                Err(e) => return Err(e),
            };

            // update bowden_load_length
            let length = trigpos[1] - move_start + base_length - self.target_toolhead_homing_dist;
            let old_set_length = self.bowden_load_length.get();
            let new_len = self.bowden_load_length_filter.update(length);
            self.bowden_load_length.set(new_len);
            let samples = self.bowden_load_length_filter.get_entry_count();
            if self.log_bowden_lengths {
                self.write_bowden_length_data(&self.bowden_load_lengths_filename, length, old_set_length, new_len, samples)?;
            }
            self.save_bowden_length("load", new_len, samples)?;
            if !(self.bowden_load_calibrated.get() || reached_sensor_early) {
                self.bowden_load_calibrated.set(true);
                self.gcode.respond_info(&format!("Calibrated bowden_load_length: {}", py_float_repr(new_len)), true);
            }
        }

        // finish loading filament into extruder
        self.reset_fil_driver()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[1] += extruder_load_length;
        self.tr_toolhead.move_to(&pos, self.extruder_load_speed)?;

        // load filament into hotend
        pos[1] += hotend_load_length;
        self.tr_toolhead.move_to(&pos, self.hotend_load_speed)?;

        // check whether servo move might overlap extruder loading move
        let hotend_load_time = if hotend_load_length != 0.0 {
            self.tr_toolhead.last_move_min_move_t().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'min_move_t'".into()))?
        } else {
            0.0
        };
        let servo_delay = py_max(0.0, self.servo_wait - hotend_load_time);

        // flush lookahead and raise servo before move ends
        let print_time = self.tr_toolhead.get_last_move_time()? - self.servo_wait + servo_delay;
        if !self.sync_to_extruder.get() {
            self.raise_servo(false, false, false, Some(print_time))?;

            // wait for servo move if necessary
            if servo_delay != 0.0 {
                self.tr_toolhead.dwell(servo_delay)?;
            }
        }

        // set active lane
        self.set_active_lane(Some(lane))?;

        // unsync extruder from filament driver
        self.tr_toolhead.wait_moves()?;
        self.restore_extruder_sync()?;

        // make lane the new default for its assigned tool
        self.make_lane_default(lane);

        // enable runout detection
        self.selector_sensor.set_active(true);

        // save heater target
        if let Some(t) = save_temp {
            self.save_heater_target(Some(t))?;
        }

        // run post-load custom gcode
        self.post_load_macro.run_gcode_from_command(None)?;
        self.toolhead()?.wait_moves()?;
        self.tr_toolhead.wait_moves()?;

        // restore gcode state
        self.gcode.run_script_from_command(&format!("RESTORE_GCODE_STATE NAME={} MOVE=1", GCODE_STATE_TOOLCHANGE))?;

        // reset next lane and tool
        self.next_lane.set(None);
        self.next_tool.set(None);

        // notify toolhead load complete
        crate::events::TradRackLoadComplete::emit(&self.printer)
    }

    fn homing_move_obj(&self, endstops: &[(EndstopRef, String)]) -> KResult<Rc<HomingMove>> {
        let th: Rc<dyn HomingToolhead> = self.tr_toolhead.clone();
        HomingMove::new(&self.printer, endstops.to_vec(), Some(th))
    }

    pub fn load_selector(&self, lane: i64, tool: Option<i64>, user_load: bool) -> KResult<i64> {
        match self.do_load_selector(lane, user_load) {
            Ok(()) => Ok(lane),
            Err(e) if e.is_command_error() => match tool {
                None => Err(e),
                Some(tool) => match self.find_replacement_lane(lane, false)? {
                    Some(l) => Ok(l),
                    None => Err(KError::command(format!(
                        "Failed to load filament into selector from any of the lanes assigned to tool {}",
                        tool
                    ))),
                },
            },
            Err(e) => Err(e),
        }
    }

    pub fn do_load_selector(&self, lane: i64, user_load: bool) -> KResult<()> {
        // move selector
        self.go_to_lane(Some(lane))?;

        // lower servo
        self.lower_servo(false)?;
        self.tr_toolhead.wait_moves()?;

        // prompt user to insert filament
        if user_load {
            self.gcode.respond_info(&format!("Please insert filament in lane {}", lane), true);
        }

        // turn the drive gear until filament is detected
        self.reset_fil_driver()?;
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        let hmove = self.homing_move_obj(&self.fil_driver_endstops)?;
        let length_key = if user_load { HomingLen::UserLoadLane } else { HomingLen::LoadSelector };
        pos[1] += self.fil_homing_lengths[length_key as usize];
        match hmove.homing_move(&pos, self.selector_sense_speed, false, true, true) {
            Ok(_) => Ok(()),
            Err(e) if e.is_command_error() => {
                self.raise_servo(false, true, true, None)?;
                logging::warning_err("trad_rack: Selector homing move failed", &e);
                Err(KError::command("Failed to load filament into selector. No trigger on selector sensor after full movement"))
            }
            Err(e) => Err(e),
        }
    }

    pub fn unload_selector(&self, base_length: Option<f64>, mark_calibrated: bool, eject: bool) -> KResult<()> {
        // check for filament in selector
        if !self.query_selector_sensor()? {
            self.gcode.respond_info("No filament detected. Attempting to load selector", true);
            let cl = self.curr_lane.get();
            let cl = self.check_lane_valid(cl)?;
            self.load_selector(cl, None, false)?;
            self.gcode.respond_info("Loaded selector. Retracting filament into module", true);
        } else {
            // lower servo and turn the drive gear until filament
            // is no longer detected
            self.lower_servo(false)?;
            self.reset_fil_driver()?;
            self.tr_toolhead.get_last_move_time()?;
            let mut pos = self.tr_toolhead.get_position();
            let move_start = pos[1];
            let hmove = self.homing_move_obj(&self.fil_driver_endstops)?;
            pos[1] -= self.fil_homing_lengths[HomingLen::BowdenUnload as usize];
            let trigpos = match hmove.homing_move(&pos, self.selector_sense_speed, true, false, true) {
                Ok(t) => t,
                Err(e) if e.is_command_error() => {
                    self.raise_servo(false, true, true, None)?;
                    logging::warning_err("trad_rack: Selector homing move failed", &e);
                    return Err(KError::command(
                        "Failed to unload filament from selector. Selector sensor still triggered after full movement",
                    ));
                }
                Err(e) => return Err(e),
            };

            // update bowden_unload_length
            if let Some(base_length) = base_length {
                if !self.ignore_next_unload_length.get() {
                    let length = move_start - trigpos[1] + base_length - self.target_selector_homing_dist;
                    let old_set_length = self.bowden_unload_length.get();
                    let new_len = self.bowden_unload_length_filter.update(length);
                    self.bowden_unload_length.set(new_len);
                    let samples = self.bowden_unload_length_filter.get_entry_count();
                    if self.log_bowden_lengths {
                        self.write_bowden_length_data(&self.bowden_unload_lengths_filename, length, old_set_length, new_len, samples)?;
                    }
                    self.save_bowden_length("unload", new_len, samples)?;
                    if mark_calibrated {
                        self.bowden_unload_calibrated.set(true);
                        self.gcode.respond_info(&format!("Calibrated bowden_unload_length: {}", py_float_repr(new_len)), true);
                    }
                }
            }
        }

        // retract filament into the module
        self.reset_fil_driver()?;
        let mut pos = self.tr_toolhead.get_position();
        let speed = if eject {
            pos[1] -= self.selector_unload_length + self.eject_length;
            self.eject_speed
        } else {
            pos[1] -= self.selector_unload_length + self.selector_unload_length_extra;
            self.selector_unload_speed
        };
        self.tr_toolhead.move_to(&pos, speed)?;

        if !eject {
            // undo extra unload length offset
            pos[1] += self.selector_unload_length_extra;
            self.tr_toolhead.move_to(&pos, self.selector_unload_speed)?;
        }

        // reset filament driver position
        self.reset_fil_driver()?;

        // raise servo
        self.raise_servo(false, true, true, None)
    }

    pub fn unload_toolhead(&self, min_temp: f64, exact_temp: f64, force_unload: bool, sync: bool, eject: bool) -> KResult<()> {
        let selector_sensor_state = self.query_selector_sensor()?;
        let toolhead_sensor_state = self.query_toolhead_sensor()?;

        // check for filament
        self.toolhead()?.wait_moves()?;
        if !(force_unload || selector_sensor_state || toolhead_sensor_state == Some(true)) {
            // reset ignore_next_unload_length
            self.ignore_next_unload_length.set(false);
            return Ok(());
        }

        // check for faulty toolhead or selector sensor
        if !force_unload && toolhead_sensor_state == Some(true) && !selector_sensor_state {
            self.gcode.respond_info(
                "WARNING: The toolhead filament sensor is triggered but the selector sensor is not. This may indicate that one of the sensors is faulty or that there is a short piece of filament in the bowden tube that does not reach the selector.",
                true,
            );
        }

        // check that the selector is at a lane
        if !self.can_lower_servo()? {
            return Err(KError::command("Selector must be moved to a lane before unloading"));
        }

        // disable runout detection
        self.selector_sensor.set_active(false);

        // notify toolhead unload started
        crate::events::TradRackUnloadStarted::emit(&self.printer)?;

        // wait for heater temp if needed
        self.wait_for_heater_temp(min_temp, exact_temp)?;

        // sync filament driver to extruder for pre-unload custom gcode
        if sync {
            self.extruder_sync_manager.sync_fil_driver_to_extruder()?;
            self.lower_servo(true)?;
        }

        // run pre-unload custom gcode
        let r = (|| {
            self.pre_unload_macro.run_gcode_from_command(None)?;
            self.toolhead()?.wait_moves()?;
            self.tr_toolhead.wait_moves()
        })();
        // finally: unsync filament driver from extruder, reset active lane
        let fin = self.extruder_sync_manager.unsync().and_then(|_| self.set_active_lane(None));
        r?;
        fin?;

        // lower servo
        self.lower_servo(true)?;

        // sync extruder to filament driver
        self.tr_toolhead.wait_moves()?;
        self.extruder_sync_manager.sync_extruder_to_fil_driver()?;

        // move filament until toolhead sensor is untriggered
        if self.unload_with_toolhead_sensor && !self.toolhead_fil_endstops.is_empty() {
            let mut pos = self.tr_toolhead.get_position();
            pos[1] -= self.fil_homing_lengths[HomingLen::UnloadToolhead as usize];
            let hmove = self.homing_move_obj(&self.toolhead_fil_endstops)?;
            if let Err(e) = hmove.homing_move(&pos, self.toolhead_sense_speed, false, false, true) {
                if !e.is_command_error() {
                    return Err(e);
                }
                self.raise_servo(false, true, true, None)?;
                self.extruder_sync_manager.unsync()?;
                logging::warning_err("trad_rack: Toolhead sensor homing move failed", &e);
                return Err(KError::command("Failed to unload toolhead. Toolhead sensor still triggered after full movement"));
            }
        }

        // get filament out of the extruder
        self.reset_fil_driver()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[1] -= self.toolhead_unload_length;
        self.tr_toolhead.move_to(&pos, self.toolhead_unload_speed)?;

        // unsync extruder from filament driver
        self.tr_toolhead.wait_moves()?;
        self.extruder_sync_manager.unsync()?;

        // move filament through the bowden tube
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        let move_start = pos[1];
        pos[1] -= self.bowden_unload_length.get();
        let hmove = self.homing_move_obj(&self.fil_driver_endstops)?;
        let mut reached_sensor_early = true;
        // move and check for early sensor trigger
        match hmove.homing_move(&pos, self.buffer_pull_speed, true, false, true) {
            // if sensor triggered early, retract before next homing move
            Ok(trigpos) => pos[1] = trigpos[1] + self.fil_homing_retract_dist,
            Err(e) if e.is_command_error() => reached_sensor_early = false,
            Err(e) => return Err(e),
        }
        self.tr_toolhead.move_to(&pos, self.buffer_pull_speed)?;

        // unload selector
        let mark_calibrated = !(self.bowden_unload_calibrated.get() || reached_sensor_early);
        self.unload_selector(Some(move_start - pos[1]), mark_calibrated, eject)?;

        // note that the current lane's buffer has been filled
        if let Some(cl) = self.curr_lane.get() {
            self.lanes_buffered.borrow_mut()[cl as usize] = true;
        }

        // reset ignore_next_unload_length
        self.ignore_next_unload_length.set(false);

        // run post-unload custom gcode
        self.post_unload_macro.run_gcode_from_command(None)?;
        self.toolhead()?.wait_moves()?;
        self.tr_toolhead.wait_moves()?;

        // notify toolhead unload complete
        crate::events::TradRackUnloadComplete::emit(&self.printer)
    }

    fn is_paused(&self) -> KResult<bool> {
        let pause_resume = self.printer.lookup_object("pause_resume")?;
        Ok(match pause_resume.get_status(self.reactor.monotonic()) {
            Some(PyVal::Dict(d)) => matches!(d.get("is_paused"), Some(PyVal::Bool(true))),
            _ => false,
        })
    }

    pub fn send_pause(&self) -> KResult<()> {
        if self.is_paused()? {
            return Ok(());
        }

        // run pause macro
        self.pause_macro.run_gcode_from_command(None)?;

        // if a toolchange is in progress, replace the PAUSE_STATE gcode state
        // with the state from right before the toolchange was initiated
        if !self.is_next_toolchange_done() {
            let gcode_move = self.printer.lookup::<GCodeMove>("gcode_move")?;
            gcode_move.copy_saved_state("PAUSE_STATE", GCODE_STATE_TOOLCHANGE)?;
        }
        Ok(())
    }

    pub fn send_resume(&self, resume_msg: Option<&str>) -> KResult<()> {
        if !self.is_paused()? {
            return Ok(());
        }
        if let Some(msg) = resume_msg.filter(|m| !m.is_empty()) {
            self.gcode.respond_info(msg, true);
        }
        self.resume_macro.run_gcode_from_command(None)
    }

    pub fn set_active_lane(&self, lane: Option<i64>) -> KResult<()> {
        self.active_lane.set(lane);
        if self.save_active_lane {
            self.save_variable(VARS_ACTIVE_LANE, &opt_str(lane))?;
        }
        Ok(())
    }

    pub fn reset_tool_map(&self) {
        *self.tool_map.borrow_mut() = (0..self.lane_count).collect();
        *self.default_lanes.borrow_mut() = (0..self.lane_count).map(Some).collect();
    }

    pub fn find_replacement_lane(&self, runout_lane: i64, check_runout_lane: bool) -> KResult<Option<i64>> {
        let tool = self.tool_map.borrow()[runout_lane as usize];
        let mut pre_dead_lanes = vec![];

        // home if selector position is uncertain
        if self.selector_pos_uncertain.get() {
            self.home()?;
        }

        // 1st pass - check lanes not marked as dead
        let mut lane = (runout_lane + 1).rem_euclid(self.lane_count);
        loop {
            if lane == runout_lane && !check_runout_lane {
                break;
            }
            if self.tool_map.borrow()[lane as usize] == tool {
                if self.lanes_dead.borrow()[lane as usize] {
                    pre_dead_lanes.push(lane);
                } else {
                    match self.load_selector(lane, None, false) {
                        Ok(_) => {
                            self.default_lanes.borrow_mut()[tool as usize] = Some(lane);
                            return Ok(Some(lane));
                        }
                        Err(e) if e.is_command_error() => self.lanes_dead.borrow_mut()[lane as usize] = true,
                        Err(e) => return Err(e),
                    }
                }
            }
            if lane == runout_lane {
                break;
            }
            lane = (lane + 1).rem_euclid(self.lane_count);
        }

        // 2nd pass - check lanes previously marked as dead
        for lane in pre_dead_lanes {
            match self.load_selector(lane, None, false) {
                Ok(_) => {
                    self.lanes_dead.borrow_mut()[lane as usize] = false;
                    self.default_lanes.borrow_mut()[tool as usize] = Some(lane);
                    return Ok(Some(lane));
                }
                Err(e) if e.is_command_error() => {}
                Err(e) => return Err(e),
            }
        }
        Ok(None)
    }

    pub fn set_default_lane(&self, tool: i64, lane: Option<i64>) {
        // set lane that was passed in
        if let Some(lane) = lane {
            self.assign_lane(lane, tool);
            self.default_lanes.borrow_mut()[tool as usize] = Some(lane);
            return;
        }
        // find a lane that is already assigned to the tool
        let found = self.tool_map.borrow().iter().position(|t| *t == tool).map(|l| l as i64);
        self.default_lanes.borrow_mut()[tool as usize] = found;
    }

    pub fn make_lane_default(&self, lane: i64) {
        let tool = self.tool_map.borrow()[lane as usize];
        self.default_lanes.borrow_mut()[tool as usize] = Some(lane);
    }

    pub fn assign_lane(&self, lane: i64, tool: i64) {
        let prev_tool = self.tool_map.borrow()[lane as usize];

        // assign lane to tool
        self.tool_map.borrow_mut()[lane as usize] = tool;

        // reassign default lane for previous tool if needed
        if self.default_lanes.borrow()[prev_tool as usize] == Some(lane) {
            self.set_default_lane(prev_tool, None);
        }

        // ensure new tool has a default lane assigned
        if self.default_lanes.borrow()[tool as usize].is_none() {
            self.default_lanes.borrow_mut()[tool as usize] = Some(lane);
        }
    }

    pub fn get_assigned_lanes(&self, tool: i64) -> Vec<i64> {
        self.tool_map.borrow().iter().enumerate().filter(|(_, t)| **t == tool).map(|(l, _)| l as i64).collect()
    }

    pub fn runout_replace_filament(&self) -> KResult<bool> {
        let mut check_runout_lane = true;

        // unload
        if self.runout_steps_done.get() < 1 {
            if let Err(e) = self.unload_toolhead(0.0, 0.0, true, true, true) {
                if !e.is_command_error() {
                    return Err(e);
                }
                self.raise_servo(false, true, true, None)?;
                self.gcode.respond_info(
                    &format!(
                        "Failed to unload. Please pull filament {} out of the toolhead and selector, then use TR_RESUME to continue.",
                        opt_str(self.runout_lane.get())
                    ),
                    true,
                );
                logging::warning_err("trad_rack: Failed to unload toolhead", &e);
                return Ok(false);
            }
            check_runout_lane = false;
            self.runout_steps_done.set(1);
        }

        // find a new lane to use
        let mut selector_already_loaded = false;
        if self.runout_steps_done.get() < 2 {
            let runout_lane = self.runout_lane.get().ok_or_else(|| KError::Other("list indices must be integers or slices, not NoneType".into()))?;
            let lane = self.find_replacement_lane(runout_lane, check_runout_lane)?;
            let Some(lane) = lane else {
                let runout_tool = self.tool_map.borrow()[runout_lane as usize];
                let assigned_lanes = self.get_assigned_lanes(runout_tool);
                self.gcode.respond_info(
                    &format!(
                        "No replacement lane found for tool {tool}. The following lanes are assigned to tool {tool}: {lanes}. Use TR_LOAD_LANE LANE=&lt;lane index&gt; to load one of these lanes, or use TR_ASSIGN_LANE LANE=&lt;lane index&gt; TOOL={tool} to assign another lane. Then use TR_RESUME to continue.",
                        tool = runout_tool,
                        lanes = int_list_repr(&assigned_lanes)
                    ),
                    true,
                );
                return Ok(false);
            };
            self.replacement_lane.set(Some(lane));
            selector_already_loaded = true;
            self.runout_lane.set(None);
            self.runout_steps_done.set(2);
        }

        // load toolhead
        self.load_toolhead(self.replacement_lane.get(), None, 0.0, 0.0, selector_already_loaded, None, None, None)?;
        Ok(true)
    }

    pub fn write_bowden_length_data(&self, filename: &str, length: f64, old_set_length: f64, new_set_length: f64, samples: usize) -> KResult<()> {
        use std::io::Write;
        let r = (|| -> std::io::Result<()> {
            let mut f = std::fs::OpenOptions::new().append(true).create(true).read(true).open(filename)?;
            if std::fs::metadata(filename)?.len() == 0 {
                f.write_all(b"time,length,diff_from_set_length,new_set_length,new_sample_count\n")?;
            }
            f.write_all(
                format!("{},{:.3},{:.3},{:.3},{}\n", strftime_local("%Y%m%d_%H%M%S"), length, length - old_set_length, new_set_length, samples)
                    .as_bytes(),
            )
        })();
        r.map_err(|e| {
            // command_error(fmt, filename, str(e)): str() of the 3 args
            let msg = crate::pyfmt::py_str_repr("Error writing to file '%s': %s");
            KError::command(format!("({}, {}, {})", msg, crate::pyfmt::py_str_repr(filename), crate::pyfmt::py_str_repr(&crate::pyfmt::py_oserror(&e, filename))))
        })
    }

    pub fn save_bowden_length(&self, mode: &str, new_set_length: f64, samples: usize) -> KResult<()> {
        let length_stats = format!("{{'new_set_length': {}, 'sample_count': {}}}", py_float_repr(new_set_length), samples);
        let var = if mode == "load" { VARS_CALIB_BOWDEN_LOAD_LENGTH } else { VARS_CALIB_BOWDEN_UNLOAD_LENGTH };
        self.save_variable(var, &length_stats)
    }

    /// one step of the _calibrate_selector() generator from `state`: the
    /// next state at a yield, None at StopIteration
    fn calibrate_selector_step(&self, state: Calib) -> KResult<Option<Calib>> {
        let extra_travel_base = 1.0;
        let extra_travel_per_lane = 0.3;
        match state {
            Calib::Done => Ok(None),
            Calib::AtLane0 => {
                // measure position of lane 0 relative to endstop
                let pos_endstop = self.tr_kinematics.get_selector_rail().get_homing_info().position_endstop;
                let max_travel = self.lane_positions.borrow()[0] - pos_endstop + extra_travel_base;
                let endstop_to_lane0 = self.measure_selector_to_endstop(max_travel)?;

                // prompt user to set the selector at the last lane
                self.prompt_selector_calibration(self.lane_count - 1)?;
                Ok(Some(Calib::AtLastLane(endstop_to_lane0)))
            }
            Calib::AtLastLane(endstop_to_lane0) => {
                // measure position of last lane relative to endstop
                let n = self.lane_count as usize;
                let lp = self.lane_positions.borrow().clone();
                let max_travel = lp[n - 1] - lp[0] + endstop_to_lane0 + extra_travel_base + (self.lane_count - 1) as f64 * extra_travel_per_lane;
                let endstop_to_last_lane = self.measure_selector_to_endstop(max_travel)?;

                // process calibration and set new lane positions
                let (pos_endstop, lane_spacing, lane_positions) =
                    self.lane_position_manager.process_selector_calibration(endstop_to_lane0, endstop_to_last_lane, 6);
                *self.lane_positions.borrow_mut() = lane_positions.clone();

                // round new config values
                let pos_endstop = py_round(pos_endstop, 3);
                let pos_min = (py_min(pos_endstop, lane_positions[0]) * 1000.0).floor() / 1000.0;
                let pos_max = (lane_positions[n - 1] * 1000.0).ceil() / 1000.0;

                // set new selector values
                let rail = self.tr_kinematics.get_selector_rail();
                rail.position_min.set(pos_min);
                rail.position_endstop.set(pos_endstop);
                rail.position_max.set(pos_max);

                // set current selector position
                self.tr_toolhead.get_last_move_time()?;
                let mut pos = self.tr_toolhead.get_position();
                pos[0] = pos_endstop;
                self.tr_toolhead.set_position(&pos, &axes_str(&[0]))?;

                // show results and prompt user to save config
                self.gcode.respond_info(
                    &format!(
                        "trad_rack: lane_spacing: {lane_spacing:.6}\n{stepper}: position_min: {pos_min:.3}\n{stepper}: position_endstop: {pos_endstop:.3}\n{stepper}: position_max: {pos_max:.3}\nMake sure to update the printer config file with these parameters so they will be kept across restarts.",
                        lane_spacing = lane_spacing,
                        stepper = SELECTOR_STEPPER_NAME,
                        pos_min = pos_min,
                        pos_endstop = pos_endstop,
                        pos_max = pos_max
                    ),
                    true,
                );
                Ok(None)
            }
        }
    }

    pub fn prompt_selector_calibration(&self, lane: i64) -> KResult<()> {
        // go to lane
        if !self.is_selector_homed()? {
            self.home()?;
        }
        self.go_to_lane(Some(lane))?;

        // disable selector motor
        self.tr_toolhead.wait_moves()?;
        let print_time = self.tr_toolhead.get_last_move_time()?;
        let stepper_enable = self.printer.lookup::<PrinterStepperEnable>("stepper_enable")?;
        stepper_enable.lookup_enable(SELECTOR_STEPPER_NAME)?.motor_disable(print_time)?;

        // load filament into the selector
        self.load_selector(lane, None, true)?;

        // extend filament past the sensor
        self.reset_fil_driver()?;
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[1] += 15.0;
        self.tr_toolhead.move_to(&pos, self.selector_unload_speed)?;

        // reset filament driver position
        self.reset_fil_driver()?;

        // raise servo
        self.raise_servo(false, true, true, None)?;

        // prompt user to position selector
        self.tr_toolhead.wait_moves()?;
        self.gcode.respond_info(
            "To ensure that the filament paths of the lane module and selector are aligned, adjust the selector's position by hand until the filament can slide smoothly with very little resistance. Then use TR_NEXT to continue selector calibration.",
            true,
        );
        Ok(())
    }

    pub fn measure_selector_to_endstop(&self, max_travel: f64) -> KResult<f64> {
        // set selector position
        let print_time = self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[0] = max_travel;
        self.tr_toolhead.set_position(&pos, &axes_str(&[0]))?;
        let stepper_enable = self.printer.lookup::<PrinterStepperEnable>("stepper_enable")?;
        stepper_enable.lookup_enable(SELECTOR_STEPPER_NAME)?.motor_enable(print_time)?;

        // unload selector into current lane
        self.unload_selector(None, false, false)?;

        // clear current lane
        self.curr_lane.set(None);

        // prepare for homing move
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[0] = 0.0;
        let rail = self.tr_kinematics.get_selector_rail();
        let endstops = rail.get_endstops();
        let hi = rail.get_homing_info();
        let speed = if hi.retract_dist != 0.0 { hi.second_homing_speed } else { hi.speed };

        // retract first if endstop is already triggered
        let move_time = self.tr_toolhead.get_last_move_time()?;
        if endstops[0].0.query_endstop(move_time)? != 0 {
            let mut curr_pos = self.tr_toolhead.get_position();
            curr_pos[0] += 5.0;
            self.tr_toolhead.move_to(&curr_pos, hi.speed)?;
            self.tr_toolhead.wait_moves()?;
        }

        // homing move
        let hmove = self.homing_move_obj(&endstops)?;
        let trigpos = hmove.homing_move(&pos, speed, true, true, true)?;

        // set selector position
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[0] = hi.position_endstop;
        self.tr_toolhead.set_position(&pos, &axes_str(&[0]))?;

        // return distance traveled
        Ok(max_travel - trigpos[0])
    }

    // resume callbacks

    fn run_resume(&self, resume: &Resume) -> KResult<(bool, Option<String>)> {
        match resume {
            Resume::LoadToolhead => self.resume_load_toolhead(),
            Resume::CheckCondition { condition, action, resume_msg, fail_msg } => {
                if condition(self)? {
                    if let Some(action) = action {
                        action(self)?;
                    }
                    return Ok((false, Some(resume_msg.clone())));
                }
                self.gcode.respond_info(fail_msg, true);
                Ok((true, None))
            }
            Resume::Runout => {
                if self.runout_replace_filament()? {
                    return Ok((false, Some("Toolhead loaded successfully. Resuming print".into())));
                }
                Ok((true, None))
            }
        }
    }

    pub fn resume_load_toolhead(&self) -> KResult<(bool, Option<String>)> {
        if !self.is_next_toolchange_done() {
            let mut selector_already_loaded = false;

            // retry loading lane
            if let Some(retry) = self.retry_lane.get() {
                if Some(retry) == self.next_lane.get() {
                    self.load_selector(retry, None, true)?;
                    selector_already_loaded = true;
                } else {
                    self.load_lane(Some(retry), false, true)?;
                }
            }

            // load next filament into toolhead
            self.load_toolhead(self.next_lane.get(), self.next_tool.get(), 0.0, 0.0, selector_already_loaded, None, None, None)?;
        }
        Ok((false, Some("Toolhead loaded successfully. Resuming print".into())))
    }

    // other resume helper functions

    fn set_up_resume_and_pause(&self, resume: Resume) -> KResult<()> {
        // clear the resume stack if the print is not paused
        // (if the stack is not already empty but the print is not paused, then
        // the user likely resumed the print manually without using TR_RESUME, so
        // the existing resume setup is no longer relevant)
        if !self.is_paused()? {
            self.resume_stack.borrow_mut().clear();
        }

        // add resume callback and arguments
        self.resume_stack.borrow_mut().push_back(resume);

        // pause the print
        self.send_pause()
    }

    pub fn unload_toolhead_and_resume(&self) -> KResult<()> {
        if self.is_paused()? {
            self.gcode.respond_info("Unloading toolhead", true);
            let g = self.gcode.create_gcode_command("TR_UNLOAD_TOOLHEAD", "TR_UNLOAD_TOOLHEAD", IndexMap::default());
            self.cmd_tr_unload_toolhead(&g)?;
            let g = self.gcode.create_gcode_command("TR_RESUME", "TR_RESUME", IndexMap::default());
            self.cmd_tr_resume(&g)?;
        }
        Ok(())
    }

    pub fn resume_act_locate_selector(&self) -> KResult<()> {
        if !self.is_selector_homed()? {
            self.home()?;
        }
        self.ignore_next_unload_length.set(false);
        Ok(())
    }

    // other functions

    pub fn set_fil_driver_multiplier(&self, multiplier: f64) -> KResult<()> {
        self.extruder_sync_manager.set_fil_driver_multiplier(multiplier)
    }

    pub fn is_fil_driver_synced(&self) -> bool {
        self.extruder_sync_manager.is_fil_driver_synced()
    }
}

impl PrinterObject for TradRack {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let opt = |v: Option<i64>| v.map(PyVal::Int).unwrap_or(PyVal::None);
        let d = crate::pydict! {
            "curr_lane" => opt(self.curr_lane.get()),
            "active_lane" => opt(self.active_lane.get()),
            "next_lane" => opt(self.next_lane.get()),
            "next_tool" => opt(self.next_tool.get()),
            "tool_map" => PyVal::List(self.tool_map.borrow().iter().map(|t| PyVal::Int(*t)).collect()),
            "selector_homed" => PyVal::Bool(self.is_selector_homed().unwrap_or(false)),
        };
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        self.resume_stack.borrow_mut().clear();
        *self.toolhead.borrow_mut() = None;
        let _ = self.rc();
    }
}

// ----------------------------------------------------------------------
// TradRackToolHead / TradRackKinematics
// ----------------------------------------------------------------------

/// TradRackToolHead(config, buffer_pull_speed, is_extruder_synced): a
/// ToolHead built with the trad_rack limits and kinematics
fn new_tr_toolhead(
    config: &ConfigWrapper,
    buffer_pull_speed: f64,
    is_extruder_synced: Rc<dyn Fn() -> bool>,
) -> KResult<(Rc<ToolHead>, Rc<TradRackKinematics>)> {
    // Velocity and acceleration control
    let tr_config = config.getsection("trad_rack");
    let sel_max_velocity = tr_config.getfloat("selector_max_velocity", Dflt::Required, Lim::new().above(0.0))?;
    let fil_max_velocity = tr_config.getfloat("filament_max_velocity", buffer_pull_speed, Lim::new().above(0.0))?;
    let max_velocity = py_max(sel_max_velocity, fil_max_velocity);
    let sel_max_accel = tr_config.getfloat("selector_max_accel", Dflt::Required, Lim::new().above(0.0))?;
    let fil_max_accel = tr_config.getfloat("filament_max_accel", 1500.0, Lim::new().above(0.0))?;
    let max_accel = py_max(sel_max_accel, fil_max_accel);
    let min_cruise_ratio = tr_config.getfloat("minimum_cruise_ratio", 0.0, Lim::new().below(1.0).min(0.0))?;
    let square_corner_velocity = tr_config.getfloat("square_corner_velocity", 0.0, Lim::new().min(0.0))?;
    let limits = ToolHeadLimits { max_velocity, max_accel, min_cruise_ratio, square_corner_velocity };
    let kin_slot: RefCell<Option<Rc<TradRackKinematics>>> = RefCell::new(None);
    let th = ToolHead::new_with(config, limits, &|th| {
        let kin = match TradRackKinematics::new(th, config, (sel_max_velocity, sel_max_accel), (fil_max_velocity, fil_max_accel), is_extruder_synced.clone()) {
            Ok(kin) => kin,
            Err(e) if matches!(e.base(), KError::Config(_) | KError::Pins(_)) => return Err(e),
            Err(e) => {
                let msg = "Error loading kinematics 'trad_rack'";
                logging::exception_err(msg, &e);
                return Err(KError::config(msg).with_context(e));
            }
        };
        *kin_slot.borrow_mut() = Some(kin.clone());
        Ok(kin)
    })?;
    let kin = kin_slot.borrow().clone().expect("trad_rack kinematics");
    Ok((th, kin))
}

pub struct TradRackKinematics {
    pub printer: Rc<Printer>,
    pub rails: [Rc<PrinterRail>; 2],
    pub sel_max_velocity: f64,
    pub sel_max_accel: f64,
    pub fil_max_velocity: f64,
    pub fil_max_accel: f64,
    pub limits: RefCell<[(f64, f64); 2]>,
    pub selector_min: f64,
    pub selector_max: f64,
    pub is_extruder_synced: Rc<dyn Fn() -> bool>,
}

impl TradRackKinematics {
    fn new(
        toolhead: &Rc<ToolHead>,
        config: &ConfigWrapper,
        sel: (f64, f64),
        fil: (f64, f64),
        is_extruder_synced: Rc<dyn Fn() -> bool>,
    ) -> KResult<Rc<Self>> {
        // Setup axis rails
        let selector_stepper_section = config.getsection(SELECTOR_STEPPER_NAME);
        let fil_driver_stepper_section = config.getsection(FIL_DRIVER_STEPPER_NAME);
        let selector_rail = lookup_multi_rail(&selector_stepper_section, true, None, false)?;
        let fil_driver_rail = lookup_multi_rail(&fil_driver_stepper_section, true, None, false)?;
        let rails = [selector_rail, fil_driver_rail];
        for (rail, axis) in rails.iter().zip(['x', 'y']) {
            rail.setup_itersolve(|| cartesian_stepper_alloc(axis).expect("axis"));
        }
        for rail in &rails {
            for s in rail.get_steppers() {
                s.set_trapq(Some(toolhead.get_trapq()));
            }
        }
        // Setup boundary checks
        let selector_min = selector_stepper_section.getfloat("position_min", Dflt::Required, Lim::new().no_note())?;
        let selector_max = selector_stepper_section.getfloat("position_max", Dflt::Required, Lim::new().no_note())?;
        Ok(Rc::new(TradRackKinematics {
            printer: config.get_printer(),
            rails,
            sel_max_velocity: sel.0,
            sel_max_accel: sel.1,
            fil_max_velocity: fil.0,
            fil_max_accel: fil.1,
            limits: RefCell::new([(1.0, -1.0); 2]),
            selector_min,
            selector_max,
            is_extruder_synced,
        }))
    }

    pub fn get_selector_rail(&self) -> Rc<PrinterRail> {
        self.rails[0].clone()
    }

    pub fn get_fil_driver_rail(&self) -> Rc<PrinterRail> {
        self.rails[1].clone()
    }

    pub fn home_axis(&self, homing_state: &Rc<Homing>, axis: usize, rail: &Rc<PrinterRail>) -> KResult<()> {
        // Determine movement
        let (position_min, position_max) = rail.get_range();
        let hi = rail.get_homing_info();
        let mut homepos = [None, None, None, None];
        homepos[axis] = Some(hi.position_endstop);
        let mut forcepos = homepos;
        if hi.positive_dir {
            forcepos[axis] = Some(hi.position_endstop - 1.5 * (hi.position_endstop - position_min));
        } else {
            forcepos[axis] = Some(hi.position_endstop + 1.5 * (position_max - hi.position_endstop));
        }
        // Perform homing
        let r: RailRef = rail.clone();
        tr_home_rails(homing_state, &[r], &forcepos, &homepos)
    }

    pub fn check_endstops(&self, mv: &Move) -> KResult<()> {
        let end_pos = &mv.end_pos;
        let limits = self.limits.borrow();
        for i in 0..2 {
            if mv.axes_d[i] != 0.0 && (end_pos[i] < limits[i].0 || end_pos[i] > limits[i].1) {
                if limits[i].0 > limits[i].1 {
                    return Err(mv.move_error("Must home axis first"));
                }
                return Err(mv.move_error("Move out of range"));
            }
        }
        Ok(())
    }
}

impl Kinematics for TradRackKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.rails.iter().flat_map(|r| r.get_steppers()).collect()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        self.rails.iter().map(|r| stepper_positions[&r.get_name(false)]).collect()
    }

    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        for (i, rail) in self.rails.iter().enumerate() {
            rail.set_position(newpos);
            if homing_axes.contains(['x', 'y'][i]) {
                self.limits.borrow_mut()[i] = rail.get_range();
            }
        }
        Ok(())
    }

    /// (TradRackKinematics has no clear_homing_state)
    fn clear_homing_state(&self, _clear_axes: &str) {}

    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()> {
        // Each axis is homed independently and in order
        for axis in homing_state.get_axes() {
            self.home_axis(homing_state, axis, &self.rails[axis])?;
        }
        Ok(())
    }

    fn check_move(&self, mv: &mut Move) -> KResult<()> {
        let (xpos, ypos) = (mv.end_pos[0], mv.end_pos[1]);
        let outside = {
            let limits = self.limits.borrow();
            xpos < limits[0].0 || xpos > limits[0].1 || ypos < limits[1].0 || ypos > limits[1].1
        };
        if outside {
            self.check_endstops(mv)?;
        }

        // Get filament driver speed and accel limits
        let (fil_max_velocity, fil_max_accel) = if (self.is_extruder_synced)() {
            let extruder = self.printer.lookup::<ToolHead>("toolhead")?.get_extruder();
            let pe = extruder
                .downcast_ref::<PrinterExtruder>()
                .ok_or_else(|| KError::Other("'DummyExtruder' object has no attribute 'max_e_velocity'".into()))?;
            (py_min(self.fil_max_velocity, pe.max_e_velocity), py_min(self.fil_max_accel, pe.max_e_accel))
        } else {
            (self.fil_max_velocity, self.fil_max_accel)
        };

        if mv.axes_d[0] != 0.0 && mv.axes_d[1] != 0.0 {
            // Move with both axes - update velocity and accel
            let vel = mv.move_d * py_min(self.sel_max_velocity / mv.axes_d[0].abs(), fil_max_velocity / mv.axes_d[1].abs());
            let accel = mv.move_d * py_min(self.sel_max_accel / mv.axes_d[0].abs(), fil_max_accel / mv.axes_d[1].abs());
            mv.limit_speed(vel, accel);
        } else if mv.axes_d[0] != 0.0 {
            // Move with selector - update velocity and accel
            mv.limit_speed(self.sel_max_velocity, self.sel_max_accel);
        } else if mv.axes_d[1] != 0.0 {
            // Move with filament driver - update velocity and accel
            mv.limit_speed(fil_max_velocity, fil_max_accel);
        }
        Ok(())
    }

    fn rails(&self) -> Vec<RailRef> {
        self.rails.iter().map(|r| r.clone() as RailRef).collect()
    }

    fn get_status(&self, _eventtime: f64) -> PyDict {
        let limits = self.limits.borrow();
        let axes: String = "xy".chars().zip(limits.iter()).filter(|(_, (l, h))| l <= h).map(|(a, _)| a).collect();
        let d = crate::pydict! {
            "homed_axes" => PyVal::Str(axes),
            "selector_min" => PyVal::Float(self.selector_min),
            "selector_max" => PyVal::Float(self.selector_max),
        };
        d
    }

}

/// TradRackHoming.home_rails(rails, forcepos, movepos)
fn tr_home_rails(homing: &Rc<Homing>, rails: &[RailRef], forcepos: &[Option<f64>], movepos: &[Option<f64>]) -> KResult<()> {
    let toolhead = homing.toolhead();
    let printer = homing.printer();
    // Notify of upcoming homing operation
    let rails_v = rails.to_vec();
    crate::events::HomingHomeRailsBegin::emit(&printer, homing, &rails_v)?;
    // Alter kinematics class to think printer is at forcepos
    let homing_axes: Vec<usize> = (0..3).filter(|&axis| forcepos[axis].is_some()).collect();
    let startpos = homing.fill_coord(forcepos);
    let homepos = homing.fill_coord(movepos);
    toolhead.set_position(&startpos, &axes_str(&homing_axes))?;
    // Perform first home
    let endstops: Vec<(EndstopRef, String)> = rails.iter().flat_map(|r| r.get_endstops()).collect();
    let hi = rails[0].get_homing_info();
    let th: Rc<dyn HomingToolhead> = toolhead.clone();
    let mut hmove = HomingMove::new(&printer, endstops.clone(), Some(th.clone()))?;
    hmove.homing_move(&homepos, hi.speed, false, true, true)?;
    // Perform second home
    if hi.retract_dist != 0.0 {
        // Retract
        let startpos = homing.fill_coord(forcepos);
        let homepos = homing.fill_coord(movepos);
        let axes_d: Vec<f64> = homepos.iter().zip(&startpos).map(|(hp, sp)| hp - sp).collect();
        let move_d = crate::pyfmt::py_sum(axes_d[..3].iter().map(|d| d * d)).sqrt();
        let retract_r = py_min(1.0, hi.retract_dist / move_d);
        let retractpos: Vec<f64> = homepos.iter().zip(&axes_d).map(|(hp, ad)| hp - ad * retract_r).collect();
        toolhead.move_to(&retractpos, hi.retract_speed)?;
        // Home again
        let startpos: Vec<f64> = retractpos.iter().zip(&axes_d).map(|(rp, ad)| rp - ad * retract_r).collect();
        toolhead.set_position(&startpos, "")?;
        hmove = HomingMove::new(&printer, endstops, Some(th))?;
        hmove.homing_move(&homepos, hi.second_homing_speed, false, true, true)?;
        if let Some(name) = hmove.check_no_movement() {
            return Err(KError::command(format!("Endstop {} still triggered after retract", name)));
        }
    }
    // Signal home operation complete
    toolhead.flush_step_generation()?;
    homing.set_trigger_positions(hmove.stepper_positions.borrow().iter().map(|sp| (sp.stepper_name.clone(), sp.trig())).collect());
    homing.clear_adjust_pos();
    crate::events::HomingHomeRailsEnd::emit(&printer, homing, &rails_v)?;
    let adjust_pos = homing.adjust_pos();
    if adjust_pos.values().any(|v| *v != 0.0) {
        // Apply any homing offsets
        let kin = toolhead.get_kinematics();
        let mut homepos = toolhead.get_position();
        let kin_spos: IndexMap<String, f64> = kin
            .get_steppers()
            .iter()
            .map(|s| {
                let sname = s.get_name(false);
                let v = s.get_commanded_position() + adjust_pos.get(&sname).copied().unwrap_or(0.0);
                (sname, v)
            })
            .collect();
        let newpos = kin.calc_position(&kin_spos);
        for &axis in &homing_axes {
            homepos[axis] = newpos[axis];
        }
        toolhead.set_position(&homepos, "")?;
    }
    Ok(())
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

pub struct TradRackServo {
    pub servo: Rc<PrinterServo>,
    pub toolhead: Rc<ToolHead>,
}

impl TradRackServo {
    pub fn set_servo(&self, width: Option<f64>, angle: Option<f64>, print_time: Option<f64>) -> KResult<()> {
        let print_time = match print_time {
            Some(t) => t,
            None => self.toolhead.get_last_move_time()?,
        };
        let value = match width {
            Some(w) => self.servo.get_pwm_from_pulse_width(w),
            None => self.servo.get_pwm_from_angle(angle.unwrap_or(0.0)),
        };
        self.servo.send_async_request(value, Some(print_time))
    }

    pub fn get_max_angle(&self) -> f64 {
        self.servo.max_angle()
    }
}

pub struct TradRackLanePositionManager {
    pub lane_count: i64,
    pub lane_spacing: Cell<f64>,
    pub lane_spacing_mods: Vec<f64>,
    pub lane_spacing_mod_internal: f64,
    pub lane_offsets: Vec<f64>,
}

impl TradRackLanePositionManager {
    pub fn new(lane_count: i64, config: &ConfigWrapper) -> KResult<Self> {
        let lane_spacing = config.getfloat("lane_spacing", Dflt::Required, Lim::new().above(0.0))?;
        let mut lane_spacing_mods = vec![];
        let mut lane_spacing_mod_internal = 0.0;
        let mut lane_offsets = vec![];
        for i in 0..lane_count {
            lane_spacing_mods.push(config.getfloat(&format!("lane_spacing_mod_{}", i), 0.0, Lim::new())?);
            lane_spacing_mod_internal += lane_spacing_mods[i as usize];
            lane_offsets.push(config.getfloat(&format!("lane_offset_{}", i), 0.0, Lim::new())?);
        }
        lane_spacing_mod_internal -= lane_spacing_mods[0];
        Ok(TradRackLanePositionManager { lane_count, lane_spacing: Cell::new(lane_spacing), lane_spacing_mods, lane_spacing_mod_internal, lane_offsets })
    }

    pub fn get_lane_positions(&self) -> Vec<f64> {
        let mut lane_positions = vec![];
        let mut curr_pos = 0.0;
        for i in 0..self.lane_count as usize {
            curr_pos += self.lane_spacing_mods[i];
            lane_positions.push(curr_pos + self.lane_offsets[i]);
            curr_pos += self.lane_spacing.get();
        }
        lane_positions
    }

    pub fn process_selector_calibration(&self, endstop_to_lane0: f64, endstop_to_last_lane: f64, lane_spacing_ndigits: usize) -> (f64, f64, Vec<f64>) {
        let n = self.lane_count as usize;
        // account for lane offsets
        let endstop_to_lane0 = endstop_to_lane0 - self.lane_offsets[0];
        let endstop_to_last_lane = endstop_to_last_lane - self.lane_offsets[n - 1];
        // calculate endstop position and lane settings
        let pos_endstop = self.lane_spacing_mods[0] - endstop_to_lane0;
        let lane_span = endstop_to_last_lane - endstop_to_lane0;
        let lane_spacing = (lane_span - self.lane_spacing_mod_internal) / (self.lane_count - 1) as f64;
        self.lane_spacing.set(py_round(lane_spacing, lane_spacing_ndigits));
        let lane_positions = self.get_lane_positions();
        (pos_endstop, self.lane_spacing.get(), lane_positions)
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum SyncState {
    ExtruderToFilDriver,
    FilDriverToExtruder,
}

pub struct TradRackExtruderSyncManager {
    pub printer: Rc<Printer>,
    pub toolhead: RefCell<Option<Rc<ToolHead>>>,
    pub tr_toolhead: Rc<ToolHead>,
    pub fil_driver_rail: Rc<PrinterRail>,
    pub sync_state: Cell<Option<SyncState>>,
    pub prev_sks: RefCell<Vec<Option<SkRef>>>,
    pub prev_trapq: RefCell<Option<TrapQRef>>,
    pub prev_rotation_dists: RefCell<Vec<f64>>,
}

impl TradRackExtruderSyncManager {
    pub fn new(printer: &Rc<Printer>, tr_toolhead: &Rc<ToolHead>, fil_driver_rail: Rc<PrinterRail>) -> Rc<Self> {
        let m = Rc::new(TradRackExtruderSyncManager {
            printer: printer.clone(),
            toolhead: RefCell::new(None),
            tr_toolhead: tr_toolhead.clone(),
            fil_driver_rail,
            sync_state: Cell::new(None),
            prev_sks: RefCell::new(vec![]),
            prev_trapq: RefCell::new(None),
            prev_rotation_dists: RefCell::new(vec![]),
        });
        let mm = m.clone();
        crate::events::KlippyConnect::on(printer, move || {
            *mm.toolhead.borrow_mut() = Some(mm.printer.lookup::<ToolHead>("toolhead")?);
            Ok(())
        });
        m
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.toolhead.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_extruder'".into()))
    }

    fn extruder(&self) -> KResult<Rc<PrinterExtruder>> {
        let ea = self.toolhead()?.get_extruder();
        let any: &dyn Any = &*ea;
        if any.is::<PrinterExtruder>() {
            let obj = self.printer.lookup_object(&ea.get_name())?;
            return crate::printer::downcast_rc::<PrinterExtruder>(obj).ok_or_else(|| KError::Other("extruder has an unexpected type".into()));
        }
        Err(KError::Other("'DummyExtruder' object has no attribute 'extruder_stepper'".into()))
    }

    pub fn get_extruder_mcu_steppers(&self) -> KResult<Vec<Rc<McuStepper>>> {
        let extruder = self.extruder()?;
        let es = extruder.extruder_stepper.clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'stepper'".into()))?;
        Ok(vec![es.stepper.clone()])
    }

    pub fn reset_fil_driver(&self) -> KResult<()> {
        self.tr_toolhead.get_last_move_time()?;
        let mut pos = self.tr_toolhead.get_position();
        pos[1] = 0.0;
        self.tr_toolhead.set_position(&pos, &axes_str(&[1]))?;
        if self.sync_state.get() == Some(SyncState::ExtruderToFilDriver) {
            for stepper in self.get_extruder_mcu_steppers()? {
                stepper.set_position(&[0.0, 0.0, 0.0]);
            }
        }
        Ok(())
    }

    fn sync(&self, sync_type: SyncState) -> KResult<()> {
        self.unsync()?;
        self.toolhead()?.flush_step_generation()?;
        self.tr_toolhead.flush_step_generation()?;

        let (steppers, external_trapq, alloc, new_pos): (Vec<Rc<McuStepper>>, TrapQRef, fn() -> SkRef, Vec<f64>) = match sync_type {
            SyncState::ExtruderToFilDriver => {
                let steppers = self.get_extruder_mcu_steppers()?;
                *self.prev_trapq.borrow_mut() = steppers[0].get_trapq();
                let external_trapq = self.tr_toolhead.get_trapq();
                self.reset_fil_driver()?;
                (steppers, external_trapq, || cartesian_stepper_alloc('y').expect("axis"), vec![0.0, 0.0, 0.0])
            }
            SyncState::FilDriverToExtruder => {
                let steppers = self.fil_driver_rail.get_steppers();
                *self.prev_trapq.borrow_mut() = Some(self.tr_toolhead.get_trapq());
                let extruder = self.extruder()?;
                let external_trapq = extruder.get_trapq_ref();
                (steppers, external_trapq, extruder_stepper_alloc, vec![extruder.last_position.get(), 0.0, 0.0])
            }
        };

        let mut prev_sks = vec![];
        let mut prev_rotation_dists = vec![];
        for stepper in &steppers {
            let stepper_kinematics = alloc();
            prev_rotation_dists.push(stepper.get_rotation_distance().0);
            prev_sks.push(stepper.set_stepper_kinematics(Some(stepper_kinematics)));
            stepper.set_trapq(Some(external_trapq.clone()));
            stepper.set_position(&new_pos);
        }
        *self.prev_sks.borrow_mut() = prev_sks;
        *self.prev_rotation_dists.borrow_mut() = prev_rotation_dists;
        self.sync_state.set(Some(sync_type));
        Ok(())
    }

    pub fn sync_extruder_to_fil_driver(&self) -> KResult<()> {
        self.sync(SyncState::ExtruderToFilDriver)
    }

    pub fn sync_fil_driver_to_extruder(&self) -> KResult<()> {
        self.sync(SyncState::FilDriverToExtruder)?;
        crate::events::TradRackSyncedToExtruder::emit(&self.printer)
    }

    pub fn unsync(&self) -> KResult<()> {
        let Some(state) = self.sync_state.get() else { return Ok(()) };

        self.toolhead()?.flush_step_generation()?;
        self.tr_toolhead.flush_step_generation()?;

        let steppers = match state {
            SyncState::ExtruderToFilDriver => self.get_extruder_mcu_steppers()?,
            SyncState::FilDriverToExtruder => {
                crate::events::TradRackUnsyncingFromExtruder::emit(&self.printer)?;
                self.fil_driver_rail.get_steppers()
            }
        };

        let prev_trapq = self.prev_trapq.borrow().clone();
        let prev_sks = self.prev_sks.borrow().clone();
        let prev_rds = self.prev_rotation_dists.borrow().clone();
        for (i, stepper) in steppers.iter().enumerate() {
            stepper.set_trapq(prev_trapq.clone());
            stepper.set_stepper_kinematics(prev_sks[i].clone());
            stepper.set_rotation_distance(prev_rds[i]);
        }
        self.sync_state.set(None);
        Ok(())
    }

    pub fn is_extruder_synced(&self) -> bool {
        self.sync_state.get() == Some(SyncState::ExtruderToFilDriver)
    }

    pub fn is_fil_driver_synced(&self) -> bool {
        self.sync_state.get() == Some(SyncState::FilDriverToExtruder)
    }

    pub fn set_fil_driver_multiplier(&self, multiplier: f64) -> KResult<()> {
        if !self.is_fil_driver_synced() {
            return Err(KError::Other("Cannot set stepper multiplier when filament driver is not synced to extruder".into()));
        }
        let rds = self.prev_rotation_dists.borrow().clone();
        for (i, stepper) in self.fil_driver_rail.get_steppers().iter().enumerate() {
            stepper.set_rotation_distance(rds[i] / multiplier);
        }
        Ok(())
    }
}

struct RunIfNoActivity;

impl RunIfNoActivity {
    fn start(toolhead: &Rc<ToolHead>, reactor: &Rc<Reactor>, callback: Rc<dyn Fn() -> KResult<()>>, delay: f64) -> KResult<()> {
        let initial_print_time = toolhead.get_last_move_time()?;
        let th = toolhead.clone();
        reactor.register_callback(
            move |eventtime| {
                let (print_time, _, lookahead_empty) = th.check_busy(eventtime);
                if lookahead_empty && print_time == initial_print_time {
                    if let Err(e) = callback() {
                        logging::exception_err("trad_rack: RunIfNoActivity callback", &e);
                    }
                }
            },
            reactor.monotonic() + delay,
        );
        Ok(())
    }
}

pub struct MovingAverageFilter {
    pub max_entries: usize,
    pub queue: RefCell<VecDeque<f64>>,
    pub total: Cell<f64>,
}

impl MovingAverageFilter {
    pub fn new(max_entries: usize) -> Self {
        MovingAverageFilter { max_entries, queue: RefCell::new(VecDeque::new()), total: Cell::new(0.0) }
    }

    pub fn update(&self, value: f64) -> f64 {
        let mut queue = self.queue.borrow_mut();
        if queue.len() >= self.max_entries {
            self.total.set(self.total.get() - queue.pop_front().unwrap_or(0.0));
        }
        self.total.set(self.total.get() + value);
        queue.push_back(value);
        self.total.get() / queue.len() as f64
    }

    pub fn reset(&self) {
        self.queue.borrow_mut().clear();
        self.total.set(0.0);
    }

    pub fn get_entry_count(&self) -> usize {
        self.queue.borrow().len()
    }
}

pub struct TradRackRunoutSensor {
    pub printer: Rc<Printer>,
    pub reactor: Rc<Reactor>,
    pub runout_callback: Rc<dyn Fn(f64) -> KResult<()>>,
    pub active: Cell<bool>,
}

impl TradRackRunoutSensor {
    fn new(config: &ConfigWrapper, runout_callback: Rc<dyn Fn(f64) -> KResult<()>>, pin: &str) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // disable config checks for duplicate pins
        let mut pin_desc = pin;
        if pin_desc.starts_with('^') || pin_desc.starts_with('~') {
            pin_desc = crate::pyfmt::py_strip(&pin_desc[1..]);
        }
        if pin_desc.starts_with('!') {
            pin_desc = crate::pyfmt::py_strip(&pin_desc[1..]);
        }
        let ppins = printer.lookup::<PinsObject>("pins")?;
        ppins.0.borrow_mut().allow_multi_use_pin(pin_desc).map_err(|e| KError::Pins(e.0))?;

        let s = Rc::new(TradRackRunoutSensor { printer: printer.clone(), reactor: printer.get_reactor(), runout_callback, active: Cell::new(false) });
        // register button
        let buttons = printer.load_as::<PrinterButtons>(config, "buttons")?;
        let w = Rc::downgrade(&s);
        buttons.register_buttons(
            &[pin],
            Rc::new(move |eventtime, state| match w.upgrade() {
                Some(s) => s.sensor_callback(eventtime, state != 0),
                None => Ok(()),
            }),
        )?;
        Ok(s)
    }

    pub fn sensor_callback(&self, eventtime: f64, state: bool) -> KResult<()> {
        if self.active.get() && !state {
            let idle_timeout = self.printer.lookup_object("idle_timeout")?;
            let printing = match idle_timeout.get_status(eventtime) {
                Some(PyVal::Dict(d)) => matches!(d.get("state"), Some(PyVal::Str(s)) if s == "Printing"),
                _ => false,
            };
            if printing {
                self.active.set(false);
                let cb = self.runout_callback.clone();
                self.reactor.register_callback(
                    move |eventtime| {
                        if let Err(e) = cb(eventtime) {
                            logging::exception_err("trad_rack runout", &e);
                        }
                    },
                    NOW,
                );
            }
        }
        Ok(())
    }

    pub fn set_active(&self, active: bool) {
        self.active.set(active);
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(TradRack::new(config)?)
}
