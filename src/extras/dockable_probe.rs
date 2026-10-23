// Dockable Probe (port of klippy/extras/dockable_probe.py)
//   This provides support for probes that are magnetically coupled
//   to the toolhead and stowed in a dock when not in use and
//
// Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2021       Paul McGowan <mental405@gmail.com>
// Copyright (C) 2023       Alan Smith <alan@airpost.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::extras::homing::{HomingMove, PrinterHoming};
use crate::extras::probe::{McuProbe, Multi, PrinterProbe};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mathutil::py_hypot;
use crate::mcu::{Endstop, EndstopRef, Mcu, McuEndstop, StepperRef};
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_min, py_str_repr};
use crate::pyval::PyVal;
use crate::reactor::{ReactorCompletion, NEVER, NOW};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::rc::{Rc, Weak};

const PROBE_VERIFY_DELAY: f64 = 0.1;

const PROBE_UNKNOWN: i64 = 0;
const PROBE_ATTACHED: i64 = 1;
const PROBE_DOCKED: i64 = 2;

const HINT_VERIFICATION_ERROR: &str = "
{0}: A probe attachment verification method
was not provided. A method to verify the probes attachment
state must be specified to prevent unintended behavior.

At least one of the following must be specified:
'check_open_attach', 'probe_sense_pin', 'dock_sense_pin'

Please see {0}.md and config_Reference.md.
";

const VIRTUAL_Z_ENDSTOP_ERROR: &str = "
DockableProbe cannot be used as Z endstop if a z position
is defined in approach/dock/extract.\"
";

/// a coordinate as _parse_coord() returns it: [x, y, z] with None for a
/// missing z
type Coord = [Option<f64>; 3];
type Point = (f64, f64);

fn xy(c: &Coord) -> Point {
    (c[0].unwrap_or(0.0), c[1].unwrap_or(0.0))
}

// Helper class to handle polling pins for probe attachment states
struct PinPollingHelper {
    printer: Rc<Printer>,
    query_endstop: EndstopRef,
    last_verify_time: Cell<f64>,
    last_verify_state: Cell<Option<bool>>,
}

impl PinPollingHelper {
    fn new(config: &ConfigWrapper, endstop: EndstopRef) -> Rc<Self> {
        Rc::new(PinPollingHelper {
            printer: config.get_printer(),
            query_endstop: endstop,
            last_verify_time: Cell::new(0.0),
            last_verify_state: Cell::new(None),
        })
    }

    fn query_pin(&self, curtime: f64) -> KResult<bool> {
        if curtime > self.last_verify_time.get() + PROBE_VERIFY_DELAY || self.last_verify_state.get().is_none() {
            self.last_verify_time.set(curtime);
            let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
            let query_time = toolhead.get_last_move_time()?;
            self.last_verify_state.set(Some(self.query_endstop.query_endstop(query_time)? != 0));
        }
        Ok(self.last_verify_state.get().unwrap_or(false))
    }
}

/// a pin query: helper.query_pin, or helper.query_pin_inv
#[derive(Clone)]
struct PinQuery {
    helper: Rc<PinPollingHelper>,
    inverted: bool,
}

impl PinQuery {
    fn query(&self, curtime: f64) -> KResult<bool> {
        Ok(self.helper.query_pin(curtime)? != self.inverted)
    }
}

// Helper class to verify probe attachment status
struct ProbeState {
    printer: Rc<Printer>,
    probe_sense_pin: PinQuery,
    dock_sense_pin: Option<PinQuery>,
    last_verify_time: Cell<f64>,
    last_verify_state: Cell<i64>,
}

impl ProbeState {
    fn new(config: &ConfigWrapper, name: &str, probe_endstop: EndstopRef) -> KResult<Rc<Self>> {
        let printer = config.get_printer();

        if !config.fileconfig_has_option("check_open_attach")
            && !config.fileconfig_has_option("probe_sense_pin")
            && !config.fileconfig_has_option("dock_sense_pin")
        {
            return Err(KError::config(HINT_VERIFICATION_ERROR.replace("{0}", name)));
        }

        let slot: Rc<RefCell<Weak<ProbeState>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                if let Some(s) = sl.borrow().upgrade() {
                    s.handle_ready();
                }
                Ok(())
            }),
        );

        // Configure sense pins as endstops so they
        // can be polled at specific times
        let config_endstop = |pin: &str| -> KResult<Rc<PinPollingHelper>> {
            let mcu_endstop = setup_endstop(&printer, pin)?;
            Ok(PinPollingHelper::new(config, mcu_endstop))
        };

        // Setup sensor pins, if configured, otherwise use probe endstop
        // as a dummy sensor.
        let ehelper = PinPollingHelper::new(config, probe_endstop);

        // Probe sense pin is optional
        let mut probe_sense_pin = match config.get_opt("probe_sense_pin")? {
            Some(pin) => PinQuery { helper: config_endstop(&pin)?, inverted: false },
            None => PinQuery { helper: ehelper.clone(), inverted: true },
        };

        // If check_open_attach is specified, it takes precedence
        // over probe_sense_pin
        if config.fileconfig_has_option("check_open_attach") {
            let check_open_attach = config.getboolean("check_open_attach", Dflt::Required)?;
            probe_sense_pin = PinQuery { helper: ehelper.clone(), inverted: check_open_attach };
        }

        // Dock sense pin is optional
        let dock_sense_pin = match config.get_opt("dock_sense_pin")? {
            Some(pin) => Some(PinQuery { helper: config_endstop(&pin)?, inverted: false }),
            None => None,
        };
        let s = Rc::new(ProbeState {
            printer,
            probe_sense_pin,
            dock_sense_pin,
            last_verify_time: Cell::new(0.0),
            last_verify_state: Cell::new(PROBE_UNKNOWN),
        });
        *slot.borrow_mut() = Rc::downgrade(&s);
        Ok(s)
    }

    fn handle_ready(&self) {
        self.last_verify_time.set(0.0);
        self.last_verify_state.set(PROBE_UNKNOWN);
    }

    fn get_probe_state(&self) -> KResult<i64> {
        let curtime = self.printer.get_reactor().monotonic();
        self.get_probe_state_with_time(curtime)
    }

    fn get_probe_state_with_time(&self, curtime: f64) -> KResult<i64> {
        if self.last_verify_state.get() == PROBE_UNKNOWN || curtime > self.last_verify_time.get() + PROBE_VERIFY_DELAY {
            self.last_verify_time.set(curtime);
            self.last_verify_state.set(PROBE_UNKNOWN);

            let a = self.probe_sense_pin.query(curtime)?;

            match &self.dock_sense_pin {
                Some(dock) => {
                    let d = dock.query(curtime)?;
                    if a && !d {
                        self.last_verify_state.set(PROBE_ATTACHED);
                    } else if d && !a {
                        self.last_verify_state.set(PROBE_DOCKED);
                    }
                }
                None => {
                    self.last_verify_state.set(if a { PROBE_ATTACHED } else { PROBE_DOCKED });
                }
            }
        }
        Ok(self.last_verify_state.get())
    }
}

/// ppins.lookup_pin(pin, can_invert=True, can_pullup=True) set up as an endstop
fn setup_endstop(printer: &Printer, pin: &str) -> KResult<Rc<McuEndstop>> {
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let pin_params = ppins.0.borrow_mut().lookup_pin(pin, true, true, None).map_err(|e| KError::Pins(e.0))?;
    let chip = pin_params.borrow().chip.clone();
    chip.setup_pin("endstop", &pin_params)
        .map_err(KError::Pins)?
        .downcast::<McuEndstop>()
        .map_err(|_| KError::Other("probe pin chip has no mcu endstop".into()))
}

const CMD_QUERY_DOCKABLE_PROBE_HELP: &str = "Prints the current probe state, valid probe states are UNKNOWN, ATTACHED, and DOCKED";
const CMD_MOVE_TO_APPROACH_PROBE_HELP: &str = "Move close to the probe dock before attaching";
const CMD_MOVE_TO_DOCK_PROBE_HELP: &str = "Move to connect the toolhead/dock to the probe";
const CMD_MOVE_TO_EXTRACT_PROBE_HELP: &str = "Move away from the dock with the probe attached";
const CMD_MOVE_TO_INSERT_PROBE_HELP: &str = "Move near the dock with the probe attached before detaching";
const CMD_MOVE_TO_DETACH_PROBE_HELP: &str = "Move away from the dock to detach the probe";
const CMD_SET_DOCKABLE_PROBE_HELP: &str = "Set probe parameters";
const CMD_ATTACH_PROBE_HELP: &str = "Check probe status and attach probe using the movement gcodes";
const CMD_DETACH_PROBE_HELP: &str = "Check probe status and detach probe using the movement gcodes";
const CMD_MOVE_AVOIDING_DOCK_HELP: &str = "Move to X Y avoiding dock safe area";

pub struct DockableProbe {
    me: Weak<DockableProbe>,
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    position_endstop: f64,
    speed: f64,
    lift_speed: f64,
    dock_retries: i64,
    pub detach_dockable_before_z_home: bool,
    auto_attach_detach: Cell<bool>,
    restore_toolhead: bool,
    travel_speed: f64,
    attach_speed: f64,
    detach_speed: f64,
    approach_position: Coord,
    detach_position: Coord,
    extract_position: Coord,
    insert_position: Coord,
    dock_position: Coord,
    z_hop: f64,
    dock_requires_z: bool,
    safe_dock_distance: f64,
    safe_points: Vec<Point>,
    activate_gcode: Rc<Template>,
    deactivate_gcode: Rc<Template>,
    pre_attach_gcode: Rc<Template>,
    post_attach_gcode: Rc<Template>,
    pre_detach_gcode: Rc<Template>,
    post_detach_gcode: Rc<Template>,
    mcu_endstop: Rc<McuEndstop>,
    finish_home_complete: RefCell<Option<Rc<ReactorCompletion<bool>>>>,
    wait_trigger_complete: RefCell<Option<Rc<ReactorCompletion<()>>>>,
    last_z: Cell<f64>,
    multi: Cell<Multi>,
    last_homed: RefCell<String>,
    pstate: Rc<ProbeState>,
    last_probe_state: Cell<i64>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
}

/// _parse_coord(config, name, default=None, expected_dims=3)
fn parse_coord(config: &ConfigWrapper, name: &str, default: Option<Coord>) -> KResult<Option<Coord>> {
    let val = if default.is_some() { config.get_opt(name)? } else { Some(config.get(name, Dflt::Required)?) };
    let error_msg = |m: &str| KError::config(format!("Unable to parse {} in {}: {}", name, config.get_name(), m));
    let val = match val {
        Some(v) if !v.is_empty() => v,
        _ => return Ok(default),
    };
    let mut vals = vec![];
    for x in val.split(',') {
        let x = x.trim();
        match py_float(x) {
            Some(f) => vals.push(f),
            None => {
                let e = format!("could not convert string to float: {}", py_str_repr(x));
                return Err(error_msg(&e).with_context(KError::Py("ValueError".into(), e)));
            }
        }
    }
    if !(2..=3).contains(&vals.len()) {
        return Err(error_msg("Invalid number of coordinates"));
    }
    let mut p = [None; 3];
    for (i, v) in vals.into_iter().enumerate() {
        p[i] = Some(v);
    }
    Ok(Some(p))
}

/// a required coordinate (python subscripts the None and fails)
fn need(c: Option<Coord>) -> KResult<Coord> {
    c.ok_or_else(|| KError::Other("'NoneType' object is not subscriptable".into()))
}

// Determine the vector of two points
fn get_distance(point1: Point, point2: Point) -> f64 {
    py_hypot(&[point2.0 - point1.0, point2.1 - point1.1])
}

fn get_angle(point1: Point, point2: Point) -> f64 {
    (point2.1 - point1.1).atan2(point2.0 - point1.0) + PI
}

// Find a point on a vector line at a specific distance
fn get_point_on_vector(point: Point, angle: f64, magnitude: f64) -> Point {
    (point.0 - magnitude * angle.cos(), point.1 - magnitude * angle.sin())
}

// Determine tangent points to a circle from external point
fn get_tangent_points(radius: f64, center: Point, point2: Point) -> Option<Vec<Point>> {
    let (cx, cy) = center;
    let d = get_distance(center, point2);
    let angle = get_angle(center, point2) - PI;
    if d < radius {
        return None;
    }
    let dev = (radius / d).acos();
    Some(vec![
        (cx + radius * (angle + dev).cos(), cy + radius * (angle + dev).sin()),
        (cx + radius * (angle - dev).cos(), cy + radius * (angle - dev).sin()),
    ])
}

// Determine closest point
fn get_closest_point(target_point: Point, points: &[Point]) -> Option<Point> {
    let distances: Vec<f64> = points.iter().map(|(x, y)| py_hypot(&[x - target_point.0, y - target_point.1])).collect();
    // find index of the closest point
    let min = distances.iter().copied().reduce(py_min)?;
    let index = distances.iter().position(|d| *d == min)?;
    Some(points[index])
}

impl DockableProbe {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let name = config.get_name().to_string();

        // Configuration Options
        let position_endstop = config.getfloat("z_offset", Dflt::Required, Lim::new())?;
        config.getfloat("x_offset", 0.0, Lim::new())?;
        config.getfloat("y_offset", 0.0, Lim::new())?;
        let speed = config.getfloat("speed", 5.0, Lim::new().above(0.0))?;
        let lift_speed = config.getfloat("lift_speed", speed, Lim::new().above(0.0))?;
        let dock_retries = config.getint("dock_retries", 0, Lim::new())?;
        let auto_attach_detach = config.getboolean("auto_attach_detach", true)?;
        let restore_toolhead = config.getboolean("restore_toolhead", true)?;
        let travel_speed = config.getfloat("travel_speed", speed, Lim::new().above(0.0))?;
        let attach_speed = config.getfloat("attach_speed", travel_speed, Lim::new().above(0.0))?;
        let detach_speed = config.getfloat("detach_speed", travel_speed, Lim::new().above(0.0))?;
        config.getfloat("sample_retract_dist", 2.0, Lim::new().above(0.0))?;
        let detach_dockable_before_z_home = config.getboolean("detach_dockable_before_z_home", false)?;

        // Positions (approach, detach, etc)
        let approach_position = parse_coord(config, "approach_position", None)?;
        let detach_position = parse_coord(config, "detach_position", None)?;
        let extract_position = parse_coord(config, "extract_position", approach_position)?;
        let insert_position = parse_coord(config, "insert_position", extract_position)?;
        let dock_position = parse_coord(config, "dock_position", None)?;
        let z_hop = config.getfloat("z_hop", 0.0, Lim::new().above(0.0))?;

        let (approach_position, dock_position) = (need(approach_position)?, need(dock_position)?);
        let dock_requires_z = approach_position[2].is_some() || dock_position[2].is_some();
        // Entry distance
        let (detach_position, insert_position) = (need(detach_position)?, need(insert_position)?);
        let approach_distance = get_distance(xy(&dock_position), xy(&approach_position));
        let detach_distance = get_distance(xy(&dock_position), xy(&detach_position));
        let insert_distance = get_distance(xy(&dock_position), xy(&insert_position));

        // Helpers to avoid dock and moves out of range ()
        let max_safe_distance = py_min(py_min(approach_distance, detach_distance), insert_distance);
        let safe_dock_distance = config.getfloat("safe_dock_distance", max_safe_distance, Lim::new().max(max_safe_distance))?;
        let safe_position = need(parse_coord(config, "safe_position", Some(approach_position))?)?;
        // _get_safe_points(safe_position)
        let angle = get_angle(xy(&dock_position), xy(&safe_position));
        let safe_points = vec![
            get_point_on_vector(xy(&dock_position), angle + PI / 4.0, safe_dock_distance),
            get_point_on_vector(xy(&dock_position), angle - PI / 4.0, safe_dock_distance),
        ];

        // Macros to run before attach and after detach
        let gcode_macro = load_gcode_macro(config)?;
        let activate_gcode = gcode_macro.load_template(config, "activate_gcode", Some(""))?;
        let deactivate_gcode = gcode_macro.load_template(config, "deactivate_gcode", Some(""))?;
        let pre_attach_gcode = gcode_macro.load_template(config, "pre_attach_gcode", Some(""))?;
        let post_attach_gcode = gcode_macro.load_template(config, "post_attach_gcode", Some(""))?;
        let pre_detach_gcode = gcode_macro.load_template(config, "pre_detach_gcode", Some(""))?;
        let post_detach_gcode = gcode_macro.load_template(config, "post_detach_gcode", Some(""))?;

        // Pins
        let pin = config.get("pin", Dflt::Required)?;
        let mcu_endstop = setup_endstop(&printer, &pin)?;

        let pstate = ProbeState::new(config, &name, mcu_endstop.clone())?;

        let p = Rc::new_cyclic(|me| DockableProbe {
            me: me.clone(),
            printer: printer.clone(),
            gcode: gcode.clone(),
            position_endstop,
            speed,
            lift_speed,
            dock_retries,
            detach_dockable_before_z_home,
            auto_attach_detach: Cell::new(auto_attach_detach),
            restore_toolhead,
            travel_speed,
            attach_speed,
            detach_speed,
            approach_position,
            detach_position,
            extract_position: extract_position.unwrap_or(approach_position),
            insert_position,
            dock_position,
            z_hop,
            dock_requires_z,
            safe_dock_distance,
            safe_points,
            activate_gcode,
            deactivate_gcode,
            pre_attach_gcode,
            post_attach_gcode,
            pre_detach_gcode,
            post_detach_gcode,
            mcu_endstop,
            finish_home_complete: RefCell::new(None),
            wait_trigger_complete: RefCell::new(None),
            // State
            last_z: Cell::new(-9999.0),
            multi: Cell::new(Multi::Off),
            last_homed: RefCell::new(String::new()),
            pstate,
            last_probe_state: Cell::new(PROBE_UNKNOWN),
            toolhead: RefCell::new(None),
        });

        // Gcode Commands
        type DCmd = fn(&DockableProbe, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, DCmd, &str); 10] = [
            ("QUERY_DOCKABLE_PROBE", DockableProbe::cmd_query_dockable_probe, CMD_QUERY_DOCKABLE_PROBE_HELP),
            ("MOVE_TO_APPROACH_PROBE", DockableProbe::cmd_move_to_approach_probe, CMD_MOVE_TO_APPROACH_PROBE_HELP),
            ("MOVE_TO_DOCK_PROBE", DockableProbe::cmd_move_to_dock_probe, CMD_MOVE_TO_DOCK_PROBE_HELP),
            ("MOVE_TO_EXTRACT_PROBE", DockableProbe::cmd_move_to_extract_probe, CMD_MOVE_TO_EXTRACT_PROBE_HELP),
            ("MOVE_TO_INSERT_PROBE", DockableProbe::cmd_move_to_insert_probe, CMD_MOVE_TO_INSERT_PROBE_HELP),
            ("MOVE_TO_DETACH_PROBE", DockableProbe::cmd_move_to_detach_probe, CMD_MOVE_TO_DETACH_PROBE_HELP),
            ("SET_DOCKABLE_PROBE", DockableProbe::cmd_set_dockable_probe, CMD_SET_DOCKABLE_PROBE_HELP),
            ("ATTACH_PROBE", DockableProbe::cmd_attach_probe, CMD_ATTACH_PROBE_HELP),
            ("DETACH_PROBE", DockableProbe::cmd_detach_probe, CMD_DETACH_PROBE_HELP),
            ("MOVE_AVOIDING_DOCK", DockableProbe::cmd_move_avoiding_dock, CMD_MOVE_AVOIDING_DOCK_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let w = p.me.clone();
            gcode.register_command(
                cmd,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(p) => f(&p, g),
                    None => Ok(()),
                })),
                false,
                Some(desc),
            )?;
        }

        // Event Handlers
        let w = p.me.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match w.upgrade() {
                Some(p) => p.handle_connect(),
                None => Ok(()),
            }),
        );
        let w = p.me.clone();
        printer.register_event_handler(
            "klippy:mcu_identify",
            Rc::new(move |_| match w.upgrade() {
                Some(p) => p.handle_config(),
                None => Ok(()),
            }),
        );
        Ok(p)
    }

    fn handle_config(&self) -> KResult<()> {
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        for stepper in kin.get_steppers() {
            if stepper.is_active_axis('z') {
                self.mcu_endstop.add_stepper(&stepper)?;
            }
        }
        Ok(())
    }

    fn handle_connect(&self) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        *self.toolhead.borrow_mut() = Some(toolhead.clone());
        let me: EndstopRef = self.me.upgrade().expect("dockable_probe");
        let me_ptr = Rc::as_ptr(&me) as *const ();
        let in_endstops = toolhead
            .get_kinematics()
            .rails()
            .iter()
            .flat_map(|r| r.get_endstops())
            .any(|(es, _)| Rc::as_ptr(&es) as *const () == me_ptr);
        let positions = [self.approach_position, self.dock_position, self.extract_position];
        if in_endstops && positions.iter().any(|pos| pos[2].is_some()) {
            return Err(KError::config(VIRTUAL_Z_ENDSTOP_ERROR));
        }
        Ok(())
    }

    fn th(&self) -> KResult<Rc<ToolHead>> {
        self.toolhead.borrow().clone().ok_or_else(|| KError::Other("'DockableProbe' object has no attribute 'toolhead'".into()))
    }

    fn get_probe_state(&self) -> KResult<i64> {
        self.pstate.get_probe_state()
    }

    // GCode Commands

    fn cmd_query_dockable_probe(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.last_probe_state.set(self.get_probe_state()?);
        let state = match self.last_probe_state.get() {
            PROBE_ATTACHED => "ATTACHED",
            PROBE_DOCKED => "DOCKED",
            _ => "UNKNOWN",
        };
        gcmd.respond_info(&format!("Probe Status: {}", state), true);
        Ok(())
    }

    fn cmd_move_to_approach_probe(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.align_z()?;
        self.move_avoiding_dock(xy(&self.approach_position), self.travel_speed)?;
        self.th()?.manual_move(&[None, None, self.approach_position[2]], self.speed)
    }

    fn cmd_move_to_dock_probe(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let th = self.th()?;
        th.manual_move(&[None, None, self.dock_position[2]], self.speed)?;
        th.manual_move(&[self.dock_position[0], self.dock_position[1], None], self.attach_speed)
    }

    fn cmd_move_to_extract_probe(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let th = self.th()?;
        th.manual_move(&[None, None, self.extract_position[2]], self.speed)?;
        th.manual_move(&[self.extract_position[0], self.extract_position[1], None], self.attach_speed)
    }

    fn cmd_move_to_insert_probe(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        self.move_avoiding_dock(xy(&self.insert_position), self.travel_speed)?;
        self.th()?.manual_move(&[None, None, self.insert_position[2]], self.speed)
    }

    fn cmd_move_to_detach_probe(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let th = self.th()?;
        th.manual_move(&[None, None, self.detach_position[2]], self.lift_speed)?;
        th.manual_move(&[self.detach_position[0], self.detach_position[1], None], self.detach_speed)
    }

    fn cmd_set_dockable_probe(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let Some(auto) = gcmd.get_opt("AUTO_ATTACH_DETACH") else { return Ok(()) };
        let v = auto
            .trim()
            .parse::<i64>()
            .map_err(|_| KError::Other(format!("invalid literal for int() with base 10: {}", py_str_repr(&auto))))?;
        self.auto_attach_detach.set(v == 1);
        Ok(())
    }

    fn cmd_attach_probe(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let return_pos = self.th()?.get_position();
        self.attach_probe(Some(&return_pos), false)
    }

    fn cmd_detach_probe(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let return_pos = self.th()?.get_position();
        self.detach_probe(Some(&return_pos))
    }

    fn cmd_move_avoiding_dock(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let pos = self.th()?.get_position();
        let x = gcmd.get_float("X", Some(pos[0]), Lim::new())?;
        let y = gcmd.get_float("Y", Some(pos[1]), Lim::new())?;
        let speed = gcmd.get_float("SPEED", Some(self.travel_speed), Lim::new().above(0.0))?;
        self.move_avoiding_dock((x, y), speed)
    }

    fn attach_probe(&self, return_pos: Option<&[f64]>, always_restore_toolhead: bool) -> KResult<()> {
        self.lower_probe()?;

        let mut retry = 0;
        while self.get_probe_state()? != PROBE_ATTACHED && retry < self.dock_retries + 1 {
            if self.get_probe_state()? != PROBE_DOCKED {
                return Err(KError::command("Attach Probe: Probe not detected in dock, aborting"));
            }
            self.pre_attach_gcode.run_gcode_from_command(None)?;
            // Call these gcodes as a script because we don't have enough
            // structs/data to call the cmd_...() funcs and supply 'gcmd'.
            // This method also has the advantage of calling user-written gcodes
            // if they've been defined.
            self.gcode.run_script_from_command(
                "\n                MOVE_TO_APPROACH_PROBE\n                MOVE_TO_DOCK_PROBE\n                MOVE_TO_EXTRACT_PROBE\n            ",
            )?;
            self.post_attach_gcode.run_gcode_from_command(None)?;

            retry += 1;
        }

        if self.get_probe_state()? != PROBE_ATTACHED {
            return Err(KError::command("Probe attach failed!"));
        }

        if let Some(rp) = return_pos.filter(|rp| !rp.is_empty()) {
            if self.restore_toolhead || always_restore_toolhead {
                // return to the original XY position, if inside safe_dock area
                // move to the closest point
                self.move_avoiding_dock((rp[0], rp[1]), self.travel_speed)?;
                // Do NOT return to the original Z position after attach
                // as the probe might crash into the bed.
            }
        }
        Ok(())
    }

    pub fn detach_probe(&self, return_pos: Option<&[f64]>) -> KResult<()> {
        let mut retry = 0;
        while self.get_probe_state()? != PROBE_DOCKED && retry < self.dock_retries + 1 {
            self.pre_detach_gcode.run_gcode_from_command(None)?;
            // Call these gcodes as a script because we don't have enough
            // structs/data to call the cmd_...() funcs and supply 'gcmd'.
            // This method also has the advantage of calling user-written gcodes
            // if they've been defined.
            self.gcode.run_script_from_command(
                "\n                MOVE_TO_INSERT_PROBE\n                MOVE_TO_DOCK_PROBE\n                MOVE_TO_DETACH_PROBE\n            ",
            )?;
            self.post_detach_gcode.run_gcode_from_command(None)?;

            retry += 1;
        }

        if self.get_probe_state()? != PROBE_DOCKED {
            return Err(KError::command("Probe detach failed!"));
        }

        if let Some(rp) = return_pos.filter(|rp| !rp.is_empty()) {
            if self.restore_toolhead {
                // return to the original XY position, if inside safe_dock area
                // move to the closest point
                self.move_avoiding_dock((rp[0], rp[1]), self.travel_speed)?;
                // Return to original Z position after detach as
                // there's no chance of the probe crashing into the bed.
                self.th()?.manual_move(&[None, None, Some(rp[2])], self.lift_speed)?;
            }
        }
        self.raise_probe()
    }

    fn auto_detach_probe(&self, return_pos: Option<&[f64]>) -> KResult<()> {
        if self.get_probe_state()? == PROBE_DOCKED {
            return Ok(());
        }
        if self.auto_attach_detach.get() {
            self.detach_probe(return_pos)?;
        }
        Ok(())
    }

    fn auto_attach_probe(&self, return_pos: Option<&[f64]>, always_restore_toolhead: bool) -> KResult<()> {
        if self.get_probe_state()? == PROBE_ATTACHED {
            return Ok(());
        }
        if !self.auto_attach_detach.get() {
            return Err(KError::command("Cannot probe, probe is not attached and auto-attach is disabled"));
        }
        self.attach_probe(return_pos, always_restore_toolhead)
    }

    // Functions for calculating points and moving the toolhead

    // Move to position avoiding the dock
    fn move_avoiding_dock(&self, end_point: Point, speed: f64) -> KResult<()> {
        let th = self.th()?;
        let pos = th.get_position();
        let mut start_point = (pos[0], pos[1]);
        let mut end_point = end_point;
        let dock = xy(&self.dock_position);
        if start_point == end_point {
            return Ok(());
        }
        let radius = self.safe_dock_distance;
        if radius == 0.0 {
            return th.manual_move(&[Some(end_point.0), Some(end_point.1), None], speed);
        }

        // redefine start_point outside safe dock area
        let mut coords = vec![];
        if radius > get_distance(dock, start_point) {
            start_point = self.get_closest_exitpoint(start_point, end_point)?;
            coords.push(start_point);
        }

        // Check if trajectory intersect safe dock area
        let intersect_points = self.get_intersect_points(start_point, end_point);

        // Define endpoint
        if intersect_points.len() == 1 {
            end_point = intersect_points[0];
        // Calculate trajectory around the dock
        } else if !intersect_points.is_empty() {
            let none = || KError::Other("cannot unpack non-iterable NoneType object".into());
            // input_tangent point
            let safe_point = get_closest_point(end_point, &self.safe_points).ok_or_else(none)?;
            let tangent_points = get_tangent_points(radius - 0.1, dock, start_point).unwrap_or_default();
            let arc_input = get_closest_point(safe_point, &tangent_points).ok_or_else(none)?;
            let clockwise = self.get_rotation_direction(start_point, arc_input);
            coords.push(arc_input);

            // output tangent point
            let safe_point = get_closest_point(start_point, &self.safe_points).ok_or_else(none)?;
            let tangent_points = get_tangent_points(radius, dock, end_point).unwrap_or_default();
            let arc_output = get_closest_point(safe_point, &tangent_points).ok_or_else(none)?;
            // determine arc travel
            coords.extend(arc_points(arc_input, arc_output, dock, clockwise));
        }
        coords.push(end_point);

        for (x, y) in coords {
            th.manual_move(&[Some(x), Some(y), None], speed)?;
        }
        Ok(())
    }

    // determine closest exit point X or Y while toolhead inside safe_dock_zone
    fn get_closest_exitpoint(&self, point1: Point, point2: Point) -> KResult<Point> {
        let (cx, cy) = xy(&self.dock_position);
        // Choose point2 if point1 is the dock position
        let (dx, dy, reference_point) = if point1 != (cx, cy) {
            (point1.0 - cx, point1.1 - cy, point1)
        } else if point2 != (cx, cy) {
            (point2.0 - cx, point2.1 - cy, point2)
        } else {
            return Err(KError::command("_move_avoiding_dock : Unable to determine exit point"));
        };
        let d = py_hypot(&[dx, dy]);
        // Ensure exit point is outside dock area.
        let magnitude = self.safe_dock_distance + 10e-8;
        let x1 = cx + magnitude * dx / d;
        let y1 = cy + magnitude * dy / d;
        let x2 = cx - magnitude * dx / d;
        let y2 = cy - magnitude * dy / d;
        Ok(get_closest_point(reference_point, &[(x1, y1), (x2, y2)]).unwrap_or((x1, y1)))
    }

    // determine intersect points between a line and a circle
    fn get_intersect_points(&self, point1: Point, point2: Point) -> Vec<Point> {
        let (x1, y1) = point1;
        let (x2, y2) = point2;
        let (cx, cy) = xy(&self.dock_position);
        let r = self.safe_dock_distance - 10e-5; // fix floating point issue

        let (dx, dy) = (x2 - x1, y2 - y1);
        let a = dx.powi(2) + dy.powi(2);
        let b = 2.0 * (dx * (x1 - cx) + dy * (y1 - cy));
        let c = (x1 - cx).powi(2) + (y1 - cy).powi(2) - r.powi(2);
        let disc = b.powi(2) - 4.0 * a * c;

        if disc < 0.0 || a == 0.0 {
            return vec![]; // Nothing to return
        }

        let t1 = (-b + disc.sqrt()) / (2.0 * a);
        let t2 = (-b - disc.sqrt()) / (2.0 * a);

        let mut intersec = vec![];
        if (0.0..=1.0).contains(&t1) {
            intersec.push((x1 + t1 * dx, y1 + t1 * dy));
        }
        if (0.0..=1.0).contains(&t2) && t1 != t2 {
            intersec.push((x1 + t2 * dx, y1 + t2 * dy));
        }
        intersec
    }

    fn get_rotation_direction(&self, point1: Point, point2: Point) -> bool {
        let dock = xy(&self.dock_position);
        let angle1 = get_angle(point1, dock);
        let angle2 = get_angle(point2, dock);
        // Direction of rotation
        let angle_diff = (angle2 - angle1).rem_euclid(2.0 * PI);
        angle_diff > PI
    }

    // Align z axis to prevent crashes
    fn align_z(&self) -> KResult<()> {
        let th = self.th()?;
        let curtime = self.printer.get_reactor().monotonic();
        let homed_axes = match PrinterObject::get_status(&*th, curtime) {
            Some(PyVal::Dict(d)) => d.get("homed_axes").map(|v| v.str()).unwrap_or_default(),
            _ => String::new(),
        };
        *self.last_homed.borrow_mut() = homed_axes;

        if self.dock_requires_z {
            self.align_z_required()?;
        }

        if self.z_hop > 0.0 {
            if self.last_homed.borrow().contains('z') {
                let tpos = th.get_position();
                if tpos[2] < self.z_hop {
                    th.manual_move(&[None, None, Some(self.z_hop)], self.lift_speed)?;
                }
            } else {
                self.force_z_hop()?;
            }
        }
        Ok(())
    }

    fn align_z_required(&self) -> KResult<()> {
        if !self.last_homed.borrow().contains('z') {
            return Err(KError::command("Cannot attach/detach probe, must home Z axis first"));
        }
        self.th()?.manual_move(&[None, None, self.approach_position[2]], self.lift_speed)
    }

    // Hop z and return to un-homed state
    fn force_z_hop(&self) -> KResult<()> {
        let th = self.th()?;
        let this_z = th.get_position()[2];
        if self.last_z.get() == this_z {
            return Ok(());
        }

        let tpos = th.get_position();
        th.set_position(&[tpos[0], tpos[1], 0.0, tpos[3]], "z")?;
        th.manual_move(&[None, None, Some(self.z_hop)], self.lift_speed)?;
        th.get_kinematics().clear_homing_state("z");
        self.last_z.set(th.get_position()[2]);
        Ok(())
    }

    // Probe Wrappers

    fn run_script(&self, template: &Template, what: &str) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let start_pos = toolhead.get_position();
        template.run_gcode_from_command(None)?;
        if toolhead.get_position()[..3] != start_pos[..3] {
            return Err(KError::command(format!("Toolhead moved during probe {} script", what)));
        }
        Ok(())
    }

    fn raise_probe(&self) -> KResult<()> {
        self.run_script(&self.deactivate_gcode, "deactivate_gcode")
    }

    fn lower_probe(&self) -> KResult<()> {
        self.run_script(&self.activate_gcode, "activate_gcode")
    }

    /// move away from the bed and auto-detach (multi_probe_end and
    /// probe_finish)
    fn lift_and_detach(&self) -> KResult<()> {
        let th = self.th()?;
        let return_pos = th.get_position();
        // Move away from the bed to ensure the probe isn't triggered,
        // preventing detaching in the event there's no probe/dock sensor.
        th.manual_move(&[None, None, Some(return_pos[2] + 2.0)], self.lift_speed)?;
        self.auto_detach_probe(Some(&return_pos))
    }

    fn wait_for_trigger(&self) -> KResult<()> {
        let c = self.finish_home_complete.borrow().clone();
        match c {
            Some(c) => {
                c.wait(NEVER, None)?;
                Ok(())
            }
            None => Err(KError::Other("'NoneType' object has no attribute 'wait'".into())),
        }
    }
}

/// _arc_points: a simplified version of gcode_arcs
fn arc_points(point1: Point, point2: Point, center: Point, clockwise: bool) -> Vec<Point> {
    let (x1, y1) = point1;
    let (x2, y2) = point2;
    let (cx, cy) = center;
    // Radius vector from center to current location
    let r_p = x1 - cx;
    let r_q = y1 - cy;
    // Determine angular travel
    let rt_alpha = x2 - cx;
    let rt_beta = y2 - cy;
    let mut angular_travel = (r_p * rt_beta - r_q * rt_alpha).atan2(r_p * rt_alpha + r_q * rt_beta);

    if angular_travel < 0.0 {
        angular_travel += 2.0 * PI;
    }
    if clockwise {
        angular_travel -= 2.0 * PI;
    }
    // Determine number of segments
    let radius = py_hypot(&[r_p, r_q]);
    let flat_mm = radius * angular_travel;

    let mm_of_travel = flat_mm.abs();
    let segments = crate::pyfmt::py_max(1.0, mm_of_travel.floor());

    // Generate coordinates
    let theta_per_segment = angular_travel / segments;
    let mut coords = vec![];

    for i in 1..segments as i64 {
        let cos_ti = (i as f64 * theta_per_segment).cos();
        let sin_ti = (i as f64 * theta_per_segment).sin();
        let r_p = (x1 - cx) * cos_ti - (y1 - cy) * sin_ti;
        let r_q = (x1 - cx) * sin_ti + (y1 - cy) * cos_ti;
        coords.push((cx + r_p, cy + r_q));
    }

    coords.push(point2);
    coords
}

impl Endstop for DockableProbe {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_endstop.get_mcu()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.mcu_endstop.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        Endstop::get_steppers(&*self.mcu_endstop)
    }
    fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        let c = Endstop::home_start(&*self.mcu_endstop, print_time, sample_time, sample_count, rest_time, triggered)?;
        *self.finish_home_complete.borrow_mut() = Some(c.clone());
        let r = self.printer.get_reactor();
        let w = self.me.clone();
        let wc = r.register_callback(
            move |_| {
                if let Some(p) = w.upgrade() {
                    if let Err(e) = p.wait_for_trigger() {
                        logging::exception_err("dockable_probe wait_for_trigger", &e);
                    }
                }
            },
            NOW,
        );
        *self.wait_trigger_complete.borrow_mut() = Some(wc);
        Ok(c)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        Endstop::home_wait(&*self.mcu_endstop, home_end_time)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        Endstop::query_endstop(&*self.mcu_endstop, print_time)
    }
    fn get_position_endstop(&self) -> Option<f64> {
        Some(self.position_endstop)
    }
}

impl McuProbe for DockableProbe {
    fn multi_probe_begin(&self, always_restore_toolhead: bool) -> KResult<()> {
        self.multi.set(Multi::First);

        // Attach probe before moving to the first probe point and
        // return to current position. Move because this can be called
        // before a multi _point_ probe and a multi probe at the same
        // point but for the latter the toolhead is already in position.
        // If the toolhead is not returned to the current position it
        // will complete the probing next to the dock. This behavior
        // is driven by probe.py which defines always_restore_toolhead
        let return_pos = self.th()?.get_position();
        self.auto_attach_probe(Some(&return_pos), always_restore_toolhead)
    }

    fn multi_probe_end(&self) -> KResult<()> {
        self.multi.set(Multi::Off);
        self.lift_and_detach()
    }

    fn probing_move(&self, pos: &[f64], speed: f64, _gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        let phoming = self.printer.lookup::<PrinterHoming>("homing")?;
        let me: EndstopRef = self.me.upgrade().expect("dockable_probe");
        Ok((phoming.probing_move(me, pos, speed)?, Some(true)))
    }

    fn probe_prepare(&self, _hmove: &Rc<HomingMove>) -> KResult<()> {
        let multi = self.multi.get();
        if multi == Multi::Off || multi == Multi::First {
            let return_pos = self.th()?.get_position();
            self.auto_attach_probe(Some(&return_pos), false)?;
        }
        if multi == Multi::First {
            self.multi.set(Multi::On);
        }
        Ok(())
    }

    fn probe_finish(&self, _hmove: &Rc<HomingMove>) -> KResult<()> {
        let wc = self.wait_trigger_complete.borrow().clone();
        match wc {
            Some(c) => {
                c.wait(NEVER, None)?;
            }
            None => return Err(KError::Other("'NoneType' object has no attribute 'wait'".into())),
        }
        if self.multi.get() == Multi::Off {
            self.lift_and_detach()?;
        }
        Ok(())
    }
}

impl PrinterObject for DockableProbe {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        // Use last_'status' here to be consistent with QUERY_PROBE_'STATUS'.
        let mut d = IndexMap::new();
        d.insert("last_status".to_string(), PyVal::Int(self.last_probe_state.get()));
        d.insert("auto_attach_detach".to_string(), PyVal::Bool(self.auto_attach_detach.get()));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.finish_home_complete.borrow_mut() = None;
        *self.wait_trigger_complete.borrow_mut() = None;
        *self.toolhead.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let msp = DockableProbe::new(config)?;
    PrinterProbe::new(config, msp.clone())?;
    Ok(msp)
}
