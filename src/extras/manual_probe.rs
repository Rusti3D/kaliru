// Helper script for manual z height probing
// (port of klippy/extras/manual_probe.py)
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_move::GCodeMove;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{downcast_rc, ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyval::{PyDict, PyVal};
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

/// the finalize callback: cb(kin_pos or None)
pub type FinalizeCallback = Rc<dyn Fn(Option<Vec<f64>>) -> KResult<()>>;

const CMD_MANUAL_PROBE_HELP: &str = "Start manual probe helper script";
const CMD_Z_ENDSTOP_CALIBRATE_HELP: &str = "Calibrate a Z endstop";
const CMD_Z_OFFSET_APPLY_ENDSTOP_HELP: &str = "Adjust the z endstop_position";

pub struct ManualProbe {
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    gcode_move: Rc<GCodeMove>,
    z_position_endstop: Option<f64>,
    a_position_endstop: Option<f64>,
    b_position_endstop: Option<f64>,
    c_position_endstop: Option<f64>,
    pub status: RefCell<PyDict>,
}

fn opt_f(v: Option<f64>) -> PyVal {
    v.map(PyVal::Float).unwrap_or(PyVal::None)
}

impl ManualProbe {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Register commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let gcode_move = printer
            .load_object(config, "gcode_move", true)?
            .and_then(downcast_rc::<GCodeMove>)
            .ok_or_else(|| KError::Other("gcode_move has an unexpected type".into()))?;
        let endstop = |section: &str| config.getsection(section).getfloat_opt("position_endstop", Lim::new().no_note());
        // Endstop value for cartesian printers with separate Z axis
        let z_position_endstop = endstop("stepper_z")?;
        // Endstop values for linear delta printers with vertical A,B,C towers
        let a_position_endstop = endstop("stepper_a")?;
        let b_position_endstop = endstop("stepper_b")?;
        let c_position_endstop = endstop("stepper_c")?;
        let mp = Rc::new(ManualProbe {
            printer: printer.clone(),
            gcode: gcode.clone(),
            gcode_move,
            z_position_endstop,
            a_position_endstop,
            b_position_endstop,
            c_position_endstop,
            status: RefCell::new(IndexMap::default()),
        });
        let m = mp.clone();
        gcode.register_command("MANUAL_PROBE", Some(Rc::new(move |g| m.cmd_manual_probe(g))), false, Some(CMD_MANUAL_PROBE_HELP))?;
        // Conditionally register appropriate commands depending on printer
        // Cartestian printers with separate Z Axis
        if mp.z_position_endstop.is_some() {
            let m = mp.clone();
            gcode.register_command(
                "Z_ENDSTOP_CALIBRATE",
                Some(Rc::new(move |g| m.cmd_z_endstop_calibrate(g))),
                false,
                Some(CMD_Z_ENDSTOP_CALIBRATE_HELP),
            )?;
            let m = mp.clone();
            gcode.register_command(
                "Z_OFFSET_APPLY_ENDSTOP",
                Some(Rc::new(move |g| m.cmd_z_offset_apply_endstop(g))),
                false,
                Some(CMD_Z_OFFSET_APPLY_ENDSTOP_HELP),
            )?;
        }
        // Linear delta printers with A,B,C towers
        if config.getsection("printer").get("kinematics", Dflt::Required)? == "delta" {
            let m = mp.clone();
            gcode.register_command(
                "Z_OFFSET_APPLY_ENDSTOP",
                Some(Rc::new(move |g| m.cmd_z_offset_apply_delta_endstops(g))),
                false,
                Some(CMD_Z_OFFSET_APPLY_ENDSTOP_HELP),
            )?;
        }
        mp.reset_status();
        Ok(mp)
    }

    fn manual_probe_finalize(&self, kin_pos: Option<Vec<f64>>) -> KResult<()> {
        if let Some(kin_pos) = kin_pos {
            self.gcode.respond_info(&format!("Z position is {:.3}", kin_pos[2]), true);
        }
        Ok(())
    }

    pub fn reset_status(&self) {
        let s = crate::pydict! {
            "is_active" => PyVal::Bool(false),
            "z_position" => PyVal::None,
            "z_position_lower" => PyVal::None,
            "z_position_upper" => PyVal::None,
        };
        *self.status.borrow_mut() = s;
    }

    fn me(&self) -> KResult<Rc<ManualProbe>> {
        self.printer.lookup::<ManualProbe>("manual_probe")
    }

    fn cmd_manual_probe(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let me = self.me()?;
        ManualProbeHelper::start(&self.printer, gcmd, Rc::new(move |kin_pos| me.manual_probe_finalize(kin_pos)))
    }

    fn z_endstop_finalize(&self, kin_pos: Option<Vec<f64>>) -> KResult<()> {
        let Some(kin_pos) = kin_pos else { return Ok(()) };
        let z_pos = self.z_position_endstop.unwrap_or(0.) - kin_pos[2];
        self.gcode.respond_info(
            &format!(
                "stepper_z: position_endstop: {:.3}\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
                z_pos
            ),
            true,
        );
        self.configfile_set("stepper_z", z_pos)
    }

    fn configfile_set(&self, section: &str, value: f64) -> KResult<()> {
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        configfile.pconfig.borrow_mut().set(section, "position_endstop", &format!("{:.3}", value))?;
        Ok(())
    }

    fn cmd_z_endstop_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let me = self.me()?;
        ManualProbeHelper::start(&self.printer, gcmd, Rc::new(move |kin_pos| me.z_endstop_finalize(kin_pos)))
    }

    fn homing_origin_z(&self) -> f64 {
        match self.gcode_move.get_status(0.) {
            Some(PyVal::Dict(d)) => d.get("homing_origin").and_then(|c| c.items().map(|v| v[2].as_f64().unwrap_or(0.))).unwrap_or(0.),
            _ => 0.,
        }
    }

    fn cmd_z_offset_apply_endstop(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let offset = self.homing_origin_z();
        if offset == 0.0 {
            self.gcode.respond_info("Nothing to do: Z Offset is 0", true);
        } else {
            let new_calibrate = self.z_position_endstop.unwrap_or(0.) - offset;
            self.gcode.respond_info(
                &format!(
                    "stepper_z: position_endstop: {:.3}\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
                    new_calibrate
                ),
                true,
            );
            self.configfile_set("stepper_z", new_calibrate)?;
        }
        Ok(())
    }

    fn cmd_z_offset_apply_delta_endstops(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let offset = self.homing_origin_z();
        if offset == 0.0 {
            self.gcode.respond_info("Nothing to do: Z Offset is 0", true);
        } else {
            // python: None - offset raises TypeError
            let sub = |v: Option<f64>| {
                v.map(|v| v - offset)
                    .ok_or_else(|| KError::Other("unsupported operand type(s) for -: 'NoneType' and 'float'".into()))
            };
            let new_a_calibrate = sub(self.a_position_endstop)?;
            let new_b_calibrate = sub(self.b_position_endstop)?;
            let new_c_calibrate = sub(self.c_position_endstop)?;
            self.gcode.respond_info(
                &format!(
                    "stepper_a: position_endstop: {:.3}\nstepper_b: position_endstop: {:.3}\nstepper_c: position_endstop: {:.3}\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
                    new_a_calibrate, new_b_calibrate, new_c_calibrate
                ),
                true,
            );
            self.configfile_set("stepper_a", new_a_calibrate)?;
            self.configfile_set("stepper_b", new_b_calibrate)?;
            self.configfile_set("stepper_c", new_c_calibrate)?;
        }
        Ok(())
    }
}

impl PrinterObject for ManualProbe {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(PyVal::Dict(self.status.borrow().clone()))
    }
}

// Verify that a manual probe isn't already in progress
pub fn verify_no_manual_probe(printer: &Printer) -> KResult<()> {
    let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
    match gcode.register_command("ACCEPT", Some(Rc::new(|_| Ok(()))), false, None) {
        Err(KError::Config(_)) => return Err(KError::command("Already in a manual Z probe. Use ABORT to abort it.")),
        r => r?,
    };
    gcode.register_command("ACCEPT", None, false, None)?;
    Ok(())
}

pub const Z_BOB_MINIMUM: f64 = 0.500;
pub const BISECT_MAX: f64 = 0.200;

const CMD_ACCEPT_HELP: &str = "Accept the current Z position";
const CMD_ABORT_HELP: &str = "Abort manual Z probing tool";
const CMD_TESTZ_HELP: &str = "Move to new Z height";

// Helper script to determine a Z height
pub struct ManualProbeHelper {
    printer: Rc<Printer>,
    finalize_callback: FinalizeCallback,
    gcode: Rc<GCodeDispatch>,
    toolhead: Rc<ToolHead>,
    manual_probe: Rc<ManualProbe>,
    speed: f64,
    past_positions: RefCell<Vec<f64>>,
    last_toolhead_pos: RefCell<Option<Vec<f64>>>,
    last_kinematics_pos: RefCell<Option<Vec<f64>>>,
    start_position: RefCell<Vec<f64>>,
}

/// bisect.bisect_left
fn bisect_left(a: &[f64], x: f64) -> usize {
    let (mut lo, mut hi) = (0, a.len());
    while lo < hi {
        let mid = (lo + hi) / 2;
        if a[mid] < x {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    lo
}

impl ManualProbeHelper {
    pub fn start(printer: &Rc<Printer>, gcmd: &GCodeCommand, finalize_callback: FinalizeCallback) -> KResult<()> {
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let toolhead = printer.lookup::<ToolHead>("toolhead")?;
        let manual_probe = printer.lookup::<ManualProbe>("manual_probe")?;
        let speed = gcmd.get_float("SPEED", Some(5.0), Lim::new())?;
        let h = Rc::new(ManualProbeHelper {
            printer: printer.clone(),
            finalize_callback,
            gcode: gcode.clone(),
            toolhead,
            manual_probe,
            speed,
            past_positions: RefCell::new(vec![]),
            last_toolhead_pos: RefCell::new(None),
            last_kinematics_pos: RefCell::new(None),
            start_position: RefCell::new(vec![]),
        });
        // Register commands
        verify_no_manual_probe(printer)?;
        let c = h.clone();
        gcode.register_command("ACCEPT", Some(Rc::new(move |g| c.cmd_accept(g))), false, Some(CMD_ACCEPT_HELP))?;
        let c = h.clone();
        gcode.register_command("NEXT", Some(Rc::new(move |g| c.cmd_accept(g))), false, None)?;
        let c = h.clone();
        gcode.register_command("ABORT", Some(Rc::new(move |_| c.finalize(false))), false, Some(CMD_ABORT_HELP))?;
        let c = h.clone();
        gcode.register_command("TESTZ", Some(Rc::new(move |g| c.cmd_testz(g))), false, Some(CMD_TESTZ_HELP))?;
        gcode.respond_info("Starting manual Z probe. Use TESTZ to adjust position.\nFinish with ACCEPT or ABORT command.", true);
        *h.start_position.borrow_mut() = h.toolhead.get_position();
        h.report_z_status(false, None)
    }

    fn get_kinematics_pos(&self) -> KResult<Vec<f64>> {
        let toolhead_pos = self.toolhead.get_position();
        if self.last_toolhead_pos.borrow().as_ref() == Some(&toolhead_pos) {
            return Ok(self.last_kinematics_pos.borrow().clone().unwrap_or_default());
        }
        self.toolhead.flush_step_generation()?;
        let kin = self.toolhead.get_kinematics();
        let kin_spos: IndexMap<String, f64> =
            kin.get_steppers().iter().map(|s| (s.get_name(false), s.get_commanded_position())).collect();
        let kin_pos = kin.calc_position(&kin_spos);
        *self.last_toolhead_pos.borrow_mut() = Some(toolhead_pos);
        *self.last_kinematics_pos.borrow_mut() = Some(kin_pos.clone());
        Ok(kin_pos)
    }

    fn move_z(&self, z_pos: f64) -> KResult<()> {
        let curpos = self.toolhead.get_position();
        let res = (|| {
            let z_bob_pos = z_pos + Z_BOB_MINIMUM;
            if curpos[2] < z_bob_pos {
                self.toolhead.manual_move(&[None, None, Some(z_bob_pos)], self.speed)?;
            }
            self.toolhead.manual_move(&[None, None, Some(z_pos)], self.speed)
        })();
        if let Err(e) = res {
            if e.is_command_error() {
                self.finalize(false)?;
            }
            return Err(e);
        }
        Ok(())
    }

    fn report_z_status(&self, warn_no_change: bool, prev_pos: Option<f64>) -> KResult<()> {
        // Get position
        let kin_pos = self.get_kinematics_pos()?;
        let z_pos = kin_pos[2];
        if warn_no_change && Some(z_pos) == prev_pos {
            self.gcode.respond_info("WARNING: No change in position (reached stepper resolution)", true);
        }
        // Find recent positions that were tested
        let pp = self.past_positions.borrow();
        let mut next_pos = bisect_left(&pp, z_pos);
        let prev_pos = next_pos as i64 - 1;
        if next_pos < pp.len() && pp[next_pos] == z_pos {
            next_pos += 1;
        }
        let (mut prev_pos_val, mut next_pos_val) = (None, None);
        let (mut prev_str, mut next_str) = ("??????".to_string(), "??????".to_string());
        if prev_pos >= 0 {
            let v = pp[prev_pos as usize];
            prev_pos_val = Some(v);
            prev_str = format!("{:.3}", v);
        }
        if next_pos < pp.len() {
            let v = pp[next_pos];
            next_pos_val = Some(v);
            next_str = format!("{:.3}", v);
        }
        let s = crate::pydict! {
            "is_active" => PyVal::Bool(true),
            "z_position" => PyVal::Float(z_pos),
            "z_position_lower" => opt_f(prev_pos_val),
            "z_position_upper" => opt_f(next_pos_val),
        };
        *self.manual_probe.status.borrow_mut() = s;
        // Find recent positions
        self.gcode.respond_info(&format!("Z position: {} --> {:.3} <-- {}", prev_str, z_pos, next_str), true);
        Ok(())
    }

    fn cmd_accept(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let pos = self.toolhead.get_position();
        let start_pos = self.start_position.borrow().clone();
        if pos[..2] != start_pos[..2] || pos[2] >= start_pos[2] {
            gcmd.respond_info("Manual probe failed! Use TESTZ commands to position the\nnozzle prior to running ACCEPT.", true);
            return self.finalize(false);
        }
        self.finalize(true)
    }

    fn cmd_testz(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Store current position for later reference
        let kin_pos = self.get_kinematics_pos()?;
        let z_pos = kin_pos[2];
        let insert_pos = {
            let mut pp = self.past_positions.borrow_mut();
            let insert_pos = bisect_left(&pp, z_pos);
            if insert_pos >= pp.len() || pp[insert_pos] != z_pos {
                pp.insert(insert_pos, z_pos);
            }
            insert_pos
        };
        // Determine next position to move to
        let req = gcmd.get("Z")?;
        let pp = self.past_positions.borrow().clone();
        let next_z_pos = if req == "+" || req == "++" {
            let mut check_z = 9999999999999.9;
            if insert_pos + 1 < pp.len() {
                check_z = pp[insert_pos + 1];
            }
            if req == "+" {
                check_z = (check_z + z_pos) / 2.0;
            }
            let b = z_pos + BISECT_MAX;
            if b < check_z {
                b
            } else {
                check_z
            }
        } else if req == "-" || req == "--" {
            let mut check_z = -9999999999999.9;
            if insert_pos > 0 {
                check_z = pp[insert_pos - 1];
            }
            if req == "-" {
                check_z = (check_z + z_pos) / 2.0;
            }
            let b = z_pos - BISECT_MAX;
            if b > check_z {
                b
            } else {
                check_z
            }
        } else {
            z_pos + gcmd.get_float("Z", None, Lim::new())?
        };
        // Move to given position and report it
        self.move_z(next_z_pos)?;
        self.report_z_status(next_z_pos != z_pos, Some(z_pos))
    }

    fn finalize(&self, success: bool) -> KResult<()> {
        self.manual_probe.reset_status();
        for cmd in ["ACCEPT", "NEXT", "ABORT", "TESTZ"] {
            self.gcode.register_command(cmd, None, false, None)?;
        }
        let mut kin_pos = None;
        if success {
            kin_pos = Some(self.get_kinematics_pos()?);
        }
        let _ = &self.printer;
        (self.finalize_callback)(kin_pos)
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ManualProbe::new(config)?)
}
