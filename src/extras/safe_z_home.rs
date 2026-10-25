// Perform Z Homing at specific XY coordinates (port of klippy/extras/safe_z_home.py)
//
// Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::gcode::{GCodeCommand, GCodeDispatch, Handler};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

pub struct SafeZHoming {
    printer: Rc<Printer>,
    pub home_x_pos: f64,
    pub home_y_pos: f64,
    z_hop: f64,
    z_hop_speed: f64,
    speed: f64,
    move_to_previous: bool,
    home_y_before_x: bool,
    gcode: Rc<GCodeDispatch>,
    prev_g28: RefCell<Option<Handler>>,
}

fn homed_axes(toolhead: &ToolHead, curtime: f64) -> String {
    toolhead.get_kinematics().get_status(curtime).get("homed_axes").map(|v| v.str()).unwrap_or_default()
}

impl SafeZHoming {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let xy = config.getfloatlist("home_xy_position", Dflt::Required, ",", Some(2))?.unwrap_or_default();
        let z_hop = config.getfloat("z_hop", 0.0, Lim::new())?;
        let z_hop_speed = config.getfloat("z_hop_speed", 15.0, Lim::new().above(0.0))?;
        let zconfig = config.getsection("stepper_z");
        let _max_z = zconfig.getfloat("position_max", Dflt::Required, Lim::new().no_note())?;
        let speed = config.getfloat("speed", 50.0, Lim::new().above(0.0))?;
        let move_to_previous = config.getboolean("move_to_previous", false)?;
        let home_y_before_x = config.getboolean("home_y_before_x", false)?;
        printer.load_object(config, "homing", true)?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let prev_g28 = gcode.register_command("G28", None, false, None)?;
        let s = Rc::new(SafeZHoming {
            printer: printer.clone(),
            home_x_pos: xy[0],
            home_y_pos: xy[1],
            z_hop,
            z_hop_speed,
            speed,
            move_to_previous,
            home_y_before_x,
            gcode: gcode.clone(),
            prev_g28: RefCell::new(prev_g28),
        });
        let ss = s.clone();
        gcode.register_command("G28", Some(Rc::new(move |g| ss.cmd_g28(g))), false, None)?;
        if config.has_section("homing_override") {
            return Err(KError::config("homing_override and safe_z_homing cannot be used simultaneously"));
        }
        Ok(s)
    }

    fn prev_g28(&self, axis: &str) -> KResult<()> {
        let mut params = IndexMap::default();
        params.insert(axis.to_string(), "0".to_string());
        let g28_gcmd = self.gcode.create_gcode_command("G28", "G28", params);
        let prev = self.prev_g28.borrow().clone();
        match prev {
            Some(f) => f(&g28_gcmd),
            None => Err(KError::Other("'NoneType' object is not callable".into())),
        }
    }

    fn cmd_g28(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;

        // Perform Z Hop if necessary
        if self.z_hop != 0.0 {
            // Check if Z axis is homed and its last known position
            let curtime = self.printer.get_reactor().monotonic();
            let homed = homed_axes(&toolhead, curtime);
            let mut pos = toolhead.get_position();

            if !homed.contains('z') {
                // Always perform the z_hop if the Z axis is not homed
                pos[2] = 0.0;
                toolhead.set_position(&pos, "z")?;
                toolhead.manual_move(&[None, None, Some(self.z_hop)], self.z_hop_speed)?;
                toolhead.get_kinematics().clear_homing_state("z");
            } else if pos[2] < self.z_hop {
                // If the Z axis is homed, and below z_hop, lift it to z_hop
                toolhead.manual_move(&[None, None, Some(self.z_hop)], self.z_hop_speed)?;
            }
        }

        // Determine which axes we need to home
        let mut need_x = gcmd.get_opt("X").is_some();
        let mut need_y = gcmd.get_opt("Y").is_some();
        let mut need_z = gcmd.get_opt("Z").is_some();
        if !need_x && !need_y && !need_z {
            need_x = true;
            need_y = true;
            need_z = true;
        }

        if need_x || need_y {
            let axis_order = if self.home_y_before_x { ["y", "x"] } else { ["x", "y"] };
            for axis in axis_order {
                if axis == "x" && need_x {
                    self.prev_g28("X")?;
                } else if axis == "y" && need_y {
                    self.prev_g28("Y")?;
                }
            }
        }

        // Home Z axis if necessary
        if need_z {
            // Throw an error if X or Y are not homed
            let curtime = self.printer.get_reactor().monotonic();
            let homed = homed_axes(&toolhead, curtime);
            if !homed.contains('x') || !homed.contains('y') {
                return Err(gcmd.error("Must home X and Y axes first"));
            }

            // Do we need to detach the probe?
            let dockable = self.printer.lookup_object_opt("dockable_probe").and_then(crate::printer::downcast_rc::<crate::extras::dockable_probe::DockableProbe>);
            if let Some(dockable) = dockable.filter(|d| d.detach_dockable_before_z_home) {
                dockable.detach_probe(None)?;
            }

            // Move to safe XY homing position
            let prevpos = toolhead.get_position();
            toolhead.manual_move(&[Some(self.home_x_pos), Some(self.home_y_pos)], self.speed)?;

            // Home Z
            self.prev_g28("Z")?;

            // Perform Z Hop again for pressure-based probes
            if self.z_hop != 0.0 {
                let pos = toolhead.get_position();
                if pos[2] < self.z_hop {
                    toolhead.manual_move(&[None, None, Some(self.z_hop)], self.z_hop_speed)?;
                }
            }

            // Move XY back to previous positions
            if self.move_to_previous {
                toolhead.manual_move(&[Some(prevpos[0]), Some(prevpos[1])], self.speed)?;
            }
        }
        Ok(())
    }
}

impl PrinterObject for SafeZHoming {
    fn teardown(&self) {
        *self.prev_g28.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(SafeZHoming::new(config)?)
}
