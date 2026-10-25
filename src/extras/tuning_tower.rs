// Helper script to adjust parameters based on Z level
// (port of klippy/extras/tuning_tower.py)
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_move::{GCodeMove, MoveTransform, MoveTransformRef};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{downcast_rc, PObj, PrinterObject};
use crate::pyformat::percent_format;
use crate::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const CANCEL_Z_DELTA: f64 = 2.0;

const CMD_TUNING_TOWER_HELP: &str = "Tool to adjust a parameter at each Z height";

pub struct TuningTower {
    me: Weak<TuningTower>,
    normal_transform: RefCell<Option<MoveTransformRef>>,
    last_position: RefCell<Vec<f64>>,
    last_z: Cell<f64>,
    start: Cell<f64>,
    factor: Cell<f64>,
    band: Cell<f64>,
    step_delta: Cell<f64>,
    step_height: Cell<f64>,
    skip: Cell<f64>,
    last_command_value: Cell<Option<f64>>,
    command_fmt: RefCell<String>,
    gcode_move: Rc<GCodeMove>,
    gcode: Rc<GCodeDispatch>,
}

impl TuningTower {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode_move = printer
            .load_object(config, "gcode_move", true)?
            .and_then(downcast_rc::<GCodeMove>)
            .ok_or_else(|| KError::Other("gcode_move has an unexpected type".into()))?;
        // Register command
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let tt = Rc::new_cyclic(|me| TuningTower {
            me: me.clone(),
            normal_transform: RefCell::new(None),
            last_position: RefCell::new(vec![0.0; 4]),
            last_z: Cell::new(0.0),
            start: Cell::new(0.0),
            factor: Cell::new(0.0),
            band: Cell::new(0.0),
            step_delta: Cell::new(0.0),
            step_height: Cell::new(0.0),
            skip: Cell::new(0.0),
            last_command_value: Cell::new(None),
            command_fmt: RefCell::new(String::new()),
            gcode_move,
            gcode: gcode.clone(),
        });
        let t = tt.clone();
        gcode.register_command("TUNING_TOWER", Some(Rc::new(move |g| t.cmd_tuning_tower(g))), false, Some(CMD_TUNING_TOWER_HELP))?;
        Ok(tt)
    }

    fn cmd_tuning_tower(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if self.normal_transform.borrow().is_some() {
            self.end_test()?;
        }
        // Get parameters
        let command = gcmd.get("COMMAND")?;
        let parameter = gcmd.get("PARAMETER")?;
        self.start.set(gcmd.get_float("START", Some(0.0), Lim::new())?);
        self.factor.set(gcmd.get_float("FACTOR", Some(0.0), Lim::new())?);
        self.band.set(gcmd.get_float("BAND", Some(0.0), Lim::new().min(0.0))?);
        self.step_delta.set(gcmd.get_float("STEP_DELTA", Some(0.0), Lim::new())?);
        self.step_height.set(gcmd.get_float("STEP_HEIGHT", Some(0.0), Lim::new().min(0.0))?);
        self.skip.set(gcmd.get_float("SKIP", Some(0.0), Lim::new().min(0.0))?);
        if self.factor.get() != 0.0 && (self.step_height.get() != 0.0 || self.step_delta.get() != 0.0) {
            return Err(gcmd.error("Cannot specify both FACTOR and STEP_DELTA/STEP_HEIGHT"));
        }
        if (self.step_delta.get() != 0.0) != (self.step_height.get() != 0.0) {
            return Err(gcmd.error("Must specify both STEP_DELTA and STEP_HEIGHT"));
        }
        // Enable test mode
        *self.command_fmt.borrow_mut() = if self.gcode.is_traditional_gcode(&command) {
            format!("{} {}%.9f", command, parameter)
        } else {
            format!("{} {}=%.9f", command, parameter)
        };
        let me: MoveTransformRef = self.me.upgrade().expect("tuning_tower");
        let nt = self.gcode_move.set_move_transform(me, true)?;
        *self.normal_transform.borrow_mut() = nt;
        self.last_z.set(-99999999.9);
        self.last_command_value.set(None);
        self.get_position();
        let mut message_parts = vec![format!("start={:.6}", self.start.get())];
        if self.factor.get() != 0.0 {
            message_parts.push(format!("factor={:.6}", self.factor.get()));
            if self.band.get() != 0.0 {
                message_parts.push(format!("band={:.6}", self.band.get()));
            }
        } else {
            message_parts.push(format!("step_delta={:.6}", self.step_delta.get()));
            message_parts.push(format!("step_height={:.6}", self.step_height.get()));
        }
        if self.skip.get() != 0.0 {
            message_parts.push(format!("skip={:.6}", self.skip.get()));
        }
        gcmd.respond_info(&format!("Starting tuning test ({})", message_parts.join(" ")), true);
        Ok(())
    }

    fn calc_value(&self, mut z: f64) -> f64 {
        if self.skip.get() != 0.0 {
            let v = z - self.skip.get();
            z = if v > 0.0 { v } else { 0.0 };
        }
        if self.step_height.get() != 0.0 {
            return self.start.get() + self.step_delta.get() * (z / self.step_height.get()).floor();
        }
        if self.band.get() != 0.0 {
            z = ((z / self.band.get()).floor() + 0.5) * self.band.get();
        }
        self.start.get() + z * self.factor.get()
    }

    fn end_test(&self) -> KResult<()> {
        self.gcode.respond_info("Ending tuning test mode", true);
        let nt = self.normal_transform.borrow_mut().take();
        if let Some(nt) = nt {
            self.gcode_move.set_move_transform(nt, true)?;
        }
        Ok(())
    }

    pub fn is_active(&self) -> bool {
        self.normal_transform.borrow().is_some()
    }

    fn gcode_position_z(&self) -> f64 {
        match self.gcode_move.get_status(0.) {
            Some(PyVal::Dict(d)) => d.get("gcode_position").and_then(|c| c.items().map(|v| v[2].as_f64().unwrap_or(0.))).unwrap_or(0.),
            _ => 0.,
        }
    }
}

impl MoveTransform for TuningTower {
    fn get_position(&self) -> Vec<f64> {
        let pos = match self.normal_transform.borrow().as_ref() {
            Some(nt) => nt.get_position(),
            None => vec![],
        };
        *self.last_position.borrow_mut() = pos.clone();
        pos
    }

    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let normal_transform = self.normal_transform.borrow().clone();
        let last_position = self.last_position.borrow().clone();
        if newpos[3] > last_position[3] && newpos[2] != self.last_z.get() && newpos[..3] != last_position[..3] {
            // Extrusion move at new z height
            let z = newpos[2];
            if z < self.last_z.get() - CANCEL_Z_DELTA {
                // Extrude at a lower z height - probably start of new print
                self.end_test()?;
            } else {
                // Process update
                let gcode_z = self.gcode_position_z();
                let newval = self.calc_value(gcode_z);
                self.last_z.set(z);
                if Some(newval) != self.last_command_value.get() {
                    self.last_command_value.set(Some(newval));
                    let fmt = self.command_fmt.borrow().clone();
                    let script = percent_format(&fmt, &PyVal::Tuple(vec![PyVal::Float(newval)]))
                        .map_err(|(_, msg)| KError::Other(msg))?;
                    self.gcode.run_script_from_command(&script)?;
                }
            }
        }
        // Forward move to actual handler
        *self.last_position.borrow_mut() = newpos.to_vec();
        match normal_transform {
            Some(nt) => nt.move_to(newpos, speed),
            None => Err(KError::Other("'NoneType' object has no attribute 'move'".into())),
        }
    }
}

impl PrinterObject for TuningTower {
    fn teardown(&self) {
        *self.normal_transform.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(TuningTower::new(config)?)
}
