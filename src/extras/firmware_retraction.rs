// Support for Marlin/Smoothie/Reprap style firmware retraction via G10/G11
// (port of klippy/extras/firmware_retraction.py)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::exclude_object::ExcludeObject;
use crate::extras::gcode_move::{GCodeMove, MoveTransform, MoveTransformRef};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_bool, py_max};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const ZHOP_MOVE_SPEED_FRACTION: f64 = 0.8;
const MAX_Z_MARGIN: f64 = 0.1;

#[derive(Clone, Copy, Default)]
struct Params {
    retract_length: f64,
    retract_speed: f64,
    unretract_extra_length: f64,
    unretract_speed: f64,
    z_hop_height: f64,
    clear_zhop: bool,
    unretract_length: f64,
}

pub struct FirmwareRetraction {
    me: Weak<FirmwareRetraction>,
    config_ref: ConfigWrapper,
    printer: Rc<Printer>,
    p: Cell<Params>,
    is_retracted: Cell<bool>,
    do_zhop: Cell<bool>,
    current_z_hop_height: Cell<f64>,
    current_unretract_length: Cell<f64>,
    current_unretract_speed: Cell<f64>,
    last_position: RefCell<Vec<f64>>,
    next_transform: RefCell<Option<MoveTransformRef>>,
    max_z_vel: f64,
    gcode_move: RefCell<Option<Rc<GCodeMove>>>,
    exclude_objects: RefCell<Option<Rc<ExcludeObject>>>,
    maximum_z: Cell<f64>,
}

const CMD_SET_RETRACTION_HELP: &str = "Set firmware retraction parameters";
const CMD_GET_RETRACTION_HELP: &str = "Report firmware retraction parameters and states";
const CMD_CLEAR_RETRACTION_HELP: &str = "Clear retraction state without retract move if enabled";
const CMD_RESET_RETRACTION_HELP: &str = "Reset retraction parameters to default values";

impl FirmwareRetraction {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Get retraction params from config, used also in RESET_RETRACTION
        // and initiate params
        let p = Self::get_config_params(config)?;
        // Get maximum printer z move velocity for zhop moves
        let printer_config = config.getsection("printer");
        let max_velocity = printer_config.getfloat("max_velocity", Dflt::Required, Lim::new())?;
        let max_z_vel = printer_config.getfloat("max_z_velocity", max_velocity, Lim::new())?;
        let fr = Rc::new_cyclic(|me| FirmwareRetraction {
            me: me.clone(),
            config_ref: config.clone(),
            printer: printer.clone(),
            p: Cell::new(p),
            // Initialize variables
            is_retracted: Cell::new(false),
            do_zhop: Cell::new(false),
            current_z_hop_height: Cell::new(0.0),
            current_unretract_length: Cell::new(0.0),
            current_unretract_speed: Cell::new(0.0),
            last_position: RefCell::new(vec![0.0; 4]),
            next_transform: RefCell::new(None),
            max_z_vel,
            gcode_move: RefCell::new(None),
            exclude_objects: RefCell::new(None),
            maximum_z: Cell::new(0.0),
        });
        let w = fr.me.clone();
        crate::events::KlippyReady::on(&printer, move || match w.upgrade() {
            Some(f) => f.handle_ready(),
            None => Ok(()),
        });
        // Register new G-code commands for setting/retrieving retraction
        // parameters as well as clearing retraction state
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        type FCmd = fn(&FirmwareRetraction, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, FCmd, Option<&str>); 6] = [
            ("SET_RETRACTION", FirmwareRetraction::cmd_set_retraction, Some(CMD_SET_RETRACTION_HELP)),
            ("GET_RETRACTION", FirmwareRetraction::cmd_get_retraction, Some(CMD_GET_RETRACTION_HELP)),
            ("CLEAR_RETRACTION", FirmwareRetraction::cmd_clear_retraction, Some(CMD_CLEAR_RETRACTION_HELP)),
            ("RESET_RETRACTION", FirmwareRetraction::cmd_reset_retraction, Some(CMD_RESET_RETRACTION_HELP)),
            // Register new G-code commands for firmware retraction/unretraction
            ("G10", FirmwareRetraction::cmd_g10, None),
            ("G11", FirmwareRetraction::cmd_g11, None),
        ];
        for (name, f, desc) in cmds {
            let w = fr.me.clone();
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(fr) => f(&fr, g),
                    None => Ok(()),
                })),
                false,
                desc,
            )?;
        }
        Ok(fr)
    }

    // Helper method to get retraction parameters from config
    fn get_config_params(config: &ConfigWrapper) -> KResult<Params> {
        let retract_length = config.getfloat("retract_length", 0.0, Lim::new().min(0.0))?;
        let retract_speed = config.getfloat("retract_speed", 20.0, Lim::new().min(1))?;
        let unretract_extra_length = config.getfloat("unretract_extra_length", 0.0, Lim::new().min(0.0))?;
        let unretract_speed = config.getfloat("unretract_speed", 10.0, Lim::new().min(1))?;
        let z_hop_height = config.getfloat("z_hop_height", 0.0, Lim::new().min(0.0))?;
        let clear_zhop = config.getboolean("clear_zhop_on_z_moves", false)?;
        Ok(Params {
            retract_length,
            retract_speed,
            unretract_extra_length,
            unretract_speed,
            z_hop_height,
            clear_zhop,
            unretract_length: retract_length + unretract_extra_length,
        })
    }

    // Helper method to register commands and instantiate required objects
    fn handle_ready(&self) -> KResult<()> {
        let gcode_move = self.printer.lookup::<GCodeMove>("gcode_move")?;
        *self.exclude_objects.borrow_mut() = self.printer.lookup_object_opt("exclude_object").and_then(downcast_rc::<ExcludeObject>);
        // Register move transformation while printer ready
        // important to track Z change request
        let me: MoveTransformRef = self.me.upgrade().expect("firmware_retraction");
        *self.next_transform.borrow_mut() = gcode_move.set_move_transform(me, true)?;
        *self.gcode_move.borrow_mut() = Some(gcode_move);
        // Get maximum_z
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let kin = toolhead.get_kinematics();
        let axis_maximum = kin.get_status(0.0).get("axis_maximum").cloned();
        let max_z = match axis_maximum {
            Some(PyVal::NamedTuple(_, _, v)) | Some(PyVal::List(v)) | Some(PyVal::Tuple(v)) => v.get(2).and_then(|x| x.as_f64()),
            _ => None,
        };
        self.maximum_z.set(max_z.ok_or_else(|| KError::Other("'axis_maximum'".into()))?);
        // Register Events to clear retraction when a new print is started, an
        // ongoing print is canceled or a print is finished
        // GCode streaming mode (most commonly done via OctoPrint)
        for ev in [crate::events::HomingMoveBegin::NAME, crate::events::StepperEnableMotorOff::NAME] {
            let w = self.me.clone();
            self.printer.register_event_handler(
                ev,
                Rc::new(move |_| {
                    if let Some(f) = w.upgrade() {
                        f.execute_clear_z_hop();
                    }
                    Ok(())
                }),
            );
        }
        Ok(())
    }

    fn cmd_set_retraction(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut p = self.p.get();
        p.retract_length = gcmd.get_float("RETRACT_LENGTH", Some(p.retract_length), Lim::new().min(0.0))?;
        p.retract_speed = gcmd.get_float("RETRACT_SPEED", Some(p.retract_speed), Lim::new().min(1.0))?;
        p.unretract_extra_length = gcmd.get_float("UNRETRACT_EXTRA_LENGTH", Some(p.unretract_extra_length), Lim::new().min(-1.0))?;
        p.unretract_speed = gcmd.get_float("UNRETRACT_SPEED", Some(p.unretract_speed), Lim::new().min(1.0))?;
        // z_hop_height with 0mm min. to prevent nozzle crash
        p.z_hop_height = gcmd.get_float("Z_HOP_HEIGHT", Some(p.z_hop_height), Lim::new().min(0.0))?;
        p.unretract_length = p.retract_length + p.unretract_extra_length;
        self.p.set(p);
        Ok(())
    }

    fn cmd_get_retraction(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let p = self.p.get();
        gcmd.respond_info(
            &format!(
                "RETRACT_LENGTH={:.5} RETRACT_SPEED={:.5} UNRETRACT_EXTRA_LENGTH={:.5} UNRETRACT_SPEED={:.5}  Z_HOP_HEIGHT={:.5}  RETRACT_STATE={}  ZHOP_STATE={}",
                p.retract_length,
                p.retract_speed,
                p.unretract_extra_length,
                p.unretract_speed,
                p.z_hop_height,
                py_bool(self.is_retracted.get()),
                py_bool(self.do_zhop.get())
            ),
            true,
        );
        Ok(())
    }

    fn cmd_clear_retraction(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.execute_clear_z_hop();
        self.is_retracted.set(false); // Reset retract flag to enable G10 command
        gcmd.respond_info("Retraction was cleared. zhop is undone on next move.", true);
        Ok(())
    }

    fn cmd_reset_retraction(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        // Reset retraction parameters to config values
        self.p.set(Self::get_config_params(&self.config_ref)?);
        Ok(())
    }

    // Helper to clear z_hop
    fn execute_clear_z_hop(&self) {
        self.do_zhop.set(false);
        self.current_z_hop_height.set(0.0); // prevent nozzle crash if G11 occurs
    }

    // Helper to skip command while in exclude object
    fn test_in_excluded_region(&self) -> bool {
        match self.exclude_objects.borrow().as_ref() {
            Some(e) => e.test_in_excluded_region(),
            None => false,
        }
    }

    fn gcode_move(&self) -> KResult<Rc<GCodeMove>> {
        self.gcode_move.borrow().clone().ok_or_else(|| KError::Other("'FirmwareRetraction' object has no attribute 'gcode_move'".into()))
    }

    // Gcode Command G10 to perform firmware retraction
    fn cmd_g10(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let (extrude_factor, speed_factor) = self.get_factors()?;
        let p = self.p.get();
        // If filament isn't retracted
        if !self.is_retracted.get() && !self.test_in_excluded_region() {
            // Check if FW retraction enabled
            if p.retract_length == 0.0 && p.z_hop_height == 0.0 {
                gcmd.respond_info("Retraction length and z_hop zero. Firmware retraction disabled. G10 Command ignored!", true);
            } else {
                // Store retract parameters for moves when retracted until next G11
                self.current_unretract_length.set(p.unretract_length);
                self.current_unretract_speed.set(p.unretract_speed);
                self.execute_clear_z_hop();
                // get position (track of bed_mesh, z_thermal_adjust changes)
                let mut position = self.position()?;
                // do retraction
                if p.retract_length > 0.0 {
                    position[3] -= p.retract_length * extrude_factor;
                    self.do_move(&position, p.retract_speed * speed_factor)?;
                }
                // do zhop
                if self.limit_zhop(gcmd, &position) {
                    self.do_zhop.set(true);
                    self.do_move(&position, ZHOP_MOVE_SPEED_FRACTION * self.max_z_vel * speed_factor)?;
                }
                // update toolhead position
                self.gcode_move()?.reset_last_position()?;
                self.is_retracted.set(true);
            }
        }
        Ok(())
    }

    // GCode Command G11 to perform filament unretraction
    fn cmd_g11(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let (extrude_factor, speed_factor) = self.get_factors()?;
        // Check if the filament is retracted
        if self.is_retracted.get() && !self.test_in_excluded_region() {
            // Check if FW retraction enabled
            if self.current_unretract_length.get() == 0.0 && self.current_z_hop_height.get() == 0.0 {
                gcmd.respond_info("Retraction length and z_hop zero. Firmware retraction disabled. G11 Command ignored!", true);
            } else {
                // get position (track of bed_mesh, z_thermal_adjust changes)
                let mut position = self.position()?;
                // undo zhop
                self.execute_clear_z_hop();
                self.do_move(&position, ZHOP_MOVE_SPEED_FRACTION * self.max_z_vel * speed_factor)?;
                // un-retract
                if self.current_unretract_length.get() > 0.0 {
                    position[3] += self.current_unretract_length.get() * extrude_factor;
                    self.do_move(&position, self.current_unretract_speed.get() * speed_factor)?;
                }
                // update toolhead position
                self.gcode_move()?.reset_last_position()?;
            }
            self.is_retracted.set(false);
        }
        Ok(())
    }

    fn next(&self) -> KResult<MoveTransformRef> {
        self.next_transform.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_position'".into()))
    }

    // gcode_move transform position helper
    fn position(&self) -> KResult<Vec<f64>> {
        let mut position = self.next()?.get_position();
        position[2] -= self.current_z_hop_height.get();
        *self.last_position.borrow_mut() = position.clone();
        Ok(position)
    }

    // gcode_move transform move helper
    fn do_move(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        // cancel zhop on z moves
        if self.do_zhop.get() && self.p.get().clear_zhop && newpos[2] != self.last_position.borrow()[2] {
            self.execute_clear_z_hop();
        }
        let adjusted_pos = [newpos[0], newpos[1], newpos[2] + self.current_z_hop_height.get(), newpos[3]];
        self.next()?.move_to(&adjusted_pos, speed)?;
        *self.last_position.borrow_mut() = newpos.to_vec();
        Ok(())
    }

    // Helper to limit z_hop while z_max is reached
    fn limit_zhop(&self, gcmd: &GCodeCommand, pos: &[f64]) -> bool {
        let z_margin = self.maximum_z.get() - pos[2] - MAX_Z_MARGIN;
        let z_hop_height = self.p.get().z_hop_height;
        if z_hop_height > z_margin {
            self.current_z_hop_height.set(py_max(0.0, z_margin));
            gcmd.respond_info(&format!("firmware_retraction : z_hop is limited to {:.5}", self.current_z_hop_height.get()), true);
        } else {
            self.current_z_hop_height.set(z_hop_height);
        }
        self.current_z_hop_height.get() > 0.0
    }

    // Helper to get extrude_factor & speed_factor
    fn get_factors(&self) -> KResult<(f64, f64)> {
        let status = self.gcode_move()?.get_status(0.0);
        let get = |k: &str| match &status {
            Some(PyVal::Dict(d)) => d.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0),
            _ => 0.0,
        };
        Ok((get("extrude_factor"), get("speed_factor")))
    }
}

impl MoveTransform for FirmwareRetraction {
    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        self.do_move(newpos, speed)
    }
    fn get_position(&self) -> Vec<f64> {
        self.position().unwrap_or_else(|_| vec![0.0; 4])
    }
}

impl PrinterObject for FirmwareRetraction {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let p = self.p.get();
        let d = crate::pydict! {
            "retract_length" => PyVal::Float(p.retract_length),
            "retract_speed" => PyVal::Float(p.retract_speed),
            "unretract_extra_length" => PyVal::Float(p.unretract_extra_length),
            "unretract_speed" => PyVal::Float(p.unretract_speed),
            "z_hop_height" => PyVal::Float(p.z_hop_height),
            "unretract_length" => PyVal::Float(p.unretract_length),
            "retract_state" => PyVal::Bool(self.is_retracted.get()),
            "zhop_state" => PyVal::Bool(self.do_zhop.get()),
        };
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(FirmwareRetraction::new(config)?)
}
