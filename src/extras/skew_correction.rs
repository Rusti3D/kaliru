// Printer Skew Correction (port of klippy/extras/skew_correction.py)
//
// This implementation is a port of Marlin's skew correction as
// implemented in planner.h, Copyright (C) Marlin Firmware
//
// https://github.com/MarlinFirmware/Marlin/tree/1.1.x/Marlin
//
// Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_move::{GCodeMove, MoveTransform, MoveTransformRef};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_float_repr};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::rc::{Rc, Weak};

fn math_domain_error() -> KError {
    KError::Other("math domain error".into())
}

pub fn calc_skew_factor(ac: f64, bd: f64, ad: f64) -> KResult<f64> {
    let sq = 2.0 * ac * ac + 2.0 * bd * bd - 4.0 * ad * ad;
    if sq < 0.0 {
        return Err(math_domain_error());
    }
    let side = sq.sqrt() / 2.0;
    let c = (ac * ac - side * side - ad * ad) / (2.0 * side * ad);
    if !(-1.0..=1.0).contains(&c) {
        return Err(math_domain_error());
    }
    Ok((PI / 2.0 - c.acos()).tan())
}

const CMD_GET_CURRENT_SKEW_HELP: &str = "Report current printer skew";
const CMD_CALC_MEASURED_SKEW_HELP: &str = "Calculate skew from measured print";
const CMD_SET_SKEW_HELP: &str = "Set skew based on lengths of measured object";
const CMD_SKEW_PROFILE_HELP: &str = "Profile management for skew_correction";

pub struct PrinterSkew {
    me: Weak<PrinterSkew>,
    printer: Rc<Printer>,
    name: String,
    current_profile_name: RefCell<String>,
    xy_factor: Cell<f64>,
    xz_factor: Cell<f64>,
    yz_factor: Cell<f64>,
    skew_profiles: RefCell<IndexMap<String, [f64; 3]>>,
    next_transform: RefCell<Option<MoveTransformRef>>,
}

impl PrinterSkew {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().to_string();
        let s = Rc::new_cyclic(|me| PrinterSkew {
            me: me.clone(),
            printer: printer.clone(),
            name: name.clone(),
            current_profile_name: RefCell::new(String::new()),
            xy_factor: Cell::new(0.0),
            xz_factor: Cell::new(0.0),
            yz_factor: Cell::new(0.0),
            skew_profiles: RefCell::new(IndexMap::new()),
            next_transform: RefCell::new(None),
        });
        s.load_storage(config)?;
        let w = s.me.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match w.upgrade() {
                Some(s) => s.handle_connect(),
                None => Ok(()),
            }),
        );
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        type SCmd = fn(&PrinterSkew, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, SCmd, &str); 4] = [
            ("GET_CURRENT_SKEW", PrinterSkew::cmd_get_current_skew, CMD_GET_CURRENT_SKEW_HELP),
            ("CALC_MEASURED_SKEW", PrinterSkew::cmd_calc_measured_skew, CMD_CALC_MEASURED_SKEW_HELP),
            ("SET_SKEW", PrinterSkew::cmd_set_skew, CMD_SET_SKEW_HELP),
            ("SKEW_PROFILE", PrinterSkew::cmd_skew_profile, CMD_SKEW_PROFILE_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let w = s.me.clone();
            gcode.register_command(
                cmd,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(s) => f(&s, g),
                    None => Ok(()),
                })),
                false,
                Some(desc),
            )?;
        }
        Ok(s)
    }

    fn handle_connect(&self) -> KResult<()> {
        let gcode_move = self.printer.lookup::<GCodeMove>("gcode_move")?;
        let me: MoveTransformRef = self.me.upgrade().expect("skew");
        *self.next_transform.borrow_mut() = gcode_move.set_move_transform(me, true)?;
        Ok(())
    }

    fn load_storage(&self, config: &ConfigWrapper) -> KResult<()> {
        // Remove primary skew_correction section, as it is not a stored profile
        for profile in config.get_prefix_sections(&self.name) {
            if profile.get_name() == self.name {
                continue;
            }
            let Some((_, name)) = profile.get_name().split_once(' ') else {
                return Err(KError::Other("list index out of range".into()));
            };
            let name = name.to_string();
            let p = [
                profile.getfloat("xy_skew", Dflt::Required, Lim::new())?,
                profile.getfloat("xz_skew", Dflt::Required, Lim::new())?,
                profile.getfloat("yz_skew", Dflt::Required, Lim::new())?,
            ];
            self.skew_profiles.borrow_mut().insert(name, p);
        }
        Ok(())
    }

    fn calc_skew(&self, pos: &[f64]) -> Vec<f64> {
        let (xy, xz, yz) = (self.xy_factor.get(), self.xz_factor.get(), self.yz_factor.get());
        let skewed_x = pos[0] - pos[1] * xy - pos[2] * (xz - (xy * yz));
        let skewed_y = pos[1] - pos[2] * yz;
        let mut out = vec![skewed_x, skewed_y];
        out.extend_from_slice(&pos[2..]);
        out
    }

    fn calc_unskew(&self, pos: &[f64]) -> Vec<f64> {
        let (xy, xz, yz) = (self.xy_factor.get(), self.xz_factor.get(), self.yz_factor.get());
        let skewed_x = pos[0] + pos[1] * xy + pos[2] * xz;
        let skewed_y = pos[1] + pos[2] * yz;
        let mut out = vec![skewed_x, skewed_y];
        out.extend_from_slice(&pos[2..]);
        out
    }

    fn next(&self) -> KResult<MoveTransformRef> {
        self.next_transform.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'move'".into()))
    }

    fn update_skew(&self, xy_factor: f64, xz_factor: f64, yz_factor: f64) -> KResult<()> {
        self.xy_factor.set(xy_factor);
        self.xz_factor.set(xz_factor);
        self.yz_factor.set(yz_factor);
        self.printer.lookup::<GCodeMove>("gcode_move")?.reset_last_position()
    }

    fn cmd_get_current_skew(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let mut out = "Current Printer Skew:".to_string();
        let factors = [("XY", self.xy_factor.get()), ("XZ", self.xz_factor.get()), ("YZ", self.yz_factor.get())];
        for (plane, fac) in factors {
            out += &format!("\n{} Skew: {:.6} radians, {:.2} degrees", plane, fac, fac.to_degrees());
        }
        gcmd.respond_info(&out, true);
        Ok(())
    }

    fn cmd_calc_measured_skew(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let ac = gcmd.get_float("AC", None, Lim::new().above(0.0))?;
        let bd = gcmd.get_float("BD", None, Lim::new().above(0.0))?;
        let ad = gcmd.get_float("AD", None, Lim::new().above(0.0))?;
        let factor = calc_skew_factor(ac, bd, ad)?;
        gcmd.respond_info(&format!("Calculated Skew: {:.6} radians, {:.2} degrees", factor, factor.to_degrees()), true);
        Ok(())
    }

    fn cmd_set_skew(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if gcmd.get_int("CLEAR", Some(0), Lim::new())? != 0 {
            return self.update_skew(0.0, 0.0, 0.0);
        }
        for (plane, factor) in [("XY", &self.xy_factor), ("XZ", &self.xz_factor), ("YZ", &self.yz_factor)] {
            let Some(lengths) = gcmd.get_opt(plane) else { continue };
            // lengths.strip().split(",", 2)
            let parsed: Option<Vec<f64>> = lengths.trim().splitn(3, ',').map(|l| py_float(l.trim())).collect();
            let lengths = match parsed {
                Some(v) if v.len() == 3 => v,
                _ => {
                    return Err(gcmd.error(format!(
                        "skew_correction: improperly formatted entry for plane [{}]\n{}",
                        plane,
                        gcmd.get_commandline()
                    )))
                }
            };
            factor.set(calc_skew_factor(lengths[0], lengths[1], lengths[2])?);
        }
        Ok(())
    }

    fn cmd_skew_profile(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if let Some(name) = gcmd.get_opt("LOAD") {
            *self.current_profile_name.borrow_mut() = name.clone();
            let prof = self.skew_profiles.borrow().get(&name).copied();
            let Some(prof) = prof else {
                gcmd.respond_info(&format!("skew_correction:  Load failed, unknown profile [{}]", name), true);
                return Ok(());
            };
            self.update_skew(prof[0], prof[1], prof[2])?;
        } else if let Some(name) = gcmd.get_opt("SAVE") {
            let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
            let cfg_name = format!("{} {}", self.name, name);
            let p = [self.xy_factor.get(), self.xz_factor.get(), self.yz_factor.get()];
            {
                let mut pconfig = configfile.pconfig.borrow_mut();
                pconfig.set(&cfg_name, "xy_skew", &py_float_repr(p[0]))?;
                pconfig.set(&cfg_name, "xz_skew", &py_float_repr(p[1]))?;
                pconfig.set(&cfg_name, "yz_skew", &py_float_repr(p[2]))?;
            }
            // Copy to local storage
            self.skew_profiles.borrow_mut().insert(name.clone(), p);
            gcmd.respond_info(
                &format!(
                    "Skew Correction state has been saved to profile [{}]\nfor the current session.  The SAVE_CONFIG command will\nupdate the printer config file and restart the printer.",
                    name
                ),
                true,
            );
        } else if let Some(name) = gcmd.get_opt("REMOVE") {
            if self.skew_profiles.borrow().contains_key(&name) {
                let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
                configfile.pconfig.borrow_mut().remove_section(&format!("skew_correction {}", name));
                self.skew_profiles.borrow_mut().shift_remove(&name);
                gcmd.respond_info(
                    &format!(
                        "Profile [{}] removed from storage for this session.\nThe SAVE_CONFIG command will update the printer\nconfiguration and restart the printer",
                        name
                    ),
                    true,
                );
            } else {
                gcmd.respond_info(&format!("skew_correction: No profile named [{}] to remove", name), true);
            }
        }
        Ok(())
    }
}

impl MoveTransform for PrinterSkew {
    fn get_position(&self) -> Vec<f64> {
        match self.next() {
            Ok(n) => self.calc_unskew(&n.get_position()),
            Err(_) => vec![],
        }
    }

    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let corrected_pos = self.calc_skew(newpos);
        self.next()?.move_to(&corrected_pos, speed)
    }
}

impl PrinterObject for PrinterSkew {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("current_profile_name".to_string(), PyVal::Str(self.current_profile_name.borrow().clone()));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.next_transform.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterSkew::new(config)?)
}
