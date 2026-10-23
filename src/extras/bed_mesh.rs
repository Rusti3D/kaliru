// Mesh Bed Leveling (port of klippy/extras/bed_mesh.py)
//
// Copyright (C) 2018-2019 Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim, ListParser};
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::extras::gcode_move::{GCodeMove, MoveTransform};
use crate::extras::probe::{as_printer_probe, probe_list_get_list, FinalizeResult, ProbePointsHelper};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{downcast_rc, ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float_repr, py_max, py_min, py_round, py_strip};
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const PROFILE_VERSION: i64 = 1;
/// PROFILE_OPTIONS: (key, type) with type 'f' float, 'i' int, 's' str
pub const PROFILE_OPTIONS: &[(&str, char)] = &[
    ("min_x", 'f'),
    ("max_x", 'f'),
    ("min_y", 'f'),
    ("max_y", 'f'),
    ("x_count", 'i'),
    ("y_count", 'i'),
    ("mesh_x_pps", 'i'),
    ("mesh_y_pps", 'i'),
    ("algo", 's'),
    ("tension", 'f'),
];

/// mesh parameters (a python dict of ints, floats and str)
pub type MeshParams = IndexMap<String, PyVal>;

fn pf(p: &MeshParams, k: &str) -> f64 {
    p.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0)
}

fn pi(p: &MeshParams, k: &str) -> i64 {
    p.get(k).and_then(|v| v.as_i64()).unwrap_or(0)
}

fn ps(p: &MeshParams, k: &str) -> String {
    p.get(k).map(|v| v.str()).unwrap_or_default()
}

// PEP 485 isclose()
pub fn isclose(a: f64, b: f64, rel_tol: f64, abs_tol: f64) -> bool {
    (a - b).abs() <= py_max(rel_tol * py_max(a.abs(), b.abs()), abs_tol)
}

fn isclose_d(a: f64, b: f64) -> bool {
    isclose(a, b, 1e-09, 0.0)
}

// return true if a coordinate is within the region
// specified by min_c and max_c
pub fn within(coord: (f64, f64), min_c: (f64, f64), max_c: (f64, f64), tol: f64) -> bool {
    (max_c.0 + tol) >= coord.0 && coord.0 >= (min_c.0 - tol) && (max_c.1 + tol) >= coord.1 && coord.1 >= (min_c.1 - tol)
}

// Constrain value between min and max
pub fn constrain(val: f64, min_val: f64, max_val: f64) -> f64 {
    py_min(max_val, py_max(min_val, val))
}

// Linear interpolation between two values
pub fn lerp(t: f64, v0: f64, v1: f64) -> f64 {
    (1.0 - t) * v0 + t * v1
}

/// "(%.2f, %.2f)" style tuple repr of floats
fn tuple_repr(v: &[f64]) -> String {
    PyVal::Tuple(v.iter().map(|x| PyVal::Float(*x)).collect()).repr()
}

fn list_repr(v: &[f64]) -> String {
    PyVal::List(v.iter().map(|x| PyVal::Float(*x)).collect()).repr()
}

fn matrix_repr(m: &[Vec<f64>]) -> String {
    PyVal::List(m.iter().map(|r| PyVal::List(r.iter().map(|x| PyVal::Float(*x)).collect())).collect()).repr()
}

fn matrix_pyval(m: &[Vec<f64>]) -> PyVal {
    PyVal::List(m.iter().map(|r| PyVal::List(r.iter().map(|x| PyVal::Float(*x)).collect())).collect())
}

// retreive commma separated pair from config
fn parse_config_pair(config: &ConfigWrapper, option: &str, default: i64, minval: Option<i64>, maxval: Option<i64>) -> KResult<(i64, i64)> {
    let pair = config
        .getintlist(option, PyVal::Tuple(vec![PyVal::Int(default), PyVal::Int(default)]), ",", None)?
        .unwrap_or_default();
    let pair = if pair.len() != 2 {
        if pair.len() != 1 {
            return Err(KError::config(format!(
                "bed_mesh: malformed '{}' value: {}",
                option,
                config.get(option, Dflt::Required)?
            )));
        }
        (pair[0], pair[0])
    } else {
        (pair[0], pair[1])
    };
    if let Some(minval) = minval {
        if pair.0 < minval || pair.1 < minval {
            return Err(KError::config(format!("Option '{}' in section bed_mesh must have a minimum of {}", option, minval)));
        }
    }
    if let Some(maxval) = maxval {
        if pair.0 > maxval || pair.1 > maxval {
            return Err(KError::config(format!("Option '{}' in section bed_mesh must have a maximum of {}", option, maxval)));
        }
    }
    Ok(pair)
}

// retreive commma separated pair from a g-code command
fn parse_gcmd_pair(gcmd: &GCodeCommand, name: &str, minval: Option<i64>, maxval: Option<i64>) -> KResult<(i64, i64)> {
    let err = || gcmd.error(format!("Unable to parse parameter '{}'", name));
    let raw = gcmd.get(name).map_err(|_| err())?;
    let pair: Option<Vec<i64>> = raw.split(',').map(|v| crate::pyfmt::py_int(py_strip(v))).collect();
    let pair = pair.ok_or_else(err)?;
    let pair = if pair.len() != 2 {
        if pair.len() != 1 {
            return Err(err());
        }
        (pair[0], pair[0])
    } else {
        (pair[0], pair[1])
    };
    if let Some(minval) = minval {
        if pair.0 < minval || pair.1 < minval {
            return Err(gcmd.error(format!("Parameter '{}' must have a minimum of {}", name, minval)));
        }
    }
    if let Some(maxval) = maxval {
        if pair.0 > maxval || pair.1 > maxval {
            return Err(gcmd.error(format!("Parameter '{}' must have a maximum of {}", name, maxval)));
        }
    }
    Ok(pair)
}

// retreive commma separated coordinate from a g-code command
fn parse_gcmd_coord(gcmd: &GCodeCommand, name: &str) -> KResult<(f64, f64)> {
    let err = || gcmd.error(format!("Unable to parse parameter '{}'", name));
    let raw = gcmd.get(name).map_err(|_| err())?;
    let v: Option<Vec<f64>> = raw.split(',').map(|v| crate::pyfmt::py_float(py_strip(v))).collect();
    match v.as_deref() {
        Some([v1, v2]) => Ok((*v1, *v2)),
        _ => Err(err()),
    }
}

// ----------------------------------------------------------------------
// BedMesh
// ----------------------------------------------------------------------

const CMD_BED_MESH_OUTPUT_HELP: &str = "Retrieve interpolated grid of probed z-points";
const CMD_BED_MESH_MAP_HELP: &str = "Serialize mesh and output to terminal";
const CMD_BED_MESH_CLEAR_HELP: &str = "Clear the Mesh so no z-adjustment is made";
const CMD_BED_MESH_OFFSET_HELP: &str = "Add X/Y offsets to the mesh lookup";
const CMD_BED_MESH_CHECK_HELP: &str = "Validate a variety of bed mesh parameters";
const CMD_BED_MESH_CALIBRATE_HELP: &str = "Perform Mesh Bed Leveling";
const CMD_BED_MESH_PROFILE_HELP: &str = "Bed Mesh Persistent Storage management";

pub const FADE_DISABLE: f64 = 0x7FFFFFFF as f64;

pub struct BedMesh {
    printer: Rc<Printer>,
    last_position: RefCell<Vec<f64>>,
    bmc: RefCell<Option<Rc<BedMeshCalibrate>>>,
    z_mesh: RefCell<Option<Rc<ZMesh>>>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    horizontal_move_z: Cell<f64>,
    fade_start: Cell<f64>,
    fade_end: Cell<f64>,
    fade_dist: Cell<f64>,
    log_fade_complete: Cell<bool>,
    base_fade_target: Cell<Option<f64>>,
    fade_target: Cell<f64>,
    tool_offset: Cell<f64>,
    splitter: RefCell<Option<Rc<MoveSplitter>>>,
    pmgr: RefCell<Option<Rc<ProfileManager>>>,
    default_mesh_name: RefCell<Option<String>>,
    status: RefCell<PyVal>,
}

impl BedMesh {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        // the config options are read in python's order: the fields fill
        // in as __init__ goes
        let bm = Rc::new(BedMesh {
            printer: printer.clone(),
            last_position: RefCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            bmc: RefCell::new(None),
            z_mesh: RefCell::new(None),
            toolhead: RefCell::new(None),
            horizontal_move_z: Cell::new(0.0),
            fade_start: Cell::new(0.0),
            fade_end: Cell::new(0.0),
            fade_dist: Cell::new(0.0),
            log_fade_complete: Cell::new(false),
            base_fade_target: Cell::new(None),
            fade_target: Cell::new(0.0),
            tool_offset: Cell::new(0.0),
            splitter: RefCell::new(None),
            pmgr: RefCell::new(None),
            default_mesh_name: RefCell::new(None),
            status: RefCell::new(PyVal::dict()),
        });
        let b = bm.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| b.handle_connect()));
        *bm.bmc.borrow_mut() = Some(BedMeshCalibrate::new(config, &bm)?);
        bm.horizontal_move_z.set(config.getfloat("horizontal_move_z", 5.0, Lim::new())?);
        let mut fade_start = config.getfloat("fade_start", 1.0, Lim::new())?;
        let mut fade_end = config.getfloat("fade_end", 0.0, Lim::new())?;
        let fade_dist = fade_end - fade_start;
        if fade_dist <= 0.0 {
            fade_start = FADE_DISABLE;
            fade_end = FADE_DISABLE;
        }
        bm.fade_start.set(fade_start);
        bm.fade_end.set(fade_end);
        bm.fade_dist.set(fade_dist);
        bm.base_fade_target.set(config.getfloat_opt("fade_target", Lim::new())?);
        *bm.splitter.borrow_mut() = Some(Rc::new(MoveSplitter::new(config, &gcode)?));
        // setup persistent storage
        *bm.pmgr.borrow_mut() = Some(ProfileManager::new(config, &bm)?);
        *bm.default_mesh_name.borrow_mut() = config.get_opt("bed_mesh_default")?;
        // register gcodes
        type BCmd = fn(&Rc<BedMesh>, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, BCmd, &str); 5] = [
            ("BED_MESH_OUTPUT", BedMesh::cmd_bed_mesh_output, CMD_BED_MESH_OUTPUT_HELP),
            ("BED_MESH_MAP", BedMesh::cmd_bed_mesh_map, CMD_BED_MESH_MAP_HELP),
            ("BED_MESH_CLEAR", BedMesh::cmd_bed_mesh_clear, CMD_BED_MESH_CLEAR_HELP),
            ("BED_MESH_OFFSET", BedMesh::cmd_bed_mesh_offset, CMD_BED_MESH_OFFSET_HELP),
            ("BED_MESH_CHECK", BedMesh::cmd_bed_mesh_check, CMD_BED_MESH_CHECK_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let b = bm.clone();
            gcode.register_command(cmd, Some(Rc::new(move |g| f(&b, g))), false, Some(desc))?;
        }
        // Register transform
        let gcode_move = printer
            .load_object(config, "gcode_move", true)?
            .and_then(downcast_rc::<GCodeMove>)
            .ok_or_else(|| KError::Other("gcode_move has an unexpected type".into()))?;
        gcode_move.set_move_transform(bm.clone(), false)?;
        // initialize status dict
        bm.update_status();
        Ok(bm)
    }

    fn splitter(&self) -> Rc<MoveSplitter> {
        self.splitter.borrow().clone().expect("splitter")
    }

    fn pmgr(&self) -> Rc<ProfileManager> {
        self.pmgr.borrow().clone().expect("pmgr")
    }

    fn bmc(&self) -> Rc<BedMeshCalibrate> {
        self.bmc.borrow().clone().expect("bmc")
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.toolhead.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_position'".into()))
    }

    fn handle_connect(&self) -> KResult<()> {
        *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);
        let default_mesh_name = self.default_mesh_name.borrow().clone();
        if let Some(name) = default_mesh_name.filter(|n| !n.is_empty()) {
            if self.pmgr().get_profiles().contains_key(&name) {
                self.pmgr().load_profile(&name)?;
            } else {
                let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
                configfile.pconfig.borrow_mut().warn(
                    "config",
                    &format!("Selected default bed mesh profile '{}' not in available profiles.", name),
                    Some("Invalid profile name"),
                    None,
                    None,
                );
            }
        }
        if get_danger_options().log_bed_mesh_at_startup {
            self.bmc().print_generated_points(&|m: &str| logging::info(m), true)?;
        }
        Ok(())
    }

    pub fn set_mesh(&self, mesh: Option<Rc<ZMesh>>) -> KResult<()> {
        match &mesh {
            Some(m) if self.fade_end.get() != FADE_DISABLE => {
                self.log_fade_complete.set(true);
                match self.base_fade_target.get() {
                    None => self.fade_target.set(m.get_z_average()),
                    Some(t) => {
                        self.fade_target.set(t);
                        let (min_z, max_z) = m.get_z_range();
                        let ft = self.fade_target.get();
                        if !(min_z <= ft && ft <= max_z) && ft != 0.0 {
                            // fade target is non-zero, out of mesh range
                            let err_target = ft;
                            *self.z_mesh.borrow_mut() = None;
                            self.fade_target.set(0.0);
                            return Err(KError::command(format!(
                                "bed_mesh: ERROR, fade_target lies outside of mesh z range\nmin: {:.4}, max: {:.4}, fade_target: {:.4}",
                                min_z, max_z, err_target
                            )));
                        }
                    }
                }
                let (min_z, max_z) = m.get_z_range();
                if self.fade_dist.get() <= py_max(min_z.abs(), max_z.abs()) {
                    *self.z_mesh.borrow_mut() = None;
                    self.fade_target.set(0.0);
                    return Err(KError::command(format!(
                        "bed_mesh:  Mesh extends outside of the fade range, please see the fade_start and fade_end options inexample-extras.cfg. fade distance: {:.2} mesh min: {:.4}mesh max: {:.4}",
                        self.fade_dist.get(), min_z, max_z
                    )));
                }
            }
            _ => self.fade_target.set(0.0),
        }
        self.tool_offset.set(0.0);
        *self.z_mesh.borrow_mut() = mesh.clone();
        self.splitter().initialize(mesh, self.fade_target.get());
        // cache the current position before a transform takes place
        let gcode_move = self.printer.lookup::<GCodeMove>("gcode_move")?;
        gcode_move.reset_last_position()?;
        self.update_status();
        Ok(())
    }

    pub fn get_z_factor(&self, mut z_pos: f64) -> f64 {
        z_pos += self.tool_offset.get();
        if z_pos >= self.fade_end.get() {
            0.0
        } else if z_pos >= self.fade_start.get() {
            (self.fade_end.get() - z_pos) / self.fade_dist.get()
        } else {
            1.0
        }
    }

    fn bm_get_position(&self) -> KResult<Vec<f64>> {
        // Return last, non-transformed position
        let toolhead = self.toolhead()?;
        let z_mesh = self.z_mesh.borrow().clone();
        match z_mesh {
            None => {
                // No mesh calibrated, so send toolhead position
                let mut lp = toolhead.get_position();
                lp[2] -= self.fade_target.get();
                *self.last_position.borrow_mut() = lp;
            }
            Some(z_mesh) => {
                // return current position minus the current z-adjustment
                let cur_pos = toolhead.get_position();
                let (x, y, z) = (cur_pos[0], cur_pos[1], cur_pos[2]);
                let max_adj = z_mesh.calc_z(x, y);
                let mut factor = 1.0;
                let fade_target = self.fade_target.get();
                let z_adj = max_adj - fade_target;
                let fade_z_pos = z + self.tool_offset.get();
                if py_min(fade_z_pos, fade_z_pos - max_adj) >= self.fade_end.get() {
                    // Fade out is complete, no factor
                    factor = 0.0;
                } else if py_max(fade_z_pos, fade_z_pos - max_adj) >= self.fade_start.get() {
                    // Likely in the process of fading out adjustment.
                    // Because we don't yet know the gcode z position, use
                    // algebra to calculate the factor from the toolhead pos
                    factor = (self.fade_end.get() + fade_target - fade_z_pos) / (self.fade_dist.get() - z_adj);
                    factor = constrain(factor, 0.0, 1.0);
                }
                let final_z_adj = factor * z_adj + fade_target;
                let mut lp = vec![x, y, z - final_z_adj];
                lp.extend_from_slice(&cur_pos[3..]);
                *self.last_position.borrow_mut() = lp;
            }
        }
        Ok(self.last_position.borrow().clone())
    }

    fn bm_move(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let toolhead = self.toolhead()?;
        let factor = self.get_z_factor(newpos[2]);
        let z_mesh = self.z_mesh.borrow().clone();
        if z_mesh.is_none() || factor == 0.0 {
            // No mesh calibrated, or mesh leveling phased out.
            let (x, y, z) = (newpos[0], newpos[1], newpos[2]);
            if self.log_fade_complete.get() {
                self.log_fade_complete.set(false);
                logging::info(&format!(
                    "bed_mesh fade complete: Current Z: {:.4} fade_target: {:.4} ",
                    z,
                    self.fade_target.get()
                ));
            }
            let mut p = vec![x, y, z + self.fade_target.get()];
            p.extend_from_slice(&newpos[3..]);
            toolhead.move_to(&p, speed)?;
        } else {
            let last = self.last_position.borrow().clone();
            let splitter = self.splitter();
            splitter.build_move(&last, newpos, factor)?;
            while !splitter.traverse_complete.get() {
                match splitter.split()? {
                    Some(split_move) => toolhead.move_to(&split_move, speed)?,
                    None => return Err(KError::command("Mesh Leveling: Error splitting move ")),
                }
            }
        }
        *self.last_position.borrow_mut() = newpos.to_vec();
        Ok(())
    }

    pub fn update_status(&self) {
        let mut st = IndexMap::new();
        st.insert("profile_name".to_string(), PyVal::from(""));
        st.insert("mesh_min".to_string(), PyVal::Tuple(vec![PyVal::Float(0.0), PyVal::Float(0.0)]));
        st.insert("mesh_max".to_string(), PyVal::Tuple(vec![PyVal::Float(0.0), PyVal::Float(0.0)]));
        st.insert("probed_matrix".to_string(), PyVal::List(vec![PyVal::List(vec![])]));
        st.insert("mesh_matrix".to_string(), PyVal::List(vec![PyVal::List(vec![])]));
        st.insert("profiles".to_string(), self.pmgr().profiles_pyval());
        if let Some(z_mesh) = self.z_mesh.borrow().as_ref() {
            let params = z_mesh.get_mesh_params();
            let t = |a: f64, b: f64| PyVal::Tuple(vec![PyVal::Float(a), PyVal::Float(b)]);
            st.insert("profile_name".to_string(), PyVal::Str(z_mesh.get_profile_name()));
            st.insert("mesh_min".to_string(), t(pf(&params, "min_x"), pf(&params, "min_y")));
            st.insert("mesh_max".to_string(), t(pf(&params, "max_x"), pf(&params, "max_y")));
            st.insert("probed_matrix".to_string(), matrix_pyval(&z_mesh.get_probed_matrix()));
            st.insert("mesh_matrix".to_string(), matrix_pyval(&z_mesh.get_mesh_matrix()));
        }
        *self.status.borrow_mut() = PyVal::Dict(st);
    }

    pub fn get_mesh(&self) -> Option<Rc<ZMesh>> {
        self.z_mesh.borrow().clone()
    }

    fn cmd_bed_mesh_output(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        if gcmd.get_int("PGP", Some(0), Lim::new())? != 0 {
            // Print Generated Points instead of mesh
            self.bmc().print_generated_points(&|m: &str| gcmd.respond_info(m, true), false)?;
        } else if let Some(z_mesh) = self.get_mesh() {
            z_mesh.print_probed_matrix(&|m: &str| gcmd.respond_info(m, true));
            z_mesh.print_mesh(&|m: &str| gcmd.respond_raw(m), Some(self.horizontal_move_z.get()));
        } else {
            gcmd.respond_info("Bed has not been probed", true);
        }
        Ok(())
    }

    fn cmd_bed_mesh_map(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        match self.get_mesh() {
            Some(z_mesh) => {
                let params = z_mesh.get_mesh_params();
                let t = |a: f64, b: f64| PyVal::Tuple(vec![PyVal::Float(a), PyVal::Float(b)]);
                let mut outdict = IndexMap::new();
                outdict.insert("mesh_min".to_string(), t(pf(&params, "min_x"), pf(&params, "min_y")));
                outdict.insert("mesh_max".to_string(), t(pf(&params, "max_x"), pf(&params, "max_y")));
                outdict.insert("z_positions".to_string(), matrix_pyval(&z_mesh.get_probed_matrix()));
                gcmd.respond_raw(&format!("mesh_map_output {}", PyVal::Dict(outdict).json()));
            }
            None => gcmd.respond_info("Bed has not been probed", true),
        }
        Ok(())
    }

    fn cmd_bed_mesh_clear(self: &Rc<Self>, _gcmd: &GCodeCommand) -> KResult<()> {
        self.set_mesh(None)
    }

    fn cmd_bed_mesh_offset(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        match self.get_mesh() {
            Some(z_mesh) => {
                let mut offsets = [None, None];
                for (i, axis) in ["X", "Y"].iter().enumerate() {
                    offsets[i] = gcmd.get_float_opt(axis, Lim::new())?;
                }
                z_mesh.set_mesh_offsets(&offsets);
                if let Some(tool_offset) = gcmd.get_float_opt("ZFADE", Lim::new())? {
                    self.tool_offset.set(tool_offset);
                }
                let gcode_move = self.printer.lookup::<GCodeMove>("gcode_move")?;
                gcode_move.reset_last_position()?;
            }
            None => gcmd.respond_info("No mesh loaded to offset", true),
        }
        Ok(())
    }

    fn cmd_bed_mesh_check(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let Some(z_mesh) = self.get_mesh() else {
            return Err(KError::command("No mesh has been loaded"));
        };

        let mut has_checks = false;

        // Validate mesh deviation if MAX_DEVIATION is specified
        if let Some(max_deviation) = gcmd.get_float_opt("MAX_DEVIATION", Lim::new())? {
            has_checks = true;
            if max_deviation <= 0.0 {
                return Err(KError::command("MAX_DEVIATION must be greater than 0"));
            }

            let (mesh_min, mesh_max) = z_mesh.get_z_range();
            let current_deviation = mesh_max - mesh_min;

            if current_deviation > max_deviation {
                return Err(KError::command(format!(
                    "Mesh deviation ({:.6}) exceeds maximum allowed deviation ({:.6})",
                    current_deviation, max_deviation
                )));
            } else {
                gcmd.respond_info(
                    &format!("Mesh deviation ({:.6}) is within the allowed maximum ({:.6})", current_deviation, max_deviation),
                    true,
                );
            }
        }

        // Validate maximum slope between adjacent points if MAX_SLOPE is specified
        if let Some(max_slope) = gcmd.get_float_opt("MAX_SLOPE", Lim::new())? {
            has_checks = true;
            if max_slope <= 0.0 {
                return Err(KError::command("MAX_SLOPE must be greater than 0"));
            }

            // Get the mesh matrix and parameters
            let mesh_matrix = z_mesh.get_mesh_matrix();
            let params = z_mesh.get_mesh_params();

            // Calculate the distance between adjacent points
            let x_dist = (pf(&params, "max_x") - pf(&params, "min_x")) / (pi(&params, "x_count") - 1) as f64;
            let y_dist = (pf(&params, "max_y") - pf(&params, "min_y")) / (pi(&params, "y_count") - 1) as f64;

            let mut max_slope_value = 0.0;
            let mut max_slope_pos: Option<(usize, usize, usize, usize)> = None;

            // Check slopes in X direction
            for y in 0..mesh_matrix.len() {
                for x in 0..mesh_matrix[0].len().saturating_sub(1) {
                    let z1 = mesh_matrix[y][x];
                    let z2 = mesh_matrix[y][x + 1];
                    let slope = ((z2 - z1) / x_dist).abs();
                    if slope > max_slope_value {
                        max_slope_value = slope;
                        max_slope_pos = Some((x, y, x + 1, y));
                    }
                }
            }

            // Check slopes in Y direction
            for x in 0..mesh_matrix[0].len() {
                for y in 0..mesh_matrix.len().saturating_sub(1) {
                    let z1 = mesh_matrix[y][x];
                    let z2 = mesh_matrix[y + 1][x];
                    let slope = ((z2 - z1) / y_dist).abs();
                    if slope > max_slope_value {
                        max_slope_value = slope;
                        max_slope_pos = Some((x, y, x, y + 1));
                    }
                }
            }

            if max_slope_value > max_slope {
                // Calculate the actual positions in bed coordinates
                let p = max_slope_pos.unwrap_or((0, 0, 0, 0));
                let x1 = pf(&params, "min_x") + p.0 as f64 * x_dist;
                let y1 = pf(&params, "min_y") + p.1 as f64 * y_dist;
                let x2 = pf(&params, "min_x") + p.2 as f64 * x_dist;
                let y2 = pf(&params, "min_y") + p.3 as f64 * y_dist;

                return Err(KError::command(format!(
                    "Maximum slope ({:.6} mm/mm) between points ({:.2},{:.2}) and ({:.2},{:.2}) exceeds allowed maximum ({:.6} mm/mm)",
                    max_slope_value, x1, y1, x2, y2, max_slope
                )));
            } else {
                gcmd.respond_info(
                    &format!("Maximum slope ({:.6} mm/mm) is within the allowed maximum ({:.6} mm/mm)", max_slope_value, max_slope),
                    true,
                );
            }
        }

        if !has_checks {
            gcmd.respond_info(
                "No validation checks specified. Available checks:\nMAX_DEVIATION - Validate maximum mesh height deviation\nMAX_SLOPE - Validate maximum slope between adjacent points",
                true,
            );
        }
        Ok(())
    }
}

impl MoveTransform for BedMesh {
    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        self.bm_move(newpos, speed)
    }
    fn get_position(&self) -> Vec<f64> {
        self.bm_get_position().unwrap_or_default()
    }
}

impl PrinterObject for BedMesh {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(self.status.borrow().clone())
    }
    fn teardown(&self) {
        *self.bmc.borrow_mut() = None;
        *self.pmgr.borrow_mut() = None;
        *self.z_mesh.borrow_mut() = None;
        if let Some(s) = self.splitter.borrow().as_ref() {
            s.initialize(None, 0.0);
        }
    }
}

// ----------------------------------------------------------------------
// BedMeshCalibrate
// ----------------------------------------------------------------------

/// ZrefMode
#[derive(Clone, Copy, PartialEq)]
enum ZrefMode {
    /// Zero reference disabled
    Disabled,
    /// Zero reference position within mesh
    InMesh,
    /// Zero refrennce position outside of mesh, probe needed
    Probe,
}

pub const ALGOS: &[&str] = &["lagrange", "bicubic"];

#[derive(Clone)]
struct OrigConfig {
    radius: Option<f64>,
    origin: Option<(f64, f64)>,
    mesh_min: (f64, f64),
    mesh_max: (f64, f64),
    mesh_config: MeshParams,
}

/// the config BedMeshCalibrate.__init__ reads before the probe helper
struct BedMeshCalibrateConfig {
    adaptive_margin: f64,
    zero_ref_pos: Option<(f64, f64)>,
    radius: Option<f64>,
    origin: Option<(f64, f64)>,
    mesh_min: (f64, f64),
    mesh_max: (f64, f64),
    mesh_config: MeshParams,
    orig_config: OrigConfig,
    faulty_regions: Vec<((f64, f64), (f64, f64))>,
}

impl BedMeshCalibrateConfig {
    fn read(config: &ConfigWrapper) -> KResult<Self> {
        let adaptive_margin = config.getfloat("adaptive_margin", 0.0, Lim::new())?;
        let zero_ref_pos = config.getfloatlist("zero_reference_position", Dflt::None, ",", Some(2))?.map(|v| (v[0], v[1]));
        let mut c = BedMeshCalibrateConfig {
            adaptive_margin,
            zero_ref_pos,
            radius: None,
            origin: None,
            mesh_min: (0.0, 0.0),
            mesh_max: (0.0, 0.0),
            mesh_config: IndexMap::new(),
            orig_config: OrigConfig { radius: None, origin: None, mesh_min: (0.0, 0.0), mesh_max: (0.0, 0.0), mesh_config: IndexMap::new() },
            faulty_regions: vec![],
        };
        c.init_mesh_config(config)?;
        Ok(c)
    }

    fn init_mesh_config(&mut self, config: &ConfigWrapper) -> KResult<()> {
        self.radius = config.getfloat_opt("mesh_radius", Lim::new().above(0.0))?;
        let (x_cnt, y_cnt, min_x, min_y, max_x, max_y);
        if let Some(radius) = self.radius {
            let origin = config
                .getfloatlist("mesh_origin", PyVal::Tuple(vec![PyVal::Float(0.0), PyVal::Float(0.0)]), ",", Some(2))?
                .unwrap_or_default();
            self.origin = Some((origin[0], origin[1]));
            let cnt = config.getint("round_probe_count", 5, Lim::new().min(3))?;
            x_cnt = cnt;
            y_cnt = cnt;
            // round beds must have an odd number of points along each axis
            if x_cnt & 1 == 0 {
                return Err(KError::config("bed_mesh: probe_count must be odd for round beds"));
            }
            // radius may have precision to .1mm
            let radius = (radius * 10.0).floor() / 10.0;
            self.radius = Some(radius);
            self.orig_config.radius = self.radius;
            self.orig_config.origin = self.origin;
            min_x = -radius;
            min_y = -radius;
            max_x = radius;
            max_y = radius;
        } else {
            // rectangular
            let (xc, yc) = parse_config_pair(config, "probe_count", 3, Some(3), None)?;
            x_cnt = xc;
            y_cnt = yc;
            let mn = config.getfloatlist("mesh_min", Dflt::Required, ",", Some(2))?.unwrap_or_default();
            let mx = config.getfloatlist("mesh_max", Dflt::Required, ",", Some(2))?.unwrap_or_default();
            min_x = mn[0];
            min_y = mn[1];
            max_x = mx[0];
            max_y = mx[1];
            if max_x <= min_x || max_y <= min_y {
                return Err(KError::config("bed_mesh: invalid min/max points"));
            }
        }
        let mc = &mut self.mesh_config;
        mc.insert("x_count".into(), PyVal::Int(x_cnt));
        mc.insert("y_count".into(), PyVal::Int(y_cnt));
        self.mesh_min = (min_x, min_y);
        self.mesh_max = (max_x, max_y);
        self.orig_config.mesh_min = self.mesh_min;
        self.orig_config.mesh_max = self.mesh_max;

        let pps = parse_config_pair(config, "mesh_pps", 2, Some(0), None)?;
        let mc = &mut self.mesh_config;
        mc.insert("mesh_x_pps".into(), PyVal::Int(pps.0));
        mc.insert("mesh_y_pps".into(), PyVal::Int(pps.1));
        let algo = py_strip(&config.get("algorithm", "lagrange")?).to_lowercase();
        mc.insert("algo".into(), PyVal::Str(algo));
        let tension = config.getfloat("bicubic_tension", 0.2, Lim::new().min(0.0).max(2.0))?;
        self.mesh_config.insert("tension".into(), PyVal::Float(tension));
        self.orig_config.mesh_config = self.mesh_config.clone();
        for i in 1..100 {
            let Some(start) = config.getfloatlist(&format!("faulty_region_{}_min", i), Dflt::None, ",", Some(2))? else {
                break;
            };
            let end = config.getfloatlist(&format!("faulty_region_{}_max", i), Dflt::Required, ",", Some(2))?.unwrap_or_default();
            // Validate the corners.  If necessary reorganize them.
            // c1 = min point, c3 = max point
            //  c4 ---- c3
            //  |        |
            //  c1 ---- c2
            let c1 = (py_min(start[0], end[0]), py_min(start[1], end[1]));
            let c3 = (py_max(start[0], end[0]), py_max(start[1], end[1]));
            let c2 = (c1.0, c3.1);
            let c4 = (c3.0, c1.1);
            let rr = |a: (f64, f64), b: (f64, f64)| format!("[{}, {}]", list_repr(&[a.0, a.1]), list_repr(&[b.0, b.1]));
            // Check for overlapping regions
            for (j, (prev_c1, prev_c3)) in self.faulty_regions.iter().enumerate() {
                let prev_c2 = (prev_c1.0, prev_c3.1);
                let prev_c4 = (prev_c3.0, prev_c1.1);
                // Validate that no existing corner is within the new region
                for coord in [*prev_c1, prev_c2, *prev_c3, prev_c4] {
                    if within(coord, c1, c3, 0.0) {
                        return Err(KError::config(format!(
                            "bed_mesh: Existing faulty_region_{} {} overlaps added faulty_region_{} {}",
                            j + 1,
                            rr(*prev_c1, *prev_c3),
                            i,
                            rr(c1, c3)
                        )));
                    }
                }
                // Validate that no new corner is within an existing region
                for coord in [c1, c2, c3, c4] {
                    if within(coord, *prev_c1, *prev_c3, 0.0) {
                        return Err(KError::config(format!(
                            "bed_mesh: Added faulty_region_{} {} overlaps existing faulty_region_{} {}",
                            i,
                            rr(c1, c3),
                            j + 1,
                            rr(*prev_c1, *prev_c3)
                        )));
                    }
                }
            }
            self.faulty_regions.push((c1, c3));
        }
        verify_algorithm(&mut self.mesh_config, &|m| KError::config(m))
    }
}

fn verify_algorithm(params: &mut MeshParams, error: &dyn Fn(String) -> KError) -> KResult<()> {
    let x_pps = pi(params, "mesh_x_pps");
    let y_pps = pi(params, "mesh_y_pps");
    let algo = ps(params, "algo");
    if !ALGOS.contains(&algo.as_str()) {
        return Err(error(format!("bed_mesh: Unknown algorithm <{}>", algo)));
    }
    // Check the algorithm against the current configuration
    let (xc, yc) = (pi(params, "x_count"), pi(params, "y_count"));
    let max_probe_cnt = xc.max(yc);
    let min_probe_cnt = xc.min(yc);
    if x_pps.max(y_pps) == 0 {
        // Interpolation disabled
        params.insert("algo".into(), PyVal::from("direct"));
    } else if algo == "lagrange" && max_probe_cnt > 6 {
        // Lagrange interpolation tends to oscillate when using more
        // than 6 samples
        return Err(error(format!(
            "bed_mesh: cannot exceed a probe_count of 6 when using lagrange interpolation. Configured Probe Count: {}, {}",
            xc, yc
        )));
    } else if algo == "bicubic" && min_probe_cnt < 4 {
        if max_probe_cnt > 6 {
            return Err(error(format!(
                "bed_mesh: invalid probe_count option when using bicubic interpolation.  Combination of 3 points on one axis with more than 6 on another is not permitted. Configured Probe Count: {}, {}",
                xc, yc
            )));
        } else {
            logging::info(&format!(
                "bed_mesh: bicubic interpolation with a probe_count of less than 4 points detected.  Forcing lagrange interpolation. Configured Probe Count: {}, {}",
                xc, yc
            ));
            params.insert("algo".into(), PyVal::from("lagrange"));
        }
    }
    Ok(())
}

pub struct BedMeshCalibrate {
    printer: Rc<Printer>,
    orig_config: OrigConfig,
    radius: Cell<Option<f64>>,
    origin: Cell<Option<(f64, f64)>>,
    mesh_min: Cell<(f64, f64)>,
    mesh_max: Cell<(f64, f64)>,
    adaptive_margin: f64,
    zero_ref_pos: Option<(f64, f64)>,
    zero_reference_mode: Cell<ZrefMode>,
    faulty_regions: Vec<((f64, f64), (f64, f64))>,
    substituted_indices: RefCell<IndexMap<usize, Vec<(f64, f64)>>>,
    bedmesh: Weak<BedMesh>,
    mesh_config: RefCell<MeshParams>,
    points: RefCell<Vec<(f64, f64)>>,
    profile_name: RefCell<Option<String>>,
    probe_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    gcode: Rc<GCodeDispatch>,
}

impl BedMeshCalibrate {
    fn new(config: &ConfigWrapper, bedmesh: &Rc<BedMesh>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let cfg = BedMeshCalibrateConfig::read(config)?;
        let bmc = Rc::new(BedMeshCalibrate {
            printer: printer.clone(),
            orig_config: cfg.orig_config,
            radius: Cell::new(cfg.radius),
            origin: Cell::new(cfg.origin),
            mesh_min: Cell::new(cfg.mesh_min),
            mesh_max: Cell::new(cfg.mesh_max),
            adaptive_margin: cfg.adaptive_margin,
            zero_ref_pos: cfg.zero_ref_pos,
            zero_reference_mode: Cell::new(ZrefMode::Disabled),
            faulty_regions: cfg.faulty_regions,
            substituted_indices: RefCell::new(IndexMap::new()),
            bedmesh: Rc::downgrade(bedmesh),
            mesh_config: RefCell::new(cfg.mesh_config),
            points: RefCell::new(vec![]),
            profile_name: RefCell::new(Some("default".into())),
            probe_helper: RefCell::new(None),
            gcode: printer.lookup::<GCodeDispatch>("gcode")?,
        });
        bmc.generate_points(&|m| KError::config(m), "automatic")?;
        let w = Rc::downgrade(&bmc);
        let probe_helper = ProbePointsHelper::new(
            config,
            Rc::new(move |offsets, positions| match w.upgrade() {
                Some(b) => b.probe_finalize(offsets, positions),
                None => Ok(FinalizeResult::Done),
            }),
            Some(bmc.get_adjusted_points()),
            "points",
            true,
            true,
        )?;
        probe_helper.minimum_points(3)?;
        *bmc.probe_helper.borrow_mut() = Some(probe_helper);
        let b = bmc.clone();
        bmc.gcode.register_command(
            "BED_MESH_CALIBRATE",
            Some(Rc::new(move |g| b.cmd_bed_mesh_calibrate(g))),
            false,
            Some(CMD_BED_MESH_CALIBRATE_HELP),
        )?;
        Ok(bmc)
    }

    fn bedmesh(&self) -> Rc<BedMesh> {
        self.bedmesh.upgrade().expect("bed mesh")
    }

    fn probe_helper(&self) -> Rc<ProbePointsHelper> {
        self.probe_helper.borrow().clone().expect("probe helper")
    }

    fn generate_points(&self, error: &dyn Fn(String) -> KError, probe_method: &str) -> KResult<()> {
        let (x_cnt, y_cnt) = {
            let mc = self.mesh_config.borrow();
            (pi(&mc, "x_count"), pi(&mc, "y_count"))
        };
        let (mut min_x, mut min_y) = self.mesh_min.get();
        let (mut max_x, mut max_y) = self.mesh_max.get();
        let x_dist = (max_x - min_x) / (x_cnt - 1) as f64;
        let y_dist = (max_y - min_y) / (y_cnt - 1) as f64;
        // floor distances down to next hundredth
        let x_dist = (x_dist * 100.0).floor() / 100.0;
        let mut y_dist = (y_dist * 100.0).floor() / 100.0;
        if x_dist < 1.0 || y_dist < 1.0 {
            return Err(error("bed_mesh: min/max points too close together".into()));
        }

        let radius = self.radius.get();
        if radius.is_some() {
            // round bed, min/max needs to be recalculated
            y_dist = x_dist;
            let new_r = (x_cnt / 2) as f64 * x_dist;
            min_x = -new_r;
            min_y = -new_r;
            max_x = new_r;
            max_y = new_r;
        } else {
            // rectangular bed, only re-calc max_x
            max_x = min_x + x_dist * (x_cnt - 1) as f64;
        }
        let mut pos_y = min_y;
        let mut points = vec![];
        let origin = self.origin.get().unwrap_or((0.0, 0.0));
        for i in 0..y_cnt {
            for j in 0..x_cnt {
                let pos_x = if i % 2 == 0 {
                    // move in positive directon
                    min_x + j as f64 * x_dist
                } else {
                    // move in negative direction
                    max_x - j as f64 * x_dist
                };
                match radius {
                    // rectangular bed, append
                    None => points.push((pos_x, pos_y)),
                    Some(r) => {
                        // round bed, check distance from origin
                        let dist_from_origin = (pos_x * pos_x + pos_y * pos_y).sqrt();
                        if dist_from_origin <= r {
                            points.push((origin.0 + pos_x, origin.1 + pos_y));
                        }
                    }
                }
            }
            pos_y += y_dist;
        }
        *self.points.borrow_mut() = points;
        let mode = match self.zero_ref_pos {
            None => ZrefMode::Disabled,
            Some(_) if probe_method == "manual" => ZrefMode::Disabled,
            // Zero Reference position within mesh
            Some(z) if within(z, self.mesh_min.get(), self.mesh_max.get(), 0.0) => ZrefMode::InMesh,
            // Zero Reference position outside of mesh
            Some(_) => ZrefMode::Probe,
        };
        self.zero_reference_mode.set(mode);
        if self.faulty_regions.is_empty() {
            return Ok(());
        }
        self.substituted_indices.borrow_mut().clear();
        if mode == ZrefMode::Probe {
            // Cannot probe a reference within a faulty region
            let z = self.zero_ref_pos.unwrap();
            for (min_c, max_c) in &self.faulty_regions {
                if within(z, *min_c, *max_c, 0.0) {
                    let opt = "zero_reference_position";
                    return Err(error(format!(
                        "bed_mesh: Cannot probe zero reference position at ({:.2}, {:.2}) as it is located within a faulty region. Check the value for option '{}'",
                        z.0, z.1, opt
                    )));
                }
            }
        }
        // Check to see if any points fall within faulty regions
        if probe_method == "manual" {
            return Ok(());
        }
        let points = self.points.borrow().clone();
        let mut last_y = points[0].1;
        let mut is_reversed = false;
        for (i, coord) in points.iter().enumerate() {
            if !isclose_d(coord.1, last_y) {
                is_reversed = !is_reversed;
            }
            last_y = coord.1;
            let mut adj_coords = vec![];
            for (min_c, max_c) in &self.faulty_regions {
                if within(*coord, *min_c, *max_c, 0.00001) {
                    // Point lies within a faulty region
                    adj_coords = vec![(min_c.0, coord.1), (coord.0, min_c.1), (coord.0, max_c.1), (max_c.0, coord.1)];
                    if is_reversed {
                        // Swap first and last points for zig-zag pattern
                        adj_coords.swap(0, 3);
                    }
                    break;
                }
            }
            if adj_coords.is_empty() {
                // coord is not located within a faulty region
                continue;
            }
            let mut valid_coords = vec![];
            for ac in adj_coords {
                // make sure that coordinates are within the mesh boundary
                match radius {
                    None => {
                        if within(ac, (min_x, min_y), (max_x, max_y), 0.000001) {
                            valid_coords.push(ac);
                        }
                    }
                    Some(r) => {
                        let dist_from_origin = (ac.0 * ac.0 + ac.1 * ac.1).sqrt();
                        if dist_from_origin <= r {
                            valid_coords.push(ac);
                        }
                    }
                }
            }
            if valid_coords.is_empty() {
                return Err(error(format!("bed_mesh: Unable to generate coordinates for faulty region at index: {}", i)));
            }
            self.substituted_indices.borrow_mut().insert(i, valid_coords);
        }
        Ok(())
    }

    pub fn print_generated_points(&self, print_func: &dyn Fn(&str), truncate: bool) -> KResult<()> {
        let (mut x_offset, mut y_offset) = (0.0, 0.0);
        if let Some(pprobe) = probe_list_get_list(&self.printer)?.get_default_probe() {
            if let Some(p) = as_printer_probe(&pprobe) {
                let o = p.get_offsets();
                x_offset = o.0;
                y_offset = o.1;
            }
        }
        print_func("bed_mesh: generated points\nIndex |  Tool Adjusted  |   Probe");
        let points = self.points.borrow();
        for (i, (x, y)) in points.iter().enumerate() {
            if i >= 50 && truncate {
                let end = points.len() - 1;
                print_func(&format!("...points {} through {} truncated", i, end));
                break;
            }
            let adj_pt = format!("({:.1}, {:.1})", x - x_offset, y - y_offset);
            let mesh_pt = format!("({:.1}, {:.1})", x, y);
            print_func(&format!("  {:<4}| {:<16}| {}", i, adj_pt, mesh_pt));
        }
        if let Some(z) = self.zero_ref_pos {
            print_func(&format!("bed_mesh: zero_reference_position is ({:.2}, {:.2})", z.0, z.1));
        }
        let si = self.substituted_indices.borrow();
        if !si.is_empty() {
            print_func("bed_mesh: faulty region points");
            for (i, v) in si.iter() {
                let pt = points[*i];
                let r = format!("[{}]", v.iter().map(|p| tuple_repr(&[p.0, p.1])).collect::<Vec<_>>().join(", "));
                print_func(&format!("{} ({:.2}, {:.2}), substituted points: {}", i, pt.0, pt.1, r));
            }
        }
        Ok(())
    }

    /// set_adaptive_mesh(gcmd) -> the mesh changed
    fn set_adaptive_mesh(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        if gcmd.get_int("ADAPTIVE", Some(0), Lim::new())? == 0 {
            return Ok(false);
        }
        let Some(exclude_objects) = self.printer.lookup_object_opt("exclude_object") else {
            gcmd.respond_info("Exclude objects not enabled. Using full mesh...", true);
            return Ok(false);
        };
        let objects = match exclude_objects.try_get_status(0.0)? {
            Some(PyVal::Dict(d)) => d.get("objects").cloned().unwrap_or(PyVal::List(vec![])),
            _ => PyVal::List(vec![]),
        };
        let objects = objects.items().map(|o| o.to_vec()).unwrap_or_default();
        if objects.is_empty() {
            return Ok(false);
        }
        let margin = gcmd.get_float("ADAPTIVE_MARGIN", Some(self.adaptive_margin), Lim::new())?;

        // List all exclude_object points by axis and iterate over
        // all polygon points, and pick the min and max or each axis
        let mut list_of_xs = vec![];
        let mut list_of_ys = vec![];
        gcmd.respond_info(&format!("Found {} objects", objects.len()), true);
        for obj in &objects {
            let polygon = match obj {
                PyVal::Dict(d) => d.get("polygon").cloned().unwrap_or(PyVal::List(vec![])),
                _ => PyVal::List(vec![]),
            };
            for point in polygon.items().unwrap_or(&[]) {
                let p = point.items().unwrap_or(&[]);
                list_of_xs.push(p[0].as_f64().unwrap_or(0.0));
                list_of_ys.push(p[1].as_f64().unwrap_or(0.0));
            }
        }
        let fmin = |v: &[f64]| v[1..].iter().fold(v[0], |a, &b| py_min(a, b));
        let fmax = |v: &[f64]| v[1..].iter().fold(v[0], |a, &b| py_max(a, b));

        // Define bounds of adaptive mesh area
        let mesh_min = [fmin(&list_of_xs), fmin(&list_of_ys)];
        let mesh_max = [fmax(&list_of_xs), fmax(&list_of_ys)];
        let mut adjusted_mesh_min = [mesh_min[0] - margin, mesh_min[1] - margin];
        let mut adjusted_mesh_max = [mesh_max[0] + margin, mesh_max[1] + margin];

        // Force margin to respect original mesh bounds
        let (omin, omax) = (self.orig_config.mesh_min, self.orig_config.mesh_max);
        adjusted_mesh_min[0] = py_max(adjusted_mesh_min[0], omin.0);
        adjusted_mesh_min[1] = py_max(adjusted_mesh_min[1], omin.1);
        adjusted_mesh_max[0] = py_min(adjusted_mesh_max[0], omax.0);
        adjusted_mesh_max[1] = py_min(adjusted_mesh_max[1], omax.1);

        let adjusted_mesh_size = (adjusted_mesh_max[0] - adjusted_mesh_min[0], adjusted_mesh_max[1] - adjusted_mesh_min[1]);

        // Compute a ratio between the adapted and original sizes
        let ratio = (adjusted_mesh_size.0 / (omax.0 - omin.0), adjusted_mesh_size.1 / (omax.1 - omin.1));

        gcmd.respond_info(&format!("Original mesh bounds: ({},{})", tuple_repr(&[omin.0, omin.1]), tuple_repr(&[omax.0, omax.1])), true);
        let (xc, yc) = {
            let mc = self.mesh_config.borrow();
            (pi(&mc, "x_count"), pi(&mc, "y_count"))
        };
        gcmd.respond_info(&format!("Original probe count: ({},{})", xc, yc), true);
        gcmd.respond_info(&format!("Adapted mesh bounds: ({},{})", list_repr(&adjusted_mesh_min), list_repr(&adjusted_mesh_max)), true);
        gcmd.respond_info(&format!("Ratio: ({}, {})", py_float_repr(ratio.0), py_float_repr(ratio.1)), true);

        let mut new_x_probe_count = (xc as f64 * ratio.0).ceil() as i64;
        let mut new_y_probe_count = (yc as f64 * ratio.1).ceil() as i64;

        // There is one case, where we may have to adjust the probe counts:
        // axis0 < 4 and axis1 > 6 (see _verify_algorithm).
        let mut min_num_of_probes = 3;
        if new_x_probe_count.max(new_y_probe_count) > 6 && new_x_probe_count.min(new_y_probe_count) < 4 {
            min_num_of_probes = 4;
        }

        new_x_probe_count = new_x_probe_count.max(min_num_of_probes);
        new_y_probe_count = new_y_probe_count.max(min_num_of_probes);

        gcmd.respond_info(&format!("Adapted probe count: ({},{})", new_x_probe_count, new_y_probe_count), true);

        // If the adapted mesh size is too small, adjust it to something
        // useful.
        let adjusted_mesh_size =
            (py_max(adjusted_mesh_size.0, new_x_probe_count as f64), py_max(adjusted_mesh_size.1, new_y_probe_count as f64));

        if let Some(radius) = self.radius.get() {
            let adapted_radius = (adjusted_mesh_size.0.powf(2.0) + adjusted_mesh_size.1.powf(2.0)).sqrt() / 2.0;
            let adapted_origin =
                (adjusted_mesh_min[0] + (adjusted_mesh_size.0 / 2.0), adjusted_mesh_min[1] + (adjusted_mesh_size.1 / 2.0));
            let to_adapted_origin = (adapted_origin.0.powf(2.0) + adapted_origin.1.powf(2.0)).sqrt();
            // If the adapted mesh size is smaller than the default/full
            // mesh, adjust the parameters. Otherwise, just do the full mesh.
            if adapted_radius + to_adapted_origin < radius {
                self.radius.set(Some(adapted_radius));
                self.origin.set(Some(adapted_origin));
                self.mesh_min.set((-adapted_radius, -adapted_radius));
                self.mesh_max.set((adapted_radius, adapted_radius));
                let mut new_probe_count = new_x_probe_count.max(new_y_probe_count);
                // Adaptive meshes require odd number of points
                new_probe_count += 1 - (new_probe_count % 2);
                let mut mc = self.mesh_config.borrow_mut();
                mc.insert("x_count".into(), PyVal::Int(new_probe_count));
                mc.insert("y_count".into(), PyVal::Int(new_probe_count));
            }
        } else {
            self.mesh_min.set((adjusted_mesh_min[0], adjusted_mesh_min[1]));
            self.mesh_max.set((adjusted_mesh_max[0], adjusted_mesh_max[1]));
            let mut mc = self.mesh_config.borrow_mut();
            mc.insert("x_count".into(), PyVal::Int(new_x_probe_count));
            mc.insert("y_count".into(), PyVal::Int(new_y_probe_count));
        }
        *self.profile_name.borrow_mut() = None;
        Ok(true)
    }

    fn update_config(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // reset default configuration
        self.radius.set(self.orig_config.radius);
        self.origin.set(self.orig_config.origin);
        self.mesh_min.set(self.orig_config.mesh_min);
        self.mesh_max.set(self.orig_config.mesh_max);
        {
            let mut mc = self.mesh_config.borrow_mut();
            let keys: Vec<String> = mc.keys().cloned().collect();
            for key in keys {
                let v = self.orig_config.mesh_config.get(&key).cloned().unwrap_or(PyVal::None);
                mc.insert(key, v);
            }
        }

        let params = gcmd.get_command_parameters();
        let mut need_cfg_update = false;
        if self.radius.get().is_some() {
            if params.contains_key("MESH_RADIUS") {
                let r = gcmd.get_float("MESH_RADIUS", None, Lim::new())?;
                let r = (r * 10.0).floor() / 10.0;
                self.radius.set(Some(r));
                self.mesh_min.set((-r, -r));
                self.mesh_max.set((r, r));
                need_cfg_update = true;
            }
            if params.contains_key("MESH_ORIGIN") {
                self.origin.set(Some(parse_gcmd_coord(gcmd, "MESH_ORIGIN")?));
                need_cfg_update = true;
            }
            if params.contains_key("ROUND_PROBE_COUNT") {
                let cnt = gcmd.get_int("ROUND_PROBE_COUNT", None, Lim::new().min(3))?;
                let mut mc = self.mesh_config.borrow_mut();
                mc.insert("x_count".into(), PyVal::Int(cnt));
                mc.insert("y_count".into(), PyVal::Int(cnt));
                need_cfg_update = true;
            }
        } else {
            if params.contains_key("MESH_MIN") {
                self.mesh_min.set(parse_gcmd_coord(gcmd, "MESH_MIN")?);
                need_cfg_update = true;
            }
            if params.contains_key("MESH_MAX") {
                self.mesh_max.set(parse_gcmd_coord(gcmd, "MESH_MAX")?);
                need_cfg_update = true;
            }
            if params.contains_key("PROBE_COUNT") {
                let (x_cnt, y_cnt) = parse_gcmd_pair(gcmd, "PROBE_COUNT", Some(3), None)?;
                let mut mc = self.mesh_config.borrow_mut();
                mc.insert("x_count".into(), PyVal::Int(x_cnt));
                mc.insert("y_count".into(), PyVal::Int(y_cnt));
                need_cfg_update = true;
            }
        }

        if params.contains_key("ALGORITHM") {
            let algo = py_strip(&gcmd.get("ALGORITHM")?).to_lowercase();
            self.mesh_config.borrow_mut().insert("algo".into(), PyVal::Str(algo));
            need_cfg_update = true;
        }

        need_cfg_update |= self.set_adaptive_mesh(gcmd)?;
        let probe_method = gcmd.get_or("METHOD", "automatic");

        verify_algorithm(&mut self.mesh_config.borrow_mut(), &|m| gcmd.error(m))?;
        self.generate_points(&|m| gcmd.error(m), &probe_method)?;
        let pts = self.get_adjusted_points();
        self.probe_helper().update_probe_points(pts, 3)?;
        if need_cfg_update {
            let msg: Vec<String> = self.mesh_config.borrow().iter().map(|(k, v)| format!("{}: {}", k, v.str())).collect();
            logging::info(&format!("Updated Mesh Configuration:\n{}", msg.join("\n")));
        }
        Ok(())
    }

    fn get_adjusted_points(&self) -> Vec<(f64, f64)> {
        let points = self.points.borrow();
        let si = self.substituted_indices.borrow();
        let mut adj_pts = vec![];
        if !si.is_empty() {
            let mut last_index = 0;
            for (i, pts) in si.iter() {
                if last_index < *i {
                    adj_pts.extend_from_slice(&points[last_index..*i]);
                }
                adj_pts.extend_from_slice(pts);
                // Add one to the last index to skip the point
                // we are replacing
                last_index = i + 1;
            }
            if last_index < points.len() {
                adj_pts.extend_from_slice(&points[last_index..]);
            }
        } else {
            adj_pts = points.clone();
        }
        if self.zero_reference_mode.get() == ZrefMode::Probe {
            adj_pts.push(self.zero_ref_pos.unwrap());
        }
        adj_pts
    }

    fn cmd_bed_mesh_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let name = gcmd.get_or("PROFILE", "default");
        if py_strip(&name).is_empty() {
            return Err(gcmd.error("Value for parameter 'PROFILE' must be specified"));
        }
        *self.profile_name.borrow_mut() = Some(name);
        self.bedmesh().set_mesh(None)?;
        self.update_config(gcmd)?;
        self.probe_helper().start_probe(gcmd)
    }

    fn probe_finalize(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        let (x_offset, y_offset, mut z_offset) = offsets;
        let mut positions: Vec<Vec<f64>> = positions.iter().map(|p| vec![py_round(p[0], 2), py_round(p[1], 2), p[2]]).collect();
        if self.zero_reference_mode.get() == ZrefMode::Probe {
            let ref_pos = positions.pop().ok_or_else(|| KError::Other("pop from empty list".into()))?;
            logging::info(&format!(
                "bed_mesh: z-offset replaced with probed z value at position ({:.2}, {:.2}, {:.6})",
                ref_pos[0], ref_pos[1], ref_pos[2]
            ));
            z_offset = ref_pos[2];
        }
        let mut params = self.mesh_config.borrow().clone();
        // min(positions, key=...) / max(...): the first extreme element
        let pick = |idx: usize, want_min: bool| -> f64 {
            let mut best = positions[0][idx];
            for p in &positions[1..] {
                if (want_min && p[idx] < best) || (!want_min && p[idx] > best) {
                    best = p[idx];
                }
            }
            best
        };
        params.insert("min_x".into(), PyVal::Float(pick(0, true) + x_offset));
        params.insert("max_x".into(), PyVal::Float(pick(0, false) + x_offset));
        params.insert("min_y".into(), PyVal::Float(pick(1, true) + y_offset));
        params.insert("max_y".into(), PyVal::Float(pick(1, false) + y_offset));
        let x_cnt = pi(&params, "x_count") as usize;
        let y_cnt = pi(&params, "y_count") as usize;

        let si = self.substituted_indices.borrow().clone();
        let gen_points = self.points.borrow().clone();
        if !si.is_empty() {
            // Replace substituted points with the original generated
            // point.  Its Z Value is the average probed Z of the
            // substituted points.
            let mut corrected_pts: Vec<Vec<f64>> = vec![];
            let mut idx_offset = 0;
            let mut start_idx = 0;
            for (i, pts) in si.iter() {
                let mut fpt = vec![gen_points[*i].0 - offsets.0, gen_points[*i].1 - offsets.1];
                // offset the index to account for additional samples
                let idx = i + idx_offset;
                // Add "normal" points
                if start_idx < idx {
                    corrected_pts.extend_from_slice(&positions[start_idx..idx.min(positions.len())]);
                }
                let sl = &positions[idx.min(positions.len())..(idx + pts.len()).min(positions.len())];
                let avg_z = sl.iter().fold(0.0, |a, p| a + p[2]) / pts.len() as f64;
                idx_offset += pts.len() - 1;
                start_idx = idx + pts.len();
                fpt.push(avg_z);
                logging::info(&format!(
                    "bed_mesh: Replacing value at faulty index {} ({:.4}, {:.4}): avg value = {:.6}, avg w/ z_offset = {:.6}",
                    i,
                    fpt[0],
                    fpt[1],
                    avg_z,
                    avg_z - z_offset
                ));
                corrected_pts.push(fpt);
            }
            if start_idx < positions.len() {
                corrected_pts.extend_from_slice(&positions[start_idx..]);
            }
            // validate corrected positions
            if gen_points.len() != corrected_pts.len() {
                self.dump_points(&positions, &corrected_pts, offsets);
                return Err(KError::command(format!(
                    "bed_mesh: invalid position list size, generated count: {}, probed count: {}",
                    gen_points.len(),
                    corrected_pts.len()
                )));
            }
            for (gen_pt, probed) in gen_points.iter().zip(&corrected_pts) {
                let off_pt = [gen_pt.0 - offsets.0, gen_pt.1 - offsets.1];
                if !isclose(off_pt[0], probed[0], 1e-09, 0.1) || !isclose(off_pt[1], probed[1], 1e-09, 0.1) {
                    self.dump_points(&positions, &corrected_pts, offsets);
                    return Err(KError::command(format!(
                        "bed_mesh: point mismatch, orig = ({:.2}, {:.2}), probed = ({:.2}, {:.2})",
                        off_pt[0], off_pt[1], probed[0], probed[1]
                    )));
                }
            }
            positions = corrected_pts;
        }

        let mut probed_matrix: Vec<Vec<f64>> = vec![];
        let mut row: Vec<f64> = vec![];
        let mut prev_pos = positions[0].clone();
        for pos in &positions {
            if !isclose(pos[1], prev_pos[1], 1e-09, 0.1) {
                // y has changed, append row and start new
                probed_matrix.push(std::mem::take(&mut row));
            }
            if pos[0] > prev_pos[0] {
                // probed in the positive direction
                row.push(pos[2] - z_offset);
            } else {
                // probed in the negative direction
                row.insert(0, pos[2] - z_offset);
            }
            prev_pos = pos.clone();
        }
        // append last row
        probed_matrix.push(row);

        // make sure the y-axis is the correct length
        if probed_matrix.len() != y_cnt {
            return Err(KError::command(format!(
                "bed_mesh: Invalid y-axis table length\nProbed table length: {} Probed Table:\n{}",
                probed_matrix.len(),
                matrix_repr(&probed_matrix)
            )));
        }

        if self.radius.get().is_some() {
            // round bed, extrapolate probed values to create a square mesh
            for ri in 0..probed_matrix.len() {
                let row_size = probed_matrix[ri].len();
                if row_size & 1 == 0 {
                    // an even number of points in a row shouldn't be possible
                    let mut msg = "bed_mesh: incorrect number of points sampled on X\n".to_string();
                    msg += "Probed Table:\n";
                    msg += &matrix_repr(&probed_matrix);
                    return Err(KError::command(msg));
                }
                let buf_cnt = x_cnt.saturating_sub(row_size) / 2;
                if buf_cnt == 0 {
                    continue;
                }
                let row = &mut probed_matrix[ri];
                let (first, last) = (row[0], row[row_size - 1]);
                let mut new_row = vec![first; buf_cnt];
                new_row.extend_from_slice(row);
                new_row.extend(std::iter::repeat(last).take(buf_cnt));
                *row = new_row;
            }
        }

        //  make sure that the x-axis is the correct length
        for row in &probed_matrix {
            if row.len() != x_cnt {
                return Err(KError::command(format!(
                    "bed_mesh: invalid x-axis table length\nProbed table length: {} Probed Table:\n{}",
                    probed_matrix.len(),
                    matrix_repr(&probed_matrix)
                )));
            }
        }

        let profile_name = self.profile_name.borrow().clone();
        let z_mesh = Rc::new(ZMesh::new(params, profile_name.as_deref()));
        z_mesh.build_mesh(probed_matrix).map_err(KError::command)?;
        if self.zero_reference_mode.get() == ZrefMode::InMesh {
            // The reference can be anywhere in the mesh, therefore
            // it is necessary to set the reference after the initial mesh
            // is generated to lookup the correct z value.
            let z = self.zero_ref_pos.unwrap();
            z_mesh.set_zero_reference(z.0, z.1);
        }
        self.bedmesh().set_mesh(Some(z_mesh))?;
        self.gcode.respond_info("Mesh Bed Leveling Complete", true);
        if let Some(name) = profile_name {
            self.bedmesh().pmgr().save_profile(&name)?;
        }
        Ok(FinalizeResult::Done)
    }

    fn dump_points(&self, probed_pts: &[Vec<f64>], corrected_pts: &[Vec<f64>], offsets: (f64, f64, f64)) {
        // logs generated points with offset applied, points received
        // from the finalize callback, and the list of corrected points
        let points = self.points.borrow();
        let max_len = points.len().max(probed_pts.len()).max(corrected_pts.len());
        logging::info(&format!(
            "bed_mesh: calibration point dump\nIndex | {:<17}| {:<25}| Corrected Point",
            "Generated Point", "Probed Point"
        ));
        for i in 0..max_len {
            let (mut gen_pt, mut probed_pt, mut corr_pt) = (String::new(), String::new(), String::new());
            if i < points.len() {
                gen_pt = format!("({:.2}, {:.2})", points[i].0 - offsets.0, points[i].1 - offsets.1);
            }
            if i < probed_pts.len() {
                let p = &probed_pts[i];
                probed_pt = format!("({:.2}, {:.2}, {:.4})", p[0], p[1], p[2]);
            }
            if i < corrected_pts.len() {
                let p = &corrected_pts[i];
                corr_pt = format!("({:.2}, {:.2}, {:.4})", p[0], p[1], p[2]);
            }
            logging::info(&format!("  {:<4}| {:<17}| {:<25}| {}", i, gen_pt, probed_pt, corr_pt));
        }
    }
}

// ----------------------------------------------------------------------
// MoveSplitter
// ----------------------------------------------------------------------

pub struct MoveSplitter {
    split_delta_z: f64,
    move_check_distance: f64,
    z_mesh: RefCell<Option<Rc<ZMesh>>>,
    fade_offset: Cell<f64>,
    prev_pos: RefCell<Vec<f64>>,
    next_pos: RefCell<Vec<f64>>,
    current_pos: RefCell<Vec<f64>>,
    z_factor: Cell<f64>,
    z_offset: Cell<f64>,
    pub traverse_complete: Cell<bool>,
    distance_checked: Cell<f64>,
    total_move_length: Cell<f64>,
    axis_move: RefCell<Vec<bool>>,
}

impl MoveSplitter {
    fn new(config: &ConfigWrapper, _gcode: &Rc<GCodeDispatch>) -> KResult<Self> {
        let split_delta_z = config.getfloat("split_delta_z", 0.025, Lim::new().min(0.01))?;
        let move_check_distance = config.getfloat("move_check_distance", 5.0, Lim::new().min(3.0))?;
        Ok(MoveSplitter {
            split_delta_z,
            move_check_distance,
            z_mesh: RefCell::new(None),
            fade_offset: Cell::new(0.0),
            prev_pos: RefCell::new(vec![]),
            next_pos: RefCell::new(vec![]),
            current_pos: RefCell::new(vec![]),
            z_factor: Cell::new(0.0),
            z_offset: Cell::new(0.0),
            traverse_complete: Cell::new(false),
            distance_checked: Cell::new(0.0),
            total_move_length: Cell::new(0.0),
            axis_move: RefCell::new(vec![]),
        })
    }

    fn initialize(&self, mesh: Option<Rc<ZMesh>>, fade_offset: f64) {
        *self.z_mesh.borrow_mut() = mesh;
        self.fade_offset.set(fade_offset);
    }

    fn build_move(&self, prev_pos: &[f64], next_pos: &[f64], factor: f64) -> KResult<()> {
        *self.prev_pos.borrow_mut() = prev_pos.to_vec();
        *self.next_pos.borrow_mut() = next_pos.to_vec();
        *self.current_pos.borrow_mut() = prev_pos.to_vec();
        self.z_factor.set(factor);
        self.z_offset.set(self.calc_z_offset(prev_pos)?);
        self.traverse_complete.set(false);
        self.distance_checked.set(0.0);
        let axes_d: Vec<f64> = next_pos.iter().zip(prev_pos).map(|(np, pp)| np - pp).collect();
        self.total_move_length.set(axes_d[..3].iter().fold(0.0, |a, d| a + d * d).sqrt());
        *self.axis_move.borrow_mut() = axes_d.iter().map(|d| !isclose(*d, 0.0, 1e-09, 1e-10)).collect();
        Ok(())
    }

    fn calc_z_offset(&self, pos: &[f64]) -> KResult<f64> {
        let z_mesh = self.z_mesh.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'calc_z'".into()))?;
        let z = z_mesh.calc_z(pos[0], pos[1]);
        let offset = self.fade_offset.get();
        Ok(self.z_factor.get() * (z - offset) + offset)
    }

    fn set_next_move(&self, distance_from_prev: f64) -> KResult<()> {
        let t = distance_from_prev / self.total_move_length.get();
        if t > 1.0 || t < 0.0 {
            return Err(KError::command("bed_mesh: Slice distance is negative or greater than entire move length"));
        }
        let (prev, next, axis_move) = (self.prev_pos.borrow(), self.next_pos.borrow(), self.axis_move.borrow());
        let mut cur = self.current_pos.borrow_mut();
        for i in 0..next.len() {
            if axis_move[i] {
                cur[i] = lerp(t, prev[i], next[i]);
            }
        }
        Ok(())
    }

    fn split(&self) -> KResult<Option<Vec<f64>>> {
        if self.traverse_complete.get() {
            // Traverse complete
            return Ok(None);
        }
        let (ax, ay) = {
            let am = self.axis_move.borrow();
            (am[0], am[1])
        };
        if ax || ay {
            // X and/or Y axis move, traverse if necessary
            while self.distance_checked.get() + self.move_check_distance < self.total_move_length.get() {
                self.distance_checked.set(self.distance_checked.get() + self.move_check_distance);
                self.set_next_move(self.distance_checked.get())?;
                let cur = self.current_pos.borrow().clone();
                let next_z = self.calc_z_offset(&cur)?;
                if (next_z - self.z_offset.get()).abs() >= self.split_delta_z {
                    self.z_offset.set(next_z);
                    let mut newpos = cur;
                    newpos[2] += self.z_offset.get();
                    return Ok(Some(newpos));
                }
            }
        }
        // end of move reached
        let next = self.next_pos.borrow().clone();
        *self.current_pos.borrow_mut() = next.clone();
        self.z_offset.set(self.calc_z_offset(&next)?);
        // Its okay to add Z-Offset to the final move, since it will not be
        // used again.
        let mut cur = self.current_pos.borrow_mut();
        cur[2] += self.z_offset.get();
        self.traverse_complete.set(true);
        Ok(Some(cur.clone()))
    }
}

// ----------------------------------------------------------------------
// ZMesh
// ----------------------------------------------------------------------

pub struct ZMesh {
    profile_name: String,
    probed_matrix: RefCell<Option<Vec<Vec<f64>>>>,
    mesh_matrix: RefCell<Option<Vec<Vec<f64>>>>,
    mesh_params: MeshParams,
    mesh_offsets: RefCell<[f64; 2]>,
    mesh_x_min: f64,
    mesh_y_min: f64,
    algo: String,
    mesh_x_count: usize,
    mesh_y_count: usize,
    x_mult: usize,
    y_mult: usize,
    mesh_x_dist: f64,
    mesh_y_dist: f64,
}

static ZMESH_ID: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0x7f0000000000);

impl ZMesh {
    pub fn new(params: MeshParams, name: Option<&str>) -> Self {
        let profile_name = match name {
            Some(n) if !n.is_empty() => n.to_string(),
            // "adaptive-%X" % id(self): an address in python
            _ => format!("adaptive-{:X}", ZMESH_ID.fetch_add(0x40, std::sync::atomic::Ordering::Relaxed)),
        };
        logging::debug("bed_mesh: probe/mesh parameters:");
        for (key, value) in params.iter() {
            logging::debug(&format!("{} :  {}", key, value.str()));
        }
        let mesh_x_min = pf(&params, "min_x");
        let mesh_x_max = pf(&params, "max_x");
        let mesh_y_min = pf(&params, "min_y");
        let mesh_y_max = pf(&params, "max_y");
        logging::debug(&format!(
            "bed_mesh: Mesh Min: ({:.2},{:.2}) Mesh Max: ({:.2},{:.2})",
            mesh_x_min, mesh_y_min, mesh_x_max, mesh_y_max
        ));
        // Set the interpolation algorithm
        let algo = ps(&params, "algo");
        // Number of points to interpolate per segment
        let mesh_x_pps = pi(&params, "mesh_x_pps") as usize;
        let mesh_y_pps = pi(&params, "mesh_y_pps") as usize;
        let px_cnt = pi(&params, "x_count") as usize;
        let py_cnt = pi(&params, "y_count") as usize;
        let mesh_x_count = (px_cnt - 1) * mesh_x_pps + px_cnt;
        let mesh_y_count = (py_cnt - 1) * mesh_y_pps + py_cnt;
        logging::debug(&format!("bed_mesh: Mesh grid size - X:{}, Y:{}", mesh_x_count, mesh_y_count));
        let mesh_x_dist = (mesh_x_max - mesh_x_min) / (mesh_x_count - 1) as f64;
        let mesh_y_dist = (mesh_y_max - mesh_y_min) / (mesh_y_count - 1) as f64;
        ZMesh {
            profile_name,
            probed_matrix: RefCell::new(None),
            mesh_matrix: RefCell::new(None),
            mesh_params: params,
            mesh_offsets: RefCell::new([0.0, 0.0]),
            mesh_x_min,
            mesh_y_min,
            algo,
            mesh_x_count,
            mesh_y_count,
            x_mult: mesh_x_pps + 1,
            y_mult: mesh_y_pps + 1,
            mesh_x_dist,
            mesh_y_dist,
        }
    }

    fn rounded(m: &Option<Vec<Vec<f64>>>) -> Vec<Vec<f64>> {
        match m {
            Some(m) => m.iter().map(|line| line.iter().map(|z| py_round(*z, 6)).collect()).collect(),
            None => vec![vec![]],
        }
    }

    pub fn get_mesh_matrix(&self) -> Vec<Vec<f64>> {
        Self::rounded(&self.mesh_matrix.borrow())
    }

    pub fn get_probed_matrix(&self) -> Vec<Vec<f64>> {
        Self::rounded(&self.probed_matrix.borrow())
    }

    pub fn get_mesh_params(&self) -> MeshParams {
        self.mesh_params.clone()
    }

    pub fn get_profile_name(&self) -> String {
        self.profile_name.clone()
    }

    pub fn print_probed_matrix(&self, print_func: &dyn Fn(&str)) {
        match self.probed_matrix.borrow().as_ref() {
            Some(pm) => {
                let mut msg = "Mesh Leveling Probed Z positions:\n".to_string();
                for line in pm {
                    for x in line {
                        msg += &format!(" {:.6}", x);
                    }
                    msg += "\n";
                }
                print_func(&msg);
            }
            None => print_func("bed_mesh: bed has not been probed"),
        }
    }

    pub fn print_mesh(&self, print_func: &dyn Fn(&str), move_z: Option<f64>) {
        let matrix = self.get_mesh_matrix();
        let mut msg = format!("Mesh X,Y: {},{}\n", self.mesh_x_count, self.mesh_y_count);
        if let Some(move_z) = move_z {
            msg += &format!("Search Height: {}\n", move_z.trunc() as i64);
        }
        let off = *self.mesh_offsets.borrow();
        msg += &format!("Mesh Offsets: X={:.4}, Y={:.4}\n", off[0], off[1]);
        msg += &format!("Mesh Average: {:.2}\n", self.get_z_average());
        let rng = self.get_z_range();
        msg += &format!("Mesh Range: min={:.4} max={:.4}\n", rng.0, rng.1);
        msg += &format!("Interpolation Algorithm: {}\n", ps(&self.mesh_params, "algo"));
        msg += "Measured points:\n";
        for y_line in (0..self.mesh_y_count).rev() {
            for z in matrix.get(y_line).map(|v| v.as_slice()).unwrap_or(&[]) {
                msg += &format!("  {:.6}", z);
            }
            msg += "\n";
        }
        print_func(&msg);
    }

    /// build_mesh(z_matrix); Err is the BedMeshError message
    pub fn build_mesh(&self, z_matrix: Vec<Vec<f64>>) -> Result<(), String> {
        *self.probed_matrix.borrow_mut() = Some(z_matrix.clone());
        match self.algo.as_str() {
            "lagrange" => self.sample_lagrange(&z_matrix),
            "bicubic" => self.sample_bicubic(&z_matrix)?,
            "direct" => *self.mesh_matrix.borrow_mut() = Some(z_matrix.clone()),
            // interpolation_algos.get() is None: calling it is a TypeError
            _ => return Err("'NoneType' object is not callable".into()),
        }
        self.print_mesh(&|m: &str| logging::debug(m), None);
        Ok(())
    }

    pub fn set_zero_reference(&self, xpos: f64, ypos: f64) {
        let offset = self.calc_z(xpos, ypos);
        logging::info(&format!("bed_mesh: setting zero reference at ({:.2}, {:.2}, {:.6})", xpos, ypos, offset));
        for matrix in [&self.probed_matrix, &self.mesh_matrix] {
            if let Some(m) = matrix.borrow_mut().as_mut() {
                for row in m.iter_mut() {
                    for v in row.iter_mut() {
                        *v -= offset;
                    }
                }
            }
        }
    }

    pub fn set_mesh_offsets(&self, offsets: &[Option<f64>]) {
        let mut mo = self.mesh_offsets.borrow_mut();
        for (i, o) in offsets.iter().enumerate() {
            if let Some(o) = o {
                mo[i] = *o;
            }
        }
    }

    pub fn get_x_coordinate(&self, index: usize) -> f64 {
        self.mesh_x_min + self.mesh_x_dist * index as f64
    }

    pub fn get_y_coordinate(&self, index: usize) -> f64 {
        self.mesh_y_min + self.mesh_y_dist * index as f64
    }

    pub fn calc_z(&self, x: f64, y: f64) -> f64 {
        match self.mesh_matrix.borrow().as_ref() {
            Some(tbl) => {
                let off = *self.mesh_offsets.borrow();
                let (tx, xidx) = self.get_linear_index(x + off[0], 0);
                let (ty, yidx) = self.get_linear_index(y + off[1], 1);
                let z0 = lerp(tx, tbl[yidx][xidx], tbl[yidx][xidx + 1]);
                let z1 = lerp(tx, tbl[yidx + 1][xidx], tbl[yidx + 1][xidx + 1]);
                lerp(ty, z0, z1)
            }
            // No mesh table generated, no z-adjustment
            None => 0.0,
        }
    }

    pub fn get_z_range(&self) -> (f64, f64) {
        match self.mesh_matrix.borrow().as_ref() {
            Some(m) => {
                let fmin = |v: &[f64]| v[1..].iter().fold(v[0], |a, &b| py_min(a, b));
                let fmax = |v: &[f64]| v[1..].iter().fold(v[0], |a, &b| py_max(a, b));
                let mins: Vec<f64> = m.iter().map(|r| fmin(r)).collect();
                let maxs: Vec<f64> = m.iter().map(|r| fmax(r)).collect();
                (fmin(&mins), fmax(&maxs))
            }
            None => (0.0, 0.0),
        }
    }

    pub fn get_z_average(&self) -> f64 {
        match self.mesh_matrix.borrow().as_ref() {
            Some(m) => {
                let total = m.iter().map(|r| r.iter().fold(0.0, |a, b| a + b)).fold(0.0, |a, b| a + b);
                let count: usize = m.iter().map(|r| r.len()).sum();
                let avg_z = total / count as f64;
                // Round average to the nearest 100th.  This
                // should produce an offset that is divisible by common
                // z step distances
                py_round(avg_z, 2)
            }
            None => 0.0,
        }
    }

    fn get_linear_index(&self, coord: f64, axis: usize) -> (f64, usize) {
        let (mesh_min, mesh_cnt, mesh_dist) = if axis == 0 {
            // X-axis
            (self.mesh_x_min, self.mesh_x_count, self.mesh_x_dist)
        } else {
            // Y-axis
            (self.mesh_y_min, self.mesh_y_count, self.mesh_y_dist)
        };
        let idx = ((coord - mesh_min) / mesh_dist).floor() as i64;
        let idx = idx.max(0).min(mesh_cnt as i64 - 2) as usize;
        let c = if axis == 0 { self.get_x_coordinate(idx) } else { self.get_y_coordinate(idx) };
        let t = (coord - c) / mesh_dist;
        (constrain(t, 0.0, 1.0), idx)
    }

    fn initial_matrix(&self, z_matrix: &[Vec<f64>]) -> Vec<Vec<f64>> {
        let (x_mult, y_mult) = (self.x_mult, self.y_mult);
        (0..self.mesh_y_count)
            .map(|j| {
                (0..self.mesh_x_count)
                    .map(|i| if i % x_mult != 0 || j % y_mult != 0 { 0.0 } else { z_matrix[j / y_mult][i / x_mult] })
                    .collect()
            })
            .collect()
    }

    fn sample_lagrange(&self, z_matrix: &[Vec<f64>]) {
        let (x_mult, y_mult) = (self.x_mult, self.y_mult);
        let mut mm = self.initial_matrix(z_matrix);
        let (xpts, ypts) = self.get_lagrange_coords();
        // Interpolate X coordinates
        for i in 0..self.mesh_y_count {
            // only interpolate X-rows that have probed coordinates
            if i % y_mult != 0 {
                continue;
            }
            for j in 0..self.mesh_x_count {
                if j % x_mult == 0 {
                    continue;
                }
                let x = self.get_x_coordinate(j);
                mm[i][j] = self.calc_lagrange(&mm, &xpts, x, i, 0);
            }
        }
        // Interpolate Y coordinates
        for i in 0..self.mesh_x_count {
            for j in 0..self.mesh_y_count {
                if j % y_mult == 0 {
                    continue;
                }
                let y = self.get_y_coordinate(j);
                mm[j][i] = self.calc_lagrange(&mm, &ypts, y, i, 1);
            }
        }
        *self.mesh_matrix.borrow_mut() = Some(mm);
    }

    fn get_lagrange_coords(&self) -> (Vec<f64>, Vec<f64>) {
        let xpts = (0..pi(&self.mesh_params, "x_count") as usize).map(|i| self.get_x_coordinate(i * self.x_mult)).collect();
        let ypts = (0..pi(&self.mesh_params, "y_count") as usize).map(|j| self.get_y_coordinate(j * self.y_mult)).collect();
        (xpts, ypts)
    }

    fn calc_lagrange(&self, mm: &[Vec<f64>], lpts: &[f64], c: f64, vec: usize, axis: usize) -> f64 {
        let pt_cnt = lpts.len();
        let mut total = 0.0;
        for i in 0..pt_cnt {
            let mut n = 1.0;
            let mut d = 1.0;
            for j in 0..pt_cnt {
                if j == i {
                    continue;
                }
                n *= c - lpts[j];
                d *= lpts[i] - lpts[j];
            }
            let z = if axis == 0 {
                // Calc X-Axis
                mm[vec][i * self.x_mult]
            } else {
                // Calc Y-Axis
                mm[i * self.y_mult][vec]
            };
            total += z * n / d;
        }
        total
    }

    fn sample_bicubic(&self, z_matrix: &[Vec<f64>]) -> Result<(), String> {
        // should work for any number of probe points above 3x3
        let (x_mult, y_mult) = (self.x_mult, self.y_mult);
        let c = pf(&self.mesh_params, "tension");
        let mut mm = self.initial_matrix(z_matrix);
        // Interpolate X values
        for y in 0..self.mesh_y_count {
            if y % y_mult != 0 {
                continue;
            }
            for x in 0..self.mesh_x_count {
                if x % x_mult == 0 {
                    continue;
                }
                let pts = self.get_x_ctl_pts(&mm, x, y)?;
                mm[y][x] = Self::cardinal_spline(pts, c);
            }
        }
        // Interpolate Y values
        for x in 0..self.mesh_x_count {
            for y in 0..self.mesh_y_count {
                if y % y_mult == 0 {
                    continue;
                }
                let pts = self.get_y_ctl_pts(&mm, x, y)?;
                mm[y][x] = Self::cardinal_spline(pts, c);
            }
        }
        *self.mesh_matrix.borrow_mut() = Some(mm);
        Ok(())
    }

    fn get_x_ctl_pts(&self, mm: &[Vec<f64>], x: usize, y: usize) -> Result<[f64; 5], String> {
        // Fetch control points and t for a X value in the mesh
        let x_mult = self.x_mult;
        let x_row = &mm[y];
        let last_pt = self.mesh_x_count as i64 - 1 - x_mult as i64;
        let xi = x as i64;
        if x < x_mult {
            Ok([x_row[0], x_row[0], x_row[x_mult], x_row[2 * x_mult], x as f64 / x_mult as f64])
        } else if xi > last_pt {
            let lp = last_pt as usize;
            Ok([x_row[lp - x_mult], x_row[lp], x_row[lp + x_mult], x_row[lp + x_mult], (xi - last_pt) as f64 / x_mult as f64])
        } else {
            let mut i = x_mult as i64;
            while i < last_pt {
                if xi > i && xi < i + x_mult as i64 {
                    let iu = i as usize;
                    return Ok([
                        x_row[iu - x_mult],
                        x_row[iu],
                        x_row[iu + x_mult],
                        x_row[iu + 2 * x_mult],
                        (xi - i) as f64 / x_mult as f64,
                    ]);
                }
                i += x_mult as i64;
            }
            Err("bed_mesh: Error finding x control points".into())
        }
    }

    fn get_y_ctl_pts(&self, mm: &[Vec<f64>], x: usize, y: usize) -> Result<[f64; 5], String> {
        // Fetch control points and t for a Y value in the mesh
        let y_mult = self.y_mult;
        let last_pt = self.mesh_y_count as i64 - 1 - y_mult as i64;
        let y_col = mm;
        let yi = y as i64;
        if y < y_mult {
            Ok([y_col[0][x], y_col[0][x], y_col[y_mult][x], y_col[2 * y_mult][x], y as f64 / y_mult as f64])
        } else if yi > last_pt {
            let lp = last_pt as usize;
            Ok([y_col[lp - y_mult][x], y_col[lp][x], y_col[lp + y_mult][x], y_col[lp + y_mult][x], (yi - last_pt) as f64 / y_mult as f64])
        } else {
            let mut i = y_mult as i64;
            while i < last_pt {
                if yi > i && yi < i + y_mult as i64 {
                    let iu = i as usize;
                    return Ok([
                        y_col[iu - y_mult][x],
                        y_col[iu][x],
                        y_col[iu + y_mult][x],
                        y_col[iu + 2 * y_mult][x],
                        (yi - i) as f64 / y_mult as f64,
                    ]);
                }
                i += y_mult as i64;
            }
            Err("bed_mesh: Error finding y control points".into())
        }
    }

    fn cardinal_spline(p: [f64; 5], tension: f64) -> f64 {
        let t = p[4];
        let t2 = t * t;
        let t3 = t2 * t;
        let m1 = tension * (p[2] - p[0]);
        let m2 = tension * (p[3] - p[1]);
        let a = p[1] * (2.0 * t3 - 3.0 * t2 + 1.0);
        let b = p[2] * (-2.0 * t3 + 3.0 * t2);
        let c = m1 * (t3 - 2.0 * t2 + t);
        let d = m2 * (t3 - t2);
        a + b + c + d
    }
}

// ----------------------------------------------------------------------
// ProfileManager
// ----------------------------------------------------------------------

#[derive(Clone)]
pub struct MeshProfile {
    points: Vec<Vec<f64>>,
    mesh_params: MeshParams,
}

pub struct ProfileManager {
    name: String,
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    bedmesh: Weak<BedMesh>,
    profiles: RefCell<IndexMap<String, MeshProfile>>,
    incompatible_profiles: Vec<String>,
}

impl ProfileManager {
    fn new(config: &ConfigWrapper, bedmesh: &Rc<BedMesh>) -> KResult<Rc<Self>> {
        let name = config.get_name().to_string();
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let mut profiles = IndexMap::new();
        let mut incompatible_profiles = vec![];
        // Fetch stored profiles from Config
        let stored_profs: Vec<ConfigWrapper> = config.get_prefix_sections(&name).into_iter().filter(|s| s.get_name() != name).collect();
        for profile in stored_profs {
            let pname = profile.get_name().splitn(2, ' ').nth(1).unwrap_or("").to_string();
            let version = profile.getint("version", 0, Lim::new())?;
            if version != PROFILE_VERSION {
                logging::info(&format!(
                    "bed_mesh: Profile [{}] not compatible with this version\nof bed_mesh.  Profile Version: {} Current Version: {} ",
                    pname, version, PROFILE_VERSION
                ));
                incompatible_profiles.push(pname);
                continue;
            }
            let zvals = profile.getlists("points", Dflt::Required, &[",", "\n"], None, ListParser::Float, true)?;
            let points: Vec<Vec<f64>> = zvals
                .as_ref()
                .and_then(|v| v.items())
                .unwrap_or(&[])
                .iter()
                .map(|r| r.items().unwrap_or(&[]).iter().map(|v| v.as_f64().unwrap_or(0.0)).collect())
                .collect();
            let mut params = IndexMap::new();
            for (key, t) in PROFILE_OPTIONS {
                let v = match t {
                    'i' => PyVal::Int(profile.getint(key, Dflt::Required, Lim::new())?),
                    'f' => PyVal::Float(profile.getfloat(key, Dflt::Required, Lim::new())?),
                    _ => PyVal::Str(profile.get(key, Dflt::Required)?),
                };
                params.insert(key.to_string(), v);
            }
            profiles.insert(pname, MeshProfile { points, mesh_params: params });
        }
        let pm = Rc::new(ProfileManager {
            name,
            printer,
            gcode: gcode.clone(),
            bedmesh: Rc::downgrade(bedmesh),
            profiles: RefCell::new(profiles),
            incompatible_profiles,
        });
        // Register GCode
        let p = pm.clone();
        gcode.register_command("BED_MESH_PROFILE", Some(Rc::new(move |g| p.cmd_bed_mesh_profile(g))), false, Some(CMD_BED_MESH_PROFILE_HELP))?;
        Ok(pm)
    }

    pub fn get_profiles(&self) -> IndexMap<String, MeshProfile> {
        self.profiles.borrow().clone()
    }

    fn profiles_pyval(&self) -> PyVal {
        let d: IndexMap<String, PyVal> = self
            .profiles
            .borrow()
            .iter()
            .map(|(k, p)| {
                let mut pd = IndexMap::new();
                pd.insert("points".to_string(), matrix_pyval(&p.points));
                pd.insert("mesh_params".to_string(), PyVal::Dict(p.mesh_params.clone()));
                (k.clone(), PyVal::Dict(pd))
            })
            .collect();
        PyVal::Dict(d)
    }

    #[allow(dead_code)]
    fn check_incompatible_profiles(&self) -> KResult<()> {
        if !self.incompatible_profiles.is_empty() {
            let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
            for profile in &self.incompatible_profiles {
                configfile.pconfig.borrow_mut().remove_section(&format!("bed_mesh {}", profile));
            }
            self.gcode.respond_info(
                &format!(
                    "The following incompatible profiles have been detected\nand are scheduled for removal:\n{}\nThe SAVE_CONFIG command will update the printer config\nfile and restart the printer",
                    self.incompatible_profiles.join("\n")
                ),
                true,
            );
        }
        Ok(())
    }

    fn bedmesh(&self) -> Rc<BedMesh> {
        self.bedmesh.upgrade().expect("bed mesh")
    }

    pub fn save_profile(&self, prof_name: &str) -> KResult<()> {
        let Some(z_mesh) = self.bedmesh().get_mesh() else {
            self.gcode.respond_info(&format!("Unable to save to profile [{}], the bed has not been probed", prof_name), true);
            return Ok(());
        };
        let probed_matrix = z_mesh.get_probed_matrix();
        let mesh_params = z_mesh.get_mesh_params();
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let cfg_name = format!("{} {}", self.name, prof_name);
        // set params
        let mut z_values = String::new();
        for line in &probed_matrix {
            z_values += "\n  ";
            for p in line {
                z_values += &format!("{:.6}, ", p);
            }
            z_values.truncate(z_values.len().saturating_sub(2));
        }
        {
            let mut cf = configfile.pconfig.borrow_mut();
            cf.set(&cfg_name, "version", &PROFILE_VERSION.to_string())?;
            cf.set(&cfg_name, "points", &z_values)?;
            for (key, value) in mesh_params.iter() {
                cf.set(&cfg_name, key, &value.str())?;
            }
        }
        // save copy in local storage
        // ensure any self.profiles returned as status remains immutable
        self.profiles.borrow_mut().insert(prof_name.to_string(), MeshProfile { points: probed_matrix, mesh_params });
        self.bedmesh().update_status();
        self.gcode.respond_info(
            &format!(
                "Bed Mesh state has been saved to profile [{}]\nfor the current session.  The SAVE_CONFIG command will\nupdate the printer config file and restart the printer.",
                prof_name
            ),
            true,
        );
        Ok(())
    }

    pub fn load_profile(&self, prof_name: &str) -> KResult<()> {
        let Some(profile) = self.profiles.borrow().get(prof_name).cloned() else {
            return Err(KError::command(format!("bed_mesh: Unknown profile [{}]", prof_name)));
        };
        let z_mesh = Rc::new(ZMesh::new(profile.mesh_params.clone(), Some(prof_name)));
        z_mesh.build_mesh(profile.points.clone()).map_err(KError::command)?;
        self.bedmesh().set_mesh(Some(z_mesh))
    }

    pub fn remove_profile(&self, prof_name: &str) -> KResult<()> {
        if self.profiles.borrow().contains_key(prof_name) {
            let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
            configfile.pconfig.borrow_mut().remove_section(&format!("bed_mesh {}", prof_name));
            self.profiles.borrow_mut().shift_remove(prof_name);
            self.bedmesh().update_status();
            self.gcode.respond_info(
                &format!(
                    "Profile [{}] removed from storage for this session.\nThe SAVE_CONFIG command will update the printer\nconfiguration and restart the printer",
                    prof_name
                ),
                true,
            );
        } else {
            self.gcode.respond_info(&format!("No profile named [{}] to remove", prof_name), true);
        }
        Ok(())
    }

    fn cmd_bed_mesh_profile(&self, gcmd: &GCodeCommand) -> KResult<()> {
        for key in ["LOAD", "SAVE", "REMOVE"] {
            if let Some(name) = gcmd.get_opt(key) {
                if py_strip(&name).is_empty() {
                    return Err(gcmd.error(format!("Value for parameter '{}' must be specified", key)));
                }
                if name == "default" && key == "SAVE" {
                    gcmd.respond_info("Profile 'default' is reserved, please choose another profile name.", true);
                } else {
                    match key {
                        "LOAD" => self.load_profile(&name)?,
                        "SAVE" => self.save_profile(&name)?,
                        _ => self.remove_profile(&name)?,
                    }
                }
                return Ok(());
            }
        }
        gcmd.respond_info(&format!("Invalid syntax '{}'", gcmd.get_commandline()), true);
        Ok(())
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(BedMesh::new(config)?)
}
