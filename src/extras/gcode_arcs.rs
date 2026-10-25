// adds support fro ARC commands via G2/G3 (port of klippy/extras/gcode_arcs.py)
//
// Copyright (C) 2019  Aleksej Vasiljkovic <achmed21@gmail.com>
//
// function planArc() originates from https://github.com/MarlinFirmware/Marlin
// Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_move::GCodeMove;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mathutil::py_hypot;
use crate::printer::{downcast_rc, PObj, PrinterObject};
use crate::pyfmt::{py_float_repr, py_max};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::cell::Cell;
use std::f64::consts::PI;
use std::rc::Rc;

// Coordinates created by this are converted into G1 commands.
//
// supports XY, XZ & YZ planes with remaining axis as helical

// Enum
pub const ARC_PLANE_X_Y: i32 = 0;
pub const ARC_PLANE_X_Z: i32 = 1;
pub const ARC_PLANE_Y_Z: i32 = 2;

// Enum
pub const X_AXIS: usize = 0;
pub const Y_AXIS: usize = 1;
pub const Z_AXIS: usize = 2;
pub const E_AXIS: usize = 3;

pub struct ArcSupport {
    mm_per_arc_segment: f64,
    gcode_move: Rc<GCodeMove>,
    gcode: Rc<GCodeDispatch>,
    plane: Cell<i32>,
}

fn coord4(v: Option<&PyVal>) -> Vec<f64> {
    match v {
        Some(PyVal::NamedTuple(_, _, vals)) | Some(PyVal::Tuple(vals)) | Some(PyVal::List(vals)) => {
            vals.iter().map(|x| x.as_f64().unwrap_or(0.0)).collect()
        }
        _ => vec![0.0; 4],
    }
}

impl ArcSupport {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mm_per_arc_segment = config.getfloat("resolution", 1.0, Lim::new().above(0.0))?;
        let gcode_move = printer
            .load_object(config, "gcode_move", true)?
            .and_then(downcast_rc::<GCodeMove>)
            .ok_or_else(|| KError::Other("gcode_move has an unexpected type".into()))?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        // backwards compatibility, prior implementation only supported XY
        let a = Rc::new(ArcSupport { mm_per_arc_segment, gcode_move, gcode: gcode.clone(), plane: Cell::new(ARC_PLANE_X_Y) });
        let s = a.clone();
        gcode.register_command("G2", Some(Rc::new(move |g| s.cmd_inner(g, true))), false, None)?;
        let s = a.clone();
        gcode.register_command("G3", Some(Rc::new(move |g| s.cmd_inner(g, false))), false, None)?;
        for (cmd, plane) in [("G17", ARC_PLANE_X_Y), ("G18", ARC_PLANE_X_Z), ("G19", ARC_PLANE_Y_Z)] {
            let s = a.clone();
            gcode.register_command(
                cmd,
                Some(Rc::new(move |_g: &GCodeCommand| {
                    s.plane.set(plane);
                    Ok(())
                })),
                false,
                None,
            )?;
        }
        Ok(a)
    }

    fn cmd_inner(&self, gcmd: &GCodeCommand, clockwise: bool) -> KResult<()> {
        let gcodestatus = match PrinterObject::get_status(&*self.gcode_move, 0.0) {
            Some(PyVal::Dict(d)) => d,
            _ => IndexMap::new(),
        };
        if !gcodestatus.get("absolute_coordinates").map(|v| v.truthy()).unwrap_or(false) {
            return Err(gcmd.error("G2/G3 does not support relative move mode"));
        }
        let current_pos = coord4(gcodestatus.get("gcode_position"));
        let absolut_extrude = gcodestatus.get("absolute_extrude").map(|v| v.truthy()).unwrap_or(false);

        // Parse parameters
        let as_target = vec![
            gcmd.get_float("X", Some(current_pos[0]), Lim::new())?,
            gcmd.get_float("Y", Some(current_pos[1]), Lim::new())?,
            gcmd.get_float("Z", Some(current_pos[2]), Lim::new())?,
        ];

        if gcmd.get_float_opt("R", Lim::new())?.is_some() {
            return Err(gcmd.error("G2/G3 does not support R moves"));
        }

        // determine the plane coordinates and the helical axis
        let i = gcmd.get_float("I", Some(0.0), Lim::new())?;
        let j = gcmd.get_float("J", Some(0.0), Lim::new())?;
        let (mut as_planar, mut axes) = ((i, j), (X_AXIS, Y_AXIS, Z_AXIS));
        if self.plane.get() == ARC_PLANE_X_Z {
            let k = gcmd.get_float("K", Some(0.0), Lim::new())?;
            as_planar = (i, k);
            axes = (X_AXIS, Z_AXIS, Y_AXIS);
        } else if self.plane.get() == ARC_PLANE_Y_Z {
            let k = gcmd.get_float("K", Some(0.0), Lim::new())?;
            as_planar = (j, k);
            axes = (Y_AXIS, Z_AXIS, X_AXIS);
        }

        if as_planar.0 == 0.0 && as_planar.1 == 0.0 {
            return Err(gcmd.error("G2/G3 requires IJ, IK or JK parameters"));
        }

        // Build linear coordinates to move
        self.plan_arc(&current_pos, &as_target, as_planar, clockwise, gcmd, absolut_extrude, axes)
    }

    // function planArc() originates from marlin plan_arc()
    // https://github.com/MarlinFirmware/Marlin
    //
    // The arc is approximated by generating many small linear segments.
    // The length of each segment is configured in MM_PER_ARC_SEGMENT
    // Arcs smaller then this value, will be a Line only
    //
    // alpha and beta axes are the current plane, helical axis is linear travel
    #[allow(clippy::too_many_arguments)]
    fn plan_arc(
        &self,
        current_pos: &[f64],
        target_pos: &[f64],
        offset: (f64, f64),
        clockwise: bool,
        gcmd: &GCodeCommand,
        absolut_extrude: bool,
        (alpha_axis, beta_axis, helical_axis): (usize, usize, usize),
    ) -> KResult<()> {
        // todo: sometimes produces full circles

        // Radius vector from center to current location
        let r_p = -offset.0;
        let r_q = -offset.1;

        // Determine angular travel
        let center_p = current_pos[alpha_axis] - r_p;
        let center_q = current_pos[beta_axis] - r_q;
        let rt_alpha = target_pos[alpha_axis] - center_p;
        let rt_beta = target_pos[beta_axis] - center_q;
        let mut angular_travel = (r_p * rt_beta - r_q * rt_alpha).atan2(r_p * rt_alpha + r_q * rt_beta);
        if angular_travel < 0.0 {
            angular_travel += 2.0 * PI;
        }
        if clockwise {
            angular_travel -= 2.0 * PI;
        }

        if angular_travel == 0.0 && current_pos[alpha_axis] == target_pos[alpha_axis] && current_pos[beta_axis] == target_pos[beta_axis] {
            // Make a circle if the angular rotation is 0 and the
            // target is current position
            angular_travel = 2.0 * PI;
        }

        // Determine number of segments
        let linear_travel = target_pos[helical_axis] - current_pos[helical_axis];
        let radius = py_hypot(&[r_p, r_q]);
        let flat_mm = radius * angular_travel;
        let mm_of_travel = if linear_travel != 0.0 { py_hypot(&[flat_mm, linear_travel]) } else { flat_mm.abs() };
        let segments = py_max(1.0, (mm_of_travel / self.mm_per_arc_segment).floor());

        // Generate coordinates
        let theta_per_segment = angular_travel / segments;
        let linear_per_segment = linear_travel / segments;

        let as_e = gcmd.get_float_opt("E", Lim::new())?;
        let as_f = gcmd.get_float_opt("F", Lim::new())?;

        let (mut e_per_move, mut e_base) = (0.0, 0.0);
        if let Some(as_e) = as_e {
            if absolut_extrude {
                e_base = current_pos[3];
            }
            e_per_move = (as_e - e_base) / segments;
        }

        for i in 1..=(segments as i64) {
            let dist_helical = i as f64 * linear_per_segment;
            let c_theta = i as f64 * theta_per_segment;
            let cos_ti = c_theta.cos();
            let sin_ti = c_theta.sin();
            let r_p = -offset.0 * cos_ti + offset.1 * sin_ti;
            let r_q = -offset.0 * sin_ti - offset.1 * cos_ti;

            let mut c = [0.0; 3];
            c[alpha_axis] = center_p + r_p;
            c[beta_axis] = center_q + r_q;
            c[helical_axis] = current_pos[helical_axis] + dist_helical;

            if i as f64 == segments {
                c = [target_pos[0], target_pos[1], target_pos[2]];
            }
            // Convert coords into G1 commands
            let mut g1_params = IndexMap::new();
            for (k, v) in ["X", "Y", "Z"].iter().zip(c) {
                g1_params.insert(k.to_string(), py_float_repr(v));
            }
            if e_per_move != 0.0 {
                g1_params.insert("E".to_string(), py_float_repr(e_base + e_per_move));
                if absolut_extrude {
                    e_base += e_per_move;
                }
            }
            if let Some(f) = as_f {
                g1_params.insert("F".to_string(), py_float_repr(f));
            }
            let g1_gcmd = self.gcode.create_gcode_command("G1", "G1", g1_params);
            self.gcode_move.cmd_g1(&g1_gcmd)?;
        }
        Ok(())
    }
}

impl PrinterObject for ArcSupport {
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ArcSupport::new(config)?)
}
