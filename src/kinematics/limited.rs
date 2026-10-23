// Per axis velocity and acceleration limits of the limited_cartesian,
// limited_corexy and limited_corexz kinematics
// (port of klippy/kinematics/limited_*.py)
//
// Copyright (C) 2020-2021  Mael Kerbiriou <piezo.wdimd@gmail.com>
// Copyright © 2024 Brandon Smith <honestbrotherstv@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::cartesian::{CartKinematics, Kind};
use super::Kinematics;
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mathutil::py_hypot;
use crate::pyfmt::{py_float_repr, py_max, py_min};
use crate::toolhead::{Move, ToolHead};
use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::rc::Rc;

#[derive(Clone, Copy, PartialEq)]
pub enum Mode {
    Cartesian,
    CoreXY,
    CoreXZ,
}

pub struct Limited {
    mode: Mode,
    max_velocities: RefCell<[f64; 3]>,
    max_accels: RefCell<[f64; 3]>,
    xy_hypot_accel: Cell<f64>,
    scale_per_axis: Cell<bool>,
    // limited_corexy
    max_x_accel: Cell<f64>,
    max_y_accel: Cell<f64>,
}

/// the limits suspend_limits() replaced
pub struct LimitsSnapshot {
    max_velocities: [f64; 3],
    max_accels: [f64; 3],
    max_x_accel: f64,
    max_y_accel: f64,
    scale_per_axis: bool,
}

fn list_repr(v: &[f64], open: &str, close: &str) -> String {
    format!("{}{}{}", open, v.iter().map(|f| py_float_repr(*f)).collect::<Vec<_>>().join(", "), close)
}

impl Limited {
    fn new(toolhead: &ToolHead, config: &ConfigWrapper, mode: Mode) -> KResult<Self> {
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let mut l = Limited {
            mode,
            max_velocities: RefCell::new([0.0; 3]),
            max_accels: RefCell::new([0.0; 3]),
            xy_hypot_accel: Cell::new(0.0),
            scale_per_axis: Cell::new(false),
            max_x_accel: Cell::new(0.0),
            max_y_accel: Cell::new(0.0),
        };
        match mode {
            Mode::Cartesian | Mode::CoreXZ => {
                let mut mv = [0.0; 3];
                let mut ma = [0.0; 3];
                for (i, ax) in "xyz".chars().enumerate() {
                    let opt = format!("max_{}_velocity", ax);
                    mv[i] = match mode {
                        Mode::Cartesian => {
                            config.getfloat(&opt, py_hypot(&[1.0, 1.0]) * max_velocity, Lim::new().above(0.0).max(max_velocity))?
                        }
                        _ => config.getfloat(&opt, max_velocity, Lim::new().above(0.0))?,
                    };
                }
                for (i, ax) in "xyz".chars().enumerate() {
                    let opt = format!("max_{}_accel", ax);
                    ma[i] = match mode {
                        Mode::Cartesian => config.getfloat(&opt, max_accel, Lim::new().above(0.0).max(max_accel))?,
                        _ => config.getfloat(&opt, max_accel, Lim::new().above(0.0))?,
                    };
                }
                l.max_velocities = RefCell::new(mv);
                l.max_accels = RefCell::new(ma);
                l.xy_hypot_accel.set(py_hypot(&ma[..2]));
            }
            Mode::CoreXY => {
                // Setup x/y axis limits
                l.max_x_accel.set(config.getfloat("max_x_accel", max_accel, Lim::new().above(0))?);
                l.max_y_accel.set(config.getfloat("max_y_accel", max_accel, Lim::new().above(0))?);
            }
        }
        l.scale_per_axis.set(config.getboolean("scale_xy_accel", false)?);
        Ok(l)
    }

    /// resonance_tester's suspend_limits(): set the xy limits to
    /// max_velocity / max_accel and turn off scale_per_axis; returns the
    /// old values
    pub fn suspend_limits(&self, max_accel: f64, max_velocity: f64) -> LimitsSnapshot {
        let snap = LimitsSnapshot {
            max_velocities: *self.max_velocities.borrow(),
            max_accels: *self.max_accels.borrow(),
            max_x_accel: self.max_x_accel.get(),
            max_y_accel: self.max_y_accel.get(),
            scale_per_axis: self.scale_per_axis.get(),
        };
        if self.mode == Mode::CoreXY {
            self.max_x_accel.set(max_accel);
            self.max_y_accel.set(max_accel);
        } else {
            *self.max_velocities.borrow_mut() = [max_velocity, max_velocity, snap.max_velocities[2]];
            *self.max_accels.borrow_mut() = [max_accel, max_accel, snap.max_accels[2]];
        }
        self.scale_per_axis.set(false);
        snap
    }

    pub fn restore_limits(&self, snap: &LimitsSnapshot) {
        if self.mode == Mode::CoreXY {
            self.max_x_accel.set(snap.max_x_accel);
            self.max_y_accel.set(snap.max_y_accel);
        } else {
            *self.max_velocities.borrow_mut() = snap.max_velocities;
            *self.max_accels.borrow_mut() = snap.max_accels;
        }
        self.scale_per_axis.set(snap.scale_per_axis);
    }

    pub fn supports_dual_carriage(&self) -> bool {
        self.mode == Mode::Cartesian
    }

    pub fn limit(&self, name: &str) -> Option<&Cell<f64>> {
        match (self.mode, name) {
            (Mode::CoreXY, "max_x_accel") => Some(&self.max_x_accel),
            (Mode::CoreXY, "max_y_accel") => Some(&self.max_y_accel),
            _ => None,
        }
    }

    fn cmd_set_kinematics_limit(&self, kin: &CartKinematics, gcmd: &GCodeCommand) -> KResult<()> {
        if self.mode == Mode::CoreXY {
            self.max_x_accel.set(gcmd.get_float("X_ACCEL", Some(self.max_x_accel.get()), Lim::new().above(0.0))?);
            self.max_y_accel.set(gcmd.get_float("Y_ACCEL", Some(self.max_y_accel.get()), Lim::new().above(0.0))?);
            kin.max_z_accel.set(gcmd.get_float("Z_ACCEL", Some(kin.max_z_accel.get()), Lim::new().above(0.0))?);
            self.set_scale(gcmd)?;
            let max_accels = [self.max_x_accel.get(), self.max_y_accel.get(), kin.max_z_accel.get()];
            if !gcmd.get_command_parameters().is_empty() {
                return Ok(());
            }
            let mut msg = vec![format!("x,y,z max_accels: {}", list_repr(&max_accels, "(", ")"))];
            msg.push(self.scale_line().to_string());
            let (ax, ay) = (self.max_x_accel.get(), self.max_y_accel.get());
            let min_accel = 1.0 / (ax.powf(-2.0) + ay.powf(-2.0)).sqrt();
            let min_angle = ax.atan2(ay);
            msg.push(format!("Minimum XY acceleration of {:.0} mm/s² reached on {:.0}° diagonals.", min_accel, 180.0 * min_angle / PI));
            gcmd.respond_info(&msg.join("\n"), true);
            return Ok(());
        }
        let mut mv = *self.max_velocities.borrow();
        for (i, ax) in "XYZ".chars().enumerate() {
            mv[i] = gcmd.get_float(&format!("{}_VELOCITY", ax), Some(mv[i]), Lim::new().above(0.0))?;
        }
        *self.max_velocities.borrow_mut() = mv;
        let mut ma = *self.max_accels.borrow();
        for (i, ax) in "XYZ".chars().enumerate() {
            ma[i] = gcmd.get_float(&format!("{}_ACCEL", ax), Some(ma[i]), Lim::new().above(0.0))?;
        }
        *self.max_accels.borrow_mut() = ma;
        self.xy_hypot_accel.set(py_hypot(&ma[..2]));
        self.set_scale(gcmd)?;
        if self.mode == Mode::CoreXZ {
            let mut msg = format!(
                "x,y,z max_velocities: {}\nx,y,z max_accels: {}\n",
                list_repr(&mv, "[", "]"),
                list_repr(&ma, "[", "]")
            );
            msg += self.scale_line();
            msg += "\n";
            msg += &format!(
                "Maximum XY velocity of {:.1} mm/s reached on {:.0} degrees diagonals.\n\
                 Maximum XY acceleration of {:.0} mm/s^2 reached on {:.0} degrees diagonals.",
                py_hypot(&mv[..2]),
                180.0 * mv[1].atan2(mv[0]) / PI,
                self.xy_hypot_accel.get(),
                180.0 * ma[1].atan2(ma[0]) / PI
            );
            gcmd.respond_info(&msg, true);
            return Ok(());
        }
        if !gcmd.get_command_parameters().is_empty() {
            return Ok(());
        }
        let mut msg = vec![
            format!("x,y,z max_velocities: {}", list_repr(&mv, "[", "]")),
            format!("x,y,z max_accels: {}", list_repr(&ma, "[", "]")),
        ];
        msg.push(self.scale_line().to_string());
        let max_velocity = py_hypot(&mv[..2]);
        let max_v_angle = mv[1].atan2(mv[0]);
        msg.push(format!("Maximum XY velocity of {:.1} mm/s reached on {:.0}° diagonals.", max_velocity, max_v_angle * 180.0 / PI));
        let max_accel = py_hypot(&ma[..2]);
        let max_a_angle = ma[1].atan2(ma[0]);
        msg.push(format!("Maximum XY acceleration of {:.1} mm/s reached on {:.0}° diagonals.", max_accel, max_a_angle * 180.0 / PI));
        gcmd.respond_info(&msg.join("\n"), false);
        Ok(())
    }

    fn set_scale(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.scale_per_axis.set(gcmd.get_int("SCALE", Some(self.scale_per_axis.get() as i64), Lim::new().min(0).max(1))? != 0);
        Ok(())
    }

    fn scale_line(&self) -> &'static str {
        if self.scale_per_axis.get() {
            "Per axis accelerations limits scale with current acceleration."
        } else {
            "Per axis accelerations limits are independent of current acceleration."
        }
    }

    fn toolhead_max_accel(kin: &CartKinematics) -> (f64, f64) {
        kin.toolhead.upgrade().map(|t| t.get_max_velocity()).unwrap_or((0.0, 0.0))
    }

    pub fn check_move(&self, kin: &CartKinematics, mv: &mut Move) -> KResult<()> {
        if !mv.is_kinematic_move {
            return Ok(());
        }
        kin.check_endstops(mv)?;
        if self.mode == Mode::CoreXY {
            let epsilon = 2.0 * f64::EPSILON;
            let (mut max_v, mut max_a) = Self::toolhead_max_accel(kin);
            let move_d = mv.move_d;
            let (x, y, z) = (mv.axes_d[0], mv.axes_d[1], mv.axes_d[2]);
            let ab_linf = py_max((x + y).abs(), (x - y).abs());
            if ab_linf > epsilon {
                max_v *= move_d / ab_linf;
                let (max_x_accel, max_y_accel) = (self.max_x_accel.get(), self.max_y_accel.get());
                let x_o_a = x / max_x_accel;
                let y_o_a = y / max_y_accel;
                let linf = py_max((x_o_a + y_o_a).abs(), (x_o_a - y_o_a).abs());
                if self.scale_per_axis.get() {
                    max_a *= move_d / (linf * py_max(max_x_accel, max_y_accel));
                } else {
                    max_a = move_d / linf;
                }
            }
            if z != 0.0 {
                let z_ratio = move_d / z.abs();
                max_v = py_min(max_v, kin.max_z_velocity.get() * z_ratio);
                max_a = py_min(max_a, kin.max_z_accel.get() * z_ratio);
            }
            mv.limit_speed(max_v, max_a);
            return Ok(());
        }
        let epsilon = f64::EPSILON;
        let (x_r, y_r, z_r) = (mv.axes_r[0], mv.axes_r[1], mv.axes_r[2]);
        let [x_max_v, y_max_v, z_max_v] = *self.max_velocities.borrow();
        let [x_max_a, y_max_a, z_max_a] = *self.max_accels.borrow();
        let x_r = py_max(x_r.abs(), epsilon);
        let y_r = py_max(y_r.abs(), epsilon);
        let mut max_v = py_min(x_max_v / x_r, y_max_v / y_r);
        let mut max_a = py_min(x_max_a / x_r, y_max_a / y_r);
        if self.scale_per_axis.get() {
            let (_, toolhead_max_a) = Self::toolhead_max_accel(kin);
            max_a *= toolhead_max_a / self.xy_hypot_accel.get();
        }
        if z_r != 0.0 {
            let z_r = z_r.abs();
            max_v = py_min(max_v, z_max_v / z_r);
            max_a = py_min(max_a, z_max_a / z_r);
        }
        mv.limit_speed(max_v, max_a);
        Ok(())
    }
}

fn load(toolhead: &Rc<ToolHead>, config: &ConfigWrapper, kind: Kind, mode: Mode, desc: Option<&str>) -> KResult<Rc<dyn Kinematics>> {
    let mut kin = CartKinematics::new(toolhead, config, kind)?;
    kin.limited = Some(Limited::new(toolhead, config, mode)?);
    let kin = Rc::new(kin);
    let gcode = config.get_printer().lookup::<GCodeDispatch>("gcode")?;
    let w = Rc::downgrade(&kin);
    gcode.register_command(
        "SET_KINEMATICS_LIMIT",
        Some(Rc::new(move |g| match w.upgrade() {
            Some(k) => k.limited.as_ref().expect("limited").cmd_set_kinematics_limit(&k, g),
            None => Ok(()),
        })),
        false,
        desc,
    )?;
    Ok(kin)
}

pub fn load_limited_cartesian(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    load(toolhead, config, Kind::Cartesian, Mode::Cartesian, Some("Set/get cartesian per axis velocity limits"))
}

pub fn load_limited_corexy(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    load(toolhead, config, Kind::CoreXY, Mode::CoreXY, Some("Set/get CoreXY per axis velocity limits"))
}

pub fn load_limited_corexz(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    load(toolhead, config, Kind::CoreXZ, Mode::CoreXZ, None)
}
