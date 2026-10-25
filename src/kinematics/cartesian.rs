// Code for handling the kinematics of cartesian robots
// (port of klippy/kinematics/cartesian.py)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// CartKinematics also carries the corexy, corexz, hybrid_corexy and
// hybrid_corexz kinematics (same rails, limits, homing and status; python
// duplicates the code per module) and the per-axis limits of the limited_*
// kinematics (see limited.rs).
use super::idex_modes::{DualCarriages, DualCarriagesRail, PRIMARY};
use super::limited::Limited;
use super::Kinematics;
use crate::chelper::kin_cartesian::cartesian_stepper_alloc;
use crate::chelper::kin_corexy::corexy_stepper_alloc;
use crate::chelper::kin_corexz::corexz_stepper_alloc;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::Homing;
use crate::gcode::coord_pyval;
use crate::pyval::PyVal;
use crate::stepper::{lookup_multi_rail, McuStepper, PrinterRail, Rail, RailRef};
use crate::toolhead::{Move, ToolHead};
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

#[derive(Clone, Copy, PartialEq)]
pub enum Kind {
    Cartesian,
    CoreXY,
    CoreXZ,
    HybridCoreXY { inverted: bool },
    HybridCoreXZ { inverted: bool },
}

pub struct CartKinematics {
    pub kind: Kind,
    rails: Vec<Rc<PrinterRail>>,
    dc_module: Option<Rc<DualCarriages>>,
    axes_min: [f64; 4],
    axes_max: [f64; 4],
    pub max_z_velocity: Cell<f64>,
    pub max_z_accel: Cell<f64>,
    limits: RefCell<[(f64, f64); 3]>,
    pub limited: Option<Limited>,
    pub toolhead: Weak<ToolHead>,
}

fn setup(rail: &PrinterRail, alloc: fn(char) -> Option<crate::chelper::itersolve::SkRef>, c: char) {
    rail.setup_itersolve(|| alloc(c).expect("stepper kinematics"));
}

/// rails[from].get_steppers() added to rails[to]'s first endstop
fn share_endstop(rails: &[Rc<PrinterRail>], from: usize, to: usize) -> KResult<()> {
    let endstop = rails[to].get_endstops()[0].0.clone();
    for s in rails[from].get_steppers() {
        endstop.add_stepper(&s)?;
    }
    Ok(())
}

impl CartKinematics {
    pub fn new(toolhead: &Rc<ToolHead>, config: &ConfigWrapper, mut kind: Kind) -> KResult<Self> {
        // Setup axis rails
        let mut rails = vec![];
        for n in ["x", "y", "z"] {
            rails.push(lookup_multi_rail(&config.getsection(&format!("stepper_{}", n)), true, None, false)?);
        }
        let mut corexy_mode = ('-', '+');
        let mut max_z_read = false;
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let read_max_z = |kin_z: &mut (f64, f64)| -> KResult<()> {
            kin_z.0 = config.getfloat("max_z_velocity", max_velocity, Lim::new().above(0.0).max(max_velocity))?;
            kin_z.1 = config.getfloat("max_z_accel", max_accel, Lim::new().above(0.0).max(max_accel))?;
            Ok(())
        };
        let mut max_z = (0.0, 0.0);
        match kind {
            Kind::Cartesian => {
                for (rail, axis) in rails.iter().zip(['x', 'y', 'z']) {
                    setup(rail, cartesian_stepper_alloc, axis);
                }
            }
            Kind::CoreXY => {
                share_endstop(&rails, 1, 0)?;
                share_endstop(&rails, 0, 1)?;
                setup(&rails[0], corexy_stepper_alloc, '+');
                setup(&rails[1], corexy_stepper_alloc, '-');
                setup(&rails[2], cartesian_stepper_alloc, 'z');
            }
            Kind::CoreXZ => {
                share_endstop(&rails, 0, 2)?;
                share_endstop(&rails, 2, 0)?;
                setup(&rails[0], corexz_stepper_alloc, '+');
                setup(&rails[1], cartesian_stepper_alloc, 'y');
                setup(&rails[2], corexz_stepper_alloc, '-');
            }
            Kind::HybridCoreXY { .. } | Kind::HybridCoreXZ { .. } => {
                let (endstop_rail, alloc): (usize, fn(char) -> _) = match kind {
                    Kind::HybridCoreXY { .. } => (1, corexy_stepper_alloc),
                    _ => (2, corexz_stepper_alloc),
                };
                share_endstop(&rails, 0, endstop_rail)?;
                let inverted = config.getboolean("invert_kinematics", false)?;
                if inverted {
                    corexy_mode = ('+', '-');
                }
                kind = match kind {
                    Kind::HybridCoreXY { .. } => Kind::HybridCoreXY { inverted },
                    _ => Kind::HybridCoreXZ { inverted },
                };
                for (i, s) in rails[0].get_steppers().iter().enumerate() {
                    let mode = if i == 0 { corexy_mode.0 } else { corexy_mode.1 };
                    s.setup_itersolve(alloc(mode).expect("stepper kinematics"));
                }
                setup(&rails[1], cartesian_stepper_alloc, 'y');
                setup(&rails[2], cartesian_stepper_alloc, 'z');
            }
        }
        if matches!(kind, Kind::CoreXY | Kind::CoreXZ) {
            // corexy.py sets the trapq and reads the z limits before the ranges
            for rail in &rails {
                for s in rail.get_steppers() {
                    s.set_trapq(Some(toolhead.get_trapq()));
                }
            }
            read_max_z(&mut max_z)?;
            max_z_read = true;
        }
        let ranges: Vec<(f64, f64)> = rails.iter().map(|r| r.get_range()).collect();
        let axes_min = [ranges[0].0, ranges[1].0, ranges[2].0, 0.0];
        let axes_max = [ranges[0].1, ranges[1].1, ranges[2].1, 0.0];
        let mut dc_module = None;
        if config.has_section("dual_carriage") && !matches!(kind, Kind::CoreXY | Kind::CoreXZ) {
            let dc_config = config.getsection("dual_carriage");
            let dc_axis = match kind {
                Kind::Cartesian => dc_config.getchoice("axis", &[("x", 0usize), ("y", 1)], Dflt::Required)?,
                // dummy for cartesian config users
                _ => dc_config.getchoice("axis", &[("x", 0usize)], "x")?,
            };
            // setup second dual carriage rail
            rails.push(lookup_multi_rail(&dc_config, true, None, false)?);
            match kind {
                Kind::Cartesian => setup(&rails[3], cartesian_stepper_alloc, ['x', 'y'][dc_axis]),
                _ => {
                    let (endstop_rail, alloc): (usize, fn(char) -> _) = match kind {
                        Kind::HybridCoreXY { .. } => (1, corexy_stepper_alloc),
                        _ => (2, corexz_stepper_alloc),
                    };
                    share_endstop(&rails, 3, endstop_rail)?;
                    for (i, s) in rails[3].get_steppers().iter().enumerate() {
                        let mode = if i == 0 { corexy_mode.1 } else { corexy_mode.0 };
                        s.setup_itersolve(alloc(mode).expect("stepper kinematics"));
                    }
                }
            }
            let dc_rail_0 = DualCarriagesRail::new(&rails[dc_axis], dc_axis, true);
            let dc_rail_1 = DualCarriagesRail::new(&rails[3], dc_axis, false);
            dc_module = Some(DualCarriages::new(&dc_config, dc_rail_0, dc_rail_1, dc_axis)?);
        }
        if !max_z_read {
            for rail in &rails {
                for s in rail.get_steppers() {
                    s.set_trapq(Some(toolhead.get_trapq()));
                }
            }
            // Setup boundary checks
            read_max_z(&mut max_z)?;
        }
        Ok(CartKinematics {
            kind,
            rails,
            dc_module,
            axes_min,
            axes_max,
            max_z_velocity: Cell::new(max_z.0),
            max_z_accel: Cell::new(max_z.1),
            limits: RefCell::new([(1.0, -1.0); 3]),
            limited: None,
            toolhead: Rc::downgrade(toolhead),
        })
    }

    pub fn check_endstops(&self, mv: &Move) -> KResult<()> {
        let end_pos = &mv.end_pos;
        let limits = self.limits.borrow();
        for i in 0..3 {
            if mv.axes_d[i] != 0.0 && (end_pos[i] < limits[i].0 || end_pos[i] > limits[i].1) {
                if limits[i].0 > limits[i].1 {
                    return Err(mv.move_error("Must home axis first"));
                }
                return Err(mv.move_error("Move out of range"));
            }
        }
        Ok(())
    }

    /// the rail of an axis (the primary dual carriage rail on its axis)
    fn axis_rail(&self, axis: usize) -> KResult<Rc<PrinterRail>> {
        match &self.dc_module {
            Some(dc) if axis == dc.axis => dc.primary_printer_rail(),
            _ => Ok(self.rails[axis].clone()),
        }
    }

    fn default_check_move(&self, mv: &mut Move) -> KResult<()> {
        let (xpos, ypos) = (mv.end_pos[0], mv.end_pos[1]);
        let outside = {
            let limits = self.limits.borrow();
            xpos < limits[0].0 || xpos > limits[0].1 || ypos < limits[1].0 || ypos > limits[1].1
        };
        if outside {
            self.check_endstops(mv)?;
        }
        if mv.axes_d[2] == 0.0 {
            // Normal XY move - use defaults
            return Ok(());
        }
        // Move with Z - update velocity and accel for slower Z axis
        self.check_endstops(mv)?;
        let z_ratio = mv.move_d / mv.axes_d[2].abs();
        mv.limit_speed(self.max_z_velocity.get() * z_ratio, self.max_z_accel.get() * z_ratio);
        Ok(())
    }
}

impl Kinematics for CartKinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>> {
        self.rails.iter().flat_map(|r| r.get_steppers()).collect()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        let pos: Vec<f64> = self.rails.iter().map(|r| stepper_positions[&r.get_name(false)]).collect();
        match self.kind {
            Kind::Cartesian => {
                let mut pos = pos;
                if let Some(dc) = &self.dc_module {
                    let primary = dc.primary_printer_rail().expect("primary rail");
                    pos[dc.axis] = stepper_positions[&primary.get_name(false)];
                }
                pos
            }
            Kind::CoreXY => vec![0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]],
            Kind::CoreXZ => vec![0.5 * (pos[0] + pos[2]), pos[1], 0.5 * (pos[0] - pos[2])],
            Kind::HybridCoreXY { inverted } | Kind::HybridCoreXZ { inverted } => {
                let o = if matches!(self.kind, Kind::HybridCoreXY { .. }) { pos[1] } else { pos[2] };
                let dc_primary = self.dc_module.as_ref().map(|dc| dc.get_rails()[1].mode.get() == PRIMARY).unwrap_or(false);
                let a = if dc_primary { pos[3] } else { pos[0] };
                let x = match (dc_primary, inverted) {
                    (true, true) => a + o,
                    (true, false) => a - o,
                    (false, true) => a - o,
                    (false, false) => a + o,
                };
                vec![x, pos[1], pos[2]]
            }
        }
    }

    fn update_limits(&self, i: usize, range: (f64, f64)) {
        let (l, h) = self.limits.borrow()[i];
        // Only update limits if this axis was already homed,
        // otherwise leave in un-homed state.
        if l <= h {
            self.limits.borrow_mut()[i] = range;
        }
    }

    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        for rail in &self.rails {
            rail.set_position(newpos);
        }
        if matches!(self.kind, Kind::CoreXY | Kind::CoreXZ) {
            for (i, a) in "xyz".chars().enumerate() {
                if homing_axes.contains(a) {
                    self.limits.borrow_mut()[i] = self.rails[i].get_range();
                }
            }
            return Ok(());
        }
        for axis_name in homing_axes.chars() {
            let axis = "xyz".find(axis_name).ok_or_else(|| KError::Other("substring not found".into()))?;
            let rail = self.axis_rail(axis)?;
            self.limits.borrow_mut()[axis] = rail.get_range();
        }
        Ok(())
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        for (axis, axis_name) in "xyz".chars().enumerate() {
            if clear_axes.contains(axis_name) {
                self.limits.borrow_mut()[axis] = (1.0, -1.0);
            }
        }
    }

    fn home_axis(&self, homing_state: &Rc<Homing>, axis: usize, rail: &Rc<PrinterRail>) -> KResult<()> {
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
        homing_state.home_rails(&[r], &forcepos, &homepos)
    }

    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()> {
        // Each axis is homed independently and in order
        for axis in homing_state.get_axes() {
            match &self.dc_module {
                Some(dc) if axis == dc.axis => dc.home(homing_state)?,
                _ => self.home_axis(homing_state, axis, &self.rails[axis])?,
            }
        }
        Ok(())
    }

    fn check_move(&self, mv: &mut Move) -> KResult<()> {
        match &self.limited {
            Some(l) => l.check_move(self, mv),
            None => self.default_check_move(mv),
        }
    }

    fn rails(&self) -> Vec<RailRef> {
        self.rails.iter().map(|r| r.clone() as RailRef).collect()
    }

    fn get_status(&self, _eventtime: f64) -> IndexMap<String, PyVal> {
        let limits = self.limits.borrow();
        let axes: String = "xyz".chars().zip(limits.iter()).filter(|(_, (l, h))| l <= h).map(|(a, _)| a).collect();
        let mut d = IndexMap::new();
        d.insert("homed_axes".to_string(), PyVal::Str(axes));
        d.insert("axis_minimum".to_string(), coord_pyval(&self.axes_min));
        d.insert("axis_maximum".to_string(), coord_pyval(&self.axes_max));
        d
    }

    fn supports_dual_carriage(&self) -> bool {
        match &self.limited {
            Some(l) => l.supports_dual_carriage(),
            None => !matches!(self.kind, Kind::CoreXY | Kind::CoreXZ),
        }
    }

    fn limit(&self, name: &str) -> Option<&Cell<f64>> {
        match name {
            "max_z_velocity" => Some(&self.max_z_velocity),
            "max_z_accel" => Some(&self.max_z_accel),
            _ => self.limited.as_ref().and_then(|l| l.limit(name)),
        }
    }

}

pub fn load_kinematics(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(CartKinematics::new(toolhead, config, Kind::Cartesian)?))
}

pub fn load_corexy(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(CartKinematics::new(toolhead, config, Kind::CoreXY)?))
}

pub fn load_corexz(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(CartKinematics::new(toolhead, config, Kind::CoreXZ)?))
}

pub fn load_hybrid_corexy(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(CartKinematics::new(toolhead, config, Kind::HybridCoreXY { inverted: false })?))
}

pub fn load_hybrid_corexz(toolhead: &Rc<ToolHead>, config: &ConfigWrapper) -> KResult<Rc<dyn Kinematics>> {
    Ok(Rc::new(CartKinematics::new(toolhead, config, Kind::HybridCoreXZ { inverted: false })?))
}
