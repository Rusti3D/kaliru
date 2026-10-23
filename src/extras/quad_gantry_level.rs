// Mechanicaly conforms a moving gantry to the bed with 4 Z steppers
// (port of klippy/extras/quad_gantry_level.py)
//
// Copyright (C) 2018  Maks Zolin <mzolin@vorondesign.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::probe::{FinalizeResult, ProbePointsHelper};
use crate::extras::z_tilt::{get_pairs, RetryHelper, ZAdjustHelper, ZAdjustStatus};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_float_repr, py_max};
use crate::pyval::PyVal;
use std::any::Any;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

const CMD_QUAD_GANTRY_LEVEL_HELP: &str = "Conform a moving, twistable gantry to the shape of a stationary bed";

/// a fitted line (m, b); a straight line keeps python's int 0 slope
#[derive(Clone, Copy)]
struct Line {
    m: f64,
    b: f64,
    int_m: bool,
}

impl Line {
    fn repr(&self) -> String {
        let m = if self.int_m { "0".to_string() } else { py_float_repr(self.m) };
        format!("({}, {})", m, py_float_repr(self.b))
    }
}

pub struct QuadGantryLevel {
    retry_helper: RetryHelper,
    max_adjust: f64,
    horizontal_move_z: f64,
    probe_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    z_status: Rc<ZAdjustStatus>,
    z_helper: Rc<ZAdjustHelper>,
    gantry_corners: Vec<(f64, f64)>,
    gcode: Rc<GCodeDispatch>,
}

impl QuadGantryLevel {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let retry_helper = RetryHelper::new(config, "Possibly Z motor numbering is wrong")?;
        let max_adjust = config.getfloat("max_adjust", 4.0, Lim::new().above(0))?;
        let horizontal_move_z = config.getfloat("horizontal_move_z", 5.0, Lim::new())?;
        let slot: Rc<RefCell<Weak<QuadGantryLevel>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        let probe_helper = ProbePointsHelper::new(
            config,
            Rc::new(move |offsets, positions| match sl.borrow().upgrade() {
                Some(q) => q.probe_finalize(offsets, positions),
                None => Ok(FinalizeResult::Done),
            }),
            None,
            "points",
            false,
            false,
        )?;
        if probe_helper.get_probe_points().len() != 4 {
            return Err(KError::config("Need exactly 4 probe points for quad_gantry_level"));
        }
        let z_status = ZAdjustStatus::new(&printer);
        let z_helper = ZAdjustHelper::new(config, 4);
        let gantry_corners = get_pairs(config, "gantry_corners")?;
        if gantry_corners.len() < 2 {
            return Err(KError::config("quad_gantry_level requires at least two gantry_corners"));
        }
        // Register QUAD_GANTRY_LEVEL command
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let q = Rc::new(QuadGantryLevel {
            retry_helper,
            max_adjust,
            horizontal_move_z,
            probe_helper: RefCell::new(Some(probe_helper)),
            z_status,
            z_helper,
            gantry_corners,
            gcode: gcode.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&q);
        let qq = q.clone();
        gcode.register_command(
            "QUAD_GANTRY_LEVEL",
            Some(Rc::new(move |g| qq.cmd_quad_gantry_level(g))),
            false,
            Some(CMD_QUAD_GANTRY_LEVEL_HELP),
        )?;
        Ok(q)
    }

    fn probe_helper(&self) -> Rc<ProbePointsHelper> {
        self.probe_helper.borrow().clone().expect("probe helper")
    }

    fn cmd_quad_gantry_level(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.z_status.reset();
        self.retry_helper.start(gcmd)?;
        self.probe_helper().start_probe(gcmd)
    }

    fn probe_finalize(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        // Mirror our perspective so the adjustments make sense
        // from the perspective of the gantry
        let z_positions: Vec<f64> = positions.iter().map(|p| self.horizontal_move_z - p[2]).collect();
        let pts: Vec<String> = z_positions.iter().enumerate().map(|(z_id, z)| format!("{}: {:.6}", z_id, z)).collect();
        self.gcode.respond_info(&format!("Gantry-relative probe points:\n{}\n", pts.join(" ")), true);
        // Calculate slope along X axis between probe point 0 and 3
        let ppx0 = [positions[0][0] + offsets.0, z_positions[0]];
        let ppx3 = [positions[3][0] + offsets.0, z_positions[3]];
        let slope_x_pp03 = Self::linefit(ppx0, ppx3);
        // Calculate slope along X axis between probe point 1 and 2
        let ppx1 = [positions[1][0] + offsets.0, z_positions[1]];
        let ppx2 = [positions[2][0] + offsets.0, z_positions[2]];
        let slope_x_pp12 = Self::linefit(ppx1, ppx2);
        logging::info(&format!("quad_gantry_level f1: {}, f2: {}", slope_x_pp03.repr(), slope_x_pp12.repr()));
        // Calculate gantry slope along Y axis between stepper 0 and 1
        let gc = &self.gantry_corners;
        let a1 = [positions[0][1] + offsets.1, Self::plot(slope_x_pp03, gc[0].0)];
        let a2 = [positions[1][1] + offsets.1, Self::plot(slope_x_pp12, gc[0].0)];
        let slope_y_s01 = Self::linefit(a1, a2);
        // Calculate gantry slope along Y axis between stepper 2 and 3
        let b1 = [positions[0][1] + offsets.1, Self::plot(slope_x_pp03, gc[1].0)];
        let b2 = [positions[1][1] + offsets.1, Self::plot(slope_x_pp12, gc[1].0)];
        let slope_y_s23 = Self::linefit(b1, b2);
        logging::info(&format!("quad_gantry_level af: {}, bf: {}", slope_y_s01.repr(), slope_y_s23.repr()));
        // Calculate z height of each stepper
        let z_height = [
            Self::plot(slope_y_s01, gc[0].1),
            Self::plot(slope_y_s01, gc[1].1),
            Self::plot(slope_y_s23, gc[1].1),
            Self::plot(slope_y_s23, gc[0].1),
        ];

        let apos: Vec<String> = ["z", "z1", "z2", "z3"].iter().zip(&z_height).map(|(n, z)| format!("{}: {}", n, pct_06f(*z))).collect();
        self.gcode.respond_info(&format!("Actuator Positions:\n{}", apos.join(" ")), true);

        let z_ave = crate::pyfmt::py_sum(z_height.iter().copied()) / z_height.len() as f64;
        self.gcode.respond_info(&format!("Average: {:.6}", z_ave), true);
        let z_adjust: Vec<f64> = z_height.iter().map(|z| z_ave - z).collect();

        let adjust_max = z_adjust[1..].iter().fold(z_adjust[0], |a, &b| py_max(a, b));
        if adjust_max > self.max_adjust {
            return Err(KError::command(format!(
                "Aborting quad_gantry_level required adjustment {:.6} is greater than max_adjust {:.6}",
                adjust_max, self.max_adjust
            )));
        }

        let speed = self.probe_helper().get_lift_speed(None)?;
        self.z_helper.adjust_steppers(&z_adjust, speed)?;
        Ok(self.z_status.check_retry_result(self.retry_helper.check_retry(&z_positions)?))
    }

    fn linefit(p1: [f64; 2], p2: [f64; 2]) -> Line {
        if p1[1] == p2[1] {
            // Straight line
            return Line { m: 0.0, b: p1[1], int_m: true };
        }
        let m = (p2[1] - p1[1]) / (p2[0] - p1[0]);
        let b = p1[1] - m * p1[0];
        Line { m, b, int_m: false }
    }

    fn plot(f: Line, x: f64) -> f64 {
        f.m * x + f.b
    }
}

/// "%06f" % v: width 6, zero padded, 6 decimals
fn pct_06f(v: f64) -> String {
    crate::pyformat::percent_format("%06f", &PyVal::Float(v)).unwrap_or_default()
}

impl PrinterObject for QuadGantryLevel {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(self.z_status.get_status())
    }
    fn teardown(&self) {
        *self.probe_helper.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(QuadGantryLevel::new(config)?)
}
