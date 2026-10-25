// Helper script to adjust bed screws tilt using Z probe
// (port of klippy/extras/screws_tilt_adjust.py)
//
// Copyright (C) 2019  Rui Caridade <rui.mcbc@gmail.com>
// Copyright (C) 2021  Matthew Lloyd <github@matthewlloyd.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::extras::probe::{FinalizeResult, ProbePointsHelper};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_float_repr, py_round};
use crate::pyval::{PyDict, PyVal};
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const CMD_SCREWS_TILT_CALCULATE_HELP: &str = "Tool to help adjust bed leveling screws by calculating the number of turns to level it.";

const THREADS: [(&str, i64); 10] = [
    ("CW-M3", 0),
    ("CCW-M3", 1),
    ("CW-M4", 2),
    ("CCW-M4", 3),
    ("CW-M5", 4),
    ("CCW-M5", 5),
    ("CW-M6", 6),
    ("CCW-M6", 7),
    ("CW-M8", 8),
    ("CCW-M8", 9),
];

// Factors used for CW-M3, CCW-M3, CW-M4, CCW-M4, CW-M5, CCW-M5, CW-M6
// and CCW-M6
fn threads_factor(thread: i64) -> f64 {
    match thread {
        0 | 1 => 0.5,
        2 | 3 => 0.7,
        4 | 5 => 0.8,
        6 | 7 => 1.0,
        8 | 9 => 1.25,
        _ => 0.5,
    }
}

pub struct ScrewsTiltAdjust {
    screws: Vec<([f64; 2], String)>,
    results: RefCell<PyDict>,
    max_diff: Cell<Option<f64>>,
    max_diff_error: Cell<bool>,
    thread: i64,
    direction: RefCell<Option<String>>,
    probe_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    gcode: Rc<GCodeDispatch>,
}

impl ScrewsTiltAdjust {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mut screws = vec![];
        // Read config
        for i in 0..99 {
            let prefix = format!("screw{}", i + 1);
            if config.get_opt(&prefix)?.is_none() {
                break;
            }
            let c = config.getfloatlist(&prefix, Dflt::Required, ",", Some(2))?.unwrap_or_default();
            let screw_coord = [c[0], c[1]];
            let screw_name = format!("screw at {:.3},{:.3}", screw_coord[0], screw_coord[1]);
            let screw_name = config.get(&format!("{}_name", prefix), screw_name.as_str())?;
            screws.push((screw_coord, screw_name));
        }
        if screws.len() < 3 {
            return Err(KError::config("screws_tilt_adjust: Must have at least three screws"));
        }
        let thread = config.getchoice("screw_thread", &THREADS, "CW-M3")?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let points: Vec<(f64, f64)> = screws.iter().map(|(c, _)| (c[0], c[1])).collect();
        let s = Rc::new(ScrewsTiltAdjust {
            screws,
            results: RefCell::new(IndexMap::default()),
            max_diff: Cell::new(None),
            max_diff_error: Cell::new(false),
            thread,
            direction: RefCell::new(None),
            probe_helper: RefCell::new(None),
            gcode: gcode.clone(),
        });
        // Initialize ProbePointsHelper
        let w = Rc::downgrade(&s);
        let probe_helper = ProbePointsHelper::new(
            config,
            Rc::new(move |offsets, positions| match w.upgrade() {
                Some(s) => s.probe_finalize(offsets, positions),
                None => Ok(FinalizeResult::Done),
            }),
            Some(points),
            "points",
            false,
            false,
        )?;
        probe_helper.minimum_points(3)?;
        *s.probe_helper.borrow_mut() = Some(probe_helper);
        // Register command
        let w = Rc::downgrade(&s);
        gcode.register_command(
            "SCREWS_TILT_CALCULATE",
            Some(Rc::new(move |g| match w.upgrade() {
                Some(s) => s.cmd_screws_tilt_calculate(g),
                None => Ok(()),
            })),
            false,
            Some(CMD_SCREWS_TILT_CALCULATE_HELP),
        )?;
        Ok(s)
    }

    fn cmd_screws_tilt_calculate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.max_diff.set(gcmd.get_float_opt("MAX_DEVIATION", crate::configfile::Lim::new())?);
        // Option to force all turns to be in the given direction (CW or CCW)
        let mut direction = gcmd.get_opt("DIRECTION");
        if let Some(d) = &direction {
            let d = d.to_uppercase();
            if d != "CW" && d != "CCW" {
                return Err(gcmd.error(format!("Error on '{}': DIRECTION must be either CW or CCW", gcmd.get_commandline())));
            }
            direction = Some(d);
        }
        *self.direction.borrow_mut() = direction;
        let helper = self.probe_helper.borrow().clone().ok_or_else(|| KError::Other("probe helper".into()))?;
        helper.start_probe(gcmd)
    }

    fn probe_finalize(&self, _offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        self.results.borrow_mut().clear();
        self.max_diff_error.set(false);
        let is_clockwise_thread = (self.thread & 1) == 0;
        let mut screw_diff = vec![];
        // Process the read Z values
        let direction = self.direction.borrow().clone();
        let (i_base, z_base) = match &direction {
            Some(direction) => {
                // Lowest or highest screw is the base position used for comparison
                let use_max = (is_clockwise_thread && direction == "CW") || (!is_clockwise_thread && direction == "CCW");
                let zs: Vec<(usize, f64)> = positions.iter().map(|p| p[2]).enumerate().collect();
                let mut best = zs[0];
                for &(i, z) in &zs[1..] {
                    // python min()/max() keep the first of equal keys
                    if (use_max && z > best.1) || (!use_max && z < best.1) {
                        best = (i, z);
                    }
                }
                best
            }
            // First screw is the base position used for comparison
            None => (0, positions[0][2]),
        };
        // Provide the user some information on how to read the results
        self.gcode.respond_info("01:20 means 1 full turn and 20 minutes, CW=clockwise, CCW=counter-clockwise", true);
        for (i, (coord, name)) in self.screws.iter().enumerate() {
            let z = positions[i][2];
            let mut entry = IndexMap::default();
            entry.insert("z".into(), PyVal::Float(z));
            if i == i_base {
                // Show the results
                self.gcode.respond_info(&format!("{} (base) : x={:.1}, y={:.1}, z={:.5}", name, coord[0], coord[1], z), true);
                let sign = if is_clockwise_thread { "CW" } else { "CCW" };
                entry.insert("sign".into(), PyVal::from(sign));
                entry.insert("adjust".into(), PyVal::from("00:00"));
                entry.insert("is_base".into(), PyVal::Bool(true));
            } else {
                // Calculate how knob must be adjusted for other positions
                let diff = z_base - z;
                screw_diff.push(diff.abs());
                let adjust = if diff.abs() < 0.001 { 0.0 } else { diff / threads_factor(self.thread) };
                let sign = match (is_clockwise_thread, adjust >= 0.0) {
                    (true, true) | (false, false) => "CW",
                    _ => "CCW",
                };
                let adjust = adjust.abs();
                let full_turns = adjust.trunc();
                let decimal_part = adjust - full_turns;
                let minutes = py_round(decimal_part * 60.0, 0);
                let adj = format!("{:02}:{:02}", full_turns as i64, minutes.trunc() as i64);
                // Show the results
                self.gcode.respond_info(&format!("{} : x={:.1}, y={:.1}, z={:.5} : adjust {} {}", name, coord[0], coord[1], z, sign, adj), true);
                entry.insert("sign".into(), PyVal::from(sign));
                entry.insert("adjust".into(), PyVal::Str(adj));
                entry.insert("is_base".into(), PyVal::Bool(false));
            }
            self.results.borrow_mut().insert(format!("screw{}", i + 1).into(), PyVal::Dict(entry));
        }
        if let Some(max_diff) = self.max_diff.get() {
            if max_diff != 0.0 && screw_diff.iter().any(|d| *d > max_diff) {
                self.max_diff_error.set(true);
                return Err(KError::command(format!(
                    "bed level exceeds configured limits ({}mm)! Adjust screws and restart print.",
                    py_float_repr(max_diff)
                )));
            }
        }
        Ok(FinalizeResult::Done)
    }
}

impl PrinterObject for ScrewsTiltAdjust {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let d = crate::pydict! {
            "error" => PyVal::Bool(self.max_diff_error.get()),
            "max_deviation" => self.max_diff.get().map(PyVal::Float).unwrap_or(PyVal::None),
            "results" => PyVal::Dict(self.results.borrow().clone()),
        };
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.probe_helper.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ScrewsTiltAdjust::new(config)?)
}
