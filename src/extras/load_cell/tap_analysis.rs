// Tap Analysis
// (port of klippy/extras/load_cell/tap_analysis.py)
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::trapq::PullMove;
use crate::error::{KError, KResult};
use crate::extras::load_cell::load_cell::{ApiClientHelper, ForceSample};
use crate::extras::motion_report::{PrinterMotionReport, NEVER_TIME};
use crate::gcode::GCodeCommand;
use crate::logging;
use crate::mathutil::np_std;
use crate::printer::Printer;
use crate::pyfmt::py_str_repr;
use crate::pyval::PyVal;
use crate::IndexMap;
use std::f64::consts::PI;
use std::rc::Rc;

/// math.degrees: x / (pi / 180)
fn degrees(x: f64) -> f64 {
    x / (PI / 180.0)
}

/// a TapValidationError, or any other exception raised while analysing
pub enum TapError {
    Validation(TapValidationError),
    Other(KError),
}

impl From<KError> for TapError {
    fn from(e: KError) -> Self {
        TapError::Other(e)
    }
}

#[derive(Clone, Debug)]
pub struct TapValidationError {
    pub error_code: String,
    pub message: String,
}

impl TapValidationError {
    pub fn new(error_code: &str, message: impl Into<String>) -> TapError {
        TapError::Validation(TapValidationError { error_code: error_code.into(), message: message.into() })
    }

    fn to_dict(&self) -> PyVal {
        let d = crate::pydict! {
            "error_code" => PyVal::Str(self.error_code.clone()),
            "message" => PyVal::Str(self.message.clone()),
        };
        PyVal::Dict(d)
    }
}

/// a tap classifier module (TapClassifierModule.classify does nothing)
pub trait TapClassifier {
    fn classify(&self, tap_analysis: &mut TapAnalysis, gcmd: Option<&GCodeCommand>) -> Result<(), TapError>;
}

// Capture and preserve a Trapezoidal Move as a python type
#[derive(Clone)]
pub struct TrapezoidalMove {
    pub print_time: f64,
    pub move_t: f64,
    pub start_v: f64,
    pub accel: f64,
    pub start_x: f64,
    pub start_y: f64,
    pub start_z: f64,
    pub x_r: f64,
    pub y_r: f64,
    pub z_r: f64,
}

impl TrapezoidalMove {
    pub fn new(m: &PullMove, time_offset: f64) -> Self {
        TrapezoidalMove {
            print_time: m.print_time - time_offset,
            move_t: m.move_t,
            start_v: m.start_v,
            accel: m.accel,
            start_x: m.start_x,
            start_y: m.start_y,
            start_z: m.start_z,
            x_r: m.x_r,
            y_r: m.y_r,
            z_r: m.z_r,
        }
    }

    pub fn to_dict(&self) -> PyVal {
        let mut d = IndexMap::default();
        for (k, v) in [
            ("print_time", self.print_time),
            ("move_t", self.move_t),
            ("start_v", self.start_v),
            ("accel", self.accel),
            ("start_x", self.start_x),
            ("start_y", self.start_y),
            ("start_z", self.start_z),
            ("x_r", self.x_r),
            ("y_r", self.y_r),
            ("z_r", self.z_r),
        ] {
            d.insert(k.to_string().into(), PyVal::Float(v));
        }
        PyVal::Dict(d)
    }
}

// point on a time/force graph
#[derive(Clone, Copy, Debug)]
pub struct ForcePoint {
    pub time: f64,
    pub force: f64,
}

impl ForcePoint {
    fn to_dict(self) -> PyVal {
        let d = crate::pydict! {
            "time" => PyVal::Float(self.time),
            "force" => PyVal::Float(self.force),
        };
        PyVal::Dict(d)
    }
}

// slope/intercept based line where x is time and y is force
#[derive(Clone, Copy, Debug)]
pub struct ForceLine {
    pub slope: f64,
    pub intercept: f64,
}

impl ForceLine {
    // measure angles between lines at the 25g = 0.1s (100ms) scale
    // Note: this is the same scale used by Prusa
    // returns +/- 0-180. Positive values represent clockwise rotation
    fn angle(&self, line: &ForceLine) -> f64 {
        let (time_scale, gram_scale) = (0.1, 25.0);
        let scaling_factor = time_scale / gram_scale;
        let this_slope = self.slope * scaling_factor;
        let other_slope = line.slope * scaling_factor;
        let radians = this_slope.atan2(1.0) - other_slope.atan2(1.0);
        degrees(radians)
    }

    pub fn find_force(&self, time: f64) -> f64 {
        self.slope * time + self.intercept
    }

    fn intersection(&self, line: &ForceLine) -> ForcePoint {
        let numerator = -self.intercept + line.intercept;
        let denominator = self.slope - line.slope;
        // lines are parallel, will not intersect
        if denominator == 0.0 {
            // to get debuggable data we want to return a clearly bad value here
            return ForcePoint { time: 0.0, force: 0.0 };
        }
        let intersection_time = numerator / denominator;
        let intersection_force = self.find_force(intersection_time);
        ForcePoint { time: intersection_time, force: intersection_force }
    }

    fn to_dict(self) -> PyVal {
        let d = crate::pydict! {
            "slope" => PyVal::Float(self.slope),
            "intercept" => PyVal::Float(self.intercept),
        };
        PyVal::Dict(d)
    }
}

/// a python index into a sequence of `len` items
fn py_idx(i: i64, len: usize) -> KResult<usize> {
    let j = if i < 0 { i + len as i64 } else { i };
    if j < 0 || j >= len as i64 {
        return Err(KError::Py("IndexError".into(), "index out of range".into()));
    }
    Ok(j as usize)
}

/// the range of a python slice [start:end] of `len` items
fn py_slice(start: i64, end: i64, len: usize) -> std::ops::Range<usize> {
    let norm = |i: i64| {
        let j = if i < 0 { i + len as i64 } else { i };
        j.clamp(0, len as i64) as usize
    };
    let (s, e) = (norm(start), norm(end));
    s..e.max(s)
}

/// np.cumsum
fn cumsum(v: impl Iterator<Item = f64>) -> Vec<f64> {
    let mut acc = 0.0;
    v.map(|x| {
        acc += x;
        acc
    })
    .collect()
}

//#########################
// Math Support Functions

// helper class for working with a time/force graph
// work with subsections to find elbows and best fit lines
pub struct ForceGraph {
    pub time: Vec<f64>,
    pub force: Vec<f64>,
    pub cum_x: Vec<f64>,
    pub cum_y: Vec<f64>,
    pub cum_xx: Vec<f64>,
    pub cum_xy: Vec<f64>,
    pub cum_yy: Vec<f64>,
}

impl ForceGraph {
    pub fn new(time: Vec<f64>, force: Vec<f64>) -> Self {
        // linear implementation:
        let cum_x = cumsum(time.iter().copied());
        let cum_y = cumsum(force.iter().copied());
        let cum_xx = cumsum(time.iter().map(|t| t * t));
        let cum_xy = cumsum(time.iter().zip(&force).map(|(t, f)| t * f));
        let cum_yy = cumsum(force.iter().map(|f| f * f));
        ForceGraph { time, force, cum_x, cum_y, cum_xx, cum_xy, cum_yy }
    }

    fn segment_sum(arr: &[f64], start_idx: i64, end_idx: i64) -> KResult<f64> {
        let prior_sum = if start_idx == 0 { 0.0 } else { arr[py_idx(start_idx - 1, arr.len())?] };
        Ok(arr[py_idx(end_idx, arr.len())?] - prior_sum)
    }

    /// Compute slope/intercept and RSS for a segment of the data
    fn least_squares(&self, start_idx: i64, end_idx: i64) -> KResult<([f64; 2], f64)> {
        let n = (end_idx - start_idx) + 1;
        if n < 2 {
            return Err(KError::Py("ValueError".into(), "Error: fewer than 2 points used".into()));
        }
        let n = n as f64;
        let sum_x = Self::segment_sum(&self.cum_x, start_idx, end_idx)?;
        let sum_y = Self::segment_sum(&self.cum_y, start_idx, end_idx)?;
        let sum_xx = Self::segment_sum(&self.cum_xx, start_idx, end_idx)?;
        let sum_xy = Self::segment_sum(&self.cum_xy, start_idx, end_idx)?;
        let sum_yy = Self::segment_sum(&self.cum_yy, start_idx, end_idx)?;

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-10 {
            // replicate Numpy behaviour for all x values being equal
            let mean_y = sum_y / n;
            let slope = 0.0;
            let intercept = mean_y;
            // RSS is just variance in y
            let rss = sum_yy - n * mean_y * mean_y;
            return Ok(([slope, intercept], crate::pyfmt::py_max(0.0, rss)));
        }
        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;
        let rss = sum_yy - 2.0 * slope * sum_xy - 2.0 * intercept * sum_y + slope * slope * sum_xx + 2.0 * slope * intercept * sum_x + n * intercept * intercept;
        Ok(([slope, intercept], crate::pyfmt::py_max(0.0, rss)))
    }

    // search exhaustively for the 2 lines that best fit the data
    // return the elbow index
    pub fn find_elbow(&self, start_idx: i64, end_idx: i64) -> KResult<i64> {
        let mut best_error = f64::INFINITY;
        let mut best_fit_index = -1;
        for i in (1 + start_idx)..(end_idx - 1) {
            let (_, r1) = self.least_squares(start_idx, i)?;
            let (_, r2) = self.least_squares(i + 1, end_idx)?;
            let error = r1 + r2;
            if error < best_error {
                best_error = error;
                best_fit_index = i;
            }
        }
        // the index returns is the first point in the second line
        Ok(best_fit_index)
    }

    // finds the index nearest to a time
    pub fn index_near(&self, instant: f64) -> i64 {
        // np.searchsorted (side="left")
        let idx = self.time.partition_point(|t| *t < instant) as i64;
        idx.min(self.time.len() as i64 - 1)
    }

    // construct a line from 2 points
    fn points_to_line(a: ForcePoint, b: ForcePoint) -> KResult<ForceLine> {
        let dt = b.time - a.time;
        if dt == 0.0 {
            return Err(KError::Py("ZeroDivisionError".into(), "float division by zero".into()));
        }
        let slope = (b.force - a.force) / dt;
        let intercept = a.force - (slope * a.time);
        Ok(ForceLine { slope, intercept })
    }

    // construct a line using a subset of the graph
    fn line(&self, start_idx: i64, end_idx: i64) -> KResult<ForceLine> {
        let (params, _rss) = self.least_squares(start_idx, end_idx)?;
        Ok(ForceLine { slope: params[0], intercept: params[1] })
    }

    // given a line and a range, calculate the standard deviation of the noise
    pub fn noise_std(&self, start_idx: i64, end_idx: i64, line: &ForceLine) -> f64 {
        let r = py_slice(start_idx, end_idx, self.force.len());
        let noise: Vec<f64> = r.map(|i| self.force[i] - line.find_force(self.time[i])).collect();
        np_std(&noise)
    }

    // true if the reference force won't be confused for noise in the graph chunk
    // reference force must be more than 3 standard deviations away from the line
    // at the reference index
    pub fn is_clear_signal(&self, start_idx: i64, end_idx: i64, line: &ForceLine, reference_idx: i64, force_idx: i64) -> KResult<bool> {
        let noise = self.noise_std(start_idx, end_idx, line);
        let noise_3_std = noise * 3.0;
        let base_force = line.find_force(self.time[py_idx(reference_idx, self.time.len())?]);
        Ok((base_force - self.force[py_idx(force_idx, self.force.len())?]).abs() > noise_3_std)
    }

    // return the first index that exceeds the median force between
    // start and end index
    pub fn split_by_force(&self, start_idx: i64, end_idx: i64) -> KResult<Option<i64>> {
        let start_f = self.force[py_idx(start_idx, self.force.len())?];
        let end_f = self.force[py_idx(end_idx, self.force.len())?];
        let median_f = (start_f + end_f) / 2.0;
        let mut scan: Vec<i64> = (start_idx..end_idx).collect();
        // if force is ascending, swap the scan direction
        if start_f > end_f {
            scan.reverse();
        }
        for i in scan {
            if self.force[py_idx(i, self.force.len())?] > median_f {
                return Ok(Some(i));
            }
        }
        Ok(None)
    }

    // break a tap event down into 6 points and 5 lines:
    //           |*----*\
    //           |       \
    //    *-----*|        \*-----*
    fn tap_decompose(
        &self,
        homing_end_time: f64,
        pullback_start_time: f64,
        pullback_cruise_time: f64,
        pullback_cruise_duration: f64,
    ) -> KResult<([ForceLine; 5], i64, i64)> {
        let homing_end_idx = self.index_near(homing_end_time);
        // use the pullback duration to trim the amount of approach data used
        let homing_start_time = homing_end_time - pullback_cruise_duration;
        let homing_start_idx = self.index_near(homing_start_time);
        // locate the point where the probe made contact with the bed
        let contact_elbow_idx = self.find_elbow(homing_start_idx, homing_end_idx)?;

        let pullback_start_idx = self.index_near(pullback_start_time);
        let pullback_cruise_idx = self.index_near(pullback_cruise_time);
        // limit use of additional data after the pullback move ends
        let pullback_end_time = pullback_cruise_time + (pullback_cruise_duration * 1.5);
        let pullback_end_idx = self.index_near(pullback_end_time);

        // l1 is the approach line
        let l1 = self.line(homing_start_idx, contact_elbow_idx)?;
        // sometime after contact_elbow_idx is the peak force and the start of
        // the dwell line
        let dwell_end = self.time[py_idx(contact_elbow_idx, self.time.len())?] + pullback_cruise_duration;
        let dwell_end_idx = pullback_start_idx.min(self.index_near(dwell_end));
        let dwell_start_idx = self.find_elbow(contact_elbow_idx, dwell_end_idx)?;
        // l2 is the compression line
        // also +1 the last index in case its sequential [1, 2]
        let l2 = self.line(contact_elbow_idx, dwell_start_idx + 1)?;
        // l3 is the dwell line
        let l3 = self.line(dwell_start_idx, pullback_start_idx)?;

        // find the approximate elbow location
        let mut break_contact_idx = self.find_elbow(pullback_cruise_idx, pullback_end_idx)?;
        // l5 is the line after decompression ends
        let mut l5 = self.line(break_contact_idx, pullback_end_idx)?;
        // split the points between the elbow and the start of movement by force
        let midpoint_idx = self.split_by_force(pullback_cruise_idx, break_contact_idx)?;
        // elbow finding success depends on their being good signal-to-noise
        // this checks if there will be enough clear data to analyze
        let mut use_curve_optimization = false;
        if let Some(midpoint_idx) = midpoint_idx {
            let clear_dwell = self.is_clear_signal(dwell_start_idx, dwell_end_idx, &l3, dwell_end_idx, midpoint_idx - 1)?;
            let clear_decomp = self.is_clear_signal(break_contact_idx, pullback_end_idx, &l5, break_contact_idx, midpoint_idx)?;
            use_curve_optimization = clear_dwell && clear_decomp;
        }
        let l4 = match midpoint_idx {
            Some(midpoint_idx) if use_curve_optimization => {
                // perform iterative refinement
                let l4_start = self.line(pullback_cruise_idx, midpoint_idx)?;
                // real break contact index
                break_contact_idx = self.find_elbow(midpoint_idx, pullback_end_idx)?;
                let l4_end = self.line(midpoint_idx, break_contact_idx)?;
                l5 = self.line(break_contact_idx, pullback_end_idx)?;
                // a synthetic l4 is built from 2 points:
                Self::points_to_line(l4_start.intersection(&l3), l4_end.intersection(&l5))?
            }
            _ => {
                // noise is too high, don't use the curve optimization
                let l4 = self.line(pullback_cruise_idx, break_contact_idx)?;
                // log for user debugging
                logging::info("TapAnalysis: curve optimization not used");
                l4
            }
        };

        Ok(([l1, l2, l3, l4, l5], homing_start_idx, pullback_end_idx))
    }
}

// Move index constants. The PROBE_START move may be deleted from the trapq if
// the probe takes longer than 30s. Indexing from the end of the list
// is always consistent:
const PROBE_CRUISE: i64 = -5;
const PROBE_HALT: i64 = -4;
const PULLBACK_START: i64 = -3;
const PULLBACK_CRUISE: i64 = -2;
const PULLBACK_END: i64 = -1;

pub struct TapAnalysis {
    pub is_valid: bool,
    pub tap_pos: Option<(f64, f64, f64)>,
    tap_points: Vec<ForcePoint>,
    tap_lines: Vec<ForceLine>,
    pub tap_angles: Vec<f64>,
    pub elapsed: f64,
    pub collection_time: f64,
    error: Option<TapValidationError>,
    pub home_end_time: Option<f64>,
    pub pullback_start_time: Option<f64>,
    pub pullback_end_time: Option<f64>,
    pub pullback_cruise_time: Option<f64>,
    pub pullback_duration: Option<f64>,
    pub homing_start_index: i64,
    pub pullback_end_index: i64,
    pub time_offset: f64,
    pub time: Vec<f64>,
    pub force: Vec<f64>,
    force_graph: ForceGraph,
    pub trigger_force: i64,
    moves: Vec<TrapezoidalMove>,
}

impl TapAnalysis {
    fn new(samples: &[ForceSample], trigger_force: i64) -> KResult<Self> {
        let first = samples.first().ok_or_else(|| KError::Py("IndexError".into(), "too many indices for array: array is 1-dimensional, but 2 were indexed".into()))?;
        let time_offset = first.time;
        let time: Vec<f64> = samples.iter().map(|s| s.time - time_offset).collect();
        let force: Vec<f64> = samples.iter().map(|s| s.force.unwrap_or(f64::NAN)).collect();
        let force_graph = ForceGraph::new(time.clone(), force.clone());
        Ok(TapAnalysis {
            is_valid: false,
            tap_pos: None,
            tap_points: vec![],
            tap_lines: vec![],
            tap_angles: vec![],
            elapsed: 0.0,
            collection_time: 0.0,
            error: None,
            home_end_time: None,
            pullback_start_time: None,
            pullback_end_time: None,
            pullback_cruise_time: None,
            pullback_duration: None,
            homing_start_index: 0,
            pullback_end_index: -1,
            time_offset,
            time,
            force,
            force_graph,
            trigger_force,
            moves: vec![],
        })
    }

    fn move_dist(m: &TrapezoidalMove, print_time: f64) -> f64 {
        let move_t = m.move_t;
        let move_time = crate::pyfmt::py_max(0.0, crate::pyfmt::py_min(move_t, print_time - m.print_time));
        (m.start_v + (0.5 * m.accel) * move_time) * move_time
    }

    fn move_pos(m: &TrapezoidalMove, dist: f64) -> (f64, f64, f64) {
        (m.start_x + (m.x_r * dist), m.start_y + (m.y_r * dist), m.start_z + (m.z_r * dist))
    }

    // get an XYZ position from the toolhead position history
    // positions before/after the captured history are assumed to be stationary
    pub fn get_toolhead_position(&self, print_time: f64) -> KResult<(f64, f64, f64)> {
        for (i, m) in self.moves.iter().enumerate() {
            let start_time = m.print_time;
            // time before first move, assume printer was stationary
            if i == 0 && print_time < start_time {
                return Ok(Self::move_pos(m, 0.0));
            }
            let end_time = if i < self.moves.len() - 1 { self.moves[i + 1].print_time } else { f64::INFINITY };
            if start_time <= print_time && print_time < end_time {
                // we have found the move
                let dist = Self::move_dist(m, print_time);
                return Ok(Self::move_pos(m, dist));
            }
        }
        // time is after last move, assume printer was stationary
        let m = self.moves.last().ok_or_else(|| KError::Py("IndexError".into(), "list index out of range".into()))?;
        let dist = Self::move_dist(m, print_time);
        Ok(Self::move_pos(m, dist))
    }

    pub fn log_trapq(&self) {
        logging::info("TapAnalysis - Trapezoidal Movement Queue contents:");
        for (i, m) in self.moves.iter().enumerate() {
            logging::info(&format!("Move {}: {}", i, m.to_dict().repr()));
        }
    }

    fn mv(&self, idx: i64) -> KResult<usize> {
        py_idx(idx, self.moves.len())
    }

    // adjust move_t of PROBE_CRUISE to match the toolhead position of PROBE_HALT
    fn recalculate_homing_end(&mut self) -> Result<f64, TapError> {
        let hi = self.mv(PROBE_CRUISE)?;
        let halt_z = self.moves[self.mv(PROBE_HALT)?].start_z;
        let homing_move = &mut self.moves[hi];
        // acceleration should be 0! This is the 'coasting' move:
        if homing_move.accel != 0.0 {
            return Err(TapValidationError::new("COASTING_MOVE_ACCELERATION", "Probing move is accelerating/decelerating which is invalid"));
        }
        // how long did it take to get to end_z?
        if homing_move.start_v == 0.0 {
            return Err(KError::Py("ZeroDivisionError".into(), "float division by zero".into()).into());
        }
        homing_move.move_t = ((halt_z - homing_move.start_z) / homing_move.start_v).abs();
        Ok(homing_move.print_time + homing_move.move_t)
    }

    // extract and save TrapQueue moves
    fn extract_trapq(&mut self, printer: &Printer) -> KResult<()> {
        let mr = printer.lookup::<PrinterMotionReport>("motion_report")?;
        let trapq = mr.get_dump_trapq("toolhead").ok_or_else(|| KError::Py("KeyError".into(), py_str_repr("toolhead")))?;
        let moves = trapq.extract_trapq(self.time[0] + self.time_offset, NEVER_TIME);
        for m in &moves {
            self.moves.push(TrapezoidalMove::new(m, self.time_offset));
        }
        Ok(())
    }

    // perform analysis, throws exceptions
    fn analyze(&mut self, printer: &Printer) -> Result<(), TapError> {
        self.extract_trapq(printer)?;
        let num_moves = self.moves.len();
        if num_moves < 5 {
            return Err(TapValidationError::new("TOO_FEW_PROBING_MOVES", "5 Probing moves expected but there were fewer"));
        } else if num_moves > 6 {
            return Err(TapValidationError::new("TOO_MANY_PROBING_MOVES", "More than 6 probing moves were found during the tap"));
        }
        self.home_end_time = Some(self.recalculate_homing_end()?);
        let pullback_start = self.moves[self.mv(PULLBACK_START)?].print_time;
        let end = &self.moves[self.mv(PULLBACK_END)?];
        let pullback_end = end.print_time + end.move_t;
        self.pullback_start_time = Some(pullback_start);
        self.pullback_end_time = Some(pullback_end);
        self.pullback_cruise_time = Some(self.moves[self.mv(PULLBACK_CRUISE)?].print_time);
        self.pullback_duration = Some(pullback_end - pullback_start);
        let (lines, i, j) = self.force_graph.tap_decompose(
            self.home_end_time.unwrap_or(0.0),
            pullback_start,
            self.pullback_cruise_time.unwrap_or(0.0),
            pullback_end - pullback_start,
        )?;
        self.homing_start_index = i;
        self.pullback_end_index = j;
        self.set_tap_lines(lines.to_vec())?;
        self.validate_order()?;
        self.validate_tap_shape()?;
        self.validate_break_contact_time()?;
        self.is_valid = true;
        Ok(())
    }

    // validate that a set of ForcePoint objects are in chronological order
    fn validate_order(&self) -> Result<(), TapError> {
        let p = &self.tap_points;
        if !(p[0].time < p[1].time && p[1].time < p[2].time && p[2].time < p[3].time && p[3].time < p[4].time && p[4].time < p[5].time) {
            return Err(TapValidationError::new("TAP_CHRONOLOGY", "Tap points are out of chronological order"));
        }
        Ok(())
    }

    // Validate that the rotations between lines form a tap shape
    fn validate_tap_shape(&self) -> Result<(), TapError> {
        let (a1, a2, a3, a4) = (self.tap_angles[0], self.tap_angles[1], self.tap_angles[2], self.tap_angles[3]);
        // with two polarities there are 2 valid tap shapes:
        if !((a1 > 0.0 && a2 < 0.0 && a3 < 0.0 && a4 > 0.0) || (a1 < 0.0 && a2 > 0.0 && a3 > 0.0 && a4 < 0.0)) {
            return Err(TapValidationError::new("TAP_SHAPE_INVALID", "Force data does not form a tap shape"));
        }
        Ok(())
    }

    // The proposed break contact point must fall inside the
    // first 3/4s of the pullback move
    fn validate_break_contact_time(&self) -> Result<(), TapError> {
        let break_contact_time = self.tap_points[4].time;
        let start_t = self.pullback_start_time.unwrap_or(0.0);
        let end_t = self.pullback_end_time.unwrap_or(0.0);
        let safety_margin = (end_t - start_t) / 4.0;
        if break_contact_time < start_t {
            return Err(TapValidationError::new("TAP_BREAK_CONTACT_TOO_EARLY", "Tap break-contact time is too early"));
        } else if break_contact_time > end_t {
            return Err(TapValidationError::new("TAP_BREAK_CONTACT_TOO_LATE", "Tap break-contact time is too late"));
        } else if break_contact_time > (end_t - safety_margin) {
            return Err(TapValidationError::new("TAP_PULLBACK_TOO_SHORT", "Tap break-contact time is too late, pullback move may be too short"));
        }
        Ok(())
    }

    fn calculate_points(&mut self) -> KResult<()> {
        let l = &self.tap_lines;
        let t0 = self.time[py_idx(self.homing_start_index, self.time.len())?];
        let p0 = ForcePoint { time: t0, force: l[0].find_force(t0) };
        let p1 = l[0].intersection(&l[1]);
        let p2 = l[1].intersection(&l[2]);
        let p3 = l[2].intersection(&l[3]);
        let p4 = l[3].intersection(&l[4]);
        let t5 = self.time[py_idx(self.pullback_end_index, self.time.len())?];
        let p5 = ForcePoint { time: t5, force: l[4].find_force(t5) };
        self.tap_points = vec![p0, p1, p2, p3, p4, p5];
        Ok(())
    }

    fn calculate_angles(&mut self) {
        let l = &self.tap_lines;
        self.tap_angles = vec![l[0].angle(&l[1]), l[1].angle(&l[2]), l[2].angle(&l[3]), l[3].angle(&l[4])];
    }

    pub fn get_trigger_force(&self) -> i64 {
        self.trigger_force
    }

    pub fn get_tap_points(&self) -> &[ForcePoint] {
        &self.tap_points
    }

    pub fn get_tap_pos(&self) -> Option<(f64, f64, f64)> {
        self.tap_pos
    }

    pub fn get_tap_lines(&self) -> &[ForceLine] {
        &self.tap_lines
    }

    // Allow TapClassifier modules to overwrite the lines
    // This also causes the tap points, angles and tap pos to be recalculated
    pub fn set_tap_lines(&mut self, tap_lines: Vec<ForceLine>) -> KResult<()> {
        self.tap_lines = tap_lines;
        self.calculate_points()?;
        self.calculate_angles();
        let break_contact_time = self.tap_points[4].time;
        self.tap_pos = Some(self.get_toolhead_position(break_contact_time)?);
        Ok(())
    }

    pub fn is_valid(&self) -> bool {
        self.is_valid
    }

    // convert to dictionary for JSON encoder
    pub fn to_dict(&self) -> PyVal {
        let floats = |v: &[f64]| PyVal::List(v.iter().map(|f| PyVal::Float(*f)).collect());
        let opt = |v: Option<f64>| v.map(PyVal::Float).unwrap_or(PyVal::None);
        let mut d = crate::pydict! {
            "time" => floats(&self.time),
            "force" => floats(&self.force),
            "tap_points" => PyVal::List(self.tap_points.iter().map(|p| p.to_dict()).collect()),
            "tap_lines" => PyVal::List(self.tap_lines.iter().map(|l| l.to_dict()).collect()),
            "tap_angles" => floats(&self.tap_angles),
        };
        d.insert(
            "tap_pos".into(),
            self.tap_pos.map(|(x, y, z)| PyVal::Tuple(vec![PyVal::Float(x), PyVal::Float(y), PyVal::Float(z)])).unwrap_or(PyVal::None),
        );
        d.insert("moves".into(), PyVal::List(self.moves.iter().map(|m| m.to_dict()).collect()));
        d.insert("home_end_time".into(), opt(self.home_end_time));
        d.insert("pullback_start_time".into(), opt(self.pullback_start_time));
        d.insert("pullback_end_time".into(), opt(self.pullback_end_time));
        d.insert("collection_time".into(), PyVal::Float(self.collection_time));
        d.insert("elapsed".into(), PyVal::Float(self.elapsed));
        d.insert("is_valid".into(), PyVal::Bool(self.is_valid));
        d.insert("error".into(), self.error.as_ref().map(|e| e.to_dict()).unwrap_or(PyVal::None));
        PyVal::Dict(d)
    }
}

// Orchestrate TapAnalysis and TapClassifier. Handle timing, error capture,
// event broadcast, clients & logging
pub struct TapAnalysisHelper {
    pub printer: Rc<Printer>,
    tap_classifier: Rc<dyn TapClassifier>,
    clients: Rc<ApiClientHelper<PyVal>>,
}

impl TapAnalysisHelper {
    pub fn new(printer: &Rc<Printer>, name: &str, tap_classifier: Rc<dyn TapClassifier>) -> KResult<Self> {
        // webhooks support
        let clients = ApiClientHelper::<PyVal>::new(printer);
        let mut header = IndexMap::default();
        header.insert("header".into(), PyVal::List(vec![PyVal::Str("probe_tap_event".into())]));
        clients.add_mux_endpoint("load_cell_probe/dump_taps", "load_cell_probe", name, PyVal::Dict(header))?;
        Ok(TapAnalysisHelper { printer: printer.clone(), tap_classifier, clients })
    }

    pub fn analyze(&self, samples: &[ForceSample], trigger_force: i64, collection_time: f64, gcmd: Option<&GCodeCommand>) -> KResult<TapAnalysis> {
        let t_start = std::time::Instant::now();
        let mut tap_analysis = TapAnalysis::new(samples, trigger_force)?;
        match tap_analysis.analyze(&self.printer) {
            Ok(()) => {}
            Err(TapError::Validation(ve)) => {
                tap_analysis.is_valid = false;
                tap_analysis.error = Some(ve);
                tap_analysis.log_trapq();
            }
            Err(TapError::Other(e)) => return Err(e),
        }
        // tap classifier always gets to process the data
        match self.tap_classifier.classify(&mut tap_analysis, gcmd) {
            Ok(()) => {}
            Err(TapError::Validation(ve)) => {
                tap_analysis.is_valid = false;
                tap_analysis.error = Some(ve);
            }
            Err(TapError::Other(e)) => return Err(e),
        }
        // total elapsed time for all calculations
        tap_analysis.elapsed = t_start.elapsed().as_secs_f64();
        tap_analysis.collection_time = collection_time;
        // broadcast tap event data:
        let mut msg = IndexMap::default();
        msg.insert("tap".into(), tap_analysis.to_dict());
        self.clients.send(&PyVal::Dict(msg));
        Self::log_errors(&tap_analysis);
        Ok(tap_analysis)
    }

    // log errors to event log
    fn log_errors(tap_analysis: &TapAnalysis) {
        // if the tap is valid, don't log any errors
        if tap_analysis.is_valid() {
            return;
        }
        // log errors
        if let Some(ve) = &tap_analysis.error {
            logging::info(&format!("Bad tap detected: {} - {}", ve.error_code, ve.message));
        }
    }
}
