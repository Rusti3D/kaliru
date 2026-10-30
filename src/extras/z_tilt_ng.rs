// Mechanical bed tilt calibration with multiple Z steppers, with
// calibration and pivot point autodetection (port of klippy/extras/z_tilt_ng.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2023  Arne Jansen <arne@die-jansens.de>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The ZAdjustHelper, ZAdjustStatus and RetryHelper copies of z_tilt_ng.py
// behave like z_tilt.py's (z_positions is a required option, so the
// z_count=None branch never runs): z_tilt.rs's are used.
use crate::configfile::{ConfigWrapper, Dflt, Lim, ListParser};
use crate::error::KResult;
use crate::extras::probe::{FinalizeResult, ProbePointsHelper};
use crate::extras::z_tilt::{get_pairs, positions_repr, RetryHelper, ZAdjustHelper, ZAdjustStatus};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mathutil::{coordinate_descent, np_cross3, np_dot3, np_mean_axis0, np_solve3, np_std, np_std_axis0};
use crate::printer::{ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::py_float_repr;
use crate::pyval::PyVal;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub fn params_to_normal_form(params: &IndexMap<String, f64>, offsets: (f64, f64, f64)) -> [f64; 4] {
    let v = [offsets.0, offsets.1, params["z_adjust"]];
    let r = [1.0, 0.0, params["x_adjust"]];
    let s = [0.0, 1.0, params["y_adjust"]];
    let cp = np_cross3(&r, &s);
    [cp[0], cp[1], cp[2], np_dot3(&cp, &v)]
}

fn intersect_3_planes(p1: &[f64; 4], p2: &[f64; 4], p3: &[f64; 4]) -> KResult<[f64; 3]> {
    let a = [[p1[0], p1[1], p1[2]], [p2[0], p2[1], p2[2]], [p3[0], p3[1], p3[2]]];
    let b = [p1[3], p2[3], p3[3]];
    np_solve3(a, b).map_err(crate::kinematics::delta::math_kerror)
}

fn params_repr(p: &IndexMap<String, f64>) -> String {
    let v: Vec<String> = p.iter().map(|(k, v)| format!("'{}': {}", k, py_float_repr(*v))).collect();
    format!("{{{}}}", v.join(", "))
}

fn list_repr(v: &[f64]) -> String {
    PyVal::List(v.iter().map(|f| PyVal::Float(*f)).collect()).repr()
}

/// "%.6f, " joined, without the last separator
fn offsets_str(v: &[f64]) -> String {
    v.iter().map(|o| format!("{:.6}", o)).collect::<Vec<_>>().join(", ")
}

const CMD_Z_TILT_ADJUST_HELP: &str = "Adjust the Z tilt";
const CMD_Z_TILT_CALIBRATE_HELP: &str = "Calibrate Z tilt with additional probing points";
const CMD_Z_TILT_AUTODETECT_HELP: &str = "Autodetect pivot point of Z motors";

const AD_ADJUSTMENTS: [[f64; 3]; 7] = [
    [0.5, -0.5, -0.5], // p1 up
    [-1.0, 1.0, 0.0],  // p2 up
    [0.0, -1.0, 1.0],  // p3 up
    [0.0, 1.0, 0.0],   // p3 + p2 up
    [1.0, -1.0, 0.0],  // p3 + p1 up
    [0.0, 1.0, -1.0],  // p2 + p1 up
    [-0.5, -0.5, 0.5], // back to level
];

pub struct ZTilt {
    pub printer: Rc<Printer>,
    pub section: String,
    pub z_positions: RefCell<Vec<(f64, f64)>>,
    pub retry_helper: Rc<RetryHelper>,
    pub probe_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    pub z_offsets: RefCell<Option<Vec<f64>>>,
    pub z_status: Rc<ZAdjustStatus>,
    pub z_helper: Rc<ZAdjustHelper>,
    pub num_probe_points: usize,
    pub cal_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    pub ad_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    pub cal_conf_avg_len: i64,
    pub ad_conf_delta: f64,
    pub use_adjustments: bool,
    // calibrate / autodetect state
    pub cal_avg_len: Cell<i64>,
    pub cal_gcmd: RefCell<Option<GCodeCommand>>,
    pub cal_runs: RefCell<Vec<Vec<f64>>>,
    pub ad_delta: Cell<f64>,
    pub ad_phase: Cell<usize>,
    pub ad_params: RefCell<Vec<IndexMap<String, f64>>>,
    pub ad_gcmd: RefCell<Option<GCodeCommand>>,
    pub ad_runs: RefCell<Vec<Vec<Vec<f64>>>>,
    pub ad_points: RefCell<Vec<Vec<f64>>>,
    pub ad_error: Cell<Option<f64>>,
}

type Finalize = fn(&ZTilt, (f64, f64, f64), &[Vec<f64>]) -> KResult<FinalizeResult>;

fn finalize_cb(slot: &Rc<RefCell<Weak<ZTilt>>>, f: Finalize) -> crate::extras::probe::PointsFinalize {
    let sl = slot.clone();
    Rc::new(move |offsets, positions| match sl.borrow().upgrade() {
        Some(z) => f(&z, offsets, positions),
        None => Ok(FinalizeResult::Done),
    })
}

impl ZTilt {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let section = config.get_name().to_string();
        let z_positions = get_pairs(config, "z_positions")?;
        let z_count = z_positions.len();
        let retry_helper = Rc::new(RetryHelper::new(config, "")?);
        let slot: Rc<RefCell<Weak<ZTilt>>> = Rc::new(RefCell::new(Weak::new()));
        let probe_helper = ProbePointsHelper::new(config, finalize_cb(&slot, ZTilt::probe_finalize), None, "points", false, false)?;
        probe_helper.minimum_points(2)?;
        let z_offsets = config
            .getlists("z_offsets", Dflt::None, &[","], Some(z_count), ListParser::Float, true)?
            .map(|v| v.items().unwrap_or(&[]).iter().filter_map(|x| x.as_f64()).collect::<Vec<f64>>());
        let z_status = ZAdjustStatus::new(&printer);
        let z_helper = ZAdjustHelper::new(config, z_count);
        // probe points for calibrate/autodetect
        let mut cal_probe_points = probe_helper.get_probe_points();
        let num_probe_points = cal_probe_points.len();
        let mut cal_helper = None;
        if config.get_opt("extra_points")?.is_some() {
            let h = ProbePointsHelper::new(config, finalize_cb(&slot, ZTilt::cal_finalize), None, "extra_points", false, false)?;
            cal_probe_points.extend(h.get_probe_points());
            h.update_probe_points(cal_probe_points.clone(), 3)?;
            cal_helper = Some(h);
        }
        let ad_helper = ProbePointsHelper::new(config, finalize_cb(&slot, ZTilt::ad_finalize), None, "points", false, false)?;
        ad_helper.update_probe_points(cal_probe_points, 3)?;
        let cal_conf_avg_len = config.getint("averaging_len", 3, Lim::new().min(1))?;
        let ad_conf_delta = config.getfloat("autodetect_delta", 1.0, Lim::new().min(0.1))?;
        let use_adjustments = config.getboolean("use_adjustments", false)?;
        let has_cal = cal_helper.is_some();
        let zt = Rc::new(ZTilt {
            printer: printer.clone(),
            section,
            z_positions: RefCell::new(z_positions),
            retry_helper,
            probe_helper: RefCell::new(Some(probe_helper)),
            z_offsets: RefCell::new(z_offsets),
            z_status,
            z_helper,
            num_probe_points,
            cal_helper: RefCell::new(cal_helper),
            ad_helper: RefCell::new(Some(ad_helper)),
            cal_conf_avg_len,
            ad_conf_delta,
            use_adjustments,
            cal_avg_len: Cell::new(cal_conf_avg_len),
            cal_gcmd: RefCell::new(None),
            cal_runs: RefCell::new(vec![]),
            ad_delta: Cell::new(ad_conf_delta),
            ad_phase: Cell::new(0),
            ad_params: RefCell::new(vec![]),
            ad_gcmd: RefCell::new(None),
            ad_runs: RefCell::new(vec![]),
            ad_points: RefCell::new(vec![]),
            ad_error: Cell::new(None),
        });
        *slot.borrow_mut() = Rc::downgrade(&zt);
        // Register Z_TILT_ADJUST command
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        type ZCmd = fn(&ZTilt, &GCodeCommand) -> KResult<()>;
        let mut cmds: Vec<(&str, ZCmd, &str)> = vec![("Z_TILT_ADJUST", ZTilt::cmd_z_tilt_adjust, CMD_Z_TILT_ADJUST_HELP)];
        if has_cal {
            cmds.push(("Z_TILT_CALIBRATE", ZTilt::cmd_z_tilt_calibrate, CMD_Z_TILT_CALIBRATE_HELP));
        }
        cmds.push(("Z_TILT_AUTODETECT", ZTilt::cmd_z_tilt_autodetect, CMD_Z_TILT_AUTODETECT_HELP));
        for (name, f, desc) in cmds {
            let w = Rc::downgrade(&zt);
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(z) => f(&z, g),
                    None => Ok(()),
                })),
                false,
                Some(desc),
            )?;
        }
        Ok(zt)
    }

    fn helper(h: &RefCell<Option<Rc<ProbePointsHelper>>>) -> Rc<ProbePointsHelper> {
        h.borrow().clone().expect("probe helper")
    }

    pub fn cmd_z_tilt_adjust(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.z_status.reset();
        self.retry_helper.start(gcmd)?;
        Self::helper(&self.probe_helper).start_probe(gcmd)
    }

    pub fn perform_coordinate_descent(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> IndexMap<String, f64> {
        // Setup for coordinate descent analysis
        let z_offset = offsets.2;
        logging::info(&format!("Calculating bed tilt with: {}", positions_repr(positions)));
        let mut params = IndexMap::default();
        params.insert("x_adjust".into(), 0.0);
        params.insert("y_adjust".into(), 0.0);
        params.insert("z_adjust".into(), z_offset);
        // Perform coordinate descent
        let adjusted_height = |pos: &[f64], params: &IndexMap<String, f64>| {
            let (x, y, z) = (pos[0], pos[1], pos[2]);
            z - x * params["x_adjust"] - y * params["y_adjust"] - params["z_adjust"]
        };
        let errorfunc = |params: &IndexMap<String, f64>| {
            let mut total_error = 0.0;
            for pos in positions {
                total_error += adjusted_height(pos, params).powf(2.0);
            }
            total_error
        };
        let keys: Vec<String> = params.keys().cloned().collect();
        let new_params = coordinate_descent(&keys, &params, &errorfunc);
        // Apply results
        logging::info(&format!("Calculated bed tilt parameters: {}", params_repr(&new_params)));
        new_params
    }

    pub fn apply_adjustments(&self, offsets: (f64, f64, f64), new_params: &IndexMap<String, f64>) -> KResult<Vec<f64>> {
        let z_offset = offsets.2;
        let speed = Self::helper(&self.probe_helper).get_lift_speed(None)?;
        let x_adjust = new_params["x_adjust"];
        let y_adjust = new_params["y_adjust"];
        let z_adjust = new_params["z_adjust"] - z_offset - x_adjust * offsets.0 - y_adjust * offsets.1;
        let adjustments: Vec<f64> = self.z_positions.borrow().iter().map(|(x, y)| x * x_adjust + y * y_adjust + z_adjust).collect();
        self.z_helper.adjust_steppers(&adjustments, speed)?;
        Ok(adjustments)
    }

    pub fn probe_finalize(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        let positions: Vec<Vec<f64>> = match self.z_offsets.borrow().as_ref() {
            Some(z_offsets) => positions.iter().zip(z_offsets).map(|(p, o)| vec![p[0], p[1], p[2] - o]).collect(),
            None => positions.to_vec(),
        };
        let new_params = self.perform_coordinate_descent(offsets, &positions);
        let adjustments = self.apply_adjustments(offsets, &new_params)?;
        let zs: Vec<f64> = if self.use_adjustments { adjustments } else { positions.iter().map(|p| p[2]).collect() };
        Ok(self.z_status.check_retry_result(self.retry_helper.check_retry(&zs)?))
    }

    pub fn cmd_z_tilt_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.cal_avg_len.set(gcmd.get_int("AVGLEN", Some(self.cal_conf_avg_len), Lim::new())?);
        *self.cal_gcmd.borrow_mut() = Some(gcmd.clone());
        self.cal_runs.borrow_mut().clear();
        Self::helper(&self.cal_helper).start_probe(gcmd)
    }

    fn respond(gcmd: &RefCell<Option<GCodeCommand>>, msg: &str) {
        if let Some(g) = gcmd.borrow().as_ref() {
            g.respond_info(msg, true);
        }
    }

    fn save_msg(gcmd: &RefCell<Option<GCodeCommand>>) {
        Self::respond(gcmd, "The SAVE_CONFIG command will update the printer config\nfile with these parameters and restart the printer.");
    }

    pub fn cal_finalize(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        let avlen = self.cal_avg_len.get() as usize;
        let new_params = self.perform_coordinate_descent(offsets, positions);
        self.apply_adjustments(offsets, &new_params)?;
        self.cal_runs.borrow_mut().push(positions.iter().map(|p| p[2]).collect());
        let runs = self.cal_runs.borrow().clone();
        if runs.len() < avlen + 1 {
            return Ok(FinalizeResult::Retry);
        }
        let n = runs.len();
        let prev_error = np_std(&np_std_axis0(&runs[n - avlen - 1..n - 1]));
        let this_error = np_std(&np_std_axis0(&runs[n - avlen..]));
        Self::respond(&self.cal_gcmd, &format!("previous error: {:.6} current error: {:.6}", prev_error, this_error));
        if this_error < prev_error {
            return Ok(FinalizeResult::Retry);
        }
        let z_offsets: Vec<f64> = np_mean_axis0(&runs[n - avlen..]).iter().map(|z| z - offsets.2).collect();
        *self.z_offsets.borrow_mut() = Some(z_offsets.clone());
        let s_zoff = offsets_str(&z_offsets[..self.num_probe_points.min(z_offsets.len())]);
        Self::respond(&self.cal_gcmd, &format!("final z_offsets are: {}", s_zoff));
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        configfile.pconfig.borrow_mut().set(&self.section, "z_offsets", &s_zoff)?;
        Self::save_msg(&self.cal_gcmd);
        Ok(FinalizeResult::Done)
    }

    pub fn ad_init(&self) {
        self.ad_phase.set(0);
        self.ad_params.borrow_mut().clear();
    }

    pub fn cmd_z_tilt_autodetect(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.cal_avg_len.set(gcmd.get_int("AVGLEN", Some(self.cal_conf_avg_len), Lim::new())?);
        self.ad_delta.set(gcmd.get_float("DELTA", Some(self.ad_conf_delta), Lim::new().min(0.1))?);
        self.ad_init();
        *self.ad_gcmd.borrow_mut() = Some(gcmd.clone());
        self.ad_runs.borrow_mut().clear();
        self.ad_points.borrow_mut().clear();
        self.ad_error.set(None);
        Self::helper(&self.ad_helper).start_probe(gcmd)
    }

    pub fn ad_finalize(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        let avlen = self.cal_avg_len.get() as usize;
        let delta = self.ad_delta.get();
        let speed = Self::helper(&self.probe_helper).get_lift_speed(None)?;
        let mut new_params = self.perform_coordinate_descent(offsets, positions);
        let phase = self.ad_phase.get();
        if (1..4).contains(&phase) {
            *new_params.get_mut("z_adjust").expect("z_adjust") -= delta / 2.0;
        }
        if (4..7).contains(&phase) {
            *new_params.get_mut("z_adjust").expect("z_adjust") += delta / 2.0;
        }
        if phase == 0 {
            self.ad_points.borrow_mut().push(positions.iter().take(self.num_probe_points).map(|p| p[2]).collect());
        }
        self.ad_params.borrow_mut().push(new_params);
        let adjustments: Vec<f64> = AD_ADJUSTMENTS[phase].iter().map(|a| a * delta).collect();
        self.z_helper.adjust_steppers(&adjustments, speed)?;
        if phase < 6 {
            self.ad_phase.set(phase + 1);
            return Ok(FinalizeResult::Retry);
        }
        // calculcate results
        let p: Vec<[f64; 4]> = self.ad_params.borrow().iter().take(7).map(|pr| params_to_normal_form(pr, offsets)).collect();

        // This is how it works.
        // To find the pivot point, we take 3 planes:
        //  a) the original untilted plane
        //  b) the plane with one motor raised, on one corner opposite the
        //     one we want to determine the pivot point of
        //  c) the plane with the other motor opposite the one we want to
        //     determine the pivot point raised
        // The intersection of all 3 planes is a point very near the pivot
        // point we search for. If the pivot point would be a point on the
        // bed surface, we would already be done. But as the actual pivot
        // point is in most cases below the bed, the intersection of the 3
        // points is behind or in front of the actual point (in X/Y). To
        // compensate for this error, we do the same calculation again, but
        // with the planes b) and c) tilted in the opposite direction and
        // take the average of the 2 points.
        let z_p1 = [intersect_3_planes(&p[0], &p[2], &p[3])?, intersect_3_planes(&p[0], &p[1], &p[3])?, intersect_3_planes(&p[0], &p[1], &p[2])?];
        let z_p2 = [intersect_3_planes(&p[0], &p[5], &p[6])?, intersect_3_planes(&p[0], &p[4], &p[6])?, intersect_3_planes(&p[0], &p[4], &p[5])?];

        // take the average of positive and negative measurement
        let z_pos: Vec<Vec<f64>> = z_p1.iter().zip(&z_p2).map(|(a, b)| (0..2).map(|i| (a[i] + b[i]) / 2.0).collect()).collect();
        let s_zpos: String = z_pos.iter().map(|zp| format!("{:.6}, {:.6}\n", zp[0], zp[1])).collect();
        Self::respond(&self.ad_gcmd, &format!("current estimated z_positions {}", s_zpos));
        self.ad_runs.borrow_mut().push(z_pos);
        // the (runs, 3, 2) means over axis 0, as rows of 6
        let flat: Vec<Vec<f64>> = self.ad_runs.borrow().iter().map(|r| r.iter().flatten().copied().collect()).collect();
        let n = flat.len();
        let sel = if n >= avlen { &flat[n - avlen..] } else { &flat[..] };
        let mean = np_mean_axis0(sel);
        *self.z_positions.borrow_mut() = mean.chunks(2).map(|c| (c[0], c[1])).collect();

        // We got a first estimate of the pivot points. Now apply the
        // adjustemts to all motors and repeat the process until the result
        // converges. We determine convergence by keeping track of the last
        // <average_len> + 1 runs and compare the standard deviation over that
        // len between the last two runs. When the error stops to decrease, we
        // are done. The final z_positions are determined by calculating the
        // average over the last <average_len> calculated positions.
        let p0 = self.ad_params.borrow()[0].clone();
        self.apply_adjustments(offsets, &p0)?;
        if n >= avlen {
            let errors = np_std_axis0(&flat[n - avlen..]);
            let error = np_std(&errors);
            match self.ad_error.get() {
                None => Self::respond(&self.ad_gcmd, &format!("current error: {:.6}", error)),
                Some(prev) => Self::respond(&self.ad_gcmd, &format!("previous error: {:.6} current error: {:.6}", prev, error)),
            }
            if let Some(prev) = self.ad_error.get() {
                if error >= prev {
                    self.ad_finalize_done(offsets)?;
                    return Ok(FinalizeResult::Done);
                }
            }
            self.ad_error.set(Some(error));
        }
        // restart
        self.ad_init();
        Ok(FinalizeResult::Retry)
    }

    pub fn ad_finalize_done(&self, offsets: (f64, f64, f64)) -> KResult<()> {
        let avlen = self.cal_avg_len.get() as usize;
        // calculate probe point z offsets
        let points = self.ad_points.borrow().clone();
        let n = points.len();
        let z_offsets: Vec<f64> = np_mean_axis0(&points[n.saturating_sub(avlen)..]).iter().map(|z| z - offsets.2).collect();
        *self.z_offsets.borrow_mut() = Some(z_offsets.clone());
        logging::info(&format!("final z_offsets {}", list_repr(&z_offsets)));
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let s_zoff = offsets_str(&z_offsets);
        let s_zpos: String = self.z_positions.borrow().iter().map(|(x, y)| format!("{:.6}, {:.6}\n", x, y)).collect();
        {
            let mut cf = configfile.pconfig.borrow_mut();
            cf.set(&self.section, "z_offsets", &s_zoff)?;
            cf.set(&self.section, "z_positions", &s_zpos)?;
        }
        Self::respond(&self.ad_gcmd, &format!("final z_positions are {}", s_zpos));
        Self::respond(&self.ad_gcmd, &format!("final z_offsets are: {}", s_zoff));
        Self::save_msg(&self.ad_gcmd);
        Ok(())
    }
}

impl PrinterObject for ZTilt {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(self.z_status.get_status())
    }
    fn teardown(&self) {
        *self.probe_helper.borrow_mut() = None;
        *self.cal_helper.borrow_mut() = None;
        *self.ad_helper.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ZTilt::new(config)?)
}
