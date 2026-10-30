// Bed tilt compensation (port of klippy/extras/bed_tilt.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_move::{GCodeMove, MoveTransform};
use crate::extras::probe::{FinalizeResult, ProbePointsHelper};
use crate::extras::z_tilt::positions_repr;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mathutil::coordinate_descent;
use crate::printer::{downcast_rc, ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::py_float_repr;
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub struct BedTilt {
    pub printer: Rc<Printer>,
    pub x_adjust: Cell<f64>,
    pub y_adjust: Cell<f64>,
    pub z_adjust: Cell<f64>,
    calibrate: RefCell<Option<Rc<BedTiltCalibrate>>>,
    pub toolhead: RefCell<Option<Rc<ToolHead>>>,
}

fn params_repr(p: &IndexMap<String, f64>) -> String {
    let v: Vec<String> = p.iter().map(|(k, v)| format!("'{}': {}", k, py_float_repr(*v))).collect();
    format!("{{{}}}", v.join(", "))
}

impl BedTilt {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let bt = Rc::new(BedTilt {
            printer: printer.clone(),
            x_adjust: Cell::new(0.0),
            y_adjust: Cell::new(0.0),
            z_adjust: Cell::new(0.0),
            calibrate: RefCell::new(None),
            toolhead: RefCell::new(None),
        });
        let b = bt.clone();
        crate::events::KlippyConnect::on(&printer, move || {
            *b.toolhead.borrow_mut() = Some(b.printer.lookup::<ToolHead>("toolhead")?);
            Ok(())
        });
        bt.x_adjust.set(config.getfloat("x_adjust", 0.0, Lim::new())?);
        bt.y_adjust.set(config.getfloat("y_adjust", 0.0, Lim::new())?);
        bt.z_adjust.set(config.getfloat("z_adjust", 0.0, Lim::new())?);
        if config.get_opt("points")?.is_some() {
            *bt.calibrate.borrow_mut() = Some(BedTiltCalibrate::new(config, &bt)?);
        }
        // Register move transform with g-code class
        let gcode_move = printer
            .load_object(config, "gcode_move", true)?
            .and_then(downcast_rc::<GCodeMove>)
            .ok_or_else(|| KError::Other("gcode_move has an unexpected type".into()))?;
        gcode_move.set_move_transform(bt.clone(), false)?;
        Ok(bt)
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.toolhead.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_position'".into()))
    }

    pub fn update_adjust(&self, x_adjust: f64, y_adjust: f64, z_adjust: f64) -> KResult<()> {
        self.x_adjust.set(x_adjust);
        self.y_adjust.set(y_adjust);
        self.z_adjust.set(z_adjust);
        let gcode_move = self.printer.lookup::<GCodeMove>("gcode_move")?;
        gcode_move.reset_last_position()?;
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let mut cf = configfile.pconfig.borrow_mut();
        cf.set("bed_tilt", "x_adjust", &format!("{:.6}", x_adjust))?;
        cf.set("bed_tilt", "y_adjust", &format!("{:.6}", y_adjust))?;
        cf.set("bed_tilt", "z_adjust", &format!("{:.6}", z_adjust))?;
        Ok(())
    }
}

impl MoveTransform for BedTilt {
    fn get_position(&self) -> Vec<f64> {
        let Ok(toolhead) = self.toolhead() else { return vec![] };
        let mut pos = toolhead.get_position();
        let (x, y) = (pos[0], pos[1]);
        pos[2] -= x * self.x_adjust.get() + y * self.y_adjust.get() + self.z_adjust.get();
        pos
    }

    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let mut p = newpos.to_vec();
        let (x, y) = (p[0], p[1]);
        p[2] += x * self.x_adjust.get() + y * self.y_adjust.get() + self.z_adjust.get();
        self.toolhead()?.move_to(&p, speed)
    }
}

impl PrinterObject for BedTilt {
    fn teardown(&self) {
        *self.calibrate.borrow_mut() = None;
        *self.toolhead.borrow_mut() = None;
    }
}

const CMD_BED_TILT_CALIBRATE_HELP: &str = "Bed tilt calibration script";

// Helper script to calibrate the bed tilt
pub struct BedTiltCalibrate {
    pub printer: Rc<Printer>,
    pub bedtilt: Weak<BedTilt>,
    pub probe_helper: RefCell<Option<Rc<ProbePointsHelper>>>,
    pub gcode: Rc<GCodeDispatch>,
}

impl BedTiltCalibrate {
    pub fn new(config: &ConfigWrapper, bedtilt: &Rc<BedTilt>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let c = Rc::new(BedTiltCalibrate { printer, bedtilt: Rc::downgrade(bedtilt), probe_helper: RefCell::new(None), gcode: gcode.clone() });
        let w = Rc::downgrade(&c);
        let probe_helper = ProbePointsHelper::new(
            config,
            Rc::new(move |offsets, positions| match w.upgrade() {
                Some(c) => c.probe_finalize(offsets, positions),
                None => Ok(FinalizeResult::Done),
            }),
            None,
            "points",
            false,
            false,
        )?;
        probe_helper.minimum_points(3)?;
        *c.probe_helper.borrow_mut() = Some(probe_helper);
        // Register BED_TILT_CALIBRATE command
        let cc = c.clone();
        gcode.register_command(
            "BED_TILT_CALIBRATE",
            Some(Rc::new(move |g: &GCodeCommand| cc.probe_helper.borrow().clone().expect("probe helper").start_probe(g))),
            false,
            Some(CMD_BED_TILT_CALIBRATE_HELP),
        )?;
        Ok(c)
    }

    pub fn probe_finalize(&self, offsets: (f64, f64, f64), positions: &[Vec<f64>]) -> KResult<FinalizeResult> {
        let bedtilt = self.bedtilt.upgrade().ok_or_else(|| KError::Other("bed_tilt gone".into()))?;
        // Setup for coordinate descent analysis
        let z_offset = offsets.2;
        logging::info(&format!("Calculating bed_tilt with: {}", positions_repr(positions)));
        let mut params = IndexMap::default();
        params.insert("x_adjust".into(), bedtilt.x_adjust.get());
        params.insert("y_adjust".into(), bedtilt.y_adjust.get());
        params.insert("z_adjust".into(), z_offset);
        logging::info(&format!("Initial bed_tilt parameters: {}", params_repr(&params)));

        // Perform coordinate descent
        let adjusted_height = |pos: &[f64], params: &IndexMap<String, f64>| {
            pos[2] - pos[0] * params["x_adjust"] - pos[1] * params["y_adjust"] - params["z_adjust"]
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
        // Update current bed_tilt calculations
        let x_adjust = new_params["x_adjust"];
        let y_adjust = new_params["y_adjust"];
        let z_adjust = new_params["z_adjust"] - z_offset - x_adjust * offsets.0 - y_adjust * offsets.1;
        bedtilt.update_adjust(x_adjust, y_adjust, z_adjust)?;
        // Log and report results
        logging::info(&format!("Calculated bed_tilt parameters: {}", params_repr(&new_params)));
        for pos in positions {
            logging::info(&format!(
                "orig: {} new: {}",
                py_float_repr(adjusted_height(pos, &params)),
                py_float_repr(adjusted_height(pos, &new_params))
            ));
        }
        let msg = format!("x_adjust: {:.6} y_adjust: {:.6} z_adjust: {:.6}", x_adjust, y_adjust, z_adjust);
        self.printer.set_rollover_info("bed_tilt", Some(&format!("bed_tilt: {}", msg)), true);
        self.gcode.respond_info(
            &format!(
                "{}\nThe above parameters have been applied to the current\nsession. The SAVE_CONFIG command will update the printer\nconfig file and restart the printer.",
                msg
            ),
            true,
        );
        Ok(FinalizeResult::Done)
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(BedTilt::new(config)?)
}
