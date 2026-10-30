// Simple Tap Classifier
// (port of klippy/extras/load_cell/tap_quality_classifier.py)
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::load_cell::tap_analysis::{TapAnalysis, TapClassifier, TapError, TapValidationError};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{ConfigFileObject, Printer};
use crate::pyfmt::{py_float_repr, py_round};
use std::cell::Cell;
use std::f64::consts::PI;
use std::rc::Rc;

pub struct TapQualityClassifierConfig {
    pub config_name: String,
    pub printer: Rc<Printer>,
    pub cfg_minimum_tap_quality: Cell<f64>,
    pub cfg_decompression_angle: Cell<Option<f64>>,
    pub minimum_tap_quality: Cell<f64>,
    pub decompression_angle: Cell<Option<f64>>,
    pub max_baseline_force_delta: f64,
    pub max_approach_force: f64,
    pub max_departure_force: f64,
    pub max_dwell_force_drop: f64,
}

impl TapQualityClassifierConfig {
    pub fn new(config: &ConfigWrapper) -> KResult<Self> {
        let cfg_minimum_tap_quality = config.getfloat("min_tap_quality", 40.0, Lim::new().min(0.0).max(100.0))?;
        let cfg_decompression_angle = config.getfloat_opt("decompression_angle", Lim::new().above(0.0).below(90.0))?;
        let cfg_max_approach_force = config.getfloat("max_approach_force", 50.0, Lim::new().min(0.0).max(100.0))? / 100.0;
        let cfg_max_baseline_force_delta = config.getfloat("max_baseline_force_delta", 25.0, Lim::new().min(0.0).max(100.0))? / 100.0;
        let cfg_max_departure_force = config.getfloat("max_departure_force", 25.0, Lim::new().min(0.0).max(100.0))? / 100.0;
        let cfg_max_dwell_force_drop = config.getfloat("max_dwell_force_drop", 75.0, Lim::new().min(0.0).max(100.0))? / 100.0;
        Ok(TapQualityClassifierConfig {
            config_name: config.get_name().to_string(),
            printer: config.get_printer(),
            cfg_minimum_tap_quality: Cell::new(cfg_minimum_tap_quality),
            cfg_decompression_angle: Cell::new(cfg_decompression_angle),
            minimum_tap_quality: Cell::new(cfg_minimum_tap_quality),
            decompression_angle: Cell::new(cfg_decompression_angle),
            max_baseline_force_delta: cfg_max_baseline_force_delta,
            max_approach_force: cfg_max_approach_force,
            max_departure_force: cfg_max_departure_force,
            max_dwell_force_drop: cfg_max_dwell_force_drop,
        })
    }

    pub fn customize(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.minimum_tap_quality.set(gcmd.get_float("MIN_TAP_QUALITY", Some(self.cfg_minimum_tap_quality.get()), Lim::new().min(0.0).max(100.0))?);
        let lim = Lim::new().above(0.0).below(90.0);
        self.decompression_angle.set(match self.cfg_decompression_angle.get() {
            Some(d) => Some(gcmd.get_float("DECOMPRESSION_ANGLE", Some(d), lim)?),
            None => gcmd.get_float_opt("DECOMPRESSION_ANGLE", lim)?,
        });
        Ok(())
    }

    pub fn save_config(&self, field: &str, value: f64) -> KResult<()> {
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        configfile.pconfig.borrow_mut().set(&self.config_name, field, &py_float_repr(value))?;
        Ok(())
    }

    pub fn set_minimum_tap_quality(&self, minimum_tap_quality: f64) -> KResult<()> {
        let minimum_tap_quality = py_round(minimum_tap_quality, 1);
        if !(0.0..=100.0).contains(&minimum_tap_quality) {
            return Err(KError::config("min_tap_quality must be between 0.0 and 100.0"));
        }
        self.cfg_minimum_tap_quality.set(minimum_tap_quality);
        self.minimum_tap_quality.set(minimum_tap_quality);
        self.save_config("min_tap_quality", minimum_tap_quality)
    }

    pub fn set_decompression_angle(&self, decompression_angle: f64) -> KResult<()> {
        let decompression_angle = py_round(decompression_angle, 1);
        if !(0.0 < decompression_angle && decompression_angle < 90.0) {
            return Err(KError::config("decompression_angle must be between 0.0 and 90.0"));
        }
        self.cfg_decompression_angle.set(Some(decompression_angle));
        self.decompression_angle.set(Some(decompression_angle));
        self.save_config("decompression_angle", decompression_angle)
    }
}

// Simple tap classifier rules:
// 1) The compression force must greater than the trigger force
// 2) Tap quality metric must be more than the minimum_tap_quality
pub struct TapQualityClassifier {
    pub printer: Rc<Printer>,
    config: TapQualityClassifierConfig,
}

impl TapQualityClassifier {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<dyn TapClassifier>> {
        Ok(Rc::new(TapQualityClassifier { printer: config.get_printer(), config: TapQualityClassifierConfig::new(config)? }))
    }

    fn slope_to_degrees(slope: f64) -> f64 {
        let (time_scale, gram_scale) = (0.1, 25.0);
        let scaling_factor = time_scale / gram_scale;
        let this_slope: f64 = 0.0;
        let other_slope = slope * scaling_factor;
        let radians = this_slope.atan2(1.0) - other_slope.atan2(1.0);
        radians / (PI / 180.0)
    }

    fn clamp(value: f64, threshold: f64) -> f64 {
        if value > threshold {
            return 1.0;
        }
        value
    }
}

impl TapClassifier for TapQualityClassifier {
    fn classify(&self, tap_analysis: &mut TapAnalysis, gcmd: Option<&GCodeCommand>) -> Result<(), TapError> {
        // This module cant rescue bad data
        if !tap_analysis.is_valid() {
            return Ok(());
        }

        // if the user does not configure an angle, don't classify the tap
        if self.config.decompression_angle.get().is_none() {
            return Ok(());
        }

        // unpack tap points with names
        let tap_points = tap_analysis.get_tap_points().to_vec();
        let approach_start_point = tap_points[0];
        let approach_end_point = tap_points[1];
        let compression_start_point = tap_points[1];
        let compression_end_point = tap_points[2];
        let dwell_start_point = tap_points[2];
        let dwell_end_point = tap_points[3];
        let decompression_end_point = tap_points[4];
        let departure_start_point = tap_points[4];
        let departure_end_point = tap_points[5];

        // only the compression force in the compression line
        let compression_force = (compression_end_point.force - compression_start_point.force).abs();

        // compression check: did we meet the minimum force requirement?
        if compression_force < tap_analysis.get_trigger_force() as f64 {
            return Err(TapValidationError::new(
                "LOW_COMPRESSION_FORCE",
                format!(
                    "Compression force, {}g, was less than the trigger force ({}g)",
                    py_float_repr(compression_force),
                    tap_analysis.get_trigger_force()
                ),
            ));
        }

        if let Some(gcmd) = gcmd {
            self.config.customize(gcmd)?;
        }

        let pct = |a: f64, b: f64| (a - b).abs() / compression_force;
        // These 2 metrics are expected to approach 0.0 in an ideal tap. If
        // they are large that usually indicates the tap is fouled.
        // They are clamped such that if they exceed 0.25, they become 1.0
        // this causes the quality score to drop to 0.0
        let baseline_delta_pct = Self::clamp(pct(compression_start_point.force, decompression_end_point.force), self.config.max_baseline_force_delta);
        let departure_force_pct = Self::clamp(pct(departure_end_point.force, departure_start_point.force), self.config.max_departure_force);

        // The approach force would also ideally be 0, but it can be subject to
        // bowden tube forces which we must allow. A larger 0.5 cutoff is used
        // to allow for this.
        let approach_force_pct = Self::clamp(pct(approach_end_point.force, approach_start_point.force), self.config.max_approach_force);

        // Dwell force drop is loosely clamped because it can drop quite a lot
        // in an acceptable tap
        let dwell_force_drop_pct = Self::clamp(pct(dwell_start_point.force, dwell_end_point.force), self.config.max_dwell_force_drop);

        // Normalize the decompression angle to the configured
        // mean angle for this printer
        let decompression_line = tap_analysis.get_tap_lines()[3];
        let decomp_angle = Self::slope_to_degrees(decompression_line.slope).abs();
        let decompression_angle = self.config.decompression_angle.get().unwrap_or(80.0);
        let normalized_decompression_angle = (decomp_angle - decompression_angle).abs() / decompression_angle;

        // Calculate tap quality score
        let tap_quality = 100.0
            * ((1.0 - approach_force_pct)
                * (1.0 - departure_force_pct)
                * (1.0 - baseline_delta_pct)
                * (1.0 - dwell_force_drop_pct)
                * (1.0 - normalized_decompression_angle));

        // TODO: should this be a debugging option? logged?
        // log info to console for now...
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        gcode.respond_info(
            &format!(
                "Tap Quality: {}%, Decompression angle:  {}",
                py_float_repr(py_round(tap_quality, 1)),
                py_float_repr(py_round(Self::slope_to_degrees(decompression_line.slope).abs(), 1))
            ),
            true,
        );

        // tap quality check
        if tap_quality < self.config.minimum_tap_quality.get() {
            return Err(TapValidationError::new(
                "LOW_TAP_QUALITY",
                format!(
                    "Tap quality, {}%, was less than the minimum tap quality {}%",
                    py_float_repr(tap_quality),
                    py_float_repr(self.config.minimum_tap_quality.get())
                ),
            ));
        }
        Ok(())
    }
}
