// Nozzle Cleanup Command (port of klippy/extras/nozzle_cleanup.py)
//
// Copyright (C) 2024  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::probe::{as_printer_probe, probe_list_get_list, GcodeNozzleScrubber, PrinterProbe, RetryPolicy};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::toolhead::ToolHead;
use std::cell::Cell;
use std::rc::{Rc, Weak};

struct NozzleCleanupOptions {
    cfg_probe_speed: f64,
    cfg_lift_speed: f64,
    cfg_horizontal_speed: f64,
    cfg_sample_retract_dist: f64,
    cfg_samples: i64,
    cfg_pattern_stepover: f64,
    cfg_pattern_x: i64,
    cfg_pattern_y: i64,
    probe_speed: Cell<f64>,
    lift_speed: Cell<f64>,
    horizontal_speed: Cell<f64>,
    sample_retract_dist: Cell<f64>,
    samples: Cell<i64>,
    pattern_stepover: Cell<f64>,
    pattern_x: Cell<i64>,
    pattern_y: Cell<i64>,
}

impl NozzleCleanupOptions {
    fn new(probe: &PrinterProbe, config: &ConfigWrapper) -> KResult<Self> {
        let cfg_probe_speed = config.getfloat("speed", probe.speed, Lim::new().above(0.0))?;
        let cfg_lift_speed = config.getfloat("lift_speed", probe.lift_speed, Lim::new().above(0.0))?;
        let cfg_horizontal_speed = config.getfloat("horizontal_speed", probe.speed, Lim::new().above(0.0))?;
        let cfg_sample_retract_dist = config.getfloat("sample_retract_dist", probe.sample_retract_dist, Lim::new().above(0.0))?;
        // Nozzle cleanup configuration defaults
        let cfg_samples = config.getint("samples", 3, Lim::new().min(1))?;
        let cfg_pattern_stepover = config.getfloat("stepover", 2.0, Lim::new().above(0.0))?;
        let cfg_pattern_x = config.getint("pattern_x", 10, Lim::new())?;
        let cfg_pattern_y = config.getint("pattern_y", 4, Lim::new())?;
        // Initialize values from config
        Ok(NozzleCleanupOptions {
            cfg_probe_speed,
            cfg_lift_speed,
            cfg_horizontal_speed,
            cfg_sample_retract_dist,
            cfg_samples,
            cfg_pattern_stepover,
            cfg_pattern_x,
            cfg_pattern_y,
            probe_speed: Cell::new(cfg_probe_speed),
            lift_speed: Cell::new(cfg_lift_speed),
            horizontal_speed: Cell::new(cfg_horizontal_speed),
            sample_retract_dist: Cell::new(cfg_sample_retract_dist),
            samples: Cell::new(cfg_samples),
            pattern_stepover: Cell::new(cfg_pattern_stepover),
            pattern_x: Cell::new(cfg_pattern_x),
            pattern_y: Cell::new(cfg_pattern_y),
        })
    }

    /// Update settings from a GCode Command instance
    fn customize(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.probe_speed.set(gcmd.get_float("SPEED", Some(self.cfg_probe_speed), Lim::new().above(0.0))?);
        self.lift_speed.set(gcmd.get_float("LIFT_SPEED", Some(self.cfg_lift_speed), Lim::new().above(0.0))?);
        self.horizontal_speed.set(gcmd.get_float("HORIZONTAL_SPEED", Some(self.cfg_horizontal_speed), Lim::new().above(0.0))?);
        self.sample_retract_dist.set(gcmd.get_float("SAMPLE_RETRACT_DIST", Some(self.cfg_sample_retract_dist), Lim::new().above(0.0))?);
        self.samples.set(gcmd.get_int("SAMPLES", Some(self.cfg_samples), Lim::new().min(1.0))?);
        self.pattern_stepover.set(gcmd.get_float("PATTERN_STEPOVER", Some(self.cfg_pattern_stepover), Lim::new().above(0.0))?);
        self.pattern_x.set(gcmd.get_int("PATTERN_X", Some(self.cfg_pattern_x), Lim::new().min(1.0))?);
        self.pattern_y.set(gcmd.get_int("PATTERN_Y", Some(self.cfg_pattern_y), Lim::new().min(1.0))?);
        Ok(())
    }

    /// Generate serpentine pattern of cleanup positions
    fn get_cleanup_positions(&self) -> Vec<(f64, f64)> {
        let mut positions = vec![];
        let (px, py, step) = (self.pattern_x.get(), self.pattern_y.get(), self.pattern_stepover.get());
        let x_count = px.abs();
        let y_count = py.abs();
        let x_dir = if px > 0 { 1.0 } else { -1.0 };
        let y_dir = if py > 0 { 1.0 } else { -1.0 };
        for row in 0..y_count {
            let y = row as f64 * step * y_dir;
            if row % 2 == 0 {
                for col in 0..x_count {
                    positions.push((col as f64 * step * x_dir, y));
                }
            } else {
                for col in (0..x_count).rev() {
                    positions.push((col as f64 * step * x_dir, y));
                }
            }
        }
        positions
    }
}

const CMD_NOZZLE_CLEANUP_HELP: &str = "Run nozzle cleanup";

pub struct NozzleCleanup {
    printer: Rc<Printer>,
    probe: Rc<PrinterProbe>,
    options: NozzleCleanupOptions,
    _retry_policy: RetryPolicy,
    nozzle_scrubber: GcodeNozzleScrubber,
}

impl NozzleCleanup {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let slot: Rc<std::cell::RefCell<Weak<NozzleCleanup>>> = Rc::new(std::cell::RefCell::new(Weak::new()));
        let sl = slot.clone();
        gcode.register_command(
            "NOZZLE_CLEANUP",
            Some(Rc::new(move |g| match sl.borrow().upgrade() {
                Some(n) => n.cmd_nozzle_cleanup(g),
                None => Ok(()),
            })),
            false,
            Some(CMD_NOZZLE_CLEANUP_HELP),
        )?;
        let probe = probe_list_get_list(&printer)?.get_config_probe(config, None)?;
        let probe = probe.as_ref().and_then(as_printer_probe).ok_or_else(|| KError::Other("'NoneType' object has no attribute 'speed'".into()))?;
        let options = NozzleCleanupOptions::new(&probe, config)?;
        let retry_policy = RetryPolicy::new(config)?;
        let nozzle_scrubber = GcodeNozzleScrubber::new(config)?;
        let n = Rc::new(NozzleCleanup { printer, probe, options, _retry_policy: retry_policy, nozzle_scrubber });
        *slot.borrow_mut() = Rc::downgrade(&n);
        Ok(n)
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.printer.lookup::<ToolHead>("toolhead")
    }

    fn horizontal_move(&self, pos: (f64, f64)) -> KResult<()> {
        self.toolhead()?.manual_move(&[Some(pos.0), Some(pos.1), None], self.options.horizontal_speed.get())
    }

    fn probing_move(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        let (_result, is_good) = self.probe.probing_move(self.options.probe_speed.get(), gcmd)?;
        Ok(is_good.unwrap_or(false))
    }

    fn retract_move(&self) -> KResult<()> {
        let toolhead = self.toolhead()?;
        let z_pos = toolhead.get_position()[2];
        toolhead.manual_move(&[None, None, Some(z_pos + self.options.sample_retract_dist.get())], self.options.lift_speed.get())
    }

    fn cmd_nozzle_cleanup(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.options.customize(gcmd)?;
        let positions = self.options.get_cleanup_positions();
        let origin_pos = self.toolhead()?.get_position();
        let max_positions = positions.len() as i64;
        let mut clean_samples = 0;

        for (i, position) in positions.iter().enumerate() {
            // move to the next position (first move is a no-op)
            let next_pos = (origin_pos[0] + position.0, origin_pos[1] + position.1);
            // move / probe / retract
            self.horizontal_move(next_pos)?;
            let is_good = self.probing_move(gcmd)?;
            self.retract_move()?;
            if is_good {
                clean_samples += 1;
                if clean_samples >= self.options.samples.get() {
                    // success on last consecutive probe
                    gcmd.respond_info("NOZZLE_CLEANUP succeeded", true);
                    return Ok(());
                }
            } else {
                clean_samples = 0;
                // maybe scrub nozzle
                self.nozzle_scrubber.clean_nozzle(gcmd, i as i64 + 1, max_positions)?;
            }
        }

        // if all positions are exhausted raise an error
        Err(KError::command(format!("NOZZLE_CLEANUP failed after trying {} positions", max_positions)))
    }
}

impl PrinterObject for NozzleCleanup {
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(NozzleCleanup::new(config)?)
}
