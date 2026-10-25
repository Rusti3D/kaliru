// Support for reading SPI magnetic angle sensors
// (port of klippy/extras/angle.py)
//
// Copyright (C) 2021,2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, BulkDataQueue, ClientCallback, MAX_BULK_MSG_SIZE};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::extras::force_move::ForceMove;
use crate::extras::load_cell::batch_helper;
use crate::extras::tmc::TmcDriver;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::jinja::value::float_divmod;
use crate::logging;
use crate::mathutil::np_lstsq;
use crate::mcu::{CommandQueryWrapper, CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::printer::{downcast_rc, ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_round, py_str_repr, py_int_base0, py_sum};
use crate::pyval::PyVal;
use crate::msgproto::Params;
use crate::stepper::McuStepper;
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const MIN_MSG_TIME: f64 = 0.100;
const TCODE_ERROR: i64 = 0xFF;

const TRINAMIC_DRIVERS: [&str; 7] = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160", "tmc5262"];

const CALIBRATION_BITS: i64 = 6; // 64 entries
const ANGLE_BITS: i64 = 16; // angles range from 0..65535

/// a (time, angle) sample
type AngleSample = (f64, i64);

fn not_sent() -> KError {
    KError::Other("'NoneType' object has no attribute 'send'".into())
}

/// the response bytes of a query
fn response(params: &Params) -> Vec<u8> {
    match params.get("response") {
        Some(Val::Bytes(b)) => b.clone(),
        _ => vec![],
    }
}

/// a gcmd.get(name, minval=0, maxval=..., parser=lambda x: int(x, 0))
fn get_reg_param(gcmd: &GCodeCommand, name: &str, maxval: f64) -> KResult<i64> {
    Ok(gcmd.get_parsed(name, None, py_int_base0, |v| *v as f64, &Lim::new().min(0.0).max(maxval))?.unwrap_or(0))
}

/// (microsteps, full_steps_per_rotation) from the configfile settings
fn get_microsteps(printer: &Printer, stepper_name: &str) -> KResult<(i64, i64)> {
    let configfile = printer.lookup::<ConfigFileObject>("configfile")?;
    let status = configfile.pconfig.borrow().get_status(0.0);
    let stconfig = match &status {
        PyVal::Dict(d) => match d.get("settings").map(PyVal::get) {
            Some(PyVal::Dict(s)) => s.get(stepper_name).cloned(),
            _ => None,
        },
        _ => None,
    };
    let field = |k: &str| match &stconfig {
        Some(PyVal::Dict(d)) => d.get(k).and_then(|v| v.as_f64()).map(|v| v as i64).ok_or_else(|| KError::Py("KeyError".into(), py_str_repr(k))),
        _ => Err(KError::Py("KeyError".into(), py_str_repr(k))),
    };
    Ok((field("microsteps")?, field("full_steps_per_rotation")?))
}

/// AngleCalibration.load_calibration(angles): the interpolation buckets,
/// angle_phase_offset and calibration_reversed
pub fn calc_calibration(angles: &[f64]) -> KResult<(Vec<i64>, f64, bool)> {
    // Calculate linear intepolation calibration buckets by solving
    // linear equations
    let angle_max = 1i64 << ANGLE_BITS;
    let calibration_count = 1usize << CALIBRATION_BITS;
    let bucket_size = angle_max / calibration_count as i64;
    let full_steps = angles.len();
    if full_steps == 0 {
        return Err(KError::Py("ZeroDivisionError".into(), "float division by zero".into()));
    }
    let nominal_step = angle_max as f64 / full_steps as f64;
    let index_of_min = |a: &[f64]| {
        let min = a.iter().copied().fold(a[0], |m, v| if v < m { v } else { m });
        a.iter().position(|v| *v == min).unwrap_or(0)
    };
    let angle_phase_offset = (index_of_min(angles) & 3) as f64 * nominal_step;
    if angles.len() < 2 {
        return Err(KError::Py("IndexError".into(), "list index out of range".into()));
    }
    let calibration_reversed = angles[angles.len() - 2] > angles[angles.len() - 1];
    let mut angles = angles.to_vec();
    if calibration_reversed {
        angles.reverse();
    }
    let first_step = index_of_min(&angles);
    angles.rotate_left(first_step);

    let mut eqs = vec![vec![0.0; calibration_count]; full_steps];
    let mut ans = vec![0.0; full_steps];
    for (step, angle) in angles.iter().enumerate() {
        let int_angle = ((angle + 0.5) as i64).rem_euclid(angle_max);
        let bucket = (int_angle as f64 / bucket_size as f64) as usize;
        let bucket_start = (bucket as i64 * bucket_size) as f64;
        let ang_diff = angle - bucket_start;
        let ang_diff_per = ang_diff / bucket_size as f64;
        let eq = &mut eqs[step];
        eq[bucket] = 1.0 - ang_diff_per;
        eq[(bucket + 1) % calibration_count] = ang_diff_per;
        ans[step] = step as f64 * nominal_step;
        if bucket + 1 >= calibration_count {
            ans[step] -= ang_diff_per * angle_max as f64;
        }
    }
    let sol = np_lstsq(&eqs, &ans);
    let isol: Vec<i64> = sol.iter().map(|s| (s + 0.5) as i64).collect();
    let mut calibration = isol.clone();
    calibration.push(isol[0] + angle_max);
    Ok((calibration, angle_phase_offset, calibration_reversed))
}

const CMD_ANGLE_CALIBRATE_HELP: &str = "Calibrate angle sensor to stepper motor";

pub struct AngleCalibration {
    printer: Rc<Printer>,
    name: String,
    stepper_name: Option<String>,
    tmc_module: RefCell<Option<Rc<TmcDriver>>>,
    mcu_stepper: RefCell<Option<Rc<McuStepper>>>,
    // Current calibration data
    mcu_pos_offset: Cell<Option<f64>>,
    angle_phase_offset: Cell<f64>,
    calibration_reversed: Cell<bool>,
    calibration: RefCell<Vec<i64>>,
}

impl AngleCalibration {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let c = Rc::new(AngleCalibration {
            printer: printer.clone(),
            name: config.get_name().to_string(),
            stepper_name: config.get_opt("stepper")?,
            tmc_module: RefCell::new(None),
            mcu_stepper: RefCell::new(None),
            mcu_pos_offset: Cell::new(None),
            angle_phase_offset: Cell::new(0.0),
            calibration_reversed: Cell::new(false),
            calibration: RefCell::new(vec![]),
        });
        let Some(stepper_name) = c.stepper_name.clone() else {
            // No calibration
            return Ok(c);
        };
        let sconfig = config.getsection(&stepper_name);
        sconfig.getint("microsteps", Dflt::Required, Lim::new().no_note())?;
        if let Some(cal) = config.get_opt("calibrate")? {
            let mut angles = vec![];
            for d in cal.split(',').map(|d| d.trim()).filter(|d| !d.is_empty()) {
                angles.push(py_float(d).ok_or_else(|| KError::Py("ValueError".into(), format!("could not convert string to float: {}", py_str_repr(d))))?);
            }
            c.load_calibration(&angles)?;
        }
        // Register commands
        let cc = c.clone();
        crate::events::StepperSyncMcuPosition::on(&printer, move |s| {
            cc.handle_sync_mcu_pos(&s);
            Ok(())
        });
        let cc = c.clone();
        crate::events::KlippyConnect::on(&printer, move || cc.connect());
        let cname = c.name.split_whitespace().last().unwrap_or_default().to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let cc = c.clone();
        gcode.register_mux_command(
            "ANGLE_CALIBRATE",
            "CHIP",
            Some(&cname),
            Rc::new(move |gcmd| cc.cmd_angle_calibrate(gcmd)),
            Some(CMD_ANGLE_CALIBRATE_HELP),
        )?;
        Ok(c)
    }

    fn stepper_name(&self) -> &str {
        self.stepper_name.as_deref().unwrap_or("None")
    }

    fn mcu_stepper(&self) -> KResult<Rc<McuStepper>> {
        self.mcu_stepper.borrow().clone().ok_or_else(|| KError::Py("AttributeError".into(), "'NoneType' object has no attribute 'get_mcu_position'".into()))
    }

    fn tmc_module(&self) -> KResult<Rc<TmcDriver>> {
        self.tmc_module.borrow().clone().ok_or_else(|| KError::Py("AttributeError".into(), "'NoneType' object has no attribute 'get_phase_offset'".into()))
    }

    fn handle_sync_mcu_pos(&self, mcu_stepper: &McuStepper) {
        if Some(mcu_stepper.get_name(false)) == self.stepper_name {
            self.mcu_pos_offset.set(None);
        }
    }

    fn calc_mcu_pos_offset(&self, sample: AngleSample) -> KResult<()> {
        // Lookup phase information
        let (mcu_phase_offset, phases) = self.tmc_module()?.get_phase_offset();
        let Some(mcu_phase_offset) = mcu_phase_offset else { return Ok(()) };
        // Find mcu position at time of sample
        let (angle_time, angle_pos) = sample;
        let mcu_pos = self.mcu_stepper()?.get_past_mcu_position(angle_time);
        // Convert angle_pos to mcu_pos units
        let (microsteps, full_steps) = get_microsteps(&self.printer, self.stepper_name())?;
        let angle_to_mcu_pos = (full_steps * microsteps) as f64 / (1i64 << ANGLE_BITS) as f64;
        let angle_mpos = angle_pos as f64 * angle_to_mcu_pos;
        // Calculate adjustment for stepper phases
        let mut phase_diff = float_divmod(
            (angle_mpos + self.angle_phase_offset.get() * angle_to_mcu_pos) - (mcu_pos + mcu_phase_offset) as f64,
            phases as f64,
        )
        .1;
        if phase_diff > (phases.div_euclid(2)) as f64 {
            phase_diff -= phases as f64;
        }
        // Store final offset
        self.mcu_pos_offset.set(Some(mcu_pos as f64 - (angle_mpos - phase_diff)));
        Ok(())
    }

    fn apply_calibration(&self, samples: &mut [AngleSample]) -> KResult<Option<f64>> {
        if self.stepper_name.is_none() {
            // python's __init__ returns before setting self.calibration
            return Err(KError::Py("AttributeError".into(), "'AngleCalibration' object has no attribute 'calibration'".into()));
        }
        let calibration = self.calibration.borrow().clone();
        if calibration.is_empty() {
            return Ok(None);
        }
        let calibration_reversed = self.calibration_reversed.get();
        let interp_bits = ANGLE_BITS - CALIBRATION_BITS;
        let interp_mask = (1i64 << interp_bits) - 1;
        let interp_round = 1i64 << (interp_bits - 1);
        for s in samples.iter_mut() {
            let (samp_time, angle) = *s;
            let bucket = ((angle & 0xFFFF) >> interp_bits) as usize;
            let cal1 = calibration[bucket];
            let cal2 = calibration[bucket + 1];
            let adj = (angle & interp_mask) * (cal2 - cal1);
            let adj = cal1 + ((adj + interp_round) >> interp_bits);
            let mut angle_diff = (adj - angle) & 0xFFFF;
            angle_diff -= (angle_diff & 0x8000) << 1;
            let mut new_angle = angle + angle_diff;
            if calibration_reversed {
                new_angle = -new_angle;
            }
            *s = (samp_time, new_angle);
        }
        if self.mcu_pos_offset.get().is_none() {
            self.calc_mcu_pos_offset(samples[0])?;
            if self.mcu_pos_offset.get().is_none() {
                return Ok(None);
            }
        }
        let stepper = self.mcu_stepper()?;
        Ok(Some(stepper.mcu_to_commanded_position_f(self.mcu_pos_offset.get().unwrap_or(0.0))))
    }

    fn load_calibration(&self, angles: &[f64]) -> KResult<()> {
        let (calibration, angle_phase_offset, calibration_reversed) = calc_calibration(angles)?;
        self.angle_phase_offset.set(angle_phase_offset);
        self.calibration_reversed.set(calibration_reversed);
        *self.calibration.borrow_mut() = calibration;
        Ok(())
    }

    fn lookup_tmc(&self) -> KResult<Rc<TmcDriver>> {
        for driver in TRINAMIC_DRIVERS {
            let driver_name = format!("{} {}", driver, self.stepper_name());
            if let Some(module) = self.printer.lookup_object_opt(&driver_name).and_then(downcast_rc::<TmcDriver>) {
                return Ok(module);
            }
        }
        Err(KError::command(format!("Unable to find TMC driver for {}", self.stepper_name())))
    }

    fn connect(&self) -> KResult<()> {
        *self.tmc_module.borrow_mut() = Some(self.lookup_tmc()?);
        let fmove = self.printer.lookup::<ForceMove>("force_move")?;
        *self.mcu_stepper.borrow_mut() = Some(fmove.lookup_stepper(self.stepper_name())?);
        Ok(())
    }

    fn get_stepper_phase(&self) -> KResult<i64> {
        let (mcu_phase_offset, phases) = self.tmc_module()?.get_phase_offset();
        let Some(mcu_phase_offset) = mcu_phase_offset else {
            return Err(KError::command(format!("Driver phase not known for {}", self.stepper_name())));
        };
        let mcu_pos = self.mcu_stepper()?.get_mcu_position(None);
        Ok((mcu_pos + mcu_phase_offset).rem_euclid(phases))
    }

    /// (forward, reverse) {step: [angles]} measurements
    fn do_calibration_moves(&self) -> KResult<(IndexMap<i64, Vec<f64>>, IndexMap<i64, Vec<f64>>)> {
        let fmove = self.printer.lookup::<ForceMove>("force_move")?;
        let mcu_stepper = self.mcu_stepper()?;
        let mv = |dist: f64, speed: f64| fmove.manual_move(&mcu_stepper, dist, speed, 0.0);
        // Start data collection
        let msgs: Rc<RefCell<Vec<PyVal>>> = Rc::new(RefCell::new(vec![]));
        let is_finished = Rc::new(Cell::new(false));
        let (m, f) = (msgs.clone(), is_finished.clone());
        let handle_batch: ClientCallback = Rc::new(move |msg: &PyVal| {
            if f.get() {
                return false;
            }
            m.borrow_mut().push(msg.clone());
            true
        });
        self.printer.lookup::<Angle>(&self.name)?.add_client(handle_batch)?;
        // Move stepper several turns (to allow internal sensor calibration)
        let (microsteps, full_steps) = get_microsteps(&self.printer, self.stepper_name())?;
        let step_dist = mcu_stepper.get_step_dist();
        let full_step_dist = step_dist * microsteps as f64;
        let rotation_dist = full_steps as f64 * full_step_dist;
        let align_dist = step_dist * self.get_stepper_phase()? as f64;
        let move_time = 0.010;
        let move_speed = full_step_dist / move_time;
        mv(-(rotation_dist + align_dist), move_speed)?;
        mv(2.0 * rotation_dist, move_speed)?;
        mv(-2.0 * rotation_dist, move_speed)?;
        mv(0.5 * rotation_dist - full_step_dist, move_speed)?;
        // Move to each full step position
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let mut times = vec![];
        let mut samp_dist = full_step_dist;
        for i in 0..2 * full_steps {
            mv(samp_dist, move_speed)?;
            let start_query_time = toolhead.get_last_move_time()? + 0.050;
            let end_query_time = start_query_time + 0.050;
            times.push((start_query_time, end_query_time));
            toolhead.dwell(0.150)?;
            if i == full_steps - 1 {
                // Reverse direction and test each full step again
                mv(0.5 * rotation_dist, move_speed)?;
                mv(-0.5 * rotation_dist + samp_dist, move_speed)?;
                samp_dist = -samp_dist;
            }
        }
        mv(0.5 * rotation_dist + align_dist, move_speed)?;
        toolhead.wait_moves()?;
        // Finish data collection
        is_finished.set(true);
        // Correlate query responses
        let mut cal: IndexMap<usize, Vec<f64>> = IndexMap::default();
        let mut step = 0;
        for msg in msgs.borrow().iter() {
            let PyVal::Dict(d) = msg else { continue };
            let Some(PyVal::List(data)) = d.get("data") else { continue };
            for row in data {
                let Some(r) = row.items() else { continue };
                let (query_time, pos) = (r[0].as_f64().unwrap_or(0.0), r[1].as_f64().unwrap_or(0.0));
                // Add to step tracking
                while step < times.len() && query_time > times[step].1 {
                    step += 1;
                }
                if step < times.len() && query_time >= times[step].0 {
                    cal.entry(step).or_default().push(pos);
                }
            }
        }
        if cal.len() != times.len() {
            return Err(KError::command("Failed calibration - incomplete sensor data"));
        }
        let fs = full_steps as usize;
        let fcal = (0..fs).map(|i| (i as i64, cal[&i].clone())).collect();
        let rcal = (0..fs).map(|i| ((fs - i - 1) as i64, cal[&(i + fs)].clone())).collect();
        Ok((fcal, rcal))
    }

    fn calc_angles(meas: &IndexMap<i64, Vec<f64>>) -> (IndexMap<i64, f64>, f64, i64) {
        let mut total_count = 0;
        let mut total_variance = 0.0;
        let mut angles = IndexMap::default();
        for (step, data) in meas {
            let count = data.len();
            let angle_avg = py_sum(data.iter().copied()) / count as f64;
            angles.insert(*step, angle_avg);
            total_count += count as i64;
            total_variance += py_sum(data.iter().map(|d| (d - angle_avg).powi(2)));
        }
        (angles, (total_variance / total_count as f64).sqrt(), total_count)
    }

    fn cmd_angle_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Perform calibration movement and capture
        let old_calibration = std::mem::take(&mut *self.calibration.borrow_mut());
        let res = self.do_calibration_moves();
        *self.calibration.borrow_mut() = old_calibration;
        let (fcal, rcal) = res?;
        // Calculate each step position average and variance
        let (_microsteps, full_steps) = get_microsteps(&self.printer, self.stepper_name())?;
        let (fangles, fstd, _ftotal) = Self::calc_angles(&fcal);
        let (rangles, rstd, _rtotal) = Self::calc_angles(&rcal);
        let distinct = |a: &IndexMap<i64, f64>| {
            let mut v: Vec<u64> = a.values().map(|f| if *f == 0.0 { 0 } else { f.to_bits() }).collect();
            v.sort();
            v.dedup();
            v.len()
        };
        if distinct(&fangles) != fangles.len() || distinct(&rangles) != rangles.len() {
            return Err(KError::command("Failed calibration - sensor not updating for each step"));
        }
        let merged: IndexMap<i64, Vec<f64>> = (0..full_steps).map(|i| (i, [fcal[&i].clone(), rcal[&i].clone()].concat())).collect();
        let (angles, std, total) = Self::calc_angles(&merged);
        gcmd.respond_info(&format!("angle: stddev={:.3} ({:.3} forward / {:.3} reverse) in {} queries", std, fstd, rstd, total), true);
        // Order data with lowest/highest magnet position first
        let mut anglist: Vec<f64> = (0..full_steps).map(|i| float_divmod(angles[&i], 0xFFFF as f64).1).collect();
        let first_ang = if angles[&0] > angles[&1] {
            anglist.iter().copied().fold(anglist[0], |m, v| if v > m { v } else { m })
        } else {
            anglist.iter().copied().fold(anglist[0], |m, v| if v < m { v } else { m })
        };
        let first_phase = anglist.iter().position(|v| *v == first_ang).unwrap_or(0) & !3;
        anglist.rotate_left(first_phase);
        // Save results
        let mut cal_contents = vec![];
        for (i, angle) in anglist.iter().enumerate() {
            if i % 8 == 0 {
                cal_contents.push("\n".to_string());
            }
            cal_contents.push(format!("{:.1}", angle));
            cal_contents.push(",".to_string());
        }
        cal_contents.pop();
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let mut cf = configfile.pconfig.borrow_mut();
        cf.remove_section(&self.name);
        cf.set(&self.name, "calibrate", &cal_contents.concat())?;
        Ok(())
    }
}

/// the sensor_type helper classes
trait AngleSensor {
    fn is_tcode_absolute(&self) -> bool {
        false
    }
    fn last_temperature(&self) -> Option<f64> {
        None
    }
    fn clear_temperature(&self) {}
    fn get_static_delay(&self) -> f64;
    fn start(&self) -> KResult<()>;
    /// (last_chip_mcu_clock, last_chip_clock, chip_freq)
    fn get_tcode_params(&self) -> (i64, i64, f64) {
        (0, 0, 0.0)
    }
    fn update_clock(&self) -> KResult<()> {
        Ok(())
    }
}

struct HelperA1333 {
    spi: Rc<McuSpi>,
}

impl HelperA1333 {
    const SPI_MODE: i64 = 3;
    const SPI_SPEED: i64 = 10000000;
}

impl AngleSensor for HelperA1333 {
    fn get_static_delay(&self) -> f64 {
        0.000001
    }
    fn start(&self) -> KResult<()> {
        // Setup for angle query
        self.spi.spi_transfer(&[0x32, 0x00], 0, 0)?;
        Ok(())
    }
}

struct HelperAS5047D {
    spi: Rc<McuSpi>,
}

impl HelperAS5047D {
    const SPI_MODE: i64 = 1;
    const SPI_SPEED: i64 = (1.0 / 0.000000350) as i64;
}

impl AngleSensor for HelperAS5047D {
    fn get_static_delay(&self) -> f64 {
        0.000100
    }
    fn start(&self) -> KResult<()> {
        // Clear any errors from device
        self.spi.spi_transfer(&[0xFF, 0xFC], 0, 0)?; // Read DIAAGC
        self.spi.spi_transfer(&[0x40, 0x01], 0, 0)?; // Read ERRFL
        self.spi.spi_transfer(&[0xC0, 0x00], 0, 0)?; // Read NOP
        Ok(())
    }
}

/// spi_angle_transfer query command, looked up at config time
fn angle_transfer_cmd(mcu: &Mcu, spi: &McuSpi, oid: i64) -> KResult<CommandQueryWrapper> {
    mcu.lookup_query_command(
        "spi_angle_transfer oid=%c data=%*s",
        "spi_angle_transfer_response oid=%c clock=%u response=%*s",
        Some(oid),
        Some(spi.get_command_queue()),
        false,
    )
}

const CMD_ANGLE_DEBUG_READ_HELP: &str = "Query low-level angle sensor register";
const CMD_ANGLE_DEBUG_WRITE_HELP: &str = "Set low-level angle sensor register";

struct HelperTLE5012B {
    spi: Rc<McuSpi>,
    oid: i64,
    last_temperature: Cell<Option<f64>>,
    mcu: Rc<Mcu>,
    spi_angle_transfer_cmd: RefCell<Option<CommandQueryWrapper>>,
    last_chip_mcu_clock: Cell<i64>,
    last_chip_clock: Cell<i64>,
    chip_freq: Cell<f64>,
}

impl HelperTLE5012B {
    const SPI_MODE: i64 = 1;
    const SPI_SPEED: i64 = 4000000;

    fn new(config: &ConfigWrapper, spi: &Rc<McuSpi>, oid: i64) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mcu = spi.get_mcu();
        let h = Rc::new(HelperTLE5012B {
            spi: spi.clone(),
            oid,
            last_temperature: Cell::new(None),
            mcu: mcu.clone(),
            spi_angle_transfer_cmd: RefCell::new(None),
            last_chip_mcu_clock: Cell::new(0),
            last_chip_clock: Cell::new(0),
            chip_freq: Cell::new(0.0),
        });
        let hh = h.clone();
        mcu.register_config_callback(Rc::new(move || {
            *hh.spi_angle_transfer_cmd.borrow_mut() = Some(angle_transfer_cmd(&hh.mcu, &hh.spi, hh.oid)?);
            Ok(())
        }));
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let hh = h.clone();
        gcode.register_mux_command("ANGLE_DEBUG_READ", "CHIP", Some(&name), Rc::new(move |g| hh.cmd_angle_debug_read(g)), Some(CMD_ANGLE_DEBUG_READ_HELP))?;
        let hh = h.clone();
        gcode.register_mux_command("ANGLE_DEBUG_WRITE", "CHIP", Some(&name), Rc::new(move |g| hh.cmd_angle_debug_write(g)), Some(CMD_ANGLE_DEBUG_WRITE_HELP))?;
        Ok(h)
    }

    fn calc_crc(data: &[u8]) -> u8 {
        let mut crc: u32 = 0xFF;
        for d in data {
            crc ^= *d as u32;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ 0x1D;
                } else {
                    crc <<= 1;
                }
            }
        }
        (!crc & 0xFF) as u8
    }

    fn send_spi(&self, msg: &[u8]) -> KResult<Params> {
        for _retry in 0..5 {
            let params = if msg[0] & 0x04 != 0 {
                let cmd = self.spi_angle_transfer_cmd.borrow();
                cmd.as_ref().ok_or_else(not_sent)?.send(&[Val::Int(self.oid), Val::Bytes(msg.to_vec())], 0, 0, true)?
            } else {
                self.spi.spi_transfer(msg, 0, 0)?
            };
            let resp = response(&params);
            let mut data = msg[..2].to_vec();
            if resp.len() >= 4 {
                data.extend_from_slice(&resp[2..resp.len() - 2]);
            }
            let crc = Self::calc_crc(&data);
            if resp.last() == Some(&crc) {
                return Ok(params);
            }
        }
        Err(KError::command("Unable to query tle5012b chip"))
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        let mut cw = 0x8000 | ((reg & 0x3F) << 4) | 0x01;
        if (0x05..=0x11).contains(&reg) {
            cw |= 0x5000;
        }
        let msg = [(cw >> 8) as u8, (cw & 0xFF) as u8, 0, 0, 0, 0];
        let resp = response(&self.send_spi(&msg)?);
        Ok(((resp[2] as i64) << 8) | resp[3] as i64)
    }

    fn write_reg(&self, reg: i64, val: i64) -> KResult<()> {
        let mut cw = ((reg & 0x3F) << 4) | 0x01;
        if (0x05..=0x11).contains(&reg) {
            cw |= 0x5000;
        }
        let msg = [(cw >> 8) as u8, (cw & 0xFF) as u8, ((val >> 8) & 0xFF) as u8, (val & 0xFF) as u8, 0, 0];
        for _retry in 0..5 {
            self.send_spi(&msg)?;
            let rval = self.read_reg(reg)?;
            if rval == val {
                return Ok(());
            }
        }
        Err(KError::command("Unable to write to tle5012b chip"))
    }

    fn mask_reg(&self, reg: i64, off: i64, on: i64) -> KResult<()> {
        let rval = self.read_reg(reg)?;
        self.write_reg(reg, (rval & !off) | on)
    }

    fn query_clock(&self) -> KResult<(i64, i64)> {
        // Read frame counter (and normalize to a 16bit counter)
        let msg = [0x84, 0x42, 0, 0, 0, 0, 0, 0]; // Read with latch, AREV and FSYNC
        let params = self.send_spi(&msg)?;
        let resp = response(&params);
        let clock = match params.get("clock") {
            Some(Val::Int(c)) => *c,
            _ => 0,
        };
        let mcu_clock = self.mcu.clock32_to_clock64(clock);
        let chip_clock = (((resp[2] & 0x7E) as i64) << 9) | (((resp[4] & 0x3E) as i64) << 4);
        // Calculate temperature
        let temper = resp[5] as i64 - (((resp[4] & 0x01) as i64) << 8);
        self.last_temperature.set(Some((temper + 152) as f64 / 2.776));
        Ok((mcu_clock, chip_clock))
    }

    fn cmd_angle_debug_read(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reg = get_reg_param(gcmd, "REG", 0x30 as f64)?;
        let val = self.read_reg(reg)?;
        gcmd.respond_info(&format!("ANGLE REG[0x{:02x}] = 0x{:04x}", reg, val), true);
        Ok(())
    }

    fn cmd_angle_debug_write(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reg = get_reg_param(gcmd, "REG", 0x30 as f64)?;
        let val = get_reg_param(gcmd, "VAL", 0xFFFF as f64)?;
        self.write_reg(reg, val)
    }
}

impl AngleSensor for HelperTLE5012B {
    fn is_tcode_absolute(&self) -> bool {
        true
    }
    fn last_temperature(&self) -> Option<f64> {
        self.last_temperature.get()
    }
    fn clear_temperature(&self) {
        self.last_temperature.set(None);
    }
    /// python's HelperTLE5012B has no get_static_delay (never called: its
    /// tcode is absolute)
    fn get_static_delay(&self) -> f64 {
        0.0
    }
    fn get_tcode_params(&self) -> (i64, i64, f64) {
        (self.last_chip_mcu_clock.get(), self.last_chip_clock.get(), self.chip_freq.get())
    }
    fn update_clock(&self) -> KResult<()> {
        let (mcu_clock, chip_clock) = self.query_clock()?;
        let mdiff = mcu_clock - self.last_chip_mcu_clock.get();
        let chip_mclock = self.last_chip_clock.get() + (mdiff as f64 * self.chip_freq.get() + 0.5) as i64;
        let mut cdiff = (chip_clock - chip_mclock) & 0xFFFF;
        cdiff -= (cdiff & 0x8000) << 1;
        let new_chip_clock = chip_mclock + cdiff;
        if mdiff == 0 {
            return Err(KError::Py("ZeroDivisionError".into(), "float division by zero".into()));
        }
        self.chip_freq.set((new_chip_clock - self.last_chip_clock.get()) as f64 / mdiff as f64);
        self.last_chip_clock.set(new_chip_clock);
        self.last_chip_mcu_clock.set(mcu_clock);
        Ok(())
    }
    fn start(&self) -> KResult<()> {
        // Clear any errors from device
        self.read_reg(0x00)?; // Read STAT
        // Initialize chip (so different chip variants work the same way)
        self.mask_reg(0x06, 0xC003, 0x4000)?; // MOD1: 42.7us, IIF disable
        self.mask_reg(0x08, 0x0007, 0x0001)?; // MOD2: Predict off, autocal=1
        self.mask_reg(0x0E, 0x0003, 0x0000)?; // MOD4: IIF mode
        // Setup starting clock values
        let (mcu_clock, chip_clock) = self.query_clock()?;
        self.last_chip_clock.set(chip_clock);
        self.last_chip_mcu_clock.set(mcu_clock);
        self.chip_freq.set((1 << 5) as f64 / self.mcu.seconds_to_clock(1.0 / 750000.0) as f64);
        self.update_clock()
    }
}

struct HelperMT6816 {
    spi: Rc<McuSpi>,
    oid: i64,
    mcu: Rc<Mcu>,
    spi_angle_transfer_cmd: RefCell<Option<CommandQueryWrapper>>,
}

impl HelperMT6816 {
    const SPI_MODE: i64 = 3;
    const SPI_SPEED: i64 = 10000000;

    fn new(config: &ConfigWrapper, spi: &Rc<McuSpi>, oid: i64) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mcu = spi.get_mcu();
        let h = Rc::new(HelperMT6816 { spi: spi.clone(), oid, mcu: mcu.clone(), spi_angle_transfer_cmd: RefCell::new(None) });
        let hh = h.clone();
        mcu.register_config_callback(Rc::new(move || {
            *hh.spi_angle_transfer_cmd.borrow_mut() = Some(angle_transfer_cmd(&hh.mcu, &hh.spi, hh.oid)?);
            Ok(())
        }));
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let hh = h.clone();
        gcode.register_mux_command("ANGLE_DEBUG_READ", "CHIP", Some(&name), Rc::new(move |g| hh.cmd_angle_debug_read(g)), Some(CMD_ANGLE_DEBUG_READ_HELP))?;
        Ok(h)
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        let msg = [reg as u8, 0, 0];
        let resp = response(&self.spi.spi_transfer(&msg, 0, 0)?);
        Ok(((resp[1] as i64) << 8) | resp[2] as i64)
    }

    fn cmd_angle_debug_read(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reg = 0x83;
        let val = self.read_reg(reg)?;
        gcmd.respond_info(&format!("ANGLE REG[0x{:02x}] = 0x{:04x}", reg, val), true);
        let angle = val >> 2;
        let parity = (val >> 1).count_ones() % 2;
        gcmd.respond_info(&format!("Angle {} ~ {:.2}", angle, angle as f64 * 360.0 / (1 << 14) as f64), true);
        gcmd.respond_info(&format!("No Mag: {}", val >> 1 & 0x1), true);
        gcmd.respond_info(&format!("Parity: {} == {}", parity, val & 0x1), true);
        Ok(())
    }
}

impl AngleSensor for HelperMT6816 {
    fn get_static_delay(&self) -> f64 {
        0.000001
    }
    fn start(&self) -> KResult<()> {
        Ok(())
    }
}

const CMD_ANGLE_CHIP_CALIBRATE_HELP: &str = "Run MT6826s calibration sequence";

struct HelperMT6826S {
    printer: Rc<Printer>,
    stepper_name: Option<String>,
    spi: Rc<McuSpi>,
    oid: i64,
    mcu: Rc<Mcu>,
    spi_angle_transfer_cmd: RefCell<Option<CommandQueryWrapper>>,
}

impl HelperMT6826S {
    const SPI_MODE: i64 = 3;
    const SPI_SPEED: i64 = 10000000;

    fn new(config: &ConfigWrapper, spi: &Rc<McuSpi>, oid: i64) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let stepper_name = config.get_opt("stepper")?;
        let mcu = spi.get_mcu();
        let h = Rc::new(HelperMT6826S {
            printer: printer.clone(),
            stepper_name,
            spi: spi.clone(),
            oid,
            mcu: mcu.clone(),
            spi_angle_transfer_cmd: RefCell::new(None),
        });
        let hh = h.clone();
        mcu.register_config_callback(Rc::new(move || {
            *hh.spi_angle_transfer_cmd.borrow_mut() = Some(angle_transfer_cmd(&hh.mcu, &hh.spi, hh.oid)?);
            Ok(())
        }));
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let hh = h.clone();
        gcode.register_mux_command("ANGLE_DEBUG_READ", "CHIP", Some(&name), Rc::new(move |g| hh.cmd_angle_debug_read(g)), Some(CMD_ANGLE_DEBUG_READ_HELP))?;
        let hh = h.clone();
        gcode.register_mux_command(
            "ANGLE_CHIP_CALIBRATE",
            "CHIP",
            Some(&name),
            Rc::new(move |g| hh.cmd_angle_chip_calibrate(g)),
            Some(CMD_ANGLE_CHIP_CALIBRATE_HELP),
        )?;
        Ok(h)
    }

    fn status_map(code: i64) -> &'static str {
        match code {
            0 => "No Calibration",
            1 => "Running Calibration",
            2 => "Calibration Failed",
            _ => "Calibration Successful",
        }
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        let reg = 0x3000 | reg;
        let msg = [(reg >> 8) as u8, (reg & 0xFF) as u8, 0];
        let resp = response(&self.spi.spi_transfer(&msg, 0, 0)?);
        Ok(resp[2] as i64)
    }

    fn write_reg(&self, reg: i64, data: i64) -> KResult<()> {
        let reg = 0x6000 | reg;
        let msg = [(reg >> 8) as u8, (reg & 0xFF) as u8, data as u8];
        self.spi.spi_transfer(&msg, 0, 0)?;
        Ok(())
    }

    fn crc8(data: &[u8]) -> u8 {
        let polynomial = 0x07;
        let mut crc: u8 = 0x00;
        for byte in data {
            crc ^= byte;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ polynomial;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

    fn read_angle(&self, reg: i64) -> KResult<(i64, i64, i64, i64)> {
        let reg = 0x3000 | reg;
        let msg = [(reg >> 8) as u8, (reg & 0xFF) as u8, 0, 0, 0, 0];
        let resp = response(&self.spi.spi_transfer(&msg, 0, 0)?);
        let angle = ((resp[2] as i64) << 7) | (resp[3] >> 1) as i64;
        let status = resp[4] as i64;
        let crc_computed = Self::crc8(&[resp[2], resp[3], resp[4]]) as i64;
        let crc = resp[5] as i64;
        Ok((angle, status, crc, crc_computed))
    }

    fn cmd_angle_chip_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let fmove = self.printer.lookup::<ForceMove>("force_move")?;
        let mcu_stepper = fmove.lookup_stepper(self.stepper_name.as_deref().unwrap_or("None"))?;
        let Some(stepper_name) = &self.stepper_name else {
            gcmd.respond_info("stepper not defined", true);
            return Ok(());
        };

        gcmd.respond_info("MT6826S Run calibration sequence", true);
        gcmd.respond_info("Motor will do 18+ rotations - ensure pulley is disconnected", true);
        let req_freq = self.read_reg(0x00E)? >> 4 & 0x7;
        // Minimal calibration speed
        let rpm = (3200 >> req_freq) + 1;
        let rps = rpm as f64 / 60.0;
        let mv = |dist: f64, speed: f64| fmove.manual_move(&mcu_stepper, dist, speed, 0.0);
        // Move stepper several turns (to allow internal sensor calibration)
        let (microsteps, full_steps) = get_microsteps(&self.printer, stepper_name)?;
        let step_dist = mcu_stepper.get_step_dist();
        let full_step_dist = step_dist * microsteps as f64;
        let rotation_dist = full_steps as f64 * full_step_dist;
        mv(2.0 * rotation_dist, rps * rotation_dist)?;
        self.write_reg(0x155, 0x5E)?;
        mv(20.0 * rotation_dist, rps * rotation_dist)?;
        let val = self.read_reg(0x113)?;
        let mut code = val >> 6;
        gcmd.respond_info(&format!("Status: {}", Self::status_map(code)), true);
        while code == 1 {
            mv(5.0 * rotation_dist, rps * rotation_dist)?;
            let val = self.read_reg(0x113)?;
            code = val >> 6;
            gcmd.respond_info(&format!("Status: {}", Self::status_map(code)), true);
        }
        if code == 2 {
            gcmd.respond_info("Calibration failed", true);
        }
        if code == 3 {
            gcmd.respond_info("Calibration success, please poweroff sensor", true);
        }
        Ok(())
    }

    fn cmd_angle_debug_read(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let reg = get_reg_param(gcmd, "REG", 0x155 as f64)?;
        if reg == 0x003 {
            let (angle, status, crc1, crc2) = self.read_angle(reg)?;
            gcmd.respond_info(&format!("ANGLE REG[0x003] = 0x{:02x}", angle >> 7), true);
            gcmd.respond_info(&format!("ANGLE REG[0x004] = 0x{:02x}", (angle << 1) & 0xFF), true);
            gcmd.respond_info(&format!("Angle {} ~ {:.2}", angle, angle as f64 * 360.0 / (1 << 15) as f64), true);
            gcmd.respond_info(&format!("Weak Mag: {}", status >> 1 & 0x1), true);
            gcmd.respond_info(&format!("Under Voltage: {}", status >> 2 & 0x1), true);
            gcmd.respond_info(&format!("CRC: 0x{:02x} == 0x{:02x}", crc1, crc2), true);
        } else if reg == 0x00E {
            let val = self.read_reg(reg)?;
            gcmd.respond_info(&format!("GPIO_DS = {}", val >> 7), true);
            gcmd.respond_info(&format!("AUTOCAL_FREQ = {}", val >> 4 & 0x7), true);
        } else if reg == 0x113 {
            let val = self.read_reg(reg)?;
            gcmd.respond_info(&format!("Status: {}", Self::status_map(val >> 6)), true);
        } else {
            let val = self.read_reg(reg)?;
            gcmd.respond_info(&format!("REG[0x{:04x}] = 0x{:02x}", reg, val), true);
        }
        Ok(())
    }
}

impl AngleSensor for HelperMT6826S {
    fn get_static_delay(&self) -> f64 {
        0.00001
    }
    fn start(&self) -> KResult<()> {
        let val = self.read_reg(0x00D)?;
        // Set histeresis to 0.003 degree
        self.write_reg(0x00D, (val & 0xF8) | 0x5)
    }
}

const BYTES_PER_SAMPLE: usize = 3;
const SAMPLES_PER_BLOCK: i64 = (MAX_BULK_MSG_SIZE / BYTES_PER_SAMPLE) as i64;

const SAMPLE_PERIOD: f64 = 0.000400;
const BATCH_UPDATES: f64 = 0.100;

pub struct Angle {
    printer: Rc<Printer>,
    sample_period: f64,
    calibration: Rc<AngleCalibration>,
    // Measurement conversion
    start_clock: Cell<i64>,
    time_shift: Cell<i64>,
    sample_ticks: Cell<i64>,
    last_sequence: Cell<i64>,
    last_angle: Cell<i64>,
    spi: Rc<McuSpi>,
    mcu: Rc<Mcu>,
    oid: i64,
    sensor_helper: Rc<dyn AngleSensor>,
    query_spi_angle_cmd: RefCell<Option<CommandWrapper>>,
    bulk_queue: BulkDataQueue,
    batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    name: String,
}

impl Angle {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let sample_period = config.getfloat("sample_period", SAMPLE_PERIOD, Lim::new().above(0.0))?;
        let calibration = AngleCalibration::new(config)?;
        // Sensor type
        let sensors = ["a1333", "as5047d", "tle5012b", "mt6816", "mt6826s"];
        let choices: Vec<(&str, &str)> = sensors.iter().map(|s| (*s, *s)).collect();
        let sensor_type = config.getchoice("sensor_type", &choices, Dflt::Required)?;
        let (spi_mode, spi_speed) = match sensor_type {
            "a1333" => (HelperA1333::SPI_MODE, HelperA1333::SPI_SPEED),
            "as5047d" => (HelperAS5047D::SPI_MODE, HelperAS5047D::SPI_SPEED),
            "tle5012b" => (HelperTLE5012B::SPI_MODE, HelperTLE5012B::SPI_SPEED),
            "mt6816" => (HelperMT6816::SPI_MODE, HelperMT6816::SPI_SPEED),
            _ => (HelperMT6826S::SPI_MODE, HelperMT6826S::SPI_SPEED),
        };
        let spi = mcu_spi_from_config(config, spi_mode, "cs_pin", spi_speed, None, false)?;
        let mcu = spi.get_mcu();
        let oid = mcu.create_oid();
        let sensor_helper: Rc<dyn AngleSensor> = match sensor_type {
            "a1333" => Rc::new(HelperA1333 { spi: spi.clone() }),
            "as5047d" => Rc::new(HelperAS5047D { spi: spi.clone() }),
            "tle5012b" => HelperTLE5012B::new(config, &spi, oid)?,
            "mt6816" => HelperMT6816::new(config, &spi, oid)?,
            _ => HelperMT6826S::new(config, &spi, oid)?,
        };
        // Setup mcu sensor_spi_angle bulk query code
        mcu.add_config_cmd(&format!("config_spi_angle oid={} spi_oid={} spi_angle_type={}", oid, spi.get_oid(), sensor_type), false, false);
        mcu.add_config_cmd(&format!("query_spi_angle oid={} clock=0 rest_ticks=0 time_shift=0", oid), false, true);
        let bulk_queue = BulkDataQueue::new(&mcu, "sensor_bulk_data", Some(oid));
        let name = config
            .get_name()
            .split_whitespace()
            .nth(1)
            .ok_or_else(|| KError::Py("IndexError".into(), "list index out of range".into()))?
            .to_string();
        let a = Rc::new(Angle {
            printer: printer.clone(),
            sample_period,
            calibration,
            start_clock: Cell::new(0),
            time_shift: Cell::new(0),
            sample_ticks: Cell::new(0),
            last_sequence: Cell::new(0),
            last_angle: Cell::new(0),
            spi,
            mcu: mcu.clone(),
            oid,
            sensor_helper,
            query_spi_angle_cmd: RefCell::new(None),
            bulk_queue,
            batch_bulk: RefCell::new(None),
            name: name.clone(),
        });
        let aa = a.clone();
        mcu.register_config_callback(Rc::new(move || aa.build_config()));
        // Process messages in batches
        let batch_bulk = batch_helper(&printer, &a, Self::process_batch, Self::start_measurements, Self::finish_measurements, BATCH_UPDATES);
        let mut api_resp = IndexMap::default();
        api_resp.insert("header".into(), PyVal::Tuple(vec![PyVal::Str("time".into()), PyVal::Str("angle".into())]));
        batch_bulk.add_mux_endpoint("angle/dump_angle", "sensor", &name, PyVal::Dict(api_resp))?;
        *a.batch_bulk.borrow_mut() = Some(batch_bulk);
        Ok(a)
    }

    fn build_config(&self) -> KResult<()> {
        let freq = self.mcu.seconds_to_clock(1.0);
        while ((TCODE_ERROR << self.time_shift.get()) as f64 / freq as f64) < 0.002 {
            self.time_shift.set(self.time_shift.get() + 1);
        }
        let cmdqueue = self.spi.get_command_queue();
        *self.query_spi_angle_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_spi_angle oid=%c clock=%u rest_ticks=%u time_shift=%c", Some(cmdqueue))?);
        Ok(())
    }

    pub fn add_client(&self, client_cb: ClientCallback) -> KResult<()> {
        let bb = self.batch_bulk.borrow().clone().ok_or_else(|| KError::Other("angle batch helper missing".into()))?;
        bb.add_client(client_cb)
    }

    // Measurement decoding
    fn extract_samples(&self, raw_samples: &[Params]) -> (Vec<AngleSample>, i64) {
        // Load variables to optimize inner loop below
        let sample_ticks = self.sample_ticks.get();
        let start_clock = self.start_clock.get();
        let mut last_sequence = self.last_sequence.get();
        let mut last_angle = self.last_angle.get();
        let mut time_shift = 0;
        let mut static_delay = 0.0;
        let (mut last_chip_mcu_clock, mut last_chip_clock, mut chip_freq, mut inv_chip_freq) = (0, 0, 0.0, 0.0);
        let is_tcode_absolute = self.sensor_helper.is_tcode_absolute();
        if is_tcode_absolute {
            let tparams = self.sensor_helper.get_tcode_params();
            (last_chip_mcu_clock, last_chip_clock, chip_freq) = tparams;
            inv_chip_freq = 1.0 / chip_freq;
        } else {
            time_shift = self.time_shift.get();
            static_delay = self.sensor_helper.get_static_delay();
        }
        // Process every message in raw_samples
        let mut error_count = 0;
        let mut samples = Vec::with_capacity(raw_samples.len() * SAMPLES_PER_BLOCK as usize);
        for params in raw_samples {
            let sequence = match params.get("sequence") {
                Some(Val::Int(s)) => *s,
                _ => 0,
            };
            let seq_diff = (sequence - last_sequence) & 0xFFFF;
            last_sequence += seq_diff;
            let samp_count = last_sequence * SAMPLES_PER_BLOCK;
            let msg_mclock = start_clock + samp_count * sample_ticks;
            let d = match params.get("data") {
                Some(Val::Bytes(b)) => b.clone(),
                _ => vec![],
            };
            for i in 0..d.len() / BYTES_PER_SAMPLE {
                let d_ta = &d[i * BYTES_PER_SAMPLE..(i + 1) * BYTES_PER_SAMPLE];
                let tcode = d_ta[0] as i64;
                if tcode == TCODE_ERROR {
                    error_count += 1;
                    continue;
                }
                let raw_angle = d_ta[1] as i64 | ((d_ta[2] as i64) << 8);
                let mut angle_diff = (raw_angle - last_angle) & 0xFFFF;
                angle_diff -= (angle_diff & 0x8000) << 1;
                last_angle += angle_diff;
                let mclock = msg_mclock + i as i64 * sample_ticks;
                let sclock = if is_tcode_absolute {
                    // tcode is tle5012b frame counter
                    let mdiff = mclock - last_chip_mcu_clock;
                    let chip_mclock = last_chip_clock + (mdiff as f64 * chip_freq + 0.5) as i64;
                    let mut cdiff = ((tcode << 10) - chip_mclock) & 0xFFFF;
                    cdiff -= (cdiff & 0x8000) << 1;
                    mclock as f64 + (cdiff - 0x800) as f64 * inv_chip_freq
                } else {
                    // tcode is mcu clock offset shifted by time_shift
                    (mclock + (tcode << time_shift)) as f64
                };
                let ptime = py_round(self.mcu.clock_to_print_time(sclock) - static_delay, 6);
                samples.push((ptime, last_angle));
            }
        }
        self.last_sequence.set(last_sequence);
        self.last_angle.set(last_angle);
        (samples, error_count)
    }

    fn query_cmd(&self) -> KResult<std::cell::Ref<'_, CommandWrapper>> {
        let c = self.query_spi_angle_cmd.borrow();
        if c.is_none() {
            return Err(not_sent());
        }
        Ok(std::cell::Ref::map(c, |c| c.as_ref().unwrap()))
    }

    // Start, stop, and process message batches
    fn start_measurements(&self) -> KResult<()> {
        logging::info(&format!("Starting angle '{}' measurements", self.name));
        self.sensor_helper.start()?;
        // Start bulk reading
        self.bulk_queue.clear_queue();
        self.last_sequence.set(0);
        let systime = self.printer.get_reactor().monotonic();
        let print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME;
        let reqclock = self.mcu.print_time_to_clock(print_time);
        self.start_clock.set(reqclock);
        let rest_ticks = self.mcu.seconds_to_clock(self.sample_period);
        self.sample_ticks.set(rest_ticks);
        self.query_cmd()?.send(&[self.oid, reqclock, rest_ticks, self.time_shift.get()].map(Val::Int), 0, reqclock as u64)
    }

    fn finish_measurements(&self) -> KResult<()> {
        // Halt bulk reading
        self.query_cmd()?.send_wait_ack(&[self.oid, 0, 0, 0].map(Val::Int), 0, 0)?;
        self.bulk_queue.clear_queue();
        self.sensor_helper.clear_temperature();
        logging::info(&format!("Stopped angle '{}' measurements", self.name));
        Ok(())
    }

    fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        if self.sensor_helper.is_tcode_absolute() {
            self.sensor_helper.update_clock()?;
        }
        let raw_samples = self.bulk_queue.pull_queue();
        if raw_samples.is_empty() {
            return Ok(PyVal::dict());
        }
        let (mut samples, error_count) = self.extract_samples(&raw_samples);
        if samples.is_empty() {
            return Ok(PyVal::dict());
        }
        let offset = self.calibration.apply_calibration(&mut samples)?;
        let d = crate::pydict! {
            "data" => PyVal::List(samples.iter().map(|(t, a)| PyVal::Tuple(vec![PyVal::Float(*t), PyVal::Int(*a)])).collect()),
            "errors" => PyVal::Int(error_count),
            "position_offset" => offset.map(PyVal::Float).unwrap_or(PyVal::None),
        };
        Ok(PyVal::Dict(d))
    }
}

impl PrinterObject for Angle {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::default();
        d.insert("temperature".into(), self.sensor_helper.last_temperature().map(PyVal::Float).unwrap_or(PyVal::None));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(Angle::new(config)?)
}

#[cfg(test)]
mod tests {
    use crate::mathutil::numpy_tests::parse_hex;

    /// RK_ANGLE_CASES: tools/gen_angle_cases.py output
    #[test]
    fn angle_calibration_vs_python() {
        let Ok(path) = std::env::var("RK_ANGLE_CASES") else { return };
        let text = std::fs::read_to_string(path).unwrap();
        // "illc" cases have ill conditioned equations (cond > 1e8, e.g. 64
        // full steps all near mid bucket): numpy's dgelsd and np_lstsq's
        // Jacobi SVD round those differently, so they are only reported
        let (mut bad, mut n, mut rand_bad) = (0, 0, 0);
        for line in text.lines() {
            let (lhs, rhs) = line.split_once(" = ").unwrap();
            let (kind, lhs) = lhs.split_once(' ').unwrap();
            let angles: Vec<f64> = lhs.split_whitespace().map(parse_hex).collect();
            let want: Vec<&str> = rhs.split_whitespace().collect();
            let (cal, phase, rev) = super::calc_calibration(&angles).unwrap();
            let k = want.len() - 2;
            let want_cal: Vec<i64> = want[..k].iter().map(|v| v.parse().unwrap()).collect();
            n += 1;
            if cal != want_cal || phase.to_bits() != parse_hex(want[k]).to_bits() || rev != (want[k + 1] == "1") {
                if kind == "illc" {
                    rand_bad += 1;
                    continue;
                }
                bad += 1;
                if bad <= 3 {
                    let diffs: Vec<(usize, i64, i64)> = cal.iter().zip(&want_cal).enumerate().filter(|(_, (a, b))| a != b).map(|(i, (a, b))| (i, *a, *b)).collect();
                    eprintln!("mismatch ({} steps): {:?}", angles.len(), diffs);
                }
            }
        }
        eprintln!("angle calibration: {}/{} mismatches ({} more in ill conditioned data)", bad, n, rand_bad);
        assert_eq!(bad, 0);
    }
}
