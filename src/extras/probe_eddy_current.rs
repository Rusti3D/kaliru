// Support for eddy current based Z probes
// (port of klippy/extras/probe_eddy_current.py)
//
// Copyright (C) 2021-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::{HomingMove, PrinterHoming};
use crate::extras::ldc1612::{EddySensorFactory, FreqSample, LDC1612};
use crate::extras::load_cell::lookup_sensor_class;
use crate::extras::manual_probe::ManualProbeHelper;
use crate::extras::probe::{McuProbe, PrinterProbe, ProbeDistanceCollector, ProbeDistanceResult};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::{Endstop, EndstopRef, Mcu, McuTrsync, StepperRef, TriggerDispatch};
use crate::printer::{ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_str_repr, py_sum};
use crate::pyval::PyVal;
use crate::reactor::ReactorCompletion;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

/// bisect.bisect (bisect_right) of a sorted list
fn bisect(a: &[f64], x: f64) -> usize {
    a.partition_point(|v| *v <= x)
}

/// the msg["data"] rows of a sensor batch message
fn msg_data(msg: &PyVal) -> Vec<FreqSample> {
    let PyVal::Dict(d) = msg else { return vec![] };
    let Some(PyVal::List(rows)) = d.get("data") else { return vec![] };
    rows.iter()
        .filter_map(|r| match r {
            PyVal::Tuple(v) if v.len() == 3 => Some((v[0].as_f64()?, v[1].as_f64()?, v[2].as_f64()?)),
            _ => None,
        })
        .collect()
}

const CMD_EDDY_CALIBRATE_HELP: &str = "Calibrate eddy current probe";

// Tool for calibrating the sensor Z detection and applying that calibration
pub struct EddyCalibration {
    me: Weak<EddyCalibration>,
    printer: Rc<Printer>,
    name: String,
    pprobe: RefCell<Option<Rc<PrinterProbe>>>,
    // Current calibration data
    cal_freqs: RefCell<Vec<f64>>,
    cal_zpos: RefCell<Vec<f64>>,
    // Probe calibrate state
    probe_speed: Cell<f64>,
}

impl EddyCalibration {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let c = Rc::new_cyclic(|me| EddyCalibration {
            me: me.clone(),
            printer: printer.clone(),
            name: config.get_name().to_string(),
            pprobe: RefCell::new(None),
            cal_freqs: RefCell::new(vec![]),
            cal_zpos: RefCell::new(vec![]),
            probe_speed: Cell::new(0.0),
        });
        if let Some(cal) = config.get_opt("calibrate")? {
            let mut parsed = vec![];
            for d in cal.split(',') {
                let mut row = vec![];
                for v in d.trim().splitn(2, ':') {
                    let f = py_float(v).ok_or_else(|| KError::Py("ValueError".into(), format!("could not convert string to float: {}", py_str_repr(v))))?;
                    row.push(f);
                }
                if row.len() < 2 {
                    return Err(KError::Py("IndexError".into(), "list index out of range".into()));
                }
                parsed.push(row);
            }
            c.load_calibration(&parsed);
        }
        // Register commands
        let cname = c.name.split_whitespace().last().unwrap_or_default().to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let cc = c.clone();
        gcode.register_mux_command(
            "PROBE_EDDY_CURRENT_CALIBRATE",
            "CHIP",
            Some(&cname),
            Rc::new(move |gcmd| cc.cmd_eddy_calibrate(gcmd)),
            Some(CMD_EDDY_CALIBRATE_HELP),
        )?;
        Ok(c)
    }

    pub fn is_calibrated(&self) -> bool {
        self.cal_freqs.borrow().len() > 2
    }

    fn load_calibration(&self, cal: &[Vec<f64>]) {
        let mut cal: Vec<(f64, f64)> = cal.iter().map(|c| (c[1], c[0])).collect();
        cal.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        *self.cal_freqs.borrow_mut() = cal.iter().map(|c| c.0).collect();
        *self.cal_zpos.borrow_mut() = cal.iter().map(|c| c.1).collect();
    }

    pub fn apply_calibration(&self, samples: &mut [FreqSample]) {
        let cal_freqs = self.cal_freqs.borrow();
        let cal_zpos = self.cal_zpos.borrow();
        for s in samples.iter_mut() {
            let (samp_time, freq, _dummy_z) = *s;
            let pos = bisect(&cal_freqs, freq);
            let zpos = if pos >= cal_zpos.len() {
                -99.9
            } else if pos == 0 {
                99.9
            } else {
                // XXX - could further optimize and avoid div by zero
                let this_freq = cal_freqs[pos];
                let prev_freq = cal_freqs[pos - 1];
                let this_zpos = cal_zpos[pos];
                let prev_zpos = cal_zpos[pos - 1];
                let gain = (this_zpos - prev_zpos) / (this_freq - prev_freq);
                let offset = prev_zpos - prev_freq * gain;
                freq * gain + offset
            };
            *s = (samp_time, freq, crate::pyfmt::py_round(zpos, 6));
        }
    }

    pub fn height_to_freq(&self, height: f64) -> KResult<f64> {
        // XXX - could optimize lookup
        let rev_zpos: Vec<f64> = self.cal_zpos.borrow().iter().rev().copied().collect();
        let rev_freqs: Vec<f64> = self.cal_freqs.borrow().iter().rev().copied().collect();
        let pos = bisect(&rev_zpos, height);
        if pos == 0 || pos >= rev_zpos.len() {
            return Err(KError::command("Invalid probe_eddy_current height"));
        }
        let this_freq = rev_freqs[pos];
        let prev_freq = rev_freqs[pos - 1];
        let this_zpos = rev_zpos[pos];
        let prev_zpos = rev_zpos[pos - 1];
        let gain = (this_freq - prev_freq) / (this_zpos - prev_zpos);
        let offset = prev_freq - prev_zpos * gain;
        Ok(height * gain + offset)
    }

    /// the {kin z: [freqs]} measurements, in insertion order
    fn do_calibration_moves(&self, move_speed: f64) -> KResult<Vec<(f64, Vec<f64>)>> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let kin = toolhead.get_kinematics();
        let all = |p: &[f64]| p.iter().map(|v| Some(*v)).collect::<Vec<_>>();
        // Start data collection
        let msgs: Rc<RefCell<Vec<PyVal>>> = Rc::new(RefCell::new(vec![]));
        let is_finished = Rc::new(Cell::new(false));
        let (m, f) = (msgs.clone(), is_finished.clone());
        let handle_batch = Rc::new(move |msg: &PyVal| {
            if f.get() {
                return false;
            }
            m.borrow_mut().push(msg.clone());
            true
        });
        self.printer.lookup::<PrinterEddyProbe>(&self.name)?.add_client(handle_batch)?;
        toolhead.dwell(1.0)?;
        // Move to each 40um position
        let max_z = 4.0;
        let samp_dist = 0.040;
        let req_zpos: Vec<f64> = (0..(max_z / samp_dist) as i64 + 1).map(|i| i as f64 * samp_dist).collect();
        let start_pos = toolhead.get_position();
        let mut times = vec![];
        for zpos in req_zpos {
            // Move to next position (always descending to reduce backlash)
            let mut hop_pos = start_pos.clone();
            hop_pos[2] += zpos + 0.500;
            toolhead.manual_move(&all(&hop_pos), move_speed)?;
            let mut next_pos = start_pos.clone();
            next_pos[2] += zpos;
            toolhead.manual_move(&all(&next_pos), move_speed)?;
            // Note sample timing
            let start_query_time = toolhead.get_last_move_time()? + 0.050;
            let end_query_time = start_query_time + 0.100;
            toolhead.dwell(0.200)?;
            // Find Z position based on actual commanded stepper position
            toolhead.flush_step_generation()?;
            let kin_spos: IndexMap<String, f64> = kin.get_steppers().iter().map(|s| (s.get_name(false), s.get_commanded_position())).collect();
            let kin_pos = kin.calc_position(&kin_spos);
            times.push((start_query_time, end_query_time, kin_pos[2]));
        }
        toolhead.dwell(1.0)?;
        toolhead.wait_moves()?;
        // Finish data collection
        is_finished.set(true);
        // Correlate query responses
        let mut cal: Vec<(f64, Vec<f64>)> = vec![];
        let mut step = 0;
        for msg in msgs.borrow().iter() {
            for (query_time, freq, _old_z) in msg_data(msg) {
                // Add to step tracking
                while step < times.len() && query_time > times[step].1 {
                    step += 1;
                }
                if step < times.len() && query_time >= times[step].0 {
                    let z = times[step].2;
                    match cal.iter_mut().find(|(k, _)| *k == z) {
                        Some((_, v)) => v.push(freq),
                        None => cal.push((z, vec![freq])),
                    }
                }
            }
        }
        if cal.len() != times.len() {
            return Err(KError::command("Failed calibration - incomplete sensor data"));
        }
        Ok(cal)
    }

    fn calc_freqs(meas: &[(f64, Vec<f64>)]) -> (Vec<(f64, f64)>, f64, i64) {
        let mut total_count = 0;
        let mut total_variance = 0.0;
        let mut positions = vec![];
        for (pos, freqs) in meas {
            let count = freqs.len();
            let freq_avg = py_sum(freqs.iter().copied()) / count as f64;
            positions.push((*pos, freq_avg));
            total_count += count as i64;
            total_variance += py_sum(freqs.iter().map(|f| (f - freq_avg).powi(2)));
        }
        (positions, (total_variance / total_count as f64).sqrt(), total_count)
    }

    fn post_manual_probe(&self, kin_pos: Option<Vec<f64>>) -> KResult<()> {
        let Some(kin_pos) = kin_pos else {
            // Manual Probe was aborted
            return Ok(());
        };
        let mut curpos = kin_pos;
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let all = |p: &[f64]| p.iter().map(|v| Some(*v)).collect::<Vec<_>>();
        let speed = self.probe_speed.get();
        // Move away from the bed
        let probe_calibrate_z = curpos[2];
        curpos[2] += 5.0;
        toolhead.manual_move(&all(&curpos), speed)?;
        // Move sensor over nozzle position
        let pprobe = self.pprobe.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_offsets'".into()))?;
        let (x_offset, y_offset, _z_offset) = pprobe.get_offsets();
        curpos[0] -= x_offset;
        curpos[1] -= y_offset;
        toolhead.manual_move(&all(&curpos), speed)?;
        // Descend back to bed
        curpos[2] -= 5.0 - 0.050;
        toolhead.manual_move(&all(&curpos), speed)?;
        // Perform calibration movement and capture
        let cal = self.do_calibration_moves(speed)?;
        // Calculate each sample position average and variance
        let (mut positions, std, total) = Self::calc_freqs(&cal);
        positions.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut last_freq = 0.0;
        for (_pos, freq) in positions.iter().rev() {
            if *freq <= last_freq {
                return Err(KError::command("Failed calibration - frequency not increasing each step"));
            }
            last_freq = *freq;
        }
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        gcode.respond_info(
            &format!(
                "probe_eddy_current: stddev={:.3} in {} queries\nThe SAVE_CONFIG command will update the printer config file\nand restart the printer.",
                std, total
            ),
            true,
        );
        // Save results
        let mut cal_contents = vec![];
        for (i, (pos, freq)) in positions.iter().enumerate() {
            if i % 3 == 0 {
                cal_contents.push("\n".to_string());
            }
            cal_contents.push(format!("{:.6}:{:.3}", pos - probe_calibrate_z, freq));
            cal_contents.push(",".to_string());
        }
        cal_contents.pop();
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        configfile.pconfig.borrow_mut().set(&self.name, "calibrate", &cal_contents.concat())?;
        Ok(())
    }

    fn cmd_eddy_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.probe_speed.set(gcmd.get_float("PROBE_SPEED", Some(5.0), Lim::new().above(0.0))?);
        // Start manual probe
        let me = self.me.upgrade().expect("eddy calibration");
        ManualProbeHelper::start(&self.printer, gcmd, Rc::new(move |kin_pos| me.post_manual_probe(kin_pos)))
    }
}

struct EddyDistanceCollector {
    endstop_wrapper: Rc<EddyEndstopWrapper>,
    samples: RefCell<Vec<ProbeDistanceResult>>,
    stop_after: Cell<Option<f64>>,
    stop_notify: Rc<ReactorCompletion<Vec<ProbeDistanceResult>>>,
}

impl EddyDistanceCollector {
    fn new(endstop_wrapper: Rc<EddyEndstopWrapper>) -> KResult<Rc<Self>> {
        let stop_notify = endstop_wrapper.printer.get_reactor().completion();
        let c = Rc::new(EddyDistanceCollector {
            endstop_wrapper: endstop_wrapper.clone(),
            samples: RefCell::new(vec![]),
            stop_after: Cell::new(None),
            stop_notify,
        });
        let cc = c.clone();
        endstop_wrapper.sensor_helper.add_client(Rc::new(move |msg| cc.collect_samples(msg)))?;
        Ok(c)
    }

    fn collect_samples(&self, msg: &PyVal) -> bool {
        for sample in msg_data(msg) {
            let time = sample.0;

            if let Some(stop_after) = self.stop_after.get() {
                if time >= stop_after {
                    let samples = std::mem::take(&mut *self.samples.borrow_mut());
                    self.stop_notify.complete(samples);
                    return false;
                }
            }

            self.samples.borrow_mut().push((time, sample.2));
        }

        true
    }
}

impl ProbeDistanceCollector for EddyDistanceCollector {
    fn finish(&self, before_time: f64) -> KResult<Vec<ProbeDistanceResult>> {
        self.stop_after.set(Some(before_time));
        let now = self.endstop_wrapper.printer.get_reactor().monotonic();
        let timeout = before_time - self.endstop_wrapper.mcu.estimated_print_time(now) + 1.0;
        let ret = self.stop_notify.wait(now + timeout, None)?;
        ret.ok_or_else(|| KError::command("Timeout waiting for eddy current probe sample collection"))
    }

    fn sample_rate(&self) -> i64 {
        self.endstop_wrapper.sensor_helper.get_sample_rate()
    }
}

// Helper for implementing PROBE style commands
pub struct EddyEndstopWrapper {
    me: Weak<EddyEndstopWrapper>,
    printer: Rc<Printer>,
    sensor_helper: Rc<LDC1612>,
    mcu: Rc<Mcu>,
    calibration: Rc<EddyCalibration>,
    z_offset: f64,
    dispatch: TriggerDispatch,
    samples: RefCell<Vec<PyVal>>,
    is_sampling: Cell<bool>,
    start_from_home: Cell<bool>,
    need_stop: Cell<bool>,
    trigger_time: Cell<f64>,
}

impl EddyEndstopWrapper {
    const REASON_SENSOR_ERROR: i64 = McuTrsync::REASON_COMMS_TIMEOUT + 1;

    fn new(config: &ConfigWrapper, sensor_helper: Rc<LDC1612>, calibration: Rc<EddyCalibration>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mcu = sensor_helper.get_mcu();
        let z_offset = config.getfloat("z_offset", Dflt::Required, Lim::new().min(0.0))?;
        let dispatch = TriggerDispatch::new(&mcu);
        let w = Rc::new_cyclic(|me| EddyEndstopWrapper {
            me: me.clone(),
            printer: printer.clone(),
            sensor_helper,
            mcu,
            calibration,
            z_offset,
            dispatch,
            samples: RefCell::new(vec![]),
            is_sampling: Cell::new(false),
            start_from_home: Cell::new(false),
            need_stop: Cell::new(false),
            trigger_time: Cell::new(0.0),
        });
        let ww = w.clone();
        printer.register_event_handler("klippy:mcu_identify", Rc::new(move |_| ww.handle_mcu_identify()));
        Ok(w)
    }

    fn handle_mcu_identify(&self) -> KResult<()> {
        let kin = self.printer.lookup::<ToolHead>("toolhead")?.get_kinematics();
        for stepper in kin.get_steppers() {
            if stepper.is_active_axis('z') {
                self.add_stepper(&stepper)?;
            }
        }
        Ok(())
    }

    // Measurement gathering
    fn start_measurements(&self, is_home: bool) -> KResult<()> {
        self.need_stop.set(false);
        if self.is_sampling.get() {
            return Ok(());
        }
        self.is_sampling.set(true);
        self.start_from_home.set(is_home);
        let w = self.me.clone();
        self.sensor_helper.add_client(Rc::new(move |msg| match w.upgrade() {
            Some(w) => w.add_measurement(msg),
            None => false,
        }))
    }

    fn stop_measurements(&self, is_home: bool) {
        if !self.is_sampling.get() || (is_home && !self.start_from_home.get()) {
            return;
        }
        self.need_stop.set(true);
    }

    fn add_measurement(&self, msg: &PyVal) -> bool {
        if self.need_stop.get() {
            self.samples.borrow_mut().clear();
            self.is_sampling.set(false);
            self.need_stop.set(false);
            return false;
        }
        self.samples.borrow_mut().push(msg.clone());
        true
    }
}

impl Endstop for EddyEndstopWrapper {
    // Interface for MCU_endstop
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.dispatch.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        self.dispatch.get_steppers()
    }
    fn home_start(&self, print_time: f64, _sample_time: f64, _sample_count: i64, _rest_time: f64, _triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        self.trigger_time.set(0.0);
        self.start_measurements(true)?;
        let trigger_freq = self.calibration.height_to_freq(self.z_offset)?;
        let trigger_completion = self.dispatch.start(print_time)?;
        self.sensor_helper.setup_home(
            print_time,
            trigger_freq,
            self.dispatch.get_oid(),
            McuTrsync::REASON_ENDSTOP_HIT,
            Self::REASON_SENSOR_ERROR,
        )?;
        Ok(trigger_completion)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.dispatch.wait_end(home_end_time)?;
        let trigger_time = self.sensor_helper.clear_home()?;
        self.stop_measurements(true);
        let res = self.dispatch.stop()?;
        if res == Self::REASON_SENSOR_ERROR {
            return Err(KError::command("Sensor readout error"));
        }
        if res >= McuTrsync::REASON_COMMS_TIMEOUT {
            return Err(KError::command("Communication timeout during homing"));
        }
        if res != McuTrsync::REASON_ENDSTOP_HIT {
            return Ok(0.0);
        }
        if self.mcu.is_fileoutput() {
            return Ok(home_end_time);
        }
        self.trigger_time.set(trigger_time);
        Ok(trigger_time)
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        Ok(0) // XXX
    }
    fn get_position_endstop(&self) -> Option<f64> {
        Some(self.z_offset)
    }
}

impl McuProbe for EddyEndstopWrapper {
    // Interface for ProbeEndstopWrapper
    fn probing_move(&self, pos: &[f64], speed: f64, _gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        // Perform probing move
        let phoming = self.printer.lookup::<PrinterHoming>("homing")?;
        let me: EndstopRef = self.me.upgrade().expect("eddy endstop");
        let trig_pos = phoming.probing_move(me, pos, speed)?;
        let trigger_time = self.trigger_time.get();
        if trigger_time == 0.0 {
            return Ok((trig_pos, Some(true)));
        }
        // Wait for samples to arrive
        let start_time = trigger_time + 0.050;
        let end_time = start_time + 0.100;
        let reactor = self.printer.get_reactor();
        loop {
            let last = self.samples.borrow().last().and_then(|m| msg_data(m).last().copied());
            if let Some(last) = last {
                if last.0 >= end_time {
                    break;
                }
            }
            let systime = reactor.monotonic();
            let est_print_time = self.mcu.estimated_print_time(systime);
            if est_print_time > trigger_time + 1.0 {
                return Err(KError::command("probe_eddy_current sensor outage"));
            }
            reactor.pause(systime + 0.010)?;
        }
        // Find position since trigger
        let samples = std::mem::take(&mut *self.samples.borrow_mut());
        let mut samp_sum = 0.0;
        let mut samp_count = 0;
        for msg in &samples {
            let data = msg_data(msg);
            let (Some(first), Some(last)) = (data.first(), data.last()) else { continue };
            if first.0 > end_time {
                break;
            }
            if last.0 < start_time {
                continue;
            }
            for (time, _freq, z) in data {
                if time >= start_time && time <= end_time {
                    samp_sum += z;
                    samp_count += 1;
                }
            }
        }
        if samp_count == 0 {
            return Err(KError::command("Unable to obtain probe_eddy_current sensor readings"));
        }
        let halt_z = samp_sum / samp_count as f64;
        // Calculate reported "trigger" position
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let mut new_pos = toolhead.get_position();
        new_pos[2] += self.z_offset - halt_z;
        Ok((new_pos, Some(true)))
    }

    fn can_collect_distance(&self) -> bool {
        true
    }

    fn begin_collect_distance(&self) -> KResult<Rc<dyn ProbeDistanceCollector>> {
        Ok(EddyDistanceCollector::new(self.me.upgrade().expect("eddy endstop"))?)
    }

    fn multi_probe_begin(&self, _always_restore_toolhead: bool) -> KResult<()> {
        if !self.calibration.is_calibrated() {
            return Err(KError::command("Must calibrate probe_eddy_current first"));
        }
        self.start_measurements(false)
    }

    fn multi_probe_end(&self) -> KResult<()> {
        self.stop_measurements(false);
        Ok(())
    }

    fn probe_prepare(&self, _hmove: &Rc<HomingMove>) -> KResult<()> {
        Ok(())
    }

    fn probe_finish(&self, _hmove: &Rc<HomingMove>) -> KResult<()> {
        Ok(())
    }
}

// Main "printer object"
pub struct PrinterEddyProbe {
    sensor_helper: Rc<LDC1612>,
}

impl PrinterEddyProbe {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let calibration = EddyCalibration::new(config)?;
        // Sensor type
        let sensor_class = lookup_sensor_class::<EddySensorFactory>(config, "eddy_current_sensors")?;
        let sensor_helper = sensor_class(config, Some(calibration.clone()))?;
        // Probe interface
        let probe = EddyEndstopWrapper::new(config, sensor_helper.clone(), calibration.clone())?;
        *calibration.pprobe.borrow_mut() = Some(PrinterProbe::new(config, probe)?);
        Ok(Rc::new(PrinterEddyProbe { sensor_helper }))
    }

    pub fn add_client(&self, cb: crate::extras::bulk_sensor::ClientCallback) -> KResult<()> {
        self.sensor_helper.add_client(cb)
    }
}

impl PrinterObject for PrinterEddyProbe {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterEddyProbe::new(config)?)
}
