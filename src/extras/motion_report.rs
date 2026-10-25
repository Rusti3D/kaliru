// Diagnostic tool for reporting stepper and kinematic positions
// (port of klippy/extras/motion_report.py)
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::stepcompress::PullHistorySteps;
use crate::chelper::trapq::{PullMove, TrapQRef};
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::bulk_sensor::{BatchBulkHelper, BATCH_INTERVAL};
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::gcode::coord_pyval;
use crate::logging;
use crate::mcu::Mcu;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::pyval::PyVal;
use crate::stepper::McuStepper;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

fn tuple(v: &[PyVal]) -> PyVal {
    PyVal::Tuple(v.to_vec())
}

fn strs(v: &[&str]) -> PyVal {
    PyVal::Tuple(v.iter().map(|s| PyVal::from(*s)).collect())
}

fn header(fields: &[&str]) -> PyVal {
    let mut d = IndexMap::new();
    d.insert("header".to_string(), strs(fields));
    PyVal::Dict(d)
}

// Extract stepper queue_step messages
pub struct DumpStepper {
    pub mcu_stepper: Rc<McuStepper>,
    last_batch_clock: Cell<u64>,
    batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
}

impl DumpStepper {
    fn new(printer: &Rc<Printer>, mcu_stepper: &Rc<McuStepper>) -> KResult<Rc<Self>> {
        let ds = Rc::new(DumpStepper { mcu_stepper: mcu_stepper.clone(), last_batch_clock: Cell::new(0), batch_bulk: RefCell::new(None) });
        let w = Rc::downgrade(&ds);
        let batch_bulk = BatchBulkHelper::new(
            printer,
            Rc::new(move |eventtime| match w.upgrade() {
                Some(ds) => Ok(ds.process_batch(eventtime)),
                None => Ok(PyVal::dict()),
            }),
            None,
            None,
            BATCH_INTERVAL,
        );
        batch_bulk.add_mux_endpoint("motion_report/dump_stepper", "name", &mcu_stepper.get_name(false), header(&["interval", "count", "add"]))?;
        *ds.batch_bulk.borrow_mut() = Some(batch_bulk);
        Ok(ds)
    }

    pub fn get_step_queue(&self, start_clock: u64, mut end_clock: u64) -> Vec<PullHistorySteps> {
        let mut res: Vec<Vec<PullHistorySteps>> = vec![];
        loop {
            let data = self.mcu_stepper.dump_steps(128, start_clock, end_clock);
            let count = data.len();
            if count == 0 {
                break;
            }
            end_clock = data[count - 1].first_clock;
            res.push(data);
            if count < 128 {
                break;
            }
        }
        res.reverse();
        res.into_iter().flat_map(|d| d.into_iter().rev()).collect()
    }

    pub fn log_steps(&self, data: &[PullHistorySteps]) {
        if data.is_empty() {
            return;
        }
        let mut out = vec![format!(
            "Dumping stepper '{}' ({}) {} queue_step:",
            self.mcu_stepper.get_name(false),
            self.mcu_stepper.get_mcu().get_name(),
            data.len()
        )];
        for (i, s) in data.iter().enumerate() {
            out.push(format!(
                "queue_step {}: t={} p={} i={} c={} a={}",
                i, s.first_clock, s.start_position, s.interval, s.step_count, s.add
            ));
        }
        logging::info(&out.join("\n"));
    }

    fn process_batch(&self, _eventtime: f64) -> PyVal {
        let mut data = self.get_step_queue(self.last_batch_clock.get(), 1 << 63);
        if data.is_empty() {
            return PyVal::dict();
        }
        if let Some(i) = data.iter().position(|d| d.step_count == 0) {
            // End block on a set_position marker to simplify timing
            data.truncate(i + 1);
        }
        let mcu = self.mcu_stepper.get_mcu();
        let first = &data[0];
        let first_clock = first.first_clock;
        let first_time = mcu.clock_to_print_time(first_clock as f64);
        let last_clock = data[data.len() - 1].last_clock;
        self.last_batch_clock.set(last_clock);
        let last_time = mcu.clock_to_print_time(last_clock as f64);
        let mcu_pos = first.start_position;
        let start_position = self.mcu_stepper.mcu_to_commanded_position(mcu_pos);
        let step_dist = self.mcu_stepper.get_step_dist();
        let tdata: Vec<PyVal> = data
            .iter()
            .map(|s| tuple(&[PyVal::Int(s.interval as i64), PyVal::Int(s.step_count as i64), PyVal::Int(s.add as i64)]))
            .collect();
        let mut d = IndexMap::new();
        d.insert("data".to_string(), PyVal::List(tdata));
        d.insert("start_position".to_string(), PyVal::Float(start_position));
        d.insert("start_mcu_position".to_string(), PyVal::Int(mcu_pos));
        d.insert("step_distance".to_string(), PyVal::Float(step_dist));
        d.insert("first_clock".to_string(), PyVal::Int(first_clock as i64));
        d.insert("first_step_time".to_string(), PyVal::Float(first_time));
        d.insert("last_clock".to_string(), PyVal::Int(last_clock as i64));
        d.insert("last_step_time".to_string(), PyVal::Float(last_time));
        PyVal::Dict(d)
    }
}

pub const NEVER_TIME: f64 = 9999999999999999.0;

/// one trapq entry as _process_batch sends it
type BatchMsg = (f64, f64, f64, f64, [f64; 3], [f64; 3]);

// Extract trapezoidal motion queue (trapq)
pub struct DumpTrapQ {
    name: String,
    trapq: TrapQRef,
    last_batch_msg: Cell<(f64, f64)>,
    last_batch_entry: RefCell<Option<BatchMsg>>,
    motion_queuing: Rc<PrinterMotionQueuing>,
    batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
}

impl DumpTrapQ {
    fn new(printer: &Rc<Printer>, name: &str, trapq: TrapQRef) -> KResult<Rc<Self>> {
        let dt = Rc::new(DumpTrapQ {
            name: name.to_string(),
            trapq,
            last_batch_msg: Cell::new((0.0, 0.0)),
            last_batch_entry: RefCell::new(None),
            motion_queuing: printer.lookup::<PrinterMotionQueuing>("motion_queuing")?,
            batch_bulk: RefCell::new(None),
        });
        let w = Rc::downgrade(&dt);
        let batch_bulk = BatchBulkHelper::new(
            printer,
            Rc::new(move |eventtime| match w.upgrade() {
                Some(dt) => Ok(dt.process_batch(eventtime)),
                None => Ok(PyVal::dict()),
            }),
            None,
            None,
            BATCH_INTERVAL,
        );
        batch_bulk.add_mux_endpoint(
            "motion_report/dump_trapq",
            "name",
            name,
            header(&["time", "duration", "start_velocity", "acceleration", "start_position", "direction"]),
        )?;
        *dt.batch_bulk.borrow_mut() = Some(batch_bulk);
        Ok(dt)
    }

    pub fn extract_trapq(&self, start_time: f64, mut end_time: f64) -> Vec<PullMove> {
        let mut res: Vec<Vec<PullMove>> = vec![];
        loop {
            let data = self.trapq.borrow().extract_old(128, start_time, end_time);
            let count = data.len();
            if count == 0 {
                break;
            }
            end_time = data[count - 1].print_time;
            res.push(data);
            if count < 128 {
                break;
            }
        }
        res.reverse();
        res.into_iter().flat_map(|d| d.into_iter().rev()).collect()
    }

    pub fn log_trapq(&self, data: &[PullMove]) {
        if data.is_empty() {
            return;
        }
        let mut out = vec![format!("Dumping trapq '{}' {} moves:", self.name, data.len())];
        for (i, m) in data.iter().enumerate() {
            out.push(format!(
                "move {}: pt={:.6} mt={:.6} sv={:.6} a={:.6} sp=({:.6},{:.6},{:.6}) ar=({:.6},{:.6},{:.6})",
                i, m.print_time, m.move_t, m.start_v, m.accel, m.start_x, m.start_y, m.start_z, m.x_r, m.y_r, m.z_r
            ));
        }
        logging::info(&out.join("\n"));
    }

    pub fn get_trapq_position(&self, print_time: f64) -> Option<([f64; 3], f64)> {
        let data = self.trapq.borrow().extract_old(1, 0.0, print_time);
        let mv = data.first()?;
        let move_time = py_max(0.0, py_min(mv.move_t, print_time - mv.print_time));
        let dist = (mv.start_v + 0.5 * mv.accel * move_time) * move_time;
        let pos = [mv.start_x + mv.x_r * dist, mv.start_y + mv.y_r * dist, mv.start_z + mv.z_r * dist];
        let velocity = mv.start_v + mv.accel * move_time;
        Some((pos, velocity))
    }

    fn process_batch(&self, _eventtime: f64) -> PyVal {
        let (lt, ld) = self.last_batch_msg.get();
        let qtime = lt + py_min(ld, 0.100);
        let data = self.extract_trapq(qtime, NEVER_TIME);
        let mut d: Vec<BatchMsg> = data
            .iter()
            .map(|m| (m.print_time, m.move_t, m.start_v, m.accel, [m.start_x, m.start_y, m.start_z], [m.x_r, m.y_r, m.z_r]))
            .collect();
        if !d.is_empty() {
            if let Some(start_drip_time) = self.motion_queuing.check_drip_timing() {
                // If homing, delay sending trapq entries that may change
                while d.last().map(|e| e.0 + e.1 >= start_drip_time).unwrap_or(false) {
                    d.pop();
                }
            }
        }
        if !d.is_empty() && Some(d[0]) == *self.last_batch_entry.borrow() {
            d.remove(0);
        }
        if d.is_empty() {
            return PyVal::dict();
        }
        let last = d[d.len() - 1];
        self.last_batch_msg.set((last.0, last.1));
        *self.last_batch_entry.borrow_mut() = Some(last);
        let f = |v: &[f64]| PyVal::Tuple(v.iter().map(|x| PyVal::Float(*x)).collect());
        let items: Vec<PyVal> = d
            .iter()
            .map(|e| tuple(&[PyVal::Float(e.0), PyVal::Float(e.1), PyVal::Float(e.2), PyVal::Float(e.3), f(&e.4), f(&e.5)]))
            .collect();
        let mut out = IndexMap::new();
        out.insert("data".to_string(), PyVal::List(items));
        PyVal::Dict(out)
    }

    fn teardown(&self) {
        if let Some(b) = self.batch_bulk.borrow_mut().take() {
            b.teardown();
        }
    }
}

pub const STATUS_REFRESH_TIME: f64 = 0.250;

pub struct PrinterMotionReport {
    printer: Rc<Printer>,
    steppers: RefCell<IndexMap<String, Rc<DumpStepper>>>,
    dtrapqs: RefCell<IndexMap<String, Rc<DumpTrapQ>>>,
    next_status_time: Cell<f64>,
    last_status: RefCell<IndexMap<String, PyVal>>,
}

impl PrinterMotionReport {
    /// self.trapqs[name]
    pub fn get_dump_trapq(&self, name: &str) -> Option<Rc<DumpTrapQ>> {
        self.dtrapqs.borrow().get(name).cloned()
    }

    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // get_status information
        let mut last_status = IndexMap::new();
        last_status.insert("live_position".to_string(), coord_pyval(&[0.0, 0.0, 0.0, 0.0]));
        last_status.insert("live_velocity".to_string(), PyVal::Float(0.0));
        last_status.insert("live_extruder_velocity".to_string(), PyVal::Float(0.0));
        last_status.insert("steppers".to_string(), PyVal::List(vec![]));
        last_status.insert("trapq".to_string(), PyVal::List(vec![]));
        let mr = Rc::new(PrinterMotionReport {
            printer: printer.clone(),
            steppers: RefCell::new(IndexMap::new()),
            dtrapqs: RefCell::new(IndexMap::new()),
            next_status_time: Cell::new(0.0),
            last_status: RefCell::new(last_status),
        });
        // Register handlers
        let m = mr.clone();
        crate::events::KlippyConnect::on(&printer, move || m.connect());
        let m = mr.clone();
        crate::events::KlippyAnalyzeShutdown::on(&printer, move |msg, details| {
            m.handle_analyze_shutdown(&msg, &details);
            Ok(())
        });
        Ok(mr)
    }

    pub fn register_stepper(&self, _config: &ConfigWrapper, mcu_stepper: &Rc<McuStepper>) -> KResult<()> {
        let ds = DumpStepper::new(&self.printer, mcu_stepper)?;
        self.steppers.borrow_mut().insert(mcu_stepper.get_name(false), ds);
        Ok(())
    }

    fn connect(&self) -> KResult<()> {
        // Lookup toolhead trapq
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let trapq = toolhead.get_trapq();
        self.dtrapqs.borrow_mut().insert("toolhead".into(), DumpTrapQ::new(&self.printer, "toolhead", trapq)?);
        // Lookup extruder trapqs
        for i in 0..99 {
            let ename = if i == 0 { "extruder".to_string() } else { format!("extruder{}", i) };
            let Some(extruder) = self.printer.lookup_object_opt(&ename) else { break };
            let etrapq = crate::kinematics::extruder::extruder_trapq(&extruder)?;
            self.dtrapqs.borrow_mut().insert(ename.clone(), DumpTrapQ::new(&self.printer, &ename, etrapq)?);
        }
        // Populate 'trapq' and 'steppers' in get_status result
        let sorted = |keys: Vec<String>| {
            let mut k = keys;
            k.sort();
            PyVal::List(k.into_iter().map(PyVal::Str).collect())
        };
        let steppers = sorted(self.steppers.borrow().keys().cloned().collect());
        let trapqs = sorted(self.dtrapqs.borrow().keys().cloned().collect());
        let mut ls = self.last_status.borrow_mut();
        ls.insert("steppers".into(), steppers);
        ls.insert("trapq".into(), trapqs);
        Ok(())
    }

    // Shutdown handling
    fn handle_analyze_shutdown(&self, msg: &str, details: &PyVal) {
        if msg != "MCU shutdown" {
            return;
        }
        let PyVal::Dict(details) = details else { return };
        let mcu_name = details.get("mcu").map(|v| v.str()).unwrap_or_default();
        let Some(mcu) = self.printer.lookup_object_opt(&mcu_name).and_then(crate::printer::downcast_rc::<Mcu>) else {
            return;
        };
        let Some(shutdown_clock) = details.get("shutdown_clock").and_then(|v| v.as_i64()) else {
            return;
        };
        let shutdown_time = mcu.clock_to_print_time(shutdown_clock as f64);
        let clock_100ms = mcu.seconds_to_clock(0.100);
        let start_clock = 0.max(shutdown_clock - clock_100ms);
        let end_clock = shutdown_clock + clock_100ms;
        // Log stepper queue_steps on mcu that started shutdown (if any)
        for dstepper in self.steppers.borrow().values() {
            if !Rc::ptr_eq(&dstepper.mcu_stepper.get_mcu(), &mcu) {
                continue;
            }
            let data = dstepper.get_step_queue(start_clock as u64, end_clock as u64);
            dstepper.log_steps(&data);
        }
        // Log trapqs around time of shutdown
        for dtrapq in self.dtrapqs.borrow().values() {
            let data = dtrapq.extract_trapq(shutdown_time - 0.100, shutdown_time + 0.100);
            dtrapq.log_trapq(&data);
        }
        // Log estimated toolhead position at time of shutdown
        let Some(dtrapq) = self.dtrapqs.borrow().get("toolhead").cloned() else { return };
        if let Some((pos, _velocity)) = dtrapq.get_trapq_position(shutdown_time) {
            let p = PyVal::Tuple(pos.iter().map(|v| PyVal::Float(*v)).collect());
            logging::info(&format!("Requested toolhead position at shutdown time {:.6}: {}", shutdown_time, p.repr()));
        }
    }

    // Status reporting
    pub fn get_status(&self, eventtime: f64) -> KResult<PyVal> {
        if eventtime < self.next_status_time.get() || self.dtrapqs.borrow().is_empty() {
            return Ok(PyVal::Dict(self.last_status.borrow().clone()));
        }
        self.next_status_time.set(eventtime + STATUS_REFRESH_TIME);
        let mut xyzpos = [0.0, 0.0, 0.0];
        let mut epos = 0.0;
        let (mut xyzvelocity, mut evelocity) = (0.0, 0.0);
        // Calculate current requested toolhead position
        let mcu = self.printer.lookup::<Mcu>("mcu")?;
        let print_time = mcu.estimated_print_time(eventtime);
        let th = self.dtrapqs.borrow().get("toolhead").cloned().expect("toolhead trapq");
        if let Some((pos, velocity)) = th.get_trapq_position(print_time) {
            xyzpos = pos;
            xyzvelocity = velocity;
        }
        // Calculate requested position of currently active extruder
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let ehandler = self.dtrapqs.borrow().get(&toolhead.get_extruder().get_name()).cloned();
        if let Some(ehandler) = ehandler {
            if let Some((pos, velocity)) = ehandler.get_trapq_position(print_time) {
                epos = pos[0];
                evelocity = velocity;
            }
        }
        // Report status
        let mut ls = self.last_status.borrow_mut();
        ls.insert("live_position".into(), coord_pyval(&[xyzpos[0], xyzpos[1], xyzpos[2], epos]));
        ls.insert("live_velocity".into(), PyVal::Float(xyzvelocity));
        ls.insert("live_extruder_velocity".into(), PyVal::Float(evelocity));
        Ok(PyVal::Dict(ls.clone()))
    }
}

impl PrinterObject for PrinterMotionReport {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        PrinterMotionReport::get_status(self, eventtime).ok()
    }
    fn try_get_status(&self, eventtime: f64) -> KResult<Option<PyVal>> {
        PrinterMotionReport::get_status(self, eventtime).map(Some)
    }
    fn teardown(&self) {
        for d in self.dtrapqs.borrow().values() {
            d.teardown();
        }
        for d in self.steppers.borrow().values() {
            if let Some(b) = d.batch_bulk.borrow_mut().take() {
                b.teardown();
            }
        }
        self.dtrapqs.borrow_mut().clear();
        self.steppers.borrow_mut().clear();
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterMotionReport::new(config)?)
}
