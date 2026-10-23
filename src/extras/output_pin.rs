// PWM and digital output pin handling (port of klippy/extras/output_pin.py)
//
// Copyright (C) 2017-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Ported so far: GCodeRequestQueue. PrinterOutputPin needs the template
// evaluator (display templates), which is not ported yet.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::mcu::Mcu;
use crate::printer::{downcast_rc, Printer};
use crate::pyfmt::py_max;
use crate::toolhead::ToolHead;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

// ----------------------------------------------------------------------
// G-Code request queuing helper
// ----------------------------------------------------------------------

pub const PIN_MIN_TIME: f64 = 0.100;

/// the "discard" / "delay" actions a request callback may return
#[derive(Clone, Copy, PartialEq)]
pub enum Action {
    Discard,
    Delay,
}

/// callback(print_time, value) -> None or (action, min_wait)
pub type RequestCallback = Rc<dyn Fn(f64, f64) -> KResult<Option<(Action, f64)>>>;

// Helper code to queue g-code requests
pub struct GCodeRequestQueue {
    printer: Rc<Printer>,
    mcu: Rc<Mcu>,
    callback: RequestCallback,
    rqueue: RefCell<Vec<(f64, f64)>>,
    next_min_flush_time: Cell<f64>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    motion_queuing: Rc<PrinterMotionQueuing>,
}

impl GCodeRequestQueue {
    pub fn new(config: &ConfigWrapper, mcu: &Rc<Mcu>, callback: RequestCallback) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let motion_queuing = printer
            .load_object(config, "motion_queuing", true)?
            .and_then(downcast_rc::<PrinterMotionQueuing>)
            .ok_or_else(|| KError::Other("motion_queuing has an unexpected type".into()))?;
        let q = Rc::new(GCodeRequestQueue {
            printer: printer.clone(),
            mcu: mcu.clone(),
            callback,
            rqueue: RefCell::new(vec![]),
            next_min_flush_time: Cell::new(0.0),
            toolhead: RefCell::new(None),
            motion_queuing: motion_queuing.clone(),
        });
        let w = Rc::downgrade(&q);
        motion_queuing.register_flush_callback(
            Rc::new(move |must_flush_time, max_step_gen_time| match w.upgrade() {
                Some(q) => q.flush_notification(must_flush_time, max_step_gen_time),
                None => Ok(()),
            }),
            false,
        );
        let w = Rc::downgrade(&q);
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| {
                if let Some(q) = w.upgrade() {
                    *q.toolhead.borrow_mut() = Some(q.printer.lookup::<ToolHead>("toolhead")?);
                }
                Ok(())
            }),
        );
        Ok(q)
    }

    fn flush_notification(&self, must_flush_time: f64, _max_step_gen_time: f64) -> KResult<()> {
        let _min_sched_time = self.mcu.min_schedule_time();
        loop {
            let (next_time, pos, req_val) = {
                let rqueue = self.rqueue.borrow();
                if rqueue.is_empty() {
                    return Ok(());
                }
                let next_time = py_max(rqueue[0].0, self.next_min_flush_time.get());
                if next_time > must_flush_time {
                    return Ok(());
                }
                // Skip requests that have been overridden with a following request
                let mut pos = 0;
                while pos + 1 < rqueue.len() && rqueue[pos + 1].0 <= next_time {
                    pos += 1;
                }
                (next_time, pos, rqueue[pos].1)
            };
            // Invoke callback for the request
            let mut min_wait = 0.0;
            let mut del = pos + 1;
            if let Some((action, mw)) = (self.callback)(next_time, req_val)? {
                // Handle special cases
                min_wait = mw;
                if action == Action::Discard {
                    self.rqueue.borrow_mut().drain(..pos + 1);
                    continue;
                }
                if action == Action::Delay {
                    del = pos;
                }
            }
            self.rqueue.borrow_mut().drain(..del);
            self.next_min_flush_time.set(next_time + py_max(min_wait, PIN_MIN_TIME));
            // Ensure following queue items are flushed
            self.motion_queuing.note_mcu_movequeue_activity(self.next_min_flush_time.get(), false);
        }
    }

    fn queue_request(&self, print_time: f64, value: f64) {
        self.rqueue.borrow_mut().push((print_time, value));
        self.motion_queuing.note_mcu_movequeue_activity(print_time, false);
    }

    pub fn queue_gcode_request(self: &Rc<Self>, value: f64) -> KResult<()> {
        let toolhead = self
            .toolhead
            .borrow()
            .clone()
            .ok_or_else(|| KError::Other("'NoneType' object has no attribute 'register_lookahead_callback'".into()))?;
        let q = self.clone();
        toolhead.register_lookahead_callback(Rc::new(move |pt| {
            q.queue_request(pt, value);
            Ok(())
        }))
    }

    pub fn send_async_request(&self, value: f64, print_time: Option<f64>) -> KResult<()> {
        let print_time = match print_time {
            Some(pt) => pt,
            None => {
                let systime = self.printer.get_reactor().monotonic();
                self.mcu.estimated_print_time(systime + PIN_MIN_TIME)
            }
        };
        loop {
            let next_time = py_max(print_time, self.next_min_flush_time.get());
            // Invoke callback for the request
            let (mut action, mut min_wait) = (None, 0.0);
            if let Some((a, mw)) = (self.callback)(next_time, value)? {
                // Handle special cases
                action = Some(a);
                min_wait = mw;
                if a == Action::Discard {
                    break;
                }
            }
            self.next_min_flush_time.set(next_time + py_max(min_wait, PIN_MIN_TIME));
            if action != Some(Action::Delay) {
                break;
            }
        }
        Ok(())
    }
}
