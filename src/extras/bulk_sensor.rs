// Tools for reading bulk sensor data from the mcu
// (port of klippy/extras/bulk_sensor.py)
//
// Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Ported so far: BatchBulkHelper, BatchWebhooksClient. BulkDataQueue,
// ClockSyncRegression and FixedFreqReader arrive with the sensor modules.
use crate::error::KResult;
use crate::logging;
use crate::printer::Printer;
use crate::pyval::PyVal;
use crate::reactor::{Timer, NEVER};
use crate::webhooks::{ClientConnection, WebHooks, WebRequest};
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

pub const BATCH_INTERVAL: f64 = 0.500;

/// batch_cb(eventtime) -> msg (an empty dict when there is nothing)
pub type BatchCallback = Rc<dyn Fn(f64) -> KResult<PyVal>>;
pub type StartStopCallback = Rc<dyn Fn() -> KResult<()>>;
/// client_cb(msg) -> keep receiving
pub type ClientCallback = Rc<dyn Fn(&PyVal) -> bool>;

// Helper to process accumulated messages in periodic batches
pub struct BatchBulkHelper {
    me: Weak<BatchBulkHelper>,
    printer: Rc<Printer>,
    batch_cb: BatchCallback,
    start_cb: Option<StartStopCallback>,
    stop_cb: Option<StartStopCallback>,
    is_started: Cell<bool>,
    batch_interval: f64,
    batch_timer: RefCell<Option<Timer>>,
    client_cbs: RefCell<Vec<ClientCallback>>,
    webhooks_start_resp: RefCell<PyVal>,
}

impl BatchBulkHelper {
    pub fn new(
        printer: &Rc<Printer>,
        batch_cb: BatchCallback,
        start_cb: Option<StartStopCallback>,
        stop_cb: Option<StartStopCallback>,
        batch_interval: f64,
    ) -> Rc<Self> {
        Rc::new_cyclic(|me| BatchBulkHelper {
            me: me.clone(),
            printer: printer.clone(),
            batch_cb,
            start_cb,
            stop_cb,
            is_started: Cell::new(false),
            batch_interval,
            batch_timer: RefCell::new(None),
            client_cbs: RefCell::new(vec![]),
            webhooks_start_resp: RefCell::new(PyVal::dict()),
        })
    }

    // Periodic batch processing
    fn start(&self) -> KResult<()> {
        if self.is_started.get() {
            return Ok(());
        }
        self.is_started.set(true);
        if let Some(cb) = &self.start_cb {
            if let Err(e) = cb() {
                if e.is_command_error() {
                    logging::exception_err("BatchBulkHelper start callback error", &e);
                    self.is_started.set(false);
                    self.client_cbs.borrow_mut().clear();
                }
                return Err(e);
            }
        }
        let reactor = self.printer.get_reactor();
        let systime = reactor.monotonic();
        let waketime = systime + self.batch_interval;
        let w = self.me.clone();
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(b) => b.proc_batch(eventtime),
                None => NEVER,
            }),
            waketime,
        );
        *self.batch_timer.borrow_mut() = Some(timer);
        Ok(())
    }

    fn stop(&self) {
        self.client_cbs.borrow_mut().clear();
        if let Some(t) = self.batch_timer.borrow_mut().take() {
            self.printer.get_reactor().unregister_timer(&t);
        }
        if !self.is_started.get() {
            return;
        }
        if let Some(cb) = &self.stop_cb {
            if let Err(e) = cb() {
                logging::exception_err("BatchBulkHelper stop callback error", &e);
                self.client_cbs.borrow_mut().clear();
            }
        }
        self.is_started.set(false);
        if !self.client_cbs.borrow().is_empty() {
            // New client started while in process of stopping
            if let Err(e) = self.start() {
                logging::exception_err("BatchBulkHelper start callback error", &e);
            }
        }
    }

    fn proc_batch(&self, eventtime: f64) -> f64 {
        let msg = match (self.batch_cb)(eventtime) {
            Ok(m) => m,
            Err(e) => {
                logging::exception_err("BatchBulkHelper batch callback error", &e);
                self.stop();
                return NEVER;
            }
        };
        if !msg.truthy() {
            return eventtime + self.batch_interval;
        }
        let cbs = self.client_cbs.borrow().clone();
        for client_cb in cbs {
            let res = client_cb(&msg);
            if !res {
                // This client no longer needs updates - unregister it
                let mut c = self.client_cbs.borrow_mut();
                if let Some(pos) = c.iter().position(|cb| Rc::ptr_eq(cb, &client_cb)) {
                    c.remove(pos);
                }
                let empty = c.is_empty();
                drop(c);
                if empty {
                    self.stop();
                    return NEVER;
                }
            }
        }
        eventtime + self.batch_interval
    }

    // Client registration
    pub fn add_client(&self, client_cb: ClientCallback) -> KResult<()> {
        self.client_cbs.borrow_mut().push(client_cb);
        self.start()
    }

    // Webhooks registration
    fn add_api_client(&self, web_request: &WebRequest) -> KResult<()> {
        let whbatch = BatchWebhooksClient::new(web_request)?;
        self.add_client(Rc::new(move |msg| whbatch.handle_batch(msg)))?;
        web_request.send(self.webhooks_start_resp.borrow().clone())
    }

    pub fn add_mux_endpoint(&self, path: &str, key: &str, value: &str, webhooks_start_resp: PyVal) -> KResult<()> {
        *self.webhooks_start_resp.borrow_mut() = webhooks_start_resp;
        let wh = self.printer.lookup::<WebHooks>("webhooks")?;
        let w = self.me.clone();
        wh.register_mux_endpoint(
            path,
            key,
            Some(value),
            Rc::new(move |wr| match w.upgrade() {
                Some(b) => b.add_api_client(wr),
                None => Ok(()),
            }),
        )
    }

    /// not in python: drop the timer and clients
    pub fn teardown(&self) {
        self.client_cbs.borrow_mut().clear();
        if let Some(t) = self.batch_timer.borrow_mut().take() {
            self.printer.get_reactor().unregister_timer(&t);
        }
    }
}

// A webhooks wrapper for use by BatchBulkHelper
pub struct BatchWebhooksClient {
    cconn: Option<Rc<ClientConnection>>,
    template: PyVal,
}

impl BatchWebhooksClient {
    pub fn new(web_request: &WebRequest) -> KResult<Self> {
        Ok(BatchWebhooksClient {
            cconn: web_request.get_client_connection(),
            template: web_request.get_dict("response_template", Some(PyVal::dict()))?,
        })
    }

    pub fn handle_batch(&self, msg: &PyVal) -> bool {
        let Some(cconn) = &self.cconn else { return false };
        if cconn.is_closed() {
            return false;
        }
        let mut tmp = match &self.template {
            PyVal::Dict(d) => d.clone(),
            _ => Default::default(),
        };
        tmp.insert("params".to_string(), msg.clone());
        cconn.send(&PyVal::Dict(tmp));
        true
    }
}
