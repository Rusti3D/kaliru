// Report canbus connection status (port of klippy/extras/canbus_stats.py)
//
// Copyright (C) 2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::logging;
use crate::mcu::{CommandQueryWrapper, Mcu};
use crate::msgproto::{Params, Val};
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

/// the status dict: rx_error, tx_error, tx_retries, bus_state
#[derive(Clone, Default)]
struct Status {
    rx_error: Option<i64>,
    tx_error: Option<i64>,
    tx_retries: Option<i64>,
    bus_state: Option<PyVal>,
}

pub struct PrinterCANBusStats {
    me: Weak<PrinterCANBusStats>,
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    name: String,
    mcu: RefCell<Option<Rc<Mcu>>>,
    get_canbus_status_cmd: RefCell<Option<CommandQueryWrapper>>,
    status: RefCell<Status>,
}

impl PrinterCANBusStats {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let s = Rc::new_cyclic(|me| PrinterCANBusStats {
            me: me.clone(),
            printer: printer.clone(),
            reactor: printer.get_reactor(),
            name: config.get_name().split_whitespace().last().unwrap_or_default().to_string(),
            mcu: RefCell::new(None),
            get_canbus_status_cmd: RefCell::new(None),
            status: RefCell::new(Status::default()),
        });
        let w = s.me.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match w.upgrade() {
                Some(s) => s.handle_connect(),
                None => Ok(()),
            }),
        );
        let w = s.me.clone();
        printer.register_event_handler(
            "klippy:shutdown",
            Rc::new(move |_| {
                if let Some(s) = w.upgrade() {
                    s.handle_shutdown();
                }
                Ok(())
            }),
        );
        Ok(s)
    }

    fn handle_shutdown(&self) {
        let mut status = self.status.borrow_mut();
        if status.bus_state.is_some() {
            // Clear bus_state on shutdown to note that the values may be stale
            status.bus_state = Some(PyVal::Str("unknown".into()));
        }
    }

    fn handle_connect(&self) -> KResult<()> {
        // Lookup mcu
        let mut mcu_name = self.name.clone();
        if mcu_name != "mcu" {
            mcu_name = format!("mcu {}", mcu_name);
        }
        let mcu = downcast_rc::<Mcu>(self.printer.lookup_object(&mcu_name)?).ok_or_else(|| KError::Other(format!("{} has an unexpected type", mcu_name)))?;
        *self.mcu.borrow_mut() = Some(mcu.clone());
        // Lookup status query command
        if mcu.try_lookup_command("get_canbus_status").is_none() {
            return Ok(());
        }
        *self.get_canbus_status_cmd.borrow_mut() = Some(mcu.lookup_query_command(
            "get_canbus_status",
            "canbus_status rx_error=%u tx_error=%u tx_retries=%u canbus_bus_state=%u",
            None,
            None,
            false,
        )?);
        // Register usb_canbus_state message handling (for usb to canbus bridge)
        let w = self.me.clone();
        mcu.register_response(
            Some(Rc::new(move |params: &Params| {
                if let Some(s) = w.upgrade() {
                    s.handle_usb_canbus_state(params);
                }
                Ok(())
            })),
            "usb_canbus_state",
            None,
        );
        // Register periodic query timer
        let w = self.me.clone();
        self.reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(s) => s.query_event(eventtime),
                None => crate::reactor::NEVER,
            }),
            NOW,
        );
        Ok(())
    }

    fn handle_usb_canbus_state(&self, params: &Params) {
        let discard = params.get("discard").and_then(|v| v.as_int()).unwrap_or(0);
        if discard != 0 {
            logging::warning(&format!("USB CANBUS bridge '{}' is discarding!", self.name));
        } else {
            logging::warning(&format!("USB CANBUS bridge '{}' is no longer discarding.", self.name));
        }
    }

    fn query_event(&self, _eventtime: f64) -> f64 {
        let prev = self.status.borrow().clone();

        // skip querying disconnected MCU and maintain previous status
        let disconnected = self.mcu.borrow().as_ref().map(|m| m.non_critical_disconnected.get()).unwrap_or(false);
        if disconnected || self.get_canbus_status_cmd.borrow().is_none() {
            *self.status.borrow_mut() = Status { bus_state: Some(PyVal::Str("disconnected".into())), ..prev };
            return self.reactor.monotonic() + 1.0;
        }

        let (prev_rx, prev_tx, prev_retries) = (prev.rx_error.unwrap_or(0), prev.tx_error.unwrap_or(0), prev.tx_retries.unwrap_or(0));
        let params = {
            let cmd = self.get_canbus_status_cmd.borrow();
            match cmd.as_ref().map(|c| c.send(&[], 0, 0, true)) {
                Some(Ok(p)) => p,
                // The connection can drop while a query is in flight (e.g. a
                // non-critical mcu disconnecting) - keep stats and retry later
                Some(Err(e)) if matches!(e, KError::Serial(_)) || e.is_command_error() => return self.reactor.monotonic() + 1.0,
                Some(Err(e)) => {
                    logging::exception_err("canbus_stats query", &e);
                    return self.reactor.monotonic() + 1.0;
                }
                None => return self.reactor.monotonic() + 1.0,
            }
        };
        let p = |k: &str| params.get(k).and_then(|v| v.as_int()).unwrap_or(0);
        let rx = prev_rx + ((p("rx_error") - prev_rx) & 0xFFFFFFFF);
        let tx = prev_tx + ((p("tx_error") - prev_tx) & 0xFFFFFFFF);
        let retries = prev_retries + ((p("tx_retries") - prev_retries) & 0xFFFFFFFF);
        let state = match params.get("canbus_bus_state") {
            Some(Val::Str(s)) => PyVal::Str(s.clone()),
            Some(v) => PyVal::Int(v.as_int().unwrap_or(0)),
            None => PyVal::None,
        };
        *self.status.borrow_mut() = Status { rx_error: Some(rx), tx_error: Some(tx), tx_retries: Some(retries), bus_state: Some(state) };
        self.reactor.monotonic() + 1.0
    }
}

impl PrinterObject for PrinterCANBusStats {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        let status = self.status.borrow();
        let Some(rx) = status.rx_error else { return Some((false, String::new())) };
        Some((
            false,
            format!(
                "canstat_{}: bus_state={} rx_error={} tx_error={} tx_retries={}",
                self.name,
                status.bus_state.as_ref().map(|v| v.str()).unwrap_or_else(|| "None".into()),
                rx,
                status.tx_error.unwrap_or(0),
                status.tx_retries.unwrap_or(0)
            ),
        ))
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let status = self.status.borrow();
        let opt = |v: Option<i64>| v.map(PyVal::Int).unwrap_or(PyVal::None);
        let mut d = IndexMap::new();
        d.insert("rx_error".to_string(), opt(status.rx_error));
        d.insert("tx_error".to_string(), opt(status.tx_error));
        d.insert("tx_retries".to_string(), opt(status.tx_retries));
        d.insert("bus_state".to_string(), status.bus_state.clone().unwrap_or(PyVal::None));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterCANBusStats::new(config)?)
}
