// More verbose information on micro-controller errors
// (port of klippy/extras/error_mcu.py)
//
// Copyright (C) 2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::py_float_repr;
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::rc::Rc;

const MESSAGE_SHUTDOWN: &str = "
Once the underlying issue is corrected, use the
\"FIRMWARE_RESTART\" command to reset the firmware, reload the
config, and restart the host software.
Printer is shutdown
";

const MESSAGE_PROTOCOL_ERROR1: &str = "
This is frequently caused by running an older version of the
firmware on the MCU(s). Fix by recompiling and flashing the
firmware.
";

const MESSAGE_PROTOCOL_ERROR2: &str = "
Once the underlying issue is corrected, use the \"RESTART\"
command to reload the config and restart the host software.
";

const MESSAGE_MCU_CONNECT_ERROR: &str = "
Once the underlying issue is corrected, use the
\"FIRMWARE_RESTART\" command to reset the firmware, reload the
config, and restart the host software.
Error configuring printer
";

const COMMON_MCU_ERRORS: &[(&[&str], &str)] = &[
    (
        &["Timer too close"],
        "
This often indicates the host computer is overloaded. Check
for other processes consuming excessive CPU time, high swap
usage, disk errors, overheating, unstable voltage, or
similar system problems on the host computer.",
    ),
    (
        &["Missed scheduling of next "],
        "
This is generally indicative of an intermittent
communication failure between micro-controller and host.",
    ),
    (
        &["ADC out of range", "Thermocouple reader fault"],
        "
This generally occurs when a heater temperature exceeds
its configured min_temp or max_temp.",
    ),
    (
        &["Rescheduled timer in the past", "Stepper too far in past"],
        "
This generally occurs when the micro-controller has been
requested to step at a rate higher than it is capable of
obtaining.",
    ),
    (
        &["Command request"],
        "
This generally occurs in response to an M112 G-Code command
or in response to an internal error in the host software.",
    ),
];

pub fn error_hint(msg: &str) -> &'static str {
    for (prefixes, help_msg) in COMMON_MCU_ERRORS {
        for prefix in *prefixes {
            if msg.starts_with(prefix) {
                return help_msg;
            }
        }
    }
    ""
}

/// a clarify callback: cb(msg, details) -> message or None
pub type ClarifyCallback = Rc<dyn Fn(&str, &PyVal) -> Option<String>>;

pub struct PrinterMcuError {
    printer: Rc<Printer>,
    clarify_callbacks: RefCell<IndexMap<String, Vec<ClarifyCallback>>>,
}

fn dict_str(details: &PyVal, key: &str) -> String {
    match details {
        PyVal::Dict(d) => d.get(key).map(|v| v.str()).unwrap_or_default(),
        _ => String::new(),
    }
}

/// " ".join(text.splitlines())[1:]
fn join_lines(text: &str) -> String {
    let joined = text.lines().collect::<Vec<_>>().join(" ");
    joined.chars().skip(1).collect()
}

impl PrinterMcuError {
    pub fn new(config: &ConfigWrapper) -> Rc<Self> {
        let printer = config.get_printer();
        let me = Rc::new(PrinterMcuError { printer: printer.clone(), clarify_callbacks: RefCell::new(IndexMap::new()) });
        let m = me.clone();
        printer.register_event_handler(
            "klippy:analyze_shutdown",
            Rc::new(move |args| {
                let (msg, details) = event_args(args);
                m.handle_analyze_shutdown(&msg, &details);
                Ok(())
            }),
        );
        let m = me.clone();
        printer.register_event_handler(
            "klippy:notify_mcu_error",
            Rc::new(move |args| {
                let (msg, details) = event_args(args);
                m.handle_notify_mcu_error(&msg, &details);
                Ok(())
            }),
        );
        me
    }

    pub fn add_clarify(&self, msg: &str, callback: ClarifyCallback) {
        self.clarify_callbacks.borrow_mut().entry(msg.to_string()).or_default().push(callback);
    }

    fn check_mcu_shutdown(&self, msg: &str, details: &PyVal) {
        let mcu_name = dict_str(details, "mcu");
        let mcu_msg = dict_str(details, "reason");
        let event_type = dict_str(details, "event_type");
        let mut prefix = format!("MCU '{}' shutdown: ", mcu_name);
        if event_type == "is_shutdown" {
            prefix = format!("Previous MCU '{}' shutdown: ", mcu_name);
        }
        // Lookup generic hint
        let hint = error_hint(&mcu_msg);
        // Add per instance help
        let cbs = self.clarify_callbacks.borrow().get(&mcu_msg).cloned().unwrap_or_default();
        let clarify: Vec<String> = cbs.iter().filter_map(|cb| cb(msg, details)).collect();
        let mut clarify_msg = String::new();
        if !clarify.is_empty() {
            let mut parts = vec![String::new(), String::new()];
            parts.extend(clarify);
            parts.push(String::new());
            clarify_msg = parts.join("\n");
        }
        // Update error message
        let newmsg = format!("{}{}{}{}{}", prefix, mcu_msg, clarify_msg, hint, MESSAGE_SHUTDOWN);
        self.printer.update_error_msg(msg, &newmsg);
    }

    fn handle_analyze_shutdown(&self, msg: &str, details: &PyVal) {
        if msg == "MCU shutdown" {
            self.check_mcu_shutdown(msg, details);
        } else {
            self.printer.update_error_msg(msg, &format!("{}{}", msg, MESSAGE_SHUTDOWN));
        }
        // Report reactor info (no good place to do this, so done here)
        let (a, b, c) = self.printer.get_reactor().get_gc_stats();
        logging::info(&format!(
            "Reactor garbage collection: ({}, {}, {})",
            py_float_repr(a),
            py_float_repr(b),
            py_float_repr(c)
        ));
    }

    fn check_protocol_error(&self, msg: &str, details: &PyVal) {
        let host_version = self.printer.start_arg("software_version").map(|v| v.str()).unwrap_or_default();
        let mut msg_update = vec![];
        let mut msg_updated = vec![];
        for (mcu_name, mcu) in self.printer.lookup_objects(Some("mcu")) {
            let mcu_version = match mcu.get_status(0.) {
                Some(PyVal::Dict(d)) if d.contains_key("mcu_version") => d["mcu_version"].str(),
                _ => {
                    logging::exception("Unable to retrieve mcu_version from mcu");
                    continue;
                }
            };
            let short = mcu_name.split_whitespace().last().unwrap_or("").to_string();
            if mcu_version != host_version {
                msg_update.push(format!("{}: Current version {}", short, mcu_version));
            } else {
                msg_updated.push(format!("{}: Current version {}", short, mcu_version));
            }
        }
        if msg_updated.is_empty() {
            msg_updated.push("<none>".to_string());
        }
        let version_msg = [
            format!("\nYour Kalico version is: {}\n", host_version),
            "MCU(s) which should be updated:".to_string(),
            format!("\n{}\n", msg_update.join("\n")),
            "Up-to-date MCU(s):".to_string(),
            format!("\n{}\n", msg_updated.join("\n")),
        ];
        let newmsg = [
            "MCU Protocol error".to_string(),
            String::new(),
            join_lines(MESSAGE_PROTOCOL_ERROR1),
            version_msg.join("\n"),
            join_lines(MESSAGE_PROTOCOL_ERROR2),
            String::new(),
            dict_str(details, "error"),
        ];
        self.printer.update_error_msg(msg, &newmsg.join("\n"));
    }

    fn check_mcu_connect_error(&self, msg: &str, details: &PyVal) {
        let newmsg = format!("{}{}", dict_str(details, "error"), MESSAGE_MCU_CONNECT_ERROR);
        self.printer.update_error_msg(msg, &newmsg);
    }

    fn handle_notify_mcu_error(&self, msg: &str, details: &PyVal) {
        if msg == "Protocol error" {
            self.check_protocol_error(msg, details);
        } else if msg == "MCU error during connect" {
            self.check_mcu_connect_error(msg, details);
        }
    }
}

/// the (msg, details) arguments of the shutdown / mcu error events
/// the (msg, details) arguments of the analyze_shutdown / notify_mcu_error events
pub fn event_args(args: &[&dyn Any]) -> (String, PyVal) {
    let msg = args.first().and_then(|a| a.downcast_ref::<String>()).cloned().unwrap_or_default();
    let details = args.get(1).and_then(|a| a.downcast_ref::<PyVal>()).cloned().unwrap_or(PyVal::None);
    (msg, details)
}

impl PrinterObject for PrinterMcuError {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterMcuError::new(config))
}
