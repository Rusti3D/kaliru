// Utility for querying the current state of all endstops
// (port of klippy/extras/query_endstops.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::EndstopRef;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::{PyDict, PyVal};
use crate::toolhead::ToolHead;
use crate::webhooks::{WebHooks, WebRequest};
use crate::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

const CMD_QUERY_ENDSTOPS_HELP: &str = "Report on the status of each endstop";

pub struct QueryEndstops {
    pub printer: Rc<Printer>,
    pub endstops: RefCell<Vec<(EndstopRef, String)>>,
    pub last_state: RefCell<Vec<(String, i64)>>,
}

fn state_str(t: i64) -> &'static str {
    if t != 0 {
        "TRIGGERED"
    } else {
        "open"
    }
}

impl QueryEndstops {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let q = Rc::new(QueryEndstops { printer: printer.clone(), endstops: RefCell::new(vec![]), last_state: RefCell::new(vec![]) });
        // Register webhook if server is available
        let webhooks = printer.lookup::<WebHooks>("webhooks")?;
        let qq = q.clone();
        webhooks.register_endpoint("query_endstops/status", Rc::new(move |wr| qq.handle_web_request(wr)))?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let qq = q.clone();
        gcode.register_command("QUERY_ENDSTOPS", Some(Rc::new(move |g| qq.cmd_query_endstops(g))), false, Some(CMD_QUERY_ENDSTOPS_HELP))?;
        let qq = q.clone();
        gcode.register_command("M119", Some(Rc::new(move |g| qq.cmd_query_endstops(g))), false, None)?;
        Ok(q)
    }

    pub fn endstops(&self) -> Vec<(EndstopRef, String)> {
        self.endstops.borrow().clone()
    }

    pub fn register_endstop(&self, mcu_endstop: EndstopRef, name: &str) {
        self.endstops.borrow_mut().push((mcu_endstop, name.to_string()));
    }

    fn query(&self) -> KResult<()> {
        let print_time = self.printer.lookup::<ToolHead>("toolhead")?.get_last_move_time()?;
        let endstops = self.endstops.borrow().clone();
        let mut state = vec![];
        for (mcu_endstop, name) in endstops {
            state.push((name, mcu_endstop.query_endstop(print_time)?));
        }
        *self.last_state.borrow_mut() = state;
        Ok(())
    }

    pub fn handle_web_request(&self, web_request: &WebRequest) -> KResult<()> {
        let gc_mutex = self.printer.lookup::<GCodeDispatch>("gcode")?.get_mutex();
        gc_mutex.lock()?;
        let res = self.query();
        gc_mutex.unlock();
        res?;
        let d: PyDict =
            self.last_state.borrow().iter().map(|(name, t)| (name.clone().into(), PyVal::from(state_str(*t)))).collect();
        web_request.send(PyVal::Dict(d))
    }

    pub fn cmd_query_endstops(&self, gcmd: &GCodeCommand) -> KResult<()> {
        // Query the endstops
        self.query()?;
        // Report results
        let msg: Vec<String> = self.last_state.borrow().iter().map(|(name, t)| format!("{}:{}", name, state_str(*t))).collect();
        gcmd.respond_raw(&msg.join(" "));
        Ok(())
    }
}

impl PrinterObject for QueryEndstops {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let lq: PyDict = self.last_state.borrow().iter().map(|(n, v)| (n.clone().into(), PyVal::Int(*v))).collect();
        let mut d = IndexMap::default();
        d.insert("last_query".into(), PyVal::Dict(lq));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        self.endstops.borrow_mut().clear();
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(QueryEndstops::new(config)?)
}
