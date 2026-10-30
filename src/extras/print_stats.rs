// Virtual SDCard print stat tracking (port of klippy/extras/print_stats.py)
//
// Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_move::GCodeMove;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::Reactor;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const CMD_SET_PRINT_STATS_INFO_HELP: &str = "Pass slicer info like layer act and total to klipper";

pub struct PrintStats {
    pub printer: Rc<Printer>,
    pub gcode_move: Rc<GCodeMove>,
    pub reactor: Rc<Reactor>,
    pub filename: RefCell<String>,
    pub error_message: RefCell<String>,
    pub state: RefCell<String>,
    pub prev_pause_duration: Cell<f64>,
    pub last_epos: Cell<f64>,
    pub filament_used: Cell<f64>,
    pub total_duration: Cell<f64>,
    pub print_start_time: Cell<Option<f64>>,
    pub last_pause_time: Cell<Option<f64>>,
    pub init_duration: Cell<f64>,
    pub info_total_layer: Cell<Option<i64>>,
    pub info_current_layer: Cell<Option<i64>>,
}

impl PrintStats {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let gcode_move = printer
            .load_object(config, "gcode_move", true)?
            .and_then(downcast_rc::<GCodeMove>)
            .ok_or_else(|| KError::Other("gcode_move has an unexpected type".into()))?;
        let ps = Rc::new(PrintStats {
            printer: printer.clone(),
            gcode_move,
            reactor: printer.get_reactor(),
            filename: RefCell::new(String::new()),
            error_message: RefCell::new(String::new()),
            state: RefCell::new("standby".into()),
            prev_pause_duration: Cell::new(0.0),
            last_epos: Cell::new(0.0),
            filament_used: Cell::new(0.0),
            total_duration: Cell::new(0.0),
            print_start_time: Cell::new(None),
            last_pause_time: Cell::new(None),
            init_duration: Cell::new(0.0),
            info_total_layer: Cell::new(None),
            info_current_layer: Cell::new(None),
        });
        ps.reset()?;
        // Register commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let p = ps.clone();
        gcode.register_command(
            "SET_PRINT_STATS_INFO",
            Some(Rc::new(move |g| p.cmd_set_print_stats_info(g))),
            false,
            Some(CMD_SET_PRINT_STATS_INFO_HELP),
        )?;
        let p = ps.clone();
        crate::events::ExtruderActivateExtruder::on(&printer, move || {
            p.last_epos.set(p.gcode_move.status_position_e());
            Ok(())
        });
        Ok(ps)
    }

    pub fn update_filament_usage(&self, _eventtime: f64) {
        let cur_epos = self.gcode_move.status_position_e();
        self.filament_used
            .set(self.filament_used.get() + (cur_epos - self.last_epos.get()) / self.gcode_move.status_extrude_factor());
        self.last_epos.set(cur_epos);
    }

    pub fn set_current_file(&self, filename: &str) -> KResult<()> {
        self.reset()?;
        *self.filename.borrow_mut() = filename.to_string();
        Ok(())
    }

    pub fn note_start(&self) -> KResult<()> {
        let curtime = self.reactor.monotonic();
        match (self.print_start_time.get(), self.last_pause_time.get()) {
            (None, _) => self.print_start_time.set(Some(curtime)),
            (Some(_), Some(last_pause_time)) => {
                // Update pause time duration
                let pause_duration = curtime - last_pause_time;
                self.prev_pause_duration.set(self.prev_pause_duration.get() + pause_duration);
                self.last_pause_time.set(None);
            }
            _ => {}
        }
        // Reset last e-position
        self.last_epos.set(self.gcode_move.status_position_e());
        *self.state.borrow_mut() = "printing".into();
        crate::events::PrintStatsStartPrinting::emit(&self.printer)?;
        self.error_message.borrow_mut().clear();
        Ok(())
    }

    pub fn note_pause(&self) -> KResult<()> {
        if self.last_pause_time.get().is_none() {
            let curtime = self.reactor.monotonic();
            self.last_pause_time.set(Some(curtime));
            // update filament usage
            self.update_filament_usage(curtime);
        }
        if *self.state.borrow() != "error" {
            *self.state.borrow_mut() = "paused".into();
            crate::events::PrintStatsPausedPrinting::emit(&self.printer)?;
        }
        Ok(())
    }

    pub fn note_complete(&self) -> KResult<()> {
        self.note_finish("complete", "");
        crate::events::PrintStatsCompletePrinting::emit(&self.printer)
    }

    pub fn note_error(&self, message: &str) -> KResult<()> {
        self.note_finish("error", message);
        crate::events::PrintStatsErrorPrinting::emit(&self.printer)
    }

    pub fn note_cancel(&self) -> KResult<()> {
        self.note_finish("cancelled", "");
        crate::events::PrintStatsCancelledPrinting::emit(&self.printer)
    }

    pub fn note_finish(&self, state: &str, error_message: &str) {
        let Some(print_start_time) = self.print_start_time.get() else { return };
        *self.state.borrow_mut() = state.into();
        *self.error_message.borrow_mut() = error_message.into();
        let eventtime = self.reactor.monotonic();
        self.total_duration.set(eventtime - print_start_time);
        if self.filament_used.get() < 0.0000001 {
            // No positive extusion detected during print
            self.init_duration.set(self.total_duration.get() - self.prev_pause_duration.get());
        }
        self.print_start_time.set(None);
    }

    pub fn cmd_set_print_stats_info(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let get_opt_default = |name: &str, default: Option<i64>| -> KResult<Option<i64>> {
            match default {
                Some(d) => Ok(Some(gcmd.get_int(name, Some(d), Lim::new().min(0))?)),
                None => gcmd.get_int_opt(name, Lim::new().min(0)),
            }
        };
        let total_layer = get_opt_default("TOTAL_LAYER", self.info_total_layer.get())?;
        let current_layer = get_opt_default("CURRENT_LAYER", self.info_current_layer.get())?;
        if total_layer == Some(0) {
            self.info_total_layer.set(None);
            self.info_current_layer.set(None);
        } else if total_layer != self.info_total_layer.get() {
            self.info_total_layer.set(total_layer);
            self.info_current_layer.set(Some(0));
        }

        if let (Some(total), Some(current)) = (self.info_total_layer.get(), current_layer) {
            if Some(current) != self.info_current_layer.get() {
                self.info_current_layer.set(Some(current.min(total)));
            }
        }
        Ok(())
    }

    pub fn reset(&self) -> KResult<()> {
        self.filename.borrow_mut().clear();
        self.error_message.borrow_mut().clear();
        *self.state.borrow_mut() = "standby".into();
        self.prev_pause_duration.set(0.0);
        self.last_epos.set(0.0);
        self.filament_used.set(0.0);
        self.total_duration.set(0.0);
        self.print_start_time.set(None);
        self.last_pause_time.set(None);
        self.init_duration.set(0.0);
        self.info_total_layer.set(None);
        self.info_current_layer.set(None);
        crate::events::PrintStatsReset::emit(&self.printer)
    }
}

impl PrinterObject for PrintStats {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        let mut time_paused = self.prev_pause_duration.get();
        if let Some(print_start_time) = self.print_start_time.get() {
            match self.last_pause_time.get() {
                // Calculate the total time spent paused during the print
                Some(last_pause_time) => time_paused += eventtime - last_pause_time,
                // Accumulate filament if not paused
                None => self.update_filament_usage(eventtime),
            }
            self.total_duration.set(eventtime - print_start_time);
            if self.filament_used.get() < 0.0000001 {
                // Track duration prior to extrusion
                self.init_duration.set(self.total_duration.get() - time_paused);
            }
        }
        let print_duration = self.total_duration.get() - self.init_duration.get() - time_paused;
        let opt = |v: Option<i64>| v.map(PyVal::Int).unwrap_or(PyVal::None);
        let info = crate::pydict! {
            "total_layer" => opt(self.info_total_layer.get()),
            "current_layer" => opt(self.info_current_layer.get()),
        };
        let d = crate::pydict! {
            "filename" => PyVal::Str(self.filename.borrow().clone()),
            "total_duration" => PyVal::Float(self.total_duration.get()),
            "print_duration" => PyVal::Float(print_duration),
            "filament_used" => PyVal::Float(self.filament_used.get()),
            "state" => PyVal::Str(self.state.borrow().clone()),
            "message" => PyVal::Str(self.error_message.borrow().clone()),
            "info" => PyVal::Dict(info),
        };
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrintStats::new(config)?)
}
