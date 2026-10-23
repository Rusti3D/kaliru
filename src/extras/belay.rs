// Belay extruder-syncing sensor support (port of klippy/extras/belay.py)
//
// Copyright (C) 2023-2024 Ryan Ghosh <rghosh776@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::buttons::PrinterButtons;
use crate::extras::extruder_stepper::PrinterExtruderStepper;
use crate::extras::trad_rack::TradRack;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::Mcu;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_bool, py_max};
use crate::pyval::PyVal;
use crate::reactor::{Reactor, Timer, NEVER, NOW};
use crate::stepper::McuStepper;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const DIRECTION_UPDATE_INTERVAL: f64 = 0.1;
const POSITION_TIME_DIFF: f64 = 0.3;

#[derive(Clone, Copy, PartialEq)]
enum BelayType {
    TradRack,
    ExtruderStepper,
}

/// the set_multiplier callable
#[derive(Clone)]
enum SetMultiplier {
    /// trad_rack.set_fil_driver_multiplier
    TradRack(Rc<TradRack>),
    /// stepper.set_rotation_distance(base_rotation_dist / m)
    Stepper(Rc<McuStepper>, f64),
}

const CMD_QUERY_BELAY_HELP: &str = "Report Belay sensor state";
const CMD_BELAY_SET_MULTIPLIER_HELP: &str = "Sets multiplier_high and/or multiplier_low. Does not persist across restarts.";
const CMD_BELAY_SET_STEPPER_HELP: &str = "Select the extruder_stepper object to be controlled by the Belay";
const CMD_BELAY_ENABLE_HELP: &str = "Manually enable Belay";
const CMD_BELAY_DISABLE_HELP: &str = "Manually disable Belay";

pub struct Belay {
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    kind: BelayType,
    extruder_stepper_name: String,
    enable_initial: bool,
    multiplier_high: Cell<f64>,
    multiplier_low: Cell<f64>,
    debug_level: i64,
    name: String,
    enabled: Cell<bool>,
    last_state: Cell<bool>,
    last_direction: Cell<bool>,
    set_multiplier: RefCell<Option<SetMultiplier>>,
    gcode: Rc<GCodeDispatch>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    update_direction_timer: RefCell<Option<Timer>>,
}

impl Belay {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();

        // initial type-specific setup
        let kind = config.getchoice(
            "extruder_type",
            &[("trad_rack", BelayType::TradRack), ("extruder_stepper", BelayType::ExtruderStepper)],
            Dflt::Required,
        )?;
        let (enable_events, disable_events, enable_initial, extruder_stepper_name): (&[&str], &[&str], bool, String) = match kind {
            BelayType::TradRack => (&["trad_rack:synced_to_extruder"], &["trad_rack:unsyncing_from_extruder"], false, String::new()),
            BelayType::ExtruderStepper => (&[], &[], true, config.get("extruder_stepper_name", Dflt::Required)?),
        };

        // register event handlers
        let slot: Rc<RefCell<Weak<Belay>>> = Rc::new(RefCell::new(Weak::new()));
        type BCall = fn(&Belay) -> KResult<()>;
        let mut handlers: Vec<(&str, BCall)> = vec![("klippy:connect", Belay::handle_connect), ("klippy:ready", Belay::handle_ready)];
        handlers.extend(enable_events.iter().map(|e| (*e, Belay::handle_enable as BCall)));
        handlers.extend(disable_events.iter().map(|e| (*e, Belay::handle_disable as BCall)));
        for (event, f) in handlers {
            let sl = slot.clone();
            printer.register_event_handler(
                event,
                Rc::new(move |_| match sl.borrow().upgrade() {
                    Some(b) => f(&b),
                    None => Ok(()),
                }),
            );
        }

        // register button
        let sensor_pin = config.get("sensor_pin", Dflt::Required)?;
        let buttons = printer.load_as::<PrinterButtons>(config, "buttons")?;
        let sl = slot.clone();
        buttons.register_buttons(
            &[&sensor_pin],
            Rc::new(move |eventtime, state| match sl.borrow().upgrade() {
                Some(b) => b.sensor_callback(eventtime, state != 0),
                None => Ok(()),
            }),
        )?;

        // read other values
        let multiplier_high = config.getfloat("multiplier_high", 1.05, Lim::new().min(1.0))?;
        let multiplier_low = config.getfloat("multiplier_low", 0.95, Lim::new().min(0.0).max(1.0))?;
        let debug_level = config.getint("debug_level", 0, Lim::new().min(0.0).max(2.0))?;

        // other variables
        let name = config.get_name().split_whitespace().nth(1).unwrap_or_default().to_string();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let b = Rc::new(Belay {
            printer: printer.clone(),
            reactor: reactor.clone(),
            kind,
            extruder_stepper_name,
            enable_initial,
            multiplier_high: Cell::new(multiplier_high),
            multiplier_low: Cell::new(multiplier_low),
            debug_level,
            name: name.clone(),
            enabled: Cell::new(false),
            last_state: Cell::new(false),
            last_direction: Cell::new(true),
            set_multiplier: RefCell::new(None),
            gcode: gcode.clone(),
            toolhead: RefCell::new(None),
            update_direction_timer: RefCell::new(None),
        });
        *slot.borrow_mut() = Rc::downgrade(&b);
        let w = Rc::downgrade(&b);
        *b.update_direction_timer.borrow_mut() = Some(reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(b) => b.update_direction(eventtime),
                None => NEVER,
            }),
            NEVER,
        ));

        // register commands
        type BCmd = fn(&Belay, &GCodeCommand) -> KResult<()>;
        let mut cmds: Vec<(&str, BCmd, &str)> = vec![
            ("BELAY_ENABLE", Belay::cmd_belay_enable, CMD_BELAY_ENABLE_HELP),
            ("BELAY_DISABLE", Belay::cmd_belay_disable, CMD_BELAY_DISABLE_HELP),
            ("QUERY_BELAY", Belay::cmd_query_belay, CMD_QUERY_BELAY_HELP),
            ("BELAY_SET_MULTIPLIER", Belay::cmd_belay_set_multiplier, CMD_BELAY_SET_MULTIPLIER_HELP),
        ];
        // register extruder_stepper-only commands
        if kind == BelayType::ExtruderStepper {
            cmds.push(("BELAY_SET_STEPPER", Belay::cmd_belay_set_stepper, CMD_BELAY_SET_STEPPER_HELP));
        }
        for (cmd, f, desc) in cmds {
            let w = Rc::downgrade(&b);
            gcode.register_mux_command(
                cmd,
                "BELAY",
                Some(&name),
                Rc::new(move |g| match w.upgrade() {
                    Some(b) => f(&b, g),
                    None => Ok(()),
                }),
                Some(desc),
            )?;
        }
        Ok(b)
    }

    fn handle_connect(&self) -> KResult<()> {
        *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);

        // finish type-specific setup
        match self.kind {
            BelayType::TradRack => {
                let trad_rack = self.printer.lookup::<TradRack>("trad_rack")?;
                *self.set_multiplier.borrow_mut() = Some(SetMultiplier::TradRack(trad_rack));
                Ok(())
            }
            BelayType::ExtruderStepper => self.set_extruder_stepper(&self.extruder_stepper_name),
        }
    }

    fn set_extruder_stepper(&self, extruder_stepper_name: &str) -> KResult<()> {
        let printer_extruder_stepper = self.printer.lookup::<PrinterExtruderStepper>(&format!("extruder_stepper {}", extruder_stepper_name))?;
        let stepper = printer_extruder_stepper.extruder_stepper.stepper.clone();
        let base_rotation_dist = stepper.get_rotation_distance().0;
        *self.set_multiplier.borrow_mut() = Some(SetMultiplier::Stepper(stepper, base_rotation_dist));
        Ok(())
    }

    fn call_set_multiplier(&self, m: f64) -> KResult<()> {
        match self.set_multiplier.borrow().clone() {
            Some(SetMultiplier::TradRack(tr)) => tr.set_fil_driver_multiplier(m),
            Some(SetMultiplier::Stepper(stepper, base)) => {
                stepper.set_rotation_distance(base / m);
                Ok(())
            }
            None => Err(KError::Other("'NoneType' object is not callable".into())),
        }
    }

    /// the enable/disable conditions beyond the enabled check: trad_rack's
    /// is_fil_driver_synced
    fn type_condition(&self) -> bool {
        match self.set_multiplier.borrow().as_ref() {
            Some(SetMultiplier::TradRack(tr)) => tr.is_fil_driver_synced(),
            _ => true,
        }
    }

    fn handle_ready(&self) -> KResult<()> {
        if self.enable_initial {
            self.handle_enable()?;
        }
        Ok(())
    }

    fn timer(&self) -> Timer {
        self.update_direction_timer.borrow().clone().expect("belay timer")
    }

    fn handle_enable(&self) -> KResult<()> {
        if self.enabled.get() || !self.type_condition() {
            return Ok(());
        }
        self.enabled.set(true);
        self.reactor.update_timer(&self.timer(), NOW);
        self.update_multiplier(true)
    }

    fn handle_disable(&self) -> KResult<()> {
        if !self.enabled.get() || !self.type_condition() {
            return Ok(());
        }
        self.reset_multiplier()?;
        self.reactor.update_timer(&self.timer(), NEVER);
        self.enabled.set(false);
        Ok(())
    }

    fn sensor_callback(&self, _eventtime: f64, state: bool) -> KResult<()> {
        self.last_state.set(state);
        if self.enabled.get() {
            self.update_multiplier(true)?;
        }
        Ok(())
    }

    fn update_multiplier(&self, print_msg: bool) -> KResult<()> {
        let multiplier = if self.last_state.get() == self.last_direction.get() {
            // compressed/forward or expanded/backward
            self.multiplier_high.get()
        } else {
            // compressed/backward or expanded/forward
            self.multiplier_low.get()
        };
        self.call_set_multiplier(multiplier)?;
        if (print_msg && self.debug_level >= 1) || self.debug_level >= 2 {
            self.gcode.respond_info(&format!("Set secondary extruder multiplier: {:.6}", multiplier), true);
        }
        Ok(())
    }

    fn reset_multiplier(&self) -> KResult<()> {
        self.call_set_multiplier(1.0)?;
        if self.debug_level >= 1 {
            self.gcode.respond_info("Reset secondary extruder multiplier", true);
        }
        Ok(())
    }

    fn update_direction(&self, eventtime: f64) -> f64 {
        if let Err(e) = self.try_update_direction(eventtime) {
            crate::logging::exception_err("belay update_direction", &e);
        }
        eventtime + DIRECTION_UPDATE_INTERVAL
    }

    fn try_update_direction(&self, eventtime: f64) -> KResult<()> {
        let mcu = self.printer.lookup::<Mcu>("mcu")?;
        let print_time = mcu.estimated_print_time(eventtime);
        let toolhead = self.toolhead.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_extruder'".into()))?;
        let extruder = toolhead.get_extruder();
        let curr_pos = extruder.find_past_position(print_time);
        let past_pos = extruder.find_past_position(py_max(0.0, print_time - POSITION_TIME_DIFF));
        let prev_direction = self.last_direction.get();
        self.last_direction.set(curr_pos >= past_pos);
        if self.last_direction.get() != prev_direction {
            if self.debug_level >= 2 {
                self.gcode.respond_info(&format!("New Belay sensor direction: {}", py_bool(self.last_direction.get())), true);
            }
            self.update_multiplier(false)?;
        }
        Ok(())
    }

    fn cmd_query_belay(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        let state_info = if self.last_state.get() { "compressed" } else { "expanded" };
        self.gcode.respond_info(&format!("belay {}: {}", self.name, state_info), true);
        Ok(())
    }

    fn cmd_belay_set_multiplier(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.multiplier_high.set(gcmd.get_float("HIGH", Some(self.multiplier_high.get()), Lim::new().min(1.0))?);
        self.multiplier_low.set(gcmd.get_float("LOW", Some(self.multiplier_low.get()), Lim::new().min(0.0).max(1.0))?);
        Ok(())
    }

    fn cmd_belay_set_stepper(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.handle_disable()?;
        self.set_extruder_stepper(&gcmd.get("STEPPER")?)?;
        self.handle_enable()
    }

    fn cmd_belay_enable(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.handle_enable()?;
        gcmd.respond_info("Belay manually enabled", true);
        Ok(())
    }

    fn cmd_belay_disable(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.handle_disable()?;
        gcmd.respond_info("Belay manually disabled", true);
        Ok(())
    }
}

impl PrinterObject for Belay {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("last_state".to_string(), PyVal::Bool(self.last_state.get()));
        d.insert("enabled".to_string(), PyVal::Bool(self.enabled.get()));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        *self.set_multiplier.borrow_mut() = None;
        *self.toolhead.borrow_mut() = None;
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(Belay::new(config)?)
}
