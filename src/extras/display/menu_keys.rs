// Support for menu button press tracking
// (port of klippy/extras/display/menu_keys.py)
//
// Copyright (C) 2018  Janar Sööt <janar.soot@gmail.com>
// Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::buttons::PrinterButtons;
use crate::printer::Printer;
use crate::reactor::{Reactor, Timer, NEVER};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const LONG_PRESS_DURATION: f64 = 0.800;

/// callback(key, eventtime)
pub type KeyCallback = Rc<dyn Fn(&str, f64) -> KResult<()>>;

pub struct MenuKeys {
    printer: Rc<Printer>,
    reactor: Rc<Reactor>,
    callback: KeyCallback,
    encoder_fast_rate: f64,
    last_encoder_cw_eventtime: Cell<f64>,
    last_encoder_ccw_eventtime: Cell<f64>,
    is_short_click: Cell<bool>,
    click_timer: RefCell<Option<Timer>>,
}

impl MenuKeys {
    pub fn new(config: &ConfigWrapper, callback: KeyCallback) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let buttons = printer.load_as::<PrinterButtons>(config, "buttons")?;
        // Register rotary encoder
        let encoder_pins = config.get_opt("encoder_pins")?;
        let encoder_steps_per_detent = config.getchoice_int("encoder_steps_per_detent", &[(2, 2), (4, 4)], 4)?;
        let slot: Rc<RefCell<std::rc::Weak<MenuKeys>>> = Rc::new(RefCell::new(std::rc::Weak::new()));
        if let Some(encoder_pins) = encoder_pins {
            let parts: Vec<&str> = encoder_pins.split(',').collect();
            let [pin1, pin2] = parts[..] else {
                let cause = crate::pyfmt::py_unpack_error(2, parts.len()).unwrap_or_default();
                return Err(KError::config("Unable to parse encoder_pins").with_context(KError::Py("ValueError".into(), cause)));
            };
            let (s1, s2) = (slot.clone(), slot.clone());
            buttons.register_rotary_encoder(
                pin1.trim(),
                pin2.trim(),
                Rc::new(move |eventtime| match s1.borrow().upgrade() {
                    Some(k) => k.encoder_cw_callback(eventtime),
                    None => Ok(()),
                }),
                Rc::new(move |eventtime| match s2.borrow().upgrade() {
                    Some(k) => k.encoder_ccw_callback(eventtime),
                    None => Ok(()),
                }),
                encoder_steps_per_detent,
            )?;
        }
        let encoder_fast_rate = config.getfloat("encoder_fast_rate", 0.030, Lim::new().above(0.0))?;
        let k = Rc::new(MenuKeys {
            printer: printer.clone(),
            reactor: reactor.clone(),
            callback,
            encoder_fast_rate,
            last_encoder_cw_eventtime: Cell::new(0.0),
            last_encoder_ccw_eventtime: Cell::new(0.0),
            // Register click button
            is_short_click: Cell::new(false),
            click_timer: RefCell::new(None),
        });
        *slot.borrow_mut() = Rc::downgrade(&k);
        let w = Rc::downgrade(&k);
        *k.click_timer.borrow_mut() = Some(reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(k) => k.long_click_event(eventtime),
                None => NEVER,
            }),
            NEVER,
        ));
        k.register_button(config, "click_pin", Button::Click, false)?;
        // Register other buttons
        k.register_button(config, "back_pin", Button::Back, true)?;
        k.register_button(config, "up_pin", Button::Up, true)?;
        k.register_button(config, "down_pin", Button::Down, true)?;
        k.register_button(config, "kill_pin", Button::Kill, true)?;
        Ok(k)
    }

    fn register_button(self: &Rc<Self>, config: &ConfigWrapper, name: &str, button: Button, push_only: bool) -> KResult<()> {
        let Some(pin) = config.get_opt(name)? else { return Ok(()) };
        let buttons = self.printer.lookup::<PrinterButtons>("buttons")?;
        let w = Rc::downgrade(self);
        let push: Rc<dyn Fn(f64) -> KResult<()>> = Rc::new(move |eventtime| match w.upgrade() {
            Some(k) => k.button_callback(button, eventtime),
            None => Ok(()),
        });
        let w = Rc::downgrade(self);
        let state_cb: crate::extras::buttons::ButtonCallback = Rc::new(move |eventtime, state| match w.upgrade() {
            Some(k) => k.click_callback(eventtime, state != 0),
            None => Ok(()),
        });
        let range_option = format!("analog_range_{}", name);
        if config.get_opt(&range_option)?.is_none() {
            if push_only {
                buttons.register_button_push(&pin, push)?;
            } else {
                buttons.register_buttons(&[&pin], state_cb)?;
            }
            return Ok(());
        }
        let range = config.getfloatlist(&range_option, Dflt::Required, ",", Some(2))?.unwrap_or_default();
        let (amin, amax) = (range[0], range[1]);
        let pullup = config.getfloat("analog_pullup_resistor", 4700.0, Lim::new().above(0.0))?;
        if push_only {
            buttons.register_adc_button_push(&pin, amin, amax, pullup, push)
        } else {
            buttons.register_adc_button(&pin, amin, amax, pullup, state_cb)
        }
    }

    // Rotary encoder callbacks
    fn encoder_cw_callback(&self, eventtime: f64) -> KResult<()> {
        let fast_rate = (eventtime - self.last_encoder_cw_eventtime.get()) <= self.encoder_fast_rate;
        self.last_encoder_cw_eventtime.set(eventtime);
        (self.callback)(if fast_rate { "fast_up" } else { "up" }, eventtime)
    }

    fn encoder_ccw_callback(&self, eventtime: f64) -> KResult<()> {
        let fast_rate = (eventtime - self.last_encoder_ccw_eventtime.get()) <= self.encoder_fast_rate;
        self.last_encoder_ccw_eventtime.set(eventtime);
        (self.callback)(if fast_rate { "fast_down" } else { "down" }, eventtime)
    }

    // Click handling
    fn long_click_event(&self, eventtime: f64) -> f64 {
        self.is_short_click.set(false);
        if let Err(e) = (self.callback)("long_click", eventtime) {
            crate::logging::exception_err("menu long_click", &e);
        }
        NEVER
    }

    fn click_callback(&self, eventtime: f64, state: bool) -> KResult<()> {
        let timer = self.click_timer.borrow().clone();
        if state {
            self.is_short_click.set(true);
            if let Some(t) = &timer {
                self.reactor.update_timer(t, eventtime + LONG_PRESS_DURATION);
            }
        } else if self.is_short_click.get() {
            if let Some(t) = &timer {
                self.reactor.update_timer(t, NEVER);
            }
            (self.callback)("click", eventtime)?;
        }
        Ok(())
    }

    // Other button callbacks
    fn button_callback(&self, button: Button, eventtime: f64) -> KResult<()> {
        match button {
            Button::Back => (self.callback)("back", eventtime),
            Button::Up => (self.callback)("up", eventtime),
            Button::Down => (self.callback)("down", eventtime),
            Button::Kill => {
                self.printer.invoke_shutdown("Shutdown due to kill button!", None);
                Ok(())
            }
            Button::Click => Ok(()),
        }
    }
}

#[derive(Clone, Copy)]
enum Button {
    Click,
    Back,
    Up,
    Down,
    Kill,
}
