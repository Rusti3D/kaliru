// Support for GPIO input edge counters (port of klippy/extras/pulse_counter.py)
//
// Copyright (C) 2021  Adrian Keet <arkeet@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::error::{KError, KResult};
use crate::mcu::{get_printer_mcu, pint, Mcu};
use crate::msgproto::Params;
use crate::pins::PinsObject;
use crate::printer::Printer;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// callback(time, count, count_time)
pub type CounterCallback = Rc<dyn Fn(f64, i64, f64) -> KResult<()>>;

pub struct McuCounter {
    pub mcu: Rc<Mcu>,
    pub oid: i64,
    pub pin: String,
    pub pullup: i64,
    pub poll_time: f64,
    pub poll_ticks: Cell<i64>,
    pub sample_time: f64,
    pub callback: RefCell<Option<CounterCallback>>,
    pub last_count: Cell<i64>,
}

impl McuCounter {
    pub fn new(printer: &Printer, pin: &str, sample_time: f64, poll_time: f64) -> KResult<Rc<Self>> {
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(pin, false, true, None).map_err(|e| KError::Pins(e.0))?;
        let (chip_name, pin, pullup) = {
            let p = pin_params.borrow();
            (p.chip_name.clone(), p.pin.clone(), p.pullup)
        };
        let mcu = get_printer_mcu(printer, &chip_name)?;
        let c = Rc::new(McuCounter {
            oid: mcu.create_oid(),
            mcu: mcu.clone(),
            pin,
            pullup,
            poll_time,
            poll_ticks: Cell::new(0),
            sample_time,
            callback: RefCell::new(None),
            last_count: Cell::new(0),
        });
        let cc = c.clone();
        mcu.register_config_callback(Rc::new(move || cc.build_config()));
        Ok(c)
    }

    pub fn build_config(self: &Rc<Self>) -> KResult<()> {
        self.mcu.add_config_cmd(&format!("config_counter oid={} pin={} pull_up={}", self.oid, self.pin, self.pullup), false, false);
        let clock = self.mcu.get_query_slot(self.oid);
        self.poll_ticks.set(self.mcu.seconds_to_clock(self.poll_time));
        let sample_ticks = self.mcu.seconds_to_clock(self.sample_time);
        self.mcu.add_config_cmd(
            &format!("query_counter oid={} clock={} poll_ticks={} sample_ticks={}", self.oid, clock, self.poll_ticks.get(), sample_ticks),
            true,
            false,
        );
        let c = self.clone();
        self.mcu.register_response(Some(Rc::new(move |p: &Params| c.handle_counter_state(p))), "counter_state", Some(self.oid));
        Ok(())
    }

    // Callback is called periodically every sample_time
    pub fn setup_callback(&self, cb: CounterCallback) {
        *self.callback.borrow_mut() = Some(cb);
    }

    pub fn handle_counter_state(&self, params: &Params) -> KResult<()> {
        let next_clock = self.mcu.clock32_to_clock64(pint(params, "next_clock"));
        let time = self.mcu.clock_to_print_time((next_clock - self.poll_ticks.get()) as f64);

        let count_clock = self.mcu.clock32_to_clock64(pint(params, "count_clock"));
        let count_time = self.mcu.clock_to_print_time(count_clock as f64);

        // handle 32-bit counter overflow
        let last_count = self.last_count.get();
        let delta_count = (pint(params, "count") - last_count) & 0xFFFFFFFF;
        let count = last_count + delta_count;
        self.last_count.set(count);

        let cb = self.callback.borrow().clone();
        if let Some(cb) = cb {
            cb(time, count, count_time)?;
        }
        Ok(())
    }
}

/// callback(time, frequency)
pub type FrequencyCallback = Rc<dyn Fn(f64, f64) -> KResult<()>>;

pub struct FrequencyCounter {
    pub callback: RefCell<Option<FrequencyCallback>>,
    pub last_time: Cell<Option<f64>>,
    pub last_count: Cell<Option<i64>>,
    pub freq: Cell<f64>,
    pub _counter: Rc<McuCounter>,
}

impl FrequencyCounter {
    pub fn new(printer: &Printer, pin: &str, sample_time: f64, poll_time: f64) -> KResult<Rc<Self>> {
        let counter = McuCounter::new(printer, pin, sample_time, poll_time)?;
        let f = Rc::new(FrequencyCounter {
            callback: RefCell::new(None),
            last_time: Cell::new(None),
            last_count: Cell::new(None),
            freq: Cell::new(0.0),
            _counter: counter.clone(),
        });
        let w = Rc::downgrade(&f);
        counter.setup_callback(Rc::new(move |time, count, count_time| match w.upgrade() {
            Some(f) => f.counter_callback(time, count, count_time),
            None => Ok(()),
        }));
        Ok(f)
    }

    pub fn counter_callback(&self, time: f64, count: i64, count_time: f64) -> KResult<()> {
        match self.last_time.get() {
            None => self.last_time.set(Some(time)), // First sample
            Some(last_time) => {
                let delta_time = count_time - last_time;
                if delta_time > 0.0 {
                    self.last_time.set(Some(count_time));
                    let delta_count = count - self.last_count.get().unwrap_or(0);
                    self.freq.set(delta_count as f64 / delta_time);
                } else {
                    // No counts since last sample
                    self.last_time.set(Some(time));
                    self.freq.set(0.0);
                }
                let cb = self.callback.borrow().clone();
                if let Some(cb) = cb {
                    cb(time, self.freq.get())?;
                }
            }
        }
        self.last_count.set(Some(count));
        Ok(())
    }

    pub fn get_frequency(&self) -> f64 {
        self.freq.get()
    }
}
