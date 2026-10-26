// Support for the PCA9533 LED driver ic (port of klippy/extras/pca9533.py)
//
// Copyright (C) 2021  Marc-Andre Denis <marcadenis@msn.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::extras::led::{Color, LEDHelper};
use crate::printer::{PObj, PrinterObject};
use crate::pyval::PyVal;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

const BACKGROUND_PRIORITY_CLOCK: u64 = 0x7FFFFFFF00000000;

const PCA9533_PWM0: u8 = 0b010;
const PCA9533_PWM1: u8 = 0b100;
const PCA9533_PLS0: u8 = 0b101;

pub struct PCA9533 {
    i2c: Rc<McuI2c>,
    pub(crate) led_helper: RefCell<Option<Rc<LEDHelper>>>,
}

impl PCA9533 {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let i2c = mcu_i2c_from_config(config, Some(98), 100000)?;
        let p = Rc::new(PCA9533 { i2c, led_helper: RefCell::new(None) });
        let w: Weak<Self> = Rc::downgrade(&p);
        let helper = LEDHelper::new(
            config,
            Rc::new(move |state, print_time| match w.upgrade() {
                Some(p) => p.update_leds(state, print_time),
                None => Ok(()),
            }),
            1,
        )?;
        *p.led_helper.borrow_mut() = Some(helper);
        let pp = p.clone();
        crate::events::KlippyConnect::on(&printer, move || pp.handle_connect());
        Ok(p)
    }

    fn handle_connect(&self) -> KResult<()> {
        self.i2c.i2c_write(&[PCA9533_PWM0, 85], 0, 0)?;
        self.i2c.i2c_write(&[PCA9533_PWM1, 170], 0, 0)?;
        let state = self.led_helper.borrow().as_ref().map(|h| h.led_state()).unwrap_or_default();
        self.update_leds(&state, None)
    }

    fn update_leds(&self, led_state: &[Color], print_time: Option<f64>) -> KResult<()> {
        let rmap = [0u8, 2, 3, 1, 1];
        let v: Vec<u8> = led_state[0].iter().map(|v| rmap[(v * 4.0).trunc() as usize]).collect();
        let (red, green, blue, white) = (v[0], v[1], v[2], v[3]);
        let ls0 = (white << 6) | (blue << 4) | (green << 2) | red;
        let mut minclock = 0;
        if let Some(pt) = print_time {
            minclock = self.i2c.get_mcu().print_time_to_clock(pt);
        }
        self.i2c.i2c_write(&[PCA9533_PLS0, ls0], minclock as u64, BACKGROUND_PRIORITY_CLOCK)
    }
}

impl PrinterObject for PCA9533 {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        self.led_helper.borrow().as_ref().map(|h| h.get_status())
    }
    fn teardown(&self) {
        if let Some(h) = self.led_helper.borrow_mut().take() {
            h.teardown();
        }
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PCA9533::new(config)?)
}
