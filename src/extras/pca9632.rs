// Support for the PCA9632 LED driver ic (port of klippy/extras/pca9632.py)
//
// Copyright (C) 2022  Ricardo Alcantara <ricardo@vulcanolabs.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::extras::led::{Color, LEDHelper};
use crate::printer::{PObj, PrinterObject};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

const BACKGROUND_PRIORITY_CLOCK: u64 = 0x7FFFFFFF00000000;

// Register addresses
const PCA9632_MODE1: u8 = 0x00;
const PCA9632_MODE2: u8 = 0x01;
const PCA9632_PWM0: u8 = 0x02;
const PCA9632_PWM1: u8 = 0x03;
const PCA9632_PWM2: u8 = 0x04;
const PCA9632_PWM3: u8 = 0x05;
const PCA9632_LEDOUT: u8 = 0x08;

const LED_PWM: u8 = 0x02;
const PCA9632_LED0: u8 = 0x00;
const PCA9632_LED1: u8 = 0x02;
const PCA9632_LED2: u8 = 0x04;
const PCA9632_LED3: u8 = 0x06;

pub struct PCA9632 {
    i2c: Rc<McuI2c>,
    color_map: Vec<usize>,
    prev_regs: RefCell<IndexMap<u8, u8>>,
    led_helper: RefCell<Option<Rc<LEDHelper>>>,
}

impl PCA9632 {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let i2c = mcu_i2c_from_config(config, Some(98), 100000)?;
        let color_order = config.get("color_order", "RGBW")?;
        let mut sorted: Vec<char> = color_order.chars().collect();
        sorted.sort();
        if sorted != ['B', 'G', 'R', 'W'] {
            return Err(KError::config(format!("Invalid color_order '{}'", color_order)));
        }
        let color_map = color_order.chars().map(|c| "RGBW".find(c).unwrap_or(0)).collect();
        let p = Rc::new(PCA9632 { i2c, color_map, prev_regs: RefCell::new(IndexMap::new()), led_helper: RefCell::new(None) });
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
        printer.register_event_handler("klippy:connect", Rc::new(move |_| pp.handle_connect()));
        Ok(p)
    }

    fn reg_write(&self, reg: u8, val: u8, minclock: u64) -> KResult<()> {
        if self.prev_regs.borrow().get(&reg) == Some(&val) {
            return Ok(());
        }
        self.prev_regs.borrow_mut().insert(reg, val);
        self.i2c.i2c_write(&[reg, val], minclock, BACKGROUND_PRIORITY_CLOCK)
    }

    fn handle_connect(&self) -> KResult<()> {
        // Configure MODE1
        self.reg_write(PCA9632_MODE1, 0x00, 0)?;
        // Configure MODE2 (DIMMING, INVERT, CHANGE ON STOP,TOTEM)
        self.reg_write(PCA9632_MODE2, 0x15, 0)?;
        let state = self.led_helper.borrow().as_ref().map(|h| h.led_state()).unwrap_or_default();
        self.update_leds(&state, None)
    }

    fn update_leds(&self, led_state: &[Color], print_time: Option<f64>) -> KResult<()> {
        let mut minclock = 0;
        if let Some(pt) = print_time {
            minclock = self.i2c.get_mcu().print_time_to_clock(pt) as u64;
        }
        let color: Vec<u8> = led_state[0].iter().map(|v| (v * 255.0 + 0.5).trunc() as i64 as u8).collect();
        let leds: Vec<u8> = self.color_map.iter().map(|idx| color[*idx]).collect();
        let (led0, led1, led2, led3) = (leds[0], leds[1], leds[2], leds[3]);
        self.reg_write(PCA9632_PWM0, led0, minclock)?;
        self.reg_write(PCA9632_PWM1, led1, minclock)?;
        self.reg_write(PCA9632_PWM2, led2, minclock)?;
        self.reg_write(PCA9632_PWM3, led3, minclock)?;

        let on = |v: u8, shift: u8| if v != 0 { LED_PWM << shift } else { 0 };
        let ledout = on(led0, PCA9632_LED0) | on(led1, PCA9632_LED1) | on(led2, PCA9632_LED2) | on(led3, PCA9632_LED3);
        self.reg_write(PCA9632_LEDOUT, ledout, minclock)
    }
}

impl PrinterObject for PCA9632 {
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
    Ok(PCA9632::new(config)?)
}
