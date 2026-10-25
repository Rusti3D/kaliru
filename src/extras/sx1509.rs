// SX1509 Extension (port of klippy/extras/sx1509.py)
//
// Copyright (C) 2018  Florian Heilmann <Florian.Heilmann@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::mcu::{DigitalOutPin, Mcu, PwmPin};
use crate::pins::{PinChip, PinsObject, SharedPinParams};
use crate::printer::{PObj, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

// Word registers
const REG_RESET: i64 = 0x7D;
const REG_CLOCK: i64 = 0x1E;
const REG_MISC: i64 = 0x1F;
const REG_DIR: i64 = 0x0E;
const REG_DATA: i64 = 0x10;
const REG_PULLUP: i64 = 0x06;
const REG_PULLDOWN: i64 = 0x08;
const REG_INPUT_DISABLE: i64 = 0x00;
const REG_ANALOG_DRIVER_ENABLE: i64 = 0x20;

// Byte registers
const REG_I_ON: [i64; 16] = [0x2A, 0x2D, 0x30, 0x33, 0x36, 0x3B, 0x40, 0x45, 0x4A, 0x4D, 0x50, 0x53, 0x56, 0x5B, 0x5F, 0x65];

pub struct SX1509 {
    me: Weak<SX1509>,
    i2c: Rc<McuI2c>,
    mcu: Rc<Mcu>,
    last_clock: Cell<i64>,
    reg_dict: RefCell<IndexMap<i64, i64>>,
    reg_i_on_dict: RefCell<IndexMap<i64, i64>>,
}

impl SX1509 {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        let i2c = mcu_i2c_from_config(config, None, 400000)?;
        let mcu = i2c.get_mcu();
        // Set up registers default values
        let reg_dict: IndexMap<i64, i64> = [
            (REG_DIR, 0xFFFF),
            (REG_DATA, 0),
            (REG_PULLUP, 0),
            (REG_PULLDOWN, 0),
            (REG_INPUT_DISABLE, 0),
            (REG_ANALOG_DRIVER_ENABLE, 0),
        ]
        .into_iter()
        .collect();
        let s = Rc::new_cyclic(|me| SX1509 {
            me: me.clone(),
            i2c,
            mcu,
            last_clock: Cell::new(0),
            reg_dict: RefCell::new(reg_dict),
            reg_i_on_dict: RefCell::new(REG_I_ON.iter().map(|r| (*r, 0)).collect()),
        });
        let ppins = printer.lookup::<PinsObject>("pins")?;
        ppins.0.borrow_mut().register_chip(&format!("sx1509_{}", name), s.clone()).map_err(|e| KError::Pins(e.0))?;
        let ss = s.clone();
        crate::events::KlippyConnect::on(&printer, move || ss.handle_connect());
        Ok(s)
    }

    fn handle_connect(&self) -> KResult<()> {
        // Reset the chip, Default RegClock/RegMisc 0x0
        self.i2c.i2c_write(&[REG_RESET as u8, 0x12], 0, 0)?;
        self.i2c.i2c_write(&[REG_RESET as u8, 0x34], 0, 0)?;
        // Enable Oscillator
        self.i2c.i2c_write(&[REG_CLOCK as u8, 1 << 6], 0, 0)?;
        // Setup Clock Divider
        self.i2c.i2c_write(&[REG_MISC as u8, 1 << 4], 0, 0)?;
        // Transfer all regs with their initial cached state
        let reactor = self.mcu.get_printer().get_reactor();
        let regs: Vec<i64> = self.reg_dict.borrow().keys().chain(self.reg_i_on_dict.borrow().keys()).copied().collect();
        for reg in regs {
            let printtime = self.mcu.estimated_print_time(reactor.monotonic());
            self.send_register(reg, printtime)?;
        }
        Ok(())
    }

    fn update(&self, reg: i64, f: impl Fn(i64) -> i64) {
        if let Some(v) = self.reg_dict.borrow_mut().get_mut(&reg) {
            *v = f(*v);
            return;
        }
        if let Some(v) = self.reg_i_on_dict.borrow_mut().get_mut(&reg) {
            *v = f(*v);
        }
    }

    fn clear_bits_in_register(&self, reg: i64, bitmask: i64) {
        self.update(reg, |v| v & !bitmask);
    }

    fn set_bits_in_register(&self, reg: i64, bitmask: i64) {
        self.update(reg, |v| v | bitmask);
    }

    fn set_register(&self, reg: i64, value: i64) {
        self.update(reg, |_| value);
    }

    fn send_register(&self, reg: i64, print_time: f64) -> KResult<()> {
        let mut data = vec![(reg & 0xFF) as u8];
        if let Some(v) = self.reg_dict.borrow().get(&reg) {
            // Word
            data.extend([((v >> 8) & 0xFF) as u8, (v & 0xFF) as u8]);
        } else if let Some(v) = self.reg_i_on_dict.borrow().get(&reg) {
            // Byte
            data.push((v & 0xFF) as u8);
        }
        let clock = self.mcu.print_time_to_clock(print_time);
        self.i2c.i2c_write_noack(&data, self.last_clock.get() as u64, clock as u64)?;
        self.last_clock.set(clock);
        Ok(())
    }
}

/// int(pin.split("_")[1])
fn sx_pin_number(pin: &str) -> Result<usize, String> {
    let part = pin.split('_').nth(1).ok_or_else(|| "list index out of range".to_string())?;
    part.trim().parse::<usize>().map_err(|_| format!("invalid literal for int() with base 10: '{}'", part))
}

impl PinChip for SX1509 {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        let me = self.me.upgrade().expect("sx1509");
        let (pin, invert) = {
            let p = pin_params.borrow();
            (p.pin.clone(), p.invert != 0)
        };
        let prefix: String = pin.chars().take(4).collect();
        if pin_type == "digital_out" && prefix == "PIN_" {
            let p: Rc<dyn DigitalOutPin> = SX1509DigitalOut::new(&me, &pin, invert)?;
            return Ok(Rc::new(p));
        } else if pin_type == "pwm" && prefix == "PIN_" {
            let p: Rc<dyn PwmPin> = SX1509Pwm::new(&me, &pin, invert)?;
            return Ok(Rc::new(p));
        }
        Err(format!("Wrong pin or incompatible type: {} with type {}! ", prefix, pin_type))
    }
}

impl PrinterObject for SX1509 {
}

pub struct SX1509DigitalOut {
    sx1509: Rc<SX1509>,
    bitmask: i64,
    invert: bool,
    start_value: Cell<bool>,
    shutdown_value: Cell<bool>,
    max_duration: Cell<f64>,
}

impl SX1509DigitalOut {
    fn new(sx1509: &Rc<SX1509>, pin: &str, invert: bool) -> Result<Rc<Self>, String> {
        let sxpin = sx_pin_number(pin)?;
        let d = Rc::new(SX1509DigitalOut {
            sx1509: sx1509.clone(),
            bitmask: 1 << sxpin,
            invert,
            start_value: Cell::new(invert),
            shutdown_value: Cell::new(invert),
            max_duration: Cell::new(2.0),
        });
        let dd = d.clone();
        sx1509.mcu.register_config_callback(Rc::new(move || {
            if dd.max_duration.get() != 0.0 {
                return Err(KError::Pins("SX1509 pins are not suitable for heaters".into()));
            }
            Ok(())
        }));
        // Set direction to output
        sx1509.clear_bits_in_register(REG_DIR, d.bitmask);
        Ok(d)
    }
}

impl DigitalOutPin for SX1509DigitalOut {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.sx1509.mcu.clone()
    }

    fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }

    fn setup_start_value(&self, start_value: bool, _shutdown_value: bool) {
        self.start_value.set(start_value ^ self.invert);
        self.shutdown_value.set(self.invert);
        // We need to set the start value here so the register is
        // updated before the SX1509 class writes it.
        if self.start_value.get() {
            self.sx1509.set_bits_in_register(REG_DATA, self.bitmask);
        } else {
            self.sx1509.clear_bits_in_register(REG_DATA, self.bitmask);
        }
    }

    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        if value ^ self.invert {
            self.sx1509.set_bits_in_register(REG_DATA, self.bitmask);
        } else {
            self.sx1509.clear_bits_in_register(REG_DATA, self.bitmask);
        }
        self.sx1509.send_register(REG_DATA, print_time)
    }
}

pub struct SX1509Pwm {
    sx1509: Rc<SX1509>,
    i_on_reg: i64,
    invert: bool,
    start_value: Cell<f64>,
    shutdown_value: Cell<f64>,
    max_duration: Cell<f64>,
    hardware_pwm: Cell<bool>,
    cycle_time: Cell<f64>,
}

impl SX1509Pwm {
    fn new(sx1509: &Rc<SX1509>, pin: &str, invert: bool) -> Result<Rc<Self>, String> {
        let sxpin = sx_pin_number(pin)?;
        let bitmask = 1i64 << sxpin;
        let i_on_reg = *REG_I_ON.get(sxpin).ok_or_else(|| "list index out of range".to_string())?;
        let p = Rc::new(SX1509Pwm {
            sx1509: sx1509.clone(),
            i_on_reg,
            invert,
            start_value: Cell::new(invert as i64 as f64),
            shutdown_value: Cell::new(invert as i64 as f64),
            max_duration: Cell::new(2.0),
            hardware_pwm: Cell::new(false),
            cycle_time: Cell::new(0.0),
        });
        let pp = p.clone();
        sx1509.mcu.register_config_callback(Rc::new(move || {
            if !pp.hardware_pwm.get() {
                return Err(KError::Pins("SX1509_pwm must have hardware_pwm enabled".into()));
            }
            if pp.max_duration.get() != 0.0 {
                return Err(KError::Pins("SX1509 pins are not suitable for heaters".into()));
            }
            Ok(())
        }));
        // Set required registers
        sx1509.set_bits_in_register(REG_INPUT_DISABLE, bitmask);
        sx1509.clear_bits_in_register(REG_PULLUP, bitmask);
        sx1509.clear_bits_in_register(REG_DIR, bitmask);
        sx1509.set_bits_in_register(REG_ANALOG_DRIVER_ENABLE, bitmask);
        sx1509.clear_bits_in_register(REG_DATA, bitmask);
        Ok(p)
    }
}

impl PwmPin for SX1509Pwm {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.sx1509.mcu.clone()
    }

    fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }

    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) -> KResult<()> {
        self.cycle_time.set(cycle_time);
        self.hardware_pwm.set(hardware_pwm);
        Ok(())
    }

    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        let (mut start_value, mut shutdown_value) = (start_value, shutdown_value);
        if self.invert {
            start_value = 1.0 - start_value;
            shutdown_value = 1.0 - shutdown_value;
        }
        self.start_value.set(py_max(0.0, py_min(1.0, start_value)));
        self.shutdown_value.set(py_max(0.0, py_min(1.0, shutdown_value)));
    }

    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        let v = (255.0 * value).trunc() as i64;
        self.sx1509.set_register(self.i_on_reg, if !self.invert { !v } else { v & 0xFF });
        self.sx1509.send_register(self.i_on_reg, print_time)
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(SX1509::new(config)?)
}
