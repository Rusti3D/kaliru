// Virtual pin that propagates its changes to multiple output pins
// (port of klippy/extras/multi_pin.py)
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::KResult;
use crate::mcu::{DigitalOutPin, Mcu, PwmPin};
use crate::pins::{PinChip, PinsObject, SharedPinParams};
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use std::any::Any;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

/// the pins a multi_pin drives
enum McuPins {
    None,
    Pwm(Vec<Rc<dyn PwmPin>>),
    Digital(Vec<Rc<dyn DigitalOutPin>>),
}

pub struct PrinterMultiPin {
    me: Weak<PrinterMultiPin>,
    printer: Rc<Printer>,
    pin_type: RefCell<Option<String>>,
    pin_list: Vec<String>,
    mcu_pins: RefCell<McuPins>,
}

/// the "multi_pin" chip: setup_pin() goes to the "multi_pin <name>" object
/// (python registers the first PrinterMultiPin as the chip)
struct MultiPinChip {
    printer: Weak<Printer>,
}

impl PinChip for MultiPinChip {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        let printer = self.printer.upgrade().ok_or("printer is gone")?;
        let pin_name = pin_params.borrow().pin.clone();
        let pin = printer.lookup_object_opt(&format!("multi_pin {}", pin_name)).and_then(downcast_rc::<PrinterMultiPin>);
        let Some(pin) = pin else {
            return Err(format!("multi_pin {} not configured", pin_name));
        };
        pin.setup_pin(pin_type, pin_params)
    }
}

impl PrinterMultiPin {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let ppins = printer.lookup::<PinsObject>("pins")?;
        // (a second multi_pin section finds the chip registered)
        let _ = ppins.0.borrow_mut().register_chip("multi_pin", Rc::new(MultiPinChip { printer: Rc::downgrade(&printer) }));
        let pin_list = config.getlist("pins", Dflt::Required, ",", None)?.unwrap_or_default();
        Ok(Rc::new_cyclic(|me| PrinterMultiPin {
            me: me.clone(),
            printer: printer.clone(),
            pin_type: RefCell::new(None),
            pin_list,
            mcu_pins: RefCell::new(McuPins::None),
        }))
    }

    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        let pin_name = pin_params.borrow().pin.clone();
        if self.pin_type.borrow().is_some() {
            return Err(format!("Can't setup multi_pin {} twice", pin_name));
        }
        *self.pin_type.borrow_mut() = Some(pin_type.to_string());
        let invert = if pin_params.borrow().invert != 0 { "!" } else { "" };
        let descs: Vec<String> = self.pin_list.iter().map(|d| format!("{}{}", invert, d)).collect();
        let me = self.me.upgrade().ok_or("multi_pin is gone")?;
        match pin_type {
            "pwm" => {
                let mut pins = vec![];
                for d in &descs {
                    pins.push(crate::pins::setup_pwm_pin(&self.printer, d).map_err(|e| e.to_string())?);
                }
                *self.mcu_pins.borrow_mut() = McuPins::Pwm(pins);
                let p: Rc<dyn PwmPin> = me;
                Ok(Rc::new(p))
            }
            "digital_out" => {
                let mut pins = vec![];
                for d in &descs {
                    pins.push(crate::pins::setup_digital_out_pin(&self.printer, d).map_err(|e| e.to_string())?);
                }
                *self.mcu_pins.borrow_mut() = McuPins::Digital(pins);
                let p: Rc<dyn DigitalOutPin> = me;
                Ok(Rc::new(p))
            }
            // ponytail: python hands back the multi_pin for any pin type
            // (only output pins work with it)
            other => Err(format!("pin '{}' of type {} is not supported here by Kaliru yet", pin_name, other)),
        }
    }

    fn first_mcu(&self) -> Rc<Mcu> {
        match &*self.mcu_pins.borrow() {
            McuPins::Pwm(p) => p[0].get_mcu(),
            McuPins::Digital(p) => p[0].get_mcu(),
            McuPins::None => panic!("multi_pin has no pins"),
        }
    }
}

impl PwmPin for PrinterMultiPin {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.first_mcu()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        if let McuPins::Pwm(pins) = &*self.mcu_pins.borrow() {
            for p in pins {
                p.setup_max_duration(max_duration);
            }
        }
    }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
        if let McuPins::Pwm(pins) = &*self.mcu_pins.borrow() {
            for p in pins {
                p.setup_cycle_time(cycle_time, hardware_pwm);
            }
        }
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        if let McuPins::Pwm(pins) = &*self.mcu_pins.borrow() {
            for p in pins {
                p.setup_start_value(start_value, shutdown_value);
            }
        }
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        if let McuPins::Pwm(pins) = &*self.mcu_pins.borrow() {
            for p in pins {
                p.set_pwm(print_time, value)?;
            }
        }
        Ok(())
    }
}

impl DigitalOutPin for PrinterMultiPin {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.first_mcu()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        if let McuPins::Digital(pins) = &*self.mcu_pins.borrow() {
            for p in pins {
                p.setup_max_duration(max_duration);
            }
        }
    }
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        if let McuPins::Digital(pins) = &*self.mcu_pins.borrow() {
            for p in pins {
                p.setup_start_value(start_value, shutdown_value);
            }
        }
    }
    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        if let McuPins::Digital(pins) = &*self.mcu_pins.borrow() {
            for p in pins {
                p.set_digital(print_time, value)?;
            }
        }
        Ok(())
    }
}

impl PrinterObject for PrinterMultiPin {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterMultiPin::new(config)?)
}
