// Define GPIO as clock output (port of klippy/extras/static_pwm_clock.py)
//
// Copyright (C) 2025  Timofey Titovets <nefelim4ag@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::pin_mcu;
use crate::logging;
use crate::mcu::Mcu;
use crate::pins::PinsObject;
use crate::printer::{PObj, PrinterObject};
use std::rc::Rc;

pub struct PrinterClockOutputPin {
    pub name: String,
    pub mcu: Rc<Mcu>,
    pub pin: String,
    pub invert: bool,
    pub frequency: f64,
}

impl PrinterClockOutputPin {
    pub fn build_config(&self) -> KResult<()> {
        self.mcu.lookup_command("stm32_timer_output pin=%u cycle_ticks=%u on_ticks=%hu", None)?;
        let mcu_freq = self.mcu.seconds_to_clock(1.0);
        let cycle_ticks = crate::jinja::value::float_divmod(mcu_freq as f64, self.frequency).0 as i64;
        // validate frequency
        let mcu_freq_rev = (cycle_ticks as f64 * self.frequency) as i64;
        if mcu_freq_rev != mcu_freq {
            let msg = format!(
                "\n            # Frequency output must be without remainder, {} != {}\n            [{}]\n            frequency = {:.6}\n            ",
                mcu_freq, mcu_freq_rev, self.name, self.frequency
            );
            return Err(KError::config(msg));
        }
        let mut value = (0.5 * cycle_ticks as f64) as i64;
        if self.invert {
            value = cycle_ticks - value;
        }
        if (value as f64) / (cycle_ticks as f64) < 0.4 {
            logging::warning(&format!("[{}] pulse width < 40%", self.name));
        }
        if (value as f64) / (cycle_ticks as f64) > 0.6 {
            logging::warning(&format!("[{}] pulse width > 60%", self.name));
        }
        self.mcu.add_config_cmd(&format!("stm32_timer_output pin={} cycle_ticks={} on_ticks={}", self.pin, cycle_ticks, value), false, false);
        Ok(())
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let desc = config.get("pin", Dflt::Required)?;
    let mcu_pin = ppins.0.borrow_mut().lookup_pin(&desc, true, false, None).map_err(|e| KError::Pins(e.0))?;
    let mcu = pin_mcu(config, &mcu_pin)?;
    let (pin, invert) = (mcu_pin.borrow().pin.clone(), mcu_pin.borrow().invert != 0);
    let frequency = config.getfloat("frequency", 100.0, Lim::new().above(1.0 / 0.3).max(520000000.0))?;
    let p = Rc::new(PrinterClockOutputPin { name: config.get_name().to_string(), mcu: mcu.clone(), pin, invert, frequency });
    let pp = p.clone();
    mcu.register_config_callback(Rc::new(move || pp.build_config()));
    Ok(p)
}

impl PrinterObject for PrinterClockOutputPin {
}
