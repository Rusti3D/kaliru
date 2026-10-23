// Support setting gpio pins at mcu start
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::gpio_out_setup;
use crate::config::CONFIG_INITIAL_PINS;
use crate::ctr::{INITIAL_PINS, IP_OUT_HIGH};
use crate::sched::R;
use crate::{decl_init, decl_initial_pins};

decl_initial_pins!(CONFIG_INITIAL_PINS);

pub fn initial_pins_setup() -> R {
    if CONFIG_INITIAL_PINS.is_empty() {
        return Ok(());
    }
    for ip in INITIAL_PINS.iter() {
        gpio_out_setup(ip.pin as u32, (ip.flags & IP_OUT_HIGH) as crate::board::GpioOutVal)?;
    }
    Ok(())
}
decl_init!(initial_pins_setup);
