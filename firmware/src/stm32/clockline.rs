// Code to enable clock lines on stm32
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::lookup_clock_line;
use crate::board::{irq_restore, irq_save};

// Enable a peripheral clock
pub fn enable_pclock(periph_base: u32) {
    let cl = lookup_clock_line(periph_base);
    let flag = irq_save();
    let en = unsafe { &*cl.en };
    en.or(cl.bit);
    en.read(); // Pause 2 cycles to ensure peripheral is enabled
    if !cl.rst.is_null() {
        // Reset peripheral
        let rst = unsafe { &*cl.rst };
        rst.write(cl.bit);
        rst.write(0);
    }
    irq_restore(flag);
}

// Check if a peripheral clock has been enabled
pub fn is_enabled_pclock(periph_base: u32) -> bool {
    let cl = lookup_clock_line(periph_base);
    unsafe { &*cl.en }.read() & cl.bit != 0
}
