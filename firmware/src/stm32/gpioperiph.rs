// Code to setup gpio on stm32 chip (except for stm32f1)
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::config::CONFIG_MACH_STM32F0;

// Set the mode, extended function and speed of a pin
pub fn gpio_peripheral(gpio: u32, mode: u32, pullup: i32) {
    let regs_p = digital_regs()[GPIO2PORT(gpio) as usize];

    // Enable GPIO clock
    gpio_clock_enable(regs_p);
    let regs = unsafe { &*regs_p };

    // Configure GPIO
    let (mode_bits, func) = (mode & 0xf, (mode >> 4) & 0xf);
    let (od, hs) = ((mode >> 8) & 0x1, (mode >> 9) & 0x1);
    let pup: u32 = if pullup != 0 {
        if pullup > 0 {
            1
        } else {
            2
        }
    } else {
        0
    };
    let (pos, af_reg) = (gpio % 16, (gpio % 16) / 8);
    let af_shift = (pos % 8) * 4;
    let af_msk = 0x0f << af_shift;
    let m_shift = pos * 2;
    let m_msk = 0x03 << m_shift;

    let afr = &regs.AFR[af_reg as usize];
    afr.write((afr.read() & !af_msk) | (func << af_shift));
    regs.MODER.write((regs.MODER.read() & !m_msk) | (mode_bits << m_shift));
    regs.PUPDR.write((regs.PUPDR.read() & !m_msk) | (pup << m_shift));
    regs.OTYPER.write((regs.OTYPER.read() & !(1 << pos)) | (od << pos));

    // Setup OSPEEDR:
    // stm32f0 is ~10Mhz at 50pF
    // stm32f2 is ~25Mhz at 40pF
    // stm32f4 is ~50Mhz at 40pF
    // stm32f7 is ~50Mhz at 40pF
    // stm32g0 is ~30Mhz at 50pF
    // stm32h7 is ~85Mhz at 50pF
    let ospeed: u32 = if hs != 0 { 0x03 } else if CONFIG_MACH_STM32F0 { 0x01 } else { 0x02 };
    regs.OSPEEDR.write((regs.OSPEEDR.read() & !m_msk) | (ospeed << m_shift));
}
