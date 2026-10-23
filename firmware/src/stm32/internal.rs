// Local definitions for STM32 code (internal.h)
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
pub use super::chip::*;
pub use super::reg::Reg;

// gpio.c
pub use super::gpio::digital_regs;
pub const fn GPIO(port: char, num: u32) -> u32 {
    (port as u32 - 'A' as u32) * 16 + num
}
pub const fn GPIO2PORT(pin: u32) -> u32 {
    pin / 16
}
pub const fn GPIO2BIT(pin: u32) -> u32 {
    1 << (pin % 16)
}

// gpioperiph.c
pub const GPIO_INPUT: u32 = 0;
pub const GPIO_OUTPUT: u32 = 1;
pub const GPIO_OPEN_DRAIN: u32 = 0x100;
pub const GPIO_HIGH_SPEED: u32 = 0x200;
pub const fn GPIO_FUNCTION(func: u32) -> u32 {
    2 | (func << 4)
}
pub const GPIO_ANALOG: u32 = 3;
#[cfg(not(CONFIG_MACH_STM32F1))]
pub use super::gpioperiph::gpio_peripheral;
#[cfg(CONFIG_MACH_STM32F1)]
pub use super::stm32f1::gpio_peripheral;

// clockline.c
pub use super::clockline::{enable_pclock, is_enabled_pclock};

// dfu_reboot.c
pub use super::dfu_reboot::{dfu_reboot, dfu_reboot_check};

// stm32??.c
pub struct Cline {
    pub en: *const Reg<u32>,
    pub rst: *const Reg<u32>,
    pub bit: u32,
}
#[cfg(CONFIG_MACH_STM32F0)]
pub use super::stm32f0::{get_pclock_frequency, gpio_clock_enable, lookup_clock_line};
#[cfg(CONFIG_MACH_STM32G0)]
pub use super::stm32g0::{get_pclock_frequency, gpio_clock_enable, lookup_clock_line};
#[cfg(CONFIG_MACH_STM32F1)]
pub use super::stm32f1::{get_pclock_frequency, gpio_clock_enable, lookup_clock_line};
#[cfg(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4))]
pub use super::stm32f4::{get_pclock_frequency, gpio_clock_enable, lookup_clock_line};
