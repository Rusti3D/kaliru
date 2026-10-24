// Local definitions for sam3/sam4 code (internal.h)
#![allow(non_snake_case)]
pub use super::chip::*;
pub use crate::reg::Reg;

pub const fn GPIO(port: char, num: u32) -> u32 {
    (port as u32 - 'A' as u32) * 32 + num
}
pub const fn GPIO2PORT(pin: u32) -> u32 {
    pin / 32
}
pub const fn GPIO2BIT(pin: u32) -> u32 {
    1 << (pin % 32)
}

pub use super::gpio::gpio_peripheral;
pub use super::main::{enable_pclock, get_pclock_frequency, is_enabled_pclock};
// (SystemInit(): the library's, or Kalico's for the sam4s and same70)
#[cfg(CONFIG_MACH_SAM3X)]
pub use super::system_sam3xa::SystemInit;
#[cfg(CONFIG_MACH_SAM4S)]
pub use super::sam4s_sysinit::SystemInit;
#[cfg(CONFIG_MACH_SAM4E)]
pub use super::system_sam4e::SystemInit;
#[cfg(CONFIG_MACH_SAME70)]
pub use super::same70_sysinit::SystemInit;
