// Local definitions for atsamd code (internal.h)
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

// GET_FUSE(REG): the bits REG##_Msk of the fuse word at REG##_ADDR
pub fn GET_FUSE(addr: u32, msk: u32, pos: u32) -> u32 {
    (unsafe { (addr as usize as *const u32).read_volatile() } & msk) >> pos
}

pub use super::clock::{enable_pclock, get_pclock_frequency, SystemInit};
pub use super::gpio::gpio_peripheral;
#[cfg(CONFIG_HAVE_SERCOM)]
pub use super::sercom::{sercom_enable_pclock, sercom_get_pclock_frequency, sercom_i2c_pins, sercom_spi_pins};
