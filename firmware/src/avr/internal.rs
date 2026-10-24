// Local definitions for avr code (avr/internal.h)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
pub use super::chip::*;
pub use crate::reg::Reg;

pub const fn GPIO(port: char, num: u8) -> u8 {
    (port as u8 - 'A' as u8) * 8 + num
}
pub const fn GPIO2PORT(pin: u8) -> u8 {
    pin / 8
}
pub const fn GPIO2BIT(pin: u8) -> u8 {
    1 << (pin % 8)
}

// struct gpio_digital_regs: a port's PINx, DDRx, PORTx registers
#[repr(C)]
pub struct GpioDigitalRegs {
    pub in_: Reg<u8>,
    pub mode: Reg<u8>,
    pub out: Reg<u8>,
}

// a pointer in a PROGMEM table (the C `volatile uint8_t * const ...[]
// PROGMEM`)
#[repr(transparent)]
pub struct PgmPtr<T>(pub *mut T);
unsafe impl<T> Sync for PgmPtr<T> {}
impl<T> Clone for PgmPtr<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> Copy for PgmPtr<T> {}

pub use super::gpio::DIGITAL_REGS;

// GPIO2REGS(pin): the port's registers, from the PROGMEM table
pub fn GPIO2REGS(pin: u8) -> *mut GpioDigitalRegs {
    super::pgm::readp(&DIGITAL_REGS[GPIO2PORT(pin) as usize]).0
}
