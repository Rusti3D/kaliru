// Local definitions for Huada HC32F460 (internal.h)
#![allow(non_snake_case)]
pub use super::chip::*;
// (the chip's en_result_t `Ok` would hide the Result one)
pub use core::result::Result::Ok;
pub use crate::reg::Reg;

// The HC32F460 library uses a port address and a shifted pin bit
//  eg en_result_t PORT_Toggle(en_port_t enPort, uint16_t u16Pin);
//  see hc32f460_gpio.h

// encode and decode gpio ports and pins
pub const fn GPIO(port: char, num: u32) -> u32 {
    (port as u32 - 'A' as u32) * 16 + num
}
pub const fn GPIO2PORT(gpio: u32) -> u32 {
    gpio / 16
}
pub const fn GPIO2BIT(gpio: u32) -> u32 {
    1 << (gpio % 16)
}
pub const fn GPIO2PIN(gpio: u32) -> u32 {
    gpio % 16
}

pub const GPIO_INPUT: u32 = 0;
pub const GPIO_OUTPUT: u32 = 1;

pub use super::gpio::gpio_peripheral;
// from local interrupts.c - helper
pub use super::interrupts::IrqRegistration;
