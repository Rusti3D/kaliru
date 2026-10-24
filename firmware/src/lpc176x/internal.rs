// Local definitions for lpc176x code (internal.h)
#![allow(non_snake_case)]
pub use super::chip::*;
pub use crate::reg::Reg;

pub const fn GPIO(port: u32, num: u32) -> u32 {
    port * 32 + num
}
pub const fn GPIO2PORT(pin: u32) -> u32 {
    pin / 32
}
pub const fn GPIO2BIT(pin: u32) -> u32 {
    1 << (pin % 32)
}

pub const PCLK_TIMER0: u32 = 1;
pub const PCLK_UART0: u32 = 3;
pub const PCLK_PWM1: u32 = 6;
pub const PCLK_I2C0: u32 = 7;
pub const PCLK_SSP1: u32 = 10;
pub const PCLK_ADC: u32 = 12;
pub const PCLK_CAN1: u32 = 13;
pub const PCLK_CAN2: u32 = 14;
pub const PCLK_I2C1: u32 = 19;
pub const PCLK_SSP0: u32 = 21;
pub const PCLK_UART3: u32 = 25;
pub const PCLK_I2C2: u32 = 26;
pub const PCLK_USB: u32 = 31;
pub use super::gpio::gpio_peripheral;
pub use super::main::{enable_pclock, get_pclock_frequency, is_enabled_pclock};
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub use super::usbserial::usb_disconnect;
