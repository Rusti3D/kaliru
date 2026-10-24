// The board's gpio.h (atsam/gpio.h): the types of the gpio, adc, pwm,
// spi and i2c functions (the same sizes as the C structs: they are part of
// the objects of the command modules)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;

// the value / pull_up parameter types of gpio_out_setup() / gpio_in_setup()
pub type GpioOutVal = u8;
pub type GpioPullUp = i8;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioOut {
    pub regs: *mut Pio,
    pub bit: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioIn {
    pub regs: *mut Pio,
    pub bit: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioPwm {
    pub reg: *mut Reg<u32>,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioAdc {
    pub chan: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SpiConfig {
    // (C: void *; an Spi or a Usart)
    pub spidev: *mut u8,
    pub cfg: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct I2cConfig {
    pub twi: *mut u8,
    pub addr: u8,
}
