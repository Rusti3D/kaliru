// The board's gpio.h (rp2040/gpio.h): the types of the gpio, adc, pwm,
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
    // (C: void *; sio_hw, or 4 bytes past it for the rp2350 high bank)
    pub sio: *mut u8,
    pub bit: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioIn {
    pub sio: *mut u8,
    pub bit: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioPwm {
    pub reg: *mut Reg<u32>,
    pub hwpwm_ticks: u32,
    pub shift: u32,
    pub mask: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioAdc {
    pub chan: u8,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SpiConfig {
    pub spi: *mut spi_hw_t,
    pub cr0: u32,
    pub cpsr: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct I2cConfig {
    pub i2c: *mut i2c_hw_t,
    pub addr: u8,
}
