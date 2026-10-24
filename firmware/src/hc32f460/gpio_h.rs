// The board's gpio.h (hc32f460/gpio.h): the types of the gpio, adc, pwm,
// spi and i2c functions (the same sizes as the C structs: they are part of
// the objects of the command modules)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]

// the value / pull_up parameter types of gpio_out_setup() / gpio_in_setup()
pub type GpioOutVal = u32;
pub type GpioPullUp = i32;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioOut {
    pub portAddress: u32, // M4_PORT or offset
    pub gpio: u16,        // the mangled pin+port
    pub bitMask: u16,     // the pin shifted to use in register
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioIn {
    pub portAddress: u32,
    pub gpio: u16,
    pub bitMask: u16,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioPwm {
    pub timer: *mut super::internal::M4_TMRA_TypeDef,
    pub chan: u32,
}

// all ADC operations will be on ADC1
#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioAdc {
    pub chan: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SpiConfig {
    pub spi: *mut (),
    pub spi_cr1: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct I2cConfig {
    pub i2c: *mut (),
    pub addr: u8,
}
