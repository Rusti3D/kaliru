// The board's gpio.h (stm32/gpio.h): the types of the gpio, adc, pwm,
// spi and i2c functions (the same sizes as the C structs: they are part of
// the objects of the command modules)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;

// the value / pull_up parameter types of gpio_out_setup() / gpio_in_setup()
pub type GpioOutVal = u32;
pub type GpioPullUp = i32;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioOut {
    pub regs: *mut GPIO_TypeDef,
    // (a union: stm32h7 uses 'oc'; all others use 'bit')
    #[cfg(not(CONFIG_MACH_STM32H7))]
    pub bit: u32,
    #[cfg(CONFIG_MACH_STM32H7)]
    pub oc: *mut OdrCache,
}

// (stm32h7_gpio.c: the cached ODR state of a pin)
#[repr(C)]
pub struct OdrCache {
    pub bsrr: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioIn {
    pub regs: *mut GPIO_TypeDef,
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
    // (C: void *)
    #[cfg(not(CONFIG_MACH_N32G45x))]
    pub adc: *mut ADC_TypeDef,
    #[cfg(CONFIG_MACH_N32G45x)]
    pub adc: *mut super::n32g45x_adc_h::ADC_Module,
    pub chan: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SpiConfig {
    pub spi: *mut SPI_TypeDef,
    // (a union with the div / mode bytes of other boards' spi code)
    pub spi_cr1: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct I2cConfig {
    pub i2c: *mut I2C_TypeDef,
    pub addr: u8,
}

