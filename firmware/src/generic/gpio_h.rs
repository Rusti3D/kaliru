// The generic gpio.h (generic/gpio.h): the board gpio types of the boards
// without one of their own (the simulator)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// the value / pull_up parameter types of gpio_out_setup() / gpio_in_setup()
pub type GpioOutVal = u8;
pub type GpioPullUp = i8;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioOut {
    pub pin: u8,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioIn {
    pub pin: u8,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioPwm {
    pub pin: u8,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpioAdc {
    pub pin: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct SpiConfig {
    pub cfg: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct I2cConfig {
    pub cfg: u32,
}
