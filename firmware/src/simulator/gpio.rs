// GPIO functions on simulator.
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::generic::gpio_h::{GpioAdc, GpioIn, GpioOut, GpioOutVal, GpioPullUp, GpioPwm, SpiConfig};
use crate::sched::R;

pub fn gpio_out_setup(pin: u32, _val: GpioOutVal) -> R<GpioOut> {
    Ok(GpioOut { pin: pin as u8 })
}
pub fn gpio_out_reset(_g: GpioOut, _val: GpioOutVal) -> R {
    Ok(())
}
pub fn gpio_out_toggle_noirq(_g: GpioOut) {}
pub fn gpio_out_toggle(_g: GpioOut) {}
pub fn gpio_out_write(_g: GpioOut, _val: u8) {}
pub fn gpio_in_setup(pin: u32, _pull_up: GpioPullUp) -> R<GpioIn> {
    Ok(GpioIn { pin: pin as u8 })
}
pub fn gpio_in_reset(_g: GpioIn, _pull_up: GpioPullUp) -> R {
    Ok(())
}
pub fn gpio_in_read(_g: GpioIn) -> u8 {
    0
}
// (C: a uint8_t val, the low byte of the pwmcmds value)
pub fn gpio_pwm_setup(pin: u32, _cycle_time: u32, _val: u16) -> R<GpioPwm> {
    Ok(GpioPwm { pin: pin as u8 })
}
pub fn gpio_pwm_write(_g: GpioPwm, _val: u16) {}
pub fn gpio_adc_setup(pin: u32) -> R<GpioAdc> {
    Ok(GpioAdc { pin: pin as u8 })
}
pub fn gpio_adc_sample(_g: GpioAdc) -> u32 {
    0
}
pub fn gpio_adc_read(_g: GpioAdc) -> R<u16> {
    Ok(0)
}
pub fn gpio_adc_cancel_sample(_g: GpioAdc) {}

pub fn spi_setup(_bus: u32, _mode: u8, _rate: u32) -> R<SpiConfig> {
    Ok(SpiConfig::default())
}
pub fn spi_prepare(_config: SpiConfig) {}
pub fn spi_transfer(_config: SpiConfig, _receive_data: u8, _data: &mut [u8]) -> R {
    Ok(())
}
