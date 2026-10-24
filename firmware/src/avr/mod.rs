// src/avr: the AVR boards (atmega, at90usb). The board interface
// (board/misc.h, board/irq.h, board/gpio.h, board/pgm.h, ...) as re-exports.
// (its avr_vector! macro is for the modules below)
#[macro_use]
#[allow(non_upper_case_globals, non_camel_case_types, non_snake_case, dead_code, unused_macros, clippy::all)]
pub mod chip {
    // avr-libc's <avr/io.h> for the mcu, made by tools/gen_avr_regs.py
    include!(concat!(env!("OUT_DIR"), "/chip.rs"));
}
pub mod gpio_h;
pub mod internal;
pub mod irq;
pub mod pgm;

#[cfg(CONFIG_WANT_ADC)]
pub mod adc;
pub mod gpio;
#[cfg(CONFIG_WANT_HARD_PWM)]
pub mod hard_pwm;
#[cfg(CONFIG_WANT_GPIO_I2C)]
pub mod i2c;
pub mod main;
#[cfg(CONFIG_SERIAL)]
pub mod serial;
#[cfg(CONFIG_WANT_GPIO_SPI)]
pub mod spi;
pub mod timer;
#[cfg(CONFIG_USBSERIAL)]
pub mod usbserial;
#[cfg(CONFIG_AVR_WATCHDOG)]
pub mod watchdog;

#[cfg(CONFIG_WANT_ADC)]
pub use adc::{gpio_adc_cancel_sample, gpio_adc_read, gpio_adc_sample, gpio_adc_setup};
#[cfg(CONFIG_SERIAL)]
pub use crate::generic::serial_irq::console_sendf;
#[cfg(CONFIG_USBSERIAL)]
pub use crate::generic::usb_cdc::console_sendf;
pub use gpio::{
    gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, gpio_out_toggle, gpio_out_toggle_noirq, gpio_out_write,
};
pub use gpio_h::{GpioAdc, GpioIn, GpioOut, GpioOutVal, GpioPullUp, GpioPwm, I2cConfig, SpiConfig};
#[cfg(CONFIG_WANT_HARD_PWM)]
pub use hard_pwm::{gpio_pwm_setup, gpio_pwm_write};
#[cfg(CONFIG_WANT_GPIO_I2C)]
pub use i2c::{i2c_read, i2c_setup, i2c_write};
pub use irq::{irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, shutdown_jump, IrqStatus};
pub use main::{crc16_ccitt, dynmem_end, dynmem_start};
#[cfg(CONFIG_SERIAL)]
pub use serial::serial_enable_tx_irq;
#[cfg(CONFIG_WANT_GPIO_SPI)]
pub use spi::{spi_prepare, spi_setup, spi_transfer};
pub use timer::{timer_from_us, timer_is_before, timer_kick, timer_read_time};
#[cfg(CONFIG_USBSERIAL)]
pub use usbserial::{
    usb_read_bulk_out, usb_read_ep0, usb_read_ep0_setup, usb_send_bulk_in, usb_send_ep0, usb_send_ep0_progmem, usb_set_address,
    usb_set_configure, usb_stall_ep0,
};
