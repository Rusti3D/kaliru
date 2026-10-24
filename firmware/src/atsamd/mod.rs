// src/atsamd: the SAMC21 / SAMD21 / SAMD51 / SAME51 / SAME54 boards. The board
// interface (board/misc.h, board/irq.h, board/gpio.h, ...) as re-exports.
#[allow(non_upper_case_globals, non_camel_case_types, non_snake_case, dead_code, clippy::all)]
pub mod chip {
    // the CMSIS device header (CONFIG_MCU), made by tools/gen_stm32_regs.py --cmsis
    include!(concat!(env!("OUT_DIR"), "/chip.rs"));
}
pub mod internal;

#[cfg(CONFIG_WANT_GPIO_ADC)]
pub mod adc;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod chipid;
#[cfg(CONFIG_MACH_SAMD21)]
pub mod clock;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod fdcan;
pub mod gpio;
pub mod gpio_h;
#[cfg(CONFIG_HAVE_GPIO_HARD_PWM)]
pub mod hard_pwm;
#[cfg(CONFIG_WANT_GPIO_I2C)]
pub mod i2c;
pub mod main;
#[cfg(CONFIG_MACH_SAMC21)]
pub mod samc21_clock;
#[cfg(CONFIG_MACH_SAMC21)]
pub use samc21_clock as clock;
#[cfg(CONFIG_MACH_SAMX5)]
pub mod samd51_clock;
#[cfg(CONFIG_MACH_SAMX5)]
pub use samd51_clock as clock;
#[cfg(any(CONFIG_MACH_SAMC21, CONFIG_MACH_SAMX5))]
pub mod samd51_watchdog;
#[cfg(CONFIG_HAVE_SERCOM)]
pub mod sercom;
#[cfg(CONFIG_SERIAL)]
pub mod serial;
#[cfg(CONFIG_WANT_GPIO_SPI)]
pub mod spi;
#[cfg(CONFIG_MACH_SAMX2)]
pub mod timer;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub mod usbserial;
#[cfg(CONFIG_MACH_SAMD21)]
pub mod watchdog;

pub use crate::generic::armcm_boot::{dynmem_end, dynmem_start};
pub use crate::generic::armcm_irq::{irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, IrqStatus};
#[cfg(CONFIG_MACH_SAMX5)]
pub use crate::generic::armcm_timer::{timer_from_us, timer_is_before, timer_kick, timer_read_time};
#[cfg(CONFIG_MACH_SAMX2)]
pub use crate::generic::timer_irq::{timer_from_us, timer_is_before};
#[cfg(CONFIG_MACH_SAMX2)]
pub use timer::{timer_kick, timer_read_time};
pub use crate::generic::crc16_ccitt::crc16_ccitt;
#[cfg(CONFIG_SERIAL)]
pub use crate::generic::serial_irq::console_sendf;
#[cfg(CONFIG_SERIAL)]
pub use serial::serial_enable_tx_irq;
#[cfg(CONFIG_USBSERIAL)]
pub use crate::generic::usb_cdc::console_sendf;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub use crate::generic::canserial::console_sendf;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub use fdcan::{canhw_get_status, canhw_send, canhw_set_filter};
#[cfg(CONFIG_WANT_GPIO_ADC)]
pub use adc::{gpio_adc_cancel_sample, gpio_adc_read, gpio_adc_sample, gpio_adc_setup};
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub use chipid::usbserial_get_serialid;
pub use gpio::{
    gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, gpio_out_toggle, gpio_out_toggle_noirq, gpio_out_write,
};
pub use gpio_h::{GpioAdc, GpioIn, GpioOut, GpioOutVal, GpioPullUp, GpioPwm, I2cConfig, SpiConfig};
#[cfg(CONFIG_HAVE_GPIO_HARD_PWM)]
pub use hard_pwm::{gpio_pwm_setup, gpio_pwm_write};
#[cfg(CONFIG_WANT_GPIO_I2C)]
pub use i2c::{i2c_read, i2c_setup, i2c_write};
pub use main::{armcm_main, bootloader_request};
#[cfg(CONFIG_WANT_GPIO_SPI)]
pub use spi::{spi_prepare, spi_setup, spi_transfer};
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub use usbserial::{
    usb_read_bulk_out, usb_read_ep0, usb_read_ep0_setup, usb_send_bulk_in, usb_send_ep0, usb_set_address, usb_set_configure,
    usb_stall_ep0,
};
