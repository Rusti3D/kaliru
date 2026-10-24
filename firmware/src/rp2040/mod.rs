// src/rp2040: the RP2040 / RP2350 boards. The board interface (board/misc.h,
// board/irq.h, board/gpio.h, ...) as re-exports.
#[allow(non_upper_case_globals, non_camel_case_types, non_snake_case, dead_code, clippy::all)]
pub mod chip {
    // the pico-sdk device headers (CONFIG_MCU), made by tools/gen_stm32_regs.py --pico
    include!(concat!(env!("OUT_DIR"), "/chip.rs"));
}
pub mod internal;

pub mod adc;
pub mod gpio;
pub mod gpio_h;
pub mod main;
#[cfg(CONFIG_RP2040_HAVE_STAGE2)]
pub mod stage2;
pub mod watchdog;
#[cfg(CONFIG_MACH_RP2040)]
pub mod bootrom;
#[cfg(CONFIG_MACH_RP2040)]
pub mod timer;
#[cfg(CONFIG_MACH_RP2350)]
pub mod rp2350_bootrom;
#[cfg(CONFIG_MACH_RP2350)]
pub use rp2350_bootrom as bootrom;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub mod usbserial;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod chipid;
#[cfg(CONFIG_SERIAL)]
pub mod serial;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod can;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod can2040;
#[cfg(CONFIG_WANT_HARD_PWM)]
pub mod hard_pwm;
#[cfg(CONFIG_WANT_GPIO_SPI)]
pub mod spi;
#[cfg(CONFIG_WANT_GPIO_I2C)]
pub mod i2c;

pub use crate::generic::armcm_boot::{dynmem_end, dynmem_start};
pub use crate::generic::armcm_irq::{irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, IrqStatus};
#[cfg(CONFIG_MACH_RP2350)]
pub use crate::generic::armcm_timer::{timer_from_us, timer_is_before, timer_kick, timer_read_time};
#[cfg(CONFIG_MACH_RP2040)]
pub use crate::generic::timer_irq::{timer_from_us, timer_is_before};
#[cfg(CONFIG_MACH_RP2040)]
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
pub use can::{canhw_get_status, canhw_send, canhw_set_filter};
#[cfg(CONFIG_WANT_ADC)]
pub use adc::{gpio_adc_cancel_sample, gpio_adc_read, gpio_adc_sample, gpio_adc_setup};
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub use chipid::usbserial_get_serialid;
pub use gpio::{
    gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, gpio_out_toggle, gpio_out_toggle_noirq, gpio_out_write,
};
pub use gpio_h::{GpioAdc, GpioIn, GpioOut, GpioOutVal, GpioPullUp, GpioPwm, I2cConfig, SpiConfig};
#[cfg(CONFIG_WANT_HARD_PWM)]
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
