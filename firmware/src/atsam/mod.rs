// src/atsam: the SAM3X / SAM4S / SAM4E / SAME70 boards. The board interface
// (board/misc.h, board/irq.h, board/gpio.h, ...) as re-exports. (Its Makefile
// builds adc, hard_pwm, i2c and spi by chip, not by the WANT_* options.)
#[allow(non_upper_case_globals, non_camel_case_types, non_snake_case, dead_code, clippy::all)]
pub mod chip {
    // the CMSIS device header (CONFIG_MCU), made by tools/gen_stm32_regs.py --cmsis
    include!(concat!(env!("OUT_DIR"), "/chip.rs"));
}
pub mod internal;

#[cfg(any(CONFIG_MACH_SAM3X, CONFIG_MACH_SAM4S))]
pub mod adc;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod chipid;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod fdcan;
pub mod gpio;
pub mod gpio_h;
#[cfg(any(CONFIG_MACH_SAM3X, CONFIG_MACH_SAM4))]
pub mod hard_pwm;
pub mod i2c;
pub mod main;
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), any(CONFIG_MACH_SAM3X, CONFIG_MACH_SAME70)))]
pub mod sam3_usb;
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), any(CONFIG_MACH_SAM3X, CONFIG_MACH_SAME70)))]
pub use sam3_usb as usb;
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), CONFIG_MACH_SAM4))]
pub mod sam4_usb;
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), CONFIG_MACH_SAM4))]
pub use sam4_usb as usb;
#[cfg(CONFIG_MACH_SAM4E)]
pub mod sam4_cache;
#[cfg(any(CONFIG_MACH_SAM4E, CONFIG_MACH_SAME70))]
pub mod sam4e_afec;
#[cfg(any(CONFIG_MACH_SAM4E, CONFIG_MACH_SAME70))]
pub use sam4e_afec as adc;
#[cfg(CONFIG_MACH_SAM4S)]
pub mod sam4s_sysinit;
#[cfg(CONFIG_MACH_SAME70)]
pub mod same70_sysinit;
#[cfg(CONFIG_SERIAL)]
pub mod serial;
pub mod spi;
#[cfg(CONFIG_MACH_SAM3X)]
#[allow(non_snake_case)]
pub mod system_sam3xa;
#[cfg(CONFIG_MACH_SAM4E)]
#[allow(non_snake_case)]
pub mod system_sam4e;

pub use crate::generic::armcm_boot::{dynmem_end, dynmem_start};
pub use crate::generic::armcm_irq::{irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, IrqStatus};
pub use crate::generic::armcm_timer::{timer_from_us, timer_is_before, timer_kick, timer_read_time};
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
pub use adc::{gpio_adc_cancel_sample, gpio_adc_read, gpio_adc_sample, gpio_adc_setup};
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub use chipid::usbserial_get_serialid;
pub use gpio::{
    gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, gpio_out_toggle, gpio_out_toggle_noirq, gpio_out_write,
};
pub use gpio_h::{GpioAdc, GpioIn, GpioOut, GpioOutVal, GpioPullUp, GpioPwm, I2cConfig, SpiConfig};
#[cfg(any(CONFIG_MACH_SAM3X, CONFIG_MACH_SAM4))]
pub use hard_pwm::{gpio_pwm_setup, gpio_pwm_write};
pub use i2c::{i2c_read, i2c_setup, i2c_write};
pub use main::{armcm_main, bootloader_request};
pub use spi::{spi_prepare, spi_setup, spi_transfer};
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub use usb::{
    usb_read_bulk_out, usb_read_ep0, usb_read_ep0_setup, usb_send_bulk_in, usb_send_ep0, usb_set_address, usb_set_configure,
    usb_stall_ep0,
};
