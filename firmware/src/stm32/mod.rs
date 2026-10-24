// src/stm32: the STM32 boards. The board interface (board/misc.h,
// board/irq.h, board/gpio.h, ...) as re-exports.
#[allow(non_upper_case_globals, non_camel_case_types, non_snake_case, dead_code, clippy::all)]
pub mod chip {
    // the CMSIS device header (CONFIG_MCU), made by tools/gen_stm32_regs.py
    include!(concat!(env!("OUT_DIR"), "/chip.rs"));
}
pub mod internal;

pub mod clockline;
pub mod dfu_reboot;
#[cfg(not(CONFIG_MACH_STM32H7))]
pub mod gpio;
#[cfg(CONFIG_MACH_STM32H7)]
pub mod stm32h7_gpio;
#[cfg(CONFIG_MACH_STM32H7)]
pub use stm32h7_gpio as gpio;
pub mod gpio_h;
#[cfg(not(CONFIG_MACH_STM32F1))]
pub mod gpioperiph;
#[cfg(CONFIG_MACH_STM32F0)]
pub mod stm32f0;
#[cfg(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0))]
pub mod stm32f0_timer;
#[cfg(CONFIG_MACH_STM32F0)]
pub mod system_stm32f0xx;
#[cfg(CONFIG_MACH_STM32G0)]
pub mod stm32g0;
#[cfg(CONFIG_MACH_STM32G4)]
pub mod stm32g4;
#[cfg(CONFIG_MACH_STM32G4)]
pub mod system_stm32g4xx;
#[cfg(CONFIG_MACH_STM32F7)]
pub mod stm32f7;
#[cfg(CONFIG_MACH_STM32F7)]
pub mod system_stm32f7xx;
#[cfg(CONFIG_MACH_STM32H7)]
pub mod stm32h7;
#[cfg(CONFIG_MACH_STM32H7)]
pub mod system_stm32h7xx;
#[cfg(CONFIG_MACH_STM32L4)]
pub mod stm32l4;
#[cfg(CONFIG_MACH_STM32L4)]
pub mod system_stm32l4xx;
#[cfg(CONFIG_MACH_STM32F1)]
pub mod stm32f1;
#[cfg(CONFIG_MACH_STM32F1)]
pub mod system_stm32f1xx;
#[cfg(CONFIG_MACH_STM32F2)]
pub mod system_stm32f2xx;
#[cfg(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4))]
pub mod stm32f4;
#[cfg(CONFIG_MACH_STM32F4)]
pub mod system_stm32f4xx;
pub mod watchdog;
#[cfg(all(
    CONFIG_WANT_ADC,
    not(CONFIG_MACH_N32G45x),
    any(CONFIG_MACH_STM32F1, CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4, CONFIG_MACH_STM32F7)
))]
pub mod adc;
#[cfg(all(CONFIG_WANT_ADC, CONFIG_MACH_N32G45x))]
#[allow(non_snake_case)]
pub mod n32g45x_adc;
#[cfg(all(CONFIG_WANT_ADC, CONFIG_MACH_N32G45x))]
pub use n32g45x_adc as adc;
// (lib/n32g45x/include/n32g45x_adc.h, made by tools/gen_stm32_regs.py)
#[cfg(all(CONFIG_WANT_ADC, CONFIG_MACH_N32G45x))]
#[allow(non_upper_case_globals, non_camel_case_types, non_snake_case, dead_code, clippy::all)]
pub mod n32g45x_adc_h;
#[cfg(all(CONFIG_WANT_ADC, CONFIG_MACH_N32G45x))]
#[allow(non_snake_case)]
pub mod n32g45x_adc_lib;
#[cfg(all(CONFIG_WANT_ADC, any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0)))]
pub mod stm32f0_adc;
#[cfg(all(CONFIG_WANT_ADC, any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0)))]
pub use stm32f0_adc as adc;
#[cfg(all(CONFIG_WANT_ADC, any(CONFIG_MACH_STM32G4, CONFIG_MACH_STM32H7, CONFIG_MACH_STM32L4)))]
pub mod stm32h7_adc;
#[cfg(all(CONFIG_WANT_ADC, any(CONFIG_MACH_STM32G4, CONFIG_MACH_STM32H7, CONFIG_MACH_STM32L4)))]
pub use stm32h7_adc as adc;
#[cfg(all(CONFIG_WANT_GPIO_SPI, not(CONFIG_MACH_STM32H7)))]
pub mod spi;
#[cfg(all(CONFIG_WANT_GPIO_SPI, CONFIG_MACH_STM32H7))]
pub mod stm32h7_spi;
#[cfg(all(CONFIG_WANT_GPIO_SPI, CONFIG_MACH_STM32H7))]
pub use stm32h7_spi as spi;
#[cfg(all(CONFIG_WANT_GPIO_I2C, any(CONFIG_MACH_STM32F1, CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4)))]
pub mod i2c;
#[cfg(all(CONFIG_WANT_GPIO_I2C, not(any(CONFIG_MACH_STM32F1, CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4))))]
pub mod stm32f0_i2c;
#[cfg(all(CONFIG_WANT_GPIO_I2C, not(any(CONFIG_MACH_STM32F1, CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4))))]
pub use stm32f0_i2c as i2c;
#[cfg(all(CONFIG_SERIAL, not(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G4, CONFIG_MACH_STM32H7))))]
pub mod serial;
#[cfg(all(CONFIG_SERIAL, any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G4, CONFIG_MACH_STM32H7)))]
pub mod stm32f0_serial;
#[cfg(all(CONFIG_SERIAL, any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G4, CONFIG_MACH_STM32H7)))]
pub use stm32f0_serial as serial;
#[cfg(CONFIG_SERIAL)]
pub use serial::serial_enable_tx_irq;
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), CONFIG_HAVE_STM32_USBFS))]
pub mod usbfs;
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), CONFIG_HAVE_STM32_USBOTG))]
pub mod usbotg;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod chipid;
#[cfg(all(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS), CONFIG_HAVE_STM32_CANBUS))]
pub mod can;
#[cfg(all(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS), CONFIG_HAVE_STM32_FDCANBUS))]
pub mod fdcan;
#[cfg(all(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS), CONFIG_HAVE_STM32_FDCANBUS))]
pub use fdcan as can;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub use can::{canhw_get_status, canhw_send, canhw_set_filter};
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub use crate::generic::canserial::console_sendf;
#[cfg(CONFIG_WANT_HARD_PWM)]
pub mod hard_pwm;
#[cfg(CONFIG_HAVE_GPIO_SDIO)]
pub mod sdio;

pub use crate::generic::armcm_boot::{dynmem_end, dynmem_start};
pub use crate::generic::armcm_irq::{irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, IrqStatus};
#[cfg(not(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0)))]
pub use crate::generic::armcm_timer::{timer_from_us, timer_is_before, timer_kick, timer_read_time};
#[cfg(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0))]
pub use crate::generic::timer_irq::{timer_from_us, timer_is_before};
#[cfg(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0))]
pub use stm32f0_timer::{timer_kick, timer_read_time};
pub use crate::generic::crc16_ccitt::crc16_ccitt;
#[cfg(CONFIG_SERIAL)]
pub use crate::generic::serial_irq::console_sendf;
#[cfg(CONFIG_USBSERIAL)]
pub use crate::generic::usb_cdc::console_sendf;
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
#[cfg(CONFIG_HAVE_GPIO_SDIO)]
pub use sdio::{sdio_prepare_data_transfer, sdio_read_data, sdio_send_command, sdio_set_speed, sdio_setup, sdio_write_data, SdioConfig};
#[cfg(CONFIG_WANT_GPIO_SPI)]
pub use spi::{spi_prepare, spi_setup, spi_transfer};
#[cfg(CONFIG_MACH_STM32F0)]
pub use stm32f0::{armcm_main, bootloader_request};
#[cfg(CONFIG_MACH_STM32G0)]
pub use stm32g0::{armcm_main, bootloader_request};
#[cfg(CONFIG_MACH_STM32G4)]
pub use stm32g4::{armcm_main, bootloader_request};
#[cfg(CONFIG_MACH_STM32L4)]
pub use stm32l4::{armcm_main, bootloader_request};
#[cfg(CONFIG_MACH_STM32F7)]
pub use stm32f7::{armcm_main, bootloader_request};
#[cfg(CONFIG_MACH_STM32H7)]
pub use stm32h7::{armcm_main, bootloader_request};
#[cfg(CONFIG_MACH_STM32F1)]
pub use stm32f1::{armcm_main, bootloader_request};
#[cfg(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4))]
pub use stm32f4::{armcm_main, bootloader_request};
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), CONFIG_HAVE_STM32_USBFS))]
pub use usbfs::{
    usb_read_bulk_out, usb_read_ep0, usb_read_ep0_setup, usb_send_bulk_in, usb_send_ep0, usb_set_address, usb_set_configure,
    usb_stall_ep0,
};
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), CONFIG_HAVE_STM32_USBOTG))]
pub use usbotg::{
    usb_read_bulk_out, usb_read_ep0, usb_read_ep0_setup, usb_send_bulk_in, usb_send_ep0, usb_set_address, usb_set_configure,
    usb_stall_ep0,
};
