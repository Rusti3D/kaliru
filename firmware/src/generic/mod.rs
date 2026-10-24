// src/generic: code shared by the boards
#[cfg(any(CONFIG_MACH_LINUX, CONFIG_MACH_SIMU))]
pub mod alloc;
#[cfg(any(CONFIG_MACH_STM32, CONFIG_MACH_RPXXXX, CONFIG_MACH_LPC176X))]
pub mod armcm_boot;
#[cfg(any(CONFIG_MACH_STM32, CONFIG_MACH_RPXXXX, CONFIG_MACH_LPC176X))]
pub mod armcm_irq;
#[cfg(any(CONFIG_MACH_STM32, CONFIG_MACH_RPXXXX, CONFIG_MACH_LPC176X))]
pub mod armcm_reset;
#[cfg(any(all(CONFIG_MACH_STM32, not(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0))), CONFIG_MACH_RP2350, CONFIG_MACH_LPC176X))]
pub mod armcm_timer;
#[cfg(any(CONFIG_MACH_STM32, CONFIG_MACH_RPXXXX, CONFIG_MACH_LPC176X))]
pub mod cmsis;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod canbus;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod canbus_h;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod canserial;
pub mod crc16_ccitt;
#[cfg(CONFIG_MACH_SIMU)]
pub mod gpio_h;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod fasthash;
#[cfg(CONFIG_SERIAL)]
pub mod serial_irq;
#[cfg(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0, CONFIG_MACH_RP2040, CONFIG_MACH_SIMU))]
pub mod timer_irq;
#[cfg(CONFIG_USBSERIAL)]
pub mod usb_cdc;
#[cfg(CONFIG_USBCANBUS)]
pub mod usb_canbus;
// (usb_canbus.c provides the usb_cdc.c functions in that mode)
#[cfg(CONFIG_USBCANBUS)]
pub use usb_canbus as usb_cdc;
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), not(CONFIG_MACH_LPC176X)))]
pub mod usb_cdc_ep;
// (the board's usb_cdc_ep.h, where it has one)
#[cfg(all(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS), CONFIG_MACH_LPC176X))]
pub use crate::lpc176x::usb_cdc_ep;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub mod usbstd;
#[cfg(CONFIG_USBSERIAL)]
pub mod usbstd_cdc;
