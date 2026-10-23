// src/generic: code shared by the boards
#[cfg(CONFIG_MACH_LINUX)]
pub mod alloc;
#[cfg(CONFIG_MACH_STM32)]
pub mod armcm_boot;
#[cfg(CONFIG_MACH_STM32)]
pub mod armcm_irq;
#[cfg(CONFIG_MACH_STM32)]
pub mod armcm_reset;
#[cfg(CONFIG_MACH_STM32)]
pub mod armcm_timer;
#[cfg(CONFIG_MACH_STM32)]
pub mod cmsis;
pub mod crc16_ccitt;
#[cfg(CONFIG_USBSERIAL)]
pub mod usb_cdc;
#[cfg(CONFIG_USBSERIAL)]
pub mod usb_cdc_ep;
#[cfg(CONFIG_USBSERIAL)]
pub mod usbstd;
#[cfg(CONFIG_USBSERIAL)]
pub mod usbstd_cdc;
