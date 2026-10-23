// src/generic: code shared by the boards
#[cfg(CONFIG_MACH_LINUX)]
pub mod alloc;
#[cfg(CONFIG_MACH_STM32)]
pub mod armcm_boot;
#[cfg(CONFIG_MACH_STM32)]
pub mod armcm_irq;
#[cfg(CONFIG_MACH_STM32)]
pub mod armcm_reset;
#[cfg(all(CONFIG_MACH_STM32, not(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0))))]
pub mod armcm_timer;
#[cfg(CONFIG_MACH_STM32)]
pub mod cmsis;
#[cfg(CONFIG_CANSERIAL)]
pub mod canbus;
#[cfg(CONFIG_CANSERIAL)]
pub mod canserial;
pub mod crc16_ccitt;
#[cfg(CONFIG_CANSERIAL)]
pub mod fasthash;
#[cfg(CONFIG_SERIAL)]
pub mod serial_irq;
#[cfg(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0))]
pub mod timer_irq;
#[cfg(CONFIG_USBSERIAL)]
pub mod usb_cdc;
#[cfg(CONFIG_USBSERIAL)]
pub mod usb_cdc_ep;
#[cfg(CONFIG_USBSERIAL)]
pub mod usbstd;
#[cfg(CONFIG_USBSERIAL)]
pub mod usbstd_cdc;
