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
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod canbus;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod canbus_h;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod canserial;
pub mod crc16_ccitt;
#[cfg(any(CONFIG_CANSERIAL, CONFIG_USBCANBUS))]
pub mod fasthash;
#[cfg(CONFIG_SERIAL)]
pub mod serial_irq;
#[cfg(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0))]
pub mod timer_irq;
#[cfg(CONFIG_USBSERIAL)]
pub mod usb_cdc;
#[cfg(CONFIG_USBCANBUS)]
pub mod usb_canbus;
// (usb_canbus.c provides the usb_cdc.c functions in that mode)
#[cfg(CONFIG_USBCANBUS)]
pub use usb_canbus as usb_cdc;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub mod usb_cdc_ep;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub mod usbstd;
#[cfg(CONFIG_USBSERIAL)]
pub mod usbstd_cdc;
