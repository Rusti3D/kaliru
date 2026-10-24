// Kaliru MCU: Kalico's micro-controller firmware (src/ of Kalico) in Rust.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(clippy::missing_safety_doc, clippy::new_without_default)]
// the micro-controller boards: no operating system
#![cfg_attr(not(any(CONFIG_MACH_LINUX, CONFIG_MACH_SIMU)), no_std, no_main)]
// (avr: inline assembly and interrupt handlers are nightly features there)
#![cfg_attr(CONFIG_MACH_AVR, feature(asm_experimental_arch, abi_avr_interrupt))]

#[allow(non_upper_case_globals, dead_code)]
pub mod config {
    // autoconf.h
    include!(concat!(env!("OUT_DIR"), "/config.rs"));
}
pub mod ctr;
// (the volatile register type of the micro-controller boards)
#[cfg(not(any(CONFIG_MACH_LINUX, CONFIG_MACH_SIMU)))]
pub mod reg;

pub mod basecmd;
pub mod command;
pub mod debugcmds;
pub mod sched;
#[cfg(CONFIG_HAVE_GPIO)]
pub mod endstop;
#[cfg(CONFIG_HAVE_GPIO)]
pub mod gpiocmds;
#[cfg(CONFIG_HAVE_GPIO)]
pub mod initial_pins;
#[cfg(CONFIG_HAVE_GPIO)]
pub mod stepper;
#[cfg(CONFIG_HAVE_GPIO)]
pub mod trsync;
#[cfg(CONFIG_WANT_ADC)]
pub mod adccmds;
#[cfg(CONFIG_WANT_SPI)]
pub mod spicmds;
#[cfg(CONFIG_WANT_I2C)]
pub mod i2ccmds;
#[cfg(CONFIG_WANT_HARD_PWM)]
pub mod pwmcmds;
#[cfg(CONFIG_HAVE_GPIO_SDIO)]
pub mod sdiocmds;
#[cfg(CONFIG_WANT_BUTTONS)]
pub mod buttons;
#[cfg(CONFIG_WANT_TMCUART)]
pub mod tmcuart;
#[cfg(CONFIG_WANT_NEOPIXEL)]
pub mod neopixel;
#[cfg(CONFIG_WANT_PULSE_COUNTER)]
pub mod pulse_counter;
#[cfg(CONFIG_WANT_ST7920)]
pub mod lcd_st7920;
#[cfg(CONFIG_WANT_HD44780)]
pub mod lcd_hd44780;
#[cfg(CONFIG_WANT_SOFTWARE_SPI)]
pub mod spi_software;
#[cfg(CONFIG_WANT_THERMOCOUPLE)]
pub mod thermocouple;
#[cfg(CONFIG_WANT_ADXL345)]
pub mod sensor_adxl345;
#[cfg(CONFIG_WANT_LIS2DW)]
pub mod sensor_lis2dw;
#[cfg(CONFIG_WANT_LDC1612)]
pub mod sensor_ldc1612;
#[cfg(CONFIG_WANT_MPU9250)]
pub mod sensor_mpu9250;
#[cfg(CONFIG_WANT_ICM20948)]
pub mod sensor_icm20948;
#[cfg(CONFIG_WANT_HX71X)]
pub mod sensor_hx71x;
#[cfg(CONFIG_WANT_ADS1220)]
pub mod sensor_ads1220;
#[cfg(CONFIG_WANT_ADS131M0X)]
pub mod sensor_ads131m0x;
#[cfg(CONFIG_WANT_CS1237)]
pub mod sensor_cs1237;
#[cfg(CONFIG_WANT_SENSOR_ANGLE)]
pub mod sensor_angle;
#[cfg(CONFIG_NEED_SENSOR_BULK)]
pub mod sensor_bulk;
#[cfg(CONFIG_NEED_SOS_FILTER)]
pub mod sos_filter;
#[cfg(CONFIG_NEED_LOAD_CELL_PROBE)]
pub mod load_cell_probe;
#[cfg(CONFIG_WANT_SOFTWARE_I2C)]
pub mod i2c_software;

#[cfg(CONFIG_WANT_INDX_HEATER)]
pub mod indx;
// (newlib's math functions, which the INDX heater uses)
#[cfg(CONFIG_WANT_INDX_HEATER)]
pub mod libm;

pub mod generic;
#[cfg(CONFIG_MACH_LINUX)]
pub mod linux;
#[cfg(CONFIG_MACH_LINUX)]
pub use linux as board;
#[cfg(CONFIG_MACH_STM32)]
pub mod stm32;
#[cfg(CONFIG_MACH_STM32)]
pub use stm32 as board;
#[cfg(CONFIG_MACH_RPXXXX)]
pub mod rp2040;
#[cfg(CONFIG_MACH_RPXXXX)]
pub use rp2040 as board;
#[cfg(CONFIG_MACH_LPC176X)]
pub mod lpc176x;
#[cfg(CONFIG_MACH_LPC176X)]
pub use lpc176x as board;
#[cfg(CONFIG_MACH_ATSAM)]
pub mod atsam;
#[cfg(CONFIG_MACH_ATSAM)]
pub use atsam as board;
#[cfg(CONFIG_MACH_ATSAMD)]
pub mod atsamd;
#[cfg(CONFIG_MACH_ATSAMD)]
pub use atsamd as board;
#[cfg(CONFIG_MACH_HC32F460)]
pub mod hc32f460;
#[cfg(CONFIG_MACH_HC32F460)]
pub use hc32f460 as board;
#[cfg(CONFIG_MACH_AVR)]
pub mod avr;
#[cfg(CONFIG_MACH_AVR)]
pub use avr as board;
#[cfg(CONFIG_MACH_SIMU)]
pub mod simulator;
#[cfg(CONFIG_MACH_SIMU)]
pub use simulator as board;

#[cfg(any(CONFIG_MACH_LINUX, CONFIG_MACH_SIMU))]
fn main() {
    std::process::exit(board::main::main());
}

// A Rust panic (an index out of bounds, ...; the C code has none of
// these checks): stop with irqs off, so the watchdog resets the chip
#[cfg(not(any(CONFIG_MACH_LINUX, CONFIG_MACH_SIMU)))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    board::irq_disable();
    #[allow(clippy::empty_loop)]
    loop {}
}
