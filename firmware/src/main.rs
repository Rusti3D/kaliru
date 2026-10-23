// Kaliru MCU: Kalico's micro-controller firmware (src/ of Kalico) in Rust.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(clippy::missing_safety_doc, clippy::new_without_default)]

#[allow(non_upper_case_globals, dead_code)]
pub mod config {
    // autoconf.h
    include!(concat!(env!("OUT_DIR"), "/config.rs"));
}
pub mod ctr;

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
#[cfg(CONFIG_WANT_SOFTWARE_I2C)]
pub mod i2c_software;

pub mod generic;
#[cfg(CONFIG_MACH_LINUX)]
pub mod linux;
#[cfg(CONFIG_MACH_LINUX)]
pub use linux as board;

fn main() {
    std::process::exit(board::main::main());
}
