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

pub mod generic;
#[cfg(CONFIG_MACH_LINUX)]
pub mod linux;
#[cfg(CONFIG_MACH_LINUX)]
pub use linux as board;

fn main() {
    std::process::exit(board::main::main());
}
