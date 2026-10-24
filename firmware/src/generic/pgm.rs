// Definitions for boards with one address space (generic/pgm.h): data
// "in program memory" (PROGMEM, `#[cfg_attr(CONFIG_MACH_AVR, link_section
// = ".progmem.data")]` here) is read like any other.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

pub const NEED_PROGMEM: bool = false;

// READP(VAR)
#[inline(always)]
pub fn readp<T: Copy>(v: &T) -> T {
    *v
}
