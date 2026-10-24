// Definitions for accessing data in program memory on AVR (avr/pgm.h).
// PROGMEM data (`#[cfg_attr(CONFIG_MACH_AVR, link_section =
// ".progmem.data")]`) is in the flash, not the ram: it is read with lpm
// (avr-libc's pgm_read_byte), byte by byte, never with an ordinary load.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use core::arch::asm;
use core::mem::{size_of, MaybeUninit};

pub const NEED_PROGMEM: bool = true;

// pgm_read_byte()
#[inline(always)]
pub fn pgm_read_byte(addr: *const u8) -> u8 {
    let v: u8;
    unsafe { asm!("lpm {0}, Z", out(reg) v, in("Z") addr, options(pure, readonly, nostack, preserves_flags)) };
    v
}

// READP(VAR): the variable's bytes from the flash
#[inline(always)]
pub fn readp<T: Copy>(v: &T) -> T {
    let mut out = MaybeUninit::<T>::uninit();
    let src = v as *const T as *const u8;
    let dst = out.as_mut_ptr() as *mut u8;
    for i in 0..size_of::<T>() {
        unsafe { dst.add(i).write(pgm_read_byte(src.add(i))) };
    }
    unsafe { out.assume_init() }
}

// memcpy_P()
#[inline(always)]
pub fn memcpy_p(dst: &mut [u8], src: *const u8) {
    for (i, d) in dst.iter_mut().enumerate() {
        *d = pgm_read_byte(unsafe { src.add(i) });
    }
}
