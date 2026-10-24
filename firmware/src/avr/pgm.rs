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

// pgm_read_word()
#[inline(always)]
fn pgm_read_word(addr: *const u8) -> u16 {
    let (lo, hi): (u8, u8);
    unsafe {
        asm!("lpm {0}, Z+", "lpm {1}, Z", out(reg) lo, out(reg) hi, inout("Z") addr => _,
             options(pure, readonly, nostack, preserves_flags))
    };
    u16::from_le_bytes([lo, hi])
}

// pgm_read_dword()
#[inline(always)]
fn pgm_read_dword(addr: *const u8) -> u32 {
    let (b0, b1, b2, b3): (u8, u8, u8, u8);
    unsafe {
        asm!("lpm {0}, Z+", "lpm {1}, Z+", "lpm {2}, Z+", "lpm {3}, Z", out(reg) b0, out(reg) b1, out(reg) b2,
             out(reg) b3, inout("Z") addr => _, options(pure, readonly, nostack, preserves_flags))
    };
    u32::from_le_bytes([b0, b1, b2, b3])
}

// READP(VAR): the variable's bytes from the flash (a 1, 2 or 4 byte value
// as avr-libc's pgm_read_byte / word / dword)
#[inline(always)]
pub fn readp<T: Copy>(v: &T) -> T {
    let src = v as *const T as *const u8;
    unsafe {
        match size_of::<T>() {
            1 => core::mem::transmute_copy(&pgm_read_byte(src)),
            2 => core::mem::transmute_copy(&pgm_read_word(src)),
            4 => core::mem::transmute_copy(&pgm_read_dword(src)),
            _ => {
                let mut out = MaybeUninit::<T>::uninit();
                let dst = out.as_mut_ptr() as *mut u8;
                for i in 0..size_of::<T>() {
                    dst.add(i).write(pgm_read_byte(src.add(i)));
                }
                out.assume_init()
            }
        }
    }
}

// memcpy_P()
#[inline(always)]
pub fn memcpy_p(dst: &mut [u8], src: *const u8) {
    for (i, d) in dst.iter_mut().enumerate() {
        *d = pgm_read_byte(unsafe { src.add(i) });
    }
}
