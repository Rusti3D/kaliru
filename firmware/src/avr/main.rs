// Main starting point for AVR boards.
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::chip::{CLKPR, FLASHEND, SP};
use super::irq::{irq_enable, irq_restore, irq_save};
use crate::config::{CONFIG_AVR_CLKPR, CONFIG_AVR_STACK_SIZE};
use crate::sched::{sched_main, R};
use crate::{decl_constant_str, decl_init};
use core::ptr::addr_of_mut;

decl_constant_str!("MCU", CONFIG_MCU);

/****************************************************************
 * Dynamic memory
 ****************************************************************/

extern "C" {
    // (the end of .bss, from the linker)
    static mut _end: u8;
}

// C's .bss holds sched.c's `static jmp_buf shutdown_jmp` (avr-libc: 24
// bytes with a 3 byte pc, a flash over 128KB, else 23), which the Rust
// shutdown path does not need: the dynamic memory starts that much after
// .bss, where C's does, so the move_count reported is C's
const JBLEN: usize = if FLASHEND as u32 > 0x1ffff { 24 } else { 23 };

// Return the start of memory available for dynamic allocations
pub fn dynmem_start() -> *mut u8 {
    unsafe { addr_of_mut!(_end).add(JBLEN) }
}

// avr-libc's memcpy / memmove / memset, as C's build has, not the larger
// ones of compiler_builtins (weak there): libc comes before it in the link
#[link(name = "c")]
extern "C" {}

// Return the end of memory available for dynamic allocations
pub fn dynmem_end() -> *mut u8 {
    // ALIGN(AVR_STACK_POINTER_REG, 256) - CONFIG_AVR_STACK_SIZE
    let sp = unsafe { (*SP).read() } as usize;
    ((sp + 255) & !255).wrapping_sub(CONFIG_AVR_STACK_SIZE as usize) as *mut u8
}

/****************************************************************
 * Misc functions
 ****************************************************************/

// Initialize the clock prescaler (if necessary)
pub fn prescaler_init() -> R {
    // (an int option is a u32 here: -1 is 0xffffffff)
    if CONFIG_AVR_CLKPR as i32 != -1 && CONFIG_AVR_CLKPR as u8 != unsafe { (*CLKPR).read() } {
        let flag = irq_save();
        unsafe {
            (*CLKPR).write(0x80);
            (*CLKPR).write(CONFIG_AVR_CLKPR as u8);
        }
        irq_restore(flag);
    }
    Ok(())
}
decl_init!(prescaler_init);

// avr-libc's _crc_ccitt_update() (util/crc16.h: its C equivalent)
fn crc_ccitt_update(crc: u16, data: u8) -> u16 {
    let mut data = data ^ crc as u8;
    data ^= data << 4;
    (((data as u16) << 8) | (crc >> 8)) ^ (data >> 4) as u16 ^ ((data as u16) << 3)
}

// Optimized crc16_ccitt for the avr processor
pub fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc = 0xFFFF;
    for &b in buf {
        crc = crc_ccitt_update(crc, b);
    }
    crc
}

// Main entry point for avr code.
#[no_mangle]
pub extern "C" fn main() -> ! {
    irq_enable();
    sched_main()
}
