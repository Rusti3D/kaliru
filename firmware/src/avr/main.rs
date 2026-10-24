// Main starting point for AVR boards.
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::chip::{CLKPR, SP};
use super::irq::{irq_enable, irq_restore, irq_save};
use crate::config::{CONFIG_AVR_CLKPR, CONFIG_AVR_STACK_SIZE, CONFIG_MCU};
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

// Return the start of memory available for dynamic allocations
pub fn dynmem_start() -> *mut u8 {
    unsafe { addr_of_mut!(_end) }
}

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
