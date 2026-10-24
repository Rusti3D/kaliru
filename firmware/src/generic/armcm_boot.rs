// ARM Cortex-M vector table and initial bootup handling
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::internal::{
    NVIC, SCB, SCB_ICSR_PENDSTCLR_Msk, SCB_ICSR_PENDSVCLR_Msk, SysTick, SysTick_CTRL_CLKSOURCE_Msk,
};
use crate::generic::cmsis::{__DSB, __ISB, __enable_irq};
use crate::{decl_armcm_irq, decl_constant_str};
use core::ptr::addr_of_mut;

// Export MCU type
decl_constant_str!("MCU", CONFIG_MCU);

// Symbols created by armcm_link.lds.S linker script
extern "C" {
    static mut _data_start: u32;
    static mut _data_end: u32;
    static mut _data_flash: u32;
    static mut _bss_start: u32;
    static mut _bss_end: u32;
    static mut _stack_start: u32;
    pub static mut _stack_end: u32;
}

/****************************************************************
 * Basic interrupt handlers
 ****************************************************************/

// Inlined version of memset (to avoid function calls during intial boot code)
#[inline(always)]
unsafe fn boot_memset(s: *mut u32, c: u32, mut n: usize) {
    let mut p = s;
    while n != 0 {
        core::ptr::write_volatile(p, c);
        p = p.add(1);
        n -= 4;
    }
}

// Inlined version of memcpy (to avoid function calls during intial boot code)
#[inline(always)]
unsafe fn boot_memcpy(dest: *mut u32, src: *const u32, mut n: usize) {
    let (mut d, mut s) = (dest, src);
    while n != 0 {
        core::ptr::write_volatile(d, *s);
        d = d.add(1);
        s = s.add(1);
        n -= 4;
    }
}

// Main initialization code (called from ResetHandler below)
#[unsafe(link_section = ".text.armcm_boot.stage_two")]
unsafe extern "C" fn reset_handler_stage_two() -> ! {
    let (nvic, scb, systick) = (&*NVIC, &*SCB, &*SysTick);

    // Clear all enabled user interrupts and user pending interrupts
    for i in 0..nvic.ICER.len() {
        nvic.ICER[i].write(0xFFFFFFFF);
        __DSB();
        nvic.ICPR[i].write(0xFFFFFFFF);
    }

    // Reset all user interrupt priorities
    #[cfg(has___CORE_CM33_H_GENERIC)]
    for ip in nvic.IPR.iter() {
        ip.write(0);
    }
    #[cfg(not(has___CORE_CM33_H_GENERIC))]
    for ip in nvic.IP.iter() {
        ip.write(0);
    }

    // Disable SysTick interrupt
    systick.CTRL.write(SysTick_CTRL_CLKSOURCE_Msk);
    __DSB();

    // Clear pending pendsv and systick interrupts
    scb.ICSR.write(SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk);

    // Reset all system interrupt priorities
    #[cfg(any(has___CORE_CM7_H_GENERIC, has___CORE_CM33_H_GENERIC))]
    for shp in scb.SHPR.iter() {
        shp.write(0);
    }
    #[cfg(not(any(has___CORE_CM7_H_GENERIC, has___CORE_CM33_H_GENERIC)))]
    for shp in scb.SHP.iter() {
        shp.write(0);
    }

    __DSB();
    __ISB();
    __enable_irq();

    // Copy global variables from flash to ram
    let count = (addr_of_mut!(_data_end) as usize - addr_of_mut!(_data_start) as usize) / 4 * 4;
    boot_memcpy(addr_of_mut!(_data_start), addr_of_mut!(_data_flash), count);

    // Clear the bss segment
    boot_memset(addr_of_mut!(_bss_start), 0, (addr_of_mut!(_bss_end) as usize - addr_of_mut!(_bss_start) as usize) / 4 * 4);

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    // Initializing the C library isn't needed...

    // Run the main board specific code
    crate::board::armcm_main();
}

// Initial code entry point - invoked by the processor after a reset
// Reset interrupts and stack to take control from bootloaders
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".text.armcm_boot.stage_one")]
pub unsafe extern "C" fn ResetHandler() {
    // __disable_irq(), then explicitly load the stack pointer, jump to stage two
    core::arch::naked_asm!(
        "cpsid i",
        "ldr r0, ={stack}",
        "ldr r1, ={stage_two}",
        "mov sp, r0",
        "bx r1",
        ".ltorg",
        stack = sym _stack_end,
        stage_two = sym reset_handler_stage_two,
    )
}
decl_armcm_irq!(ResetHandler, -15);

// Code called for any undefined interrupts
#[unsafe(no_mangle)]
pub unsafe extern "C" fn DefaultHandler() {
    #[allow(clippy::empty_loop)]
    loop {}
}

/****************************************************************
 * Dynamic memory range
 ****************************************************************/

// Return the start of memory available for dynamic allocations
pub fn dynmem_start() -> *mut u8 {
    addr_of_mut!(_bss_end) as *mut u8
}

// Return the end of memory available for dynamic allocations
pub fn dynmem_end() -> *mut u8 {
    addr_of_mut!(_stack_start) as *mut u8
}

// The NVIC priority / enable of armcm_enable_irq!() (armcm_boot.h)
pub use crate::generic::cmsis::{NVIC_EnableIRQ, NVIC_SetPriority};
