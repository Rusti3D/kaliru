// The CMSIS core functions the firmware uses (lib/cmsis-core core_cm3.h /
// core_cm4.h and cmsis_gcc.h; the same code in both cores).
#![allow(non_snake_case)]
use crate::board::internal::{
    IRQn_Type, NVIC, SCB, SCB_AIRCR_PRIGROUP_Msk, SCB_AIRCR_SYSRESETREQ_Msk, SCB_AIRCR_VECTKEY_Pos, __NVIC_PRIO_BITS,
};
use core::arch::asm;

#[inline(always)]
pub fn __enable_irq() {
    unsafe { asm!("cpsie i", options(nostack)) }
}

#[inline(always)]
pub fn __disable_irq() {
    unsafe { asm!("cpsid i", options(nostack)) }
}

#[inline(always)]
pub fn __DSB() {
    unsafe { asm!("dsb 0xF", options(nostack)) }
}

#[inline(always)]
pub fn __ISB() {
    unsafe { asm!("isb 0xF", options(nostack)) }
}

#[inline(always)]
pub fn __NOP() {
    unsafe { asm!("nop", options(nomem, nostack)) }
}

pub fn NVIC_EnableIRQ(irqn: IRQn_Type) {
    if irqn >= 0 {
        let nvic = unsafe { &*NVIC };
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        nvic.ISER[(irqn as u32 >> 5) as usize].write(1 << (irqn as u32 & 0x1F));
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

pub fn NVIC_DisableIRQ(irqn: IRQn_Type) {
    if irqn >= 0 {
        let nvic = unsafe { &*NVIC };
        nvic.ICER[(irqn as u32 >> 5) as usize].write(1 << (irqn as u32 & 0x1F));
        __DSB();
        __ISB();
    }
}

pub fn NVIC_SetPriority(irqn: IRQn_Type, priority: u32) {
    let v = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xFF) as u8;
    if irqn >= 0 {
        unsafe { &*NVIC }.IP[irqn as usize].write(v);
    } else {
        unsafe { &*SCB }.SHP[((irqn as u32 & 0xF) - 4) as usize].write(v);
    }
}

pub fn NVIC_SystemReset() -> ! {
    // Ensure all outstanding memory accesses included buffered write are
    // completed before reset
    __DSB();
    let scb = unsafe { &*SCB };
    // Keep priority group unchanged
    scb.AIRCR.write((0x5FA << SCB_AIRCR_VECTKEY_Pos) | (scb.AIRCR.read() & SCB_AIRCR_PRIGROUP_Msk) | SCB_AIRCR_SYSRESETREQ_Msk);
    // Ensure completion of memory access
    __DSB();
    // wait until reset
    loop {
        __NOP();
    }
}
