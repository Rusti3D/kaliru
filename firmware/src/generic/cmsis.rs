// The CMSIS core functions the firmware uses (lib/cmsis-core core_cm0.h /
// core_cm0plus.h / core_cm3.h / core_cm4.h and cmsis_gcc.h; the M3 / M4 code
// is the same, as is the M0 / M0+ code).
#![allow(non_snake_case)]
use crate::board::internal::{IRQn_Type, NVIC, SCB, SCB_AIRCR_SYSRESETREQ_Msk, SCB_AIRCR_VECTKEY_Pos, __NVIC_PRIO_BITS};
use core::arch::asm;
use core::sync::atomic::{compiler_fence, Ordering};

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
pub fn __DMB() {
    unsafe { asm!("dmb 0xF", options(nostack)) }
}

#[inline(always)]
pub fn __NOP() {
    unsafe { asm!("nop", options(nomem, nostack)) }
}

#[cfg(not(any(has___CORE_CM0_H_GENERIC, has___CORE_CM0PLUS_H_GENERIC)))]
mod nvic {
    use super::*;
    use crate::board::internal::SCB_AIRCR_PRIGROUP_Msk;

    pub fn NVIC_EnableIRQ(irqn: IRQn_Type) {
        if irqn >= 0 {
            let nvic = unsafe { &*NVIC };
            compiler_fence(Ordering::SeqCst);
            nvic.ISER[(irqn as u32 >> 5) as usize].write(1 << (irqn as u32 & 0x1F));
            compiler_fence(Ordering::SeqCst);
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
            // (the Cortex-M33 calls it IPR)
            #[cfg(has___CORE_CM33_H_GENERIC)]
            unsafe { &*NVIC }.IPR[irqn as usize].write(v);
            #[cfg(not(has___CORE_CM33_H_GENERIC))]
            unsafe { &*NVIC }.IP[irqn as usize].write(v);
        } else {
            // (the Cortex-M7 / M33 call it SHPR)
            #[cfg(any(has___CORE_CM7_H_GENERIC, has___CORE_CM33_H_GENERIC))]
            unsafe { &*SCB }.SHPR[((irqn as u32 & 0xF) - 4) as usize].write(v);
            #[cfg(not(any(has___CORE_CM7_H_GENERIC, has___CORE_CM33_H_GENERIC)))]
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
}

// Cortex-M0 / M0+: one 32bit ISER / ICER, priorities packed four to a word
#[cfg(any(has___CORE_CM0_H_GENERIC, has___CORE_CM0PLUS_H_GENERIC))]
mod nvic {
    use super::*;

    fn _BIT_SHIFT(irqn: IRQn_Type) -> u32 {
        ((irqn as u32) & 0x03) * 8
    }
    fn _SHP_IDX(irqn: IRQn_Type) -> usize {
        ((((irqn as u32) & 0x0F).wrapping_sub(8)) >> 2) as usize
    }
    fn _IP_IDX(irqn: IRQn_Type) -> usize {
        ((irqn as u32) >> 2) as usize
    }

    pub fn NVIC_EnableIRQ(irqn: IRQn_Type) {
        if irqn >= 0 {
            compiler_fence(Ordering::SeqCst);
            unsafe { &*NVIC }.ISER[0].write(1 << (irqn as u32 & 0x1F));
            compiler_fence(Ordering::SeqCst);
        }
    }

    pub fn NVIC_DisableIRQ(irqn: IRQn_Type) {
        if irqn >= 0 {
            unsafe { &*NVIC }.ICER[0].write(1 << (irqn as u32 & 0x1F));
            __DSB();
            __ISB();
        }
    }

    pub fn NVIC_SetPriority(irqn: IRQn_Type, priority: u32) {
        let v = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xFF) << _BIT_SHIFT(irqn);
        let mask = !(0xFF << _BIT_SHIFT(irqn));
        if irqn >= 0 {
            let r = &unsafe { &*NVIC }.IP[_IP_IDX(irqn)];
            r.write((r.read() & mask) | v);
        } else {
            let r = &unsafe { &*SCB }.SHP[_SHP_IDX(irqn)];
            r.write((r.read() & mask) | v);
        }
    }

    pub fn NVIC_SystemReset() -> ! {
        // Ensure all outstanding memory accesses included buffered write are
        // completed before reset
        __DSB();
        unsafe { &*SCB }.AIRCR.write((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);
        // Ensure completion of memory access
        __DSB();
        // wait until reset
        loop {
            __NOP();
        }
    }
}
pub use nvic::*;

// D-Cache Clean by address (cachel1_armv7.h, Cortex-M7)
#[cfg(has___CORE_CM7_H_GENERIC)]
pub fn SCB_CleanDCache_by_Addr(addr: *mut u32, dsize: i32) {
    use crate::board::internal::{__DCACHE_PRESENT, __SCB_DCACHE_LINE_SIZE};
    if __DCACHE_PRESENT == 1 && dsize > 0 {
        let mut op_size = dsize + ((addr as u32) & (__SCB_DCACHE_LINE_SIZE - 1)) as i32;
        let mut op_addr = addr as u32;

        __DSB();

        loop {
            unsafe { &*SCB }.DCCMVAC.write(op_addr); // register accepts only 32byte aligned values, only bits 31..5 are valid
            op_addr = op_addr.wrapping_add(__SCB_DCACHE_LINE_SIZE);
            op_size -= __SCB_DCACHE_LINE_SIZE as i32;
            if op_size <= 0 {
                break;
            }
        }

        __DSB();
        __ISB();
    }
}

// Enable I-Cache (cachel1_armv7.h, Cortex-M7)
#[cfg(has___CORE_CM7_H_GENERIC)]
pub fn SCB_EnableICache() {
    use crate::board::internal::{SCB_CCR_IC_Msk, __ICACHE_PRESENT};
    if __ICACHE_PRESENT == 1 {
        let scb = unsafe { &*SCB };
        if scb.CCR.read() & SCB_CCR_IC_Msk != 0 {
            return; // return if ICache is already enabled
        }

        __DSB();
        __ISB();
        scb.ICIALLU.write(0); // invalidate I-Cache
        __DSB();
        __ISB();
        scb.CCR.or(SCB_CCR_IC_Msk); // enable I-Cache
        __DSB();
        __ISB();
    }
}

// Enable D-Cache (cachel1_armv7.h, Cortex-M7)
#[cfg(has___CORE_CM7_H_GENERIC)]
pub fn SCB_EnableDCache() {
    use crate::board::internal::*;
    if __DCACHE_PRESENT == 1 {
        let scb = unsafe { &*SCB };
        if scb.CCR.read() & SCB_CCR_DC_Msk != 0 {
            return; // return if DCache is already enabled
        }

        scb.CSSELR.write(0); // select Level 1 data cache
        __DSB();

        let ccsidr = scb.CCSIDR.read();

        // invalidate D-Cache
        let mut sets = (ccsidr & SCB_CCSIDR_NUMSETS_Msk) >> SCB_CCSIDR_NUMSETS_Pos;
        loop {
            let mut ways = (ccsidr & SCB_CCSIDR_ASSOCIATIVITY_Msk) >> SCB_CCSIDR_ASSOCIATIVITY_Pos;
            loop {
                scb.DCISW.write(((sets << SCB_DCISW_SET_Pos) & SCB_DCISW_SET_Msk) | ((ways << SCB_DCISW_WAY_Pos) & SCB_DCISW_WAY_Msk));
                if ways == 0 {
                    break;
                }
                ways -= 1;
            }
            if sets == 0 {
                break;
            }
            sets -= 1;
        }
        __DSB();

        scb.CCR.or(SCB_CCR_DC_Msk); // enable D-Cache

        __DSB();
        __ISB();
    }
}
