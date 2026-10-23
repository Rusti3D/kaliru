// STM32F0 timer support
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals, non_snake_case)]
use super::internal::*;
use crate::board::{irq_disable, irq_enable, irq_restore, irq_save};
use crate::ctr::Global;
use crate::generic::armcm_irq::armcm_irq_shutdown;
use crate::generic::timer_irq::timer_dispatch_many;
use crate::sched::{sched_add_timer, Timer, R, SF_RESCHEDULE};
use crate::{armcm_enable_irq, decl_init, decl_shutdown};
use core::ptr::{null_mut, read_volatile};

/****************************************************************
 * Low level timer code
 ****************************************************************/

// Use 32bit TIM2 timer if available (otherwise use 16bit TIM3 timer)
#[cfg(has_TIM2)]
mod tim {
    use super::*;
    pub const TIMx: *mut TIM_TypeDef = TIM2;
    pub const TIMx_IRQn: IRQn_Type = TIM2_IRQn;
    pub const HAVE_TIMER_32BIT: bool = true;
}
#[cfg(all(not(has_TIM2), has_TIM3))]
mod tim {
    use super::*;
    pub const TIMx: *mut TIM_TypeDef = TIM3;
    // Some chips have slightly different register names
    #[cfg(CONFIG_MACH_STM32G0B0)]
    pub const TIMx_IRQn: IRQn_Type = TIM3_TIM4_IRQn;
    #[cfg(not(CONFIG_MACH_STM32G0B0))]
    pub const TIMx_IRQn: IRQn_Type = TIM3_IRQn;
    pub const HAVE_TIMER_32BIT: bool = false;
}
use tim::*;

fn tim() -> &'static TIM_TypeDef {
    unsafe { &*TIMx }
}

#[inline(always)]
fn timer_get() -> u32 {
    tim().CNT.read()
}

#[inline(always)]
fn timer_set(next: u32) {
    tim().CCR1.write(next);
    tim().SR.write(0);
}

// Activate timer dispatch as soon as possible
pub fn timer_kick() {
    timer_set(timer_get().wrapping_add(50));
}

/****************************************************************
 * 16bit hardware timer to 32bit conversion
 ****************************************************************/

// High bits of timer (top 17 bits)
static TIMER_HIGH: Global<u32> = Global::new(0);

// Return the current time (in absolute clock ticks).
#[inline(always)]
pub fn timer_read_time() -> u32 {
    if HAVE_TIMER_32BIT {
        return timer_get();
    }
    let th = unsafe { read_volatile(TIMER_HIGH.ptr()) };
    let cur = timer_get();
    // Combine timer_high (high 17 bits) and current time (low 16
    // bits) using method that handles rollovers correctly.
    (th ^ cur).wrapping_add(th & 0xffff)
}

// Update timer_high every 0x8000 clock ticks
unsafe fn timer_event(t: *mut Timer) -> R<u8> {
    let th = TIMER_HIGH.get();
    *th = th.wrapping_add(0x8000);
    (*t).waketime = th.wrapping_add(0x8000);
    Ok(SF_RESCHEDULE)
}
static WRAP_TIMER: Global<Timer> = Global::new(Timer { next: null_mut(), func: Some(timer_event), waketime: 0x8000 });

pub fn timer_reset() -> R {
    if !HAVE_TIMER_32BIT {
        sched_add_timer(WRAP_TIMER.ptr())?;
    }
    Ok(())
}
decl_shutdown!(timer_reset);

/****************************************************************
 * Setup and irqs
 ****************************************************************/

// Hardware timer IRQ handler - dispatch software timers
// (C aligns it to 16 bytes, for stable benchmarks only)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn TIMx_IRQHandler() {
    irq_disable();
    let next = match timer_dispatch_many() {
        Ok(next) => next,
        Err(s) => armcm_irq_shutdown(s),
    };
    timer_set(next);
    irq_enable();
}

pub fn timer_init() -> R {
    let flag = irq_save();
    enable_pclock(TIMx as u32);
    let t = tim();
    t.CNT.write(0);
    t.DIER.write(TIM_DIER_CC1IE);
    t.CCER.write(TIM_CCER_CC1E);
    armcm_enable_irq!(TIMx_IRQHandler, TIMx_IRQn, 2);
    timer_kick();
    timer_reset()?;
    t.CR1.write(TIM_CR1_CEN);
    irq_restore(flag);
    Ok(())
}
decl_init!(timer_init);
