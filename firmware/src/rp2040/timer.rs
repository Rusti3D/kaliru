// rp2040 timer support
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::{irq_disable, irq_enable};
use crate::generic::armcm_irq::armcm_irq_shutdown;
use crate::generic::timer_irq::timer_dispatch_many;
use crate::sched::R;
use crate::{armcm_enable_irq, decl_init};

/****************************************************************
 * Low level timer code
 ****************************************************************/

fn timer() -> &'static timer_hw_t {
    unsafe { &*timer_hw }
}

// Return the current time (in absolute clock ticks).
#[inline(always)]
pub fn timer_read_time() -> u32 {
    timer().timerawl.read()
}

#[inline(always)]
fn timer_set(next: u32) {
    timer().alarm[0].write(next);
}

// Activate timer dispatch as soon as possible
pub fn timer_kick() {
    timer_set(timer_read_time().wrapping_add(50));
}

/****************************************************************
 * Setup and irqs
 ****************************************************************/

// Hardware timer IRQ handler - dispatch software timers
// (C aligns it to 16 bytes, for stable benchmarks only)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn TIMER0_IRQHandler() {
    irq_disable();
    timer().intr.write(1);
    let next = match timer_dispatch_many() {
        Ok(next) => next,
        Err(s) => armcm_irq_shutdown(s),
    };
    timer_set(next);
    irq_enable();
}

pub fn timer_init() -> R {
    irq_disable();
    enable_pclock(RESETS_RESET_TIMER_BITS);
    timer().timelw.write(0);
    timer().timehw.write(0);
    armcm_enable_irq!(TIMER0_IRQHandler, TIMER_IRQ_0_IRQn, 2);
    timer().inte.write(1);
    timer_kick();
    irq_enable();
    Ok(())
}
decl_init!(timer_init);
