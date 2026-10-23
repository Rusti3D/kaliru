// Timer based on ARM Cortex-M3/M4 SysTick and DWT logic
//
// Copyright (C) 2017-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::internal::{
    CoreDebug, CoreDebug_DEMCR_TRCENA_Msk, DWT_CTRL_CYCCNTENA_Msk, SCB_ICSR_PENDSTSET_Msk, SysTick_CTRL_CLKSOURCE_Msk,
    SysTick_CTRL_ENABLE_Msk, SysTick_CTRL_TICKINT_Msk, SysTick_IRQn, DWT, SCB,
};
use crate::config::CONFIG_CLOCK_FREQ;
use crate::ctr::Global;
use crate::generic::armcm_irq::{armcm_irq_shutdown, irq_disable, irq_enable, irq_restore, irq_save};
use crate::generic::cmsis::NVIC_SetPriority;
use crate::sched::{sched_add_timer, sched_check_set_tasks_busy, sched_timer_dispatch, Timer, R, SF_RESCHEDULE};
use crate::{decl_armcm_irq, decl_constant, decl_init, decl_shutdown, decl_task, try_shutdown};
use core::ptr::null_mut;

decl_constant!("CLOCK_FREQ", CONFIG_CLOCK_FREQ);

// Return the number of clock ticks for a given number of microseconds
pub fn timer_from_us(us: u32) -> u32 {
    us.wrapping_mul(CONFIG_CLOCK_FREQ / 1000000)
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over.
pub fn timer_is_before(time1: u32, time2: u32) -> bool {
    (time1.wrapping_sub(time2) as i32) < 0
}

// Set the next irq time
fn timer_set_diff(value: u32) {
    let systick = unsafe { &*crate::board::internal::SysTick };
    systick.LOAD.write(value);
    systick.VAL.write(0);
    systick.LOAD.write(0);
}

// Return the current time (in absolute clock ticks).
pub fn timer_read_time() -> u32 {
    unsafe { &*DWT }.CYCCNT.read()
}

// Activate timer dispatch as soon as possible
pub fn timer_kick() {
    let systick = unsafe { &*crate::board::internal::SysTick };
    systick.LOAD.write(0);
    systick.VAL.write(0);
    unsafe { &*SCB }.ICSR.write(SCB_ICSR_PENDSTSET_Msk);
}

// Implement simple early-boot delay mechanism
pub fn udelay(usecs: u32) {
    let (coredebug, dwt) = unsafe { (&*CoreDebug, &*DWT) };
    if coredebug.DEMCR.read() & CoreDebug_DEMCR_TRCENA_Msk == 0 {
        coredebug.DEMCR.or(CoreDebug_DEMCR_TRCENA_Msk);
        dwt.CTRL.or(DWT_CTRL_CYCCNTENA_Msk);
    }

    let end = timer_read_time().wrapping_add(timer_from_us(usecs));
    while timer_is_before(timer_read_time(), end) {}
}

// Dummy timer to avoid scheduling a SysTick irq greater than 0xffffff
unsafe fn timer_wrap_event(t: *mut Timer) -> R<u8> {
    (*t).waketime = (*t).waketime.wrapping_add(0xffffff);
    Ok(SF_RESCHEDULE)
}
static WRAP_TIMER: Global<Timer> = Global::new(Timer { next: null_mut(), func: Some(timer_wrap_event), waketime: 0xffffff });

pub fn timer_reset() -> R {
    if timer_from_us(100000) <= 0xffffff {
        // Timer in sched.c already ensures SysTick wont overflow
        return Ok(());
    }
    sched_add_timer(WRAP_TIMER.ptr())
}
decl_shutdown!(timer_reset);

pub fn timer_init() -> R {
    let (coredebug, dwt) = unsafe { (&*CoreDebug, &*DWT) };
    // Enable Debug Watchpoint and Trace (DWT) for its 32bit timer
    coredebug.DEMCR.or(CoreDebug_DEMCR_TRCENA_Msk);
    dwt.CTRL.or(DWT_CTRL_CYCCNTENA_Msk);
    dwt.CYCCNT.write(0);

    // Schedule a recurring timer on fast cpus
    timer_reset()?;

    // Enable SysTick
    let flag = irq_save();
    NVIC_SetPriority(SysTick_IRQn, 2);
    unsafe { &*crate::board::internal::SysTick }
        .CTRL
        .write(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
    timer_kick();
    irq_restore(flag);
    Ok(())
}
decl_init!(timer_init);

static TIMER_REPEAT_UNTIL: Global<u32> = Global::new(0);
fn timer_repeat_ticks() -> u32 {
    timer_from_us(100)
}

const TIMER_MIN_TRY_TICKS: i32 = 90;
fn timer_defer_repeat_ticks() -> u32 {
    timer_from_us(5)
}

// Invoke timers
fn timer_dispatch_many() -> R<u32> {
    let mut tru = *TIMER_REPEAT_UNTIL.get();
    loop {
        // Run the next software timer
        let next = sched_timer_dispatch()?;

        let now = timer_read_time();
        let mut diff = next.wrapping_sub(now) as i32;
        if diff > TIMER_MIN_TRY_TICKS {
            // Schedule next timer normally.
            return Ok(diff as u32);
        }

        if timer_is_before(tru, now) {
            // Check if there are too many repeat timers
            if diff < (timer_from_us(1000) as i32).wrapping_neg() {
                try_shutdown!("Rescheduled timer in the past");
            }
            if sched_check_set_tasks_busy() {
                *TIMER_REPEAT_UNTIL.get() = now.wrapping_add(timer_repeat_ticks());
                return Ok(timer_defer_repeat_ticks());
            }
            tru = now.wrapping_add(timer_repeat_ticks());
            *TIMER_REPEAT_UNTIL.get() = tru;
        }

        // Next timer in the past or near future - wait for it to be ready
        irq_enable();
        while diff > 0 {
            diff = next.wrapping_sub(timer_read_time()) as i32;
        }
        irq_disable();
    }
}

// IRQ handler (C aligns it to 16 bytes, for stable benchmarks only)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn SysTick_Handler() {
    irq_disable();
    let diff = match timer_dispatch_many() {
        Ok(diff) => diff,
        Err(s) => armcm_irq_shutdown(s),
    };
    timer_set_diff(diff);
    irq_enable();
}
decl_armcm_irq!(SysTick_Handler, SysTick_IRQn);

// Make sure timer_repeat_until doesn't wrap 32bit comparisons
pub fn timer_task() -> R {
    let now = timer_read_time();
    irq_disable();
    let tru = TIMER_REPEAT_UNTIL.get();
    if timer_is_before(*tru, now) {
        *tru = now;
    }
    irq_enable();
    Ok(())
}
decl_task!(timer_task);
