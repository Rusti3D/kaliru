// Generic interrupt based timer helper functions
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::{irq_disable, irq_enable, timer_read_time};
use crate::config::CONFIG_CLOCK_FREQ;
use crate::ctr::Global;
use crate::sched::{sched_check_set_tasks_busy, sched_timer_dispatch, R};
use crate::{decl_constant, decl_task, try_shutdown};

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

static TIMER_REPEAT_UNTIL: Global<u32> = Global::new(0);
fn timer_repeat_ticks() -> u32 {
    timer_from_us(100)
}
fn timer_min_try_ticks() -> u32 {
    timer_from_us(2)
}
fn timer_defer_repeat_ticks() -> u32 {
    timer_from_us(5)
}

// Invoke timers - called from board irq code.
pub fn timer_dispatch_many() -> R<u32> {
    let mut tru = *TIMER_REPEAT_UNTIL.get();
    loop {
        // Run the next software timer
        let next = sched_timer_dispatch()?;

        let now = timer_read_time();
        let mut diff = next.wrapping_sub(now) as i32;
        if diff > timer_min_try_ticks() as i32 {
            // Schedule next timer normally.
            return Ok(next);
        }

        if timer_is_before(tru, now) {
            // Check if there are too many repeat timers
            if diff < (timer_from_us(1000) as i32).wrapping_neg() {
                try_shutdown!("Rescheduled timer in the past");
            }
            if sched_check_set_tasks_busy() {
                *TIMER_REPEAT_UNTIL.get() = now.wrapping_add(timer_repeat_ticks());
                return Ok(now.wrapping_add(timer_defer_repeat_ticks()));
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
