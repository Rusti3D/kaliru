// AVR timer interrupt scheduling code.
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::chip::*;
use super::irq::{avr_irq_shutdown, irq_disable, irq_enable, irq_restore, irq_save};
use crate::config::CONFIG_CLOCK_FREQ;
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_set_tasks_busy, sched_timer_dispatch, Timer, R, SF_RESCHEDULE};
use crate::{decl_constant, decl_init, decl_shutdown, try_shutdown};
use core::ptr::null_mut;

/****************************************************************
 * Low level timer code
 ****************************************************************/

decl_constant!("CLOCK_FREQ", CONFIG_CLOCK_FREQ);

// Return the number of clock ticks for a given number of microseconds
pub fn timer_from_us(us: u32) -> u32 {
    us.wrapping_mul(CONFIG_CLOCK_FREQ / 1000000)
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over. (C: hand coded asm of this)
#[inline(always)]
pub fn timer_is_before(time1: u32, time2: u32) -> bool {
    (time1.wrapping_sub(time2) as i32) < 0
}

#[inline(always)]
fn timer_get() -> u16 {
    unsafe { (*TCNT1).read() }
}

#[inline(always)]
fn timer_set(next: u16) {
    unsafe { (*OCR1A).write(next) }
}

#[inline(always)]
fn timer_repeat_set(next: u16) {
    unsafe {
        // Timer1B is used to limit the number of timers run from a timer1A irq
        (*OCR1B).write(next);
        (*TIFR1).write(1 << OCF1B);
    }
}

// Activate timer dispatch as soon as possible
pub fn timer_kick() {
    timer_set(timer_get().wrapping_add(50));
    unsafe { (*TIFR1).write(1 << OCF1A) };
}

// (the timer that runs every ~2ms: allows 16bit comparison optimizations)
static WRAP_TIMER: Global<Timer> = Global::new(Timer { next: null_mut(), func: Some(timer_event), waketime: 0x8000 });

pub fn timer_reset() -> R {
    sched_add_timer(WRAP_TIMER.ptr())
}
decl_shutdown!(timer_reset);

pub fn timer_init() -> R {
    let flag = irq_save();
    unsafe {
        // no outputs
        (*TCCR1A).write(0);
        // Normal Mode
        (*TCCR1B).write(1 << CS10);
        // Setup for first irq
        (*TCNT1).write(0);
    }
    timer_kick();
    timer_repeat_set(timer_get().wrapping_add(50));
    timer_reset()?;
    unsafe {
        (*TIFR1).write(1 << TOV1);
        // enable interrupt
        (*TIMSK1).write(1 << OCIE1A);
    }
    irq_restore(flag);
    Ok(())
}
decl_init!(timer_init);

/****************************************************************
 * 32bit timer wrappers
 ****************************************************************/

static TIMER_HIGH: Global<u16> = Global::new(0);

// Return the current time (in absolute clock ticks).
pub fn timer_read_time() -> u32 {
    let flag = irq_save();
    let lo = timer_get();
    let mut hi = *TIMER_HIGH.get();
    if unsafe { (*TIFR1).read() } & (1 << TOV1) != 0 {
        irq_restore(flag);
        // (the high byte of the 16bit counter)
        if (lo >> 8) < 0xff {
            hi = hi.wrapping_add(1);
        }
        return (hi as u32) << 16 | lo as u32;
    }
    irq_restore(flag);
    (hi as u32) << 16 | lo as u32
}

// Timer that runs every ~2ms - allows 16bit comparison optimizations
unsafe fn timer_event(_t: *mut Timer) -> R<u8> {
    let high = TIMER_HIGH.get();
    let nextwake = &mut (*WRAP_TIMER.ptr()).waketime;
    if (*TIFR1).read() & (1 << TOV1) != 0 {
        // Hardware timer has overflowed - update overflow counter
        (*TIFR1).write(1 << TOV1);
        *high = high.wrapping_add(1);
        *nextwake = (*high as u32) << 16 | 0x8000;
    } else {
        *nextwake = (high.wrapping_add(1) as u32) << 16;
    }
    Ok(SF_RESCHEDULE)
}

const TIMER_REPEAT_TICKS: u16 = 3000;

const TIMER_MIN_ENTRY_TICKS: u16 = 44;
const TIMER_MIN_EXIT_TICKS: u16 = 47;
const TIMER_MIN_TRY_TICKS: u16 = TIMER_MIN_ENTRY_TICKS + TIMER_MIN_EXIT_TICKS;
const TIMER_DEFER_REPEAT_TICKS: u16 = 256;

// (an OCF1B flag: too many timers run from this irq)
fn repeat_due() -> bool {
    (unsafe { (*TIFR1).read() } & (1 << OCF1B)) != 0
}

// Hardware timer IRQ handler - dispatch software timers
fn timer_dispatch() -> R {
    let mut next: u16;
    'done: loop {
        // Run the next software timer
        next = sched_timer_dispatch()? as u16;

        loop {
            let diff = timer_get().wrapping_sub(next) as i16;
            if diff >= 0 {
                // Another timer is pending - briefly allow irqs and then run it
                irq_enable();
                if !repeat_due() {
                    irq_disable();
                    break;
                }
            } else {
                if diff <= -(TIMER_MIN_TRY_TICKS as i16) {
                    // Schedule next timer normally
                    break 'done;
                }
                irq_enable();
                if !repeat_due() {
                    irq_disable();
                    continue;
                }
            }

            // check_defer: Check if there are too many repeat timers
            irq_disable();
            let now = timer_get();
            if (next.wrapping_sub(now) as i16) < timer_from_us(1000).wrapping_neg() as i16 {
                try_shutdown!("Rescheduled timer in the past");
            }
            if sched_check_set_tasks_busy() {
                timer_repeat_set(now.wrapping_add(TIMER_REPEAT_TICKS));
                next = now.wrapping_add(TIMER_DEFER_REPEAT_TICKS);
                break 'done;
            }
            timer_repeat_set(now.wrapping_add(TIMER_REPEAT_TICKS));
            timer_set(now);
        }
    }
    timer_set(next);
    Ok(())
}

#[export_name = avr_vector!(TIMER1_COMPA_vect)]
pub unsafe extern "avr-interrupt" fn timer1_compa_isr() {
    if let Err(s) = timer_dispatch() {
        avr_irq_shutdown(s);
    }
}
