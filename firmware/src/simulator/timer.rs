// Example code for running timers in a polling mode
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::config::CONFIG_CLOCK_FREQ;
use crate::ctr::Global;
use crate::generic::timer_irq::{timer_dispatch_many, timer_is_before};
use crate::sched::R;
use crate::decl_init;

// Helper function that returns the system time as a 32bit counter
fn get_system_time() -> u32 {
    let mut ts: libc::timespec = unsafe { core::mem::zeroed() };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    let t = ts.tv_sec as f64 + ts.tv_nsec as f64 * 0.000000001;
    // (C: a double to uint32_t conversion)
    (t * CONFIG_CLOCK_FREQ as f64) as u64 as u32
}

/****************************************************************
 * Timers
 ****************************************************************/

static NEXT_WAKE_TIME: Global<u32> = Global::new(0);

// Return the current time (in absolute clock ticks).
pub fn timer_read_time() -> u32 {
    get_system_time()
}

// Activate timer dispatch as soon as possible
pub fn timer_kick() {
    *NEXT_WAKE_TIME.get() = timer_read_time();
}

// Invoke timers - called below from irq_poll()
fn do_timer_dispatch() -> R {
    *NEXT_WAKE_TIME.get() = timer_dispatch_many()?;
    Ok(())
}

pub fn timer_init() -> R {
    timer_kick();
    Ok(())
}
decl_init!(timer_init);

/****************************************************************
 * Interrupts
 ****************************************************************/

pub type IrqStatus = u32;

// Disable hardware interrupts
pub fn irq_disable() {}

// Enable hardware interrupts
pub fn irq_enable() {}

// Disable hardware interrupts in not already disabled
pub fn irq_save() -> IrqStatus {
    0
}

// Restore hardware interrupts to state from flag returned by irq_save()
pub fn irq_restore(_flag: IrqStatus) {}

// Atomically enable hardware interrupts and sleep processor until next irq
pub fn irq_wait() -> R {
    // XXX - sleep to prevent excessive cpu usage in simulator
    unsafe { libc::usleep(1) };

    irq_poll()
}

// Check if an interrupt is active (used only on architectures that do
// not have hardware interrupts)
pub fn irq_poll() -> R {
    let now = timer_read_time();
    if !timer_is_before(now, *NEXT_WAKE_TIME.get()) {
        do_timer_dispatch()?;
    }
    Ok(())
}
