// Basic scheduling functions and startup/shutdown code.
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// sched_shutdown() longjmps to sched_main() in C. Here it returns the
// Shutdown error that every caller passes up with `?` to sched_main(),
// which then runs the shutdown handlers: the same frames are left.
use crate::board::{irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, timer_from_us, timer_is_before, timer_kick, timer_read_time};
use crate::ctr::Global;
use crate::{sendf, shutdown, try_shutdown};
use core::ptr::{addr_of_mut, null_mut};

/// an emergency stop with its static string id (sched_shutdown)
#[derive(Clone, Copy, Debug)]
pub struct Shutdown(pub u8);

pub type R<T = ()> = Result<T, Shutdown>;

pub type TimerFunc = unsafe fn(*mut Timer) -> R<u8>;

// Timer structure for scheduling timed events (see sched_add_timer() )
#[repr(C)]
pub struct Timer {
    pub next: *mut Timer,
    pub func: Option<TimerFunc>,
    pub waketime: u32,
}

impl Timer {
    pub const fn new(func: Option<TimerFunc>) -> Timer {
        Timer { next: null_mut(), func, waketime: 0 }
    }
}

pub const SF_DONE: u8 = 0;
pub const SF_RESCHEDULE: u8 = 1;

// Task waking struct
#[repr(C)]
pub struct TaskWake {
    pub wake: u8,
}

impl TaskWake {
    pub const fn new() -> TaskWake {
        TaskWake { wake: 0 }
    }
}

struct SchedStatus {
    timer_list: *mut Timer,
    last_insert: *mut Timer,
    tasks_status: i8,
    tasks_busy: i8,
    shutdown_status: u8,
    shutdown_reason: u8,
}

static PERIODIC_TIMER: Global<Timer> =
    Global::new(Timer { next: SENTINEL_TIMER.ptr(), func: Some(periodic_event), waketime: 0 });
static SENTINEL_TIMER: Global<Timer> = Global::new(Timer { next: null_mut(), func: Some(sentinel_event), waketime: 0x80000000 });
static DELETED_TIMER: Global<Timer> = Global::new(Timer { next: null_mut(), func: Some(deleted_event), waketime: 0 });

static SCHED_STATUS: Global<SchedStatus> = Global::new(SchedStatus {
    timer_list: PERIODIC_TIMER.ptr(),
    last_insert: PERIODIC_TIMER.ptr(),
    tasks_status: 0,
    tasks_busy: 0,
    shutdown_status: 0,
    shutdown_reason: 0,
});

/****************************************************************
 * Timers
 ****************************************************************/

// The periodic_timer simplifies the timer code by ensuring there is
// always a timer on the timer list and that there is always a timer
// not far in the future.
unsafe fn periodic_event(_t: *mut Timer) -> R<u8> {
    // Make sure the stats task runs periodically
    sched_wake_tasks();
    // Reschedule timer
    let p = PERIODIC_TIMER.get();
    p.waketime = p.waketime.wrapping_add(timer_from_us(100000));
    SENTINEL_TIMER.get().waketime = p.waketime.wrapping_add(0x80000000);
    Ok(SF_RESCHEDULE)
}

// The sentinel timer is always the last timer on timer_list - its
// presence allows the code to avoid checking for NULL while
// traversing timer_list.  Since sentinel_timer.waketime is always
// equal to (periodic_timer.waketime + 0x80000000) any added timer
// must always have a waketime less than one of these two timers.
unsafe fn sentinel_event(_t: *mut Timer) -> R<u8> {
    shutdown!("sentinel timer called");
}

// Find position for a timer in timer_list and insert it
#[inline(always)]
unsafe fn insert_timer(mut pos: *mut Timer, t: *mut Timer, waketime: u32) {
    let mut prev;
    loop {
        prev = pos;
        pos = (*pos).next;
        if timer_is_before(waketime, (*pos).waketime) {
            break;
        }
    }
    (*t).next = pos;
    (*prev).next = t;
}

// Schedule a function call at a supplied time.
pub fn sched_add_timer(add: *mut Timer) -> R {
    unsafe {
        let waketime = (*add).waketime;
        let flag = irq_save();
        let ss = SCHED_STATUS.get();
        let tl = ss.timer_list;
        if timer_is_before(waketime, (*tl).waketime) {
            // This timer is before all other scheduled timers
            if timer_is_before(waketime, timer_read_time()) {
                try_shutdown!("Timer too close");
            }
            let deleted = DELETED_TIMER.ptr();
            if tl == deleted {
                (*add).next = (*deleted).next;
            } else {
                (*add).next = tl;
            }
            (*deleted).waketime = waketime;
            (*deleted).next = add;
            ss.timer_list = deleted;
            timer_kick();
        } else {
            insert_timer(tl, add, waketime);
        }
        irq_restore(flag);
    }
    Ok(())
}

// The deleted timer is used when deleting an active timer.
unsafe fn deleted_event(_t: *mut Timer) -> R<u8> {
    Ok(SF_DONE)
}

// Remove a timer that may be live.
pub fn sched_del_timer(del: *mut Timer) {
    unsafe {
        let flag = irq_save();
        let ss = SCHED_STATUS.get();
        if ss.timer_list == del {
            // Deleting the next active timer - replace with deleted_timer
            let deleted = DELETED_TIMER.ptr();
            (*deleted).waketime = (*del).waketime;
            (*deleted).next = (*del).next;
            ss.timer_list = deleted;
        } else {
            // Find and remove from timer list (if present)
            let mut pos = ss.timer_list;
            while !(*pos).next.is_null() {
                if (*pos).next == del {
                    (*pos).next = (*del).next;
                    break;
                }
                pos = (*pos).next;
            }
        }
        if ss.last_insert == del {
            ss.last_insert = PERIODIC_TIMER.ptr();
        }
        irq_restore(flag);
    }
}

// Invoke the next timer - called from board hardware irq code.
pub fn sched_timer_dispatch() -> R<u32> {
    unsafe {
        // Invoke timer callback
        let ss = SCHED_STATUS.ptr();
        let t = (*ss).timer_list;
        let res = match (*t).func {
            None if crate::config::CONFIG_INLINE_STEPPER_HACK => crate::stepper::stepper_event(t)?,
            None => unreachable!(),
            Some(f) => f(t)?,
        };
        let updated_waketime = (*t).waketime;

        // Update timer_list (rescheduling current timer if necessary)
        let mut next_waketime = updated_waketime;
        if res == SF_DONE {
            next_waketime = (*(*t).next).waketime;
            (*ss).timer_list = (*t).next;
            if (*ss).last_insert == t {
                (*ss).last_insert = (*t).next;
            }
        } else if !timer_is_before(updated_waketime, (*(*t).next).waketime) {
            next_waketime = (*(*t).next).waketime;
            (*ss).timer_list = (*t).next;
            let mut pos = (*ss).last_insert;
            if timer_is_before(updated_waketime, (*pos).waketime) {
                pos = (*ss).timer_list;
            }
            insert_timer(pos, t, updated_waketime);
            (*ss).last_insert = t;
        }

        Ok(next_waketime)
    }
}

// Remove all user timers
pub fn sched_timer_reset() {
    unsafe {
        let ss = SCHED_STATUS.get();
        let deleted = DELETED_TIMER.ptr();
        let periodic = PERIODIC_TIMER.ptr();
        ss.timer_list = deleted;
        (*deleted).waketime = (*periodic).waketime;
        (*deleted).next = periodic;
        ss.last_insert = periodic;
        (*periodic).next = SENTINEL_TIMER.ptr();
    }
    timer_kick();
}

/****************************************************************
 * Tasks
 ****************************************************************/

const TS_IDLE: i8 = -1;
const TS_REQUESTED: i8 = 0;
const TS_RUNNING: i8 = 1;

// Note that at least one task is ready to run
pub fn sched_wake_tasks() {
    unsafe { core::ptr::write_volatile(addr_of_mut!((*SCHED_STATUS.ptr()).tasks_status), TS_REQUESTED) };
}

fn tasks_status() -> i8 {
    unsafe { core::ptr::read_volatile(addr_of_mut!((*SCHED_STATUS.ptr()).tasks_status)) }
}

// Check if tasks busy (called from low-level timer dispatch code)
pub fn sched_check_set_tasks_busy() -> bool {
    let ss = SCHED_STATUS.get();
    // Return busy if tasks never idle between two consecutive calls
    if ss.tasks_busy >= TS_REQUESTED {
        return true;
    }
    ss.tasks_busy = ss.tasks_status;
    false
}

// Note that a task is ready to run
pub fn sched_wake_task(w: *mut TaskWake) {
    sched_wake_tasks();
    unsafe { core::ptr::write_volatile(addr_of_mut!((*w).wake), 1) };
}

// Check if a task is ready to run (as indicated by sched_wake_task)
pub fn sched_check_wake(w: *mut TaskWake) -> bool {
    unsafe {
        if core::ptr::read_volatile(addr_of_mut!((*w).wake)) == 0 {
            return false;
        }
        core::ptr::write_volatile(addr_of_mut!((*w).wake), 0);
    }
    true
}

// Main task dispatch loop
fn run_tasks() -> R {
    let mut start = timer_read_time();
    loop {
        // Check if can sleep
        irq_poll()?;
        if tasks_status() != TS_REQUESTED {
            start = start.wrapping_sub(timer_read_time());
            irq_disable();
            if tasks_status() != TS_REQUESTED {
                // Sleep processor (only run timers) until tasks woken
                let ss = SCHED_STATUS.get();
                ss.tasks_status = TS_IDLE;
                ss.tasks_busy = TS_IDLE;
                loop {
                    irq_wait()?;
                    if tasks_status() == TS_REQUESTED {
                        break;
                    }
                }
            }
            irq_enable();
            start = start.wrapping_add(timer_read_time());
        }
        SCHED_STATUS.get().tasks_status = TS_RUNNING;

        // Run all tasks
        crate::ctr::ctr_run_taskfuncs()?;

        // Update statistics
        let cur = timer_read_time();
        crate::basecmd::stats_update(start, cur)?;
        start = cur;
    }
}

/****************************************************************
 * Shutdown processing
 ****************************************************************/

// Return true if the machine is in an emergency stop state
pub fn sched_is_shutdown() -> bool {
    SCHED_STATUS.get().shutdown_status != 0
}

// Transition out of shutdown state
pub fn sched_clear_shutdown() -> R {
    let ss = SCHED_STATUS.get();
    if ss.shutdown_status == 0 {
        shutdown!("Shutdown cleared when not shutdown");
    }
    if ss.shutdown_status == 2 {
        // Ignore attempt to clear shutdown if still processing shutdown
        return Ok(());
    }
    ss.shutdown_status = 0;
    Ok(())
}

// Invoke all shutdown functions (as declared by DECL_SHUTDOWN)
fn run_shutdown(reason: u8) -> R {
    irq_disable();
    let cur = timer_read_time();
    let ss = SCHED_STATUS.get();
    if ss.shutdown_status == 0 {
        ss.shutdown_reason = reason;
    }
    ss.shutdown_status = 2;
    sched_timer_reset();
    // a shutdown in a shutdown handler runs them all again (C longjmps
    // back into run_shutdown)
    crate::ctr::ctr_run_shutdownfuncs()?;
    SCHED_STATUS.get().shutdown_status = 1;
    irq_enable();

    sendf!("shutdown clock=%u static_string_id=%hu", cur, SCHED_STATUS.get().shutdown_reason as u16)
}

// Report the last shutdown reason code
pub fn sched_report_shutdown() -> R {
    sendf!("is_shutdown static_string_id=%hu", SCHED_STATUS.get().shutdown_reason as u16)
}

// Shutdown the machine if not already in the process of shutting down
#[inline(always)]
pub fn sched_try_shutdown(reason: u8) -> R {
    if SCHED_STATUS.get().shutdown_status == 0 {
        return Err(sched_shutdown(reason));
    }
    Ok(())
}

// Force the machine to immediately run the shutdown handlers
pub fn sched_shutdown(reason: u8) -> Shutdown {
    irq_disable();
    Shutdown(reason)
}

/****************************************************************
 * Startup
 ****************************************************************/

// Main loop of program
pub fn sched_main() -> ! {
    // an init function's shutdown: C longjmps before setjmp; report it
    // like any other
    let res = crate::ctr::ctr_run_initfuncs().and_then(|_| sendf!("starting"));

    irq_disable();
    sched_main_loop(res)
}

// The loop of sched_main() after its setjmp: `res` is a Shutdown when
// entered from a longjmp (a shutdown inside an irq handler on boards with
// irqs, see generic/armcm_irq.rs)
pub fn sched_main_loop(mut res: R) -> ! {
    loop {
        if let Err(Shutdown(reason)) = res {
            // setjmp returned from a longjmp: run the shutdown, then
            // the tasks again
            res = run_shutdown(reason);
            if res.is_err() {
                continue;
            }
        }
        irq_enable();
        res = run_tasks();
    }
}
