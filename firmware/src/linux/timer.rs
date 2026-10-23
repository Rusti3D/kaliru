// Handling of timers on linux systems
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::{console_sleep, report_errno};
use crate::config::CONFIG_CLOCK_FREQ;
use crate::ctr::Global;
use crate::sched::{sched_check_set_tasks_busy, sched_timer_dispatch, R};
use crate::{decl_constant, decl_init, try_shutdown};
use core::ptr::{addr_of_mut, read_volatile, write_volatile};

pub const NSECS: i64 = 1000000000;
pub const NSECS_PER_TICK: i32 = (NSECS / CONFIG_CLOCK_FREQ as i64) as i32;

// Global storage for timer handling
struct TimerInfo {
    // Last time reported by timer_read_time()
    last_read_time: u32,
    // Fields for converting from a systime to ticks
    start_sec: libc::time_t,
    // Flags for tracking irq_enable()/irq_disable()
    must_wake_timers: u32,
    // Time of next software timer (also used to convert from ticks to systime)
    next_wake_counter: u32,
    next_wake: libc::timespec,
    // Unix signal tracking
    t_alarm: libc::timer_t,
    ss_alarm: libc::sigset_t,
    ss_sleep: libc::sigset_t,
}

static TIMER_INFO: Global<TimerInfo> = Global::new(TimerInfo {
    last_read_time: 0,
    start_sec: 0,
    must_wake_timers: 0,
    next_wake_counter: 0,
    next_wake: libc::timespec { tv_sec: 0, tv_nsec: 0 },
    t_alarm: core::ptr::null_mut(),
    ss_alarm: unsafe { core::mem::zeroed() },
    ss_sleep: unsafe { core::mem::zeroed() },
});

fn must_wake_timers() -> u32 {
    unsafe { read_volatile(addr_of_mut!((*TIMER_INFO.ptr()).must_wake_timers)) }
}

fn set_must_wake_timers(v: u32) {
    unsafe { write_volatile(addr_of_mut!((*TIMER_INFO.ptr()).must_wake_timers), v) }
}

/****************************************************************
 * Timespec helpers
 ****************************************************************/

// Convert a 'struct timespec' to a counter value
fn timespec_to_time(ts: libc::timespec) -> u32 {
    let ti = TIMER_INFO.get();
    ((ts.tv_sec - ti.start_sec) * CONFIG_CLOCK_FREQ as libc::time_t + ts.tv_nsec / NSECS_PER_TICK as libc::c_long) as u32
}

// Convert an internal time counter to a 'struct timespec'
fn timespec_from_time(time: u32) -> libc::timespec {
    let ti = TIMER_INFO.get();
    let counter_diff = time.wrapping_sub(ti.next_wake_counter) as i32;
    let mut ts = libc::timespec { tv_sec: ti.next_wake.tv_sec, tv_nsec: 0 };
    // an int multiply, as in C
    ts.tv_nsec = ti.next_wake.tv_nsec + counter_diff.wrapping_mul(NSECS_PER_TICK) as libc::c_long;
    if ts.tv_nsec as libc::c_ulong >= NSECS as libc::c_ulong {
        if ts.tv_nsec < 0 {
            ts.tv_sec -= 1;
            ts.tv_nsec += NSECS as libc::c_long;
        } else {
            ts.tv_sec += 1;
            ts.tv_nsec -= NSECS as libc::c_long;
        }
    }
    ts
}

// Return the current time
fn timespec_read() -> libc::timespec {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts
}

/****************************************************************
 * Timers
 ****************************************************************/

decl_constant!("CLOCK_FREQ", CONFIG_CLOCK_FREQ);

// Check if a given time has past
pub fn timer_check_periodic(ts: &mut u32) -> bool {
    let lrt = TIMER_INFO.get().last_read_time;
    if timer_is_before(lrt, *ts) {
        return false;
    }
    *ts = lrt.wrapping_add(timer_from_us(2000000));
    true
}

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

// Return the current time (in clock ticks)
pub fn timer_read_time() -> u32 {
    let t = timespec_to_time(timespec_read());
    TIMER_INFO.get().last_read_time = t;
    t
}

// Activate timer dispatch as soon as possible
pub fn timer_kick() {
    let it = libc::itimerspec {
        it_interval: libc::timespec { tv_sec: 0, tv_nsec: 0 },
        it_value: libc::timespec { tv_sec: 0, tv_nsec: 1 },
    };
    unsafe { libc::timer_settime(TIMER_INFO.get().t_alarm, libc::TIMER_ABSTIME, &it, core::ptr::null_mut()) };
}

const TIMER_IDLE_REPEAT_COUNT: u32 = 100;
const TIMER_REPEAT_COUNT: u32 = 20;

fn timer_min_try_ticks() -> u32 {
    timer_from_us(2)
}

// Invoke timers
fn timer_dispatch() -> R {
    let mut repeat_count = TIMER_REPEAT_COUNT;
    let mut next;
    loop {
        // Run the next software timer
        next = sched_timer_dispatch()?;

        repeat_count -= 1;
        let lrt = TIMER_INFO.get().last_read_time;
        if !timer_is_before(lrt, next) && repeat_count != 0 {
            // Can run next timer without overhead of calling timer_read_time()
            continue;
        }

        let now = timer_read_time();
        let mut diff = next.wrapping_sub(now) as i32;
        if diff > timer_min_try_ticks() as i32 {
            // Schedule next timer normally.
            break;
        }

        if repeat_count == 0 {
            // Check if there are too many repeat timers
            if diff < (timer_from_us(100000) as i32).wrapping_neg() {
                try_shutdown!("Rescheduled timer in the past");
            }
            if sched_check_set_tasks_busy() {
                return Ok(());
            }
            repeat_count = TIMER_IDLE_REPEAT_COUNT;
        }

        // Next timer in the past or near future - wait for it to be ready
        while diff > 0 {
            diff = next.wrapping_sub(timer_read_time()) as i32;
        }
    }

    // Schedule SIGALRM signal
    let ts = timespec_from_time(next);
    let ti = TIMER_INFO.get();
    let it = libc::itimerspec { it_interval: libc::timespec { tv_sec: 0, tv_nsec: 0 }, it_value: ts };
    ti.next_wake = ts;
    ti.next_wake_counter = next;
    set_must_wake_timers(0);
    unsafe { libc::timer_settime(ti.t_alarm, libc::TIMER_ABSTIME, &it, core::ptr::null_mut()) };
    Ok(())
}

// OS signal handler
extern "C" fn timer_signal(_signal: libc::c_int) {
    set_must_wake_timers(1);
}

pub fn timer_init() -> R {
    unsafe {
        let ti = TIMER_INFO.get();
        // Initialize ss_alarm signal set
        let ret = libc::sigemptyset(&mut ti.ss_alarm);
        if ret < 0 {
            report_errno("sigemptyset", ret);
            return Ok(());
        }
        let ret = libc::sigaddset(&mut ti.ss_alarm, libc::SIGALRM);
        if ret < 0 {
            report_errno("sigaddset", ret);
            return Ok(());
        }
        // Initialize ss_sleep signal set
        let ret = libc::sigprocmask(0, core::ptr::null(), &mut ti.ss_sleep);
        if ret < 0 {
            report_errno("sigprocmask ss_sleep", ret);
            return Ok(());
        }
        let ret = libc::sigdelset(&mut ti.ss_sleep, libc::SIGALRM);
        if ret < 0 {
            report_errno("sigdelset", ret);
            return Ok(());
        }
        // Initialize timespec_to_time() and timespec_from_time()
        let curtime = timespec_read();
        ti.start_sec = curtime.tv_sec + 1;
        ti.next_wake = curtime;
        ti.next_wake_counter = timespec_to_time(curtime);
        // Initialize t_alarm signal based timer
        let ret = libc::timer_create(libc::CLOCK_MONOTONIC, core::ptr::null_mut(), &mut ti.t_alarm);
        if ret < 0 {
            report_errno("timer_create", ret);
            return Ok(());
        }
        let mut act: libc::sigaction = core::mem::zeroed();
        act.sa_sigaction = timer_signal as extern "C" fn(libc::c_int) as libc::sighandler_t;
        act.sa_flags = libc::SA_RESTART;
        let ret = libc::sigaction(libc::SIGALRM, &act, core::ptr::null_mut());
        if ret < 0 {
            report_errno("sigaction", ret);
            return Ok(());
        }
    }
    timer_kick();
    Ok(())
}
decl_init!(timer_init);

// Block SIGALRM signal
pub fn timer_disable_signals() {
    unsafe { libc::sigprocmask(libc::SIG_BLOCK, &TIMER_INFO.get().ss_alarm, core::ptr::null_mut()) };
}

// Restore reception of SIGALRM signal
pub fn timer_enable_signals() {
    unsafe { libc::sigprocmask(libc::SIG_UNBLOCK, &TIMER_INFO.get().ss_alarm, core::ptr::null_mut()) };
}

/****************************************************************
 * Interrupt wrappers
 ****************************************************************/

pub type IrqStatus = libc::c_ulong;

pub fn irq_disable() {}

pub fn irq_enable() {}

pub fn irq_save() -> IrqStatus {
    0
}

pub fn irq_restore(_flag: IrqStatus) {}

pub fn irq_wait() -> R {
    // Must atomically sleep until signaled
    if must_wake_timers() == 0 {
        timer_disable_signals();
        if must_wake_timers() == 0 {
            console_sleep(&TIMER_INFO.get().ss_sleep);
        }
        timer_enable_signals();
    }
    irq_poll()
}

pub fn irq_poll() -> R {
    if must_wake_timers() != 0 {
        timer_dispatch()?;
    }
    Ok(())
}
