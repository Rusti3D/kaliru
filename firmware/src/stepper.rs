// Handling of stepper drivers.
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{
    foreach_oid, move_alloc, move_free, move_queue_clear, move_queue_empty, move_queue_pop, move_queue_push, move_queue_setup,
    oid_alloc, oid_lookup, MoveNode, MoveQueueHead, OidType,
};
use crate::board::{gpio_out_setup, gpio_out_toggle_noirq, gpio_out_write, irq_disable, irq_enable, timer_from_us, timer_is_before, timer_read_time, GpioOut};
use crate::config::{CONFIG_CLOCK_FREQ, CONFIG_INLINE_STEPPER_HACK};
use crate::sched::{sched_add_timer, sched_del_timer, Timer, R, SF_DONE, SF_RESCHEDULE};
use crate::trsync::{trsync_add_signal, trsync_oid_lookup, TrsyncSignal};
use crate::{decl_command, decl_constant, decl_shutdown, sendf, shutdown};
use core::mem::{offset_of, size_of};

decl_constant!("STEPPER_STEP_BOTH_EDGE", 1);

const HAVE_OPTIMIZED_PATH: bool =
    cfg!(any(all(CONFIG_INLINE_STEPPER_HACK, CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE), all(CONFIG_INLINE_STEPPER_HACK, CONFIG_MACH_AVR)));
const HAVE_EDGE_OPTIMIZATION: bool = cfg!(all(CONFIG_INLINE_STEPPER_HACK, CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE));
const HAVE_AVR_OPTIMIZATION: bool =
    cfg!(all(CONFIG_INLINE_STEPPER_HACK, CONFIG_MACH_AVR, not(CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE)));

#[repr(C)]
struct StepperMove {
    node: MoveNode,
    interval: u32,
    add: i16,
    count: u16,
    flags: u8,
}

const MF_DIR: u8 = 1 << 0;

#[repr(C)]
struct Stepper {
    time: Timer,
    interval: u32,
    add: i16,
    count: u32,
    next_step_time: u32,
    step_pulse_ticks: u32,
    step_pin: GpioOut,
    dir_pin: GpioOut,
    position: u32,
    mq: MoveQueueHead,
    stop_signal: TrsyncSignal,
    flags: u8,
}

const POSITION_BIAS: u32 = 0x40000000;

const SF_LAST_DIR: u8 = 1 << 0;
const SF_NEXT_DIR: u8 = 1 << 1;
const SF_INVERT_STEP: u8 = 1 << 2;
const SF_NEED_RESET: u8 = 1 << 3;
const SF_SINGLE_SCHED: u8 = 1 << 4;
const SF_OPTIMIZED_PATH: u8 = 1 << 5;
const SF_HAVE_ADD: u8 = 1 << 6;

fn oid_type() -> OidType {
    command_config_stepper as *const ()
}

// Setup a stepper for the next move in its queue
unsafe fn stepper_load_next(s: &mut Stepper) -> R<u8> {
    if move_queue_empty(&s.mq) {
        // There is no next move - the queue is empty
        s.count = 0;
        return Ok(SF_DONE);
    }

    // Read next 'struct stepper_move'
    let mn = move_queue_pop(&mut s.mq);
    let m = mn as *mut StepperMove;
    let move_interval = (*m).interval;
    let move_count = (*m).count as u32;
    let move_add = (*m).add;
    let need_dir_change = (*m).flags & MF_DIR != 0;
    move_free(m as *mut u8);

    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    s.position = (if need_dir_change { s.position.wrapping_neg() } else { s.position }).wrapping_add(move_count);

    // Load next move into 'struct stepper'
    s.add = move_add;
    s.interval = move_interval.wrapping_add(move_add as i32 as u32);
    if HAVE_OPTIMIZED_PATH && s.flags & SF_OPTIMIZED_PATH != 0 {
        // Using optimized stepper_event_edge() or stepper_event_avr()
        s.time.waketime = s.time.waketime.wrapping_add(move_interval);
        if HAVE_AVR_OPTIMIZATION {
            s.flags = if move_add != 0 { s.flags | SF_HAVE_ADD } else { s.flags & !SF_HAVE_ADD };
        }
        s.count = move_count;
    } else {
        // Using fully scheduled stepper_event_full() code (the scheduler
        // may be called twice for each step)
        let was_active = s.count != 0;
        let mut min_next_time = s.time.waketime;
        s.next_step_time = s.next_step_time.wrapping_add(move_interval);
        s.time.waketime = s.next_step_time;
        s.count = if s.flags & SF_SINGLE_SCHED != 0 { move_count } else { move_count * 2 };
        if was_active && timer_is_before(s.next_step_time, min_next_time) {
            // Actively stepping and next step event close to the last unstep
            let diff = s.next_step_time.wrapping_sub(min_next_time) as i32;
            if diff < timer_from_us(1000).wrapping_neg() as i32 {
                shutdown!("Stepper too far in past");
            }
            s.time.waketime = min_next_time;
        }
        if was_active && need_dir_change {
            // Must ensure minimum time between step change and dir change
            if s.flags & SF_SINGLE_SCHED != 0 {
                while timer_is_before(timer_read_time(), min_next_time) {}
            }
            gpio_out_toggle_noirq(s.dir_pin);
            let curtime = timer_read_time();
            min_next_time = curtime.wrapping_add(s.step_pulse_ticks);
            if timer_is_before(s.time.waketime, min_next_time) {
                s.time.waketime = min_next_time;
            }
            return Ok(SF_RESCHEDULE);
        }
    }

    // Set new direction (if needed)
    if need_dir_change {
        gpio_out_toggle_noirq(s.dir_pin);
    }
    Ok(SF_RESCHEDULE)
}

// Edge optimization only enabled when fastest rate notably slower than 100ns
const EDGE_STEP_TICKS: u32 = CONFIG_CLOCK_FREQ.div_ceil(8000000);
#[cfg(all(CONFIG_INLINE_STEPPER_HACK, CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE))]
decl_constant!("STEPPER_OPTIMIZED_EDGE", EDGE_STEP_TICKS);

// Optimized step function to step on each step pin edge
unsafe fn stepper_event_edge(t: *mut Timer) -> R<u8> {
    let s = &mut *(t as *mut Stepper);
    gpio_out_toggle_noirq(s.step_pin);
    let count = s.count.wrapping_sub(1);
    if count != 0 {
        s.count = count;
        s.time.waketime = s.time.waketime.wrapping_add(s.interval);
        s.interval = s.interval.wrapping_add(s.add as i32 as u32);
        return Ok(SF_RESCHEDULE);
    }
    stepper_load_next(s)
}

const AVR_STEP_TICKS: u32 = 40; // minimum instructions between step gpio pulses
#[cfg(all(CONFIG_INLINE_STEPPER_HACK, CONFIG_MACH_AVR, not(CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE)))]
decl_constant!("STEPPER_OPTIMIZED_UNSTEP", AVR_STEP_TICKS);

// AVR optimized step function
unsafe fn stepper_event_avr(t: *mut Timer) -> R<u8> {
    let s = &mut *(t as *mut Stepper);
    gpio_out_toggle_noirq(s.step_pin);
    // the low 16 bits of s->count (little endian)
    let pcount = &mut s.count as *mut u32 as *mut u16;
    let count = (*pcount).wrapping_sub(1);
    if count != 0 {
        *pcount = count;
        s.time.waketime = s.time.waketime.wrapping_add(s.interval);
        gpio_out_toggle_noirq(s.step_pin);
        if s.flags & SF_HAVE_ADD != 0 {
            s.interval = s.interval.wrapping_add(s.add as i32 as u32);
        }
        return Ok(SF_RESCHEDULE);
    }
    let ret = stepper_load_next(s)?;
    gpio_out_toggle_noirq(s.step_pin);
    Ok(ret)
}

// Regular "fully scheduled" step function
unsafe fn stepper_event_full(t: *mut Timer) -> R<u8> {
    let s = &mut *(t as *mut Stepper);
    gpio_out_toggle_noirq(s.step_pin);
    let curtime = timer_read_time();
    let min_next_time = curtime.wrapping_add(s.step_pulse_ticks);
    let count = s.count.wrapping_sub(1);
    'reschedule_min: {
        if count & 1 != 0 && s.flags & SF_SINGLE_SCHED == 0 {
            // Schedule unstep event
            break 'reschedule_min;
        }
        if count != 0 {
            s.next_step_time = s.next_step_time.wrapping_add(s.interval);
            s.interval = s.interval.wrapping_add(s.add as i32 as u32);
            if timer_is_before(s.next_step_time, min_next_time) {
                // The next step event is too close - push it back
                break 'reschedule_min;
            }
            s.count = count;
            s.time.waketime = s.next_step_time;
            return Ok(SF_RESCHEDULE);
        }
        s.time.waketime = min_next_time;
        return stepper_load_next(s);
    }
    s.count = count;
    s.time.waketime = min_next_time;
    Ok(SF_RESCHEDULE)
}

// Optimized entry point for step function (may be inlined into sched.c code)
pub unsafe fn stepper_event(t: *mut Timer) -> R<u8> {
    if HAVE_EDGE_OPTIMIZATION {
        return stepper_event_edge(t);
    }
    if HAVE_AVR_OPTIMIZATION {
        return stepper_event_avr(t);
    }
    stepper_event_full(t)
}

pub fn command_config_stepper(args: &[u32]) -> R {
    let s = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Stepper>() as u16)? as *mut Stepper) };
    let invert_step = args[3] as i8;
    if invert_step > 0 {
        s.flags = SF_INVERT_STEP;
    } else if invert_step < 0 {
        s.flags = SF_SINGLE_SCHED;
    }
    s.step_pin = gpio_out_setup(args[1], s.flags & SF_INVERT_STEP)?;
    s.dir_pin = gpio_out_setup(args[2], 0)?;
    s.position = POSITION_BIAS.wrapping_neg();
    s.step_pulse_ticks = args[4];
    move_queue_setup(&mut s.mq, size_of::<StepperMove>())?;
    if HAVE_EDGE_OPTIMIZATION {
        if invert_step < 0 && s.step_pulse_ticks <= EDGE_STEP_TICKS {
            s.flags |= SF_OPTIMIZED_PATH;
        } else {
            s.time.func = Some(stepper_event_full);
        }
    } else if HAVE_AVR_OPTIMIZATION {
        if invert_step >= 0 && s.step_pulse_ticks <= AVR_STEP_TICKS {
            s.flags |= SF_SINGLE_SCHED | SF_OPTIMIZED_PATH;
        } else {
            s.time.func = Some(stepper_event_full);
        }
    } else if !CONFIG_INLINE_STEPPER_HACK {
        s.time.func = Some(stepper_event_full);
    }
    Ok(())
}
decl_command!(
    command_config_stepper,
    concat!("config_stepper oid=%c step_pin=%c", " dir_pin=%c invert_step=%c step_pulse_ticks=%u")
);

// Return the 'struct stepper' for a given stepper oid
fn stepper_oid_lookup(oid: u8) -> R<&'static mut Stepper> {
    Ok(unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut Stepper) })
}

// Schedule a set of steps with a given timing
pub fn command_queue_step(args: &[u32]) -> R {
    let s = stepper_oid_lookup(args[0] as u8)?;
    let m = unsafe { &mut *(move_alloc()? as *mut StepperMove) };
    m.interval = args[1];
    m.count = args[2] as u16;
    if m.count == 0 {
        shutdown!("Invalid count parameter");
    }
    m.add = args[3] as i16;
    m.flags = 0;

    irq_disable();
    let mut flags = s.flags;
    if (flags & SF_LAST_DIR != 0) != (flags & SF_NEXT_DIR != 0) {
        flags ^= SF_LAST_DIR;
        m.flags |= MF_DIR;
    }
    if s.count != 0 {
        s.flags = flags;
        move_queue_push(&mut m.node, &mut s.mq);
    } else if flags & SF_NEED_RESET != 0 {
        move_free(m as *mut StepperMove as *mut u8);
    } else {
        s.flags = flags;
        move_queue_push(&mut m.node, &mut s.mq);
        unsafe { stepper_load_next(s)? };
        sched_add_timer(&mut s.time)?;
    }
    irq_enable();
    Ok(())
}
decl_command!(command_queue_step, "queue_step oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
pub fn command_set_next_step_dir(args: &[u32]) -> R {
    let s = stepper_oid_lookup(args[0] as u8)?;
    let nextdir = if args[1] != 0 { SF_NEXT_DIR } else { 0 };
    irq_disable();
    s.flags = (s.flags & !SF_NEXT_DIR) | nextdir;
    irq_enable();
    Ok(())
}
decl_command!(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
pub fn command_reset_step_clock(args: &[u32]) -> R {
    let s = stepper_oid_lookup(args[0] as u8)?;
    let waketime = args[1];
    irq_disable();
    if s.count != 0 {
        shutdown!("Can't reset time when stepper active");
    }
    s.time.waketime = waketime;
    s.next_step_time = waketime;
    s.flags &= !SF_NEED_RESET;
    irq_enable();
    Ok(())
}
decl_command!(command_reset_step_clock, "reset_step_clock oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
fn stepper_get_position(s: &Stepper) -> u32 {
    let mut position = s.position;
    // If stepper is mid-move, subtract out steps not yet taken
    if s.flags & SF_SINGLE_SCHED != 0 {
        position = position.wrapping_sub(s.count);
    } else {
        position = position.wrapping_sub(s.count / 2);
    }
    // The top bit of s->position is an optimized reverse direction flag
    if position & 0x80000000 != 0 {
        return position.wrapping_neg();
    }
    position
}

// Report the current position of the stepper
pub fn command_stepper_get_position(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let s = stepper_oid_lookup(oid)?;
    irq_disable();
    let position = stepper_get_position(s);
    irq_enable();
    sendf!("stepper_position oid=%c pos=%i", oid, position.wrapping_sub(POSITION_BIAS))
}
decl_command!(command_stepper_get_position, "stepper_get_position oid=%c");

// Stop all moves for a given stepper (caller must disable IRQs)
unsafe fn stepper_stop(tss: *mut TrsyncSignal, _reason: u8) {
    let s = &mut *((tss as *mut u8).sub(offset_of!(Stepper, stop_signal)) as *mut Stepper);
    sched_del_timer(&mut s.time);
    s.time.waketime = 0;
    s.next_step_time = 0;
    s.position = stepper_get_position(s).wrapping_neg();
    s.count = 0;
    s.flags = (s.flags & (SF_INVERT_STEP | SF_SINGLE_SCHED | SF_OPTIMIZED_PATH)) | SF_NEED_RESET;
    gpio_out_write(s.dir_pin, 0);
    if s.flags & SF_SINGLE_SCHED == 0 || (HAVE_AVR_OPTIMIZATION && s.flags & SF_OPTIMIZED_PATH != 0) {
        // Must return step pin to "unstep" state
        gpio_out_write(s.step_pin, s.flags & SF_INVERT_STEP);
    }
    while !move_queue_empty(&s.mq) {
        let mn = move_queue_pop(&mut s.mq);
        move_free(mn as *mut u8);
    }
}

// Set the stepper to stop on a "trigger event" (used in homing)
pub fn command_stepper_stop_on_trigger(args: &[u32]) -> R {
    let s = stepper_oid_lookup(args[0] as u8)?;
    let ts = trsync_oid_lookup(args[1] as u8)?;
    trsync_add_signal(ts, &mut s.stop_signal, stepper_stop)
}
decl_command!(command_stepper_stop_on_trigger, "stepper_stop_on_trigger oid=%c trsync_oid=%c");

pub fn stepper_shutdown() -> R {
    for p in foreach_oid(oid_type()) {
        let s = unsafe { &mut *(p as *mut Stepper) };
        move_queue_clear(&mut s.mq);
        unsafe { stepper_stop(&mut s.stop_signal, 0) };
    }
    Ok(())
}
decl_shutdown!(stepper_shutdown);
