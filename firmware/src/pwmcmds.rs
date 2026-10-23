// Commands for controlling hardware based pulse-width-modulator pins
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{
    foreach_oid, move_alloc, move_free, move_queue_clear, move_queue_empty, move_queue_first, move_queue_pop, move_queue_push,
    move_queue_setup, oid_alloc, oid_lookup, MoveNode, MoveQueueHead, OidType,
};
use crate::board::{gpio_pwm_setup, gpio_pwm_write, irq_disable, irq_enable, timer_is_before, GpioPwm};
use crate::sched::{sched_add_timer, sched_del_timer, Timer, TimerFunc, R, SF_DONE, SF_RESCHEDULE};
use crate::{decl_command, decl_shutdown, shutdown};
use core::mem::size_of;

#[repr(C)]
struct PwmOut {
    timer: Timer,
    pin: GpioPwm,
    max_duration: u32,
    default_value: u16,
    mq: MoveQueueHead,
}

#[repr(C)]
struct PwmMove {
    node: MoveNode,
    waketime: u32,
    value: u16,
}

fn oid_type() -> OidType {
    command_config_pwm_out as *const ()
}

unsafe fn pwm_end_event(_timer: *mut Timer) -> R<u8> {
    shutdown!("Missed scheduling of next hard pwm event");
}

unsafe fn pwm_event(timer: *mut Timer) -> R<u8> {
    // Apply next update and remove it from queue
    let p = &mut *(timer as *mut PwmOut);
    let mn = move_queue_pop(&mut p.mq);
    let m = mn as *mut PwmMove;
    let value = (*m).value;
    gpio_pwm_write(p.pin, value);
    move_free(m as *mut u8);

    // Check if more updates queued
    if move_queue_empty(&p.mq) {
        if value == p.default_value || p.max_duration == 0 {
            return Ok(SF_DONE);
        }

        // Start the safety timeout
        p.timer.waketime = p.timer.waketime.wrapping_add(p.max_duration);
        p.timer.func = Some(pwm_end_event);
        return Ok(SF_RESCHEDULE);
    }

    // Schedule next update
    let nn = move_queue_first(&p.mq);
    let wake = (*(nn as *mut PwmMove)).waketime;
    if value != p.default_value && p.max_duration != 0 && timer_is_before(p.timer.waketime.wrapping_add(p.max_duration), wake) {
        shutdown!("Scheduled pwm event will exceed max_duration");
    }
    p.timer.waketime = wake;
    Ok(SF_RESCHEDULE)
}

pub fn command_config_pwm_out(args: &[u32]) -> R {
    let pin = gpio_pwm_setup(args[1], args[2], args[3] as u16)?;
    let p = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<PwmOut>() as u16)? as *mut PwmOut) };
    p.pin = pin;
    p.default_value = args[4] as u16;
    p.max_duration = args[5];
    p.timer.func = Some(pwm_event);
    move_queue_setup(&mut p.mq, size_of::<PwmMove>())
}
decl_command!(
    command_config_pwm_out,
    concat!("config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu", " default_value=%hu max_duration=%u")
);

fn is_func(f: Option<TimerFunc>, g: TimerFunc) -> bool {
    matches!(f, Some(f) if f as usize == g as usize)
}

pub fn command_queue_pwm_out(args: &[u32]) -> R {
    let p = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut PwmOut) };
    let m = unsafe { &mut *(move_alloc()? as *mut PwmMove) };
    m.waketime = args[1];
    m.value = args[2] as u16;

    irq_disable();
    let need_add_timer = move_queue_push(&mut m.node, &mut p.mq);
    irq_enable();
    if !need_add_timer {
        return Ok(());
    }

    // queue was empty and a timer needs to be added
    sched_del_timer(&mut p.timer);
    if is_func(p.timer.func, pwm_end_event) && timer_is_before(p.timer.waketime, m.waketime) {
        shutdown!("Scheduled pwm event will exceed max_duration");
    }
    p.timer.func = Some(pwm_event);
    p.timer.waketime = m.waketime;
    sched_add_timer(&mut p.timer)
}
decl_command!(command_queue_pwm_out, "queue_pwm_out oid=%c clock=%u value=%hu");

pub fn pwm_shutdown() -> R {
    for q in foreach_oid(oid_type()) {
        let p = unsafe { &mut *(q as *mut PwmOut) };
        gpio_pwm_write(p.pin, p.default_value);
        p.timer.func = Some(pwm_event);
        move_queue_clear(&mut p.mq);
    }
    Ok(())
}
decl_shutdown!(pwm_shutdown);

pub fn command_set_pwm_out(args: &[u32]) -> R {
    gpio_pwm_setup(args[0], args[1], args[2] as u16)?;
    Ok(())
}
decl_command!(command_set_pwm_out, "set_pwm_out pin=%u cycle_ticks=%u value=%hu");
