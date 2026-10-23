// Commands for counting edges on GPIO input pins
//
// Copyright (C) 2021  Adrian Keet <arkeet@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_in_read, gpio_in_setup, irq_disable, irq_enable, timer_is_before, GpioIn};
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::{decl_command, decl_task, sendf};
use core::mem::size_of;

#[repr(C)]
struct Counter {
    timer: Timer,
    poll_ticks: u32,
    sample_ticks: u32,
    next_sample_time: u32,
    count: u32,
    last_count_time: u32,
    flags: u8,
    pin: GpioIn,
}

const CF_PENDING: u8 = 1;

static COUNTER_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_counter as *const ()
}

unsafe fn counter_event(timer: *mut Timer) -> R<u8> {
    let c = &mut *(timer as *mut Counter);

    let time = c.timer.waketime;
    let last_value = (c.count & 1) as u8;
    let value = gpio_in_read(c.pin);
    if last_value != value {
        c.count = c.count.wrapping_add(1);
        c.last_count_time = time;
    }
    // useful invariant: c->count & 1 == value

    if timer_is_before(c.next_sample_time, time) {
        c.flags |= CF_PENDING;
        c.next_sample_time = time.wrapping_add(c.sample_ticks);
        sched_wake_task(COUNTER_WAKE.ptr());
    }

    c.timer.waketime = c.timer.waketime.wrapping_add(c.poll_ticks);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_counter(args: &[u32]) -> R {
    let c = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Counter>() as u16)? as *mut Counter) };
    c.pin = gpio_in_setup(args[1], args[2] as i8)?;
    c.timer.func = Some(counter_event);
    Ok(())
}
decl_command!(command_config_counter, "config_counter oid=%c pin=%u pull_up=%c");

pub fn command_query_counter(args: &[u32]) -> R {
    let c = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Counter) };
    sched_del_timer(&mut c.timer);
    c.timer.waketime = args[1];
    c.poll_ticks = args[2];
    c.sample_ticks = args[3];
    c.next_sample_time = c.timer.waketime; // sample immediately
    sched_add_timer(&mut c.timer)
}
decl_command!(command_query_counter, "query_counter oid=%c clock=%u poll_ticks=%u sample_ticks=%u");

pub fn counter_task() -> R {
    if !sched_check_wake(COUNTER_WAKE.ptr()) {
        return Ok(());
    }

    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let c = unsafe { &mut *(p as *mut Counter) };
        if c.flags & CF_PENDING == 0 {
            continue;
        }
        irq_disable();
        let waketime = c.timer.waketime;
        let count = c.count;
        let count_time = c.last_count_time;
        c.flags &= !CF_PENDING;
        irq_enable();
        sendf!("counter_state oid=%c next_clock=%u count=%u count_clock=%u", oid, waketime, count, count_time)?;
    }
    Ok(())
}
decl_task!(counter_task);
