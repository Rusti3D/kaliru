// Handling of end stops.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_in_read, gpio_in_setup, irq_disable, irq_enable, GpioIn};
use crate::command::Args;
use crate::sched::{sched_add_timer, sched_del_timer, Timer, R, SF_DONE, SF_RESCHEDULE};
use crate::trsync::{trsync_do_trigger, trsync_oid_lookup, Trsync};
use crate::{decl_command, sendf};
use core::mem::size_of;
use core::ptr::null_mut;

#[repr(C)]
struct Endstop {
    time: Timer,
    pin: GpioIn,
    rest_time: u32,
    sample_time: u32,
    nextwake: u32,
    ts: *mut Trsync,
    flags: u8,
    sample_count: u8,
    trigger_count: u8,
    trigger_reason: u8,
}

const ESF_PIN_HIGH: u8 = 1 << 0;
const ESF_HOMING: u8 = 1 << 1;

fn oid_type() -> OidType {
    command_config_endstop as *const ()
}

// Timer callback for an end stop
unsafe fn endstop_event(t: *mut Timer) -> R<u8> {
    let e = &mut *(t as *mut Endstop);
    let val = gpio_in_read(e.pin);
    let nextwake = e.time.waketime.wrapping_add(e.rest_time);
    if (if val != 0 { !e.flags } else { e.flags }) & ESF_PIN_HIGH != 0 {
        // No match - reschedule for the next attempt
        e.time.waketime = nextwake;
        return Ok(SF_RESCHEDULE);
    }
    e.nextwake = nextwake;
    e.time.func = Some(endstop_oversample_event);
    endstop_oversample_event(t)
}

// Timer callback for an end stop that is sampling extra times
unsafe fn endstop_oversample_event(t: *mut Timer) -> R<u8> {
    let e = &mut *(t as *mut Endstop);
    let val = gpio_in_read(e.pin);
    if (if val != 0 { !e.flags } else { e.flags }) & ESF_PIN_HIGH != 0 {
        // No longer matching - reschedule for the next attempt
        e.time.func = Some(endstop_event);
        e.time.waketime = e.nextwake;
        e.trigger_count = e.sample_count;
        return Ok(SF_RESCHEDULE);
    }
    let count = e.trigger_count.wrapping_sub(1);
    if count == 0 {
        trsync_do_trigger(e.ts, e.trigger_reason);
        return Ok(SF_DONE);
    }
    e.trigger_count = count;
    e.time.waketime = e.time.waketime.wrapping_add(e.sample_time);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_endstop(args: &Args) -> R {
    let e = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Endstop>() as u16)? as *mut Endstop) };
    e.pin = gpio_in_setup(args[1], args[2] as crate::board::GpioPullUp)?;
    Ok(())
}
decl_command!(command_config_endstop, "config_endstop oid=%c pin=%c pull_up=%c");

// Home an axis
pub fn command_endstop_home(args: &Args) -> R {
    let e = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Endstop) };
    sched_del_timer(&mut e.time);
    e.time.waketime = args[1];
    e.sample_time = args[2];
    e.sample_count = args[3] as u8;
    if e.sample_count == 0 {
        // Disable end stop checking
        e.ts = null_mut();
        e.flags = 0;
        return Ok(());
    }
    e.rest_time = args[4];
    e.time.func = Some(endstop_event);
    e.trigger_count = e.sample_count;
    e.flags = ESF_HOMING | if args[5] != 0 { ESF_PIN_HIGH } else { 0 };
    e.ts = trsync_oid_lookup(args[6] as u8)?;
    e.trigger_reason = args[7] as u8;
    sched_add_timer(&mut e.time)
}
decl_command!(
    command_endstop_home,
    concat!(
        "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c",
        " rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c"
    )
);

pub fn command_endstop_query_state(args: &Args) -> R {
    let oid = args[0] as u8;
    let e = unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut Endstop) };

    irq_disable();
    let eflags = e.flags;
    let nextwake = e.nextwake;
    irq_enable();

    sendf!(
        "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
        oid,
        (eflags & ESF_HOMING != 0) as u8,
        nextwake,
        gpio_in_read(e.pin)
    )
}
decl_command!(command_endstop_query_state, "endstop_query_state oid=%c");
