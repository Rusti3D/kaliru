// Commands for controlling GPIO output pins
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{
    foreach_oid, move_alloc, move_free, move_queue_clear, move_queue_empty, move_queue_first, move_queue_pop, move_queue_push,
    move_queue_setup, oid_alloc, oid_lookup, MoveNode, MoveQueueHead, OidType,
};
use crate::board::{gpio_out_setup, gpio_out_toggle_noirq, gpio_out_write, irq_disable, irq_enable, timer_is_before, timer_read_time, GpioOut};
use crate::sched::{sched_add_timer, sched_del_timer, Timer, R, SF_DONE, SF_RESCHEDULE};
use crate::{decl_command, decl_shutdown, shutdown};
use core::mem::size_of;

#[repr(C)]
struct DigitalOut {
    timer: Timer,
    on_duration: u32,
    off_duration: u32,
    end_time: u32,
    pin: GpioOut,
    max_duration: u32,
    cycle_time: u32,
    mq: MoveQueueHead,
    flags: u8,
}

#[repr(C)]
struct DigitalMove {
    node: MoveNode,
    waketime: u32,
    on_duration: u32,
}

const DF_ON: u8 = 1 << 0;
const DF_TOGGLING: u8 = 1 << 1;
const DF_CHECK_END: u8 = 1 << 2;
const DF_DEFAULT_ON: u8 = 1 << 4;

fn oid_type() -> OidType {
    command_config_digital_out as *const ()
}

// Software PWM toggle event
unsafe fn digital_toggle_event(timer: *mut Timer) -> R<u8> {
    let d = &mut *(timer as *mut DigitalOut);
    gpio_out_toggle_noirq(d.pin);
    d.flags ^= DF_ON;
    let mut waketime = d.timer.waketime;
    if d.flags & DF_ON != 0 {
        waketime = waketime.wrapping_add(d.on_duration);
    } else {
        waketime = waketime.wrapping_add(d.off_duration);
    }
    if d.flags & DF_CHECK_END != 0 && !timer_is_before(waketime, d.end_time) {
        // End of normal pulsing - next event loads new pwm settings
        d.timer.func = Some(digital_load_event);
        waketime = d.end_time;
    }
    d.timer.waketime = waketime;
    Ok(SF_RESCHEDULE)
}

// Load next pin output setting
unsafe fn digital_load_event(timer: *mut Timer) -> R<u8> {
    // Apply next update and remove it from queue
    let d = &mut *(timer as *mut DigitalOut);
    if move_queue_empty(&d.mq) {
        shutdown!("Missed scheduling of next digital out event");
    }
    let mn = move_queue_pop(&mut d.mq);
    let m = mn as *mut DigitalMove;
    let on_duration = (*m).on_duration;
    let mut flags = if on_duration != 0 { DF_ON } else { 0 };
    gpio_out_write(d.pin, flags);
    move_free(m as *mut u8);

    // Calculate next end_time and flags
    let mut end_time: u32 = 0;
    if flags == 0 || on_duration >= d.cycle_time {
        // Pin is in an always on or always off state
        if (flags == 0) != (d.flags & DF_DEFAULT_ON == 0) && d.max_duration != 0 {
            end_time = d.timer.waketime.wrapping_add(d.max_duration);
            flags |= DF_CHECK_END;
        }
    } else {
        flags |= DF_TOGGLING;
        if d.max_duration != 0 {
            end_time = d.timer.waketime.wrapping_add(d.max_duration);
            flags |= DF_CHECK_END;
        }
    }
    if !move_queue_empty(&d.mq) {
        let nn = move_queue_first(&d.mq);
        let wake = (*(nn as *mut DigitalMove)).waketime;
        if flags & DF_CHECK_END != 0 && timer_is_before(end_time, wake) {
            shutdown!("Scheduled digital out event will exceed max_duration");
        }
        end_time = wake;
        flags |= DF_CHECK_END;
    }
    d.end_time = end_time;
    d.flags = flags | (d.flags & DF_DEFAULT_ON);

    // Schedule next event
    if flags & DF_TOGGLING == 0 {
        if flags & DF_CHECK_END == 0 {
            // Pin not toggling and nothing scheduled
            return Ok(SF_DONE);
        }
        d.timer.waketime = end_time;
        return Ok(SF_RESCHEDULE);
    }
    let waketime = d.timer.waketime.wrapping_add(on_duration);
    if flags & DF_CHECK_END != 0 && !timer_is_before(waketime, end_time) {
        d.timer.waketime = end_time;
        return Ok(SF_RESCHEDULE);
    }
    d.timer.func = Some(digital_toggle_event);
    d.timer.waketime = waketime;
    d.on_duration = on_duration;
    d.off_duration = d.cycle_time.wrapping_sub(on_duration);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_digital_out(args: &[u32]) -> R {
    let pin = gpio_out_setup(args[1], (args[2] != 0) as u8)?;
    let d = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<DigitalOut>() as u16)? as *mut DigitalOut) };
    d.pin = pin;
    d.flags = (if args[2] != 0 { DF_ON } else { 0 }) | (if args[3] != 0 { DF_DEFAULT_ON } else { 0 });
    d.max_duration = args[4];
    move_queue_setup(&mut d.mq, size_of::<DigitalMove>())
}
decl_command!(
    command_config_digital_out,
    concat!("config_digital_out oid=%c pin=%u value=%c", " default_value=%c max_duration=%u")
);

fn lookup(oid: u32) -> R<&'static mut DigitalOut> {
    Ok(unsafe { &mut *(oid_lookup(oid as u8, oid_type())? as *mut DigitalOut) })
}

pub fn command_set_digital_out_pwm_cycle(args: &[u32]) -> R {
    let d = lookup(args[0])?;
    irq_disable();
    if !move_queue_empty(&d.mq) {
        shutdown!("Can not set soft pwm cycle ticks while updates pending");
    }
    d.cycle_time = args[1];
    irq_enable();
    Ok(())
}
decl_command!(command_set_digital_out_pwm_cycle, "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u");

pub fn command_queue_digital_out(args: &[u32]) -> R {
    let d = lookup(args[0])?;
    let m = move_alloc()? as *mut DigitalMove;
    let time = args[1];
    unsafe {
        (*m).waketime = time;
        (*m).on_duration = args[2];
    }

    irq_disable();
    let first_on_queue = move_queue_push(m as *mut MoveNode, &mut d.mq);
    if !first_on_queue {
        irq_enable();
        return Ok(());
    }
    let flags = d.flags;
    if flags & DF_CHECK_END != 0 && timer_is_before(d.end_time, time) {
        shutdown!("Scheduled digital out event will exceed max_duration");
    }
    d.end_time = time;
    d.flags = flags | DF_CHECK_END;
    if flags & DF_TOGGLING != 0 && timer_is_before(d.timer.waketime, time) {
        // digital_toggle_event() will schedule a load event when ready
    } else {
        // Schedule the loading of the parameters at the requested time
        sched_del_timer(&mut d.timer);
        d.timer.waketime = time;
        d.timer.func = Some(digital_load_event);
        sched_add_timer(&mut d.timer)?;
    }
    irq_enable();
    Ok(())
}
decl_command!(command_queue_digital_out, "queue_digital_out oid=%c clock=%u on_ticks=%u");

pub fn command_update_digital_out(args: &[u32]) -> R {
    let d = lookup(args[0])?;
    sched_del_timer(&mut d.timer);
    if !move_queue_empty(&d.mq) {
        shutdown!("update_digital_out not valid with active queue");
    }
    let value = args[1] as u8;
    let flags = d.flags;
    let on_flag = if value != 0 { DF_ON } else { 0 };
    gpio_out_write(d.pin, on_flag);
    if (on_flag == 0) != (flags & DF_DEFAULT_ON == 0) && d.max_duration != 0 {
        d.end_time = timer_read_time().wrapping_add(d.max_duration);
        d.timer.waketime = d.end_time;
        d.timer.func = Some(digital_load_event);
        d.flags = (flags & DF_DEFAULT_ON) | on_flag | DF_CHECK_END;
        sched_add_timer(&mut d.timer)?;
    } else {
        d.flags = (flags & DF_DEFAULT_ON) | on_flag;
    }
    Ok(())
}
decl_command!(command_update_digital_out, "update_digital_out oid=%c value=%c");

pub fn digital_out_shutdown() {
    for p in foreach_oid(oid_type()) {
        let d = unsafe { &mut *(p as *mut DigitalOut) };
        gpio_out_write(d.pin, d.flags & DF_DEFAULT_ON);
        d.flags = if d.flags & DF_DEFAULT_ON != 0 { DF_ON | DF_DEFAULT_ON } else { 0 };
        move_queue_clear(&mut d.mq);
    }
}
decl_shutdown!(digital_out_shutdown);

pub fn command_set_digital_out(args: &[u32]) -> R {
    gpio_out_setup(args[0], args[1] as u8)?;
    Ok(())
}
decl_command!(command_set_digital_out, "set_digital_out pin=%u value=%c");
