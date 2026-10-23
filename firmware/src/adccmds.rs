// Commands for controlling GPIO analog-to-digital input pins
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{foreach_oid, oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_adc_cancel_sample, gpio_adc_read, gpio_adc_sample, gpio_adc_setup, irq_disable, irq_enable, GpioAdc};
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::{decl_command, decl_shutdown, decl_task, sendf, try_shutdown};
use core::mem::size_of;

#[repr(C)]
struct AnalogIn {
    timer: Timer,
    rest_time: u32,
    sample_time: u32,
    next_begin_time: u32,
    value: u16,
    min_value: u16,
    max_value: u16,
    pin: GpioAdc,
    invalid_count: u8,
    range_check_count: u8,
    state: u8,
    sample_count: u8,
}

static ANALOG_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_analog_in as *const ()
}

unsafe fn analog_in_event(timer: *mut Timer) -> R<u8> {
    let a = &mut *(timer as *mut AnalogIn);
    let sample_delay = gpio_adc_sample(a.pin);
    if sample_delay != 0 {
        a.timer.waketime = a.timer.waketime.wrapping_add(sample_delay);
        return Ok(SF_RESCHEDULE);
    }
    let mut value = gpio_adc_read(a.pin)?;
    let mut state = a.state;
    if state >= a.sample_count {
        state = 0;
    } else {
        value = value.wrapping_add(a.value);
    }
    a.value = value;
    a.state = state.wrapping_add(1);
    if a.state < a.sample_count {
        a.timer.waketime = a.timer.waketime.wrapping_add(a.sample_time);
        return Ok(SF_RESCHEDULE);
    }
    if a.range_check_count > 0 {
        if a.value >= a.min_value && a.value <= a.max_value {
            a.invalid_count = 0;
        } else {
            a.invalid_count = a.invalid_count.wrapping_add(1);
            if a.invalid_count >= a.range_check_count {
                try_shutdown!("ADC out of range");
                a.invalid_count = 0;
            }
        }
    }
    sched_wake_task(ANALOG_WAKE.ptr());
    a.next_begin_time = a.next_begin_time.wrapping_add(a.rest_time);
    a.timer.waketime = a.next_begin_time;
    Ok(SF_RESCHEDULE)
}

pub fn command_config_analog_in(args: &[u32]) -> R {
    let pin = gpio_adc_setup(args[1])?;
    let a = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<AnalogIn>() as u16)? as *mut AnalogIn) };
    a.timer.func = Some(analog_in_event);
    a.pin = pin;
    a.state = 1;
    Ok(())
}
decl_command!(command_config_analog_in, "config_analog_in oid=%c pin=%u");

pub fn command_query_analog_in(args: &[u32]) -> R {
    let a = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut AnalogIn) };
    sched_del_timer(&mut a.timer);
    gpio_adc_cancel_sample(a.pin);
    a.next_begin_time = args[1];
    a.timer.waketime = a.next_begin_time;
    a.sample_time = args[2];
    a.sample_count = args[3] as u8;
    a.state = a.sample_count.wrapping_add(1);
    a.rest_time = args[4];
    a.min_value = args[5] as u16;
    a.max_value = args[6] as u16;
    a.range_check_count = args[7] as u8;
    if a.sample_count == 0 {
        return Ok(());
    }
    sched_add_timer(&mut a.timer)
}
decl_command!(
    command_query_analog_in,
    concat!(
        "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c",
        " rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c"
    )
);

pub fn analog_in_task() -> R {
    if !sched_check_wake(ANALOG_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let a = unsafe { &mut *(p as *mut AnalogIn) };
        if a.state != a.sample_count {
            continue;
        }
        irq_disable();
        if a.state != a.sample_count {
            irq_enable();
            continue;
        }
        let value = a.value;
        let next_begin_time = a.next_begin_time;
        a.state = a.state.wrapping_add(1);
        irq_enable();
        sendf!("analog_in_state oid=%c next_clock=%u value=%hu", oid, next_begin_time, value)?;
    }
    Ok(())
}
decl_task!(analog_in_task);

pub fn analog_in_shutdown() -> R {
    for p in foreach_oid(oid_type()) {
        let a = unsafe { &mut *(p as *mut AnalogIn) };
        gpio_adc_cancel_sample(a.pin);
        if a.sample_count != 0 {
            a.state = a.sample_count.wrapping_add(1);
            a.next_begin_time = a.next_begin_time.wrapping_add(a.rest_time);
            a.timer.waketime = a.next_begin_time;
            sched_add_timer(&mut a.timer)?;
        }
    }
    Ok(())
}
decl_shutdown!(analog_in_shutdown);
