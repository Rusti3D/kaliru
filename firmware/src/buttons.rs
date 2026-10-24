// Report on user interface buttons
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_in_read, gpio_in_setup, irq_disable, irq_enable, GpioIn};
use crate::command::Args;
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::{decl_command, decl_task, sendf, shutdown};
use core::mem::size_of;

#[repr(C)]
struct Buttons {
    time: Timer,
    rest_ticks: u32,
    pressed: u8,
    last_pressed: u8,
    report_count: u8,
    reports: [u8; 8],
    ack_count: u8,
    retransmit_state: u8,
    retransmit_count: u8,
    button_count: u8,
    pins: [GpioIn; 0],
}

impl Buttons {
    fn pin(&mut self, i: usize) -> &mut GpioIn {
        unsafe { &mut *self.pins.as_mut_ptr().add(i) }
    }
}

const BF_NO_RETRANSMIT: u8 = 0x80;
const BF_PENDING: u8 = 0xff;
const BF_ACKED: u8 = 0xfe;

static BUTTONS_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_buttons as *const ()
}

unsafe fn buttons_event(t: *mut Timer) -> R<u8> {
    let b = &mut *(t as *mut Buttons);

    // Read pins
    let mut status: u8 = 0;
    let mut bit: u8 = 1;
    for i in 0..b.button_count as usize {
        let val = gpio_in_read(*b.pin(i));
        if val != 0 {
            status |= bit;
        }
        bit = bit.wrapping_shl(1);
    }

    // Check if any pins have changed since last time
    let diff = status ^ b.pressed;
    if diff != 0 {
        // At least one pin has changed - do button debouncing
        let debounced = !(status ^ b.last_pressed);
        if diff & debounced != 0 {
            // Pin has been consistently different - report it
            b.pressed = (b.pressed & !debounced) | (status & debounced);
            if (b.report_count as usize) < b.reports.len() {
                b.reports[b.report_count as usize] = b.pressed;
                b.report_count += 1;
                sched_wake_task(BUTTONS_WAKE.ptr());
                b.retransmit_state = BF_PENDING;
            }
        }
    }
    b.last_pressed = status;

    // Check if a retransmit is needed
    let mut retransmit_state = b.retransmit_state;
    if retransmit_state & BF_NO_RETRANSMIT == 0 {
        retransmit_state = retransmit_state.wrapping_sub(1);
        if retransmit_state & BF_NO_RETRANSMIT != 0 {
            // timeout - do retransmit
            sched_wake_task(BUTTONS_WAKE.ptr());
        }
        b.retransmit_state = retransmit_state;
    }

    // Reschedule timer
    b.time.waketime = b.time.waketime.wrapping_add(b.rest_ticks);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_buttons(args: &Args) -> R {
    let button_count = args[1] as u8;
    if button_count > 8 {
        shutdown!("Max of 8 buttons");
    }
    let size = size_of::<Buttons>() + size_of::<GpioIn>() * button_count as usize;
    let b = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size as u16)? as *mut Buttons) };
    b.button_count = button_count;
    b.time.func = Some(buttons_event);
    Ok(())
}
decl_command!(command_config_buttons, "config_buttons oid=%c button_count=%c");

fn lookup(oid: u32) -> R<&'static mut Buttons> {
    Ok(unsafe { &mut *(oid_lookup(oid as u8, oid_type())? as *mut Buttons) })
}

pub fn command_buttons_add(args: &Args) -> R {
    let b = lookup(args[0])?;
    let pos = args[1] as u8;
    if pos >= b.button_count {
        shutdown!("Set button past maximum button count");
    }
    *b.pin(pos as usize) = gpio_in_setup(args[2], args[3] as crate::board::GpioPullUp)?;
    Ok(())
}
decl_command!(command_buttons_add, "buttons_add oid=%c pos=%c pin=%u pull_up=%c");

pub fn command_buttons_query(args: &Args) -> R {
    let b = lookup(args[0])?;
    sched_del_timer(&mut b.time);
    b.time.waketime = args[1];
    b.rest_ticks = args[2];
    b.pressed = args[4] as u8;
    b.last_pressed = args[4] as u8;
    b.ack_count = 0;
    b.report_count = 0;
    b.retransmit_state = BF_ACKED;
    b.retransmit_count = args[3] as u8;
    if b.retransmit_count >= BF_NO_RETRANSMIT {
        shutdown!("Invalid buttons retransmit count");
    }
    if b.rest_ticks == 0 {
        return Ok(());
    }
    sched_add_timer(&mut b.time)
}
decl_command!(
    command_buttons_query,
    concat!("buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c", " invert=%c")
);

pub fn command_buttons_ack(args: &Args) -> R {
    let b = lookup(args[0])?;
    let count = args[1] as u8;
    b.ack_count = b.ack_count.wrapping_add(count);
    irq_disable();
    if count >= b.report_count {
        b.report_count = 0;
        b.retransmit_state = BF_ACKED;
    } else {
        let pending = b.report_count - count;
        for i in 0..pending as usize {
            b.reports[i] = b.reports[i + count as usize];
        }
        b.report_count = pending;
    }
    irq_enable();
    Ok(())
}
decl_command!(command_buttons_ack, "buttons_ack oid=%c count=%c");

pub fn buttons_task() -> R {
    if !sched_check_wake(BUTTONS_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let b = unsafe { &mut *(p as *mut Buttons) };
        // See if need to transmit buttons_state
        if b.retransmit_state != BF_PENDING {
            continue;
        }
        // Generate message
        irq_disable();
        let report_count = b.report_count;
        if report_count == 0 {
            irq_enable();
            continue;
        }
        b.retransmit_state = b.retransmit_count;
        irq_enable();
        sendf!("buttons_state oid=%c ack_count=%c state=%*s", oid, b.ack_count, &b.reports[..report_count as usize])?;
    }
    Ok(())
}
decl_task!(buttons_task);
