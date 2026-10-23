// Handling of synchronized "trigger" dispatch
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{foreach_oid, oid_alloc, oid_lookup, OidType};
use crate::board::{irq_disable, irq_enable, irq_restore, irq_save, timer_read_time};
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_DONE, SF_RESCHEDULE};
use crate::{decl_command, decl_shutdown, decl_task, sendf, shutdown};
use core::mem::{offset_of, size_of};
use core::ptr::null_mut;

pub type TrsyncCallback = unsafe fn(*mut TrsyncSignal, u8);

#[repr(C)]
pub struct TrsyncSignal {
    pub next: *mut TrsyncSignal,
    pub func: Option<TrsyncCallback>,
}

#[repr(C)]
pub struct Trsync {
    report_time: Timer,
    expire_time: Timer,
    report_ticks: u32,
    signals: *mut TrsyncSignal,
    flags: u8,
    trigger_reason: u8,
    expire_reason: u8,
}

const TSF_CAN_TRIGGER: u8 = 1 << 0;
const TSF_REPORT: u8 = 1 << 2;

static TRSYNC_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_trsync as *const ()
}

// Activate a trigger
pub fn trsync_do_trigger(ts: *mut Trsync, reason: u8) {
    let flag = irq_save();
    unsafe {
        let flags = (*ts).flags;
        if flags & TSF_CAN_TRIGGER != 0 {
            (*ts).trigger_reason = reason;
            (*ts).flags = (flags & !TSF_CAN_TRIGGER) | TSF_REPORT;
            // Dispatch signals
            while !(*ts).signals.is_null() {
                let tss = (*ts).signals;
                let func = (*tss).func;
                (*ts).signals = (*tss).next;
                (*tss).next = null_mut();
                (*tss).func = None;
                (func.unwrap())(tss, reason);
            }
            sched_wake_task(TRSYNC_WAKE.ptr());
        }
    }
    irq_restore(flag);
}

// Timeout handler
unsafe fn trsync_expire_event(t: *mut Timer) -> R<u8> {
    let ts = (t as *mut u8).sub(offset_of!(Trsync, expire_time)) as *mut Trsync;
    trsync_do_trigger(ts, (*ts).expire_reason);
    Ok(SF_DONE)
}

// Report handler
unsafe fn trsync_report_event(t: *mut Timer) -> R<u8> {
    let ts = &mut *(t as *mut Trsync);
    ts.flags |= TSF_REPORT;
    sched_wake_task(TRSYNC_WAKE.ptr());
    ts.report_time.waketime = ts.report_time.waketime.wrapping_add(ts.report_ticks);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_trsync(args: &[u32]) -> R {
    let ts = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Trsync>() as u16)? as *mut Trsync) };
    ts.report_time.func = Some(trsync_report_event);
    ts.expire_time.func = Some(trsync_expire_event);
    Ok(())
}
decl_command!(command_config_trsync, "config_trsync oid=%c");

// Return the 'struct trsync' for a given trsync oid
pub fn trsync_oid_lookup(oid: u8) -> R<*mut Trsync> {
    Ok(oid_lookup(oid, oid_type())? as *mut Trsync)
}

// Add a callback to invoke on a trigger
pub fn trsync_add_signal(ts: *mut Trsync, tss: *mut TrsyncSignal, func: TrsyncCallback) -> R {
    let flag = irq_save();
    unsafe {
        if (*tss).func.is_some() {
            shutdown!("Can't add signal that is already active");
        }
        (*tss).func = Some(func);
        (*tss).next = (*ts).signals;
        (*ts).signals = tss;
    }
    irq_restore(flag);
    Ok(())
}

// Disable trigger and unregister any signal handlers (caller must disable IRQs)
fn trsync_clear(ts: &mut Trsync) {
    sched_del_timer(&mut ts.report_time);
    sched_del_timer(&mut ts.expire_time);
    let mut tss = ts.signals;
    while !tss.is_null() {
        unsafe {
            let next = (*tss).next;
            (*tss).func = None;
            (*tss).next = null_mut();
            tss = next;
        }
    }
    ts.signals = null_mut();
    ts.flags = 0;
    ts.trigger_reason = 0;
    ts.expire_reason = 0;
}

pub fn command_trsync_start(args: &[u32]) -> R {
    let ts = unsafe { &mut *trsync_oid_lookup(args[0] as u8)? };
    irq_disable();
    trsync_clear(ts);
    ts.flags = TSF_CAN_TRIGGER;
    ts.report_time.waketime = args[1];
    ts.report_ticks = args[2];
    if ts.report_ticks != 0 {
        sched_add_timer(&mut ts.report_time)?;
    }
    ts.expire_reason = args[3] as u8;
    irq_enable();
    Ok(())
}
decl_command!(
    command_trsync_start,
    concat!("trsync_start oid=%c report_clock=%u report_ticks=%u", " expire_reason=%c")
);

pub fn command_trsync_set_timeout(args: &[u32]) -> R {
    let ts = unsafe { &mut *trsync_oid_lookup(args[0] as u8)? };
    irq_disable();
    let flags = ts.flags;
    if flags & TSF_CAN_TRIGGER != 0 {
        sched_del_timer(&mut ts.expire_time);
        ts.expire_time.waketime = args[1];
        sched_add_timer(&mut ts.expire_time)?;
    }
    irq_enable();
    Ok(())
}
decl_command!(command_trsync_set_timeout, "trsync_set_timeout oid=%c clock=%u");

fn trsync_report(oid: u8, flags: u8, reason: u8, clock: u32) -> R {
    sendf!(
        "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
        oid,
        (flags & TSF_CAN_TRIGGER != 0) as u8,
        reason,
        clock
    )
}

pub fn command_trsync_trigger(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let ts = trsync_oid_lookup(oid)?;
    irq_disable();
    trsync_do_trigger(ts, args[1] as u8);
    let ts = unsafe { &mut *ts };
    sched_del_timer(&mut ts.report_time);
    sched_del_timer(&mut ts.expire_time);
    ts.flags = 0;
    let trigger_reason = ts.trigger_reason;
    irq_enable();
    trsync_report(oid, 0, trigger_reason, 0)
}
decl_command!(command_trsync_trigger, "trsync_trigger oid=%c reason=%c");

pub fn trsync_task() -> R {
    if !sched_check_wake(TRSYNC_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let ts = unsafe { &mut *(p as *mut Trsync) };
        if ts.flags & TSF_REPORT == 0 {
            continue;
        }
        let time = timer_read_time();
        irq_disable();
        let trigger_reason = ts.trigger_reason;
        let flags = ts.flags;
        ts.flags = flags & !TSF_REPORT;
        irq_enable();

        trsync_report(oid, flags, trigger_reason, time)?;
    }
    Ok(())
}
decl_task!(trsync_task);

pub fn trsync_shutdown() {
    for p in foreach_oid(oid_type()) {
        trsync_clear(unsafe { &mut *(p as *mut Trsync) });
    }
}
decl_shutdown!(trsync_shutdown);
