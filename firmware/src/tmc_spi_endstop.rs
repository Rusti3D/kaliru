// Sensorless homing by polling the tmc stallguard flag over spi
//
// Copyright (C) 2026  Rogerio Goncalves <rogerlz@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{irq_disable, irq_enable, timer_read_time};
use crate::command::Args;
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::spicmds::{spidev_oid_lookup, spidev_transfer, SpiDev};
use crate::trsync::{trsync_do_trigger, trsync_oid_lookup, Trsync};
use crate::{decl_command, decl_task, sendf, shutdown};
use core::mem::size_of;
use core::ptr::null_mut;

// 5 byte datagram per driver, up to 8 drivers in a daisy chain
const MAX_DATA_LEN: usize = 40;
const SPI_STATUS_SG2: u8 = 0x04;

#[repr(C)]
struct TmcSpiEndstop {
    timer: Timer,
    rest_ticks: u32,
    trigger_clock: u32,
    spi: *mut SpiDev,
    ts: *mut Trsync,
    pending: u8,
    primed: u8,
    data_len: u8,
    status_pos: u8,
    trigger_reason: u8,
}

static TMC_SPI_ENDSTOP_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_tmc_spi_endstop as *const ()
}

unsafe fn tmc_spi_endstop_event(t: *mut Timer) -> R<u8> {
    let e = &mut *(t as *mut TmcSpiEndstop);
    e.pending = 1;
    sched_wake_task(TMC_SPI_ENDSTOP_WAKE.ptr());
    e.timer.waketime = e.timer.waketime.wrapping_add(e.rest_ticks);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_tmc_spi_endstop(args: &Args) -> R {
    let (data_len, status_pos) = (args[2] as u8, args[3] as u8);
    if data_len as usize > MAX_DATA_LEN || status_pos >= data_len {
        shutdown!("Invalid tmc_spi_endstop config");
    }
    let e = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<TmcSpiEndstop>() as u16)? as *mut TmcSpiEndstop) };
    e.timer.func = Some(tmc_spi_endstop_event);
    e.spi = spidev_oid_lookup(args[1] as u8)?;
    e.data_len = data_len;
    e.status_pos = status_pos;
    Ok(())
}
decl_command!(
    command_config_tmc_spi_endstop,
    concat!("config_tmc_spi_endstop oid=%c spi_oid=%c data_len=%c", " status_pos=%c")
);

pub fn command_tmc_spi_endstop_home(args: &Args) -> R {
    let e = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut TmcSpiEndstop) };
    sched_del_timer(&mut e.timer);
    e.pending = 0;
    e.primed = 0;
    e.rest_ticks = args[2];
    if e.rest_ticks == 0 {
        e.ts = null_mut();
        return Ok(());
    }
    e.timer.waketime = args[1];
    e.trigger_clock = args[1];
    e.ts = trsync_oid_lookup(args[3] as u8)?;
    e.trigger_reason = args[4] as u8;
    sched_add_timer(&mut e.timer)
}
decl_command!(
    command_tmc_spi_endstop_home,
    concat!("tmc_spi_endstop_home oid=%c clock=%u rest_ticks=%u", " trsync_oid=%c trigger_reason=%c")
);

pub fn command_tmc_spi_endstop_query_state(args: &Args) -> R {
    let e = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut TmcSpiEndstop) };
    sendf!("tmc_spi_endstop_state oid=%c trigger_clock=%u", args[0] as u8, e.trigger_clock)
}
decl_command!(command_tmc_spi_endstop_query_state, "tmc_spi_endstop_query_state oid=%c");

pub fn tmc_spi_endstop_task() -> R {
    if !sched_check_wake(TMC_SPI_ENDSTOP_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let e = unsafe { &mut *(p as *mut TmcSpiEndstop) };
        if e.pending == 0 {
            continue;
        }
        irq_disable();
        e.pending = 0;
        irq_enable();
        let mut data = [0u8; MAX_DATA_LEN];
        let time = timer_read_time();
        spidev_transfer(e.spi, 1, &mut data[..e.data_len as usize])?;
        // The status bits are latched at the previous transfer, which
        // happened long before homing started; skip the first result
        if e.primed == 0 {
            e.primed = 1;
            continue;
        }
        if data[e.status_pos as usize] & SPI_STATUS_SG2 == 0 {
            continue;
        }
        sched_del_timer(&mut e.timer);
        e.pending = 0;
        e.trigger_clock = time;
        trsync_do_trigger(e.ts, e.trigger_reason);
    }
    Ok(())
}
decl_task!(tmc_spi_endstop_task);
