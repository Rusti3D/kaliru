// Support for gathering acceleration data from ADXL345 chip
//
// Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{irq_disable, irq_enable, timer_read_time};
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_DONE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, sensor_bulk_status, SensorBulk};
use crate::spicmds::{spidev_oid_lookup, spidev_transfer, SpiDev};
use crate::{decl_command, decl_task};
use core::mem::size_of;

#[repr(C)]
struct Adxl345 {
    timer: Timer,
    rest_ticks: u32,
    spi: *mut SpiDev,
    flags: u8,
    sb: SensorBulk,
}

const AX_PENDING: u8 = 1 << 0;

static ADXL345_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_adxl345 as *const ()
}

// Event handler that wakes adxl345_task() periodically
unsafe fn adxl345_event(timer: *mut Timer) -> R<u8> {
    let ax = &mut *(timer as *mut Adxl345);
    ax.flags |= AX_PENDING;
    sched_wake_task(ADXL345_WAKE.ptr());
    Ok(SF_DONE)
}

pub fn command_config_adxl345(args: &[u32]) -> R {
    let ax = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Adxl345>() as u16)? as *mut Adxl345) };
    ax.timer.func = Some(adxl345_event);
    ax.spi = spidev_oid_lookup(args[1] as u8)?;
    Ok(())
}
decl_command!(command_config_adxl345, "config_adxl345 oid=%c spi_oid=%c");

// Helper code to reschedule the adxl345_event() timer
fn adxl_reschedule_timer(ax: &mut Adxl345) -> R {
    irq_disable();
    ax.timer.waketime = timer_read_time().wrapping_add(ax.rest_ticks);
    sched_add_timer(&mut ax.timer)?;
    irq_enable();
    Ok(())
}

// Chip registers
const AR_DATAX0: u8 = 0x32;
const AR_FIFO_STATUS: u8 = 0x39;
const AM_READ: u8 = 0x80;
const AM_MULTI: u8 = 0x40;

const SET_FIFO_CTL: u8 = 0x90;

const BYTES_PER_SAMPLE: usize = 5;

fn bad_high(b: u8) -> bool {
    (b & 0xf0) != 0 && (b & 0xf0) != 0xf0
}

// Query accelerometer data
fn adxl_query(ax: &mut Adxl345, oid: u8) -> R {
    // Read data
    let mut msg = [AR_DATAX0 | AM_READ | AM_MULTI, 0, 0, 0, 0, 0, 0, 0, 0];
    spidev_transfer(ax.spi, 1, &mut msg)?;
    // Extract x, y, z measurements
    let mut fifo_status = msg[8] & !0x80; // Ignore trigger bit
    let dc = ax.sb.data_count as usize;
    let d = &mut ax.sb.data[dc..dc + BYTES_PER_SAMPLE];
    if bad_high(msg[2]) || bad_high(msg[4]) || bad_high(msg[6]) || msg[7] != SET_FIFO_CTL || fifo_status > 32 {
        // Data error - may be a CS, MISO, MOSI, or SCLK glitch
        d.fill(0xff);
        fifo_status = 0;
    } else {
        // Copy data
        d[0] = msg[1]; // x low bits
        d[1] = msg[3]; // y low bits
        d[2] = msg[5]; // z low bits
        d[3] = (msg[2] & 0x1f) | (msg[6] << 5); // x high bits and z high bits
        d[4] = (msg[4] & 0x1f) | ((msg[6] << 2) & 0x60); // y high and z high
    }
    ax.sb.data_count += BYTES_PER_SAMPLE as u8;
    if ax.sb.data_count as usize + BYTES_PER_SAMPLE > ax.sb.data.len() {
        sensor_bulk_report(&mut ax.sb, oid)?;
    }
    // Check fifo status
    if fifo_status >= 31 {
        ax.sb.possible_overflows = ax.sb.possible_overflows.wrapping_add(1);
    }
    if fifo_status > 1 {
        // More data in fifo - wake this task again
        sched_wake_task(ADXL345_WAKE.ptr());
    } else {
        // Sleep until next check time
        ax.flags &= !AX_PENDING;
        adxl_reschedule_timer(ax)?;
    }
    Ok(())
}

pub fn command_query_adxl345(args: &[u32]) -> R {
    let ax = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Adxl345) };

    sched_del_timer(&mut ax.timer);
    ax.flags = 0;
    if args[1] == 0 {
        // End measurements
        return Ok(());
    }

    // Start new measurements query
    ax.rest_ticks = args[1];
    sensor_bulk_reset(&mut ax.sb);
    adxl_reschedule_timer(ax)
}
decl_command!(command_query_adxl345, "query_adxl345 oid=%c rest_ticks=%u");

pub fn command_query_adxl345_status(args: &[u32]) -> R {
    let ax = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Adxl345) };
    let mut msg = [AR_FIFO_STATUS | AM_READ, 0x00];

    let time1 = timer_read_time();
    spidev_transfer(ax.spi, 1, &mut msg)?;
    let time2 = timer_read_time();

    let fifo_status = msg[1] & !0x80; // Ignore trigger bit
    if fifo_status > 32 {
        // Query error - don't send response - host will retry
        return Ok(());
    }
    sensor_bulk_status(&ax.sb, args[0] as u8, time1, time2.wrapping_sub(time1), fifo_status as u32 * BYTES_PER_SAMPLE as u32)
}
decl_command!(command_query_adxl345_status, "query_adxl345_status oid=%c");

pub fn adxl345_task() -> R {
    if !sched_check_wake(ADXL345_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let ax = unsafe { &mut *(p as *mut Adxl345) };
        let flags = ax.flags;
        if flags & AX_PENDING != 0 {
            adxl_query(ax, oid)?;
        }
    }
    Ok(())
}
decl_task!(adxl345_task);
