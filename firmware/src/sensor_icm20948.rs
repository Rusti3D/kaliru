// Support for gathering acceleration data from icm20948 chip
//
// Copyright (C) 2024 Paul Hansel <github@paulhansel.com>
// Copyright (C) 2023 Matthew Swabey <matthew@swabey.org>
// Copyright (C) 2022 Harry Beyel <harry3b9@gmail.com>
// Copyright (C) 2020-2023 Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{irq_disable, irq_enable, timer_read_time};
use crate::command::Args;
use crate::ctr::Global;
use crate::i2ccmds::{i2c_dev_read, i2c_shutdown_on_err, i2cdev_oid_lookup, I2cDev};
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_DONE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, sensor_bulk_status, SensorBulk};
use crate::{decl_command, decl_task};
use core::mem::size_of;

// Chip registers
const AR_FIFO_COUNT_H: u8 = 0x70;
const AR_FIFO: u8 = 0x72;
const AR_INT_STATUS: u8 = 0x1B; // INT_STATUS_2

const FIFO_OVERFLOW_INT: u8 = 0x0F; // from datasheet

const BYTES_PER_BLOCK: u16 = 48;

#[repr(C)]
struct Icm20948 {
    timer: Timer,
    rest_ticks: u32,
    i2c: *mut I2cDev,
    fifo_max: u16,
    fifo_pkts_bytes: u16,
    flags: u8,
    sb: SensorBulk,
}

const AX_PENDING: u8 = 1 << 0;

static ICM20948_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_icm20948 as *const ()
}

// Event handler that wakes icm20948_task() periodically
unsafe fn icm20948_event(timer: *mut Timer) -> R<u8> {
    let ax = &mut *(timer as *mut Icm20948);
    ax.flags |= AX_PENDING;
    sched_wake_task(ICM20948_WAKE.ptr());
    Ok(SF_DONE)
}

pub fn command_config_icm20948(args: &Args) -> R {
    let ic = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Icm20948>() as u16)? as *mut Icm20948) };
    ic.timer.func = Some(icm20948_event);
    ic.i2c = i2cdev_oid_lookup(args[1] as u8)?;
    Ok(())
}
decl_command!(command_config_icm20948, "config_icm20948 oid=%c i2c_oid=%c");

// Helper code to reschedule the icm20948_event() timer
fn ic20948_reschedule_timer(ic: &mut Icm20948) -> R {
    irq_disable();
    ic.timer.waketime = timer_read_time().wrapping_add(ic.rest_ticks);
    sched_add_timer(&mut ic.timer)?;
    irq_enable();
    Ok(())
}

fn read_mpu(i2c: *mut I2cDev, reg: &[u8], read: &mut [u8]) -> R {
    let ret = i2c_dev_read(i2c, reg, read)?;
    i2c_shutdown_on_err(ret)
}

// Reads the fifo byte count from the device.
fn get_fifo_status(ic: &mut Icm20948) -> R<u16> {
    let reg = [AR_FIFO_COUNT_H];
    let mut msg = [0u8; 2];
    read_mpu(ic.i2c, &reg, &mut msg)?;
    let fifo_bytes = (((msg[0] & 0x1f) as u16) << 8) | msg[1] as u16;
    if fifo_bytes > ic.fifo_max {
        ic.fifo_max = fifo_bytes;
    }
    Ok(fifo_bytes)
}

// Query accelerometer data
fn ic20948_query(ic: &mut Icm20948, oid: u8) -> R {
    // If not enough bytes to fill report read MPU FIFO's fill
    if ic.fifo_pkts_bytes < BYTES_PER_BLOCK {
        ic.fifo_pkts_bytes = get_fifo_status(ic)?;
    }

    // If we have enough bytes to fill the buffer do it and send report
    if ic.fifo_pkts_bytes >= BYTES_PER_BLOCK {
        let reg = [AR_FIFO];
        read_mpu(ic.i2c, &reg, &mut ic.sb.data[..BYTES_PER_BLOCK as usize])?;
        ic.sb.data_count = BYTES_PER_BLOCK as u8;
        ic.fifo_pkts_bytes -= BYTES_PER_BLOCK;
        sensor_bulk_report(&mut ic.sb, oid)?;
    }

    // If we have enough bytes remaining to fill another report wake again
    //  otherwise schedule timed wakeup
    if ic.fifo_pkts_bytes >= BYTES_PER_BLOCK {
        sched_wake_task(ICM20948_WAKE.ptr());
    } else {
        ic.flags &= !AX_PENDING;
        ic20948_reschedule_timer(ic)?;
    }
    Ok(())
}

pub fn command_query_icm20948(args: &Args) -> R {
    let ic = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Icm20948) };

    sched_del_timer(&mut ic.timer);
    ic.flags = 0;
    if args[1] == 0 {
        // End measurements

        // Uncomment and rebuilt to check for FIFO overruns when tuning
        //output("mpu9240 fifo_max=%u", ic->fifo_max);
        return Ok(());
    }

    // Start new measurements query
    ic.rest_ticks = args[1];
    sensor_bulk_reset(&mut ic.sb);
    ic.fifo_max = 0;
    ic.fifo_pkts_bytes = 0;
    ic20948_reschedule_timer(ic)
}
decl_command!(command_query_icm20948, "query_icm20948 oid=%c rest_ticks=%u");

pub fn command_query_icm20948_status(args: &Args) -> R {
    let ic = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Icm20948) };

    // Detect if a FIFO overrun occurred
    let int_reg = [AR_INT_STATUS];
    let mut int_msg = [0u8];
    read_mpu(ic.i2c, &int_reg, &mut int_msg)?;
    if int_msg[0] & FIFO_OVERFLOW_INT != 0 {
        ic.sb.possible_overflows = ic.sb.possible_overflows.wrapping_add(1);
    }

    // Read latest FIFO count (with precise timing)
    let reg = [AR_FIFO_COUNT_H];
    let mut msg = [0u8; 2];
    let time1 = timer_read_time();
    read_mpu(ic.i2c, &reg, &mut msg)?;
    let time2 = timer_read_time();
    let fifo_bytes = (((msg[0] & 0x1f) as u16) << 8) | msg[1] as u16;

    // Report status
    sensor_bulk_status(&ic.sb, args[0] as u8, time1, time2.wrapping_sub(time1), fifo_bytes as u32)
}
decl_command!(command_query_icm20948_status, "query_icm20948_status oid=%c");

pub fn icm20948_task() -> R {
    if !sched_check_wake(ICM20948_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let ic = unsafe { &mut *(p as *mut Icm20948) };
        let flags = ic.flags;
        if flags & AX_PENDING != 0 {
            ic20948_query(ic, oid)?;
        }
    }
    Ok(())
}
decl_task!(icm20948_task);
