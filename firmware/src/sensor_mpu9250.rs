// Support for gathering acceleration data from mpu9250 chip
//
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
const AR_FIFO_COUNT_H: u8 = 0x72;
const AR_FIFO: u8 = 0x74;
const AR_INT_STATUS: u8 = 0x3A;

const FIFO_OVERFLOW_INT: u8 = 0x10;

const BYTES_PER_BLOCK: u16 = 48;

#[repr(C)]
struct Mpu9250 {
    timer: Timer,
    rest_ticks: u32,
    i2c: *mut I2cDev,
    fifo_max: u16,
    fifo_pkts_bytes: u16,
    flags: u8,
    sb: SensorBulk,
}

const AX_PENDING: u8 = 1 << 0;

static MPU9250_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_mpu9250 as *const ()
}

// Event handler that wakes mpu9250_task() periodically
unsafe fn mpu9250_event(timer: *mut Timer) -> R<u8> {
    let ax = &mut *(timer as *mut Mpu9250);
    ax.flags |= AX_PENDING;
    sched_wake_task(MPU9250_WAKE.ptr());
    Ok(SF_DONE)
}

pub fn command_config_mpu9250(args: &Args) -> R {
    let mp = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Mpu9250>() as u16)? as *mut Mpu9250) };
    mp.timer.func = Some(mpu9250_event);
    mp.i2c = i2cdev_oid_lookup(args[1] as u8)?;
    Ok(())
}
decl_command!(command_config_mpu9250, "config_mpu9250 oid=%c i2c_oid=%c");

// Helper code to reschedule the mpu9250_event() timer
fn mp9250_reschedule_timer(mp: &mut Mpu9250) -> R {
    irq_disable();
    mp.timer.waketime = timer_read_time().wrapping_add(mp.rest_ticks);
    sched_add_timer(&mut mp.timer)?;
    irq_enable();
    Ok(())
}

fn read_mpu(i2c: *mut I2cDev, reg: &[u8], read: &mut [u8]) -> R {
    let ret = i2c_dev_read(i2c, reg, read)?;
    i2c_shutdown_on_err(ret)
}

// Reads the fifo byte count from the device.
fn get_fifo_status(mp: &mut Mpu9250) -> R<u16> {
    let reg = [AR_FIFO_COUNT_H];
    let mut msg = [0u8; 2];
    read_mpu(mp.i2c, &reg, &mut msg)?;
    let fifo_bytes = (((msg[0] & 0x1f) as u16) << 8) | msg[1] as u16;
    if fifo_bytes > mp.fifo_max {
        mp.fifo_max = fifo_bytes;
    }
    Ok(fifo_bytes)
}

// Query accelerometer data
fn mp9250_query(mp: &mut Mpu9250, oid: u8) -> R {
    // If not enough bytes to fill report read MPU FIFO's fill
    if mp.fifo_pkts_bytes < BYTES_PER_BLOCK {
        mp.fifo_pkts_bytes = get_fifo_status(mp)?;
    }

    // If we have enough bytes to fill the buffer do it and send report
    if mp.fifo_pkts_bytes >= BYTES_PER_BLOCK {
        let reg = [AR_FIFO];
        read_mpu(mp.i2c, &reg, &mut mp.sb.data[..BYTES_PER_BLOCK as usize])?;
        mp.sb.data_count = BYTES_PER_BLOCK as u8;
        mp.fifo_pkts_bytes -= BYTES_PER_BLOCK;
        sensor_bulk_report(&mut mp.sb, oid)?;
    }

    // If we have enough bytes remaining to fill another report wake again
    //  otherwise schedule timed wakeup
    if mp.fifo_pkts_bytes >= BYTES_PER_BLOCK {
        sched_wake_task(MPU9250_WAKE.ptr());
    } else {
        mp.flags &= !AX_PENDING;
        mp9250_reschedule_timer(mp)?;
    }
    Ok(())
}

pub fn command_query_mpu9250(args: &Args) -> R {
    let mp = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Mpu9250) };

    sched_del_timer(&mut mp.timer);
    mp.flags = 0;
    if args[1] == 0 {
        // End measurements

        // Uncomment and rebuilt to check for FIFO overruns when tuning
        //output("mpu9240 fifo_max=%u", mp->fifo_max);
        return Ok(());
    }

    // Start new measurements query
    mp.rest_ticks = args[1];
    sensor_bulk_reset(&mut mp.sb);
    mp.fifo_max = 0;
    mp.fifo_pkts_bytes = 0;
    mp9250_reschedule_timer(mp)
}
decl_command!(command_query_mpu9250, "query_mpu9250 oid=%c rest_ticks=%u");

pub fn command_query_mpu9250_status(args: &Args) -> R {
    let mp = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Mpu9250) };

    // Detect if a FIFO overrun occurred
    let int_reg = [AR_INT_STATUS];
    let mut int_msg = [0u8];
    read_mpu(mp.i2c, &int_reg, &mut int_msg)?;
    if int_msg[0] & FIFO_OVERFLOW_INT != 0 {
        mp.sb.possible_overflows = mp.sb.possible_overflows.wrapping_add(1);
    }

    // Read latest FIFO count (with precise timing)
    let reg = [AR_FIFO_COUNT_H];
    let mut msg = [0u8; 2];
    let time1 = timer_read_time();
    read_mpu(mp.i2c, &reg, &mut msg)?;
    let time2 = timer_read_time();
    let fifo_bytes = (((msg[0] & 0x1f) as u16) << 8) | msg[1] as u16;

    // Report status
    sensor_bulk_status(&mp.sb, args[0] as u8, time1, time2.wrapping_sub(time1), fifo_bytes as u32)
}
decl_command!(command_query_mpu9250_status, "query_mpu9250_status oid=%c");

pub fn mpu9250_task() -> R {
    if !sched_check_wake(MPU9250_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let mp = unsafe { &mut *(p as *mut Mpu9250) };
        let flags = mp.flags;
        if flags & AX_PENDING != 0 {
            mp9250_query(mp, oid)?;
        }
    }
    Ok(())
}
decl_task!(mpu9250_task);
