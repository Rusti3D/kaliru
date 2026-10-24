// Support for eddy current sensor data from ldc1612 chip
//
// Copyright (C) 2023 Alan.Ma <tech@biqu3d.com>
// Copyright (C) 2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_in_read, gpio_in_setup, irq_disable, irq_enable, timer_is_before, timer_read_time, GpioIn};
use crate::command::Args;
use crate::ctr::Global;
use crate::i2ccmds::{i2c_dev_read, i2c_shutdown_on_err, i2cdev_oid_lookup, I2cDev};
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, sensor_bulk_status, SensorBulk};
use crate::trsync::{trsync_do_trigger, trsync_oid_lookup, Trsync};
use crate::{decl_command, decl_task, sendf};
use core::mem::size_of;
use core::ptr::null_mut;

const LDC_PENDING: u8 = 1 << 0;
const LDC_HAVE_INTB: u8 = 1 << 1;
const LH_AWAIT_HOMING: u8 = 1 << 1;
const LH_CAN_TRIGGER: u8 = 1 << 2;

#[repr(C)]
struct Ldc1612 {
    timer: Timer,
    rest_ticks: u32,
    i2c: *mut I2cDev,
    flags: u8,
    sb: SensorBulk,
    intb_pin: GpioIn,
    // homing
    ts: *mut Trsync,
    homing_flags: u8,
    trigger_reason: u8,
    error_reason: u8,
    trigger_threshold: u32,
    homing_clock: u32,
}

static LDC1612_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_ldc1612 as *const ()
}

fn lookup(oid: u32) -> R<&'static mut Ldc1612> {
    Ok(unsafe { &mut *(oid_lookup(oid as u8, oid_type())? as *mut Ldc1612) })
}

// Check if the intb line is "asserted"
fn check_intb_asserted(ld: &Ldc1612) -> bool {
    gpio_in_read(ld.intb_pin) == 0
}

// Query ldc1612 data
unsafe fn ldc1612_event(timer: *mut Timer) -> R<u8> {
    let ld = &mut *(timer as *mut Ldc1612);
    if ld.flags & LDC_PENDING != 0 {
        ld.sb.possible_overflows = ld.sb.possible_overflows.wrapping_add(1);
    }
    if ld.flags & LDC_HAVE_INTB == 0 || check_intb_asserted(ld) {
        ld.flags |= LDC_PENDING;
        sched_wake_task(LDC1612_WAKE.ptr());
    }
    ld.timer.waketime = ld.timer.waketime.wrapping_add(ld.rest_ticks);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_ldc1612(args: &Args) -> R {
    let ld = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Ldc1612>() as u16)? as *mut Ldc1612) };
    ld.timer.func = Some(ldc1612_event);
    ld.i2c = i2cdev_oid_lookup(args[1] as u8)?;
    Ok(())
}
decl_command!(command_config_ldc1612, "config_ldc1612 oid=%c i2c_oid=%c");

pub fn command_config_ldc1612_with_intb(args: &Args) -> R {
    command_config_ldc1612(args)?;
    let ld = lookup(args[0])?;
    ld.intb_pin = gpio_in_setup(args[2], 1)?;
    ld.flags = LDC_HAVE_INTB;
    Ok(())
}
decl_command!(command_config_ldc1612_with_intb, "config_ldc1612_with_intb oid=%c i2c_oid=%c intb_pin=%c");

pub fn command_ldc1612_setup_home(args: &Args) -> R {
    let ld = lookup(args[0])?;

    ld.trigger_threshold = args[2];
    if ld.trigger_threshold == 0 {
        ld.ts = null_mut();
        ld.homing_flags = 0;
        return Ok(());
    }
    ld.homing_clock = args[1];
    ld.ts = trsync_oid_lookup(args[3] as u8)?;
    ld.trigger_reason = args[4] as u8;
    ld.error_reason = args[5] as u8;
    ld.homing_flags = LH_AWAIT_HOMING | LH_CAN_TRIGGER;
    Ok(())
}
decl_command!(
    command_ldc1612_setup_home,
    concat!("ldc1612_setup_home oid=%c clock=%u threshold=%u", " trsync_oid=%c trigger_reason=%c error_reason=%c")
);

pub fn command_query_ldc1612_home_state(args: &Args) -> R {
    let ld = lookup(args[0])?;
    sendf!(
        "ldc1612_home_state oid=%c homing=%c trigger_clock=%u",
        args[0],
        (ld.homing_flags & LH_CAN_TRIGGER != 0) as u8,
        ld.homing_clock
    )
}
decl_command!(command_query_ldc1612_home_state, "query_ldc1612_home_state oid=%c");

// Check if a sample should trigger a homing event
fn check_home(ld: &mut Ldc1612, data: u32) {
    let mut homing_flags = ld.homing_flags;
    if homing_flags & LH_CAN_TRIGGER == 0 {
        return;
    }
    if data > 0x0fffffff {
        // Sensor reports an issue - cancel homing
        ld.homing_flags = 0;
        trsync_do_trigger(ld.ts, ld.error_reason);
        return;
    }
    let time = timer_read_time();
    if homing_flags & LH_AWAIT_HOMING != 0 && timer_is_before(time, ld.homing_clock) {
        return;
    }
    homing_flags &= !LH_AWAIT_HOMING;
    if data > ld.trigger_threshold {
        homing_flags = 0;
        ld.homing_clock = time;
        trsync_do_trigger(ld.ts, ld.trigger_reason);
    }
    ld.homing_flags = homing_flags;
}

// Chip registers
const REG_DATA0_MSB: u8 = 0x00;
const REG_DATA0_LSB: u8 = 0x01;
const REG_STATUS: u8 = 0x18;

// Read a register on the ldc1612
fn read_reg(i2c: *mut I2cDev, reg: u8, res: &mut [u8]) -> R {
    let ret = i2c_dev_read(i2c, &[reg], &mut res[..2])?;
    i2c_shutdown_on_err(ret)
}

// Read the status register on the ldc1612
fn read_reg_status(ld: &Ldc1612) -> R<u16> {
    let mut data_status = [0u8; 2];
    read_reg(ld.i2c, REG_STATUS, &mut data_status)?;
    Ok(((data_status[0] as u16) << 8) | data_status[1] as u16)
}

const BYTES_PER_SAMPLE: usize = 4;

// Query ldc1612 data
fn ldc1612_query(ld: &mut Ldc1612, oid: u8) -> R {
    // Check if data available (and clear INTB line)
    let status = read_reg_status(ld)?;
    irq_disable();
    ld.flags &= !LDC_PENDING;
    irq_enable();
    if status & 0x08 == 0 {
        return Ok(());
    }

    // Read coil0 frequency
    let dc = ld.sb.data_count as usize;
    let i2c = ld.i2c;
    read_reg(i2c, REG_DATA0_MSB, &mut ld.sb.data[dc..])?;
    read_reg(i2c, REG_DATA0_LSB, &mut ld.sb.data[dc + 2..])?;
    ld.sb.data_count += BYTES_PER_SAMPLE as u8;

    // Check for endstop trigger
    let d = &ld.sb.data[dc..dc + 4];
    let data = u32::from_be_bytes([d[0], d[1], d[2], d[3]]);
    check_home(ld, data);

    // Flush local buffer if needed
    if ld.sb.data_count as usize + BYTES_PER_SAMPLE > ld.sb.data.len() {
        sensor_bulk_report(&mut ld.sb, oid)?;
    }
    Ok(())
}

pub fn command_query_ldc1612(args: &Args) -> R {
    let ld = lookup(args[0])?;

    sched_del_timer(&mut ld.timer);
    ld.flags &= !LDC_PENDING;
    if args[1] == 0 {
        // End measurements
        return Ok(());
    }

    // Start new measurements query
    ld.rest_ticks = args[1];
    sensor_bulk_reset(&mut ld.sb);
    irq_disable();
    ld.timer.waketime = timer_read_time().wrapping_add(ld.rest_ticks);
    sched_add_timer(&mut ld.timer)?;
    irq_enable();
    Ok(())
}
decl_command!(command_query_ldc1612, "query_ldc1612 oid=%c rest_ticks=%u");

pub fn command_query_status_ldc1612(args: &Args) -> R {
    let ld = lookup(args[0])?;

    if ld.flags & LDC_HAVE_INTB != 0 {
        // Check if a sample is pending in the chip via the intb line
        irq_disable();
        let time = timer_read_time();
        let p = check_intb_asserted(ld);
        irq_enable();
        return sensor_bulk_status(&ld.sb, args[0] as u8, time, 0, if p { BYTES_PER_SAMPLE as u32 } else { 0 });
    }

    // Query sensor to see if a sample is pending
    let time1 = timer_read_time();
    let status = read_reg_status(ld)?;
    let time2 = timer_read_time();

    let fifo = if status & 0x08 != 0 { BYTES_PER_SAMPLE as u32 } else { 0 };
    sensor_bulk_status(&ld.sb, args[0] as u8, time1, time2.wrapping_sub(time1), fifo)
}
decl_command!(command_query_status_ldc1612, "query_status_ldc1612 oid=%c");

pub fn ldc1612_task() -> R {
    if !sched_check_wake(LDC1612_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let ld = unsafe { &mut *(p as *mut Ldc1612) };
        let flags = ld.flags;
        if flags & LDC_PENDING == 0 {
            continue;
        }
        ldc1612_query(ld, oid)?;
    }
    Ok(())
}
decl_task!(ldc1612_task);
