// Communicating with a PCA9685 pwm device via linux i2c
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::{report_errno, set_non_blocking};
use super::i2c::I2C_SLAVE;
use crate::basecmd::{
    foreach_oid, move_alloc, move_free, move_queue_clear, move_queue_empty, move_queue_first, move_queue_pop, move_queue_push,
    move_queue_setup, oid_alloc, oid_lookup, MoveNode, MoveQueueHead, OidType,
};
use crate::board::{irq_disable, irq_enable, timer_from_us, timer_is_before};
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_del_timer, Timer, TimerFunc, R, SF_DONE, SF_RESCHEDULE};
use crate::{decl_command, decl_constant, decl_shutdown, shutdown, try_shutdown};
use core::mem::size_of;
use std::ffi::CString;

/****************************************************************
 * Low-level i2c
 ****************************************************************/

const P9_MODE1: u8 = 0x00;
const P9_PRESCALE: u8 = 0xfe;
const CHANNEL_MAX: u8 = 15;
const CHANNEL_ALL: u8 = 61;
const VALUE_MAX: u16 = 4096;

const OSC_MHZ: u32 = 25;
fn clock_mhz() -> u32 {
    timer_from_us(1)
}

#[derive(Clone, Copy)]
#[repr(C)]
struct I2cS {
    bus: u8,
    addr: u8,
    cycle_ticks: u32,
    fd: libc::c_int,
}

static DEVICES: Global<[I2cS; 16]> = Global::new([I2cS { bus: 0, addr: 0, cycle_ticks: 0, fd: 0 }; 16]);
static DEVICES_COUNT: Global<usize> = Global::new(0);

fn pca9685_write(fd: libc::c_int, channel: u8, value: u16) -> R {
    let mut value = value;
    let mut full_on: u8 = 0x00;
    if value >= VALUE_MAX {
        full_on = 0x10;
        value = 0x0000;
    } else if value == 0 {
        value = 0x1000;
    }
    let msg: [u8; 5] = [channel.wrapping_mul(4).wrapping_add(0x06), 0x00, full_on, value as u8, (value >> 8) as u8];
    let ret = unsafe { libc::write(fd, msg.as_ptr() as *const libc::c_void, msg.len()) };
    if ret < 0 {
        report_errno("write value i2c", ret as i32);
        try_shutdown!("Unable to update PCA9685 value");
    }
    Ok(())
}

fn write_msg(fd: libc::c_int, msg: &[u8]) -> isize {
    unsafe { libc::write(fd, msg.as_ptr() as *const libc::c_void, msg.len()) }
}

fn open_i2c(bus: u8, addr: u8, cycle_ticks: u32) -> R<libc::c_int> {
    // Find existing device (if already opened)
    let devices = DEVICES.get();
    let count = DEVICES_COUNT.get();
    for d in &devices[..*count] {
        if d.bus == bus && d.addr == addr {
            if cycle_ticks != d.cycle_ticks {
                shutdown!("All PCA9685 channels must have the same cycle_ticks");
            }
            return Ok(d.fd);
        }
    }

    // Setup new I2C device
    if *count >= devices.len() {
        shutdown!("Too many i2c devices");
    }
    let fname = CString::new(format!("/dev/i2c-{}", bus as i32)).unwrap();
    let fd = unsafe { libc::open(fname.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
    'fail: {
        if fd < 0 {
            report_errno("open i2c", fd);
            break 'fail;
        }
        let ret = unsafe { libc::ioctl(fd, I2C_SLAVE as _, addr as libc::c_ulong) };
        if ret < 0 {
            report_errno("ioctl i2c", fd);
            break 'fail;
        }
        if set_non_blocking(fd) < 0 {
            break 'fail;
        }

        // Init PCA9685
        let ret = write_msg(fd, &[P9_MODE1, 0x31]);
        if ret < 0 {
            report_errno("write sleep i2c", ret as i32);
            break 'fail;
        }
        let d = 4096u32.wrapping_mul(clock_mhz());
        let freq = (OSC_MHZ.wrapping_mul(cycle_ticks).wrapping_add(d / 2) / d).wrapping_sub(1);
        let freq = if freq > 0xff { 0xff } else if freq < 0x03 { 0x03 } else { freq };
        let ret = write_msg(fd, &[P9_PRESCALE, freq as u8]);
        if ret < 0 {
            report_errno("write freq i2c", ret as i32);
            break 'fail;
        }
        let ret = write_msg(fd, &[P9_MODE1, 0x21]);
        if ret < 0 {
            report_errno("write unsleep i2c", ret as i32);
            break 'fail;
        }
        unsafe { libc::usleep(500) };
        pca9685_write(fd, CHANNEL_ALL, 0)?;

        devices[*count] = I2cS { bus, addr, cycle_ticks, fd };
        *count += 1;
        return Ok(fd);
    }
    if fd >= 0 {
        unsafe { libc::close(fd) };
    }
    shutdown!("Unable to open and init PCA9685 device");
}

/****************************************************************
 * Command interface
 ****************************************************************/

#[repr(C)]
struct I2cPwm {
    timer: Timer,
    fd: libc::c_int,
    channel: u8,
    default_value: u16,
    max_duration: u32,
    mq: MoveQueueHead,
}

#[repr(C)]
struct Pca9685Move {
    node: MoveNode,
    waketime: u32,
    value: u16,
}

decl_constant!("PCA9685_MAX", VALUE_MAX);

fn oid_type() -> OidType {
    command_config_pca9685 as *const ()
}

unsafe fn pca9685_end_event(_timer: *mut Timer) -> R<u8> {
    shutdown!("Missed scheduling of next pca9685 event");
}

unsafe fn pca9685_event(timer: *mut Timer) -> R<u8> {
    // Apply next update and remove it from queue
    let p = &mut *(timer as *mut I2cPwm);
    let mn = move_queue_pop(&mut p.mq);
    let m = mn as *mut Pca9685Move;
    let value = (*m).value;
    pca9685_write(p.fd, p.channel, value)?;
    move_free(m as *mut u8);

    // Check if more updates queued
    if move_queue_empty(&p.mq) {
        if value == p.default_value || p.max_duration == 0 {
            return Ok(SF_DONE);
        }

        // Start the safety timeout
        p.timer.waketime = p.timer.waketime.wrapping_add(p.max_duration);
        p.timer.func = Some(pca9685_end_event);
        return Ok(SF_RESCHEDULE);
    }

    // Schedule next update
    let nn = move_queue_first(&p.mq);
    let wake = (*(nn as *mut Pca9685Move)).waketime;
    if value != p.default_value && p.max_duration != 0 && timer_is_before(p.timer.waketime.wrapping_add(p.max_duration), wake) {
        shutdown!("Scheduled pca9685 event will exceed max_duration");
    }
    p.timer.waketime = wake;
    Ok(SF_RESCHEDULE)
}

pub fn command_config_pca9685(args: &[u32]) -> R {
    let (bus, addr, channel) = (args[1] as u8, args[2] as u8, args[3] as u8);
    let (value, default_value) = (args[5] as u16, args[6] as u16);
    if channel > CHANNEL_MAX || value > VALUE_MAX || default_value > VALUE_MAX {
        shutdown!("Invalid pca9685 channel or value");
    }
    let fd = open_i2c(bus, addr, args[4])?;
    if value != 0 {
        pca9685_write(fd, channel, value)?;
    }
    let p = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<I2cPwm>() as u16)? as *mut I2cPwm) };
    p.fd = fd;
    p.channel = channel;
    p.default_value = default_value;
    p.max_duration = args[7];
    p.timer.func = Some(pca9685_event);
    move_queue_setup(&mut p.mq, size_of::<Pca9685Move>())
}
decl_command!(
    command_config_pca9685,
    concat!("config_pca9685 oid=%c bus=%c addr=%c", " channel=%c cycle_ticks=%u value=%hu", " default_value=%hu max_duration=%u")
);

fn is_func(f: Option<TimerFunc>, g: TimerFunc) -> bool {
    matches!(f, Some(f) if f as usize == g as usize)
}

pub fn command_queue_pca9685_out(args: &[u32]) -> R {
    let p = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut I2cPwm) };
    let m = unsafe { &mut *(move_alloc()? as *mut Pca9685Move) };
    m.waketime = args[1];
    m.value = args[2] as u16;
    if m.value > VALUE_MAX {
        shutdown!("Invalid pca9685 value");
    }

    irq_disable();
    let need_add_timer = move_queue_push(&mut m.node, &mut p.mq);
    irq_enable();
    if !need_add_timer {
        return Ok(());
    }

    // queue was empty and a timer needs to be added
    sched_del_timer(&mut p.timer);
    if is_func(p.timer.func, pca9685_end_event) && timer_is_before(p.timer.waketime, m.waketime) {
        shutdown!("Scheduled pca9685 event will exceed max_duration");
    }
    p.timer.func = Some(pca9685_event);
    p.timer.waketime = m.waketime;
    sched_add_timer(&mut p.timer)
}
decl_command!(command_queue_pca9685_out, "queue_pca9685_out oid=%c clock=%u value=%hu");

pub fn command_set_pca9685_out(args: &[u32]) -> R {
    let (bus, addr, channel) = (args[0] as u8, args[1] as u8, args[2] as u8);
    let value = args[4] as u16;
    if channel > CHANNEL_MAX || value > VALUE_MAX {
        shutdown!("Invalid pca9685 channel or value");
    }
    let fd = open_i2c(bus, addr, args[3])?;
    pca9685_write(fd, channel, value)
}
decl_command!(
    command_set_pca9685_out,
    concat!("set_pca9685_out bus=%c addr=%c", " channel=%c cycle_ticks=%u value=%hu")
);

pub fn pca9685_shutdown() -> R {
    let devices = DEVICES.get();
    for d in &devices[..*DEVICES_COUNT.get()] {
        pca9685_write(d.fd, CHANNEL_ALL, 0)?;
    }
    for q in foreach_oid(oid_type()) {
        let p = unsafe { &mut *(q as *mut I2cPwm) };
        if p.default_value != 0 {
            pca9685_write(p.fd, p.channel, p.default_value)?;
        }
        p.timer.func = Some(pca9685_event);
        move_queue_clear(&mut p.mq);
    }
    Ok(())
}
decl_shutdown!(pca9685_shutdown);
