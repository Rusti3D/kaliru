// Linux i2c implementation
//
// Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::{report_errno, set_non_blocking};
use crate::ctr::Global;
use crate::i2ccmds::{I2C_BUS_NACK, I2C_BUS_SUCCESS};
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown};
use std::ffi::CString;

decl_enumeration_range!("i2c_bus", "i2c.0", 0, 15);

// <linux/i2c-dev.h>, <linux/i2c.h>
pub const I2C_SLAVE: libc::c_ulong = 0x0703;
pub const I2C_FUNCS: libc::c_ulong = 0x0705;
pub const I2C_RDWR: libc::c_ulong = 0x0707;
pub const I2C_FUNC_I2C: libc::c_ulong = 0x00000001;
pub const I2C_M_RD: u16 = 0x0001;

#[repr(C)]
#[derive(Clone, Copy)]
struct I2cMsg {
    addr: u16,
    flags: u16,
    len: u16,
    buf: *mut u8,
}

#[repr(C)]
struct I2cRdwrIoctlData {
    msgs: *mut I2cMsg,
    nmsgs: u32,
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct I2cConfig {
    fd: libc::c_int,
    addr: u8,
}

#[derive(Clone, Copy)]
#[repr(C)]
struct I2cS {
    bus: u32,
    addr: u8,
    fd: libc::c_int,
}

static DEVICES: Global<[I2cS; 16]> = Global::new([I2cS { bus: 0, addr: 0, fd: 0 }; 16]);
static DEVICES_COUNT: Global<usize> = Global::new(0);

fn i2c_open(bus: u32, addr: u8) -> R<libc::c_int> {
    // Find existing device (if already opened)
    let devices = DEVICES.get();
    let count = DEVICES_COUNT.get();
    for d in &devices[..*count] {
        if d.bus == bus && d.addr == addr {
            return Ok(d.fd);
        }
    }

    let fname = CString::new(format!("/dev/i2c-{}", bus as i32)).unwrap();
    let fd = unsafe { libc::open(fname.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
    'fail: {
        if fd < 0 {
            report_errno("open i2c", fd);
            break 'fail;
        }
        // Test for I2C_RDWR support
        let mut i2c_funcs: libc::c_ulong = 0; // datatype from ioctl spec.
        unsafe { libc::ioctl(fd, I2C_FUNCS as _, &mut i2c_funcs) };
        if i2c_funcs & I2C_FUNC_I2C == 0 {
            report_errno("i2c does not support I2C_RDWR", fd);
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

        devices[*count] = I2cS { bus, addr, fd };
        *count += 1;

        return Ok(fd);
    }
    if fd >= 0 {
        unsafe { libc::close(fd) };
    }
    shutdown!("Unable to open i2c device");
}

pub fn i2c_setup(bus: u32, _rate: u32, addr: u8) -> R<I2cConfig> {
    // Note:  The rate is set by the kernel driver, for a Raspberry Pi this
    // is done with the following setting in /boot/config.txt:
    //
    // dtparam=i2c_baudrate=<rate>

    let fd = i2c_open(bus, addr)?;
    Ok(I2cConfig { fd, addr })
}

pub fn i2c_write(config: I2cConfig, data: &[u8]) -> R<i32> {
    let mut msgs = [I2cMsg { addr: config.addr as u16, flags: 0x0, len: data.len() as u16, buf: data.as_ptr() as *mut u8 }];
    let mut i2c_data = I2cRdwrIoctlData { msgs: msgs.as_mut_ptr(), nmsgs: 1 };

    let ret = unsafe { libc::ioctl(config.fd, I2C_RDWR as _, &mut i2c_data) };

    if ret < 0 {
        return Ok(I2C_BUS_NACK);
    }

    Ok(I2C_BUS_SUCCESS)
}

pub fn i2c_read(config: I2cConfig, reg: &[u8], data: &mut [u8]) -> R<i32> {
    let mut msgs = [I2cMsg { addr: 0, flags: 0, len: 0, buf: core::ptr::null_mut() }; 2];
    msgs[1] = I2cMsg { addr: config.addr as u16, flags: I2C_M_RD, len: data.len() as u16, buf: data.as_mut_ptr() };
    let mut i2c_data = I2cRdwrIoctlData { msgs: core::ptr::null_mut(), nmsgs: 0 };

    if !reg.is_empty() {
        msgs[0] = I2cMsg { addr: config.addr as u16, flags: 0x0, len: reg.len() as u16, buf: reg.as_ptr() as *mut u8 };
        i2c_data.nmsgs = 2;
        i2c_data.msgs = msgs.as_mut_ptr();
    } else {
        i2c_data.nmsgs = 1;
        i2c_data.msgs = unsafe { msgs.as_mut_ptr().add(1) };
    }

    let ret = unsafe { libc::ioctl(config.fd, I2C_RDWR as _, &mut i2c_data) };

    if ret < 0 {
        return Ok(I2C_BUS_NACK);
    }

    Ok(I2C_BUS_SUCCESS)
}
