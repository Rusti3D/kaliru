// Commands for sending messages on an I2C bus
//
// Copyright (C) 2018  Florian Heilmann <Florian.Heilmann@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::I2cConfig;
use crate::command::{command_decode_buffer, command_decode_ptr};
#[cfg(CONFIG_WANT_SOFTWARE_I2C)]
use crate::i2c_software::I2cSoftware;
use crate::sched::R;
use crate::{decl_command, sendf, shutdown};
use core::mem::size_of;

// I2C ERROR Codes
pub const I2C_BUS_SUCCESS: i32 = 0;
pub const I2C_BUS_NACK: i32 = 1;
pub const I2C_BUS_TIMEOUT: i32 = 2;
pub const I2C_BUS_START_NACK: i32 = 3;
pub const I2C_BUS_START_READ_NACK: i32 = 4;

#[cfg(not(CONFIG_WANT_SOFTWARE_I2C))]
pub enum I2cSoftware {}

#[repr(C)]
#[derive(Clone, Copy)]
union I2cBus {
    i2c_hw: I2cConfig,
    i2c_sw: *mut I2cSoftware,
}

#[repr(C)]
pub struct I2cDev {
    bus: I2cBus,
    flags: u8,
}

const IF_SOFTWARE: u8 = 1;
const IF_HARDWARE: u8 = 2;

// i2c_write / i2c_read: the board's, or (no hardware I2C) none
#[cfg(CONFIG_WANT_GPIO_I2C)]
use crate::board::{i2c_read, i2c_setup, i2c_write};
#[cfg(not(CONFIG_WANT_GPIO_I2C))]
fn i2c_write(_i2c: I2cConfig, _data: &[u8]) -> R<i32> {
    Ok(0)
}
#[cfg(not(CONFIG_WANT_GPIO_I2C))]
fn i2c_read(_i2c: I2cConfig, _reg: &[u8], _read: &mut [u8]) -> R<i32> {
    Ok(0)
}

fn oid_type() -> OidType {
    command_config_i2c as *const ()
}

pub fn command_config_i2c(args: &[u32]) -> R {
    oid_alloc(args[0] as u8, oid_type(), size_of::<I2cDev>() as u16)?;
    Ok(())
}
decl_command!(command_config_i2c, "config_i2c oid=%c");

pub fn i2cdev_oid_lookup(oid: u8) -> R<*mut I2cDev> {
    Ok(oid_lookup(oid, oid_type())? as *mut I2cDev)
}

#[cfg(CONFIG_WANT_GPIO_I2C)]
pub fn command_i2c_set_bus(args: &[u32]) -> R {
    let addr = (args[3] & 0x7f) as u8;
    let i2c = unsafe { &mut *i2cdev_oid_lookup(args[0] as u8)? };
    i2c.bus.i2c_hw = i2c_setup(args[1], args[2], addr)?;
    i2c.flags |= IF_HARDWARE;
    Ok(())
}
#[cfg(not(CONFIG_WANT_GPIO_I2C))]
pub fn command_i2c_set_bus(_args: &[u32]) -> R {
    shutdown!("hardware I2C is not supported/enabled");
}
decl_command!(command_i2c_set_bus, "i2c_set_bus oid=%c i2c_bus=%u rate=%u address=%u");

#[cfg(CONFIG_WANT_SOFTWARE_I2C)]
pub fn i2cdev_set_software_bus(i2c: *mut I2cDev, is: *mut I2cSoftware) -> R {
    let i2c = unsafe { &mut *i2c };
    i2c.bus.i2c_sw = is;
    i2c.flags |= IF_SOFTWARE;
    Ok(())
}
#[cfg(not(CONFIG_WANT_SOFTWARE_I2C))]
pub fn i2cdev_set_software_bus(_i2c: *mut I2cDev, _is: *mut I2cSoftware) -> R {
    shutdown!("software I2C is not supported/enabled");
}

pub fn i2c_shutdown_on_err(ret: i32) -> R {
    match ret {
        I2C_BUS_NACK => shutdown!("I2C NACK"),
        I2C_BUS_START_NACK => shutdown!("I2C START NACK"),
        I2C_BUS_START_READ_NACK => shutdown!("I2C START READ NACK"),
        I2C_BUS_TIMEOUT => shutdown!("I2C Timeout"),
        _ => Ok(()),
    }
}

#[cfg(CONFIG_WANT_SOFTWARE_I2C)]
pub fn i2c_dev_write(i2c: *mut I2cDev, data: &[u8]) -> R<i32> {
    let i2c = unsafe { &*i2c };
    if i2c.flags & IF_SOFTWARE != 0 {
        crate::i2c_software::i2c_software_write(unsafe { i2c.bus.i2c_sw }, data)
    } else {
        i2c_write(unsafe { i2c.bus.i2c_hw }, data)
    }
}
#[cfg(not(CONFIG_WANT_SOFTWARE_I2C))]
pub fn i2c_dev_write(i2c: *mut I2cDev, data: &[u8]) -> R<i32> {
    i2c_write(unsafe { (*i2c).bus.i2c_hw }, data)
}

pub fn command_i2c_write(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let i2c = oid_lookup(oid, oid_type())? as *mut I2cDev;
    let data = command_decode_buffer(args[1] as u8 as u32, args[2]);
    let ret = i2c_dev_write(i2c, data)?;
    i2c_shutdown_on_err(ret)
}
decl_command!(command_i2c_write, "i2c_write oid=%c data=%*s");

#[cfg(CONFIG_WANT_SOFTWARE_I2C)]
pub fn i2c_dev_read(i2c: *mut I2cDev, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let i2c = unsafe { &*i2c };
    if i2c.flags & IF_SOFTWARE != 0 {
        crate::i2c_software::i2c_software_read(unsafe { i2c.bus.i2c_sw }, reg, read)
    } else {
        i2c_read(unsafe { i2c.bus.i2c_hw }, reg, read)
    }
}
#[cfg(not(CONFIG_WANT_SOFTWARE_I2C))]
pub fn i2c_dev_read(i2c: *mut I2cDev, reg: &[u8], read: &mut [u8]) -> R<i32> {
    i2c_read(unsafe { (*i2c).bus.i2c_hw }, reg, read)
}

pub fn command_i2c_read(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let i2c = oid_lookup(oid, oid_type())? as *mut I2cDev;
    let reg_len = args[1] as u8 as usize;
    let reg = unsafe { core::slice::from_raw_parts(command_decode_ptr(args[2]), reg_len) };
    let data_len = args[3] as u8 as usize;
    let mut buf = [0u8; 256];
    let data = &mut buf[..data_len];
    let ret = i2c_dev_read(i2c, reg, data)?;
    i2c_shutdown_on_err(ret)?;
    sendf!("i2c_read_response oid=%c response=%*s", oid, &*data)
}
decl_command!(command_i2c_read, "i2c_read oid=%c reg=%*s read_len=%u");
