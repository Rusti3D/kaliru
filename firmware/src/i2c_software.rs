// Software I2C emulation
//
// Copyright (C) 2023-2025  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2023  Alan.Ma <tech@biqu3d.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::alloc_chunk;
use crate::board::{gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, timer_is_before, timer_read_time, GpioIn, GpioOut};
use crate::command::Args;
use crate::decl_command;
use crate::i2ccmds::{i2cdev_oid_lookup, i2cdev_set_software_bus, I2C_BUS_NACK, I2C_BUS_START_NACK, I2C_BUS_START_READ_NACK, I2C_BUS_SUCCESS};
use crate::sched::R;
use core::mem::size_of;

#[repr(C)]
pub struct I2cSoftware {
    scl_out: GpioOut,
    sda_out: GpioOut,
    scl_in: GpioIn,
    sda_in: GpioIn,
    addr: u8,
    ticks: u32,
}

pub fn command_i2c_set_sw_bus(args: &Args) -> R {
    let i2c = i2cdev_oid_lookup(args[0] as u8)?;
    let is = alloc_chunk(size_of::<I2cSoftware>())? as *mut I2cSoftware;
    let s = unsafe { &mut *is };
    s.ticks = args[3];
    s.addr = ((args[4] & 0x7f) << 1) as u8; // address format shifted
    s.scl_out = gpio_out_setup(args[1], 1)?;
    s.scl_in = gpio_in_setup(args[1], 1)?;
    s.sda_out = gpio_out_setup(args[2], 1)?;
    s.sda_in = gpio_in_setup(args[2], 1)?;
    i2cdev_set_software_bus(i2c, is)
}
decl_command!(
    command_i2c_set_sw_bus,
    concat!("i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u", " pulse_ticks=%u address=%u")
);

fn i2c_delay(ticks: u32) {
    let end = timer_read_time().wrapping_add(ticks);
    while timer_is_before(timer_read_time(), end) {}
}


fn i2c_software_send_ack(is: &I2cSoftware, ack: bool) -> R {
    if !ack {
        gpio_out_reset(is.sda_out, 0)?;
    }
    i2c_delay(is.ticks);
    gpio_in_reset(is.scl_in, 1)?;
    i2c_delay(is.ticks);
    gpio_out_reset(is.scl_out, 0)?;
    Ok(())
}

fn i2c_software_read_ack(is: &I2cSoftware, state: u8) -> R<u8> {
    if state == 0 {
        gpio_in_reset(is.sda_in, 1)?;
    }
    i2c_delay(is.ticks);
    gpio_in_reset(is.scl_in, 1)?;
    let nack = gpio_in_read(is.sda_in);
    i2c_delay(is.ticks);
    gpio_out_reset(is.scl_out, 0)?;
    Ok(nack)
}

fn i2c_software_send_byte(is: &I2cSoftware, mut b: u8) -> R<i32> {
    let mut state = 2;
    for _ in 0..8 {
        if b & 0x80 != 0 {
            if state != 1 {
                gpio_in_reset(is.sda_in, 1)?;
            }
            state = 1;
        } else {
            if state > 0 {
                gpio_out_reset(is.sda_out, 0)?;
            }
            state = 0;
        }
        b <<= 1;
        i2c_delay(is.ticks);
        gpio_in_reset(is.scl_in, 1)?;
        i2c_delay(is.ticks);
        gpio_out_reset(is.scl_out, 0)?;
    }

    if i2c_software_read_ack(is, state)? != 0 {
        return Ok(I2C_BUS_NACK);
    }

    Ok(I2C_BUS_SUCCESS)
}

fn i2c_software_read_byte(is: &I2cSoftware, remaining: usize) -> R<u8> {
    let mut b: u8 = 0;
    gpio_in_reset(is.sda_in, 1)?;
    for _ in 0..8 {
        i2c_delay(is.ticks);
        gpio_in_reset(is.scl_in, 1)?;
        i2c_delay(is.ticks);
        b <<= 1;
        b |= gpio_in_read(is.sda_in);
        gpio_out_reset(is.scl_out, 0)?;
    }
    i2c_software_send_ack(is, remaining == 0)?;
    Ok(b)
}

fn i2c_software_start(is: &I2cSoftware, addr: u8) -> R<i32> {
    i2c_delay(is.ticks);
    gpio_in_reset(is.sda_in, 1)?;
    gpio_in_reset(is.scl_in, 1)?;
    i2c_delay(is.ticks);
    gpio_out_reset(is.sda_out, 0)?;
    i2c_delay(is.ticks);
    gpio_out_reset(is.scl_out, 0)?;

    let ret = i2c_software_send_byte(is, addr)?;
    if ret == I2C_BUS_NACK {
        return Ok(I2C_BUS_START_NACK);
    }
    Ok(ret)
}

fn i2c_software_stop(is: &I2cSoftware) -> R {
    gpio_out_reset(is.sda_out, 0)?;
    i2c_delay(is.ticks);
    gpio_in_reset(is.scl_in, 1)?;
    i2c_delay(is.ticks);
    gpio_in_reset(is.sda_in, 1)?;
    Ok(())
}

pub fn i2c_software_write(is: *mut I2cSoftware, write: &[u8]) -> R<i32> {
    let is = unsafe { &*is };
    let mut ret = i2c_software_start(is, is.addr)?;
    if ret == I2C_BUS_SUCCESS {
        for &b in write {
            ret = i2c_software_send_byte(is, b)?;
            if ret != I2C_BUS_SUCCESS {
                break;
            }
        }
    }
    i2c_software_stop(is)?;
    Ok(ret)
}

pub fn i2c_software_read(is: *mut I2cSoftware, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let is = unsafe { &*is };
    let mut ret;
    'out: {
        if !reg.is_empty() {
            // write the register
            ret = i2c_software_start(is, is.addr)?;
            if ret != I2C_BUS_SUCCESS {
                break 'out;
            }
            for &b in reg {
                ret = i2c_software_send_byte(is, b)?;
                if ret != I2C_BUS_SUCCESS {
                    break 'out;
                }
            }
        }
        // start/re-start and read data
        ret = i2c_software_start(is, is.addr | 0x01)?;
        if ret != I2C_BUS_SUCCESS {
            ret = I2C_BUS_START_READ_NACK;
            break 'out;
        }
        let n = read.len();
        for (i, r) in read.iter_mut().enumerate() {
            *r = i2c_software_read_byte(is, n - 1 - i)?;
        }
    }
    i2c_software_stop(is)?;
    Ok(ret)
}
