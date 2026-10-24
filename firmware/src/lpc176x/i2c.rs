// I2C functions on lpc176x
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::I2cConfig;
use super::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time};
use crate::ctr::Global;
use crate::i2ccmds::I2C_BUS_SUCCESS;
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

struct I2cInfo {
    i2c: *mut LPC_I2C_TypeDef,
    scl_pin: u8,
    sda_pin: u8,
    function: u8,
    pclk: u8,
}
unsafe impl Sync for I2cInfo {}

decl_enumeration!("i2c_bus", "i2c1", 0);
decl_enumeration!("i2c_bus", "i2c1a", 1);
decl_enumeration!("i2c_bus", "i2c0", 2);
decl_enumeration!("i2c_bus", "i2c2", 3);
decl_constant_str!("BUS_PINS_i2c1", "P0.1,P0.0");
decl_constant_str!("BUS_PINS_i2c1a", "P0.20,P0.19");
decl_constant_str!("BUS_PINS_i2c0", "P0.28,P0.27");
decl_constant_str!("BUS_PINS_i2c2", "P0.11,P0.10");

const fn info(i2c: *mut LPC_I2C_TypeDef, scl_pin: u32, sda_pin: u32, function: u8, pclk: u32) -> I2cInfo {
    I2cInfo { i2c, scl_pin: scl_pin as u8, sda_pin: sda_pin as u8, function, pclk: pclk as u8 }
}

static I2C_BUS: [I2cInfo; 4] = [
    info(LPC_I2C1, GPIO(0, 1), GPIO(0, 0), 3, PCLK_I2C1),
    info(LPC_I2C1, GPIO(0, 20), GPIO(0, 19), 3, PCLK_I2C1),
    info(LPC_I2C0, GPIO(0, 28), GPIO(0, 27), 1, PCLK_I2C0),
    info(LPC_I2C2, GPIO(0, 11), GPIO(0, 10), 2, PCLK_I2C2),
];

// i2c connection status flags
const IF_START: u32 = 1 << 5;
const IF_STOP: u32 = 1 << 4;
const IF_IRQ: u32 = 1 << 3;
const IF_ACK: u32 = 1 << 2;
const IF_ENA: u32 = 1 << 6;

static HAVE_RUN_INIT: Global<u8> = Global::new(0);

pub fn i2c_setup(bus: u32, _rate: u32, addr: u8) -> R<I2cConfig> {
    if bus as usize >= I2C_BUS.len() {
        shutdown!("Invalid i2c bus");
    }

    let info = &I2C_BUS[bus as usize];
    let i2c = unsafe { &*info.i2c };
    let have_run_init = HAVE_RUN_INIT.get();
    if *have_run_init & (1 << bus) == 0 {
        *have_run_init |= 1 << bus;

        // Init pins
        gpio_peripheral(info.scl_pin as u32, info.function as u32, 0);
        gpio_peripheral(info.sda_pin as u32, info.function as u32, 0);

        // Set 100Khz frequency
        enable_pclock(info.pclk as u32);
        let pclk = get_pclock_frequency(info.pclk as u32);
        let pulse = pclk / (100000 * 2);
        i2c.I2SCLL.write(pulse);
        i2c.I2SCLH.write(pulse);

        // Enable interface
        i2c.I2CONCLR.write(IF_START | IF_IRQ | IF_ACK | IF_ENA);
        i2c.I2CONSET.write(IF_ENA);
    }

    Ok(I2cConfig { i2c: info.i2c, addr: addr << 1 })
}

fn i2c_wait(i2c: &LPC_I2C_TypeDef, bit: u32, timeout: u32) -> R {
    loop {
        let flags = i2c.I2CONSET.read();
        if flags & bit != 0 {
            break;
        }
        if !timer_is_before(timer_read_time(), timeout) {
            shutdown!("i2c timeout");
        }
    }
    Ok(())
}

fn i2c_start(i2c: &LPC_I2C_TypeDef, timeout: u32) -> R {
    i2c.I2CONCLR.write(IF_ACK | IF_IRQ | IF_START);
    i2c.I2CONSET.write(IF_ACK | IF_START);
    i2c_wait(i2c, IF_IRQ, timeout)?;
    let status = i2c.I2STAT.read();
    if status != 0x10 && status != 0x08 {
        shutdown!("Failed to send i2c start");
    }
    i2c.I2CONCLR.write(IF_START);
    Ok(())
}

fn i2c_send_byte(i2c: &LPC_I2C_TypeDef, b: u8, timeout: u32) -> R<u32> {
    i2c.I2DAT.write(b as u32);
    i2c.I2CONCLR.write(IF_IRQ);
    i2c_wait(i2c, IF_IRQ, timeout)?;
    Ok(i2c.I2STAT.read())
}

fn i2c_read_byte(i2c: &LPC_I2C_TypeDef, timeout: u32, remaining: u8) -> R<u8> {
    if remaining == 0 {
        i2c.I2CONCLR.write(IF_ACK | IF_IRQ);
    } else {
        i2c.I2CONSET.write(IF_ACK);
        i2c.I2CONCLR.write(IF_IRQ);
    }
    i2c_wait(i2c, IF_IRQ, timeout)?;
    Ok(i2c.I2DAT.read() as u8)
}

fn i2c_stop(i2c: &LPC_I2C_TypeDef, timeout: u32) -> R {
    i2c.I2CONSET.write(IF_STOP);
    i2c.I2CONCLR.write(IF_IRQ);
    i2c_wait(i2c, IF_STOP, timeout)
}

pub fn i2c_write(config: I2cConfig, write: &[u8]) -> R<i32> {
    let i2c = unsafe { &*config.i2c };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));

    i2c_start(i2c, timeout)?;
    i2c_send_byte(i2c, config.addr, timeout)?;
    for &b in write {
        i2c_send_byte(i2c, b, timeout)?;
    }
    i2c_stop(i2c, timeout)?;

    Ok(I2C_BUS_SUCCESS)
}

pub fn i2c_read(config: I2cConfig, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let i2c = unsafe { &*config.i2c };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));
    let addr = config.addr | 0x01;

    if !reg.is_empty() {
        // write the register
        i2c_start(i2c, timeout)?;
        i2c_send_byte(i2c, config.addr, timeout)?;
        for &b in reg {
            i2c_send_byte(i2c, b, timeout)?;
        }
        i2c_stop(i2c, timeout)?;
    }
    // start/re-start and read data
    i2c_start(i2c, timeout)?;
    i2c_send_byte(i2c, addr, timeout)?;
    let read_len = read.len();
    for (i, r) in read.iter_mut().enumerate() {
        *r = i2c_read_byte(i2c, timeout, (read_len - 1 - i) as u8)?;
    }
    i2c_stop(i2c, timeout)?;

    Ok(I2C_BUS_SUCCESS)
}
