// I2C functions on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::{irq_restore, irq_save, timer_from_us, timer_is_before, timer_read_time, I2cConfig};
use crate::config::CONFIG_MACH_STM32F1;
use crate::ctr::{concat, concat_len};
use crate::i2ccmds::{I2C_BUS_NACK, I2C_BUS_START_NACK, I2C_BUS_START_READ_NACK, I2C_BUS_SUCCESS, I2C_BUS_TIMEOUT};
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

#[derive(Clone, Copy)]
struct I2cInfo {
    i2c: *mut I2C_TypeDef,
    scl_pin: u8,
    sda_pin: u8,
}
unsafe impl Sync for I2cInfo {}

decl_enumeration!("i2c_bus", "i2c1", 0);
decl_constant_str!("BUS_PINS_i2c1", "PB6,PB7");
decl_enumeration!("i2c_bus", "i2c1a", 1);
decl_constant_str!("BUS_PINS_i2c1a", "PB8,PB9");
decl_enumeration!("i2c_bus", "i2c2", 2);
decl_constant_str!("BUS_PINS_i2c2", "PB10,PB11");
#[cfg(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4))]
mod buses {
    use crate::{decl_constant_str, decl_enumeration};
    decl_enumeration!("i2c_bus", "i2c3", 3);
    decl_constant_str!("BUS_PINS_i2c3", "PA8,PC9");
    #[cfg(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4x5))]
    mod more {
        use crate::{decl_constant_str, decl_enumeration};
        decl_enumeration!("i2c_bus", "i2c2a", 4);
        decl_constant_str!("BUS_PINS_i2c2a", "PH4,PH5");
        decl_enumeration!("i2c_bus", "i2c3a", 5);
        decl_constant_str!("BUS_PINS_i2c3a", "PH7,PH8");
        decl_enumeration!("i2c_bus", "i2c2_PF1_PF0", 6);
        decl_constant_str!("BUS_PINS_i2c2_PF1_PF0", "PF1,PF0");
    }
}

const fn info(i2c: *mut I2C_TypeDef, scl: (char, u32), sda: (char, u32)) -> I2cInfo {
    I2cInfo { i2c, scl_pin: GPIO(scl.0, scl.1) as u8, sda_pin: GPIO(sda.0, sda.1) as u8 }
}

const I2C_BUS_BASE: &[I2cInfo] = &[
    info(I2C1, ('B', 6), ('B', 7)),
    info(I2C1, ('B', 8), ('B', 9)),
    info(I2C2, ('B', 10), ('B', 11)),
];
#[cfg(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4))]
const I2C_BUS_I2C3: &[I2cInfo] = &[info(I2C3, ('A', 8), ('C', 9))];
#[cfg(not(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4)))]
const I2C_BUS_I2C3: &[I2cInfo] = &[];
#[cfg(all(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4), any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4x5)))]
const I2C_BUS_MORE: &[I2cInfo] = &[info(I2C2, ('H', 4), ('H', 5)), info(I2C3, ('H', 7), ('H', 8)), info(I2C2, ('F', 1), ('F', 0))];
#[cfg(not(all(any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4), any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4x5))))]
const I2C_BUS_MORE: &[I2cInfo] = &[];
const I2C_BUS_PARTS: &[&[I2cInfo]] = &[I2C_BUS_BASE, I2C_BUS_I2C3, I2C_BUS_MORE];
static I2C_BUS: [I2cInfo; concat_len(I2C_BUS_PARTS)] = concat(I2C_BUS_PARTS);

// Work around stm32 errata causing busy bit to be stuck
fn i2c_busy_errata(scl_pin: u8, sda_pin: u8) {
    if !CONFIG_MACH_STM32F1 {
        return;
    }
    let (scl, sda) = (scl_pin as u32, sda_pin as u32);
    gpio_peripheral(scl, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
    gpio_peripheral(sda, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
    gpio_peripheral(sda, GPIO_OUTPUT | GPIO_OPEN_DRAIN, -1);
    gpio_peripheral(scl, GPIO_OUTPUT | GPIO_OPEN_DRAIN, -1);
    gpio_peripheral(scl, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
    gpio_peripheral(sda, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
}

pub fn i2c_setup(bus: u32, _rate: u32, addr: u8) -> R<I2cConfig> {
    // Lookup requested i2c bus
    if bus as usize >= I2C_BUS.len() {
        shutdown!("Unsupported i2c bus");
    }
    let ii = &I2C_BUS[bus as usize];
    let i2c = unsafe { &*ii.i2c };

    if !is_enabled_pclock(ii.i2c as u32) {
        // Enable i2c clock and gpio
        enable_pclock(ii.i2c as u32);
        i2c_busy_errata(ii.scl_pin, ii.sda_pin);
        gpio_peripheral(ii.scl_pin as u32, GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN, 1);
        gpio_peripheral(ii.sda_pin as u32, GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN, 1);
        i2c.CR1.write(I2C_CR1_SWRST);
        i2c.CR1.write(0);

        // Set 100Khz frequency and enable
        let pclk = get_pclock_frequency(ii.i2c as u32);
        i2c.CR2.write(pclk / 1000000);
        i2c.CCR.write(pclk / 100000 / 2);
        i2c.TRISE.write((pclk / 1000000) + 1);
        i2c.CR1.write(I2C_CR1_PE);
    }

    Ok(I2cConfig { i2c: ii.i2c, addr: addr << 1 })
}

fn i2c_wait(i2c: &I2C_TypeDef, set: u32, clear: u32, timeout: u32) -> i32 {
    loop {
        let sr1 = i2c.SR1.read();
        if (sr1 & set) == set && (sr1 & clear) == 0 {
            return I2C_BUS_SUCCESS;
        }
        if sr1 & I2C_SR1_AF != 0 {
            return I2C_BUS_NACK;
        }
        if !timer_is_before(timer_read_time(), timeout) {
            return I2C_BUS_TIMEOUT;
        }
    }
}

fn i2c_start(i2c: &I2C_TypeDef, addr: u8, xfer_len: u8, timeout: u32) -> R<i32> {
    i2c.CR1.write(I2C_CR1_START | I2C_CR1_PE);
    i2c_wait(i2c, I2C_SR1_SB, 0, timeout);
    i2c.DR.write(addr as u32);
    if addr & 0x01 != 0 {
        i2c.CR1.or(I2C_CR1_ACK);
    }
    let ret = i2c_wait(i2c, I2C_SR1_ADDR, 0, timeout);
    let flag = irq_save();
    let sr2 = i2c.SR2.read();
    if addr & 0x01 != 0 && xfer_len == 1 {
        i2c.CR1.write(I2C_CR1_STOP | I2C_CR1_PE);
    }
    irq_restore(flag);
    if sr2 & I2C_SR2_MSL == 0 {
        shutdown!("Failed to send i2c addr");
    }
    Ok(ret)
}

fn i2c_send_byte(i2c: &I2C_TypeDef, b: u8, timeout: u32) -> i32 {
    i2c.DR.write(b as u32);
    i2c_wait(i2c, I2C_SR1_TXE, 0, timeout)
}

fn i2c_read_byte(i2c: &I2C_TypeDef, timeout: u32, remaining: u8) -> u8 {
    i2c_wait(i2c, I2C_SR1_RXNE, 0, timeout);
    let flag = irq_save();
    let b = i2c.DR.read() as u8;
    if remaining == 1 {
        i2c.CR1.write(I2C_CR1_STOP | I2C_CR1_PE);
    }
    irq_restore(flag);
    b
}

fn i2c_stop(i2c: &I2C_TypeDef, timeout: u32) -> i32 {
    i2c.CR1.write(I2C_CR1_STOP | I2C_CR1_PE);
    i2c_wait(i2c, 0, I2C_SR1_TXE, timeout)
}

pub fn i2c_write(config: I2cConfig, write: &[u8]) -> R<i32> {
    let i2c = unsafe { &*config.i2c };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));

    let mut ret = i2c_start(i2c, config.addr, write.len() as u8, timeout)?;
    if ret == I2C_BUS_NACK {
        ret = I2C_BUS_START_NACK;
    }
    let mut it = write.iter();
    while ret == I2C_BUS_SUCCESS {
        let Some(b) = it.next() else { break };
        ret = i2c_send_byte(i2c, *b, timeout);
    }
    let timeout_err = i2c_stop(i2c, timeout);

    if ret == I2C_BUS_SUCCESS && timeout_err != I2C_BUS_SUCCESS {
        ret = timeout_err;
    }
    Ok(ret)
}

pub fn i2c_read(config: I2cConfig, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let i2c = unsafe { &*config.i2c };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));
    let addr = config.addr | 0x01;
    let mut ret;

    'abrt: {
        if !reg.is_empty() {
            // write the register
            ret = i2c_start(i2c, config.addr, reg.len() as u8, timeout)?;
            if ret == I2C_BUS_NACK {
                ret = I2C_BUS_START_NACK;
            }
            let mut it = reg.iter();
            while ret == I2C_BUS_SUCCESS {
                let Some(b) = it.next() else { break };
                ret = i2c_send_byte(i2c, *b, timeout);
            }
            if ret != I2C_BUS_SUCCESS {
                break 'abrt;
            }
        }
        // start/re-start and read data
        ret = i2c_start(i2c, addr, read.len() as u8, timeout)?;
        if ret == I2C_BUS_NACK {
            ret = I2C_BUS_START_READ_NACK;
        }
        if ret != I2C_BUS_SUCCESS {
            break 'abrt;
        }
        let mut read_len = read.len() as u8;
        for r in read.iter_mut() {
            read_len -= 1;
            *r = i2c_read_byte(i2c, timeout, read_len);
        }
        // covers read timeout
        return Ok(i2c_wait(i2c, 0, I2C_SR1_RXNE, timeout));
    }
    i2c_stop(i2c, timeout);
    Ok(ret)
}
