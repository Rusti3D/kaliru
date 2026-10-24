// I2C functions on AVR
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio::gpio_out_setup;
use super::gpio_h::I2cConfig;
use super::internal::*;
use super::timer::{timer_from_us, timer_is_before, timer_read_time};
use crate::config::CONFIG_CLOCK_FREQ;
use crate::i2ccmds::I2C_BUS_SUCCESS;
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

decl_enumeration!("i2c_bus", "twi", 0);

#[cfg(any(CONFIG_MACH_atmega168, CONFIG_MACH_atmega328, CONFIG_MACH_atmega328p))]
mod pins {
    use super::*;
    pub const SCL: u8 = GPIO('C', 5);
    pub const SDA: u8 = GPIO('C', 4);
    decl_constant_str!("BUS_PINS_twi", "PC5,PC4");
}
#[cfg(any(CONFIG_MACH_atmega644p, CONFIG_MACH_atmega1284p))]
mod pins {
    use super::*;
    pub const SCL: u8 = GPIO('C', 0);
    pub const SDA: u8 = GPIO('C', 1);
    decl_constant_str!("BUS_PINS_twi", "PC0,PC1");
}
#[cfg(any(
    CONFIG_MACH_at90usb1286,
    CONFIG_MACH_at90usb646,
    CONFIG_MACH_atmega32u4,
    CONFIG_MACH_atmega1280,
    CONFIG_MACH_atmega2560
))]
mod pins {
    use super::*;
    pub const SCL: u8 = GPIO('D', 0);
    pub const SDA: u8 = GPIO('D', 1);
    decl_constant_str!("BUS_PINS_twi", "PD0,PD1");
}
use pins::*;

pub fn i2c_setup(bus: u32, rate: u32, addr: u8) -> R<I2cConfig> {
    if bus != 0 {
        shutdown!("Unsupported i2c bus");
    }

    if unsafe { (*TWCR).read() } & (1 << TWEN) == 0 {
        // Setup output pins and enable pullups
        gpio_out_setup(SDA as u32, 1)?;
        gpio_out_setup(SCL as u32, 1)?;

        unsafe {
            // Set frequency avoiding pulling in integer divide
            (*TWSR).write(0);
            if rate >= 400000 {
                (*TWBR).write((((CONFIG_CLOCK_FREQ / 400000) - 16) / 2) as u8);
            } else {
                (*TWBR).write((((CONFIG_CLOCK_FREQ / 100000) - 16) / 2) as u8);
            }

            // Enable interface
            (*TWCR).write(1 << TWEN);
        }
    }
    Ok(I2cConfig { addr: addr << 1 })
}

fn i2c_wait(timeout: u32) -> R {
    loop {
        if unsafe { (*TWCR).read() } & (1 << TWINT) != 0 {
            break;
        }
        if !timer_is_before(timer_read_time(), timeout) {
            shutdown!("i2c timeout");
        }
    }
    Ok(())
}

fn i2c_start(timeout: u32) -> R {
    unsafe { (*TWCR).write((1 << TWEN) | (1 << TWINT) | (1 << TWSTA)) };
    i2c_wait(timeout)?;
    let status = unsafe { (*TWSR).read() } as u32;
    if status != 0x10 && status != 0x08 {
        shutdown!("Failed to send i2c start");
    }
    Ok(())
}

fn i2c_send_byte(b: u8, timeout: u32) -> R {
    unsafe {
        (*TWDR).write(b);
        (*TWCR).write((1 << TWEN) | (1 << TWINT));
    }
    i2c_wait(timeout)
}

fn i2c_receive_byte(read: &mut u8, timeout: u32, send_ack: bool) -> R {
    unsafe { (*TWCR).write((1 << TWEN) | (1 << TWINT) | ((send_ack as u8) << TWEA)) };
    i2c_wait(timeout)?;
    *read = unsafe { (*TWDR).read() };
    Ok(())
}

fn i2c_stop(_timeout: u32) {
    unsafe { (*TWCR).write((1 << TWEN) | (1 << TWINT) | (1 << TWSTO)) };
}

pub fn i2c_write(config: I2cConfig, write: &[u8]) -> R<i32> {
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));

    i2c_start(timeout)?;
    i2c_send_byte(config.addr, timeout)?;
    for &b in write {
        i2c_send_byte(b, timeout)?;
    }
    i2c_stop(timeout);

    Ok(I2C_BUS_SUCCESS)
}

pub fn i2c_read(config: I2cConfig, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));
    i2c_start(timeout)?;
    i2c_send_byte(config.addr, timeout)?;
    for &b in reg {
        i2c_send_byte(b, timeout)?;
    }
    i2c_start(timeout)?;
    i2c_send_byte(config.addr | 0x1, timeout)?;
    let mut left = read.len();
    for r in read.iter_mut() {
        left -= 1;
        i2c_receive_byte(r, timeout, left != 0)?;
    }
    i2c_stop(timeout);

    Ok(I2C_BUS_SUCCESS)
}
