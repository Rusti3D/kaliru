// SAM4 I2C Port
//
// Copyright (C) 2018  Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::gpio_h::I2cConfig;
use super::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time};
use crate::ctr::{concat, concat_len};
use crate::i2ccmds::I2C_BUS_SUCCESS;
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration_range, shutdown};

// (same70_i2c.h: the same70 calls the peripheral TWIHS; the registers
// as functions of the twi device)
#[cfg(not(CONFIG_MACH_SAME70))]
mod twi {
    use super::*;
    pub type Twi = crate::board::chip::Twi;
    pub fn CR(t: &Twi) -> &Reg<u32> {
        &t.TWI_CR
    }
    pub fn CWGR(t: &Twi) -> &Reg<u32> {
        &t.TWI_CWGR
    }
    pub fn IADR(t: &Twi) -> &Reg<u32> {
        &t.TWI_IADR
    }
    pub fn IDR(t: &Twi) -> &Reg<u32> {
        &t.TWI_IDR
    }
    pub fn MMR(t: &Twi) -> &Reg<u32> {
        &t.TWI_MMR
    }
    pub fn RHR(t: &Twi) -> &Reg<u32> {
        &t.TWI_RHR
    }
    pub fn SR(t: &Twi) -> &Reg<u32> {
        &t.TWI_SR
    }
    pub fn THR(t: &Twi) -> &Reg<u32> {
        &t.TWI_THR
    }
}
#[cfg(CONFIG_MACH_SAME70)]
mod twi {
    use super::*;
    pub type Twi = Twihs;
    pub use crate::board::chip::{
        TWIHS_CR_MSDIS as TWI_CR_MSDIS, TWIHS_CR_MSEN as TWI_CR_MSEN, TWIHS_CR_START as TWI_CR_START, TWIHS_CR_STOP as TWI_CR_STOP,
        TWIHS_CR_SVDIS as TWI_CR_SVDIS, TWIHS_CR_SWRST as TWI_CR_SWRST, TWIHS_CWGR_CHDIV as TWI_CWGR_CHDIV,
        TWIHS_CWGR_CKDIV as TWI_CWGR_CKDIV, TWIHS_CWGR_CLDIV as TWI_CWGR_CLDIV, TWIHS_MMR_DADR as TWI_MMR_DADR,
        TWIHS_MMR_IADRSZ_Msk as TWI_MMR_IADRSZ_Msk, TWIHS_MMR_IADRSZ_Pos as TWI_MMR_IADRSZ_Pos, TWIHS_MMR_MREAD as TWI_MMR_MREAD,
        TWIHS_SR_NACK as TWI_SR_NACK, TWIHS_SR_RXRDY as TWI_SR_RXRDY, TWIHS_SR_TXCOMP as TWI_SR_TXCOMP,
        TWIHS_SR_TXRDY as TWI_SR_TXRDY,
    };
    pub fn CR(t: &Twi) -> &Reg<u32> {
        &t.TWIHS_CR
    }
    pub fn CWGR(t: &Twi) -> &Reg<u32> {
        &t.TWIHS_CWGR
    }
    pub fn IADR(t: &Twi) -> &Reg<u32> {
        &t.TWIHS_IADR
    }
    pub fn IDR(t: &Twi) -> &Reg<u32> {
        &t.TWIHS_IDR
    }
    pub fn MMR(t: &Twi) -> &Reg<u32> {
        &t.TWIHS_MMR
    }
    pub fn RHR(t: &Twi) -> &Reg<u32> {
        &t.TWIHS_RHR
    }
    pub fn SR(t: &Twi) -> &Reg<u32> {
        &t.TWIHS_SR
    }
    pub fn THR(t: &Twi) -> &Reg<u32> {
        &t.TWIHS_THR
    }
}
use twi::*;

#[derive(Clone, Copy)]
struct TwiInfo {
    dev: *mut twi::Twi,
    dev_id: u32,
    scl_pin: u8,
    sda_pin: u8,
    periph: char,
}
unsafe impl Sync for TwiInfo {}

const fn info(dev: *mut twi::Twi, dev_id: u32, scl: u32, sda: u32, periph: char) -> TwiInfo {
    TwiInfo { dev, dev_id, scl_pin: scl as u8, sda_pin: sda as u8, periph }
}

// I2C pin definitions
#[cfg(CONFIG_MACH_SAM3X)]
mod bus {
    use super::*;
    decl_enumeration_range!("i2c_bus", "twi0", 0, 2);
    decl_constant_str!("BUS_PINS_twi0", "PA18,PA17");
    decl_constant_str!("BUS_PINS_twi1", "PB13,PB12");
    pub const PRD_CALC_NUM: u32 = 4;
    pub const TWI_BUS: &[TwiInfo] =
        &[info(TWI0, ID_TWI0, GPIO('A', 18), GPIO('A', 17), 'A'), info(TWI1, ID_TWI1, GPIO('B', 13), GPIO('B', 12), 'A')];
}
#[cfg(CONFIG_MACH_SAM4)]
mod bus {
    use super::*;
    decl_enumeration_range!("i2c_bus", "twi0", 0, 2);
    decl_constant_str!("BUS_PINS_twi0", "PA4,PA3");
    decl_constant_str!("BUS_PINS_twi1", "PB5,PB4");
    pub const PRD_CALC_NUM: u32 = 4;
    pub const TWI_BUS: &[TwiInfo] =
        &[info(TWI0, ID_TWI0, GPIO('A', 4), GPIO('A', 3), 'A'), info(TWI1, ID_TWI1, GPIO('B', 5), GPIO('B', 4), 'A')];
}
#[cfg(CONFIG_MACH_SAME70)]
mod bus {
    use super::*;
    decl_enumeration_range!("i2c_bus", "twihs0", 0, 3);
    decl_constant_str!("BUS_PINS_twihs0", "PA4,PA3");
    decl_constant_str!("BUS_PINS_twihs1", "PB5,PB4");
    decl_constant_str!("BUS_PINS_twihs2", "PD28,PD27");
    pub const PRD_CALC_NUM: u32 = 3;
    pub const TWI_BUS: &[TwiInfo] = &[
        info(TWIHS0, ID_TWIHS0, GPIO('A', 4), GPIO('A', 3), 'A'),
        info(TWIHS1, ID_TWIHS1, GPIO('B', 5), GPIO('B', 4), 'A'),
        info(TWIHS2, ID_TWIHS2, GPIO('D', 28), GPIO('D', 27), 'C'),
    ];
}
use bus::PRD_CALC_NUM;
static TWI_BUS: [TwiInfo; concat_len(&[bus::TWI_BUS])] = concat(&[bus::TWI_BUS]);

fn init_pins(bus: usize) {
    let si = &TWI_BUS[bus];
    gpio_peripheral(si.scl_pin as u32, si.periph, 1);
    gpio_peripheral(si.sda_pin as u32, si.periph, 1);
    enable_pclock(si.dev_id);
}

fn i2c_init(bus: usize, rate: u32) {
    let p_twi = unsafe { &*TWI_BUS[bus].dev };
    IDR(p_twi).write(0xFFFFFFFF);
    SR(p_twi).read();
    CR(p_twi).write(TWI_CR_SWRST);
    RHR(p_twi).read();
    CR(p_twi).write(TWI_CR_MSDIS);
    CR(p_twi).write(TWI_CR_SVDIS);
    CR(p_twi).write(TWI_CR_MSEN);

    let pclk = get_pclock_frequency(TWI_BUS[bus].dev_id);
    let mut ckdiv = 0;
    let mut cldiv = (pclk / ((if rate > 384000 { 384000 } else { rate }) * 2)).wrapping_sub(PRD_CALC_NUM);

    while cldiv > 255 && ckdiv < 7 {
        ckdiv += 1;
        cldiv /= 2;
    }

    let chdiv;
    if rate > 348000 {
        let mut d = (pclk / ((2 * rate - 384000) * 2)).wrapping_sub(PRD_CALC_NUM);
        while d > 255 && ckdiv < 7 {
            ckdiv += 1;
            d /= 2;
        }
        chdiv = d;
    } else {
        chdiv = cldiv;
    }
    CWGR(p_twi).write(TWI_CWGR_CLDIV(cldiv) | TWI_CWGR_CHDIV(chdiv) | TWI_CWGR_CKDIV(ckdiv));
}

fn addr_to_u32(addr: &[u8]) -> R<u32> {
    let addr_len = addr.len();
    // (C reads addr[0] whatever the length)
    let mut address = addr.first().copied().unwrap_or(0) as u32;
    if addr_len > 1 {
        address <<= 8;
        address |= addr[1] as u32;
    }
    if addr_len > 2 {
        address <<= 8;
        address |= addr[2] as u32;
    }
    if addr_len > 3 {
        shutdown!("Addresses larger than 3 bytes are not supported");
    }
    Ok(address)
}

pub fn i2c_setup(bus: u32, rate: u32, addr: u8) -> R<I2cConfig> {
    if bus as usize >= TWI_BUS.len() || rate > 400000 {
        shutdown!("Invalid i2c_setup parameters!");
    }
    let p_twi = TWI_BUS[bus as usize].dev;
    init_pins(bus as usize);
    i2c_init(bus as usize, rate);
    Ok(I2cConfig { twi: p_twi as *mut u8, addr })
}

pub fn i2c_write(config: I2cConfig, write: &[u8]) -> R<i32> {
    let p_twi = unsafe { &*(config.twi as *const twi::Twi) };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));
    let mut bytes_to_send = write.len();
    let mut w = write.iter();
    MMR(p_twi).write(TWI_MMR_DADR(config.addr as u32));
    loop {
        let status = SR(p_twi).read();
        if status & TWI_SR_NACK != 0 {
            shutdown!("I2C NACK error encountered!");
        }
        if status & TWI_SR_TXRDY == 0 {
            if !timer_is_before(timer_read_time(), timeout) {
                shutdown!("I2C timeout occured");
            }
            continue;
        }
        if bytes_to_send == 0 {
            break;
        }
        THR(p_twi).write(*w.next().unwrap() as u32);
        bytes_to_send -= 1;
    }
    CR(p_twi).write(TWI_CR_STOP);
    while SR(p_twi).read() & TWI_SR_TXCOMP == 0 {}

    Ok(I2C_BUS_SUCCESS)
}

pub fn i2c_read(config: I2cConfig, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let p_twi = unsafe { &*(config.twi as *const twi::Twi) };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));
    let mut bytes_to_send = read.len();
    let mut pos = 0;
    MMR(p_twi).write(0);
    MMR(p_twi).write(
        TWI_MMR_MREAD | TWI_MMR_DADR(config.addr as u32) | (((reg.len() as u32) << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk),
    );
    IADR(p_twi).write(0);
    IADR(p_twi).write(addr_to_u32(reg)?);
    let mut stop_sent = if bytes_to_send == 1 {
        CR(p_twi).write(TWI_CR_START | TWI_CR_STOP);
        true
    } else {
        CR(p_twi).write(TWI_CR_START);
        false
    };
    while bytes_to_send > 0 {
        let status = SR(p_twi).read();
        if status & TWI_SR_NACK != 0 {
            shutdown!("I2C NACK error encountered!");
        }
        if bytes_to_send == 1 && !stop_sent {
            CR(p_twi).write(TWI_CR_STOP);
            stop_sent = true;
        }
        if status & TWI_SR_RXRDY == 0 {
            if !timer_is_before(timer_read_time(), timeout) {
                shutdown!("I2C timeout occured");
            }
            continue;
        }
        read[pos] = RHR(p_twi).read() as u8;
        pos += 1;
        bytes_to_send -= 1;
    }
    while SR(p_twi).read() & TWI_SR_TXCOMP == 0 {}
    SR(p_twi).read();

    Ok(I2C_BUS_SUCCESS)
}
