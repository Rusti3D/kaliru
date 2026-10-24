// i2c support on samd
//
// Copyright (C) 2019  Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::I2cConfig;
use super::internal::*;
use crate::ctr::Global;
use crate::i2ccmds::I2C_BUS_SUCCESS;
use crate::sched::R;
use crate::shutdown;

const TIME_RISE: u64 = 125; // 125 nanoseconds
const I2C_FREQ: u32 = 100000;
const I2C_FREQ_FAST: u32 = 400000;

static HAVE_RUN_INIT: Global<u8> = Global::new(0);

fn i2c_init(bus: u32, rate: u32, si: &SercomI2cm) {
    let have_run_init = HAVE_RUN_INIT.get();
    if *have_run_init & (1 << bus) != 0 {
        return;
    }
    *have_run_init |= 1 << bus;

    // Configure i2c
    si.CTRLA.reg.write(0);
    let areg = SERCOM_I2CM_CTRLA_LOWTOUTEN | SERCOM_I2CM_CTRLA_INACTOUT(3) | SERCOM_I2CM_CTRLA_MODE(5);
    si.CTRLA.reg.write(areg);
    let freq = sercom_get_pclock_frequency(bus);
    // (C: the arithmetic in unsigned long long, TIME_RISE being one)
    let baud = if rate < I2C_FREQ_FAST {
        ((freq / I2C_FREQ) as u64 - 10 - freq as u64 * TIME_RISE / 1000000000) / 2
    } else {
        ((freq / I2C_FREQ_FAST) as u64 - 10 - freq as u64 * TIME_RISE / 1000000000) / 2
    } as u32;
    si.BAUD.reg.write(baud);
    si.CTRLA.reg.write(areg | SERCOM_I2CM_CTRLA_ENABLE);
    while si.SYNCBUSY.reg.read() & SERCOM_I2CM_SYNCBUSY_ENABLE != 0 {}

    // Go into idle mode
    si.STATUS.reg.write(SERCOM_I2CM_STATUS_BUSSTATE(1) as _);
    while si.SYNCBUSY.reg.read() & SERCOM_I2CM_SYNCBUSY_SYSOP != 0 {}
}

pub fn i2c_setup(bus: u32, rate: u32, addr: u8) -> R<I2cConfig> {
    let sercom = sercom_enable_pclock(bus)?;
    sercom_i2c_pins(bus)?;
    let si = unsafe { &*sercom }.I2CM();
    i2c_init(bus, rate, si);
    Ok(I2cConfig { si: si as *const SercomI2cm as *mut SercomI2cm, addr: addr << 1 })
}

fn i2c_wait(si: &SercomI2cm) -> R {
    loop {
        let intflag = si.INTFLAG.reg.read() as u32;
        if intflag & SERCOM_I2CM_INTFLAG_MB == 0 {
            if si.STATUS.reg.read() as u32 & SERCOM_I2CM_STATUS_BUSERR != 0 {
                shutdown!("i2c buserror");
            }
            continue;
        }
        if intflag & SERCOM_I2CM_INTFLAG_ERROR != 0 {
            shutdown!("i2c error");
        }
        break;
    }
    Ok(())
}

fn i2c_start(si: &SercomI2cm, addr: u8) -> R {
    si.ADDR.reg.write(addr as u32);
    i2c_wait(si)
}

fn i2c_send_byte(si: &SercomI2cm, b: u8) -> R {
    si.DATA.reg.write(b as _);
    i2c_wait(si)
}

fn i2c_stop(si: &SercomI2cm) {
    si.CTRLB.reg.write(SERCOM_I2CM_CTRLB_CMD(3));
}

pub fn i2c_write(config: I2cConfig, write: &[u8]) -> R<i32> {
    let si = unsafe { &*config.si };
    i2c_start(si, config.addr)?;
    for &b in write {
        i2c_send_byte(si, b)?;
    }
    i2c_stop(si);

    Ok(I2C_BUS_SUCCESS)
}

// (C: si->SYNCBUSY.bit.SYSOP)
fn sysop_busy(si: &SercomI2cm) -> bool {
    si.SYNCBUSY.reg.read() & SERCOM_I2CM_SYNCBUSY_SYSOP != 0
}

pub fn i2c_read(config: I2cConfig, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let si = unsafe { &*config.si };

    // start in write mode and write register if provided
    if !reg.is_empty() {
        // start in write mode
        si.ADDR.reg.write(config.addr as u32);
        while si.INTFLAG.reg.read() as u32 & SERCOM_I2CM_INTFLAG_MB == 0 {}

        // write registers
        for &b in reg {
            si.DATA.reg.write(b as _);
            while si.INTFLAG.reg.read() as u32 & SERCOM_I2CM_INTFLAG_MB == 0 {}
        }
    }

    // start with read bit enabled
    si.ADDR.reg.write((config.addr | 0x1) as u32);

    // read bytes from slave
    let mut read_len = read.len();
    for r in read.iter_mut() {
        read_len -= 1;
        while si.INTFLAG.reg.read() as u32 & SERCOM_I2CM_INTFLAG_SB == 0 {}

        if read_len != 0 {
            // set ACK response
            si.CTRLB.reg.and(!SERCOM_I2CM_CTRLB_ACKACT);
            while sysop_busy(si) {}

            // execute ACK succeded by byte read
            si.CTRLB.reg.or(SERCOM_I2CM_CTRLB_CMD(2));
            while sysop_busy(si) {}
        } else {
            // set NACK response
            si.CTRLB.reg.or(SERCOM_I2CM_CTRLB_ACKACT);
            while sysop_busy(si) {}

            // execute NACK succeded by stop condition
            si.CTRLB.reg.or(SERCOM_I2CM_CTRLB_CMD(3));
            while sysop_busy(si) {}
        }

        // read received data byte
        *r = si.DATA.reg.read() as u8;
    }

    Ok(I2C_BUS_SUCCESS)
}
