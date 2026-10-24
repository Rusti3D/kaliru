// INDX toolboard i2c bus (i2c_bus.h, i2c_bus.c++)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::atsamd::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time};
use crate::sched::R;
use crate::{output, shutdown};

pub struct I2cBus {
    si: &'static SercomI2cm,
}

// (C: si->SYNCBUSY.bit.SYSOP)
fn sysop_busy(si: &SercomI2cm) -> bool {
    si.SYNCBUSY.reg.read() & SERCOM_I2CM_SYNCBUSY_SYSOP != 0
}

impl I2cBus {
    // (C: the sercom_gclk and sercom_id arguments are not used)
    pub fn new(sercom: &'static SercomI2cm, _sercom_gclk: u32, _sercom_id: u32) -> I2cBus {
        let si = sercom;
        enable_pclock(SERCOM4_GCLK_ID_CORE, ID_SERCOM4 as i32);

        // Reset and disable
        si.CTRLA.reg.or(SERCOM_I2CM_CTRLA_SWRST);
        while si.SYNCBUSY.reg.read() & SERCOM_I2CM_SYNCBUSY_SWRST != 0 {}

        // 400 kHz fast mode
        si.CTRLA.reg.write(
            SERCOM_I2CM_CTRLA_LOWTOUTEN | SERCOM_I2CM_CTRLA_INACTOUT(3) | SERCOM_I2CM_CTRLA_SPEED(0x0) | SERCOM_I2CM_CTRLA_MODE(0x5),
        );
        let pfreq = get_pclock_frequency(SERCOM4_GCLK_ID_CORE);
        // (C: pfreq * 125 in 32 bits)
        let baud = (pfreq / 400000 - 10 - pfreq.wrapping_mul(125) / 1000000000) / 2;
        si.BAUD.reg.write(baud);

        // Enable bus
        si.CTRLA.reg.or(SERCOM_I2CM_CTRLA_ENABLE);
        while si.SYNCBUSY.reg.read() & SERCOM_I2CM_SYNCBUSY_ENABLE != 0 {}

        // Go to idle mode
        si.STATUS.reg.modify(SERCOM_I2CM_STATUS_BUSSTATE_Msk as u16, SERCOM_I2CM_STATUS_BUSSTATE(0x1) as u16);
        while sysop_busy(si) {}
        I2cBus { si }
    }

    pub fn transfer(&self, address: u8, send: &[u8], recv: &mut [u8]) -> R<bool> {
        let si = self.si;
        if !send.is_empty() {
            i2c_start(si, address, false)?;
            if !i2c_wait(si)? {
                i2c_end(si);
                return Ok(false);
            }
            if !i2c_send(si, send)? {
                i2c_end(si);
                return Ok(false);
            }
        }

        if recv.is_empty() {
            if !send.is_empty() {
                i2c_end(si);
            }
            return Ok(true);
        }

        i2c_start(si, address, true)?;
        i2c_recv(si, recv)
    }
}

fn i2c_wait(si: &SercomI2cm) -> R<bool> {
    let deadline = timer_read_time().wrapping_add(timer_from_us(500));
    while timer_is_before(timer_read_time(), deadline) {
        let intflag = si.INTFLAG.reg.read() as u32;
        if intflag & SERCOM_I2CM_INTFLAG_MB == 0 {
            if si.STATUS.reg.read() as u32 & SERCOM_I2CM_STATUS_BUSERR != 0 {
                return Ok(false);
            }
            continue;
        }
        if intflag & SERCOM_I2CM_INTFLAG_ERROR != 0 {
            return Ok(false);
        }
        return Ok(true);
    }
    output!("i2c timeout")?;
    Ok(false)
}

fn i2c_start(si: &SercomI2cm, address: u8, read: bool) -> R {
    let busstate = (si.STATUS.reg.read() as u32 & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos;
    if busstate == 0x3 {
        shutdown!("i2c bus was busy");
    }
    si.ADDR.reg.write(((address as u32) << 1) | (read as u32 & 0x1));
    Ok(())
}

#[inline(never)]
fn i2c_end(si: &SercomI2cm) {
    si.CTRLB.reg.write(SERCOM_I2CM_CTRLB_CMD(0x3));
    while sysop_busy(si) {}
}

#[inline(never)]
fn i2c_send(si: &SercomI2cm, send: &[u8]) -> R<bool> {
    for &b in send {
        si.DATA.reg.write(b as _);
        if !i2c_wait(si)? {
            return Ok(false);
        }
    }
    Ok(true)
}

#[inline(never)]
fn i2c_recv(si: &SercomI2cm, buffer: &mut [u8]) -> R<bool> {
    let sb = |si: &SercomI2cm| si.INTFLAG.reg.read() as u32 & SERCOM_I2CM_INTFLAG_SB != 0;
    let mut len = buffer.len();
    for b in buffer.iter_mut() {
        len -= 1;
        let deadline = timer_read_time().wrapping_add(timer_from_us(500));
        while !sb(si) && timer_is_before(timer_read_time(), deadline) {}
        if !sb(si) {
            i2c_end(si);
            shutdown!("timeout");
        }

        if len != 0 {
            // ACK
            si.CTRLB.reg.and(!SERCOM_I2CM_CTRLB_ACKACT);
            while sysop_busy(si) {}

            // Do read
            si.CTRLB.reg.modify(SERCOM_I2CM_CTRLB_CMD_Msk, SERCOM_I2CM_CTRLB_CMD(0x2));
            while sysop_busy(si) {}
        } else {
            // NACK
            si.CTRLB.reg.or(SERCOM_I2CM_CTRLB_ACKACT);
            while sysop_busy(si) {}

            // Issue stop
            si.CTRLB.reg.modify(SERCOM_I2CM_CTRLB_CMD_Msk, SERCOM_I2CM_CTRLB_CMD(0x3));
            while sysop_busy(si) {}
        }

        *b = si.DATA.reg.read() as u8;
    }

    Ok(true)
}
