// spi support on samd
//
// Copyright (C) 2019  Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::SpiConfig;
use super::internal::*;
use crate::ctr::Global;
use crate::sched::R;

static HAVE_RUN_INIT: Global<u8> = Global::new(0);

pub fn spi_init(bus: u32, ss: &SercomSpi, ctrla: u32, baud: u32) {
    let have_run_init = HAVE_RUN_INIT.get();
    if *have_run_init & (1 << bus) != 0 {
        return;
    }
    *have_run_init |= 1 << bus;

    ss.CTRLA.reg.write(0);
    ss.CTRLA.reg.write(ctrla & !SERCOM_SPI_CTRLA_ENABLE);
    ss.CTRLB.reg.write(SERCOM_SPI_CTRLB_RXEN);
    ss.BAUD.reg.write(baud as _);
    ss.CTRLA.reg.write(ctrla);
}

pub fn spi_setup(bus: u32, mode: u8, rate: u32) -> R<SpiConfig> {
    let dipo_dopo = sercom_spi_pins(bus)?;
    let ctrla = SERCOM_SPI_CTRLA_MODE(3) | ((mode as u32) << SERCOM_SPI_CTRLA_CPHA_Pos) | dipo_dopo | SERCOM_SPI_CTRLA_ENABLE;
    let sercom = sercom_enable_pclock(bus)?;
    let ss = unsafe { &*sercom }.SPI();
    let baud = sercom_get_pclock_frequency(bus) / (2 * rate) - 1;
    spi_init(bus, ss, ctrla, baud);
    Ok(SpiConfig { ss: ss as *const SercomSpi as *mut SercomSpi, ctrla, baud })
}

pub fn spi_prepare(config: SpiConfig) {
    let (ctrla, baud) = (config.ctrla, config.baud);
    let ss = unsafe { &*config.ss };
    if ctrla == ss.CTRLA.reg.read() && baud == ss.BAUD.reg.read() as u32 {
        return;
    }
    ss.CTRLA.reg.write(ctrla & !SERCOM_SPI_CTRLA_ENABLE);
    ss.CTRLA.reg.write(ctrla & !SERCOM_SPI_CTRLA_ENABLE);
    ss.BAUD.reg.write(baud as _);
    ss.CTRLA.reg.write(ctrla);
}

pub fn spi_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) -> R {
    let ss = unsafe { &*config.ss };
    if receive_data != 0 {
        for d in data.iter_mut() {
            ss.DATA.reg.write(*d as _);
            // wait for receive register
            while ss.INTFLAG.reg.read() as u32 & SERCOM_SPI_INTFLAG_RXC == 0 {}
            // get data
            *d = ss.DATA.reg.read() as u8;
        }
    } else {
        for d in data.iter() {
            ss.DATA.reg.write(*d as _);
            // wait for receive register
            while ss.INTFLAG.reg.read() as u32 & SERCOM_SPI_INTFLAG_RXC == 0 {}
            // read data (to clear RXC)
            ss.DATA.reg.read();
        }
    }
    Ok(())
}
