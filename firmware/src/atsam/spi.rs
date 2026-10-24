// SPI transmissions on sam3
//
// Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
// Copyright (C) 2018  Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals)]
use super::gpio_h::SpiConfig;
use super::internal::*;
use crate::ctr::{concat, concat_len};
use crate::sched::R;
use crate::shutdown;
#[allow(unused_imports)]
use crate::{decl_constant_str, decl_enumeration, decl_enumeration_range};

// Fixes for upstream header changes
#[cfg(CONFIG_MACH_SAME70)]
mod names {
    pub use crate::board::chip::{
        US_MR_SPI_CPHA as US_MR_CPHA, US_MR_SPI_CPOL as US_MR_CPOL, US_MR_USART_CHMODE_NORMAL as US_MR_CHMODE_NORMAL,
    };
}
#[cfg(CONFIG_MACH_SAME70)]
use names::*;

/****************************************************************
 * SPI/USART buses and pins
 ****************************************************************/

#[derive(Clone, Copy)]
struct SpiInfo {
    dev: *mut u8,
    dev_id: u32,
    miso_pin: u8,
    mosi_pin: u8,
    sck_pin: u8,
    rx_periph: char,
    tx_periph: char,
    sck_periph: char,
}
unsafe impl Sync for SpiInfo {}

#[allow(clippy::too_many_arguments)]
const fn info(dev: *mut u8, dev_id: u32, miso: u32, mosi: u32, sck: u32, rx: char, tx: char, sckp: char) -> SpiInfo {
    SpiInfo { dev, dev_id, miso_pin: miso as u8, mosi_pin: mosi as u8, sck_pin: sck as u8, rx_periph: rx, tx_periph: tx, sck_periph: sckp }
}

#[cfg(CONFIG_MACH_SAM3X)]
mod bus {
    use super::*;
    decl_enumeration!("spi_bus", "spi0", 0);
    decl_enumeration_range!("spi_bus", "usart0", 1, 3);
    decl_constant_str!("BUS_PINS_spi0", "PA25,PA26,PA27");
    decl_constant_str!("BUS_PINS_usart0", "PA10,PA11,PA17");
    decl_constant_str!("BUS_PINS_usart1", "PA12,PA13,PA16");
    decl_constant_str!("BUS_PINS_usart2", "PB21,PB20,PB24");
    pub const SPI_BUS: &[SpiInfo] = &[
        info(SPI0 as *mut u8, ID_SPI0, GPIO('A', 25), GPIO('A', 26), GPIO('A', 27), 'A', 'A', 'A'),
        info(USART0 as *mut u8, ID_USART0, GPIO('A', 10), GPIO('A', 11), GPIO('A', 17), 'A', 'A', 'B'),
        info(USART1 as *mut u8, ID_USART1, GPIO('A', 12), GPIO('A', 13), GPIO('A', 16), 'A', 'A', 'A'),
        info(USART2 as *mut u8, ID_USART2, GPIO('B', 21), GPIO('B', 20), GPIO('B', 24), 'A', 'A', 'A'),
    ];
}
#[cfg(CONFIG_MACH_SAM4S)]
mod bus {
    use super::*;
    decl_enumeration!("spi_bus", "spi", 0);
    decl_enumeration_range!("spi_bus", "usart0", 1, 2);
    decl_constant_str!("BUS_PINS_spi", "PA12,PA13,PA14");
    decl_constant_str!("BUS_PINS_usart0", "PA5,PA6,PA2");
    decl_constant_str!("BUS_PINS_usart1", "PA21,PA22,PA23");
    pub const SPI_BUS: &[SpiInfo] = &[
        info(SPI as *mut u8, ID_SPI, GPIO('A', 12), GPIO('A', 13), GPIO('A', 14), 'A', 'A', 'A'),
        info(USART0 as *mut u8, ID_USART0, GPIO('A', 5), GPIO('A', 6), GPIO('A', 2), 'A', 'A', 'B'),
        info(USART1 as *mut u8, ID_USART1, GPIO('A', 21), GPIO('A', 22), GPIO('A', 23), 'A', 'A', 'A'),
    ];
}
#[cfg(CONFIG_MACH_SAM4E)]
mod bus {
    use super::*;
    decl_enumeration!("spi_bus", "spi", 0);
    decl_enumeration_range!("spi_bus", "usart0", 1, 2);
    decl_constant_str!("BUS_PINS_spi", "PA12,PA13,PA14");
    decl_constant_str!("BUS_PINS_usart0", "PB0,PB1,PB13");
    decl_constant_str!("BUS_PINS_usart1", "PA21,PA22,PA23");
    pub const SPI_BUS: &[SpiInfo] = &[
        info(SPI as *mut u8, ID_SPI, GPIO('A', 12), GPIO('A', 13), GPIO('A', 14), 'A', 'A', 'A'),
        info(USART0 as *mut u8, ID_USART0, GPIO('B', 0), GPIO('B', 1), GPIO('B', 13), 'C', 'C', 'C'),
        info(USART1 as *mut u8, ID_USART1, GPIO('A', 21), GPIO('A', 22), GPIO('A', 23), 'A', 'A', 'A'),
    ];
}
#[cfg(CONFIG_MACH_SAME70)]
mod bus {
    use super::*;
    decl_enumeration_range!("spi_bus", "spi0", 0, 2);
    decl_enumeration_range!("spi_bus", "usart0", 2, 3);
    decl_constant_str!("BUS_PINS_spi0", "PD20,PD21,PD22");
    decl_constant_str!("BUS_PINS_usart0", "PB0,PB1,PB13");
    decl_constant_str!("BUS_PINS_usart1", "PA21,PB4,PA23");
    decl_constant_str!("BUS_PINS_usart2", "PD15,PD16,PD17");
    pub const SPI_BUS: &[SpiInfo] = &[
        info(SPI0 as *mut u8, ID_SPI0, GPIO('D', 20), GPIO('D', 21), GPIO('D', 22), 'B', 'B', 'B'),
        info(SPI1 as *mut u8, ID_SPI1, GPIO('C', 26), GPIO('C', 27), GPIO('C', 24), 'C', 'C', 'C'),
        info(USART0 as *mut u8, ID_USART0, GPIO('B', 0), GPIO('B', 1), GPIO('B', 13), 'C', 'C', 'C'),
        info(USART1 as *mut u8, ID_USART1, GPIO('A', 21), GPIO('B', 4), GPIO('A', 23), 'A', 'D', 'A'),
        info(USART2 as *mut u8, ID_USART2, GPIO('D', 15), GPIO('D', 16), GPIO('D', 17), 'B', 'B', 'B'),
    ];
}
static SPI_BUS: [SpiInfo; concat_len(&[bus::SPI_BUS])] = concat(&[bus::SPI_BUS]);

fn is_spihw(dev: *mut u8) -> bool {
    #[cfg(CONFIG_MACH_SAM3X)]
    return dev == SPI0 as *mut u8;
    #[cfg(CONFIG_MACH_SAME70)]
    return dev == SPI0 as *mut u8 || dev == SPI1 as *mut u8;
    #[cfg(not(any(CONFIG_MACH_SAM3X, CONFIG_MACH_SAME70)))]
    return dev == SPI as *mut u8;
}

fn init_pins(bus: usize) {
    let si = &SPI_BUS[bus];
    gpio_peripheral(si.sck_pin as u32, si.sck_periph, 0);
    gpio_peripheral(si.miso_pin as u32, si.rx_periph, 1);
    gpio_peripheral(si.mosi_pin as u32, si.tx_periph, 0);
    enable_pclock(si.dev_id);
}

/****************************************************************
 * SPI hardware
 ****************************************************************/

const CHANNEL: usize = 0; // Use same channel for all

fn spihw_init(bus: usize) {
    init_pins(bus);
    let p_spi = unsafe { &*(SPI_BUS[bus].dev as *const Spi) };

    /* Disable SPI */
    p_spi.SPI_CR.write(SPI_CR_SPIDIS);
    /* Execute a software reset of the SPI twice */
    p_spi.SPI_CR.write(SPI_CR_SWRST);
    p_spi.SPI_CR.write(SPI_CR_SWRST);

    p_spi.SPI_MR.write(
        SPI_MR_MSTR |       // Set master mode
        SPI_MR_MODFDIS |    // Disable fault detection
        SPI_MR_PCS(CHANNEL as u32), // Fixes peripheral select
    );
    p_spi.SPI_IDR.write(0xffffffff); // Disable ISRs

    /* Clear Chip Select Registers */
    p_spi.SPI_CSR[0].write(0);
    p_spi.SPI_CSR[1].write(0);
    p_spi.SPI_CSR[2].write(0);
    p_spi.SPI_CSR[3].write(0);

    /* Set basic channel config */
    p_spi.SPI_CSR[CHANNEL].write(0);
    /* Enable SPI */
    p_spi.SPI_CR.write(SPI_CR_SPIEN);
}

fn spihw_setup(bus: usize, mode: u8, rate: u32) -> SpiConfig {
    // Make sure bus is enabled
    spihw_init(bus);

    let pclk = get_pclock_frequency(ID_USART0);
    let clock_div = if rate < pclk / 255 {
        255
    } else if rate >= pclk / 2 {
        2
    } else {
        pclk / (rate + 1) + 1
    };

    /****** Will be written to SPI_CSRx register ******/
    // CSAAT  : Chip Select Active After Transfer
    let mut config = SPI_CSR_CSAAT;
    config |= SPI_CSR_BITS_8_BIT; // TODO: support for SPI_CSR_BITS_16_BIT
    // NOTE: NCPHA is inverted, CPHA normal!!
    match mode {
        0 => config |= SPI_CSR_NCPHA,
        1 => config |= 0,
        2 => {
            config |= SPI_CSR_NCPHA;
            config |= SPI_CSR_CPOL;
        }
        3 => config |= SPI_CSR_CPOL,
        _ => {}
    }

    config |= (clock_div & 0xff) << SPI_CSR_SCBR_Pos;
    SpiConfig { spidev: SPI_BUS[bus].dev, cfg: config }
}

fn spihw_prepare(config: SpiConfig) {
    let p_spi = unsafe { &*(config.spidev as *const Spi) };
    p_spi.SPI_CSR[CHANNEL].write(config.cfg);
}

fn spihw_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) {
    let p_spi = unsafe { &*(config.spidev as *const Spi) };
    if receive_data != 0 {
        for d in data.iter_mut() {
            p_spi.SPI_TDR.write(*d as u32);
            // wait for receive register
            while p_spi.SPI_SR.read() & SPI_SR_RDRF == 0 {}
            // get data
            *d = p_spi.SPI_RDR.read() as u8;
        }
    } else {
        for d in data.iter() {
            p_spi.SPI_TDR.write(*d as u32);
            // wait for receive register
            while p_spi.SPI_SR.read() & SPI_SR_RDRF == 0 {}
            // read data (to clear RDRF)
            p_spi.SPI_RDR.read();
        }
    }
}

/****************************************************************
 * USART hardware
 ****************************************************************/

fn usart_setup(bus: usize, mode: u8, rate: u32) -> SpiConfig {
    init_pins(bus);
    let p_usart = unsafe { &*(SPI_BUS[bus].dev as *const Usart) };

    p_usart.US_MR.write(0);
    p_usart.US_RTOR.write(0);
    p_usart.US_TTGR.write(0);

    p_usart.US_CR.write(US_CR_RSTTX | US_CR_RSTRX | US_CR_TXDIS | US_CR_RXDIS);

    let pclk = get_pclock_frequency(ID_USART0);
    let br = pclk.div_ceil(rate);
    p_usart.US_BRGR.write(br << US_BRGR_CD_Pos);

    let mut reg = US_MR_CHRL_8_BIT | US_MR_USART_MODE_SPI_MASTER | US_MR_CLKO | US_MR_CHMODE_NORMAL;
    match mode {
        0 => {
            reg |= US_MR_CPHA;
            reg &= !US_MR_CPOL;
        }
        1 => {
            reg &= !US_MR_CPHA;
            reg &= !US_MR_CPOL;
        }
        2 => {
            reg |= US_MR_CPHA;
            reg |= US_MR_CPOL;
        }
        3 => {
            reg &= !US_MR_CPHA;
            reg |= US_MR_CPOL;
        }
        _ => {}
    }

    p_usart.US_MR.or(reg);
    p_usart.US_CR.write(US_CR_RXEN | US_CR_TXEN);
    SpiConfig { spidev: SPI_BUS[bus].dev, cfg: p_usart.US_MR.read() }
}

fn usart_prepare(_config: SpiConfig) {}

fn usart_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) {
    let p_usart = unsafe { &*(config.spidev as *const Usart) };
    if receive_data != 0 {
        for d in data.iter_mut() {
            let co = *d as u32 & 0x000000FF;
            while p_usart.US_CSR.read() & US_CSR_TXRDY == 0 {}
            p_usart.US_THR.write(US_THR_TXCHR(co));
            while p_usart.US_CSR.read() & US_CSR_RXRDY == 0 {}
            let ci = p_usart.US_RHR.read() & US_RHR_RXCHR_Msk;
            *d = ci as u8;
        }
    } else {
        for d in data.iter() {
            let co = *d as u32 & 0x000000FF;
            while p_usart.US_CSR.read() & US_CSR_TXRDY == 0 {}
            p_usart.US_THR.write(US_THR_TXCHR(co));
            while p_usart.US_CSR.read() & US_CSR_RXRDY == 0 {}
            let _ = p_usart.US_RHR.read() & US_RHR_RXCHR_Msk;
        }
    }
}

/****************************************************************
 * Interface
 ****************************************************************/

pub fn spi_setup(bus: u32, mode: u8, rate: u32) -> R<SpiConfig> {
    if bus as usize >= SPI_BUS.len() {
        shutdown!("Invalid spi bus");
    }
    if is_spihw(SPI_BUS[bus as usize].dev) {
        return Ok(spihw_setup(bus as usize, mode, rate));
    }
    Ok(usart_setup(bus as usize, mode, rate))
}

pub fn spi_prepare(config: SpiConfig) {
    if is_spihw(config.spidev) {
        spihw_prepare(config);
    } else {
        usart_prepare(config);
    }
}

pub fn spi_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) -> R {
    if is_spihw(config.spidev) {
        spihw_transfer(config, receive_data, data);
    } else {
        usart_transfer(config, receive_data, data);
    }
    Ok(())
}
