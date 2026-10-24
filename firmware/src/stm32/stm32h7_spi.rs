// SPI functions on STM32H7
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time, SpiConfig};
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

struct SpiInfo {
    spi: *mut SPI_TypeDef,
    miso_pin: u8,
    mosi_pin: u8,
    sck_pin: u8,
    function: u8,
}
unsafe impl Sync for SpiInfo {}

decl_enumeration!("spi_bus", "spi2", 0);
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
decl_enumeration!("spi_bus", "spi1", 1);
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
decl_enumeration!("spi_bus", "spi1a", 2);
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
decl_enumeration!("spi_bus", "spi2a", 3);
decl_constant_str!("BUS_PINS_spi2a", "PC2,PC3,PB10");
decl_enumeration!("spi_bus", "spi3a", 4);
decl_constant_str!("BUS_PINS_spi3a", "PC11,PC12,PC10");
decl_enumeration!("spi_bus", "spi4", 5);
decl_constant_str!("BUS_PINS_spi4", "PE13,PE14,PE12");
decl_enumeration!("spi_bus", "spi5", 6);
decl_constant_str!("BUS_PINS_spi5", "PF8,PF9,PF7");
decl_enumeration!("spi_bus", "spi5a", 7);
decl_constant_str!("BUS_PINS_spi5a", "PH7,PF11,PH6");
decl_enumeration!("spi_bus", "spi6", 8);
decl_constant_str!("BUS_PINS_spi6", "PG12,PG14,PG13");
decl_enumeration!("spi_bus", "spi2b", 9);
decl_constant_str!("BUS_PINS_spi2b", "PI2,PI3,PI1");

const fn info(spi: *mut SPI_TypeDef, miso: (char, u32), mosi: (char, u32), sck: (char, u32), func: u32) -> SpiInfo {
    SpiInfo {
        spi,
        miso_pin: GPIO(miso.0, miso.1) as u8,
        mosi_pin: GPIO(mosi.0, mosi.1) as u8,
        sck_pin: GPIO(sck.0, sck.1) as u8,
        function: GPIO_FUNCTION(func) as u8,
    }
}

static SPI_BUS: [SpiInfo; 10] = [
    info(SPI2, ('B', 14), ('B', 15), ('B', 13), 5),
    info(SPI1, ('A', 6), ('A', 7), ('A', 5), 5),
    info(SPI1, ('B', 4), ('B', 5), ('B', 3), 5),
    info(SPI2, ('C', 2), ('C', 3), ('B', 10), 5),
    info(SPI3, ('C', 11), ('C', 12), ('C', 10), 6),
    info(SPI4, ('E', 13), ('E', 14), ('E', 12), 5),
    info(SPI5, ('F', 8), ('F', 9), ('F', 7), 5),
    info(SPI5, ('H', 7), ('F', 11), ('H', 6), 5),
    info(SPI6, ('G', 12), ('G', 14), ('G', 13), 5),
    info(SPI2, ('I', 2), ('I', 3), ('I', 1), 5),
];

// (the spi_config union: .div and .mode are the first two bytes of spi_cr1)
fn div_mode(config: SpiConfig) -> (u32, u32) {
    (config.spi_cr1 & 0xff, (config.spi_cr1 >> 8) & 0xff)
}

pub fn spi_setup(bus: u32, mode: u8, rate: u32) -> R<SpiConfig> {
    if bus as usize >= SPI_BUS.len() {
        shutdown!("Invalid spi bus");
    }

    // Enable SPI
    let b = &SPI_BUS[bus as usize];
    if !is_enabled_pclock(b.spi as u32) {
        enable_pclock(b.spi as u32);
        gpio_peripheral(b.miso_pin as u32, b.function as u32, 1);
        gpio_peripheral(b.mosi_pin as u32, b.function as u32, 0);
        gpio_peripheral(b.sck_pin as u32, b.function as u32, 0);
    }

    // Calculate CR1 register
    let pclk = get_pclock_frequency(b.spi as u32);
    let mut div: u32 = 0;
    while (pclk >> (div + 1)) > rate && div < 7 {
        div += 1;
    }

    Ok(SpiConfig { spi: b.spi, spi_cr1: (div & 0xff) | ((mode as u32) << 8) })
}

pub fn spi_prepare(config: SpiConfig) {
    let (div, mode) = div_mode(config);
    let spi = unsafe { &*config.spi };

    // Load frequency
    spi.CFG1.write((div << SPI_CFG1_MBR_Pos) | (7 << SPI_CFG1_DSIZE_Pos));
    // Load mode
    let cfg2 = (mode << SPI_CFG2_CPHA_Pos) | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR | SPI_CFG2_SSOE;
    let diff = spi.CFG2.read() ^ cfg2;
    spi.CFG2.write(cfg2);
    let end = timer_read_time().wrapping_add(timer_from_us(1));
    if diff & SPI_CFG2_CPOL_Msk != 0 {
        while timer_is_before(timer_read_time(), end) {}
    }
}

const MAX_FIFO: usize = 8; // Limit tx fifo usage so rx fifo doesn't overrun

pub fn spi_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) -> R {
    let len = data.len();
    let (mut wptr, mut i) = (0, 0);
    let spi = unsafe { &*config.spi };

    spi.CR2.write((len as u8 as u32) << SPI_CR2_TSIZE_Pos);
    // Enable SPI and start transfer, these MUST be set in this sequence
    spi.CR1.write(SPI_CR1_SSI | SPI_CR1_SPE);
    spi.CR1.write(SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE);

    let txdr = spi.TXDR.ptr() as *mut u8;
    let rxdr = spi.RXDR.ptr() as *mut u8;
    while i < len {
        let sr = spi.SR.read() & (SPI_SR_TXP | SPI_SR_RXP);
        if sr == SPI_SR_TXP && wptr < len && wptr < i + MAX_FIFO {
            unsafe { core::ptr::write_volatile(txdr, data[wptr]) };
            wptr += 1;
        }
        if sr & SPI_SR_RXP == 0 {
            continue;
        }
        let rdata = unsafe { core::ptr::read_volatile(rxdr) };
        if receive_data != 0 {
            data[i] = rdata;
        }
        i += 1;
    }

    while spi.SR.read() & SPI_SR_EOT == 0 {}

    // Clear flags and disable SPI
    spi.IFCR.write(0xFFFFFFFF);
    spi.CR1.write(SPI_CR1_SSI);
    Ok(())
}
