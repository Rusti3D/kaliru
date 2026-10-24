// SPI support on lpc176x
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::SpiConfig;
use super::internal::*;
use crate::ctr::Global;
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

struct SpiInfo {
    spi: *mut LPC_SSP_TypeDef,
    miso_pin: u8,
    mosi_pin: u8,
    sck_pin: u8,
    pclk: u8,
}
unsafe impl Sync for SpiInfo {}

decl_enumeration!("spi_bus", "ssp0", 0);
decl_constant_str!("BUS_PINS_ssp0", "P0.17,P0.18,P0.15");
decl_enumeration!("spi_bus", "ssp1", 1);
decl_constant_str!("BUS_PINS_ssp1", "P0.8,P0.9,P0.7");

static SPI_BUS: [SpiInfo; 2] = [
    SpiInfo { spi: LPC_SSP0, miso_pin: GPIO(0, 17) as u8, mosi_pin: GPIO(0, 18) as u8, sck_pin: GPIO(0, 15) as u8, pclk: PCLK_SSP0 as u8 },
    SpiInfo { spi: LPC_SSP1, miso_pin: GPIO(0, 8) as u8, mosi_pin: GPIO(0, 9) as u8, sck_pin: GPIO(0, 7) as u8, pclk: PCLK_SSP1 as u8 },
];

static HAVE_RUN_INIT: Global<[i32; 2]> = Global::new([0; 2]);

fn spi_init(bus: usize) {
    let have_run_init = HAVE_RUN_INIT.get();
    if have_run_init[bus] != 0 {
        return;
    }
    have_run_init[bus] = 1;

    // Configure MISO0, MOSI0, SCK0 pins
    let b = &SPI_BUS[bus];
    gpio_peripheral(b.miso_pin as u32, 2, 0);
    gpio_peripheral(b.mosi_pin as u32, 2, 0);
    gpio_peripheral(b.sck_pin as u32, 2, 0);

    // Setup clock
    enable_pclock(b.pclk as u32);

    // Set initial registers
    let spi = unsafe { &*b.spi };
    spi.CR0.write(0x07);
    spi.CPSR.write(254);
    spi.CR1.write(1 << 1);
}

pub fn spi_setup(bus: u32, mode: u8, rate: u32) -> R<SpiConfig> {
    if bus as usize >= SPI_BUS.len() {
        shutdown!("Invalid spi_setup parameters");
    }

    // Make sure bus is enabled
    spi_init(bus as usize);

    // Setup clock rate and mode
    let b = &SPI_BUS[bus as usize];
    let mut res = SpiConfig { spi: b.spi, cr0: 0, cpsr: 0 };
    let pclk = get_pclock_frequency(b.pclk as u32);
    let div = (pclk / 2).div_ceil(rate) << 1;
    res.cpsr = if div < 2 {
        2
    } else if div > 254 {
        254
    } else {
        div
    };
    res.cr0 = 0x07 | (((mode & 2) as u32) << 5) | (((mode & 1) as u32) << 7);

    Ok(res)
}

pub fn spi_prepare(config: SpiConfig) {
    let spi = unsafe { &*config.spi };
    spi.CR0.write(config.cr0);
    spi.CPSR.write(config.cpsr);
}

pub fn spi_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) -> R {
    let spi = unsafe { &*config.spi };
    if receive_data != 0 {
        for d in data.iter_mut() {
            spi.DR.write(*d as u32);
            // wait for read data to be ready
            while spi.SR.read() & (1 << 2) == 0 {}
            // get data
            *d = spi.DR.read() as u8;
        }
    } else {
        for d in data.iter() {
            spi.DR.write(*d as u32);
            // wait for read data to be ready
            while spi.SR.read() & (1 << 2) == 0 {}
            // read data (to clear receive fifo)
            spi.DR.read();
        }
    }
    Ok(())
}
