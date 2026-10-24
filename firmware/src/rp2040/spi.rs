// SPI functions on rp2040
//
// Copyright (C) 2021  Lasse Dalegaard <dalegaard@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::SpiConfig;
use super::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time};
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

decl_enumeration!("spi_bus", "spi0_gpio0_gpio3_gpio2", 0);
decl_constant_str!("BUS_PINS_spi0_gpio0_gpio3_gpio2", "gpio0,gpio3,gpio2");
decl_enumeration!("spi_bus", "spi0_gpio4_gpio7_gpio6", 1);
decl_constant_str!("BUS_PINS_spi0_gpio4_gpio7_gpio6", "gpio4,gpio7,gpio6");
decl_enumeration!("spi_bus", "spi0_gpio16_gpio19_gpio18", 2);
decl_constant_str!("BUS_PINS_spi0_gpio16_gpio19_gpio18", "gpio16,gpio19,gpio18");
decl_enumeration!("spi_bus", "spi0_gpio20_gpio23_gpio22", 3);
decl_constant_str!("BUS_PINS_spi0_gpio20_gpio23_gpio22", "gpio20,gpio23,gpio22");
decl_enumeration!("spi_bus", "spi0_gpio4_gpio3_gpio2", 4);
decl_constant_str!("BUS_PINS_spi0_gpio4_gpio3_gpio2", "gpio4,gpio3,gpio2");

decl_enumeration!("spi_bus", "spi1_gpio8_gpio11_gpio10", 5);
decl_constant_str!("BUS_PINS_spi1_gpio8_gpio11_gpio10", "gpio8,gpio11,gpio10");
decl_enumeration!("spi_bus", "spi1_gpio12_gpio15_gpio14", 6);
decl_constant_str!("BUS_PINS_spi1_gpio12_gpio15_gpio14", "gpio12,gpio15,gpio14");
decl_enumeration!("spi_bus", "spi1_gpio24_gpio27_gpio26", 7);
decl_constant_str!("BUS_PINS_spi1_gpio24_gpio27_gpio26", "gpio24,gpio27,gpio26");
decl_enumeration!("spi_bus", "spi1_gpio12_gpio11_gpio10", 8);
decl_constant_str!("BUS_PINS_spi1_gpio12_gpio11_gpio10", "gpio12,gpio11,gpio10");

//Deprecated "spi0a" style mappings
decl_enumeration!("spi_bus", "spi0a", 0);
decl_constant_str!("BUS_PINS_spi0a", "gpio0,gpio3,gpio2");
decl_enumeration!("spi_bus", "spi0b", 1);
decl_constant_str!("BUS_PINS_spi0b", "gpio4,gpio7,gpio6");
decl_enumeration!("spi_bus", "spi0c", 2);
decl_constant_str!("BUS_PINS_spi0c", "gpio16,gpio19,gpio18");
decl_enumeration!("spi_bus", "spi0d", 3);
decl_constant_str!("BUS_PINS_spi0d", "gpio20,gpio23,gpio22");
decl_enumeration!("spi_bus", "spi0e", 4);
decl_constant_str!("BUS_PINS_spi0e", "gpio4,gpio3,gpio2");

decl_enumeration!("spi_bus", "spi1a", 5);
decl_constant_str!("BUS_PINS_spi1a", "gpio8,gpio11,gpio10");
decl_enumeration!("spi_bus", "spi1b", 6);
decl_constant_str!("BUS_PINS_spi1b", "gpio12,gpio15,gpio14");
decl_enumeration!("spi_bus", "spi1c", 7);
decl_constant_str!("BUS_PINS_spi1c", "gpio24,gpio27,gpio26");

struct SpiInfo {
    spi: *mut spi_hw_t,
    miso_pin: u8,
    mosi_pin: u8,
    sck_pin: u8,
    pclk: u32,
}
unsafe impl Sync for SpiInfo {}

const fn info(spi: *mut spi_hw_t, miso_pin: u8, mosi_pin: u8, sck_pin: u8, pclk: u32) -> SpiInfo {
    SpiInfo { spi, miso_pin, mosi_pin, sck_pin, pclk }
}

static SPI_BUS: [SpiInfo; 9] = [
    info(spi0_hw, 0, 3, 2, RESETS_RESET_SPI0_BITS),
    info(spi0_hw, 4, 7, 6, RESETS_RESET_SPI0_BITS),
    info(spi0_hw, 16, 19, 18, RESETS_RESET_SPI0_BITS),
    info(spi0_hw, 20, 23, 22, RESETS_RESET_SPI0_BITS),
    info(spi0_hw, 4, 3, 2, RESETS_RESET_SPI0_BITS),
    info(spi1_hw, 8, 11, 10, RESETS_RESET_SPI1_BITS),
    info(spi1_hw, 12, 15, 14, RESETS_RESET_SPI1_BITS),
    info(spi1_hw, 24, 27, 26, RESETS_RESET_SPI1_BITS),
    info(spi1_hw, 12, 11, 10, RESETS_RESET_SPI1_BITS),
];

pub fn spi_setup(bus: u32, mode: u8, rate: u32) -> R<SpiConfig> {
    if bus as usize >= SPI_BUS.len() {
        shutdown!("Invalid spi bus");
    }
    let b = &SPI_BUS[bus as usize];

    if !is_enabled_pclock(b.pclk) {
        enable_pclock(b.pclk);
    }

    gpio_peripheral(b.miso_pin as u32, 1, 1)?;
    gpio_peripheral(b.mosi_pin as u32, 1, 0)?;
    gpio_peripheral(b.sck_pin as u32, 1, 0)?;

    let pclk = get_pclock_frequency(b.pclk);

    let mut res = SpiConfig { spi: b.spi, cr0: 0, cpsr: 0 };

    let mut prescale = 2u32;
    while prescale <= 254 {
        if (prescale + 2).wrapping_mul(256).wrapping_mul(rate) > pclk {
            break;
        }
        prescale += 2;
    }

    let mut postdiv = 255u32;
    while postdiv > 0 {
        if prescale.wrapping_mul(postdiv).wrapping_mul(rate) < pclk {
            break;
        }
        postdiv -= 1;
    }

    res.cr0 |= postdiv << SPI_SSPCR0_SCR_LSB;
    res.cr0 |= ((mode & 2 != 0) as u32) << SPI_SSPCR0_SPO_LSB;
    res.cr0 |= ((mode & 1 != 0) as u32) << SPI_SSPCR0_SPH_LSB;
    res.cr0 |= 7 << SPI_SSPCR0_DSS_LSB; // 8bit mode
    res.cpsr = prescale;

    // Enable the peripheral
    unsafe { &*b.spi }.cr1.write(SPI_SSPCR1_SSE_BITS);

    Ok(res)
}

pub fn spi_prepare(config: SpiConfig) {
    let spi = unsafe { &*config.spi };
    if spi.cr0.read() == config.cr0 && spi.cpsr.read() == config.cpsr {
        return;
    }
    let diff = spi.cr0.read() ^ config.cr0;
    spi.cr1.write(0);
    spi.cr0.write(config.cr0);
    spi.cpsr.write(config.cpsr);
    spi.cr1.write(SPI_SSPCR1_SSE_BITS);
    // Give time for state to update before caller changes CS pin
    let end = timer_read_time().wrapping_add(timer_from_us(1));
    if diff & SPI_SSPCR0_SPO_BITS != 0 {
        while timer_is_before(timer_read_time(), end) {}
    }
}

const MAX_FIFO: usize = 8; // Max rx fifo size (don't tx past this size)

pub fn spi_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) -> R {
    let spi = unsafe { &*config.spi };
    let end = data.len();
    // (C: the write pointer and the read pointer `data`)
    let (mut wptr, mut rptr) = (0, 0);
    while rptr < end {
        let sr = spi.sr.read() & (SPI_SSPSR_TNF_BITS | SPI_SSPSR_RNE_BITS);
        if sr == SPI_SSPSR_TNF_BITS && wptr < end && wptr < rptr + MAX_FIFO {
            spi.dr.write(data[wptr] as u32);
            wptr += 1;
        }
        if sr & SPI_SSPSR_RNE_BITS == 0 {
            continue;
        }
        let rdata = spi.dr.read() as u8;
        if receive_data != 0 {
            data[rptr] = rdata;
        }
        rptr += 1;
    }
    // Wait for any remaining SCLK updates before returning
    while spi.sr.read() & (SPI_SSPSR_TFE_BITS | SPI_SSPSR_BSY_BITS) != SPI_SSPSR_TFE_BITS {}
    Ok(())
}
