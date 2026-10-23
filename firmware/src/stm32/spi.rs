// SPI functions on STM32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::SpiConfig;
#[allow(unused_imports)]
use crate::ctr::{concat, concat_len};
#[allow(unused_imports)]
use crate::config::CONFIG_MACH_AT32F403;
use crate::sched::R;
use crate::shutdown;

#[derive(Clone, Copy)]
struct SpiInfo {
    spi: *mut SPI_TypeDef,
    miso_pin: u8,
    mosi_pin: u8,
    sck_pin: u8,
    miso_af: u8,
    mosi_af: u8,
    sck_af: u8,
}
unsafe impl Sync for SpiInfo {}

// (the stm32f0 / f2 / f7 / g0 / g4 / l4 bus tables come with those chips)
#[cfg(CONFIG_MACH_STM32F1)]
mod buses {
    use crate::{decl_constant_str, decl_enumeration};
    decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
    decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
    decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
    decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
    decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
    decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
    decl_enumeration!("spi_bus", "spi3_PB4_PB5_PB3", 3);
    decl_constant_str!("BUS_PINS_spi3_PB4_PB5_PB3", "PB4,PB5,PB3");
    // Deprecated "spi1" style mappings
    decl_enumeration!("spi_bus", "spi2", 0);
    decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
    decl_enumeration!("spi_bus", "spi1", 1);
    decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
    decl_enumeration!("spi_bus", "spi1a", 2);
    decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
    decl_enumeration!("spi_bus", "spi3", 3);
    decl_constant_str!("BUS_PINS_spi3", "PB4,PB5,PB3");
}
#[cfg(CONFIG_MACH_STM32F4)]
mod buses {
    use crate::{decl_constant_str, decl_enumeration};
    decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
    decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
    decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
    decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
    decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
    decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
    decl_enumeration!("spi_bus", "spi2_PC2_PC3_PB10", 3);
    decl_constant_str!("BUS_PINS_spi2_PC2_PC3_PB10", "PC2,PC3,PB10");
    decl_enumeration!("spi_bus", "spi3_PB4_PB5_PB3", 4);
    decl_constant_str!("BUS_PINS_spi3_PB4_PB5_PB3", "PB4,PB5,PB3");
    decl_enumeration!("spi_bus", "spi3_PC11_PC12_PC10", 5);
    decl_constant_str!("BUS_PINS_spi3_PC11_PC12_PC10", "PC11,PC12,PC10");
    decl_enumeration!("spi_bus", "spi2_PI2_PI3_PI1", 6);
    decl_constant_str!("BUS_PINS_spi2_PI2_PI3_PI1", "PI2,PI3,PI1");
    #[cfg(has_SPI4)]
    decl_enumeration!("spi_bus", "spi4_PE13_PE14_PE12", 7);
    #[cfg(has_SPI4)]
    decl_constant_str!("BUS_PINS_spi4_PE13_PE14_PE12", "PE13,PE14,PE12");
    #[cfg(has_SPI4)]
    #[allow(dead_code)]
    const SPI6_INDEX: u32 = 1 + 7;
    #[cfg(not(has_SPI4))]
    #[allow(dead_code)]
    const SPI6_INDEX: u32 = 0 + 7;
    #[cfg(has_SPI6)]
    decl_enumeration!("spi_bus", "spi6_PG12_PG14_PG13", SPI6_INDEX);
    #[cfg(has_SPI6)]
    decl_constant_str!("BUS_PINS_spi6_PG12_PG14_PG13", "PG12,PG14,PG13");
    // Deprecated "spi1" style mappings
    decl_enumeration!("spi_bus", "spi2", 0);
    decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
    decl_enumeration!("spi_bus", "spi1", 1);
    decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
    decl_enumeration!("spi_bus", "spi1a", 2);
    decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
    decl_enumeration!("spi_bus", "spi2a", 3);
    decl_constant_str!("BUS_PINS_spi2a", "PC2,PC3,PB10");
    decl_enumeration!("spi_bus", "spi3", 4);
    decl_constant_str!("BUS_PINS_spi3", "PB4,PB5,PB3");
    decl_enumeration!("spi_bus", "spi3a", 5);
    decl_constant_str!("BUS_PINS_spi3a", "PC11,PC12,PC10");
    decl_enumeration!("spi_bus", "spi2b", 6);
    decl_constant_str!("BUS_PINS_spi2b", "PI2,PI3,PI1");
    #[cfg(has_SPI4)]
    decl_enumeration!("spi_bus", "spi4", 7);
    #[cfg(has_SPI4)]
    decl_constant_str!("BUS_PINS_spi4", "PE13,PE14,PE12");
}

const fn spi_function(spi: *mut SPI_TypeDef, miso: (char, u32), mosi: (char, u32), sck: (char, u32), f: [u32; 3]) -> SpiInfo {
    SpiInfo {
        spi,
        miso_pin: GPIO(miso.0, miso.1) as u8,
        mosi_pin: GPIO(mosi.0, mosi.1) as u8,
        sck_pin: GPIO(sck.0, sck.1) as u8,
        miso_af: GPIO_FUNCTION(f[0]) as u8,
        mosi_af: GPIO_FUNCTION(f[1]) as u8,
        sck_af: GPIO_FUNCTION(f[2]) as u8,
    }
}

#[cfg(CONFIG_MACH_STM32F1)]
static SPI_BUS: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [0, 0, 0]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [0, 0, 0]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [5, 5, 5]),
    spi_function(SPI3, ('B', 4), ('B', 5), ('B', 3), [0, 0, 0]),
];
#[cfg(CONFIG_MACH_STM32F4)]
const SPI_BUS_BASE: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [5, 5, 5]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [5, 5, 5]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [5, 5, 5]),
    spi_function(SPI2, ('C', 2), ('C', 3), ('B', 10), [5, 5, 5]),
    spi_function(SPI3, ('B', 4), ('B', 5), ('B', 3), [6, 6, 6]),
    spi_function(SPI3, ('C', 11), ('C', 12), ('C', 10), [6, 6, 6]),
    spi_function(SPI2, ('I', 2), ('I', 3), ('I', 1), [5, 5, 5]),
];

#[cfg(all(CONFIG_MACH_STM32F4, has_SPI4))]
const SPI_BUS_SPI4: &[SpiInfo] = &[spi_function(SPI4, ('E', 13), ('E', 14), ('E', 12), [5, 5, 5])];
#[cfg(all(CONFIG_MACH_STM32F4, not(has_SPI4)))]
const SPI_BUS_SPI4: &[SpiInfo] = &[];
#[cfg(all(CONFIG_MACH_STM32F4, has_SPI6))]
const SPI_BUS_SPI6: &[SpiInfo] = &[spi_function(SPI6, ('G', 12), ('G', 14), ('G', 13), [5, 5, 5])];
#[cfg(all(CONFIG_MACH_STM32F4, not(has_SPI6)))]
const SPI_BUS_SPI6: &[SpiInfo] = &[];
#[cfg(CONFIG_MACH_STM32F4)]
const SPI_BUS_PARTS: &[&[SpiInfo]] = &[SPI_BUS_BASE, SPI_BUS_SPI4, SPI_BUS_SPI6];
#[cfg(CONFIG_MACH_STM32F4)]
static SPI_BUS: [SpiInfo; concat_len(SPI_BUS_PARTS)] = concat(SPI_BUS_PARTS);

pub fn spi_setup(bus: u32, mode: u8, rate: u32) -> R<SpiConfig> {
    if bus as usize >= SPI_BUS.len() {
        shutdown!("Invalid spi bus");
    }

    // Enable SPI
    let b = &SPI_BUS[bus as usize];
    let spi = unsafe { &*b.spi };
    if !is_enabled_pclock(b.spi as u32) {
        enable_pclock(b.spi as u32);
        gpio_peripheral(b.miso_pin as u32, b.miso_af as u32, 1);
        gpio_peripheral(b.mosi_pin as u32, b.mosi_af as u32, 0);
        gpio_peripheral(b.sck_pin as u32, b.sck_af as u32, 0);

        // Configure CR2 on stm32 f0/f7/g0/l4/g4
        #[cfg(has_SPI_CR2_FRXTH)]
        spi.CR2.write(SPI_CR2_FRXTH | (7 << SPI_CR2_DS_Pos));
    }
    let _ = spi;

    // Calculate CR1 register
    let pclk = get_pclock_frequency(b.spi as u32);
    let mut div = 0;
    while (pclk >> (div + 1)) > rate && div < 7 {
        div += 1;
    }
    let cr1 = (mode as u32) << SPI_CR1_CPHA_Pos | (div << SPI_CR1_BR_Pos) | SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

    Ok(SpiConfig { spi: b.spi, spi_cr1: cr1 })
}

pub fn spi_prepare(config: SpiConfig) {
    let spi = unsafe { &*config.spi };
    let cr1 = spi.CR1.read();
    if !CONFIG_MACH_AT32F403 && cr1 == config.spi_cr1 {
        return;
    }
    // The SPE bit must be disabled before changing CPOL/CPHA bits
    spi.CR1.write(cr1 & !SPI_CR1_SPE);
    spi.CR1.read(); // Force flush of previous write
    spi.CR1.write(config.spi_cr1);
}

// stm32 f0/f7/g0/l4/g4 supports buffering
const CAN_BUFFER: bool = cfg!(has_SPI_CR2_FRXTH);

pub fn spi_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) -> R {
    let spi = unsafe { &*config.spi };
    let dr = spi.DR.ptr() as *mut u8;
    let len = data.len();
    let (mut wptr, mut i) = (0, 0);
    while i < len {
        if CAN_BUFFER {
            let sr = spi.SR.read() & (SPI_SR_TXE | SPI_SR_RXNE);
            if sr == SPI_SR_TXE && wptr < len {
                unsafe { core::ptr::write_volatile(dr, data[wptr]) };
                wptr += 1;
            }
            if sr & SPI_SR_RXNE == 0 {
                continue;
            }
        } else {
            unsafe { core::ptr::write_volatile(dr, data[i]) };
            while spi.SR.read() & SPI_SR_RXNE == 0 {}
        }
        let rdata = unsafe { core::ptr::read_volatile(dr) };
        if receive_data != 0 {
            data[i] = rdata;
        }
        i += 1;
    }
    // Wait for any remaining SCLK updates before returning
    while spi.SR.read() & (SPI_SR_TXE | SPI_SR_BSY) != SPI_SR_TXE {}
    Ok(())
}
