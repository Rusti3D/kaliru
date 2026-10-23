// SPI functions on STM32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::SpiConfig;
use crate::ctr::{concat, concat_len};
#[allow(unused_imports)]
use crate::config::CONFIG_MACH_AT32F403;
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

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

#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("spi_bus", "spi2_PC2_PC3_PB10", 3);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_spi2_PC2_PC3_PB10", "PC2,PC3,PB10");
// Deprecated "spi1" style mappings
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("spi_bus", "spi2", 0);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("spi_bus", "spi1", 1);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("spi_bus", "spi1a", 2);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("spi_bus", "spi2a", 3);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_spi2a", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_enumeration!("spi_bus", "spi3_PB4_PB5_PB3", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_constant_str!("BUS_PINS_spi3_PB4_PB5_PB3", "PB4,PB5,PB3");
// Deprecated "spi1" style mappings
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_enumeration!("spi_bus", "spi2", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_enumeration!("spi_bus", "spi1", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_enumeration!("spi_bus", "spi1a", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_enumeration!("spi_bus", "spi3", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
decl_constant_str!("BUS_PINS_spi3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi2_PC2_PC3_PB10", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi2_PC2_PC3_PB10", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi3_PB4_PB5_PB3", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi3_PB4_PB5_PB3", "PB4,PB5,PB3");
// Deprecated "spi1" style mappings
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi2", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi1", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi1a", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi2a", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi2a", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_enumeration!("spi_bus", "spi3", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
decl_constant_str!("BUS_PINS_spi3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi2_PC2_PC3_PB10", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi2_PC2_PC3_PB10", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi3_PB4_PB5_PB3", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi3_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi3_PC11_PC12_PC10", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi3_PC11_PC12_PC10", "PC11,PC12,PC10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi2_PI2_PI3_PI1", 6);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi2_PI2_PI3_PI1", "PI2,PI3,PI1");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI4))]
decl_enumeration!("spi_bus", "spi4_PE13_PE14_PE12", 7);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI4))]
decl_constant_str!("BUS_PINS_spi4_PE13_PE14_PE12", "PE13,PE14,PE12");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI4))]
#[allow(dead_code)]
const SPI6_INDEX: u32 = 1 + 7;
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, not(has_SPI4)))]
#[allow(dead_code)]
const SPI6_INDEX: u32 = 0 + 7;
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI6))]
decl_enumeration!("spi_bus", "spi6_PG12_PG14_PG13",  SPI6_INDEX);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI6))]
decl_constant_str!("BUS_PINS_spi6_PG12_PG14_PG13", "PG12,PG14,PG13");
// Deprecated "spi1" style mappings
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi2", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi1", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi1a", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi2a", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi2a", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi3", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi3a", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi3a", "PC11,PC12,PC10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_enumeration!("spi_bus", "spi2b", 6);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
decl_constant_str!("BUS_PINS_spi2b", "PI2,PI3,PI1");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI4))]
decl_enumeration!("spi_bus", "spi4", 7);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI4))]
decl_constant_str!("BUS_PINS_spi4", "PE13,PE14,PE12");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi2_PC2_PC3_PB10", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi2_PC2_PC3_PB10", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi3_PB4_PB5_PB3", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi3_PB4_PB5_PB3", "PB4,PB5,PB3");
// Deprecated "spi1" style mappings
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi2", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi1", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi1a", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi2a", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi2a", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_enumeration!("spi_bus", "spi3", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_spi3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi2_PC2_PC3_PB10", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi2_PC2_PC3_PB10", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi2_PB2_PB11_PB10", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi2_PB2_PB11_PB10", "PB2,PB11,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_SPI3))]
decl_enumeration!("spi_bus", "spi3_PB4_PB5_PB3", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_SPI3))]
decl_constant_str!("BUS_PINS_spi3_PB4_PB5_PB3", "PB4,PB5,PB3");
// Deprecated "spi1" style mappings
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi2", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi1", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi1a", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("spi_bus", "spi2a", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_spi2a", "PC2,PC3,PB10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_SPI3))]
decl_enumeration!("spi_bus", "spi3", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_SPI3))]
decl_constant_str!("BUS_PINS_spi3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi2_PA10_PA11_PF1", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi2_PA10_PA11_PF1", "PA10,PA11,PF1");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi3_PB4_PB5_PB3", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi3_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi3_PC11_PC12_PC10", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi3_PC11_PC12_PC10", "PC11,PC12,PC10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4, has_SPI4))]
decl_enumeration!("spi_bus", "spi4_PE13_PE14_PE12", 6);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4, has_SPI4))]
decl_constant_str!("BUS_PINS_spi4_PE13_PE14_PE12", "PE13,PE14,PE12");
// Deprecated "spi1" style mappings
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi2", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi1", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi1a", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi3", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_enumeration!("spi_bus", "spi3a", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_spi3a", "PC11,PC12,PC10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4, has_SPI4))]
decl_enumeration!("spi_bus", "spi4", 6);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4, has_SPI4))]
decl_constant_str!("BUS_PINS_spi4", "PE13,PE14,PE12");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_enumeration!("spi_bus", "spi2_PB14_PB15_PB13", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_spi2_PB14_PB15_PB13", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_enumeration!("spi_bus", "spi1_PA6_PA7_PA5", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_spi1_PA6_PA7_PA5", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_enumeration!("spi_bus", "spi1_PB4_PB5_PB3", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_spi1_PB4_PB5_PB3", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_enumeration!("spi_bus", "spi2_PC2_PC3_PB10", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_spi2_PC2_PC3_PB10", "PC2,PC3,PB10");
// Deprecated "spi1" style mappings
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_enumeration!("spi_bus", "spi2", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_spi2", "PB14,PB15,PB13");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_enumeration!("spi_bus", "spi1", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_spi1", "PA6,PA7,PA5");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_enumeration!("spi_bus", "spi1a", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_spi1a", "PB4,PB5,PB3");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_enumeration!("spi_bus", "spi2a", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_spi2a", "PC2,PC3,PB10");


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

// (the C table, its #if blocks as parts: tools/port_table.py)
#[cfg(CONFIG_MACH_STM32F0)]
const SPI_BUS_0: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [0, 0, 0]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [0, 0, 0]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [0, 0, 0]),
    spi_function(SPI2, ('C', 2), ('C', 3), ('B', 10), [1, 1, 5]),
];
#[cfg(not(CONFIG_MACH_STM32F0))]
const SPI_BUS_0: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1))]
const SPI_BUS_1: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [0, 0, 0]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [0, 0, 0]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [5, 5, 5]),
    spi_function(SPI3, ('B', 4), ('B', 5), ('B', 3), [0, 0, 0]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F1)))]
const SPI_BUS_1: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2))]
const SPI_BUS_2: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [5, 5, 5]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [5, 5, 5]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [5, 5, 5]),
    spi_function(SPI2, ('C', 2), ('C', 3), ('B', 10), [5, 5, 5]),
    spi_function(SPI3, ('B', 4), ('B', 5), ('B', 3), [6, 6, 6]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), CONFIG_MACH_STM32F2)))]
const SPI_BUS_2: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4))]
const SPI_BUS_3: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [5, 5, 5]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [5, 5, 5]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [5, 5, 5]),
    spi_function(SPI2, ('C', 2), ('C', 3), ('B', 10), [5, 5, 5]),
    spi_function(SPI3, ('B', 4), ('B', 5), ('B', 3), [6, 6, 6]),
    spi_function(SPI3, ('C', 11), ('C', 12), ('C', 10), [6, 6, 6]),
    spi_function(SPI2, ('I', 2), ('I', 3), ('I', 1), [5, 5, 5]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4)))]
const SPI_BUS_3: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI4))]
const SPI_BUS_4: &[SpiInfo] = &[
    spi_function(SPI4, ('E', 13), ('E', 14), ('E', 12), [5, 5, 5]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI4)))]
const SPI_BUS_4: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI6))]
const SPI_BUS_5: &[SpiInfo] = &[
    spi_function(SPI6, ('G', 12), ('G', 14), ('G', 13), [5, 5, 5]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), CONFIG_MACH_STM32F4, has_SPI6)))]
const SPI_BUS_5: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7))]
const SPI_BUS_6: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [5, 5, 5]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [5, 5, 5]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [5, 5, 5]),
    spi_function(SPI2, ('C', 2), ('C', 3), ('B', 10), [5, 5, 5]),
    spi_function(SPI3, ('B', 4), ('B', 5), ('B', 3), [6, 6, 6]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), CONFIG_MACH_STM32F7)))]
const SPI_BUS_6: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
const SPI_BUS_7: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [0, 0, 0]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [0, 0, 0]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [0, 0, 0]),
    spi_function(SPI2, ('C', 2), ('C', 3), ('B', 10), [1, 1, 5]),
    spi_function(SPI2, ('B', 2), ('B', 11), ('B', 10), [1, 0, 5]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0)))]
const SPI_BUS_7: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_SPI3))]
const SPI_BUS_8: &[SpiInfo] = &[
    spi_function(SPI3, ('B', 4), ('B', 5), ('B', 3), [9, 9, 9]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_SPI3)))]
const SPI_BUS_8: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4))]
const SPI_BUS_9: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [5, 5, 5]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [5, 5, 5]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [5, 5, 5]),
    spi_function(SPI2, ('A', 10), ('A', 11), ('F', 1), [5, 5, 5]),
    spi_function(SPI3, ('B', 4), ('B', 5), ('B', 3), [6, 6, 6]),
    spi_function(SPI3, ('C', 11), ('C', 12), ('C', 10), [6, 6, 6]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4)))]
const SPI_BUS_9: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4, has_SPI4))]
const SPI_BUS_10: &[SpiInfo] = &[
    spi_function(SPI4, ('E', 13), ('E', 14), ('E', 12), [5, 5, 5]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32G4, has_SPI4)))]
const SPI_BUS_10: &[SpiInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4))]
const SPI_BUS_11: &[SpiInfo] = &[
    spi_function(SPI2, ('B', 14), ('B', 15), ('B', 13), [5, 5, 5]),
    spi_function(SPI1, ('A', 6), ('A', 7), ('A', 5), [5, 5, 5]),
    spi_function(SPI1, ('B', 4), ('B', 5), ('B', 3), [5, 5, 5]),
    spi_function(SPI2, ('C', 2), ('C', 3), ('B', 10), [5, 5, 5]),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F1), not(CONFIG_MACH_STM32F2), not(CONFIG_MACH_STM32F4), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32L4)))]
const SPI_BUS_11: &[SpiInfo] = &[];
const SPI_BUS_PARTS: &[&[SpiInfo]] = &[SPI_BUS_0, SPI_BUS_1, SPI_BUS_2, SPI_BUS_3, SPI_BUS_4, SPI_BUS_5, SPI_BUS_6, SPI_BUS_7, SPI_BUS_8, SPI_BUS_9, SPI_BUS_10, SPI_BUS_11];
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
