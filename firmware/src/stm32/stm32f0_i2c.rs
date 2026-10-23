// I2C functions on stm32f0
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time, I2cConfig};
use crate::ctr::{concat, concat_len};
use crate::i2ccmds::{I2C_BUS_NACK, I2C_BUS_START_NACK, I2C_BUS_START_READ_NACK, I2C_BUS_SUCCESS, I2C_BUS_TIMEOUT};
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

#[derive(Clone, Copy)]
struct I2cInfo {
    i2c: *mut I2C_TypeDef,
    scl_pin: u8,
    sda_pin: u8,
    function: u8,
}
unsafe impl Sync for I2cInfo {}

#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("i2c_bus", "i2c1_PB6_PB7", 0);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("i2c_bus", "i2c1_PF1_PF0", 1);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_i2c1_PF1_PF0", "PF1,PF0");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("i2c_bus", "i2c1_PB8_PB9", 2);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("i2c_bus", "i2c1_PB8_PB7", 3);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_i2c1_PB8_PB7", "PB8,PB7");
// Deprecated "i2c1a" style mappings
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("i2c_bus", "i2c1", 0);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_i2c1", "PB6,PB7");
#[cfg(CONFIG_MACH_STM32F0)]
decl_enumeration!("i2c_bus", "i2c1a", 1);
#[cfg(CONFIG_MACH_STM32F0)]
decl_constant_str!("BUS_PINS_i2c1a", "PF1,PF0");
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F7))]
decl_enumeration!("i2c_bus", "i2c1_PB6_PB7", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F7))]
decl_constant_str!("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("i2c_bus", "i2c1_PB6_PB7", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("i2c_bus", "i2c1_PB8_PB9", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("i2c_bus", "i2c1_PA9_PA10", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_i2c1_PA9_PA10", "PA9,PA10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("i2c_bus", "i2c2_PB10_PB11", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_i2c2_PB10_PB11", "PB10,PB11");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("i2c_bus", "i2c2_PB13_PB14", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_i2c2_PB13_PB14", "PB13,PB14");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_enumeration!("i2c_bus", "i2c2_PA7_PA6", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
decl_constant_str!("BUS_PINS_i2c2_PA7_PA6", "PA7,PA6");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_I2C3))]
decl_enumeration!("i2c_bus", "i2c3_PB3_PB4", 6);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_I2C3))]
decl_constant_str!("BUS_PINS_i2c3_PB3_PB4", "PB3,PB4");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_I2C3))]
decl_enumeration!("i2c_bus", "i2c3_PC0_PC1", 7);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_I2C3))]
decl_constant_str!("BUS_PINS_i2c3_PC0_PC1", "PC0,PC1");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_I2C3))]
decl_enumeration!("i2c_bus", "i2c3_PA7_PA6", 8);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_I2C3))]
decl_constant_str!("BUS_PINS_i2c3_PA7_PA6", "PA7,PA6");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_enumeration!("i2c_bus", "i2c1_PB6_PB7", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_enumeration!("i2c_bus", "i2c1_PB8_PB9", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_enumeration!("i2c_bus", "i2c1_PA9_PA10", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_i2c1_PA9_PA10", "PA9,PA10");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_enumeration!("i2c_bus", "i2c2_PB10_PB11", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_i2c2_PB10_PB11", "PB10,PB11");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_enumeration!("i2c_bus", "i2c2_PB13_PB14", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_i2c2_PB13_PB14", "PB13,PB14");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_enumeration!("i2c_bus", "i2c3_PA7_PB4", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
decl_constant_str!("BUS_PINS_i2c3_PA7_PB4", "PA7,PB4");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_enumeration!("i2c_bus", "i2c1_PA13_PA14", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_i2c1_PA13_PA14", "PA13,PA14");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_enumeration!("i2c_bus", "i2c1_PA15_PA14", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_i2c1_PA15_PA14", "PA15,PA14");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_enumeration!("i2c_bus", "i2c1_PB8_PB7", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_i2c1_PB8_PB7", "PB8,PB7");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_enumeration!("i2c_bus", "i2c1_PB8_PB9", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_enumeration!("i2c_bus", "i2c2_PA9_PA8", 4);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_i2c2_PA9_PA8", "PA9,PA8");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_enumeration!("i2c_bus", "i2c2_PC4_PF0", 5);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_i2c2_PC4_PF0", "PC4,PF0");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_enumeration!("i2c_bus", "i2c3_PC8_PC9", 6);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_i2c3_PC8_PC9", "PC8,PC9");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_enumeration!("i2c_bus", "i2c3_PC8_PC11", 7);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
decl_constant_str!("BUS_PINS_i2c3_PC8_PC11", "PC8,PC11");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
decl_enumeration!("i2c_bus", "i2c1_PB6_PB7", 0);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
decl_constant_str!("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
decl_enumeration!("i2c_bus", "i2c1_PB8_PB9", 1);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
decl_constant_str!("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
decl_enumeration!("i2c_bus", "i2c2_PB10_PB11", 2);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
decl_constant_str!("BUS_PINS_i2c2_PB10_PB11", "PB10,PB11");
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
decl_enumeration!("i2c_bus", "i2c3_PA8_PC9", 3);
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
decl_constant_str!("BUS_PINS_i2c3_PA8_PC9", "PA8,PC9");


const fn info(i2c: *mut I2C_TypeDef, scl: (char, u32), sda: (char, u32), func: u32) -> I2cInfo {
    I2cInfo { i2c, scl_pin: GPIO(scl.0, scl.1) as u8, sda_pin: GPIO(sda.0, sda.1) as u8, function: GPIO_FUNCTION(func) as u8 }
}

// (the C table, its #if blocks as parts: tools/port_table.py)
#[cfg(CONFIG_MACH_STM32F0)]
const I2C_BUS_0: &[I2cInfo] = &[
    info(I2C1, ('B', 6), ('B', 7), 1),
    info(I2C1, ('F', 1), ('F', 0), 1),
    info(I2C1, ('B', 8), ('B', 9), 1),
    info(I2C1, ('B', 8), ('B', 7), 1),
];
#[cfg(not(CONFIG_MACH_STM32F0))]
const I2C_BUS_0: &[I2cInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F7))]
const I2C_BUS_1: &[I2cInfo] = &[
    info(I2C1, ('B', 6), ('B', 7), 1),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32F7)))]
const I2C_BUS_1: &[I2cInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0))]
const I2C_BUS_2: &[I2cInfo] = &[
    info(I2C1, ('B', 6), ('B', 7), 6),
    info(I2C1, ('B', 8), ('B', 9), 6),
    info(I2C1, ('A', 9), ('A', 10), 6),
    info(I2C2, ('B', 10), ('B', 11), 6),
    info(I2C2, ('B', 13), ('B', 14), 6),
    info(I2C2, ('A', 7), ('A', 6), 8),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0)))]
const I2C_BUS_2: &[I2cInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_I2C3))]
const I2C_BUS_3: &[I2cInfo] = &[
    info(I2C3, ('B', 3), ('B', 4), 6),
    info(I2C3, ('C', 0), ('C', 1), 6),
    info(I2C3, ('A', 7), ('A', 6), 9),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), CONFIG_MACH_STM32G0, has_I2C3)))]
const I2C_BUS_3: &[I2cInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4))]
const I2C_BUS_4: &[I2cInfo] = &[
    info(I2C1, ('B', 6), ('B', 7), 4),
    info(I2C1, ('B', 8), ('B', 9), 4),
    info(I2C1, ('A', 9), ('A', 10), 4),
    info(I2C2, ('B', 10), ('B', 11), 4),
    info(I2C2, ('B', 13), ('B', 14), 4),
    info(I2C3, ('A', 7), ('B', 4), 4),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), CONFIG_MACH_STM32L4)))]
const I2C_BUS_4: &[I2cInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4))]
const I2C_BUS_5: &[I2cInfo] = &[
    info(I2C1, ('A', 13), ('A', 14), 4),
    info(I2C1, ('A', 15), ('A', 14), 4),
    info(I2C1, ('B', 8), ('B', 7), 4),
    info(I2C1, ('B', 8), ('B', 9), 4),
    info(I2C2, ('A', 9), ('A', 8), 4),
    info(I2C2, ('C', 4), ('F', 0), 4),
    info(I2C3, ('C', 8), ('C', 9), 8),
    info(I2C3, ('C', 8), ('C', 11), 8),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), CONFIG_MACH_STM32G4)))]
const I2C_BUS_5: &[I2cInfo] = &[];
#[cfg(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7))]
const I2C_BUS_6: &[I2cInfo] = &[
    info(I2C1, ('B', 6), ('B', 7), 4),
    info(I2C1, ('B', 8), ('B', 9), 4),
    info(I2C2, ('B', 10), ('B', 11), 4),
    info(I2C3, ('A', 8), ('C', 9), 4),
];
#[cfg(not(all(not(CONFIG_MACH_STM32F0), not(CONFIG_MACH_STM32F7), not(CONFIG_MACH_STM32G0), not(CONFIG_MACH_STM32L4), not(CONFIG_MACH_STM32G4), CONFIG_MACH_STM32H7)))]
const I2C_BUS_6: &[I2cInfo] = &[];
const I2C_BUS_PARTS: &[&[I2cInfo]] = &[I2C_BUS_0, I2C_BUS_1, I2C_BUS_2, I2C_BUS_3, I2C_BUS_4, I2C_BUS_5, I2C_BUS_6];
static I2C_BUS: [I2cInfo; concat_len(I2C_BUS_PARTS)] = concat(I2C_BUS_PARTS);

pub fn i2c_setup(bus: u32, rate: u32, addr: u8) -> R<I2cConfig> {
    // Lookup requested i2c bus
    if bus as usize >= I2C_BUS.len() {
        shutdown!("Unsupported i2c bus");
    }
    let ii = &I2C_BUS[bus as usize];
    let i2c = unsafe { &*ii.i2c };

    if !is_enabled_pclock(ii.i2c as u32) {
        // Enable i2c clock and gpio
        enable_pclock(ii.i2c as u32);
        gpio_peripheral(ii.scl_pin as u32, ii.function as u32 | GPIO_OPEN_DRAIN, 1);
        gpio_peripheral(ii.sda_pin as u32, ii.function as u32 | GPIO_OPEN_DRAIN, 1);

        // Set 100Khz frequency and enable
        let nom_i2c_clock: u32 = 12000000; // 12mhz internal clock (83.3ns tick)
        let mut scll: u32 = 60; // 60 * 83.3ns = 5us
        let mut sclh: u32 = 48; // 48 * 83.3ns = 4us
        let mut sdadel: u32 = 6; // 6 * 83.3ns = 500ns
        let mut scldel: u32 = 15; // 15 * 83.3ns = 1250ns
        // Clamp the rate to 400Khz
        if rate >= 400000 {
            scll = 15; // 15 * 83.3ns = 1250ns
            sclh = 6; // 6 * 83.3 = 500ns
            sdadel = 4; // 4 * 83.3 = 333ns
            scldel = 6; // 6 * 83.3 = 500ns
        }

        let pclk = get_pclock_frequency(ii.i2c as u32);
        let presc = pclk.div_ceil(nom_i2c_clock);
        i2c.TIMINGR.write(
            ((presc - 1) << I2C_TIMINGR_PRESC_Pos)
                | ((scll - 1) << I2C_TIMINGR_SCLL_Pos)
                | ((sclh - 1) << I2C_TIMINGR_SCLH_Pos)
                | (sdadel << I2C_TIMINGR_SDADEL_Pos)
                | ((scldel - 1) << I2C_TIMINGR_SCLDEL_Pos),
        );
        i2c.CR1.write(I2C_CR1_PE);
    }

    Ok(I2cConfig { i2c: ii.i2c, addr: addr << 1 })
}

fn i2c_wait(i2c: &I2C_TypeDef, set: u32, timeout: u32) -> i32 {
    loop {
        let isr = i2c.ISR.read();
        if isr & set != 0 {
            return I2C_BUS_SUCCESS;
        }
        if isr & I2C_ISR_NACKF != 0 {
            i2c.ICR.write(I2C_ICR_NACKCF);
            return I2C_BUS_NACK;
        }
        if !timer_is_before(timer_read_time(), timeout) {
            return I2C_BUS_TIMEOUT;
        }
    }
}

#[allow(unused_assignments)] // (ret starts as C's I2C_BUS_SUCCESS)
pub fn i2c_write(config: I2cConfig, write: &[u8]) -> R<i32> {
    let i2c = unsafe { &*config.i2c };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));
    let mut ret = I2C_BUS_SUCCESS;
    let mut pos = 0; // (write - write_orig)

    // Send start and address
    i2c.CR2.write(I2C_CR2_START | config.addr as u32 | ((write.len() as u8 as u32) << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND);
    'abrt: {
        for b in write.iter() {
            ret = i2c_wait(i2c, I2C_ISR_TXIS, timeout);
            if ret != I2C_BUS_SUCCESS {
                break 'abrt;
            }
            i2c.TXDR.write(*b as u32);
            pos += 1;
        }
        return Ok(i2c_wait(i2c, I2C_ISR_TXE, timeout));
    }
    if pos == 0 && ret == I2C_BUS_NACK {
        ret = I2C_BUS_START_NACK;
    }
    i2c.CR2.or(I2C_CR2_STOP);
    Ok(ret)
}

#[allow(unused_assignments)] // (ret starts as C's I2C_BUS_SUCCESS)
pub fn i2c_read(config: I2cConfig, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let i2c = unsafe { &*config.i2c };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));
    let mut ret = I2C_BUS_SUCCESS;
    let (mut reg_pos, mut read_pos) = (0, 0); // (reg - write_orig), (read - read_orig)

    'abrt: {
        'abrt_read: {
            if !reg.is_empty() {
                // Send start, address, reg
                i2c.CR2.write(I2C_CR2_START | config.addr as u32 | ((reg.len() as u8 as u32) << I2C_CR2_NBYTES_Pos));
                for b in reg.iter() {
                    ret = i2c_wait(i2c, I2C_ISR_TXIS, timeout);
                    if ret != I2C_BUS_SUCCESS {
                        break 'abrt;
                    }
                    i2c.TXDR.write(*b as u32);
                    reg_pos += 1;
                }
                i2c_wait(i2c, I2C_ISR_TC, timeout);
            }

            // send restart, read data
            i2c.CR2.write(
                I2C_CR2_START | I2C_CR2_RD_WRN | config.addr as u32 | ((read.len() as u8 as u32) << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND,
            );
            for r in read.iter_mut() {
                ret = i2c_wait(i2c, I2C_ISR_RXNE, timeout);
                if ret != I2C_BUS_SUCCESS {
                    break 'abrt_read;
                }
                *r = i2c.RXDR.read() as u8;
                read_pos += 1;
            }
            return Ok(i2c_wait(i2c, I2C_ISR_STOPF, timeout));
        }
        if read_pos == 0 && ret == I2C_BUS_NACK {
            ret = I2C_BUS_START_READ_NACK;
        }
    }
    if reg_pos == 0 && ret == I2C_BUS_NACK {
        ret = I2C_BUS_START_NACK;
    }
    i2c.CR2.or(I2C_CR2_STOP);
    Ok(ret)
}
