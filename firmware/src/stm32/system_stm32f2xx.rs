// CMSIS Cortex-M3 Device Peripheral Access Layer System Source File
// (lib/stm32f2/system_stm32f2xx.c): the SystemInit() the firmware calls.
//
// Copyright (c) 2017 STMicroelectronics. All rights reserved.
// (BSD-3-Clause, see Kalico lib/stm32f2)
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::{FLASH_BASE, RCC, SCB};

// Vector Table base offset field (VECT_TAB_SRAM is not defined)
const VECT_TAB_OFFSET: u32 = 0x00;

// (SystemCoreClock is the variable's initial value: the firmware never
// calls SystemCoreClockUpdate())
pub const SystemCoreClock: u32 = 16000000;

// Setup the microcontroller system: the vector table location and the
// RCC clock configuration (DATA_IN_ExtSRAM is not defined: no
// SystemInit_ExtMemCtl())
pub fn SystemInit() {
    let (rcc, scb) = unsafe { (&*RCC, &*SCB) };
    // Reset the RCC clock configuration to the default reset state
    // Set HSION bit
    rcc.CR.or(0x00000001);

    // Reset CFGR register
    rcc.CFGR.write(0x00000000);

    // Reset HSEON, CSSON and PLLON bits
    rcc.CR.and(0xFEF6FFFF);

    // Reset PLLCFGR register
    rcc.PLLCFGR.write(0x24003010);

    // Reset HSEBYP bit
    rcc.CR.and(0xFFFBFFFF);

    // Disable all interrupts
    rcc.CIR.write(0x00000000);

    // Configure the Vector Table location add offset address
    scb.VTOR.write(FLASH_BASE | VECT_TAB_OFFSET); // Vector Table Relocation in Internal FLASH
}
