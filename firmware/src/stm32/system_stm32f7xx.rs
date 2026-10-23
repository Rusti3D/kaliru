// CMSIS Cortex-M7 Device Peripheral Access Layer System Source File
// (lib/stm32f7/system_stm32f7xx.c): the SystemInit() the firmware calls.
//
// Copyright (c) 2016 STMicroelectronics. All rights reserved.
// (BSD-3-Clause, see Kalico lib/stm32f7)
#![allow(non_snake_case)]
use super::internal::{FLASH_BASE, RCC, SCB, __FPU_PRESENT, __FPU_USED};

// Vector Table base offset field (VECT_TAB_SRAM is not defined)
const VECT_TAB_OFFSET: u32 = 0x00;

// Setup the microcontroller system
pub fn SystemInit() {
    let (rcc, scb) = unsafe { (&*RCC, &*SCB) };
    // FPU settings
    if __FPU_PRESENT == 1 && __FPU_USED == 1 {
        scb.CPACR.or((3 << (10 * 2)) | (3 << (11 * 2))); // set CP10 and CP11 Full Access
    }
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
