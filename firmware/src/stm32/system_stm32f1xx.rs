// CMSIS Cortex-M3 Device Peripheral Access Layer System Source File
// (lib/stm32f1/system_stm32f1xx.c): the SystemInit() the firmware calls.
//
// Copyright (c) 2017 STMicroelectronics. All rights reserved.
// (BSD-3-Clause, see Kalico lib/stm32f1)
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::{FLASH_BASE, RCC, SCB};

// Vector Table base offset field (VECT_TAB_SRAM is not defined)
const VECT_TAB_OFFSET: u32 = 0x00000000;

// (SystemCoreClock is the variable's initial value: the firmware never
// calls SystemCoreClockUpdate())
#[allow(dead_code)]
pub const SystemCoreClock: u32 = 16000000;

// Setup the microcontroller system: initialize the Embedded Flash
// Interface, the PLL and update the SystemCoreClock variable
// (stm32f103xe: not an STM32F105xC / F107xC / F100xB / F100xE, no
// DATA_IN_ExtSRAM)
pub fn SystemInit() {
    let (rcc, scb) = unsafe { (&*RCC, &*SCB) };
    // Reset the RCC clock configuration to the default reset state(for debug purpose)
    // Set HSION bit
    rcc.CR.or(0x00000001);

    // Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits
    rcc.CFGR.and(0xF8FF0000);

    // Reset HSEON, CSSON and PLLON bits
    rcc.CR.and(0xFEF6FFFF);

    // Reset HSEBYP bit
    rcc.CR.and(0xFFFBFFFF);

    // Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits
    rcc.CFGR.and(0xFF80FFFF);

    // Disable all interrupts and clear pending bits
    rcc.CIR.write(0x009F0000);

    scb.VTOR.write(FLASH_BASE | VECT_TAB_OFFSET); // Vector Table Relocation in Internal FLASH.
}
