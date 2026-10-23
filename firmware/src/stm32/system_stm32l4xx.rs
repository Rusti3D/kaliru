// CMSIS Cortex-M4 Device Peripheral Access Layer System Source File
// (lib/stm32l4/system_stm32l4xx.c): the SystemInit() the firmware calls.
//
// Copyright (c) 2017 STMicroelectronics. All rights reserved.
// (BSD-3-Clause, see Kalico lib/stm32l4)
#![allow(non_snake_case)]
use super::internal::{RCC, RCC_CR_MSION, SCB, __FPU_PRESENT, __FPU_USED};

// Setup the microcontroller system (USER_VECT_TAB_ADDRESS is not defined:
// the vector table is not moved here)
pub fn SystemInit() {
    let rcc = unsafe { &*RCC };
    // FPU settings
    if __FPU_PRESENT == 1 && __FPU_USED == 1 {
        unsafe { &*SCB }.CPACR.or((3 << 20) | (3 << 22)); // set CP10 and CP11 Full Access
    }

    // Reset the RCC clock configuration to the default reset state
    // Set MSION bit
    rcc.CR.or(RCC_CR_MSION);

    // Reset CFGR register
    rcc.CFGR.write(0x00000000);

    // Reset HSEON, CSSON , HSION, and PLLON bits
    rcc.CR.and(0xEAF6FFFF);

    // Reset PLLCFGR register
    rcc.PLLCFGR.write(0x00001000);

    // Reset HSEBYP bit
    rcc.CR.and(0xFFFBFFFF);

    // Disable all interrupts
    rcc.CIER.write(0x00000000);
}
