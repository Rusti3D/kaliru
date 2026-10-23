// CMSIS Cortex-M4 Device Peripheral Access Layer System Source File
// (lib/stm32g4/system_stm32g4xx.c): the SystemInit() the firmware calls.
//
// Copyright (c) 2019 STMicroelectronics. All rights reserved.
// (BSD-3-Clause, see Kalico lib/stm32g4)
#![allow(non_snake_case)]
use super::internal::{SCB, __FPU_PRESENT, __FPU_USED};

// Setup the microcontroller system: the FPU setting (USER_VECT_TAB_ADDRESS
// is not defined: the vector table is not moved here)
pub fn SystemInit() {
    // FPU settings
    if __FPU_PRESENT == 1 && __FPU_USED == 1 {
        unsafe { &*SCB }.CPACR.or((3 << (10 * 2)) | (3 << (11 * 2))); // set CP10 and CP11 Full Access
    }
}
