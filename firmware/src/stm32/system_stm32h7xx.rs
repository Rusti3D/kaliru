// CMSIS Cortex-M Device Peripheral Access Layer System Source File
// (lib/stm32h7/system_stm32h7xx.c): the SystemInit() the firmware calls.
//
// Copyright (c) 2017 STMicroelectronics. All rights reserved.
// (BSD-3-Clause, see Kalico lib/stm32h7)
#![allow(non_snake_case)]
use super::internal::*;

// Vector Table base offset field (VECT_TAB_SRAM is not defined)
const VECT_TAB_OFFSET: u32 = 0x00000000;

// Setup the microcontroller system (DATA_IN_D2_SRAM and DUAL_CORE /
// CORE_CM4 are not defined)
pub fn SystemInit() {
    let (rcc, scb, flash) = unsafe { (&*RCC, &*SCB, &*FLASH) };
    // FPU settings
    if __FPU_PRESENT == 1 && __FPU_USED == 1 {
        scb.CPACR.or((3 << (10 * 2)) | (3 << (11 * 2))); // set CP10 and CP11 Full Access
    }
    // Reset the RCC clock configuration to the default reset state
    // Increasing the CPU frequency
    if FLASH_LATENCY_DEFAULT > (flash.ACR.read() & FLASH_ACR_LATENCY) {
        // Program the new number of wait states to the LATENCY bits in the FLASH_ACR register
        flash.ACR.modify(FLASH_ACR_LATENCY, FLASH_LATENCY_DEFAULT);
    }

    // Set HSION bit
    rcc.CR.or(RCC_CR_HSION);

    // Reset CFGR register
    rcc.CFGR.write(0x00000000);

    // Reset HSEON, HSECSSON, CSION, HSI48ON, CSIKERON, PLL1ON, PLL2ON and PLL3ON bits
    rcc.CR.and(0xEAF6ED7F);

    // Decreasing the number of wait states because of lower CPU frequency
    if FLASH_LATENCY_DEFAULT < (flash.ACR.read() & FLASH_ACR_LATENCY) {
        // Program the new number of wait states to the LATENCY bits in the FLASH_ACR register
        flash.ACR.modify(FLASH_ACR_LATENCY, FLASH_LATENCY_DEFAULT);
    }

    #[cfg(has_D3_SRAM_BASE)]
    {
        // Reset D1CFGR register
        rcc.D1CFGR.write(0x00000000);
        // Reset D2CFGR register
        rcc.D2CFGR.write(0x00000000);
        // Reset D3CFGR register
        rcc.D3CFGR.write(0x00000000);
    }
    #[cfg(not(has_D3_SRAM_BASE))]
    {
        // Reset CDCFGR1 register
        rcc.CDCFGR1.write(0x00000000);
        // Reset CDCFGR2 register
        rcc.CDCFGR2.write(0x00000000);
        // Reset SRDCFGR register
        rcc.SRDCFGR.write(0x00000000);
    }

    // Reset PLLCKSELR register
    rcc.PLLCKSELR.write(0x02020200);

    // Reset PLLCFGR register
    rcc.PLLCFGR.write(0x01FF0000);
    // Reset PLL1DIVR register
    rcc.PLL1DIVR.write(0x01010280);
    // Reset PLL1FRACR register
    rcc.PLL1FRACR.write(0x00000000);

    // Reset PLL2DIVR register
    rcc.PLL2DIVR.write(0x01010280);

    // Reset PLL2FRACR register

    rcc.PLL2FRACR.write(0x00000000);
    // Reset PLL3DIVR register
    rcc.PLL3DIVR.write(0x01010280);

    // Reset PLL3FRACR register
    rcc.PLL3FRACR.write(0x00000000);

    // Reset HSEBYP bit
    rcc.CR.and(0xFFFBFFFF);

    // Disable all interrupts
    rcc.CIER.write(0x00000000);

    if STM32H7_DEV_ID == 0x450 {
        // dual core CM7 or single core line
        if (unsafe { &*DBGMCU }.IDCODE.read() & 0xFFFF0000) < 0x20000000 {
            // if stm32h7 revY
            // Change  the switch matrix read issuing capability to 1 for the AXI SRAM target (Target 7)
            unsafe { core::ptr::write_volatile(0x51008108 as *mut u32, 0x000000001) };
        }
    }

    // Disable the FMC bank1 (enabled after reset).
    // This, prevents CPU speculation access on this bank which blocks the use of FMC during
    // 24us. During this time the others FMC master (such as LTDC) cannot use it!
    unsafe { &*FMC_Bank1_R }.BTCR[0].write(0x000030D2);

    // Configure the Vector Table location add offset address for cortex-M7
    scb.VTOR.write(FLASH_BANK1_BASE | VECT_TAB_OFFSET); // Vector Table Relocation in Internal FLASH
}
