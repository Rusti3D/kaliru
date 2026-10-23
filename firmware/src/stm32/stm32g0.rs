// Code to setup clocks on stm32g0
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::config::*;
use crate::decl_constant_str;
use crate::generic::armcm_reset::try_request_canboot;
use crate::sched::sched_main;

/****************************************************************
 * Clock setup
 ****************************************************************/

const FREQ_PERIPH: u32 = 64000000;
#[allow(dead_code)]
const FREQ_USB: u32 = 48000000;

// Map a peripheral address to its enable bits
pub fn lookup_clock_line(periph_base: u32) -> Cline {
    let rcc = unsafe { &*RCC };
    let apb1 = |bit: u32| Cline { en: &rcc.APBENR1, rst: &rcc.APBRSTR1, bit };
    let apb2 = |bit: u32| Cline { en: &rcc.APBENR2, rst: &rcc.APBRSTR2, bit };
    if periph_base >= IOPORT_BASE {
        let bit = 1 << ((periph_base - IOPORT_BASE) / 0x400);
        return Cline { en: &rcc.IOPENR, rst: &rcc.IOPRSTR, bit };
    } else if periph_base >= AHBPERIPH_BASE {
        let bit = 1 << ((periph_base - AHBPERIPH_BASE) / 0x400);
        return Cline { en: &rcc.AHBENR, rst: &rcc.AHBRSTR, bit };
    }
    #[cfg(has_USART5_BASE)]
    if periph_base == USART5_BASE {
        return apb1(1 << 8);
    }
    #[cfg(has_USART6_BASE)]
    if periph_base == USART6_BASE {
        return apb1(1 << 9);
    }
    #[cfg(any(has_FDCAN1_BASE, has_FDCAN2_BASE))]
    if periph_base == FDCAN1_BASE || periph_base == FDCAN2_BASE {
        return apb1(1 << 12);
    }
    #[cfg(has_USB_BASE)]
    if periph_base == USB_BASE {
        return apb1(1 << 13);
    }
    #[cfg(has_CRS_BASE)]
    if periph_base == CRS_BASE {
        return apb1(1 << 16);
    }
    #[cfg(has_I2C3_BASE)]
    if periph_base == I2C3_BASE {
        return apb1(1 << 23);
    }
    if periph_base == TIM1_BASE {
        return apb2(1 << 11);
    }
    if periph_base == SPI1_BASE {
        return apb2(1 << 12);
    }
    if periph_base == USART1_BASE {
        return apb2(1 << 14);
    }
    if periph_base == TIM14_BASE {
        return apb2(1 << 15);
    }
    if periph_base == TIM15_BASE {
        return apb2(1 << 16);
    }
    if periph_base == TIM16_BASE {
        return apb2(1 << 17);
    }
    if periph_base == TIM17_BASE {
        return apb2(1 << 18);
    }
    if periph_base == ADC1_BASE {
        return apb2(1 << 20);
    }
    if periph_base >= APBPERIPH_BASE && periph_base < APBPERIPH_BASE + 32 * 0x400 {
        let bit = 1 << ((periph_base - APBPERIPH_BASE) / 0x400);
        return apb1(bit);
    }
    // unknown peripheral. returning .bit=0 makes this a no-op
    Cline { en: &rcc.APBENR1, rst: core::ptr::null(), bit: 0 }
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_periph_base: u32) -> u32 {
    FREQ_PERIPH
}

// Enable a GPIO peripheral clock
pub fn gpio_clock_enable(regs: *mut GPIO_TypeDef) {
    let rcc = unsafe { &*RCC };
    let rcc_pos = (regs as u32 - IOPORT_BASE) / 0x400;
    rcc.IOPENR.or(1 << rcc_pos);
    rcc.IOPENR.read();
}

// PLL (g0) input: 2.66 to 16Mhz, vco: 96 to 344Mhz, output: 12 to 64Mhz

#[cfg(not(CONFIG_STM32_CLOCK_REF_INTERNAL))]
decl_constant_str!("RESERVE_PINS_crystal", "PF0,PF1");

// Configure and enable the PLL as clock source
fn clock_setup() {
    let rcc = unsafe { &*RCC };
    let (pll_base, pll_freq): (u32, u32) = (4000000, 192000000);
    let mut pllcfgr;
    if !CONFIG_STM32_CLOCK_REF_INTERNAL {
        // Configure PLL from external crystal (HSE)
        let div = CONFIG_CLOCK_REF_FREQ / pll_base;
        rcc.CR.or(RCC_CR_HSEON);
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | ((div - 1) << RCC_PLLCFGR_PLLM_Pos);
    } else {
        // Configure PLL from internal 16Mhz oscillator (HSI)
        let div = 16000000 / pll_base;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | ((div - 1) << RCC_PLLCFGR_PLLM_Pos);
    }
    pllcfgr |= (pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos;
    pllcfgr |= (pll_freq / CONFIG_CLOCK_FREQ - 1) << RCC_PLLCFGR_PLLR_Pos;
    #[cfg(has_RCC_PLLCFGR_PLLQ)]
    {
        pllcfgr |= ((pll_freq / FREQ_USB - 1) << RCC_PLLCFGR_PLLQ_Pos) | RCC_PLLCFGR_PLLQEN;
    }
    rcc.PLLCFGR.write(pllcfgr | RCC_PLLCFGR_PLLREN);
    rcc.CR.or(RCC_CR_PLLON);

    // Wait for PLL lock
    while rcc.CR.read() & RCC_CR_PLLRDY == 0 {}

    // Switch system clock to PLL
    rcc.CFGR.write(2 << RCC_CFGR_SW_Pos);
    while rcc.CFGR.read() & RCC_CFGR_SWS_Msk != (2 << RCC_CFGR_SWS_Pos) {}

    // Use PLLQCLK for USB (setting USBSEL=2 works in practice)
    #[cfg(has_USB_BASE)]
    rcc.CCIPR2.write(2 << RCC_CCIPR2_USBSEL_Pos);
}

/****************************************************************
 * Bootloader
 ****************************************************************/

// Handle USB reboot requests
pub fn bootloader_request() {
    try_request_canboot();
    dfu_reboot();
}

/****************************************************************
 * Startup
 ****************************************************************/

// Main entry point - called from armcm_boot.c:ResetHandler()
pub fn armcm_main() -> ! {
    let rcc = unsafe { &*RCC };
    // Disable internal pull-down resistors on UCPDx CCx pins
    unsafe { &*SYSCFG }.CFGR1.or(SYSCFG_CFGR1_UCPD1_STROBE | SYSCFG_CFGR1_UCPD2_STROBE);
    unsafe { &*SCB }.VTOR.write(crate::ctr::VectorTable.as_ptr() as u32);

    // Reset clock registers (in case bootloader has changed them)
    rcc.CR.or(RCC_CR_HSION);
    while rcc.CR.read() & RCC_CR_HSIRDY == 0 {}
    rcc.CFGR.write(0x00000000);
    rcc.CR.write(RCC_CR_HSION);
    while rcc.CR.read() & RCC_CR_PLLRDY != 0 {}
    rcc.PLLCFGR.write(0x00001000);
    rcc.IOPENR.write(0x00000000);
    rcc.AHBENR.write(0x00000100);
    rcc.APBENR1.write(0x00000000);
    rcc.APBENR2.write(0x00000000);

    dfu_reboot_check();

    // Set flash latency, cache and prefetch; use reset value as base
    let mut acr: u32 = 0x00040600;
    acr = (acr & !FLASH_ACR_LATENCY) | (2 << FLASH_ACR_LATENCY_Pos);
    acr |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN;
    unsafe { &*FLASH }.ACR.write(acr);

    // Configure main clock
    clock_setup();

    sched_main();
}
