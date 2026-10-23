// Code to setup clocks and gpio on stm32g4
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::system_stm32g4xx::SystemInit;
use crate::config::*;
use crate::decl_constant_str;
use crate::generic::armcm_reset::try_request_canboot;
use crate::sched::sched_main;

const FREQ_PERIPH_DIV: u32 = 2;
const FREQ_PERIPH: u32 = CONFIG_CLOCK_FREQ / FREQ_PERIPH_DIV;

// Map a peripheral address to its enable bits
pub fn lookup_clock_line(periph_base: u32) -> Cline {
    let rcc = unsafe { &*RCC };
    if periph_base < APB2PERIPH_BASE {
        let pos = (periph_base - APB1PERIPH_BASE) / 0x400;
        if pos < 32 {
            #[cfg(has_FDCAN2_BASE)]
            if periph_base == FDCAN2_BASE {
                return Cline { en: &rcc.APB1ENR1, rst: &rcc.APB1RSTR1, bit: 1 << 25 };
            }
            Cline { en: &rcc.APB1ENR1, rst: &rcc.APB1RSTR1, bit: 1 << pos }
        } else {
            Cline { en: &rcc.APB1ENR2, rst: &rcc.APB1RSTR2, bit: 1 << (pos - 32) }
        }
    } else if periph_base < AHB1PERIPH_BASE {
        let pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        Cline { en: &rcc.APB2ENR, rst: &rcc.APB2RSTR, bit: 1 << pos }
    } else if periph_base < AHB2PERIPH_BASE {
        let pos = (periph_base - AHB1PERIPH_BASE) / 0x400;
        Cline { en: &rcc.AHB1ENR, rst: &rcc.AHB1RSTR, bit: 1 << pos }
    } else {
        if periph_base == ADC12_COMMON_BASE {
            return Cline { en: &rcc.AHB2ENR, rst: &rcc.AHB2RSTR, bit: RCC_AHB2ENR_ADC12EN };
        }
        let pos = (periph_base - AHB2PERIPH_BASE) / 0x400;
        Cline { en: &rcc.AHB2ENR, rst: &rcc.AHB2RSTR, bit: 1 << pos }
    }
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_periph_base: u32) -> u32 {
    FREQ_PERIPH
}

// Enable a GPIO peripheral clock
pub fn gpio_clock_enable(regs: *mut GPIO_TypeDef) {
    let rcc = unsafe { &*RCC };
    let rcc_pos = (regs as u32 - GPIOA_BASE) / 0x400;
    rcc.AHB2ENR.or(1 << rcc_pos);
    rcc.AHB2ENR.read();
}

// PLL (g4) input: 2.66 to 16Mhz, vco: 96 to 344Mhz, output: 2.06 to 170Mhz

#[cfg(not(CONFIG_STM32_CLOCK_REF_INTERNAL))]
decl_constant_str!("RESERVE_PINS_crystal", "PF0,PF1");

fn enable_clock_stm32g4() {
    let rcc = unsafe { &*RCC };
    let pll_freq = CONFIG_CLOCK_FREQ * 2;
    let pll_base: u32 = if CONFIG_STM32_CLOCK_REF_25M { 5000000 } else { 4000000 };
    let pllcfgr;

    if !CONFIG_STM32_CLOCK_REF_INTERNAL {
        // Configure PLL from external crystal (HSE)
        let div = CONFIG_CLOCK_REF_FREQ / pll_base - 1;
        rcc.CR.or(RCC_CR_HSEON);
        while rcc.CR.read() & RCC_CR_HSERDY == 0 {}
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
    } else {
        // Configure PLL from internal 16Mhz oscillator (HSI)
        let div = 16000000 / pll_base - 1;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
        rcc.CR.or(RCC_CR_HSION);
        while rcc.CR.read() & RCC_CR_HSIRDY == 0 {}
    }
    rcc.PLLCFGR.write(pllcfgr | ((pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLR_Pos));
    rcc.CR.or(RCC_CR_PLLON);

    // Enable 48Mhz USB clock using clock recovery
    if CONFIG_USB {
        rcc.CRRCR.or(RCC_CRRCR_HSI48ON);
        while rcc.CRRCR.read() & RCC_CRRCR_HSI48RDY == 0 {}
        enable_pclock(CRS_BASE);
        unsafe { &*CRS }.CR.or(CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
    }

    // Use PCLK for FDCAN
    rcc.CCIPR.write(2 << RCC_CCIPR_FDCANSEL_Pos);
}

// Main clock setup called at chip startup
fn clock_setup() {
    let rcc = unsafe { &*RCC };
    enable_clock_stm32g4();

    // Set flash latency
    let latency = if CONFIG_CLOCK_FREQ > 150000000 {
        FLASH_ACR_LATENCY_5WS
    } else if CONFIG_CLOCK_FREQ > 120000000 {
        FLASH_ACR_LATENCY_4WS
    } else if CONFIG_CLOCK_FREQ > 90000000 {
        FLASH_ACR_LATENCY_3WS
    } else if CONFIG_CLOCK_FREQ > 60000000 {
        FLASH_ACR_LATENCY_2WS
    } else if CONFIG_CLOCK_FREQ > 30000000 {
        FLASH_ACR_LATENCY_1WS
    } else {
        FLASH_ACR_LATENCY_0WS
    };
    unsafe { &*FLASH }.ACR.write(latency | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_DBG_SWEN);

    enable_pclock(PWR_BASE);
    let pwr = unsafe { &*PWR };
    pwr.CR3.or(PWR_CR3_APC); // allow gpio pullup/down
    if CONFIG_CLOCK_FREQ > 150000000 {
        // Enable "range 1 boost" mode
        pwr.CR5.write(0);
    }

    // Wait for PLL lock
    while rcc.CR.read() & RCC_CR_PLLRDY == 0 {}

    rcc.PLLCFGR.or(RCC_PLLCFGR_PLLREN);

    // Switch system clock to PLL
    rcc.CFGR.write(RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_SW_PLL);
    while rcc.CFGR.read() & RCC_CFGR_SWS_Msk != RCC_CFGR_SWS_PLL {}
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
    dfu_reboot_check();

    // Run SystemInit() and then restore VTOR
    SystemInit();
    unsafe { &*SCB }.VTOR.write(crate::ctr::VectorTable.as_ptr() as u32);

    clock_setup();

    sched_main();
}
