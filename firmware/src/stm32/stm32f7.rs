// Code to setup clocks on stm32f7
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::system_stm32f7xx::SystemInit;
use crate::config::*;
use crate::decl_constant_str;
use crate::generic::armcm_reset::try_request_canboot;
use crate::sched::sched_main;

/****************************************************************
 * Clock setup
 ****************************************************************/

const FREQ_PERIPH_DIV: u32 = 4;
const FREQ_PERIPH: u32 = CONFIG_CLOCK_FREQ / FREQ_PERIPH_DIV;
const FREQ_USB: u32 = 48000000;

// Map a peripheral address to its enable bits
pub fn lookup_clock_line(periph_base: u32) -> Cline {
    let rcc = unsafe { &*RCC };
    if periph_base >= AHB1PERIPH_BASE {
        let bit = 1 << ((periph_base - AHB1PERIPH_BASE) / 0x400);
        Cline { en: &rcc.AHB1ENR, rst: &rcc.AHB1RSTR, bit }
    } else if periph_base >= APB2PERIPH_BASE {
        let bit = 1 << ((periph_base - APB2PERIPH_BASE) / 0x400);
        if bit & 0x700 != 0 {
            // Skip ADC peripheral reset as they share a bit
            return Cline { en: &rcc.APB2ENR, rst: core::ptr::null(), bit };
        }
        Cline { en: &rcc.APB2ENR, rst: &rcc.APB2RSTR, bit }
    } else {
        let bit = 1 << ((periph_base - APB1PERIPH_BASE) / 0x400);
        Cline { en: &rcc.APB1ENR, rst: &rcc.APB1RSTR, bit }
    }
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_periph_base: u32) -> u32 {
    FREQ_PERIPH
}

// Enable a GPIO peripheral clock
pub fn gpio_clock_enable(regs: *mut GPIO_TypeDef) {
    let rcc = unsafe { &*RCC };
    let rcc_pos = (regs as u32 - AHB1PERIPH_BASE) / 0x400;
    rcc.AHB1ENR.or(1 << rcc_pos);
    rcc.AHB1ENR.read();
}

// PLL (f765) input: 0.95 to 2.1Mhz, vco: 100 to 432Mhz, output: 24 to 216Mhz

#[cfg(not(CONFIG_STM32_CLOCK_REF_INTERNAL))]
decl_constant_str!("RESERVE_PINS_crystal", "PH0,PH1");

// Main clock setup called at chip startup
fn clock_setup() {
    let rcc = unsafe { &*RCC };
    // Configure and enable PLL
    let (pll_base, pll_freq): (u32, u32) = (2000000, CONFIG_CLOCK_FREQ * 2);
    let pllcfgr;
    if !CONFIG_STM32_CLOCK_REF_INTERNAL {
        // Configure 216Mhz PLL from external crystal (HSE)
        let div = CONFIG_CLOCK_REF_FREQ / pll_base;
        rcc.CR.or(RCC_CR_HSEON);
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
    } else {
        // Configure 216Mhz PLL from internal 16Mhz oscillator (HSI)
        let div = 16000000 / pll_base;
        pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
    }
    rcc.PLLCFGR.write(
        pllcfgr
            | ((pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos)
            | (0 << RCC_PLLCFGR_PLLP_Pos) //  /2
            | ((pll_freq / FREQ_USB) << RCC_PLLCFGR_PLLQ_Pos)
            | (2 << RCC_PLLCFGR_PLLR_Pos),
    );
    rcc.CR.or(RCC_CR_PLLON);

    // Enable "over drive"
    enable_pclock(PWR_BASE);
    let pwr = unsafe { &*PWR };
    pwr.CR1.write((3 << PWR_CR1_VOS_Pos) | PWR_CR1_ODEN);
    while pwr.CSR1.read() & PWR_CSR1_ODRDY == 0 {}
    pwr.CR1.write((3 << PWR_CR1_VOS_Pos) | PWR_CR1_ODEN | PWR_CR1_ODSWEN);
    while pwr.CSR1.read() & PWR_CSR1_ODSWRDY == 0 {}

    // Enable 48Mhz USB clock
    if CONFIG_USB {
        // setup PLLSAI
        let (plls_base, plls_freq): (u32, u32) = (2000000, FREQ_USB * 4);
        rcc.PLLSAICFGR.write(
            ((plls_freq / plls_base) << RCC_PLLSAICFGR_PLLSAIN_Pos) // *96
                | (((plls_freq / FREQ_USB) / 2 - 1) << RCC_PLLSAICFGR_PLLSAIP_Pos) // /4
                | ((plls_freq / FREQ_USB) << RCC_PLLSAICFGR_PLLSAIQ_Pos),
        );
        // enable PLLSAI and wait for PLLSAI lock
        rcc.CR.or(RCC_CR_PLLSAION);
        while rcc.CR.read() & RCC_CR_PLLSAIRDY == 0 {}
        // set CLK48 source to PLLSAI
        rcc.DCKCFGR2.write(RCC_DCKCFGR2_CK48MSEL); // RCC_CLK48SOURCE_PLLSAIP
    }

    // Set flash latency
    unsafe { &*FLASH }.ACR.modify(FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_7WS);

    // Wait for PLL lock
    while rcc.CR.read() & RCC_CR_PLLRDY == 0 {}

    // Switch system clock to PLL
    rcc.CFGR.write(RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV4 | RCC_CFGR_SW_PLL);
    while rcc.CFGR.read() & RCC_CFGR_SWS_Msk != RCC_CFGR_SWS_PLL {}
}

/****************************************************************
 * Bootloader
 ****************************************************************/

// Handle reboot requests
pub fn bootloader_request() {
    try_request_canboot();
    dfu_reboot();
}

/****************************************************************
 * Startup
 ****************************************************************/

// Main entry point - called from armcm_boot.c:ResetHandler()
pub fn armcm_main() -> ! {
    // Run SystemInit() and then restore VTOR
    SystemInit();
    unsafe { &*SCB }.VTOR.write(crate::ctr::VectorTable.as_ptr() as u32);

    // Reset peripheral clocks (for some bootloaders that don't)
    let rcc = unsafe { &*RCC };
    rcc.AHB1ENR.write(0x00100000);
    rcc.AHB2ENR.write(0x00000000);
    rcc.AHB3ENR.write(0x00000000);
    rcc.APB1ENR.write(0x00000400);
    rcc.APB2ENR.write(0x00000000);

    dfu_reboot_check();

    // STM32F7 specific DWT unlock required prior to timer_init() DWT setup.
    unsafe { &*CoreDebug }.DEMCR.or(CoreDebug_DEMCR_TRCENA_Msk);
    unsafe { &*DWT }.LAR.write(0xC5ACCE55);

    clock_setup();

    sched_main();
}
