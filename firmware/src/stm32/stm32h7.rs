// Code to setup clocks on stm32h7
//
// Copyright (C) 2020 Konstantin Vogel <konstantin.vogel@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::system_stm32h7xx::SystemInit;
use crate::config::*;
use crate::decl_constant_str;
use crate::generic::armcm_reset::try_request_canboot;
use crate::generic::cmsis::{SCB_EnableDCache, SCB_EnableICache};
use crate::sched::sched_main;

/****************************************************************
 * Clock setup
 ****************************************************************/

const FREQ_PERIPH: u32 = CONFIG_CLOCK_FREQ / 4;

// Map a peripheral address to its enable bits
pub fn lookup_clock_line(periph_base: u32) -> Cline {
    let rcc = unsafe { &*RCC };
    if periph_base >= D3_AHB1PERIPH_BASE {
        let bit = 1 << ((periph_base - D3_AHB1PERIPH_BASE) / 0x400);
        Cline { en: &rcc.AHB4ENR, rst: &rcc.AHB4RSTR, bit }
    } else if periph_base >= D3_APB1PERIPH_BASE {
        let bit = 1 << ((periph_base - D3_APB1PERIPH_BASE) / 0x400);
        Cline { en: &rcc.APB4ENR, rst: &rcc.APB4RSTR, bit }
    } else if periph_base >= D1_AHB1PERIPH_BASE {
        let bit = 1 << ((periph_base - D1_AHB1PERIPH_BASE) / 0x400);
        Cline { en: &rcc.AHB3ENR, rst: &rcc.AHB3RSTR, bit }
    } else if periph_base >= D1_APB1PERIPH_BASE {
        let bit = 1 << ((periph_base - D1_APB1PERIPH_BASE) / 0x400);
        Cline { en: &rcc.APB3ENR, rst: &rcc.APB3RSTR, bit }
    } else if periph_base >= D2_AHB2PERIPH_BASE {
        let bit = 1 << ((periph_base - D2_AHB2PERIPH_BASE) / 0x400);
        Cline { en: &rcc.AHB2ENR, rst: &rcc.AHB2RSTR, bit }
    } else if periph_base >= D2_AHB1PERIPH_BASE {
        if periph_base == ADC12_COMMON_BASE {
            return Cline { en: &rcc.AHB1ENR, rst: &rcc.AHB1RSTR, bit: RCC_AHB1ENR_ADC12EN };
        }
        let bit = 1 << ((periph_base - D2_AHB1PERIPH_BASE) / 0x400);
        Cline { en: &rcc.AHB1ENR, rst: &rcc.AHB1RSTR, bit }
    } else if periph_base >= D2_APB2PERIPH_BASE {
        let bit = 1 << ((periph_base - D2_APB2PERIPH_BASE) / 0x400);
        Cline { en: &rcc.APB2ENR, rst: &rcc.APB2RSTR, bit }
    } else {
        if periph_base == FDCAN2_BASE {
            // FDCAN1 and FDCAN2 share same clock enable
            return Cline { en: &rcc.APB1HENR, rst: &rcc.APB1HRSTR, bit: RCC_APB1HENR_FDCANEN };
        }
        let offset = (periph_base - D2_APB1PERIPH_BASE) / 0x400;
        if offset < 32 {
            Cline { en: &rcc.APB1LENR, rst: &rcc.APB1LRSTR, bit: 1 << offset }
        } else {
            Cline { en: &rcc.APB1HENR, rst: &rcc.APB1HRSTR, bit: 1 << (offset - 32) }
        }
    }
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_periph_base: u32) -> u32 {
    FREQ_PERIPH
}

// Enable a GPIO peripheral clock
pub fn gpio_clock_enable(regs: *mut GPIO_TypeDef) {
    let rcc = unsafe { &*RCC };
    let pos = (regs as u32 - D3_AHB1PERIPH_BASE) / 0x400;
    rcc.AHB4ENR.or(1 << pos);
    rcc.AHB4ENR.read();
}

// PLL1 (h723) input: 2 to 16Mhz, vco: 192 to 836Mhz, output: 1.5 to 550Mhz
// PLL1 (h743v) input: 2 to 16Mhz, vco: 192 to 960Mhz, output: 1.5 to 480Mhz

#[cfg(not(CONFIG_STM32_CLOCK_REF_INTERNAL))]
decl_constant_str!("RESERVE_PINS_crystal", "PH0,PH1");

// Main clock and power setup called at chip startup
fn clock_setup() {
    let (rcc, pwr, flash) = unsafe { (&*RCC, &*PWR, &*FLASH) };
    // Enable low dropout regulator
    pwr.CR3.write(PWR_CR3_LDOEN);
    while pwr.CSR1.read() & PWR_CSR1_ACTVOSRDY == 0 {}

    // Setup pll1 frequency
    let mut pll_base: u32 = if CONFIG_STM32_CLOCK_REF_25M { 5000000 } else { 4000000 };
    let pll_freq = CONFIG_CLOCK_FREQ * if CONFIG_MACH_STM32H723 { 1 } else { 2 };
    let mut rcc_cr = RCC_CR_HSION;
    if !CONFIG_STM32_CLOCK_REF_INTERNAL {
        // Configure PLL from external crystal (HSE)
        rcc_cr |= RCC_CR_HSEON;
        rcc.CR.write(rcc_cr);
        while rcc.CR.read() & RCC_CR_HSERDY == 0 {}
        rcc.PLLCKSELR.write(RCC_PLLCKSELR_PLLSRC_HSE | ((CONFIG_CLOCK_REF_FREQ / pll_base) << RCC_PLLCKSELR_DIVM1_Pos));
    } else {
        // Configure PLL from internal 64Mhz oscillator (HSI)
        // HSI frequency of 64Mhz is integer divisible with 4Mhz
        pll_base = 4000000;
        rcc.PLLCKSELR.write(RCC_PLLCKSELR_PLLSRC_HSI | ((64000000 / pll_base) << RCC_PLLCKSELR_DIVM1_Pos));
    }
    // Set input frequency range of PLL1 according to pll_base
    // 3 = 8-16Mhz, 2 = 4-8Mhz
    rcc.PLLCFGR.write(
        (2 << RCC_PLLCFGR_PLL1RGE_Pos)
            // Enable PLL1Q (used by some peripherals)
            | RCC_PLLCFGR_DIVQ1EN
            // Enable PLL1P (for cpu clock)
            | RCC_PLLCFGR_DIVP1EN,
    );
    // Set multiplier DIVN1 and post divider DIVP1
    rcc.PLL1DIVR.write(
        ((pll_freq / pll_base - 1) << RCC_PLL1DIVR_N1_Pos)
            // Set PLL1Q frequency (some peripherals directly use pll1_q_ck)
            | ((pll_freq / FREQ_PERIPH - 1) << RCC_PLL1DIVR_Q1_Pos)
            // Set PLL1P cpu clock frequency
            | ((pll_freq / CONFIG_CLOCK_FREQ - 1) << RCC_PLL1DIVR_P1_Pos),
    );

    // Enable VOS1 power mode (or VOS0 if freq>400Mhz)
    if CONFIG_MACH_STM32H723 {
        pwr.D3CR.write((if CONFIG_CLOCK_FREQ > 400000000 { 0 } else { 3 }) << PWR_D3CR_VOS_Pos);
        while pwr.D3CR.read() & PWR_D3CR_VOSRDY == 0 {}
    } else {
        pwr.D3CR.write(3 << PWR_D3CR_VOS_Pos);
        while pwr.D3CR.read() & PWR_D3CR_VOSRDY == 0 {}
        if CONFIG_CLOCK_FREQ > 400000000 {
            enable_pclock(SYSCFG as u32);
            #[cfg(has_SYSCFG_PWRCR_ODEN)]
            unsafe { &*SYSCFG }.PWRCR.or(SYSCFG_PWRCR_ODEN);
            while pwr.D3CR.read() & PWR_D3CR_VOSRDY == 0 {}
        }
    }

    SCB_EnableICache();
    SCB_EnableDCache();

    // Set flash latency according to clock frequency
    let flash_acr_latency = if CONFIG_CLOCK_FREQ > 450000000 { FLASH_ACR_LATENCY_4WS } else { FLASH_ACR_LATENCY_2WS };
    flash.ACR.write(flash_acr_latency | (2 << FLASH_ACR_WRHIGHFREQ_Pos));
    while flash.ACR.read() & flash_acr_latency == 0 {}

    // Set HPRE clock to div 2 (CONFIG_CLOCK_FREQ/2) and set
    // peripheral clocks another div 2 (CONFIG_CLOCK_FREQ/4)
    rcc.D1CFGR.write(RCC_D1CFGR_HPRE_DIV2 | RCC_D1CFGR_D1PPRE_DIV2);
    rcc.D2CFGR.write(RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2);
    rcc.D3CFGR.write(RCC_D3CFGR_D3PPRE_DIV2);

    // Switch on PLL1
    rcc_cr |= RCC_CR_PLL1ON;
    rcc.CR.write(rcc_cr);
    while rcc.CR.read() & RCC_CR_PLL1RDY == 0 {}

    // Switch system clock source (SYSCLK) to PLL1
    rcc.CFGR.write(RCC_CFGR_SW_PLL1);
    while rcc.CFGR.read() & RCC_CFGR_SWS_Msk != RCC_CFGR_SWS_PLL1 {}

    // Set the source of FDCAN clock to pll1_q_ck
    rcc.D2CCIP1R.write(1 << RCC_D2CCIP1R_FDCANSEL_Pos);

    // Configure USB to use HSI48 clock
    if CONFIG_USB {
        rcc_cr |= RCC_CR_HSI48ON;
        rcc.CR.write(rcc_cr);
        while rcc.CR.read() & RCC_CR_HSI48RDY == 0 {}
        enable_pclock(CRS as u32);
        let crs = unsafe { &*CRS };
        crs.CFGR.and(!CRS_CFGR_SYNCSRC);
        crs.CR.write((crs.CR.read() & CRS_CR_TRIM) | CRS_CR_CEN | CRS_CR_AUTOTRIMEN);
        rcc.D2CCIP2R.write(RCC_D2CCIP2R_USBSEL);
    }
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
    let rcc = unsafe { &*RCC };
    // Run SystemInit() and then restore VTOR
    SystemInit();
    rcc.D1CCIPR.write(0x00000000);
    rcc.D2CCIP1R.write(0x00000000);
    rcc.D2CCIP2R.write(0x00000000);
    rcc.D3CCIPR.write(0x00000000);
    rcc.APB1LENR.write(0x00000000);
    rcc.APB1HENR.write(0x00000000);
    rcc.APB2ENR.write(0x00000000);
    rcc.APB3ENR.write(0x00000000);
    rcc.APB4ENR.write(0x00000000);
    rcc.AHB1ENR.write(0x00000000);

    unsafe { &*SCB }.VTOR.write(crate::ctr::VectorTable.as_ptr() as u32);

    dfu_reboot_check();

    clock_setup();

    sched_main();
}
