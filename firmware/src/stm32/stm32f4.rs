// Code to setup clocks on stm32f2/stm32f4
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
#[cfg(CONFIG_MACH_STM32F2)]
use super::system_stm32f2xx::SystemInit;
#[cfg(CONFIG_MACH_STM32F4)]
use super::system_stm32f4xx::SystemInit;
use crate::board::irq_disable;
use crate::config::*;
use crate::decl_constant_str;
use crate::generic::armcm_reset::try_request_canboot;
use crate::generic::cmsis::NVIC_SystemReset;
use crate::sched::sched_main;

/****************************************************************
 * Clock setup
 ****************************************************************/

const FREQ_PERIPH_DIV: u32 = if CONFIG_MACH_STM32F401 || CONFIG_MACH_STM32F411 { 2 } else { 4 };
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

// PLL (f207) input: 0.95 to 2.1Mhz, vco: 192 to 432Mhz, output: 24 to 120Mhz
// PLL (f401) input: 0.95 to 2.1Mhz, vco: 192 to 432Mhz, output: 24 to 84Mhz
// PLL (f405/7) input: 0.95 to 2.1Mhz, vco: 100 to 432Mhz, output: 24 to 168Mhz
// PLL (f446) input: 0.95 to 2.1Mhz, vco: 100 to 432Mhz, output: 12.5 to 180Mhz

#[cfg(not(CONFIG_STM32_CLOCK_REF_INTERNAL))]
decl_constant_str!("RESERVE_PINS_crystal", "PH0,PH1");

// Clock configuration
fn enable_clock_stm32f20x() {
    #[cfg(CONFIG_MACH_STM32F207)]
    {
        let rcc = unsafe { &*RCC };
        let (pll_base, pll_freq): (u32, u32) = (1000000, CONFIG_CLOCK_FREQ * 2);
        let pllcfgr;
        if !CONFIG_STM32_CLOCK_REF_INTERNAL {
            // Configure 120Mhz PLL from external crystal (HSE)
            let div = CONFIG_CLOCK_REF_FREQ / pll_base;
            rcc.CR.or(RCC_CR_HSEON);
            pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
        } else {
            // Configure 120Mhz PLL from internal 16Mhz oscillator (HSI)
            let div = 16000000 / pll_base;
            pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
        }
        rcc.PLLCFGR.write(
            pllcfgr
                | ((pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos)
                | (0 << RCC_PLLCFGR_PLLP_Pos)
                | ((pll_freq / FREQ_USB) << RCC_PLLCFGR_PLLQ_Pos),
        );
        rcc.CR.or(RCC_CR_PLLON);
    }
}

fn enable_clock_stm32f40x() {
    #[cfg(any(CONFIG_MACH_STM32F401, CONFIG_MACH_STM32F411, CONFIG_MACH_STM32F4x5))]
    {
        let rcc = unsafe { &*RCC };
        let pll_base: u32 = if CONFIG_STM32_CLOCK_REF_25M { 1000000 } else { 2000000 };
        let pllp: u32 = if CONFIG_MACH_STM32F401 || CONFIG_MACH_STM32F411 { 4 } else { 2 };
        let pll_freq = CONFIG_CLOCK_FREQ * pllp;
        let pllcfgr;
        if !CONFIG_STM32_CLOCK_REF_INTERNAL {
            // Configure 168Mhz PLL from external crystal (HSE)
            let div = CONFIG_CLOCK_REF_FREQ / pll_base;
            rcc.CR.or(RCC_CR_HSEON);
            pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
        } else {
            // Configure 168Mhz PLL from internal 16Mhz oscillator (HSI)
            let div = 16000000 / pll_base;
            pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
        }
        rcc.PLLCFGR.write(
            pllcfgr
                | ((pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos)
                | (((pllp >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos)
                | ((pll_freq / FREQ_USB) << RCC_PLLCFGR_PLLQ_Pos),
        );
        rcc.CR.or(RCC_CR_PLLON);
    }
}

fn enable_clock_stm32f446() {
    #[cfg(CONFIG_MACH_STM32F446)]
    {
        let rcc = unsafe { &*RCC };
        let (pll_base, pll_freq): (u32, u32) = (2000000, CONFIG_CLOCK_FREQ * 2);
        let pllcfgr;
        if !CONFIG_STM32_CLOCK_REF_INTERNAL {
            // Configure 180Mhz PLL from external crystal (HSE)
            let div = CONFIG_CLOCK_REF_FREQ / pll_base;
            rcc.CR.or(RCC_CR_HSEON);
            pllcfgr = RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
        } else {
            // Configure 180Mhz PLL from internal 16Mhz oscillator (HSI)
            let div = 16000000 / pll_base;
            pllcfgr = RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
        }
        rcc.PLLCFGR.write(
            pllcfgr
                | ((pll_freq / pll_base) << RCC_PLLCFGR_PLLN_Pos)
                | (0 << RCC_PLLCFGR_PLLP_Pos)
                | ((pll_freq / FREQ_USB) << RCC_PLLCFGR_PLLQ_Pos)
                | (6 << RCC_PLLCFGR_PLLR_Pos),
        );
        rcc.CR.or(RCC_CR_PLLON);

        // Enable "over drive"
        enable_pclock(PWR_BASE);
        let pwr = unsafe { &*PWR };
        pwr.CR.write((3 << PWR_CR_VOS_Pos) | PWR_CR_ODEN);
        while pwr.CSR.read() & PWR_CSR_ODRDY == 0 {}
        pwr.CR.write((3 << PWR_CR_VOS_Pos) | PWR_CR_ODEN | PWR_CR_ODSWEN);
        while pwr.CSR.read() & PWR_CSR_ODSWRDY == 0 {}

        // Enable 48Mhz USB clock for USB or for SDIO
        if CONFIG_USB || CONFIG_HAVE_GPIO_SDIO {
            let r#ref = if CONFIG_STM32_CLOCK_REF_INTERNAL { 16000000 } else { CONFIG_CLOCK_REF_FREQ };
            let (plls_base, plls_freq): (u32, u32) = (2000000, FREQ_USB * 4);
            rcc.PLLSAICFGR.write(
                ((r#ref / plls_base) << RCC_PLLSAICFGR_PLLSAIM_Pos)
                    | ((plls_freq / plls_base) << RCC_PLLSAICFGR_PLLSAIN_Pos)
                    | (((plls_freq / FREQ_USB) / 2 - 1) << RCC_PLLSAICFGR_PLLSAIP_Pos)
                    | ((plls_freq / FREQ_USB) << RCC_PLLSAICFGR_PLLSAIQ_Pos),
            );
            rcc.CR.or(RCC_CR_PLLSAION);
            while rcc.CR.read() & RCC_CR_PLLSAIRDY == 0 {}

            rcc.DCKCFGR2.write(RCC_DCKCFGR2_CK48MSEL);
        } else {
            // Reset value just in case the booloader modified the default value
            rcc.DCKCFGR2.write(0);
        }

        // Set SDIO clk to PLL48CLK
        if CONFIG_HAVE_GPIO_SDIO {
            rcc.DCKCFGR2.modify(RCC_DCKCFGR2_SDIOSEL, 0);
        }
    }
}

// Main clock setup called at chip startup
fn clock_setup() {
    let rcc = unsafe { &*RCC };
    // Set voltage scaling to support 96MHz for F411
    #[cfg(CONFIG_MACH_STM32F411)]
    {
        enable_pclock(PWR_BASE);
        unsafe { &*PWR }.CR.modify(PWR_CR_VOS_Msk, 3 << PWR_CR_VOS_Pos);
    }

    // Configure and enable PLL
    if CONFIG_MACH_STM32F207 {
        enable_clock_stm32f20x();
    } else if CONFIG_MACH_STM32F401 || CONFIG_MACH_STM32F411 || CONFIG_MACH_STM32F4x5 {
        enable_clock_stm32f40x();
    } else {
        enable_clock_stm32f446();
    }

    // Set flash latency
    let flash = unsafe { &*FLASH };
    if CONFIG_MACH_STM32F411 {
        flash.ACR.write(FLASH_ACR_LATENCY_3WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
    } else {
        flash.ACR.write(FLASH_ACR_LATENCY_5WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
    }

    // Wait for PLL lock
    while rcc.CR.read() & RCC_CR_PLLRDY == 0 {}

    // Switch system clock to PLL
    if FREQ_PERIPH_DIV == 2 {
        rcc.CFGR.write(RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_SW_PLL);
    } else {
        rcc.CFGR.write(RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV4 | RCC_CFGR_SW_PLL);
    }
    while rcc.CFGR.read() & RCC_CFGR_SWS_Msk != RCC_CFGR_SWS_PLL {}
}

/****************************************************************
 * Bootloader
 ****************************************************************/

// Reboot into USB "HID" bootloader
fn usb_hid_bootloader() {
    irq_disable();
    let (rcc, pwr) = unsafe { (&*RCC, &*PWR) };
    rcc.APB1ENR.or(RCC_APB1ENR_PWREN);
    rcc.APB1ENR.read();
    pwr.CR.or(PWR_CR_DBP);
    unsafe { &*RTC }.BKP4R.write(0x424C); // HID Bootloader magic key
    pwr.CR.and(!PWR_CR_DBP);
    NVIC_SystemReset();
}

// Handle reboot requests
pub fn bootloader_request() {
    try_request_canboot();
    if CONFIG_STM32_FLASH_START_4000 {
        usb_hid_bootloader();
    }
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

    // Reset peripheral clocks (for some bootloaders that don't)
    let rcc = unsafe { &*RCC };
    rcc.AHB1ENR.write(0x38000);
    rcc.AHB2ENR.write(0);
    rcc.APB1ENR.write(0);
    rcc.APB2ENR.write(0);

    clock_setup();

    sched_main();
}
