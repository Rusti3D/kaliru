// Code to setup clocks on stm32f0
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::system_stm32f0xx::SystemInit;
use crate::config::*;
use crate::decl_constant_str;
use crate::generic::armcm_reset::try_request_canboot;
use crate::sched::sched_main;
use core::ptr::addr_of_mut;

/****************************************************************
 * Clock setup
 ****************************************************************/

const FREQ_PERIPH: u32 = 48000000;

// Map a peripheral address to its enable bits
pub fn lookup_clock_line(periph_base: u32) -> Cline {
    let rcc = unsafe { &*RCC };
    if periph_base >= AHB2PERIPH_BASE {
        let bit = 1 << ((periph_base - AHB2PERIPH_BASE) / 0x400 + 17);
        Cline { en: &rcc.AHBENR, rst: &rcc.AHBRSTR, bit }
    } else if periph_base >= SYSCFG_BASE {
        let bit = 1 << ((periph_base - SYSCFG_BASE) / 0x400);
        Cline { en: &rcc.APB2ENR, rst: &rcc.APB2RSTR, bit }
    } else {
        let bit = 1 << ((periph_base - APBPERIPH_BASE) / 0x400);
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
    let rcc_pos = (regs as u32 - AHB2PERIPH_BASE) / 0x400;
    rcc.AHBENR.or(1 << (rcc_pos + 17));
    rcc.AHBENR.read();
}

// PLL (f0) input: 1 to 24Mhz, output: 16 to 48Mhz

#[cfg(not(CONFIG_STM32_CLOCK_REF_INTERNAL))]
decl_constant_str!("RESERVE_PINS_crystal", "PF0,PF1");

// Configure and enable the PLL as clock source
fn pll_setup() {
    let rcc = unsafe { &*RCC };
    let cfgr;
    if !CONFIG_STM32_CLOCK_REF_INTERNAL {
        // Configure 48Mhz PLL from external crystal (HSE)
        let div = CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ;
        rcc.CR.write((rcc.CR.read() & !RCC_CR_HSITRIM) | RCC_CR_HSEON | (CONFIG_STM32F0_TRIM << RCC_CR_HSITRIM_Pos));
        cfgr = RCC_CFGR_PLLSRC_HSE_PREDIV | ((div - 2) << RCC_CFGR_PLLMUL_Pos);
    } else {
        // Configure 48Mhz PLL from internal 8Mhz oscillator (HSI)
        let div2 = (CONFIG_CLOCK_FREQ / 8000000) * 2;
        cfgr = RCC_CFGR_PLLSRC_HSI_DIV2 | ((div2 - 2) << RCC_CFGR_PLLMUL_Pos);
    }
    rcc.CFGR.write(cfgr);
    rcc.CR.or(RCC_CR_PLLON);

    // Wait for PLL lock
    while rcc.CR.read() & RCC_CR_PLLRDY == 0 {}

    // Switch system clock to PLL
    rcc.CFGR.write(cfgr | RCC_CFGR_SW_PLL);
    while rcc.CFGR.read() & RCC_CFGR_SWS_Msk != RCC_CFGR_SWS_PLL {}

    // Setup CFGR3 register
    #[allow(unused_mut)]
    let mut cfgr3 = RCC_CFGR3_I2C1SW;
    // Select PLL as source for USB clock
    #[cfg(CONFIG_USB)]
    {
        cfgr3 |= RCC_CFGR3_USBSW;
    }
    rcc.CFGR3.write(cfgr3);
}

// Configure and enable internal 48Mhz clock on the stm32f042
fn hsi48_setup() {
    #[cfg(CONFIG_MACH_STM32F0x2)]
    {
        let rcc = unsafe { &*RCC };
        // Enable HSI48
        rcc.CR2.or(RCC_CR2_HSI48ON);
        while rcc.CR2.read() & RCC_CR2_HSI48RDY == 0 {}

        // Switch system clock to HSI48
        rcc.CFGR.write(RCC_CFGR_SW_HSI48);
        while rcc.CFGR.read() & RCC_CFGR_SWS_Msk != RCC_CFGR_SWS_HSI48 {}

        // Enable USB clock recovery
        if CONFIG_USB {
            enable_pclock(CRS_BASE);
            unsafe { &*CRS }.CR.or(CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
        }

        // Setup I2C1 clock
        rcc.CFGR3.write(RCC_CFGR3_I2C1SW);
    }
}

// Enable high speed internal 14Mhz clock for ADC
fn hsi14_setup() {
    let rcc = unsafe { &*RCC };
    // Enable HSI14 for ADC
    rcc.CR2.write(RCC_CR2_HSI14ON);
    while rcc.CR2.read() & RCC_CR2_HSI14RDY == 0 {}
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

// Symbols created by armcm_link.lds.S linker script
extern "C" {
    static mut _ram_vectortable_start: u32;
    static mut _ram_vectortable_end: u32;
    static mut _text_vectortable_start: u32;
}

// Copy vector table and remap ram so new vector table is used
fn enable_ram_vectortable() {
    let start = addr_of_mut!(_ram_vectortable_start);
    let count = (addr_of_mut!(_ram_vectortable_end) as usize - start as usize) / 4 * 4;
    unsafe { core::ptr::copy_nonoverlapping(addr_of_mut!(_text_vectortable_start) as *const u8, start as *mut u8, count) };
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    unsafe { &*SYSCFG }.CFGR1.or(3 << SYSCFG_CFGR1_MEM_MODE_Pos);
}

// Main entry point - called from armcm_boot.c:ResetHandler()
pub fn armcm_main() -> ! {
    dfu_reboot_check();
    SystemInit();

    enable_pclock(SYSCFG_BASE);
    if CONFIG_ARMCM_RAM_VECTORTABLE {
        enable_ram_vectortable();
    }

    // Set flash latency
    unsafe { &*FLASH }.ACR.write((1 << FLASH_ACR_LATENCY_Pos) | FLASH_ACR_PRFTBE);

    // Configure main clock
    if CONFIG_MACH_STM32F0x2 && CONFIG_STM32_CLOCK_REF_INTERNAL && CONFIG_USB {
        hsi48_setup();
    } else {
        pll_setup();
    }

    // Turn on hsi14 oscillator for ADC
    hsi14_setup();

    // Support pin remapping USB/CAN pins on low pinout stm32f042
    if CONFIG_STM32_USB_PA11_PA12_REMAP || CONFIG_STM32_CANBUS_PA11_PA12_REMAP {
        unsafe { &*SYSCFG }.CFGR1.or(1 << 4); // SYSCFG_CFGR1_PA11_PA12_RMP
    }

    sched_main();
}
