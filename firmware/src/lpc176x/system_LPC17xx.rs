// CMSIS Cortex-M3 Device Peripheral Access Layer Source File for the
// NXP LPC17xx Device Series (lib/lpc176x/device/system_LPC17xx.c, with
// Kalico's lpc176x.patch)
//
// Copyright (C) 2009 ARM Limited. All rights reserved.
#![allow(non_upper_case_globals)]
use super::internal::*;
use crate::config::CONFIG_MACH_LPC1769;
use crate::ctr::Global;

/*--------------------- Clock Configuration ----------------------------------*/
const CLOCK_SETUP: bool = true;
const SCS_Val: u32 = 0x00000020;
const CLKSRCSEL_Val: u32 = 0x00000001;
const PLL0_SETUP: bool = true;

const PLL0CFG_Val: u32 = if CONFIG_MACH_LPC1769 { 0x0000000E } else { 0x00010018 };
const PLL1_SETUP: bool = true;
const PLL1CFG_Val: u32 = 0x00000023;
const CCLKCFG_Val: u32 = 0x00000002;
const USBCLKCFG_Val: u32 = 0x00000000;

const PCLKSEL0_Val: u32 = 0x55515155;
const PCLKSEL1_Val: u32 = 0x54555455;
const PCONP_Val: u32 = 0x042887DE;
const CLKOUTCFG_Val: u32 = 0x00000000;

/*--------------------- Flash Accelerator Configuration ----------------------*/
const FLASH_SETUP: bool = true;
const FLASHCFG_Val: u32 = 0x0000303A;

/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
const XTAL: u32 = 12000000; /* Oscillator frequency               */
const OSC_CLK: u32 = XTAL; /* Main oscillator frequency          */
const RTC_CLK: u32 = 32000; /* RTC oscillator frequency           */
const IRC_OSC: u32 = 4000000; /* Internal RC oscillator frequency   */

/* F_cco0 = (2 * M * F_in) / N  */
const __M: u64 = ((PLL0CFG_Val & 0x7FFF) + 1) as u64;
const __N: u64 = (((PLL0CFG_Val >> 16) & 0x00FF) + 1) as u64;
const fn __FCCO(f_in: u32) -> u64 {
    (2 * __M * f_in as u64) / __N
}
const __CCLK_DIV: u64 = ((CCLKCFG_Val & 0x00FF) + 1) as u64;

/* Determine core clock frequency according to settings */
// (PLL0_SETUP, CLKSRCSEL_Val 1: the main oscillator)
const __CORE_CLK: u32 = (__FCCO(OSC_CLK) / __CCLK_DIV) as u32;

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
// System Clock Frequency (Core Clock)
pub static SystemCoreClock: Global<u32> = Global::new(__CORE_CLK);

fn sc() -> &'static LPC_SC_TypeDef {
    unsafe { &*LPC_SC }
}

// Update SystemCoreClock according to Clock Register Values
pub fn SystemCoreClockUpdate() {
    let sc = sc();
    let pll = |f: u32| -> u32 {
        // (C: unsigned long long products, stored to a uint32_t)
        ((f as u64 * (2 * ((sc.PLL0STAT.read() & 0x7FFF) + 1) as u64))
            / (((sc.PLL0STAT.read() >> 16) & 0xFF) + 1) as u64
            / ((sc.CCLKCFG.read() & 0xFF) + 1) as u64) as u32
    };
    let clk = SystemCoreClock.get();
    if ((sc.PLL0STAT.read() >> 24) & 3) == 3 {
        /* If PLL0 enabled and connected */
        match sc.CLKSRCSEL.read() & 0x03 {
            0 | 3 => *clk = pll(IRC_OSC), /* Int. RC oscillator => PLL0 */
            1 => *clk = pll(OSC_CLK),     /* Main oscillator => PLL0 */
            2 => *clk = pll(RTC_CLK),     /* RTC oscillator => PLL0 */
            _ => {}
        }
    } else {
        match sc.CLKSRCSEL.read() & 0x03 {
            0 | 3 => *clk = IRC_OSC / ((sc.CCLKCFG.read() & 0xFF) + 1),
            1 => *clk = OSC_CLK / ((sc.CCLKCFG.read() & 0xFF) + 1),
            2 => *clk = RTC_CLK / ((sc.CCLKCFG.read() & 0xFF) + 1),
            _ => {}
        }
    }
}

// Setup the microcontroller system.
// Initialize the System.
pub fn SystemInit() {
    let sc = sc();
    if CLOCK_SETUP {
        /* Clock Setup */
        sc.SCS.write(SCS_Val);
        if sc.SCS.read() & (1 << 5) != 0 {
            /* If Main Oscillator is enabled  */
            while (sc.SCS.read() & (1 << 6)) == 0 {} /* Wait for Oscillator to be ready    */
        }

        sc.CCLKCFG.write(CCLKCFG_Val); /* Setup Clock Divider                */
        /* Periphral clock must be selected before PLL0 enabling and connecting
         * - according errata.lpc1768-16.March.2010 -
         */
        sc.PCLKSEL0.write(PCLKSEL0_Val); /* Peripheral Clock Selection         */
        sc.PCLKSEL1.write(PCLKSEL1_Val);

        if PLL0_SETUP {
            sc.CLKSRCSEL.write(CLKSRCSEL_Val); /* Select Clock Source for PLL0       */

            sc.PLL0CFG.write(PLL0CFG_Val); /* configure PLL0                     */
            sc.PLL0FEED.write(0xAA);
            sc.PLL0FEED.write(0x55);

            sc.PLL0CON.write(0x01); /* PLL0 Enable                        */
            sc.PLL0FEED.write(0xAA);
            sc.PLL0FEED.write(0x55);
            while sc.PLL0STAT.read() & (1 << 26) == 0 {} /* Wait for PLOCK0                    */

            sc.PLL0CON.write(0x03); /* PLL0 Enable & Connect              */
            sc.PLL0FEED.write(0xAA);
            sc.PLL0FEED.write(0x55);
            while sc.PLL0STAT.read() & ((1 << 25) | (1 << 24)) == 0 {} /* Wait for PLLC0_STAT & PLLE0_STAT */
        }

        if PLL1_SETUP {
            sc.PLL1CFG.write(PLL1CFG_Val);
            sc.PLL1FEED.write(0xAA);
            sc.PLL1FEED.write(0x55);

            sc.PLL1CON.write(0x01); /* PLL1 Enable                        */
            sc.PLL1FEED.write(0xAA);
            sc.PLL1FEED.write(0x55);
            while sc.PLL1STAT.read() & (1 << 10) == 0 {} /* Wait for PLOCK1                    */

            sc.PLL1CON.write(0x03); /* PLL1 Enable & Connect              */
            sc.PLL1FEED.write(0xAA);
            sc.PLL1FEED.write(0x55);
            while sc.PLL1STAT.read() & ((1 << 9) | (1 << 8)) == 0 {} /* Wait for PLLC1_STAT & PLLE1_STAT */
        } else {
            sc.USBCLKCFG.write(USBCLKCFG_Val); /* Setup USB Clock Divider            */
        }

        sc.PCONP.write(PCONP_Val); /* Power Control for Peripherals      */

        sc.CLKOUTCFG.write(CLKOUTCFG_Val); /* Clock Output Configuration         */
    }

    if FLASH_SETUP {
        /* Flash Accelerator Setup            */
        sc.FLASHCFG.write((sc.FLASHCFG.read() & !0x0000F000) | FLASHCFG_Val);
    }
}
