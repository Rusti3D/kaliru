// Startup code on rp2040
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::config::{CONFIG_CLOCK_FREQ, CONFIG_MACH_RP2040};
use crate::generic::armcm_reset::try_request_canboot;
use crate::sched::sched_main;
use core::ptr::addr_of_mut;

/****************************************************************
 * Ram IRQ vector table
 ****************************************************************/

// Symbols created by rpxxxx_link.lds.S linker script
extern "C" {
    static mut _ram_vectortable_start: u32;
    static mut _ram_vectortable_end: u32;
    static mut _text_vectortable_start: u32;
}

// Copy vector table to ram and activate it
fn enable_ram_vectortable() {
    let start = addr_of_mut!(_ram_vectortable_start);
    let count = (addr_of_mut!(_ram_vectortable_end) as usize - start as usize) / 4 * 4;
    unsafe { core::ptr::copy_nonoverlapping(addr_of_mut!(_text_vectortable_start) as *const u8, start as *mut u8, count) };
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    unsafe { &*SCB }.VTOR.write(start as u32);
}

/****************************************************************
 * Bootloader
 ****************************************************************/

pub fn bootloader_request() {
    unsafe { &*watchdog_hw }.ctrl.write(0);
    try_request_canboot();
    bootrom_reboot_usb_bootloader();
}

/****************************************************************
 * Clock setup
 ****************************************************************/

const FREQ_XOSC: u32 = 12000000;
const FREQ_SYS: u32 = if CONFIG_MACH_RP2040 { 200000000 } else { CONFIG_CLOCK_FREQ };
const FBDIV: u32 = if FREQ_SYS == 200000000 { 100 } else { 125 };
const FREQ_USB: u32 = 48000000;

pub fn set_vsel() {
    // Set internal voltage regulator output to 1.15V on rp2040
    #[cfg(CONFIG_MACH_RP2040)]
    {
        let vreg = unsafe { &*vreg_and_chip_reset_hw };
        let mut cval = vreg.vreg.read();
        let vref = VREG_AND_CHIP_RESET_VREG_VSEL_RESET + 1;
        cval &= !VREG_AND_CHIP_RESET_VREG_VSEL_BITS;
        cval |= vref << VREG_AND_CHIP_RESET_VREG_VSEL_LSB;
        vreg.vreg.write(cval);
    }
}

fn resets() -> &'static resets_hw_t {
    unsafe { &*resets_hw }
}

pub fn enable_pclock(reset_bit: u32) {
    resets().reset.or(reset_bit);
    resets().reset.and(!reset_bit);
    while resets().reset_done.read() & reset_bit == 0 {}
}

pub fn is_enabled_pclock(reset_bit: u32) -> bool {
    resets().reset.read() & reset_bit == 0
}

pub fn get_pclock_frequency(_reset_bit: u32) -> u32 {
    FREQ_SYS
}

// PLL (rp2xxx) input: 5 to 100Mhz, vco: 750 to 1600Mhz

fn xosc_setup() {
    let xosc = unsafe { &*xosc_hw };
    xosc.startup.write(FREQ_XOSC.div_ceil(1000 * 256)); // 1ms
    xosc.ctrl.write(XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ | (XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB));
    while xosc.status.read() & XOSC_STATUS_STABLE_BITS == 0 {}
}

fn pll_setup(pll: *mut pll_hw_t, mul: u32, postdiv: u32) {
    let pll = unsafe { &*pll };
    // Setup pll
    let (refdiv, fbdiv, mut postdiv2, mut postdiv1) = (1, mul, 1, postdiv);
    while postdiv1 > 0x07 {
        postdiv1 >>= 1;
        postdiv2 <<= 1;
    }
    pll.cs.write(refdiv);
    pll.fbdiv_int.write(fbdiv);
    pll.pwr.write(PLL_PWR_DSMPD_BITS | PLL_PWR_POSTDIVPD_BITS);
    while pll.cs.read() & PLL_CS_LOCK_BITS == 0 {}

    // Setup post divider
    pll.prim.write((postdiv1 << PLL_PRIM_POSTDIV1_LSB) | (postdiv2 << PLL_PRIM_POSTDIV2_LSB));
    pll.pwr.write(PLL_PWR_DSMPD_BITS);
}

fn clk(clk_id: u32) -> &'static clock_hw_t {
    unsafe { &(*clocks_hw).clk[clk_id as usize] }
}

fn clk_aux_setup(clk_id: u32, aux_id: u32) {
    let clk = clk(clk_id);
    clk.ctrl.write(0);
    clk.ctrl.write(aux_id | CLOCKS_CLK_PERI_CTRL_ENABLE_BITS);
}

fn clock_setup() {
    // Set clk_sys and clk_ref to use internal clock
    let csys = clk(clk_sys);
    csys.ctrl.and(!CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while csys.selected.read() != 0x1 {}
    let cref = clk(clk_ref);
    cref.ctrl.and(!CLOCKS_CLK_REF_CTRL_SRC_BITS);
    while cref.selected.read() != 0x1 {}

    // Reset peripherals (that can be)
    resets().reset.write(!(RESETS_RESET_IO_QSPI_BITS | RESETS_RESET_PADS_QSPI_BITS));

    // Setup xosc, pll_sys, and switch clk_sys
    xosc_setup();
    enable_pclock(RESETS_RESET_PLL_SYS_BITS);
    set_vsel();
    pll_setup(pll_sys_hw, FBDIV, FBDIV * FREQ_XOSC / FREQ_SYS);
    csys.ctrl.write(0);
    csys.div.write(1 << CLOCKS_CLK_SYS_DIV_INT_LSB);
    csys.ctrl.write(CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX);
    while csys.selected.read() & (1 << 1) == 0 {}

    // Setup pll_usb
    enable_pclock(RESETS_RESET_PLL_USB_BITS);
    pll_setup(pll_usb_hw, 80, 80 * FREQ_XOSC / FREQ_USB);

    // Setup peripheral clocks
    clk_aux_setup(clk_peri, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS);
    clk_aux_setup(clk_adc, CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB);
    clk_aux_setup(clk_usb, CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB);

    // Enable watchdog tick (at 12Mhz)
    cref.div.write(1 << CLOCKS_CLK_REF_DIV_INT_LSB);
    cref.ctrl.write(CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC);
    while cref.selected.read() & (1 << 2) == 0 {}
    #[cfg(CONFIG_MACH_RP2040)]
    unsafe { &*watchdog_hw }.tick.write(1 | WATCHDOG_TICK_ENABLE_BITS);
    #[cfg(not(CONFIG_MACH_RP2040))]
    {
        let tick = unsafe { &(*ticks_hw).ticks[TICK_WATCHDOG as usize] };
        tick.cycles.write(1);
        tick.ctrl.write(TICKS_WATCHDOG_CTRL_ENABLE_BITS);
    }

    // Enable GPIO control
    enable_pclock(RESETS_RESET_IO_BANK0_BITS | RESETS_RESET_PADS_BANK0_BITS);
}

// Main entry point - called from armcm_boot.c:ResetHandler()
pub fn armcm_main() -> ! {
    enable_ram_vectortable();
    clock_setup();
    sched_main();
}
