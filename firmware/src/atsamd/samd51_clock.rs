// Code to setup peripheral clocks on the SAMD51
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::internal::*;
use crate::config::{CONFIG_CLOCK_FREQ, CONFIG_CLOCK_REF_X25M, CONFIG_CLOCK_REF_X25M_XOSC0, CONFIG_CLOCK_REF_X32K, CONFIG_USB};
#[allow(unused_imports)]
use crate::decl_constant_str;

// The "generic clock generators" that are configured
const CLKGEN_MAIN: u32 = 0;
const CLKGEN_48M: u32 = 3;
const CLKGEN_2M: u32 = 4;

const FREQ_MAIN: u32 = CONFIG_CLOCK_FREQ;
const FREQ_32K: u32 = 32768;
const FREQ_96M: u32 = 96000000;
const FREQ_48M: u32 = 48000000;
const FREQ_2M: u32 = 2000000;

// DIV_ROUND_CLOSEST of unsigned values
fn div_round_closest(x: u32, d: u32) -> u32 {
    (x + d / 2) / d
}

fn gclk() -> &'static Gclk {
    unsafe { &*GCLK }
}
fn oscctrl() -> &'static Oscctrl {
    unsafe { &*OSCCTRL }
}

// Configure a clock generator using a given source as input
fn gen_clock(clkgen_id: u32, flags: u32) {
    gclk().GENCTRL[clkgen_id as usize].reg.write(flags | GCLK_GENCTRL_GENEN);
    while gclk().SYNCBUSY.reg.read() & GCLK_SYNCBUSY_GENCTRL(1 << clkgen_id) != 0 {}
}

// Route a peripheral clock to a given clkgen
fn route_pclock(pclk_id: u32, clkgen_id: u32) {
    let val = GCLK_PCHCTRL_GEN(clkgen_id) | GCLK_PCHCTRL_CHEN;
    gclk().PCHCTRL[pclk_id as usize].reg.write(val);
    while gclk().PCHCTRL[pclk_id as usize].reg.read() != val {}
}

// Enable a peripheral clock and power to that peripheral
pub fn enable_pclock(pclk_id: u32, pm_id: i32) {
    route_pclock(pclk_id, CLKGEN_48M);
    if pm_id >= 0 {
        let (pm_port, pm_bit) = (pm_id as u32 / 32, 1u32 << (pm_id as u32 % 32));
        let r = unsafe { &*(*MCLK).APBAMASK.reg.ptr().add(pm_port as usize).cast::<Reg<u32>>() };
        r.or(pm_bit);
    }
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_pclk_id: u32) -> u32 {
    FREQ_48M
}

// Configure a dpll to a given clock multiplier
fn config_dpll(pll: usize, mul: u32, ctrlb: u32) {
    let d = &oscctrl().Dpll[pll];
    d.DPLLCTRLA.reg.write(0);
    while d.DPLLSYNCBUSY.reg.read() & OSCCTRL_DPLLSYNCBUSY_ENABLE != 0 {}
    d.DPLLRATIO.reg.write(OSCCTRL_DPLLRATIO_LDR(mul - 1));
    while d.DPLLSYNCBUSY.reg.read() & OSCCTRL_DPLLSYNCBUSY_DPLLRATIO != 0 {}
    d.DPLLCTRLB.reg.write(ctrlb | OSCCTRL_DPLLCTRLB_LBYPASS);
    d.DPLLCTRLA.reg.write(OSCCTRL_DPLLCTRLA_ENABLE as _);
    let mask = OSCCTRL_DPLLSTATUS_CLKRDY | OSCCTRL_DPLLSTATUS_LOCK;
    while (d.DPLLSTATUS.reg.read() & mask) != mask {}
}

// Configure the dfll
fn config_dfll(dfllmul: u32, ctrlb: u32) {
    let o = oscctrl();
    // Disable the dfllmul and reenable in this order due to chip errata
    o.DFLLCTRLA.reg.write(0);
    while o.DFLLSYNC.reg.read() as u32 & OSCCTRL_DFLLSYNC_ENABLE != 0 {}
    o.DFLLMUL.reg.write(dfllmul);
    while o.DFLLSYNC.reg.read() as u32 & OSCCTRL_DFLLSYNC_DFLLMUL != 0 {}
    o.DFLLCTRLB.reg.write(0);
    while o.DFLLSYNC.reg.read() as u32 & OSCCTRL_DFLLSYNC_DFLLCTRLB != 0 {}
    o.DFLLCTRLA.reg.write(OSCCTRL_DFLLCTRLA_ENABLE as _);
    while o.DFLLSYNC.reg.read() as u32 & OSCCTRL_DFLLSYNC_ENABLE != 0 {}
    o.DFLLVAL.reg.write(o.DFLLVAL.reg.read());
    while o.DFLLSYNC.reg.read() as u32 & OSCCTRL_DFLLSYNC_DFLLVAL != 0 {}
    o.DFLLCTRLB.reg.write(ctrlb as _);
    while o.DFLLSYNC.reg.read() as u32 & OSCCTRL_DFLLSYNC_DFLLCTRLB != 0 {}
}

#[cfg(CONFIG_CLOCK_REF_X32K)]
decl_constant_str!("RESERVE_PINS_crystal", "PA0,PA1");
#[cfg(all(not(CONFIG_CLOCK_REF_X32K), CONFIG_CLOCK_REF_X25M_XOSC0))]
decl_constant_str!("RESERVE_PINS_crystal", "PA14,PA15");
#[cfg(all(not(CONFIG_CLOCK_REF_X32K), not(CONFIG_CLOCK_REF_X25M_XOSC0), CONFIG_CLOCK_REF_X25M))]
decl_constant_str!("RESERVE_PINS_crystal", "PB22,PB23");

// Initialize the clocks using an external 32K crystal
fn clock_init_32k() {
    // Enable external 32Khz crystal
    let val = OSC32KCTRL_XOSC32K_ENABLE | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_CGM_XT | OSC32KCTRL_XOSC32K_XTALEN;
    let osc32 = unsafe { &*OSC32KCTRL };
    osc32.XOSC32K.reg.write(val as _);
    while osc32.STATUS.reg.read() & OSC32KCTRL_STATUS_XOSC32KRDY == 0 {}

    // Generate 120Mhz clock on PLL0 (with XOSC32 as reference)
    let mut mul = div_round_closest(FREQ_MAIN, FREQ_32K);
    config_dpll(0, mul, OSCCTRL_DPLLCTRLB_REFCLK_XOSC32);

    // Switch main clock to 120Mhz PLL0
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_DPLL0);

    // Generate 96MHz clock on PLL1 (with XOSC32 as reference), divide by 2 for GCLK = 48 MHz
    mul = div_round_closest(FREQ_96M, FREQ_32K);
    config_dpll(1, mul, OSCCTRL_DPLLCTRLB_REFCLK_XOSC32);
    gen_clock(CLKGEN_48M, GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_DIV(FREQ_96M / FREQ_48M));
}

// Initialize the clocks using an external 25M crystal
fn clock_init_25m() {
    #[cfg(CONFIG_CLOCK_REF_X25M_XOSC0)]
    let (xosc, xoscrdy, refclk) = (0usize, OSCCTRL_STATUS_XOSCRDY0, OSCCTRL_DPLLCTRLB_REFCLK_XOSC0);
    #[cfg(not(CONFIG_CLOCK_REF_X25M_XOSC0))]
    let (xosc, xoscrdy, refclk) = (1usize, OSCCTRL_STATUS_XOSCRDY1, OSCCTRL_DPLLCTRLB_REFCLK_XOSC1);
    // Enable XOSC
    let freq_xosc = 25000000;
    let val = OSCCTRL_XOSCCTRL_ENABLE | OSCCTRL_XOSCCTRL_XTALEN | OSCCTRL_XOSCCTRL_IPTAT(3) | OSCCTRL_XOSCCTRL_IMULT(6);
    oscctrl().XOSCCTRL[xosc].reg.write(val);
    while oscctrl().STATUS.reg.read() & xoscrdy == 0 {}

    // Generate 120Mhz clock on PLL0 (with XOSC as reference)
    let p0div = 10;
    let p0mul = div_round_closest(FREQ_MAIN, freq_xosc / p0div);
    let p0ctrlb = OSCCTRL_DPLLCTRLB_DIV(p0div / 2 - 1);
    config_dpll(0, p0mul, p0ctrlb | refclk);

    // Switch main clock to 120Mhz PLL0
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_DPLL0);

    // Generate 96MHz clock on PLL1 (with XOSC as reference), divide by 2 for GCLK = 48 MHz
    let p1div = 50;
    let p1mul = div_round_closest(FREQ_96M, freq_xosc / p1div);
    let p1ctrlb = OSCCTRL_DPLLCTRLB_DIV(p1div / 2 - 1);
    config_dpll(1, p1mul, p1ctrlb | refclk);
    gen_clock(CLKGEN_48M, GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_DIV(FREQ_96M / FREQ_48M));
}

// Initialize clocks from factory calibrated internal clock
fn clock_init_internal() {
    // Enable USB clock recovery mode if applicable
    if CONFIG_USB {
        // Temporarily switch main clock to internal 32K clock
        gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_OSCULP32K);

        // Configure DFLL48M clock (with USB 1Khz SOF as reference)
        let mul = div_round_closest(FREQ_48M, 1000);
        let dfllmul = OSCCTRL_DFLLMUL_FSTEP(10) | OSCCTRL_DFLLMUL_MUL(mul);
        let ctrlb = OSCCTRL_DFLLCTRLB_MODE | OSCCTRL_DFLLCTRLB_USBCRM | OSCCTRL_DFLLCTRLB_CCDIS;
        config_dfll(dfllmul, ctrlb);
    }

    // Route factory calibrated DFLL48M to CLKGEN_48M
    gen_clock(CLKGEN_48M, GCLK_GENCTRL_SRC_DFLL);

    // Generate CLKGEN_2M (with CLKGEN_48M as reference)
    let div = div_round_closest(FREQ_48M, FREQ_2M);
    gen_clock(CLKGEN_2M, GCLK_GENCTRL_SRC_DFLL | GCLK_GENCTRL_DIV(div));

    // Generate 120Mhz clock on PLL0 (with CLKGEN_2M as reference)
    route_pclock(OSCCTRL_GCLK_ID_FDPLL0, CLKGEN_2M);
    let mul = div_round_closest(FREQ_MAIN, FREQ_2M);
    config_dpll(0, mul, OSCCTRL_DPLLCTRLB_REFCLK_GCLK);

    // Switch main clock to 120Mhz PLL0
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_DPLL0);
}

pub fn SystemInit() {
    // Reset GCLK
    gclk().CTRLA.reg.write(GCLK_CTRLA_SWRST as _);
    while gclk().SYNCBUSY.reg.read() & GCLK_SYNCBUSY_SWRST != 0 {}

    // Init clocks
    if CONFIG_CLOCK_REF_X32K {
        clock_init_32k();
    } else if CONFIG_CLOCK_REF_X25M || CONFIG_CLOCK_REF_X25M_XOSC0 {
        clock_init_25m();
    } else {
        clock_init_internal();
    }

    // Enable SAMD51 cache
    unsafe { &*CMCC }.CTRL.reg.write(1);
}
