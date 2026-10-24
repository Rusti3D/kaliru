// Code to setup peripheral clocks on the SAMC21
//
// Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2023  Luke Vuksta <wulfstawulfsta@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::internal::*;
use crate::config::{CONFIG_CLOCK_FREQ, CONFIG_CLOCK_REF_X12M};
use crate::decl_constant_str;

// The "generic clock generators" that are configured
const CLKGEN_MAIN: u32 = 0;

const FREQ_MAIN: u32 = CONFIG_CLOCK_FREQ;
#[allow(dead_code)]
const FREQ_48M: u32 = 48000000;

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
    // Don't do anything if already enabled (Timer Counter).
    let r = &gclk().PCHCTRL[pclk_id as usize].reg;
    if r.read() != val {
        r.write(val);
        while r.read() != val {}
    }
}

// Enable a peripheral clock and power to that peripheral
pub fn enable_pclock(pclk_id: u32, pm_id: i32) {
    route_pclock(pclk_id, CLKGEN_MAIN);
    if pm_id >= 0 {
        let (pm_port, pm_bit) = (pm_id as u32 / 32, 1u32 << (pm_id as u32 % 32));
        let r = unsafe { &*(*MCLK).APBAMASK.reg.ptr().add(pm_port as usize).cast::<Reg<u32>>() };
        r.or(pm_bit);
    }
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_pclk_id: u32) -> u32 {
    FREQ_MAIN
}

// Configure a dpll to a given clock multiplier
fn config_dpll(mul: u32, ctrlb: u32) {
    let o = oscctrl();
    o.DPLLCTRLA.reg.write(0);
    while o.DPLLSYNCBUSY.reg.read() as u32 & OSCCTRL_DPLLSYNCBUSY_ENABLE != 0 {}
    o.DPLLRATIO.reg.write(OSCCTRL_DPLLRATIO_LDR(mul - 1));
    while o.DPLLSYNCBUSY.reg.read() as u32 & OSCCTRL_DPLLSYNCBUSY_DPLLRATIO != 0 {}
    o.DPLLCTRLB.reg.write(ctrlb | OSCCTRL_DPLLCTRLB_LBYPASS);
    o.DPLLCTRLA.reg.write(OSCCTRL_DPLLCTRLA_ENABLE as _);
    let mask = OSCCTRL_DPLLSTATUS_CLKRDY | OSCCTRL_DPLLSTATUS_LOCK;
    while (o.DPLLSTATUS.reg.read() as u32 & mask) != mask {}
}

#[cfg(CONFIG_CLOCK_REF_X12M)]
decl_constant_str!("RESERVE_PINS_crystal", "PA14,PA15");

// Initialize the clocks using an external 12M crystal
fn clock_init_12m() {
    // Enable XOSC1
    let freq_xosc = 12000000;
    let val = OSCCTRL_XOSCCTRL_ENABLE | OSCCTRL_XOSCCTRL_XTALEN | OSCCTRL_XOSCCTRL_GAIN(3) | OSCCTRL_XOSCCTRL_AMPGC;
    oscctrl().XOSCCTRL.reg.write(val as _);
    while oscctrl().STATUS.reg.read() & OSCCTRL_STATUS_XOSCRDY == 0 {}

    // Generate 48Mhz clock on PLL (with XOSC as reference)
    let p0div = 24;
    let p0mul = div_round_closest(FREQ_MAIN, freq_xosc / p0div);
    let p0ctrlb = OSCCTRL_DPLLCTRLB_DIV(p0div / 2 - 1);
    config_dpll(p0mul, p0ctrlb | OSCCTRL_DPLLCTRLB_REFCLK(1));

    // Switch main clock to 48Mhz PLL
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL96M_Val));
}

#[cfg(CONFIG_CLOCK_REF_X25M)]
decl_constant_str!("RESERVE_PINS_crystal", "PA14,PA15");

// Initialize the clocks using an external 25M crystal
fn clock_init_25m() {
    // Enable XOSC1
    let freq_xosc = 25000000;
    let val = OSCCTRL_XOSCCTRL_ENABLE | OSCCTRL_XOSCCTRL_XTALEN | OSCCTRL_XOSCCTRL_GAIN(4) | OSCCTRL_XOSCCTRL_AMPGC;
    oscctrl().XOSCCTRL.reg.write(val as _);
    while oscctrl().STATUS.reg.read() & OSCCTRL_STATUS_XOSCRDY == 0 {}

    // Generate 48Mhz clock on PLL (with XOSC as reference)
    let p0div = 50;
    let p0mul = div_round_closest(FREQ_MAIN, freq_xosc / p0div);
    let p0ctrlb = OSCCTRL_DPLLCTRLB_DIV(p0div / 2 - 1);
    config_dpll(p0mul, p0ctrlb | OSCCTRL_DPLLCTRLB_REFCLK(1));

    // Switch main clock to 48Mhz PLL
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL96M_Val));
}

pub fn SystemInit() {
    // Set NVM wait states for 48MHz. This might need to be set higher if
    // running at a lower voltage.
    unsafe { &*NVMCTRL }.CTRLB.reg.or(NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT | NVMCTRL_CTRLB_RWS(2));

    oscctrl().OSC48MDIV.reg.write(OSCCTRL_OSC48MDIV_DIV(0) as _);
    while oscctrl().OSC48MSYNCBUSY.reg.read() & OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV != 0 {}

    // Reset GCLK
    gclk().CTRLA.reg.write(GCLK_CTRLA_SWRST as _);
    while gclk().SYNCBUSY.reg.read() & GCLK_SYNCBUSY_SWRST != 0 {}

    // Init clocks
    if CONFIG_CLOCK_REF_X12M {
        clock_init_12m();
    } else {
        clock_init_25m();
    }
}
