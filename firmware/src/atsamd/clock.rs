// Code to setup peripheral clocks on the SAMD21
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::internal::*;
#[allow(unused_imports)]
use crate::config::{CONFIG_CLOCK_REF_X32K, CONFIG_USB};
#[allow(unused_imports)]
use crate::decl_constant_str;

// The "generic clock generators" that are configured
const CLKGEN_MAIN: u32 = 0;
#[allow(dead_code)]
const CLKGEN_ULP32K: u32 = 2;

const FREQ_MAIN: u32 = 48000000;
const FREQ_32K: u32 = 32768;

// DIV_ROUND_CLOSEST of unsigned values
fn div_round_closest(x: u32, d: u32) -> u32 {
    (x + d / 2) / d
}

fn gclk() -> &'static Gclk {
    unsafe { &*GCLK }
}
fn sysctrl() -> &'static Sysctrl {
    unsafe { &*SYSCTRL }
}

// Configure a clock generator using a given source as input
fn gen_clock(clkgen_id: u32, flags: u32) {
    gclk().GENDIV.reg.write(GCLK_GENDIV_ID(clkgen_id));
    gclk().GENCTRL.reg.write(GCLK_GENCTRL_ID(clkgen_id) | flags | GCLK_GENCTRL_GENEN);
}

// Route a peripheral clock to a given clkgen
fn route_pclock(pclk_id: u32, clkgen_id: u32) {
    gclk().CLKCTRL.reg.write((GCLK_CLKCTRL_ID(pclk_id) | GCLK_CLKCTRL_GEN(clkgen_id) | GCLK_CLKCTRL_CLKEN) as u16);
    while gclk().STATUS.reg.read() as u32 & GCLK_STATUS_SYNCBUSY != 0 {}
}

// Enable a peripheral clock and power to that peripheral
pub fn enable_pclock(pclk_id: u32, pm_id: i32) {
    route_pclock(pclk_id, CLKGEN_MAIN);
    if pm_id >= 0 {
        let (pm_port, pm_bit) = (pm_id as u32 / 32, 1u32 << (pm_id as u32 % 32));
        let r = unsafe { &*(*PM).APBAMASK.reg.ptr().add(pm_port as usize).cast::<Reg<u32>>() };
        r.or(pm_bit);
    }
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_pclk_id: u32) -> u32 {
    FREQ_MAIN
}

#[cfg(CONFIG_CLOCK_REF_X32K)]
decl_constant_str!("RESERVE_PINS_crystal", "PA0,PA1");

// Initialize the clocks using an external 32K crystal
fn clock_init_32k() {
    // Enable external 32Khz crystal
    let val = SYSCTRL_XOSC32K_STARTUP(6) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;
    sysctrl().XOSC32K.reg.write(val as u16);
    sysctrl().XOSC32K.reg.write((val | SYSCTRL_XOSC32K_ENABLE) as u16);
    while sysctrl().PCLKSR.reg.read() & SYSCTRL_PCLKSR_XOSC32KRDY == 0 {}

    // Generate 48Mhz clock on DPLL (with XOSC32K as reference)
    sysctrl().DPLLCTRLA.reg.write(0);
    let mul = div_round_closest(FREQ_MAIN, FREQ_32K);
    sysctrl().DPLLRATIO.reg.write(SYSCTRL_DPLLRATIO_LDR(mul - 1));
    sysctrl().DPLLCTRLB.reg.write(SYSCTRL_DPLLCTRLB_LBYPASS);
    sysctrl().DPLLCTRLA.reg.write(SYSCTRL_DPLLCTRLA_ENABLE as u8);
    let mask = SYSCTRL_DPLLSTATUS_CLKRDY | SYSCTRL_DPLLSTATUS_LOCK;
    while (sysctrl().DPLLSTATUS.reg.read() as u32 & mask) != mask {}

    // Switch main clock to DPLL clock
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_DPLL96M);
}

// Initialize clocks from factory calibrated internal clock
fn clock_init_internal() {
    // Configure DFLL48M clock in open loop mode
    sysctrl().DFLLCTRL.reg.write(0);
    let coarse = GET_FUSE(FUSES_DFLL48M_COARSE_CAL_ADDR, FUSES_DFLL48M_COARSE_CAL_Msk, FUSES_DFLL48M_COARSE_CAL_Pos);
    let fine = GET_FUSE(FUSES_DFLL48M_FINE_CAL_ADDR, FUSES_DFLL48M_FINE_CAL_Msk, FUSES_DFLL48M_FINE_CAL_Pos);
    sysctrl().DFLLVAL.reg.write(SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine));
    if CONFIG_USB {
        // Enable USB clock recovery mode
        let mul = div_round_closest(FREQ_MAIN, 1000);
        sysctrl().DFLLMUL.reg.write(SYSCTRL_DFLLMUL_FSTEP(10) | SYSCTRL_DFLLMUL_MUL(mul));
        sysctrl().DFLLCTRL.reg.write(
            (SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_CCDIS | SYSCTRL_DFLLCTRL_ENABLE) as u16,
        );
    } else {
        sysctrl().DFLLCTRL.reg.write(SYSCTRL_DFLLCTRL_ENABLE as u16);
    }
    while sysctrl().PCLKSR.reg.read() & SYSCTRL_PCLKSR_DFLLRDY == 0 {}

    // Switch main clock to DFLL48M clock
    gen_clock(CLKGEN_MAIN, GCLK_GENCTRL_SRC_DFLL48M);
}

pub fn SystemInit() {
    // Setup flash to work with 48Mhz clock
    unsafe { &*NVMCTRL }.CTRLB.reg.write(NVMCTRL_CTRLB_RWS_HALF);

    // Reset GCLK
    gclk().CTRL.reg.write(GCLK_CTRL_SWRST as u8);
    while gclk().CTRL.reg.read() as u32 & GCLK_CTRL_SWRST != 0 {}

    // Init clocks
    if CONFIG_CLOCK_REF_X32K {
        clock_init_32k();
    } else {
        clock_init_internal();
    }
}
