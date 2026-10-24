// System initialization for the SAM4E (lib/sam4e/gcc/system_sam4e.c)
//
// Copyright (c) 2012 Atmel Corporation. All rights reserved.
#![allow(non_upper_case_globals)]
use super::internal::*;
use crate::ctr::Global;

/* Clock Settings (120MHz) */
const SYS_BOARD_OSCOUNT: u32 = CKGR_MOR_MOSCXTST(0x8);
const SYS_BOARD_PLLAR: u32 = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0x13) | CKGR_PLLAR_PLLACOUNT(0x3f) | CKGR_PLLAR_DIVA(0x1);
const SYS_BOARD_MCKR: u32 = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;

const SYS_CKGR_MOR_KEY_VALUE: u32 = CKGR_MOR_KEY(0x37); /* Key to unlock MOR register */

pub static SystemCoreClock: Global<u32> = Global::new(CHIP_FREQ_MAINCK_RC_4MHZ);

fn pmc() -> &'static Pmc {
    unsafe { &*PMC }
}

// Setup the microcontroller system.
// Initialize the System and update the SystemFrequency variable.
pub fn SystemInit() {
    let pmc = pmc();
    /* Set FWS according to SYS_BOARD_MCKR configuration */
    unsafe { &*EFC }.EEFC_FMR.write(EEFC_FMR_FWS(5));

    /* Initialize main oscillator */
    if pmc.CKGR_MOR.read() & CKGR_MOR_MOSCSEL == 0 {
        pmc.CKGR_MOR.write(SYS_CKGR_MOR_KEY_VALUE | SYS_BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN);

        while pmc.PMC_SR.read() & PMC_SR_MOSCXTS == 0 {}
    }

    /* Switch to 3-20MHz Xtal oscillator */
    pmc.CKGR_MOR.write(SYS_CKGR_MOR_KEY_VALUE | SYS_BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCSEL);

    while pmc.PMC_SR.read() & PMC_SR_MOSCSELS == 0 {}

    pmc.PMC_MCKR.write((pmc.PMC_MCKR.read() & !PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK);

    while pmc.PMC_SR.read() & PMC_SR_MCKRDY == 0 {}

    /* Initialize PLLA */
    pmc.CKGR_PLLAR.write(SYS_BOARD_PLLAR);
    while pmc.PMC_SR.read() & PMC_SR_LOCKA == 0 {}

    /* Switch to main clock */
    pmc.PMC_MCKR.write((SYS_BOARD_MCKR & !PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK);
    while pmc.PMC_SR.read() & PMC_SR_MCKRDY == 0 {}

    /* Switch to PLLA */
    pmc.PMC_MCKR.write(SYS_BOARD_MCKR);
    while pmc.PMC_SR.read() & PMC_SR_MCKRDY == 0 {}

    *SystemCoreClock.get() = CHIP_FREQ_CPU_MAX;
}

pub fn SystemCoreClockUpdate() {
    let pmc = pmc();
    let clk = SystemCoreClock.get();
    /* Determine clock frequency according to clock register values */
    let css = pmc.PMC_MCKR.read() & PMC_MCKR_CSS_Msk;
    if css == PMC_MCKR_CSS_SLOW_CLK {
        /* Slow clock */
        if unsafe { &*SUPC }.SUPC_SR.read() & SUPC_SR_OSCSEL != 0 {
            *clk = CHIP_FREQ_XTAL_32K;
        } else {
            *clk = CHIP_FREQ_SLCK_RC;
        }
    } else if css == PMC_MCKR_CSS_MAIN_CLK || css == PMC_MCKR_CSS_PLLA_CLK {
        /* Main clock, PLLA clock */
        if pmc.CKGR_MOR.read() & CKGR_MOR_MOSCSEL != 0 {
            *clk = CHIP_FREQ_XTAL_12M;
        } else {
            *clk = CHIP_FREQ_MAINCK_RC_4MHZ;

            let rcf = pmc.CKGR_MOR.read() & CKGR_MOR_MOSCRCF_Msk;
            if rcf == CKGR_MOR_MOSCRCF_8_MHz {
                *clk *= 2;
            } else if rcf == CKGR_MOR_MOSCRCF_12_MHz {
                *clk *= 3;
            }
        }

        if css == PMC_MCKR_CSS_PLLA_CLK && (pmc.PMC_MCKR.read() & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_PLLA_CLK {
            *clk *= ((pmc.CKGR_PLLAR.read() & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos) + 1;
            *clk /= (pmc.CKGR_PLLAR.read() & CKGR_PLLAR_DIVA_Msk) >> CKGR_PLLAR_DIVA_Pos;
        }
    }

    if (pmc.PMC_MCKR.read() & PMC_MCKR_PRES_Msk) == PMC_MCKR_PRES_CLK_3 {
        *clk /= 3;
    } else {
        *clk >>= (pmc.PMC_MCKR.read() & PMC_MCKR_PRES_Msk) >> PMC_MCKR_PRES_Pos;
    }
}

// Initialize flash.
pub fn system_init_flash(ul_clk: u32) {
    /* Set FWS for embedded Flash access according to operating frequency */
    let fws = if ul_clk < CHIP_FREQ_FWS_0 {
        0
    } else if ul_clk < CHIP_FREQ_FWS_1 {
        1
    } else if ul_clk < CHIP_FREQ_FWS_2 {
        2
    } else if ul_clk < CHIP_FREQ_FWS_3 {
        3
    } else if ul_clk < CHIP_FREQ_FWS_4 {
        4
    } else {
        5
    };
    unsafe { &*EFC }.EEFC_FMR.write(EEFC_FMR_FWS(fws));
}
