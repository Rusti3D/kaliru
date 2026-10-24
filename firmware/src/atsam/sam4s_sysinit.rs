// This code is from lib/sam4e/gcc/system_sam4e.c - it is unclear why
// it is not defined in the Atmel sam4s cmsis code.
#![allow(non_upper_case_globals, non_snake_case)]
use super::internal::*;
use crate::ctr::Global;

/* Clock Settings (120MHz) */
const SYS_BOARD_OSCOUNT: u32 = CKGR_MOR_MOSCXTST(0x8);
const SYS_BOARD_PLLAR: u32 = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0x13) | CKGR_PLLAR_PLLACOUNT(0x3f) | CKGR_PLLAR_DIVA(0x1);
const SYS_BOARD_MCKR: u32 = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;

/* Key to unlock MOR register */
const SYS_CKGR_MOR_KEY_VALUE: u32 = CKGR_MOR_KEY(0x37);

pub static SystemCoreClock: Global<u32> = Global::new(CHIP_FREQ_MAINCK_RC_4MHZ);

pub fn SystemInit() {
    let pmc = unsafe { &*PMC };
    /* Set FWS according to SYS_BOARD_MCKR configuration */
    unsafe { &*EFC0 }.EEFC_FMR.write(EEFC_FMR_FWS(5) | EEFC_FMR_CLOE);

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
