// Main starting point for SAM3/SAM4 boards
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::irq_disable;
use crate::config::{CONFIG_CLOCK_FREQ, CONFIG_MACH_SAME70};
use crate::sched::{sched_main, R};
use crate::{decl_command, decl_init, decl_task};

const FREQ_PERIPH_DIV: u32 = if CONFIG_MACH_SAME70 { 2 } else { 1 };
const FREQ_PERIPH: u32 = CONFIG_CLOCK_FREQ / FREQ_PERIPH_DIV;

#[cfg(CONFIG_MACH_SAME70)]
const FREQ_SAME70_CAN: u32 = 80000000;

/****************************************************************
 * watchdog handler
 ****************************************************************/

fn wdt() -> &'static Wdt {
    unsafe { &*WDT }
}

pub fn watchdog_reset() -> R {
    wdt().WDT_CR.write(0xA5000001);
    Ok(())
}
decl_task!(watchdog_reset);

pub fn watchdog_init() -> R {
    let timeout = 500 * 32768 / 128 / 1000; // 500ms timeout
    wdt().WDT_MR.write(WDT_MR_WDRSTEN | WDT_MR_WDV(timeout) | WDT_MR_WDD(timeout));
    Ok(())
}
decl_init!(watchdog_init);

/****************************************************************
 * Peripheral clocks
 ****************************************************************/

fn pmc() -> &'static Pmc {
    unsafe { &*PMC }
}

// Check if a peripheral clock has been enabled
pub fn is_enabled_pclock(id: u32) -> bool {
    if id < 32 {
        pmc().PMC_PCSR0.read() & (1 << id) != 0
    } else {
        pmc().PMC_PCSR1.read() & (1 << (id - 32)) != 0
    }
}

// Enable a peripheral clock
pub fn enable_pclock(id: u32) {
    if id < 32 {
        pmc().PMC_PCER0.write(1 << id);
    } else {
        pmc().PMC_PCER1.write(1 << (id - 32));
    }
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_id: u32) -> u32 {
    #[cfg(CONFIG_MACH_SAME70)]
    if _id == MCAN0_CLOCK_ID || _id == MCAN1_CLOCK_ID {
        return FREQ_SAME70_CAN;
    }
    FREQ_PERIPH
}

/****************************************************************
 * Resets
 ****************************************************************/

#[cfg(CONFIG_MACH_SAME70)]
const RST_PARAMS: u32 = (0xA5 << RSTC_CR_KEY_Pos) | RSTC_CR_PROCRST;
#[cfg(not(CONFIG_MACH_SAME70))]
const RST_PARAMS: u32 = (0xA5 << RSTC_CR_KEY_Pos) | RSTC_CR_PROCRST | RSTC_CR_PERRST;

pub fn command_reset(_args: &[u32]) -> R {
    irq_disable();
    unsafe { &*RSTC }.RSTC_CR.write(RST_PARAMS);
    #[allow(clippy::empty_loop)]
    loop {}
}
decl_command!(command_reset, HF_IN_SHUTDOWN, "reset");

#[cfg(any(CONFIG_MACH_SAM3X, CONFIG_MACH_SAM4S))]
const EFC_HW: *mut Efc = EFC0;
#[cfg(any(CONFIG_MACH_SAM4E, CONFIG_MACH_SAME70))]
const EFC_HW: *mut Efc = EFC;

// (C: noinline, aligned to 16 for predictable flash code access)
#[inline(never)]
pub fn bootloader_request() {
    irq_disable();
    let efc = unsafe { &*EFC_HW };
    // Request boot from ROM (instead of boot from flash)
    while (efc.EEFC_FSR.read() & EEFC_FSR_FRDY) == 0 {}
    efc.EEFC_FCR.write(EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(1) | EEFC_FCR_FKEY_PASSWD);
    while (efc.EEFC_FSR.read() & EEFC_FSR_FRDY) == 0 {}
    // Reboot
    unsafe { &*RSTC }.RSTC_CR.write(RST_PARAMS);
    #[allow(clippy::empty_loop)]
    loop {}
}

/****************************************************************
 * Startup
 ****************************************************************/

fn matrix_init() {
    // The ATSAM sram is in a "no default master" state at reset
    // (despite the specs).  That typically adds 1 wait cycle to every
    // memory access.  Set it to "last access master" to avoid that.
    unsafe { &*MATRIX }.MATRIX_SCFG[0].write(MATRIX_SCFG_SLOT_CYCLE(64) | MATRIX_SCFG_DEFMSTR_TYPE(1));
}

// Main entry point - called from armcm_boot.c:ResetHandler()
pub fn armcm_main() -> ! {
    SystemInit();
    matrix_init();
    sched_main();
}
