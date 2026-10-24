// Main starting point for LPC176x boards.
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::system_LPC17xx::SystemInit;
use crate::board::irq_disable;
use crate::config::{CONFIG_CLOCK_FREQ, CONFIG_FLASH_APPLICATION_ADDRESS};
use crate::generic::armcm_reset::try_request_canboot;
use crate::generic::cmsis::NVIC_SystemReset;
use crate::sched::{sched_main, R};
use crate::{decl_init, decl_task};

/****************************************************************
 * watchdog handler
 ****************************************************************/

fn wdt() -> &'static LPC_WDT_TypeDef {
    unsafe { &*LPC_WDT }
}

pub fn watchdog_reset() -> R {
    wdt().WDFEED.write(0xaa);
    wdt().WDFEED.write(0x55);
    Ok(())
}
decl_task!(watchdog_reset);

pub fn watchdog_init() -> R {
    wdt().WDTC.write(4000000 / 2); // 500ms timeout
    wdt().WDCLKSEL.write(1 << 31); // Lock to internal RC
    wdt().WDMOD.write(0x03); // select reset and enable
    watchdog_reset()
}
decl_init!(watchdog_init);

/****************************************************************
 * misc functions
 ****************************************************************/

// Try to reboot into bootloader
pub fn bootloader_request() {
    if CONFIG_FLASH_APPLICATION_ADDRESS == 0 {
        return;
    }
    try_request_canboot();
    // Disable USB and pause for 5ms so host recognizes a disconnect
    irq_disable();
    // (C: if (CONFIG_USB) usb_disconnect())
    #[cfg(CONFIG_USB)]
    usb_disconnect();
    // The "LPC17xx-DFU-Bootloader" will enter the bootloader if the
    // watchdog timeout flag is set.
    wdt().WDMOD.write(0x07);
    NVIC_SystemReset();
}

fn sc() -> &'static LPC_SC_TypeDef {
    unsafe { &*LPC_SC }
}

// Check if a peripheral clock has been enabled
pub fn is_enabled_pclock(pclk: u32) -> bool {
    sc().PCONP.read() & (1 << pclk) != 0
}

// Enable a peripheral clock
pub fn enable_pclock(pclk: u32) {
    sc().PCONP.or(1 << pclk);
}

// Return the frequency of the given peripheral clock
pub fn get_pclock_frequency(_pclk: u32) -> u32 {
    CONFIG_CLOCK_FREQ
}

// Main entry point - called from armcm_boot.c:ResetHandler()
pub fn armcm_main() -> ! {
    SystemInit();
    sched_main();
}
