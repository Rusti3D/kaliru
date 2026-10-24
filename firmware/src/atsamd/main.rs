// Main starting point for SAMD boards
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::irq_disable;
use crate::config::CONFIG_FLASH_APPLICATION_ADDRESS;
use crate::generic::cmsis::NVIC_SystemReset;
use crate::sched::sched_main;

#[allow(dead_code)]
fn writel(addr: u32, v: u32) {
    unsafe { (addr as usize as *mut u32).write_volatile(v) };
}

pub fn bootloader_request() {
    if CONFIG_FLASH_APPLICATION_ADDRESS == 0 {
        return;
    }
    // Bootloader hack
    irq_disable();
    #[cfg(all(CONFIG_MACH_SAMD21, CONFIG_SAMD21_BOOTLOADER_UF2))]
    writel(0x20007FFC, 0xf01669ef);
    #[cfg(all(CONFIG_MACH_SAMD21, CONFIG_SAMD21_BOOTLOADER_SAMBA))]
    writel(0x20007FFC, 0x07738135);
    // (C warns: no SAMD21 bootloader type set)
    #[cfg(all(CONFIG_MACH_SAMD21, not(any(CONFIG_SAMD21_BOOTLOADER_UF2, CONFIG_SAMD21_BOOTLOADER_SAMBA))))]
    writel(0x20007FFC, 0x07738135);
    #[cfg(CONFIG_MACH_SAMX5)]
    writel(HSRAM_ADDR + HSRAM_SIZE - 4, 0xf01669ef);
    NVIC_SystemReset();
}

// Main entry point - called from armcm_boot.c:ResetHandler()
pub fn armcm_main() -> ! {
    SystemInit();
    sched_main();
}
