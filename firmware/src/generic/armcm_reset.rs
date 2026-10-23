// Generic reset command handler for ARM Cortex-M boards
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::irq_disable;
use crate::config::{CONFIG_FLASH_APPLICATION_ADDRESS, CONFIG_FLASH_BOOT_ADDRESS};
use crate::decl_command;
use crate::generic::cmsis::NVIC_SystemReset;
use crate::sched::R;

const CANBOOT_SIGNATURE: u64 = 0x21746f6f426e6143;
const CANBOOT_REQUEST: u64 = 0x5984E3FA6CA1589B;
const CANBOOT_BYPASS: u64 = 0x7b06ec45a9a8243d;

fn canboot_reset(req_signature: u64) {
    if CONFIG_FLASH_APPLICATION_ADDRESS == CONFIG_FLASH_BOOT_ADDRESS {
        // No bootloader
        return;
    }
    unsafe {
        let bl_vectors = CONFIG_FLASH_BOOT_ADDRESS as usize as *const u32;
        let boot_sig = (core::ptr::read_volatile(bl_vectors.add(1)) as usize).wrapping_sub(9) as *mut u64;
        let req_sig = core::ptr::read_volatile(bl_vectors) as usize as *mut u64;
        if boot_sig as usize % 8 != 0 || core::ptr::read_volatile(boot_sig) != CANBOOT_SIGNATURE || req_sig as usize % 8 != 0 {
            return;
        }
        irq_disable();
        core::ptr::write_volatile(req_sig, req_signature);
        // (Cortex-M7: SCB_CleanDCache_by_Addr comes with those boards)
        NVIC_SystemReset();
    }
}

pub fn try_request_canboot() {
    canboot_reset(CANBOOT_REQUEST);
}

pub fn command_reset(_args: &[u32]) -> R {
    canboot_reset(CANBOOT_BYPASS);
    NVIC_SystemReset();
}
decl_command!(command_reset, HF_IN_SHUTDOWN, "reset");
