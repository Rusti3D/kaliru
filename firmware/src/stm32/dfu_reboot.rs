// Reboot into stm32 ROM dfu bootloader
//
// Copyright (C) 2019-2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::irq_disable;
use crate::config::{CONFIG_HAVE_BOOTLOADER_REQUEST, CONFIG_RAM_SIZE, CONFIG_RAM_START, CONFIG_STM32_DFU_ROM_ADDRESS};
use crate::generic::cmsis::NVIC_SystemReset;

// Many stm32 chips have a USB capable "DFU bootloader" in their ROM.
// In order to invoke that bootloader it is necessary to reset the
// chip and jump to a chip specific hardware address.
// To reset the chip, the dfu_reboot() code sets a flag in memory (at
// an arbitrary position that is unlikely to be overwritten during a
// chip reset), and resets the chip.  If dfu_reboot_check() sees that
// flag on the next boot it will perform a code jump to the ROM
// address.

// Location of ram address to set internal flag
#[cfg(CONFIG_MACH_STM32H7)]
const USB_BOOT_FLAG_ADDR: usize = 0x24000000 + 0x8000; // Place flag in "AXI SRAM"
#[cfg(not(CONFIG_MACH_STM32H7))]
const USB_BOOT_FLAG_ADDR: usize = (CONFIG_RAM_START + CONFIG_RAM_SIZE - 1024) as usize;

// Signature to set in memory to flag that a dfu reboot is requested
const USB_BOOT_FLAG: u64 = 0x55534220424f4f54; // "USB BOOT"

// Flag that bootloader is desired and reboot
pub fn dfu_reboot() {
    if CONFIG_STM32_DFU_ROM_ADDRESS == 0 || !CONFIG_HAVE_BOOTLOADER_REQUEST {
        return;
    }
    irq_disable();
    unsafe { core::ptr::write_volatile(USB_BOOT_FLAG_ADDR as *mut u64, USB_BOOT_FLAG) };
    // (#if __CORTEX_M >= 7)
    #[cfg(has___CORE_CM7_H_GENERIC)]
    crate::generic::cmsis::SCB_CleanDCache_by_Addr(USB_BOOT_FLAG_ADDR as *mut u32, 8);
    NVIC_SystemReset();
}

// Check if rebooting into system DFU Bootloader
pub fn dfu_reboot_check() {
    if CONFIG_STM32_DFU_ROM_ADDRESS == 0 || !CONFIG_HAVE_BOOTLOADER_REQUEST {
        return;
    }
    let bflag = USB_BOOT_FLAG_ADDR as *mut u64;
    unsafe {
        if core::ptr::read_volatile(bflag) != USB_BOOT_FLAG {
            return;
        }
        core::ptr::write_volatile(bflag, 0);
        let sysbase = CONFIG_STM32_DFU_ROM_ADDRESS as usize as *const u32;
        core::arch::asm!(
            "mov sp, {sp}",
            "bx {pc}",
            sp = in(reg) core::ptr::read_volatile(sysbase),
            pc = in(reg) core::ptr::read_volatile(sysbase.add(1)),
            options(noreturn),
        );
    }
}
