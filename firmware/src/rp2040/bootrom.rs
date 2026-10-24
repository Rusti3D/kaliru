// Hardware interface to bootrom on rp2040
//
// Copyright (C) 2021  Lasse Dalegaard <dalegaard@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::internal::*;
use crate::board::{irq_disable, irq_enable};
use crate::ctr::Global;
use core::sync::atomic::{compiler_fence, Ordering};

const fn ROM_TABLE_CODE(c1: u8, c2: u8) -> u32 {
    c1 as u32 | ((c2 as u32) << 8)
}

// All functions in here need to be RAM-resident, as we may need
// to (especially for the flash functions) call while the XIP layer
// is unavailable. (The rp2040 link puts all code in ram; the C
// _ramfunc functions are also never inlined.)

#[inline(always)]
fn rom_hword_as_ptr(rom_address: u16) -> usize {
    unsafe { ((rom_address as usize) as *const u16).read_volatile() as usize }
}

#[inline(never)]
#[unsafe(link_section = ".ramfunc.bootrom.rom_func_lookup")]
fn rom_func_lookup(code: u32) -> usize {
    // Table and lookup function are provided by the BOOTROM
    let f: extern "C" fn(*const u16, u32) -> usize = unsafe { core::mem::transmute(rom_hword_as_ptr(0x18)) };
    let table = rom_hword_as_ptr(0x14) as *const u16;
    f(table, code)
}

#[inline(never)]
#[unsafe(link_section = ".ramfunc.bootrom.reset_to_usb_boot")]
fn reset_to_usb_boot(gpio_activity_pin_mask: u32, disable_interface_mask: u32) {
    let f: extern "C" fn(u32, u32) = unsafe { core::mem::transmute(rom_func_lookup(ROM_TABLE_CODE(b'U', b'B'))) };
    f(gpio_activity_pin_mask, disable_interface_mask);
}

// a rom function without arguments
#[inline(always)]
fn rom_call(code: u32) {
    let f: extern "C" fn() = unsafe { core::mem::transmute(rom_func_lookup(code)) };
    f();
}

#[inline(never)]
#[unsafe(link_section = ".ramfunc.bootrom.connect_internal_flash")]
fn connect_internal_flash() {
    rom_call(ROM_TABLE_CODE(b'I', b'F'));
}

#[inline(never)]
#[unsafe(link_section = ".ramfunc.bootrom.flash_exit_xip")]
fn flash_exit_xip() {
    rom_call(ROM_TABLE_CODE(b'E', b'X'));
}

#[inline(never)]
#[unsafe(link_section = ".ramfunc.bootrom.flash_flush_cache")]
fn flash_flush_cache() {
    rom_call(ROM_TABLE_CODE(b'F', b'C'));
}

/****************************************************************
 * Reboot to USB rom bootloader
 ****************************************************************/

pub fn bootrom_reboot_usb_bootloader() {
    reset_to_usb_boot(0, 0);
}

/****************************************************************
 * Unique id reading
 ****************************************************************/

// Functions for reading out the flash chip ID. Adapted from the official
// Pi SDK.

#[inline(never)]
#[unsafe(link_section = ".ramfunc.bootrom.flash_cs_force")]
fn flash_cs_force(high: bool) {
    let field_val =
        if high { IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_HIGH } else { IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW };
    hw_write_masked(
        unsafe { &(*ioqspi_hw).io[1].ctrl },
        field_val << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS,
    );
}

// To re-enable XIP we need to call flash_enter_xip. It's available in the
// bootrom, but that version is a generic one that works for most devices and
// the tradeoff for that is enabling a low performance mode.
// Instead we copy out the boot2 XIP enabling stage, and save it in RAM
// so we can call it later on.

const BOOT2_SIZE: usize = 0x100;

#[repr(C, align(16))]
struct Boot2Copy([u8; BOOT2_SIZE]);
static BOOT2_COPY: Global<Boot2Copy> = Global::new(Boot2Copy([0; BOOT2_SIZE]));

fn flash_enter_xip_prepare() {
    let target = XIP_BASE as usize as *const u8;
    unsafe { core::ptr::copy_nonoverlapping(target, BOOT2_COPY.get().0.as_mut_ptr(), BOOT2_SIZE) };
    compiler_fence(Ordering::SeqCst);
}

#[inline(never)]
#[unsafe(link_section = ".ramfunc.bootrom.flash_enter_xip_perform")]
fn flash_enter_xip_perform() {
    let f: extern "C" fn() = unsafe { core::mem::transmute(BOOT2_COPY.ptr() as usize + 1) };
    f();
}

const FLASH_RUID_CMD: u8 = 0x4B;
const FLASH_RUID_DUMMY_BYTES: usize = 4;
const FLASH_RUID_DATA_BYTES: usize = 8;
const FLASH_RUID_TOTAL_BYTES: usize = 1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES;

#[inline(never)]
#[unsafe(link_section = ".ramfunc.bootrom.bootrom_read_unique_id")]
pub fn bootrom_read_unique_id(out: &mut [u8]) {
    let mut txbuf = [0u8; FLASH_RUID_TOTAL_BYTES];
    let mut rxbuf = [0u8; FLASH_RUID_TOTAL_BYTES];

    let (mut txptr, mut rxptr) = (0, 0);

    let mut tx_remaining = FLASH_RUID_TOTAL_BYTES;
    let mut rx_remaining = FLASH_RUID_TOTAL_BYTES;

    txbuf[0] = FLASH_RUID_CMD;

    // Set up flash so we can work with it without XIP getting in the way
    flash_enter_xip_prepare();
    irq_disable();
    compiler_fence(Ordering::SeqCst);
    connect_internal_flash();
    flash_exit_xip();
    flash_cs_force(false);

    let ssi = unsafe { &*ssi_hw };
    while tx_remaining != 0 || rx_remaining != 0 {
        let flags = ssi.sr.read();
        let can_put = flags & SSI_SR_TFNF_BITS != 0;
        let can_get = flags & SSI_SR_RFNE_BITS != 0;
        if can_put && tx_remaining != 0 {
            ssi.dr0.write(txbuf[txptr] as u32);
            txptr += 1;
            tx_remaining -= 1;
        }
        if can_get && rx_remaining != 0 {
            rxbuf[rxptr] = ssi.dr0.read() as u8;
            rxptr += 1;
            rx_remaining -= 1;
        }
    }

    // Restore XIP
    flash_cs_force(true);
    flash_flush_cache();
    flash_enter_xip_perform();
    compiler_fence(Ordering::SeqCst);
    irq_enable();

    let start = 1 + FLASH_RUID_DUMMY_BYTES;
    out[..FLASH_RUID_DATA_BYTES].copy_from_slice(&rxbuf[start..start + FLASH_RUID_DATA_BYTES]);
}
