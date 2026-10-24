// Code for interacting with bootrom on rp235x chips
//
// Copyright (C) 2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;

fn rom_func_lookup(code: u32) -> usize {
    let rom_table_lookup: extern "C" fn(u32, u32) -> usize =
        unsafe { core::mem::transmute((BOOTROM_TABLE_LOOKUP_OFFSET as usize as *const u16).read_volatile() as usize) };
    rom_table_lookup(code, RT_FLAG_FUNC_ARM_SEC)
}

/****************************************************************
 * Reboot to USB rom bootloader
 ****************************************************************/

pub fn bootrom_reboot_usb_bootloader() {
    let func: extern "C" fn(u32, u32, u32, u32) -> i32 = unsafe { core::mem::transmute(rom_func_lookup(ROM_FUNC_REBOOT)) };
    func(REBOOT2_FLAG_REBOOT_TYPE_BOOTSEL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, 0, 0);
}

/****************************************************************
 * Unique id reading
 ****************************************************************/

const PICO_UNIQUE_BOARD_ID_SIZE_BYTES: usize = 8;

pub fn bootrom_read_unique_id(out: &mut [u8]) {
    let func: extern "C" fn(*mut u8, u32, u32) -> i32 = unsafe { core::mem::transmute(rom_func_lookup(ROM_FUNC_GET_SYS_INFO)) };
    let mut data = [0u8; 9 * 4];
    func(data.as_mut_ptr(), 9, SYS_INFO_CHIP_INFO);
    for i in 0..PICO_UNIQUE_BOARD_ID_SIZE_BYTES {
        out[i] = data[PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1 + 2 * 4 - i];
    }
}
