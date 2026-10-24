// Local definitions for RPxxxx chips (internal.h)
#![allow(non_snake_case)]
pub use super::chip::*;
pub use crate::reg::Reg;

// main.c
pub use super::main::{enable_pclock, get_pclock_frequency, is_enabled_pclock};
// gpio.c
pub use super::gpio::gpio_peripheral;
// bootrom.c / rp2350_bootrom.c
pub use super::bootrom::{bootrom_read_unique_id, bootrom_reboot_usb_bootloader};

// (pico-sdk hardware/address_mapped.h: the atomic set / clear / xor
// register aliases)
pub fn hw_set_alias<T>(p: *mut T) -> &'static T {
    unsafe { &*((REG_ALIAS_SET_BITS + p as u32) as *const T) }
}
pub fn hw_clear_alias<T>(p: *mut T) -> &'static T {
    unsafe { &*((REG_ALIAS_CLR_BITS + p as u32) as *const T) }
}
fn alias_write(addr: &Reg<u32>, alias: u32, mask: u32) {
    unsafe { ((alias + addr.ptr() as u32) as *mut u32).write_volatile(mask) };
}
pub fn hw_set_bits(addr: &Reg<u32>, mask: u32) {
    alias_write(addr, REG_ALIAS_SET_BITS, mask);
}
pub fn hw_clear_bits(addr: &Reg<u32>, mask: u32) {
    alias_write(addr, REG_ALIAS_CLR_BITS, mask);
}
pub fn hw_xor_bits(addr: &Reg<u32>, mask: u32) {
    alias_write(addr, REG_ALIAS_XOR_BITS, mask);
}
pub fn hw_write_masked(addr: &Reg<u32>, values: u32, write_mask: u32) {
    hw_xor_bits(addr, (addr.read() ^ values) & write_mask);
}
