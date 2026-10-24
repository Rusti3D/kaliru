// Software CANbus implementation for rp2040 (lib/can2040/can2040.c)
//
// Copyright (C) 2022-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals)]
use super::internal::*;
use crate::config::CONFIG_MACH_RP2350;
use crate::generic::cmsis::__DMB;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{compiler_fence, Ordering};

/****************************************************************
 * can2040.h
 ****************************************************************/

// (struct can2040_msg: the layout of canbus.h's struct canbus_msg, which
// rp2040/can.c casts it to and from)
pub use crate::generic::canbus_h::CanbusMsg as Can2040Msg;

pub const CAN2040_ID_RTR: u32 = 1 << 30;
pub const CAN2040_ID_EFF: u32 = 1 << 31;

pub const CAN2040_NOTIFY_RX: u32 = 1 << 20;
pub const CAN2040_NOTIFY_TX: u32 = 1 << 21;
pub const CAN2040_NOTIFY_ERROR: u32 = 1 << 23;

pub type Can2040RxCb = unsafe fn(cd: *mut Can2040, notify: u32, msg: *mut Can2040Msg);

#[repr(C)]
#[derive(Clone, Copy, PartialEq)]
pub struct Can2040Stats {
    pub rx_total: u32,
    pub tx_total: u32,
    pub tx_attempt: u32,
    pub parse_error: u32,
}

#[repr(C)]
struct Can2040Bitunstuffer {
    stuffed_bits: u32,
    count_stuff: u32,
    unstuffed_bits: u32,
    count_unstuff: u32,
}

#[repr(C)]
struct Can2040Transmit {
    msg: Can2040Msg,
    crc: u32,
    stuffed_words: u32,
    stuffed_data: [u32; 5],
}

#[repr(C)]
pub struct Can2040 {
    // Setup
    pio_num: u32,
    pio_hw: *mut pio_hw_t,
    gpio_rx: u32,
    gpio_tx: u32,
    rx_cb: Option<Can2040RxCb>,
    stats: Can2040Stats,

    // Bit unstuffing
    unstuf: Can2040Bitunstuffer,
    raw_bit_count: u32,

    // Input data state
    parse_state: u32,
    parse_crc: u32,
    parse_crc_bits: u32,
    parse_crc_pos: u32,
    parse_msg: Can2040Msg,

    // Reporting
    report_state: u32,

    // Transmits
    tx_state: u32,
    tx_pull_pos: u32,
    tx_push_pos: u32,
    tx_queue: [Can2040Transmit; 4],
}

impl Can2040 {
    // (the state of can2040_setup()'s memset)
    pub const fn new() -> Self {
        const MSG: Can2040Msg = Can2040Msg { id: 0, dlc: 0, data: [0; 8] };
        const TX: Can2040Transmit = Can2040Transmit { msg: MSG, crc: 0, stuffed_words: 0, stuffed_data: [0; 5] };
        Can2040 {
            pio_num: 0,
            pio_hw: core::ptr::null_mut(),
            gpio_rx: 0,
            gpio_tx: 0,
            rx_cb: None,
            stats: Can2040Stats { rx_total: 0, tx_total: 0, tx_attempt: 0, parse_error: 0 },
            unstuf: Can2040Bitunstuffer { stuffed_bits: 0, count_stuff: 0, unstuffed_bits: 0, count_unstuff: 0 },
            raw_bit_count: 0,
            parse_state: 0,
            parse_crc: 0,
            parse_crc_bits: 0,
            parse_crc_pos: 0,
            parse_msg: MSG,
            report_state: 0,
            tx_state: 0,
            tx_pull_pos: 0,
            tx_push_pos: 0,
            tx_queue: [TX; 4],
        }
    }
}

/****************************************************************
 * rp2040 and low-level helper functions
 ****************************************************************/

// Determine if the target is an rp2350
const IS_RP2350: bool = CONFIG_MACH_RP2350;

fn barrier() {
    compiler_fence(Ordering::SeqCst);
}

// Helper functions for writing to "io" memory
fn writel(addr: *mut u32, val: u32) {
    barrier();
    unsafe { write_volatile(addr, val) };
}
fn readl(addr: *const u32) -> u32 {
    let val = unsafe { read_volatile(addr) };
    barrier();
    val
}

// rp2040 helper function to clear a hardware reset bit
fn rp2040_clear_reset(reset_bit: u32) {
    let resets = unsafe { &*resets_hw };
    if resets.reset.read() & reset_bit != 0 {
        hw_clear_bits(&resets.reset, reset_bit);
        while resets.reset_done.read() & reset_bit == 0 {}
    }
}

// Helper to set the mode and extended function of a pin
fn rp2040_gpio_peripheral(gpio: u32, func: u32, pull_up: i32) {
    unsafe { &*padsbank0_hw }.io[gpio as usize].write(
        PADS_BANK0_GPIO0_IE_BITS
            | (PADS_BANK0_GPIO0_DRIVE_VALUE_4MA << PADS_BANK0_GPIO0_DRIVE_MSB)
            | if pull_up > 0 { PADS_BANK0_GPIO0_PUE_BITS } else { 0 }
            | if pull_up < 0 { PADS_BANK0_GPIO0_PDE_BITS } else { 0 },
    );
    unsafe { &*iobank0_hw }.io[gpio as usize].ctrl.write(func << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB);
}

/****************************************************************
 * rp2040 PIO support
 ****************************************************************/

const PIO_CLOCK_PER_BIT: u32 = 32;
const PIO_RX_WAKE_BITS: u32 = 10;

const can2040_offset_sync_found_end_of_message: u32 = 2;
const can2040_offset_sync_signal_start: u32 = 4;
const can2040_offset_sync_entry: u32 = 6;
const can2040_offset_sync_end: u32 = 13;
const can2040_offset_shared_rx_read: u32 = 13;
const can2040_offset_shared_rx_end: u32 = 15;
const can2040_offset_match_load_next: u32 = 18;
const can2040_offset_tx_conflict: u32 = 24;
const can2040_offset_match_end: u32 = 25;
const can2040_offset_tx_got_recessive: u32 = 25;
const can2040_offset_tx_write_pin: u32 = 27;

static CAN2040_PROGRAM_INSTRUCTIONS: [u16; 32] = [
    0x0085, //  0: jmp    y--, 5
    0x0048, //  1: jmp    x--, 8
    0xe029, //  2: set    x, 9
    0x00cc, //  3: jmp    pin, 12
    0xc000, //  4: irq    nowait 0
    0x00c0, //  5: jmp    pin, 0
    0xc040, //  6: irq    clear 0
    0xe429, //  7: set    x, 9                   [4]
    0xf043, //  8: set    y, 3                   [16]
    0xc104, //  9: irq    nowait 4               [1]
    0x03c5, // 10: jmp    pin, 5                 [3]
    0x0307, // 11: jmp    7                      [3]
    0x0043, // 12: jmp    x--, 3
    0x20c4, // 13: wait   1 irq, 4
    0x4001, // 14: in     pins, 1
    0xa046, // 15: mov    y, isr
    0x01b2, // 16: jmp    x != y, 18             [1]
    0xc002, // 17: irq    nowait 2
    0x40eb, // 18: in     osr, 11
    0x4054, // 19: in     y, 20
    0xa047, // 20: mov    y, osr
    0x8080, // 21: pull   noblock
    0xa027, // 22: mov    x, osr
    0x0098, // 23: jmp    y--, 24
    0xa0e2, // 24: mov    osr, y
    0x6021, // 25: out    x, 1
    0x00df, // 26: jmp    pin, 31
    0xb801, // 27: mov    pins, x                [24]
    0x02d9, // 28: jmp    pin, 25                [2]
    0x0058, // 29: jmp    x--, 24
    0x6021, // 30: out    x, 1
    0x011b, // 31: jmp    27                     [1]
];

// Local names for PIO state machine IRQs
const SI_MAYTX: u32 = PIO_IRQ0_INTE_SM0_BITS;
const SI_MATCHED: u32 = PIO_IRQ0_INTE_SM2_BITS;
const SI_ACKDONE: u32 = PIO_IRQ0_INTE_SM3_BITS;
const SI_RX_DATA: u32 = PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS;
const SI_TXPENDING: u32 = PIO_IRQ0_INTE_SM1_BITS; // Misc bit manually forced

fn pio(cd: &Can2040) -> &'static pio_hw_t {
    unsafe { &*cd.pio_hw }
}

// Return the gpio bank offset (on rp2350 chips)
fn pio_gpiobase(cd: &Can2040) -> u32 {
    if !IS_RP2350 {
        return 0;
    }
    if cd.gpio_rx > 31 || cd.gpio_tx > 31 {
        16
    } else {
        0
    }
}

// Setup PIO "sync" state machine (state machine 0)
fn pio_sync_setup(cd: &Can2040) {
    let pio_hw = pio(cd);
    let sm = &pio_hw.sm[0];
    let gpio_rx = cd.gpio_rx.wrapping_sub(pio_gpiobase(cd)) & 0x1f;
    sm.execctrl.write(
        (gpio_rx << PIO_SM0_EXECCTRL_JMP_PIN_LSB)
            | ((can2040_offset_sync_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB)
            | (can2040_offset_sync_signal_start << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB),
    );
    sm.pinctrl.write((1 << PIO_SM0_PINCTRL_SET_COUNT_LSB) | (gpio_rx << PIO_SM0_PINCTRL_SET_BASE_LSB));
    sm.instr.write(0xe080); // set pindirs, 0
    sm.pinctrl.write(0);
    pio_hw.txf[0].write(9 + 6 * PIO_CLOCK_PER_BIT / 2);
    sm.instr.write(0x80a0); // pull block
    sm.instr.write(can2040_offset_sync_entry); // jmp sync_entry
}

// Setup PIO "rx" state machine (state machine 1)
fn pio_rx_setup(cd: &Can2040) {
    let sm = &pio(cd).sm[1];
    let gpio_rx = cd.gpio_rx.wrapping_sub(pio_gpiobase(cd)) & 0x1f;
    sm.execctrl.write(
        ((can2040_offset_shared_rx_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB)
            | (can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB),
    );
    sm.pinctrl.write(gpio_rx << PIO_SM0_PINCTRL_IN_BASE_LSB);
    sm.shiftctrl.write(0); // flush fifo on a restart
    sm.shiftctrl.write(
        PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS | (PIO_RX_WAKE_BITS << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB) | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS,
    );
    sm.instr.write(can2040_offset_shared_rx_read); // jmp shared_rx_read
}

// Setup PIO "match" state machine (state machine 2)
fn pio_match_setup(cd: &Can2040) {
    let sm = &pio(cd).sm[2];
    sm.execctrl.write(
        ((can2040_offset_match_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB)
            | (can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB),
    );
    let gpio_rx = cd.gpio_rx.wrapping_sub(pio_gpiobase(cd)) & 0x1f;
    sm.pinctrl.write(gpio_rx << PIO_SM0_PINCTRL_IN_BASE_LSB);
    sm.shiftctrl.write(0);
    sm.instr.write(0xe040); // set y, 0
    sm.instr.write(0xa0e2); // mov osr, y
    sm.instr.write(0xa02a); // mov x, !y
    sm.instr.write(can2040_offset_match_load_next); // jmp match_load_next
}

// Setup PIO "tx" state machine (state machine 3)
fn pio_tx_setup(cd: &Can2040) {
    let sm = &pio(cd).sm[3];
    let gpio_rx = cd.gpio_rx.wrapping_sub(pio_gpiobase(cd)) & 0x1f;
    let gpio_tx = cd.gpio_tx.wrapping_sub(pio_gpiobase(cd)) & 0x1f;
    sm.execctrl.write(
        (gpio_rx << PIO_SM0_EXECCTRL_JMP_PIN_LSB)
            | (can2040_offset_tx_conflict << PIO_SM0_EXECCTRL_WRAP_TOP_LSB)
            | (can2040_offset_tx_conflict << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB),
    );
    sm.shiftctrl.write(PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    sm.pinctrl.write(
        (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB)
            | (1 << PIO_SM0_PINCTRL_OUT_COUNT_LSB)
            | (gpio_tx << PIO_SM0_PINCTRL_SET_BASE_LSB)
            | (gpio_tx << PIO_SM0_PINCTRL_OUT_BASE_LSB),
    );
    sm.instr.write(0xe001); // set pins, 1
    sm.instr.write(0xe081); // set pindirs, 1
}

// Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 11 idle bits
fn pio_sync_normal_start_signal(cd: &Can2040) {
    let eom_idx = can2040_offset_sync_found_end_of_message as usize;
    pio(cd).instr_mem[eom_idx].write(0xe12a); // set x, 10 [1]
}

// Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 17 idle bits
fn pio_sync_slow_start_signal(cd: &Can2040) {
    let eom_idx = can2040_offset_sync_found_end_of_message as usize;
    pio(cd).instr_mem[eom_idx].write(0xa127); // mov x, osr [1]
}

// Test if PIO "rx" state machine has overflowed its fifos
fn pio_rx_check_stall(cd: &Can2040) -> bool {
    pio(cd).fdebug.read() & (1 << (PIO_FDEBUG_RXSTALL_LSB + 1)) != 0
}

// Set PIO "match" state machine to raise a "matched" signal on a bit sequence
fn pio_match_check(cd: &Can2040, match_key: u32) {
    pio(cd).txf[2].write(match_key);
}

// Calculate pos+bits identifier for PIO "match" state machine
fn pio_match_calc_key(raw_bits: u32, rx_bit_pos: u32) -> u32 {
    (raw_bits & 0x1fffff) | (rx_bit_pos.wrapping_neg() << 21)
}

// Cancel any pending checks on PIO "match" state machine
fn pio_match_clear(cd: &Can2040) {
    pio_match_check(cd, 0);
}

// Flush and halt PIO "tx" state machine
fn pio_tx_reset(cd: &Can2040) {
    let pio_hw = pio(cd);
    pio_hw.ctrl.write(0x07 << PIO_CTRL_SM_ENABLE_LSB);
    pio_hw.ctrl.write((0x07 << PIO_CTRL_SM_ENABLE_LSB) | (0x08 << PIO_CTRL_SM_RESTART_LSB));
    pio_hw.irq.write((SI_MATCHED | SI_ACKDONE) >> 8); // clear PIO irq flags
    // Clear tx fifo
    let sm = &pio_hw.sm[3];
    sm.shiftctrl.write(0);
    sm.shiftctrl.write(PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
}

// Queue a message for transmission on PIO "tx" state machine
fn pio_tx_send(cd: &Can2040, data: &[u32]) {
    let pio_hw = pio(cd);
    pio_tx_reset(cd);
    pio_hw.instr_mem[can2040_offset_tx_got_recessive as usize].write(0x6021); // out x, 1
    for &d in data {
        pio_hw.txf[3].write(d);
    }
    let sm = &pio_hw.sm[3];
    sm.instr.write(0xe001); // set pins, 1
    sm.instr.write(0x6021); // out x, 1
    sm.instr.write(can2040_offset_tx_write_pin); // jmp tx_write_pin
    sm.instr.write(0x20c0); // wait 1 irq, 0
    pio_hw.ctrl.write(0x0f << PIO_CTRL_SM_ENABLE_LSB);
}

// Set PIO "tx" state machine to inject an ack after a CRC match
fn pio_tx_inject_ack(cd: &Can2040, match_key: u32) {
    let pio_hw = pio(cd);
    pio_tx_reset(cd);
    pio_hw.instr_mem[can2040_offset_tx_got_recessive as usize].write(0xc023); // irq wait 3
    pio_hw.txf[3].write(0x7fffffff);
    let sm = &pio_hw.sm[3];
    sm.instr.write(0xe001); // set pins, 1
    sm.instr.write(0x6021); // out x, 1
    sm.instr.write(can2040_offset_tx_write_pin); // jmp tx_write_pin
    sm.instr.write(0x20c2); // wait 1 irq, 2
    pio_hw.ctrl.write(0x0f << PIO_CTRL_SM_ENABLE_LSB);

    pio_match_check(cd, match_key);
}

// Did PIO "tx" state machine unexpectedly finish a transmit attempt?
fn pio_tx_did_fail(cd: &Can2040) -> bool {
    let pio_hw = pio(cd);
    // Check for passive/dominant bit conflict without parser noticing
    if pio_hw.sm[3].addr.read() == can2040_offset_tx_conflict {
        return pio_hw.intr.read() & SI_RX_DATA == 0;
    }
    // Check for unexpected drain of transmit queue without parser noticing
    pio_hw.flevel.read() & PIO_FLEVEL_TX3_BITS == 0 && (pio_hw.intr.read() & (SI_MAYTX | SI_RX_DATA)) == SI_MAYTX
}

// Enable host irqs for state machine signals
fn pio_irq_set(cd: &Can2040, sm_irqs: u32) {
    pio(cd).inte0.write(sm_irqs | SI_RX_DATA);
}

// Completely disable host irqs
fn pio_irq_disable(cd: &Can2040) {
    pio(cd).inte0.write(0);
}

// Return current host irq mask
fn pio_irq_get(cd: &Can2040) -> u32 {
    pio(cd).inte0.read()
}

// Raise the txpending flag
fn pio_signal_set_txpending(cd: &Can2040) {
    pio(cd).irq_force.write(SI_TXPENDING >> 8);
}

// Clear the txpending flag
fn pio_signal_clear_txpending(cd: &Can2040) {
    pio(cd).irq.write(SI_TXPENDING >> 8);
}

// Setup PIO state machines
fn pio_sm_setup(cd: &Can2040) {
    // Reset state machines
    let pio_hw = pio(cd);
    pio_hw.ctrl.write(PIO_CTRL_SM_RESTART_BITS | PIO_CTRL_CLKDIV_RESTART_BITS);
    pio_hw.fdebug.write(0xffffffff);
    pio_hw.irq.write(0xff);
    pio_signal_set_txpending(cd);

    // Load pio program
    for (i, &ins) in CAN2040_PROGRAM_INSTRUCTIONS.iter().enumerate() {
        pio_hw.instr_mem[i].write(ins as u32);
    }

    // Set initial state machine state
    pio_sync_setup(cd);
    pio_rx_setup(cd);
    pio_match_setup(cd);
    pio_tx_setup(cd);

    // Start state machines
    pio_hw.ctrl.write(0x07 << PIO_CTRL_SM_ENABLE_LSB);
}

// Initial setup of gpio pins and PIO state machines
fn pio_setup(cd: &Can2040, sys_clock: u32, bitrate: u32) {
    // Configure pio0 clock
    #[allow(unused_mut)]
    let mut rb = if cd.pio_num != 0 { RESETS_RESET_PIO1_BITS } else { RESETS_RESET_PIO0_BITS };
    #[cfg(CONFIG_MACH_RP2350)]
    if cd.pio_num == 2 {
        rb = RESETS_RESET_PIO2_BITS;
    }
    rp2040_clear_reset(rb);

    // Setup and sync pio state machine clocks
    let pio_hw = pio(cd);
    let div = ((256 / PIO_CLOCK_PER_BIT) * sys_clock) / bitrate;
    for i in 0..4 {
        pio_hw.sm[i].clkdiv.write(div << PIO_SM0_CLKDIV_FRAC_LSB);
    }

    // Configure gpiobase (on rp2350)
    #[cfg(CONFIG_MACH_RP2350)]
    pio_hw.gpiobase.write(pio_gpiobase(cd));

    // Configure state machines
    pio_sm_setup(cd);

    // Map Rx/Tx gpios
    let pio_func = 6 + cd.pio_num;
    rp2040_gpio_peripheral(cd.gpio_rx, pio_func, 1);
    rp2040_gpio_peripheral(cd.gpio_tx, pio_func, 0);
}

/****************************************************************
 * CRC calculation
 ****************************************************************/

// Calculated 8-bit crc table (see scripts/crc.py)
static CRC_TABLE: [u16; 256] = [
    0x0000, 0x4599, 0x4eab, 0x0b32, 0x58cf, 0x1d56, 0x1664, 0x53fd, 0x7407, 0x319e,
    0x3aac, 0x7f35, 0x2cc8, 0x6951, 0x6263, 0x27fa, 0x2d97, 0x680e, 0x633c, 0x26a5,
    0x7558, 0x30c1, 0x3bf3, 0x7e6a, 0x5990, 0x1c09, 0x173b, 0x52a2, 0x015f, 0x44c6,
    0x4ff4, 0x0a6d, 0x5b2e, 0x1eb7, 0x1585, 0x501c, 0x03e1, 0x4678, 0x4d4a, 0x08d3,
    0x2f29, 0x6ab0, 0x6182, 0x241b, 0x77e6, 0x327f, 0x394d, 0x7cd4, 0x76b9, 0x3320,
    0x3812, 0x7d8b, 0x2e76, 0x6bef, 0x60dd, 0x2544, 0x02be, 0x4727, 0x4c15, 0x098c,
    0x5a71, 0x1fe8, 0x14da, 0x5143, 0x73c5, 0x365c, 0x3d6e, 0x78f7, 0x2b0a, 0x6e93,
    0x65a1, 0x2038, 0x07c2, 0x425b, 0x4969, 0x0cf0, 0x5f0d, 0x1a94, 0x11a6, 0x543f,
    0x5e52, 0x1bcb, 0x10f9, 0x5560, 0x069d, 0x4304, 0x4836, 0x0daf, 0x2a55, 0x6fcc,
    0x64fe, 0x2167, 0x729a, 0x3703, 0x3c31, 0x79a8, 0x28eb, 0x6d72, 0x6640, 0x23d9,
    0x7024, 0x35bd, 0x3e8f, 0x7b16, 0x5cec, 0x1975, 0x1247, 0x57de, 0x0423, 0x41ba,
    0x4a88, 0x0f11, 0x057c, 0x40e5, 0x4bd7, 0x0e4e, 0x5db3, 0x182a, 0x1318, 0x5681,
    0x717b, 0x34e2, 0x3fd0, 0x7a49, 0x29b4, 0x6c2d, 0x671f, 0x2286, 0x2213, 0x678a,
    0x6cb8, 0x2921, 0x7adc, 0x3f45, 0x3477, 0x71ee, 0x5614, 0x138d, 0x18bf, 0x5d26,
    0x0edb, 0x4b42, 0x4070, 0x05e9, 0x0f84, 0x4a1d, 0x412f, 0x04b6, 0x574b, 0x12d2,
    0x19e0, 0x5c79, 0x7b83, 0x3e1a, 0x3528, 0x70b1, 0x234c, 0x66d5, 0x6de7, 0x287e,
    0x793d, 0x3ca4, 0x3796, 0x720f, 0x21f2, 0x646b, 0x6f59, 0x2ac0, 0x0d3a, 0x48a3,
    0x4391, 0x0608, 0x55f5, 0x106c, 0x1b5e, 0x5ec7, 0x54aa, 0x1133, 0x1a01, 0x5f98,
    0x0c65, 0x49fc, 0x42ce, 0x0757, 0x20ad, 0x6534, 0x6e06, 0x2b9f, 0x7862, 0x3dfb,
    0x36c9, 0x7350, 0x51d6, 0x144f, 0x1f7d, 0x5ae4, 0x0919, 0x4c80, 0x47b2, 0x022b,
    0x25d1, 0x6048, 0x6b7a, 0x2ee3, 0x7d1e, 0x3887, 0x33b5, 0x762c, 0x7c41, 0x39d8,
    0x32ea, 0x7773, 0x248e, 0x6117, 0x6a25, 0x2fbc, 0x0846, 0x4ddf, 0x46ed, 0x0374,
    0x5089, 0x1510, 0x1e22, 0x5bbb, 0x0af8, 0x4f61, 0x4453, 0x01ca, 0x5237, 0x17ae,
    0x1c9c, 0x5905, 0x7eff, 0x3b66, 0x3054, 0x75cd, 0x2630, 0x63a9, 0x689b, 0x2d02,
    0x276f, 0x62f6, 0x69c4, 0x2c5d, 0x7fa0, 0x3a39, 0x310b, 0x7492, 0x5368, 0x16f1,
    0x1dc3, 0x585a, 0x0ba7, 0x4e3e, 0x450c, 0x0095,
];

// Update a crc with 8 bits of data
fn crc_byte(crc: u32, data: u32) -> u32 {
    (crc << 8) ^ CRC_TABLE[(((crc >> 7) ^ data) & 0xff) as usize] as u32
}

// Update a crc with 8, 16, 24, or 32 bits of data
#[inline(always)]
fn crc_bytes(mut crc: u32, data: u32, num: u32) -> u32 {
    // (the C switch falls through from its case down to case 1)
    if num != 3 && num != 2 && num != 1 {
        crc = crc_byte(crc, data >> 24);
    }
    if num != 2 && num != 1 {
        crc = crc_byte(crc, data >> 16);
    }
    if num != 1 {
        crc = crc_byte(crc, data >> 8);
    }
    crc_byte(crc, data)
}

/****************************************************************
 * Bit unstuffing
 ****************************************************************/

// (The shift counts below stay under 32, as in the C code.)

// Add 'count' number of bits from 'data' to the 'bu' unstuffer
fn unstuf_add_bits(bu: &mut Can2040Bitunstuffer, data: u32, count: u32) {
    let mask = (1 << count) - 1;
    bu.stuffed_bits = (bu.stuffed_bits << count) | (data & mask);
    bu.count_stuff = count;
}

// Reset state and set the next desired 'num_bits' unstuffed bits to extract
fn unstuf_set_count(bu: &mut Can2040Bitunstuffer, num_bits: u32) {
    bu.unstuffed_bits = 0;
    bu.count_unstuff = num_bits;
}

// Clear bitstuffing state (used after crc field to avoid bitstuffing ack field)
fn unstuf_clear_state(bu: &mut Can2040Bitunstuffer) {
    let lb = 1u32 << bu.count_stuff;
    bu.stuffed_bits = (bu.stuffed_bits & (lb.wrapping_sub(1))) | (lb << 1);
}

// Restore raw bitstuffing state (used to undo unstuf_clear_state() )
fn unstuf_restore_state(bu: &mut Can2040Bitunstuffer, data: u32) {
    let cs = bu.count_stuff;
    bu.stuffed_bits = (bu.stuffed_bits & ((1u32 << cs).wrapping_sub(1))) | (data << cs);
}

// (C: 1 << n of an int, then used as uint32_t)
fn mask(n: u32) -> u32 {
    (1u32 << n).wrapping_sub(1)
}

// Pull bits from unstuffer (as specified in unstuf_set_count() )
fn unstuf_pull_bits_rp2040(bu: &mut Can2040Bitunstuffer) -> i32 {
    let sb = bu.stuffed_bits;
    let edges = sb ^ (sb >> 1);
    let e2 = edges | (edges >> 1);
    let e4 = e2 | (e2 >> 2);
    let rm_bits = !e4;
    let (mut cs, mut cu) = (bu.count_stuff, bu.count_unstuff);
    if cs == 0 {
        // Need more data
        return 1;
    }
    loop {
        let mut try_cnt = if cs > cu { cu } else { cs };
        loop {
            let try_mask = mask(try_cnt) << (cs + 1 - try_cnt);
            if rm_bits & try_mask == 0 {
                // No stuff bits in try_cnt bits - copy into unstuffed_bits
                cu -= try_cnt;
                bu.count_unstuff = cu;
                cs -= try_cnt;
                bu.count_stuff = cs;
                bu.unstuffed_bits |= ((sb >> cs) & mask(try_cnt)) << cu;
                if cu == 0 {
                    // Extracted desired bits
                    return 0;
                }
                break;
            }
            cs -= 1;
            bu.count_stuff = cs;
            if rm_bits & (1 << (cs + 1)) != 0 {
                // High bit is a stuff bit
                if rm_bits & (1 << cs) != 0 {
                    // Six consecutive bits - a bitstuff error
                    if sb & (1 << cs) != 0 {
                        return -1;
                    }
                    return -2;
                }
                break;
            }
            // High bit not a stuff bit - limit try_cnt and retry
            cu -= 1;
            bu.count_unstuff = cu;
            bu.unstuffed_bits |= ((sb >> cs) & 1) << cu;
            try_cnt /= 2;
        }
        if cs == 0 {
            // Need more data
            return 1;
        }
    }
}

// Pull bits from unstuffer (optimized for rp2350)
fn unstuf_pull_bits(bu: &mut Can2040Bitunstuffer) -> i32 {
    if !IS_RP2350 {
        return unstuf_pull_bits_rp2040(bu);
    }
    let sb = bu.stuffed_bits;
    let edges = sb ^ (sb >> 1);
    let e2 = edges | (edges >> 1);
    let e4 = e2 | (e2 >> 2);
    let rm_bits = !e4;
    let (mut cs, mut cu) = (bu.count_stuff, bu.count_unstuff);
    loop {
        if cs == 0 {
            // Need more data
            return 1;
        }
        let try_cnt = if cs > cu { cu } else { cs };
        let try_mask = mask(try_cnt) << (cs + 1 - try_cnt);
        let rm_masked_bits = rm_bits & try_mask;
        if rm_masked_bits == 0 {
            // No stuff bits in try_cnt bits - copy into unstuffed_bits
            cu -= try_cnt;
            bu.count_unstuff = cu;
            cs -= try_cnt;
            bu.count_stuff = cs;
            bu.unstuffed_bits |= ((sb >> cs) & mask(try_cnt)) << cu;
            if cu == 0 {
                // Extracted desired bits
                return 0;
            }
            // Need more data
            return 1;
        }
        // Copy any leading bits prior to stuff bit (may be zero)
        let copy_cnt = cs - (31 - rm_masked_bits.leading_zeros());
        cs -= copy_cnt;
        cu -= copy_cnt;
        bu.count_unstuff = cu;
        bu.unstuffed_bits |= ((sb >> cs) & mask(copy_cnt)) << cu;
        // High bit is now a stuff bit - remove it
        cs -= 1;
        bu.count_stuff = cs;
        if rm_bits & (1 << cs) != 0 {
            // Six consecutive bits - a bitstuff error
            if sb & (1 << cs) != 0 {
                return -1;
            }
            return -2;
        }
    }
}

// Return most recent raw (still stuffed) bits
fn unstuf_get_raw(bu: &Can2040Bitunstuffer) -> u32 {
    bu.stuffed_bits >> bu.count_stuff
}

/****************************************************************
 * Bit stuffing
 ****************************************************************/

// Stuff 'num_bits' bits in '*pb' - upper bits must already be stuffed
fn bitstuff_rp2040(pb: &mut u32, mut num_bits: u32) -> u32 {
    let mut b = *pb;
    let mut count = num_bits;
    'done: loop {
        let mut try_cnt = num_bits;
        let edges = b ^ (b >> 1);
        let e2 = edges | (edges >> 1);
        let e4 = e2 | (e2 >> 2);
        let add_bits = !e4;
        loop {
            let try_mask = mask(try_cnt) << (num_bits - try_cnt);
            if add_bits & try_mask == 0 {
                // No stuff bits needed in try_cnt bits
                if try_cnt >= num_bits {
                    break 'done;
                }
                num_bits -= try_cnt;
                try_cnt = (num_bits + 1) / 2;
                continue;
            }
            if add_bits & (1 << (num_bits - 1)) != 0 {
                // A stuff bit must be inserted prior to the high bit
                let low_mask = mask(num_bits);
                let low = b & low_mask;
                let high = (b & !(low_mask >> 1)) << 1;
                b = high ^ low ^ (1 << (num_bits - 1));
                count += 1;
                if num_bits <= 4 {
                    break 'done;
                }
                num_bits -= 4;
                break;
            }
            // High bit doesn't need stuff bit - accept it, limit try_cnt, retry
            num_bits -= 1;
            try_cnt /= 2;
        }
    }
    *pb = b;
    count
}

// Stuff 'num_bits' bits in '*pb' (optimized for rp2350)
fn bitstuff(pb: &mut u32, mut num_bits: u32) -> u32 {
    if !IS_RP2350 {
        return bitstuff_rp2040(pb, num_bits);
    }
    let mut b = *pb;
    let mut count = num_bits;
    loop {
        let edges = b ^ (b >> 1);
        let e2 = edges | (edges >> 1);
        let e4 = e2 | (e2 >> 2);
        let add_bits = !e4;
        let add_masked_bits = add_bits & mask(num_bits);
        if add_masked_bits == 0 {
            // No more stuff bits needed
            break;
        }
        // Insert a stuff bit
        let stuff_pos = 1 + 31 - add_masked_bits.leading_zeros();
        let low_mask = mask(stuff_pos);
        let low = b & low_mask;
        let high = (b & !(low_mask >> 1)) << 1;
        b = high ^ low ^ (1 << (stuff_pos - 1));
        count += 1;
        if stuff_pos <= 4 {
            break;
        }
        num_bits = stuff_pos - 4;
    }
    *pb = b;
    count
}

// State storage for building bit stuffed transmit messages
struct BitstufferS<'a> {
    prev_stuffed: u32,
    bitpos: u32,
    buf: &'a mut [u32; 5],
}

// Push 'count' bits of 'data' into stuffer without performing bit stuffing
fn bs_pushraw(bs: &mut BitstufferS, data: u32, count: u32) {
    let bitpos = bs.bitpos;
    let wp = (bitpos / 32) as usize;
    let bitused = bitpos % 32;
    let bitavail = 32 - bitused;
    if bitavail >= count {
        bs.buf[wp] |= data << (bitavail - count);
    } else {
        bs.buf[wp] |= data >> (count - bitavail);
        bs.buf[wp + 1] |= data << (32 - (count - bitavail));
    }
    bs.bitpos = bitpos + count;
}

// Push 'count' bits of 'data' into stuffer
fn bs_push(bs: &mut BitstufferS, mut data: u32, count: u32) {
    data &= mask(count);
    let mut stuf = (bs.prev_stuffed << count) | data;
    let newcount = bitstuff(&mut stuf, count);
    bs_pushraw(bs, stuf, newcount);
    bs.prev_stuffed = stuf;
}

// Pad final word of stuffer with high bits
fn bs_finalize(bs: &mut BitstufferS) -> u32 {
    let bitpos = bs.bitpos;
    let words = bitpos.div_ceil(32);
    let extra = words * 32 - bitpos;
    if extra != 0 {
        bs.buf[words as usize - 1] |= mask(extra);
    }
    words
}

/****************************************************************
 * Transmit state tracking
 ****************************************************************/

// Transmit states (stored in cd->tx_state)
const TS_IDLE: u32 = 0;
const TS_QUEUED: u32 = 1;
const TS_ACKING_RX: u32 = 2;
const TS_CONFIRM_TX: u32 = 3;

// Calculate queue array position from a transmit index
fn tx_qpos(cd: &Can2040, pos: u32) -> usize {
    (pos % cd.tx_queue.len() as u32) as usize
}

// Queue the next message for transmission in the PIO
fn tx_schedule_transmit(cd: &mut Can2040) -> u32 {
    if cd.tx_state == TS_QUEUED && !pio_tx_did_fail(cd) {
        // Already queued or actively transmitting
        return 0;
    }
    let tx_pull_pos = cd.tx_pull_pos;
    if readl(&cd.tx_push_pos) == tx_pull_pos {
        // No new messages to transmit
        cd.tx_state = TS_IDLE;
        pio_signal_clear_txpending(cd);
        __DMB();
        if readl(&cd.tx_push_pos) == tx_pull_pos {
            return SI_TXPENDING;
        }
        // Raced with can2040_transmit() - msg is now available for transmit
        pio_signal_set_txpending(cd);
    }
    cd.tx_state = TS_QUEUED;
    cd.stats.tx_attempt = cd.stats.tx_attempt.wrapping_add(1);
    let qt = &cd.tx_queue[tx_qpos(cd, tx_pull_pos)];
    pio_tx_send(cd, &qt.stuffed_data[..qt.stuffed_words as usize]);
    0
}

// Setup PIO state for ack injection
fn tx_inject_ack(cd: &mut Can2040, match_key: u32) {
    cd.tx_state = TS_ACKING_RX;
    pio_tx_inject_ack(cd, match_key);
}

// Check if the current parsed message is feedback from current transmit
fn tx_check_local_message(cd: &mut Can2040) -> i32 {
    if cd.tx_state != TS_QUEUED {
        return 0;
    }
    let qt = &cd.tx_queue[tx_qpos(cd, cd.tx_pull_pos)];
    let (pm, tm) = (&cd.parse_msg, &qt.msg);
    if tm.id == pm.id {
        if qt.crc != cd.parse_crc || tm.dlc != pm.dlc || tm.data32(0) != pm.data32(0) || tm.data32(1) != pm.data32(1) {
            // Message with same id that differs in content - an error
            return -1;
        }
        // This is a self transmit
        cd.tx_state = TS_CONFIRM_TX;
        return 1;
    }
    0
}

/****************************************************************
 * Notification callbacks
 ****************************************************************/

// Report state flags (stored in cd->report_state)
const RS_NEED_EOF_FLAG: u32 = 1 << 2;
// States
const RS_IDLE: u32 = 0;
const RS_NEED_RX_ACK: u32 = 1;
const RS_NEED_TX_ACK: u32 = 2;
#[allow(dead_code)]
const RS_NEED_RX_EOF: u32 = RS_NEED_RX_ACK | RS_NEED_EOF_FLAG;
const RS_NEED_TX_EOF: u32 = RS_NEED_TX_ACK | RS_NEED_EOF_FLAG;

fn rx_cb(cd: &mut Can2040, notify: u32, msg: *mut Can2040Msg) {
    let cb = cd.rx_cb.unwrap();
    unsafe { cb(cd, notify, msg) };
}

// Report error to calling code (via callback interface)
fn report_callback_error(cd: &mut Can2040, error_code: u32) {
    let mut msg = Can2040Msg { id: 0, dlc: 0, data: [0; 8] };
    rx_cb(cd, CAN2040_NOTIFY_ERROR | error_code, &mut msg);
}

// Report a received message to calling code (via callback interface)
fn report_callback_rx_msg(cd: &mut Can2040) {
    cd.stats.rx_total = cd.stats.rx_total.wrapping_add(1);
    let msg = &mut cd.parse_msg as *mut Can2040Msg;
    rx_cb(cd, CAN2040_NOTIFY_RX, msg);
}

// Report a message that was successfully transmited (via callback interface)
fn report_callback_tx_msg(cd: &mut Can2040) {
    let pos = cd.tx_pull_pos.wrapping_add(1);
    writel(&mut cd.tx_pull_pos, pos);
    cd.stats.tx_total = cd.stats.tx_total.wrapping_add(1);
    let msg = &mut cd.parse_msg as *mut Can2040Msg;
    rx_cb(cd, CAN2040_NOTIFY_TX, msg);
}

// EOF phase complete - report message (rx or tx) to calling code
fn report_handle_eof(cd: &mut Can2040) {
    if cd.report_state & RS_NEED_EOF_FLAG != 0 {
        // RS_NEED_xX_EOF
        // Successfully processed a new message - report to calling code
        pio_sync_normal_start_signal(cd);
        if cd.report_state == RS_NEED_TX_EOF {
            report_callback_tx_msg(cd);
        } else {
            report_callback_rx_msg(cd);
        }
    }
    cd.report_state = RS_IDLE;
    pio_match_clear(cd);
}

// Check if message being processed is an rx message (not self feedback from tx)
fn report_is_not_in_tx(cd: &Can2040) -> bool {
    cd.report_state & RS_NEED_TX_ACK == 0
}

// Parser found a new message start
fn report_note_message_start(cd: &Can2040) {
    pio_irq_set(cd, SI_MAYTX);
}

// Setup for ack injection (if receiving) or ack confirmation (if transmit)
fn report_note_crc_start(cd: &mut Can2040) -> i32 {
    let ret = tx_check_local_message(cd);
    if ret != 0 {
        if ret < 0 {
            return -1;
        }
        // This is a self transmit - setup tx eof "matched" signal
        cd.report_state = RS_NEED_TX_ACK;
        let bits = (cd.parse_crc_bits << 9) | 0x0ff;
        pio_match_check(cd, pio_match_calc_key(bits, cd.parse_crc_pos.wrapping_add(9)));
        return 0;
    }

    // Setup for ack inject (after rx fifos fully drained)
    cd.report_state = RS_NEED_RX_ACK;
    pio_signal_set_txpending(cd);
    pio_irq_set(cd, SI_MAYTX | SI_TXPENDING);
    0
}

// Parser successfully found matching crc
fn report_note_crc_success(cd: &Can2040) {
    if cd.report_state == RS_NEED_TX_ACK {
        // Enable "matched" irq for fast back-to-back transmit scheduling
        pio_irq_set(cd, SI_MAYTX | SI_MATCHED);
    }
}

// Parser found successful ack
fn report_note_ack_success(cd: &mut Can2040) {
    if cd.report_state == RS_IDLE {
        // Got "matched" signal already
        return;
    }
    // Transition RS_NEED_xX_ACK to RS_NEED_xX_EOF
    cd.report_state |= RS_NEED_EOF_FLAG;
}

// Parser found successful EOF
fn report_note_eof_success(cd: &mut Can2040) {
    if cd.report_state == RS_IDLE {
        // Got "matched" signal already
        return;
    }
    report_handle_eof(cd);
    pio_irq_set(cd, SI_TXPENDING);
}

// Parser found unexpected data on input
fn report_note_discarding(cd: &mut Can2040) {
    if cd.report_state != RS_IDLE {
        cd.report_state = RS_IDLE;
        pio_match_clear(cd);
    }
    pio_sync_slow_start_signal(cd);
    pio_irq_set(cd, SI_MAYTX | SI_TXPENDING);
}

// Received PIO rx "ackdone" irq
fn report_line_ackdone(cd: &mut Can2040) {
    // Setup "matched" irq for fast rx callbacks
    let bits = (cd.parse_crc_bits << 8) | 0x7f;
    pio_match_check(cd, pio_match_calc_key(bits, cd.parse_crc_pos.wrapping_add(8)));
    // Schedule next transmit (so it is ready for next frame line arbitration)
    let check_txpending = tx_schedule_transmit(cd);
    pio_irq_set(cd, SI_MAYTX | SI_MATCHED | check_txpending);
}

// Received PIO "matched" irq
fn report_line_matched(cd: &mut Can2040) {
    // A match event indicates an ack and eof are present
    if cd.report_state != RS_IDLE {
        // Transition RS_NEED_xX_ACK to RS_NEED_xX_EOF (if not already there)
        cd.report_state |= RS_NEED_EOF_FLAG;
        report_handle_eof(cd);
    }
    // Implement fast back-to-back tx scheduling (if applicable)
    let check_txpending = tx_schedule_transmit(cd);
    pio_irq_set(cd, check_txpending);
}

// Received 10+ passive bits on the line (between 10 and 17 bits)
fn report_line_maytx(cd: &mut Can2040) {
    // Line is idle - may be unexpected EOF, missed ack injection,
    // or missed "matched" signal.
    if cd.report_state != RS_IDLE {
        report_handle_eof(cd);
    }
    let check_txpending = tx_schedule_transmit(cd);
    pio_irq_set(cd, check_txpending);
}

// Schedule a transmit
fn report_line_txpending(cd: &mut Can2040) {
    let pio_irqs = pio_irq_get(cd);
    if pio_irqs == (SI_MAYTX | SI_TXPENDING | SI_RX_DATA) && cd.report_state == RS_NEED_RX_ACK {
        // Ack inject request from report_note_crc_start()
        let mk = pio_match_calc_key(cd.parse_crc_bits, cd.parse_crc_pos);
        tx_inject_ack(cd, mk);
        pio_irq_set(cd, SI_MAYTX | SI_ACKDONE);
        return;
    }
    // Tx request from can2040_transmit(), report_note_eof_success(),
    // or report_note_discarding().
    let check_txpending = tx_schedule_transmit(cd);
    pio_irq_set(cd, (pio_irqs & !SI_TXPENDING) | check_txpending);
}

/****************************************************************
 * Input state tracking
 ****************************************************************/

// Parsing states (stored in cd->parse_state)
const MS_START: u32 = 0;
const MS_HEADER: u32 = 1;
const MS_EXT_HEADER: u32 = 2;
const MS_DATA0: u32 = 3;
const MS_DATA1: u32 = 4;
const MS_CRC: u32 = 5;
const MS_ACK: u32 = 6;
const MS_EOF0: u32 = 7;
const MS_EOF1: u32 = 8;
const MS_DISCARD: u32 = 9;

// Reset any bits in the incoming parsing state
fn data_state_clear_bits(cd: &mut Can2040) {
    cd.unstuf.count_stuff = 0;
    cd.unstuf.stuffed_bits = 0;
    cd.raw_bit_count = 0;
}

// Transition to the next parsing state
fn data_state_go_next(cd: &mut Can2040, state: u32, num_bits: u32) {
    cd.parse_state = state;
    unstuf_set_count(&mut cd.unstuf, num_bits);
}

// Transition to the MS_DISCARD state - drop all bits until 6 passive bits
fn data_state_go_discard(cd: &mut Can2040) {
    if pio_rx_check_stall(cd) {
        // CPU couldn't keep up for some read data - must reset pio state
        data_state_clear_bits(cd);
        pio_sm_setup(cd);
        report_callback_error(cd, 0);
    }

    data_state_go_next(cd, MS_DISCARD, 32);

    // Clear report state and update hw irqs after transition to MS_DISCARD
    report_note_discarding(cd);
}

// Note a data parse error and transition to discard state
fn data_state_go_error(cd: &mut Can2040) {
    cd.stats.parse_error = cd.stats.parse_error.wrapping_add(1);
    data_state_go_discard(cd);
}

// Received six dominant bits on the line
fn data_state_line_error(cd: &mut Can2040) {
    if cd.parse_state == MS_DISCARD {
        data_state_go_discard(cd);
    } else {
        data_state_go_error(cd);
    }
}

// Received six unexpected passive bits on the line
fn data_state_line_passive(cd: &mut Can2040) {
    if cd.parse_state != MS_DISCARD && cd.parse_state != MS_START {
        // Bitstuff error
        data_state_go_error(cd);
        return;
    }

    let stuffed_bits = unstuf_get_raw(&cd.unstuf);
    let dom_bits = !stuffed_bits;
    if dom_bits == 0 {
        // Counter overflow in "sync" state machine - reset it
        data_state_clear_bits(cd);
        pio_sm_setup(cd);
        data_state_go_discard(cd);
        return;
    }

    // Look for sof after 10 passive bits (most "PIO sync" will produce)
    if dom_bits & 0x3ff == 0 {
        data_state_go_next(cd, MS_START, 1);
        return;
    }

    data_state_go_discard(cd);
}

// Transition to MS_CRC state - await 16 bits of crc
fn data_state_go_crc(cd: &mut Can2040) {
    cd.parse_crc &= 0x7fff;

    // Calculate raw stuffed bits after crc and crc delimiter
    let crcstart_bitpos = cd.raw_bit_count.wrapping_sub(cd.unstuf.count_stuff).wrapping_sub(1);
    let mut crc_bits = (unstuf_get_raw(&cd.unstuf) << 15) | cd.parse_crc;
    let crc_bitcount = bitstuff(&mut crc_bits, 15 + 1) - 1;
    cd.parse_crc_bits = (crc_bits << 1) | 0x01; // Add crc delimiter
    cd.parse_crc_pos = crcstart_bitpos.wrapping_add(crc_bitcount).wrapping_add(1);

    let ret = report_note_crc_start(cd);
    if ret != 0 {
        data_state_go_error(cd);
        return;
    }
    data_state_go_next(cd, MS_CRC, 16);
}

// Transition to MS_DATA0 state (if applicable) - await data bits
fn data_state_go_data(cd: &mut Can2040, mut id: u32, data: u32) {
    if data & (0x03 << 4) != 0 {
        // Not a supported header
        data_state_go_discard(cd);
        return;
    }
    cd.parse_msg.data = [0; 8];
    let mut dlc = data & 0x0f;
    cd.parse_msg.dlc = dlc;
    if data & (1 << 6) != 0 {
        dlc = 0;
        id |= CAN2040_ID_RTR;
    }
    cd.parse_msg.id = id;
    if dlc != 0 {
        data_state_go_next(cd, MS_DATA0, if dlc >= 4 { 32 } else { dlc * 8 });
    } else {
        data_state_go_crc(cd);
    }
}

// Handle reception of first bit of header (after start-of-frame (SOF))
fn data_state_update_start(cd: &mut Can2040, data: u32) {
    cd.parse_msg.id = data;
    report_note_message_start(cd);
    data_state_go_next(cd, MS_HEADER, 17);
}

// Handle reception of next 17 header bits
fn data_state_update_header(cd: &mut Can2040, mut data: u32) {
    data |= cd.parse_msg.id << 17;
    if (data & 0x60) == 0x60 {
        // Extended header
        cd.parse_msg.id = data;
        data_state_go_next(cd, MS_EXT_HEADER, 20);
        return;
    }
    cd.parse_crc = crc_bytes(0, data, 3);
    data_state_go_data(cd, (data >> 7) & 0x7ff, data);
}

// Handle reception of additional 20 bits of "extended header"
fn data_state_update_ext_header(cd: &mut Can2040, data: u32) {
    let hdr1 = cd.parse_msg.id;
    let crc = crc_bytes(0, hdr1 >> 4, 2);
    cd.parse_crc = crc_bytes(crc, ((hdr1 & 0x0f) << 20) | data, 3);
    let id = ((hdr1 << 11) & 0x1ffc0000) | ((hdr1 << 13) & 0x3e000) | (data >> 7) | CAN2040_ID_EFF;
    data_state_go_data(cd, id, data);
}

// Handle reception of first 1-4 bytes of data content
fn data_state_update_data0(cd: &mut Can2040, data: u32) {
    let dlc = cd.parse_msg.dlc;
    let bits = if dlc >= 4 { 32 } else { dlc * 8 };
    cd.parse_crc = crc_bytes(cd.parse_crc, data, dlc);
    cd.parse_msg.set_data32(0, (data << (32 - bits)).swap_bytes());
    if dlc > 4 {
        data_state_go_next(cd, MS_DATA1, if dlc >= 8 { 32 } else { (dlc - 4) * 8 });
    } else {
        data_state_go_crc(cd);
    }
}

// Handle reception of bytes 5-8 of data content
fn data_state_update_data1(cd: &mut Can2040, data: u32) {
    let dlc = cd.parse_msg.dlc;
    let bits = if dlc >= 8 { 32 } else { (dlc - 4) * 8 };
    cd.parse_crc = crc_bytes(cd.parse_crc, data, dlc - 4);
    cd.parse_msg.set_data32(1, (data << (32 - bits)).swap_bytes());
    data_state_go_crc(cd);
}

// Handle reception of 16 bits of message CRC (15 crc bits + crc delimiter)
fn data_state_update_crc(cd: &mut Can2040, data: u32) {
    if ((cd.parse_crc << 1) | 1) != data {
        data_state_go_error(cd);
        return;
    }

    report_note_crc_success(cd);
    unstuf_clear_state(&mut cd.unstuf);
    data_state_go_next(cd, MS_ACK, 2);
}

// Handle reception of 2 bits of ack phase (ack, ack delimiter)
fn data_state_update_ack(cd: &mut Can2040, data: u32) {
    if data != 0x01 {
        // Undo unstuf_clear_state() for correct SOF detection in
        // data_state_line_passive()
        let bits = (cd.parse_crc_bits << 2) | data;
        unstuf_restore_state(&mut cd.unstuf, bits);

        data_state_go_error(cd);
        return;
    }
    report_note_ack_success(cd);
    data_state_go_next(cd, MS_EOF0, 4);
}

// Handle reception of first four end-of-frame (EOF) bits
fn data_state_update_eof0(cd: &mut Can2040, data: u32) {
    if data != 0x0f || pio_rx_check_stall(cd) {
        data_state_go_error(cd);
        return;
    }
    unstuf_clear_state(&mut cd.unstuf);
    data_state_go_next(cd, MS_EOF1, 5);
}

// Handle reception of end-of-frame (EOF) bits 5-7 and first two IFS bits
fn data_state_update_eof1(cd: &mut Can2040, data: u32) {
    if data == 0x1f {
        // Success
        report_note_eof_success(cd);
        data_state_go_next(cd, MS_START, 1);
    } else if data >= 0x1c || (data >= 0x18 && report_is_not_in_tx(cd)) {
        // Message fully transmitted - followed by "overload frame"
        report_note_eof_success(cd);
        data_state_go_discard(cd);
    } else {
        data_state_go_error(cd);
    }
}

// Handle data received while in MS_DISCARD state
fn data_state_update_discard(cd: &mut Can2040, _data: u32) {
    data_state_go_discard(cd);
}

// Update parsing state after reading the bits of the current field
fn data_state_update(cd: &mut Can2040, data: u32) {
    match cd.parse_state {
        MS_START => data_state_update_start(cd, data),
        MS_HEADER => data_state_update_header(cd, data),
        MS_EXT_HEADER => data_state_update_ext_header(cd, data),
        MS_DATA0 => data_state_update_data0(cd, data),
        MS_DATA1 => data_state_update_data1(cd, data),
        MS_CRC => data_state_update_crc(cd, data),
        MS_ACK => data_state_update_ack(cd, data),
        MS_EOF0 => data_state_update_eof0(cd, data),
        MS_EOF1 => data_state_update_eof1(cd, data),
        MS_DISCARD => data_state_update_discard(cd, data),
        _ => {}
    }
}

/****************************************************************
 * Input processing
 ****************************************************************/

// Process incoming data from PIO "rx" state machine
fn process_rx(cd: &mut Can2040, rx_data: u32) {
    unstuf_add_bits(&mut cd.unstuf, rx_data, PIO_RX_WAKE_BITS);
    cd.raw_bit_count = cd.raw_bit_count.wrapping_add(PIO_RX_WAKE_BITS);

    // undo bit stuffing
    loop {
        let ret = unstuf_pull_bits(&mut cd.unstuf);
        if ret > 0 {
            // Need more data
            break;
        } else if ret == 0 {
            // Pulled the next field - process it
            let bits = cd.unstuf.unstuffed_bits;
            data_state_update(cd, bits);
        } else if ret == -1 {
            // 6 consecutive high bits
            data_state_line_passive(cd);
        } else {
            // 6 consecutive low bits
            data_state_line_error(cd);
        }
    }
}

// Main API irq notification function
pub fn can2040_pio_irq_handler(cd: &mut Can2040) {
    let pio_hw = pio(cd);
    let mut ints = pio_hw.ints0.read();
    while ints & SI_RX_DATA != 0 {
        let rx_data = pio_hw.rxf[1].read();
        process_rx(cd, rx_data);
        ints = pio_hw.ints0.read();
        if ints == 0 {
            return;
        }
    }

    if ints & SI_ACKDONE != 0 {
        // Ack of received message completed successfully
        report_line_ackdone(cd);
    } else if ints & SI_MATCHED != 0 {
        // Transmit message completed successfully
        report_line_matched(cd);
    } else if ints & SI_MAYTX != 0 {
        // Bus is idle, but not all bits may have been flushed yet
        report_line_maytx(cd);
    } else if ints & SI_TXPENDING != 0 {
        // Schedule a transmit
        report_line_txpending(cd);
    }
}

/****************************************************************
 * Transmit queuing
 ****************************************************************/

// API function to check if transmit space available
pub fn can2040_check_transmit(cd: &Can2040) -> bool {
    let tx_pull_pos = readl(&cd.tx_pull_pos);
    let tx_push_pos = cd.tx_push_pos;
    let pending = tx_push_pos.wrapping_sub(tx_pull_pos);
    pending < cd.tx_queue.len() as u32
}

// API function to transmit a message
pub fn can2040_transmit(cd: &mut Can2040, msg: &Can2040Msg) -> i32 {
    let tx_pull_pos = readl(&cd.tx_pull_pos);
    let tx_push_pos = cd.tx_push_pos;
    let pending = tx_push_pos.wrapping_sub(tx_pull_pos);
    if pending >= cd.tx_queue.len() as u32 {
        // Tx queue full
        return -1;
    }

    // Copy msg into transmit queue
    let qpos = tx_qpos(cd, tx_push_pos);
    let qt = &mut cd.tx_queue[qpos];
    let id = msg.id;
    if id & CAN2040_ID_EFF != 0 {
        qt.msg.id = id & !0x20000000;
    } else {
        qt.msg.id = id & (CAN2040_ID_RTR | 0x7ff);
    }
    qt.msg.dlc = msg.dlc & 0x0f;
    let mut data_len = if qt.msg.dlc > 8 { 8 } else { qt.msg.dlc } as usize;
    if qt.msg.id & CAN2040_ID_RTR != 0 {
        data_len = 0;
    }
    qt.msg.data = [0; 8];
    qt.msg.data[..data_len].copy_from_slice(&msg.data[..data_len]);

    // Calculate crc and stuff bits
    let mut crc = 0;
    qt.stuffed_data = [0; 5];
    let msg_id = qt.msg.id;
    let edlc = qt.msg.dlc | if msg_id & CAN2040_ID_RTR != 0 { 0x40 } else { 0 };
    let data = qt.msg.data;
    let mut bs = BitstufferS { prev_stuffed: 1, bitpos: 0, buf: &mut qt.stuffed_data };
    if msg_id & CAN2040_ID_EFF != 0 {
        // Extended header
        let id = msg_id;
        let h1 = ((id & 0x1ffc0000) >> 11) | 0x60 | ((id & 0x3e000) >> 13);
        let h2 = ((id & 0x1fff) << 7) | edlc;
        crc = crc_bytes(crc, h1 >> 4, 2);
        crc = crc_bytes(crc, ((h1 & 0x0f) << 20) | h2, 3);
        bs_push(&mut bs, h1, 19);
        bs_push(&mut bs, h2, 20);
    } else {
        // Standard header
        let hdr = ((msg_id & 0x7ff) << 7) | edlc;
        crc = crc_bytes(crc, hdr, 3);
        bs_push(&mut bs, hdr, 19);
    }
    for &b in &data[..data_len] {
        let v = b as u32;
        crc = crc_byte(crc, v);
        bs_push(&mut bs, v, 8);
    }
    let qcrc = crc & 0x7fff;
    bs_push(&mut bs, qcrc, 15);
    bs_pushraw(&mut bs, 1, 1);
    let words = bs_finalize(&mut bs);
    qt.crc = qcrc;
    qt.stuffed_words = words;

    // Submit
    writel(&mut cd.tx_push_pos, tx_push_pos.wrapping_add(1));

    // Wakeup if in TS_IDLE state
    __DMB();
    pio_signal_set_txpending(cd);

    0
}

/****************************************************************
 * Setup
 ****************************************************************/

// API function to initialize can2040 code
pub fn can2040_setup(cd: &mut Can2040, pio_num: u32) {
    *cd = Can2040::new();
    cd.pio_num = (pio_num != 0) as u32;
    cd.pio_hw = if cd.pio_num != 0 { pio1_hw } else { pio0_hw };
    #[cfg(CONFIG_MACH_RP2350)]
    if pio_num == 2 {
        cd.pio_num = pio_num;
        cd.pio_hw = pio2_hw;
    }
}

// API function to configure callback
pub fn can2040_callback_config(cd: &mut Can2040, rx_cb: Can2040RxCb) {
    cd.rx_cb = Some(rx_cb);
}

// API function to start CANbus interface
pub fn can2040_start(cd: &mut Can2040, sys_clock: u32, bitrate: u32, gpio_rx: u32, gpio_tx: u32) {
    cd.gpio_rx = gpio_rx;
    cd.gpio_tx = gpio_tx;
    data_state_clear_bits(cd);
    pio_setup(cd, sys_clock, bitrate);
    data_state_go_discard(cd);
}

// API function to stop can2040 code
pub fn can2040_stop(cd: &mut Can2040) {
    pio_irq_disable(cd);
    pio_sm_setup(cd);
}

// API function to access can2040 statistics
pub fn can2040_get_statistics(cd: &Can2040, stats: &mut Can2040Stats) {
    loop {
        *stats = unsafe { read_volatile(&cd.stats) };
        if *stats == unsafe { read_volatile(&cd.stats) } {
            // Successfully copied data
            return;
        }
        // Raced with irq handler update - retry copy
    }
}
