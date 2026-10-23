// Support for bit-banging commands to WS2812 type "neopixel" LEDs
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_out_setup, gpio_out_toggle_noirq, gpio_out_write, irq_disable, irq_enable, irq_poll, timer_from_us, timer_read_time, GpioOut};
use crate::command::command_decode_ptr;
use crate::sched::R;
use crate::{decl_command, sendf, shutdown};
use core::mem::size_of;

// The WS2812 uses a bit-banging protocol where each bit is
// transmitted as a gpio high pulse of variable length.  The various
// specs are unclear, but it is believed the timing requirements are:
// - A zero bit must have a high pulse less than 500ns.
// - A one bit must have a high pulse longer than 650ns.
// - The total bit time (gpio high to following gpio high) must not
//   exceed ~5000ns. The average bit time must be at least 1250ns.
// - The specs generally indicate a minimum high pulse and low pulse
//   of 200ns, but the actual requirement might be smaller.

/****************************************************************
 * Timing
 ****************************************************************/

type NeopixelTime = u32;

#[inline(always)]
fn nsecs_to_ticks(ns: u32) -> NeopixelTime {
    timer_from_us(ns.wrapping_mul(1000)).div_ceil(1000000)
}

#[inline]
fn neopixel_check_elapsed(t1: NeopixelTime, t2: NeopixelTime, ticks: NeopixelTime) -> bool {
    t2.wrapping_sub(t1) >= ticks
}

fn neopixel_get_time() -> NeopixelTime {
    timer_read_time()
}

#[inline]
fn neopixel_delay(start: NeopixelTime, ticks: NeopixelTime) {
    while !neopixel_check_elapsed(start, neopixel_get_time(), ticks) {}
}

// Minimum amount of time for a '1 bit' to be reliably detected
fn pulse_long_ticks() -> NeopixelTime {
    nsecs_to_ticks(800)
}
// Minimum amount of time for any level change to be reliably detected
fn edge_min_ticks() -> NeopixelTime {
    nsecs_to_ticks(200)
}
// Minimum average time needed to transmit each bit (two level changes)
fn bit_min_ticks() -> NeopixelTime {
    nsecs_to_ticks(1250)
}

/****************************************************************
 * Neopixel interface
 ****************************************************************/

#[repr(C)]
struct Neopixel {
    pin: GpioOut,
    bit_max_ticks: NeopixelTime,
    last_req_time: u32,
    reset_min_ticks: u32,
    data_size: u16,
    data: [u8; 0],
}

impl Neopixel {
    fn data(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.data.as_mut_ptr(), self.data_size as usize) }
    }
}

fn oid_type() -> OidType {
    command_config_neopixel as *const ()
}

pub fn command_config_neopixel(args: &[u32]) -> R {
    let pin = gpio_out_setup(args[1], 0)?;
    let data_size = args[2] as u16;
    if data_size & 0x8000 != 0 {
        shutdown!("Invalid neopixel data_size");
    }
    let n = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), (size_of::<Neopixel>() + data_size as usize) as u16)? as *mut Neopixel) };
    n.pin = pin;
    n.data_size = data_size;
    n.bit_max_ticks = args[3];
    n.reset_min_ticks = args[4];
    Ok(())
}
decl_command!(
    command_config_neopixel,
    concat!("config_neopixel oid=%c pin=%u", " data_size=%hu bit_max_ticks=%u reset_min_ticks=%u")
);

fn send_data(n: &mut Neopixel) -> R<i32> {
    // Make sure the reset time has elapsed since last request
    let (last_req_time, rmt) = (n.last_req_time, n.reset_min_ticks);
    let mut cur = timer_read_time();
    while cur.wrapping_sub(last_req_time) < rmt {
        irq_poll()?;
        cur = timer_read_time();
    }

    // Transmit data
    let pin = n.pin;
    let mut last_start = neopixel_get_time();
    let bit_max_ticks = n.bit_max_ticks;
    'fail: {
        for &b in n.data().iter() {
            let mut byte = b;
            for _ in 0..8 {
                if byte & 0x80 != 0 {
                    // Long pulse
                    neopixel_delay(last_start, bit_min_ticks());
                    irq_disable();
                    let start = neopixel_get_time();
                    gpio_out_toggle_noirq(pin);
                    irq_enable();

                    if neopixel_check_elapsed(last_start, start, bit_max_ticks) {
                        break 'fail;
                    }
                    last_start = start;
                    byte <<= 1;

                    neopixel_delay(start, pulse_long_ticks());
                    irq_disable();
                    gpio_out_toggle_noirq(pin);
                    irq_enable();

                    neopixel_delay(neopixel_get_time(), edge_min_ticks());
                } else {
                    // Short pulse
                    neopixel_delay(last_start, bit_min_ticks());
                    irq_disable();
                    let start = neopixel_get_time();
                    gpio_out_toggle_noirq(pin);
                    neopixel_delay(start, edge_min_ticks());
                    gpio_out_toggle_noirq(pin);
                    irq_enable();

                    if neopixel_check_elapsed(last_start, start, bit_max_ticks) {
                        break 'fail;
                    }
                    last_start = start;
                    byte <<= 1;
                }
            }
        }
        n.last_req_time = timer_read_time();
        return Ok(0);
    }
    // A hardware irq messed up the transmission - report a failure
    gpio_out_write(pin, 0);
    n.last_req_time = timer_read_time();
    Ok(-1)
}

pub fn command_neopixel_update(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let n = unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut Neopixel) };
    let pos = args[1] as u16 as usize;
    let data_len = args[2] as u8 as usize;
    let data = command_decode_ptr(args[3]);
    if pos & 0x8000 != 0 || pos + data_len > n.data_size as usize {
        shutdown!("Invalid neopixel update command");
    }
    n.data()[pos..pos + data_len].copy_from_slice(unsafe { core::slice::from_raw_parts(data, data_len) });
    Ok(())
}
decl_command!(command_neopixel_update, "neopixel_update oid=%c pos=%hu data=%*s");

pub fn command_neopixel_send(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let n = unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut Neopixel) };
    let ret = send_data(n)?;
    sendf!("neopixel_result oid=%c success=%c", oid, if ret != 0 { 0u8 } else { 1 })
}
decl_command!(command_neopixel_send, "neopixel_send oid=%c");
