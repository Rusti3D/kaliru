// Commands for sending messages to a 4-bit hd44780 lcd driver
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{foreach_oid, oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_out_setup, gpio_out_toggle, gpio_out_write, irq_disable, irq_enable, irq_poll, timer_from_us, timer_is_before, timer_read_time, GpioOut};
use crate::command::{command_decode_buffer, Args};
use crate::config::{CONFIG_HAVE_STRICT_TIMING, CONFIG_MACH_AVR};
use crate::sched::R;
use crate::{decl_command, decl_shutdown};
use core::mem::size_of;

#[repr(C)]
struct Hd44780 {
    last_cmd_time: u32,
    cmd_wait_ticks: u32,
    last: u8,
    rs: GpioOut,
    e: GpioOut,
    d4: GpioOut,
    d5: GpioOut,
    d6: GpioOut,
    d7: GpioOut,
}

fn oid_type() -> OidType {
    command_config_hd44780 as *const ()
}

/****************************************************************
 * Transmit functions
 ****************************************************************/

#[inline(always)]
fn nsecs_to_ticks(ns: u32) -> u32 {
    timer_from_us(ns.wrapping_mul(1000)) / 1000000
}

fn ndelay(ticks: u32) -> R {
    if CONFIG_MACH_AVR {
        // Slower MCUs don't require a delay
        return Ok(());
    }
    let end = timer_read_time().wrapping_add(ticks);
    while timer_is_before(timer_read_time(), end) {
        irq_poll()?;
    }
    Ok(())
}

// Write 4 bits to the hd44780 using the 4bit parallel interface
#[inline(always)]
fn hd44780_xmit_bits(toggle: u8, e: GpioOut, d4: GpioOut, d5: GpioOut, d6: GpioOut, d7: GpioOut) -> R {
    gpio_out_toggle(e);
    if toggle & 0x10 != 0 {
        gpio_out_toggle(d4);
    }
    if toggle & 0x20 != 0 {
        gpio_out_toggle(d5);
    }
    if toggle & 0x40 != 0 {
        gpio_out_toggle(d6);
    }
    if toggle & 0x80 != 0 {
        gpio_out_toggle(d7);
    }
    ndelay(nsecs_to_ticks(230))?;
    gpio_out_toggle(e);
    Ok(())
}

// Transmit 8 bits to the chip
fn hd44780_xmit_byte(h: &mut Hd44780, data: u8) -> R {
    let (e, d4, d5, d6, d7) = (h.e, h.d4, h.d5, h.d6, h.d7);
    hd44780_xmit_bits(h.last ^ data, e, d4, d5, d6, d7)?;
    h.last = data << 4;
    ndelay(nsecs_to_ticks(500 - 230))?;
    hd44780_xmit_bits(data ^ h.last, e, d4, d5, d6, d7)
}

// Transmit a series of bytes to the chip
fn hd44780_xmit(h: &mut Hd44780, data: &[u8]) -> R {
    let mut last_cmd_time = h.last_cmd_time;
    let cmd_wait_ticks = h.cmd_wait_ticks;
    for &b in data {
        while timer_read_time().wrapping_sub(last_cmd_time) < cmd_wait_ticks {
            irq_poll()?;
        }
        hd44780_xmit_byte(h, b)?;
        last_cmd_time = timer_read_time();
    }
    h.last_cmd_time = last_cmd_time;
    Ok(())
}

/****************************************************************
 * Interface
 ****************************************************************/

pub fn command_config_hd44780(args: &Args) -> R {
    let h = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Hd44780>() as u16)? as *mut Hd44780) };
    h.rs = gpio_out_setup(args[1], 0)?;
    h.e = gpio_out_setup(args[2], 0)?;
    h.d4 = gpio_out_setup(args[3], 0)?;
    h.d5 = gpio_out_setup(args[4], 0)?;
    h.d6 = gpio_out_setup(args[5], 0)?;
    h.d7 = gpio_out_setup(args[6], 0)?;

    if !CONFIG_HAVE_STRICT_TIMING {
        h.cmd_wait_ticks = args[7];
        return Ok(());
    }

    // Calibrate cmd_wait_ticks
    irq_disable();
    let start = timer_read_time();
    hd44780_xmit_byte(h, 0)?;
    let end = timer_read_time();
    irq_enable();
    let diff = end.wrapping_sub(start);
    let delay_ticks = args[7];
    if delay_ticks > diff {
        h.cmd_wait_ticks = delay_ticks - diff;
    }
    Ok(())
}
decl_command!(
    command_config_hd44780,
    concat!("config_hd44780 oid=%c rs_pin=%u e_pin=%u", " d4_pin=%u d5_pin=%u d6_pin=%u d7_pin=%u delay_ticks=%u")
);

pub fn command_hd44780_send_cmds(args: &Args) -> R {
    let h = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Hd44780) };
    gpio_out_write(h.rs, 0);
    let cmds = command_decode_buffer(args[1] as u8 as u32, args[2]);
    hd44780_xmit(h, cmds)
}
decl_command!(command_hd44780_send_cmds, "hd44780_send_cmds oid=%c cmds=%*s");

pub fn command_hd44780_send_data(args: &Args) -> R {
    let h = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Hd44780) };
    gpio_out_write(h.rs, 1);
    let data = command_decode_buffer(args[1] as u8 as u32, args[2]);
    hd44780_xmit(h, data)
}
decl_command!(command_hd44780_send_data, "hd44780_send_data oid=%c data=%*s");

pub fn hd44780_shutdown() -> R {
    for p in foreach_oid(oid_type()) {
        let h = unsafe { &mut *(p as *mut Hd44780) };
        gpio_out_write(h.rs, 0);
        gpio_out_write(h.e, 0);
        gpio_out_write(h.d4, 0);
        gpio_out_write(h.d5, 0);
        gpio_out_write(h.d6, 0);
        gpio_out_write(h.d7, 0);
        h.last = 0;
    }
    Ok(())
}
decl_shutdown!(hd44780_shutdown);
