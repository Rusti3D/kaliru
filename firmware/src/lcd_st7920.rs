// Commands for sending messages to an st7920 lcd driver
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
struct St7920 {
    last_cmd_time: u32,
    sync_wait_ticks: u32,
    cmd_wait_ticks: u32,
    sclk: GpioOut,
    sid: GpioOut,
}

fn oid_type() -> OidType {
    command_config_st7920 as *const ()
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

const SYNC_CMD: u8 = 0xf8;
const SYNC_DATA: u8 = 0xfa;

// Write eight bits to the st7920 via the serial interface
fn st7920_xmit_byte(s: &St7920, mut data: u8) -> R {
    let (sclk, sid) = (s.sclk, s.sid);
    for _ in 0..8 {
        if data & 0x80 != 0 {
            gpio_out_toggle(sid);
            data = !data;
        }
        ndelay(nsecs_to_ticks(200))?;
        gpio_out_toggle(sclk);
        ndelay(nsecs_to_ticks(200))?;
        data <<= 1;
        gpio_out_toggle(sclk);
    }
    Ok(())
}

// Transmit a series of command bytes to the chip
fn st7920_xmit(s: &mut St7920, cmds: &[u8]) -> R {
    let Some((&first, rest)) = cmds.split_first() else {
        return Ok(());
    };

    // Send first byte (with longer delay)
    let mut last_cmd_time = s.last_cmd_time;
    let mut wait_ticks = s.sync_wait_ticks;
    st7920_xmit_byte(s, first & 0xf0)?;
    while timer_read_time().wrapping_sub(last_cmd_time) < wait_ticks {
        // Can't complete transfer until delay complete
        irq_poll()?;
    }
    st7920_xmit_byte(s, first << 4)?;
    last_cmd_time = timer_read_time();

    // Send subsequent bytes
    wait_ticks = s.cmd_wait_ticks;
    for &cmd in rest {
        st7920_xmit_byte(s, cmd & 0xf0)?;
        while timer_read_time().wrapping_sub(last_cmd_time) < wait_ticks {
            // Can't complete transfer until delay complete
            irq_poll()?;
        }
        st7920_xmit_byte(s, cmd << 4)?;
        last_cmd_time = timer_read_time();
    }

    s.last_cmd_time = last_cmd_time;
    Ok(())
}

/****************************************************************
 * Interface
 ****************************************************************/

pub fn command_config_st7920(args: &Args) -> R {
    let s = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<St7920>() as u16)? as *mut St7920) };
    s.sclk = gpio_out_setup(args[2], 0)?;
    s.sid = gpio_out_setup(args[3], 0)?;
    gpio_out_setup(args[1], 1)?;

    if !CONFIG_HAVE_STRICT_TIMING {
        s.sync_wait_ticks = args[4];
        s.cmd_wait_ticks = args[5];
        return Ok(());
    }

    // Calibrate cmd_wait_ticks
    st7920_xmit_byte(s, SYNC_CMD)?;
    st7920_xmit_byte(s, 0x20)?;
    irq_disable();
    let start = timer_read_time();
    st7920_xmit_byte(s, 0x00)?;
    let end = timer_read_time();
    irq_enable();
    s.last_cmd_time = end;
    let diff = end.wrapping_sub(start);
    let sync_delay_ticks = args[4];
    if sync_delay_ticks > diff {
        s.sync_wait_ticks = sync_delay_ticks - diff;
    }
    let cmd_delay_ticks = args[5];
    if cmd_delay_ticks > diff {
        s.cmd_wait_ticks = cmd_delay_ticks - diff;
    }
    Ok(())
}
decl_command!(
    command_config_st7920,
    concat!("config_st7920 oid=%c cs_pin=%u sclk_pin=%u sid_pin=%u", " sync_delay_ticks=%u cmd_delay_ticks=%u")
);

pub fn command_st7920_send_cmds(args: &Args) -> R {
    let s = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut St7920) };
    st7920_xmit_byte(s, SYNC_CMD)?;
    let cmds = command_decode_buffer(args[1] as u8 as u32, args[2]);
    st7920_xmit(s, cmds)
}
decl_command!(command_st7920_send_cmds, "st7920_send_cmds oid=%c cmds=%*s");

pub fn command_st7920_send_data(args: &Args) -> R {
    let s = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut St7920) };
    st7920_xmit_byte(s, SYNC_DATA)?;
    let data = command_decode_buffer(args[1] as u8 as u32, args[2]);
    st7920_xmit(s, data)
}
decl_command!(command_st7920_send_data, "st7920_send_data oid=%c data=%*s");

pub fn st7920_shutdown() -> R {
    for p in foreach_oid(oid_type()) {
        let s = unsafe { &*(p as *mut St7920) };
        gpio_out_write(s.sclk, 0);
        gpio_out_write(s.sid, 0);
    }
    Ok(())
}
decl_shutdown!(st7920_shutdown);
