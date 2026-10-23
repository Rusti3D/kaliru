// Commands for sending messages to a TMC2208 via its single wire UART
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{foreach_oid, oid_alloc, oid_lookup, OidType};
use crate::board::{
    gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, gpio_out_toggle_noirq, gpio_out_write, irq_disable,
    irq_enable, timer_from_us, timer_read_time, GpioIn, GpioOut,
};
use crate::command::command_decode_ptr;
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_wake_task, TaskWake, Timer, R, SF_DONE, SF_RESCHEDULE};
use crate::{decl_command, decl_shutdown, decl_task, sendf, shutdown};
use core::mem::size_of;

#[repr(C)]
struct Tmcuart {
    timer: Timer,
    tx_pin: GpioOut,
    rx_pin: GpioIn,
    flags: u8,
    pos: u8,
    read_count: u8,
    write_count: u8,
    cfg_bit_time: u32,
    bit_time: u32,
    data: [u8; 10],
}

const TU_LINE_HIGH: u8 = 1 << 0;
const TU_ACTIVE: u8 = 1 << 1;
const TU_READ_SYNC: u8 = 1 << 2;
const TU_REPORT: u8 = 1 << 3;
const TU_PULLUP: u8 = 1 << 4;
const TU_SINGLE_WIRE: u8 = 1 << 5;

static TMCUART_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_tmcuart as *const ()
}

// Restore uart line to normal "idle" mode
fn tmcuart_reset_line(t: &mut Tmcuart) -> R {
    if t.flags & TU_SINGLE_WIRE != 0 {
        gpio_out_reset(t.tx_pin, 1)?;
    } else {
        gpio_out_write(t.tx_pin, 1);
    }
    t.flags = (t.flags & (TU_PULLUP | TU_SINGLE_WIRE)) | TU_LINE_HIGH;
    Ok(())
}

// Helper function to end a transmission and schedule a response
fn tmcuart_finalize(t: &mut Tmcuart) -> R<u8> {
    tmcuart_reset_line(t)?;
    t.flags |= TU_REPORT;
    sched_wake_task(TMCUART_WAKE.ptr());
    Ok(SF_DONE)
}

// Event handler for reading uart bits
unsafe fn tmcuart_read_event(timer: *mut Timer) -> R<u8> {
    let t = &mut *(timer as *mut Tmcuart);
    let v = gpio_in_read(t.rx_pin);
    // Read and store bit
    let mut pos = t.pos;
    let mask = 1u8 << (pos & 0x07);
    let mut data = t.data[(pos >> 3) as usize];
    if v != 0 {
        data |= mask;
    } else {
        data &= !mask;
    }
    t.data[(pos >> 3) as usize] = data;
    pos = pos.wrapping_add(1);
    if pos >= t.read_count {
        return tmcuart_finalize(t);
    }
    t.pos = pos;
    t.timer.waketime = t.timer.waketime.wrapping_add(t.bit_time);
    Ok(SF_RESCHEDULE)
}

// Event handler for detecting start of data reception
unsafe fn tmcuart_read_sync_event(timer: *mut Timer) -> R<u8> {
    let t = &mut *(timer as *mut Tmcuart);
    let v = gpio_in_read(t.rx_pin);
    if v != 0 {
        t.flags |= TU_READ_SYNC;
    } else if t.flags & TU_READ_SYNC != 0 {
        // Now synchronized - begin reading
        t.pos = 0;
        t.timer.func = Some(tmcuart_read_event);
        return tmcuart_read_event(timer);
    }
    let pos = t.pos;
    t.pos = pos.wrapping_add(1);
    if pos >= 64 {
        // Timeout
        t.read_count = 0;
        return tmcuart_finalize(t);
    }
    t.timer.waketime = t.timer.waketime.wrapping_add(t.bit_time);
    Ok(SF_RESCHEDULE)
}

// Event handler called at end of uart writing
unsafe fn tmcuart_send_finish_event(timer: *mut Timer) -> R<u8> {
    let t = &mut *(timer as *mut Tmcuart);
    if t.read_count == 0 {
        // This is a tx only operation - success
        return tmcuart_finalize(t);
    }
    // Prepare for message rx
    if t.flags & TU_SINGLE_WIRE != 0 {
        gpio_in_reset(t.rx_pin, (t.flags & TU_PULLUP) as crate::board::GpioPullUp)?;
    }
    t.pos = 0;
    t.timer.func = Some(tmcuart_read_sync_event);
    t.timer.waketime = t.timer.waketime.wrapping_add(t.bit_time.wrapping_mul(4));
    Ok(SF_RESCHEDULE)
}

// Event handler for sending uart bits
unsafe fn tmcuart_send_event(timer: *mut Timer) -> R<u8> {
    let t = &mut *(timer as *mut Tmcuart);
    // Toggle uart output
    gpio_out_toggle_noirq(t.tx_pin);
    t.flags ^= TU_LINE_HIGH;
    // Determine next toggle time
    let line_state = t.flags & TU_LINE_HIGH;
    let bit_time = t.bit_time;
    let mut next = bit_time;
    let mut pos = t.pos;
    loop {
        pos = pos.wrapping_add(1);
        if pos >= t.write_count {
            // No more toggles necessary - schedule finish event
            t.timer.func = Some(tmcuart_send_finish_event);
            t.timer.waketime = t.timer.waketime.wrapping_add(next);
            return Ok(SF_RESCHEDULE);
        }
        let data = t.data[(pos >> 3) as usize];
        let bit = (data >> (pos & 0x07)) & 0x01;
        if bit != line_state {
            break;
        }
        next = next.wrapping_add(bit_time);
    }
    t.pos = pos;
    t.timer.waketime = t.timer.waketime.wrapping_add(next);
    Ok(SF_RESCHEDULE)
}

// Event handler for sending sync nibble with enhanced baud detection
unsafe fn tmcuart_send_sync_event(timer: *mut Timer) -> R<u8> {
    let t = &mut *(timer as *mut Tmcuart);
    // Toggle uart output and note toggle time
    gpio_out_toggle_noirq(t.tx_pin);
    let cur = timer_read_time();
    t.flags ^= TU_LINE_HIGH;
    // Determine next wakeup time
    t.pos = t.pos.wrapping_add(1);
    if t.pos == 1 {
        // First bit just sent - record scheduling offset for later use
        let offset = cur.wrapping_sub(t.timer.waketime);
        t.bit_time = offset;
    } else if t.pos >= 5 {
        // Last bit of sync nibble just sent - calculate actual baud rate used
        let offset = cur.wrapping_sub(t.timer.waketime);
        let start_offset = t.bit_time;
        let diff = offset.wrapping_sub(start_offset) as i32;
        // DIV_ROUND_CLOSEST(diff, 4)
        t.bit_time = t.cfg_bit_time.wrapping_add(((diff + 2) / 4) as u32);
        t.timer.func = Some(tmcuart_send_event);
        t.timer.waketime = t.timer.waketime.wrapping_add((diff as u32).wrapping_add(t.bit_time));
        return Ok(SF_RESCHEDULE);
    }
    t.timer.waketime = t.timer.waketime.wrapping_add(t.cfg_bit_time);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_tmcuart(args: &[u32]) -> R {
    let t = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Tmcuart>() as u16)? as *mut Tmcuart) };
    let pull_up = args[2] as u8;
    let (rx_pin, tx_pin) = (args[1], args[3]);
    t.rx_pin = gpio_in_setup(rx_pin, (pull_up != 0) as crate::board::GpioPullUp)?;
    t.tx_pin = gpio_out_setup(tx_pin, 1)?;
    t.cfg_bit_time = args[4];
    t.flags = TU_LINE_HIGH | (if pull_up != 0 { TU_PULLUP } else { 0 }) | (if rx_pin == tx_pin { TU_SINGLE_WIRE } else { 0 });
    Ok(())
}
decl_command!(
    command_config_tmcuart,
    concat!("config_tmcuart oid=%c rx_pin=%u pull_up=%c", " tx_pin=%u bit_time=%u")
);

// Parse and schedule a TMC UART transmission request
pub fn command_tmcuart_send(args: &[u32]) -> R {
    let t = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Tmcuart) };
    if t.flags & TU_ACTIVE != 0 {
        // Uart is busy - silently drop this request (host should retransmit)
        return Ok(());
    }
    let write_len = args[1] as u8 as usize;
    let write = command_decode_ptr(args[2]);
    let read_len = args[3] as u8 as usize;
    if write_len > t.data.len() || read_len > t.data.len() {
        shutdown!("tmcuart data too large");
    }
    t.data[..write_len].copy_from_slice(unsafe { core::slice::from_raw_parts(write, write_len) });
    t.pos = 0;
    t.flags = (t.flags & (TU_LINE_HIGH | TU_PULLUP | TU_SINGLE_WIRE)) | TU_ACTIVE;
    t.write_count = (write_len * 8) as u8;
    t.read_count = (read_len * 8) as u8;
    if write_len >= 1 && (t.data[0] & 0x3f) == 0x2a {
        t.timer.func = Some(tmcuart_send_sync_event);
    } else {
        t.bit_time = t.cfg_bit_time;
        t.timer.func = Some(tmcuart_send_event);
    }
    irq_disable();
    t.timer.waketime = timer_read_time().wrapping_add(timer_from_us(200));
    sched_add_timer(&mut t.timer)?;
    irq_enable();
    Ok(())
}
decl_command!(command_tmcuart_send, "tmcuart_send oid=%c write=%*s read=%c");

// Report completed response message back to host
pub fn tmcuart_task() -> R {
    if !sched_check_wake(TMCUART_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let t = unsafe { &mut *(p as *mut Tmcuart) };
        if t.flags & TU_REPORT == 0 {
            continue;
        }
        irq_disable();
        t.flags &= !TU_REPORT;
        irq_enable();
        sendf!("tmcuart_response oid=%c read=%*s", oid, &t.data[..(t.read_count / 8) as usize])?;
    }
    Ok(())
}
decl_task!(tmcuart_task);

pub fn tmcuart_shutdown() -> R {
    for p in foreach_oid(oid_type()) {
        tmcuart_reset_line(unsafe { &mut *(p as *mut Tmcuart) })?;
    }
    Ok(())
}
decl_shutdown!(tmcuart_shutdown);
