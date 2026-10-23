// Support for the Chipsea CS1237 24-bit ADC
//
// Copyright (C) 2026  Xiaoyue Cui <2508041672@qq.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{
    gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, gpio_out_write, irq_disable, irq_enable, irq_poll,
    timer_from_us, timer_is_before, timer_read_time, GpioIn, GpioOut,
};
use crate::config::CONFIG_MACH_AVR;
use crate::ctr::Global;
use crate::load_cell_probe::{load_cell_probe_oid_lookup, load_cell_probe_report_sample, LoadCellProbe};
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, sensor_bulk_status, SensorBulk};
use crate::{decl_command, decl_enumeration, decl_task};
use core::mem::size_of;

#[repr(C)]
struct Cs1237Adc {
    timer: Timer,
    rest_ticks: u32,
    config: u8,
    flags: u8,
    is_configured: u8,
    dout: GpioIn,
    dout_out: GpioOut,
    sclk: GpioOut,
    sb: SensorBulk,
    lce: *mut LoadCellProbe,
}

const CS_PENDING: u8 = 1 << 0;
const CS_OVERFLOW: u8 = 1 << 1;
const CS_CONFIG_PENDING: u8 = 1 << 2;
const CS_CONFIG_VERIFY: u8 = 1 << 3;

// Internal errors transmitted in sample reports
const SE_DESYNC: u32 = 1;
const SE_READ_TOO_LONG: u32 = 2;
const SE_CONFIG: u32 = 3;

decl_enumeration!("cs1237_error:", "SERIAL_DATA_DESYNCHRONISED", SE_DESYNC);
decl_enumeration!("cs1237_error:", "READ_TOOK_TOO_LONG", SE_READ_TOO_LONG);
decl_enumeration!("cs1237_error:", "CONFIG_NOT_ACCEPTED", SE_CONFIG);

const BYTES_PER_SAMPLE: u8 = 4;
const CS1237_WRITE_CONFIG: u32 = 0x65;

static WAKE_CS1237: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_cs1237 as *const ()
}

fn lookup(oid: u8) -> R<&'static mut Cs1237Adc> {
    Ok(unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut Cs1237Adc) })
}

// The data sheet specifies at least 455ns for each SCLK high/low pulse.
fn min_pulse_time() -> u32 {
    nsecs_to_ticks(455)
}

fn nsecs_to_ticks(ns: u32) -> u32 {
    timer_from_us(ns.wrapping_mul(1000)).div_ceil(1000000)
}

fn cs1237_delay_noirq() {
    if CONFIG_MACH_AVR {
        // Avoid slow timer calculations for this sub-microsecond delay.
        return;
    }
    let end = timer_read_time().wrapping_add(min_pulse_time());
    while timer_is_before(timer_read_time(), end) {}
}

fn cs1237_delay() -> R {
    if CONFIG_MACH_AVR {
        // Pin and interrupt operations provide the remaining low pulse time.
        return Ok(());
    }
    let end = timer_read_time().wrapping_add(min_pulse_time());
    while timer_is_before(timer_read_time(), end) {
        irq_poll()?;
    }
    Ok(())
}

fn cs1237_read_bits(cs: &Cs1237Adc, num_bits: u32) -> R<u32> {
    let mut value: u32 = 0;
    for _ in 0..num_bits {
        irq_disable();
        gpio_out_write(cs.sclk, 1);
        cs1237_delay_noirq();
        let bit = gpio_in_read(cs.dout);
        gpio_out_write(cs.sclk, 0);
        irq_enable();
        cs1237_delay()?;
        value = (value << 1) | bit as u32;
    }
    Ok(value)
}

fn cs1237_write_bits(cs: &Cs1237Adc, value: u32, num_bits: u32) -> R {
    let mut mask: u32 = 1u32.wrapping_shl(num_bits.wrapping_sub(1));
    while mask != 0 {
        gpio_out_write(cs.dout_out, (value & mask != 0) as u8);
        cs1237_delay()?;
        irq_disable();
        gpio_out_write(cs.sclk, 1);
        cs1237_delay_noirq();
        gpio_out_write(cs.sclk, 0);
        irq_enable();
        cs1237_delay()?;
        mask >>= 1;
    }
    Ok(())
}

// Called immediately after the 24 ADC data clocks. Clocks 25-29 announce a
// command, clocks 30-36 contain the seven bit command, clock 37 changes bus
// direction, clocks 38-45 contain Config, and clock 46 releases DRDY/DOUT.
fn cs1237_write_config(cs: &Cs1237Adc) -> R {
    cs1237_read_bits(cs, 5)?;
    let cmd = (CS1237_WRITE_CONFIG << 9) | (1 << 8) | cs.config as u32;
    gpio_out_reset(cs.dout_out, 0)?;
    cs1237_write_bits(cs, cmd, 7 + 1 + 8)?;
    gpio_in_reset(cs.dout, 0)?;
    cs1237_read_bits(cs, 1)?;
    Ok(())
}

fn cs1237_is_data_ready(cs: &Cs1237Adc) -> bool {
    gpio_in_read(cs.dout) == 0
}

unsafe fn cs1237_event(timer: *mut Timer) -> R<u8> {
    let cs = &mut *(timer as *mut Cs1237Adc);
    let mut rest_ticks = cs.rest_ticks;
    if cs.flags & CS_PENDING != 0 {
        cs.sb.possible_overflows = cs.sb.possible_overflows.wrapping_add(1);
        cs.flags |= CS_OVERFLOW;
        rest_ticks = rest_ticks.wrapping_mul(4);
    } else if cs1237_is_data_ready(cs) {
        cs.flags |= CS_PENDING;
        sched_wake_task(WAKE_CS1237.ptr());
        rest_ticks = rest_ticks.wrapping_mul(8);
    }
    cs.timer.waketime = cs.timer.waketime.wrapping_add(rest_ticks);
    Ok(SF_RESCHEDULE)
}

fn add_sample(cs: &mut Cs1237Adc, oid: u8, counts: u32) -> R {
    let dc = cs.sb.data_count as usize;
    cs.sb.data[dc..dc + 4].copy_from_slice(&counts.to_le_bytes());
    cs.sb.data_count += BYTES_PER_SAMPLE;
    if cs.sb.data_count as usize + BYTES_PER_SAMPLE as usize > cs.sb.data.len() {
        sensor_bulk_report(&mut cs.sb, oid)?;
    }
    Ok(())
}

fn cs1237_read_adc(cs: &mut Cs1237Adc, oid: u8) -> R {
    let mut old_flags = cs.flags;
    let raw;

    if old_flags & CS_CONFIG_PENDING != 0 {
        // Read and discard the sample produced with the power-on settings.
        raw = cs1237_read_bits(cs, 24)?;
        cs1237_write_config(cs)?;
    } else {
        // Bits 25-26 are update1 and a reserved zero.  Clock 27 only
        // forces DOUT high until the next conversion is ready - its
        // level depends on when the chip drives the line, not on the
        // conversion, so discard it instead of checking it.
        raw = cs1237_read_bits(cs, 27)? >> 1;
    }

    irq_disable();
    old_flags |= cs.flags;
    cs.flags &= !(CS_PENDING | CS_OVERFLOW);
    if old_flags & CS_CONFIG_PENDING != 0 {
        cs.flags &= !CS_CONFIG_PENDING;
        cs.flags |= CS_CONFIG_VERIFY;
    }
    irq_enable();

    if old_flags & CS_CONFIG_PENDING != 0 {
        return Ok(());
    }

    let status = raw & 0x03;
    let mut counts = raw >> 2;
    if counts & 0x800000 != 0 {
        counts |= 0xff000000;
    }

    let mut error = 0;
    if status & 0x01 != 0 {
        error = SE_DESYNC;
    } else if old_flags & CS_CONFIG_VERIFY != 0 && status & 0x02 == 0 {
        error = SE_CONFIG;
    } else if old_flags & CS_OVERFLOW != 0 {
        error = SE_READ_TOO_LONG;
    }

    if old_flags & CS_CONFIG_VERIFY != 0 {
        irq_disable();
        cs.flags &= !CS_CONFIG_VERIFY;
        if error != 0 {
            cs.flags |= CS_CONFIG_PENDING;
        } else {
            cs.is_configured = 1;
        }
        irq_enable();
    }
    if error != 0 {
        counts = error << 24;
    } else if !cs.lce.is_null() {
        // probe is optional, report if enabled
        load_cell_probe_report_sample(cs.lce, counts as i32)?;
    }
    add_sample(cs, oid, counts)
}

pub fn command_config_cs1237(args: &[u32]) -> R {
    let cs = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Cs1237Adc>() as u16)? as *mut Cs1237Adc) };
    cs.timer.func = Some(cs1237_event);
    cs.config = (args[1] & 0x7f) as u8;
    cs.dout = gpio_in_setup(args[2], 0)?;
    cs.dout_out = gpio_out_setup(args[2], 0)?;
    gpio_in_reset(cs.dout, 0)?;
    cs.sclk = gpio_out_setup(args[3], 0)?;
    gpio_out_write(cs.sclk, 1);
    Ok(())
}
decl_command!(
    command_config_cs1237,
    concat!("config_cs1237 oid=%c config=%c", " dout_pin=%u sclk_pin=%u")
);

pub fn cs1237_attach_load_cell_probe(args: &[u32]) -> R {
    let cs = lookup(args[0] as u8)?;
    cs.lce = load_cell_probe_oid_lookup(args[1] as u8)?;
    Ok(())
}
decl_command!(
    cs1237_attach_load_cell_probe,
    concat!("cs1237_attach_load_cell_probe", " oid=%c load_cell_probe_oid=%c")
);

pub fn command_query_cs1237(args: &[u32]) -> R {
    let cs = lookup(args[0] as u8)?;
    sched_del_timer(&mut cs.timer);
    cs.flags = 0;
    cs.rest_ticks = args[1];
    if cs.rest_ticks == 0 {
        gpio_out_write(cs.sclk, 1);
        return Ok(());
    }
    gpio_in_reset(cs.dout, 0)?;
    gpio_out_write(cs.sclk, 0);
    if cs.is_configured == 0 {
        cs.flags = CS_CONFIG_PENDING;
    }
    sensor_bulk_reset(&mut cs.sb);
    irq_disable();
    cs.timer.waketime = timer_read_time().wrapping_add(cs.rest_ticks);
    sched_add_timer(&mut cs.timer)?;
    irq_enable();
    Ok(())
}
decl_command!(command_query_cs1237, "query_cs1237 oid=%c rest_ticks=%u");

pub fn command_query_cs1237_status(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let cs = lookup(oid)?;
    irq_disable();
    let start_t = timer_read_time();
    let ready = cs1237_is_data_ready(cs);
    irq_enable();
    sensor_bulk_status(&cs.sb, oid, start_t, 0, if ready { BYTES_PER_SAMPLE as u32 } else { 0 })
}
decl_command!(command_query_cs1237_status, "query_cs1237_status oid=%c");

pub fn cs1237_capture_task() -> R {
    if !sched_check_wake(WAKE_CS1237.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let cs = unsafe { &mut *(p as *mut Cs1237Adc) };
        if cs.flags & CS_PENDING != 0 {
            cs1237_read_adc(cs, oid)?;
        }
    }
    Ok(())
}
decl_task!(cs1237_capture_task);
