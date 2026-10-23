// Support for bit-banging commands to HX711 and HX717 ADC chips
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{
    gpio_in_read, gpio_in_setup, gpio_out_setup, gpio_out_toggle_noirq, gpio_out_write, irq_disable, irq_enable, irq_poll, timer_from_us,
    timer_is_before, timer_read_time, GpioIn, GpioOut,
};
use crate::config::CONFIG_MACH_AVR;
use crate::ctr::Global;
use crate::load_cell_probe::{load_cell_probe_oid_lookup, load_cell_probe_report_sample, LoadCellProbe};
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, sensor_bulk_status, SensorBulk};
use crate::{decl_command, decl_task, shutdown};
use core::mem::size_of;

#[repr(C)]
struct Hx71xAdc {
    timer: Timer,
    gain_channel: u8, // the gain+channel selection (1-4)
    flags: u8,
    rest_ticks: u32,
    last_error: u32,
    dout: GpioIn,  // pin used to receive data from the hx71x
    sclk: GpioOut, // pin used to generate clock for the hx71x
    sb: SensorBulk,
    lce: *mut LoadCellProbe,
}

const HX_PENDING: u8 = 1 << 0;
const HX_OVERFLOW: u8 = 1 << 1;

const BYTES_PER_SAMPLE: u8 = 4;
const SAMPLE_ERROR_DESYNC: u32 = 1 << 31;
const SAMPLE_ERROR_READ_TOO_LONG: u32 = 1 << 30;

static WAKE_HX71X: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_hx71x as *const ()
}

/****************************************************************
 * Low-level bit-banging
 ****************************************************************/

fn min_pulse_time() -> u32 {
    nsecs_to_ticks(200)
}

fn nsecs_to_ticks(ns: u32) -> u32 {
    timer_from_us(ns.wrapping_mul(1000)) / 1000000
}

// Pause for 200ns
fn hx71x_delay_noirq() {
    if CONFIG_MACH_AVR {
        // Optimize avr, as calculating time takes longer than needed delay
        return;
    }
    let end = timer_read_time().wrapping_add(min_pulse_time());
    while timer_is_before(timer_read_time(), end) {}
}

// Pause for a minimum of 200ns
fn hx71x_delay() -> R {
    if CONFIG_MACH_AVR {
        // Optimize avr, as calculating time takes longer than needed delay
        return Ok(());
    }
    let end = timer_read_time().wrapping_add(min_pulse_time());
    while timer_is_before(timer_read_time(), end) {
        irq_poll()?;
    }
    Ok(())
}

// Read 'num_bits' from the sensor
fn hx71x_raw_read(dout: GpioIn, sclk: GpioOut, num_bits: u32) -> R<u32> {
    let mut bits_read: u32 = 0;
    for _ in 0..num_bits {
        irq_disable();
        gpio_out_toggle_noirq(sclk);
        hx71x_delay_noirq();
        gpio_out_toggle_noirq(sclk);
        let bit = gpio_in_read(dout);
        irq_enable();
        hx71x_delay()?;
        bits_read = (bits_read << 1) | bit as u32;
    }
    Ok(bits_read)
}

/****************************************************************
 * HX711 and HX717 Sensor Support
 ****************************************************************/

// Check if data is ready
fn hx71x_is_data_ready(hx71x: &Hx71xAdc) -> bool {
    gpio_in_read(hx71x.dout) == 0
}

// Event handler that wakes wake_hx71x() periodically
unsafe fn hx71x_event(timer: *mut Timer) -> R<u8> {
    let hx71x = &mut *(timer as *mut Hx71xAdc);
    let mut rest_ticks = hx71x.rest_ticks;
    let flags = hx71x.flags;
    if flags & HX_PENDING != 0 {
        hx71x.sb.possible_overflows = hx71x.sb.possible_overflows.wrapping_add(1);
        hx71x.flags = HX_PENDING | HX_OVERFLOW;
        rest_ticks = rest_ticks.wrapping_mul(4);
    } else if hx71x_is_data_ready(hx71x) {
        // New sample pending
        hx71x.flags = HX_PENDING;
        sched_wake_task(WAKE_HX71X.ptr());
        rest_ticks = rest_ticks.wrapping_mul(8);
    }
    hx71x.timer.waketime = hx71x.timer.waketime.wrapping_add(rest_ticks);
    Ok(SF_RESCHEDULE)
}

fn add_sample(hx71x: &mut Hx71xAdc, oid: u8, counts: u32, force_flush: bool) -> R {
    // Add measurement to buffer
    let dc = hx71x.sb.data_count as usize;
    hx71x.sb.data[dc..dc + 4].copy_from_slice(&counts.to_le_bytes());
    hx71x.sb.data_count += BYTES_PER_SAMPLE;

    if hx71x.sb.data_count as usize + BYTES_PER_SAMPLE as usize > hx71x.sb.data.len() || force_flush {
        sensor_bulk_report(&mut hx71x.sb, oid)?;
    }
    Ok(())
}

// hx71x ADC query
fn hx71x_read_adc(hx71x: &mut Hx71xAdc, oid: u8) -> R {
    // Read from sensor
    let gain_channel = hx71x.gain_channel as u32;
    let adc = hx71x_raw_read(hx71x.dout, hx71x.sclk, 24 + gain_channel)?;

    // Clear pending flag (and note if an overflow occurred)
    irq_disable();
    let flags = hx71x.flags;
    hx71x.flags = 0;
    irq_enable();

    // Extract report from raw data
    let mut counts = adc >> gain_channel;
    if counts & 0x800000 != 0 {
        counts |= 0xFF000000;
    }

    // Check for errors
    let extras_mask = (1u32 << gain_channel) - 1;
    if (adc & extras_mask) != extras_mask {
        // Transfer did not complete correctly
        hx71x.last_error = SAMPLE_ERROR_DESYNC;
    } else if flags & HX_OVERFLOW != 0 {
        // Transfer took too long
        hx71x.last_error = SAMPLE_ERROR_READ_TOO_LONG;
    }

    // forever send errors until reset
    if hx71x.last_error != 0 {
        counts = hx71x.last_error;
    }

    // probe is optional, report if enabled
    if hx71x.last_error == 0 && !hx71x.lce.is_null() {
        load_cell_probe_report_sample(hx71x.lce, counts as i32)?;
    }

    // Add measurement to buffer
    add_sample(hx71x, oid, counts, false)
}

// Create a hx71x sensor
pub fn command_config_hx71x(args: &[u32]) -> R {
    let hx71x = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Hx71xAdc>() as u16)? as *mut Hx71xAdc) };
    hx71x.timer.func = Some(hx71x_event);
    let gain_channel = args[1] as u8;
    if !(1..=4).contains(&gain_channel) {
        shutdown!("HX71x gain/channel out of range 1-4");
    }
    hx71x.gain_channel = gain_channel;
    hx71x.dout = gpio_in_setup(args[2], 1)?;
    hx71x.sclk = gpio_out_setup(args[3], 0)?;
    gpio_out_write(hx71x.sclk, 1); // put chip in power down state
    Ok(())
}
decl_command!(
    command_config_hx71x,
    concat!("config_hx71x oid=%c gain_channel=%c", " dout_pin=%u sclk_pin=%u")
);

fn lookup(oid: u8) -> R<&'static mut Hx71xAdc> {
    Ok(unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut Hx71xAdc) })
}

pub fn hx71x_attach_load_cell_probe(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let hx71x = lookup(oid)?;
    hx71x.lce = load_cell_probe_oid_lookup(args[1] as u8)?;
    Ok(())
}
decl_command!(
    hx71x_attach_load_cell_probe,
    concat!("hx71x_attach_load_cell_probe oid=%c", " load_cell_probe_oid=%c")
);

// start/stop capturing ADC data
pub fn command_query_hx71x(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let hx71x = lookup(oid)?;
    sched_del_timer(&mut hx71x.timer);
    hx71x.flags = 0;
    hx71x.last_error = 0;
    hx71x.rest_ticks = args[1];
    if hx71x.rest_ticks == 0 {
        // End measurements
        gpio_out_write(hx71x.sclk, 1); // put chip in power down state
        return Ok(());
    }
    // Start new measurements
    gpio_out_write(hx71x.sclk, 0); // wake chip from power down
    sensor_bulk_reset(&mut hx71x.sb);
    irq_disable();
    hx71x.timer.waketime = timer_read_time().wrapping_add(hx71x.rest_ticks);
    sched_add_timer(&mut hx71x.timer)?;
    irq_enable();
    Ok(())
}
decl_command!(command_query_hx71x, "query_hx71x oid=%c rest_ticks=%u");

pub fn command_query_hx71x_status(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let hx71x = lookup(oid)?;
    irq_disable();
    let start_t = timer_read_time();
    let is_data_ready = hx71x_is_data_ready(hx71x);
    irq_enable();
    let pending_bytes = if is_data_ready { BYTES_PER_SAMPLE } else { 0 };
    sensor_bulk_status(&hx71x.sb, oid, start_t, 0, pending_bytes as u32)
}
decl_command!(command_query_hx71x_status, "query_hx71x_status oid=%c");

// Background task that performs measurements
pub fn hx71x_capture_task() -> R {
    if !sched_check_wake(WAKE_HX71X.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let hx71x = unsafe { &mut *(p as *mut Hx71xAdc) };
        if hx71x.flags != 0 {
            hx71x_read_adc(hx71x, oid)?;
        }
    }
    Ok(())
}
decl_task!(hx71x_capture_task);
