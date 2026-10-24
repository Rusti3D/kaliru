// Support for ADS131M02 and ADS131M04 ADC chips
//
// Copyright (C) 2025 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_in_read, gpio_in_setup, irq_disable, irq_enable, timer_read_time, GpioIn};
use crate::command::Args;
use crate::ctr::Global;
use crate::load_cell_probe::{load_cell_probe_oid_lookup, load_cell_probe_report_sample, LoadCellProbe};
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, sensor_bulk_status, SensorBulk};
use crate::spicmds::{spidev_oid_lookup, spidev_transfer, SpiDev};
use crate::{decl_command, decl_task, shutdown};
use core::mem::size_of;
use core::sync::atomic::{compiler_fence, Ordering};

#[repr(C)]
struct Ads131m0xAdc {
    timer: Timer,
    rest_ticks: u32,
    hard_error_latch: u32,
    data_ready: GpioIn,
    spi: *mut SpiDev,
    pending_flag: u8,
    sensor_channel_count: u8,
    channel_mask: u8,
    sampled_channels: u8,
    sample_bytes: u8,
    frame_size: u8,
    sb: SensorBulk,
    lce: *mut LoadCellProbe,
}

// Error codes sent as sample values (use high bits to distinguish from valid data)
const SAMPLE_ERROR_CRC: u32 = 1 << 31;
const SAMPLE_ERROR_RESET: u32 = 1 << 30;
const BYTES_PER_SAMPLE: u8 = 4;
const SENSOR_WORD_SIZE: u8 = 3;
const MAX_ADC_CHANNELS: u8 = 4;
const MAX_FRAME_SIZE: usize = (2 + MAX_ADC_CHANNELS as usize) * SENSOR_WORD_SIZE as usize;
// Bit 10 of 16-bit status, in byte 0 (bits 15:8)
const STATUS_RESET_BIT: u8 = 1 << 2;

static WAKE_ADS131M0X: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_ads131m0x as *const ()
}

fn lookup(oid: u8) -> R<&'static mut Ads131m0xAdc> {
    Ok(unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut Ads131m0xAdc) })
}

// Add a measurement to the buffer
#[inline]
fn buffer_append_int32(sb: &mut SensorBulk, val: i32) {
    let dc = sb.data_count as usize;
    sb.data[dc..dc + 4].copy_from_slice(&val.to_le_bytes());
    sb.data_count += BYTES_PER_SAMPLE;
}

/****************************************************************
 * ads131m0x sensor support
 ****************************************************************/

fn calc_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        let mut x = ((crc >> 8) as u8) ^ b;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((x as u16) << 12) ^ ((x as u16) << 5) ^ x as u16;
    }
    crc
}

#[inline]
fn has_crc_error(adc: &Ads131m0xAdc, msg: &[u8]) -> bool {
    let crc_offset = (adc.frame_size - SENSOR_WORD_SIZE) as usize;
    let calc_crc = calc_crc16(&msg[..crc_offset]);
    let recv_crc = ((msg[crc_offset] as u16) << 8) | msg[crc_offset + 1] as u16;
    calc_crc != recv_crc
}

#[inline]
fn is_data_ready(adc: &Ads131m0xAdc) -> bool {
    gpio_in_read(adc.data_ready) == 0
}

#[inline]
fn extract_counts(msg: &[u8], index: u8) -> i32 {
    let offset = (SENSOR_WORD_SIZE + SENSOR_WORD_SIZE * index) as usize;
    let mut counts = ((msg[offset] as u32) << 16) | ((msg[offset + 1] as u32) << 8) | msg[offset + 2] as u32;
    if counts & 0x800000 != 0 {
        counts |= 0xFF000000;
    }
    counts as i32
}

#[inline]
fn ads131m0x_flush(adc: &mut Ads131m0xAdc, oid: u8) -> R {
    if adc.sb.data_count as usize + adc.sample_bytes as usize > adc.sb.data.len() {
        sensor_bulk_report(&mut adc.sb, oid)?;
    }
    Ok(())
}

fn publish_samples(adc: &mut Ads131m0xAdc, oid: u8, msg: &[u8]) -> R {
    let mut sum: i32 = 0;
    for i in 0..adc.sensor_channel_count {
        // skip channels that are not enabled
        if adc.channel_mask & (1 << i) == 0 {
            continue;
        }
        let counts = extract_counts(msg, i);
        sum = sum.wrapping_add(counts);
        buffer_append_int32(&mut adc.sb, counts);
    }
    if !adc.lce.is_null() {
        load_cell_probe_report_sample(adc.lce, sum)?;
    }
    ads131m0x_flush(adc, oid)
}

fn publish_error(adc: &mut Ads131m0xAdc, oid: u8, sample_error: i32) -> R {
    for _ in 0..adc.sampled_channels {
        buffer_append_int32(&mut adc.sb, sample_error);
    }
    ads131m0x_flush(adc, oid)
}

// Event handler that wakes wake_ads131m0x() periodically
unsafe fn ads131m0x_event(timer: *mut Timer) -> R<u8> {
    let adc = &mut *(timer as *mut Ads131m0xAdc);
    let mut rest_ticks = adc.rest_ticks;
    if adc.pending_flag != 0 {
        adc.sb.possible_overflows = adc.sb.possible_overflows.wrapping_add(1);
        rest_ticks = rest_ticks.wrapping_mul(4);
    } else if is_data_ready(adc) {
        adc.pending_flag = 1;
        sched_wake_task(WAKE_ADS131M0X.ptr());
        rest_ticks = rest_ticks.wrapping_mul(8);
    }
    adc.timer.waketime = adc.timer.waketime.wrapping_add(rest_ticks);
    Ok(SF_RESCHEDULE)
}

fn ads131m0x_read_adc(adc: &mut Ads131m0xAdc, oid: u8) -> R {
    // Typical communication frame at 24-bit word length:
    // 3-byte STATUS + 3-byte CH0 + 3-byte CH1 + 3-byte CRC.
    let mut msg = [0u8; MAX_FRAME_SIZE];
    spidev_transfer(adc.spi, 1, &mut msg[..adc.frame_size as usize])?;
    adc.pending_flag = 0;
    compiler_fence(Ordering::SeqCst);

    // Check for unexpected device reset (RESET bit in status byte 0)
    // This is a hard error - once set, keep sending until measurement restart
    if msg[0] & STATUS_RESET_BIT != 0 {
        adc.hard_error_latch = SAMPLE_ERROR_RESET;
    }
    if adc.hard_error_latch != 0 {
        let e = adc.hard_error_latch as i32;
        publish_error(adc, oid, e)
    } else if has_crc_error(adc, &msg) {
        publish_error(adc, oid, SAMPLE_ERROR_CRC as i32)
    } else {
        publish_samples(adc, oid, &msg)
    }
}

pub fn command_config_ads131m0x(args: &Args) -> R {
    let adc = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Ads131m0xAdc>() as u16)? as *mut Ads131m0xAdc) };
    adc.timer.func = Some(ads131m0x_event);
    adc.pending_flag = 0;
    adc.spi = spidev_oid_lookup(args[1] as u8)?;
    adc.data_ready = gpio_in_setup(args[2], 0)?;
    adc.sensor_channel_count = args[3] as u8;
    adc.channel_mask = args[4] as u8;
    if adc.sensor_channel_count == 0 || adc.sensor_channel_count > MAX_ADC_CHANNELS {
        shutdown!("ads131m0x invalid sensor_channel_count");
    }
    // check that the maximum bit set in the mask is valid
    if adc.channel_mask == 0 || (adc.channel_mask as u32) >> adc.sensor_channel_count != 0 {
        shutdown!("ads131m0x invalid channel_mask");
    }
    // count number of bits set in the mask
    let mut mask_bits = adc.channel_mask;
    adc.sampled_channels = 0;
    while mask_bits != 0 {
        mask_bits &= mask_bits - 1; // clear the least significant bit set
        adc.sampled_channels += 1;
    }
    adc.sample_bytes = BYTES_PER_SAMPLE * adc.sampled_channels;
    adc.frame_size = (2 + adc.sensor_channel_count) * SENSOR_WORD_SIZE;
    Ok(())
}
decl_command!(
    command_config_ads131m0x,
    concat!("config_ads131m0x oid=%c spi_oid=%c data_ready_pin=%u", " sensor_channel_count=%c channel_mask=%c")
);

pub fn ads131m0x_attach_load_cell_probe(args: &Args) -> R {
    let oid = args[0] as u8;
    let adc = lookup(oid)?;
    adc.lce = load_cell_probe_oid_lookup(args[1] as u8)?;
    Ok(())
}
decl_command!(ads131m0x_attach_load_cell_probe, "ads131m0x_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");

pub fn command_query_ads131m0x(args: &Args) -> R {
    let oid = args[0] as u8;
    let adc = lookup(oid)?;
    sched_del_timer(&mut adc.timer);
    adc.pending_flag = 0;
    adc.rest_ticks = args[1];
    if adc.rest_ticks == 0 {
        return Ok(());
    }
    adc.hard_error_latch = 0;
    sensor_bulk_reset(&mut adc.sb);
    irq_disable();
    adc.timer.waketime = timer_read_time().wrapping_add(adc.rest_ticks);
    sched_add_timer(&mut adc.timer)?;
    irq_enable();
    Ok(())
}
decl_command!(command_query_ads131m0x, "query_ads131m0x oid=%c rest_ticks=%u");

pub fn command_query_ads131m0x_status(args: &Args) -> R {
    let oid = args[0] as u8;
    let adc = lookup(oid)?;
    irq_disable();
    let start_t = timer_read_time();
    let is_ready = is_data_ready(adc);
    irq_enable();
    let pending_bytes = if is_ready { adc.sample_bytes } else { 0 };
    sensor_bulk_status(&adc.sb, oid, start_t, 0, pending_bytes as u32)
}
decl_command!(command_query_ads131m0x_status, "query_ads131m0x_status oid=%c");

// Background task that performs measurements
pub fn ads131m0x_capture_task() -> R {
    if !sched_check_wake(WAKE_ADS131M0X.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let adc = unsafe { &mut *(p as *mut Ads131m0xAdc) };
        if adc.pending_flag != 0 {
            ads131m0x_read_adc(adc, oid)?;
        }
    }
    Ok(())
}
decl_task!(ads131m0x_capture_task);
