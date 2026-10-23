// Support for ADS1220 ADC Chip
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_in_read, gpio_in_setup, irq_disable, irq_enable, timer_read_time, GpioIn};
use crate::ctr::Global;
use crate::load_cell_probe::{load_cell_probe_oid_lookup, load_cell_probe_report_sample, LoadCellProbe};
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, sensor_bulk_status, SensorBulk};
use crate::spicmds::{spidev_oid_lookup, spidev_transfer, SpiDev};
use crate::{decl_command, decl_task};
use core::mem::size_of;
use core::sync::atomic::{compiler_fence, Ordering};

#[repr(C)]
struct Ads1220Adc {
    timer: Timer,
    rest_ticks: u32,
    data_ready: GpioIn,
    spi: *mut SpiDev,
    pending_flag: u8,
    sb: SensorBulk,
    lce: *mut LoadCellProbe,
}

const BYTES_PER_SAMPLE: u8 = 4;

static WAKE_ADS1220: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_ads1220 as *const ()
}

fn lookup(oid: u8) -> R<&'static mut Ads1220Adc> {
    Ok(unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut Ads1220Adc) })
}

/****************************************************************
 * ADS1220 Sensor Support
 ****************************************************************/

fn ads1220_is_data_ready(ads1220: &Ads1220Adc) -> bool {
    gpio_in_read(ads1220.data_ready) == 0
}

// Event handler that wakes wake_ads1220() periodically
unsafe fn ads1220_event(timer: *mut Timer) -> R<u8> {
    let ads1220 = &mut *(timer as *mut Ads1220Adc);
    let mut rest_ticks = ads1220.rest_ticks;
    if ads1220.pending_flag != 0 {
        ads1220.sb.possible_overflows = ads1220.sb.possible_overflows.wrapping_add(1);
        rest_ticks = rest_ticks.wrapping_mul(4);
    } else if ads1220_is_data_ready(ads1220) {
        ads1220.pending_flag = 1;
        sched_wake_task(WAKE_ADS1220.ptr());
        rest_ticks = rest_ticks.wrapping_mul(8);
    }
    ads1220.timer.waketime = ads1220.timer.waketime.wrapping_add(rest_ticks);
    Ok(SF_RESCHEDULE)
}

// Add a measurement to the buffer
fn add_sample(ads1220: &mut Ads1220Adc, oid: u8, counts: u32) -> R {
    let dc = ads1220.sb.data_count as usize;
    ads1220.sb.data[dc..dc + 4].copy_from_slice(&counts.to_le_bytes());
    ads1220.sb.data_count += BYTES_PER_SAMPLE;

    if ads1220.sb.data_count as usize + BYTES_PER_SAMPLE as usize > ads1220.sb.data.len() {
        sensor_bulk_report(&mut ads1220.sb, oid)?;
    }
    Ok(())
}

// ADS1220 ADC query
fn ads1220_read_adc(ads1220: &mut Ads1220Adc, oid: u8) -> R {
    let mut msg = [0u8; 3];
    spidev_transfer(ads1220.spi, 1, &mut msg)?;
    ads1220.pending_flag = 0;
    compiler_fence(Ordering::SeqCst);

    // create 24 bit int from bytes
    let mut counts = ((msg[0] as u32) << 16) | ((msg[1] as u32) << 8) | msg[2] as u32;

    // extend 2's complement 24 bits to 32bits
    if counts & 0x800000 != 0 {
        counts |= 0xFF000000;
    }

    // endstop is optional, report if enabled and no errors
    if !ads1220.lce.is_null() {
        load_cell_probe_report_sample(ads1220.lce, counts as i32)?;
    }

    add_sample(ads1220, oid, counts)
}

// Create an ads1220 sensor
pub fn command_config_ads1220(args: &[u32]) -> R {
    let ads1220 = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Ads1220Adc>() as u16)? as *mut Ads1220Adc) };
    ads1220.timer.func = Some(ads1220_event);
    ads1220.pending_flag = 0;
    ads1220.spi = spidev_oid_lookup(args[1] as u8)?;
    ads1220.data_ready = gpio_in_setup(args[2], 0)?;
    Ok(())
}
decl_command!(
    command_config_ads1220,
    concat!("config_ads1220 oid=%c", " spi_oid=%c data_ready_pin=%u")
);

pub fn ads1220_attach_load_cell_probe(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let ads1220 = lookup(oid)?;
    ads1220.lce = load_cell_probe_oid_lookup(args[1] as u8)?;
    Ok(())
}
decl_command!(ads1220_attach_load_cell_probe, "ads1220_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");

// start/stop capturing ADC data
pub fn command_query_ads1220(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let ads1220 = lookup(oid)?;
    sched_del_timer(&mut ads1220.timer);
    ads1220.pending_flag = 0;
    ads1220.rest_ticks = args[1];
    if ads1220.rest_ticks == 0 {
        // End measurements
        return Ok(());
    }
    // Start new measurements
    sensor_bulk_reset(&mut ads1220.sb);
    irq_disable();
    ads1220.timer.waketime = timer_read_time().wrapping_add(ads1220.rest_ticks);
    sched_add_timer(&mut ads1220.timer)?;
    irq_enable();
    Ok(())
}
decl_command!(command_query_ads1220, "query_ads1220 oid=%c rest_ticks=%u");

pub fn command_query_ads1220_status(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let ads1220 = lookup(oid)?;
    irq_disable();
    let start_t = timer_read_time();
    let is_data_ready = ads1220_is_data_ready(ads1220);
    irq_enable();
    let pending_bytes = if is_data_ready { BYTES_PER_SAMPLE } else { 0 };
    sensor_bulk_status(&ads1220.sb, oid, start_t, 0, pending_bytes as u32)
}
decl_command!(command_query_ads1220_status, "query_ads1220_status oid=%c");

// Background task that performs measurements
pub fn ads1220_capture_task() -> R {
    if !sched_check_wake(WAKE_ADS1220.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let ads1220 = unsafe { &mut *(p as *mut Ads1220Adc) };
        if ads1220.pending_flag != 0 {
            ads1220_read_adc(ads1220, oid)?;
        }
    }
    Ok(())
}
decl_task!(ads1220_capture_task);
