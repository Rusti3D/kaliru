// Basic support for common SPI controlled thermocouple chips
//
// Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{irq_disable, irq_enable};
use crate::command::Args;
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::spicmds::{spidev_oid_lookup, spidev_transfer, SpiDev};
use crate::{decl_command, decl_enumeration, decl_task, sendf, shutdown, try_shutdown};
use core::mem::size_of;

const TS_CHIP_MAX31855: u8 = 0;
const TS_CHIP_MAX31856: u8 = 1;
const TS_CHIP_MAX31865: u8 = 2;
const TS_CHIP_MAX6675: u8 = 3;

decl_enumeration!("thermocouple_type", "MAX31855", TS_CHIP_MAX31855);
decl_enumeration!("thermocouple_type", "MAX31856", TS_CHIP_MAX31856);
decl_enumeration!("thermocouple_type", "MAX31865", TS_CHIP_MAX31865);
decl_enumeration!("thermocouple_type", "MAX6675", TS_CHIP_MAX6675);

#[repr(C)]
struct ThermocoupleSpi {
    timer: Timer,
    rest_time: u32,
    min_value: u32, // Min allowed ADC value
    max_value: u32, // Max allowed ADC value
    spi: *mut SpiDev,
    max_invalid: u8,
    invalid_count: u8,
    chip_type: u8,
    flags: u8,
}

const TS_PENDING: u8 = 1;

static THERMOCOUPLE_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_thermocouple as *const ()
}

unsafe fn thermocouple_event(timer: *mut Timer) -> R<u8> {
    let spi = &mut *(timer as *mut ThermocoupleSpi);
    // Trigger task to read and send results
    sched_wake_task(THERMOCOUPLE_WAKE.ptr());
    spi.flags |= TS_PENDING;
    spi.timer.waketime = spi.timer.waketime.wrapping_add(spi.rest_time);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_thermocouple(args: &Args) -> R {
    let chip_type = args[2] as u8;
    if chip_type > TS_CHIP_MAX6675 {
        shutdown!("Invalid thermocouple chip type");
    }
    let spi = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<ThermocoupleSpi>() as u16)? as *mut ThermocoupleSpi) };
    spi.timer.func = Some(thermocouple_event);
    spi.spi = spidev_oid_lookup(args[1] as u8)?;
    spi.chip_type = chip_type;
    Ok(())
}
decl_command!(command_config_thermocouple, "config_thermocouple oid=%c spi_oid=%c thermocouple_type=%c");

pub fn command_query_thermocouple(args: &Args) -> R {
    let spi = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut ThermocoupleSpi) };

    sched_del_timer(&mut spi.timer);
    spi.timer.waketime = args[1];
    spi.rest_time = args[2];
    if spi.rest_time == 0 {
        return Ok(());
    }
    spi.min_value = args[3];
    spi.max_value = args[4];
    spi.max_invalid = args[5] as u8;
    spi.invalid_count = 0;
    sched_add_timer(&mut spi.timer)
}
decl_command!(
    command_query_thermocouple,
    concat!("query_thermocouple oid=%c clock=%u rest_ticks=%u", " min_value=%u max_value=%u max_invalid_count=%c")
);

fn thermocouple_respond(spi: &mut ThermocoupleSpi, next_begin_time: u32, value: u32, fault: u8, oid: u8) -> R {
    sendf!("thermocouple_result oid=%c next_clock=%u value=%u fault=%c", oid, next_begin_time, value, fault)?;
    if spi.max_invalid > 0 {
        /* check the result and stop if below or above allowed range */
        if fault != 0 || value < spi.min_value || value > spi.max_value {
            spi.invalid_count = spi.invalid_count.wrapping_add(1);
            if spi.invalid_count < spi.max_invalid {
                return Ok(());
            }
            try_shutdown!("Thermocouple reader fault");
        }
        spi.invalid_count = 0;
    }
    Ok(())
}

fn thermocouple_handle_max31855(spi: &mut ThermocoupleSpi, next_begin_time: u32, oid: u8) -> R {
    let mut msg = [0u8; 4];
    spidev_transfer(spi.spi, 1, &mut msg)?;
    let value = u32::from_be_bytes(msg);
    thermocouple_respond(spi, next_begin_time, value, (value & 0x07) as u8, oid)
}

const MAX31856_LTCBH_REG: u8 = 0x0C;
const MAX31856_SR_REG: u8 = 0x0F;

fn thermocouple_handle_max31856(spi: &mut ThermocoupleSpi, next_begin_time: u32, oid: u8) -> R {
    let mut msg = [MAX31856_LTCBH_REG, 0x00, 0x00, 0x00];
    spidev_transfer(spi.spi, 1, &mut msg)?;
    let value = u32::from_be_bytes(msg) & 0x00ffffff;
    // Read faults
    msg[0] = MAX31856_SR_REG;
    msg[1] = 0x00;
    spidev_transfer(spi.spi, 1, &mut msg[..2])?;
    thermocouple_respond(spi, next_begin_time, value, msg[1], oid)
}

const MAX31865_RTDMSB_REG: u8 = 0x01;
const MAX31865_FAULTSTAT_REG: u8 = 0x07;

fn thermocouple_handle_max31865(spi: &mut ThermocoupleSpi, next_begin_time: u32, oid: u8) -> R {
    let mut msg = [MAX31865_RTDMSB_REG, 0x00, 0x00, 0x00];
    spidev_transfer(spi.spi, 1, &mut msg[..3])?;
    let value = (u32::from_be_bytes(msg) >> 8) & 0xffff;
    // Read faults
    msg[0] = MAX31865_FAULTSTAT_REG;
    msg[1] = 0x00;
    spidev_transfer(spi.spi, 1, &mut msg[..2])?;
    let fault = (msg[1] & !0x03) | (value & 0x0001) as u8;
    thermocouple_respond(spi, next_begin_time, value, fault, oid)
}

fn thermocouple_handle_max6675(spi: &mut ThermocoupleSpi, next_begin_time: u32, oid: u8) -> R {
    let mut msg = [0u8; 2];
    spidev_transfer(spi.spi, 1, &mut msg)?;
    let value = u16::from_be_bytes(msg);
    thermocouple_respond(spi, next_begin_time, value as u32, (value & 0x06) as u8, oid)
}

// task to read thermocouple and send response
pub fn thermocouple_task() -> R {
    if !sched_check_wake(THERMOCOUPLE_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let spi = unsafe { &mut *(p as *mut ThermocoupleSpi) };
        if spi.flags & TS_PENDING == 0 {
            continue;
        }
        irq_disable();
        let next_begin_time = spi.timer.waketime;
        spi.flags &= !TS_PENDING;
        irq_enable();
        match spi.chip_type {
            TS_CHIP_MAX31855 => thermocouple_handle_max31855(spi, next_begin_time, oid)?,
            TS_CHIP_MAX31856 => thermocouple_handle_max31856(spi, next_begin_time, oid)?,
            TS_CHIP_MAX31865 => thermocouple_handle_max31865(spi, next_begin_time, oid)?,
            TS_CHIP_MAX6675 => thermocouple_handle_max6675(spi, next_begin_time, oid)?,
            _ => {}
        }
    }
    Ok(())
}
decl_task!(thermocouple_task);
