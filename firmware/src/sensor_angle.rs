// Support for querying magnetic angle sensors via SPI
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, oid_next, OidType};
use crate::board::{gpio_out_write, irq_disable, irq_enable, irq_poll, timer_from_us, timer_is_before, timer_read_time};
use crate::command::{command_decode_buffer, Args};
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, SensorBulk};
use crate::spicmds::{spidev_get_cs_pin, spidev_have_cs_pin, spidev_oid_lookup, spidev_transfer, SpiDev};
use crate::{decl_command, decl_enumeration, decl_task, sendf, shutdown};
use core::mem::size_of;

const SA_CHIP_A1333: u8 = 0;
const SA_CHIP_AS5047D: u8 = 1;
const SA_CHIP_TLE5012B: u8 = 2;
const SA_CHIP_MT6816: u8 = 3;
const SA_CHIP_MT6826S: u8 = 4;
const SA_CHIP_MAX: u8 = 5;

decl_enumeration!("spi_angle_type", "a1333", SA_CHIP_A1333);
decl_enumeration!("spi_angle_type", "as5047d", SA_CHIP_AS5047D);
decl_enumeration!("spi_angle_type", "tle5012b", SA_CHIP_TLE5012B);
decl_enumeration!("spi_angle_type", "mt6816", SA_CHIP_MT6816);
decl_enumeration!("spi_angle_type", "mt6826s", SA_CHIP_MT6826S);

const TCODE_ERROR: u32 = 0xff;
const SE_OVERFLOW: u32 = 0;
const SE_SCHEDULE: u32 = 1;
const SE_SPI_TIME: u32 = 2;
const SE_CRC: u32 = 3;
const SE_DUP: u32 = 4;
const SE_NO_ANGLE: u32 = 5;

fn max_spi_read_time() -> u32 {
    timer_from_us(50)
}

#[repr(C)]
struct SpiAngle {
    timer: Timer,
    rest_ticks: u32,
    spi: *mut SpiDev,
    flags: u8,
    chip_type: u8,
    time_shift: u8,
    overflow: u8,
    sb: SensorBulk,
}

const SA_PENDING: u8 = 1 << 2;

const BYTES_PER_SAMPLE: usize = 3;

static ANGLE_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_spi_angle as *const ()
}

// Event handler that wakes spi_angle_task() periodically
unsafe fn angle_event(timer: *mut Timer) -> R<u8> {
    let sa = &mut *(timer as *mut SpiAngle);
    let flags = sa.flags;
    if sa.flags & SA_PENDING != 0 {
        sa.overflow = sa.overflow.wrapping_add(1);
    } else {
        sa.flags = flags | SA_PENDING;
    }
    sched_wake_task(ANGLE_WAKE.ptr());
    sa.timer.waketime = sa.timer.waketime.wrapping_add(sa.rest_ticks);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_spi_angle(args: &Args) -> R {
    let chip_type = args[2] as u8;
    if chip_type > SA_CHIP_MAX {
        shutdown!("Invalid spi_angle chip type");
    }
    let sa = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<SpiAngle>() as u16)? as *mut SpiAngle) };
    sa.timer.func = Some(angle_event);
    sa.spi = spidev_oid_lookup(args[1] as u8)?;
    if !spidev_have_cs_pin(sa.spi) {
        shutdown!("angle sensor requires cs pin");
    }
    sa.chip_type = chip_type;
    Ok(())
}
decl_command!(command_config_spi_angle, "config_spi_angle oid=%c spi_oid=%c spi_angle_type=%c");

// Send spi_angle_data message if buffer is full
fn angle_check_report(sa: &mut SpiAngle, oid: u8) -> R {
    if sa.sb.data_count as usize + BYTES_PER_SAMPLE > sa.sb.data.len() {
        sensor_bulk_report(&mut sa.sb, oid)?;
    }
    Ok(())
}

// Add an entry to the measurement buffer
fn angle_add(sa: &mut SpiAngle, tcode: u32, data: u32) {
    let dc = sa.sb.data_count as usize;
    sa.sb.data[dc] = tcode as u8;
    sa.sb.data[dc + 1] = data as u8;
    sa.sb.data[dc + 2] = (data >> 8) as u8;
    sa.sb.data_count += BYTES_PER_SAMPLE as u8;
}

// Add an error indicator to the measurement buffer
fn angle_add_error(sa: &mut SpiAngle, error_code: u32) {
    angle_add(sa, TCODE_ERROR, error_code);
}

// Add a measurement to the buffer
fn angle_add_data(sa: &mut SpiAngle, stime: u32, mtime: u32, angle: u32) {
    let mut tdiff = mtime.wrapping_sub(stime);
    if sa.time_shift != 0 {
        tdiff = tdiff.wrapping_add(1 << (sa.time_shift - 1)) >> sa.time_shift;
    }
    if tdiff >= TCODE_ERROR {
        angle_add_error(sa, SE_SCHEDULE);
        return;
    }
    angle_add(sa, tdiff, angle);
}

// a1333 sensor query
fn a1333_query(sa: &mut SpiAngle, stime: u32) -> R {
    let mut msg = [0x32u8, 0x00];
    let mtime1 = timer_read_time();
    spidev_transfer(sa.spi, 1, &mut msg)?;
    let mtime2 = timer_read_time();
    // Data is latched on first sclk edge of response
    if mtime2.wrapping_sub(mtime1) > max_spi_read_time() {
        angle_add_error(sa, SE_SPI_TIME);
    } else if msg[0] & 0x80 != 0 {
        angle_add_error(sa, SE_CRC);
    } else {
        angle_add_data(sa, stime, mtime1, ((msg[0] as u32) << 9) | ((msg[1] as u32) << 1));
    }
    Ok(())
}

fn bit_parity(msg: &[u8]) -> u32 {
    let mut parity = (msg[0] ^ msg[1]) as u32;
    parity ^= parity >> 4;
    parity ^= parity >> 2;
    parity ^= parity >> 1;
    parity
}

// as5047d sensor query
fn as5047d_query(sa: &mut SpiAngle, stime: u32) -> R {
    let mut msg = [0x7Fu8, 0xFE];
    let mtime1 = timer_read_time();
    spidev_transfer(sa.spi, 0, &mut msg)?;
    let mtime2 = timer_read_time();
    // Data is latched on CS pin rising after query request
    if mtime2.wrapping_sub(mtime1) > max_spi_read_time() {
        angle_add_error(sa, SE_SPI_TIME);
        return Ok(());
    }
    msg[0] = 0xC0;
    msg[1] = 0x00;
    spidev_transfer(sa.spi, 1, &mut msg)?;
    let parity = bit_parity(&msg);
    if parity & 1 != 0 {
        angle_add_error(sa, SE_CRC);
    } else if msg[0] & 0x40 != 0 {
        angle_add_error(sa, SE_NO_ANGLE);
    } else {
        angle_add_data(sa, stime, mtime2, ((msg[0] as u32) << 10) | ((msg[1] as u32) << 2));
    }
    Ok(())
}

fn mt6816_query(sa: &mut SpiAngle, stime: u32) -> R {
    let mut msg = [0x83u8, 0x00, 0x00];
    let mtime1 = timer_read_time();
    spidev_transfer(sa.spi, 1, &mut msg)?;
    let mtime2 = timer_read_time();
    // Data is latched on first sclk edge of response
    if mtime2.wrapping_sub(mtime1) > max_spi_read_time() {
        angle_add_error(sa, SE_SPI_TIME);
        return Ok(());
    }
    let parity = bit_parity(&msg[1..]);
    if parity & 1 != 0 {
        angle_add_error(sa, SE_CRC);
    } else if msg[2] & 0x02 != 0 {
        angle_add_error(sa, SE_NO_ANGLE);
    } else {
        angle_add_data(sa, stime, mtime2, ((msg[1] as u32) << 8) | (msg[2] & 0xfc) as u32);
    }
    Ok(())
}

fn crc8_mt(mut crc: u8, data: u8) -> u8 {
    crc ^= data;
    for _ in 0..8 {
        crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
    }
    crc
}

fn mt6826s_query(sa: &mut SpiAngle, stime: u32) -> R {
    let mut msg = [0x30u8, 0x03, 0x00, 0x00, 0x00, 0x00];
    let mtime1 = timer_read_time();
    spidev_transfer(sa.spi, 1, &mut msg)?;
    let mtime2 = timer_read_time();
    // Data is latched on first sclk edge of response
    if mtime2.wrapping_sub(mtime1) > max_spi_read_time() {
        angle_add_error(sa, SE_SPI_TIME);
        return Ok(());
    }
    let mut crc = 0;
    for &b in &msg[2..5] {
        crc = crc8_mt(crc, b);
    }

    if crc != msg[5] {
        angle_add_error(sa, SE_CRC);
    } else if msg[4] & 0x02 != 0 {
        angle_add_error(sa, SE_NO_ANGLE);
    } else {
        angle_add_data(sa, stime, mtime2, ((msg[2] as u32) << 8) | msg[3] as u32);
    }
    Ok(())
}

const TLE_READ: u8 = 0x80;
const TLE_READ_LATCH: u8 = TLE_READ | 0x04;
const TLE_REG_AVAL: u8 = 0x02;

// crc8 "J1850" calculation for tle5012b messages
fn crc8(mut crc: u8, data: u8) -> u8 {
    crc ^= data;
    for _ in 0..8 {
        crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x1d } else { crc << 1 };
    }
    crc
}

// microsecond delay helper
fn udelay(usecs: u32) -> R {
    let end = timer_read_time().wrapping_add(timer_from_us(usecs));
    while !timer_is_before(end, timer_read_time()) {
        irq_poll()?;
    }
    Ok(())
}

// Latch data (data is latched on rising CS of a NULL message); the time
fn tle5012b_latch(sa: &SpiAngle) -> R<u32> {
    let cs_pin = spidev_get_cs_pin(sa.spi);
    gpio_out_write(cs_pin, 0);
    udelay(1)?;
    irq_disable();
    gpio_out_write(cs_pin, 1);
    let mtime = timer_read_time();
    irq_enable();
    Ok(mtime)
}

// tle5012b sensor query
fn tle5012b_query(sa: &mut SpiAngle, stime: u32) -> R {
    let mtime = tle5012b_latch(sa)?;

    let mut msg = [0u8; 10];
    msg[0] = TLE_READ_LATCH;
    msg[1] = (TLE_REG_AVAL << 4) | 0x03;
    let mut crc: u8 = 0x05; // 0x05 == crc8(crc8(0xff, msg[0]), msg[1])
    spidev_transfer(sa.spi, 1, &mut msg)?;
    for &b in &msg[2..8] {
        crc = crc8(crc, b);
    }
    if !crc != msg[9] {
        angle_add_error(sa, SE_CRC);
    } else if msg[8] & (1 << 4) == 0 {
        angle_add_error(sa, SE_NO_ANGLE);
    } else if msg[2] & 0x80 == 0 {
        angle_add_error(sa, SE_DUP);
    } else if mtime.wrapping_sub(stime) > timer_from_us(32 * 32 * 1000000 / 750000) {
        angle_add_error(sa, SE_SCHEDULE);
    } else {
        angle_add(sa, ((msg[6] >> 1) & 0x3f) as u32, ((msg[2] as u32) << 9) | ((msg[3] as u32) << 1));
    }
    Ok(())
}

pub fn command_query_spi_angle(args: &Args) -> R {
    let sa = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut SpiAngle) };

    sched_del_timer(&mut sa.timer);
    sa.flags = 0;
    if args[2] == 0 {
        // End measurements
        return Ok(());
    }

    // Start new measurements query
    sa.timer.waketime = args[1];
    sa.rest_ticks = args[2];
    sensor_bulk_reset(&mut sa.sb);
    sa.time_shift = args[3] as u8;
    sched_add_timer(&mut sa.timer)
}
decl_command!(command_query_spi_angle, "query_spi_angle oid=%c clock=%u rest_ticks=%u time_shift=%c");

pub fn command_spi_angle_transfer(args: &Args) -> R {
    let oid = args[0] as u8;
    let sa = unsafe { &mut *(oid_lookup(oid, oid_type())? as *mut SpiAngle) };
    let data = command_decode_buffer(args[1] as u8 as u32, args[2]);
    let mut data_len = data.len();
    let mtime;
    let chip = sa.chip_type;
    if chip == SA_CHIP_TLE5012B {
        mtime = tle5012b_latch(sa)?;
        spidev_transfer(sa.spi, 1, data)?;
    } else {
        let mtime1 = timer_read_time();
        spidev_transfer(sa.spi, 1, data)?;
        let mtime2 = timer_read_time();
        if mtime2.wrapping_sub(mtime1) > max_spi_read_time() {
            data_len = 0;
        }
        mtime = if chip == SA_CHIP_AS5047D { mtime2 } else { mtime1 };
    }
    sendf!("spi_angle_transfer_response oid=%c clock=%u response=%*s", oid, mtime, &data[..data_len])
}
decl_command!(command_spi_angle_transfer, "spi_angle_transfer oid=%c data=%*s");

// Background task that performs measurements
pub fn spi_angle_task() -> R {
    if !sched_check_wake(ANGLE_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let sa = unsafe { &mut *(p as *mut SpiAngle) };
        let flags = sa.flags;
        if flags & SA_PENDING == 0 {
            continue;
        }
        irq_disable();
        let mut stime = sa.timer.waketime;
        let mut overflow = sa.overflow;
        sa.flags = 0;
        sa.overflow = 0;
        irq_enable();
        stime = stime.wrapping_sub(sa.rest_ticks);
        while overflow > 0 {
            overflow -= 1;
            angle_add_error(sa, SE_OVERFLOW);
            angle_check_report(sa, oid)?;
        }
        let chip = sa.chip_type;
        if chip == SA_CHIP_A1333 {
            a1333_query(sa, stime)?;
        } else if chip == SA_CHIP_AS5047D {
            as5047d_query(sa, stime)?;
        } else if chip == SA_CHIP_TLE5012B {
            tle5012b_query(sa, stime)?;
        } else if chip == SA_CHIP_MT6816 {
            mt6816_query(sa, stime)?;
        } else if chip == SA_CHIP_MT6826S {
            mt6826s_query(sa, stime)?;
        }
        angle_check_report(sa, oid)?;
    }
    Ok(())
}
decl_task!(spi_angle_task);
