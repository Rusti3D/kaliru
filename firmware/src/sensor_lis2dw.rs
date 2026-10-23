// Support for gathering acceleration data from LIS2DW chip
//
// Copyright (C) 2023  Zhou.XianMing <zhouxm@biqu3d.com>
// Copyright (C) 2020-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{irq_disable, irq_enable, timer_read_time};
use crate::config::{CONFIG_WANT_I2C, CONFIG_WANT_SPI};
use crate::ctr::Global;
use crate::i2ccmds::{i2c_dev_read, i2c_shutdown_on_err, i2cdev_oid_lookup, I2cDev};
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_DONE};
use crate::sensor_bulk::{sensor_bulk_report, sensor_bulk_reset, sensor_bulk_status, SensorBulk};
use crate::spicmds::{spidev_oid_lookup, spidev_transfer, SpiDev};
use crate::{decl_command, decl_enumeration, decl_task, shutdown};
use core::mem::size_of;

const LIS_AR_DATAX0: u8 = 0x28;
const LIS_AM_READ: u8 = 0x80;
const LIS_MS_SPI: u8 = 0x40;
const LIS_MS_I2C: u8 = 0x80;
const LIS_FIFO_SAMPLES: u8 = 0x2F;

const BYTES_PER_SAMPLE: u8 = 6;
const BYTES_PER_BLOCK: u8 = 48;

#[repr(C)]
#[derive(Clone, Copy)]
union Bus {
    spi: *mut SpiDev,
    i2c: *mut I2cDev,
}

#[repr(C)]
struct Lis2dw {
    timer: Timer,
    rest_ticks: u32,
    bus: Bus,
    bus_type: u8,
    flags: u8,
    model: u8,
    fifo_bytes_pending: u8,
    sb: SensorBulk,
}

const LIS_PENDING: u8 = 1 << 0;

const SPI_SERIAL: u32 = 0;
const I2C_SERIAL: u32 = 1;

decl_enumeration!("bus_oid_type", "spi", SPI_SERIAL);
decl_enumeration!("bus_oid_type", "i2c", I2C_SERIAL);

const LIS2DW: u32 = 0;
const LIS3DH: u32 = 1;

decl_enumeration!("lis_chip_type", "LIS2DW", LIS2DW);
decl_enumeration!("lis_chip_type", "LIS3DH", LIS3DH);

static LIS2DW_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn oid_type() -> OidType {
    command_config_lis2dw as *const ()
}

// Event handler that wakes lis2dw_task() periodically
unsafe fn lis2dw_event(timer: *mut Timer) -> R<u8> {
    let ax = &mut *(timer as *mut Lis2dw);
    ax.flags |= LIS_PENDING;
    sched_wake_task(LIS2DW_WAKE.ptr());
    Ok(SF_DONE)
}

pub fn command_config_lis2dw(args: &[u32]) -> R {
    let ax = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<Lis2dw>() as u16)? as *mut Lis2dw) };
    ax.timer.func = Some(lis2dw_event);

    match args[2] {
        SPI_SERIAL => {
            if CONFIG_WANT_SPI {
                ax.bus.spi = spidev_oid_lookup(args[1] as u8)?;
                ax.bus_type = SPI_SERIAL as u8;
            } else {
                shutdown!("bus_type spi unsupported");
            }
        }
        I2C_SERIAL => {
            if CONFIG_WANT_I2C {
                ax.bus.i2c = i2cdev_oid_lookup(args[1] as u8)?;
                ax.bus_type = I2C_SERIAL as u8;
            } else {
                shutdown!("bus_type i2c unsupported");
            }
        }
        _ => shutdown!("bus_type invalid"),
    }

    match args[3] {
        LIS2DW | LIS3DH => ax.model = args[3] as u8,
        _ => shutdown!("model type invalid"),
    }
    Ok(())
}
decl_command!(
    command_config_lis2dw,
    concat!("config_lis2dw oid=%c", " bus_oid=%c bus_oid_type=%c lis_chip_type=%c")
);

// Helper code to reschedule the lis2dw_event() timer
fn lis2dw_reschedule_timer(ax: &mut Lis2dw) -> R {
    irq_disable();
    ax.timer.waketime = timer_read_time().wrapping_add(ax.rest_ticks);
    sched_add_timer(&mut ax.timer)?;
    irq_enable();
    Ok(())
}

// Update local status tracking from newly read fifo status register
fn update_fifo_status(ax: &mut Lis2dw, fifo_status: u8) {
    if fifo_status & 0x40 != 0 {
        ax.sb.possible_overflows = ax.sb.possible_overflows.wrapping_add(1);
    }

    let pending = if ax.model == LIS3DH as u8 {
        if fifo_status & 0x20 != 0 {
            0
        } else {
            fifo_status & 0x1F
        }
    } else {
        fifo_status & 0x3F
    };
    ax.fifo_bytes_pending = pending.wrapping_mul(BYTES_PER_SAMPLE);
}

// Query fifo status register
fn query_fifo_status(ax: &mut Lis2dw) -> R {
    let mut fifo_status = 0u8;
    if CONFIG_WANT_SPI && ax.bus_type == SPI_SERIAL as u8 {
        let mut fifo = [LIS_FIFO_SAMPLES | LIS_AM_READ, 0x00];
        spidev_transfer(unsafe { ax.bus.spi }, 1, &mut fifo)?;
        fifo_status = fifo[1];
    } else if CONFIG_WANT_I2C && ax.bus_type == I2C_SERIAL as u8 {
        let fifo_reg = [LIS_FIFO_SAMPLES];
        let mut st = [0u8];
        let ret = i2c_dev_read(unsafe { ax.bus.i2c }, &fifo_reg, &mut st)?;
        fifo_status = st[0];
        i2c_shutdown_on_err(ret)?;
    }
    update_fifo_status(ax, fifo_status);
    Ok(())
}

// Read 8 samples from FIFO via SPI
fn read_fifo_block_spi(ax: &mut Lis2dw) -> R {
    let mut msg = [0u8; BYTES_PER_BLOCK as usize + 1];
    msg[0] = LIS_AR_DATAX0 | LIS_AM_READ;
    if ax.model == LIS3DH as u8 {
        msg[0] |= LIS_MS_SPI;
    }

    spidev_transfer(unsafe { ax.bus.spi }, 1, &mut msg)?;
    ax.sb.data[..BYTES_PER_BLOCK as usize].copy_from_slice(&msg[1..]);
    Ok(())
}

// Read 8 samples from FIFO via i2c
fn read_fifo_block_i2c(ax: &mut Lis2dw) -> R {
    let mut msg_reg = [LIS_AR_DATAX0];
    if ax.model == LIS3DH as u8 {
        msg_reg[0] |= LIS_MS_I2C;
    }

    let ret = i2c_dev_read(unsafe { ax.bus.i2c }, &msg_reg, &mut ax.sb.data[..BYTES_PER_BLOCK as usize])?;
    i2c_shutdown_on_err(ret)
}

// Read from fifo and transmit data to host
fn read_fifo_block(ax: &mut Lis2dw, oid: u8) -> R {
    if CONFIG_WANT_SPI && ax.bus_type == SPI_SERIAL as u8 {
        read_fifo_block_spi(ax)?;
    } else if CONFIG_WANT_I2C && ax.bus_type == I2C_SERIAL as u8 {
        read_fifo_block_i2c(ax)?;
    }
    ax.sb.data_count = BYTES_PER_BLOCK;
    sensor_bulk_report(&mut ax.sb, oid)?;
    ax.fifo_bytes_pending = ax.fifo_bytes_pending.wrapping_sub(BYTES_PER_BLOCK);
    Ok(())
}

// Query accelerometer data
fn lis2dw_query(ax: &mut Lis2dw, oid: u8) -> R {
    if ax.fifo_bytes_pending < BYTES_PER_BLOCK {
        query_fifo_status(ax)?;
    }

    if ax.fifo_bytes_pending >= BYTES_PER_BLOCK {
        read_fifo_block(ax, oid)?;
    }

    // check if we need to run the task again (more packets in fifo?)
    if ax.fifo_bytes_pending >= BYTES_PER_BLOCK {
        // More data in fifo - wake this task again
        sched_wake_task(LIS2DW_WAKE.ptr());
    } else {
        // Sleep until next check time
        ax.flags &= !LIS_PENDING;
        lis2dw_reschedule_timer(ax)?;
    }
    Ok(())
}

pub fn command_query_lis2dw(args: &[u32]) -> R {
    let ax = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Lis2dw) };

    sched_del_timer(&mut ax.timer);
    ax.flags = 0;
    if args[1] == 0 {
        // End measurements
        return Ok(());
    }

    // Start new measurements query
    ax.rest_ticks = args[1];
    ax.fifo_bytes_pending = 0;
    sensor_bulk_reset(&mut ax.sb);
    lis2dw_reschedule_timer(ax)
}
decl_command!(command_query_lis2dw, "query_lis2dw oid=%c rest_ticks=%u");

pub fn command_query_lis2dw_status(args: &[u32]) -> R {
    let ax = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Lis2dw) };
    let mut time1 = 0;
    let mut time2 = 0;
    let mut fifo_status = 0u8;

    if CONFIG_WANT_SPI && ax.bus_type == SPI_SERIAL as u8 {
        let mut fifo = [LIS_FIFO_SAMPLES | LIS_AM_READ, 0x00];
        time1 = timer_read_time();
        spidev_transfer(unsafe { ax.bus.spi }, 1, &mut fifo)?;
        time2 = timer_read_time();
        fifo_status = fifo[1];
    } else if CONFIG_WANT_I2C && ax.bus_type == I2C_SERIAL as u8 {
        let fifo_reg = [LIS_FIFO_SAMPLES];
        let mut st = [0u8];
        time1 = timer_read_time();
        let ret = i2c_dev_read(unsafe { ax.bus.i2c }, &fifo_reg, &mut st)?;
        time2 = timer_read_time();
        fifo_status = st[0];
        i2c_shutdown_on_err(ret)?;
    }
    update_fifo_status(ax, fifo_status);

    sensor_bulk_status(&ax.sb, args[0] as u8, time1, time2.wrapping_sub(time1), ax.fifo_bytes_pending as u32)
}
decl_command!(command_query_lis2dw_status, "query_lis2dw_status oid=%c");

pub fn lis2dw_task() -> R {
    if !sched_check_wake(LIS2DW_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let ax = unsafe { &mut *(p as *mut Lis2dw) };
        let flags = ax.flags;
        if flags & LIS_PENDING != 0 {
            lis2dw_query(ax, oid)?;
        }
    }
    Ok(())
}
decl_task!(lis2dw_task);
