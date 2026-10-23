// Very basic shift-register support via a Linux SPI device
//
// Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::{report_errno, set_non_blocking};
use crate::ctr::Global;
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown, try_shutdown};
use std::ffi::CString;

// used by the declarations (a C macro)
#[allow(dead_code)]
const fn spibus(chip: u32, pin: u32) -> u32 {
    (chip << 8) + pin
}

decl_enumeration_range!("spi_bus", "spidev0.0", spibus(0, 0), 16);
decl_enumeration_range!("spi_bus", "spidev1.0", spibus(1, 0), 16);
decl_enumeration_range!("spi_bus", "spidev2.0", spibus(2, 0), 16);
decl_enumeration_range!("spi_bus", "spidev3.0", spibus(3, 0), 16);
decl_enumeration_range!("spi_bus", "spidev4.0", spibus(4, 0), 16);
decl_enumeration_range!("spi_bus", "spidev5.0", spibus(5, 0), 16);
decl_enumeration_range!("spi_bus", "spidev6.0", spibus(6, 0), 16);
decl_enumeration_range!("spi_bus", "spidev7.0", spibus(7, 0), 16);

// <linux/spi/spidev.h>
#[repr(C)]
struct SpiIocTransfer {
    tx_buf: u64,
    rx_buf: u64,
    len: u32,
    speed_hz: u32,
    delay_usecs: u16,
    bits_per_word: u8,
    cs_change: u8,
    tx_nbits: u8,
    rx_nbits: u8,
    word_delay_usecs: u8,
    pad: u8,
}

/// _IOW(type, nr, size)
const fn iow(typ: u32, nr: u32, size: usize) -> libc::c_ulong {
    ((1 << 30) | ((size as u32) << 16) | (typ << 8) | nr) as libc::c_ulong
}
const SPI_IOC_MAGIC: u32 = b'k' as u32;
const SPI_IOC_WR_MODE: libc::c_ulong = iow(SPI_IOC_MAGIC, 1, 1);
const SPI_IOC_WR_MAX_SPEED_HZ: libc::c_ulong = iow(SPI_IOC_MAGIC, 4, 4);
const SPI_IOC_MESSAGE_1: libc::c_ulong = iow(SPI_IOC_MAGIC, 0, core::mem::size_of::<SpiIocTransfer>());

#[derive(Clone, Copy)]
#[repr(C)]
pub struct SpiConfig {
    fd: libc::c_int,
    rate: libc::c_int,
}

#[derive(Clone, Copy)]
#[repr(C)]
struct SpiS {
    bus: u32,
    dev: u32,
    fd: libc::c_int,
}

static DEVICES: Global<[SpiS; 16]> = Global::new([SpiS { bus: 0, dev: 0, fd: 0 }; 16]);
static DEVICES_COUNT: Global<usize> = Global::new(0);

fn spi_open(bus: u32, dev: u32) -> R<libc::c_int> {
    // Find existing device (if already opened)
    let devices = DEVICES.get();
    let count = DEVICES_COUNT.get();
    for d in &devices[..*count] {
        if d.bus == bus && d.dev == dev {
            return Ok(d.fd);
        }
    }

    // Setup new SPI device
    if *count >= devices.len() {
        shutdown!("Too many spi devices");
    }
    let fname = CString::new(format!("/dev/spidev{}.{}", bus as i32, dev as i32)).unwrap();
    let fd = unsafe { libc::open(fname.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
    if fd < 0 {
        report_errno("open spi", fd);
        shutdown!("Unable to open spi device");
    }
    if set_non_blocking(fd) < 0 {
        shutdown!("Unable to set non-blocking on spi device");
    }

    devices[*count] = SpiS { bus, dev, fd };
    *count += 1;
    Ok(fd)
}

pub fn spi_setup(bus: u32, mode: u8, rate: u32) -> R<SpiConfig> {
    let (bus_id, dev_id) = (bus >> 8, bus & 0xff);
    let fd = spi_open(bus_id, dev_id)?;
    let mut rate = rate;
    let ret = unsafe { libc::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ as _, &mut rate) };
    if ret < 0 {
        report_errno("ioctl set max spi speed", ret);
        shutdown!("Unable to set SPI speed");
    }
    let mut mode = mode;
    let ret = unsafe { libc::ioctl(fd, SPI_IOC_WR_MODE as _, &mut mode) };
    if ret < 0 {
        report_errno("ioctl set spi mode", ret);
        shutdown!("Unable to set SPI mode");
    }
    Ok(SpiConfig { fd, rate: rate as libc::c_int })
}

pub fn spi_prepare(_config: SpiConfig) {}

pub fn spi_transfer(config: SpiConfig, receive_data: u8, data: &mut [u8]) -> R {
    let len = data.len();
    if len == 0 {
        return Ok(());
    }

    if receive_data != 0 {
        let mut transfer: SpiIocTransfer = unsafe { core::mem::zeroed() };
        transfer.tx_buf = data.as_ptr() as u64;
        transfer.rx_buf = data.as_mut_ptr() as u64;
        transfer.len = len as u32;
        transfer.speed_hz = config.rate as u32;
        transfer.bits_per_word = 8;
        transfer.cs_change = 0;
        let ret = unsafe { libc::ioctl(config.fd, SPI_IOC_MESSAGE_1 as _, &mut transfer) };
        if ret < 0 {
            report_errno("spi ioctl", ret);
            try_shutdown!("Unable to issue spi ioctl");
        }
    } else {
        let ret = unsafe { libc::write(config.fd, data.as_ptr() as *const libc::c_void, len) };
        if ret < 0 {
            report_errno("write spi", ret as i32);
            try_shutdown!("Unable to write to spi");
        }
    }
    Ok(())
}
