// Commands for sending messages on an SPI bus
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{foreach_oid, oid_alloc, oid_lookup, OidType};
use crate::board::{gpio_out_setup, gpio_out_write, GpioOut, SpiConfig};
use crate::command::{command_decode_buffer, command_decode_ptr};
use crate::sched::R;
#[cfg(CONFIG_WANT_SOFTWARE_SPI)]
use crate::spi_software::SpiSoftware;
use crate::{decl_command, decl_shutdown, sendf, shutdown};
use core::mem::size_of;

#[cfg(not(CONFIG_WANT_SOFTWARE_SPI))]
pub enum SpiSoftware {}

#[repr(C)]
#[derive(Clone, Copy)]
union SpiBus {
    spi_config: SpiConfig,
    spi_software: *mut SpiSoftware,
}

#[repr(C)]
pub struct SpiDev {
    bus: SpiBus,
    pin: GpioOut,
    flags: u8,
}

const SF_HAVE_PIN: u8 = 1;
const SF_SOFTWARE: u8 = 2;
const SF_HARDWARE: u8 = 4;
const SF_CS_ACTIVE_HIGH: u8 = 8;

// spi_transfer / spi_prepare: the board's, or (no hardware SPI) none
#[cfg(CONFIG_WANT_GPIO_SPI)]
use crate::board::{spi_prepare, spi_setup, spi_transfer};
#[cfg(not(CONFIG_WANT_GPIO_SPI))]
fn spi_transfer(_spi: SpiConfig, _receive_data: u8, _data: &mut [u8]) -> R {
    Ok(())
}
#[cfg(not(CONFIG_WANT_GPIO_SPI))]
fn spi_prepare(_spi: SpiConfig) {}

fn oid_type() -> OidType {
    command_config_spi as *const ()
}

fn shutdown_oid_type() -> OidType {
    command_config_spi_shutdown as *const ()
}

pub fn command_config_spi(args: &[u32]) -> R {
    let spi = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<SpiDev>() as u16)? as *mut SpiDev) };
    let cs_active_high = args[2] as u8;
    spi.pin = gpio_out_setup(args[1], (cs_active_high == 0) as u8)?;
    spi.flags |= SF_HAVE_PIN | if cs_active_high != 0 { SF_CS_ACTIVE_HIGH } else { 0 };
    Ok(())
}
decl_command!(command_config_spi, "config_spi oid=%c pin=%u cs_active_high=%c");

pub fn command_config_spi_without_cs(args: &[u32]) -> R {
    oid_alloc(args[0] as u8, oid_type(), size_of::<SpiDev>() as u16)?;
    Ok(())
}
decl_command!(command_config_spi_without_cs, "config_spi_without_cs oid=%c");

pub fn spidev_oid_lookup(oid: u8) -> R<*mut SpiDev> {
    Ok(oid_lookup(oid, oid_type())? as *mut SpiDev)
}

#[cfg(CONFIG_WANT_GPIO_SPI)]
pub fn command_spi_set_bus(args: &[u32]) -> R {
    let spi = unsafe { &mut *spidev_oid_lookup(args[0] as u8)? };
    let mode = args[2] as u8;
    if mode > 3 || spi.flags & (SF_SOFTWARE | SF_HARDWARE) != 0 {
        shutdown!("Invalid spi config");
    }
    spi.bus.spi_config = spi_setup(args[1], mode, args[3])?;
    spi.flags |= SF_HARDWARE;
    Ok(())
}
#[cfg(not(CONFIG_WANT_GPIO_SPI))]
pub fn command_spi_set_bus(_args: &[u32]) -> R {
    shutdown!("hardware SPI is not supported/enabled");
}
decl_command!(command_spi_set_bus, "spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u");

#[cfg(CONFIG_WANT_SOFTWARE_SPI)]
pub fn spidev_set_software_bus(spi: *mut SpiDev, ss: *mut SpiSoftware) -> R {
    let spi = unsafe { &mut *spi };
    if spi.flags & (SF_SOFTWARE | SF_HARDWARE) != 0 {
        shutdown!("Invalid spi config");
    }
    spi.bus.spi_software = ss;
    spi.flags |= SF_SOFTWARE;
    Ok(())
}
#[cfg(not(CONFIG_WANT_SOFTWARE_SPI))]
pub fn spidev_set_software_bus(_spi: *mut SpiDev, _ss: *mut SpiSoftware) -> R {
    shutdown!("software SPI is not supported/enabled");
}

pub fn spidev_have_cs_pin(spi: *mut SpiDev) -> bool {
    unsafe { (*spi).flags & SF_HAVE_PIN != 0 }
}

pub fn spidev_get_cs_pin(spi: *mut SpiDev) -> GpioOut {
    unsafe { (*spi).pin }
}

// the bus of a software or hardware spi device
#[cfg(CONFIG_WANT_SOFTWARE_SPI)]
fn bus_prepare(spi: &SpiDev) {
    if spi.flags & SF_SOFTWARE != 0 {
        crate::spi_software::spi_software_prepare(unsafe { spi.bus.spi_software });
    } else {
        spi_prepare(unsafe { spi.bus.spi_config });
    }
}
#[cfg(not(CONFIG_WANT_SOFTWARE_SPI))]
fn bus_prepare(spi: &SpiDev) {
    spi_prepare(unsafe { spi.bus.spi_config });
}

#[cfg(CONFIG_WANT_SOFTWARE_SPI)]
fn bus_transfer(spi: &SpiDev, receive_data: u8, data: &mut [u8]) -> R {
    if spi.flags & SF_SOFTWARE != 0 {
        crate::spi_software::spi_software_transfer(unsafe { spi.bus.spi_software }, receive_data, data);
        Ok(())
    } else {
        spi_transfer(unsafe { spi.bus.spi_config }, receive_data, data)
    }
}
#[cfg(not(CONFIG_WANT_SOFTWARE_SPI))]
fn bus_transfer(spi: &SpiDev, receive_data: u8, data: &mut [u8]) -> R {
    spi_transfer(unsafe { spi.bus.spi_config }, receive_data, data)
}

pub fn spidev_transfer(spi: *mut SpiDev, receive_data: u8, data: &mut [u8]) -> R {
    let spi = unsafe { &*spi };
    let flags = spi.flags;
    if flags & (SF_SOFTWARE | SF_HARDWARE) == 0 {
        // Not yet initialized
        return Ok(());
    }

    bus_prepare(spi);

    if flags & SF_HAVE_PIN != 0 {
        gpio_out_write(spi.pin, (flags & SF_CS_ACTIVE_HIGH != 0) as u8);
    }

    bus_transfer(spi, receive_data, data)?;

    if flags & SF_HAVE_PIN != 0 {
        gpio_out_write(spi.pin, (flags & SF_CS_ACTIVE_HIGH == 0) as u8);
    }
    Ok(())
}

pub fn command_spi_transfer(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let spi = spidev_oid_lookup(oid)?;
    let data = command_decode_buffer(args[1] as u8 as u32, args[2]);
    spidev_transfer(spi, 1, data)?;
    sendf!("spi_transfer_response oid=%c response=%*s", oid, &*data)
}
decl_command!(command_spi_transfer, "spi_transfer oid=%c data=%*s");

pub fn command_spi_send(args: &[u32]) -> R {
    let spi = spidev_oid_lookup(args[0] as u8)?;
    let data = command_decode_buffer(args[1] as u8 as u32, args[2]);
    spidev_transfer(spi, 0, data)
}
decl_command!(command_spi_send, "spi_send oid=%c data=%*s");

/****************************************************************
 * Shutdown handling
 ****************************************************************/

#[repr(C)]
struct SpidevShutdown {
    spi: *mut SpiDev,
    shutdown_msg_len: u8,
    // the message follows (a flexible array member)
}

fn shutdown_msg(sd: *mut SpidevShutdown) -> &'static mut [u8] {
    unsafe {
        let p = (sd as *mut u8).add(size_of::<SpidevShutdown>());
        core::slice::from_raw_parts_mut(p, (*sd).shutdown_msg_len as usize)
    }
}

pub fn command_config_spi_shutdown(args: &[u32]) -> R {
    let spi = spidev_oid_lookup(args[1] as u8)?;
    let shutdown_msg_len = args[2] as u8;
    let sd = oid_alloc(args[0] as u8, shutdown_oid_type(), (size_of::<SpidevShutdown>() + shutdown_msg_len as usize) as u16)?
        as *mut SpidevShutdown;
    unsafe {
        (*sd).spi = spi;
        (*sd).shutdown_msg_len = shutdown_msg_len;
    }
    let msg = command_decode_ptr(args[3]);
    shutdown_msg(sd).copy_from_slice(unsafe { core::slice::from_raw_parts(msg, shutdown_msg_len as usize) });
    Ok(())
}
decl_command!(command_config_spi_shutdown, "config_spi_shutdown oid=%c spi_oid=%c shutdown_msg=%*s");

pub fn spidev_shutdown() -> R {
    // Cancel any transmissions that may be in progress
    for p in foreach_oid(oid_type()) {
        let spi = unsafe { &*(p as *mut SpiDev) };
        if spi.flags & SF_HAVE_PIN != 0 {
            gpio_out_write(spi.pin, (spi.flags & SF_CS_ACTIVE_HIGH == 0) as u8);
        }
    }

    // Send shutdown messages
    for p in foreach_oid(shutdown_oid_type()) {
        let sd = p as *mut SpidevShutdown;
        spidev_transfer(unsafe { (*sd).spi }, 0, shutdown_msg(sd))?;
    }
    Ok(())
}
decl_shutdown!(spidev_shutdown);
