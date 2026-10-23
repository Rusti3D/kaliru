// Commands for sending messages on an SDIO bus
//
// Copyright (C) 2022 H. Gregor Molter <gregor.molter@secretlab.de>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{sdio_prepare_data_transfer, sdio_read_data, sdio_send_command, sdio_set_speed, sdio_setup, sdio_write_data, SdioConfig};
use crate::command::command_decode_ptr;
use crate::sched::R;
use crate::{decl_command, sendf};
use core::mem::size_of;

#[repr(C)]
pub struct SdiodevS {
    sdio_config: SdioConfig,
    blocksize: u32,
    speed: u32,
    data_buffer: [u8; 4096],
}

const TIMEOUT_MSEC: u32 = 500;

fn oid_type() -> OidType {
    command_config_sdio as *const ()
}

pub fn command_config_sdio(args: &[u32]) -> R {
    let sdio = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<SdiodevS>() as u16)? as *mut SdiodevS) };
    sdio.blocksize = args[1];
    sdio.speed = 400000; // Initial speed set to ~400khz
    Ok(())
}
decl_command!(command_config_sdio, "config_sdio oid=%c blocksize=%u");

pub fn sdiodev_oid_lookup(oid: u8) -> R<*mut SdiodevS> {
    Ok(oid_lookup(oid, oid_type())? as *mut SdiodevS)
}

pub fn command_sdio_set_bus(args: &[u32]) -> R {
    let sdio = unsafe { &mut *sdiodev_oid_lookup(args[0] as u8)? };
    sdio.sdio_config = sdio_setup(args[1])?;
    Ok(())
}
decl_command!(command_sdio_set_bus, "sdio_set_bus oid=%c sdio_bus=%u");

pub fn command_sdio_set_speed(args: &[u32]) -> R {
    let sdio = unsafe { &mut *sdiodev_oid_lookup(args[0] as u8)? };
    sdio.speed = args[1];
    sdio_set_speed(sdio.sdio_config, sdio.speed);
    Ok(())
}
decl_command!(command_sdio_set_speed, "sdio_set_speed oid=%c speed=%u");

pub fn command_sdio_send_command(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let cmd = args[1] as u8;
    let argument = args[2];
    let wait = args[3] as u8;
    let sdio = unsafe { &mut *sdiodev_oid_lookup(oid)? };
    let mut response = [0u8; 16];
    let mut response_len = 0u8;
    let err = sdio_send_command(sdio.sdio_config, cmd, argument, wait, Some(&mut response), &mut response_len);
    sendf!("sdio_send_command_response oid=%c error=%c response=%*s", oid, err, &response[..response_len as usize])
}
decl_command!(command_sdio_send_command, "sdio_send_command oid=%c cmd=%c argument=%u wait=%c");

pub fn command_sdio_read_data(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let cmd = args[1] as u8;
    let argument = args[2];
    let mut data_len = 0;
    let sdio = unsafe { &mut *sdiodev_oid_lookup(oid)? };
    let timeout = TIMEOUT_MSEC.wrapping_mul(sdio.speed) / 1000;
    let mut err = sdio_prepare_data_transfer(sdio.sdio_config, 1, 1, sdio.blocksize, timeout);
    if err == 0 {
        let mut unused = 0;
        err = sdio_send_command(sdio.sdio_config, cmd, argument, 1, None, &mut unused);
        if err == 0 {
            data_len = sdio.blocksize;
            if data_len as usize <= sdio.data_buffer.len() {
                err = sdio_read_data(sdio.sdio_config, Some(&mut sdio.data_buffer), 1, sdio.blocksize);
            } else {
                data_len = 0;
            }
        }
    }
    sendf!("sdio_read_data_response oid=%c error=%c read=%u", oid, err, data_len)
}
decl_command!(command_sdio_read_data, "sdio_read_data oid=%c cmd=%c argument=%u");

pub fn command_sdio_write_data(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let cmd = args[1] as u8;
    let argument = args[2];
    let mut data_len = 0;
    let sdio = unsafe { &mut *sdiodev_oid_lookup(oid)? };
    let timeout = TIMEOUT_MSEC.wrapping_mul(sdio.speed) / 1000;
    let mut err = sdio_prepare_data_transfer(sdio.sdio_config, 0, 1, sdio.blocksize, timeout);
    if err == 0 {
        let mut unused = 0;
        err = sdio_send_command(sdio.sdio_config, cmd, argument, 1, None, &mut unused);
        if err == 0 {
            data_len = sdio.blocksize;
            if data_len as usize <= sdio.data_buffer.len() {
                err = sdio_write_data(sdio.sdio_config, Some(&sdio.data_buffer), 1, sdio.blocksize);
            } else {
                data_len = 0;
            }
        }
    }
    sendf!("sdio_write_data_response oid=%c error=%c write=%u", oid, err, data_len)
}
decl_command!(command_sdio_write_data, "sdio_write_data oid=%c cmd=%c argument=%u");

// (offset + len wraps as the C uint32_t sum does; the buffer address is
// then computed the same way)
pub fn command_sdio_read_data_buffer(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let offset = args[1];
    let mut len = args[2] as u8;
    let sdio = unsafe { &mut *sdiodev_oid_lookup(oid)? };
    let buf = sdio.data_buffer.as_ptr().wrapping_add(offset as usize);

    if offset.wrapping_add(len as u32) as usize > sdio.data_buffer.len() {
        len = 0;
    }
    let data = unsafe { core::slice::from_raw_parts(buf, len as usize) };
    sendf!("sdio_read_data_buffer_response oid=%c data=%*s", oid, data)
}
decl_command!(command_sdio_read_data_buffer, "sdio_read_data_buffer oid=%c offset=%u len=%c");

pub fn command_sdio_write_data_buffer(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let offset = args[1];
    let write_data_len = args[2] as u8;
    let write_data = command_decode_ptr(args[3]);
    let sdio = unsafe { &mut *sdiodev_oid_lookup(oid)? };
    let buf = sdio.data_buffer.as_mut_ptr().wrapping_add(offset as usize);

    if offset.wrapping_add(write_data_len as u32) as usize <= sdio.data_buffer.len() {
        unsafe { core::ptr::copy(write_data, buf, write_data_len as usize) };
    }
    Ok(())
}
decl_command!(command_sdio_write_data_buffer, "sdio_write_data_buffer oid=%c offset=%u data=%*s");
