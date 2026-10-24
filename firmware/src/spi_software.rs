// Software SPI emulation
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::alloc_chunk;
use crate::board::{gpio_in_read, gpio_in_setup, gpio_out_setup, gpio_out_toggle, gpio_out_write, timer_is_before, timer_read_time, GpioIn, GpioOut};
use crate::command::Args;
use crate::sched::R;
use crate::spicmds::{spidev_oid_lookup, spidev_set_software_bus};
use crate::{decl_command, shutdown};
use core::mem::size_of;

#[repr(C)]
pub struct SpiSoftware {
    miso: GpioIn,
    mosi: GpioOut,
    sclk: GpioOut,
    sck_ticks: u32,
    mode: u8,
}

pub fn command_spi_set_sw_bus(args: &Args) -> R {
    let mode = args[4] as u8;
    let pulse_ticks = args[5];
    if mode > 3 {
        shutdown!("Invalid spi config");
    }

    let spi = spidev_oid_lookup(args[0] as u8)?;
    let ss = alloc_chunk(size_of::<SpiSoftware>())? as *mut SpiSoftware;
    let s = unsafe { &mut *ss };
    s.miso = gpio_in_setup(args[1], 1)?;
    s.mosi = gpio_out_setup(args[2], 0)?;
    s.sclk = gpio_out_setup(args[3], 0)?;
    s.mode = mode;
    s.sck_ticks = pulse_ticks;
    spidev_set_software_bus(spi, ss)
}
decl_command!(
    command_spi_set_sw_bus,
    concat!("spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u", " mode=%u pulse_ticks=%u")
);

fn spi_delay(end: u32) {
    while timer_is_before(timer_read_time(), end) {}
}

pub fn spi_software_prepare(ss: *mut SpiSoftware) {
    let ss = unsafe { &*ss };
    gpio_out_write(ss.sclk, ss.mode & 0x02);
    let end = timer_read_time().wrapping_add(ss.sck_ticks);
    spi_delay(end);
}

pub fn spi_software_transfer(ss: *mut SpiSoftware, receive_data: u8, data: &mut [u8]) {
    let ss = unsafe { &*ss };
    let t1 = ss.sck_ticks >> 1;
    let t2 = ss.sck_ticks - t1;
    let mut end = timer_read_time().wrapping_add(t1);
    for d in data.iter_mut() {
        let mut outbuf = *d;
        let mut inbuf: u8 = 0;
        if ss.mode & 0x01 != 0 {
            for _ in 0..8 {
                spi_delay(end);
                // MODE 1 & 3
                gpio_out_toggle(ss.sclk);
                gpio_out_write(ss.mosi, outbuf & 0x80);
                end = timer_read_time().wrapping_add(t2);
                outbuf <<= 1;
                inbuf <<= 1;
                spi_delay(end);
                gpio_out_toggle(ss.sclk);
                inbuf |= gpio_in_read(ss.miso);
                end = timer_read_time().wrapping_add(t1);
            }
        } else {
            for _ in 0..8 {
                spi_delay(end);
                // MODE 0 & 2
                gpio_out_write(ss.mosi, outbuf & 0x80);
                gpio_out_toggle(ss.sclk);
                end = timer_read_time().wrapping_add(t2);
                outbuf <<= 1;
                inbuf <<= 1;
                spi_delay(end);
                inbuf |= gpio_in_read(ss.miso);
                gpio_out_toggle(ss.sclk);
                end = timer_read_time().wrapping_add(t1);
            }
        }

        if receive_data != 0 {
            *d = inbuf;
        }
    }
}
