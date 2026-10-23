// Read analog values from Linux IIO device
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::{report_errno, set_non_blocking};
use crate::sched::R;
use crate::{decl_constant, decl_enumeration_range, shutdown, try_shutdown};
use std::ffi::CString;

decl_constant!("ADC_MAX", 4095); // Assume 12bit adc

const ANALOG_START: u32 = 1 << 12;

decl_enumeration_range!("pin", "analog0", ANALOG_START, 8);

#[derive(Clone, Copy)]
#[repr(C)]
pub struct GpioAdc {
    fd: libc::c_int,
}

pub fn gpio_adc_setup(pin: u32) -> R<GpioAdc> {
    let fname = CString::new(format!("/sys/bus/iio/devices/iio:device0/in_voltage{}_raw", pin.wrapping_sub(ANALOG_START) as i32))
        .unwrap();
    let fd = unsafe { libc::open(fname.as_ptr(), libc::O_RDONLY | libc::O_CLOEXEC) };
    'fail: {
        if fd < 0 {
            report_errno("analog open", fd);
            break 'fail;
        }
        if set_non_blocking(fd) < 0 {
            break 'fail;
        }
        return Ok(GpioAdc { fd });
    }
    if fd >= 0 {
        unsafe { libc::close(fd) };
    }
    shutdown!("Unable to open adc device");
}

pub fn gpio_adc_sample(_g: GpioAdc) -> u32 {
    0
}

/// C atoi(): leading space, a sign, the digits
pub fn atoi(buf: &[u8]) -> i32 {
    let mut i = 0;
    while i < buf.len() && matches!(buf[i], b' ' | b'\t' | b'\n' | b'\r' | 0x0b | 0x0c) {
        i += 1;
    }
    let neg = i < buf.len() && buf[i] == b'-';
    if i < buf.len() && (buf[i] == b'-' || buf[i] == b'+') {
        i += 1;
    }
    let mut v: i32 = 0;
    while i < buf.len() && buf[i].is_ascii_digit() {
        v = v.wrapping_mul(10).wrapping_add((buf[i] - b'0') as i32);
        i += 1;
    }
    if neg {
        v.wrapping_neg()
    } else {
        v
    }
}

pub fn gpio_adc_read(g: GpioAdc) -> R<u16> {
    let mut buf = [0u8; 64];
    let ret = unsafe { libc::pread(g.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len() - 1, 0) };
    if ret <= 0 {
        report_errno("analog read", ret as i32);
        try_shutdown!("Error on analog read");
        return Ok(0);
    }
    Ok(atoi(&buf[..ret as usize]) as u16)
}

pub fn gpio_adc_cancel_sample(_g: GpioAdc) {}
