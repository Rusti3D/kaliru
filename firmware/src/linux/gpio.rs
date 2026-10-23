// Very basic support via a Linux gpiod device
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::{report_errno, set_close_on_exec};
use crate::ctr::Global;
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown};
use std::ffi::CString;

// internal.h
pub const MAX_GPIO_LINES: u32 = 288;
pub const fn gpio(port: u32, num: u32) -> u32 {
    port * MAX_GPIO_LINES + num
}
pub const fn gpio2port(pin: u32) -> u32 {
    pin / MAX_GPIO_LINES
}
pub const fn gpio2pin(pin: u32) -> u32 {
    pin % MAX_GPIO_LINES
}

const GPIO_CONSUMER: &[u8] = b"klipper";

decl_enumeration_range!("pin", "gpio0", gpio(0, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip0/gpio0", gpio(0, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip1/gpio0", gpio(1, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip2/gpio0", gpio(2, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip3/gpio0", gpio(3, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip4/gpio0", gpio(4, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip5/gpio0", gpio(5, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip6/gpio0", gpio(6, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip7/gpio0", gpio(7, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip8/gpio0", gpio(8, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip9/gpio0", gpio(9, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip10/gpio0", gpio(10, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip11/gpio0", gpio(11, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip12/gpio0", gpio(12, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip13/gpio0", gpio(13, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip14/gpio0", gpio(14, 0), MAX_GPIO_LINES);
decl_enumeration_range!("pin", "gpiochip15/gpio0", gpio(15, 0), MAX_GPIO_LINES);

// <linux/gpio.h> (the v1 line handle ABI)
const GPIOHANDLES_MAX: usize = 64;
const GPIOHANDLE_REQUEST_INPUT: u32 = 1 << 0;
const GPIOHANDLE_REQUEST_OUTPUT: u32 = 1 << 1;
const GPIOHANDLE_REQUEST_BIAS_PULL_UP: u32 = 1 << 5;
const GPIOHANDLE_REQUEST_BIAS_PULL_DOWN: u32 = 1 << 6;

#[repr(C)]
struct GpiohandleRequest {
    lineoffsets: [u32; GPIOHANDLES_MAX],
    flags: u32,
    default_values: [u8; GPIOHANDLES_MAX],
    consumer_label: [u8; 32],
    lines: u32,
    fd: libc::c_int,
}

#[repr(C)]
struct GpiohandleData {
    values: [u8; GPIOHANDLES_MAX],
}

/// _IOWR(type, nr, size)
const fn iowr(typ: u32, nr: u32, size: usize) -> libc::c_ulong {
    ((3 << 30) | ((size as u32) << 16) | (typ << 8) | nr) as libc::c_ulong
}
const GPIO_GET_LINEHANDLE_IOCTL: libc::c_ulong = iowr(0xB4, 0x03, core::mem::size_of::<GpiohandleRequest>());
const GPIOHANDLE_GET_LINE_VALUES_IOCTL: libc::c_ulong = iowr(0xB4, 0x08, core::mem::size_of::<GpiohandleData>());
const GPIOHANDLE_SET_LINE_VALUES_IOCTL: libc::c_ulong = iowr(0xB4, 0x09, core::mem::size_of::<GpiohandleData>());

#[repr(C)]
pub struct GpioLine {
    chipid: libc::c_int,
    offset: libc::c_int,
    fd: libc::c_int,
    state: libc::c_int,
}

const NUM_LINES: usize = 16 * MAX_GPIO_LINES as usize;
static GPIO_LINES: Global<[GpioLine; NUM_LINES]> =
    Global::new([const { GpioLine { chipid: 0, offset: 0, fd: 0, state: 0 } }; NUM_LINES]);
static GPIO_CHIP_FD: Global<[libc::c_int; 16]> = Global::new([-1; 16]);

fn line(pin: u32) -> *mut GpioLine {
    unsafe { (*GPIO_LINES.ptr()).as_mut_ptr().add(pin as usize) }
}

fn get_chip_fd(chip_id: u8) -> R<libc::c_int> {
    let fds = GPIO_CHIP_FD.get();
    if fds[chip_id as usize] >= 0 {
        return Ok(fds[chip_id as usize]);
    }
    let chip_filename = CString::new(format!("/dev/gpiochip{}", chip_id)).unwrap();
    let ret = unsafe { libc::access(chip_filename.as_ptr(), libc::F_OK) };
    if ret < 0 {
        report_errno("gpio access", ret);
        shutdown!("GPIO chip device not found");
    }
    let fd = unsafe { libc::open(chip_filename.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
    if fd < 0 {
        report_errno("gpio open", fd);
        shutdown!("Unable to open GPIO chip device");
    }
    fds[chip_id as usize] = fd;
    for i in 0..MAX_GPIO_LINES {
        let l = unsafe { &mut *line(gpio(chip_id as u32, i)) };
        l.offset = i as libc::c_int;
        l.fd = -1;
        l.chipid = chip_id as libc::c_int;
    }
    Ok(fd)
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct GpioOut {
    line: *mut GpioLine,
}

pub fn gpio_out_setup(pin: u32, val: u8) -> R<GpioOut> {
    if pin as usize >= NUM_LINES {
        shutdown!("Not an output pin");
    }
    let l = line(pin);
    unsafe {
        (*l).offset = gpio2pin(pin) as libc::c_int;
        (*l).chipid = gpio2port(pin) as libc::c_int;
    }
    let g = GpioOut { line: l };
    gpio_out_reset(g, val)?;
    Ok(g)
}

fn gpio_release_line(line: *mut GpioLine) {
    unsafe {
        if (*line).fd > 0 {
            libc::close((*line).fd);
            (*line).fd = -1;
        }
    }
}

fn line_request(l: *mut GpioLine, flags: u32, val: u8) -> GpiohandleRequest {
    let mut req: GpiohandleRequest = unsafe { core::mem::zeroed() };
    req.lines = 1;
    req.flags = flags;
    req.lineoffsets[0] = unsafe { (*l).offset } as u32;
    req.default_values[0] = (val != 0) as u8;
    req.consumer_label[..GPIO_CONSUMER.len()].copy_from_slice(GPIO_CONSUMER);
    req
}

pub fn gpio_out_reset(g: GpioOut, val: u8) -> R {
    gpio_release_line(g.line);
    let mut req = line_request(g.line, GPIOHANDLE_REQUEST_OUTPUT, val);
    let fd = get_chip_fd(unsafe { (*g.line).chipid } as u8)?;
    let ret = unsafe { libc::ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL as _, &mut req) };
    if ret < 0 {
        report_errno("gpio_out_reset get line", ret);
        shutdown!("Unable to open out GPIO chip line");
    }
    set_close_on_exec(req.fd);
    unsafe {
        (*g.line).fd = req.fd;
        (*g.line).state = (val != 0) as libc::c_int;
    }
    Ok(())
}

pub fn gpio_out_write(g: GpioOut, val: u8) {
    let mut data = GpiohandleData { values: [0; GPIOHANDLES_MAX] };
    data.values[0] = (val != 0) as u8;
    unsafe {
        libc::ioctl((*g.line).fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL as _, &mut data);
        (*g.line).state = (val != 0) as libc::c_int;
    }
}

pub fn gpio_out_toggle_noirq(g: GpioOut) {
    gpio_out_write(g, (unsafe { (*g.line).state } == 0) as u8);
}

pub fn gpio_out_toggle(g: GpioOut) {
    gpio_out_toggle_noirq(g);
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct GpioIn {
    line: *mut GpioLine,
}

pub fn gpio_in_setup(pin: u32, pull_up: i8) -> R<GpioIn> {
    if pin as usize >= NUM_LINES {
        shutdown!("Not an input pin");
    }
    let l = line(pin);
    unsafe {
        (*l).offset = gpio2pin(pin) as libc::c_int;
        (*l).chipid = gpio2port(pin) as libc::c_int;
    }
    let g = GpioIn { line: l };
    gpio_in_reset(g, pull_up)?;
    Ok(g)
}

pub fn gpio_in_reset(g: GpioIn, pull_up: i8) -> R {
    gpio_release_line(g.line);
    let mut flags = GPIOHANDLE_REQUEST_INPUT;
    if pull_up > 0 {
        flags |= GPIOHANDLE_REQUEST_BIAS_PULL_UP;
    } else if pull_up < 0 {
        flags |= GPIOHANDLE_REQUEST_BIAS_PULL_DOWN;
    }
    let mut req = line_request(g.line, flags, 0);
    let fd = get_chip_fd(unsafe { (*g.line).chipid } as u8)?;
    let ret = unsafe { libc::ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL as _, &mut req) };
    if ret < 0 {
        report_errno("gpio_in_reset get line", ret);
        shutdown!("Unable to open in GPIO chip line");
    }
    set_close_on_exec(req.fd);
    unsafe { (*g.line).fd = req.fd };
    Ok(())
}

pub fn gpio_in_read(g: GpioIn) -> u8 {
    let mut data = GpiohandleData { values: [0; GPIOHANDLES_MAX] };
    unsafe { libc::ioctl((*g.line).fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL as _, &mut data) };
    data.values[0]
}
