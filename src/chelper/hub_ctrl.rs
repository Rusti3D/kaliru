// USB hub port power control (port of lib/hub-ctrl/hub-ctrl.c and
// klippy/chelper run_hub_ctrl)
//
// Copyright (C) 2006 Free Software Initiative of Japan
// Author: NIIBE Yutaka  <gniibe at fsij.org>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Deliberate differences: python builds hub-ctrl.c with gcc / libusb-0.1 and
// runs `sudo <dir>/hub-ctrl -h 0 -P 2 -p N`; here the kaliru executable is
// the tool (`kaliru --hub-ctrl ...`, run the same way through sudo) and talks
// to the hubs through Linux usbdevfs. Hubs are numbered in libusb's order
// (libusb-0.1 keeps libusb-1.0's device list order, the udev scan sorted by
// sysfs path).
use crate::logging;
use std::os::fd::AsRawFd;

const USB_TYPE_CLASS: u8 = 0x01 << 5;
const USB_RECIP_DEVICE: u8 = 0x00;
const USB_RECIP_OTHER: u8 = 0x03;
const USB_ENDPOINT_IN: u8 = 0x80;
const USB_RT_HUB: u8 = USB_TYPE_CLASS | USB_RECIP_DEVICE;
const USB_RT_PORT: u8 = USB_TYPE_CLASS | USB_RECIP_OTHER;
const USB_DIR_IN: u8 = 0x80;
const USB_REQ_GET_STATUS: u8 = 0x00;
const USB_REQ_CLEAR_FEATURE: u8 = 0x01;
const USB_REQ_SET_FEATURE: u8 = 0x03;
const USB_REQ_GET_DESCRIPTOR: u8 = 0x06;
const USB_DT_HUB: u16 = 0x29;
const USB_CLASS_HUB: u8 = 9;
const USB_PORT_FEAT_POWER: u16 = 8;
const USB_PORT_FEAT_INDICATOR: u16 = 22;

const COMMAND_SET_NONE: i32 = 0;
const COMMAND_SET_LED: i32 = 1;
const COMMAND_SET_POWER: i32 = 2;
const HUB_LED_GREEN: i64 = 2;

const HUB_CHAR_LPSM: u8 = 0x0003;
const HUB_CHAR_PORTIND: u8 = 0x0080;
/// sizeof(struct usb_hub_descriptor)
const HUB_DESCRIPTOR_SIZE: i64 = 7;

const CTRL_TIMEOUT: u32 = 1000;
const USB_STATUS_SIZE: usize = 4;

/// struct usbdevfs_ctrltransfer
#[repr(C)]
struct CtrlTransfer {
    b_request_type: u8,
    b_request: u8,
    w_value: u16,
    w_index: u16,
    w_length: u16,
    timeout: u32,
    data: *mut libc::c_void,
}

/// USBDEVFS_CONTROL: _IOWR('U', 0, struct usbdevfs_ctrltransfer)
const USBDEVFS_CONTROL: u64 = (3 << 30) | ((std::mem::size_of::<CtrlTransfer>() as u64) << 16) | ((b'U' as u64) << 8);

/// an open hub (usb_dev_handle)
struct Handle(std::fs::File);

impl Handle {
    /// usb_control_msg(): bytes transferred, or the errno
    fn control_msg(&self, request_type: u8, request: u8, value: u16, index: u16, buf: &mut [u8]) -> Result<i64, std::io::Error> {
        let mut ct = CtrlTransfer {
            b_request_type: request_type,
            b_request: request,
            w_value: value,
            w_index: index,
            w_length: buf.len() as u16,
            timeout: CTRL_TIMEOUT,
            data: if buf.is_empty() { std::ptr::null_mut() } else { buf.as_mut_ptr() as *mut libc::c_void },
        };
        // SAFETY: ct points at buf (len bytes) or null for no data; the fd is open
        let r = unsafe { libc::ioctl(self.0.as_raw_fd(), USBDEVFS_CONTROL as _, &mut ct) };
        if r < 0 {
            return Err(std::io::Error::last_os_error());
        }
        Ok(r as i64)
    }
}

/// a usb device of the bus listing (struct usb_device)
struct Device {
    busnum: i64,
    devnum: i64,
    bus_dirname: String,
    class: u8,
}

impl Device {
    fn open(&self) -> Option<Handle> {
        let path = format!("/dev/bus/usb/{}/{:03}", self.bus_dirname, self.devnum);
        std::fs::OpenOptions::new().read(true).write(true).open(path).ok().map(Handle)
    }
}

/// every usb device, in libusb's order
fn usb_devices() -> Vec<Device> {
    let mut devs: Vec<(std::path::PathBuf, Device)> = vec![];
    let Ok(entries) = std::fs::read_dir("/sys/bus/usb/devices") else { return vec![] };
    for e in entries.flatten() {
        let p = e.path();
        let read = |f: &str| std::fs::read_to_string(p.join(f)).ok().map(|s| s.trim().to_string());
        let (Some(busnum), Some(devnum), Some(class)) = (read("busnum"), read("devnum"), read("bDeviceClass")) else { continue };
        let (Ok(busnum), Ok(devnum), Ok(class)) = (busnum.parse::<i64>(), devnum.parse::<i64>(), u8::from_str_radix(&class, 16)) else { continue };
        let syspath = std::fs::canonicalize(&p).unwrap_or(p);
        devs.push((syspath, Device { busnum, devnum, bus_dirname: format!("{:03}", busnum), class }));
    }
    devs.sort_by(|a, b| a.0.cmp(&b.0));
    devs.into_iter().map(|(_, d)| d).collect()
}

struct HubInfo {
    busnum: i64,
    devnum: i64,
    dev: Device,
    nport: usize,
}

fn hub_port_status(uh: &Handle, nport: usize) {
    println!(" Hub Port Status:");
    for i in 0..nport {
        let mut buf = [0u8; USB_STATUS_SIZE];
        if let Err(e) = uh.control_msg(USB_ENDPOINT_IN | USB_TYPE_CLASS | USB_RECIP_OTHER, USB_REQ_GET_STATUS, 0, i as u16 + 1, &mut buf) {
            eprintln!("cannot read port {} status, {} ({})", i + 1, e, e.raw_os_error().unwrap_or(0));
            break;
        }
        print!("   Port {}: {:02x}{:02x}.{:02x}{:02x}", i + 1, buf[3], buf[2], buf[1], buf[0]);
        let flags = |b: u8, names: &[(u8, &str)]| names.iter().filter(|(m, _)| b & m != 0).map(|(_, n)| *n).collect::<String>();
        print!("{}", flags(buf[2], &[(0x10, " C_RESET"), (0x08, " C_OC"), (0x04, " C_SUSPEND"), (0x02, " C_ENABLE"), (0x01, " C_CONNECT")]));
        print!("{}", flags(buf[1], &[(0x10, " indicator"), (0x08, " test"), (0x04, " highspeed"), (0x02, " lowspeed"), (0x01, " power")]));
        println!("{}", flags(buf[0], &[(0x10, " RESET"), (0x08, " oc"), (0x04, " suspend"), (0x02, " enable"), (0x01, " connect")]));
    }
}

fn usb_find_hubs(listing: bool, verbose: bool, busnum: i64, devnum: i64, hub: i64) -> Vec<HubInfo> {
    let mut hubs: Vec<HubInfo> = vec![];
    for dev in usb_devices() {
        if dev.class != USB_CLASS_HUB {
            continue;
        }
        let print = listing || (verbose && ((dev.busnum == busnum && dev.devnum == devnum) || hub == hubs.len() as i64));
        let Some(uh) = dev.open() else { continue };
        let mut buf = [0u8; 1024];
        match uh.control_msg(USB_DIR_IN | USB_RT_HUB, USB_REQ_GET_DESCRIPTOR, USB_DT_HUB << 8, 0, &mut buf) {
            Ok(len) if len > HUB_DESCRIPTOR_SIZE => {
                let chars = buf[3];
                if chars & HUB_CHAR_PORTIND == 0 && (chars & HUB_CHAR_LPSM) >= 2 {
                    continue;
                }
                if print {
                    println!("Hub #{} at {}:{:03}", hubs.len(), dev.bus_dirname, dev.devnum);
                }
                match chars & HUB_CHAR_LPSM {
                    0 if print => eprintln!(" INFO: ganged switching."),
                    1 if print => eprintln!(" INFO: individual power switching."),
                    2 | 3 if print => eprintln!(" WARN: No power switching."),
                    _ => {}
                }
                if print && chars & HUB_CHAR_PORTIND == 0 {
                    eprintln!(" WARN: Port indicators are NOT supported.");
                }
            }
            r => {
                let e = match r {
                    Err(e) => e,
                    Ok(_) => std::io::Error::last_os_error(),
                };
                eprintln!("Can't get hub descriptor: {}", e);
                continue;
            }
        }
        let nport = buf[2] as usize;
        if verbose {
            hub_port_status(&uh, nport);
        }
        hubs.push(HubInfo { busnum: dev.busnum, devnum: dev.devnum, dev, nport });
    }
    hubs
}

fn exit_with_usage(progname: &str) -> i32 {
    eprintln!("Usage: {} [{{-h HUBNUM | -b BUSNUM -d DEVNUM}}] \\\n          [-P PORT] [{{-p [VALUE]|-l [VALUE]}}]", progname);
    1
}

/// atoi()
fn atoi(s: &str) -> i64 {
    let s = s.trim_start();
    let (neg, digits) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s.strip_prefix('+').unwrap_or(s)),
    };
    let n: i64 = digits.chars().take_while(|c| c.is_ascii_digit()).fold(0, |a, c| a * 10 + (c as i64 - '0' as i64));
    if neg { -n } else { n }
}

/// the hub-ctrl program: argv without the program name; the exit status
pub fn main(progname: &str, argv: &[String]) -> i32 {
    let (mut busnum, mut devnum) = (0, 0);
    let mut cmd = COMMAND_SET_NONE;
    let mut port = 1;
    let mut value = 0;
    let mut listing = argv.is_empty();
    let mut verbose = false;
    let mut hub: i64 = -1;
    let mut i = 0;
    while i < argv.len() {
        let a = &argv[i];
        if !a.starts_with('-') {
            return exit_with_usage(progname);
        }
        match a.chars().nth(1) {
            Some('h') => {
                i += 1;
                if i >= argv.len() || busnum > 0 || devnum > 0 {
                    return exit_with_usage(progname);
                }
                hub = atoi(&argv[i]);
            }
            Some('b') => {
                i += 1;
                if i >= argv.len() || hub >= 0 {
                    return exit_with_usage(progname);
                }
                busnum = atoi(&argv[i]);
            }
            Some('d') => {
                i += 1;
                if i >= argv.len() || hub >= 0 {
                    return exit_with_usage(progname);
                }
                devnum = atoi(&argv[i]);
            }
            Some('P') => {
                i += 1;
                if i >= argv.len() {
                    return exit_with_usage(progname);
                }
                port = atoi(&argv[i]);
            }
            Some('l') => {
                if cmd != COMMAND_SET_NONE {
                    return exit_with_usage(progname);
                }
                i += 1;
                value = if i < argv.len() { atoi(&argv[i]) } else { HUB_LED_GREEN };
                cmd = COMMAND_SET_LED;
            }
            Some('p') => {
                if cmd != COMMAND_SET_NONE {
                    return exit_with_usage(progname);
                }
                i += 1;
                value = if i < argv.len() { atoi(&argv[i]) } else { 0 };
                cmd = COMMAND_SET_POWER;
            }
            Some('v') => {
                verbose = true;
                if argv.len() == 1 {
                    listing = true;
                }
            }
            _ => return exit_with_usage(progname),
        }
        i += 1;
    }
    if (busnum > 0 && devnum <= 0) || (busnum <= 0 && devnum > 0) {
        return exit_with_usage(progname);
    }
    // Default is the hub #0
    if hub < 0 && busnum == 0 {
        hub = 0;
    }
    // Default is POWER
    if cmd == COMMAND_SET_NONE {
        cmd = COMMAND_SET_POWER;
    }
    let hubs = usb_find_hubs(listing, verbose, busnum, devnum, hub);
    if hubs.is_empty() {
        eprintln!("No hubs found.");
        return 1;
    }
    if listing {
        return 0;
    }
    if hub < 0 {
        hub = hubs.iter().position(|h| h.busnum == busnum && h.devnum == devnum).map(|i| i as i64).unwrap_or(-1);
    }
    let uh = if hub >= 0 && (hub as usize) < hubs.len() { hubs[hub as usize].dev.open() } else { None };
    let Some(uh) = uh else {
        eprintln!("Device not found.");
        return 1;
    };
    let (request, feature, index) = if cmd == COMMAND_SET_POWER {
        if value != 0 {
            (USB_REQ_SET_FEATURE, USB_PORT_FEAT_POWER, port)
        } else {
            (USB_REQ_CLEAR_FEATURE, USB_PORT_FEAT_POWER, port)
        }
    } else {
        (USB_REQ_SET_FEATURE, USB_PORT_FEAT_INDICATOR, (value << 8) | port)
    };
    if verbose {
        println!("Send control message (REQUEST={}, FEATURE={}, INDEX={})", request, feature, index);
    }
    let mut result = 0;
    if let Err(e) = uh.control_msg(USB_RT_PORT, request, feature, index as u16, &mut []) {
        eprintln!("failed to control.\n: {}", e);
        result = 1;
    }
    if verbose {
        hub_port_status(&uh, hubs[hub as usize].nport);
    }
    result
}

/// run_hub_ctrl(enable_power): `sudo <hub-ctrl> -h 0 -P 2 -p N` (python's
/// os.system: the outcome is not checked)
pub fn run_hub_ctrl(enable_power: i64) {
    let exe = std::env::current_exe().map(|p| p.to_string_lossy().into_owned()).unwrap_or_else(|_| "kaliru".into());
    let status = std::process::Command::new("sudo")
        .arg(&exe)
        .args(["--hub-ctrl", "-h", "0", "-P", "2", "-p", &enable_power.to_string()])
        .status();
    if let Err(e) = status {
        logging::info(&format!("hub-ctrl: {}", e));
    }
}
