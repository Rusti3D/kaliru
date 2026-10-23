// Main starting point for micro-controller code running on linux systems
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::{console_setup, report_errno};
use super::watchdog::watchdog_setup;
use crate::ctr::Global;
use crate::sched::{sched_is_shutdown, sched_main, R};
use crate::{decl_command, decl_constant_str, shutdown};
use std::ffi::{CStr, CString};

decl_constant_str!("MCU", "linux");

/****************************************************************
 * Real-time setup
 ****************************************************************/

fn realtime_setup() -> i32 {
    unsafe {
        let mut sp: libc::sched_param = core::mem::zeroed();
        sp.sched_priority = libc::sched_get_priority_max(libc::SCHED_FIFO) / 2;
        let ret = libc::sched_setscheduler(0, libc::SCHED_FIFO, &sp);
        if ret < 0 {
            report_errno("sched_setscheduler", ret);
            return -1;
        }
        // Lock ourselves into memory
        let ret = libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE);
        if ret != 0 {
            report_errno("mlockall", ret);
            return -1;
        }
    }
    0
}

/****************************************************************
 * Restart
 ****************************************************************/

static ORIG_ARGV: Global<Vec<*mut libc::c_char>> = Global::new(Vec::new());

pub fn command_config_reset(_args: &[u32]) -> R {
    if !sched_is_shutdown() {
        shutdown!("config_reset only available when shutdown");
    }
    let argv = ORIG_ARGV.get();
    let ret = unsafe { libc::execv(argv[0], argv.as_ptr() as *const *const libc::c_char) };
    report_errno("execv", ret);
    Ok(())
}
decl_command!(command_config_reset, HF_IN_SHUTDOWN, "config_reset");

/****************************************************************
 * Startup
 ****************************************************************/

fn eprint(s: &str) {
    unsafe { libc::write(libc::STDERR_FILENO, s.as_ptr() as *const libc::c_void, s.len()) };
}

pub fn main() -> i32 {
    // Parse program args
    let args: Vec<CString> = std::env::args_os()
        .map(|a| CString::new(std::os::unix::ffi::OsStrExt::as_bytes(a.as_os_str())).unwrap())
        .collect();
    let argv = ORIG_ARGV.get();
    *argv = args.iter().map(|a| a.as_ptr() as *mut libc::c_char).collect();
    argv.push(core::ptr::null_mut());
    // the C strings live for the whole program
    std::mem::forget(args);
    let argc = (argv.len() - 1) as libc::c_int;
    let (mut watchdog, mut realtime) = (false, false);
    let mut serial = "/tmp/klipper_host_mcu".to_string();
    let mut pty_group_name: Option<String> = None;
    let mut pty_mode: libc::mode_t = 0o660;
    let optstring = CString::new("wrI:g:m:").unwrap();
    loop {
        let opt = unsafe { libc::getopt(argc, argv.as_ptr(), optstring.as_ptr()) };
        if opt == -1 {
            break;
        }
        let optarg = || unsafe { CStr::from_ptr(optarg_ptr()) }.to_string_lossy().into_owned();
        match opt as u8 {
            b'w' => watchdog = true,
            b'r' => realtime = true,
            b'I' => serial = optarg(),
            b'g' => pty_group_name = Some(optarg()),
            b'm' => {
                let a = optarg();
                // strtoul(optarg, NULL, 8): the leading octal digits
                let digits: String = a.trim_start().chars().take_while(|c| ('0'..='7').contains(c)).collect();
                let Ok(input_pty_mode) = u64::from_str_radix(if digits.is_empty() { "0" } else { &digits }, 8) else {
                    eprint(&format!("unable to parse pty mode: {}", a));
                    return -1;
                };
                if input_pty_mode > 0o777 {
                    eprint(&format!("mode {} invalid (setuid/gid/sticky not supported)", a));
                    return -1;
                }
                pty_mode = input_pty_mode as libc::mode_t;
            }
            _ => {
                let prog = unsafe { CStr::from_ptr(argv[0]) }.to_string_lossy().into_owned();
                eprint(&format!("Usage: {} [-w] [-r] [-I path] [-g group] [-m filemode]\n", prog));
                return -1;
            }
        }
    }

    // Initial setup
    if realtime {
        let ret = realtime_setup();
        if ret != 0 {
            return ret;
        }
    }
    let mut pty_group_id = libc::gid_t::MAX;
    if let Some(name) = pty_group_name {
        let cname = CString::new(name.clone()).unwrap();
        let pty_group = unsafe { libc::getgrnam(cname.as_ptr()) };
        if pty_group.is_null() {
            eprint(&format!("Unable to find group \"{}\"\n", name));
            return -1;
        }
        pty_group_id = unsafe { (*pty_group).gr_gid };
    }

    if console_setup(&serial, pty_mode, pty_group_id) != 0 {
        return -1;
    }
    if watchdog {
        let ret = watchdog_setup();
        if ret != 0 {
            return ret;
        }
    }

    // Main loop
    sched_main()
}

extern "C" {
    #[link_name = "optarg"]
    static mut C_OPTARG: *mut libc::c_char;
}

fn optarg_ptr() -> *mut libc::c_char {
    unsafe { *core::ptr::addr_of!(C_OPTARG) }
}
