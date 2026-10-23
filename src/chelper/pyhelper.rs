// Helper functions shared by the chelper ports (port of
// klippy/chelper/pyhelper.c)
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use std::sync::RwLock;

// Return the monotonic system time as a double
pub fn get_monotonic() -> f64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    let ret = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut ts) };
    if ret != 0 {
        report_errno("clock_gettime", ret);
        return 0.;
    }
    ts.tv_sec as f64 + ts.tv_nsec as f64 * 0.000000001
}

// Fill a 'struct timespec' with a system time stored in a double
pub fn fill_time(time: f64) -> libc::timespec {
    let t = time as libc::time_t;
    libc::timespec { tv_sec: t, tv_nsec: ((time - t as f64) * 1000000000.) as _ }
}

fn default_logger(msg: &str) {
    eprintln!("{}", msg);
}

static LOGGING_CALLBACK: RwLock<fn(&str)> = RwLock::new(default_logger);

// klippy points this at logging.error (chelper/__init__.py get_ffi)
pub fn set_python_logging_callback(func: fn(&str)) {
    *LOGGING_CALLBACK.write().unwrap() = func;
}

// Log an error message (the C version truncates to 511 bytes)
pub fn errorf(msg: &str) {
    let mut end = msg.len().min(511);
    while !msg.is_char_boundary(end) {
        end -= 1;
    }
    (LOGGING_CALLBACK.read().unwrap())(&msg[..end]);
}

// Report 'errno' in a message written to stderr
pub fn report_errno(where_: &str, rc: i32) {
    let e = std::io::Error::last_os_error();
    let errno = e.raw_os_error().unwrap_or(0);
    let msg = unsafe { std::ffi::CStr::from_ptr(libc::strerror(errno)) }.to_string_lossy().into_owned();
    errorf(&format!("Got error {} in {}: ({}){}", rc, where_, errno, msg));
}

// Translate a binary string into an ASCII string with escape sequences
// (outbuf_size as in the C callers: output stops 5 bytes before the end)
pub fn dump_string(outbuf_size: usize, inbuf: &[u8]) -> String {
    let outend = outbuf_size.saturating_sub(5);
    let mut o = String::new();
    for &c in inbuf {
        if o.len() >= outend {
            break;
        }
        if c > 31 && c < 127 && c != b'\\' {
            o.push(c as char);
            continue;
        }
        o.push_str(&format!("\\x{:x}{:x}", c >> 4, c & 0x0f));
    }
    o
}

// Set custom thread names
pub fn set_thread_name(name: &str) -> i32 {
    let mut buf = [0u8; 16];
    let n = name.len().min(15);
    buf[..n].copy_from_slice(&name.as_bytes()[..n]);
    unsafe { libc::prctl(libc::PR_SET_NAME, buf.as_ptr()) }
}
