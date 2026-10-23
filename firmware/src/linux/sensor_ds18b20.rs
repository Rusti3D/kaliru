// Communicate with a DS18B20 temperature sensor on linux
//
// Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::analog::atoi;
use super::console::report_errno;
use super::timer::{timer_disable_signals, timer_enable_signals};
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{irq_disable, irq_enable};
use crate::command::command_decode_ptr;
use crate::ctr::Global;
use crate::sched::{sched_add_timer, sched_check_wake, sched_del_timer, sched_wake_task, TaskWake, Timer, R, SF_RESCHEDULE};
use crate::{decl_command, decl_task, sendf, shutdown, try_shutdown};
use core::mem::size_of;
use std::ffi::CString;

const W1_READ_TIMEOUT_SEC: libc::time_t = 5;

// Status of a sensor
const W1_IDLE: u8 = 0; // No read requested yet
const W1_READ_REQUESTED: u8 = 1; // Reading or waiting to read
const W1_READY: u8 = 2; // Read complete, waiting to report
const W1_ERROR: u8 = 3; // Request shutdown

const TS_PENDING: u8 = 1;

#[repr(C)]
struct Ds18 {
    timer: Timer,
    rest_time: u32,
    min_value: i32,
    max_value: i32,
    flags: u8,

    // Set by main thread in configuration phase.
    // Should only be accessed by reader thread after configuration.
    fd: libc::c_int,

    // Used for guarding shared members.
    lock: libc::pthread_mutex_t,
    cond: libc::pthread_cond_t,

    // Protect all reads/writes to the following members using the mutex
    // once reader thread is initialized.
    temperature: libc::c_int,
    request_time: libc::timespec,
    status: u8,
    error_count: u8,
    max_error_count: u8,
    ignore_limits: u8,
}

fn oid_type() -> OidType {
    command_config_ds18b20 as *const ()
}

// Lock ds18_s mutex, set error status, unlock mutex.
unsafe fn locking_handle_read_error(d: *mut Ds18) {
    libc::pthread_mutex_lock(&mut (*d).lock);
    (*d).status = W1_ERROR;
    (*d).error_count = (*d).error_count.wrapping_add(1);
    if (*d).error_count <= (*d).max_error_count {
        libc::pthread_mutex_unlock(&mut (*d).lock);
    } else {
        libc::pthread_mutex_unlock(&mut (*d).lock);
        libc::pthread_exit(core::ptr::null_mut());
    }
}

// The kernel interface to DS18B20 sensors is a sysfs entry that blocks for
// around 750ms when read. Most of this is idle time waiting for the result
// to be ready. Read in a separate thread in order to avoid blocking time-
// sensitive work.
extern "C" fn reader_start_routine(param: *mut libc::c_void) -> *mut libc::c_void {
    let d = param as *mut Ds18;
    unsafe {
        loop {
            // Wait for requests to read temperature sensors
            libc::pthread_mutex_lock(&mut (*d).lock);
            while (*d).status != W1_READ_REQUESTED {
                libc::pthread_cond_wait(&mut (*d).cond, &mut (*d).lock);
            }
            libc::pthread_mutex_unlock(&mut (*d).lock);

            // Read temp.
            // The temperature data is at the end of the report, after a "t=".
            // Example (3.062 degrees C):
            //
            // 31 00 4b 46 7f ff 0c 10 77 : crc=77 YES
            // 31 00 4b 46 7f ff 0c 10 77 t=3062
            let mut data = [0u8; 128];
            let ret = libc::read((*d).fd, data.as_mut_ptr() as *mut libc::c_void, data.len() - 1);
            if ret < 0 {
                report_errno("read DS18B20", ret as i32);
                locking_handle_read_error(d);
                continue;
            }
            let text = &data[..ret as usize];
            // strstr stops at a nul
            let text = &text[..text.iter().position(|&c| c == 0).unwrap_or(text.len())];
            let temp_string = text.windows(2).position(|w| w == b"t=");
            let Some(pos) = temp_string.filter(|&p| p + 2 < text.len()) else {
                locking_handle_read_error(d);
                continue;
            };
            // Don't pass 't' and '=' to atoi
            let val = atoi(&text[pos + 2..]);

            // Store temperature
            libc::pthread_mutex_lock(&mut (*d).lock);
            (*d).status = W1_READY;
            (*d).temperature = val;
            libc::pthread_mutex_unlock(&mut (*d).lock);

            // Seek file in preparation of next read
            let ret = libc::lseek((*d).fd, 0, libc::SEEK_SET);
            if ret < 0 {
                report_errno("seek DS18B20", ret as i32);
                locking_handle_read_error(d);
                continue;
            }
        }
    }
}

static DS18_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

unsafe fn ds18_event(timer: *mut Timer) -> R<u8> {
    let d = &mut *(timer as *mut Ds18);
    // Trigger task to read and send results
    sched_wake_task(DS18_WAKE.ptr());
    d.flags |= TS_PENDING;
    d.timer.waketime = d.timer.waketime.wrapping_add(d.rest_time);
    Ok(SF_RESCHEDULE)
}

pub fn command_config_ds18b20(args: &[u32]) -> R {
    // Open kernel port
    let serial_len = args[1] as u8 as usize;
    let serial = unsafe { core::slice::from_raw_parts(command_decode_ptr(args[2]), serial_len) };
    // the goto labels of the C version: which failure
    let fail = 'fail: {
        if serial.contains(&b'/') {
            break 'fail 1;
        }
        // snprintf(fname, 56, ...): at most 55 bytes
        let mut fname = b"/sys/bus/w1/devices/".to_vec();
        fname.extend_from_slice(&serial[..serial.iter().position(|&c| c == 0).unwrap_or(serial.len())]);
        fname.extend_from_slice(b"/w1_slave");
        fname.truncate(55);
        let fname = CString::new(fname).unwrap();
        let fd = unsafe { libc::open(fname.as_ptr(), libc::O_RDONLY | libc::O_CLOEXEC) };
        if fd < 0 {
            report_errno("open ds18", fd);
            break 'fail 2;
        }

        let dp = oid_alloc(args[0] as u8, oid_type(), size_of::<Ds18>() as u16)? as *mut Ds18;
        let d = unsafe { &mut *dp };
        d.max_error_count = args[3] as u8;
        d.error_count = 0;
        d.ignore_limits = args[4] as u8;
        d.timer.func = Some(ds18_event);
        d.fd = fd;
        d.status = W1_IDLE;
        let ret = unsafe { libc::pthread_mutex_init(&mut d.lock, core::ptr::null()) };
        if ret != 0 {
            break 'fail 3;
        }
        let ret = unsafe { libc::pthread_cond_init(&mut d.cond, core::ptr::null()) };
        if ret != 0 {
            break 'fail 4;
        }

        let mut reader_tid: libc::pthread_t = 0; // Not used
        timer_disable_signals();
        let ret = unsafe { libc::pthread_create(&mut reader_tid, core::ptr::null(), reader_start_routine, dp as *mut libc::c_void) };
        timer_enable_signals();
        if ret != 0 {
            break 'fail 5;
        }

        return Ok(());
    };
    match fail {
        1 => shutdown!("Invalid DS18B20 serial id, must not contain '/'"),
        2 => shutdown!("Invalid DS18B20 serial id, could not open for reading"),
        3 => shutdown!("Could not start DS18B20 reader thread (mutex init)"),
        4 => shutdown!("Could not start DS18B20 reader thread (cond init)"),
        _ => shutdown!("Could not start DS18B20 reader thread"),
    }
}
decl_command!(command_config_ds18b20, "config_ds18b20 oid=%c serial=%*s max_error_count=%c ignore_limits=%c");

pub fn command_query_ds18b20(args: &[u32]) -> R {
    let d = unsafe { &mut *(oid_lookup(args[0] as u8, oid_type())? as *mut Ds18) };

    sched_del_timer(&mut d.timer);
    d.timer.waketime = args[1];
    d.rest_time = args[2];
    if d.rest_time == 0 {
        return Ok(());
    }
    d.min_value = args[3] as i32;
    d.max_value = args[4] as i32;
    sched_add_timer(&mut d.timer)
}
decl_command!(
    command_query_ds18b20,
    concat!("query_ds18b20 oid=%c clock=%u rest_ticks=%u", " min_value=%i max_value=%i")
);

// Report temperature if ready, and set back to pending.
fn ds18_send_and_request(d: &mut Ds18, next_begin_time: u32, oid: u8) -> R {
    let mut request_time = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    let ret = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut request_time) };
    if ret == -1 {
        report_errno("get monotonic clock time", ret);
        try_shutdown!("Error getting monotonic clock time");
        return Ok(());
    }

    unsafe { libc::pthread_mutex_lock(&mut d.lock) };
    if d.status == W1_ERROR {
        if d.error_count > d.max_error_count {
            try_shutdown!("Error reading DS18B20 sensor");
            unsafe { libc::pthread_mutex_unlock(&mut d.lock) };
            return Ok(());
        } else {
            sendf!("ds18b20_result oid=%c next_clock=%u value=%i fault=%u", oid, next_begin_time, d.temperature, d.error_count)?;
            d.status = W1_READ_REQUESTED;
        }
    } else if d.status == W1_IDLE {
        // This happens the first time requesting a temperature.
        // Nothing to report yet.
        d.request_time = request_time;
        d.status = W1_READ_REQUESTED;
    } else if d.status == W1_READY {
        // Report the previous temperature and request a new one.
        sendf!("ds18b20_result oid=%c next_clock=%u value=%i fault=%u", oid, next_begin_time, d.temperature, 0u32)?;
        if d.ignore_limits == 0 && (d.temperature < d.min_value || d.temperature > d.max_value) {
            unsafe { libc::pthread_mutex_unlock(&mut d.lock) };
            try_shutdown!("DS18B20 out of range");
            return Ok(());
        }
        d.request_time = request_time;
        d.status = W1_READ_REQUESTED;
        d.error_count = 0; //successful reading, reset error count
    } else if d.status == W1_READ_REQUESTED {
        // Reader thread is already reading (or will be soon).
        // This can happen if two queries come in quick enough
        // succession. Wait for the existing read to finish.
        // This could also happen if the reader thread has hung. In that case,
        // shut down the MCU. To tell the difference, see if the request time
        // is too far in the past.
        if request_time.tv_sec - d.request_time.tv_sec > W1_READ_TIMEOUT_SEC {
            unsafe { libc::pthread_mutex_unlock(&mut d.lock) };
            try_shutdown!("DS18B20 sensor didn't respond in time");
            return Ok(());
        }
    }
    unsafe {
        libc::pthread_cond_signal(&mut d.cond);
        libc::pthread_mutex_unlock(&mut d.lock);
    }
    Ok(())
}

// task to read temperature and send response
pub fn ds18_task() -> R {
    if !sched_check_wake(DS18_WAKE.ptr()) {
        return Ok(());
    }
    let mut oid: u8 = 255;
    loop {
        let p = crate::basecmd::oid_next(&mut oid, oid_type());
        if p.is_null() {
            break;
        }
        let d = unsafe { &mut *(p as *mut Ds18) };
        if d.flags & TS_PENDING == 0 {
            continue;
        }
        irq_disable();
        let next_begin_time = d.timer.waketime;
        d.flags &= !TS_PENDING;
        irq_enable();
        ds18_send_and_request(d, next_begin_time, oid)?;
    }
    Ok(())
}
decl_task!(ds18_task);
