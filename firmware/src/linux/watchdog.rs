// Support for Linux watchdog
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::{report_errno, set_non_blocking};
use super::timer::timer_check_periodic;
use crate::ctr::Global;
use crate::decl_task;
use crate::sched::R;

static WATCHDOG_FD: Global<libc::c_int> = Global::new(-1);

pub fn watchdog_setup() -> i32 {
    let ret = unsafe { libc::open(c"/dev/watchdog".as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
    if ret < 0 {
        report_errno("watchdog open", ret);
        return -1;
    }
    *WATCHDOG_FD.get() = ret;
    set_non_blocking(ret)
}

static NEXT_WATCHDOG_TIME: Global<u32> = Global::new(0);

pub fn watchdog_task() -> R {
    let fd = *WATCHDOG_FD.get();
    if fd < 0 || !timer_check_periodic(NEXT_WATCHDOG_TIME.get()) {
        return Ok(());
    }
    let ret = unsafe { libc::write(fd, b".".as_ptr() as *const libc::c_void, 1) };
    if ret <= 0 {
        report_errno("watchdog write", ret as i32);
    }
    Ok(())
}
decl_task!(watchdog_task);
