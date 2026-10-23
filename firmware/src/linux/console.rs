// TTY based IO
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::command::{command_encode_and_frame, command_find_and_dispatch, CommandEncoder, SendArg, MESSAGE_MAX};
use crate::ctr::Global;
use crate::sched::{sched_check_wake, sched_wake_task, TaskWake, R};
use crate::{decl_task, shutdown};
use std::ffi::{CStr, CString};

static MAIN_PFD: Global<[libc::pollfd; 1]> = Global::new([libc::pollfd { fd: 0, events: 0, revents: 0 }]);
const MP_TTY_IDX: usize = 0;

fn errno() -> libc::c_int {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

// Report 'errno' in a message written to stderr
pub fn report_errno(where_: &str, rc: i32) {
    let e = errno();
    let msg = unsafe { CStr::from_ptr(libc::strerror(e)) }.to_string_lossy();
    let line = format!("Got error {} in {}: ({}){}\n", rc, where_, e, msg);
    unsafe { libc::write(libc::STDERR_FILENO, line.as_ptr() as *const libc::c_void, line.len()) };
}

/****************************************************************
 * Setup
 ****************************************************************/

pub fn set_non_blocking(fd: libc::c_int) -> i32 {
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    if flags < 0 {
        report_errno("fcntl getfl", flags);
        return -1;
    }
    let ret = unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) };
    if ret < 0 {
        report_errno("fcntl setfl", flags);
        return -1;
    }
    0
}

pub fn set_close_on_exec(fd: libc::c_int) -> i32 {
    let ret = unsafe { libc::fcntl(fd, libc::F_SETFD, libc::FD_CLOEXEC) };
    if ret < 0 {
        report_errno("fcntl set cloexec", ret);
        return -1;
    }
    0
}

pub fn console_setup(name: &str, mode: libc::mode_t, group: libc::gid_t) -> i32 {
    unsafe {
        // Open pseudo-tty
        let mut ti: libc::termios = core::mem::zeroed();
        let (mut mfd, mut sfd) = (0, 0);
        let ret = libc::openpty(&mut mfd, &mut sfd, core::ptr::null_mut(), &mut ti, core::ptr::null_mut());
        if ret != 0 {
            report_errno("openpty", ret);
            return -1;
        }
        if set_non_blocking(mfd) != 0 || set_close_on_exec(mfd) != 0 || set_close_on_exec(sfd) != 0 {
            return -1;
        }
        let pfd = MAIN_PFD.get();
        pfd[MP_TTY_IDX].fd = mfd;
        pfd[MP_TTY_IDX].events = libc::POLLIN;

        // Create symlink to tty
        let cname = CString::new(name).unwrap();
        libc::unlink(cname.as_ptr());
        let tname = libc::ttyname(sfd);
        if tname.is_null() {
            report_errno("ttyname", 0);
            return -1;
        }
        let ret = libc::symlink(tname, cname.as_ptr());
        if ret != 0 {
            report_errno("symlink", ret);
            return -1;
        }
        let ret = libc::chmod(tname, mode);
        if ret != 0 {
            report_errno("chmod", ret);
            return -1;
        }

        if group != libc::gid_t::MAX {
            let ret = libc::chown(tname, libc::uid_t::MAX, group);
            if ret != 0 {
                report_errno("chgrp", ret);
                return -1;
            }
        }

        // Make sure stderr is non-blocking
        if set_non_blocking(libc::STDERR_FILENO) != 0 {
            return -1;
        }
    }
    0
}

/****************************************************************
 * Console handling
 ****************************************************************/

static CONSOLE_WAKE: Global<TaskWake> = Global::new(TaskWake::new());
static RECEIVE_BUF: Global<[u8; 4096]> = Global::new([0; 4096]);
static RECEIVE_POS: Global<usize> = Global::new(0);

pub fn console_receive_buffer() -> *mut u8 {
    RECEIVE_BUF.ptr() as *mut u8
}

// Process any incoming commands
pub fn console_task() -> R {
    if !sched_check_wake(CONSOLE_WAKE.ptr()) {
        return Ok(());
    }

    // Read data
    let buf = RECEIVE_BUF.get();
    let receive_pos = *RECEIVE_POS.get();
    let mut ret = unsafe {
        libc::read(
            MAIN_PFD.get()[MP_TTY_IDX].fd,
            buf[receive_pos..].as_mut_ptr() as *mut libc::c_void,
            buf.len() - receive_pos,
        )
    };
    if ret < 0 {
        if errno() == libc::EWOULDBLOCK {
            ret = 0;
        } else {
            report_errno("read", ret as i32);
            return Ok(());
        }
    }
    let ret = ret as usize;
    if ret == 15 && buf[receive_pos + 14] == b'\n' && &buf[receive_pos..receive_pos + 15] == b"FORCE_SHUTDOWN\n" {
        shutdown!("Force shutdown command");
    }

    // Find and dispatch message blocks in the input
    let mut len = receive_pos + ret;
    let msglen = len.min(MESSAGE_MAX);
    let mut pop_count = 0;
    let ret = command_find_and_dispatch(&buf[..msglen], &mut pop_count)?;
    if ret != 0 {
        len -= pop_count;
        if len != 0 {
            buf.copy_within(pop_count..pop_count + len, 0);
            sched_wake_task(CONSOLE_WAKE.ptr());
        }
    }
    *RECEIVE_POS.get() = len;
    Ok(())
}
decl_task!(console_task);

// Encode and transmit a "response" message
pub fn console_sendf(ce: &CommandEncoder, args: &[SendArg]) -> R {
    // Generate message
    let mut buf = [0u8; MESSAGE_MAX];
    let msglen = command_encode_and_frame(&mut buf, ce, args)?;

    // Transmit message
    let ret = unsafe { libc::write(MAIN_PFD.get()[MP_TTY_IDX].fd, buf.as_ptr() as *const libc::c_void, msglen) };
    if ret < 0 {
        report_errno("write", ret as i32);
    }
    Ok(())
}

// Sleep until a signal received (waking early for console input if needed)
pub fn console_sleep(sigset: &libc::sigset_t) {
    let pfd = MAIN_PFD.get();
    let ret = unsafe { libc::ppoll(pfd.as_mut_ptr(), pfd.len() as libc::nfds_t, core::ptr::null(), sigset) };
    if ret <= 0 {
        if errno() != libc::EINTR {
            report_errno("ppoll main_pfd", ret);
        }
        return;
    }
    if pfd[MP_TTY_IDX].revents != 0 {
        sched_wake_task(CONSOLE_WAKE.ptr());
    }
}
