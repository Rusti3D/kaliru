// Code for dispatching timer and file descriptor events
// (port of klippy/chelper/pollreactor.c)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The C version stores function pointers per fd / timer slot; here the
// owner implements PrHandler and gets the slot number.
use super::pyhelper::{get_monotonic, report_errno};
use std::os::fd::RawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub const PR_NOW: f64 = 0.;
pub const PR_NEVER: f64 = 9999999999999999.;

pub trait PrHandler {
    fn fd_event(&mut self, pr: &mut PollReactor, pos: usize, eventtime: f64);
    fn timer_event(&mut self, pr: &mut PollReactor, pos: usize, eventtime: f64) -> f64;
}

pub struct PollReactor {
    fds: Vec<libc::pollfd>,
    timers: Vec<f64>,
    next_timer: f64,
    // set from other threads (pollreactor_do_exit)
    pub must_exit: Arc<AtomicBool>,
}

impl PollReactor {
    // Allocate a new 'struct pollreactor' object
    pub fn new(num_fds: usize, num_timers: usize) -> Self {
        PollReactor {
            fds: vec![libc::pollfd { fd: 0, events: 0, revents: 0 }; num_fds],
            timers: vec![PR_NEVER; num_timers],
            next_timer: PR_NEVER,
            must_exit: Arc::new(AtomicBool::new(false)),
        }
    }

    // Add a callback for when a file descriptor (fd) becomes readable
    pub fn add_fd(&mut self, pos: usize, fd: RawFd, write_only: bool) {
        self.fds[pos] = libc::pollfd { fd, events: libc::POLLHUP | if write_only { 0 } else { libc::POLLIN }, revents: 0 };
    }

    // Add a timer callback
    pub fn add_timer(&mut self, pos: usize) {
        self.timers[pos] = PR_NEVER;
    }

    // Return the last schedule wake-up time for a timer
    pub fn get_timer(&self, pos: usize) -> f64 {
        self.timers[pos]
    }

    // Set the wake-up time for a given timer
    pub fn update_timer(&mut self, pos: usize, waketime: f64) {
        self.timers[pos] = waketime;
        if waketime < self.next_timer {
            self.next_timer = waketime;
        }
    }

    // Internal code to invoke timer callbacks
    fn check_timers(&mut self, h: &mut dyn PrHandler, eventtime: f64, mut busy: bool) -> i32 {
        if eventtime >= self.next_timer {
            // Find and run pending timers
            self.next_timer = PR_NEVER;
            for i in 0..self.timers.len() {
                let mut t = self.timers[i];
                if eventtime >= t {
                    busy = true;
                    t = h.timer_event(self, i, eventtime);
                    self.timers[i] = t;
                }
                if t < self.next_timer {
                    self.next_timer = t;
                }
            }
        }
        if busy {
            return 0;
        }
        // Calculate sleep duration
        let timeout = ((self.next_timer - eventtime) * 1000.).ceil();
        if timeout < 1. {
            1
        } else if timeout > 1000. {
            1000
        } else {
            timeout as i32
        }
    }

    // Repeatedly check for timer and fd events and invoke their callbacks
    pub fn run(&mut self, h: &mut dyn PrHandler) {
        let mut eventtime = get_monotonic();
        let mut busy = true;
        while !self.is_exit() {
            let timeout = self.check_timers(h, eventtime, busy);
            busy = false;
            let ret = unsafe { libc::poll(self.fds.as_mut_ptr(), self.fds.len() as libc::nfds_t, timeout) };
            eventtime = get_monotonic();
            if ret > 0 {
                busy = true;
                for i in 0..self.fds.len() {
                    if self.fds[i].revents != 0 {
                        h.fd_event(self, i, eventtime);
                    }
                }
            } else if ret < 0 {
                report_errno("poll", ret);
                self.do_exit();
            }
        }
    }

    // Request that a currently running pollreactor_run() loop exit
    pub fn do_exit(&self) {
        self.must_exit.store(true, Ordering::SeqCst);
    }

    // Check if a pollreactor_run() loop has been requested to exit
    pub fn is_exit(&self) -> bool {
        self.must_exit.load(Ordering::SeqCst)
    }
}

pub fn fd_set_non_blocking(fd: RawFd) -> i32 {
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
