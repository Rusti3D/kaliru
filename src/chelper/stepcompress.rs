// Stepper pulse schedule compression (port of klippy/chelper/stepcompress.c)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The goal of this code is to take a series of scheduled stepper
// pulse times and compress them into a handful of commands that can
// be efficiently transmitted and executed on a microcontroller (mcu).
// The mcu accepts step pulse commands that take interval, count, and
// add parameters such that 'count' pulses occur, with each step event
// calculating the next step event time using:
//  next_wake_time = last_wake_time + interval; interval += add
//
// The step time buffer keeps C's pointer arithmetic as indices into a Vec
// whose length is the C allocation size. The history list is newest first.
use super::errorf;
use super::msgblock::{message_alloc_and_encode, QueueMessage};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

pub const ERROR_RET: i32 = -989898989;

const CHECK_LINES: bool = true;
const QUEUE_START_SIZE: usize = 1024;

/// a syncemitter's transmit message queue (shared with its stepcompress)
pub type MsgQueue = Rc<RefCell<VecDeque<QueueMessage>>>;

/// struct pull_history_steps
#[derive(Clone, Copy, Debug, Default)]
pub struct PullHistorySteps {
    pub first_clock: u64,
    pub last_clock: u64,
    pub start_position: i64,
    pub step_count: i32,
    pub interval: i32,
    pub add: i32,
}

#[derive(Clone, Copy, Debug, Default)]
struct HistorySteps {
    first_clock: u64,
    last_clock: u64,
    start_position: i64,
    step_count: i32,
    interval: i32,
    add: i32,
}

#[derive(Clone, Copy, Debug)]
struct StepMove {
    interval: u32,
    count: u16,
    add: i16,
}

pub struct StepCompress {
    // Buffer management
    queue: Vec<u32>,
    queue_pos: usize,
    queue_next: usize,
    // Internal tracking
    max_error: u32,
    mcu_time_offset: f64,
    mcu_freq: f64,
    last_step_print_time: f64,
    // Message generation
    last_step_clock: u64,
    msg_queue: MsgQueue,
    oid: u32,
    queue_step_msgtag: i32,
    set_next_step_dir_msgtag: i32,
    sdir: i32,
    invert_sdir: i32,
    // Step+dir+step filter
    next_step_clock: u64,
    next_step_dir: i32,
    // History tracking
    last_position: i64,
    history_list: VecDeque<HistorySteps>,
}

/****************************************************************
 * Step compression
 ****************************************************************/

#[inline]
fn idiv_up(n: i32, d: i32) -> i32 {
    if n >= 0 {
        (n + d - 1) / d
    } else {
        n / d
    }
}

#[inline]
fn idiv_down(n: i32, d: i32) -> i32 {
    if n >= 0 {
        n / d
    } else {
        (n - d + 1) / d
    }
}

#[derive(Clone, Copy)]
struct Points {
    minp: i32,
    maxp: i32,
}

// The maximum add delta between two valid quadratic sequences of the
// form "add*count*(count-1)/2 + interval*count" is "(6 + 4*sqrt(2)) *
// maxerror / (count*count)".  The "6 + 4*sqrt(2)" is 11.65685, but
// using 11 works well in practice.
const QUADRATIC_DEV: i32 = 11;

// Maximium clock delta between messages in the queue
const CLOCK_DIFF_MAX: u64 = 3 << 28;

const SDS_FILTER_TIME: f64 = 0.000750;

impl StepCompress {
    // Allocate a new 'stepcompress' object
    pub fn alloc(msg_queue: MsgQueue) -> Self {
        StepCompress {
            queue: vec![],
            queue_pos: 0,
            queue_next: 0,
            max_error: 0,
            mcu_time_offset: 0.,
            mcu_freq: 0.,
            last_step_print_time: 0.,
            last_step_clock: 0,
            msg_queue,
            oid: 0,
            queue_step_msgtag: 0,
            set_next_step_dir_msgtag: 0,
            sdir: -1,
            invert_sdir: 0,
            next_step_clock: 0,
            next_step_dir: 0,
            last_position: 0,
            history_list: VecDeque::new(),
        }
    }

    // Given a requested step time, return the minimum and maximum
    // acceptable times
    #[inline]
    fn minmax_point(&self, pos: usize) -> Points {
        let lsc = self.last_step_clock as u32;
        let point = self.queue[pos].wrapping_sub(lsc);
        let prevpoint = if pos > self.queue_pos { self.queue[pos - 1].wrapping_sub(lsc) } else { 0 };
        let mut max_error = point.wrapping_sub(prevpoint) / 2;
        if max_error > self.max_error {
            max_error = self.max_error;
        }
        Points { minp: point.wrapping_sub(max_error) as i32, maxp: point as i32 }
    }

    // Find a 'step_move' that covers a series of step times
    fn compress_bisect_add(&self) -> StepMove {
        let mut qlast = self.queue_next;
        if qlast > self.queue_pos + 65535 {
            qlast = self.queue_pos + 65535;
        }
        let point = self.minmax_point(self.queue_pos);
        let (mut outer_mininterval, mut outer_maxinterval) = (point.minp, point.maxp);
        let (mut add, mut minadd, mut maxadd): (i32, i32, i32) = (0, -0x8000, 0x7fff);
        let (mut bestinterval, mut bestcount, mut bestadd, mut bestreach) = (0i32, 1i32, 1i32, i32::MIN);
        let (mut zerointerval, mut zerocount) = (0i32, 0i32);

        loop {
            // Find longest valid sequence with the given 'add'
            let mut nextpoint;
            let mut nextmininterval = outer_mininterval;
            let mut nextmaxinterval = outer_maxinterval;
            let mut interval = nextmaxinterval;
            let mut nextcount: i32 = 1;
            loop {
                nextcount += 1;
                if self.queue_pos + (nextcount - 1) as usize >= qlast {
                    let count = nextcount - 1;
                    return StepMove { interval: interval as u32, count: count as u16, add: add as i16 };
                }
                nextpoint = self.minmax_point(self.queue_pos + (nextcount - 1) as usize);
                let nextaddfactor = nextcount.wrapping_mul(nextcount - 1) / 2;
                let c = add.wrapping_mul(nextaddfactor);
                if nextmininterval.wrapping_mul(nextcount) < nextpoint.minp.wrapping_sub(c) {
                    nextmininterval = idiv_up(nextpoint.minp.wrapping_sub(c), nextcount);
                }
                if nextmaxinterval.wrapping_mul(nextcount) > nextpoint.maxp.wrapping_sub(c) {
                    nextmaxinterval = idiv_down(nextpoint.maxp.wrapping_sub(c), nextcount);
                }
                if nextmininterval > nextmaxinterval {
                    break;
                }
                interval = nextmaxinterval;
            }

            // Check if this is the best sequence found so far
            let count = nextcount - 1;
            let addfactor = count.wrapping_mul(count - 1) / 2;
            let reach = add.wrapping_mul(addfactor).wrapping_add(interval.wrapping_mul(count));
            if reach > bestreach || (reach == bestreach && interval > bestinterval) {
                bestinterval = interval;
                bestcount = count;
                bestadd = add;
                bestreach = reach;
                if add == 0 {
                    zerointerval = interval;
                    zerocount = count;
                }
                if count > 0x200 {
                    // No 'add' will improve sequence; avoid integer overflow
                    break;
                }
            }

            // Check if a greater or lesser add could extend the sequence
            let nextaddfactor = nextcount.wrapping_mul(nextcount - 1) / 2;
            let nextreach = add.wrapping_mul(nextaddfactor).wrapping_add(interval.wrapping_mul(nextcount));
            if nextreach < nextpoint.minp {
                minadd = add + 1;
                outer_maxinterval = nextmaxinterval;
            } else {
                maxadd = add - 1;
                outer_mininterval = nextmininterval;
            }

            // The maximum valid deviation between two quadratic sequences
            // can be calculated and used to further limit the add range.
            if count > 1 {
                // uint32_t max_error makes this an unsigned division
                let errdelta =
                    (self.max_error.wrapping_mul(QUADRATIC_DEV as u32) / count.wrapping_mul(count) as u32) as i32;
                if minadd < add - errdelta {
                    minadd = add - errdelta;
                }
                if maxadd > add + errdelta {
                    maxadd = add + errdelta;
                }
            }

            // See if next point would further limit the add range
            let c = outer_maxinterval.wrapping_mul(nextcount);
            if minadd.wrapping_mul(nextaddfactor) < nextpoint.minp.wrapping_sub(c) {
                minadd = idiv_up(nextpoint.minp.wrapping_sub(c), nextaddfactor);
            }
            let c = outer_mininterval.wrapping_mul(nextcount);
            if maxadd.wrapping_mul(nextaddfactor) > nextpoint.maxp.wrapping_sub(c) {
                maxadd = idiv_down(nextpoint.maxp.wrapping_sub(c), nextaddfactor);
            }

            // Bisect valid add range and try again with new 'add'
            if minadd > maxadd {
                break;
            }
            add = maxadd - (maxadd - minadd) / 4;
        }
        if zerocount + zerocount / 16 >= bestcount {
            // Prefer add=0 if it's similar to the best found sequence
            return StepMove { interval: zerointerval as u32, count: zerocount as u16, add: 0 };
        }
        StepMove { interval: bestinterval as u32, count: bestcount as u16, add: bestadd as i16 }
    }

    /****************************************************************
     * Step compress checking
     ****************************************************************/

    // Verify that a given 'step_move' matches the actual step times
    fn check_line(&self, mv: StepMove) -> i32 {
        if !CHECK_LINES {
            return 0;
        }
        if mv.count == 0 || (mv.interval == 0 && mv.add == 0 && mv.count > 1) || mv.interval >= 0x80000000 {
            errorf(&format!(
                "stepcompress o={} i={} c={} a={}: Invalid sequence",
                self.oid as i32, mv.interval as i32, mv.count, mv.add
            ));
            return ERROR_RET;
        }
        let mut interval = mv.interval;
        let mut p: u32 = 0;
        for i in 0..mv.count {
            let point = self.minmax_point(self.queue_pos + i as usize);
            p = p.wrapping_add(interval);
            if p_lt(p, point.minp) || p_gt(p, point.maxp) {
                errorf(&format!(
                    "stepcompress o={} i={} c={} a={}: Point {}: {} not in {}:{}",
                    self.oid as i32,
                    mv.interval as i32,
                    mv.count,
                    mv.add,
                    i + 1,
                    p as i32,
                    point.minp,
                    point.maxp
                ));
                return ERROR_RET;
            }
            if interval >= 0x80000000 {
                errorf(&format!(
                    "stepcompress o={} i={} c={} a={}: Point {}: interval overflow {}",
                    self.oid as i32,
                    mv.interval as i32,
                    mv.count,
                    mv.add,
                    i + 1,
                    interval as i32
                ));
                return ERROR_RET;
            }
            interval = interval.wrapping_add(mv.add as i32 as u32);
        }
        0
    }

    /****************************************************************
     * Step compress interface
     ****************************************************************/

    // Fill message id information
    pub fn fill(&mut self, oid: u32, max_error: u32, queue_step_msgtag: i32, set_next_step_dir_msgtag: i32) {
        self.oid = oid;
        self.max_error = max_error;
        self.queue_step_msgtag = queue_step_msgtag;
        self.set_next_step_dir_msgtag = set_next_step_dir_msgtag;
    }

    // Set the inverted stepper direction flag
    pub fn set_invert_sdir(&mut self, invert_sdir: u32) {
        let invert_sdir = (invert_sdir != 0) as i32;
        if invert_sdir != self.invert_sdir {
            self.invert_sdir = invert_sdir;
            if self.sdir >= 0 {
                self.sdir ^= 1;
            }
        }
    }

    // Expire the stepcompress history older than the given clock
    pub fn history_expire(&mut self, end_clock: u64) {
        while let Some(hs) = self.history_list.back() {
            if hs.last_clock > end_clock {
                break;
            }
            self.history_list.pop_back();
        }
    }

    pub fn get_oid(&self) -> u32 {
        self.oid
    }

    pub fn get_step_dir(&self) -> i32 {
        self.next_step_dir
    }

    // Determine the "print time" of the last_step_clock
    fn calc_last_step_print_time(&mut self) {
        let lsc = self.last_step_clock as f64;
        self.last_step_print_time = self.mcu_time_offset + (lsc - 0.5) / self.mcu_freq;
    }

    // Set the conversion rate of 'print_time' to mcu clock
    pub fn set_time(&mut self, time_offset: f64, mcu_freq: f64) {
        self.mcu_time_offset = time_offset;
        self.mcu_freq = mcu_freq;
        self.calc_last_step_print_time();
    }

    // Helper to create a queue_step command from a 'struct step_move'
    fn add_move(&mut self, first_clock: u64, mv: &StepMove) {
        let count = mv.count as i32;
        let addfactor = count * (count - 1) / 2;
        let ticks = ((mv.add as i32).wrapping_mul(addfactor) as u32).wrapping_add(mv.interval.wrapping_mul((count - 1) as u32));
        let last_clock = first_clock.wrapping_add(ticks as u64);

        // Create and queue a queue_step command
        let msg = [self.queue_step_msgtag as u32, self.oid, mv.interval, mv.count as u32, mv.add as i32 as u32];
        let mut qm = message_alloc_and_encode(&msg);
        qm.min_clock = self.last_step_clock;
        qm.req_clock = self.last_step_clock;
        if mv.count == 1 && first_clock >= self.last_step_clock.wrapping_add(CLOCK_DIFF_MAX) {
            qm.req_clock = first_clock;
        }
        self.msg_queue.borrow_mut().push_back(qm);
        self.last_step_clock = last_clock;

        // Create and store move in history tracking
        let hs = HistorySteps {
            first_clock,
            last_clock,
            start_position: self.last_position,
            interval: mv.interval as i32,
            add: mv.add as i32,
            step_count: if self.sdir != 0 { count } else { -count },
        };
        self.last_position += hs.step_count as i64;
        self.history_list.push_front(hs);
    }

    // Convert previously scheduled steps into commands for the mcu
    fn queue_flush(&mut self, move_clock: u64) -> i32 {
        if self.queue_pos >= self.queue_next {
            return 0;
        }
        while self.last_step_clock < move_clock {
            let mv = self.compress_bisect_add();
            let ret = self.check_line(mv);
            if ret != 0 {
                return ret;
            }

            self.add_move(self.last_step_clock.wrapping_add(mv.interval as u64), &mv);

            if self.queue_pos + mv.count as usize >= self.queue_next {
                self.queue_pos = 0;
                self.queue_next = 0;
                break;
            }
            self.queue_pos += mv.count as usize;
        }
        self.calc_last_step_print_time();
        0
    }

    // Generate a queue_step for a step far in the future from the last step
    fn flush_far(&mut self, abs_step_clock: u64) -> i32 {
        let mv = StepMove { interval: abs_step_clock.wrapping_sub(self.last_step_clock) as u32, count: 1, add: 0 };
        self.add_move(abs_step_clock, &mv);
        self.calc_last_step_print_time();
        0
    }

    // Send the set_next_step_dir command
    fn set_next_step_dir(&mut self, sdir: i32) -> i32 {
        if self.sdir == sdir {
            return 0;
        }
        let ret = self.queue_flush(u64::MAX);
        if ret != 0 {
            return ret;
        }
        self.sdir = sdir;
        let msg = [self.set_next_step_dir_msgtag as u32, self.oid, (sdir ^ self.invert_sdir) as u32];
        let mut qm = message_alloc_and_encode(&msg);
        qm.req_clock = self.last_step_clock;
        self.msg_queue.borrow_mut().push_back(qm);
        0
    }

    // Slow path for queue_append() - handle next step far in future
    fn queue_append_far(&mut self) -> i32 {
        let step_clock = self.next_step_clock;
        self.next_step_clock = 0;
        let ret = self.queue_flush(step_clock.wrapping_sub(CLOCK_DIFF_MAX).wrapping_add(1));
        if ret != 0 {
            return ret;
        }
        if step_clock >= self.last_step_clock.wrapping_add(CLOCK_DIFF_MAX) {
            return self.flush_far(step_clock);
        }
        self.push_step(step_clock);
        0
    }

    // *sc->queue_next++ = clock (the caller made room)
    #[inline]
    fn push_step(&mut self, clock: u64) {
        self.queue[self.queue_next] = clock as u32;
        self.queue_next += 1;
    }

    // Slow path for queue_append() - expand the internal queue storage
    fn queue_append_extend(&mut self) -> i32 {
        if self.queue_next - self.queue_pos > 65535 + 2000 {
            // No point in keeping more than 64K steps in memory
            let flush = self.queue[self.queue_next - 65535].wrapping_sub(self.last_step_clock as u32);
            let ret = self.queue_flush(self.last_step_clock.wrapping_add(flush as u64));
            if ret != 0 {
                return ret;
            }
        }

        if self.queue_next >= self.queue.len() {
            // Make room in the queue
            let in_use = self.queue_next - self.queue_pos;
            if self.queue_pos > 0 {
                // Shuffle the internal queue to avoid having to allocate more ram
                self.queue.copy_within(self.queue_pos..self.queue_next, 0);
            } else {
                // Expand the internal queue of step times
                let mut alloc = self.queue.len();
                if alloc == 0 {
                    alloc = QUEUE_START_SIZE;
                }
                while in_use >= alloc {
                    alloc *= 2;
                }
                self.queue.resize(alloc, 0);
            }
            self.queue_pos = 0;
            self.queue_next = in_use;
        }

        self.push_step(self.next_step_clock);
        self.next_step_clock = 0;
        0
    }

    // Add a step time to the queue (flushing the queue if needed)
    fn queue_append(&mut self) -> i32 {
        if self.next_step_dir != self.sdir {
            let ret = self.set_next_step_dir(self.next_step_dir);
            if ret != 0 {
                return ret;
            }
        }
        if self.next_step_clock >= self.last_step_clock.wrapping_add(CLOCK_DIFF_MAX) {
            return self.queue_append_far();
        }
        if self.queue_next >= self.queue.len() {
            return self.queue_append_extend();
        }
        self.push_step(self.next_step_clock);
        self.next_step_clock = 0;
        0
    }

    // Add next step time
    pub fn append(&mut self, sdir: i32, print_time: f64, step_time: f64) -> i32 {
        // Calculate step clock
        let offset = print_time - self.last_step_print_time;
        let rel_sc = (step_time + offset) * self.mcu_freq;
        let step_clock = self.last_step_clock.wrapping_add(f64_to_u64(rel_sc));
        // Flush previous pending step (if any)
        if self.next_step_clock != 0 {
            if sdir != self.next_step_dir {
                let diff = step_clock.wrapping_sub(self.next_step_clock) as i64 as f64;
                if diff < SDS_FILTER_TIME * self.mcu_freq {
                    // Rollback last step to avoid rapid step+dir+step
                    self.next_step_clock = 0;
                    self.next_step_dir = sdir;
                    return 0;
                }
            }
            let ret = self.queue_append();
            if ret != 0 {
                return ret;
            }
        }
        // Store this step as the next pending step
        self.next_step_clock = step_clock;
        self.next_step_dir = sdir;
        0
    }

    // Commit next pending step (ie, do not allow a rollback)
    pub fn commit(&mut self) -> i32 {
        if self.next_step_clock != 0 {
            return self.queue_append();
        }
        0
    }

    // Flush pending steps
    pub fn flush(&mut self, move_clock: u64) -> i32 {
        if self.next_step_clock != 0 && move_clock >= self.next_step_clock {
            let ret = self.queue_append();
            if ret != 0 {
                return ret;
            }
        }
        self.queue_flush(move_clock)
    }

    // Reset the internal state of the stepcompress object
    pub fn reset(&mut self, last_step_clock: u64) -> i32 {
        let ret = self.flush(u64::MAX);
        if ret != 0 {
            return ret;
        }
        self.last_step_clock = last_step_clock;
        self.sdir = -1;
        self.calc_last_step_print_time();
        0
    }

    // Set last_position in the stepcompress object
    pub fn set_last_position(&mut self, clock: u64, last_position: i64) -> i32 {
        let ret = self.flush(u64::MAX);
        if ret != 0 {
            return ret;
        }
        self.last_position = last_position;

        // Add a marker to the history list
        let hs = HistorySteps { first_clock: clock, last_clock: clock, start_position: last_position, ..Default::default() };
        self.history_list.push_front(hs);
        0
    }

    // Search history of moves to find a past position at a given clock
    pub fn find_past_position(&self, clock: u64) -> i64 {
        let mut last_position = self.last_position;
        for hs in &self.history_list {
            if clock < hs.first_clock {
                last_position = hs.start_position;
                continue;
            }
            if clock >= hs.last_clock {
                return hs.start_position + hs.step_count as i64;
            }
            let (interval, add) = (hs.interval, hs.add);
            let ticks = (clock.wrapping_sub(hs.first_clock) as i32).wrapping_add(interval);
            let offset: i32 = if add == 0 {
                ticks / interval
            } else {
                // Solve for "count" using quadratic formula
                let a = 0.5 * add as f64;
                let b = interval as f64 - 0.5 * add as f64;
                let c = -(ticks as f64);
                (((b * b - 4. * a * c).sqrt() - b) / (2. * a)) as i32
            };
            if hs.step_count < 0 {
                return hs.start_position - offset as i64;
            }
            return hs.start_position + offset as i64;
        }
        last_position
    }

    // Return history of queue_step commands
    pub fn extract_old(&self, max: usize, start_clock: u64, end_clock: u64) -> Vec<PullHistorySteps> {
        let mut res = vec![];
        for hs in &self.history_list {
            if start_clock >= hs.last_clock || res.len() >= max {
                break;
            }
            if end_clock <= hs.first_clock {
                continue;
            }
            res.push(PullHistorySteps {
                first_clock: hs.first_clock,
                last_clock: hs.last_clock,
                start_position: hs.start_position,
                step_count: hs.step_count,
                interval: hs.interval,
                add: hs.add,
            });
        }
        res
    }
}

/// C's (uint64_t) conversion of a double. A negative value is undefined
/// behaviour in C: gcc on x86-64 truncates through int64 (wraps), while
/// arm / aarch64 convert with saturation (0) like rust's `as`.
#[inline]
fn f64_to_u64(v: f64) -> u64 {
    if cfg!(target_arch = "x86_64") && v < 0. {
        v as i64 as u64
    } else {
        v as u64
    }
}

// check_line()'s "p < point.minp || p > point.maxp": C compares the
// uint32_t p against the int32_t bounds after converting them to uint32_t
#[inline]
fn p_lt(p: u32, bound: i32) -> bool {
    p < bound as u32
}

#[inline]
fn p_gt(p: u32, bound: i32) -> bool {
    p > bound as u32
}
