// Stepper step transmit synchronization (port of klippy/chelper/steppersync.c)
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The steppersync object is used to synchronize the output of mcu
// step commands.  The mcu can only queue a limited number of step
// commands - this code tracks when items on the mcu step queue become
// free so that new commands can be transmitted.  It also ensures the
// mcu step queue is ordered between steppers so that no stepper
// starves the other steppers of space in the mcu step queue.
//
// Threading difference: C runs each syncemitter's step generation in its
// own background thread and waits for all of them before flushing. Here
// the emitters generate in turn on the calling thread; each one only
// writes its own message queue, so the flushed output is the same.
use super::errorf;
use super::itersolve::{self, SkRef};
use super::msgblock::{clock_fill, clock_from_time, message_alloc_and_encode, ClockEstimate};
use super::serialqueue::{self, CommandQueue, SerialQueue, MAX_CLOCK};
use super::stepcompress::{MsgQueue, StepCompress};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::Arc;

/****************************************************************
 * SyncEmitter - message generation for each stepper
 ****************************************************************/

pub struct SyncEmitter {
    // Transmit message queue
    msg_queue: MsgQueue,
    sc: Option<RefCell<StepCompress>>,
    sk: RefCell<Option<SkRef>>,
    name: String,
}

pub type SyncEmitterRef = Rc<SyncEmitter>;

impl SyncEmitter {
    // Allocate syncemitter
    fn alloc(name: &str, alloc_stepcompress: bool) -> Self {
        let msg_queue: MsgQueue = Rc::new(RefCell::new(VecDeque::new()));
        // char name[16]
        let mut name = name.to_string();
        while name.len() > 15 {
            name.pop();
        }
        let sc = alloc_stepcompress.then(|| RefCell::new(StepCompress::alloc(msg_queue.clone())));
        SyncEmitter { msg_queue, sc, sk: RefCell::new(None), name }
    }

    /// syncemitter_get_stepcompress() (None where C returns NULL)
    pub fn get_stepcompress(&self) -> Option<&RefCell<StepCompress>> {
        self.sc.as_ref()
    }

    // Store a reference to stepper_kinematics
    pub fn set_stepper_kinematics(&self, sk: Option<SkRef>) {
        *self.sk.borrow_mut() = sk;
    }

    // Report current stepper_kinematics
    pub fn get_stepper_kinematics(&self) -> Option<SkRef> {
        self.sk.borrow().clone()
    }

    // Queue an mcu command that will consume space in the mcu move queue
    pub fn queue_msg(&self, req_clock: u64, data: &[u32]) {
        let mut qm = message_alloc_and_encode(data);
        qm.min_clock = req_clock;
        qm.req_clock = req_clock;
        self.msg_queue.borrow_mut().push_back(qm);
    }

    // Generate steps (via itersolve) and flush
    fn generate_steps(&self, gen_steps_time: f64, flush_clock: u64, clear_history_clock: u64) -> i32 {
        let (Some(sc), Some(sk)) = (self.sc.as_ref(), self.sk.borrow().clone()) else { return 0 };
        let mut sc = sc.borrow_mut();
        // Generate steps
        let ret = itersolve::generate_steps(&mut sk.borrow_mut(), &mut sc, gen_steps_time);
        if ret != 0 {
            return ret;
        }
        // Flush steps
        let ret = sc.flush(flush_clock);
        if ret != 0 {
            return ret;
        }
        // Clear history
        sc.history_expire(clear_history_clock);
        0
    }

    // se_background_thread()'s work for one request
    fn gen_steps(&self, gen_steps_time: f64, flush_clock: u64, clear_history_clock: u64) -> i32 {
        if self.sc.is_none() || self.sk.borrow().is_none() {
            return 0;
        }
        let res = self.generate_steps(gen_steps_time, flush_clock, clear_history_clock);
        if res != 0 {
            errorf(&format!("Error in syncemitter '{}' step generation", self.name));
        }
        res
    }
}

/****************************************************************
 * StepperSync - sort move queue for a micro-controller
 ****************************************************************/

#[derive(Default)]
pub struct StepperSync {
    // Serial port
    sq: Option<Arc<SerialQueue>>,
    cq: Option<CommandQueue>,
    // The syncemitters that generate messages on this mcu
    se_list: Vec<SyncEmitterRef>,
    // Convert from time to clock
    ce: ClockEstimate,
    // Storage for list of pending move clocks
    move_clocks: Vec<u64>,
}

pub type StepperSyncRef = Rc<RefCell<StepperSync>>;

impl StepperSync {
    // Allocate a new syncemitter instance
    pub fn alloc_syncemitter(&mut self, name: &str, alloc_stepcompress: bool) -> SyncEmitterRef {
        let se = Rc::new(SyncEmitter::alloc(name, alloc_stepcompress));
        self.se_list.push(se.clone());
        se
    }

    // Fill information on mcu move queue
    pub fn setup_movequeue(&mut self, sq: Option<Arc<SerialQueue>>, move_num: usize) {
        if let Some(cq) = &self.cq {
            serialqueue::free_commandqueue(cq);
        }
        self.sq = sq;
        self.cq = Some(serialqueue::alloc_commandqueue());
        self.move_clocks = vec![0; move_num];
    }

    // Set the conversion rate of 'print_time' to mcu clock
    pub fn set_time(&mut self, time_offset: f64, mcu_freq: f64) {
        clock_fill(&mut self.ce, mcu_freq, time_offset, 0, 0);
        for se in &self.se_list {
            if let Some(sc) = &se.sc {
                sc.borrow_mut().set_time(time_offset, mcu_freq);
            }
        }
    }

    // Implement a binary heap algorithm to track when the next available
    // 'struct move' in the mcu will be available
    fn heap_replace(&mut self, req_clock: u64) {
        let mc = &mut self.move_clocks;
        let nmc = mc.len();
        let mut pos = 0;
        loop {
            let (child1_pos, child2_pos) = (2 * pos + 1, 2 * pos + 2);
            let child2_clock = if child2_pos < nmc { mc[child2_pos] } else { u64::MAX };
            let child1_clock = if child1_pos < nmc { mc[child1_pos] } else { u64::MAX };
            if req_clock <= child1_clock && req_clock <= child2_clock {
                mc[pos] = req_clock;
                break;
            }
            if child1_clock < child2_clock {
                mc[pos] = child1_clock;
                pos = child1_pos;
            } else {
                mc[pos] = child2_clock;
                pos = child2_pos;
            }
        }
    }

    // Find and transmit any scheduled steps prior to the given 'move_clock'
    fn flush(&mut self, move_clock: u64) {
        // Order commands by the reqclock of each pending command
        let mut msgs = vec![];
        loop {
            // Find message with lowest reqclock
            let mut req_clock = MAX_CLOCK;
            let mut found = None;
            for (i, se) in self.se_list.iter().enumerate() {
                if let Some(m) = se.msg_queue.borrow().front() {
                    if m.req_clock < req_clock {
                        found = Some(i);
                        req_clock = m.req_clock;
                    }
                }
            }
            let Some(i) = found else { break };
            let qm_min_clock = self.se_list[i].msg_queue.borrow().front().map(|m| m.min_clock).unwrap_or(0);
            if qm_min_clock != 0 && req_clock > move_clock {
                break;
            }

            let next_avail = self.move_clocks.first().copied().unwrap_or(0);
            if qm_min_clock != 0 {
                // The qm->min_clock field is overloaded to indicate that
                // the command uses the 'move queue' and to store the time
                // that move queue item becomes available.
                self.heap_replace(qm_min_clock);
            }
            // Reset the min_clock to its normal meaning (minimum transmit time)
            let mut qm = self.se_list[i].msg_queue.borrow_mut().pop_front().expect("message");
            qm.min_clock = next_avail;

            // Batch this command
            msgs.push(qm);
        }

        // Transmit commands
        if !msgs.is_empty() {
            if let (Some(sq), Some(cq)) = (&self.sq, &self.cq) {
                sq.send_batch(cq, msgs);
            }
        }
    }
}

/****************************************************************
 * StepperSyncMgr - manage a list of steppersync
 ****************************************************************/

#[derive(Default)]
pub struct StepperSyncMgr {
    ss_list: Vec<StepperSyncRef>,
}

impl StepperSyncMgr {
    // Allocate a new 'steppersyncmgr' object
    pub fn alloc() -> Self {
        Self::default()
    }

    // Allocate a new 'steppersync' object
    pub fn alloc_steppersync(&mut self) -> StepperSyncRef {
        let ss = Rc::new(RefCell::new(StepperSync::default()));
        self.ss_list.push(ss.clone());
        ss
    }

    // Generate and flush steps
    pub fn gen_steps(&self, flush_time: f64, gen_steps_time: f64, clear_history_time: f64) -> i32 {
        let mut res = 0;
        // Step generation
        let mut results = vec![];
        for ss in &self.ss_list {
            let ss = ss.borrow();
            let flush_clock = clock_from_time(&ss.ce, flush_time);
            let clear_clock = clock_from_time(&ss.ce, clear_history_time);
            results.push(ss.se_list.iter().map(|se| se.gen_steps(gen_steps_time, flush_clock, clear_clock)).collect::<Vec<_>>());
        }
        // Collect step generation results
        for (ss, rets) in self.ss_list.iter().zip(results) {
            for ret in rets {
                if ret != 0 {
                    res = ret;
                }
            }
            if res != 0 {
                continue;
            }
            let mut ss = ss.borrow_mut();
            let flush_clock = clock_from_time(&ss.ce, flush_time);
            ss.flush(flush_clock);
        }
        res
    }
}

impl Drop for StepperSyncMgr {
    // steppersyncmgr_free()
    fn drop(&mut self) {
        for ss in &self.ss_list {
            if let Some(cq) = &ss.borrow().cq {
                serialqueue::free_commandqueue(cq);
            }
        }
    }
}
