// Serial port command queuing (port of klippy/chelper/serialqueue.c)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// This goal of this code is to handle low-level serial port
// communications with a microcontroller (mcu).  The code queues messages to
// be transmitted, schedules transmission of commands at specified mcu
// clock times, prioritizes commands, and handles retransmissions.  A
// background thread is launched to do this work and minimize latency.
//
// Same structure as the C code: one background thread running a
// pollreactor, the shared state behind one mutex (sq->lock), receivers
// waiting on a condition variable. C linked lists are VecDeques.
//
// RK_SQ_TRACE: like the reference built with kaliru/tools/ref_trace.patch,
// every message handed to send_batch() is written to that file.
use super::msgblock::{
    clock_fill, clock_from_time, message_fill, msgblock_check, msgblock_crc16_ccitt, ClockEstimate, QueueMessage,
    MESSAGE_DEST, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_PAYLOAD_MAX, MESSAGE_POS_LEN, MESSAGE_POS_SEQ,
    MESSAGE_SEQ_MASK, MESSAGE_SYNC, MESSAGE_TRAILER_CRC, MESSAGE_TRAILER_SIZE, MESSAGE_TRAILER_SYNC,
};
use super::pollreactor::{fd_set_non_blocking, PollReactor, PrHandler, PR_NEVER, PR_NOW};
use super::pyhelper::{errorf, get_monotonic, report_errno, set_thread_name};
use std::collections::VecDeque;
use std::io::Write;
use std::os::fd::RawFd;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};

pub const MAX_CLOCK: u64 = 0x7fffffffffffffff;
pub const BACKGROUND_PRIORITY_CLOCK: u64 = 0x7fffffff00000000;

const SQPF_SERIAL: usize = 0;
const SQPF_PIPE: usize = 1;
const SQPF_NUM: usize = 2;

const SQPT_RETRANSMIT: usize = 0;
const SQPT_COMMAND: usize = 1;
const SQPT_NUM: usize = 2;

pub const SQT_UART: u8 = b'u';
pub const SQT_CAN: u8 = b'c';
pub const SQT_DEBUGFILE: u8 = b'f';

const MIN_RTO: f64 = 0.025;
const MAX_RTO: f64 = 5.000;
const MAX_PENDING_BLOCKS: u64 = 12;
const MIN_REQTIME_DELTA: f64 = 0.100;
const MIN_BACKGROUND_DELTA: f64 = 0.005;

const DEBUG_QUEUE_SENT: usize = 100;
const DEBUG_QUEUE_RECEIVE: usize = 100;

/// struct pull_queue_message
#[derive(Clone, Debug)]
pub struct PullQueueMessage {
    pub msg: Vec<u8>,
    pub len: i32,
    pub sent_time: f64,
    pub receive_time: f64,
    pub notify_id: u64,
}

/// A low-latency message handler called from the background thread for
/// messages starting with `prefix`
pub struct FastReader {
    pub prefix: Vec<u8>,
    pub func: Box<dyn Fn(&[u8]) + Send + Sync>,
}

struct CqLists {
    upcoming_queue: VecDeque<QueueMessage>,
    ready_queue: VecDeque<QueueMessage>,
}

/// struct command_queue. Its lists are only touched with the owning
/// serialqueue's lock held (as in C); the inner mutex makes that safe.
pub struct CommandQueueInner {
    lists: Mutex<CqLists>,
    trace_id: usize,
}

pub type CommandQueue = Arc<CommandQueueInner>;

static TRACE_NEXT_ID: AtomicUsize = AtomicUsize::new(0);
static TRACE_FILE: Mutex<Option<std::fs::File>> = Mutex::new(None);
static TRACE_OPENED: AtomicBool = AtomicBool::new(false);

struct State {
    receive_waiting: bool,
    // Baud / clock tracking
    receive_window: i32,
    bittime_adjust: f64,
    idle_time: f64,
    ce: ClockEstimate,
    last_receive_sent_time: f64,
    // Retransmit support
    send_seq: u64,
    receive_seq: u64,
    ignore_nak_seq: u64,
    last_ack_seq: u64,
    retransmit_seq: u64,
    rtt_sample_seq: u64,
    sent_queue: VecDeque<QueueMessage>,
    srtt: f64,
    rttvar: f64,
    rto: f64,
    // Pending transmission message queues
    pending_queues: Vec<CommandQueue>,
    ready_bytes: i32,
    upcoming_bytes: i32,
    need_ack_bytes: i32,
    last_ack_bytes: i32,
    need_kick_clock: u64,
    notify_queue: VecDeque<QueueMessage>,
    last_write_fail_time: f64,
    // Received messages
    receive_queue: VecDeque<QueueMessage>,
    // Fastreader support
    fast_readers: Vec<Arc<FastReader>>,
    // Debugging
    old_sent: VecDeque<QueueMessage>,
    old_receive: VecDeque<QueueMessage>,
    // Stats
    bytes_write: u32,
    bytes_read: u32,
    bytes_retransmit: u32,
    bytes_invalid: u32,
}

struct Shared {
    serial_fd: RawFd,
    serial_fd_type: u8,
    client_id: u32,
    pipe_fds: [RawFd; 2],
    name: String,
    lock: Mutex<State>,
    cond: Condvar,
    fast_reader_dispatch_lock: Mutex<()>,
    must_exit: Arc<AtomicBool>,
}

pub struct SerialQueue {
    shared: Arc<Shared>,
    tid: Mutex<Option<std::thread::JoinHandle<()>>>,
}

// Create a series of empty messages and add them to a list
fn debug_queue_alloc(count: usize) -> VecDeque<QueueMessage> {
    (0..count).map(|_| QueueMessage::default()).collect()
}

// Copy a message to a debug queue and free old debug messages
fn debug_queue_add(root: &mut VecDeque<QueueMessage>, qm: QueueMessage) {
    root.push_back(qm);
    root.pop_front();
}

impl Shared {
    // Wake up the receiver thread if it is waiting
    fn check_wake_receive(&self, st: &mut State) {
        if st.receive_waiting {
            st.receive_waiting = false;
            self.cond.notify_one();
        }
    }

    // Write to the internal pipe to wake the background thread if in poll
    fn kick_bg_thread(&self) {
        let ret = unsafe { libc::write(self.pipe_fds[1], b".".as_ptr() as *const libc::c_void, 1) };
        if ret < 0 {
            report_errno("pipe write", ret as i32);
        }
    }
}

// Minimum number of bits in a canbus message
const CANBUS_PACKET_BITS: u32 = (1 + 11 + 3 + 4) + (16 + 2 + 7 + 3);
const CANBUS_IFS_BITS: u32 = 4;

// Determine minimum time needed to transmit a given number of bytes
fn calculate_bittime(sh: &Shared, st: &State, bytes: u32) -> f64 {
    if sh.serial_fd_type == SQT_CAN {
        let pkts = bytes.div_ceil(8);
        let bits = bytes * 8 + pkts * CANBUS_PACKET_BITS - CANBUS_IFS_BITS;
        st.bittime_adjust * bits as f64
    } else {
        st.bittime_adjust * bytes as f64
    }
}

/// struct can_frame (linux/can.h)
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct CanFrame {
    can_id: u32,
    can_dlc: u8,
    pad: u8,
    res0: u8,
    len8_dlc: u8,
    data: [u8; 8],
}

// The background thread's side: pollreactor callbacks and the input buffer
struct Bg {
    sh: Arc<Shared>,
    input_buf: [u8; 4096],
    need_sync: bool,
    input_pos: usize,
}

impl Bg {
    // Update internal state when the receive sequence increases
    fn update_receive_seq(&self, pr: &mut PollReactor, st: &mut State, eventtime: f64, rseq: u64) {
        // Remove from sent queue
        let mut sent_seq = st.receive_seq;
        loop {
            let Some(sent) = st.sent_queue.pop_front() else {
                // Got an ack for a message not sent; must be connection init
                st.send_seq = rseq;
                st.last_receive_sent_time = 0.;
                break;
            };
            st.need_ack_bytes -= sent.len as i32;
            let (receive_time, len) = (sent.receive_time, sent.len);
            debug_queue_add(&mut st.old_sent, sent);
            sent_seq = sent_seq.wrapping_add(1);
            if rseq == sent_seq {
                // Found sent message corresponding with the received sequence
                st.last_receive_sent_time = receive_time;
                st.last_ack_bytes = len as i32;
                break;
            }
        }
        st.receive_seq = rseq;
        pr.update_timer(SQPT_COMMAND, PR_NOW);

        // Update retransmit info
        if st.rtt_sample_seq != 0 && rseq > st.rtt_sample_seq && st.last_receive_sent_time != 0. {
            // RFC6298 rtt calculations
            let delta = eventtime - st.last_receive_sent_time;
            if st.srtt == 0. {
                st.rttvar = delta / 2.0;
                st.srtt = delta * 10.0; // use a higher start default
            } else {
                st.rttvar = (3.0 * st.rttvar + (st.srtt - delta).abs()) / 4.0;
                st.srtt = (7.0 * st.srtt + delta) / 8.0;
            }
            let mut rttvar4 = st.rttvar * 4.0;
            if rttvar4 < 0.001 {
                rttvar4 = 0.001;
            }
            st.rto = st.srtt + rttvar4;
            if st.rto < MIN_RTO {
                st.rto = MIN_RTO;
            } else if st.rto > MAX_RTO {
                st.rto = MAX_RTO;
            }
            st.rtt_sample_seq = 0;
        }
        match st.sent_queue.front() {
            None => pr.update_timer(SQPT_RETRANSMIT, PR_NEVER),
            Some(sent) => {
                let nr = eventtime + st.rto + calculate_bittime(&self.sh, st, sent.len as u32);
                pr.update_timer(SQPT_RETRANSMIT, nr);
            }
        }
    }

    // Process a well formed input message
    fn handle_message(&self, pr: &mut PollReactor, eventtime: f64, len: usize) {
        let sh = &self.sh;
        let mut st = sh.lock.lock().unwrap();

        // Calculate receive sequence number
        let rseq_delta =
            ((self.input_buf[MESSAGE_POS_SEQ] as u64).wrapping_sub(st.receive_seq)) & MESSAGE_SEQ_MASK as u64;
        let rseq = st.receive_seq.wrapping_add(rseq_delta);
        if rseq != st.receive_seq {
            // New sequence number
            if rseq > st.send_seq && st.receive_seq != 1 {
                // An ack for a message not sent?  Out of order message?
                st.bytes_invalid = st.bytes_invalid.wrapping_add(len as u32);
                return;
            }
            self.update_receive_seq(pr, &mut st, eventtime, rseq);
        }
        st.bytes_read = st.bytes_read.wrapping_add(len as u32);

        // Check for pending messages on notify_queue
        let mut must_wake = false;
        while let Some(qm) = st.notify_queue.front() {
            let wake_seq = rseq.wrapping_sub(1).wrapping_sub(if len > MESSAGE_MIN { 1 } else { 0 });
            let notify_msg_sent_seq = qm.req_clock;
            if notify_msg_sent_seq > wake_seq {
                break;
            }
            let mut qm = st.notify_queue.pop_front().unwrap();
            qm.len = 0;
            qm.sent_time = st.last_receive_sent_time;
            qm.receive_time = eventtime;
            st.receive_queue.push_back(qm);
            must_wake = true;
        }

        // Process message
        if len == MESSAGE_MIN {
            // Ack/nak message
            if st.last_ack_seq < rseq {
                st.last_ack_seq = rseq;
            } else if rseq > st.ignore_nak_seq && !st.sent_queue.is_empty() {
                // Duplicate Ack is a Nak - do fast retransmit
                pr.update_timer(SQPT_RETRANSMIT, PR_NOW);
            }
        } else {
            // Data message - add to receive queue
            let mut qm = message_fill(&self.input_buf[..len]);
            qm.sent_time = if rseq > st.retransmit_seq { st.last_receive_sent_time } else { 0. };
            qm.receive_time = get_monotonic(); // must be time post read()
            qm.receive_time -= calculate_bittime(sh, &st, len as u32);
            st.receive_queue.push_back(qm);
            must_wake = true;
        }

        // Check fast readers
        let readers = st.fast_readers.clone();
        for fr in readers {
            let plen = fr.prefix.len();
            if len < plen + MESSAGE_MIN || self.input_buf[MESSAGE_HEADER_SIZE..MESSAGE_HEADER_SIZE + plen] != fr.prefix[..] {
                continue;
            }
            // Release main lock and invoke callback
            let dispatch = sh.fast_reader_dispatch_lock.lock().unwrap();
            if must_wake {
                sh.check_wake_receive(&mut st);
            }
            drop(st);
            (fr.func)(&self.input_buf[..len]);
            drop(dispatch);
            return;
        }

        if must_wake {
            sh.check_wake_receive(&mut st);
        }
    }

    // Callback for input activity on the serial fd
    fn input_event(&mut self, pr: &mut PollReactor, eventtime: f64) {
        let sh = self.sh.clone();
        if sh.serial_fd_type == SQT_CAN {
            let mut cf = CanFrame::default();
            let ret = unsafe {
                libc::read(sh.serial_fd, &mut cf as *mut CanFrame as *mut libc::c_void, std::mem::size_of::<CanFrame>())
            };
            if ret <= 0 {
                report_errno("can read", ret as i32);
                pr.do_exit();
                return;
            }
            if cf.can_id != sh.client_id + 1 {
                return;
            }
            let dlc = cf.can_dlc as usize;
            self.input_buf[self.input_pos..self.input_pos + dlc].copy_from_slice(&cf.data[..dlc]);
            self.input_pos += dlc;
        } else {
            let ret = unsafe {
                libc::read(
                    sh.serial_fd,
                    self.input_buf[self.input_pos..].as_mut_ptr() as *mut libc::c_void,
                    self.input_buf.len() - self.input_pos,
                )
            };
            if ret <= 0 {
                if ret < 0 {
                    report_errno("read", ret as i32);
                } else {
                    errorf("Got EOF when reading from device");
                }
                pr.do_exit();
                return;
            }
            self.input_pos += ret as usize;
        }
        loop {
            let len = msgblock_check(&mut self.need_sync, &self.input_buf[..self.input_pos]);
            if len == 0 {
                // Need more data
                return;
            }
            let len = if len > 0 {
                // Received a valid message
                self.handle_message(pr, eventtime, len as usize);
                len as usize
            } else {
                // Skip bad data at beginning of input
                let len = (-len) as usize;
                let mut st = sh.lock.lock().unwrap();
                st.bytes_invalid = st.bytes_invalid.wrapping_add(len as u32);
                len
            };
            self.input_pos -= len;
            if self.input_pos != 0 {
                self.input_buf.copy_within(len..len + self.input_pos, 0);
            }
        }
    }

    // Callback for input activity on the pipe fd (wakes command_event)
    fn kick_event(&mut self, pr: &mut PollReactor, _eventtime: f64) {
        let mut dummy = [0u8; 4096];
        let ret = unsafe { libc::read(self.sh.pipe_fds[0], dummy.as_mut_ptr() as *mut libc::c_void, dummy.len()) };
        if ret < 0 {
            report_errno("pipe read", ret as i32);
        }
        pr.update_timer(SQPT_COMMAND, PR_NOW);
    }

    // OS write of data to be sent to the mcu
    fn do_write(&self, pr: &mut PollReactor, st: &mut State, buf: &[u8]) {
        let sh = &self.sh;
        if sh.serial_fd_type != SQT_CAN {
            let ret = unsafe { libc::write(sh.serial_fd, buf.as_ptr() as *const libc::c_void, buf.len()) };
            if ret < 0 {
                report_errno("write", ret as i32);
            }
            return;
        }
        // Write to CAN fd
        let mut buf = buf;
        while !buf.is_empty() {
            let size = buf.len().min(8);
            let mut cf = CanFrame { can_id: sh.client_id, can_dlc: size as u8, ..Default::default() };
            cf.data[..size].copy_from_slice(&buf[..size]);
            let ret = unsafe {
                libc::write(sh.serial_fd, &cf as *const CanFrame as *const libc::c_void, std::mem::size_of::<CanFrame>())
            };
            if ret < 0 {
                report_errno("can write", ret as i32);
                let curtime = get_monotonic();
                if st.last_write_fail_time == 0. {
                    st.last_write_fail_time = curtime;
                } else if curtime > st.last_write_fail_time + 10.0 {
                    errorf("Halting reads due to CAN write errors.");
                    pr.do_exit();
                }
                return;
            }
            st.last_write_fail_time = 0.0;
            buf = &buf[size..];
        }
    }

    // Callback timer for when a retransmit should be done
    fn retransmit_event(&mut self, pr: &mut PollReactor, eventtime: f64) -> f64 {
        let sh = self.sh.clone();
        if sh.serial_fd_type == SQT_UART {
            let ret = unsafe { libc::tcflush(sh.serial_fd, libc::TCOFLUSH) };
            if ret < 0 {
                report_errno("tcflush", ret);
            }
        }

        let mut st = sh.lock.lock().unwrap();

        // Retransmit all pending messages
        let mut buf = Vec::with_capacity(MESSAGE_MAX * MAX_PENDING_BLOCKS as usize + 1);
        let mut first_buflen = 0;
        buf.push(MESSAGE_SYNC);
        for qm in &st.sent_queue {
            buf.extend_from_slice(&qm.msg[..qm.len]);
            if first_buflen == 0 {
                first_buflen = qm.len + 1;
            }
        }
        self.do_write(pr, &mut st, &buf);
        st.bytes_retransmit = st.bytes_retransmit.wrapping_add(buf.len() as u32);

        // Update rto
        if pr.get_timer(SQPT_RETRANSMIT) == PR_NOW {
            // Retransmit due to nak
            st.ignore_nak_seq = st.receive_seq;
            if st.receive_seq < st.retransmit_seq {
                // Second nak for this retransmit - don't allow third
                st.ignore_nak_seq = st.retransmit_seq;
            }
        } else {
            // Retransmit due to timeout
            st.rto *= 2.0;
            if st.rto > MAX_RTO {
                st.rto = MAX_RTO;
            }
            st.ignore_nak_seq = st.send_seq;
        }
        st.retransmit_seq = st.send_seq;
        st.rtt_sample_seq = 0;
        st.idle_time = eventtime + calculate_bittime(&sh, &st, buf.len() as u32);
        eventtime + st.rto + calculate_bittime(&sh, &st, first_buflen as u32)
    }

    // Construct a block of data to be sent to the serial port
    fn build_and_send_command(&self, pr: &mut PollReactor, st: &mut State, out: &mut Vec<u8>, pending: usize, eventtime: f64) -> usize {
        let start = out.len();
        out.extend_from_slice(&[0, 0]);
        let mut len = MESSAGE_HEADER_SIZE;
        while st.ready_bytes != 0 {
            // Find highest priority message (message with lowest req_clock)
            let mut min_clock = MAX_CLOCK;
            let mut cq_idx = None;
            for (i, q) in st.pending_queues.iter().enumerate() {
                let lists = q.lists.lock().unwrap();
                if let Some(m) = lists.ready_queue.front() {
                    if m.req_clock < min_clock {
                        min_clock = m.req_clock;
                        cq_idx = Some(i);
                    }
                }
            }
            let ci = cq_idx.expect("ready_bytes without ready message");
            let cq = st.pending_queues[ci].clone();
            let mut lists = cq.lists.lock().unwrap();
            // Append message to outgoing command
            if len + lists.ready_queue.front().unwrap().len > MESSAGE_MAX - MESSAGE_TRAILER_SIZE {
                break;
            }
            let mut qm = lists.ready_queue.pop_front().unwrap();
            if lists.ready_queue.is_empty() && lists.upcoming_queue.is_empty() {
                st.pending_queues.remove(ci);
            }
            drop(lists);
            out.extend_from_slice(&qm.msg[..qm.len]);
            len += qm.len;
            st.ready_bytes -= qm.len as i32;
            if qm.notify_id != 0 {
                // Message requires notification - add to notify list
                qm.req_clock = st.send_seq;
                st.notify_queue.push_back(qm);
            }
        }

        // Fill header / trailer
        len += MESSAGE_TRAILER_SIZE;
        out.extend_from_slice(&[0, 0, 0]);
        let buf = &mut out[start..];
        buf[MESSAGE_POS_LEN] = len as u8;
        buf[MESSAGE_POS_SEQ] = MESSAGE_DEST | (st.send_seq as u8 & MESSAGE_SEQ_MASK);
        let crc = msgblock_crc16_ccitt(&buf[..len - MESSAGE_TRAILER_SIZE]);
        buf[len - MESSAGE_TRAILER_CRC] = (crc >> 8) as u8;
        buf[len - MESSAGE_TRAILER_CRC + 1] = (crc & 0xff) as u8;
        buf[len - MESSAGE_TRAILER_SYNC] = MESSAGE_SYNC;

        // Store message block
        let mut idletime = if eventtime > st.idle_time { eventtime } else { st.idle_time };
        idletime += calculate_bittime(&self.sh, st, (pending + len) as u32);
        let mut sent = message_fill(&out[start..start + len]);
        sent.sent_time = eventtime;
        sent.receive_time = idletime;
        if st.sent_queue.is_empty() {
            pr.update_timer(SQPT_RETRANSMIT, idletime + st.rto);
        }
        if st.rtt_sample_seq == 0 {
            st.rtt_sample_seq = st.send_seq;
        }
        st.send_seq = st.send_seq.wrapping_add(1);
        st.need_ack_bytes += len as i32;
        st.sent_queue.push_back(sent);
        len
    }

    // Determine the time the next serial data should be sent
    fn check_send_command(&self, st: &mut State, pending: usize, eventtime: f64) -> f64 {
        if st.send_seq.wrapping_sub(st.receive_seq) >= MAX_PENDING_BLOCKS && st.receive_seq != u64::MAX {
            // Need an ack before more messages can be sent
            return PR_NEVER;
        }
        if st.send_seq > st.receive_seq && st.receive_window != 0 {
            let mut need_ack_bytes = st.need_ack_bytes + MESSAGE_MAX as i32;
            if st.last_ack_seq < st.receive_seq {
                need_ack_bytes += st.last_ack_bytes;
            }
            if need_ack_bytes > st.receive_window {
                // Wait for ack from past messages before sending next message
                return PR_NEVER;
            }
        }

        // Check for stalled messages now ready
        let mut idletime = if eventtime > st.idle_time { eventtime } else { st.idle_time };
        idletime += calculate_bittime(&self.sh, st, (pending + MESSAGE_MIN) as u32);
        let ack_clock = clock_from_time(&st.ce, idletime);
        let mut min_stalled_clock = MAX_CLOCK;
        let mut min_ready_clock = MAX_CLOCK;
        let queues = st.pending_queues.clone();
        for cq in &queues {
            let mut lists = cq.lists.lock().unwrap();
            // Move messages from the upcoming_queue to the ready_queue
            while let Some(qm) = lists.upcoming_queue.front() {
                if ack_clock < qm.min_clock {
                    if qm.min_clock < min_stalled_clock {
                        min_stalled_clock = qm.min_clock;
                    }
                    break;
                }
                let qm = lists.upcoming_queue.pop_front().unwrap();
                st.upcoming_bytes -= qm.len as i32;
                st.ready_bytes += qm.len as i32;
                lists.ready_queue.push_back(qm);
            }
            // Update min_ready_clock
            if let Some(qm) = lists.ready_queue.front() {
                let mut req_clock = qm.req_clock;
                let bgtime = if pending != 0 { idletime } else { st.idle_time };
                let bgoffset = MIN_REQTIME_DELTA + MIN_BACKGROUND_DELTA;
                if req_clock == BACKGROUND_PRIORITY_CLOCK {
                    req_clock = clock_from_time(&st.ce, bgtime + bgoffset);
                }
                if req_clock < min_ready_clock {
                    min_ready_clock = req_clock;
                }
            }
        }

        // Check for messages to send
        if st.ready_bytes >= MESSAGE_PAYLOAD_MAX as i32 {
            return PR_NOW;
        }
        if st.ce.est_freq == 0. {
            if st.ready_bytes != 0 {
                return PR_NOW;
            }
            st.need_kick_clock = MAX_CLOCK;
            return PR_NEVER;
        }
        let reqclock_delta = (MIN_REQTIME_DELTA * st.ce.est_freq) as u64;
        if min_ready_clock <= ack_clock.wrapping_add(reqclock_delta) {
            return PR_NOW;
        }
        let mut wantclock = min_ready_clock - reqclock_delta;
        if min_stalled_clock < wantclock {
            wantclock = min_stalled_clock;
        }
        st.need_kick_clock = wantclock;
        // C: (wantclock - ack_clock) is a uint64_t divided by a double
        idletime + wantclock.wrapping_sub(ack_clock) as f64 / st.ce.est_freq
    }

    // Callback timer to send data to the serial port
    fn command_event(&mut self, pr: &mut PollReactor, eventtime: f64) -> f64 {
        let sh = self.sh.clone();
        let mut st = sh.lock.lock().unwrap();
        let bufsize = MESSAGE_MAX * MAX_PENDING_BLOCKS as usize;
        let mut buf: Vec<u8> = Vec::with_capacity(bufsize);
        let mut waketime;
        loop {
            waketime = self.check_send_command(&mut st, buf.len(), eventtime);
            if waketime != PR_NOW || buf.len() + MESSAGE_MAX > bufsize {
                if !buf.is_empty() {
                    // Write message blocks
                    self.do_write(pr, &mut st, &buf);
                    st.bytes_write = st.bytes_write.wrapping_add(buf.len() as u32);
                    let idletime = if eventtime > st.idle_time { eventtime } else { st.idle_time };
                    st.idle_time = idletime + calculate_bittime(&sh, &st, buf.len() as u32);
                    buf.clear();
                }
                if waketime != PR_NOW {
                    break;
                }
            }
            let pending = buf.len();
            self.build_and_send_command(pr, &mut st, &mut buf, pending, eventtime);
        }
        waketime
    }
}

impl PrHandler for Bg {
    fn fd_event(&mut self, pr: &mut PollReactor, pos: usize, eventtime: f64) {
        match pos {
            SQPF_SERIAL => self.input_event(pr, eventtime),
            _ => self.kick_event(pr, eventtime),
        }
    }
    fn timer_event(&mut self, pr: &mut PollReactor, pos: usize, eventtime: f64) -> f64 {
        match pos {
            SQPT_RETRANSMIT => self.retransmit_event(pr, eventtime),
            _ => self.command_event(pr, eventtime),
        }
    }
}

/// serialqueue_alloc_commandqueue()
pub fn alloc_commandqueue() -> CommandQueue {
    Arc::new(CommandQueueInner {
        lists: Mutex::new(CqLists { upcoming_queue: VecDeque::new(), ready_queue: VecDeque::new() }),
        trace_id: TRACE_NEXT_ID.fetch_add(1, Ordering::SeqCst),
    })
}

/// serialqueue_free_commandqueue()
pub fn free_commandqueue(cq: &CommandQueue) {
    let lists = cq.lists.lock().unwrap();
    if !lists.ready_queue.is_empty() || !lists.upcoming_queue.is_empty() {
        errorf("Memory leak! Can't free non-empty commandqueue");
    }
}

fn trace(name: &str, cq: &CommandQueue, msgs: &[QueueMessage]) {
    if !TRACE_OPENED.swap(true, Ordering::SeqCst) {
        if let Ok(path) = std::env::var("RK_SQ_TRACE") {
            *TRACE_FILE.lock().unwrap() = std::fs::File::create(path).ok();
        }
    }
    let mut f = TRACE_FILE.lock().unwrap();
    let Some(f) = f.as_mut() else { return };
    let mut out = String::new();
    for qm in msgs {
        out.push_str(&format!("{} q{} {} {} ", name, cq.trace_id, qm.min_clock, qm.req_clock));
        for b in &qm.msg[..qm.len] {
            out.push_str(&format!("{:02x}", b));
        }
        out.push('\n');
    }
    let _ = f.write_all(out.as_bytes());
}

impl SerialQueue {
    /// serialqueue_alloc(); None where C returns NULL
    pub fn alloc(serial_fd: RawFd, serial_fd_type: u8, client_id: u32, name: &str) -> Option<Arc<SerialQueue>> {
        // char name[16]
        let mut name = name.to_string();
        while name.len() > 15 {
            name.pop();
        }
        let mut pipe_fds = [0 as RawFd; 2];
        let ret = unsafe { libc::pipe(pipe_fds.as_mut_ptr()) };
        if ret != 0 {
            report_errno("init", ret);
            return None;
        }

        // Reactor setup
        let mut pr = PollReactor::new(SQPF_NUM, SQPT_NUM);
        pr.add_fd(SQPF_SERIAL, serial_fd, serial_fd_type == SQT_DEBUGFILE);
        pr.add_fd(SQPF_PIPE, pipe_fds[0], false);
        pr.add_timer(SQPT_RETRANSMIT);
        pr.add_timer(SQPT_COMMAND);
        fd_set_non_blocking(serial_fd);
        fd_set_non_blocking(pipe_fds[0]);
        fd_set_non_blocking(pipe_fds[1]);

        // Retransmit setup
        let (receive_seq, rto) = if serial_fd_type == SQT_DEBUGFILE {
            // Debug file output
            (u64::MAX, PR_NEVER)
        } else {
            (1, MIN_RTO)
        };
        let st = State {
            receive_waiting: false,
            receive_window: 0,
            bittime_adjust: 0.,
            idle_time: 0.,
            ce: ClockEstimate::default(),
            last_receive_sent_time: 0.,
            send_seq: 1,
            receive_seq,
            ignore_nak_seq: 0,
            last_ack_seq: 0,
            retransmit_seq: 0,
            rtt_sample_seq: 0,
            sent_queue: VecDeque::new(),
            srtt: 0.,
            rttvar: 0.,
            rto,
            // Queues
            pending_queues: vec![],
            ready_bytes: 0,
            upcoming_bytes: 0,
            need_ack_bytes: 0,
            last_ack_bytes: 0,
            need_kick_clock: MAX_CLOCK,
            notify_queue: VecDeque::new(),
            last_write_fail_time: 0.,
            receive_queue: VecDeque::new(),
            fast_readers: vec![],
            // Debugging
            old_sent: debug_queue_alloc(DEBUG_QUEUE_SENT),
            old_receive: debug_queue_alloc(DEBUG_QUEUE_RECEIVE),
            bytes_write: 0,
            bytes_read: 0,
            bytes_retransmit: 0,
            bytes_invalid: 0,
        };
        let shared = Arc::new(Shared {
            serial_fd,
            serial_fd_type,
            client_id,
            pipe_fds,
            name: name.clone(),
            lock: Mutex::new(st),
            cond: Condvar::new(),
            fast_reader_dispatch_lock: Mutex::new(()),
            must_exit: pr.must_exit.clone(),
        });

        // Thread setup
        let mut bg = Bg { sh: shared.clone(), input_buf: [0; 4096], need_sync: false, input_pos: 0 };
        let tid = std::thread::Builder::new().spawn(move || {
            // Main background thread for reading/writing to serial port
            set_thread_name(&bg.sh.name);
            pr.run(&mut bg);
            let mut st = bg.sh.lock.lock().unwrap();
            bg.sh.check_wake_receive(&mut st);
        });
        let tid = match tid {
            Ok(t) => t,
            Err(_) => {
                report_errno("init", -1);
                return None;
            }
        };
        Some(Arc::new(SerialQueue { shared, tid: Mutex::new(Some(tid)) }))
    }

    fn is_exit(&self) -> bool {
        self.shared.must_exit.load(Ordering::SeqCst)
    }

    /// Request that the background thread exit
    pub fn exit(&self) {
        self.shared.must_exit.store(true, Ordering::SeqCst);
        self.shared.kick_bg_thread();
        if let Some(t) = self.tid.lock().unwrap().take() {
            if t.join().is_err() {
                report_errno("pthread_join", -1);
            }
        }
    }

    /// Add a low-latency message handler
    pub fn add_fastreader(&self, fr: Arc<FastReader>) {
        self.shared.lock.lock().unwrap().fast_readers.push(fr);
    }

    /// Remove a previously registered low-latency message handler
    pub fn rm_fastreader(&self, fr: &Arc<FastReader>) {
        self.shared.lock.lock().unwrap().fast_readers.retain(|f| !Arc::ptr_eq(f, fr));
        // XXX - goofy locking
        drop(self.shared.fast_reader_dispatch_lock.lock().unwrap());
    }

    /// Add a batch of messages to the given command_queue
    pub fn send_batch(&self, cq: &CommandQueue, mut msgs: Vec<QueueMessage>) {
        // Make sure min_clock is set in list and calculate total bytes
        let mut len = 0;
        for qm in msgs.iter_mut() {
            if qm.min_clock.wrapping_add(3 << 29) < qm.req_clock && qm.req_clock != BACKGROUND_PRIORITY_CLOCK {
                // Avoid mcu clock comparison 31-bit overflow issues
                qm.min_clock = qm.req_clock - (3 << 29);
            }
            len += qm.len;
        }
        trace(&self.shared.name, cq, &msgs);
        if len == 0 {
            return;
        }
        let first_min_clock = msgs[0].min_clock;

        // Add list to cq->upcoming_queue
        let mut st = self.shared.lock.lock().unwrap();
        {
            let mut lists = cq.lists.lock().unwrap();
            if lists.ready_queue.is_empty() && lists.upcoming_queue.is_empty() {
                st.pending_queues.push(cq.clone());
            }
            lists.upcoming_queue.extend(msgs);
        }
        st.upcoming_bytes += len as i32;
        let mut mustwake = false;
        if first_min_clock < st.need_kick_clock {
            st.need_kick_clock = 0;
            mustwake = true;
        }
        drop(st);

        // Wake the background thread if necessary
        if mustwake {
            self.shared.kick_bg_thread();
        }
    }

    /// Helper to send a single message
    pub fn send_one(&self, cq: &CommandQueue, qm: QueueMessage) {
        self.send_batch(cq, vec![qm]);
    }

    /// Schedule the transmission of a message on the serial port at a
    /// given time and priority.
    pub fn send(&self, cq: &CommandQueue, msg: &[u8], min_clock: u64, req_clock: u64, notify_id: u64) {
        let mut qm = message_fill(msg);
        qm.min_clock = min_clock;
        qm.req_clock = req_clock;
        qm.notify_id = notify_id;
        self.send_one(cq, qm);
    }

    /// Return a message read from the serial port (or wait for one if none
    /// available); len -1 when the background thread exited
    pub fn pull(&self) -> PullQueueMessage {
        let mut st = self.shared.lock.lock().unwrap();
        // Wait for message to be available
        while st.receive_queue.is_empty() {
            if self.is_exit() {
                return PullQueueMessage { msg: vec![], len: -1, sent_time: 0., receive_time: 0., notify_id: 0 };
            }
            st.receive_waiting = true;
            st = self.shared.cond.wait(st).unwrap();
        }

        // Remove message from queue
        let qm = st.receive_queue.pop_front().unwrap();

        // Copy message
        let pqm = PullQueueMessage {
            msg: qm.msg[..qm.len].to_vec(),
            len: qm.len as i32,
            sent_time: qm.sent_time,
            receive_time: qm.receive_time,
            notify_id: qm.notify_id,
        };
        if qm.len != 0 {
            debug_queue_add(&mut st.old_receive, qm);
        }
        pqm
    }

    pub fn set_wire_frequency(&self, frequency: f64) {
        let mut st = self.shared.lock.lock().unwrap();
        if self.shared.serial_fd_type == SQT_CAN {
            st.bittime_adjust = 1. / frequency;
        } else {
            // An 8N1 serial line is 10 bits per byte (1 start, 8 data, 1 stop)
            st.bittime_adjust = 10. / frequency;
        }
    }

    pub fn set_receive_window(&self, receive_window: i32) {
        self.shared.lock.lock().unwrap().receive_window = receive_window;
    }

    /// Set the estimated clock rate of the mcu on the other end of the
    /// serial port
    pub fn set_clock_est(&self, est_freq: f64, conv_time: f64, conv_clock: u64, last_clock: u64) {
        let mut st = self.shared.lock.lock().unwrap();
        clock_fill(&mut st.ce, est_freq, conv_time, conv_clock, last_clock);
    }

    /// Return the latest clock estimate
    pub fn get_clock_est(&self) -> ClockEstimate {
        self.shared.lock.lock().unwrap().ce
    }

    /// The statistics string for the serial port
    pub fn get_stats(&self) -> String {
        let st = self.shared.lock.lock().unwrap();
        // C prints the 64bit sequence numbers cast to int with %u
        let seq = |v: u64| v as i32 as u32;
        format!(
            "bytes_write={} bytes_read={} bytes_retransmit={} bytes_invalid={} send_seq={} receive_seq={} retransmit_seq={} srtt={:.3} rttvar={:.3} rto={:.3} ready_bytes={} upcoming_bytes={}",
            st.bytes_write,
            st.bytes_read,
            st.bytes_retransmit,
            st.bytes_invalid,
            seq(st.send_seq),
            seq(st.receive_seq),
            seq(st.retransmit_seq),
            st.srtt,
            st.rttvar,
            st.rto,
            st.ready_bytes as u32,
            st.upcoming_bytes as u32
        )
    }

    /// Extract old messages stored in the debug queues
    pub fn extract_old(&self, sentq: bool, max: usize) -> Vec<PullQueueMessage> {
        let count = if sentq { DEBUG_QUEUE_SENT } else { DEBUG_QUEUE_RECEIVE };
        let replacement = debug_queue_alloc(count);
        // Atomically replace existing debug list with new zero'd list
        let current = {
            let mut st = self.shared.lock.lock().unwrap();
            let root = if sentq { &mut st.old_sent } else { &mut st.old_receive };
            std::mem::replace(root, replacement)
        };
        // Walk the debug list
        current
            .into_iter()
            .filter(|qm| qm.len != 0)
            .take(max)
            .map(|qm| PullQueueMessage {
                msg: qm.msg[..qm.len].to_vec(),
                len: qm.len as i32,
                sent_time: qm.sent_time,
                receive_time: qm.receive_time,
                notify_id: 0,
            })
            .collect()
    }
}

impl Drop for SerialQueue {
    /// serialqueue_free()
    // (unlike C, which leaks them, the wake-up pipe fds are closed)
    fn drop(&mut self) {
        if !self.is_exit() {
            self.exit();
        }
        let mut st = self.shared.lock.lock().unwrap();
        st.pending_queues.clear();
        drop(st);
        unsafe {
            libc::close(self.shared.pipe_fds[0]);
            libc::close(self.shared.pipe_fds[1]);
        }
    }
}
