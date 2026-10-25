// Helper code for the Klipper mcu protocol "message blocks"
// (port of klippy/chelper/msgblock.c)
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

pub const MESSAGE_MIN: usize = 5;
pub const MESSAGE_MAX: usize = 64;
pub const MESSAGE_HEADER_SIZE: usize = 2;
pub const MESSAGE_TRAILER_SIZE: usize = 3;
pub const MESSAGE_POS_LEN: usize = 0;
pub const MESSAGE_POS_SEQ: usize = 1;
pub const MESSAGE_TRAILER_CRC: usize = 3;
pub const MESSAGE_TRAILER_SYNC: usize = 1;
pub const MESSAGE_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;
pub const MESSAGE_SEQ_MASK: u8 = 0x0f;
pub const MESSAGE_DEST: u8 = 0x10;
pub const MESSAGE_SYNC: u8 = 0x7E;

// The C version overlays min/req_clock with sent/receive_time in a union;
// separate fields here (no code reads one through the other).
#[derive(Clone, Debug)]
pub struct QueueMessage {
    pub len: usize,
    pub msg: [u8; MESSAGE_MAX],
    // Filled when on a command queue
    pub min_clock: u64,
    pub req_clock: u64,
    // Filled when in sent/receive queues
    pub sent_time: f64,
    pub receive_time: f64,
    pub notify_id: u64,
}

// message_alloc(): zero filled
impl Default for QueueMessage {
    fn default() -> Self {
        QueueMessage {
            len: 0,
            msg: [0; MESSAGE_MAX],
            min_clock: 0,
            req_clock: 0,
            sent_time: 0.,
            receive_time: 0.,
            notify_id: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ClockEstimate {
    pub last_clock: u64,
    pub conv_clock: u64,
    pub conv_time: f64,
    pub est_freq: f64,
}


/****************************************************************
 * Serial protocol helpers
 ****************************************************************/

// Implement the standard crc "ccitt" algorithm on the given buffer
pub fn msgblock_crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in buf {
        let mut data = b;
        data ^= (crc & 0xff) as u8;
        data ^= data << 4;
        crc = (((data as u16) << 8) | (crc >> 8)) ^ ((data >> 4) as u16)
            ^ ((data as u16) << 3);
    }
    crc
}

// Verify a buffer starts with a valid mcu message
pub fn msgblock_check(need_sync: &mut bool, buf: &[u8]) -> isize {
    let buf_len = buf.len();
    if buf_len < MESSAGE_MIN {
        // Need more data
        return 0;
    }
    'error: {
        if *need_sync {
            break 'error;
        }
        let msglen = buf[MESSAGE_POS_LEN] as usize;
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&msglen) {
            break 'error;
        }
        let msgseq = buf[MESSAGE_POS_SEQ];
        if (msgseq & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
            break 'error;
        }
        if buf_len < msglen {
            // Need more data
            return 0;
        }
        if buf[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
            break 'error;
        }
        let msgcrc = ((buf[msglen - MESSAGE_TRAILER_CRC] as u16) << 8)
            | buf[msglen - MESSAGE_TRAILER_CRC + 1] as u16;
        let crc = msgblock_crc16_ccitt(&buf[..msglen - MESSAGE_TRAILER_SIZE]);
        if crc != msgcrc {
            break 'error;
        }
        return msglen as isize;
    }
    // Discard bytes until next SYNC found
    if let Some(pos) = buf.iter().position(|&c| c == MESSAGE_SYNC) {
        *need_sync = false;
        return -(pos as isize + 1);
    }
    *need_sync = true;
    -(buf_len as isize)
}

// Encode an integer as a variable length quantity (vlq)
fn encode_int(out: &mut Vec<u8>, v: u32) {
    let sv = v as i32 as i64;
    let n = if (-(1 << 5)..(3 << 5)).contains(&sv) {
        1
    } else if (-(1 << 12)..(3 << 12)).contains(&sv) {
        2
    } else if (-(1 << 19)..(3 << 19)).contains(&sv) {
        3
    } else if (-(1 << 26)..(3 << 26)).contains(&sv) {
        4
    } else {
        5
    };
    if n >= 5 {
        out.push(((v >> 28) | 0x80) as u8);
    }
    if n >= 4 {
        out.push((((v >> 21) & 0x7f) | 0x80) as u8);
    }
    if n >= 3 {
        out.push((((v >> 14) & 0x7f) | 0x80) as u8);
    }
    if n >= 2 {
        out.push((((v >> 7) & 0x7f) | 0x80) as u8);
    }
    out.push((v & 0x7f) as u8);
}

// Parse an integer that was encoded as a "variable length quantity"
fn parse_int(msg: &[u8], p: &mut usize) -> u32 {
    let mut c = msg[*p];
    *p += 1;
    let mut v = (c & 0x7f) as u32;
    if (c & 0x60) == 0x60 {
        v |= (-0x20i32) as u32;
    }
    while c & 0x80 != 0 {
        c = msg[*p];
        *p += 1;
        v = (v << 7) | (c & 0x7f) as u32;
    }
    v
}

// Parse the VLQ contents of a message
pub fn msgblock_decode(data: &mut [u32], msg: &[u8]) -> i32 {
    let mut p = MESSAGE_HEADER_SIZE;
    let end = msg.len() - MESSAGE_TRAILER_SIZE;
    for d in data.iter_mut() {
        if p >= end {
            return -1;
        }
        *d = parse_int(msg, &mut p);
    }
    if p != end {
        // Invalid message
        return -1;
    }
    0
}


/****************************************************************
 * Command queues
 ****************************************************************/

// Allocate a queue_message and fill it with the specified data
pub fn message_fill(data: &[u8]) -> QueueMessage {
    let mut qm = QueueMessage::default();
    qm.msg[..data.len()].copy_from_slice(data);
    qm.len = data.len();
    qm
}

// Allocate a queue_message and fill it with a series of encoded vlq integers
pub fn message_alloc_and_encode(data: &[u32]) -> QueueMessage {
    let mut qm = QueueMessage::default();
    let mut out = Vec::with_capacity(MESSAGE_MAX);
    for &d in data {
        encode_int(&mut out, d);
        if out.len() > MESSAGE_PAYLOAD_MAX {
            crate::chelper::errorf("Encode error");
            qm.len = 0;
            return qm;
        }
    }
    qm.msg[..out.len()].copy_from_slice(&out);
    qm.len = out.len();
    qm
}


/****************************************************************
 * Clock estimation
 ****************************************************************/

// Extend a 32bit clock value to its full 64bit value
pub fn clock_from_clock32(ce: &ClockEstimate, clock32: u32) -> u64 {
    ce.last_clock.wrapping_add((clock32.wrapping_sub(ce.last_clock as u32)) as i32 as i64 as u64)
}

// Convert a clock to its estimated time
pub fn clock_to_time(ce: &ClockEstimate, clock: u64) -> f64 {
    ce.conv_time + (clock.wrapping_sub(ce.conv_clock) as i64) as f64 / ce.est_freq
}

// Convert a time to the nearest clock value
pub fn clock_from_time(ce: &ClockEstimate, time: f64) -> u64 {
    // C's (int64_t) cast truncates; `as i64` matches for in-range values
    (((time - ce.conv_time) * ce.est_freq + 0.5) as i64 as u64).wrapping_add(ce.conv_clock)
}

// Fill the fields of a 'struct clock_estimate'
pub fn clock_fill(ce: &mut ClockEstimate, est_freq: f64, conv_time: f64, conv_clock: u64, last_clock: u64) {
    ce.est_freq = est_freq;
    ce.conv_time = conv_time;
    ce.conv_clock = conv_clock;
    ce.last_clock = last_clock;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vlq_round_trip() {
        for &v in &[0u32, 1, 95, 96, 0xffffffe0, 0xffffffdf, 12287, 12288, 0x7fffffff, 0x80000000, 0xffffffff] {
            let qm = message_alloc_and_encode(&[v]);
            let mut msg = vec![0u8; MESSAGE_HEADER_SIZE];
            msg.extend_from_slice(&qm.msg[..qm.len]);
            msg.extend_from_slice(&[0, 0, 0]);
            let mut out = [0u32];
            assert_eq!(msgblock_decode(&mut out, &msg), 0);
            assert_eq!(out[0], v);
        }
    }

    #[test]
    fn check_resync() {
        let mut need_sync = false;
        // bad length byte, sync at index 3
        assert_eq!(msgblock_check(&mut need_sync, &[0xff, 0, 0, 0x7e, 0]), -4);
        assert!(!need_sync);
        assert_eq!(msgblock_check(&mut need_sync, &[0xff, 0, 0, 0, 0]), -5);
        assert!(need_sync);
    }
}
