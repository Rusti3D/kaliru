// Code for parsing incoming commands and encoding outgoing messages
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals)]
use crate::board::{console_receive_buffer, console_sendf, crc16_ccitt, irq_poll};
use crate::ctr::{Global, COMMAND_INDEX};
use crate::sched::{sched_is_shutdown, sched_report_shutdown, R};
use crate::{decl_shutdown, shutdown};

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

// Flags for command handler declarations.
pub const HF_IN_SHUTDOWN: u8 = 0x01; // Handler can run even when in emergency stop

pub const PT_uint32: u8 = 0;
pub const PT_int32: u8 = 1;
pub const PT_uint16: u8 = 2;
pub const PT_int16: u8 = 3;
pub const PT_byte: u8 = 4;
pub const PT_string: u8 = 5;
pub const PT_progmem_buffer: u8 = 6;
pub const PT_buffer: u8 = 7;

pub struct CommandEncoder {
    pub encoded_msgid: u16,
    pub max_size: u8,
    pub num_params: u8,
    pub param_types: &'static [u8],
}

pub type CommandFunc = fn(&[u32]) -> R;

pub struct CommandParser {
    pub encoded_msgid: u16,
    pub num_args: u8,
    pub flags: u8,
    pub num_params: u8,
    pub param_types: &'static [u8],
    pub func: Option<CommandFunc>,
}

impl CommandParser {
    pub const EMPTY: CommandParser =
        CommandParser { encoded_msgid: 0, num_args: 0, flags: 0, num_params: 0, param_types: &[], func: None };
}

/// a sendf() argument (the C varargs)
#[derive(Clone, Copy)]
pub enum SendArg<'a> {
    Int(u32),
    /// %s: a nul terminated string (up to the end of the slice here)
    Str(&'a [u8]),
    /// %*s / %.*s: a length and a pointer
    Buf(&'a [u8]),
}

macro_rules! int_arg {
    ($($t:ty),*) => {$(
        impl From<$t> for SendArg<'_> {
            fn from(v: $t) -> Self {
                SendArg::Int(v as i32 as u32)
            }
        }
    )*};
}
int_arg!(u8, u16, i8, i16, i32);

impl From<u32> for SendArg<'_> {
    fn from(v: u32) -> Self {
        SendArg::Int(v)
    }
}
impl From<bool> for SendArg<'_> {
    fn from(v: bool) -> Self {
        SendArg::Int(v as u32)
    }
}
impl<'a> From<&'a [u8]> for SendArg<'a> {
    fn from(v: &'a [u8]) -> Self {
        SendArg::Buf(v)
    }
}
impl<'a, const N: usize> From<&'a [u8; N]> for SendArg<'a> {
    fn from(v: &'a [u8; N]) -> Self {
        SendArg::Buf(v)
    }
}
impl<'a> From<&'a str> for SendArg<'a> {
    fn from(v: &'a str) -> Self {
        SendArg::Str(v.as_bytes())
    }
}

static NEXT_SEQUENCE: Global<u8> = Global::new(MESSAGE_DEST);

fn command_encode_ptr(p: *const u8) -> u32 {
    if core::mem::size_of::<usize>() > core::mem::size_of::<u32>() {
        return (p as usize).wrapping_sub(console_receive_buffer() as usize) as u32;
    }
    p as usize as u32
}

pub fn command_decode_ptr(v: u32) -> *mut u8 {
    if core::mem::size_of::<usize>() > core::mem::size_of::<u32>() {
        return console_receive_buffer().wrapping_add(v as usize);
    }
    v as usize as *mut u8
}

/// the bytes of a buffer argument (length, encoded pointer)
pub fn command_decode_buffer<'a>(len: u32, ptr: u32) -> &'a mut [u8] {
    unsafe { core::slice::from_raw_parts_mut(command_decode_ptr(ptr), len as usize) }
}

/****************************************************************
 * Binary message parsing
 ****************************************************************/

// Encode an integer as a variable length quantity (vlq)
fn encode_int(buf: &mut [u8], mut p: usize, v: u32) -> usize {
    let sv = v as i32;
    let n = if sv < (3 << 5) && sv >= -(1 << 5) {
        1
    } else if sv < (3 << 12) && sv >= -(1 << 12) {
        2
    } else if sv < (3 << 19) && sv >= -(1 << 19) {
        3
    } else if sv < (3 << 26) && sv >= -(1 << 26) {
        4
    } else {
        5
    };
    if n >= 5 {
        buf[p] = ((v >> 28) | 0x80) as u8;
        p += 1;
    }
    if n >= 4 {
        buf[p] = (((v >> 21) & 0x7f) | 0x80) as u8;
        p += 1;
    }
    if n >= 3 {
        buf[p] = (((v >> 14) & 0x7f) | 0x80) as u8;
        p += 1;
    }
    if n >= 2 {
        buf[p] = (((v >> 7) & 0x7f) | 0x80) as u8;
        p += 1;
    }
    buf[p] = (v & 0x7f) as u8;
    p + 1
}

// Parse an integer that was encoded as a "variable length quantity"
fn parse_int(buf: &[u8], pp: &mut usize) -> u32 {
    let mut p = *pp;
    let mut c = buf[p];
    p += 1;
    let mut v = (c & 0x7f) as u32;
    if (c & 0x60) == 0x60 {
        v |= (-0x20i32) as u32;
    }
    while c & 0x80 != 0 {
        c = buf[p];
        p += 1;
        v = (v << 7) | (c & 0x7f) as u32;
    }
    *pp = p;
    v
}

// Write an encoded msgid (optimized 2-byte VLQ encoder)
fn encode_msgid(buf: &mut [u8], mut p: usize, encoded_msgid: u16) -> usize {
    if encoded_msgid >= 0x80 {
        buf[p] = ((encoded_msgid >> 7) | 0x80) as u8;
        p += 1;
    }
    buf[p] = (encoded_msgid & 0x7f) as u8;
    p + 1
}

// Parse an encoded msgid (optimized 2-byte parser, return as positive number)
pub fn command_parse_msgid(buf: &[u8], pp: &mut usize) -> u16 {
    let mut p = *pp;
    let mut encoded_msgid = buf[p] as u16;
    p += 1;
    if encoded_msgid & 0x80 != 0 {
        encoded_msgid = ((encoded_msgid & 0x7f) << 7) | buf[p] as u16;
        p += 1;
    }
    *pp = p;
    encoded_msgid
}

// Parse an incoming command into 'args'
pub fn command_parsef(buf: &[u8], mut p: usize, maxend: usize, cp: &CommandParser, args: &mut [u32]) -> R<usize> {
    'error: {
        let mut a = 0;
        for &t in &cp.param_types[..cp.num_params as usize] {
            if p > maxend {
                break 'error;
            }
            match t {
                PT_uint32 | PT_int32 | PT_uint16 | PT_int16 | PT_byte => {
                    args[a] = parse_int(buf, &mut p);
                    a += 1;
                }
                PT_buffer => {
                    let len = buf[p] as usize;
                    p += 1;
                    if p + len > maxend {
                        break 'error;
                    }
                    args[a] = len as u32;
                    args[a + 1] = command_encode_ptr(buf[p..].as_ptr());
                    a += 2;
                    p += len;
                }
                _ => break 'error,
            }
        }
        if p > maxend {
            break 'error;
        }
        return Ok(p);
    }
    shutdown!("Command parser error");
}

// Encode a message
fn command_encodef(buf: &mut [u8; MESSAGE_MAX], ce: &CommandEncoder, args: &[SendArg]) -> R<usize> {
    let max_size = ce.max_size as usize;
    if max_size <= MESSAGE_MIN {
        // Ack/Nak message
        return Ok(max_size);
    }
    let mut p = MESSAGE_HEADER_SIZE;
    let maxend = p + max_size - MESSAGE_MIN;
    p = encode_msgid(buf, p, ce.encoded_msgid);
    'error: {
        for (i, &t) in ce.param_types[..ce.num_params as usize].iter().enumerate() {
            if p > maxend {
                break 'error;
            }
            match (t, args.get(i)) {
                (PT_uint32 | PT_int32 | PT_uint16 | PT_int16 | PT_byte, Some(SendArg::Int(v))) => {
                    p = encode_int(buf, p, *v);
                }
                (PT_string, Some(SendArg::Str(s))) => {
                    let lenp = p;
                    p += 1;
                    for &ch in s.iter() {
                        if ch == 0 || p >= maxend {
                            break;
                        }
                        buf[p] = ch;
                        p += 1;
                    }
                    buf[lenp] = (p - lenp - 1) as u8;
                }
                (PT_progmem_buffer | PT_buffer, Some(SendArg::Buf(s))) => {
                    let v = s.len().min(maxend - p);
                    buf[p] = v as u8;
                    p += 1;
                    buf[p..p + v].copy_from_slice(&s[..v]);
                    p += v;
                }
                _ => break 'error,
            }
        }
        return Ok(p + MESSAGE_TRAILER_SIZE);
    }
    shutdown!("Message encode error");
}

// Add header and trailer bytes to a message block
fn command_add_frame(buf: &mut [u8], msglen: usize) {
    buf[MESSAGE_POS_LEN] = msglen as u8;
    buf[MESSAGE_POS_SEQ] = *NEXT_SEQUENCE.get();
    let crc = crc16_ccitt(&buf[..msglen - MESSAGE_TRAILER_SIZE]);
    buf[msglen - MESSAGE_TRAILER_CRC] = (crc >> 8) as u8;
    buf[msglen - MESSAGE_TRAILER_CRC + 1] = crc as u8;
    buf[msglen - MESSAGE_TRAILER_SYNC] = MESSAGE_SYNC;
}

// Encode a message and then add a message block frame around it
pub fn command_encode_and_frame(buf: &mut [u8; MESSAGE_MAX], ce: &CommandEncoder, args: &[SendArg]) -> R<usize> {
    let msglen = command_encodef(buf, ce, args)?;
    command_add_frame(buf, msglen);
    Ok(msglen)
}

static IN_SENDF: Global<u8> = Global::new(0);

// Encode and transmit a "response" message
pub fn command_sendf(ce: &CommandEncoder, args: &[SendArg]) -> R {
    let in_sendf = IN_SENDF.ptr();
    unsafe {
        if core::ptr::read_volatile(in_sendf) != 0 {
            // This sendf call was made from an irq handler while the main
            // code was already in sendf - just drop this sendf request.
            return Ok(());
        }
        core::ptr::write_volatile(in_sendf, 1);
    }
    let r = console_sendf(ce, args);
    unsafe { core::ptr::write_volatile(in_sendf, 0) };
    r
}

pub fn sendf_shutdown() {
    unsafe { core::ptr::write_volatile(IN_SENDF.ptr(), 0) };
}
decl_shutdown!(sendf_shutdown);

/****************************************************************
 * Command routing
 ****************************************************************/

// Find the command handler associated with a command
fn command_lookup_parser(cmdid: u16) -> R<&'static CommandParser> {
    if cmdid == 0 || cmdid as usize >= COMMAND_INDEX.len() {
        shutdown!("Invalid command");
    }
    Ok(&COMMAND_INDEX[cmdid as usize])
}

// Empty message (for ack/nak transmission)
pub static ENCODE_ACKNAK: CommandEncoder = CommandEncoder { encoded_msgid: 0, max_size: MESSAGE_MIN as u8, num_params: 0, param_types: &[] };

const CF_NEED_SYNC: u8 = 1 << 0;
const CF_NEED_VALID: u8 = 1 << 1;

static SYNC_STATE: Global<u8> = Global::new(0);

/// the result of command_find_block: 1 a message, 0 more data needed,
/// -1 bytes to discard; with the pop count
// Find the next complete message block
pub fn command_find_block(buf: &[u8], pop_count: &mut usize) -> R<i8> {
    let buf_len = buf.len();
    let sync_state = SYNC_STATE.get();
    let mut nak_only = false;
    'error: {
        if buf_len != 0 && *sync_state & CF_NEED_SYNC != 0 {
            break 'error;
        }
        if buf_len < MESSAGE_MIN {
            // need_more_data
            *pop_count = 0;
            return Ok(0);
        }
        let msglen = buf[MESSAGE_POS_LEN] as usize;
        let msgseq = buf[MESSAGE_POS_SEQ];
        let bad = msglen < MESSAGE_MIN || msglen > MESSAGE_MAX || (msgseq & !MESSAGE_SEQ_MASK) != MESSAGE_DEST;
        if !bad && buf_len < msglen {
            *pop_count = 0;
            return Ok(0);
        }
        let valid = !bad
            && buf[msglen - MESSAGE_TRAILER_SYNC] == MESSAGE_SYNC
            && ((buf[msglen - MESSAGE_TRAILER_CRC] as u16) << 8 | buf[msglen - MESSAGE_TRAILER_CRC + 1] as u16)
                == crc16_ccitt(&buf[..msglen - MESSAGE_TRAILER_SIZE]);
        if valid {
            *sync_state &= !CF_NEED_VALID;
            *pop_count = msglen;
            // Check sequence number
            let next_sequence = NEXT_SEQUENCE.get();
            if msgseq != *next_sequence {
                // Lost message - discard messages until it is retransmitted
                nak_only = true;
                break 'error;
            }
            *next_sequence = ((msgseq + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
            return Ok(1);
        }
        // error:
        if buf[0] == MESSAGE_SYNC {
            // Ignore (do not nak) leading SYNC bytes
            *pop_count = 1;
            return Ok(-1);
        }
        *sync_state |= CF_NEED_SYNC;
    }
    if !nak_only {
        // need_sync: Discard bytes until next SYNC found
        match buf.iter().position(|&b| b == MESSAGE_SYNC) {
            Some(i) => {
                *sync_state &= !CF_NEED_SYNC;
                *pop_count = i + 1;
            }
            None => *pop_count = buf_len,
        }
        if *sync_state & CF_NEED_VALID != 0 {
            return Ok(-1);
        }
        *sync_state |= CF_NEED_VALID;
    }
    // nak:
    command_sendf(&ENCODE_ACKNAK, &[])?;
    Ok(-1)
}

// Dispatch all the commands found in a message block
pub fn command_dispatch(buf: &[u8], msglen: usize) -> R {
    let mut p = MESSAGE_HEADER_SIZE;
    let msgend = msglen - MESSAGE_TRAILER_SIZE;
    while p < msgend {
        let cmdid = command_parse_msgid(buf, &mut p);
        let cp = command_lookup_parser(cmdid)?;
        let mut args = [0u32; 64];
        let args = &mut args[..cp.num_args as usize];
        p = command_parsef(buf, p, msgend, cp, args)?;
        if sched_is_shutdown() && cp.flags & HF_IN_SHUTDOWN == 0 {
            sched_report_shutdown()?;
            continue;
        }
        irq_poll()?;
        (cp.func.unwrap())(args)?;
    }
    Ok(())
}

// Send an ack message to the host (notifying that it can send more data)
pub fn command_send_ack() -> R {
    command_sendf(&ENCODE_ACKNAK, &[])
}

// Find a message block and then dispatch all the commands in it
pub fn command_find_and_dispatch(buf: &[u8], pop_count: &mut usize) -> R<i8> {
    let ret = command_find_block(buf, pop_count)?;
    if ret > 0 {
        command_dispatch(buf, *pop_count)?;
        command_send_ack()?;
    }
    Ok(ret)
}
