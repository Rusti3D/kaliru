// Code for parsing incoming commands and encoding outgoing messages
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals)]
use crate::board::pgm::{readp, NEED_PROGMEM};
use crate::board::{console_sendf, crc16_ccitt, irq_poll};
use crate::ctr::{Global, COMMAND_INDEX, MAX_COMMAND_ARGS};
use core::ffi::{c_int, c_uint};
use core::marker::PhantomData;
use core::mem::size_of;
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

/// a table of num_params parameter types (C: `const uint8_t *param_types`:
/// one pointer, not a slice)
#[derive(Clone, Copy)]
pub struct ParamTypes(pub *const u8);
unsafe impl Sync for ParamTypes {}

impl ParamTypes {
    pub const NONE: ParamTypes = ParamTypes(core::ptr::null());
}

pub struct CommandEncoder {
    pub encoded_msgid: u16,
    pub max_size: u8,
    pub num_params: u8,
    pub param_types: ParamTypes,
}

// The arguments of a command (as many as the longest command has: an index
// known at compile time needs no bounds check)
pub type Args = [u32; MAX_COMMAND_ARGS];
pub type CommandFunc = fn(&Args) -> R;

pub struct CommandParser {
    pub encoded_msgid: u16,
    pub num_args: u8,
    pub flags: u8,
    pub num_params: u8,
    pub param_types: ParamTypes,
    pub func: Option<CommandFunc>,
}

impl CommandParser {
    pub const EMPTY: CommandParser =
        CommandParser { encoded_msgid: 0, num_args: 0, flags: 0, num_params: 0, param_types: ParamTypes::NONE, func: None };
}

/// a sendf() argument, a C vararg: an int, or a string / buffer (its
/// pointer and length), which the encoder's param_types tell apart. (Untagged,
/// as C's: 4 bytes on AVR, so command_sendf4() gets them in registers.)
#[derive(Clone, Copy)]
pub union SendArg<'a> {
    int: u32,
    buf: SendBuf<'a>,
    none: (),
}

#[derive(Clone, Copy)]
struct SendBuf<'a> {
    ptr: *const u8,
    len: usize,
    _data: PhantomData<&'a [u8]>,
}

impl<'a> SendArg<'a> {
    /// (no argument: command_sendf4()'s unused ones, never read)
    pub const NONE: SendArg<'static> = SendArg { none: () };

    pub const fn int(v: u32) -> Self {
        SendArg { int: v }
    }

    /// a %*s / %.*s buffer, or a %s string (up to a nul or its end here)
    pub const fn bytes(b: &'a [u8]) -> Self {
        SendArg { buf: SendBuf { ptr: b.as_ptr(), len: b.len(), _data: PhantomData } }
    }

    /// (the encoder's param_types say which it is)
    unsafe fn as_int(self) -> u32 {
        self.int
    }

    unsafe fn as_bytes(self) -> &'a [u8] {
        core::slice::from_raw_parts(self.buf.ptr, self.buf.len)
    }
}

macro_rules! int_arg {
    ($($t:ty),*) => {$(
        impl From<$t> for SendArg<'_> {
            fn from(v: $t) -> Self {
                SendArg::int(v as i32 as u32)
            }
        }
    )*};
}
int_arg!(u8, u16, i8, i16, i32);

impl From<u32> for SendArg<'_> {
    fn from(v: u32) -> Self {
        SendArg::int(v)
    }
}
impl From<bool> for SendArg<'_> {
    fn from(v: bool) -> Self {
        SendArg::int(v as u32)
    }
}
impl<'a> From<&'a [u8]> for SendArg<'a> {
    fn from(v: &'a [u8]) -> Self {
        SendArg::bytes(v)
    }
}
impl<'a, const N: usize> From<&'a [u8; N]> for SendArg<'a> {
    fn from(v: &'a [u8; N]) -> Self {
        SendArg::bytes(v)
    }
}
impl<'a> From<&'a str> for SendArg<'a> {
    fn from(v: &'a str) -> Self {
        SendArg::bytes(v.as_bytes())
    }
}

static NEXT_SEQUENCE: Global<u8> = Global::new(MESSAGE_DEST);

// (the C `if (sizeof(size_t) > sizeof(uint32_t))` as a cfg: 32bit boards
// have no console_receive_buffer())
fn command_encode_ptr(p: *const u8) -> u32 {
    #[cfg(target_pointer_width = "64")]
    return (p as usize).wrapping_sub(crate::board::console_receive_buffer() as usize) as u32;
    #[cfg(not(target_pointer_width = "64"))]
    return p as usize as u32;
}

pub fn command_decode_ptr(v: u32) -> *mut u8 {
    #[cfg(target_pointer_width = "64")]
    return crate::board::console_receive_buffer().wrapping_add(v as usize);
    #[cfg(not(target_pointer_width = "64"))]
    return v as usize as *mut u8;
}

/// the bytes of a buffer argument (length, encoded pointer)
pub fn command_decode_buffer<'a>(len: u32, ptr: u32) -> &'a mut [u8] {
    unsafe { core::slice::from_raw_parts_mut(command_decode_ptr(ptr), len as usize) }
}

/****************************************************************
 * Binary message parsing
 ****************************************************************/

// Encode an integer as a variable length quantity (vlq)
unsafe fn encode_int(mut p: *mut u8, v: u32) -> *mut u8 {
    let sv = v as i32;
    // (C jumps into the run of stores: the number of 7 bit groups before
    // the last)
    let n: u8 = if sv < (3 << 5) && sv >= -(1 << 5) {
        0
    } else if sv < (3 << 12) && sv >= -(1 << 12) {
        1
    } else if sv < (3 << 19) && sv >= -(1 << 19) {
        2
    } else if sv < (3 << 26) && sv >= -(1 << 26) {
        3
    } else {
        4
    };
    if n >= 4 {
        p.write(((v >> 28) | 0x80) as u8);
        p = p.add(1);
    }
    if n >= 3 {
        p.write((((v >> 21) & 0x7f) | 0x80) as u8);
        p = p.add(1);
    }
    if n >= 2 {
        p.write((((v >> 14) & 0x7f) | 0x80) as u8);
        p = p.add(1);
    }
    if n >= 1 {
        p.write((((v >> 7) & 0x7f) | 0x80) as u8);
        p = p.add(1);
    }
    p.write((v & 0x7f) as u8);
    p.add(1)
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
unsafe fn encode_msgid(mut p: *mut u8, encoded_msgid: u16) -> *mut u8 {
    if encoded_msgid >= 0x80 {
        p.write(((encoded_msgid >> 7) | 0x80) as u8);
        p = p.add(1);
    }
    p.write((encoded_msgid & 0x7f) as u8);
    p.add(1)
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
        let num_params = readp(&cp.num_params);
        let mut param_types = readp(&cp.param_types).0;
        for _ in 0..num_params {
            if p > maxend {
                break 'error;
            }
            let t = readp(unsafe { &*param_types });
            param_types = unsafe { param_types.add(1) };
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

// Encode a message (into `buf`, which has room for the encoder's max_size,
// as C's callers make sure)
unsafe fn command_encodef(buf: *mut u8, ce: &CommandEncoder, args: &[SendArg]) -> R<usize> {
    let max_size = readp(&ce.max_size) as usize;
    if max_size <= MESSAGE_MIN {
        // Ack/Nak message
        return Ok(max_size);
    }
    let mut p = buf.add(MESSAGE_HEADER_SIZE);
    let maxend = p.add(max_size - MESSAGE_MIN);
    let num_params = readp(&ce.num_params);
    let mut param_types = readp(&ce.param_types).0;
    p = encode_msgid(p, readp(&ce.encoded_msgid));
    let mut args = args.iter();
    'error: {
        for _ in 0..num_params {
            if p > maxend {
                break 'error;
            }
            let t = readp(&*param_types);
            param_types = param_types.add(1);
            let Some(&arg) = args.next() else {
                break 'error;
            };
            match t {
                PT_uint32 | PT_int32 | PT_uint16 | PT_int16 | PT_byte => {
                    let v = arg.as_int();
                    // (a 16bit int board passes these as an int / unsigned int)
                    let v = if size_of::<u32>() > size_of::<c_int>() && t >= PT_uint16 {
                        if t == PT_int16 {
                            v as c_int as i32 as u32
                        } else {
                            v as c_uint as u32
                        }
                    } else {
                        v
                    };
                    p = encode_int(p, v);
                }
                PT_string => {
                    let lenp = p;
                    p = p.add(1);
                    for &ch in arg.as_bytes() {
                        if ch == 0 || p >= maxend {
                            break;
                        }
                        p.write(ch);
                        p = p.add(1);
                    }
                    lenp.write((p.offset_from(lenp) - 1) as u8);
                }
                PT_progmem_buffer | PT_buffer => {
                    let s = arg.as_bytes();
                    let v = s.len().min(maxend.offset_from(p) as usize);
                    p.write(v as u8);
                    p = p.add(1);
                    if t == PT_progmem_buffer && NEED_PROGMEM {
                        // memcpy_P()
                        for (i, b) in s[..v].iter().enumerate() {
                            p.add(i).write(readp(b));
                        }
                    } else {
                        core::ptr::copy_nonoverlapping(s.as_ptr(), p, v);
                    }
                    p = p.add(v);
                }
                _ => break 'error,
            }
        }
        return Ok(p.offset_from(buf) as usize + MESSAGE_TRAILER_SIZE);
    }
    shutdown!("Message encode error");
}

// Add header and trailer bytes to a message block
unsafe fn command_add_frame(buf: *mut u8, msglen: usize) {
    buf.add(MESSAGE_POS_LEN).write(msglen as u8);
    buf.add(MESSAGE_POS_SEQ).write(*NEXT_SEQUENCE.get());
    let crc = crc16_ccitt(core::slice::from_raw_parts(buf, msglen - MESSAGE_TRAILER_SIZE));
    buf.add(msglen - MESSAGE_TRAILER_CRC).write((crc >> 8) as u8);
    buf.add(msglen - MESSAGE_TRAILER_CRC + 1).write(crc as u8);
    buf.add(msglen - MESSAGE_TRAILER_SYNC).write(MESSAGE_SYNC);
}

// Encode a message and then add a message block frame around it (`buf`
// has room for the encoder's max_size)
pub fn command_encode_and_frame(buf: &mut [u8], ce: &CommandEncoder, args: &[SendArg]) -> R<usize> {
    unsafe {
        let msglen = command_encodef(buf.as_mut_ptr(), ce, args)?;
        command_add_frame(buf.as_mut_ptr(), msglen);
        Ok(msglen)
    }
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

// (sendf!() with 1 to 4 arguments, the unused ones SendArg::NONE: they come
// in registers and the array is made here, not at every sendf!(); the
// encoder reads num_params of them)
#[inline(never)]
pub fn command_sendf4(ce: &CommandEncoder, a: SendArg, b: SendArg, c: SendArg, d: SendArg) -> R {
    command_sendf(ce, &[a, b, c, d])
}

pub fn sendf_shutdown() -> R {
    unsafe { core::ptr::write_volatile(IN_SENDF.ptr(), 0) };
    Ok(())
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
#[cfg_attr(CONFIG_MACH_AVR, link_section = ".progmem.data")]
pub static ENCODE_ACKNAK: CommandEncoder = CommandEncoder { encoded_msgid: 0, max_size: MESSAGE_MIN as u8, num_params: 0, param_types: ParamTypes::NONE };

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
        // (C: a variable length array of READP(cp->num_args); here one of
        // the longest command's size, of which the parser fills num_args)
        let mut args: Args = [0; MAX_COMMAND_ARGS];
        p = command_parsef(buf, p, msgend, cp, &mut args[..readp(&cp.num_args) as usize])?;
        if sched_is_shutdown() && readp(&cp.flags) & HF_IN_SHUTDOWN == 0 {
            sched_report_shutdown()?;
            continue;
        }
        irq_poll()?;
        (readp(&cp.func).unwrap())(&args)?;
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
