// Basic infrastructure commands.
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::{dynmem_end, dynmem_start, irq_disable, irq_enable, irq_restore, irq_save, timer_from_us, timer_read_time};
use crate::command::Args;
use crate::ctr::{Global, COMMAND_IDENTIFY_DATA};
use crate::sched::{sched_clear_shutdown, sched_is_shutdown, sched_timer_reset, R};
use crate::{decl_command, decl_constant, decl_init, decl_shutdown, sendf, shutdown};
use core::ptr::null_mut;

/****************************************************************
 * Low level allocation
 ****************************************************************/

const PTR_ALIGN: usize = core::mem::align_of::<*mut u8>();

const fn align(x: usize, a: usize) -> usize {
    (x + a - 1) & !(a - 1)
}

static ALLOC_END: Global<*mut u8> = Global::new(null_mut());

pub fn alloc_init() -> R {
    *ALLOC_END.get() = align(dynmem_start() as usize, PTR_ALIGN) as *mut u8;
    Ok(())
}
decl_init!(alloc_init);

// Allocate an area of memory
pub fn alloc_chunk(size: usize) -> R<*mut u8> {
    let alloc_end = ALLOC_END.get();
    if *alloc_end as usize + size > dynmem_end() as usize {
        shutdown!("alloc_chunk failed");
    }
    let data = *alloc_end;
    *alloc_end = alloc_end.wrapping_add(align(size, PTR_ALIGN));
    unsafe { core::ptr::write_bytes(data, 0, size) };
    Ok(data)
}

// Allocate an array of chunks
fn alloc_chunks(size: usize, count: usize, avail: &mut u16) -> R<*mut u8> {
    let mut can_alloc = 0;
    let mut p = *ALLOC_END.get() as usize;
    let end = dynmem_end() as usize;
    while can_alloc < count && p + size <= end {
        can_alloc += 1;
        p += size;
    }
    if can_alloc == 0 {
        shutdown!("alloc_chunks failed");
    }
    let data = alloc_chunk(p - *ALLOC_END.get() as usize)?;
    *avail = can_alloc as u16;
    Ok(data)
}

/****************************************************************
 * Move queue
 ****************************************************************/

#[repr(C)]
pub struct MoveNode {
    pub next: *mut MoveNode,
}

#[repr(C)]
pub struct MoveQueueHead {
    pub first: *mut MoveNode,
    pub last: *mut MoveNode,
}

struct Moves {
    free_list: *mut MoveNode,
    list: *mut u8,
    count: u16,
    item_size: u8,
}

static MOVES: Global<Moves> = Global::new(Moves { free_list: null_mut(), list: null_mut(), count: 0, item_size: 0 });

// Is the config and move queue finalized?
fn is_finalized() -> bool {
    MOVES.get().count != 0
}

// Free previously allocated storage from move_alloc(). Caller must
// disable irqs.
pub fn move_free(m: *mut u8) {
    let mf = m as *mut MoveNode;
    let moves = MOVES.get();
    unsafe { (*mf).next = moves.free_list };
    moves.free_list = mf;
}

// Allocate runtime storage
pub fn move_alloc() -> R<*mut u8> {
    let flag = irq_save();
    let moves = MOVES.get();
    let mf = moves.free_list;
    if mf.is_null() {
        shutdown!("Move queue overflow");
    }
    moves.free_list = unsafe { (*mf).next };
    irq_restore(flag);
    Ok(mf as *mut u8)
}

// Check if a move_queue is empty
pub fn move_queue_empty(mh: &MoveQueueHead) -> bool {
    mh.first.is_null()
}

// Return first node in a move queue
pub fn move_queue_first(mh: &MoveQueueHead) -> *mut MoveNode {
    mh.first
}

// Add move to queue
pub fn move_queue_push(m: *mut MoveNode, mh: &mut MoveQueueHead) -> bool {
    unsafe {
        (*m).next = null_mut();
        if !mh.first.is_null() {
            (*mh.last).next = m;
            mh.last = m;
            return false;
        }
    }
    mh.first = m;
    mh.last = m;
    true
}

// Remove first item from queue (caller must ensure queue not empty)
pub fn move_queue_pop(mh: &mut MoveQueueHead) -> *mut MoveNode {
    let mn = mh.first;
    mh.first = unsafe { (*mn).next };
    mn
}

// Completely clear move queue (used in shutdown handlers)
pub fn move_queue_clear(mh: &mut MoveQueueHead) {
    mh.first = null_mut();
}

// Initialize a move_queue with nodes of the give size
pub fn move_queue_setup(mh: &mut MoveQueueHead, size: usize) -> R {
    mh.first = null_mut();
    mh.last = null_mut();

    if size > u8::MAX as usize || is_finalized() {
        shutdown!("Invalid move request size");
    }
    let moves = MOVES.get();
    if size > moves.item_size as usize {
        moves.item_size = size as u8;
    }
    Ok(())
}

pub fn move_reset() -> R {
    let moves = MOVES.get();
    if moves.count == 0 {
        return Ok(());
    }
    // Add everything in move_list to the free list.
    let size = moves.item_size as usize;
    for i in 0..moves.count as usize - 1 {
        let mf = moves.list.wrapping_add(i * size) as *mut MoveNode;
        unsafe { (*mf).next = moves.list.wrapping_add((i + 1) * size) as *mut MoveNode };
    }
    let mf = moves.list.wrapping_add((moves.count as usize - 1) * size) as *mut MoveNode;
    unsafe { (*mf).next = null_mut() };
    moves.free_list = moves.list as *mut MoveNode;
    Ok(())
}
decl_shutdown!(move_reset);

fn move_finalize() -> R {
    if is_finalized() {
        shutdown!("Already finalized");
    }
    let mut dummy = MoveQueueHead { first: null_mut(), last: null_mut() };
    move_queue_setup(&mut dummy, core::mem::size_of::<MoveNode>())?;
    let moves = MOVES.get();
    moves.list = alloc_chunks(moves.item_size as usize, 1024, &mut moves.count)?;
    move_reset()?;
    Ok(())
}

/****************************************************************
 * Generic object ids (oid)
 ****************************************************************/

/// the type of an oid's object: the address of its config function
pub type OidType = *const ();

#[repr(C)]
struct OidS {
    typ: OidType,
    data: *mut u8,
}

struct Oids {
    oids: *mut OidS,
    count: u8,
}

static OIDS: Global<Oids> = Global::new(Oids { oids: null_mut(), count: 0 });

pub fn oid_lookup(oid: u8, typ: OidType) -> R<*mut u8> {
    let o = OIDS.get();
    if oid >= o.count || unsafe { (*o.oids.add(oid as usize)).typ } != typ {
        shutdown!("Invalid oid type");
    }
    Ok(unsafe { (*o.oids.add(oid as usize)).data })
}

pub fn oid_alloc(oid: u8, typ: OidType, size: u16) -> R<*mut u8> {
    let o = OIDS.get();
    if oid >= o.count || !unsafe { (*o.oids.add(oid as usize)).typ }.is_null() || is_finalized() {
        shutdown!("Can't assign oid");
    }
    unsafe { (*o.oids.add(oid as usize)).typ = typ };
    let data = alloc_chunk(size as usize)?;
    unsafe { (*o.oids.add(oid as usize)).data = data };
    Ok(data)
}

/// the next oid of type `typ` after *i (foreach_oid; *i starts at 255)
pub fn oid_next(i: &mut u8, typ: OidType) -> *mut u8 {
    let o = OIDS.get();
    let mut oid = *i;
    loop {
        oid = oid.wrapping_add(1);
        if oid >= o.count {
            return null_mut();
        }
        let e = unsafe { &*o.oids.add(oid as usize) };
        if e.typ == typ {
            *i = oid;
            return e.data;
        }
    }
}

/// foreach_oid(pos, data, oidtype): the objects of a type
pub fn foreach_oid(typ: OidType) -> impl Iterator<Item = *mut u8> {
    let mut pos: u8 = 255;
    core::iter::from_fn(move || {
        let d = oid_next(&mut pos, typ);
        (!d.is_null()).then_some(d)
    })
}

pub fn command_allocate_oids(args: &Args) -> R {
    let o = OIDS.get();
    if !o.oids.is_null() {
        shutdown!("oids already allocated");
    }
    let count = args[0] as u8;
    o.oids = alloc_chunk(core::mem::size_of::<OidS>() * count as usize)? as *mut OidS;
    OIDS.get().count = count;
    Ok(())
}
decl_command!(command_allocate_oids, "allocate_oids count=%c");

/****************************************************************
 * Config CRC
 ****************************************************************/

static CONFIG_CRC: Global<u32> = Global::new(0);

pub fn command_get_config(_args: &Args) -> R {
    sendf!(
        "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
        is_finalized() as u8,
        *CONFIG_CRC.get(),
        sched_is_shutdown() as u8,
        MOVES.get().count
    )
}
decl_command!(command_get_config, HF_IN_SHUTDOWN, "get_config");

pub fn command_finalize_config(args: &Args) -> R {
    move_finalize()?;
    *CONFIG_CRC.get() = args[0];
    Ok(())
}
decl_command!(command_finalize_config, "finalize_config crc=%u");

// Attempt a full manual reset of the config
pub fn config_reset(_args: &Args) -> R {
    if !sched_is_shutdown() {
        shutdown!("config_reset only available when shutdown");
    }
    irq_disable();
    *CONFIG_CRC.get() = 0;
    *OIDS.get() = Oids { oids: null_mut(), count: 0 };
    *MOVES.get() = Moves { free_list: null_mut(), list: null_mut(), count: 0, item_size: 0 };
    alloc_init()?;
    sched_timer_reset();
    sched_clear_shutdown()?;
    irq_enable();
    Ok(())
}

/****************************************************************
 * Timing and load stats
 ****************************************************************/

pub fn command_get_clock(_args: &Args) -> R {
    sendf!("clock clock=%u", timer_read_time())
}
decl_command!(command_get_clock, HF_IN_SHUTDOWN, "get_clock");

struct Stats {
    send_time: u32,
    send_time_high: u32,
    count: u32,
    sum: u32,
    sumsq: u32,
}

static STATS: Global<Stats> = Global::new(Stats { send_time: 0, send_time_high: 0, count: 0, sum: 0, sumsq: 0 });

pub fn command_get_uptime(_args: &Args) -> R {
    let cur = timer_read_time();
    let s = STATS.get();
    let high = s.send_time_high + (cur < s.send_time) as u32;
    sendf!("uptime high=%u clock=%u", high, cur)
}
decl_command!(command_get_uptime, HF_IN_SHUTDOWN, "get_uptime");

// Similar to timer_is_before(), but handles full 32bit duration
fn timer_has_elapsed(start: u32, cur: u32, duration: u32) -> bool {
    cur.wrapping_sub(start) >= duration
}

const SUMSQ_BASE: u32 = 256;
decl_constant!("STATS_SUMSQ_BASE", SUMSQ_BASE);

pub fn stats_update(start: u32, cur: u32) -> R {
    let s = STATS.get();
    let diff = cur.wrapping_sub(start);
    s.count = s.count.wrapping_add(1);
    s.sum = s.sum.wrapping_add(diff);
    // Calculate sum of diff^2 - be careful of integer overflow
    let mut nextsumsq = if diff <= 0xffff {
        s.sumsq.wrapping_add((diff * diff).div_ceil(SUMSQ_BASE))
    } else if diff <= 0xfffff {
        s.sumsq.wrapping_add(diff.div_ceil(SUMSQ_BASE).wrapping_mul(diff))
    } else {
        0xffffffff
    };
    if nextsumsq < s.sumsq {
        nextsumsq = 0xffffffff;
    }
    s.sumsq = nextsumsq;

    if !timer_has_elapsed(s.send_time, cur, timer_from_us(5000000)) {
        return Ok(());
    }
    sendf!("stats count=%u sum=%u sumsq=%u", s.count, s.sum, s.sumsq)?;
    let s = STATS.get();
    if cur < s.send_time {
        s.send_time_high += 1;
    }
    s.send_time = cur;
    s.count = 0;
    s.sum = 0;
    s.sumsq = 0;
    Ok(())
}

/****************************************************************
 * Misc commands
 ****************************************************************/

pub fn command_emergency_stop(_args: &Args) -> R {
    shutdown!("Command request");
}
decl_command!(command_emergency_stop, HF_IN_SHUTDOWN, "emergency_stop");

pub fn command_clear_shutdown(_args: &Args) -> R {
    sched_clear_shutdown()
}
decl_command!(command_clear_shutdown, HF_IN_SHUTDOWN, "clear_shutdown");

pub fn command_identify(args: &Args) -> R {
    let offset = args[0];
    let mut count = args[1] as u8 as u32;
    let isize = COMMAND_IDENTIFY_DATA.len() as u32;
    if offset >= isize {
        count = 0;
    } else if offset + count > isize {
        count = isize - offset;
    }
    let data = &COMMAND_IDENTIFY_DATA[(offset.min(isize)) as usize..(offset.min(isize) + count) as usize];
    sendf!("identify_response offset=%u data=%.*s", offset, data)
}
decl_command!(command_identify, HF_IN_SHUTDOWN, "identify offset=%u count=%c");
