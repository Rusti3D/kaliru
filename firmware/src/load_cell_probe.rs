// Load Cell based end stops.
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::board::{timer_is_before, timer_read_time};
use crate::sched::{sched_add_timer, sched_del_timer, Timer, R, SF_DONE, SF_RESCHEDULE};
use crate::sos_filter::{sos_filter_oid_lookup, sosfilt, SosFilter};
use crate::trsync::{trsync_do_trigger, trsync_oid_lookup, Trsync};
use crate::{decl_command, sendf, shutdown};
use core::mem::size_of;
use core::ptr::null_mut;

// Q2.29
type FixedQ2 = i32;
const FIXEDQ2: u32 = 2;
const FIXEDQ2_FRAC_BITS: u32 = (32 - FIXEDQ2) - 1;

// Q32.29 - a Q2.29 value stored in int64
type FixedQ32 = i64;
const FIXEDQ32_FRAC_BITS: u32 = FIXEDQ2_FRAC_BITS;

// Q16.15
type FixedQ16 = i32;
const FIXEDQ16: u32 = 16;
const FIXEDQ16_FRAC_BITS: u32 = (32 - FIXEDQ16) - 1;

// Q48.15 - a Q16.15 value stored in int64
type FixedQ48 = i64;
const FIXEDQ48_FRAC_BITS: u32 = FIXEDQ16_FRAC_BITS;

const MAX_TRIGGER_GRAMS: u32 = (1 << FIXEDQ16) - 1;
const ERROR_SAFETY_RANGE: u8 = 0;
const ERROR_OVERFLOW: u8 = 1;
const ERROR_WATCHDOG: u8 = 2;

// Flags
const FLAG_IS_HOMING: u8 = 1 << 0;
const FLAG_IS_HOMING_TRIGGER: u8 = 1 << 1;
const FLAG_AWAIT_HOMING: u8 = 1 << 2;

// Endstop Structure
#[repr(C)]
pub struct LoadCellProbe {
    time: Timer,
    trigger_grams: u32,
    trigger_ticks: u32,
    last_sample_ticks: u32,
    rest_ticks: u32,
    homing_start_time: u32,
    ts: *mut Trsync,
    safety_counts_min: i32,
    safety_counts_max: i32,
    tare_counts: i32,
    flags: u8,
    trigger_reason: u8,
    error_reason: u8,
    watchdog_max: u8,
    watchdog_count: u8,
    trigger_grams_fixed: FixedQ16,
    grams_per_count: FixedQ2,
    sf: *mut SosFilter,
}

fn oid_type() -> OidType {
    command_config_load_cell_probe as *const ()
}

#[inline]
fn overflows_int32(value: i64) -> bool {
    value > i32::MAX as i64 || value < i32::MIN as i64
}

// returns the integer part of a fixedQ48_t
#[allow(dead_code)]
#[inline]
fn round_fixed_q48(fixed_value: i64) -> i64 {
    fixed_value >> FIXEDQ48_FRAC_BITS
}

// Convert sensor counts to grams
#[inline]
fn counts_to_grams(lcp: &LoadCellProbe, counts: i32) -> FixedQ48 {
    // tearing ensures readings are referenced to 0.0g
    let delta = counts.wrapping_sub(lcp.tare_counts);
    // convert sensor counts to grams by multiplication: 124 * 0.051 = 6.324
    // this optimizes to single cycle SMULL instruction
    let product: FixedQ32 = delta as i64 * lcp.grams_per_count as i64;
    // after multiplication there are 30 fraction bits, reduce to 15
    // caller verifies this wont overflow a 32bit int when truncated
    product >> (FIXEDQ32_FRAC_BITS - FIXEDQ48_FRAC_BITS)
}

#[inline]
fn is_flag_set(mask: u8, lcp: &LoadCellProbe) -> bool {
    mask & lcp.flags != 0
}

#[inline]
fn set_flag(mask: u8, lcp: &mut LoadCellProbe) {
    lcp.flags |= mask;
}

#[inline]
fn clear_flag(mask: u8, lcp: &mut LoadCellProbe) {
    lcp.flags &= !mask;
}

pub fn try_trigger(lcp: &mut LoadCellProbe, ticks: u32) {
    let is_homing_triggered = is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp);
    if !is_homing_triggered {
        // the first triggering sample when homing sets the trigger time
        lcp.trigger_ticks = ticks;
        // this flag latches until a reset, disabling further triggering
        set_flag(FLAG_IS_HOMING_TRIGGER, lcp);
        trsync_do_trigger(lcp.ts, lcp.trigger_reason);
    }
}

pub fn trigger_error(lcp: &mut LoadCellProbe, error_code: u8) {
    trsync_do_trigger(lcp.ts, lcp.error_reason.wrapping_add(error_code));
}

// Used by Sensors to report new raw ADC sample
pub fn load_cell_probe_report_sample(lcp: *mut LoadCellProbe, sample: i32) -> R {
    let lcp = unsafe { &mut *lcp };
    // only process samples when homing
    let is_homing = is_flag_set(FLAG_IS_HOMING, lcp);
    if !is_homing {
        return Ok(());
    }

    // save new sample
    let ticks = timer_read_time();
    lcp.last_sample_ticks = ticks;
    lcp.watchdog_count = 0;

    // do not trigger before homing start time
    let await_homing = is_flag_set(FLAG_AWAIT_HOMING, lcp);
    if await_homing && timer_is_before(ticks, lcp.homing_start_time) {
        return Ok(());
    }
    clear_flag(FLAG_AWAIT_HOMING, lcp);

    // check for safety limit violations
    let is_safety_trigger = sample <= lcp.safety_counts_min || sample >= lcp.safety_counts_max;
    // too much force, this is an error while homing
    if is_safety_trigger {
        trigger_error(lcp, ERROR_SAFETY_RANGE);
        return Ok(());
    }

    // convert sample to grams
    let raw_grams = counts_to_grams(lcp, sample);
    if overflows_int32(raw_grams) {
        trigger_error(lcp, ERROR_OVERFLOW);
        return Ok(());
    }

    // perform filtering
    let filtered_grams: FixedQ16 = sosfilt(lcp.sf, raw_grams as FixedQ16)?;

    // update trigger state
    if filtered_grams.wrapping_abs() >= lcp.trigger_grams_fixed {
        let t = lcp.last_sample_ticks;
        try_trigger(lcp, t);
    }
    Ok(())
}

// Timer callback that monitors for timeouts
unsafe fn watchdog_event(t: *mut Timer) -> R<u8> {
    let lcp = &mut *(t as *mut LoadCellProbe);
    let is_homing = is_flag_set(FLAG_IS_HOMING, lcp);
    let is_homing_trigger = is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp);
    // the watchdog stops when not homing or when trsync becomes triggered
    if !is_homing || is_homing_trigger {
        return Ok(SF_DONE);
    }

    if lcp.watchdog_count > lcp.watchdog_max {
        trigger_error(lcp, ERROR_WATCHDOG);
    }
    lcp.watchdog_count = lcp.watchdog_count.wrapping_add(1);

    // A sample was recently delivered, continue monitoring
    lcp.time.waketime = lcp.time.waketime.wrapping_add(lcp.rest_ticks);
    Ok(SF_RESCHEDULE)
}

fn set_endstop_range(
    lcp: &mut LoadCellProbe,
    safety_counts_min: i32,
    safety_counts_max: i32,
    tare_counts: i32,
    trigger_grams: u32,
    grams_per_count: FixedQ2,
) -> R {
    if safety_counts_max < safety_counts_min {
        shutdown!("Safety range reversed");
    }
    if trigger_grams > MAX_TRIGGER_GRAMS {
        shutdown!("trigger_grams too large");
    }
    // grams_per_count must be a positive fraction in Q2 format
    let one: FixedQ2 = 1 << FIXEDQ2_FRAC_BITS;
    if grams_per_count < 0 || grams_per_count >= one {
        shutdown!("grams_per_count is invalid");
    }
    lcp.safety_counts_min = safety_counts_min;
    lcp.safety_counts_max = safety_counts_max;
    lcp.tare_counts = tare_counts;
    lcp.trigger_grams = trigger_grams;
    lcp.trigger_grams_fixed = (trigger_grams << FIXEDQ16_FRAC_BITS) as FixedQ16;
    lcp.grams_per_count = grams_per_count;
    Ok(())
}

// Create a load_cell_probe
pub fn command_config_load_cell_probe(args: &[u32]) -> R {
    let lcp = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size_of::<LoadCellProbe>() as u16)? as *mut LoadCellProbe) };
    lcp.flags = 0;
    lcp.trigger_ticks = 0;
    lcp.watchdog_max = 0;
    lcp.watchdog_count = 0;
    lcp.sf = sos_filter_oid_lookup(args[1] as u8)?;
    set_endstop_range(lcp, 0, 0, 0, 0, 0)
}
decl_command!(command_config_load_cell_probe, concat!("config_load_cell_probe", " oid=%c sos_filter_oid=%c"));

// Lookup a load_cell_probe
pub fn load_cell_probe_oid_lookup(oid: u8) -> R<*mut LoadCellProbe> {
    Ok(oid_lookup(oid, oid_type())? as *mut LoadCellProbe)
}

// Set the triggering range and tare value
pub fn command_load_cell_probe_set_range(args: &[u32]) -> R {
    let lcp = unsafe { &mut *load_cell_probe_oid_lookup(args[0] as u8)? };
    set_endstop_range(lcp, args[1] as i32, args[2] as i32, args[3] as i32, args[4], args[5] as FixedQ16)
}
decl_command!(
    command_load_cell_probe_set_range,
    concat!(
        "load_cell_probe_set_range",
        " oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i",
        " trigger_grams=%u grams_per_count=%i"
    )
);

// Home an axis
pub fn command_load_cell_probe_home(args: &[u32]) -> R {
    let lcp = unsafe { &mut *load_cell_probe_oid_lookup(args[0] as u8)? };
    sched_del_timer(&mut lcp.time);
    // clear the homing trigger flag
    clear_flag(FLAG_IS_HOMING_TRIGGER, lcp);
    clear_flag(FLAG_IS_HOMING, lcp);
    lcp.trigger_ticks = 0;
    lcp.ts = null_mut();
    // 0 samples indicates homing is finished
    if args[3] == 0 {
        // Disable end stop checking
        return Ok(());
    }
    lcp.ts = trsync_oid_lookup(args[1] as u8)?;
    lcp.trigger_reason = args[2] as u8;
    lcp.error_reason = args[3] as u8;
    lcp.time.waketime = args[4];
    lcp.homing_start_time = args[4];
    lcp.rest_ticks = args[5];
    lcp.watchdog_max = args[6] as u8;
    lcp.watchdog_count = 0;
    lcp.time.func = Some(watchdog_event);
    set_flag(FLAG_IS_HOMING, lcp);
    set_flag(FLAG_AWAIT_HOMING, lcp);
    sched_add_timer(&mut lcp.time)
}
decl_command!(
    command_load_cell_probe_home,
    concat!("load_cell_probe_home oid=%c trsync_oid=%c trigger_reason=%c", " error_reason=%c clock=%u rest_ticks=%u timeout=%u")
);

pub fn command_load_cell_probe_query_state(args: &[u32]) -> R {
    let oid = args[0] as u8;
    let lcp = unsafe { &*load_cell_probe_oid_lookup(args[0] as u8)? };
    sendf!(
        "load_cell_probe_state oid=%c is_homing_trigger=%c trigger_ticks=%u",
        oid,
        is_flag_set(FLAG_IS_HOMING_TRIGGER, lcp) as u8,
        lcp.trigger_ticks
    )
}
decl_command!(command_load_cell_probe_query_state, "load_cell_probe_query_state oid=%c");
