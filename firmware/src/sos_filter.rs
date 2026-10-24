// Second Order sections Filter implementation using Fixed Point math
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::basecmd::{oid_alloc, oid_lookup, OidType};
use crate::command::Args;
use crate::sched::R;
use crate::{decl_command, shutdown};
use core::mem::size_of;

type FixedQCoeff = i32;
type FixedQValue = i32;

// filter strucutre sizes
const SECTION_WIDTH: usize = 5;
const STATE_WIDTH: usize = 2;

#[repr(C)]
struct SosFilterSection {
    // filter composed of second order sections
    coeff: [FixedQCoeff; SECTION_WIDTH], // aka sos
    state: [FixedQValue; STATE_WIDTH],   // aka zi
}

#[repr(C)]
pub struct SosFilter {
    max_sections: u8,
    n_sections: u8,
    coeff_frac_bits: u8,
    is_active: u8,
    coeff_rounding: u32,
    // filter composed of second order sections
    filter: [SosFilterSection; 0],
}

impl SosFilter {
    fn section(&mut self, i: usize) -> &mut SosFilterSection {
        unsafe { &mut *self.filter.as_mut_ptr().add(i) }
    }
}

fn oid_type() -> OidType {
    command_config_sos_filter as *const ()
}

#[inline]
fn overflows_int32(value: i64) -> bool {
    value > i32::MAX as i64 || value < i32::MIN as i64
}

// Multiply a coefficient in fixedQ_coeff_t by a value fixedQ_value_t
#[inline]
fn fixed_mul(sf: &SosFilter, coeff: FixedQCoeff, value: FixedQValue) -> R<FixedQValue> {
    // This optimizes to single cycle SMULL on Arm Coretex M0+
    let mut product = coeff as i64 * value as i64;
    // round up at the last bit to be shifted away
    product = product.wrapping_add(sf.coeff_rounding as i64);
    // shift the decimal right to discard the coefficient fractional bits
    let result = product.wrapping_shr(sf.coeff_frac_bits as u32);
    // check for overflow of int32_t
    if overflows_int32(result) {
        shutdown!("fixed_mul: overflow");
    }
    // truncate significant 32 bits
    Ok(result as FixedQValue)
}

// Apply the sosfilt algorithm to a new datapoint
// returns the fixedQ_value_t filtered value
pub fn sosfilt(sf: *mut SosFilter, unfiltered_value: i32) -> R<i32> {
    let sf = unsafe { &mut *sf };
    if sf.is_active == 0 {
        shutdown!("sos_filter not property initialized");
    }

    // an empty filter performs no filtering
    if sf.n_sections == 0 {
        return Ok(unfiltered_value);
    }

    let mut cur_val: FixedQValue = unfiltered_value;
    // foreach section
    for section_idx in 0..sf.n_sections as usize {
        let (c, s) = {
            let section = sf.section(section_idx);
            (section.coeff, section.state)
        };
        // apply the section's filter coefficients to input
        let mut next_val = fixed_mul(sf, c[0], cur_val)?;
        next_val = next_val.wrapping_add(s[0]);
        let state0 = fixed_mul(sf, c[1], cur_val)?.wrapping_sub(fixed_mul(sf, c[3], next_val)?).wrapping_add(s[1]);
        let state1 = fixed_mul(sf, c[2], cur_val)?.wrapping_sub(fixed_mul(sf, c[4], next_val)?);
        let section = sf.section(section_idx);
        section.state[0] = state0;
        section.state[1] = state1;
        cur_val = next_val;
    }

    Ok(cur_val)
}

// Create an sos_filter
pub fn command_config_sos_filter(args: &Args) -> R {
    let max_sections = args[1] as u8;
    let size = size_of::<SosFilter>() + size_of::<SosFilterSection>() * max_sections as usize;
    let sf = unsafe { &mut *(oid_alloc(args[0] as u8, oid_type(), size as u16)? as *mut SosFilter) };
    sf.max_sections = max_sections;
    sf.is_active = 0;
    Ok(())
}
decl_command!(command_config_sos_filter, concat!("config_sos_filter oid=%c", " max_sections=%c"));

// Lookup an sos_filter
pub fn sos_filter_oid_lookup(oid: u8) -> R<*mut SosFilter> {
    Ok(oid_lookup(oid, oid_type())? as *mut SosFilter)
}

// Check that a section index parameter is valid
fn validate_section_index(sf: &SosFilter, section_idx: u8) -> R {
    if section_idx >= sf.max_sections {
        shutdown!("Filter section index larger than max_sections");
    }
    Ok(())
}

// Set one section of the filter
pub fn command_sos_filter_set_section(args: &Args) -> R {
    let sf = unsafe { &mut *sos_filter_oid_lookup(args[0] as u8)? };
    // setting a section marks the filter as inactive
    sf.is_active = 0;
    let section_idx = args[1] as u8;
    validate_section_index(sf, section_idx)?;
    // copy section data
    let arg_base = 2;
    for i in 0..SECTION_WIDTH {
        sf.section(section_idx as usize).coeff[i] = args[i + arg_base] as i32;
    }
    Ok(())
}
decl_command!(
    command_sos_filter_set_section,
    concat!("sos_filter_set_section oid=%c section_idx=%c", " sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i")
);

// Set the state of one section of the filter
pub fn command_sos_filter_set_state(args: &Args) -> R {
    let sf = unsafe { &mut *sos_filter_oid_lookup(args[0] as u8)? };
    // setting a section's state marks the filter as inactive
    sf.is_active = 0;
    // copy state data
    let section_idx = args[1] as u8;
    validate_section_index(sf, section_idx)?;
    let arg_base = 2;
    let section = sf.section(section_idx as usize);
    section.state[0] = args[arg_base] as i32;
    section.state[1] = args[1 + arg_base] as i32;
    Ok(())
}
decl_command!(command_sos_filter_set_state, "sos_filter_set_state oid=%c section_idx=%c state0=%i state1=%i");

// Set one section of the filter
pub fn command_sos_filter_activate(args: &Args) -> R {
    let sf = unsafe { &mut *sos_filter_oid_lookup(args[0] as u8)? };
    let n_sections = args[1] as u8;
    if n_sections > sf.max_sections {
        shutdown!("Filter section index larger than max_sections");
    }
    sf.n_sections = n_sections;
    let coeff_int_bits = args[2] as u8;
    sf.coeff_frac_bits = 31u8.wrapping_sub(coeff_int_bits);
    // an int shift, as in C
    sf.coeff_rounding = 1i32.wrapping_shl((sf.coeff_frac_bits as i32 - 1) as u32) as u32;
    // mark filter as ready to use
    sf.is_active = 1;
    Ok(())
}
decl_command!(command_sos_filter_activate, "sos_filter_set_active oid=%c n_sections=%c coeff_int_bits=%c");
