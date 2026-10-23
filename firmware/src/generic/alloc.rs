// Generic implementation of dynamic memory pool
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::ctr::Global;

static DYNMEM_POOL: Global<[u8; 20 * 1024]> = Global::new([0; 20 * 1024]);

// Return the start of memory available for dynamic allocations
pub fn dynmem_start() -> *mut u8 {
    DYNMEM_POOL.ptr() as *mut u8
}

// Return the end of memory available for dynamic allocations
pub fn dynmem_end() -> *mut u8 {
    dynmem_start().wrapping_add(20 * 1024)
}
