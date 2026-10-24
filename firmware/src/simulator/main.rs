// Main starting point for host simulator.
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::sched::sched_main;

// Main entry point for simulator.
pub fn main() -> i32 {
    sched_main();
}
