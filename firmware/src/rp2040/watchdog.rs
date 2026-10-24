// Watchdog code on rp2040
//
// Copyright (C) 2021-2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::sched::R;
use crate::{decl_init, decl_task};

pub fn watchdog_reset() -> R {
    unsafe { &*watchdog_hw }.load.write(0x800000); // ~350ms
    Ok(())
}
decl_task!(watchdog_reset);

pub fn watchdog_init() -> R {
    unsafe { &*psm_hw }.wdsel.write(PSM_WDSEL_BITS & !(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
    watchdog_reset()?;
    unsafe { &*watchdog_hw }.ctrl.write(
        WATCHDOG_CTRL_PAUSE_DBG0_BITS | WATCHDOG_CTRL_PAUSE_DBG1_BITS | WATCHDOG_CTRL_PAUSE_JTAG_BITS | WATCHDOG_CTRL_ENABLE_BITS,
    );
    Ok(())
}
decl_init!(watchdog_init);
