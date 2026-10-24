// Watchdog handler on SAMD21 boards
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::sched::R;
use crate::{decl_init, decl_task};

fn wdt() -> &'static Wdt {
    unsafe { &*WDT }
}

pub fn watchdog_reset() -> R {
    if wdt().SYNCBUSY.reg.read() & WDT_SYNCBUSY_CLEAR == 0 {
        wdt().CLEAR.reg.write(0xa5);
    }
    Ok(())
}
decl_task!(watchdog_reset);

pub fn watchdog_init() -> R {
    #[cfg(CONFIG_MACH_SAMC21)]
    wdt().CONFIG.reg.write(WDT_CONFIG_PER_CYC512 as u8); // 500ms timeout
    #[cfg(CONFIG_MACH_SAMX5)]
    wdt().CONFIG.reg.write(WDT_CONFIG_PER(6) as u8); // 500ms timeout
    wdt().CTRLA.reg.write(WDT_CTRLA_ENABLE as u8);
    Ok(())
}
decl_init!(watchdog_init);
