// Watchdog handler on STM32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#[cfg(not(CONFIG_MACH_STM32H7))]
use super::internal::IWDG;
// stm32h7 libraries only define IWDG1 and IWDG2
#[cfg(CONFIG_MACH_STM32H7)]
use super::internal::IWDG1 as IWDG;
use crate::sched::R;
use crate::{decl_init, decl_task};

pub fn watchdog_reset() -> R {
    unsafe { &*IWDG }.KR.write(0xAAAA);
    Ok(())
}
decl_task!(watchdog_reset);

pub fn watchdog_init() -> R {
    let iwdg = unsafe { &*IWDG };
    iwdg.KR.write(0x5555);
    iwdg.PR.write(0);
    iwdg.RLR.write(0x0FFF); // 410-512ms timeout (depending on stm32 chip)
    iwdg.KR.write(0xCCCC);
    Ok(())
}
decl_init!(watchdog_init);
