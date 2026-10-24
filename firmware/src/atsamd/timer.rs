// SAMD21 timer interrupt scheduling
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::board::{irq_disable, irq_enable, irq_restore, irq_save};
use crate::generic::armcm_irq::armcm_irq_shutdown;
use crate::generic::timer_irq::timer_dispatch_many;
use crate::sched::R;
use crate::{armcm_enable_irq, decl_init};

#[cfg(CONFIG_MACH_SAMC21)]
mod tcp {
    use super::*;
    pub const TCp: *mut Tc = TC0;
    pub const TCp_IRQn: IRQn_Type = TC0_IRQn;
    pub const TCp_GCLK_ID: u32 = TC0_GCLK_ID;
    pub const ID_TCp: u32 = ID_TC0;
    pub const TCd_GCLK_ID: u32 = TC1_GCLK_ID;
    pub const ID_TCd: u32 = ID_TC1;
}
#[cfg(not(CONFIG_MACH_SAMC21))]
mod tcp {
    use super::*;
    pub const TCp: *mut Tc = TC4;
    pub const TCp_IRQn: IRQn_Type = TC4_IRQn;
    pub const TCp_GCLK_ID: u32 = TC4_GCLK_ID;
    pub const ID_TCp: u32 = ID_TC4;
    pub const TCd_GCLK_ID: u32 = TC3_GCLK_ID;
    pub const ID_TCd: u32 = ID_TC3;
}
use tcp::*;

fn tc() -> &'static TcCount32 {
    unsafe { &*TCp }.COUNT32()
}

// Set the next irq time
fn timer_set(value: u32) {
    tc().CC[0].reg.write(value);
    tc().INTFLAG.reg.write(TC_INTFLAG_MC0 as u8);
}

// Return the current time (in absolute clock ticks).
pub fn timer_read_time() -> u32 {
    #[cfg(CONFIG_MACH_SAMC21)]
    {
        tc().CTRLBSET.reg.or(TC_CTRLBSET_CMD(TC_CTRLBCLR_CMD_READSYNC_Val) as u8);
        while tc().SYNCBUSY.reg.read() & TC_SYNCBUSY_COUNT != 0 {}
    }
    tc().COUNT.reg.read()
}

// Activate timer dispatch as soon as possible
pub fn timer_kick() {
    timer_set(timer_read_time().wrapping_add(50));
}

// IRQ handler
// (C aligns it to 16 bytes, for stable benchmarks only)
#[unsafe(no_mangle)]
pub unsafe extern "C" fn TCp_Handler() {
    irq_disable();
    let next = match timer_dispatch_many() {
        Ok(next) => next,
        Err(s) => armcm_irq_shutdown(s),
    };
    timer_set(next);
    irq_enable();
}

pub fn timer_init() -> R {
    // Supply power and clock to the timer
    enable_pclock(TCd_GCLK_ID, ID_TCd as i32);
    enable_pclock(TCp_GCLK_ID, ID_TCp as i32);

    // Configure the timer
    let tc = tc();
    let flag = irq_save();
    tc.CTRLA.reg.write(0);
    tc.CTRLA.reg.write(TC_CTRLA_MODE_COUNT32 as _);
    armcm_enable_irq!(TCp_Handler, TCp_IRQn, 2);
    tc.INTENSET.reg.write(TC_INTENSET_MC0 as u8);
    tc.COUNT.reg.write(0);
    timer_kick();
    tc.CTRLA.reg.write((TC_CTRLA_MODE_COUNT32 | TC_CTRLA_ENABLE) as _);
    irq_restore(flag);
    Ok(())
}
decl_init!(timer_init);
