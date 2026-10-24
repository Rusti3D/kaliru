// Initialization of AVR watchdog timer.
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::chip::{addr, WDCE, WDE, WDIE, WDP3, WDTCSR};
use super::irq::irq_disable;
use crate::command::Args;
use crate::ctr::Global;
use crate::sched::{sched_shutdown, R};
use crate::{decl_command, decl_init, decl_task, static_string};
use core::arch::{asm, global_asm};

// (avr/wdt.h)
const WDTO_15MS: u8 = 0;
const WDTO_500MS: u8 = 5;

// avr-libc's wdt_enable() (avr/wdt.h, a memory mapped watchdog control
// register): the change enable write, then the new value within 4 cycles
fn wdt_enable(value: u8) {
    let wdce_wde: u8 = (1 << WDCE) | (1 << WDE);
    let v: u8 = (if value & 0x08 != 0 { 1 << WDP3 } else { 0 }) | (1 << WDE) | (value & 0x07);
    unsafe {
        // (C: __tmp_reg__, r0, which LLVM reserves; any free one here)
        asm!(
            "in {sreg}, 0x3f",
            "cli",
            "wdr",
            "st Z, {wdce_wde}",
            "out 0x3f, {sreg}",
            "st Z, {v}",
            sreg = out(reg) _,
            wdce_wde = in(reg) wdce_wde,
            v = in(reg) v,
            in("Z") WDTCSR,
        )
    }
}

// wdt_reset()
fn wdt_reset() {
    unsafe { asm!("wdr") }
}

static WATCHDOG_SHUTDOWN: Global<u8> = Global::new(0);

#[export_name = avr_vector!(WDT_vect)]
pub unsafe extern "avr-interrupt" fn wdt_isr() {
    *WATCHDOG_SHUTDOWN.get() = 1;
    sched_shutdown(static_string!("Watchdog timer!"));
}

pub fn watchdog_reset() -> R {
    wdt_reset();
    let ws = WATCHDOG_SHUTDOWN.get();
    if *ws != 0 {
        unsafe { (*WDTCSR).write(1 << WDIE) };
        *ws = 0;
    }
    Ok(())
}
decl_task!(watchdog_reset);

pub fn watchdog_init() -> R {
    // 0.5s timeout, interrupt and system reset
    wdt_enable(WDTO_500MS);
    unsafe { (*WDTCSR).write(1 << WDIE) };
    Ok(())
}
decl_init!(watchdog_init);

// Very early reset of the watchdog: MCUSR = 0; wdt_disable(). (C: a naked
// function in .init3, which avr-libc's startup code runs through, before
// the ram is set up; r1 is the zero register by then)
global_asm!(
    ".pushsection .init3,\"ax\",@progbits",
    ".global watchdog_early_init",
    "watchdog_early_init:",
    "sts {mcusr}, r1",
    "in r0, 0x3f",
    "cli",
    "wdr",
    "lds r24, {wdtcsr}",
    "ori r24, {wdce_wde}",
    "sts {wdtcsr}, r24",
    "sts {wdtcsr}, r1",
    "out 0x3f, r0",
    ".popsection",
    mcusr = const addr::MCUSR,
    wdtcsr = const addr::WDTCSR,
    wdce_wde = const (1u8 << WDCE) | (1u8 << WDE),
);

// Support reset on AVR via the watchdog timer
pub fn command_reset(_args: &Args) -> R {
    irq_disable();
    wdt_enable(WDTO_15MS);
    #[allow(clippy::empty_loop)]
    loop {}
}
decl_command!(command_reset, HF_IN_SHUTDOWN, "reset");
