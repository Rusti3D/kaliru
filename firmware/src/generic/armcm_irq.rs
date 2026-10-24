// Definitions for irq enable/disable on ARM Cortex-M processors
//
// Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::internal::__CORTEX_M;
use crate::decl_shutdown;
use crate::generic::armcm_boot::_stack_end;
use crate::sched::{sched_main_loop, Shutdown, R};
use core::arch::asm;

pub type IrqStatus = u32;

pub fn irq_disable() {
    unsafe { asm!("cpsid i", options(nostack)) }
}

pub fn irq_enable() {
    unsafe { asm!("cpsie i", options(nostack)) }
}

pub fn irq_save() -> IrqStatus {
    let flag: IrqStatus;
    unsafe { asm!("mrs {}, primask", out(reg) flag, options(nostack)) };
    irq_disable();
    flag
}

pub fn irq_restore(flag: IrqStatus) {
    unsafe { asm!("msr primask, {}", in(reg) flag, options(nostack)) }
}

pub fn irq_wait() -> R {
    if __CORTEX_M == 7 {
        // Cortex-m7 may disable cpu counter on wfi, so use nop
        unsafe { asm!("cpsie i", "nop", "cpsid i", options(nostack)) }
    } else {
        unsafe { asm!("cpsie i", "wfi", "cpsid i", options(nostack)) }
    }
    Ok(())
}

pub fn irq_poll() -> R {
    Ok(())
}

// Clear the active irq if a shutdown happened in an irq handler
pub fn clear_active_irq() -> R {
    let mut psr: u32;
    unsafe { asm!("mrs {}, xpsr", out(reg) psr, options(nomem, nostack)) };
    if psr & 0x1ff == 0 {
        // Shutdown did not occur in an irq - nothing to do.
        return Ok(());
    }
    // Clear active irq status
    psr = 1 << 24; // T-bit
    unsafe {
        asm!(
            "push {{ {psr} }}",
            "adr {temp}, 1f",
            "push {{ {temp} }}",
            "push {{ r0, r1, r2, r3, r4, lr }}",
            "bx {exc}",
            ".balign 4",
            "1:",
            temp = out(reg) _,
            psr = in(reg) psr,
            exc = in(reg) 0xfffffff9u32,
            out("r12") _,
        )
    }
    Ok(())
}
decl_shutdown!(clear_active_irq);

// A shutdown inside an irq handler. C's sched_shutdown() longjmps from the
// handler back into sched_main() (still in handler mode, which
// clear_active_irq() above then leaves); here the handler passes the
// Shutdown to this function, which drops the stack the same way and
// continues in the sched_main() loop.
pub fn armcm_irq_shutdown(s: Shutdown) -> ! {
    unsafe {
        // (a bx: a thumb-1 b only reaches 2KB)
        asm!(
            "mov sp, {stack}",
            "bx {entry}",
            stack = in(reg) core::ptr::addr_of!(_stack_end),
            entry = in(reg) armcm_irq_shutdown_entry as extern "C" fn(u32) -> !,
            in("r0") s.0 as u32,
            options(noreturn),
        )
    }
}

extern "C" fn armcm_irq_shutdown_entry(reason: u32) -> ! {
    sched_main_loop(Some(reason as u8))
}
