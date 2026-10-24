// Definitions for irq enable/disable on AVR (avr/irq.h)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::chip::SREG;
use crate::sched::{sched_main_loop, R};
use core::arch::asm;
use core::sync::atomic::{compiler_fence, Ordering};

pub type IrqStatus = u8;

#[inline(always)]
pub fn irq_disable() {
    unsafe { asm!("cli") }
}

#[inline(always)]
pub fn irq_enable() {
    unsafe { asm!("sei") }
}

#[inline(always)]
pub fn irq_save() -> IrqStatus {
    let flag = unsafe { (*SREG).read() };
    irq_disable();
    flag
}

#[inline(always)]
pub fn irq_restore(flag: IrqStatus) {
    compiler_fence(Ordering::SeqCst);
    unsafe { (*SREG).write(flag) }
}

#[inline(always)]
pub fn irq_wait() -> R {
    unsafe { asm!("sei", "nop", "cli") };
    Ok(())
}

#[inline(always)]
pub fn irq_poll() -> R {
    Ok(())
}

// sched_shutdown(): C longjmps back into sched_main() (irqs stay disabled;
// from an irq handler its reti is not run). Here the stack is dropped the
// same way (to its top, RAMEND, where avr-libc's startup put it) and the
// sched_main() loop is entered again with the reason. (SPH / SPL are io
// registers 0x3e / 0x3d on every AVR.)
pub fn shutdown_jump(reason: u8) -> ! {
    let top = super::chip::RAMEND;
    unsafe {
        asm!(
            "out 0x3e, {hi}",
            "out 0x3d, {lo}",
            "ijmp",
            hi = in(reg) (top >> 8) as u8,
            lo = in(reg) top as u8,
            in("r24") reason,
            in("Z") shutdown_entry as extern "C" fn(u8) -> !,
            options(noreturn),
        )
    }
}

extern "C" fn shutdown_entry(reason: u8) -> ! {
    sched_main_loop(Some(reason))
}
