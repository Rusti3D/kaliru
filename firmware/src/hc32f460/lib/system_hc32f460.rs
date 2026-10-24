// lib/hc32f460/mcu/common/system_hc32f460.c: the system clock variable
//
// Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
// (BSD 3-Clause, see lib/hc32f460)
use super::super::internal::*;
use crate::ctr::Global;

// System Clock Frequency (Core Clock) Variable according CMSIS
pub static HRC_VALUE: Global<u32> = Global::new(HRC_16MHz_VALUE);
pub static SystemCoreClock: Global<u32> = Global::new(MRC_VALUE);

// HRC_FREQ_MON() of system_hc32f460.h: the ICG1.HRCFREQSEL monitor
fn HRC_FREQ_MON() -> u32 {
    unsafe { (0x40010684 as *const u32).read_volatile() }
}

// Setup the microcontroller system. Initialize the System and update
// the SystemCoreClock variable. (The CPACR write of __FPU_USED builds is
// left out: Kalico builds soft-float, __FPU_USED is 0.)
pub fn SystemInit() {
    // SystemCoreClock = 168000000ul;
    SystemCoreClockUpdate();
}

pub fn SystemCoreClockUpdate() {
    let sysreg = unsafe { &*M4_SYSREG };
    let hrc = HRC_VALUE.get();
    let clock = SystemCoreClock.get();

    // Select proper HRC_VALUE according to ICG1.HRCFREQSEL bit
    // ICG1.HRCFREQSEL = '0' represent HRC_VALUE = 20000000UL
    // ICG1.HRCFREQSEL = '1' represent HRC_VALUE = 16000000UL
    if 1 == (HRC_FREQ_MON() & 1) {
        *hrc = HRC_16MHz_VALUE;
    } else {
        *hrc = HRC_20MHz_VALUE;
    }

    let tmp = sysreg.CMU_CKSWR.field(stc_sysreg_cmu_ckswr_field_t::CKSW) as u8;
    match tmp {
        0x00 => *clock = *hrc,         // use internal high speed RC
        0x01 => *clock = MRC_VALUE,    // use internal middle speed RC
        0x02 => *clock = LRC_VALUE,    // use internal low speed RC
        0x03 => *clock = XTAL_VALUE,   // use external high speed OSC
        0x04 => *clock = XTAL32_VALUE, // use external low speed OSC
        0x05 => {
            // use MPLL
            // PLLCLK = ((pllsrc / pllm) * plln) / pllp
            let pllsource = sysreg.CMU_PLLCFGR.field(stc_sysreg_cmu_pllcfgr_field_t::PLLSRC);
            let plln = sysreg.CMU_PLLCFGR.field(stc_sysreg_cmu_pllcfgr_field_t::MPLLN);
            let pllp = sysreg.CMU_PLLCFGR.field(stc_sysreg_cmu_pllcfgr_field_t::MPLLP);
            let pllm = sysreg.CMU_PLLCFGR.field(stc_sysreg_cmu_pllcfgr_field_t::MPLLM);
            if 0 == pllsource {
                // use exteranl high speed OSC as PLL source
                *clock = XTAL_VALUE / (pllm + 1) * (plln + 1) / (pllp + 1);
            } else if 1 == pllsource {
                // use interanl high RC as PLL source
                *clock = *hrc / (pllm + 1) * (plln + 1) / (pllp + 1);
            } else {
                // Reserved
            }
        }
        _ => {}
    }
}
