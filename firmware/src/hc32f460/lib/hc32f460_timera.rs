// lib/hc32f460/driver/src/hc32f460_timera.c: timer A
//
// Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
// (BSD 3-Clause, see lib/hc32f460)
use super::super::internal::*;

// Timera base init structure (hc32f460_timera.h)
pub struct stc_timera_base_init_t {
    pub enClkDiv: u32,
    pub enCntMode: u32,
    pub enCntDir: u32,
    pub enSyncStartupEn: u32,
    pub u16PeriodVal: u16,
}

// Timera compare init structure (hc32f460_timera.h)
pub struct stc_timera_compare_init_t {
    pub u16CompareVal: u16,
    pub enStartCountOutput: u32,
    pub enStopCountOutput: u32,
    pub enCompareMatchOutput: u32,
    pub enPeriodMatchOutput: u32,
    pub enSpecifyOutput: u32,
    pub enCacheEn: u32,
    pub enTriangularTroughTransEn: u32,
    pub enTriangularCrestTransEn: u32,
    pub u16CompareCacheVal: u16,
}

// Timera hardware startup config structure (hc32f460_timera.h)
#[derive(Default)]
pub struct stc_timera_hw_startup_config_t {
    pub enTrigRisingStartupEn: u32,
    pub enTrigFallingStartupEn: u32,
    pub enSpecifyEventStartupEn: u32,
}

// Parameter valid check for normal timera unit
fn IS_VALID_NORMAL_TIMERA_UNIT(x: &M4_TMRA_TypeDef) -> bool {
    [M4_TMRA1, M4_TMRA2, M4_TMRA3, M4_TMRA4, M4_TMRA5, M4_TMRA6].iter().any(|&t| core::ptr::eq(t, x))
}

// TIMERA_CALC_REG_ADDR(reg, chl): the register `chl` 4 byte steps after reg
fn TIMERA_CALC_REG_ADDR(reg: &Reg<u16>, chl: u32) -> &Reg<u16> {
    unsafe { &*((reg.ptr() as u32 + chl * 0x4) as *const Reg<u16>) }
}

// Timera base function initialize
pub fn TIMERA_BaseInit(TIMERAx: &M4_TMRA_TypeDef, pstcBaseInit: &stc_timera_base_init_t) {
    use stc_tmra_bcstr_field_t as bcstr;

    if IS_VALID_NORMAL_TIMERA_UNIT(TIMERAx) {
        // Configure control status register
        TIMERAx.BCSTR.set_field(bcstr::CKDIV, pstcBaseInit.enClkDiv);
        TIMERAx.BCSTR.set_field(bcstr::MODE, pstcBaseInit.enCntMode);
        TIMERAx.BCSTR.set_field(bcstr::DIR, pstcBaseInit.enCntDir);

        // Unit 1 sync startup invalid
        if !(core::ptr::eq(M4_TMRA1, TIMERAx) && Enable == pstcBaseInit.enSyncStartupEn) {
            TIMERAx.BCSTR.set_field(bcstr::SYNST, pstcBaseInit.enSyncStartupEn);
        }

        // Configure period value register
        TIMERAx.PERAR.write(pstcBaseInit.u16PeriodVal);
    }
}

// Timera compare function initialize
pub fn TIMERA_CompareInit(TIMERAx: &M4_TMRA_TypeDef, enChannel: u32, pstcCompareInit: &stc_timera_compare_init_t) {
    use stc_tmra_bconr_field_t as bconr;
    use stc_tmra_pconr_field_t as pconr;

    if IS_VALID_NORMAL_TIMERA_UNIT(TIMERAx) {
        // Configure port control register
        let pstcTimeraPort = TIMERA_CALC_REG_ADDR(&TIMERAx.PCONR1, enChannel);
        pstcTimeraPort.set_field(pconr::STAC, pstcCompareInit.enStartCountOutput);
        pstcTimeraPort.set_field(pconr::STPC, pstcCompareInit.enStopCountOutput);
        pstcTimeraPort.set_field(pconr::CMPC, pstcCompareInit.enCompareMatchOutput);
        pstcTimeraPort.set_field(pconr::PERC, pstcCompareInit.enPeriodMatchOutput);
        pstcTimeraPort.set_field(pconr::FORC, pstcCompareInit.enSpecifyOutput);

        // Configure cache control register
        if TimeraCh1 == enChannel || TimeraCh3 == enChannel || TimeraCh5 == enChannel || TimeraCh7 == enChannel {
            let pstcTimeraCache = TIMERA_CALC_REG_ADDR(&TIMERAx.BCONR1, enChannel);
            pstcTimeraCache.set_field(bconr::BSE0, pstcCompareInit.enTriangularCrestTransEn);
            pstcTimeraCache.set_field(bconr::BSE1, pstcCompareInit.enTriangularTroughTransEn);
            pstcTimeraCache.set_field(bconr::BEN, pstcCompareInit.enCacheEn);
            // Configure compare cache value register
            let pstcTimeraCompare = TIMERA_CALC_REG_ADDR(&TIMERAx.CMPAR1, enChannel + 1);
            pstcTimeraCompare.set_field(stc_tmra_cmpar_field_t::CMP, pstcCompareInit.u16CompareCacheVal as u32);
        }

        // Configure compare value register
        let pstcTimeraCompare = TIMERA_CALC_REG_ADDR(&TIMERAx.CMPAR1, enChannel);
        pstcTimeraCompare.set_field(stc_tmra_cmpar_field_t::CMP, pstcCompareInit.u16CompareVal as u32);

        // Set compare output function
        let pstcTimeraCapture = TIMERA_CALC_REG_ADDR(&TIMERAx.CCONR1, enChannel);
        pstcTimeraCapture.set_field(stc_tmra_cconr_field_t::CAPMD, 0);
    }
}

// Configure timera compare value
pub fn TIMERA_SetCompareValue(TIMERAx: &M4_TMRA_TypeDef, enChannel: u32, u16CompareVal: u16) {
    if IS_VALID_NORMAL_TIMERA_UNIT(TIMERAx) {
        let pstcTimeraCompare = TIMERA_CALC_REG_ADDR(&TIMERAx.CMPAR1, enChannel);
        pstcTimeraCompare.set_field(stc_tmra_cmpar_field_t::CMP, u16CompareVal as u32);
    }
}

// Enable or disable timera compare function
pub fn TIMERA_CompareCmd(TIMERAx: &M4_TMRA_TypeDef, enChannel: u32, enNewSta: u32) {
    if IS_VALID_NORMAL_TIMERA_UNIT(TIMERAx) {
        let pstcTimeraPort = TIMERA_CALC_REG_ADDR(&TIMERAx.PCONR1, enChannel);
        pstcTimeraPort.set_field(stc_tmra_pconr_field_t::OUTEN, enNewSta);
    }
}

// Configure timera hardware startup
pub fn TIMERA_HwStartupConfig(TIMERAx: &M4_TMRA_TypeDef, pstcHwStartup: &stc_timera_hw_startup_config_t) {
    use stc_tmra_hconr_field_t as hconr;

    if IS_VALID_NORMAL_TIMERA_UNIT(TIMERAx) {
        TIMERAx.HCONR.set_field(hconr::HSTA0, pstcHwStartup.enTrigRisingStartupEn);
        TIMERAx.HCONR.set_field(hconr::HSTA1, pstcHwStartup.enTrigFallingStartupEn);
        TIMERAx.HCONR.set_field(hconr::HSTA2, pstcHwStartup.enSpecifyEventStartupEn);
    }
}
