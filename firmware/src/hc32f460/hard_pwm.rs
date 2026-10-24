// Hardware PWM support on HC32F460
//
// Copyright (C) 2022  Steven Gotthardt <gotthardt@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::gpio_h::GpioPwm;
use super::internal::*;
use super::lib::hc32f460_gpio::PORT_SetFunc;
use super::lib::hc32f460_pwc::PWC_Fcg2PeriphClockCmd;
use super::lib::hc32f460_timera::*;
use crate::sched::R;
use crate::{decl_constant, shutdown};

#[allow(dead_code)]
const MAX_PWM: u32 = (1 << 16) - 1;
decl_constant!("PWM_MAX", MAX_PWM);

// timer A (general purpose) has 6 units and each has 8 PWM
// M4_TMRA_TypeDef* timer;

struct GpioPwmInfo {
    gpio: u8,
    unit: u8, // 6 units in Timer A
    chan: u32,
}

const fn info(gpio: u32, unit: u8, chan: u32) -> GpioPwmInfo {
    GpioPwmInfo { gpio: gpio as u8, unit, chan }
}

// Timer A (general purpose) is only timer used
// These are for pin function 4 only
// Some PWM come out on more than 1 pin
// 64pin package
static pwm_mapping: [GpioPwmInfo; 57] = [
    info(GPIO('A', 0), 2, TimeraCh1),
    info(GPIO('A', 1), 2, TimeraCh2),
    info(GPIO('A', 2), 2, TimeraCh3),
    info(GPIO('A', 3), 2, TimeraCh4),
    info(GPIO('A', 5), 2, TimeraCh1),
    info(GPIO('A', 8), 1, TimeraCh1),
    info(GPIO('A', 9), 1, TimeraCh2),
    info(GPIO('A', 10), 1, TimeraCh3),
    info(GPIO('A', 11), 1, TimeraCh4),
    info(GPIO('A', 13), 2, TimeraCh5),
    info(GPIO('A', 14), 2, TimeraCh6),
    info(GPIO('A', 15), 2, TimeraCh1),
    info(GPIO('B', 0), 1, TimeraCh6),
    info(GPIO('B', 1), 1, TimeraCh7),
    info(GPIO('B', 2), 1, TimeraCh8),
    info(GPIO('B', 3), 2, TimeraCh2),
    info(GPIO('B', 4), 3, TimeraCh1),
    info(GPIO('B', 5), 3, TimeraCh2),
    info(GPIO('B', 6), 4, TimeraCh1),
    info(GPIO('B', 7), 4, TimeraCh2),
    info(GPIO('B', 8), 4, TimeraCh3),
    info(GPIO('B', 9), 4, TimeraCh4),
    info(GPIO('B', 10), 2, TimeraCh3),
    info(GPIO('B', 12), 1, TimeraCh8),
    info(GPIO('B', 13), 1, TimeraCh5),
    info(GPIO('B', 14), 1, TimeraCh6),
    info(GPIO('B', 15), 1, TimeraCh7),
    info(GPIO('C', 0), 2, TimeraCh5),
    info(GPIO('C', 1), 2, TimeraCh6),
    info(GPIO('C', 2), 2, TimeraCh7),
    info(GPIO('C', 3), 2, TimeraCh8),
    info(GPIO('C', 6), 3, TimeraCh1),
    info(GPIO('C', 7), 3, TimeraCh2),
    info(GPIO('C', 8), 3, TimeraCh3),
    info(GPIO('C', 9), 3, TimeraCh4),
    info(GPIO('C', 10), 2, TimeraCh7),
    info(GPIO('C', 11), 2, TimeraCh8),
    info(GPIO('C', 13), 4, TimeraCh8),
    info(GPIO('C', 14), 4, TimeraCh5),
    info(GPIO('C', 15), 4, TimeraCh6),
    info(GPIO('D', 2), 2, TimeraCh4),
    info(GPIO('D', 12), 4, TimeraCh1),
    info(GPIO('D', 13), 4, TimeraCh2),
    info(GPIO('D', 14), 4, TimeraCh3),
    info(GPIO('D', 15), 4, TimeraCh4),
    info(GPIO('E', 2), 3, TimeraCh5),
    info(GPIO('E', 3), 3, TimeraCh6),
    info(GPIO('E', 4), 3, TimeraCh7),
    info(GPIO('E', 5), 3, TimeraCh8),
    info(GPIO('E', 8), 1, TimeraCh5),
    info(GPIO('E', 9), 1, TimeraCh1),
    info(GPIO('E', 10), 1, TimeraCh6),
    info(GPIO('E', 11), 1, TimeraCh2),
    info(GPIO('E', 12), 1, TimeraCh7),
    info(GPIO('E', 13), 1, TimeraCh3),
    info(GPIO('E', 14), 1, TimeraCh4),
    info(GPIO('E', 15), 1, TimeraCh8),
];

// (C: the gpio and val arguments are uint8_t, val is not used)
pub fn gpio_pwm_setup(pin: u32, cycle_time: u32, _val: u16) -> R<GpioPwm> {
    let gpio = pin as u8;
    // Find gpio pin in pwm_regs table
    let Some(p) = pwm_mapping.iter().find(|p| p.gpio == gpio) else {
        shutdown!("Not a valid PWM pin");
    };

    // select registers - could make it programmatic
    // All TimerA power control bits are in PWC_FCG2
    let timerA = match p.unit {
        2 => {
            PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA2, Enable);
            M4_TMRA2
        }
        3 => {
            PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA3, Enable);
            M4_TMRA3
        }
        4 => {
            PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA4, Enable);
            M4_TMRA4
        }
        5 => {
            PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA5, Enable);
            M4_TMRA5
        }
        6 => {
            PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA6, Enable);
            M4_TMRA6
        }
        // default, case 1
        _ => {
            PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA1, Enable);
            M4_TMRA1
        }
    };
    let timer = unsafe { &*timerA };

    // set the function - only using #4 (Func_Tima0)
    // (C: the pin number, not its bit mask, is the u16Pin argument)
    PORT_SetFunc(GPIO2PORT(gpio as u32), GPIO2PIN(gpio as u32) as u16, Func_Tima0, Disable);

    /* Configuration timera unit 1 base structure */
    let stcTimeraInit = stc_timera_base_init_t {
        enClkDiv: TimeraPclkDiv128, // 128 chosen - use below
        enCntMode: TimeraCountModeTriangularWave,
        enCntDir: TimeraCountDirUp,
        enSyncStartupEn: Disable,
        u16PeriodVal: (cycle_time / 2 / 128 / 2) as u16,
    };
    TIMERA_BaseInit(timer, &stcTimeraInit);

    /* Configuration timera unit 1 compare structure */
    let u16CompareVal = (stcTimeraInit.u16PeriodVal as u32 * 4 / 5) as u16;
    let stcTimerCompareInit = stc_timera_compare_init_t {
        u16CompareVal,
        enStartCountOutput: TimeraCountStartOutputHigh,
        enStopCountOutput: TimeraCountStopOutputHigh,
        enCompareMatchOutput: TimeraCompareMatchOutputReverse,
        enPeriodMatchOutput: TimeraPeriodMatchOutputKeep,
        enSpecifyOutput: TimeraSpecifyOutputInvalid,
        enCacheEn: Disable,
        enTriangularTroughTransEn: Enable,
        enTriangularCrestTransEn: Disable,
        u16CompareCacheVal: u16CompareVal,
    };
    TIMERA_CompareInit(timer, p.chan, &stcTimerCompareInit);
    TIMERA_CompareCmd(timer, p.chan, Enable);

    // default setup - all disabled
    let stcTimeraHwConfig = stc_timera_hw_startup_config_t::default();
    TIMERA_HwStartupConfig(timer, &stcTimeraHwConfig);

    Ok(GpioPwm { timer: timerA, chan: p.chan })
}

pub fn gpio_pwm_write(g: GpioPwm, val: u16) {
    TIMERA_SetCompareValue(unsafe { &*g.timer }, g.chan, val);
}
