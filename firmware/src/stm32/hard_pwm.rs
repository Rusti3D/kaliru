// Hardware PWM support on stm32
//
// Copyright (C) 2021  Michael Kurz <michi.kurz@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::GpioPwm;
use crate::config::CONFIG_CLOCK_FREQ;
#[allow(unused_imports)]
use crate::ctr::{concat, concat_len};
use crate::sched::R;
use crate::{decl_command, decl_constant, shutdown};

const MAX_PWM: u32 = 256;
decl_constant!("PWM_MAX", MAX_PWM);

#[derive(Clone, Copy)]
struct GpioPwmInfo {
    timer: *mut TIM_TypeDef,
    pin: u8,
    channel: u8,
    function: u8,
}
unsafe impl Sync for GpioPwmInfo {}

const fn p(timer: *mut TIM_TypeDef, port: char, num: u32, channel: u8, func: u32) -> GpioPwmInfo {
    GpioPwmInfo { timer, pin: GPIO(port, num) as u8, channel, function: GPIO_FUNCTION(func) as u8 }
}

// (the stm32f0 / f7 / g0 / g4 / h7 tables come with those chips)
#[cfg(CONFIG_MACH_STM32F1)]
static PWM_REGS: &[GpioPwmInfo] = &[
    p(TIM2, 'A', 0, 1, 2),
    p(TIM2, 'A', 1, 2, 2),
    p(TIM2, 'A', 2, 3, 2),
    p(TIM2, 'A', 3, 4, 2),
    p(TIM2, 'A', 15, 1, 1),
    p(TIM2, 'B', 3, 2, 1),
    p(TIM2, 'B', 10, 3, 1),
    p(TIM2, 'B', 11, 4, 1),
    p(TIM3, 'A', 6, 1, 1),
    p(TIM3, 'A', 7, 2, 1),
    p(TIM3, 'B', 0, 3, 1),
    p(TIM3, 'B', 1, 4, 1),
    p(TIM3, 'C', 6, 1, 2),
    p(TIM3, 'C', 7, 2, 2),
    p(TIM3, 'C', 8, 3, 2),
    p(TIM3, 'C', 9, 4, 2),
    p(TIM4, 'D', 12, 1, 2),
    p(TIM4, 'D', 13, 2, 2),
    p(TIM4, 'D', 14, 3, 2),
    p(TIM4, 'D', 15, 4, 2),
    p(TIM4, 'B', 6, 1, 2),
    p(TIM4, 'B', 7, 2, 2),
    p(TIM4, 'B', 8, 3, 2),
    p(TIM4, 'B', 9, 4, 2),
];
// (the C table, its #if blocks as parts)
#[cfg(all(CONFIG_MACH_STM32F4, any(CONFIG_MACH_STM32F401, CONFIG_MACH_STM32F411)))]
const PWM_REGS_0: &[GpioPwmInfo] = &[p(TIM3, 'A', 6, 1, 2), p(TIM3, 'C', 6, 1, 3), p(TIM3, 'A', 7, 2, 2), p(TIM3, 'C', 7, 2, 3), p(TIM3, 'C', 8, 3, 3), p(TIM3, 'C', 9, 4, 3)];
#[cfg(all(CONFIG_MACH_STM32F4, not(any(CONFIG_MACH_STM32F401, CONFIG_MACH_STM32F411))))]
const PWM_REGS_0: &[GpioPwmInfo] = &[];
#[cfg(all(CONFIG_MACH_STM32F4, CONFIG_MACH_STM32F411))]
const PWM_REGS_1: &[GpioPwmInfo] = &[p(TIM9, 'A', 2, 1, 3), p(TIM9, 'A', 3, 2, 3)];
#[cfg(all(CONFIG_MACH_STM32F4, not(CONFIG_MACH_STM32F411)))]
const PWM_REGS_1: &[GpioPwmInfo] = &[];
#[cfg(all(CONFIG_MACH_STM32F4, CONFIG_MACH_STM32F446))]
const PWM_REGS_2: &[GpioPwmInfo] = &[p(TIM2, 'B', 2, 4, 1)];
#[cfg(all(CONFIG_MACH_STM32F4, not(CONFIG_MACH_STM32F446)))]
const PWM_REGS_2: &[GpioPwmInfo] = &[];
#[cfg(all(CONFIG_MACH_STM32F4, any(CONFIG_MACH_STM32F4x5, CONFIG_MACH_STM32F446)))]
const PWM_REGS_3: &[GpioPwmInfo] = &[p(TIM8, 'C', 6, 1, 3), p(TIM8, 'C', 7, 2, 3), p(TIM8, 'C', 8, 3, 3), p(TIM8, 'C', 9, 4, 3), p(TIM11, 'F', 7, 1, 3), p(TIM12, 'B', 14, 1, 9), p(TIM12, 'B', 15, 2, 9), p(TIM13, 'A', 6, 1, 9), p(TIM13, 'F', 8, 1, 9), p(TIM14, 'A', 7, 1, 9), p(TIM14, 'F', 9, 1, 9)];
#[cfg(all(CONFIG_MACH_STM32F4, not(any(CONFIG_MACH_STM32F4x5, CONFIG_MACH_STM32F446))))]
const PWM_REGS_3: &[GpioPwmInfo] = &[];
// Pins that map to all klipper defined STM32F4 procs
#[cfg(CONFIG_MACH_STM32F4)]
const PWM_REGS_4: &[GpioPwmInfo] = &[
    p(TIM1, 'A', 8, 1, 1),
    p(TIM1, 'E', 9, 1, 1),
    p(TIM1, 'A', 9, 2, 1),
    p(TIM1, 'E', 11, 2, 1),
    p(TIM1, 'A', 10, 3, 1),
    p(TIM1, 'E', 13, 3, 1),
    p(TIM1, 'A', 11, 4, 1),
    p(TIM1, 'E', 14, 4, 1),
    p(TIM2, 'A', 5, 1, 1),
    p(TIM2, 'A', 15, 1, 1),
    p(TIM2, 'B', 3, 2, 1),
    p(TIM2, 'B', 10, 3, 1),
    p(TIM2, 'B', 11, 4, 1),
    p(TIM3, 'B', 4, 1, 2),
    p(TIM3, 'B', 5, 2, 2),
    p(TIM3, 'B', 0, 3, 2),
    p(TIM3, 'B', 1, 4, 2),
    p(TIM4, 'B', 6, 1, 2),
    p(TIM4, 'D', 12, 1, 2),
    p(TIM4, 'B', 7, 2, 2),
    p(TIM4, 'D', 13, 2, 2),
    p(TIM4, 'D', 14, 3, 2),
    p(TIM4, 'D', 15, 4, 2),
    p(TIM5, 'A', 0, 1, 2),
    p(TIM5, 'A', 1, 2, 2),
    p(TIM5, 'A', 2, 3, 2),
    p(TIM5, 'A', 3, 4, 2),
];
#[cfg(all(CONFIG_MACH_STM32F4, not(CONFIG_MACH_STM32F411)))]
const PWM_REGS_5: &[GpioPwmInfo] = &[p(TIM9, 'E', 5, 1, 3), p(TIM9, 'E', 6, 2, 3)];
#[cfg(all(CONFIG_MACH_STM32F4, not(not(CONFIG_MACH_STM32F411))))]
const PWM_REGS_5: &[GpioPwmInfo] = &[];
#[cfg(CONFIG_MACH_STM32F4)]
const PWM_REGS_6: &[GpioPwmInfo] = &[
    p(TIM10, 'B', 8, 1, 3),
    p(TIM11, 'B', 9, 1, 3),
];
#[cfg(CONFIG_MACH_STM32F4)]
const PWM_REGS_PARTS: &[&[GpioPwmInfo]] = &[PWM_REGS_0, PWM_REGS_1, PWM_REGS_2, PWM_REGS_3, PWM_REGS_4, PWM_REGS_5, PWM_REGS_6];
#[cfg(CONFIG_MACH_STM32F4)]
static PWM_REGS: [GpioPwmInfo; concat_len(PWM_REGS_PARTS)] = concat(PWM_REGS_PARTS);

fn gpio_timer_setup(pin: u8, cycle_time: u32, val: u32, is_clock_out: bool) -> R<GpioPwm> {
    // Find a valid mapping for this pin. If multiple mappings exist,
    // prefer the first one that is either unused or already compatible.
    let mut found = None;
    let mut pin_found = false;
    let (mut hwpwm_ticks, mut prescaler, mut pwm_val) = (0u32, 0u32, 0u32);
    for pr in PWM_REGS.iter() {
        if pr.pin != pin {
            continue;
        }
        pin_found = true;
        // Map cycle_time to pwm clock divisor
        let pclk = get_pclock_frequency(pr.timer as u32);
        let mut pclock_div = CONFIG_CLOCK_FREQ / pclk;
        if pclock_div > 1 {
            pclock_div /= 2; // Timers run at twice the normal pclock frequency
        }
        let pcycle_time = cycle_time / pclock_div;
        // Convert requested cycle time (cycle_time/CLOCK_FREQ) to actual
        // cycle time (hwpwm_ticks*prescaler*pclock_div/CLOCK_FREQ).
        if !is_clock_out {
            // In normal mode, allow the pulse frequency (cycle_time) to change
            // in order to maintain the requested duty ratio (val/MAX_PWM).
            hwpwm_ticks = MAX_PWM;
            prescaler = pcycle_time / MAX_PWM;
            pwm_val = val;
            if prescaler > u16::MAX as u32 + 1 {
                prescaler = u16::MAX as u32 + 1;
            } else if prescaler < 1 {
                prescaler = 1;
            }
        } else {
            // In clock output mode, allow the pulse width enable duration
            // (val) to change in order to maintain the requested frequency.
            pwm_val = val / pclock_div;
            hwpwm_ticks = pcycle_time;
            prescaler = 1;
            while hwpwm_ticks > u16::MAX as u32 {
                pwm_val /= 2;
                hwpwm_ticks /= 2;
                prescaler *= 2;
            }
        }
        // Pick this pin config if timer is free, or already running with
        // matching timing so this channel can share the same timer base.
        let timer = unsafe { &*pr.timer };
        if timer.CR1.read() & TIM_CR1_CEN == 0
            || (timer.PSC.read() == prescaler.wrapping_sub(1) as u16 as u32
                && timer.ARR.read() == hwpwm_ticks.wrapping_sub(1) as u16 as u32)
        {
            found = Some(pr);
            break;
        }
    }
    // Never found a matching pin
    if !pin_found {
        shutdown!("Not a valid PWM pin");
    }
    // Never found a free pin configuration
    let Some(p) = found else {
        shutdown!("PWM already programmed at different speed");
    };

    gpio_peripheral(p.pin as u32, p.function as u32, 0);

    // Enable requested pwm hardware block
    if !is_enabled_pclock(p.timer as u32) {
        enable_pclock(p.timer as u32);
    }
    let timer = unsafe { &*p.timer };
    // dont alter the timer if it already has an identical configuration
    if timer.CR1.read() & TIM_CR1_CEN == 0 {
        timer.PSC.write(prescaler.wrapping_sub(1));
        timer.ARR.write(hwpwm_ticks.wrapping_sub(1));
        timer.EGR.or(TIM_EGR_UG);
    }

    // Enable requested channel of hardware pwm block
    let channel;
    match p.channel {
        1 => {
            channel = GpioPwm { reg: timer.CCR1.ptr() as *mut Reg<u32> };
            timer.CCER.and(!TIM_CCER_CC1E);
            timer.CCMR1.and(!(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S));
            timer.CCMR1.or(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE);
            gpio_pwm_write(channel, pwm_val as u16);
            timer.CCER.or(TIM_CCER_CC1E);
        }
        2 => {
            channel = GpioPwm { reg: timer.CCR2.ptr() as *mut Reg<u32> };
            timer.CCER.and(!TIM_CCER_CC2E);
            timer.CCMR1.and(!(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S));
            timer.CCMR1.or(TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2FE);
            gpio_pwm_write(channel, pwm_val as u16);
            timer.CCER.or(TIM_CCER_CC2E);
        }
        3 => {
            channel = GpioPwm { reg: timer.CCR3.ptr() as *mut Reg<u32> };
            timer.CCER.and(!TIM_CCER_CC3E);
            timer.CCMR2.and(!(TIM_CCMR2_OC3M | TIM_CCMR2_CC3S));
            timer.CCMR2.or(TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3FE);
            gpio_pwm_write(channel, pwm_val as u16);
            timer.CCER.or(TIM_CCER_CC3E);
        }
        4 => {
            channel = GpioPwm { reg: timer.CCR4.ptr() as *mut Reg<u32> };
            timer.CCER.and(!TIM_CCER_CC4E);
            timer.CCMR2.and(!(TIM_CCMR2_OC4M | TIM_CCMR2_CC4S));
            timer.CCMR2.or(TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4FE);
            gpio_pwm_write(channel, pwm_val as u16);
            timer.CCER.or(TIM_CCER_CC4E);
        }
        _ => shutdown!("Invalid PWM channel"),
    }
    // Enable PWM output
    timer.CR1.or(TIM_CR1_CEN);

    // Advanced timers need MOE enabled.  On standard timers this is a
    // write to reserved memory, but that seems harmless in practice.
    timer.BDTR.write(TIM_BDTR_MOE);

    Ok(channel)
}

pub fn gpio_pwm_setup(pin: u32, cycle_time: u32, val: u16) -> R<GpioPwm> {
    gpio_timer_setup(pin as u8, cycle_time, val as u32, false)
}

pub fn command_stm32_timer_output(args: &[u32]) -> R {
    gpio_timer_setup(args[0] as u8, args[1], args[2], true)?;
    Ok(())
}
decl_command!(command_stm32_timer_output, "stm32_timer_output pin=%u cycle_ticks=%u on_ticks=%hu");

// (the C uint32_t value: a %hu at most, so a u16 here)
pub fn gpio_pwm_write(g: GpioPwm, val: u16) {
    unsafe { &*g.reg }.write(val as u32);
}
