// Hardware PWM support on rp2040
//
// Copyright (C) 2021  Lasse Dalegaard <dalegaard@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioPwm;
use super::internal::*;
use crate::config::CONFIG_CLOCK_FREQ;
use crate::sched::R;
use crate::{decl_constant, shutdown};

const MAX_PWM: u32 = 1 << 15;
decl_constant!("PWM_MAX", MAX_PWM);

pub fn gpio_pwm_setup(pin: u32, cycle_time: u32, val: u16) -> R<GpioPwm> {
    if pin >= 30 {
        shutdown!("invalid gpio pin");
    }

    // All pins on the rp2040 can be used for PWM, there are 8 PWM
    // slices and each has two channels.
    let slice = unsafe { &(*pwm_hw).slice[((pin >> 1) & 0x7) as usize] };
    let channel = pin & 1;

    // The rp2040 has an 8.4 fractional divider, so we'll map the requested
    // cycle time into that. The cycle_time we receive from Klippy is in
    // relation to the crystal frequency and so we need to scale it up to match
    // the PWM clock.
    let pclk = get_pclock_frequency(RESETS_RESET_PWM_BITS);
    let mult_16ticks = (16u64 * pclk as u64 / CONFIG_CLOCK_FREQ as u64) as u32;
    let mut cycle_16ticks_64 = cycle_time as u64 * mult_16ticks as u64;
    if cycle_16ticks_64 > 0x80000000 {
        // Host never sends large cycle_time, so no need for elaborate handling
        cycle_16ticks_64 = 0x80000000;
    }
    let cycle_16ticks = cycle_16ticks_64 as u32;

    // Convert requested cycle time (cycle_time/CLOCK_FREQ) to actual
    // cycle time (hwpwm_ticks*(pwm_div/16)/FREQ_SYS).
    let mut shift = 4;
    let (mut hwpwm_ticks, mut pwm_div);
    loop {
        hwpwm_ticks = (cycle_16ticks + (1 << (shift - 1))) >> shift;
        pwm_div = 1 << shift;
        if hwpwm_ticks <= u16::MAX as u32 {
            break;
        }
        shift += 1;
    }
    if pwm_div > 0xfff {
        pwm_div = 0xfff;
    }
    if hwpwm_ticks < 2 {
        hwpwm_ticks = 2;
    }

    // Enable clock
    if !is_enabled_pclock(RESETS_RESET_PWM_BITS) {
        enable_pclock(RESETS_RESET_PWM_BITS);
    }

    // If this PWM slice hasn't been set up yet, we do the full set
    // up cycle. If it's already been set up however, we check that
    // the cycle time requested now matches the cycle time already
    // set on the slice. This allows both channels to be utilized,
    // as long as their cycle times are the same.
    if slice.csr.read() & PWM_CH0_CSR_EN_BITS == 0 {
        slice.div.write(pwm_div);
        slice.top.write(hwpwm_ticks - 1);
        slice.ctr.write(PWM_CH0_CTR_RESET);
        slice.cc.write(PWM_CH0_CC_RESET);
        slice.csr.write(PWM_CH0_CSR_EN_BITS);
    } else {
        if slice.div.read() != pwm_div || slice.top.read() != hwpwm_ticks - 1 {
            shutdown!("PWM pin has different cycle time from another in the same slice");
        }

        // PWM is already enabled on this slice, we'll check if the
        // aliasing GPIO pin is already set up for PWM function. If it
        // is, then we need to bail out.
        let alias_pin = (!pin & 0x10) | (pin & 0xF);
        let alias_ctrl = unsafe { &*iobank0_hw }.io[alias_pin as usize].ctrl.read();
        let alias_func = alias_ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS;
        if alias_func == IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PWM_A_0 {
            shutdown!("Aliasing PWM pin already has PWM enabled");
        }
    }

    let out = GpioPwm {
        reg: slice.cc.ptr() as *mut Reg<u32>,
        hwpwm_ticks,
        shift: if channel != 0 { PWM_CH0_CC_B_LSB } else { PWM_CH0_CC_A_LSB },
        mask: if channel != 0 { PWM_CH0_CC_B_BITS } else { PWM_CH0_CC_A_BITS },
    };

    gpio_peripheral(pin, IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PWM_A_0, 0)?;
    gpio_pwm_write(out, val);

    Ok(out)
}

pub fn gpio_pwm_write(g: GpioPwm, val: u16) {
    // DIV_ROUND_CLOSEST(val * g.hwpwm_ticks, MAX_PWM)
    let r = (val as u32).wrapping_mul(g.hwpwm_ticks).wrapping_add(MAX_PWM / 2) / MAX_PWM;
    hw_write_masked(unsafe { &*g.reg }, r << g.shift, g.mask);
}
