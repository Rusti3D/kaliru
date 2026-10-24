// Hardware PWM support on samd21
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioPwm;
use super::internal::*;
use crate::sched::R;
use crate::{decl_constant, shutdown};

// Available TCC devices
struct TccInfo {
    tcc: *mut Tcc,
    pclk_id: u32,
    pm_id: u32,
}
unsafe impl Sync for TccInfo {}

static TCC_INFO: [TccInfo; 3] = [
    TccInfo { tcc: TCC0, pclk_id: TCC0_GCLK_ID, pm_id: ID_TCC0 },
    TccInfo { tcc: TCC1, pclk_id: TCC1_GCLK_ID, pm_id: ID_TCC1 },
    TccInfo { tcc: TCC2, pclk_id: TCC2_GCLK_ID, pm_id: ID_TCC2 },
];

// PWM pins and their TCC device/channel
struct GpioPwmInfo {
    gpio: u8,
    ptype: u8,
    tcc: u8,
    channel: u8,
}

const fn pi(gpio: u32, ptype: char, tcc: u8, channel: u8) -> GpioPwmInfo {
    GpioPwmInfo { gpio: gpio as u8, ptype: ptype as u8, tcc, channel }
}

static PWM_REGS: [GpioPwmInfo; 20] = [
    pi(GPIO('A', 4), 'E', 0, 0),
    pi(GPIO('A', 5), 'E', 0, 1),
    pi(GPIO('A', 6), 'E', 1, 0),
    pi(GPIO('A', 7), 'E', 1, 1),
    pi(GPIO('A', 8), 'E', 0, 0),
    pi(GPIO('A', 9), 'E', 0, 1),
    pi(GPIO('A', 10), 'E', 1, 0),
    pi(GPIO('A', 11), 'E', 1, 1),
    pi(GPIO('A', 12), 'E', 2, 0),
    pi(GPIO('A', 13), 'E', 2, 1),
    pi(GPIO('A', 16), 'E', 2, 0),
    pi(GPIO('A', 17), 'E', 2, 1),
    pi(GPIO('A', 18), 'F', 0, 2),
    pi(GPIO('A', 19), 'F', 0, 3),
    pi(GPIO('A', 24), 'F', 1, 2),
    pi(GPIO('A', 25), 'F', 1, 3),
    pi(GPIO('A', 30), 'E', 1, 0),
    pi(GPIO('A', 31), 'E', 1, 1),
    pi(GPIO('B', 30), 'E', 0, 0),
    pi(GPIO('B', 31), 'E', 0, 1),
];

const MAX_PWM: u32 = 255;
decl_constant!("PWM_MAX", MAX_PWM);

// (C: a uint8_t val, the low byte of the pwmcmds value)
pub fn gpio_pwm_setup(pin: u32, cycle_time: u32, val: u16) -> R<GpioPwm> {
    // Find pin in pwm_regs table
    let mut i = 0;
    let p = loop {
        if i >= PWM_REGS.len() {
            shutdown!("Not a valid PWM pin");
        }
        if PWM_REGS[i].gpio as u32 == pin {
            break &PWM_REGS[i];
        }
        i += 1;
    };

    // Enable timer clock
    let ti = &TCC_INFO[p.tcc as usize];
    enable_pclock(ti.pclk_id, ti.pm_id as i32);

    // Map cycle_time to pwm clock divisor
    let cs: u32 = match cycle_time {
        x if x < (1 + 2) * MAX_PWM / 2 => 0,
        x if x < (2 + 4) * MAX_PWM / 2 => 1,
        x if x < (4 + 8) * MAX_PWM / 2 => 2,
        x if x < (8 + 16) * MAX_PWM / 2 => 3,
        x if x < (16 + 64) * MAX_PWM / 2 => 4,
        x if x < (64 + 256) * MAX_PWM / 2 => 5,
        x if x < (256 + 1024) * MAX_PWM / 2 => 6,
        _ => 7,
    };
    let ctrla = TCC_CTRLA_ENABLE | TCC_CTRLA_PRESCALER(cs);

    // Enable timer
    let tcc = unsafe { &*ti.tcc };
    let old_ctrla = tcc.CTRLA.reg.read();
    if old_ctrla != ctrla {
        if old_ctrla & TCC_CTRLA_ENABLE != 0 {
            shutdown!("PWM already programmed at different speed");
        }
        tcc.CTRLA.reg.write(ctrla & !TCC_CTRLA_ENABLE);
        tcc.WAVE.reg.write(TCC_WAVE_WAVEGEN_NPWM);
        tcc.PER.reg.write(MAX_PWM);
        tcc.CTRLA.reg.write(ctrla);
    }

    // Set initial value
    #[cfg(CONFIG_MACH_SAMD21)]
    let reg = tcc.CCB[p.channel as usize].reg.ptr();
    #[cfg(CONFIG_MACH_SAMC21)]
    let reg = tcc.CCBUF[p.channel as usize].reg.ptr();
    let g = GpioPwm { reg: reg as *mut Reg<u32> };
    gpio_pwm_write(g, val as u8 as u16);

    // Route output to pin
    gpio_peripheral(pin, p.ptype as char, 0);

    Ok(g)
}

// (C: a uint8_t val)
pub fn gpio_pwm_write(g: GpioPwm, val: u16) {
    unsafe { &*g.reg }.write(val as u8 as u32);
}
