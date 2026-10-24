// Hardware PWM support on lpc176x
//
// Copyright (C) 2021  Michael Kurz <michi.kurz@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioPwm;
use super::internal::*;
use crate::config::CONFIG_CLOCK_FREQ;
use crate::sched::R;
use crate::{decl_constant, shutdown};

const MAX_PWM: u32 = 255;
decl_constant!("PWM_MAX", MAX_PWM);

struct GpioPwmInfo {
    pin: u8,
    channel: u8,
    function: u8,
    reg: *mut Reg<u32>,
}
unsafe impl Sync for GpioPwmInfo {}

fn pwm1() -> &'static LPC_PWM_TypeDef {
    unsafe { &*LPC_PWM1 }
}

// (&LPC_PWM1->MRn)
const fn mr(off: usize) -> *mut Reg<u32> {
    (LPC_PWM1 as *mut u8).wrapping_add(off) as *mut Reg<u32>
}

const fn info(pin: u32, channel: u8, function: u8, reg: *mut Reg<u32>) -> GpioPwmInfo {
    GpioPwmInfo { pin: pin as u8, channel, function, reg }
}

macro_rules! mr {
    ($f:ident) => {
        mr(core::mem::offset_of!(LPC_PWM_TypeDef, $f))
    };
}

static PWM_REGS: [GpioPwmInfo; 14] = [
    info(GPIO(1, 18), 1, 2, mr!(MR1)),
    info(GPIO(2, 0), 1, 1, mr!(MR1)),
    info(GPIO(1, 20), 2, 2, mr!(MR2)),
    info(GPIO(2, 1), 2, 1, mr!(MR2)),
    info(GPIO(3, 25), 2, 3, mr!(MR2)),
    info(GPIO(1, 21), 3, 2, mr!(MR3)),
    info(GPIO(2, 2), 3, 1, mr!(MR3)),
    info(GPIO(3, 26), 3, 3, mr!(MR3)),
    info(GPIO(1, 23), 4, 2, mr!(MR4)),
    info(GPIO(2, 3), 4, 1, mr!(MR4)),
    info(GPIO(1, 24), 5, 2, mr!(MR5)),
    info(GPIO(2, 4), 5, 1, mr!(MR5)),
    info(GPIO(1, 26), 6, 2, mr!(MR6)),
    info(GPIO(2, 5), 6, 1, mr!(MR6)),
];

// (C: a uint8_t val, the low byte of the pwmcmds value)
pub fn gpio_pwm_setup(pin: u32, cycle_time: u32, val: u16) -> R<GpioPwm> {
    // Find pin in pwm_regs table
    let mut i = 0;
    let p = loop {
        if i >= PWM_REGS.len() {
            shutdown!("Not a valid PWM pin");
        }
        if PWM_REGS[i].pin as u32 == pin {
            break &PWM_REGS[i];
        }
        i += 1;
    };

    // Map cycle_time to pwm clock divisor
    let clock_div = CONFIG_CLOCK_FREQ / get_pclock_frequency(PCLK_PWM1);
    let mut prescaler = (cycle_time / clock_div) / (MAX_PWM - 1);
    if prescaler > 0 {
        prescaler -= 1;
    }

    gpio_peripheral(p.pin as u32, p.function as u32, 0);

    // Enable clock
    if !is_enabled_pclock(PCLK_PWM1) {
        enable_pclock(PCLK_PWM1);
    }

    let pwm = pwm1();
    if pwm.TCR.read() & 0x1 != 0 {
        if pwm.PR.read() != prescaler {
            shutdown!("PWM already programmed at different speed");
        }
    } else {
        pwm.PCR.write(0);
        pwm.PR.write(prescaler as u16 as u32);
        pwm.MR0.write(MAX_PWM - 1);
        pwm.TCR.write(0x2); // Reset PWM TC and PC
        pwm.MCR.write(0x2); // Reset Counter on MR0 match
        pwm.TCR.write(0x8); // Enable PWM mode
        pwm.TCR.or(0x1); // Enable PWM peripheral
    }

    if pwm.PCR.read() & (1 << (p.channel + 8)) != 0 {
        shutdown!("PWM output already in use");
    }

    // Setup channel
    let channel = GpioPwm { reg: p.reg, channel: p.channel };
    gpio_pwm_write(channel, val as u8 as u16);

    // Enable channel
    pwm.PCR.or(1 << (p.channel + 8));

    Ok(channel)
}

pub fn gpio_pwm_write(g: GpioPwm, val: u16) {
    unsafe { &*g.reg }.write(val as u32);
    pwm1().LER.or(1 << g.channel);
}
