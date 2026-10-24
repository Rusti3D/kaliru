// Hardware PWM support on atsam
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioPwm;
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::ctr::{concat, concat_len};
use crate::sched::R;
use crate::{decl_constant, shutdown};

const MAX_PWM: u32 = 255;
decl_constant!("PWM_MAX", MAX_PWM);

/****************************************************************
 * TC hardware device
 ****************************************************************/

#[derive(Clone, Copy)]
struct GpioTcInfo {
    gpio: u8,
    ptype: char,
    id: u8,
    reg: *mut Reg<u32>,
}
unsafe impl Sync for GpioTcInfo {}

// &TCn->TC_CHANNEL[ch].TC_Rx
const fn tc_reg(tc: *mut Tc, ch: usize, b: bool) -> *mut Reg<u32> {
    let off = core::mem::offset_of!(Tc, TC_CHANNEL)
        + ch * core::mem::size_of::<TcChannel>()
        + if b { core::mem::offset_of!(TcChannel, TC_RB) } else { core::mem::offset_of!(TcChannel, TC_RA) };
    (tc as *mut u8).wrapping_add(off) as *mut Reg<u32>
}

const fn tci(gpio: u32, ptype: char, id: u32, tc: *mut Tc, ch: usize, b: bool) -> GpioTcInfo {
    GpioTcInfo { gpio: gpio as u8, ptype, id: id as u8, reg: tc_reg(tc, ch, b) }
}

#[cfg(CONFIG_MACH_SAM3X)]
const TC_REGS_BASE: &[GpioTcInfo] = &[
    tci(GPIO('B', 25), 'B', ID_TC0, TC0, 0, false),
    tci(GPIO('A', 2), 'A', ID_TC1, TC0, 1, false),
    tci(GPIO('A', 5), 'A', ID_TC2, TC0, 2, false),
    tci(GPIO('B', 27), 'B', ID_TC0, TC0, 0, true),
    tci(GPIO('A', 3), 'A', ID_TC1, TC0, 1, true),
    tci(GPIO('A', 6), 'A', ID_TC2, TC0, 2, true),
    tci(GPIO('B', 0), 'B', ID_TC3, TC1, 0, false),
    tci(GPIO('B', 2), 'B', ID_TC4, TC1, 1, false),
    tci(GPIO('B', 4), 'B', ID_TC5, TC1, 2, false),
    tci(GPIO('B', 1), 'B', ID_TC3, TC1, 0, true),
    tci(GPIO('B', 3), 'B', ID_TC4, TC1, 1, true),
    tci(GPIO('B', 5), 'B', ID_TC5, TC1, 2, true),
];
#[cfg(CONFIG_MACH_SAM3X8E)]
const TC_REGS_EXTRA: &[GpioTcInfo] = &[
    tci(GPIO('C', 25), 'B', ID_TC6, TC2, 0, false),
    tci(GPIO('C', 28), 'B', ID_TC7, TC2, 1, false),
    tci(GPIO('D', 7), 'B', ID_TC8, TC2, 2, false),
    tci(GPIO('C', 26), 'B', ID_TC6, TC2, 0, true),
    tci(GPIO('C', 29), 'B', ID_TC7, TC2, 1, true),
    tci(GPIO('D', 8), 'B', ID_TC8, TC2, 2, true),
];
#[cfg(CONFIG_MACH_SAM4)]
const TC_REGS_BASE: &[GpioTcInfo] = &[
    tci(GPIO('A', 0), 'B', ID_TC0, TC0, 0, false),
    tci(GPIO('A', 15), 'B', ID_TC1, TC0, 1, false),
    tci(GPIO('A', 26), 'B', ID_TC2, TC0, 2, false),
    tci(GPIO('A', 1), 'B', ID_TC0, TC0, 0, true),
    tci(GPIO('A', 16), 'B', ID_TC1, TC0, 1, true),
    tci(GPIO('A', 27), 'B', ID_TC2, TC0, 2, true),
    tci(GPIO('C', 23), 'B', ID_TC3, TC1, 0, false),
    tci(GPIO('C', 26), 'B', ID_TC4, TC1, 1, false),
    tci(GPIO('C', 29), 'B', ID_TC5, TC1, 2, false),
    tci(GPIO('C', 24), 'B', ID_TC3, TC1, 0, true),
    tci(GPIO('C', 27), 'B', ID_TC4, TC1, 1, true),
    tci(GPIO('C', 30), 'B', ID_TC5, TC1, 2, true),
];
#[cfg(CONFIG_MACH_SAM4E8E)]
const TC_REGS_EXTRA: &[GpioTcInfo] = &[
    tci(GPIO('C', 5), 'B', ID_TC6, TC2, 0, false),
    tci(GPIO('C', 8), 'B', ID_TC7, TC2, 1, false),
    tci(GPIO('C', 11), 'B', ID_TC8, TC2, 2, false),
    tci(GPIO('C', 6), 'B', ID_TC6, TC2, 0, true),
    tci(GPIO('C', 9), 'B', ID_TC7, TC2, 1, true),
    tci(GPIO('C', 12), 'B', ID_TC8, TC2, 2, true),
];
#[cfg(not(any(CONFIG_MACH_SAM3X8E, CONFIG_MACH_SAM4E8E)))]
const TC_REGS_EXTRA: &[GpioTcInfo] = &[];
const TC_REGS_PARTS: &[&[GpioTcInfo]] = &[TC_REGS_BASE, TC_REGS_EXTRA];
static TC_REGS: [GpioTcInfo; concat_len(TC_REGS_PARTS)] = concat(TC_REGS_PARTS);

fn tc_is_tc(g: GpioPwm) -> bool {
    (g.reg as u32 & !0xffff) == (TC0 as u32 & !0xffff)
}
fn tc_from_reg(g: GpioPwm) -> &'static TcChannel {
    unsafe { &*((g.reg as u32 & !0x3f) as *const TcChannel) }
}
fn tc_is_b(g: GpioPwm) -> bool {
    (g.reg as u32 & 0x3f) == (tc_reg(TC0, 0, true) as u32 & 0x3f)
}

fn gpio_tc_write(g: GpioPwm, val: u32) {
    let tc = tc_from_reg(g);
    let mut mask = TC_CMR_ACPA_Msk | TC_CMR_ACPC_Msk;
    let mut bits = TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET;
    if val == 0 {
        bits = TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR;
    } else if val >= MAX_PWM {
        bits = TC_CMR_ACPA_SET | TC_CMR_ACPC_SET;
    }
    if tc_is_b(g) {
        mask <<= 8;
        bits <<= 8;
    }
    let flag = irq_save();
    tc.TC_CMR.write((tc.TC_CMR.read() & !mask) | bits);
    unsafe { &*g.reg }.write(val);
    irq_restore(flag);
}

fn gpio_tc_setup(pin: u32, cycle_time: u32, val: u8) -> R<GpioPwm> {
    // Find pin in tc_regs table
    let mut i = 0;
    let p = loop {
        if i >= TC_REGS.len() {
            shutdown!("Not a valid PWM pin");
        }
        if TC_REGS[i].gpio as u32 == pin {
            break &TC_REGS[i];
        }
        i += 1;
    };

    // Map cycle_time to clock divisor
    let mut div = TC_CMR_TCCLKS_TIMER_CLOCK4;
    if cycle_time < (MAX_PWM * 8 + MAX_PWM * 2) / 2 {
        div = TC_CMR_TCCLKS_TIMER_CLOCK1;
    } else if cycle_time < (MAX_PWM * 32 + MAX_PWM * 8) / 2 {
        div = TC_CMR_TCCLKS_TIMER_CLOCK2;
    } else if cycle_time < (MAX_PWM * 128 + MAX_PWM * 32) / 2 {
        div = TC_CMR_TCCLKS_TIMER_CLOCK3;
    }

    // Enable clock
    enable_pclock(p.id as u32);

    // Enable PWM output
    let g = GpioPwm { reg: p.reg };
    let tc = tc_from_reg(g);
    let prev_cmr = tc.TC_CMR.read();
    if prev_cmr != 0 && (prev_cmr & TC_CMR_TCCLKS_Msk) != div {
        shutdown!("PWM already programmed at different speed");
    }
    gpio_peripheral(pin, p.ptype, 0);
    if prev_cmr != 0 {
        gpio_tc_write(g, val as u32);
    } else {
        tc.TC_CMR.write(TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | div | TC_CMR_EEVT_XC0);
        gpio_tc_write(g, val as u32);
        tc.TC_RC.write(MAX_PWM);
        tc.TC_CCR.write(TC_CCR_CLKEN | TC_CCR_SWTRG);
    }
    Ok(g)
}

/****************************************************************
 * PWM hardware device
 ****************************************************************/

#[derive(Clone, Copy)]
struct GpioPwmInfo {
    gpio: u8,
    channel: u8,
    ptype: char,
}

const fn pi(gpio: u32, channel: u8, ptype: char) -> GpioPwmInfo {
    GpioPwmInfo { gpio: gpio as u8, channel, ptype }
}

#[cfg(CONFIG_MACH_SAM3X)]
const PWM_REGS_BASE: &[GpioPwmInfo] = &[
    pi(GPIO('A', 21), 0, 'B'),
    pi(GPIO('B', 16), 0, 'B'),
    pi(GPIO('A', 12), 1, 'B'),
    pi(GPIO('B', 17), 1, 'B'),
    pi(GPIO('A', 20), 2, 'B'),
    pi(GPIO('B', 18), 2, 'B'),
    pi(GPIO('A', 0), 3, 'B'),
    pi(GPIO('B', 19), 3, 'B'),
];
#[cfg(CONFIG_MACH_SAM3X8E)]
const PWM_REGS_EXTRA: &[GpioPwmInfo] = &[
    pi(GPIO('C', 2), 0, 'B'),
    pi(GPIO('C', 4), 1, 'B'),
    pi(GPIO('C', 6), 2, 'B'),
    pi(GPIO('C', 8), 3, 'B'),
    pi(GPIO('C', 21), 4, 'B'),
    pi(GPIO('C', 22), 5, 'B'),
    pi(GPIO('C', 23), 6, 'B'),
    pi(GPIO('C', 24), 7, 'B'),
];
#[cfg(not(CONFIG_MACH_SAM3X8E))]
const PWM_REGS_EXTRA: &[GpioPwmInfo] = &[];
#[cfg(CONFIG_MACH_SAM4)]
const PWM_REGS_BASE: &[GpioPwmInfo] = &[
    pi(GPIO('A', 19), 0, 'B'),
    pi(GPIO('B', 5), 0, 'B'),
    pi(GPIO('C', 0), 0, 'B'),
    pi(GPIO('C', 13), 0, 'B'),
    pi(GPIO('A', 20), 1, 'B'),
    pi(GPIO('B', 12), 1, 'A'),
    pi(GPIO('C', 1), 1, 'B'),
    pi(GPIO('C', 15), 1, 'B'),
    pi(GPIO('A', 16), 2, 'C'),
    pi(GPIO('A', 30), 2, 'A'),
    pi(GPIO('B', 13), 2, 'A'),
    pi(GPIO('C', 2), 2, 'B'),
    pi(GPIO('A', 15), 3, 'C'),
    pi(GPIO('C', 3), 3, 'B'),
    pi(GPIO('C', 22), 3, 'B'),
];
const PWM_REGS_PARTS: &[&[GpioPwmInfo]] = &[PWM_REGS_BASE, PWM_REGS_EXTRA];
static PWM_REGS: [GpioPwmInfo; concat_len(PWM_REGS_PARTS)] = concat(PWM_REGS_PARTS);

// (C: a uint8_t val, the low byte of the pwmcmds value)
pub fn gpio_pwm_setup(pin: u32, cycle_time: u32, val: u16) -> R<GpioPwm> {
    let val = val as u8;
    // Find pin in pwm_regs table
    let mut i = 0;
    let p = loop {
        if i >= PWM_REGS.len() {
            return gpio_tc_setup(pin, cycle_time, val);
        }
        if PWM_REGS[i].gpio as u32 == pin {
            break &PWM_REGS[i];
        }
        i += 1;
    };

    // Map cycle_time to pwm clock divisor
    let mut div = 0;
    while div < 10 {
        if cycle_time < ((MAX_PWM << (div + 1)) + (MAX_PWM << div)) / 2 {
            break;
        }
        div += 1;
    }

    // Enable clock
    enable_pclock(ID_PWM);

    // Enable PWM output
    let pwm = unsafe { &*PWM };
    if pwm.PWM_SR.read() & (1 << p.channel) != 0 {
        shutdown!("PWM channel already in use");
    }
    gpio_peripheral(pin, p.ptype, 0);
    let ch = &pwm.PWM_CH_NUM[p.channel as usize];
    ch.PWM_CMR.write(div);
    ch.PWM_CPRD.write(MAX_PWM);
    ch.PWM_CDTY.write(val as u32);
    pwm.PWM_ENA.write(1 << p.channel);

    Ok(GpioPwm { reg: &ch.PWM_CDTYUPD as *const Reg<u32> as *mut Reg<u32> })
}

pub fn gpio_pwm_write(g: GpioPwm, val: u16) {
    if tc_is_tc(g) {
        gpio_tc_write(g, val as u32);
    } else {
        unsafe { &*g.reg }.write(val as u32);
    }
}
