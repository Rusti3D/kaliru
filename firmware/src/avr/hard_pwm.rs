// Hardware PWM pin support
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioPwm;
use super::internal::*;
use super::irq::{irq_restore, irq_save};
use super::pgm::readp;
use crate::ctr::{concat, concat_len};
use crate::sched::R;
use crate::{decl_constant, shutdown};

#[derive(Clone, Copy)]
struct GpioPwmInfo {
    pin: u8,
    ocr: PgmPtr<u8>,
    rega: PgmPtr<Reg<u8>>,
    regb: PgmPtr<Reg<u8>>,
    en_bit: u8,
    flags: u8,
}

const GP_8BIT: u8 = 1;
const GP_AFMT: u8 = 2;

const fn p(pin: u8, ocr: *mut u8, rega: *mut Reg<u8>, regb: *mut Reg<u8>, en_bit: u8, flags: u8) -> GpioPwmInfo {
    GpioPwmInfo { pin, ocr: PgmPtr(ocr), rega: PgmPtr(rega), regb: PgmPtr(regb), en_bit, flags }
}

#[cfg(any(CONFIG_MACH_atmega168, CONFIG_MACH_atmega328, CONFIG_MACH_atmega328p))]
const PWM_REGS_PARTS: &[&[GpioPwmInfo]] = &[&[
    p(GPIO('D', 6), OCR0A as *mut u8, TCCR0A, TCCR0B, 1 << COM0A1, GP_8BIT),
    p(GPIO('D', 5), OCR0B as *mut u8, TCCR0A, TCCR0B, 1 << COM0B1, GP_8BIT),
    p(GPIO('B', 1), OCR1A as *mut u8, TCCR1A, TCCR1B, 1 << COM1A1, 0),
    p(GPIO('B', 2), OCR1B as *mut u8, TCCR1A, TCCR1B, 1 << COM1B1, 0),
    p(GPIO('B', 3), OCR2A as *mut u8, TCCR2A, TCCR2B, 1 << COM2A1, GP_8BIT | GP_AFMT),
    p(GPIO('D', 3), OCR2B as *mut u8, TCCR2A, TCCR2B, 1 << COM2B1, GP_8BIT | GP_AFMT),
]];
#[cfg(any(CONFIG_MACH_atmega644p, CONFIG_MACH_atmega1284p))]
const PWM_REGS_012: &[GpioPwmInfo] = &[
    p(GPIO('B', 3), OCR0A as *mut u8, TCCR0A, TCCR0B, 1 << COM0A1, GP_8BIT),
    p(GPIO('B', 4), OCR0B as *mut u8, TCCR0A, TCCR0B, 1 << COM0B1, GP_8BIT),
    p(GPIO('D', 5), OCR1A as *mut u8, TCCR1A, TCCR1B, 1 << COM1A1, 0),
    p(GPIO('D', 4), OCR1B as *mut u8, TCCR1A, TCCR1B, 1 << COM1B1, 0),
    p(GPIO('D', 7), OCR2A as *mut u8, TCCR2A, TCCR2B, 1 << COM2A1, GP_8BIT | GP_AFMT),
    p(GPIO('D', 6), OCR2B as *mut u8, TCCR2A, TCCR2B, 1 << COM2B1, GP_8BIT | GP_AFMT),
];
#[cfg(all(any(CONFIG_MACH_atmega644p, CONFIG_MACH_atmega1284p), has_OCR3A))]
const PWM_REGS_3: &[GpioPwmInfo] = &[
    p(GPIO('B', 6), OCR3A as *mut u8, TCCR3A, TCCR3B, 1 << COM3A1, 0),
    p(GPIO('B', 7), OCR3B as *mut u8, TCCR3A, TCCR3B, 1 << COM3B1, 0),
];
#[cfg(all(any(CONFIG_MACH_atmega644p, CONFIG_MACH_atmega1284p), not(has_OCR3A)))]
const PWM_REGS_3: &[GpioPwmInfo] = &[];
#[cfg(any(CONFIG_MACH_atmega644p, CONFIG_MACH_atmega1284p))]
const PWM_REGS_PARTS: &[&[GpioPwmInfo]] = &[PWM_REGS_012, PWM_REGS_3];
#[cfg(any(CONFIG_MACH_at90usb1286, CONFIG_MACH_at90usb646, CONFIG_MACH_atmega32u4))]
const PWM_REGS_01: &[GpioPwmInfo] = &[
    p(GPIO('B', 7), OCR0A as *mut u8, TCCR0A, TCCR0B, 1 << COM0A1, GP_8BIT),
    p(GPIO('D', 0), OCR0B as *mut u8, TCCR0A, TCCR0B, 1 << COM0B1, GP_8BIT),
    p(GPIO('B', 5), OCR1A as *mut u8, TCCR1A, TCCR1B, 1 << COM1A1, 0),
    p(GPIO('B', 6), OCR1B as *mut u8, TCCR1A, TCCR1B, 1 << COM1B1, 0),
    p(GPIO('B', 7), OCR1C as *mut u8, TCCR1A, TCCR1B, 1 << COM1C1, 0),
];
#[cfg(all(any(CONFIG_MACH_at90usb1286, CONFIG_MACH_at90usb646, CONFIG_MACH_atmega32u4), has_OCR2A))]
const PWM_REGS_2: &[GpioPwmInfo] = &[
    p(GPIO('B', 4), OCR2A as *mut u8, TCCR2A, TCCR2B, 1 << COM2A1, GP_8BIT | GP_AFMT),
    p(GPIO('D', 1), OCR2B as *mut u8, TCCR2A, TCCR2B, 1 << COM2B1, GP_8BIT | GP_AFMT),
];
#[cfg(all(any(CONFIG_MACH_at90usb1286, CONFIG_MACH_at90usb646, CONFIG_MACH_atmega32u4), not(has_OCR2A)))]
const PWM_REGS_2: &[GpioPwmInfo] = &[];
#[cfg(any(CONFIG_MACH_at90usb1286, CONFIG_MACH_at90usb646, CONFIG_MACH_atmega32u4))]
const PWM_REGS_3: &[GpioPwmInfo] = &[
    p(GPIO('C', 6), OCR3A as *mut u8, TCCR3A, TCCR3B, 1 << COM3A1, 0),
    p(GPIO('C', 5), OCR3B as *mut u8, TCCR3A, TCCR3B, 1 << COM3B1, 0),
    p(GPIO('C', 4), OCR3C as *mut u8, TCCR3A, TCCR3B, 1 << COM3C1, 0),
];
#[cfg(any(CONFIG_MACH_at90usb1286, CONFIG_MACH_at90usb646, CONFIG_MACH_atmega32u4))]
const PWM_REGS_PARTS: &[&[GpioPwmInfo]] = &[PWM_REGS_01, PWM_REGS_2, PWM_REGS_3];
#[cfg(any(CONFIG_MACH_atmega1280, CONFIG_MACH_atmega2560))]
const PWM_REGS_PARTS: &[&[GpioPwmInfo]] = &[&[
    p(GPIO('B', 7), OCR0A as *mut u8, TCCR0A, TCCR0B, 1 << COM0A1, GP_8BIT),
    p(GPIO('G', 5), OCR0B as *mut u8, TCCR0A, TCCR0B, 1 << COM0B1, GP_8BIT),
    p(GPIO('B', 5), OCR1A as *mut u8, TCCR1A, TCCR1B, 1 << COM1A1, 0),
    p(GPIO('B', 6), OCR1B as *mut u8, TCCR1A, TCCR1B, 1 << COM1B1, 0),
    p(GPIO('B', 7), OCR1C as *mut u8, TCCR1A, TCCR1B, 1 << COM1C1, 0),
    p(GPIO('B', 4), OCR2A as *mut u8, TCCR2A, TCCR2B, 1 << COM2A1, GP_8BIT | GP_AFMT),
    p(GPIO('H', 6), OCR2B as *mut u8, TCCR2A, TCCR2B, 1 << COM2B1, GP_8BIT | GP_AFMT),
    p(GPIO('E', 3), OCR3A as *mut u8, TCCR3A, TCCR3B, 1 << COM3A1, 0),
    p(GPIO('E', 4), OCR3B as *mut u8, TCCR3A, TCCR3B, 1 << COM3B1, 0),
    p(GPIO('E', 5), OCR3C as *mut u8, TCCR3A, TCCR3B, 1 << COM3C1, 0),
    p(GPIO('H', 3), OCR4A as *mut u8, TCCR4A, TCCR4B, 1 << COM4A1, 0),
    p(GPIO('H', 4), OCR4B as *mut u8, TCCR4A, TCCR4B, 1 << COM4B1, 0),
    p(GPIO('H', 5), OCR4C as *mut u8, TCCR4A, TCCR4B, 1 << COM4C1, 0),
    p(GPIO('L', 3), OCR5A as *mut u8, TCCR5A, TCCR5B, 1 << COM5A1, 0),
    p(GPIO('L', 4), OCR5B as *mut u8, TCCR5A, TCCR5B, 1 << COM5B1, 0),
    p(GPIO('L', 5), OCR5C as *mut u8, TCCR5A, TCCR5B, 1 << COM5C1, 0),
]];

#[link_section = ".progmem.data"]
static PWM_REGS: [GpioPwmInfo; concat_len(PWM_REGS_PARTS)] = concat(PWM_REGS_PARTS);

decl_constant!("PWM_MAX", 255);

// (the cycle_time limits of the pwm clock divisors)
const fn ct(a: u32, b: u32) -> u32 {
    (a + b) * 510 / 2
}

pub fn gpio_pwm_setup(pin: u32, cycle_time: u32, val: u16) -> R<GpioPwm> {
    let pin = pin as u8;
    // Find pin in pwm_regs table
    let mut i = 0;
    loop {
        if i >= PWM_REGS.len() {
            shutdown!("Not a valid PWM pin");
        }
        if readp(&PWM_REGS[i].pin) == pin {
            break;
        }
        i += 1;
    }
    let p = &PWM_REGS[i];

    // Map cycle_time to pwm clock divisor
    let flags = readp(&p.flags);
    let cs: u8 = if flags & GP_AFMT != 0 {
        match cycle_time {
            c if c < ct(1, 8) => 1,
            c if c < ct(8, 32) => 2,
            c if c < ct(32, 64) => 3,
            c if c < ct(64, 128) => 4,
            c if c < ct(128, 256) => 5,
            c if c < ct(256, 1024) => 6,
            _ => 7,
        }
    } else {
        match cycle_time {
            c if c < ct(1, 8) => 1,
            c if c < ct(8, 64) => 2,
            c if c < ct(64, 256) => 3,
            c if c < ct(256, 1024) => 4,
            _ => 5,
        }
    };
    let rega = readp(&p.rega).0;
    let regb = readp(&p.regb).0;
    let en_bit = readp(&p.en_bit);
    let gpio_regs = GPIO2REGS(pin);
    let gpio_bit = GPIO2BIT(pin);
    let g = GpioPwm { reg: readp(&p.ocr).0, size8: flags & GP_8BIT };
    if rega == TCCR1A {
        shutdown!("Can not use timer1 for PWM; timer1 is used for timers");
    }

    // Setup PWM timer
    let flag = irq_save();
    unsafe {
        let old_cs = (*regb).read() & 0x07;
        if old_cs != 0 && old_cs != cs {
            shutdown!("PWM already programmed at different speed");
        }
        (*regb).write(cs);

        // Set default value and enable output
        gpio_pwm_write(g, val);
        (*rega).write((*rega).read() | (1 << WGM00) | en_bit);
        (*gpio_regs).mode.write((*gpio_regs).mode.read() | gpio_bit);
    }
    irq_restore(flag);

    Ok(g)
}

pub fn gpio_pwm_write(g: GpioPwm, val: u16) {
    // (the C prototype's uint8_t val)
    let val = val as u8;
    if g.size8 != 0 {
        unsafe { (g.reg as *mut u8).write_volatile(val) };
    } else {
        let flag = irq_save();
        unsafe { (g.reg as *mut u16).write_volatile(val as u16) };
        irq_restore(flag);
    }
}
