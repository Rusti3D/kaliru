// GPIO functions on AVR.
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::{GpioIn, GpioOut, GpioOutVal, GpioPullUp};
use super::internal::*;
use super::irq::{irq_restore, irq_save};
use crate::ctr::{concat, concat_len};
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown};
use core::ptr::null_mut;

#[cfg(has_PINA)]
decl_enumeration_range!("pin", "PA0", GPIO('A', 0), 8);
decl_enumeration_range!("pin", "PB0", GPIO('B', 0), 8);
decl_enumeration_range!("pin", "PC0", GPIO('C', 0), 8);
decl_enumeration_range!("pin", "PD0", GPIO('D', 0), 8);
#[cfg(CONFIG_MACH_atmega328p)]
decl_enumeration_range!("pin", "PE0", GPIO('E', 0), 8);
#[cfg(has_PINE)]
decl_enumeration_range!("pin", "PE0", GPIO('E', 0), 8);
#[cfg(has_PINE)]
decl_enumeration_range!("pin", "PF0", GPIO('F', 0), 8);
#[cfg(has_PING)]
decl_enumeration_range!("pin", "PG0", GPIO('G', 0), 8);
#[cfg(has_PING)]
decl_enumeration_range!("pin", "PH0", GPIO('H', 0), 8);
#[cfg(has_PING)]
decl_enumeration_range!("pin", "PJ0", GPIO('J', 0), 8);
#[cfg(has_PING)]
decl_enumeration_range!("pin", "PK0", GPIO('K', 0), 8);
#[cfg(has_PING)]
decl_enumeration_range!("pin", "PL0", GPIO('L', 0), 8);

type P = PgmPtr<GpioDigitalRegs>;
const fn regs(pin: *mut Reg<u8>) -> P {
    PgmPtr(pin as *mut GpioDigitalRegs)
}
#[cfg(has_PINA)]
const REGS_A: &[P] = &[regs(PINA)];
#[cfg(not(has_PINA))]
const REGS_A: &[P] = &[PgmPtr(null_mut())];
const REGS_BCD: &[P] = &[regs(PINB), regs(PINC), regs(PIND)];
// (PINE on atmega328pb)
#[cfg(CONFIG_MACH_atmega328p)]
const REGS_328PB: &[P] = &[PgmPtr((0x0C + __SFR_OFFSET as usize) as *mut GpioDigitalRegs)];
#[cfg(not(CONFIG_MACH_atmega328p))]
const REGS_328PB: &[P] = &[];
#[cfg(has_PINE)]
const REGS_EF: &[P] = &[regs(PINE), regs(PINF)];
#[cfg(not(has_PINE))]
const REGS_EF: &[P] = &[];
#[cfg(has_PING)]
const REGS_GL: &[P] = &[regs(PING), regs(PINH), PgmPtr(null_mut()), regs(PINJ), regs(PINK), regs(PINL)];
#[cfg(not(has_PING))]
const REGS_GL: &[P] = &[];
const DIGITAL_REGS_PARTS: &[&[P]] = &[REGS_A, REGS_BCD, REGS_328PB, REGS_EF, REGS_GL];
#[link_section = ".progmem.data"]
pub static DIGITAL_REGS: [P; concat_len(DIGITAL_REGS_PARTS)] = concat(DIGITAL_REGS_PARTS);

pub fn gpio_out_setup(pin: u32, val: GpioOutVal) -> R<GpioOut> {
    let pin = pin as u8;
    'fail: {
        if GPIO2PORT(pin) as usize >= DIGITAL_REGS.len() {
            break 'fail;
        }
        let regs = GPIO2REGS(pin);
        if regs.is_null() {
            break 'fail;
        }
        let g = GpioOut { regs, bit: GPIO2BIT(pin) };
        gpio_out_reset(g, val)?;
        return Ok(g);
    }
    shutdown!("Not an output pin");
}

pub fn gpio_out_reset(g: GpioOut, val: GpioOutVal) -> R {
    let regs = unsafe { &*g.regs };
    let flag = irq_save();
    let newmode = regs.mode.read() | g.bit;
    let newout = if val != 0 { regs.out.read() | g.bit } else { regs.out.read() & !g.bit };
    regs.out.write(newout);
    regs.mode.write(newmode);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_out_toggle_noirq(g: GpioOut) {
    unsafe { (*g.regs).in_.write(g.bit) }
}

pub fn gpio_out_toggle(g: GpioOut) {
    gpio_out_toggle_noirq(g);
}

pub fn gpio_out_write(g: GpioOut, val: u8) {
    let regs = unsafe { &*g.regs };
    let flag = irq_save();
    regs.out.write(if val != 0 { regs.out.read() | g.bit } else { regs.out.read() & !g.bit });
    irq_restore(flag);
}

pub fn gpio_in_setup(pin: u32, pull_up: GpioPullUp) -> R<GpioIn> {
    let pin = pin as u8;
    'fail: {
        if GPIO2PORT(pin) as usize >= DIGITAL_REGS.len() || pull_up < 0 {
            break 'fail;
        }
        let regs = GPIO2REGS(pin);
        if regs.is_null() {
            break 'fail;
        }
        let g = GpioIn { regs, bit: GPIO2BIT(pin) };
        gpio_in_reset(g, pull_up)?;
        return Ok(g);
    }
    shutdown!("Not a valid input pin");
}

pub fn gpio_in_reset(g: GpioIn, pull_up: GpioPullUp) -> R {
    let regs = unsafe { &*g.regs };
    let flag = irq_save();
    let newout = if pull_up > 0 { regs.out.read() | g.bit } else { regs.out.read() & !g.bit };
    let newmode = regs.mode.read() & !g.bit;
    regs.mode.write(newmode);
    regs.out.write(newout);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_in_read(g: GpioIn) -> u8 {
    (unsafe { (*g.regs).in_.read() } & g.bit != 0) as u8
}
