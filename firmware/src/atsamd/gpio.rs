// samd gpio functions
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::{GpioIn, GpioOut, GpioOutVal, GpioPullUp};
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown};

fn port_group(bank: u32) -> *mut PortGroup {
    unsafe { core::ptr::addr_of_mut!((*PORT).Group[bank as usize]) }
}

/****************************************************************
 * Pin multiplexing
 ****************************************************************/

// (ptype 0: no peripheral function)
pub fn gpio_peripheral(gpio: u32, ptype: char, pull_up: i32) {
    let (bank, bit) = (GPIO2PORT(gpio), gpio % 32);
    let pg = unsafe { &*port_group(bank) };
    if ptype != '\0' {
        let pmux = &pg.PMUX[(bit / 2) as usize].reg;
        let shift = if bit & 1 != 0 { 4 } else { 0 };
        let mask = !(0xfu8 << shift);
        pmux.write((pmux.read() & mask) | (((ptype as u8).wrapping_sub(b'A')) << shift));
    }
    if pull_up != 0 {
        if pull_up > 0 {
            pg.OUTSET.reg.write(1 << bit);
        } else {
            pg.OUTCLR.reg.write(1 << bit);
        }
    }

    pg.PINCFG[bit as usize].reg.write(
        (if ptype != '\0' { PORT_PINCFG_PMUXEN } else { 0 } | if pull_up != 0 { PORT_PINCFG_PULLEN } else { 0 }) as u8,
    );
}

/****************************************************************
 * General Purpose Input Output (GPIO) pins
 ****************************************************************/

#[cfg(CONFIG_MACH_SAMX2)]
mod ports {
    use super::*;
    pub const NUM_PORT: u32 = 2;
    decl_enumeration_range!("pin", "PA0", GPIO('A', 0), 32);
    decl_enumeration_range!("pin", "PB0", GPIO('B', 0), 32);
}
#[cfg(CONFIG_MACH_SAMX5)]
mod ports {
    use super::*;
    pub const NUM_PORT: u32 = 4;
    decl_enumeration_range!("pin", "PA0", GPIO('A', 0), 32);
    decl_enumeration_range!("pin", "PB0", GPIO('B', 0), 32);
    decl_enumeration_range!("pin", "PC0", GPIO('C', 0), 32);
    decl_enumeration_range!("pin", "PD0", GPIO('D', 0), 32);
}
use ports::NUM_PORT;

pub fn gpio_out_setup(pin: u32, val: GpioOutVal) -> R<GpioOut> {
    if GPIO2PORT(pin) >= NUM_PORT {
        shutdown!("Not an output pin");
    }
    let g = GpioOut { regs: port_group(GPIO2PORT(pin)), bit: GPIO2BIT(pin) };
    gpio_out_reset(g, val)?;
    Ok(g)
}

fn set_pincfg(pg: &PortGroup, bit: u32, cfg: u8) {
    pg.PINCFG[bit.trailing_zeros() as usize].reg.write(cfg);
}

pub fn gpio_out_reset(g: GpioOut, val: GpioOutVal) -> R {
    let pg = unsafe { &*g.regs };
    let flag = irq_save();
    if val != 0 {
        pg.OUTSET.reg.write(g.bit);
    } else {
        pg.OUTCLR.reg.write(g.bit);
    }
    pg.DIRSET.reg.write(g.bit);
    set_pincfg(pg, g.bit, 0);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_out_toggle_noirq(g: GpioOut) {
    unsafe { &*g.regs }.OUTTGL.reg.write(g.bit);
}

pub fn gpio_out_toggle(g: GpioOut) {
    gpio_out_toggle_noirq(g);
}

pub fn gpio_out_write(g: GpioOut, val: u8) {
    let pg = unsafe { &*g.regs };
    if val != 0 {
        pg.OUTSET.reg.write(g.bit);
    } else {
        pg.OUTCLR.reg.write(g.bit);
    }
}

pub fn gpio_in_setup(pin: u32, pull_up: GpioPullUp) -> R<GpioIn> {
    if GPIO2PORT(pin) >= NUM_PORT {
        shutdown!("Not an input pin");
    }
    let g = GpioIn { regs: port_group(GPIO2PORT(pin)), bit: GPIO2BIT(pin) };
    gpio_in_reset(g, pull_up)?;
    Ok(g)
}

pub fn gpio_in_reset(g: GpioIn, pull_up: GpioPullUp) -> R {
    let pg = unsafe { &*g.regs };
    let flag = irq_save();
    let mut cfg = PORT_PINCFG_INEN;
    if pull_up != 0 {
        cfg |= PORT_PINCFG_PULLEN;
        if pull_up > 0 {
            pg.OUTSET.reg.write(g.bit);
        } else {
            pg.OUTCLR.reg.write(g.bit);
        }
    }
    set_pincfg(pg, g.bit, cfg as u8);
    pg.DIRCLR.reg.write(g.bit);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_in_read(g: GpioIn) -> u8 {
    (unsafe { &*g.regs }.IN.reg.read() & g.bit != 0) as u8
}
