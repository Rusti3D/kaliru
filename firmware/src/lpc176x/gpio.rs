// GPIO functions on lpc176x
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::{GpioIn, GpioOut, GpioOutVal, GpioPullUp};
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown};

/****************************************************************
 * Pin mappings
 ****************************************************************/

decl_enumeration_range!("pin", "P0.0", GPIO(0, 0), 32);
decl_enumeration_range!("pin", "P1.0", GPIO(1, 0), 32);
decl_enumeration_range!("pin", "P2.0", GPIO(2, 0), 32);
decl_enumeration_range!("pin", "P3.0", GPIO(3, 0), 32);
decl_enumeration_range!("pin", "P4.0", GPIO(4, 0), 32);

const DIGITAL_REGS: [*mut LPC_GPIO_TypeDef; 5] = [LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4];

// Set the mode and extended function of a pin
pub fn gpio_peripheral(gpio: u32, func: u32, pull_up: i32) {
    let (mut bank_pos, mut pin_pos) = (GPIO2PORT(gpio) * 2, (gpio % 32) * 2);
    if pin_pos >= 32 {
        pin_pos -= 32;
        bank_pos += 1;
    }
    let sel_bits = (func & 0x03) << pin_pos;
    let mask = !(0x03 << pin_pos);
    let mode = (if pull_up != 0 {
        if pull_up > 0 {
            0x00
        } else {
            0x03
        }
    } else {
        0x02
    }) << pin_pos;
    let pincon = unsafe { &*LPC_PINCON };
    let pinsel = pincon.PINSEL0.ptr();
    let pinmode = pincon.PINMODE0.ptr();
    let flag = irq_save();
    unsafe {
        let ps = pinsel.add(bank_pos as usize);
        ps.write_volatile((ps.read_volatile() & mask) | sel_bits);
        let pm = pinmode.add(bank_pos as usize);
        pm.write_volatile((pm.read_volatile() & mask) | mode);
    }
    irq_restore(flag);
}

// Convert a register and bit location back to an integer pin identifier
fn regs_to_pin(regs: *mut LPC_GPIO_TypeDef, bit: u32) -> u32 {
    for (i, &r) in DIGITAL_REGS.iter().enumerate() {
        if r == regs {
            return GPIO(i as u32, bit.trailing_zeros());
        }
    }
    0
}

/****************************************************************
 * General Purpose Input Output (GPIO) pins
 ****************************************************************/

pub fn gpio_out_setup(pin: u32, val: GpioOutVal) -> R<GpioOut> {
    if GPIO2PORT(pin) as usize >= DIGITAL_REGS.len() {
        shutdown!("Not an output pin");
    }
    let regs = DIGITAL_REGS[GPIO2PORT(pin) as usize];
    let g = GpioOut { regs, bit: GPIO2BIT(pin) };
    gpio_out_reset(g, val)?;
    Ok(g)
}

pub fn gpio_out_reset(g: GpioOut, val: GpioOutVal) -> R {
    let regs = unsafe { &*g.regs };
    let pin = regs_to_pin(g.regs, g.bit);
    let flag = irq_save();
    regs.FIOPIN.write((regs.FIOSET.read() & !g.bit) | if val != 0 { g.bit } else { 0 });
    regs.FIODIR.or(g.bit);
    gpio_peripheral(pin, 0, 0);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_out_toggle_noirq(g: GpioOut) {
    let regs = unsafe { &*g.regs };
    regs.FIOPIN.write(regs.FIOSET.read() ^ g.bit);
}

pub fn gpio_out_toggle(g: GpioOut) {
    let flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

pub fn gpio_out_write(g: GpioOut, val: u8) {
    let regs = unsafe { &*g.regs };
    if val != 0 {
        regs.FIOSET.write(g.bit);
    } else {
        regs.FIOCLR.write(g.bit);
    }
}

pub fn gpio_in_setup(pin: u32, pull_up: GpioPullUp) -> R<GpioIn> {
    if GPIO2PORT(pin) as usize >= DIGITAL_REGS.len() {
        shutdown!("Not an input pin");
    }
    let regs = DIGITAL_REGS[GPIO2PORT(pin) as usize];
    let g = GpioIn { regs, bit: GPIO2BIT(pin) };
    gpio_in_reset(g, pull_up)?;
    Ok(g)
}

pub fn gpio_in_reset(g: GpioIn, pull_up: GpioPullUp) -> R {
    let regs = unsafe { &*g.regs };
    let pin = regs_to_pin(g.regs, g.bit);
    let flag = irq_save();
    gpio_peripheral(pin, 0, pull_up as i32);
    regs.FIODIR.and(!g.bit);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_in_read(g: GpioIn) -> u8 {
    (unsafe { &*g.regs }.FIOPIN.read() & g.bit != 0) as u8
}
