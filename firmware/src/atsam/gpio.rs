// GPIO functions on sam3/sam4
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::{GpioIn, GpioOut, GpioOutVal, GpioPullUp};
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::config::CONFIG_MACH_SAM3X;
use crate::ctr::{concat, concat_len};
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown};

#[derive(Clone, Copy)]
struct GpioInfo {
    dev: *mut Pio,
    dev_id: u8,
}
unsafe impl Sync for GpioInfo {}

decl_enumeration_range!("pin", "PA0", GPIO('A', 0), 32);
decl_enumeration_range!("pin", "PB0", GPIO('B', 0), 32);
#[cfg(has_PIOC)]
decl_enumeration_range!("pin", "PC0", GPIO('C', 0), 32);
#[cfg(has_PIOD)]
decl_enumeration_range!("pin", "PD0", GPIO('D', 0), 32);
#[cfg(has_PIOE)]
decl_enumeration_range!("pin", "PE0", GPIO('E', 0), 32);

const DIGITAL_REGS_AB: &[GpioInfo] = &[GpioInfo { dev: PIOA, dev_id: ID_PIOA as u8 }, GpioInfo { dev: PIOB, dev_id: ID_PIOB as u8 }];
#[cfg(has_PIOC)]
const DIGITAL_REGS_C: &[GpioInfo] = &[GpioInfo { dev: PIOC, dev_id: ID_PIOC as u8 }];
#[cfg(not(has_PIOC))]
const DIGITAL_REGS_C: &[GpioInfo] = &[];
#[cfg(has_PIOD)]
const DIGITAL_REGS_D: &[GpioInfo] = &[GpioInfo { dev: PIOD, dev_id: ID_PIOD as u8 }];
#[cfg(not(has_PIOD))]
const DIGITAL_REGS_D: &[GpioInfo] = &[];
#[cfg(has_PIOE)]
const DIGITAL_REGS_E: &[GpioInfo] = &[GpioInfo { dev: PIOE, dev_id: ID_PIOE as u8 }];
#[cfg(not(has_PIOE))]
const DIGITAL_REGS_E: &[GpioInfo] = &[];
const DIGITAL_REGS_PARTS: &[&[GpioInfo]] = &[DIGITAL_REGS_AB, DIGITAL_REGS_C, DIGITAL_REGS_D, DIGITAL_REGS_E];
static DIGITAL_REGS: [GpioInfo; concat_len(DIGITAL_REGS_PARTS)] = concat(DIGITAL_REGS_PARTS);

/****************************************************************
 * Pin multiplexing
 ****************************************************************/

fn set_pull_up(regs: &Pio, bit: u32, pull_up: i32) {
    #[cfg(CONFIG_MACH_SAM3X)]
    {
        if pull_up > 0 {
            regs.PIO_PUER.write(bit);
        } else {
            regs.PIO_PUDR.write(bit);
        }
    }
    #[cfg(not(CONFIG_MACH_SAM3X))]
    {
        if pull_up > 0 {
            regs.PIO_PPDDR.write(bit);
            regs.PIO_PUER.write(bit);
        } else if pull_up < 0 {
            regs.PIO_PUDR.write(bit);
            regs.PIO_PPDER.write(bit);
        } else {
            regs.PIO_PUDR.write(bit);
            regs.PIO_PPDDR.write(bit);
        }
    }
    // Check if this pin is a "system IO pin" and disable if so
    #[cfg(CONFIG_MACH_SAM4S)]
    if regs as *const Pio == PIOB as *const Pio && (bit & 0x1cf0) != 0 {
        unsafe { &*MATRIX }.CCFG_SYSIO.or(bit);
    }
    #[cfg(CONFIG_MACH_SAME70)]
    if regs as *const Pio == PIOB as *const Pio && (bit & 0x10f0) != 0 {
        unsafe { &*MATRIX }.CCFG_SYSIO.or(bit);
    }
}

pub fn gpio_peripheral(gpio: u32, ptype: char, pull_up: i32) {
    let (bank, bit, pt) = (GPIO2PORT(gpio), GPIO2BIT(gpio), (ptype as u32).wrapping_sub('A' as u32));
    let regs = unsafe { &*DIGITAL_REGS[bank as usize].dev };

    #[cfg(CONFIG_MACH_SAM3X)]
    regs.PIO_ABSR.write((regs.PIO_ABSR.read() & !bit) | if pt & 0x01 != 0 { bit } else { 0 });
    #[cfg(not(CONFIG_MACH_SAM3X))]
    {
        regs.PIO_ABCDSR[0].write((regs.PIO_ABCDSR[0].read() & !bit) | if pt & 0x01 != 0 { bit } else { 0 });
        regs.PIO_ABCDSR[1].write((regs.PIO_ABCDSR[1].read() & !bit) | if pt & 0x02 != 0 { bit } else { 0 });
    }
    set_pull_up(regs, bit, pull_up);
    regs.PIO_PDR.write(bit);
}

/****************************************************************
 * General Purpose Input Output (GPIO) pins
 ****************************************************************/

pub fn gpio_out_setup(pin: u32, val: GpioOutVal) -> R<GpioOut> {
    if GPIO2PORT(pin) as usize >= DIGITAL_REGS.len() {
        shutdown!("Not an output pin");
    }
    let regs = DIGITAL_REGS[GPIO2PORT(pin) as usize].dev;
    let g = GpioOut { regs, bit: GPIO2BIT(pin) };
    gpio_out_reset(g, val)?;
    Ok(g)
}

pub fn gpio_out_reset(g: GpioOut, val: GpioOutVal) -> R {
    let regs = unsafe { &*g.regs };
    let flag = irq_save();
    if val != 0 {
        regs.PIO_SODR.write(g.bit);
    } else {
        regs.PIO_CODR.write(g.bit);
    }
    regs.PIO_OER.write(g.bit);
    regs.PIO_OWER.write(g.bit);
    regs.PIO_PER.write(g.bit);
    set_pull_up(regs, g.bit, 0);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_out_toggle_noirq(g: GpioOut) {
    unsafe { &*g.regs }.PIO_ODSR.xor(g.bit);
}

pub fn gpio_out_toggle(g: GpioOut) {
    let flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

pub fn gpio_out_write(g: GpioOut, val: u8) {
    let regs = unsafe { &*g.regs };
    if val != 0 {
        regs.PIO_SODR.write(g.bit);
    } else {
        regs.PIO_CODR.write(g.bit);
    }
}

pub fn gpio_in_setup(pin: u32, pull_up: GpioPullUp) -> R<GpioIn> {
    if GPIO2PORT(pin) as usize >= DIGITAL_REGS.len() || (CONFIG_MACH_SAM3X && pull_up < 0) {
        shutdown!("Not a valid input pin");
    }
    let port = GPIO2PORT(pin) as usize;
    enable_pclock(DIGITAL_REGS[port].dev_id as u32);
    let g = GpioIn { regs: DIGITAL_REGS[port].dev, bit: GPIO2BIT(pin) };
    gpio_in_reset(g, pull_up)?;
    Ok(g)
}

pub fn gpio_in_reset(g: GpioIn, pull_up: GpioPullUp) -> R {
    let regs = unsafe { &*g.regs };
    let flag = irq_save();
    set_pull_up(regs, g.bit, pull_up as i32);
    regs.PIO_ODR.write(g.bit);
    regs.PIO_PER.write(g.bit);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_in_read(g: GpioIn) -> u8 {
    (unsafe { &*g.regs }.PIO_PDSR.read() & g.bit != 0) as u8
}
