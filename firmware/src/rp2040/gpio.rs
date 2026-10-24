// GPIO functions on rp2040
//
// Copyright (C) 2021-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::{GpioIn, GpioOut, GpioOutVal, GpioPullUp};
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::config::CONFIG_MACH_RP2350;
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown};

/****************************************************************
 * Pin mappings
 ****************************************************************/

#[cfg(CONFIG_MACH_RP2040)]
const NUM_GPIO: u32 = 30;
#[cfg(not(CONFIG_MACH_RP2040))]
const NUM_GPIO: u32 = 48;

decl_enumeration_range!("pin", "gpio0", 0, NUM_GPIO);

// Set the mode and extended function of a pin
pub fn gpio_peripheral(gpio: u32, func: u32, pull_up: i32) -> R {
    if gpio >= NUM_GPIO {
        shutdown!("Not a valid pin");
    }
    unsafe { &*padsbank0_hw }.io[gpio as usize].write(
        PADS_BANK0_GPIO0_IE_BITS
            | (PADS_BANK0_GPIO0_DRIVE_VALUE_4MA << PADS_BANK0_GPIO0_DRIVE_MSB)
            | if pull_up > 0 { PADS_BANK0_GPIO0_PUE_BITS } else { 0 }
            | if pull_up < 0 { PADS_BANK0_GPIO0_PDE_BITS } else { 0 },
    );
    unsafe { &*iobank0_hw }.io[gpio as usize].ctrl.write(func << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB);
    Ok(())
}

// Convert a register and bit location back to an integer pin identifier
fn mask_to_pin(sio: *mut u8, mask: u32) -> u32 {
    let mut pin = mask.trailing_zeros();
    if CONFIG_MACH_RP2350 && sio != sio_hw as *mut u8 {
        pin += 32;
    }
    pin
}

fn sio(p: *mut u8) -> &'static sio_hw_t {
    unsafe { &*(p as *const sio_hw_t) }
}

/****************************************************************
 * General Purpose Input Output (GPIO) pins
 ****************************************************************/

pub fn gpio_out_setup(pin: u32, val: GpioOutVal) -> R<GpioOut> {
    if pin >= NUM_GPIO {
        shutdown!("Not a valid pin");
    }
    let (mut pin, mut sio) = (pin as u8, sio_hw as *mut u8);
    if CONFIG_MACH_RP2350 && pin >= 32 {
        pin -= 32;
        sio = sio.wrapping_add(4);
    }
    let g = GpioOut { sio, bit: 1 << pin };
    gpio_out_reset(g, val)?;
    Ok(g)
}

pub fn gpio_out_reset(g: GpioOut, val: GpioOutVal) -> R {
    let pin = mask_to_pin(g.sio, g.bit);
    let flag = irq_save();
    gpio_out_write(g, val);
    sio(g.sio).gpio_oe_set.write(g.bit);
    gpio_peripheral(pin, 5, 0)?;
    irq_restore(flag);
    Ok(())
}

pub fn gpio_out_toggle_noirq(g: GpioOut) {
    sio(g.sio).gpio_togl.write(g.bit);
}

pub fn gpio_out_toggle(g: GpioOut) {
    gpio_out_toggle_noirq(g);
}

pub fn gpio_out_write(g: GpioOut, val: u8) {
    if val != 0 {
        sio(g.sio).gpio_set.write(g.bit);
    } else {
        sio(g.sio).gpio_clr.write(g.bit);
    }
}

pub fn gpio_in_setup(pin: u32, pull_up: GpioPullUp) -> R<GpioIn> {
    if pin >= NUM_GPIO {
        shutdown!("Not a valid pin");
    }
    let (mut pin, mut sio) = (pin as u8, sio_hw as *mut u8);
    if CONFIG_MACH_RP2350 && pin >= 32 {
        pin -= 32;
        sio = sio.wrapping_add(4);
    }
    let g = GpioIn { sio, bit: 1 << pin };
    gpio_in_reset(g, pull_up)?;
    Ok(g)
}

pub fn gpio_in_reset(g: GpioIn, pull_up: GpioPullUp) -> R {
    let pin = mask_to_pin(g.sio, g.bit);
    let flag = irq_save();
    gpio_peripheral(pin, 5, pull_up as i32)?;
    sio(g.sio).gpio_oe_clr.write(g.bit);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_in_read(g: GpioIn) -> u8 {
    (sio(g.sio).gpio_in.read() & g.bit != 0) as u8
}
