// GPIO functions optimized for stm32h7
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::*;
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::sched::R;
use crate::{decl_enumeration_range, shutdown};

decl_enumeration_range!("pin", "PA0", GPIO('A', 0), 16);
decl_enumeration_range!("pin", "PB0", GPIO('B', 0), 16);
decl_enumeration_range!("pin", "PC0", GPIO('C', 0), 16);
#[cfg(has_GPIOD)]
decl_enumeration_range!("pin", "PD0", GPIO('D', 0), 16);
#[cfg(has_GPIOE)]
decl_enumeration_range!("pin", "PE0", GPIO('E', 0), 16);
#[cfg(has_GPIOF)]
decl_enumeration_range!("pin", "PF0", GPIO('F', 0), 16);
#[cfg(has_GPIOG)]
decl_enumeration_range!("pin", "PG0", GPIO('G', 0), 16);
#[cfg(has_GPIOH)]
decl_enumeration_range!("pin", "PH0", GPIO('H', 0), 16);
#[cfg(has_GPIOI)]
decl_enumeration_range!("pin", "PI0", GPIO('I', 0), 16);

struct Regs<const N: usize>([*mut GPIO_TypeDef; N]);
unsafe impl<const N: usize> Sync for Regs<N> {}

// ['A' - 'A'] = GPIOA, ... up to the last port the chip has
const fn port_regs() -> [*mut GPIO_TypeDef; 9] {
    let mut r = [core::ptr::null_mut(); 9];
    r[0] = GPIOA;
    r[1] = GPIOB;
    r[2] = GPIOC;
    #[cfg(has_GPIOD)]
    {
        r[3] = GPIOD;
    }
    #[cfg(has_GPIOE)]
    {
        r[4] = GPIOE;
    }
    #[cfg(has_GPIOF)]
    {
        r[5] = GPIOF;
    }
    #[cfg(has_GPIOG)]
    {
        r[6] = GPIOG;
    }
    #[cfg(has_GPIOH)]
    {
        r[7] = GPIOH;
    }
    #[cfg(has_GPIOI)]
    {
        r[8] = GPIOI;
    }
    r
}
const fn port_count() -> usize {
    let r = port_regs();
    let mut n = r.len();
    while n > 0 && r[n - 1].is_null() {
        n -= 1;
    }
    n
}
const DIGITAL_REGS_LEN: usize = port_count();
static DIGITAL_REGS: Regs<DIGITAL_REGS_LEN> = Regs({
    let all = port_regs();
    let mut r = [core::ptr::null_mut(); DIGITAL_REGS_LEN];
    let mut i = 0;
    while i < DIGITAL_REGS_LEN {
        r[i] = all[i];
        i += 1;
    }
    r
});

pub fn digital_regs() -> &'static [*mut GPIO_TypeDef] {
    &DIGITAL_REGS.0
}

// Convert a register and bit location back to an integer pin identifier
fn regs_to_pin(regs: *mut GPIO_TypeDef, bit: u32) -> u32 {
    for (i, r) in digital_regs().iter().enumerate() {
        if *r == regs {
            return GPIO((b'A' + i as u8) as char, bit.trailing_zeros());
        }
    }
    0
}

// Verify that a gpio is a valid pin
fn gpio_valid(pin: u32) -> bool {
    let port = GPIO2PORT(pin) as usize;
    port < digital_regs().len() && !digital_regs()[port].is_null()
}

// The stm32h7 has very slow read access to the gpio registers.  Cache
// the ODR register in memory to speed up toggle operations.
#[repr(C, align(64))]
struct OdrCaches([OdrCache; DIGITAL_REGS_LEN * 16]);
static ODR_CACHE: crate::ctr::Global<OdrCaches> =
    crate::ctr::Global::new(OdrCaches([const { OdrCache { bsrr: 0 } }; DIGITAL_REGS_LEN * 16]));

fn odr_cache() -> *mut OdrCache {
    ODR_CACHE.ptr() as *mut OdrCache
}

pub fn gpio_out_setup(pin: u32, val: GpioOutVal) -> R<GpioOut> {
    if !gpio_valid(pin) {
        shutdown!("Not an output pin");
    }
    let port = GPIO2PORT(pin);
    let regs = digital_regs()[port as usize];
    gpio_clock_enable(regs);

    let g = GpioOut { regs, oc: odr_cache().wrapping_add(pin as usize) };
    gpio_out_reset(g, val)?;
    Ok(g)
}

pub fn gpio_out_reset(g: GpioOut, val: GpioOutVal) -> R {
    let regs = unsafe { &*g.regs };
    let pin = (g.oc as usize - odr_cache() as usize) / core::mem::size_of::<OdrCache>();
    let bit_num = (g.oc as u32 / core::mem::size_of::<OdrCache>() as u32) % 16;
    let bsrr = 1 << if val != 0 { bit_num } else { bit_num + 16 };
    let flag = irq_save();
    unsafe { (*g.oc).bsrr = bsrr };
    regs.BSRR.write(bsrr);
    gpio_peripheral(pin as u32, GPIO_OUTPUT, 0);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_out_toggle_noirq(g: GpioOut) {
    let regs = unsafe { &*g.regs };
    let oc = unsafe { &mut *g.oc };
    let bsrr = (oc.bsrr << 16) | (oc.bsrr >> 16);
    oc.bsrr = bsrr;
    regs.BSRR.write(bsrr);
}

pub fn gpio_out_toggle(g: GpioOut) {
    let flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

// (C: a uint32_t val; the callers all pass uint8_t values)
pub fn gpio_out_write(g: GpioOut, val: u8) {
    let regs = unsafe { &*g.regs };
    let bit_num = (g.oc as u32 / core::mem::size_of::<OdrCache>() as u32) % 16;
    let bsrr = 1 << if val != 0 { bit_num } else { bit_num + 16 };
    let flag = irq_save();
    unsafe { (*g.oc).bsrr = bsrr };
    regs.BSRR.write(bsrr);
    irq_restore(flag);
}

pub fn gpio_in_setup(pin: u32, pull_up: GpioPullUp) -> R<GpioIn> {
    if !gpio_valid(pin) {
        shutdown!("Not a valid input pin");
    }
    let regs = digital_regs()[GPIO2PORT(pin) as usize];
    let g = GpioIn { regs, bit: GPIO2BIT(pin) };
    gpio_in_reset(g, pull_up)?;
    Ok(g)
}

pub fn gpio_in_reset(g: GpioIn, pull_up: GpioPullUp) -> R {
    let pin = regs_to_pin(g.regs, g.bit);
    let flag = irq_save();
    gpio_peripheral(pin, GPIO_INPUT, pull_up);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_in_read(g: GpioIn) -> u8 {
    (unsafe { &*g.regs }.IDR.read() & g.bit != 0) as u8
}
