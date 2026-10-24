// GPIO functions on HC32F460
//
// Copyright (C) 2022  Steven Gotthardt <gotthardt@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::gpio_h::*;
use super::internal::*;
use super::lib::hc32f460_gpio::{stc_port_init_t, PORT_Init, PORT_SetFunc};
use crate::board::{irq_restore, irq_save};
use crate::decl_enumeration_range;
use crate::sched::R;
use core::mem::offset_of;

// 64pin package
decl_enumeration_range!("pin", "PA0", GPIO('A', 0), 16);
decl_enumeration_range!("pin", "PB0", GPIO('B', 0), 16);
decl_enumeration_range!("pin", "PC0", GPIO('C', 0), 16);
decl_enumeration_range!("pin", "PD0", GPIO('D', 0), 16);
decl_enumeration_range!("pin", "PE0", GPIO('E', 0), 16);
decl_enumeration_range!("pin", "PH2", PortH * 16 + 2, 1); // H: special case

// HC32F460 ports are in one M4_PORT - offset by 0x10
// eg toggle: M4_PORT->POTRA + 0x10 => M4_PORT->POTRB
// 'gpio' is port (0-4) * 16 + pinPosition (0-15)
const POTR_OFFSET: u32 = offset_of!(M4_PORT_TypeDef, POTRA) as u32; // output flip
#[allow(dead_code)]
const PODR_OFFSET: u32 = offset_of!(M4_PORT_TypeDef, PODRA) as u32; // output data
const PIDR_OFFSET: u32 = offset_of!(M4_PORT_TypeDef, PIDRA) as u32; // input data
const POSR_OFFSET: u32 = offset_of!(M4_PORT_TypeDef, POSRA) as u32; // output set
const PORR_OFFSET: u32 = offset_of!(M4_PORT_TypeDef, PORRA) as u32; // output reset
const PORT_OFFSET: u32 = offset_of!(M4_PORT_TypeDef, PIDRB) as u32; // space between PORTS

// the 16 bit port register at an address
fn reg16(addr: u32) -> &'static Reg<u16> {
    unsafe { &*(addr as *const Reg<u16>) }
}

pub fn gpio_peripheral(gpio: u32, func: u32, pull_up: i32) {
    let flag = irq_save();

    let stcPortInit = stc_port_init_t {
        enPinMode: func,
        enLatch: Disable,
        enExInt: Disable,
        enInvert: Disable,
        enPullUp: if pull_up != 0 { Enable } else { Disable },
        enPinDrv: Pin_Drv_L,
        enPinOType: Pin_OType_Cmos,
        enPinSubFunc: Disable,
    };

    // make the port GPIO and disable the sub functionality
    PORT_SetFunc(GPIO2PORT(gpio), GPIO2BIT(gpio) as u16, Func_Gpio, Disable);
    PORT_Init(GPIO2PORT(gpio), GPIO2BIT(gpio) as u16, &stcPortInit);

    irq_restore(flag);
}

pub fn gpio_out_setup(gpio: u32, val: GpioOutVal) -> R<GpioOut> {
    let port = M4_PORT as u32 + GPIO2PORT(gpio) * PORT_OFFSET;
    let g = GpioOut { gpio: gpio as u16, portAddress: port, bitMask: GPIO2BIT(gpio) as u16 };
    gpio_out_reset(g, val)?;

    Ok(g)
}

pub fn gpio_out_reset(g: GpioOut, val: GpioOutVal) -> R {
    let flag = irq_save();
    if val != 0 {
        reg16(g.portAddress + POSR_OFFSET).write(g.bitMask);
    } else {
        reg16(g.portAddress + PORR_OFFSET).write(g.bitMask);
    }

    gpio_peripheral(g.gpio as u32, Pin_Mode_Out, 0);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_out_toggle_noirq(g: GpioOut) {
    reg16(g.portAddress + POTR_OFFSET).write(g.bitMask);
}

pub fn gpio_out_toggle(g: GpioOut) {
    let flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

pub fn gpio_out_write(g: GpioOut, val: u8) {
    if val != 0 {
        reg16(g.portAddress + POSR_OFFSET).write(g.bitMask);
    } else {
        reg16(g.portAddress + PORR_OFFSET).write(g.bitMask);
    }
}

pub fn gpio_in_setup(gpio: u32, pull_up: GpioPullUp) -> R<GpioIn> {
    let port = M4_PORT as u32 + GPIO2PORT(gpio) * PORT_OFFSET;

    let g = GpioIn { gpio: gpio as u16, portAddress: port, bitMask: GPIO2BIT(gpio) as u16 };
    gpio_in_reset(g, pull_up)?;

    Ok(g)
}

pub fn gpio_in_reset(g: GpioIn, pull_up: GpioPullUp) -> R {
    let flag = irq_save();
    gpio_peripheral(g.gpio as u32, Pin_Mode_In, pull_up);
    irq_restore(flag);
    Ok(())
}

pub fn gpio_in_read(g: GpioIn) -> u8 {
    (reg16(g.portAddress + PIDR_OFFSET).read() & g.bitMask != 0) as u8
}
