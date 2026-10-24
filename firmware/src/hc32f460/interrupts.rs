// Interrupt support for HC32F460
// The library interrupt support is huge and redefines systick
//
// Copyright (C) 2022  Steven Gotthardt <gotthardt@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::internal::*;
use crate::generic::cmsis::{NVIC_ClearPendingIRQ, NVIC_EnableIRQ, NVIC_SetPriority};

const IRQ_PRIORITY_DEFAULT: u32 = 15;

/* the interrupts on the hc32f460 can be 're-assigned'
    The author can choose the irqType (IRQ000_Handler, etc...)
     that the source (irqSrc) triggers
     */

pub fn IrqRegistration(irqSrc: u32, irqType: IRQn_Type) {
    let addr = unsafe { (*M4_INTC).SEL0.ptr() } as u32 + 4 * irqType as u32;
    let stcIntSel = unsafe { &*(addr as *const Reg<stc_intc_sel_field_t::T>) };

    // what is the source of the selected interrupt? (USART, etc...)
    stcIntSel.set_field(stc_intc_sel_field_t::INTSEL, irqSrc);

    // set priority and enable
    NVIC_SetPriority(irqType, IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(irqType);
    NVIC_EnableIRQ(irqType);
}
