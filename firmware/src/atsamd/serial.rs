// samd21 serial port
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::config::CONFIG_SERIAL_BAUD;
use crate::generic::serial_irq::{serial_get_tx_byte, serial_rx_byte};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};

fn usart() -> &'static SercomUsart {
    unsafe { &*SERCOM0 }.USART()
}

pub fn serial_enable_tx_irq() {
    usart().INTENSET.reg.write(SERCOM_USART_INTENSET_DRE as _);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn SERCOM0_Handler() {
    let status = usart().INTFLAG.reg.read() as u32;
    if status & SERCOM_USART_INTFLAG_RXC != 0 {
        serial_rx_byte(usart().DATA.reg.read() as u8);
    }
    if status & SERCOM_USART_INTFLAG_DRE != 0 {
        let mut data = 0;
        let ret = serial_get_tx_byte(&mut data);
        if ret != 0 {
            usart().INTENCLR.reg.write(SERCOM_USART_INTENSET_DRE as _);
        } else {
            usart().DATA.reg.write(data as _);
        }
    }
}

decl_constant_str!("RESERVE_PINS_serial", "PA11,PA10");

pub fn serial_init() -> R {
    // Enable serial clock
    enable_pclock(SERCOM0_GCLK_ID_CORE, ID_SERCOM0 as i32);
    // Enable pins
    gpio_peripheral(GPIO('A', 11), 'C', 0);
    gpio_peripheral(GPIO('A', 10), 'C', 0);
    // Configure serial
    let su = usart();
    su.CTRLA.reg.write(0);
    let areg = SERCOM_USART_CTRLA_MODE(1)
        | SERCOM_USART_CTRLA_DORD
        | SERCOM_USART_CTRLA_SAMPR(1)
        | SERCOM_USART_CTRLA_RXPO(3)
        | SERCOM_USART_CTRLA_TXPO(1);
    su.CTRLA.reg.write(areg);
    su.CTRLB.reg.write(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);
    let freq = get_pclock_frequency(SERCOM0_GCLK_ID_CORE);
    let baud8 = freq / (2 * CONFIG_SERIAL_BAUD);
    // (the BAUD register's FRAC view)
    su.BAUD.reg.write((SERCOM_USART_BAUD_FRAC_BAUD(baud8 / 8) | SERCOM_USART_BAUD_FRAC_FP(baud8 % 8)) as _);
    // enable irqs
    su.INTENSET.reg.write(SERCOM_USART_INTENSET_RXC as _);
    su.CTRLA.reg.write(areg | SERCOM_USART_CTRLA_ENABLE);
    #[cfg(CONFIG_MACH_SAMD21)]
    armcm_enable_irq!(SERCOM0_Handler, SERCOM0_IRQn, 0);
    #[cfg(CONFIG_MACH_SAMX5)]
    {
        armcm_enable_irq!(SERCOM0_Handler, SERCOM0_0_IRQn, 0);
        armcm_enable_irq!(SERCOM0_Handler, SERCOM0_1_IRQn, 0);
        armcm_enable_irq!(SERCOM0_Handler, SERCOM0_2_IRQn, 0);
        armcm_enable_irq!(SERCOM0_Handler, SERCOM0_3_IRQn, 0);
    }
    Ok(())
}
decl_init!(serial_init);
