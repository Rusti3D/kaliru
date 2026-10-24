// AVR serial port code.
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::config::{CONFIG_CLOCK_FREQ, CONFIG_SERIAL_BAUD, CONFIG_SERIAL_BAUD_U2X};
use crate::generic::serial_irq::{serial_get_tx_byte, serial_rx_byte};
use crate::sched::R;
use crate::{decl_constant_str, decl_init};

// Reserve serial pins
#[cfg(all(CONFIG_SERIAL_PORT = "0", any(CONFIG_MACH_atmega1280, CONFIG_MACH_atmega2560)))]
decl_constant_str!("RESERVE_PINS_serial", "PE0,PE1");
#[cfg(all(CONFIG_SERIAL_PORT = "0", not(any(CONFIG_MACH_atmega1280, CONFIG_MACH_atmega2560))))]
decl_constant_str!("RESERVE_PINS_serial", "PD0,PD1");
#[cfg(CONFIG_SERIAL_PORT = "1")]
decl_constant_str!("RESERVE_PINS_serial", "PD2,PD3");
#[cfg(CONFIG_SERIAL_PORT = "2")]
decl_constant_str!("RESERVE_PINS_serial", "PH0,PH1");
#[cfg(CONFIG_SERIAL_PORT = "3")]
decl_constant_str!("RESERVE_PINS_serial", "PJ0,PJ1");

// Serial port register aliases (UCSRxA, ... of port CONFIG_SERIAL_PORT)
#[cfg(CONFIG_SERIAL_PORT = "0")]
use super::chip::{
    RXCIE0 as RXCIEx, RXEN0 as RXENx, TXEN0 as TXENx, U2X0 as U2Xx, UBRR0 as UBRRx, UCSR0A as UCSRxA, UCSR0B as UCSRxB, UCSR0C as UCSRxC,
    UCSZ00 as UCSZx0, UCSZ01 as UCSZx1, UDR0 as UDRx, UDRIE0 as UDRIEx,
};
#[cfg(CONFIG_SERIAL_PORT = "1")]
use super::chip::{
    RXCIE1 as RXCIEx, RXEN1 as RXENx, TXEN1 as TXENx, U2X1 as U2Xx, UBRR1 as UBRRx, UCSR1A as UCSRxA, UCSR1B as UCSRxB, UCSR1C as UCSRxC,
    UCSZ10 as UCSZx0, UCSZ11 as UCSZx1, UDR1 as UDRx, UDRIE1 as UDRIEx,
};
#[cfg(CONFIG_SERIAL_PORT = "2")]
use super::chip::{
    RXCIE2 as RXCIEx, RXEN2 as RXENx, TXEN2 as TXENx, U2X2 as U2Xx, UBRR2 as UBRRx, UCSR2A as UCSRxA, UCSR2B as UCSRxB, UCSR2C as UCSRxC,
    UCSZ20 as UCSZx0, UCSZ21 as UCSZx1, UDR2 as UDRx, UDRIE2 as UDRIEx,
};
#[cfg(CONFIG_SERIAL_PORT = "3")]
use super::chip::{
    RXCIE3 as RXCIEx, RXEN3 as RXENx, TXEN3 as TXENx, U2X3 as U2Xx, UBRR3 as UBRRx, UCSR3A as UCSRxA, UCSR3B as UCSRxB, UCSR3C as UCSRxC,
    UCSZ30 as UCSZx0, UCSZ31 as UCSZx1, UDR3 as UDRx, UDRIE3 as UDRIEx,
};

pub fn serial_init() -> R {
    unsafe {
        (*UCSRxA).write(if CONFIG_SERIAL_BAUD_U2X { 1 << U2Xx } else { 0 });
        let cm: u32 = if CONFIG_SERIAL_BAUD_U2X { 8 } else { 16 };
        // DIV_ROUND_CLOSEST(CONFIG_CLOCK_FREQ, cm * CONFIG_SERIAL_BAUD) - 1UL
        let d = cm * CONFIG_SERIAL_BAUD;
        (*UBRRx).write(((CONFIG_CLOCK_FREQ + d / 2) / d - 1) as u16);
        (*UCSRxC).write((1 << UCSZx1) | (1 << UCSZx0));
        (*UCSRxB).write((1 << RXENx) | (1 << TXENx) | (1 << RXCIEx) | (1 << UDRIEx));
    }
    Ok(())
}
decl_init!(serial_init);

// Rx interrupt - data available to be read.
// (the atmega168 / atmega328 don't have an id in the irq names)
#[cfg_attr(has_USART_RX_vect_num, export_name = avr_vector!(USART_RX_vect))]
#[cfg_attr(all(not(has_USART_RX_vect_num), CONFIG_SERIAL_PORT = "0"), export_name = avr_vector!(USART0_RX_vect))]
#[cfg_attr(all(not(has_USART_RX_vect_num), CONFIG_SERIAL_PORT = "1"), export_name = avr_vector!(USART1_RX_vect))]
#[cfg_attr(all(not(has_USART_RX_vect_num), CONFIG_SERIAL_PORT = "2"), export_name = avr_vector!(USART2_RX_vect))]
#[cfg_attr(all(not(has_USART_RX_vect_num), CONFIG_SERIAL_PORT = "3"), export_name = avr_vector!(USART3_RX_vect))]
pub unsafe extern "avr-interrupt" fn usart_rx_isr() {
    serial_rx_byte((*UDRx).read());
}

// Tx interrupt - data can be written to serial.
#[cfg_attr(has_USART_RX_vect_num, export_name = avr_vector!(USART_UDRE_vect))]
#[cfg_attr(all(not(has_USART_RX_vect_num), CONFIG_SERIAL_PORT = "0"), export_name = avr_vector!(USART0_UDRE_vect))]
#[cfg_attr(all(not(has_USART_RX_vect_num), CONFIG_SERIAL_PORT = "1"), export_name = avr_vector!(USART1_UDRE_vect))]
#[cfg_attr(all(not(has_USART_RX_vect_num), CONFIG_SERIAL_PORT = "2"), export_name = avr_vector!(USART2_UDRE_vect))]
#[cfg_attr(all(not(has_USART_RX_vect_num), CONFIG_SERIAL_PORT = "3"), export_name = avr_vector!(USART3_UDRE_vect))]
pub unsafe extern "avr-interrupt" fn usart_udre_isr() {
    let mut data = 0;
    let ret = serial_get_tx_byte(&mut data);
    if ret != 0 {
        (*UCSRxB).write((*UCSRxB).read() & !(1 << UDRIEx));
    } else {
        (*UDRx).write(data);
    }
}

// Enable tx interrupts
pub fn serial_enable_tx_irq() {
    unsafe { (*UCSRxB).write((*UCSRxB).read() | (1 << UDRIEx)) };
}
