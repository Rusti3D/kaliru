// sam3/sam4 serial port
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::config::CONFIG_SERIAL_BAUD;
use crate::generic::serial_irq::{serial_get_tx_byte, serial_rx_byte};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};

// Serial port pins
#[cfg(CONFIG_MACH_SAM3X)]
mod port {
    use super::*;
    pub const UARTx_IRQn: IRQn_Type = UART_IRQn;
    pub const Port: *mut Uart = UART;
    pub const Pmc_id: u32 = ID_UART;
    pub const rx_pin: u32 = GPIO('A', 8);
    pub const tx_pin: u32 = GPIO('A', 9);
    pub const uart_periph: char = 'A';
    decl_constant_str!("RESERVE_PINS_serial", "PA8,PA9");
}
#[cfg(CONFIG_MACH_SAM4S)]
mod port {
    use super::*;
    pub const UARTx_IRQn: IRQn_Type = UART1_IRQn;
    pub const Port: *mut Uart = UART1;
    pub const Pmc_id: u32 = ID_UART1;
    pub const rx_pin: u32 = GPIO('B', 2);
    pub const tx_pin: u32 = GPIO('B', 3);
    pub const uart_periph: char = 'A';
    decl_constant_str!("RESERVE_PINS_serial", "PB2,PB3");
}
#[cfg(CONFIG_MACH_SAM4E)]
mod port {
    use super::*;
    pub const UARTx_IRQn: IRQn_Type = UART0_IRQn;
    pub const Port: *mut Uart = UART0;
    pub const Pmc_id: u32 = ID_UART0;
    pub const rx_pin: u32 = GPIO('A', 9);
    pub const tx_pin: u32 = GPIO('A', 10);
    pub const uart_periph: char = 'A';
    decl_constant_str!("RESERVE_PINS_serial", "PA9,PA10");
}
#[cfg(CONFIG_MACH_SAME70)]
mod port {
    use super::*;
    pub const UARTx_IRQn: IRQn_Type = UART2_IRQn;
    pub const Port: *mut Uart = UART2;
    pub const Pmc_id: u32 = ID_UART2;
    pub const rx_pin: u32 = GPIO('D', 25);
    pub const tx_pin: u32 = GPIO('D', 26);
    pub const uart_periph: char = 'C';
    decl_constant_str!("RESERVE_PINS_serial", "PD25,PD26");
}
use port::*;

fn port() -> &'static Uart {
    unsafe { &*Port }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn UARTx_Handler() {
    let status = port().UART_SR.read();
    if status & UART_SR_RXRDY != 0 {
        serial_rx_byte(port().UART_RHR.read() as u8);
    }
    if status & UART_SR_TXRDY != 0 {
        let mut data = 0;
        let ret = serial_get_tx_byte(&mut data);
        if ret != 0 {
            port().UART_IDR.write(UART_IDR_TXRDY);
        } else {
            port().UART_THR.write(data as u32);
        }
    }
}

pub fn serial_enable_tx_irq() {
    port().UART_IER.write(UART_IDR_TXRDY);
}

pub fn serial_init() -> R {
    gpio_peripheral(rx_pin, uart_periph, 1);
    gpio_peripheral(tx_pin, uart_periph, 0);

    // Reset uart
    enable_pclock(Pmc_id);
    port().UART_CR.write(UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS);
    port().UART_IDR.write(0xFFFFFFFF);

    // Enable uart
    port().UART_MR.write(UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL);
    port().UART_BRGR.write(get_pclock_frequency(Pmc_id) / (16 * CONFIG_SERIAL_BAUD));
    port().UART_IER.write(UART_IER_RXRDY);
    armcm_enable_irq!(UARTx_Handler, UARTx_IRQn, 0);
    port().UART_CR.write(UART_CR_RXEN | UART_CR_TXEN);
    Ok(())
}
decl_init!(serial_init);
