// rp2040 serial
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::config::CONFIG_SERIAL_BAUD;
use crate::generic::serial_irq::{serial_get_tx_byte, serial_rx_byte};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_init};

// Dynamically select UART and IRQ based on configuration

#[cfg(CONFIG_RPXXXX_SERIAL_UART0_PINS_0_1)]
mod port {
    use super::*;
    pub const GPIO_Rx: u32 = 1;
    pub const GPIO_Tx: u32 = 0;
    pub const UARTx: *mut uart_hw_t = uart0_hw;
    pub const UARTx_IRQn: IRQn_Type = UART0_IRQ_IRQn;
}
#[cfg(CONFIG_RPXXXX_SERIAL_UART0_PINS_12_13)]
mod port {
    use super::*;
    pub const GPIO_Rx: u32 = 13;
    pub const GPIO_Tx: u32 = 12;
    pub const UARTx: *mut uart_hw_t = uart0_hw;
    pub const UARTx_IRQn: IRQn_Type = UART0_IRQ_IRQn;
}
#[cfg(CONFIG_RPXXXX_SERIAL_UART0_PINS_16_17)]
mod port {
    use super::*;
    pub const GPIO_Rx: u32 = 17;
    pub const GPIO_Tx: u32 = 16;
    pub const UARTx: *mut uart_hw_t = uart0_hw;
    pub const UARTx_IRQn: IRQn_Type = UART0_IRQ_IRQn;
}
// (C defines UARTx as uart1_hw, then again as uart0_hw: the last one counts)
#[cfg(CONFIG_RPXXXX_SERIAL_UART0_PINS_28_29)]
mod port {
    use super::*;
    pub const GPIO_Rx: u32 = 29;
    pub const GPIO_Tx: u32 = 28;
    pub const UARTx: *mut uart_hw_t = uart0_hw;
    pub const UARTx_IRQn: IRQn_Type = UART0_IRQ_IRQn;
}
#[cfg(CONFIG_RPXXXX_SERIAL_UART1_PINS_4_5)]
mod port {
    use super::*;
    pub const GPIO_Rx: u32 = 5;
    pub const GPIO_Tx: u32 = 4;
    pub const UARTx: *mut uart_hw_t = uart1_hw;
    pub const UARTx_IRQn: IRQn_Type = UART1_IRQ_IRQn;
}
#[cfg(CONFIG_RPXXXX_SERIAL_UART1_PINS_8_9)]
mod port {
    use super::*;
    pub const GPIO_Rx: u32 = 9;
    pub const GPIO_Tx: u32 = 8;
    pub const UARTx: *mut uart_hw_t = uart1_hw;
    pub const UARTx_IRQn: IRQn_Type = UART1_IRQ_IRQn;
}
#[cfg(CONFIG_RPXXXX_SERIAL_UART1_PINS_20_21)]
mod port {
    use super::*;
    pub const GPIO_Rx: u32 = 20;
    pub const GPIO_Tx: u32 = 21;
    pub const UARTx: *mut uart_hw_t = uart1_hw;
    pub const UARTx_IRQn: IRQn_Type = UART1_IRQ_IRQn;
}
#[cfg(CONFIG_RPXXXX_SERIAL_UART1_PINS_24_25)]
mod port {
    use super::*;
    pub const GPIO_Rx: u32 = 24;
    pub const GPIO_Tx: u32 = 25;
    pub const UARTx: *mut uart_hw_t = uart1_hw;
    pub const UARTx_IRQn: IRQn_Type = UART1_IRQ_IRQn;
}
use port::*;

fn uart() -> &'static uart_hw_t {
    unsafe { &*UARTx }
}

// Write tx bytes to the serial port
fn kick_tx() {
    loop {
        if uart().fr.read() & UART_UARTFR_TXFF_BITS != 0 {
            // Output fifo full - enable tx irq
            uart().imsc.write(UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS | UART_UARTIMSC_TXIM_BITS);
            break;
        }
        let mut data = 0;
        let ret = serial_get_tx_byte(&mut data);
        if ret != 0 {
            // No more data to send - disable tx irq
            uart().imsc.write(UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
            break;
        }
        uart().dr.write(data as u32);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn UARTx_IRQHandler() {
    let mis = uart().mis.read();
    if mis & (UART_UARTMIS_RXMIS_BITS | UART_UARTMIS_RTMIS_BITS) != 0 {
        loop {
            serial_rx_byte(uart().dr.read() as u8);
            if uart().fr.read() & UART_UARTFR_RXFE_BITS != 0 {
                break;
            }
        }
    } else if mis & UART_UARTMIS_TXMIS_BITS != 0 {
        kick_tx();
    }
}

pub fn serial_enable_tx_irq() {
    if uart().fr.read() & UART_UARTFR_TXFF_BITS == 0 {
        let flag = irq_save();
        kick_tx();
        irq_restore(flag);
    }
}

pub fn serial_init() -> R {
    let pclk = if UARTx == uart0_hw {
        enable_pclock(RESETS_RESET_UART0_BITS);
        get_pclock_frequency(RESETS_RESET_UART0_BITS)
    } else {
        enable_pclock(RESETS_RESET_UART1_BITS);
        get_pclock_frequency(RESETS_RESET_UART1_BITS)
    };

    // Setup baud
    // DIV_ROUND_CLOSEST(pclk * 4, CONFIG_SERIAL_BAUD)
    let div = (pclk * 4 + CONFIG_SERIAL_BAUD / 2) / CONFIG_SERIAL_BAUD;
    uart().ibrd.write(div >> 6);
    uart().fbrd.write(div & 0x3f);

    // Enable fifo, set 8N1
    uart().lcr_h.write(UART_UARTLCR_H_FEN_BITS | UART_UARTLCR_H_WLEN_BITS);
    uart().ifls.write(0);
    uart().cr.write(UART_UARTCR_RXE_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_UARTEN_BITS);

    // Setup pins
    gpio_peripheral(GPIO_Rx, 2, 1)?;
    gpio_peripheral(GPIO_Tx, 2, 0)?;

    // Enable receive irq
    armcm_enable_irq!(UARTx_IRQHandler, UARTx_IRQn, 0);
    uart().imsc.write(UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
    Ok(())
}
decl_init!(serial_init);
