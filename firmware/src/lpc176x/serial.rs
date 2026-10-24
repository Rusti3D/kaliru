// lpc176x serial port
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::config::CONFIG_SERIAL_BAUD;
use crate::generic::serial_irq::{serial_get_tx_byte, serial_rx_byte};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};

#[cfg(CONFIG_LPC_SERIAL_UART0_P03_P02)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "P0.3,P0.2");
    pub const GPIO_Rx: u32 = GPIO(0, 3);
    pub const GPIO_Tx: u32 = GPIO(0, 2);
    pub const GPIO_FUNCTION_UARTx: u32 = 1;
    pub type Uart = LPC_UART0_TypeDef;
    pub const LPC_UARTx: *mut Uart = LPC_UART0;
    pub const UARTx_IRQn: IRQn_Type = UART0_IRQn;
    pub const PCLK_UARTx: u32 = PCLK_UART0;
}
#[cfg(CONFIG_LPC_SERIAL_UART3_P429_P428)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "P4.29,P4.28");
    pub const GPIO_Rx: u32 = GPIO(4, 29);
    pub const GPIO_Tx: u32 = GPIO(4, 28);
    pub const GPIO_FUNCTION_UARTx: u32 = 3;
    pub type Uart = LPC_UART_TypeDef;
    pub const LPC_UARTx: *mut Uart = LPC_UART3;
    pub const UARTx_IRQn: IRQn_Type = UART3_IRQn;
    pub const PCLK_UARTx: u32 = PCLK_UART3;
}
use port::*;

fn uart() -> &'static Uart {
    unsafe { &*LPC_UARTx }
}

// Write tx bytes to the serial port
fn kick_tx() {
    loop {
        if uart().LSR.read() & (1 << 5) == 0 {
            // Output fifo full - enable tx irq
            uart().IER.write(0x03);
            break;
        }
        let mut data = 0;
        let ret = serial_get_tx_byte(&mut data);
        if ret != 0 {
            // No more data to send - disable tx irq
            uart().IER.write(0x01);
            break;
        }
        uart().THR().write(data);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn UARTx_IRQHandler() {
    let iir = uart().IIR.read();
    let status = iir & 0x0f;
    if status == 0x04 {
        serial_rx_byte(uart().RBR().read());
    } else if status == 0x02 {
        kick_tx();
    }
}

pub fn serial_enable_tx_irq() {
    if uart().LSR.read() & (1 << 5) != 0 {
        let flag = irq_save();
        kick_tx();
        irq_restore(flag);
    }
}

pub fn serial_init() -> R {
    // Setup baud
    enable_pclock(PCLK_UARTx);
    uart().LCR.write(1 << 7); // set DLAB bit
    let pclk = get_pclock_frequency(PCLK_UARTx);
    let div = pclk / (CONFIG_SERIAL_BAUD * 16);
    uart().DLL().write((div & 0xff) as u8);
    uart().DLM().write(((div >> 8) & 0xff) as u8);
    uart().FDR.write(0x10);
    uart().LCR.write(3); // 8N1 ; clear DLAB bit

    // Enable fifo
    uart().FCR().write(0x01);

    // Setup pins
    gpio_peripheral(GPIO_Rx, GPIO_FUNCTION_UARTx, 0);
    gpio_peripheral(GPIO_Tx, GPIO_FUNCTION_UARTx, 0);

    // Enable receive irq
    armcm_enable_irq!(UARTx_IRQHandler, UARTx_IRQn, 0);
    uart().IER.write(0x01);
    Ok(())
}
decl_init!(serial_init);
