// STM32 serial
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::config::CONFIG_SERIAL_BAUD;
use crate::generic::serial_irq::{serial_get_tx_byte, serial_rx_byte};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};

// Select the configured serial port
#[cfg(CONFIG_STM32_SERIAL_USART1)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA10,PA9");
    pub const GPIO_Rx: u32 = GPIO('A', 10);
    pub const GPIO_Tx: u32 = GPIO('A', 9);
    pub const GPIO_AF_MODE: u32 = 7;
    pub const USARTx: *mut USART_TypeDef = USART1;
    pub const USARTx_IRQn: IRQn_Type = USART1_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART1_ALT_PB7_PB6)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PB7,PB6");
    pub const GPIO_Rx: u32 = GPIO('B', 7);
    pub const GPIO_Tx: u32 = GPIO('B', 6);
    pub const GPIO_AF_MODE: u32 = 7;
    pub const USARTx: *mut USART_TypeDef = USART1;
    pub const USARTx_IRQn: IRQn_Type = USART1_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART2)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA3,PA2");
    pub const GPIO_Rx: u32 = GPIO('A', 3);
    pub const GPIO_Tx: u32 = GPIO('A', 2);
    pub const GPIO_AF_MODE: u32 = 7;
    pub const USARTx: *mut USART_TypeDef = USART2;
    pub const USARTx_IRQn: IRQn_Type = USART2_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART2_ALT_PD6_PD5)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PD6,PD5");
    pub const GPIO_Rx: u32 = GPIO('D', 6);
    pub const GPIO_Tx: u32 = GPIO('D', 5);
    pub const GPIO_AF_MODE: u32 = 7;
    pub const USARTx: *mut USART_TypeDef = USART2;
    pub const USARTx_IRQn: IRQn_Type = USART2_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART3)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PB11,PB10");
    pub const GPIO_Rx: u32 = GPIO('B', 11);
    pub const GPIO_Tx: u32 = GPIO('B', 10);
    pub const GPIO_AF_MODE: u32 = 7;
    pub const USARTx: *mut USART_TypeDef = USART3;
    pub const USARTx_IRQn: IRQn_Type = USART3_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART3_ALT_PD9_PD8)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PD9,PD8");
    pub const GPIO_Rx: u32 = GPIO('D', 9);
    pub const GPIO_Tx: u32 = GPIO('D', 8);
    pub const GPIO_AF_MODE: u32 = 7;
    pub const USARTx: *mut USART_TypeDef = USART3;
    pub const USARTx_IRQn: IRQn_Type = USART3_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART6)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA12,PA11");
    pub const GPIO_Rx: u32 = GPIO('A', 12);
    pub const GPIO_Tx: u32 = GPIO('A', 11);
    pub const GPIO_AF_MODE: u32 = 8;
    pub const USARTx: *mut USART_TypeDef = USART6;
    pub const USARTx_IRQn: IRQn_Type = USART6_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART6_ALT_PC7_PC6)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PC7,PC6");
    pub const GPIO_Rx: u32 = GPIO('C', 7);
    pub const GPIO_Tx: u32 = GPIO('C', 6);
    pub const GPIO_AF_MODE: u32 = 8;
    pub const USARTx: *mut USART_TypeDef = USART6;
    pub const USARTx_IRQn: IRQn_Type = USART6_IRQn;
}
use port::*;

const CR1_FLAGS: u32 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;

#[unsafe(no_mangle)]
pub unsafe extern "C" fn USARTx_IRQHandler() {
    let usart = &*USARTx;
    let sr = usart.SR.read();
    if sr & (USART_SR_RXNE | USART_SR_ORE) != 0 {
        // The ORE flag is automatically cleared by reading SR, followed
        // by reading DR.
        serial_rx_byte(usart.DR.read() as u8);
    }
    if sr & USART_SR_TXE != 0 && usart.CR1.read() & USART_CR1_TXEIE != 0 {
        let mut data = 0;
        let ret = serial_get_tx_byte(&mut data);
        if ret != 0 {
            usart.CR1.write(CR1_FLAGS);
        } else {
            usart.DR.write(data as u32);
        }
    }
}

pub fn serial_enable_tx_irq() {
    unsafe { &*USARTx }.CR1.write(CR1_FLAGS | USART_CR1_TXEIE);
}

pub fn serial_init() -> R {
    let usart = unsafe { &*USARTx };
    enable_pclock(USARTx as u32);

    let pclk = get_pclock_frequency(USARTx as u32);
    // DIV_ROUND_CLOSEST(pclk, CONFIG_SERIAL_BAUD)
    let div = (pclk + CONFIG_SERIAL_BAUD / 2) / CONFIG_SERIAL_BAUD;
    usart.BRR.write(((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
    usart.CR1.write(CR1_FLAGS);
    armcm_enable_irq!(USARTx_IRQHandler, USARTx_IRQn, 0);

    gpio_peripheral(GPIO_Rx, GPIO_FUNCTION(GPIO_AF_MODE), 1);
    gpio_peripheral(GPIO_Tx, GPIO_FUNCTION(GPIO_AF_MODE), 0);
    Ok(())
}
decl_init!(serial_init);
