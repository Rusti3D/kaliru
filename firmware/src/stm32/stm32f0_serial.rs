// STM32F0 serial
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals, non_snake_case)]
use crate::config::CONFIG_SERIAL_BAUD;
use crate::generic::serial_irq::{serial_get_tx_byte, serial_rx_byte};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};
use names::*;

// The chip header names, with the C #defines that rename some of them
#[allow(dead_code)]
mod names {
    #[allow(unused_imports)]
    pub use crate::config::{CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G4, CONFIG_MACH_STM32H7};
    pub use crate::stm32::internal::*;

    // The stm32f031 has same pins for USART2, but everything is routed to USART1
    #[cfg(CONFIG_MACH_STM32F031)]
    pub const USART2: *mut USART_TypeDef = USART1;
    #[cfg(CONFIG_MACH_STM32F031)]
    pub const USART2_IRQn: IRQn_Type = USART1_IRQn;

    // Some of the stm32g0 MCUs have slightly different register names
    #[cfg(all(CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G0B1))]
    mod g0 {
        use super::*;
        pub const USART2_IRQn: IRQn_Type = USART2_LPUART2_IRQn;
        pub const USART3_IRQn: IRQn_Type = USART3_4_5_6_LPUART1_IRQn;
        pub const USART4_IRQn: IRQn_Type = USART3_4_5_6_LPUART1_IRQn;
        pub const USART5_IRQn: IRQn_Type = USART3_4_5_6_LPUART1_IRQn;
        pub const USART6_IRQn: IRQn_Type = USART3_4_5_6_LPUART1_IRQn;
    }
    #[cfg(all(CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G0B0))]
    mod g0 {
        use super::*;
        // (#define USART2_IRQn USART2_IRQn: the chip's name)
        pub const USART3_IRQn: IRQn_Type = USART3_4_5_6_IRQn;
        pub const USART4_IRQn: IRQn_Type = USART3_4_5_6_IRQn;
        pub const USART5_IRQn: IRQn_Type = USART3_4_5_6_IRQn;
        pub const USART6_IRQn: IRQn_Type = USART3_4_5_6_IRQn;
    }
    #[cfg(all(CONFIG_MACH_STM32G0, any(CONFIG_MACH_STM32G0B1, CONFIG_MACH_STM32G0B0)))]
    pub use g0::*;
    #[cfg(CONFIG_MACH_STM32G0)]
    pub const USART_CR1_RXNEIE: u32 = USART_CR1_RXNEIE_RXFNEIE;
    #[cfg(CONFIG_MACH_STM32G0)]
    pub const USART_CR1_TXEIE: u32 = USART_CR1_TXEIE_TXFNFIE;
    #[cfg(any(CONFIG_MACH_STM32G0, CONFIG_MACH_STM32H7))]
    pub const USART_ISR_RXNE: u32 = USART_ISR_RXNE_RXFNE;
    #[cfg(any(CONFIG_MACH_STM32G0, CONFIG_MACH_STM32H7))]
    pub const USART_ISR_TXE: u32 = USART_ISR_TXE_TXFNF;
    #[cfg(any(CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G4))]
    pub const USART_BRR_DIV_MANTISSA_Pos: u32 = 4;
    #[cfg(any(CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G4))]
    pub const USART_BRR_DIV_FRACTION_Pos: u32 = 0;
}

// Select the configured serial port
#[cfg(CONFIG_STM32_SERIAL_USART1)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA10,PA9");
    pub const GPIO_Rx: u32 = GPIO('A', 10);
    pub const GPIO_Tx: u32 = GPIO('A', 9);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(if CONFIG_MACH_STM32H7 | CONFIG_MACH_STM32G4 { 7 } else { 1 });
    pub const USARTx: *mut USART_TypeDef = USART1;
    pub const USARTx_IRQn: IRQn_Type = USART1_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART1_ALT_PB7_PB6)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PB7,PB6");
    pub const GPIO_Rx: u32 = GPIO('B', 7);
    pub const GPIO_Tx: u32 = GPIO('B', 6);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(if CONFIG_MACH_STM32H7 | CONFIG_MACH_STM32G4 { 7 } else { 0 });
    pub const USARTx: *mut USART_TypeDef = USART1;
    pub const USARTx_IRQn: IRQn_Type = USART1_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART2)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA3,PA2");
    pub const GPIO_Rx: u32 = GPIO('A', 3);
    pub const GPIO_Tx: u32 = GPIO('A', 2);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(if CONFIG_MACH_STM32H7 | CONFIG_MACH_STM32G4 { 7 } else { 1 });
    pub const USARTx: *mut USART_TypeDef = USART2;
    pub const USARTx_IRQn: IRQn_Type = USART2_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART2_ALT_PA15_PA14)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA15,PA14");
    pub const GPIO_Rx: u32 = GPIO('A', 15);
    pub const GPIO_Tx: u32 = GPIO('A', 14);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(if CONFIG_MACH_STM32G4 { 7 } else { 1 });
    pub const USARTx: *mut USART_TypeDef = USART2;
    pub const USARTx_IRQn: IRQn_Type = USART2_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART2_ALT_PB4_PB3)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PB4,PB3");
    pub const GPIO_Rx: u32 = GPIO('B', 4);
    pub const GPIO_Tx: u32 = GPIO('B', 3);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(7);
    pub const USARTx: *mut USART_TypeDef = USART2;
    pub const USARTx_IRQn: IRQn_Type = USART2_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART2_ALT_PD6_PD5)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PD6,PD5");
    pub const GPIO_Rx: u32 = GPIO('D', 6);
    pub const GPIO_Tx: u32 = GPIO('D', 5);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(7);
    pub const USARTx: *mut USART_TypeDef = USART2;
    pub const USARTx_IRQn: IRQn_Type = USART2_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART3)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PB11,PB10");
    pub const GPIO_Rx: u32 = GPIO('B', 11);
    pub const GPIO_Tx: u32 = GPIO('B', 10);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(7);
    pub const USARTx: *mut USART_TypeDef = USART3;
    pub const USARTx_IRQn: IRQn_Type = USART3_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART3_ALT_PD9_PD8)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PD9,PD8");
    pub const GPIO_Rx: u32 = GPIO('D', 9);
    pub const GPIO_Tx: u32 = GPIO('D', 8);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(if CONFIG_MACH_STM32G0 { 0 } else { 7 });
    pub const USARTx: *mut USART_TypeDef = USART3;
    pub const USARTx_IRQn: IRQn_Type = USART3_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART3_ALT_PC11_PC10)]
mod port {
    use super::*;
    //  Currently only supports STM32G474.
    decl_constant_str!("RESERVE_PINS_serial", "PC11,PC10");
    pub const GPIO_Rx: u32 = GPIO('C', 11);
    pub const GPIO_Tx: u32 = GPIO('C', 10);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(7);
    pub const USARTx: *mut USART_TypeDef = USART3;
    pub const USARTx_IRQn: IRQn_Type = USART3_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_UART4)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA1,PA0");
    pub const GPIO_Rx: u32 = GPIO('A', 1);
    pub const GPIO_Tx: u32 = GPIO('A', 0);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(8);
    pub const USARTx: *mut USART_TypeDef = UART4;
    pub const USARTx_IRQn: IRQn_Type = UART4_IRQn;
}
#[cfg(CONFIG_STM32_SERIAL_USART5)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PD2,PD3");
    pub const GPIO_Rx: u32 = GPIO('D', 2);
    pub const GPIO_Tx: u32 = GPIO('D', 3);
    pub const USARTx_FUNCTION: u32 = GPIO_FUNCTION(3);
    pub const USARTx: *mut USART_TypeDef = USART5;
    pub const USARTx_IRQn: IRQn_Type = USART5_IRQn;
}
use port::*;

const CR1_FLAGS: u32 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;

#[unsafe(no_mangle)]
pub unsafe extern "C" fn USARTx_IRQHandler() {
    let usart = &*USARTx;
    let sr = usart.ISR.read();
    if sr & USART_ISR_RXNE != 0 {
        serial_rx_byte(usart.RDR.read() as u8);
    }
    if sr & USART_ISR_TXE != 0 && usart.CR1.read() & USART_CR1_TXEIE != 0 {
        let mut data = 0;
        let ret = serial_get_tx_byte(&mut data);
        if ret != 0 {
            usart.CR1.write(CR1_FLAGS);
        } else {
            usart.TDR.write(data as _);
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
    usart.BRR.write(((div / 16) << USART_BRR_DIV_MANTISSA_Pos) | ((div % 16) << USART_BRR_DIV_FRACTION_Pos));
    usart.CR3.write(USART_CR3_OVRDIS); // disable the ORE ISR
    usart.CR1.write(CR1_FLAGS);
    armcm_enable_irq!(USARTx_IRQHandler, USARTx_IRQn, 0);

    gpio_peripheral(GPIO_Rx, USARTx_FUNCTION, 1);
    gpio_peripheral(GPIO_Tx, USARTx_FUNCTION, 0);
    Ok(())
}
decl_init!(serial_init);
