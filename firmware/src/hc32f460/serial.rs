// HC32F460 serial
//
// Copyright (C) 2022  Steven Gotthardt <gotthardt@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use super::lib::hc32f460_gpio::PORT_SetFunc;
use super::lib::hc32f460_pwc::PWC_Fcg1PeriphClockCmd;
use super::lib::hc32f460_usart::*;
use crate::config::CONFIG_SERIAL_BAUD;
use crate::generic::serial_irq::{serial_get_tx_byte, serial_rx_byte};
use crate::sched::R;
use crate::{decl_armcm_irq, decl_constant_str, decl_init};

const USART_BAUDRATE: u32 = CONFIG_SERIAL_BAUD;

// Aquila 1.0.3 pins for TX, RX to CH304 on PA15 and PA09
//    for the LCD connector: TX, RX on PC00 and PC01
#[cfg(CONFIG_HC32F460_SERIAL_PA15_PA9)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA15,PA9");
    pub const USART_RX_PORT: u32 = PortA;
    pub const USART_RX_PIN: u32 = Pin15;
    pub const USART_TX_PORT: u32 = PortA;
    pub const USART_TX_PIN: u32 = Pin09;
    pub const USART_NUM: u32 = 1;
}
#[cfg(CONFIG_HC32F460_SERIAL_PC0_PC1)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PC0,PC1");
    pub const USART_RX_PORT: u32 = PortC;
    pub const USART_RX_PIN: u32 = Pin00;
    pub const USART_TX_PORT: u32 = PortC;
    pub const USART_TX_PIN: u32 = Pin01;
    pub const USART_NUM: u32 = 1;
}
#[cfg(CONFIG_HC32F460_SERIAL_PA3_PA2)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA3,PA2");
    pub const USART_RX_PORT: u32 = PortA;
    pub const USART_RX_PIN: u32 = Pin03;
    pub const USART_TX_PORT: u32 = PortA;
    pub const USART_TX_PIN: u32 = Pin02;
    pub const USART_NUM: u32 = 1;
}
#[cfg(CONFIG_HC32F460_SERIAL_PH2_PB10)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PH2,PB10");
    pub const USART_RX_PORT: u32 = PortH;
    pub const USART_RX_PIN: u32 = Pin02;
    pub const USART_TX_PORT: u32 = PortB;
    pub const USART_TX_PIN: u32 = Pin10;
    pub const USART_NUM: u32 = 3;
}
#[cfg(CONFIG_HC32F460_SERIAL_PA7_PA8)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA7,PA8");
    pub const USART_RX_PORT: u32 = PortA;
    pub const USART_RX_PIN: u32 = Pin07;
    pub const USART_TX_PORT: u32 = PortA;
    pub const USART_TX_PIN: u32 = Pin08;
    pub const USART_NUM: u32 = 1;
}
#[cfg(CONFIG_HC32F460_SERIAL_PA13_PA14)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_serial", "PA13,PA14");
    pub const USART_RX_PORT: u32 = PortA;
    pub const USART_RX_PIN: u32 = Pin13;
    pub const USART_TX_PORT: u32 = PortA;
    pub const USART_TX_PIN: u32 = Pin14;
    pub const USART_NUM: u32 = 1;
}
use pins::*;

// (USART_NUM == 1: use USART 1 for serial connection, 3: USART 3)
const USARTx: *mut M4_USART_TypeDef = if USART_NUM == 1 { M4_USART1 } else { M4_USART3 };

const USART_RX_FUNC: u32 = if USART_NUM == 1 { Func_Usart1_Rx } else { Func_Usart3_Rx };
const USART_TX_FUNC: u32 = if USART_NUM == 1 { Func_Usart1_Tx } else { Func_Usart3_Tx };
const USART_RI_NUM: u32 = if USART_NUM == 1 { INT_USART1_RI } else { INT_USART3_RI };
const USART_TI_NUM: u32 = if USART_NUM == 1 { INT_USART1_TI } else { INT_USART3_TI };
const USART_EI_NUM: u32 = if USART_NUM == 1 { INT_USART1_EI } else { INT_USART3_EI };
const USART_TCI_NUM: u32 = if USART_NUM == 1 { INT_USART1_TCI } else { INT_USART3_TCI };

const USART_ENABLE: u32 = PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 | PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;

fn usart() -> &'static M4_USART_TypeDef {
    unsafe { &*USARTx }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn serialError() {
    if Set == USART_GetStatus(usart(), UsartFrameErr) {
        USART_ClearStatus(usart(), UsartFrameErr);

        // use it anyway
        serial_rx_byte(usart().DR.field(stc_usart_dr_field_t::RDR) as u8);
    }

    if Set == USART_GetStatus(usart(), UsartOverrunErr) {
        USART_ClearStatus(usart(), UsartOverrunErr);
    }
}
decl_armcm_irq!(serialError, Int001_IRQn);

#[unsafe(no_mangle)]
pub unsafe extern "C" fn serialTxComplete() {
    USART_FuncCmd(usart(), UsartTx, Disable);
    USART_FuncCmd(usart(), UsartTxCmpltInt, Disable);
}
decl_armcm_irq!(serialTxComplete, Int003_IRQn);

#[unsafe(no_mangle)]
pub unsafe extern "C" fn serialRx() {
    let data = USART_RecData(usart());

    // call to klipper generic/serial_irq function
    serial_rx_byte(data as u8);
}
decl_armcm_irq!(serialRx, Int000_IRQn);

#[unsafe(no_mangle)]
pub unsafe extern "C" fn serialTxEmpty() {
    let mut data2send = 0u8;

    // use helper from generic - zero means byte ready
    if serial_get_tx_byte(&mut data2send) == 0 {
        //USARTx->DR_f.TDR = data2send;
        USART_SendData(usart(), data2send as u16);
    } else {
        // no more data - stop the interrupt
        USART_FuncCmd(usart(), UsartTxCmpltInt, Enable);
        USART_FuncCmd(usart(), UsartTxEmptyInt, Disable);
    }
}
decl_armcm_irq!(serialTxEmpty, Int002_IRQn);

// called by generic framework
pub fn serial_enable_tx_irq() {
    /* Enable TX && TX empty interrupt */
    USART_FuncCmd(usart(), UsartTxAndTxEmptyInt, Enable);
}

pub fn serial_init() -> R {
    let stcInitCfg = stc_usart_uart_init_t {
        enClkMode: UsartIntClkCkNoOutput,
        enClkDiv: UsartClkDiv_1,
        enDataLength: UsartDataBits8,
        enDirection: UsartDataLsbFirst,
        enStopBit: UsartOneStopBit,
        enParity: UsartParityNone,
        enSampleMode: UsartSampleBit16,
        enDetectMode: UsartStartBitFallEdge,
        enHwFlow: UsartRtsEnable,
    };

    // Function Clock Control - USART enable (sets bit to a 0)
    PWC_Fcg1PeriphClockCmd(USART_ENABLE, Enable);

    /* Initialize port pins for USART IO - Disable = NO subfunction */
    PORT_SetFunc(USART_RX_PORT, USART_RX_PIN as u16, USART_RX_FUNC, Disable);
    PORT_SetFunc(USART_TX_PORT, USART_TX_PIN as u16, USART_TX_FUNC, Disable);

    /* Initialize UART */
    USART_UART_Init(usart(), &stcInitCfg);

    /* Set baudrate */
    USART_SetBaudrate(usart(), USART_BAUDRATE);

    /* A word on interrupts in HC32F460
      In other vendors (STM) the irqs are assigned names in the vector list
        eg: USART1_IRQn but HC32F460 has numbered IRQs: IRQ000_IRQn IRQ143_IRQn
        Using INT_USART1_RI from hc32f460.h put in to IRQn->INTSEL
         if n = 5 then the USART1_RI iterrupt will be at IRQ005_IRQn
         For the specific case of each USART there are 5 separate irqs to map
    */

    /* Set USART RX IRQ */
    IrqRegistration(USART_RI_NUM, Int000_IRQn);

    /* Set USART err */
    IrqRegistration(USART_EI_NUM, Int001_IRQn);

    /* Set USART TX IRQ */
    IrqRegistration(USART_TI_NUM, Int002_IRQn);

    /* Set USART TX complete IRQ */
    IrqRegistration(USART_TCI_NUM, Int003_IRQn);

    /* Enable RX && RX interrupt function */
    USART_FuncCmd(usart(), UsartRx, Enable);
    USART_FuncCmd(usart(), UsartRxInt, Enable);
    Ok(())
}
decl_init!(serial_init);
