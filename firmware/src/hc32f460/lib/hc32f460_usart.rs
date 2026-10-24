// lib/hc32f460/driver/src/hc32f460_usart.c: the USART in UART mode
//
// Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
// (BSD 3-Clause, see lib/hc32f460)
use super::super::internal::*;
use super::system_hc32f460::SystemCoreClock;

// Uart mode initialization configuration (hc32f460_usart.h)
pub struct stc_usart_uart_init_t {
    pub enClkMode: u32,
    pub enClkDiv: u32,
    pub enDataLength: u32,
    pub enDirection: u32,
    pub enStopBit: u32,
    pub enParity: u32,
    pub enSampleMode: u32,
    pub enDetectMode: u32,
    pub enHwFlow: u32,
}

// Parameter valid check for USART Instances.
fn IS_VALID_USART(USARTx: &M4_USART_TypeDef) -> bool {
    [M4_USART1, M4_USART2, M4_USART3, M4_USART4].iter().any(|&u| core::ptr::eq(u, USARTx))
}

// Initialize UART mode of the specified USART.
pub fn USART_UART_Init(USARTx: &M4_USART_TypeDef, pstcInitCfg: &stc_usart_uart_init_t) {
    use stc_usart_cr1_field_t as cr1;
    use stc_usart_cr2_field_t as cr2;
    use stc_usart_cr3_field_t as cr3;
    // (the local stc_usart_xx_field_t values, built field by field)
    let mut PR_f = 0u32;
    let mut CR1_f = 0u32;
    let mut CR2_f = 0u32;
    let mut CR3_f = 0u32;

    // Check USARTx && pstcInitCfg pointer
    if IS_VALID_USART(USARTx) {
        // Set default value
        USARTx.CR1.write(0x801B0000);
        USARTx.CR2.write(0x00000000);
        USARTx.CR3.write(0x00000000);
        USARTx.BRR.write(0x0000FFFF);
        USARTx.PR.write(0x00000000);

        // Set USART mode
        CR3_f = cr3::SCEN.set(CR3_f, 0);
        CR1_f = cr1::MS.set(CR1_f, 0);

        PR_f = stc_usart_pr_field_t::PSC.set(PR_f, pstcInitCfg.enClkDiv);
        CR1_f = cr1::M.set(CR1_f, pstcInitCfg.enDataLength);
        CR1_f = cr1::ML.set(CR1_f, pstcInitCfg.enDirection);
        CR2_f = cr2::STOP.set(CR2_f, pstcInitCfg.enStopBit);
        CR2_f = cr2::CLKC.set(CR2_f, pstcInitCfg.enClkMode);

        match pstcInitCfg.enParity {
            UsartParityNone => {
                CR1_f = cr1::PCE.set(CR1_f, 0);
            }
            UsartParityEven => {
                CR1_f = cr1::PS.set(CR1_f, 0);
                CR1_f = cr1::PCE.set(CR1_f, 1);
            }
            UsartParityOdd => {
                CR1_f = cr1::PS.set(CR1_f, 1);
                CR1_f = cr1::PCE.set(CR1_f, 1);
            }
            _ => {}
        }

        CR3_f = cr3::CTSE.set(CR3_f, pstcInitCfg.enHwFlow);
        CR1_f = cr1::SBS.set(CR1_f, pstcInitCfg.enDetectMode);
        CR1_f = cr1::OVER8.set(CR1_f, pstcInitCfg.enSampleMode);

        USARTx.PR.write(PR_f);
        USARTx.CR2.write(CR2_f);
        USARTx.CR3.write(CR3_f);
        USARTx.CR1.write(CR1_f);
    }
}

// Get USART status (en_flag_status_t)
pub fn USART_GetStatus(USARTx: &M4_USART_TypeDef, enStatus: u32) -> u32 {
    if USARTx.SR.read() & enStatus != 0 {
        Set
    } else {
        Reset
    }
}

// Clear USART status.
pub fn USART_ClearStatus(USARTx: &M4_USART_TypeDef, enStatus: u32) {
    use stc_usart_cr1_field_t as cr1;

    // Check USARTx pointer
    if IS_VALID_USART(USARTx) {
        match enStatus {
            UsartParityErr => USARTx.CR1.set_field(cr1::CPE, 1),
            UsartFrameErr => USARTx.CR1.set_field(cr1::CFE, 1),
            UsartOverrunErr => USARTx.CR1.set_field(cr1::CORE, 1),
            UsartRxTimeOut => USARTx.CR1.set_field(cr1::CRTOF, 1),
            _ => {}
        }
    }
}

// Configure USART function.
pub fn USART_FuncCmd(USARTx: &M4_USART_TypeDef, enFunc: u32, enCmd: u32) {
    use stc_usart_cr1_field_t as cr1;

    // Check USARTx pointer
    if IS_VALID_USART(USARTx) {
        match enFunc {
            UsartRx => USARTx.CR1.set_field(cr1::RE, enCmd),
            UsartRxInt => USARTx.CR1.set_field(cr1::RIE, enCmd),
            UsartTx => USARTx.CR1.set_field(cr1::TE, enCmd),
            UsartTxEmptyInt => USARTx.CR1.set_field(cr1::TXEIE, enCmd),
            UsartTimeOut => USARTx.CR1.set_field(cr1::RTOE, enCmd),
            UsartTimeOutInt => USARTx.CR1.set_field(cr1::RTOIE, enCmd),
            UsartSilentMode => USARTx.CR1.set_field(cr1::SLME, enCmd),
            UsartParityCheck => USARTx.CR1.set_field(cr1::PCE, enCmd),
            UsartNoiseFilter => USARTx.CR1.set_field(cr1::NFE, enCmd),
            UsartTxCmpltInt => USARTx.CR1.set_field(cr1::TCIE, enCmd),
            UsartTxAndTxEmptyInt => {
                // (one write of the register with both bits)
                let mut CR1_f = USARTx.CR1.read();
                CR1_f = cr1::TE.set(CR1_f, enCmd);
                CR1_f = cr1::TXEIE.set(CR1_f, enCmd);
                USARTx.CR1.write(CR1_f);
            }
            UsartFracBaudrate => USARTx.CR1.set_field(cr1::FBME, enCmd),
            UsartMulProcessor => USARTx.CR2.set_field(stc_usart_cr2_field_t::MPE, enCmd),
            UsartSmartCard => USARTx.CR3.set_field(stc_usart_cr3_field_t::SCEN, enCmd),
            UsartCts => USARTx.CR3.set_field(stc_usart_cr3_field_t::CTSE, enCmd),
            _ => {}
        }
    }
}

// USART send data.
pub fn USART_SendData(USARTx: &M4_USART_TypeDef, u16Data: u16) {
    USARTx.DR.set_field(stc_usart_dr_field_t::TDR, u16Data as u32);
}

// USART receive data.
pub fn USART_RecData(USARTx: &M4_USART_TypeDef) -> u16 {
    USARTx.DR.field(stc_usart_dr_field_t::RDR) as u16
}

// Set USART baudrate.
pub fn USART_SetBaudrate(USARTx: &M4_USART_TypeDef, u32Baudrate: u32) {
    // Check USARTx pointer
    if IS_VALID_USART(USARTx) {
        if 1 == USARTx.CR3.field(stc_usart_cr3_field_t::SCEN) {
            SetScBaudrate(USARTx, u32Baudrate);
        } else if 1 == USARTx.CR1.field(stc_usart_cr1_field_t::MS) {
            SetClkSyncBaudrate(USARTx, u32Baudrate);
        } else {
            SetUartBaudrate(USARTx, u32Baudrate);
        }
    }
}

// Set the baudrate registers (FBME, BRR) as all three modes do
fn SetBaudrateRegs(USARTx: &M4_USART_TypeDef, DIV_Integer: u32, DIV_Fraction: u32) {
    USARTx.CR1.set_field(stc_usart_cr1_field_t::FBME, if DIV_Fraction > 0x7F { 0 } else { 1 });
    USARTx.BRR.set_field(stc_usart_brr_field_t::DIV_FRACTION, DIV_Fraction);
    USARTx.BRR.set_field(stc_usart_brr_field_t::DIV_INTEGER, DIV_Integer);
}

// Set UART mode baudrate.
fn SetUartBaudrate(USARTx: &M4_USART_TypeDef, u32Baudrate: u32) {
    let mut DIV_Fraction: u32 = 0xFFFFFFFF;

    // Check USARTx pointer
    if IS_VALID_USART(USARTx) {
        let C = UsartGetClk(USARTx);

        if C > 0 {
            let B = u32Baudrate;
            let OVER8 = USARTx.CR1.field(stc_usart_cr1_field_t::OVER8);
            // FBME = 0 Calculation formula
            // B = C / (8 * (2 - OVER8) * (DIV_Integer + 1))
            // DIV_Integer = (C / (B * 8 * (2 - OVER8))) - 1
            let DIV = (C as f32 / (B as f32 * 8.0 * (2.0 - OVER8 as f32))) - 1.0;
            let DIV_Integer = DIV as u32;

            if !((DIV < 0.0) || (DIV_Integer > 0xFF)) {
                let mut ok = true;
                if (DIV - DIV_Integer as f32) > 0.00001 {
                    // FBME = 1 Calculation formula
                    // DIV_Fraction = (((2 - OVER8) * (DIV_Integer + 1) * 2048 * B) / C) - 128
                    let u64Tmp = (2u64 - OVER8 as u64) * (DIV_Integer as u64 + 1) * B as u64;
                    DIV_Fraction = (2048 * u64Tmp / C as u64).wrapping_sub(128) as u32;
                    if DIV_Fraction > 0x7F {
                        ok = false;
                    }
                }

                if ok {
                    SetBaudrateRegs(USARTx, DIV_Integer, DIV_Fraction);
                }
            }
        }
    }
}

// Set smart card mode baudrate.
fn SetScBaudrate(USARTx: &M4_USART_TypeDef, u32Baudrate: u32) {
    let mut DIV_Fraction: u32 = 0xFFFFFFFF;
    const au16EtuClkCnts: [u16; 8] = [32, 64, 93, 128, 186, 256, 372, 512];

    // Check USARTx pointer
    if IS_VALID_USART(USARTx) {
        let C = UsartGetClk(USARTx);

        if C > 0 {
            let B = u32Baudrate;
            let S = au16EtuClkCnts[USARTx.CR3.field(stc_usart_cr3_field_t::BCN) as usize] as u32;

            // FBME = 0 Calculation formula
            // B = C / (2 * S * (DIV_Integer + 1))
            // DIV_Integer = (C / (B * 2 * S)) - 1
            let DIV = (C as f32 / (B as f32 * S as f32 * 2.0)) - 1.0;
            let DIV_Integer = DIV as u32;

            if !((DIV < 0.0) || (DIV_Integer > 0xFF)) {
                let mut ok = true;
                if (DIV - DIV_Integer as f32) > 0.00001 {
                    // FBME = 1 Calculation formula
                    // DIV_Fraction = ((DIV_Integer + 1) * B * S * 512 / C) - 128
                    // (C: (DIV_Integer + 1) in 64 bits, times B and S in 64 bits)
                    let u64Tmp = (DIV_Integer as u64 + 1) * B as u64 * S as u64;
                    DIV_Fraction = (512 * u64Tmp / C as u64).wrapping_sub(128) as u32;
                    if DIV_Fraction > 0x7F {
                        ok = false;
                    }
                }

                if ok {
                    SetBaudrateRegs(USARTx, DIV_Integer, DIV_Fraction);
                }
            }
        }
    }
}

// Set clock sync mode baudrate.
fn SetClkSyncBaudrate(USARTx: &M4_USART_TypeDef, u32Baudrate: u32) {
    let mut DIV_Fraction: u32 = 0xFFFFFFFF;

    // Check USARTx pointer
    if IS_VALID_USART(USARTx) {
        let C = UsartGetClk(USARTx);
        if C > 0 {
            let B = u32Baudrate;

            // FBME = 0 Calculation formula
            // B = C / (4 * (DIV_Integer + 1))
            // DIV_Integer = (C / (B * 4)) - 1
            let DIV = (C as f32 / (B as f32 * 4.0)) - 1.0;
            let DIV_Integer = DIV as u32;

            if !((DIV < 0.0) || (DIV_Integer > 0xFF)) {
                let mut ok = true;
                if (DIV - DIV_Integer as f32) > 0.00001 {
                    // FBME = 1 Calculation formula
                    // DIV_Fraction = ((DIV_Integer + 1) * B * 1024 / C) - 128
                    let u64Tmp = (DIV_Integer as u64 + 1) * B as u64;
                    DIV_Fraction = (1024 * u64Tmp / C as u64).wrapping_sub(128) as u32;
                    if DIV_Fraction > 0x7F {
                        ok = false;
                    }
                }

                if ok {
                    SetBaudrateRegs(USARTx, DIV_Integer, DIV_Fraction);
                }
            }
        }
    }
}

// Get USART clock.
fn UsartGetClk(USARTx: &M4_USART_TypeDef) -> u32 {
    let sysreg = unsafe { &*M4_SYSREG };
    let u32PClk1 = *SystemCoreClock.get() / (1 << sysreg.CMU_SCFGR.field(stc_sysreg_cmu_scfgr_field_t::PCLK1S));
    u32PClk1 / (1 << (2 * USARTx.PR.field(stc_usart_pr_field_t::PSC)))
}
