// Serial over CAN emulation for LPC176x boards.
//
// Copyright (C) 2026  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::config::CONFIG_CANBUS_FREQUENCY;
use crate::ctr::Global;
use crate::generic::canbus::{
    canbus_notify_tx, canbus_process_data, CanbusMsg, CanbusStatus, CANBUS_STATE_ACTIVE, CANBUS_STATE_OFF, CANBUS_STATE_PASSIVE,
    CANBUS_STATE_WARN, CANMSG_DATA_LEN, CANMSG_ID_EFF, CANMSG_ID_RTR,
};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};

#[cfg(CONFIG_LPC_CANBUS_P0_0_P0_1)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "P0.0,P0.1");
    pub const LPC_CANx: *mut LPC_CAN_TypeDef = LPC_CAN1;
    pub const PCLK_CANx: u32 = PCLK_CAN1;
    pub const GPIO_Rx: u32 = GPIO(0, 0);
    pub const GPIO_Tx: u32 = GPIO(0, 1);
    pub const CAN_FUNCTION_Rx: u32 = 1;
    pub const CAN_FUNCTION_Tx: u32 = 1;
}
#[cfg(CONFIG_LPC_CANBUS_P0_21_P0_22)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "P0.21,P0.22");
    pub const LPC_CANx: *mut LPC_CAN_TypeDef = LPC_CAN1;
    pub const PCLK_CANx: u32 = PCLK_CAN1;
    pub const GPIO_Rx: u32 = GPIO(0, 21);
    pub const GPIO_Tx: u32 = GPIO(0, 22);
    pub const CAN_FUNCTION_Rx: u32 = 2;
    pub const CAN_FUNCTION_Tx: u32 = 2;
}
#[cfg(CONFIG_LPC_CANBUS_P0_4_P0_5)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "P0.4,P0.5");
    pub const LPC_CANx: *mut LPC_CAN_TypeDef = LPC_CAN2;
    pub const PCLK_CANx: u32 = PCLK_CAN2;
    pub const GPIO_Rx: u32 = GPIO(0, 4);
    pub const GPIO_Tx: u32 = GPIO(0, 5);
    pub const CAN_FUNCTION_Rx: u32 = 2;
    pub const CAN_FUNCTION_Tx: u32 = 2;
}
#[cfg(CONFIG_LPC_CANBUS_P2_7_P2_8)]
mod port {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "P2.7,P2.8");
    pub const LPC_CANx: *mut LPC_CAN_TypeDef = LPC_CAN2;
    pub const PCLK_CANx: u32 = PCLK_CAN2;
    pub const GPIO_Rx: u32 = GPIO(2, 7);
    pub const GPIO_Tx: u32 = GPIO(2, 8);
    pub const CAN_FUNCTION_Rx: u32 = 1;
    pub const CAN_FUNCTION_Tx: u32 = 1;
}
use port::*;

const CAN_MOD_RM: u32 = 1 << 0;

const CAN_CMR_TR: u32 = 1 << 0;
const CAN_CMR_RRB: u32 = 1 << 2;
const CAN_CMR_CDO: u32 = 1 << 3;
const CAN_CMR_STB1: u32 = 1 << 5;

const CAN_GSR_ES: u32 = 1 << 6;
const CAN_GSR_BS: u32 = 1 << 7;
const CAN_GSR_RXERR_Pos: u32 = 16;
const CAN_GSR_TXERR_Pos: u32 = 24;

const CAN_IER_RIE: u32 = 1 << 0;
const CAN_IER_TIE1: u32 = 1 << 1;
const CAN_IER_EIE: u32 = 1 << 2;
const CAN_IER_DOIE: u32 = 1 << 3;
const CAN_IER_EPIE: u32 = 1 << 5;
const CAN_IER_BEIE: u32 = 1 << 7;

const CAN_ICR_RI: u32 = 1 << 0;
const CAN_ICR_TI1: u32 = 1 << 1;
const CAN_ICR_EI: u32 = 1 << 2;
const CAN_ICR_DOI: u32 = 1 << 3;
const CAN_ICR_EPI: u32 = 1 << 5;
const CAN_ICR_BEI: u32 = 1 << 7;
const CAN_ICR_ERRDIR_RECEIVE: u32 = 1 << 21;

const CAN_SR_RBS: u32 = 1 << 0;
const CAN_SR_TBS1: u32 = 1 << 2;

const CAN_TFI_DLC_Pos: u32 = 16;
const CAN_TFI_RTR: u32 = 1 << 30;
const CAN_TFI_FF: u32 = 1 << 31;

const CAN_RFS_DLC_Pos: u32 = 16;
const CAN_RFS_RTR: u32 = 1 << 30;
const CAN_RFS_FF: u32 = 1 << 31;

const CAN_BTR_SJW_Pos: u32 = 14;
const CAN_BTR_TSEG1_Pos: u32 = 16;
const CAN_BTR_TSEG2_Pos: u32 = 20;

const CANAF_AFMR_ACCBP: u32 = 1 << 1;

const CAN_ERROR_PASSIVE_LIMIT: u32 = 128;
const CAN_ERROR_WARNING_LIMIT: u32 = 96;

struct CanErrors {
    rx_error: u32,
    tx_error: u32,
}
static CAN_Errors: Global<CanErrors> = Global::new(CanErrors { rx_error: 0, tx_error: 0 });

fn can() -> &'static LPC_CAN_TypeDef {
    unsafe { &*LPC_CANx }
}

fn make_btr(sjw: u32, time_seg1: u32, time_seg2: u32, brp: u32) -> u32 {
    ((brp - 1) & 0x3ff)
        | ((sjw - 1) << CAN_BTR_SJW_Pos)
        | ((time_seg1 - 1) << CAN_BTR_TSEG1_Pos)
        | ((time_seg2 - 1) << CAN_BTR_TSEG2_Pos)
}

// DIV_ROUND_CLOSEST of unsigned values
fn div_round_closest(x: u32, d: u32) -> u32 {
    (x + d / 2) / d
}

fn compute_btr(pclock: u32, bitrate: u32) -> u32 {
    let bit_clocks = pclock / bitrate;
    let mut best_sample_error = !0u32;
    let mut best_brp = bit_clocks / 10;
    let (mut time_seg1, mut time_seg2) = (7, 2);
    for qs in (8..=25u32).rev() {
        if bit_clocks % qs != 0 {
            continue;
        }
        let brp = bit_clocks / qs;
        if brp == 0 || brp > 1024 {
            continue;
        }

        let mut cur_time_seg2 = div_round_closest(qs, 8);
        if cur_time_seg2 < 2 {
            cur_time_seg2 = 2;
        }
        if cur_time_seg2 > 8 {
            cur_time_seg2 = 8;
        }

        let mut cur_time_seg1 = qs - (1 + cur_time_seg2);
        if cur_time_seg1 > 16 {
            cur_time_seg1 = 16;
            cur_time_seg2 = qs - (1 + cur_time_seg1);
        }
        if cur_time_seg1 == 0 || cur_time_seg2 < 2 || cur_time_seg2 > 8 {
            continue;
        }

        let sample_point = div_round_closest((1 + cur_time_seg1) * 1000, qs);
        let sample_error = if sample_point > 875 { sample_point - 875 } else { 875 - sample_point };
        if sample_error < best_sample_error {
            best_sample_error = sample_error;
            best_brp = brp;
            time_seg1 = cur_time_seg1;
            time_seg2 = cur_time_seg2;
        }
    }

    make_btr(2, time_seg1, time_seg2, best_brp)
}

// Transmit a packet
pub fn canhw_send(msg: &CanbusMsg) -> i32 {
    let c = can();
    if c.SR.read() & CAN_SR_TBS1 == 0 {
        let flag = irq_save();
        c.IER.or(CAN_IER_TIE1);
        if c.SR.read() & CAN_SR_TBS1 != 0 {
            c.IER.and(!CAN_IER_TIE1);
            irq_restore(flag);
            canbus_notify_tx();
        } else {
            irq_restore(flag);
        }
        return -1;
    }

    let mut tfi = CANMSG_DATA_LEN(msg) << CAN_TFI_DLC_Pos;
    if msg.id & CANMSG_ID_EFF != 0 {
        tfi |= CAN_TFI_FF;
    }
    if msg.id & CANMSG_ID_RTR != 0 {
        tfi |= CAN_TFI_RTR;
    }

    c.TFI1.write(tfi);
    c.TID1.write(msg.id & if msg.id & CANMSG_ID_EFF != 0 { 0x1fffffff } else { 0x7ff });
    c.TDA1.write(msg.data32(0));
    c.TDB1.write(msg.data32(1));
    c.CMR.write(CAN_CMR_STB1 | CAN_CMR_TR);
    CANMSG_DATA_LEN(msg) as i32
}

// Setup the receive packet filter
pub fn canhw_set_filter(_id: u32) {
    // Keep the LPC acceptance filter in bypass mode; canserial filters in
    // software and this avoids a fragile AF LUT rewrite while the bus is live.
}

// Report interface status
pub fn canhw_get_status(status: &mut CanbusStatus) {
    let flag = irq_save();
    let gsr = can().GSR.read();
    let e = CAN_Errors.get();
    let (rx_error, tx_error) = (e.rx_error, e.tx_error);
    irq_restore(flag);

    let rx_count = (gsr >> CAN_GSR_RXERR_Pos) & 0xff;
    let tx_count = (gsr >> CAN_GSR_TXERR_Pos) & 0xff;
    status.rx_error = rx_error;
    status.tx_error = tx_error;
    status.bus_state = if gsr & CAN_GSR_BS != 0 {
        CANBUS_STATE_OFF
    } else if rx_count >= CAN_ERROR_PASSIVE_LIMIT || tx_count >= CAN_ERROR_PASSIVE_LIMIT {
        CANBUS_STATE_PASSIVE
    } else if gsr & CAN_GSR_ES != 0 {
        CANBUS_STATE_WARN
    } else {
        CANBUS_STATE_ACTIVE
    };
}

fn can_process_rx() {
    let c = can();
    while c.SR.read() & CAN_SR_RBS != 0 {
        let rfs = c.RFS.read();
        let mut msg = CanbusMsg { id: 0, dlc: 0, data: [0; 8] };
        if rfs & CAN_RFS_FF != 0 {
            msg.id = (c.RID.read() & 0x1fffffff) | CANMSG_ID_EFF;
        } else {
            msg.id = c.RID.read() & 0x7ff;
        }
        msg.id |= if rfs & CAN_RFS_RTR != 0 { CANMSG_ID_RTR } else { 0 };
        msg.dlc = (rfs >> CAN_RFS_DLC_Pos) & 0x0f;
        msg.set_data32(0, c.RDA.read());
        msg.set_data32(1, c.RDB.read());
        c.CMR.write(CAN_CMR_RRB);

        canbus_process_data(&msg);
    }
}

// This function handles CAN global interrupts
#[unsafe(no_mangle)]
pub unsafe extern "C" fn CAN_IRQHandler() {
    let c = can();
    let icr = c.ICR.read();
    let e = CAN_Errors.get();

    if icr & (CAN_ICR_RI | CAN_ICR_DOI) != 0 {
        can_process_rx();
    }

    if icr & CAN_ICR_DOI != 0 {
        e.rx_error = e.rx_error.wrapping_add(1);
        c.CMR.write(CAN_CMR_CDO);
    }

    if icr & CAN_ICR_TI1 != 0 {
        c.IER.and(!CAN_IER_TIE1);
        canbus_notify_tx();
    }

    if icr & CAN_ICR_BEI != 0 {
        if icr & CAN_ICR_ERRDIR_RECEIVE != 0 {
            e.rx_error = e.rx_error.wrapping_add(1);
        } else {
            e.tx_error = e.tx_error.wrapping_add(1);
        }
    }

    if icr & (CAN_ICR_EI | CAN_ICR_EPI) != 0 {
        let gsr = c.GSR.read();
        if ((gsr >> CAN_GSR_RXERR_Pos) & 0xff) >= CAN_ERROR_WARNING_LIMIT {
            e.rx_error = e.rx_error.wrapping_add(1);
        }
        if ((gsr >> CAN_GSR_TXERR_Pos) & 0xff) >= CAN_ERROR_WARNING_LIMIT {
            e.tx_error = e.tx_error.wrapping_add(1);
        }
    }
}

pub fn can_init() -> R {
    let c = can();
    // Enable clock and hold the CAN controller in reset while configuring it.
    enable_pclock(PCLK_CANx);
    c.MOD.write(CAN_MOD_RM);
    c.IER.write(0);

    gpio_peripheral(GPIO_Rx, CAN_FUNCTION_Rx, 1);
    gpio_peripheral(GPIO_Tx, CAN_FUNCTION_Tx, 0);

    unsafe { &*LPC_CANAF }.AFMR.write(CANAF_AFMR_ACCBP);
    c.BTR.write(compute_btr(get_pclock_frequency(PCLK_CANx), CONFIG_CANBUS_FREQUENCY));
    c.EWL.write(CAN_ERROR_WARNING_LIMIT);

    // Clear any stale interrupt state and enter normal operating mode.
    let _icr = c.ICR.read();
    c.MOD.write(0);

    armcm_enable_irq!(CAN_IRQHandler, CAN_IRQn, 0);
    c.IER.write(CAN_IER_RIE | CAN_IER_EIE | CAN_IER_DOIE | CAN_IER_EPIE | CAN_IER_BEIE);
    Ok(())
}
decl_init!(can_init);
