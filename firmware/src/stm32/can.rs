// Serial over CAN emulation for STM32 boards.
//
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <liquidpontus@yahoo.se>
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals, non_snake_case)]
use super::internal::*;
use crate::board::{irq_disable, irq_enable, irq_restore, irq_save};
use crate::config::{CONFIG_CANBUS_FILTER, CONFIG_CANBUS_FREQUENCY};
use crate::ctr::Global;
use crate::generic::canbus::{
    canbus_notify_tx, canbus_process_data, CanbusMsg, CanbusStatus, CANBUS_STATE_OFF, CANBUS_STATE_PASSIVE, CANBUS_STATE_WARN,
    CANMSG_DATA_LEN, CANMSG_ID_EFF, CANMSG_ID_RTR,
};
use crate::generic::canserial::CANBUS_ID_ADMIN;
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};

#[cfg(any(CONFIG_STM32_CANBUS_PA11_PA12, CONFIG_STM32_CANBUS_PA11_PA12_REMAP))]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PA11,PA12");
    pub const GPIO_Rx: u32 = GPIO('A', 11);
    pub const GPIO_Tx: u32 = GPIO('A', 12);
}
#[cfg(CONFIG_STM32_CANBUS_PB8_PB9)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PB8,PB9");
    pub const GPIO_Rx: u32 = GPIO('B', 8);
    pub const GPIO_Tx: u32 = GPIO('B', 9);
}
#[cfg(CONFIG_STM32_CANBUS_PI9_PH13)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PI9,PH13");
    pub const GPIO_Rx: u32 = GPIO('I', 9);
    pub const GPIO_Tx: u32 = GPIO('H', 13);
}
#[cfg(CONFIG_STM32_CANBUS_PB5_PB6)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PB5,PB6");
    pub const GPIO_Rx: u32 = GPIO('B', 5);
    pub const GPIO_Tx: u32 = GPIO('B', 6);
}
#[cfg(CONFIG_STM32_CANBUS_PB12_PB13)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PB12,PB13");
    pub const GPIO_Rx: u32 = GPIO('B', 12);
    pub const GPIO_Tx: u32 = GPIO('B', 13);
}
#[cfg(CONFIG_STM32_CANBUS_PD0_PD1)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PD0,PD1");
    pub const GPIO_Rx: u32 = GPIO('D', 0);
    pub const GPIO_Tx: u32 = GPIO('D', 1);
}
use pins::*;

#[cfg(CONFIG_MACH_STM32F0)]
mod soc {
    use super::*;
    pub const SOC_CAN: *mut CAN_TypeDef = CAN;
    pub const FILTER_CAN: *mut CAN_TypeDef = CAN;
    pub const CAN_RX0_IRQn: IRQn_Type = CEC_CAN_IRQn;
    pub const CAN_RX1_IRQn: IRQn_Type = CEC_CAN_IRQn;
    pub const CAN_TX_IRQn: IRQn_Type = CEC_CAN_IRQn;
    pub const CAN_SCE_IRQn: IRQn_Type = CEC_CAN_IRQn;
    pub const CAN_FUNCTION: u32 = GPIO_FUNCTION(4); // Alternative function mapping number
}
#[cfg(CONFIG_MACH_STM32F1)]
mod soc {
    use super::*;
    pub const SOC_CAN: *mut CAN_TypeDef = CAN1;
    pub const FILTER_CAN: *mut CAN_TypeDef = CAN1;
    pub const CAN_RX0_IRQn: IRQn_Type = CAN1_RX0_IRQn;
    pub const CAN_RX1_IRQn: IRQn_Type = CAN1_RX1_IRQn;
    pub const CAN_TX_IRQn: IRQn_Type = CAN1_TX_IRQn;
    pub const CAN_SCE_IRQn: IRQn_Type = CAN1_SCE_IRQn;
    pub const CAN_FUNCTION: u32 = GPIO_FUNCTION(9); // Alternative function mapping number
}
#[cfg(all(
    CONFIG_MACH_STM32F4,
    any(CONFIG_STM32_CANBUS_PA11_PA12, CONFIG_STM32_CANBUS_PB8_PB9, CONFIG_STM32_CANBUS_PD0_PD1, CONFIG_STM32_CANBUS_PI9_PH13)
))]
mod soc {
    use super::*;
    pub const SOC_CAN: *mut CAN_TypeDef = CAN1;
    pub const FILTER_CAN: *mut CAN_TypeDef = CAN1;
    pub const CAN_RX0_IRQn: IRQn_Type = CAN1_RX0_IRQn;
    pub const CAN_RX1_IRQn: IRQn_Type = CAN1_RX1_IRQn;
    pub const CAN_TX_IRQn: IRQn_Type = CAN1_TX_IRQn;
    pub const CAN_SCE_IRQn: IRQn_Type = CAN1_SCE_IRQn;
    pub const CAN_FUNCTION: u32 = GPIO_FUNCTION(9); // Alternative function mapping number
}
#[cfg(all(
    CONFIG_MACH_STM32F4,
    not(any(CONFIG_STM32_CANBUS_PA11_PA12, CONFIG_STM32_CANBUS_PB8_PB9, CONFIG_STM32_CANBUS_PD0_PD1, CONFIG_STM32_CANBUS_PI9_PH13)),
    any(CONFIG_STM32_CANBUS_PB5_PB6, CONFIG_STM32_CANBUS_PB12_PB13)
))]
mod soc {
    use super::*;
    pub const SOC_CAN: *mut CAN_TypeDef = CAN2;
    pub const FILTER_CAN: *mut CAN_TypeDef = CAN1;
    pub const CAN_RX0_IRQn: IRQn_Type = CAN2_RX0_IRQn;
    pub const CAN_RX1_IRQn: IRQn_Type = CAN2_RX1_IRQn;
    pub const CAN_TX_IRQn: IRQn_Type = CAN2_TX_IRQn;
    pub const CAN_SCE_IRQn: IRQn_Type = CAN2_SCE_IRQn;
    pub const CAN_FUNCTION: u32 = GPIO_FUNCTION(9); // Alternative function mapping number
}
use soc::*;

fn can() -> &'static CAN_TypeDef {
    unsafe { &*SOC_CAN }
}

// Transmit a packet
pub fn canhw_send(msg: &CanbusMsg) -> i32 {
    let c = can();
    let tsr = c.TSR.read();
    if tsr & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2) == 0 {
        // No space in transmit fifo - enable tx irq
        irq_disable();
        c.IER.or(CAN_IER_TMEIE);
        irq_enable();
        return -1;
    }
    let mut mbox = 2;
    if tsr & CAN_TSR_TME0 != 0 {
        mbox = 0;
    } else if tsr & CAN_TSR_TME1 != 0 {
        mbox = 1;
    }
    let mb = &c.sTxMailBox[mbox];

    // Set up the DLC
    mb.TDTR.write((mb.TDTR.read() & 0xFFFFFFF0) | (msg.dlc & 0x0F));

    // Set up the data field
    mb.TDLR.write(msg.data32(0));
    mb.TDHR.write(msg.data32(1));

    // Request transmission
    let mut tir = if msg.id & CANMSG_ID_EFF != 0 {
        ((msg.id & 0x1fffffff) << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE
    } else {
        (msg.id & 0x7ff) << CAN_TI0R_STID_Pos
    };
    tir |= if msg.id & CANMSG_ID_RTR != 0 { CAN_TI0R_RTR } else { 0 };
    mb.TIR.write(tir | CAN_TI0R_TXRQ);
    CANMSG_DATA_LEN(msg) as i32
}

// Setup the receive packet filter
pub fn canhw_set_filter(mut id: u32) {
    let fcan = unsafe { &*FILTER_CAN };
    // Select the start slave bank
    let mut fmr = fcan.FMR.read();
    if FILTER_CAN != SOC_CAN {
        // Using CAN2 with filter on CAN1 - assign CAN2 to first filter
        fmr &= !CAN_FMR_CAN2SB;
    }
    fcan.FMR.write(fmr | CAN_FMR_FINIT);
    // Initialisation mode for the filter
    fcan.FA1R.write(0);

    if CONFIG_CANBUS_FILTER {
        let mask = CAN_TI0R_STID | CAN_TI0R_IDE | CAN_TI0R_RTR;
        fcan.sFilterRegister[0].FR1.write(CANBUS_ID_ADMIN << CAN_RI0R_STID_Pos);
        fcan.sFilterRegister[0].FR2.write(mask);
        fcan.sFilterRegister[1].FR1.write((id + 1) << CAN_RI0R_STID_Pos);
        fcan.sFilterRegister[1].FR2.write(mask);
        fcan.sFilterRegister[2].FR1.write(id << CAN_RI0R_STID_Pos);
        fcan.sFilterRegister[2].FR2.write(mask);
    } else {
        fcan.sFilterRegister[0].FR1.write(0);
        fcan.sFilterRegister[0].FR2.write(0);
        id = 0;
    }

    // 32-bit scale for the filter
    fcan.FS1R.write((1 << 0) | (1 << 1) | (1 << 2));

    // Filter activation
    fcan.FA1R.write((1 << 0) | if id != 0 { (1 << 1) | (1 << 2) } else { 0 });
    // Leave the initialisation mode for the filter
    fcan.FMR.write(fmr & !CAN_FMR_FINIT);
}

struct CanErrors {
    rx_error: u32,
    tx_error: u32,
}
static CAN_ERRORS: Global<CanErrors> = Global::new(CanErrors { rx_error: 0, tx_error: 0 });

// Report interface status
pub fn canhw_get_status(status: &mut CanbusStatus) {
    let flag = irq_save();
    let esr = can().ESR.read();
    let e = CAN_ERRORS.get();
    let (rx_error, tx_error) = (e.rx_error, e.tx_error);
    irq_restore(flag);

    status.rx_error = rx_error;
    status.tx_error = tx_error;
    status.bus_state = if esr & CAN_ESR_BOFF != 0 {
        CANBUS_STATE_OFF
    } else if esr & CAN_ESR_EPVF != 0 {
        CANBUS_STATE_PASSIVE
    } else if esr & CAN_ESR_EWGF != 0 {
        CANBUS_STATE_WARN
    } else {
        0
    };
}

// This function handles CAN global interrupts
#[unsafe(no_mangle)]
pub unsafe extern "C" fn CAN_IRQHandler() {
    let c = can();
    if c.RF0R.read() & CAN_RF0R_FMP0 != 0 {
        // Read and ack data packet
        let mb = &c.sFIFOMailBox[0];
        let rir = mb.RIR.read();
        let mut msg = CanbusMsg { id: 0, dlc: 0, data: [0; 8] };
        msg.id = if rir & CAN_RI0R_IDE != 0 {
            ((rir >> CAN_RI0R_EXID_Pos) & 0x1fffffff) | CANMSG_ID_EFF
        } else {
            (rir >> CAN_RI0R_STID_Pos) & 0x7ff
        };
        msg.id |= if rir & CAN_RI0R_RTR != 0 { CANMSG_ID_RTR } else { 0 };
        msg.dlc = mb.RDTR.read() & CAN_RDT0R_DLC;
        msg.set_data32(0, mb.RDLR.read());
        msg.set_data32(1, mb.RDHR.read());
        c.RF0R.write(CAN_RF0R_RFOM0);

        // Process packet
        canbus_process_data(&msg);
    }

    // Check for transmit ready
    let ier = c.IER.read();
    if ier & CAN_IER_TMEIE != 0 && c.TSR.read() & (CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2) != 0 {
        // Tx
        c.IER.write(ier & !CAN_IER_TMEIE);
        canbus_notify_tx();
    }

    // Check for error irq
    let msr = c.MSR.read();
    if msr & CAN_MSR_ERRI != 0 {
        let esr = c.ESR.read();
        let lec = (esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
        if lec != 0 && lec != 7 {
            c.ESR.write(7 << CAN_ESR_LEC_Pos);
            let e = CAN_ERRORS.get();
            if (3..=5).contains(&lec) {
                e.tx_error += 1;
            } else {
                e.rx_error += 1;
            }
        }
        c.MSR.write(CAN_MSR_ERRI);
    }
}

fn make_btr(
    sjw: u32,       // Sync jump width, ... hmm
    time_seg1: u32, // time segment before sample point, 1 .. 16
    time_seg2: u32, // time segment after sample point, 1 .. 8
    brp: u32,       // Baud rate prescaler, 1 .. 1024
) -> u32 {
    (sjw.wrapping_sub(1)) << CAN_BTR_SJW_Pos
        | (time_seg1.wrapping_sub(1)) << CAN_BTR_TS1_Pos
        | (time_seg2.wrapping_sub(1)) << CAN_BTR_TS2_Pos
        | (brp.wrapping_sub(1)) << CAN_BTR_BRP_Pos
}

fn compute_btr(pclock: u32, bitrate: u32) -> u32 {
    // Some equations:
    // Tpclock = 1 / pclock
    // Tq      = brp * Tpclock
    // Tbs1    = Tq * TS1
    // Tbs2    = Tq * TS2
    // NominalBitTime = Tq + Tbs1 + Tbs2
    // BaudRate = 1/NominalBitTime
    //
    // Bit value sample point is after Tq+Tbs1. Ideal sample point
    // is at 87.5% of NominalBitTime
    //
    // Use the lowest brp where ts1 and ts2 are in valid range

    let bit_clocks = pclock / bitrate; // clock ticks per bit

    let sjw = 2;
    let mut qs = 18;
    // Find number of time quantas that gives us the exact wanted bit time
    while qs > 9 {
        // check that bit_clocks / quantas is an integer
        let brp_rem = bit_clocks % qs;
        if brp_rem == 0 {
            break;
        }
        qs -= 1;
    }
    let brp = bit_clocks / qs;
    let time_seg2 = qs / 8; // sample at ~87.5%
    let time_seg1 = qs - (1 + time_seg2);

    make_btr(sjw, time_seg1, time_seg2, brp)
}

pub fn can_init() -> R {
    let c = can();
    // Enable clock
    enable_pclock(SOC_CAN as u32);
    if FILTER_CAN != SOC_CAN {
        // Also enable CAN1 clock if using CAN2 with filter on CAN1
        enable_pclock(FILTER_CAN as u32);
    }

    gpio_peripheral(GPIO_Rx, CAN_FUNCTION, 1);
    gpio_peripheral(GPIO_Tx, CAN_FUNCTION, 0);

    let pclock = get_pclock_frequency(SOC_CAN as u32);

    let btr = compute_btr(pclock, CONFIG_CANBUS_FREQUENCY);

    // ##-1- Configure the CAN #######################################

    // Request initialisation
    c.MCR.write(CAN_MCR_INRQ);
    // Wait the acknowledge
    while c.MSR.read() & CAN_MSR_INAK == 0 {}

    c.BTR.write(btr);

    // TXFP makes packets posted to the TX mboxes transmit in chronologcal order
    // ABOM makes the hardware automatically leave bus-off state
    c.MCR.write(CAN_MCR_TXFP | CAN_MCR_ABOM);
    // Wait the acknowledge
    while c.MSR.read() & CAN_MSR_INAK != 0 {}

    // ##-2- Configure the CAN Filter #######################################
    canhw_set_filter(0);

    // ##-3- Configure Interrupts #################################
    armcm_enable_irq!(CAN_IRQHandler, CAN_RX0_IRQn, 0);
    if CAN_RX0_IRQn != CAN_RX1_IRQn {
        armcm_enable_irq!(CAN_IRQHandler, CAN_RX1_IRQn, 0);
    }
    if CAN_RX0_IRQn != CAN_TX_IRQn {
        armcm_enable_irq!(CAN_IRQHandler, CAN_TX_IRQn, 0);
    }
    if CAN_RX0_IRQn != CAN_SCE_IRQn {
        armcm_enable_irq!(CAN_IRQHandler, CAN_SCE_IRQn, 0);
    }
    c.IER.write(CAN_IER_FMPIE0 | CAN_IER_ERRIE | CAN_IER_LECIE);
    Ok(())
}
decl_init!(can_init);
