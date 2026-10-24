// FDCAN support on stm32 chips
//
// Copyright (C) 2021-2022  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <liquidpontus@yahoo.se>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals, non_snake_case)]
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::config::{CONFIG_CANBUS_FILTER, CONFIG_CANBUS_FREQUENCY};
use crate::ctr::Global;
use crate::generic::canbus::{
    canbus_notify_tx, canbus_process_data, CanbusMsg, CanbusStatus, CANBUS_STATE_OFF, CANBUS_STATE_PASSIVE, CANBUS_STATE_WARN,
    CANMSG_DATA_LEN, CANMSG_ID_EFF, CANMSG_ID_RTR,
};
use crate::generic::canserial::CANBUS_ID_ADMIN;
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};
use core::ptr::{addr_of_mut, read_volatile, write_volatile};

/****************************************************************
 * Pin configuration
 ****************************************************************/

#[cfg(CONFIG_STM32_CANBUS_PA11_PA12)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PA11,PA12");
    pub const GPIO_Rx: u32 = GPIO('A', 11);
    pub const GPIO_Tx: u32 = GPIO('A', 12);
}
#[cfg(CONFIG_STM32_CANBUS_PA11_PB9)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PA11,PB9");
    pub const GPIO_Rx: u32 = GPIO('A', 11);
    pub const GPIO_Tx: u32 = GPIO('B', 9);
}
#[cfg(CONFIG_STM32_CANBUS_PB8_PB9)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PB8,PB9");
    pub const GPIO_Rx: u32 = GPIO('B', 8);
    pub const GPIO_Tx: u32 = GPIO('B', 9);
}
#[cfg(CONFIG_STM32_CANBUS_PD0_PD1)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PD0,PD1");
    pub const GPIO_Rx: u32 = GPIO('D', 0);
    pub const GPIO_Tx: u32 = GPIO('D', 1);
}
#[cfg(CONFIG_STM32_CANBUS_PD12_PD13)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PD12,PD13");
    pub const GPIO_Rx: u32 = GPIO('D', 12);
    pub const GPIO_Tx: u32 = GPIO('D', 13);
}
#[cfg(CONFIG_STM32_CANBUS_PB0_PB1)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PB0,PB1");
    pub const GPIO_Rx: u32 = GPIO('B', 0);
    pub const GPIO_Tx: u32 = GPIO('B', 1);
}
#[cfg(CONFIG_STM32_CANBUS_PC2_PC3)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PC2,PC3");
    pub const GPIO_Rx: u32 = GPIO('C', 2);
    pub const GPIO_Tx: u32 = GPIO('C', 3);
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
#[cfg(CONFIG_STM32_CANBUS_PH13_PH14)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PH14,PH13");
    pub const GPIO_Rx: u32 = GPIO('H', 14);
    pub const GPIO_Tx: u32 = GPIO('H', 13);
}
use pins::*;

#[cfg(not(any(
    CONFIG_STM32_CANBUS_PB0_PB1,
    CONFIG_STM32_CANBUS_PC2_PC3,
    CONFIG_STM32_CANBUS_PB5_PB6,
    CONFIG_STM32_CANBUS_PB12_PB13
)))]
mod soc {
    use super::*;
    pub const SOC_CAN: *mut FDCAN_GlobalTypeDef = FDCAN1;
    pub const MSG_RAM_INDEX: usize = 0;
    #[cfg(any(CONFIG_MACH_STM32H7, CONFIG_MACH_STM32G4))]
    pub const CAN_IT0_IRQn: IRQn_Type = FDCAN1_IT0_IRQn;
}
#[cfg(any(CONFIG_STM32_CANBUS_PB0_PB1, CONFIG_STM32_CANBUS_PC2_PC3, CONFIG_STM32_CANBUS_PB5_PB6, CONFIG_STM32_CANBUS_PB12_PB13))]
mod soc {
    use super::*;
    pub const SOC_CAN: *mut FDCAN_GlobalTypeDef = FDCAN2;
    pub const MSG_RAM_INDEX: usize = 1;
    #[cfg(any(CONFIG_MACH_STM32H7, CONFIG_MACH_STM32G4))]
    pub const CAN_IT0_IRQn: IRQn_Type = FDCAN2_IT0_IRQn;
}
use soc::*;

#[cfg(CONFIG_MACH_STM32G0)]
const CAN_IT0_IRQn: IRQn_Type = TIM16_FDCAN_IT0_IRQn;
#[cfg(CONFIG_MACH_STM32G0)]
const CAN_FUNCTION: u32 = GPIO_FUNCTION(3); // Alternative function mapping number

#[cfg(any(CONFIG_MACH_STM32H7, CONFIG_MACH_STM32G4))]
const CAN_FUNCTION: u32 = GPIO_FUNCTION(9); // Alternative function mapping number

/****************************************************************
 * Message ram layout
 ****************************************************************/

#[repr(C)]
struct FdcanFifo {
    id_section: u32,
    dlc_section: u32,
    data: [u32; 64 / 4],
}

const FDCAN_XTD: u32 = 1 << 30;
const FDCAN_RTR: u32 = 1 << 29;

#[repr(C)]
struct FdcanMsgRam {
    FLS: [u32; 28], // Filter list standard
    FLE: [u32; 16], // Filter list extended
    RXF0: [FdcanFifo; 3],
    RXF1: [FdcanFifo; 3],
    TEF: [u32; 6], // Tx event FIFO
    TXFIFO: [FdcanFifo; 3],
}

// (struct fdcan_ram_layout { fdcan1, fdcan2 } at SRAMCAN_BASE)
fn msg_ram() -> *mut FdcanMsgRam {
    (SRAMCAN_BASE as *mut FdcanMsgRam).wrapping_add(MSG_RAM_INDEX)
}

fn can() -> &'static FDCAN_GlobalTypeDef {
    unsafe { &*SOC_CAN }
}

/****************************************************************
 * CANbus code
 ****************************************************************/

const FDCAN_IE_TC: u32 = FDCAN_IE_TCE | FDCAN_IE_TCFE | FDCAN_IE_TFEE;

// Transmit a packet
pub fn canhw_send(msg: &CanbusMsg) -> i32 {
    let c = can();
    let txfqs = c.TXFQS.read();
    if txfqs & FDCAN_TXFQS_TFQF != 0 {
        // No space in transmit fifo - wait for irq
        return -1;
    }

    let w_index = (txfqs & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    unsafe {
        let txfifo = addr_of_mut!((*msg_ram()).TXFIFO[w_index as usize]);
        let mut ids = if msg.id & CANMSG_ID_EFF != 0 { (msg.id & 0x1fffffff) | FDCAN_XTD } else { (msg.id & 0x7ff) << 18 };
        ids |= if msg.id & CANMSG_ID_RTR != 0 { FDCAN_RTR } else { 0 };
        write_volatile(addr_of_mut!((*txfifo).id_section), ids);
        write_volatile(addr_of_mut!((*txfifo).dlc_section), (msg.dlc & 0x0f) << 16);
        write_volatile(addr_of_mut!((*txfifo).data[0]), msg.data32(0));
        write_volatile(addr_of_mut!((*txfifo).data[1]), msg.data32(1));
    }
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    c.TXBAR.write(1 << w_index);
    CANMSG_DATA_LEN(msg) as i32
}

fn can_filter(index: usize, id: u32) {
    let v = (0x2 << 30) // Classic filter
        | (0x1 << 27) // Store in Rx FIFO 0 if filter matches
        | (id << 16)
        | 0x7FF; // mask all enabled
    unsafe { write_volatile(addr_of_mut!((*msg_ram()).FLS[index]), v) };
}

// Setup the receive packet filter
pub fn canhw_set_filter(id: u32) {
    if !CONFIG_CANBUS_FILTER {
        return;
    }
    let c = can();
    // Request initialisation
    c.CCCR.or(FDCAN_CCCR_INIT);
    // Wait the acknowledge
    while c.CCCR.read() & FDCAN_CCCR_INIT == 0 {}
    // Enable configuration change
    c.CCCR.or(FDCAN_CCCR_CCE);

    // Load filter
    can_filter(0, CANBUS_ID_ADMIN);
    can_filter(1, id);
    can_filter(2, id + 1);

    #[cfg(any(CONFIG_MACH_STM32G0, CONFIG_MACH_STM32G4))]
    c.RXGFC.write(((if id != 0 { 3 } else { 1 }) << FDCAN_RXGFC_LSS_Pos) | (0x02 << FDCAN_RXGFC_ANFS_Pos));
    #[cfg(CONFIG_MACH_STM32H7)]
    {
        let flssa = unsafe { addr_of_mut!((*msg_ram()).FLS) } as u32 - SRAMCAN_BASE;
        c.SIDFC.write(flssa | ((if id != 0 { 3 } else { 1 }) << FDCAN_SIDFC_LSS_Pos));
        c.GFC.write(0x02 << FDCAN_GFC_ANFS_Pos);
    }

    // Leave the initialisation mode for the filter
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    c.CCCR.and(!FDCAN_CCCR_CCE);
    c.CCCR.and(!FDCAN_CCCR_INIT);
}

struct CanErrors {
    rx_error: u32,
    tx_error: u32,
}
static CAN_ERRORS: Global<CanErrors> = Global::new(CanErrors { rx_error: 0, tx_error: 0 });

// Report interface status
pub fn canhw_get_status(status: &mut CanbusStatus) {
    let flag = irq_save();
    let psr = can().PSR.read();
    let lec = psr & FDCAN_PSR_LEC_Msk;
    let e = CAN_ERRORS.get();
    if lec != 0 && lec != 7 {
        // Reading PSR clears it - so update state here
        if (3..=5).contains(&lec) {
            e.tx_error += 1;
        } else {
            e.rx_error += 1;
        }
    }
    let (rx_error, tx_error) = (e.rx_error, e.tx_error);
    irq_restore(flag);

    status.rx_error = rx_error;
    status.tx_error = tx_error;
    status.bus_state = if psr & FDCAN_PSR_BO != 0 {
        CANBUS_STATE_OFF
    } else if psr & FDCAN_PSR_EP != 0 {
        CANBUS_STATE_PASSIVE
    } else if psr & FDCAN_PSR_EW != 0 {
        CANBUS_STATE_WARN
    } else {
        0
    };
}

// This function handles CAN global interrupts
#[unsafe(no_mangle)]
pub unsafe extern "C" fn CAN_IRQHandler() {
    let c = can();
    let ir = c.IR.read();

    if ir & FDCAN_IE_RF0NE != 0 {
        c.IR.write(FDCAN_IE_RF0NE);

        let rxf0s = c.RXF0S.read();
        if rxf0s & FDCAN_RXF0S_F0FL != 0 {
            // Read and ack data packet
            let idx = (rxf0s & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
            let rxf0 = addr_of_mut!((*msg_ram()).RXF0[idx as usize]);
            let ids = read_volatile(addr_of_mut!((*rxf0).id_section));
            let mut msg = CanbusMsg { id: 0, dlc: 0, data: [0; 8] };
            msg.id = if ids & FDCAN_XTD != 0 { (ids & 0x1fffffff) | CANMSG_ID_EFF } else { (ids >> 18) & 0x7ff };
            msg.id |= if ids & FDCAN_RTR != 0 { CANMSG_ID_RTR } else { 0 };
            msg.dlc = (read_volatile(addr_of_mut!((*rxf0).dlc_section)) >> 16) & 0x0f;
            msg.set_data32(0, read_volatile(addr_of_mut!((*rxf0).data[0])));
            msg.set_data32(1, read_volatile(addr_of_mut!((*rxf0).data[1])));
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
            c.RXF0A.write(idx);

            // Process packet
            canbus_process_data(&msg);
        }
    }
    if ir & FDCAN_IE_TC != 0 {
        // Tx
        c.IR.write(FDCAN_IE_TC);
        canbus_notify_tx();
    }
    if ir & (FDCAN_IR_PED | FDCAN_IR_PEA) != 0 {
        // Bus error
        let psr = c.PSR.read();
        c.IR.write(FDCAN_IR_PED | FDCAN_IR_PEA);
        let lec = psr & FDCAN_PSR_LEC_Msk;
        if lec != 0 && lec != 7 {
            let e = CAN_ERRORS.get();
            if (3..=5).contains(&lec) {
                e.tx_error += 1;
            } else {
                e.rx_error += 1;
            }
        }
    }
}

fn make_btr(
    sjw: u32,       // Sync jump width, ... hmm
    time_seg1: u32, // time segment before sample point, 1 .. 16
    time_seg2: u32, // time segment after sample point, 1 .. 8
    brp: u32,       // Baud rate prescaler, 1 .. 1024
) -> u32 {
    (sjw.wrapping_sub(1)) << FDCAN_NBTP_NSJW_Pos
        | (time_seg1.wrapping_sub(1)) << FDCAN_NBTP_NTSEG1_Pos
        | (time_seg2.wrapping_sub(1)) << FDCAN_NBTP_NTSEG2_Pos
        | (brp.wrapping_sub(1)) << FDCAN_NBTP_NBRP_Pos
}

fn compute_btr(pclock: u32, bitrate: u32) -> u32 {
    // Some equations:
    // Tpclock = 1 / pclock
    // Tq      = brp * Tpclock
    // Tbs1    = Tq * TS1
    // Tbs2    = Tq * TS2
    // NominalBitTime = Tq + Tbs1 + Tbs2
    // BaudRate = 1/NominalBitTime
    // Bit value sample point is after Tq+Tbs1. Ideal sample point
    // is at 87.5% of NominalBitTime
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
    enable_pclock(SOC_CAN as u32);

    gpio_peripheral(GPIO_Rx, CAN_FUNCTION, 1);
    gpio_peripheral(GPIO_Tx, CAN_FUNCTION, 0);

    let pclock = get_pclock_frequency(SOC_CAN as u32);

    let btr = compute_btr(pclock, CONFIG_CANBUS_FREQUENCY);

    // ##-1- Configure the CAN #######################################

    // Exit from sleep mode
    c.CCCR.and(!FDCAN_CCCR_CSR);
    // Wait the acknowledge
    while c.CCCR.read() & FDCAN_CCCR_CSA != 0 {}
    // Request initialisation
    c.CCCR.or(FDCAN_CCCR_INIT);
    // Wait the acknowledge
    while c.CCCR.read() & FDCAN_CCCR_INIT == 0 {}
    // Enable configuration change
    c.CCCR.or(FDCAN_CCCR_CCE);

    // Disable protocol exception handling
    c.CCCR.or(FDCAN_CCCR_PXHD);

    c.NBTP.write(btr);

    #[cfg(CONFIG_MACH_STM32H7)]
    {
        // Setup message RAM addresses
        let ram = msg_ram();
        let f0sa = unsafe { addr_of_mut!((*ram).RXF0) } as u32 - SRAMCAN_BASE;
        c.RXF0C.write(f0sa | (3 << FDCAN_RXF0C_F0S_Pos));
        c.RXESC.write((7 << FDCAN_RXESC_F1DS_Pos) | (7 << FDCAN_RXESC_F0DS_Pos));
        let tbsa = unsafe { addr_of_mut!((*ram).TXFIFO) } as u32 - SRAMCAN_BASE;
        c.TXBC.write(tbsa | (3 << FDCAN_TXBC_TFQS_Pos));
        c.TXESC.write(7 << FDCAN_TXESC_TBDS_Pos);
    }

    // Leave the initialisation mode
    c.CCCR.and(!FDCAN_CCCR_CCE);
    c.CCCR.and(!FDCAN_CCCR_INIT);

    // ##-2- Configure the CAN Filter #######################################
    canhw_set_filter(0);

    // ##-3- Configure Interrupts #################################
    armcm_enable_irq!(CAN_IRQHandler, CAN_IT0_IRQn, 1);
    c.ILE.write(FDCAN_ILE_EINT0);
    c.IE.write(FDCAN_IE_RF0NE | FDCAN_IE_TC | FDCAN_IE_PEDE | FDCAN_IE_PEAE);
    Ok(())
}
decl_init!(can_init);
