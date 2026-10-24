// CANbus support on atsame70 chips
//
// Copyright (C) 2021-2025  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::board::{irq_restore, irq_save};
use crate::config::{CONFIG_CANBUS_FILTER, CONFIG_CANBUS_FREQUENCY, CONFIG_USBCANBUS};
use crate::ctr::Global;
use crate::generic::canbus::{
    canbus_notify_tx, canbus_process_data, CanbusMsg, CanbusStatus, CANBUS_STATE_OFF, CANBUS_STATE_PASSIVE, CANBUS_STATE_WARN,
    CANMSG_DATA_LEN, CANMSG_ID_EFF, CANMSG_ID_RTR,
};
use crate::generic::canserial::CANBUS_ID_ADMIN;
use crate::generic::cmsis::__DMB;
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};
use core::sync::atomic::{compiler_fence, Ordering};

/****************************************************************
 * Pin configuration
 ****************************************************************/

#[cfg(CONFIG_ATSAM_CANBUS_PB3_PB2)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PB3,PB2");
    pub const GPIO_Rx: u32 = GPIO('B', 3);
    pub const GPIO_Tx: u32 = GPIO('B', 2);
    pub const CANx_GCLK_ID: u32 = MCAN0_CLOCK_ID;
}
#[cfg(CONFIG_ATSAM_CANBUS_PC12_PD12)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_CAN", "PC12,PD12");
    pub const GPIO_Rx: u32 = GPIO('C', 12);
    pub const GPIO_Tx: u32 = GPIO('D', 12);
    pub const CANx_GCLK_ID: u32 = MCAN1_CLOCK_ID;
}
use pins::*;

// (C: #if CANx_GCLK_ID == MCAN0_CLOCK_ID)
#[cfg(CONFIG_ATSAM_CANBUS_PB3_PB2)]
mod canx {
    use super::*;
    pub const CAN_FUNCTION_Rx: char = 'A';
    pub const CAN_FUNCTION_Tx: char = 'A';
    pub const CANx: *mut Mcan = MCAN0;
    pub const CANx_IRQn: IRQn_Type = MCAN0_INT0_IRQn;
    pub fn CCFG_CANxDMABA() -> &'static Reg<u32> {
        unsafe { &(*MATRIX).CCFG_CAN0 }
    }
    pub const CCFG_CANxDMABA_Msk: u32 = CCFG_CAN0_CAN0DMABA_Msk;
}
#[cfg(not(CONFIG_ATSAM_CANBUS_PB3_PB2))]
mod canx {
    use super::*;
    pub const CAN_FUNCTION_Rx: char = 'C';
    pub const CAN_FUNCTION_Tx: char = 'B';
    pub const CANx: *mut Mcan = MCAN1;
    pub const CANx_IRQn: IRQn_Type = MCAN1_INT0_IRQn;
    pub fn CCFG_CANxDMABA() -> &'static Reg<u32> {
        unsafe { &(*MATRIX).CCFG_SYSIO }
    }
    pub const CCFG_CANxDMABA_Msk: u32 = CCFG_SYSIO_CAN1DMABA_Msk;
}
use canx::*;

fn can() -> &'static Mcan {
    unsafe { &*CANx }
}

/****************************************************************
 * Message ram layout
 ****************************************************************/

#[repr(C)]
#[derive(Clone, Copy)]
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

const FIFO0: FdcanFifo = FdcanFifo { id_section: 0, dlc_section: 0, data: [0; 16] };

// Message ram is in DTCM - locate it there to avoid cache.
#[unsafe(link_section = ".dtcm.bss")]
static MSG_RAM: Global<FdcanMsgRam> =
    Global::new(FdcanMsgRam { FLS: [0; 28], FLE: [0; 16], RXF0: [FIFO0; 3], RXF1: [FIFO0; 3], TEF: [0; 6], TXFIFO: [FIFO0; 3] });

fn msg_ram() -> *mut FdcanMsgRam {
    MSG_RAM.ptr()
}

/****************************************************************
 * CANbus code
 ****************************************************************/

const FDCAN_IE_TC: u32 = MCAN_IE_TCE | MCAN_IE_TCFE | MCAN_IE_TFEE;

// Transmit a packet
pub fn canhw_send(msg: &CanbusMsg) -> i32 {
    let txfqs = can().MCAN_TXFQS.read();
    if txfqs & MCAN_TXFQS_TFQF != 0 {
        // No space in transmit fifo - wait for irq
        return -1;
    }

    let w_index = (txfqs & MCAN_TXFQS_TFQPI_Msk) >> MCAN_TXFQS_TFQPI_Pos;
    let txfifo = unsafe { core::ptr::addr_of_mut!((*msg_ram()).TXFIFO[w_index as usize]) };
    let mut ids = if msg.id & CANMSG_ID_EFF != 0 { (msg.id & 0x1fffffff) | FDCAN_XTD } else { (msg.id & 0x7ff) << 18 };
    ids |= if msg.id & CANMSG_ID_RTR != 0 { FDCAN_RTR } else { 0 };
    unsafe {
        core::ptr::addr_of_mut!((*txfifo).id_section).write_volatile(ids);
        core::ptr::addr_of_mut!((*txfifo).dlc_section).write_volatile((msg.dlc & 0x0f) << 16);
        core::ptr::addr_of_mut!((*txfifo).data[0]).write_volatile(msg.data32(0));
        core::ptr::addr_of_mut!((*txfifo).data[1]).write_volatile(msg.data32(1));
    }
    __DMB();
    can().MCAN_TXBAR.read();
    can().MCAN_TXBAR.write(1u32 << w_index);
    CANMSG_DATA_LEN(msg) as i32
}

fn can_filter(index: usize, id: u32) {
    unsafe {
        core::ptr::addr_of_mut!((*msg_ram()).FLS[index]).write_volatile(
            (0x2 << 30) // Classic filter
                | (0x1 << 27) // Store in Rx FIFO 0 if filter matches
                | (id << 16)
                | 0x7FF, // mask all enabled
        )
    };
}

// Setup the receive packet filter
pub fn canhw_set_filter(id: u32) {
    if !CONFIG_CANBUS_FILTER {
        return;
    }
    let c = can();
    /* Request initialisation */
    c.MCAN_CCCR.or(MCAN_CCCR_INIT);
    /* Wait the acknowledge */
    while c.MCAN_CCCR.read() & MCAN_CCCR_INIT == 0 {}
    /* Enable configuration change */
    c.MCAN_CCCR.or(MCAN_CCCR_CCE);

    // Load filter
    can_filter(0, CANBUS_ID_ADMIN);
    can_filter(1, id);
    can_filter(2, id + 1);

    let flssa = (unsafe { core::ptr::addr_of!((*msg_ram()).FLS) } as u32)
        .wrapping_sub(CCFG_CANxDMABA_Msk & CCFG_CANxDMABA().read());
    c.MCAN_SIDFC.write(flssa | ((if id != 0 { 3 } else { 1 }) << MCAN_SIDFC_LSS_Pos));
    c.MCAN_GFC.write(0x02 << MCAN_GFC_ANFS_Pos);

    /* Leave the initialisation mode for the filter */
    compiler_fence(Ordering::SeqCst);
    c.MCAN_CCCR.and(!MCAN_CCCR_CCE);
    c.MCAN_CCCR.and(!MCAN_CCCR_INIT);
}

struct CanErrors {
    rx_error: u32,
    tx_error: u32,
}
static CAN_Errors: Global<CanErrors> = Global::new(CanErrors { rx_error: 0, tx_error: 0 });

// Report interface status
pub fn canhw_get_status(status: &mut CanbusStatus) {
    let flag = irq_save();
    let psr = can().MCAN_PSR.read();
    let lec = psr & MCAN_PSR_LEC_Msk;
    let e = CAN_Errors.get();
    if lec != 0 && lec != 7 {
        // Reading PSR clears it - so update state here
        if (3..=5).contains(&lec) {
            e.tx_error = e.tx_error.wrapping_add(1);
        } else {
            e.rx_error = e.rx_error.wrapping_add(1);
        }
    }
    let (rx_error, tx_error) = (e.rx_error, e.tx_error);
    irq_restore(flag);

    status.rx_error = rx_error;
    status.tx_error = tx_error;
    status.bus_state = if psr & MCAN_PSR_BO != 0 {
        CANBUS_STATE_OFF
    } else if psr & MCAN_PSR_EP != 0 {
        CANBUS_STATE_PASSIVE
    } else if psr & MCAN_PSR_EW != 0 {
        CANBUS_STATE_WARN
    } else {
        0
    };
}

// This function handles CAN global interrupts
#[unsafe(no_mangle)]
pub unsafe extern "C" fn CAN_IRQHandler() {
    let c = can();
    let ir = c.MCAN_IR.read();

    if ir & MCAN_IE_RF0NE != 0 {
        c.MCAN_IR.write(MCAN_IE_RF0NE);

        let rxf0s = c.MCAN_RXF0S.read();
        if rxf0s & MCAN_RXF0S_F0FL_Msk != 0 {
            // Read and ack data packet
            let idx = (rxf0s & MCAN_RXF0S_F0GI_Msk) >> MCAN_RXF0S_F0GI_Pos;
            let rxf0 = core::ptr::addr_of!((*msg_ram()).RXF0[idx as usize]);
            let ids = core::ptr::addr_of!((*rxf0).id_section).read_volatile();
            let mut msg = CanbusMsg { id: 0, dlc: 0, data: [0; 8] };
            if ids & FDCAN_XTD != 0 {
                msg.id = (ids & 0x1fffffff) | CANMSG_ID_EFF;
            } else {
                msg.id = (ids >> 18) & 0x7ff;
            }
            msg.id |= if ids & FDCAN_RTR != 0 { CANMSG_ID_RTR } else { 0 };
            msg.dlc = (core::ptr::addr_of!((*rxf0).dlc_section).read_volatile() >> 16) & 0x0f;
            msg.set_data32(0, core::ptr::addr_of!((*rxf0).data[0]).read_volatile());
            msg.set_data32(1, core::ptr::addr_of!((*rxf0).data[1]).read_volatile());
            compiler_fence(Ordering::SeqCst);
            c.MCAN_RXF0A.write(idx);

            // Process packet
            canbus_process_data(&msg);
        }
    }
    if ir & FDCAN_IE_TC != 0 {
        // Tx
        c.MCAN_IR.write(FDCAN_IE_TC);
        canbus_notify_tx();
    }
    if ir & (MCAN_IR_PED | MCAN_IR_PEA) != 0 {
        // Bus error
        let psr = c.MCAN_PSR.read();
        c.MCAN_IR.write(MCAN_IR_PED | MCAN_IR_PEA);
        let lec = psr & MCAN_PSR_LEC_Msk;
        if lec != 0 && lec != 7 {
            let e = CAN_Errors.get();
            if (3..=5).contains(&lec) {
                e.tx_error = e.tx_error.wrapping_add(1);
            } else {
                e.rx_error = e.rx_error.wrapping_add(1);
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
    ((sjw - 1) << MCAN_NBTP_NSJW_Pos)
        | ((time_seg1 - 1) << MCAN_NBTP_NTSEG1_Pos)
        | ((time_seg2 - 1) << MCAN_NBTP_NTSEG2_Pos)
        | ((brp - 1) << MCAN_NBTP_NBRP_Pos)
}

fn compute_btr(pclock: u32, bitrate: u32) -> u32 {
    /*
        Some equations:
        Tpclock = 1 / pclock
        Tq      = brp * Tpclock
        Tbs1    = Tq * TS1
        Tbs2    = Tq * TS2
        NominalBitTime = Tq + Tbs1 + Tbs2
        BaudRate = 1/NominalBitTime
        Bit value sample point is after Tq+Tbs1. Ideal sample point
        is at 87.5% of NominalBitTime
        Use the lowest brp where ts1 and ts2 are in valid range
     */

    let bit_clocks = pclock / bitrate; // clock ticks per bit

    let sjw = 2;
    // Find number of time quantas that gives us the exact wanted bit time
    let mut qs = 18;
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
    let pmc = unsafe { &*PMC };
    if !CONFIG_USBCANBUS {
        // Configure UPLL for USB and CAN
        pmc.CKGR_UCKR.write(CKGR_UCKR_UPLLCOUNT(3) | CKGR_UCKR_UPLLEN);
        while pmc.PMC_SR.read() & PMC_SR_LOCKU == 0 {}
    }

    // Configure PCK5 for CAN use
    pmc.PMC_PCK[5].write(PMC_PCK_CSS_UPLL_CLK | PMC_PCK_PRES(5));
    while pmc.PMC_SR.read() & PMC_SR_PCKRDY5 == 0 {}
    pmc.PMC_SCER.or(PMC_SCER_PCK5);

    enable_pclock(CANx_GCLK_ID);

    gpio_peripheral(GPIO_Rx, CAN_FUNCTION_Rx, 1);
    gpio_peripheral(GPIO_Tx, CAN_FUNCTION_Tx, 0);

    let pclock = get_pclock_frequency(CANx_GCLK_ID);

    let btr = compute_btr(pclock, CONFIG_CANBUS_FREQUENCY);

    /*##-1- Configure the CAN #######################################*/

    let c = can();
    /* Exit from sleep mode */
    c.MCAN_CCCR.and(!MCAN_CCCR_CSR);
    /* Wait the acknowledge */
    while c.MCAN_CCCR.read() & MCAN_CCCR_CSA != 0 {}
    /* Request initialization */
    c.MCAN_CCCR.or(MCAN_CCCR_INIT);
    /* Wait the acknowledge */
    while c.MCAN_CCCR.read() & MCAN_CCCR_INIT == 0 {}
    /* Enable configuration change */
    c.MCAN_CCCR.or(MCAN_CCCR_CCE);

    /* Disable protocol exception handling */
    c.MCAN_CCCR.or(MCAN_CCCR_PXHD);

    c.MCAN_NBTP.write(btr);

    /* Setup message RAM addresses */
    let ram = msg_ram();
    let ccfg = CCFG_CANxDMABA().read() & !CCFG_CANxDMABA_Msk;
    CCFG_CANxDMABA().write(ccfg | ((ram as u32) & CCFG_CANxDMABA_Msk));
    let base = CCFG_CANxDMABA_Msk & CCFG_CANxDMABA().read();
    let f0sa = (unsafe { core::ptr::addr_of!((*ram).RXF0) } as u32).wrapping_sub(base);
    c.MCAN_RXF0C.write(f0sa | (3 << MCAN_RXF0C_F0S_Pos));
    c.MCAN_RXESC.write((7 << MCAN_RXESC_F1DS_Pos) | (7 << MCAN_RXESC_F0DS_Pos));
    let base = CCFG_CANxDMABA_Msk & CCFG_CANxDMABA().read();
    let tbsa = (unsafe { core::ptr::addr_of!((*ram).TXFIFO) } as u32).wrapping_sub(base);
    c.MCAN_TXBC.write(tbsa | (3 << MCAN_TXBC_TFQS_Pos));
    c.MCAN_TXESC.write(7 << MCAN_TXESC_TBDS_Pos);

    /* Leave the initialisation mode */
    c.MCAN_CCCR.and(!MCAN_CCCR_CCE);
    c.MCAN_CCCR.and(!MCAN_CCCR_INIT);

    /*##-2- Configure the CAN Filter #######################################*/
    canhw_set_filter(0);

    /*##-3- Configure Interrupts #################################*/
    armcm_enable_irq!(CAN_IRQHandler, CANx_IRQn, 1);
    c.MCAN_ILE.write(MCAN_ILE_EINT0);
    c.MCAN_IE.write(MCAN_IE_RF0NE | FDCAN_IE_TC | MCAN_IE_PEDE | MCAN_IE_PEAE);
    Ok(())
}
decl_init!(can_init);
