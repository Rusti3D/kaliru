// Hardware interface to USB on SAM3X
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::ctr::Global;
use crate::generic::cmsis::__DMB;
use crate::generic::usb_cdc::{usb_notify_bulk_in, usb_notify_bulk_out, usb_notify_ep0};
use crate::generic::usb_cdc_ep::{
    USB_CDC_EP0_SIZE, USB_CDC_EP_ACM, USB_CDC_EP_ACM_SIZE, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_IN_SIZE, USB_CDC_EP_BULK_OUT,
    USB_CDC_EP_BULK_OUT_SIZE,
};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_init};

// (same70_usb.h: the same70 calls the peripheral USBHS; the registers as
// functions of the device)
#[cfg(not(CONFIG_MACH_SAME70))]
mod hs {
    use super::*;
    pub type Uotg = Uotghs;
    pub const UOTGHS_: *mut Uotg = UOTGHS;
    pub const CFG_UOTGHS_CTRL: u32 = UOTGHS_CTRL_UIMOD | UOTGHS_CTRL_OTGPADE | UOTGHS_CTRL_USBE;
    pub fn CTRL(u: &Uotg) -> &Reg<u32> {
        &u.UOTGHS_CTRL
    }
    pub fn DEVCTRL(u: &Uotg) -> &Reg<u32> {
        &u.UOTGHS_DEVCTRL
    }
    pub fn DEVEPT(u: &Uotg) -> &Reg<u32> {
        &u.UOTGHS_DEVEPT
    }
    pub fn DEVEPTCFG(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.UOTGHS_DEVEPTCFG[i]
    }
    pub fn DEVEPTICR(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.UOTGHS_DEVEPTICR[i]
    }
    pub fn DEVEPTIDR(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.UOTGHS_DEVEPTIDR[i]
    }
    pub fn DEVEPTIER(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.UOTGHS_DEVEPTIER[i]
    }
    pub fn DEVEPTISR(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.UOTGHS_DEVEPTISR[i]
    }
    pub fn DEVICR(u: &Uotg) -> &Reg<u32> {
        &u.UOTGHS_DEVICR
    }
    pub fn DEVIDR(u: &Uotg) -> &Reg<u32> {
        &u.UOTGHS_DEVIDR
    }
    pub fn DEVIER(u: &Uotg) -> &Reg<u32> {
        &u.UOTGHS_DEVIER
    }
    pub fn DEVISR(u: &Uotg) -> &Reg<u32> {
        &u.UOTGHS_DEVISR
    }
}
#[cfg(CONFIG_MACH_SAME70)]
mod hs {
    use super::*;
    // Missing in upstream headers
    pub const USBHS_RAM_ADDR: u32 = 0xa0100000;
    pub type Uotg = Usbhs;
    pub const UOTGHS_: *mut Uotg = USBHS;
    pub const UOTGHS_RAM_ADDR: u32 = USBHS_RAM_ADDR;
    pub use crate::board::chip::{
        ID_USBHS as ID_UOTGHS, USBHS_CTRL_UIMOD as UOTGHS_CTRL_UIMOD, USBHS_CTRL_USBE as UOTGHS_CTRL_USBE,
        USBHS_DEVCTRL_ADDEN as UOTGHS_DEVCTRL_ADDEN, USBHS_DEVCTRL_SPDCONF_FORCED_FS as UOTGHS_DEVCTRL_SPDCONF_FORCED_FS,
        USBHS_DEVEPTCFG_ALLOC as UOTGHS_DEVEPTCFG_ALLOC, USBHS_DEVEPTCFG_EPBK_1_BANK as UOTGHS_DEVEPTCFG_EPBK_1_BANK,
        USBHS_DEVEPTCFG_EPBK_2_BANK as UOTGHS_DEVEPTCFG_EPBK_2_BANK, USBHS_DEVEPTCFG_EPDIR_IN as UOTGHS_DEVEPTCFG_EPDIR_IN,
        USBHS_DEVEPTCFG_EPSIZE_16_BYTE as UOTGHS_DEVEPTCFG_EPSIZE_16_BYTE,
        USBHS_DEVEPTCFG_EPSIZE_32_BYTE as UOTGHS_DEVEPTCFG_EPSIZE_32_BYTE,
        USBHS_DEVEPTCFG_EPSIZE_64_BYTE as UOTGHS_DEVEPTCFG_EPSIZE_64_BYTE,
        USBHS_DEVEPTCFG_EPSIZE_8_BYTE as UOTGHS_DEVEPTCFG_EPSIZE_8_BYTE, USBHS_DEVEPTCFG_EPTYPE_BLK as UOTGHS_DEVEPTCFG_EPTYPE_BLK,
        USBHS_DEVEPTCFG_EPTYPE_CTRL as UOTGHS_DEVEPTCFG_EPTYPE_CTRL,
        USBHS_DEVEPTCFG_EPTYPE_INTRPT as UOTGHS_DEVEPTCFG_EPTYPE_INTRPT, USBHS_DEVEPTICR_RXOUTIC as UOTGHS_DEVEPTICR_RXOUTIC,
        USBHS_DEVEPTICR_RXSTPIC as UOTGHS_DEVEPTICR_RXSTPIC, USBHS_DEVEPTICR_TXINIC as UOTGHS_DEVEPTICR_TXINIC,
        USBHS_DEVEPTIDR_FIFOCONC as UOTGHS_DEVEPTIDR_FIFOCONC, USBHS_DEVEPTIDR_RXOUTEC as UOTGHS_DEVEPTIDR_RXOUTEC,
        USBHS_DEVEPTIDR_TXINEC as UOTGHS_DEVEPTIDR_TXINEC, USBHS_DEVEPTIER_RXOUTES as UOTGHS_DEVEPTIER_RXOUTES,
        USBHS_DEVEPTIER_RXSTPES as UOTGHS_DEVEPTIER_RXSTPES, USBHS_DEVEPTIER_STALLRQS as UOTGHS_DEVEPTIER_STALLRQS,
        USBHS_DEVEPTIER_TXINES as UOTGHS_DEVEPTIER_TXINES, USBHS_DEVEPTISR_BYCT_Msk as UOTGHS_DEVEPTISR_BYCT_Msk,
        USBHS_DEVEPTISR_BYCT_Pos as UOTGHS_DEVEPTISR_BYCT_Pos, USBHS_DEVEPTISR_RXOUTI as UOTGHS_DEVEPTISR_RXOUTI,
        USBHS_DEVEPTISR_RXSTPI as UOTGHS_DEVEPTISR_RXSTPI, USBHS_DEVEPTISR_TXINI as UOTGHS_DEVEPTISR_TXINI,
        USBHS_DEVEPT_EPEN0 as UOTGHS_DEVEPT_EPEN0, USBHS_DEVICR_EORSTC as UOTGHS_DEVICR_EORSTC,
        USBHS_DEVIER_EORSTES as UOTGHS_DEVIER_EORSTES, USBHS_DEVIER_PEP_0 as UOTGHS_DEVIER_PEP_0,
        USBHS_DEVISR_EORST as UOTGHS_DEVISR_EORST, USBHS_DEVISR_PEP_0 as UOTGHS_DEVISR_PEP_0,
    };
    pub const UOTGHS_IRQn: IRQn_Type = USBHS_IRQn;
    pub const CFG_UOTGHS_CTRL: u32 = UOTGHS_CTRL_UIMOD | UOTGHS_CTRL_USBE;
    pub fn CTRL(u: &Uotg) -> &Reg<u32> {
        &u.USBHS_CTRL
    }
    pub fn DEVCTRL(u: &Uotg) -> &Reg<u32> {
        &u.USBHS_DEVCTRL
    }
    pub fn DEVEPT(u: &Uotg) -> &Reg<u32> {
        &u.USBHS_DEVEPT
    }
    pub fn DEVEPTCFG(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.USBHS_DEVEPTCFG[i]
    }
    pub fn DEVEPTICR(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.USBHS_DEVEPTICR[i]
    }
    pub fn DEVEPTIDR(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.USBHS_DEVEPTIDR[i]
    }
    pub fn DEVEPTIER(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.USBHS_DEVEPTIER[i]
    }
    pub fn DEVEPTISR(u: &Uotg, i: usize) -> &Reg<u32> {
        &u.USBHS_DEVEPTISR[i]
    }
    pub fn DEVICR(u: &Uotg) -> &Reg<u32> {
        &u.USBHS_DEVICR
    }
    pub fn DEVIDR(u: &Uotg) -> &Reg<u32> {
        &u.USBHS_DEVIDR
    }
    pub fn DEVIER(u: &Uotg) -> &Reg<u32> {
        &u.USBHS_DEVIER
    }
    pub fn DEVISR(u: &Uotg) -> &Reg<u32> {
        &u.USBHS_DEVISR
    }
}
use hs::*;

fn usb() -> &'static Uotg {
    unsafe { &*UOTGHS_ }
}

const fn EP_SIZE(s: usize) -> u32 {
    if s == 64 {
        UOTGHS_DEVEPTCFG_EPSIZE_64_BYTE
    } else if s == 32 {
        UOTGHS_DEVEPTCFG_EPSIZE_32_BYTE
    } else if s == 16 {
        UOTGHS_DEVEPTCFG_EPSIZE_16_BYTE
    } else {
        UOTGHS_DEVEPTCFG_EPSIZE_8_BYTE
    }
}

fn usb_fifo(ep: usize) -> *mut u8 {
    (UOTGHS_RAM_ADDR as usize + ep * 0x8000) as *mut u8
}

fn usb_write_packet(ep: usize, data: *const u8, len: usize) {
    let dest = usb_fifo(ep);
    for i in 0..len {
        unsafe { dest.add(i).write_volatile(*data.add(i)) };
    }
    __DMB();
}

fn usb_read_packet(ep: usize, data: *mut u8, len: usize) {
    let src = usb_fifo(ep);
    for i in 0..len {
        unsafe { *data.add(i) = src.add(i).read_volatile() };
    }
}

pub fn usb_read_bulk_out(data: *mut u8, max_len: u8) -> i8 {
    let u = usb();
    let eps = DEVEPTISR(u, USB_CDC_EP_BULK_OUT).read();
    if eps & UOTGHS_DEVEPTISR_RXOUTI == 0 {
        // No data ready
        DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << USB_CDC_EP_BULK_OUT);
        return -1;
    }
    let len = (eps & UOTGHS_DEVEPTISR_BYCT_Msk) >> UOTGHS_DEVEPTISR_BYCT_Pos;
    usb_read_packet(USB_CDC_EP_BULK_OUT, data, if len > max_len as u32 { max_len as usize } else { len as usize });
    DEVEPTICR(u, USB_CDC_EP_BULK_OUT).write(UOTGHS_DEVEPTICR_RXOUTIC);
    DEVEPTIDR(u, USB_CDC_EP_BULK_OUT).write(UOTGHS_DEVEPTIDR_FIFOCONC);
    len as i8
}

pub fn usb_send_bulk_in(data: *const u8, len: u8) -> i8 {
    let u = usb();
    let eps = DEVEPTISR(u, USB_CDC_EP_BULK_IN).read();
    if eps & UOTGHS_DEVEPTISR_TXINI == 0 {
        // Buffer full
        DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << USB_CDC_EP_BULK_IN);
        return -1;
    }
    usb_write_packet(USB_CDC_EP_BULK_IN, data, len as usize);
    DEVEPTICR(u, USB_CDC_EP_BULK_IN).write(UOTGHS_DEVEPTICR_TXINIC);
    DEVEPTIDR(u, USB_CDC_EP_BULK_IN).write(UOTGHS_DEVEPTIDR_FIFOCONC);
    len as i8
}

pub fn usb_read_ep0(data: *mut u8, max_len: u8) -> i8 {
    let u = usb();
    let eps = DEVEPTISR(u, 0).read();
    if eps & UOTGHS_DEVEPTISR_RXSTPI != 0 {
        return -2;
    }
    if eps & UOTGHS_DEVEPTISR_RXOUTI == 0 {
        // Not ready to receive data
        DEVEPTIER(u, 0).write(UOTGHS_DEVEPTIER_RXOUTES);
        DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << 0);
        return -1;
    }
    usb_read_packet(0, data, max_len as usize);
    if DEVEPTISR(u, 0).read() & UOTGHS_DEVEPTISR_RXSTPI != 0 {
        return -2;
    }
    DEVEPTICR(u, 0).write(UOTGHS_DEVEPTICR_RXOUTIC);
    max_len as i8
}

pub fn usb_read_ep0_setup(data: *mut u8, max_len: u8) -> i8 {
    let u = usb();
    let eps = DEVEPTISR(u, 0).read();
    if eps & UOTGHS_DEVEPTISR_RXSTPI == 0 {
        // No data ready to read
        DEVEPTIDR(u, 0).write(UOTGHS_DEVEPTIDR_TXINEC | UOTGHS_DEVEPTIDR_RXOUTEC);
        DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << 0);
        return -1;
    }
    usb_read_packet(0, data, max_len as usize);
    DEVEPTICR(u, 0).write(UOTGHS_DEVEPTICR_RXSTPIC | UOTGHS_DEVEPTICR_RXOUTIC);
    max_len as i8
}

pub fn usb_send_ep0(data: *const u8, len: u8) -> i8 {
    let u = usb();
    let eps = DEVEPTISR(u, 0).read();
    if eps & (UOTGHS_DEVEPTISR_RXSTPI | UOTGHS_DEVEPTISR_RXOUTI) != 0 {
        return -2;
    }
    if eps & UOTGHS_DEVEPTISR_TXINI == 0 {
        // Not ready to send
        DEVEPTIER(u, 0).write(UOTGHS_DEVEPTIER_TXINES | UOTGHS_DEVEPTIER_RXOUTES);
        DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << 0);
        return -1;
    }
    usb_write_packet(0, data, len as usize);
    DEVEPTICR(u, 0).write(UOTGHS_DEVEPTICR_TXINIC);
    len as i8
}

pub fn usb_stall_ep0() {
    let u = usb();
    DEVEPTIER(u, 0).write(UOTGHS_DEVEPTIER_STALLRQS);
    DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << 0);
}

static SET_ADDRESS: Global<u8> = Global::new(0);

pub fn usb_set_address(addr: u8) {
    *SET_ADDRESS.get() = addr | UOTGHS_DEVCTRL_ADDEN as u8;
    usb_send_ep0(core::ptr::null(), 0);
    let u = usb();
    DEVEPTIER(u, 0).write(UOTGHS_DEVEPTIER_TXINES);
    DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << 0);
}

pub fn usb_set_configure() {
    let u = usb();
    DEVEPT(u).write(
        (UOTGHS_DEVEPT_EPEN0 << 0)
            | (UOTGHS_DEVEPT_EPEN0 << USB_CDC_EP_BULK_OUT)
            | (UOTGHS_DEVEPT_EPEN0 << USB_CDC_EP_BULK_IN)
            | (UOTGHS_DEVEPT_EPEN0 << USB_CDC_EP_ACM),
    );

    DEVEPTCFG(u, USB_CDC_EP_BULK_OUT).write(
        EP_SIZE(USB_CDC_EP_BULK_OUT_SIZE) | UOTGHS_DEVEPTCFG_EPTYPE_BLK | UOTGHS_DEVEPTCFG_EPBK_2_BANK | UOTGHS_DEVEPTCFG_ALLOC,
    );
    DEVEPTIER(u, USB_CDC_EP_BULK_OUT).write(UOTGHS_DEVEPTIER_RXOUTES);
    DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << USB_CDC_EP_BULK_OUT);

    DEVEPTCFG(u, USB_CDC_EP_BULK_IN).write(
        EP_SIZE(USB_CDC_EP_BULK_IN_SIZE)
            | UOTGHS_DEVEPTCFG_EPDIR_IN
            | UOTGHS_DEVEPTCFG_EPTYPE_BLK
            | UOTGHS_DEVEPTCFG_EPBK_2_BANK
            | UOTGHS_DEVEPTCFG_ALLOC,
    );
    DEVEPTIER(u, USB_CDC_EP_BULK_IN).write(UOTGHS_DEVEPTIER_TXINES);
    DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << USB_CDC_EP_BULK_IN);

    DEVEPTCFG(u, USB_CDC_EP_ACM).write(
        EP_SIZE(USB_CDC_EP_ACM_SIZE)
            | UOTGHS_DEVEPTCFG_EPDIR_IN
            | UOTGHS_DEVEPTCFG_EPTYPE_INTRPT
            | UOTGHS_DEVEPTCFG_EPBK_2_BANK
            | UOTGHS_DEVEPTCFG_ALLOC,
    );
}

// Configure endpoint 0 after usb reset completes
fn handle_end_reset() {
    let u = usb();
    DEVEPT(u).write(UOTGHS_DEVEPT_EPEN0 << 0);
    DEVEPTCFG(u, 0)
        .write(EP_SIZE(USB_CDC_EP0_SIZE) | UOTGHS_DEVEPTCFG_EPTYPE_CTRL | UOTGHS_DEVEPTCFG_EPBK_1_BANK | UOTGHS_DEVEPTCFG_ALLOC);
    DEVEPTIER(u, 0).write(UOTGHS_DEVEPTIER_RXSTPES);
    DEVIER(u).write(UOTGHS_DEVIER_PEP_0 << 0);

    DEVICR(u).write(UOTGHS_DEVICR_EORSTC);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn UOTGHS_Handler() {
    let u = usb();
    let s = DEVISR(u).read();
    if s & UOTGHS_DEVISR_EORST != 0 {
        handle_end_reset();
        return;
    }
    DEVIDR(u).write(s);
    if s & (UOTGHS_DEVISR_PEP_0 << 0) != 0 {
        usb_notify_ep0();

        let set_address = SET_ADDRESS.get();
        if *set_address != 0 && (DEVEPTISR(u, 0).read() & UOTGHS_DEVEPTISR_TXINI) != 0 {
            // Ack from set_address command sent - now update address
            DEVCTRL(u).write(*set_address as u32 | UOTGHS_DEVCTRL_SPDCONF_FORCED_FS);
            *set_address = 0;
        }
    }
    if s & (UOTGHS_DEVISR_PEP_0 << USB_CDC_EP_BULK_OUT) != 0 {
        usb_notify_bulk_out();
    }
    if s & (UOTGHS_DEVISR_PEP_0 << USB_CDC_EP_BULK_IN) != 0 {
        usb_notify_bulk_in();
    }
}

pub fn usbserial_init() -> R {
    // Setup USB clock
    enable_pclock(ID_UOTGHS);
    let pmc = unsafe { &*PMC };
    pmc.CKGR_UCKR.write(CKGR_UCKR_UPLLCOUNT(3) | CKGR_UCKR_UPLLEN);
    while pmc.PMC_SR.read() & PMC_SR_LOCKU == 0 {}

    // Enable USB
    let u = usb();
    CTRL(u).write(CFG_UOTGHS_CTRL);
    DEVCTRL(u).write(UOTGHS_DEVCTRL_SPDCONF_FORCED_FS);

    // Enable interrupts
    armcm_enable_irq!(UOTGHS_Handler, UOTGHS_IRQn, 1);
    DEVIER(u).write(UOTGHS_DEVIER_EORSTES);
    Ok(())
}
decl_init!(usbserial_init);
