// Hardware interface to SAM4 USB Device Port
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::ctr::Global;
use crate::generic::usb_cdc::{usb_notify_bulk_in, usb_notify_bulk_out, usb_notify_ep0};
use crate::generic::usb_cdc_ep::{USB_CDC_EP_ACM, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_OUT};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};

const CSR_EP0: u32 = UDP_CSR_EPTYPE_CTRL | UDP_CSR_EPEDS;
const CSR_ACM: u32 = UDP_CSR_EPTYPE_INT_IN | UDP_CSR_EPEDS;
const CSR_BULK_OUT: u32 = UDP_CSR_EPTYPE_BULK_OUT | UDP_CSR_EPEDS;
const CSR_BULK_IN: u32 = UDP_CSR_EPTYPE_BULK_IN | UDP_CSR_EPEDS;

fn udp() -> &'static Udp {
    unsafe { &*UDP }
}

fn usb_write_packet(ep: usize, data: *const u8, len: usize) {
    for i in 0..len {
        udp().UDP_FDR[ep].write(unsafe { *data.add(i) } as u32);
    }
}

fn usb_read_packet(ep: usize, csr: u32, data: *mut u8, max_len: u32) -> u32 {
    let pk_len = (csr & UDP_CSR_RXBYTECNT_Msk) >> UDP_CSR_RXBYTECNT_Pos;
    let len = if pk_len < max_len { pk_len } else { max_len };
    for i in 0..len as usize {
        unsafe { *data.add(i) = udp().UDP_FDR[ep].read() as u8 };
    }
    len
}

static NEXT_BK: Global<u32> = Global::new(UDP_CSR_RX_DATA_BK0);

pub fn usb_read_bulk_out(data: *mut u8, max_len: u8) -> i8 {
    let other_irqs = UDP_CSR_RXSETUP | UDP_CSR_STALLSENT | UDP_CSR_TXCOMP;
    let csr = udp().UDP_CSR[USB_CDC_EP_BULK_OUT].read();
    let mut bk = csr & (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
    if bk == 0 {
        // Not ready to receive data
        if csr & other_irqs != 0 {
            udp().UDP_CSR[USB_CDC_EP_BULK_OUT].write(CSR_BULK_OUT | UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
        }
        udp().UDP_IER.write(1 << USB_CDC_EP_BULK_OUT);
        return -1;
    }
    let len = usb_read_packet(USB_CDC_EP_BULK_OUT, csr, data, max_len as u32);
    let next_bk = NEXT_BK.get();
    if bk == (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1) {
        bk = *next_bk;
    }
    *next_bk = bk ^ (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
    udp().UDP_CSR[USB_CDC_EP_BULK_OUT].write(CSR_BULK_OUT | *next_bk);
    len as i8
}

pub fn usb_send_bulk_in(data: *const u8, len: u8) -> i8 {
    let other_irqs = UDP_CSR_RXSETUP | UDP_CSR_STALLSENT | UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1;
    let csr = udp().UDP_CSR[USB_CDC_EP_BULK_IN].read();
    if csr & UDP_CSR_TXPKTRDY != 0 {
        // Not ready to send
        if csr & other_irqs != 0 {
            udp().UDP_CSR[USB_CDC_EP_BULK_IN].write(CSR_BULK_IN | UDP_CSR_TXCOMP);
        }
        udp().UDP_IER.write(1 << USB_CDC_EP_BULK_IN);
        return -1;
    }
    usb_write_packet(USB_CDC_EP_BULK_IN, data, len as usize);
    udp().UDP_CSR[USB_CDC_EP_BULK_IN].write(CSR_BULK_IN | UDP_CSR_TXPKTRDY);
    len as i8
}

pub fn usb_read_ep0(data: *mut u8, max_len: u8) -> i8 {
    let other_irqs = UDP_CSR_RXSETUP | UDP_CSR_STALLSENT | UDP_CSR_TXCOMP | UDP_CSR_RX_DATA_BK1;
    let csr = udp().UDP_CSR[0].read();
    if csr & other_irqs != 0 {
        return -2;
    }
    if csr & UDP_CSR_RX_DATA_BK0 == 0 {
        // Not ready to receive data
        udp().UDP_IER.write(1 << 0);
        return -1;
    }
    let len = usb_read_packet(0, csr, data, max_len as u32);
    if udp().UDP_CSR[0].read() & other_irqs != 0 {
        return -2;
    }
    udp().UDP_CSR[0].write(CSR_EP0 | other_irqs);
    len as i8
}

pub fn usb_read_ep0_setup(data: *mut u8, max_len: u8) -> i8 {
    let other_irqs = UDP_CSR_STALLSENT | UDP_CSR_TXCOMP | UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1;
    let csr = udp().UDP_CSR[0].read();
    if csr & UDP_CSR_RXSETUP == 0 {
        // No data ready to read
        if csr & other_irqs != 0 {
            udp().UDP_CSR[0].write(CSR_EP0 | UDP_CSR_RXSETUP);
        }
        udp().UDP_IER.write(1 << 0);
        return -1;
    }
    let len = usb_read_packet(0, csr, data, max_len as u32);
    let dir = if unsafe { *data } & 0x80 != 0 { UDP_CSR_DIR } else { 0 };
    udp().UDP_CSR[0].write(CSR_EP0 | dir);
    len as i8
}

pub fn usb_send_ep0(data: *const u8, len: u8) -> i8 {
    let other_irqs = UDP_CSR_RXSETUP | UDP_CSR_STALLSENT | UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1;
    let csr = udp().UDP_CSR[0].read();
    if csr & other_irqs != 0 {
        return -2;
    }
    if csr & UDP_CSR_TXPKTRDY != 0 {
        // Not ready to send
        udp().UDP_IER.write(1 << 0);
        return -1;
    }
    usb_write_packet(0, data, len as usize);
    let dir = csr & UDP_CSR_DIR;
    udp().UDP_CSR[0].write(CSR_EP0 | dir | UDP_CSR_TXPKTRDY | other_irqs);
    len as i8
}

pub fn usb_stall_ep0() {
    udp().UDP_CSR[0].write(CSR_EP0 | UDP_CSR_FORCESTALL);
    udp().UDP_IER.write(1 << 0);
}

static SET_ADDRESS: Global<u32> = Global::new(0);

pub fn usb_set_address(addr: u8) {
    *SET_ADDRESS.get() = addr as u32 | UDP_FADDR_FEN;
    usb_send_ep0(core::ptr::null(), 0);
    udp().UDP_IER.write(1 << 0);
}

pub fn usb_set_configure() {
    udp().UDP_CSR[USB_CDC_EP_ACM].write(CSR_ACM);
    udp().UDP_CSR[USB_CDC_EP_BULK_OUT].write(CSR_BULK_OUT);
    udp().UDP_CSR[USB_CDC_EP_BULK_IN].write(CSR_BULK_IN);
    udp().UDP_GLB_STAT.or(UDP_GLB_STAT_CONFG);
}

// Configure endpoint 0 after usb reset completes
fn handle_end_reset() {
    udp().UDP_ICR.write(UDP_ISR_ENDBUSRES);

    udp().UDP_CSR[0].write(CSR_EP0);
    udp().UDP_IER.write(1 << 0);

    udp().UDP_TXVC.write(UDP_TXVC_PUON);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn UDP_Handler() {
    let s = udp().UDP_ISR.read();
    udp().UDP_IDR.write(s);
    if s & UDP_ISR_ENDBUSRES != 0 {
        handle_end_reset();
    }
    if s & UDP_ISR_RXRSM != 0 {
        udp().UDP_ICR.write(UDP_ISR_RXRSM);
    }
    if s & (1 << 0) != 0 {
        usb_notify_ep0();

        let set_address = SET_ADDRESS.get();
        if *set_address != 0 && udp().UDP_CSR[0].read() & UDP_CSR_TXCOMP != 0 {
            // Ack from set_address command sent - now update address
            udp().UDP_FADDR.write(*set_address);
            udp().UDP_GLB_STAT.or(UDP_GLB_STAT_FADDEN);
            *set_address = 0;
        }
    }
    if s & (1 << USB_CDC_EP_BULK_OUT) != 0 {
        usb_notify_bulk_out();
    }
    if s & (1 << USB_CDC_EP_BULK_IN) != 0 {
        usb_notify_bulk_in();
    }
}

decl_constant_str!("RESERVE_PINS_USB", "PB10,PB11");

pub fn usbserial_init() -> R {
    // Enable clocks
    enable_pclock(ID_UDP);
    let pmc = unsafe { &*PMC };
    pmc.PMC_USB.write(PMC_USB_USBDIV(5 - 1)); // PLLA=240Mhz; divide by 5 for 48Mhz
    pmc.PMC_SCER.write(PMC_SCER_UDP);

    // Enable USB pullup
    udp().UDP_TXVC.write(UDP_TXVC_PUON | UDP_TXVC_TXVDIS);

    // Enable interrupts
    udp().UDP_ICR.write(0xffffffff);
    armcm_enable_irq!(UDP_Handler, UDP_IRQn, 1);
    Ok(())
}
decl_init!(usbserial_init);
