// Hardware interface to USB on AVR at90usb
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::pgm::pgm_read_byte;
use crate::ctr::Global;
use crate::generic::usb_cdc::{usb_notify_bulk_in, usb_notify_bulk_out, usb_notify_ep0};
use crate::generic::usb_cdc_ep::{
    USB_CDC_EP0_SIZE, USB_CDC_EP_ACM, USB_CDC_EP_ACM_SIZE, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_IN_SIZE, USB_CDC_EP_BULK_OUT,
    USB_CDC_EP_BULK_OUT_SIZE,
};
use crate::sched::R;
use crate::decl_init;
use core::ptr::null;

// EPCFG0X definitions
const EP_TYPE_CONTROL: u8 = 0x00;
const EP_TYPE_BULK_IN: u8 = 0x81;
const EP_TYPE_BULK_OUT: u8 = 0x80;
const EP_TYPE_INTERRUPT_IN: u8 = 0xC1;

// EPCFG1X definitions
const EP_SINGLE_BUFFER: u8 = 0x02;
const EP_DOUBLE_BUFFER: u8 = 0x06;
const fn EP_SIZE(s: usize) -> u8 {
    if s == 64 {
        0x30
    } else if s == 32 {
        0x20
    } else if s == 16 {
        0x10
    } else {
        0x00
    }
}

fn r(reg: *mut Reg<u8>) -> u8 {
    unsafe { (*reg).read() }
}
fn w(reg: *mut Reg<u8>, v: u8) {
    unsafe { (*reg).write(v) }
}

fn usb_write_packet(data: *const u8, len: u8) {
    for i in 0..len as usize {
        w(UEDATX, unsafe { *data.add(i) });
    }
}

fn usb_write_packet_progmem(data: *const u8, len: u8) {
    for i in 0..len as usize {
        w(UEDATX, pgm_read_byte(unsafe { data.add(i) }));
    }
}

fn usb_read_packet(data: *mut u8, len: u8) {
    for i in 0..len as usize {
        unsafe { *data.add(i) = r(UEDATX) };
    }
}

pub fn usb_read_bulk_out(data: *mut u8, _max_len: u8) -> i8 {
    w(UENUM, USB_CDC_EP_BULK_OUT as u8);
    if r(UEINTX) & (1 << RXOUTI) == 0 {
        // No data ready
        w(UEIENX, 1 << RXOUTE);
        return -1;
    }
    let len = r(UEBCLX);
    usb_read_packet(data, len);
    w(UEINTX, !((1 << FIFOCON) | (1 << RXOUTI)));
    len as i8
}

pub fn usb_send_bulk_in(data: *const u8, len: u8) -> i8 {
    w(UENUM, USB_CDC_EP_BULK_IN as u8);
    if r(UEINTX) & (1 << TXINI) == 0 {
        // Buffer full
        w(UEIENX, 1 << TXINE);
        return -1;
    }
    usb_write_packet(data, len);
    w(UEINTX, !((1 << FIFOCON) | (1 << TXINI) | (1 << RXOUTI)));
    len as i8
}

pub fn usb_read_ep0(data: *mut u8, max_len: u8) -> i8 {
    w(UENUM, 0);
    let ueintx = r(UEINTX);
    if ueintx & (1 << RXSTPI) != 0 {
        return -2;
    }
    if ueintx & (1 << RXOUTI) == 0 {
        // Not ready to receive data
        w(UEIENX, (1 << RXSTPE) | (1 << RXOUTE));
        return -1;
    }
    usb_read_packet(data, max_len);
    if r(UEINTX) & (1 << RXSTPI) != 0 {
        return -2;
    }
    w(UEINTX, !(1 << RXOUTI));
    max_len as i8
}

pub fn usb_read_ep0_setup(data: *mut u8, max_len: u8) -> i8 {
    w(UENUM, 0);
    let ueintx = r(UEINTX);
    if ueintx & (1 << RXSTPI) == 0 {
        // No data ready to read
        w(UEIENX, 1 << RXSTPE);
        return -1;
    }
    usb_read_packet(data, max_len);
    w(UEINTX, !((1 << RXSTPI) | (1 << RXOUTI)));
    max_len as i8
}

fn _usb_send_ep0(data: *const u8, len: u8, progmem: bool) -> i8 {
    w(UENUM, 0);
    let ueintx = r(UEINTX);
    if ueintx & ((1 << RXSTPI) | (1 << RXOUTI)) != 0 {
        return -2;
    }
    if ueintx & (1 << TXINI) == 0 {
        // Not ready to send
        w(UEIENX, (1 << RXSTPE) | (1 << RXOUTE) | (1 << TXINE));
        return -1;
    }
    if progmem {
        usb_write_packet_progmem(data, len);
    } else {
        usb_write_packet(data, len);
    }
    w(UEINTX, !(1 << TXINI));
    len as i8
}

pub fn usb_send_ep0(data: *const u8, len: u8) -> i8 {
    _usb_send_ep0(data, len, false)
}

pub fn usb_send_ep0_progmem(data: *const u8, len: u8) -> i8 {
    _usb_send_ep0(data, len, true)
}

pub fn usb_stall_ep0() {
    w(UENUM, 0);
    w(UECONX, (1 << STALLRQ) | (1 << EPEN));
    w(UEIENX, 1 << RXSTPE);
}

static SET_ADDRESS: Global<u8> = Global::new(0);

pub fn usb_set_address(addr: u8) {
    *SET_ADDRESS.get() = addr | (1 << ADDEN);
    _usb_send_ep0(null(), 0, false);
    w(UEIENX, (1 << RXSTPE) | (1 << TXINE));
}

pub fn usb_set_configure() {
    w(UENUM, USB_CDC_EP_ACM as u8);
    w(UECONX, 1 << EPEN);
    w(UECFG0X, EP_TYPE_INTERRUPT_IN);
    w(UECFG1X, EP_SIZE(USB_CDC_EP_ACM_SIZE) | EP_SINGLE_BUFFER);

    w(UENUM, USB_CDC_EP_BULK_OUT as u8);
    w(UECONX, 1 << EPEN);
    w(UECFG0X, EP_TYPE_BULK_OUT);
    w(UECFG1X, EP_SIZE(USB_CDC_EP_BULK_OUT_SIZE) | EP_DOUBLE_BUFFER);
    w(UEIENX, 1 << RXOUTE);

    w(UENUM, USB_CDC_EP_BULK_IN as u8);
    w(UECONX, 1 << EPEN);
    w(UECFG0X, EP_TYPE_BULK_IN);
    w(UECFG1X, EP_SIZE(USB_CDC_EP_BULK_IN_SIZE) | EP_DOUBLE_BUFFER);
    w(UEIENX, 1 << TXINE);
}

#[cfg(CONFIG_MACH_at90usb1286)]
const UHWCON_INIT: u8 = (1 << UIMOD) | (1 << UVREGE);
#[cfg(CONFIG_MACH_at90usb1286)]
const PLLCSR_INIT: u8 = (1 << PLLP2) | (1 << PLLP0) | (1 << PLLE);
#[cfg(CONFIG_MACH_at90usb646)]
const UHWCON_INIT: u8 = (1 << UIMOD) | (1 << UVREGE);
#[cfg(CONFIG_MACH_at90usb646)]
const PLLCSR_INIT: u8 = (1 << PLLP2) | (1 << PLLP1) | (1 << PLLE);
#[cfg(CONFIG_MACH_atmega32u4)]
const UHWCON_INIT: u8 = 1 << UVREGE;
#[cfg(CONFIG_MACH_atmega32u4)]
const PLLCSR_INIT: u8 = (1 << PINDIV) | (1 << PLLE);

pub fn usbserial_init() -> R {
    // Set USB controller to device mode
    w(UHWCON, UHWCON_INIT);

    // Enable USB clock
    w(USBCON, (1 << USBE) | (1 << FRZCLK));
    w(PLLCSR, PLLCSR_INIT);
    while r(PLLCSR) & (1 << PLOCK) == 0 {}
    w(USBCON, (1 << USBE) | (1 << OTGPADE));

    // Enable USB pullup
    w(UDCON, 0);

    // Enable interrupts
    w(UDIEN, 1 << EORSTE);
    Ok(())
}
decl_init!(usbserial_init);

#[export_name = avr_vector!(USB_GEN_vect)]
pub unsafe extern "avr-interrupt" fn usb_gen_isr() {
    let udint = r(UDINT);
    w(UDINT, 0);
    if udint & (1 << EORSTI) != 0 {
        // Configure endpoint 0 after usb reset completes
        let old_uenum = r(UENUM);
        w(UENUM, 0);
        w(UECONX, 1 << EPEN);
        w(UECFG0X, EP_TYPE_CONTROL);
        w(UECFG1X, EP_SIZE(USB_CDC_EP0_SIZE) | EP_SINGLE_BUFFER);
        w(UEIENX, 1 << RXSTPE);
        w(UENUM, old_uenum);
    }
}

#[export_name = avr_vector!(USB_COM_vect)]
pub unsafe extern "avr-interrupt" fn usb_com_isr() {
    let ueint = r(UEINT);
    let old_uenum = r(UENUM);
    if ueint & (1 << 0) != 0 {
        w(UENUM, 0);
        w(UEIENX, 0);
        usb_notify_ep0();

        let ueintx = r(UEINTX);
        let set_address = SET_ADDRESS.get();
        if ueintx & (1 << RXSTPI) == 0 && ueintx & (1 << TXINI) != 0 && *set_address != 0 {
            // Ack from set_address command sent - now update address
            w(UDADDR, *set_address);
            *set_address = 0;
        }
    }
    if ueint & (1 << USB_CDC_EP_BULK_OUT) != 0 {
        w(UENUM, USB_CDC_EP_BULK_OUT as u8);
        w(UEIENX, 0);
        usb_notify_bulk_out();
    }
    if ueint & (1 << USB_CDC_EP_BULK_IN) != 0 {
        w(UENUM, USB_CDC_EP_BULK_IN as u8);
        w(UEIENX, 0);
        usb_notify_bulk_in();
    }
    w(UENUM, old_uenum);
}
