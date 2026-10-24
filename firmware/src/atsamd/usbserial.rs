// Hardware interface to USB on samd
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::internal::*;
use crate::config::CONFIG_MACH_SAMD21;
use crate::ctr::Global;
use crate::generic::usb_cdc::{usb_notify_bulk_in, usb_notify_bulk_out, usb_notify_ep0};
use crate::generic::usb_cdc_ep::{
    USB_CDC_EP0_SIZE, USB_CDC_EP_ACM, USB_CDC_EP_ACM_SIZE, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_IN_SIZE, USB_CDC_EP_BULK_OUT,
    USB_CDC_EP_BULK_OUT_SIZE,
};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};
use core::mem::MaybeUninit;

/****************************************************************
 * USB transfer memory
 ****************************************************************/

#[repr(C, align(4))]
struct Buf<const N: usize>([u8; N]);

static EP0OUT: Global<Buf<USB_CDC_EP0_SIZE>> = Global::new(Buf([0; USB_CDC_EP0_SIZE]));
static EP0IN: Global<Buf<USB_CDC_EP0_SIZE>> = Global::new(Buf([0; USB_CDC_EP0_SIZE]));
static ACMIN: Global<Buf<USB_CDC_EP_ACM_SIZE>> = Global::new(Buf([0; USB_CDC_EP_ACM_SIZE]));
static BULKOUT: Global<Buf<USB_CDC_EP_BULK_OUT_SIZE>> = Global::new(Buf([0; USB_CDC_EP_BULK_OUT_SIZE]));
static BULKIN: Global<Buf<USB_CDC_EP_BULK_IN_SIZE>> = Global::new(Buf([0; USB_CDC_EP_BULK_IN_SIZE]));

// Convert 64, 32, 16, 8 sized buffer to 3, 2, 1, 0 for PCKSIZE.SIZE register
const fn BSIZE(size: usize) -> u32 {
    size.trailing_zeros() - 3
}

// (the descriptors of the endpoints up to the highest one used; the C
// initializes them statically, here usbserial_init() does before
// handing them to the hardware)
const NUM_DESC: usize = {
    let mut n = USB_CDC_EP_ACM;
    if USB_CDC_EP_BULK_OUT > n {
        n = USB_CDC_EP_BULK_OUT;
    }
    if USB_CDC_EP_BULK_IN > n {
        n = USB_CDC_EP_BULK_IN;
    }
    n + 1
};
static USB_DESC: Global<MaybeUninit<[UsbDeviceDescriptor; NUM_DESC]>> = Global::new(MaybeUninit::zeroed());

fn usb_desc(ep: usize) -> &'static UsbDeviceDescriptor {
    unsafe { &(*USB_DESC.ptr()).assume_init_ref()[ep] }
}

fn desc_setup(ep: usize, bank: usize, buf: *mut u8, size: usize) {
    let d = &usb_desc(ep).DeviceDescBank[bank];
    d.ADDR.reg.write(buf as u32);
    d.PCKSIZE.reg.write(USB_DEVICE_PCKSIZE_SIZE(BSIZE(size)));
}

fn usb_device() -> &'static UsbDevice {
    unsafe { &*USB }.DEVICE()
}
fn ep(n: usize) -> &'static UsbDeviceEndpoint {
    &usb_device().DeviceEndpoint[n]
}

fn usb_write_packet(n: usize, bank: usize, data: *const u8, len: u8) -> i8 {
    // Check if there is room for this packet
    let ude = ep(n);
    let sts = ude.EPSTATUS.reg.read() as u32;
    let bkrdy = if bank != 0 { USB_DEVICE_EPSTATUS_BK1RDY } else { USB_DEVICE_EPSTATUS_BK0RDY };
    if sts & bkrdy != 0 {
        return -1;
    }
    // Copy the packet to the given buffer
    let uddb = &usb_desc(n).DeviceDescBank[bank];
    unsafe { core::ptr::copy_nonoverlapping(data, uddb.ADDR.reg.read() as usize as *mut u8, len as usize) };
    // Inform the USB hardware of the available packet
    let pcksize = uddb.PCKSIZE.reg.read();
    let c = pcksize & !USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk;
    uddb.PCKSIZE.reg.write(c | USB_DEVICE_PCKSIZE_BYTE_COUNT(len as u32));
    ude.EPSTATUSSET.reg.write(bkrdy as u8);
    len as i8
}

fn usb_read_packet(n: usize, bank: usize, data: *mut u8, max_len: u8) -> i8 {
    // Check if there is a packet ready
    let ude = ep(n);
    let sts = ude.EPSTATUS.reg.read() as u32;
    let bkrdy = if bank != 0 { USB_DEVICE_EPSTATUS_BK1RDY } else { USB_DEVICE_EPSTATUS_BK0RDY };
    if sts & bkrdy == 0 {
        return -1;
    }
    // Copy the packet to the given buffer
    let uddb = &usb_desc(n).DeviceDescBank[bank];
    let pcksize = uddb.PCKSIZE.reg.read();
    let mut c = pcksize & USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk;
    if c > max_len as u32 {
        c = max_len as u32;
    }
    unsafe { core::ptr::copy_nonoverlapping(uddb.ADDR.reg.read() as usize as *const u8, data, c as usize) };
    // Notify the USB hardware that the space is now available
    ude.EPSTATUSCLR.reg.write(bkrdy as u8);
    c as i8
}

/****************************************************************
 * Interface
 ****************************************************************/

pub fn usb_read_bulk_out(data: *mut u8, max_len: u8) -> i8 {
    usb_read_packet(USB_CDC_EP_BULK_OUT, 0, data, max_len)
}

pub fn usb_send_bulk_in(data: *const u8, len: u8) -> i8 {
    usb_write_packet(USB_CDC_EP_BULK_IN, 1, data, len)
}

pub fn usb_read_ep0(data: *mut u8, max_len: u8) -> i8 {
    usb_read_packet(0, 0, data, max_len)
}

pub fn usb_read_ep0_setup(data: *mut u8, max_len: u8) -> i8 {
    usb_read_ep0(data, max_len)
}

pub fn usb_send_ep0(data: *const u8, len: u8) -> i8 {
    usb_write_packet(0, 1, data, len)
}

pub fn usb_stall_ep0() {
    ep(0).EPSTATUSSET.reg.write(USB_DEVICE_EPSTATUS_STALLRQ(3) as u8);
}

static SET_ADDRESS: Global<u8> = Global::new(0);

pub fn usb_set_address(addr: u8) {
    unsafe { SET_ADDRESS.ptr().write_volatile(addr | USB_DEVICE_DADD_ADDEN as u8) };
    usb_send_ep0(core::ptr::null(), 0);
}

pub fn usb_set_configure() {
    ep(USB_CDC_EP_ACM).EPCFG.reg.write(USB_DEVICE_EPCFG_EPTYPE1(4) as u8);

    ep(USB_CDC_EP_BULK_OUT).EPCFG.reg.write(USB_DEVICE_EPCFG_EPTYPE0(3) as u8);
    ep(USB_CDC_EP_BULK_OUT).EPINTENSET.reg.write((USB_DEVICE_EPINTENSET_TRCPT0 | USB_DEVICE_EPINTENSET_TRCPT1) as u8);

    ep(USB_CDC_EP_BULK_IN).EPCFG.reg.write(USB_DEVICE_EPCFG_EPTYPE1(3) as u8);
    ep(USB_CDC_EP_BULK_IN).EPINTENSET.reg.write((USB_DEVICE_EPINTENSET_TRCPT0 | USB_DEVICE_EPINTENSET_TRCPT1) as u8);
}

/****************************************************************
 * Setup and interrupts
 ****************************************************************/

#[unsafe(no_mangle)]
pub unsafe extern "C" fn USB_Handler() {
    let d = usb_device();
    let s = d.INTFLAG.reg.read() as u32;
    if s & USB_DEVICE_INTFLAG_EORST != 0 {
        d.INTFLAG.reg.write(USB_DEVICE_INTFLAG_EORST as _);
        // Enable endpoint 0 irqs
        ep(0).EPINTENSET.reg.write(
            (USB_DEVICE_EPINTENSET_TRCPT0 | USB_DEVICE_EPINTENSET_TRCPT1 | USB_DEVICE_EPINTENSET_RXSTP) as u8,
        );
    }

    let epsum = d.EPINTSMRY.reg.read() as u32;
    if epsum & (1 << 0) != 0 {
        let sts = ep(0).EPINTFLAG.reg.read();
        ep(0).EPINTFLAG.reg.write(sts);
        let set_address = SET_ADDRESS.get();
        if *set_address != 0 && sts as u32 & USB_DEVICE_EPINTFLAG_TRCPT1 != 0 {
            // Apply address after last "in" message transmitted
            d.DADD.reg.write(*set_address);
            *set_address = 0;
        }
        usb_notify_ep0();
    }
    if epsum & (1 << USB_CDC_EP_BULK_OUT) != 0 {
        let sts = ep(USB_CDC_EP_BULK_OUT).EPINTFLAG.reg.read();
        ep(USB_CDC_EP_BULK_OUT).EPINTFLAG.reg.write(sts);
        usb_notify_bulk_out();
    }
    if epsum & (1 << USB_CDC_EP_BULK_IN) != 0 {
        let sts = ep(USB_CDC_EP_BULK_IN).EPINTFLAG.reg.read();
        ep(USB_CDC_EP_BULK_IN).EPINTFLAG.reg.write(sts);
        usb_notify_bulk_in();
    }
}

decl_constant_str!("RESERVE_PINS_USB", "PA24,PA25");

pub fn usbserial_init() -> R {
    // (the static initialization of the C usb_desc[])
    desc_setup(0, 0, EP0OUT.ptr() as *mut u8, USB_CDC_EP0_SIZE);
    desc_setup(0, 1, EP0IN.ptr() as *mut u8, USB_CDC_EP0_SIZE);
    desc_setup(USB_CDC_EP_ACM, 1, ACMIN.ptr() as *mut u8, USB_CDC_EP_ACM_SIZE);
    desc_setup(USB_CDC_EP_BULK_OUT, 0, BULKOUT.ptr() as *mut u8, USB_CDC_EP_BULK_OUT_SIZE);
    desc_setup(USB_CDC_EP_BULK_IN, 1, BULKIN.ptr() as *mut u8, USB_CDC_EP_BULK_IN_SIZE);

    // configure usb clock
    enable_pclock(USB_GCLK_ID, ID_USB as i32);
    // configure USBD+ and USBD- pins
    let ptype = if CONFIG_MACH_SAMD21 { 'G' } else { 'H' };
    gpio_peripheral(GPIO('A', 24), ptype, 0);
    gpio_peripheral(GPIO('A', 25), ptype, 0);
    let trim = GET_FUSE(USB_FUSES_TRIM_ADDR, USB_FUSES_TRIM_Msk, USB_FUSES_TRIM_Pos);
    let transp = GET_FUSE(USB_FUSES_TRANSP_ADDR, USB_FUSES_TRANSP_Msk, USB_FUSES_TRANSP_Pos);
    let transn = GET_FUSE(USB_FUSES_TRANSN_ADDR, USB_FUSES_TRANSN_Msk, USB_FUSES_TRANSN_Pos);
    let d = usb_device();
    d.PADCAL.reg.write((USB_PADCAL_TRIM(trim) | USB_PADCAL_TRANSP(transp) | USB_PADCAL_TRANSN(transn)) as _);
    // Enable USB in device mode
    d.CTRLA.reg.write(USB_CTRLA_ENABLE as _);
    d.DESCADD.reg.write(USB_DESC.ptr() as u32);
    ep(0).EPCFG.reg.write((USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1)) as u8);
    ep(USB_CDC_EP_ACM).EPCFG.reg.write(USB_DEVICE_EPCFG_EPTYPE1(4) as u8);
    d.CTRLB.reg.write(0);
    // enable irqs
    d.INTENSET.reg.write(USB_DEVICE_INTENSET_EORST as _);
    #[cfg(CONFIG_MACH_SAMD21)]
    armcm_enable_irq!(USB_Handler, USB_IRQn, 1);
    #[cfg(CONFIG_MACH_SAMX5)]
    {
        armcm_enable_irq!(USB_Handler, USB_0_IRQn, 1);
        armcm_enable_irq!(USB_Handler, USB_1_IRQn, 1);
        armcm_enable_irq!(USB_Handler, USB_2_IRQn, 1);
        armcm_enable_irq!(USB_Handler, USB_3_IRQn, 1);
    }
    Ok(())
}
decl_init!(usbserial_init);
