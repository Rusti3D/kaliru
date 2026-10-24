// Hardware interface to USB on lpc176x
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::usb_cdc_ep::{USB_CDC_EP0_SIZE, USB_CDC_EP_ACM_SIZE, USB_CDC_EP_BULK_IN_SIZE, USB_CDC_EP_BULK_OUT_SIZE};
use crate::generic::armcm_timer::udelay;
use crate::generic::cmsis::{NVIC_DisableIRQ, NVIC_EnableIRQ};
use crate::generic::usb_cdc::{usb_notify_bulk_in, usb_notify_bulk_out, usb_notify_ep0};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init, decl_shutdown};

// Internal endpoint addresses
const EP0OUT: u32 = 0x00;
const EP0IN: u32 = 0x01;
const EP1IN: u32 = 0x03;
const EP2OUT: u32 = 0x04;
const EP5IN: u32 = 0x0b;

// USB device interupt status flags
const EP_SLOW: u32 = 1 << 2;
const DEV_STAT: u32 = 1 << 3;
const CCEMPTY: u32 = 1 << 4;
const CDFULL: u32 = 1 << 5;
const EP_RLZED: u32 = 1 << 8;

const RD_EN: u32 = 1 << 0;
const WR_EN: u32 = 1 << 1;

fn usb() -> &'static LPC_USB_TypeDef {
    unsafe { &*LPC_USB }
}

fn usb_irq_disable() {
    NVIC_DisableIRQ(USB_IRQn);
}

fn usb_irq_enable() {
    NVIC_EnableIRQ(USB_IRQn);
}

fn usb_wait(flag: u32) {
    while usb().USBDevIntSt.read() & flag == 0 {}
    usb().USBDevIntClr.write(flag);
}

/****************************************************************
 * Serial Interface Engine (SIE) functions
 ****************************************************************/

const SIE_CMD_SELECT: u32 = 0x00;
const SIE_CMD_SET_ENDPOINT_STATUS: u32 = 0x40;
const SIE_CMD_SET_ADDRESS: u32 = 0xD0;
const SIE_CMD_CONFIGURE: u32 = 0xD8;
const SIE_CMD_SET_DEVICE_STATUS: u32 = 0xFE;
const SIE_CMD_CLEAR_BUFFER: u32 = 0xF2;
const SIE_CMD_VALIDATE_BUFFER: u32 = 0xFA;

fn sie_cmd(cmd: u32) {
    usb().USBDevIntClr.write(CDFULL | CCEMPTY);
    usb().USBCmdCode.write(0x00000500 | (cmd << 16));
    usb_wait(CCEMPTY);
}

fn sie_cmd_write(cmd: u32, data: u32) {
    sie_cmd(cmd);
    usb().USBCmdCode.write(0x00000100 | (data << 16));
    usb_wait(CCEMPTY);
}

fn sie_cmd_read(cmd: u32) -> u32 {
    sie_cmd(cmd);
    usb().USBCmdCode.write(0x00000200 | (cmd << 16));
    usb_wait(CDFULL);
    usb().USBCmdData.read()
}

fn sie_select_and_clear(idx: u32) -> u32 {
    usb().USBEpIntClr.write(1 << idx);
    usb_wait(CDFULL);
    usb().USBCmdData.read()
}

/****************************************************************
 * Interface
 ****************************************************************/

fn usb_write_packet(ep: u32, data: *const u8, len: u8) -> i8 {
    usb_irq_disable();
    let sts = sie_cmd_read(SIE_CMD_SELECT | ep);
    if sts & 0x01 != 0 {
        // Output buffers full
        usb_irq_enable();
        return -1;
    }

    usb().USBCtrl.write(WR_EN | ((ep / 2) << 2));
    usb().USBTxPLen.write(len as u32);
    if len == 0 {
        usb().USBTxData.write(0);
    }
    for i in 0..(len as usize).div_ceil(4) {
        // (C copies 4 bytes, past the end of the data on the last word: the
        // bytes after len are not sent)
        let mut d = [0u8; 4];
        let n = (len as usize - i * 4).min(4);
        unsafe { core::ptr::copy_nonoverlapping(data.add(i * 4), d.as_mut_ptr(), n) };
        usb().USBTxData.write(u32::from_le(u32::from_ne_bytes(d)));
    }
    sie_cmd(SIE_CMD_VALIDATE_BUFFER);
    usb_irq_enable();

    len as i8
}

fn usb_read_packet(ep: u32, data: *mut u8, max_len: u8) -> i8 {
    usb_irq_disable();
    let mut sts = sie_cmd_read(SIE_CMD_SELECT | ep);
    if sts & 0x01 == 0 {
        // No data available
        usb_irq_enable();
        return -1;
    }

    // Determine packet size
    usb().USBCtrl.write(RD_EN | ((ep / 2) << 2));
    let mut plen = usb().USBRxPLen.read();
    while plen & (1 << 11) == 0 {
        plen = usb().USBRxPLen.read();
    }
    plen &= 0x3FF;
    if plen > max_len as u32 {
        // XXX - return error code?  must keep reading?
        plen = max_len as u32;
    }
    // Copy data
    let mut xfer = plen as usize;
    let mut p = data;
    loop {
        let d = u32::from_le(usb().USBRxData.read()).to_ne_bytes();
        if xfer <= d.len() {
            unsafe { core::ptr::copy_nonoverlapping(d.as_ptr(), p, xfer) };
            break;
        }
        unsafe { core::ptr::copy_nonoverlapping(d.as_ptr(), p, d.len()) };
        p = p.wrapping_add(d.len());
        xfer -= d.len();
    }
    // Clear space for next packet
    sts = sie_cmd_read(SIE_CMD_CLEAR_BUFFER);
    usb_irq_enable();
    if sts & 0x01 != 0 {
        // Packet overwritten
        return -1;
    }

    plen as i8
}

pub fn usb_read_bulk_out(data: *mut u8, max_len: u8) -> i8 {
    usb_read_packet(EP2OUT, data, max_len)
}

pub fn usb_send_bulk_in(data: *const u8, len: u8) -> i8 {
    usb_write_packet(EP5IN, data, len)
}

pub fn usb_read_ep0(data: *mut u8, max_len: u8) -> i8 {
    usb_read_packet(EP0OUT, data, max_len)
}

pub fn usb_read_ep0_setup(data: *mut u8, max_len: u8) -> i8 {
    usb_read_ep0(data, max_len)
}

pub fn usb_send_ep0(data: *const u8, len: u8) -> i8 {
    usb_write_packet(EP0IN, data, len)
}

pub fn usb_stall_ep0() {
    usb_irq_disable();
    sie_cmd_write(SIE_CMD_SET_ENDPOINT_STATUS, 1 << 7);
    usb_irq_enable();
}

pub fn usb_set_address(addr: u8) {
    usb_irq_disable();
    sie_cmd_write(SIE_CMD_SET_ADDRESS, addr as u32 | (1 << 7));
    usb_irq_enable();
    usb_send_ep0(core::ptr::null(), 0);
}

fn realize_endpoint(idx: u32, packet_size: u32) {
    usb().USBDevIntClr.write(EP_RLZED);
    usb().USBReEp.or(1 << idx);
    usb().USBEpInd.write(idx);
    usb().USBMaxPSize.write(packet_size);
    usb_wait(EP_RLZED);
    usb().USBEpIntEn.or(1 << idx);
    sie_cmd_write(SIE_CMD_SET_ENDPOINT_STATUS | idx, 0);
}

pub fn usb_set_configure() {
    usb_irq_disable();
    realize_endpoint(EP1IN, USB_CDC_EP_ACM_SIZE as u32);
    realize_endpoint(EP2OUT, USB_CDC_EP_BULK_OUT_SIZE as u32);
    realize_endpoint(EP5IN, USB_CDC_EP_BULK_IN_SIZE as u32);
    sie_cmd_write(SIE_CMD_CONFIGURE, 1);
    usb_irq_enable();
}

// Force a USB disconnect (used during reboot into bootloader)
pub fn usb_disconnect() {
    sie_cmd_write(SIE_CMD_SET_DEVICE_STATUS, 0);
    udelay(5000);
}

/****************************************************************
 * Setup and interrupts
 ****************************************************************/

#[unsafe(no_mangle)]
pub unsafe extern "C" fn USB_IRQHandler() {
    let udis = usb().USBDevIntSt.read();
    if udis & DEV_STAT != 0 {
        usb().USBDevIntClr.write(DEV_STAT);
        // XXX - should handle reset and other states
    }
    if udis & EP_SLOW != 0 {
        let ueis = usb().USBEpIntSt.read();
        if ueis & (1 << EP0OUT) != 0 {
            sie_select_and_clear(EP0OUT);
            usb_notify_ep0();
        }
        if ueis & (1 << EP0IN) != 0 {
            sie_select_and_clear(EP0IN);
            usb_notify_ep0();
        }
        if ueis & (1 << EP2OUT) != 0 {
            sie_select_and_clear(EP2OUT);
            usb_notify_bulk_out();
        }
        if ueis & (1 << EP5IN) != 0 {
            sie_select_and_clear(EP5IN);
            usb_notify_bulk_in();
        }
        usb().USBDevIntClr.write(EP_SLOW);
    }
}

decl_constant_str!("RESERVE_PINS_USB", "P0.30,P0.29,P2.9");

pub fn usbserial_init() -> R {
    usb_irq_disable();
    // enable power
    enable_pclock(PCLK_USB);
    // enable clock
    usb().USBClkCtrl.write(0x12);
    while usb().USBClkSt.read() != 0x12 {}
    // configure USBD-, USBD+, and USB Connect pins
    gpio_peripheral(GPIO(0, 30), 1, 0);
    gpio_peripheral(GPIO(0, 29), 1, 0);
    gpio_peripheral(GPIO(2, 9), 1, 0);
    // enforce a minimum time bus is disconnected before connecting
    udelay(5000);
    // setup endpoints
    realize_endpoint(EP0OUT, USB_CDC_EP0_SIZE as u32);
    realize_endpoint(EP0IN, USB_CDC_EP0_SIZE as u32);
    sie_cmd_write(SIE_CMD_SET_DEVICE_STATUS, 1);
    // enable irqs
    usb().USBDevIntEn.write(DEV_STAT | EP_SLOW);
    armcm_enable_irq!(USB_IRQHandler, USB_IRQn, 1);
    Ok(())
}
decl_init!(usbserial_init);

pub fn usbserial_shutdown() -> R {
    usb_irq_enable();
    Ok(())
}
decl_shutdown!(usbserial_shutdown);
