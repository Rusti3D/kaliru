// Hardware interface to "fullspeed USB controller"
//
// Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals, non_snake_case)]
use super::internal::*;
use crate::board::{gpio_in_setup, gpio_out_setup};
use crate::config::{CONFIG_MACH_AT32F403, CONFIG_MACH_STM32F1, CONFIG_STM32_USB_DOUBLE_BUFFER_TX};
use crate::ctr::Global;
use crate::generic::armcm_timer::udelay;
use crate::generic::usb_cdc::{usb_notify_bulk_in, usb_notify_bulk_out, usb_notify_ep0};
use crate::generic::usb_cdc_ep::{USB_CDC_EP0_SIZE, USB_CDC_EP_ACM, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_OUT, USB_CDC_EP_BULK_OUT_SIZE};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};
use core::ptr::{read_volatile, write_volatile};

// Transfer memory is accessed with 32bits, but contains only 16bits of data
#[cfg(CONFIG_MACH_STM32F1)]
type EpmWord = u32;
#[cfg(CONFIG_MACH_STM32F1)]
const WSIZE: usize = 2;
// Transfer memory is accessed with 16bits and contains 16bits of data
#[cfg(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32L4, CONFIG_MACH_STM32G4))]
type EpmWord = u16;
#[cfg(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32L4, CONFIG_MACH_STM32G4))]
const WSIZE: usize = 2;
// Transfer memory is accessed with 32bits and contains 32bits of data
#[cfg(CONFIG_MACH_STM32G0)]
type EpmWord = u32;
#[cfg(CONFIG_MACH_STM32G0)]
const WSIZE: usize = 4;

// Different chips have different names for the USB irq
#[cfg(any(CONFIG_MACH_STM32F1, CONFIG_MACH_STM32G4))]
const USBx_IRQn: IRQn_Type = USB_LP_IRQn;
#[cfg(all(not(any(CONFIG_MACH_STM32F1, CONFIG_MACH_STM32G4)), CONFIG_MACH_STM32G0B1))]
const USBx_IRQn: IRQn_Type = USB_UCPD1_2_IRQn;
#[cfg(not(any(CONFIG_MACH_STM32F1, CONFIG_MACH_STM32G4, CONFIG_MACH_STM32G0B1)))]
const USBx_IRQn: IRQn_Type = USB_IRQn;

// (the stm32g0 register names - USB_DRD_FS, USB_CHEP_ADDR, ... - come with
// that chip)

// Some chip variants do not define these fields
#[cfg(has_USB_EP_DTOG_TX_Pos)]
const DTOG_TX_POS: u32 = USB_EP_DTOG_TX_Pos;
#[cfg(has_USB_EP_DTOG_TX_Pos)]
const DTOG_RX_POS: u32 = USB_EP_DTOG_RX_Pos;
#[cfg(not(has_USB_EP_DTOG_TX_Pos))]
const DTOG_TX_POS: u32 = 6;
#[cfg(not(has_USB_EP_DTOG_TX_Pos))]
const DTOG_RX_POS: u32 = 14;

fn USBR() -> &'static USB_TypeDef {
    unsafe { &*USB }
}

/****************************************************************
 * USB transfer memory
 ****************************************************************/

// Layout of the USB transfer memory
fn EPM() -> *mut EpmWord {
    USB_PMAADDR as *mut EpmWord
}
fn EPM_EP_DESC(ep: usize, bufnum: usize) -> *mut EpmWord {
    EPM().wrapping_add((ep * 2 + bufnum) * (4 / WSIZE))
}
const EPM_BUF_OFFSET: usize = 0x10;
const EPM_EP_BUF_SIZE: usize = 64 / WSIZE + 1;
fn EPM_EP_BUF(ep: usize, bufnum: usize) -> *mut EpmWord {
    EPM().wrapping_add(EPM_BUF_OFFSET + (ep * 2 + bufnum) * EPM_EP_BUF_SIZE)
}
const BUFTX: usize = 0;
const BUFRX: usize = 1;

fn rd(p: *mut EpmWord) -> u32 {
    unsafe { read_volatile(p) as u32 }
}
fn wr(p: *mut EpmWord, v: u32) {
    unsafe { write_volatile(p, v as EpmWord) }
}

// Configure the usb descriptor for an endpoint
fn epm_ep_desc_setup(ep: usize, bufnum: usize, rx_size: u32) {
    let addr = ((EPM_EP_BUF(ep, bufnum) as usize - EPM() as usize) / core::mem::size_of::<EpmWord>() * WSIZE) as u32;
    let count_rx = if rx_size <= 30 { rx_size.div_ceil(2) << 10 } else { ((rx_size.div_ceil(32) - 1) << 10) | 0x8000 };
    let desc = EPM_EP_DESC(ep, bufnum);
    if WSIZE == 2 {
        wr(desc, addr);
        wr(desc.wrapping_add(1), count_rx);
    } else {
        wr(desc, addr | (count_rx << 16));
    }
}

// Return number of read bytes on an rx endpoint
fn epm_get_ep_count_rx(ep: usize, bufnum: usize) -> u32 {
    let desc = EPM_EP_DESC(ep, bufnum);
    if WSIZE == 2 {
        return rd(desc.wrapping_add(1)) & 0x3ff;
    }
    (rd(desc) >> 16) & 0x3ff
}

// Set number of bytes ready to be transmitted on a tx endpoint
fn epm_set_ep_count_tx(ep: usize, bufnum: usize, count: u32) {
    let desc = EPM_EP_DESC(ep, bufnum);
    if WSIZE == 2 {
        wr(desc.wrapping_add(1), count);
    } else {
        let addr_tx = ((EPM_EP_BUF(ep, bufnum) as usize - EPM() as usize) / core::mem::size_of::<EpmWord>() * WSIZE) as u32;
        wr(desc, addr_tx | (count << 16));
    }
}

// Setup the transfer descriptors in dedicated usb memory
fn btable_configure() {
    epm_ep_desc_setup(0, BUFTX, 0);
    epm_ep_desc_setup(0, BUFRX, USB_CDC_EP0_SIZE as u32);
    epm_ep_desc_setup(USB_CDC_EP_ACM, BUFTX, 0);
    epm_ep_desc_setup(USB_CDC_EP_ACM, BUFRX, 0);
    epm_ep_desc_setup(USB_CDC_EP_BULK_OUT, 0, USB_CDC_EP_BULK_OUT_SIZE as u32);
    epm_ep_desc_setup(USB_CDC_EP_BULK_OUT, 1, USB_CDC_EP_BULK_OUT_SIZE as u32);
    epm_ep_desc_setup(USB_CDC_EP_BULK_IN, 0, 0);
    epm_ep_desc_setup(USB_CDC_EP_BULK_IN, 1, 0);
}

// Read a packet stored in dedicated usb memory
fn btable_read_packet(ep: usize, bufnum: usize, mut dest: *mut u8, max_len: u32) -> u32 {
    let mut src = EPM_EP_BUF(ep, bufnum);
    let mut count = epm_get_ep_count_rx(ep, bufnum);
    if count > max_len {
        count = max_len;
    }
    let w = WSIZE as u32;
    let put = |dest: &mut *mut u8, b: u32| unsafe {
        **dest = b as u8;
        *dest = dest.add(1);
    };
    for _ in 0..count / w {
        let d = rd(src);
        src = src.wrapping_add(1);
        put(&mut dest, d);
        put(&mut dest, d >> 8);
        if WSIZE == 4 {
            put(&mut dest, d >> 16);
            put(&mut dest, d >> 24);
        }
    }
    if count & (w - 1) != 0 {
        let d = rd(src);
        put(&mut dest, d);
        if (count & (w - 1)) > 1 {
            put(&mut dest, d >> 8);
        }
        if (count & (w - 1)) > 2 {
            put(&mut dest, d >> 16);
        }
    }
    count
}

// Write a packet to dedicated usb memory
fn btable_write_packet(ep: usize, bufnum: usize, mut src: *const u8, count: u32) {
    let mut dest = EPM_EP_BUF(ep, bufnum);
    let w = WSIZE as u32;
    let get = |src: &mut *const u8| unsafe {
        let b = **src as u32;
        *src = src.add(1);
        b
    };
    for _ in 0..count / w {
        let (b1, b2) = (get(&mut src), get(&mut src));
        let (mut b3, mut b4) = (0, 0);
        if WSIZE == 4 {
            b3 = get(&mut src);
            b4 = get(&mut src);
        }
        wr(dest, b1 | (b2 << 8) | (b3 << 16) | (b4 << 24));
        dest = dest.wrapping_add(1);
    }
    if count & (w - 1) != 0 {
        let mut d = get(&mut src);
        if (count & (w - 1)) > 1 {
            d |= get(&mut src) << 8;
        }
        if (count & (w - 1)) > 2 {
            d |= get(&mut src) << 16;
        }
        wr(dest, d);
    }
    epm_set_ep_count_tx(ep, bufnum, count);
}

/****************************************************************
 * USB endpoint register
 ****************************************************************/

fn USB_EPR(ep: usize) -> *mut u32 {
    (USB_BASE as *mut u32).wrapping_add(ep)
}
fn epr_read(ep: usize) -> u32 {
    unsafe { read_volatile(USB_EPR(ep)) }
}
fn epr_write(ep: usize, v: u32) {
    unsafe { write_volatile(USB_EPR(ep), v) }
}

const EPR_TBITS: u32 = USB_EP_DTOG_RX | USB_EPRX_STAT | USB_EP_DTOG_TX | USB_EPTX_STAT;
const EPR_RWBITS: u32 = USB_EPADDR_FIELD | USB_EP_KIND | USB_EP_TYPE_MASK;
const EPR_RWCBITS: u32 = USB_EP_CTR_RX | USB_EP_CTR_TX;

// Calculate the memory update needed to set the epr register
fn calc_epr_bits(epr: u32, mask: u32, value: u32) -> u32 {
    let tmask = mask & EPR_TBITS;
    let tvalue = value & tmask;
    let rwmask = mask & EPR_RWBITS;
    let rwbits = value & rwmask;
    let cbits = EPR_RWCBITS & !mask;
    (((epr & (EPR_RWBITS | tmask)) ^ tvalue) & !rwmask) | rwbits | cbits
}

// Check if double buffering endpoint hardware can no longer send/receive
fn epr_is_dbuf_blocking(epr: u32) -> bool {
    ((epr >> (DTOG_RX_POS - DTOG_TX_POS)) ^ epr) & USB_EP_DTOG_TX == 0
}

/****************************************************************
 * USB interface
 ****************************************************************/

static BULK_OUT_POP_COUNT: Global<u32> = Global::new(0);
static BULK_OUT_PUSH_FLAG: Global<u32> = Global::new(0);

fn readl(v: &Global<u32>) -> u32 {
    unsafe { read_volatile(v.ptr()) }
}
fn writel(v: &Global<u32>, x: u32) {
    unsafe { write_volatile(v.ptr(), x) }
}

pub fn usb_read_bulk_out(data: *mut u8, max_len: u8) -> i8 {
    if readl(&BULK_OUT_PUSH_FLAG) != 0 {
        // No data ready
        return -1;
    }
    let ep = USB_CDC_EP_BULK_OUT;
    let pop = BULK_OUT_POP_COUNT.get();
    let bufnum = (*pop & 1) as usize;
    *pop = pop.wrapping_add(1);
    let count = btable_read_packet(ep, bufnum, data, max_len as u32);
    writel(&BULK_OUT_PUSH_FLAG, USB_EP_DTOG_TX);

    // Check if irq handler pulled another packet before push flag update
    let epr = epr_read(ep);
    if epr_is_dbuf_blocking(epr) && readl(&BULK_OUT_PUSH_FLAG) != 0 {
        // Second packet was already read - must notify hardware
        writel(&BULK_OUT_PUSH_FLAG, 0);
        epr_write(ep, calc_epr_bits(epr, 0, 0) | USB_EP_DTOG_TX);
    }

    count as i8
}

static BULK_IN_PUSH_POS: Global<u32> = Global::new(0);
static BULK_IN_POP_FLAG: Global<u32> = Global::new(0);
const BI_START: u32 = 2;

// Send bulk packet to host with double buffering optimization
fn usb_send_bulk_in_double_buffer(data: *const u8, len: u8) -> i8 {
    if readl(&BULK_IN_POP_FLAG) != 0 {
        // No buffer space available
        return -1;
    }
    let ep = USB_CDC_EP_BULK_IN;
    let bipp = *BULK_IN_PUSH_POS.get();
    let bufnum = (bipp & 1) as usize;
    *BULK_IN_PUSH_POS.get() = bipp ^ 1;
    btable_write_packet(ep, bufnum, data, len as u32);
    writel(&BULK_IN_POP_FLAG, USB_EP_DTOG_RX);

    // Check if hardware needs to be notified
    let epr = epr_read(ep);
    if epr_is_dbuf_blocking(epr) && readl(&BULK_IN_POP_FLAG) != 0 {
        writel(&BULK_IN_POP_FLAG, 0);
        if bipp & BI_START != 0 {
            // Two packets are always sent when starting in double
            // buffering mode, so wait for second packet before starting.
            if bipp == (BI_START | 1) {
                *BULK_IN_PUSH_POS.get() = 0;
                if !CONFIG_MACH_AT32F403 {
                    writel(&BULK_IN_POP_FLAG, USB_EP_KIND); // Dummy flag
                }
                epr_write(ep, calc_epr_bits(epr, USB_EPTX_STAT, USB_EP_TX_VALID));
            }
        } else {
            epr_write(ep, calc_epr_bits(epr, 0, 0) | USB_EP_DTOG_RX);
        }
    }

    len as i8
}

// Send bulk usb packet to host
pub fn usb_send_bulk_in(data: *const u8, len: u8) -> i8 {
    if CONFIG_STM32_USB_DOUBLE_BUFFER_TX {
        return usb_send_bulk_in_double_buffer(data, len);
    }
    let ep = USB_CDC_EP_BULK_IN;
    let epr = epr_read(ep);
    if (epr & USB_EPTX_STAT) != USB_EP_TX_NAK {
        // No buffer space available
        return -1;
    }
    btable_write_packet(ep, BUFTX, data, len as u32);
    epr_write(ep, calc_epr_bits(epr, USB_EPTX_STAT, USB_EP_TX_VALID));
    len as i8
}

pub fn usb_read_ep0(data: *mut u8, max_len: u8) -> i8 {
    let ep = 0;
    let epr = epr_read(ep);
    if (epr & USB_EPRX_STAT) != USB_EP_RX_NAK {
        // No data ready
        return -1;
    }
    let count = btable_read_packet(ep, BUFRX, data, max_len as u32);
    epr_write(ep, calc_epr_bits(epr, USB_EPRX_STAT | USB_EPTX_STAT, USB_EP_RX_VALID | USB_EP_TX_NAK));
    count as i8
}

pub fn usb_read_ep0_setup(data: *mut u8, max_len: u8) -> i8 {
    usb_read_ep0(data, max_len)
}

pub fn usb_send_ep0(data: *const u8, len: u8) -> i8 {
    let ep = 0;
    let epr = epr_read(ep);
    if (epr & USB_EPRX_STAT) != USB_EP_RX_VALID {
        // Transfer interrupted
        return -2;
    }
    if (epr & USB_EPTX_STAT) != USB_EP_TX_NAK {
        // No buffer space available
        return -1;
    }
    btable_write_packet(ep, BUFTX, data, len as u32);
    epr_write(ep, calc_epr_bits(epr, USB_EPTX_STAT, USB_EP_TX_VALID));
    len as i8
}

pub fn usb_stall_ep0() {
    let ep = 0;
    let epr = epr_read(ep);
    epr_write(ep, calc_epr_bits(epr, USB_EPRX_STAT | USB_EPTX_STAT, USB_EP_RX_STALL | USB_EP_TX_STALL));
}

static SET_ADDRESS: Global<u8> = Global::new(0);

pub fn usb_set_address(addr: u8) {
    unsafe { write_volatile(SET_ADDRESS.ptr(), addr | USB_DADDR_EF as u8) };
    usb_send_ep0(core::ptr::null(), 0);
}

pub fn usb_set_configure() {
    let ep = USB_CDC_EP_BULK_OUT;
    *BULK_OUT_POP_COUNT.get() = 0;
    epr_write(ep, calc_epr_bits(epr_read(ep), USB_EPRX_STAT, USB_EP_RX_VALID));

    if CONFIG_STM32_USB_DOUBLE_BUFFER_TX {
        *BULK_IN_PUSH_POS.get() = BI_START;
        writel(&BULK_IN_POP_FLAG, 0);
    }
}

/****************************************************************
 * Setup and interrupts
 ****************************************************************/

// Configure interface after a USB reset event
fn usb_reset() {
    epr_write(0, USB_EP_CONTROL | USB_EP_RX_VALID | USB_EP_TX_NAK);

    epr_write(USB_CDC_EP_ACM, USB_CDC_EP_ACM as u32 | USB_EP_INTERRUPT | USB_EP_RX_NAK | USB_EP_TX_NAK);

    epr_write(
        USB_CDC_EP_BULK_OUT,
        USB_CDC_EP_BULK_OUT as u32 | USB_EP_BULK | USB_EP_KIND | USB_EP_RX_NAK | USB_EP_DTOG_TX,
    );
    *BULK_OUT_PUSH_FLAG.get() = USB_EP_DTOG_TX;

    let mut bi_epr_flags = USB_CDC_EP_BULK_IN as u32 | USB_EP_BULK | USB_EP_TX_NAK;
    if CONFIG_STM32_USB_DOUBLE_BUFFER_TX {
        bi_epr_flags |= USB_EP_KIND;
        *BULK_IN_POP_FLAG.get() = USB_EP_DTOG_RX;
    }
    epr_write(USB_CDC_EP_BULK_IN, bi_epr_flags);

    let usb = USBR();
    usb.CNTR.write((USB_CNTR_CTRM | USB_CNTR_RESETM) as _);
    usb.DADDR.write(USB_DADDR_EF as _);
}

// Main irq handler
#[unsafe(no_mangle)]
pub unsafe extern "C" fn USB_IRQHandler() {
    let usb = USBR();
    let istr = usb.ISTR.read() as u32;
    if istr & USB_ISTR_CTR != 0 {
        // Endpoint activity
        let ep = (istr & USB_ISTR_EP_ID) as usize;
        let epr = epr_read(ep);
        if ep == USB_CDC_EP_BULK_OUT {
            epr_write(ep, calc_epr_bits(epr, USB_EP_CTR_RX | USB_EP_CTR_TX, 0) | *BULK_OUT_PUSH_FLAG.get());
            *BULK_OUT_PUSH_FLAG.get() = 0;
            usb_notify_bulk_out();
        } else if ep == USB_CDC_EP_BULK_IN {
            let mut ne = calc_epr_bits(epr, USB_EP_CTR_RX | USB_EP_CTR_TX, 0);
            if CONFIG_STM32_USB_DOUBLE_BUFFER_TX {
                ne |= *BULK_IN_POP_FLAG.get();
                *BULK_IN_POP_FLAG.get() = 0;
            }
            epr_write(ep, ne);
            usb_notify_bulk_in();
        } else if ep == 0 {
            epr_write(ep, calc_epr_bits(epr, USB_EP_CTR_RX | USB_EP_CTR_TX, 0));
            usb_notify_ep0();
            let set_address = SET_ADDRESS.get();
            if epr & USB_EP_CTR_TX != 0 && *set_address != 0 {
                // Apply address after last "in" message transmitted
                usb.DADDR.write(*set_address as _);
                *set_address = 0;
            }
        }
    }
    if istr & USB_ISTR_RESET != 0 {
        // USB Reset
        usb.ISTR.write(!(USB_ISTR_RESET as u16) as _);
        usb_reset();
    }
}

decl_constant_str!("RESERVE_PINS_USB", "PA11,PA12");

// Initialize the usb controller
pub fn usb_init() -> R {
    if CONFIG_MACH_STM32F1 {
        // Pull the D+ pin low briefly to signal a new connection
        gpio_out_setup(GPIO('A', 12), 0)?;
        udelay(5000);
        gpio_in_setup(GPIO('A', 12), 0)?;
    }

    // Enable USB clock
    enable_pclock(USB_BASE);

    // Setup USB packet memory
    btable_configure();

    // Enable USB pullup
    #[cfg(has_USB_BCDR_DPPU)]
    USBR().BCDR.write(USB_BCDR_DPPU as _);

    // Reset usb controller and enable interrupts
    let usb = USBR();
    usb.CNTR.write(USB_CNTR_FRES as _);
    usb.DADDR.write(0);
    usb.CNTR.write(USB_CNTR_RESETM as _);
    usb.ISTR.write(0);
    armcm_enable_irq!(USB_IRQHandler, USBx_IRQn, 1);
    Ok(())
}
decl_init!(usb_init);
