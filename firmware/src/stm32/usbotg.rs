// Hardware interface to "USB OTG (on the go) controller" on stm32
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::internal::*;
use crate::armcm_enable_irq;
use crate::config::CONFIG_STM32_USB_DOUBLE_BUFFER_TX;
use crate::ctr::Global;
use crate::decl_constant_str;
use crate::decl_init;
use crate::generic::cmsis::{NVIC_DisableIRQ, NVIC_EnableIRQ};
use crate::generic::usb_cdc::{usb_notify_bulk_in, usb_notify_bulk_out, usb_notify_ep0};
use crate::generic::usb_cdc_ep::{USB_CDC_EP_ACM, USB_CDC_EP_ACM_SIZE, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_IN_SIZE, USB_CDC_EP_BULK_OUT, USB_CDC_EP_BULK_OUT_SIZE};
use crate::sched::R;

#[cfg(CONFIG_STM32_USB_PB14_PB15)]
mod pins {
    use super::*;
    pub const IS_OTG_HS: bool = true;
    pub const GPIO_D_NEG: u32 = GPIO('B', 14);
    pub const GPIO_D_POS: u32 = GPIO('B', 15);
    pub const GPIO_FUNC: u32 = GPIO_FUNCTION(12);
    decl_constant_str!("RESERVE_PINS_USB1", "PB14,PB15");
}
#[cfg(not(CONFIG_STM32_USB_PB14_PB15))]
mod pins {
    use super::*;
    pub const IS_OTG_HS: bool = crate::config::CONFIG_MACH_STM32H723;
    pub const GPIO_D_NEG: u32 = GPIO('A', 11);
    pub const GPIO_D_POS: u32 = GPIO('A', 12);
    pub const GPIO_FUNC: u32 = GPIO_FUNCTION(10);
    decl_constant_str!("RESERVE_PINS_USB", "PA11,PA12");
}
use pins::*;

// (the OTG_HS controller of the stm32h7 / PB14_PB15 boards comes with those)
const USB_PERIPH_BASE: u32 = USB_OTG_FS_PERIPH_BASE;
const OTG_IRQn: IRQn_Type = OTG_FS_IRQn;
const _: () = assert!(!IS_OTG_HS);

fn usb_irq_disable() {
    NVIC_DisableIRQ(OTG_IRQn);
}

fn usb_irq_enable() {
    NVIC_EnableIRQ(OTG_IRQn);
}

/****************************************************************
 * USB transfer memory
 ****************************************************************/

fn OTG() -> &'static USB_OTG_GlobalTypeDef {
    unsafe { &*(USB_PERIPH_BASE as *const USB_OTG_GlobalTypeDef) }
}
fn OTGD() -> &'static USB_OTG_DeviceTypeDef {
    unsafe { &*((USB_PERIPH_BASE + USB_OTG_DEVICE_BASE) as *const USB_OTG_DeviceTypeDef) }
}
fn EPFIFO(ep: u32) -> *mut u32 {
    (USB_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep << 12)) as *mut u32
}
fn EPIN(ep: u32) -> &'static USB_OTG_INEndpointTypeDef {
    unsafe { &*((USB_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep << 5)) as *const USB_OTG_INEndpointTypeDef) }
}
fn EPOUT(ep: u32) -> &'static USB_OTG_OUTEndpointTypeDef {
    unsafe { &*((USB_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep << 5)) as *const USB_OTG_OUTEndpointTypeDef) }
}

fn writel(addr: *mut u32, v: u32) {
    unsafe { core::ptr::write_volatile(addr, v) }
}
fn readl(addr: *mut u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr) }
}

// Setup the USB fifos
fn fifo_configure() {
    let otg = OTG();
    // Reserve memory for Rx fifo
    let sz = (4 * 1 + 6) + 4 * ((USB_CDC_EP_BULK_OUT_SIZE as u32 / 4) + 1) + (2 * 1);
    otg.GRXFSIZ.write(sz);

    // Tx fifos
    let (mut fpos, ep_size) = (sz, 0x10);
    otg.DIEPTXF0_HNPTXFSIZ.write((fpos << USB_OTG_TX0FSA_Pos) | (ep_size << USB_OTG_TX0FD_Pos));
    fpos += ep_size;

    otg.DIEPTXF[USB_CDC_EP_ACM - 1].write((fpos << USB_OTG_DIEPTXF_INEPTXSA_Pos) | (ep_size << USB_OTG_DIEPTXF_INEPTXFD_Pos));
    fpos += ep_size;

    otg.DIEPTXF[USB_CDC_EP_BULK_IN - 1].write((fpos << USB_OTG_DIEPTXF_INEPTXSA_Pos) | (ep_size << USB_OTG_DIEPTXF_INEPTXFD_Pos));
}

// Write a packet to a tx fifo
fn fifo_write_packet(ep: u32, mut src: *const u8, len: u32) -> i8 {
    let fifo = EPFIFO(ep);
    let epi = EPIN(ep);
    epi.DIEPINT.write(USB_OTG_DIEPINT_XFRC);
    epi.DIEPTSIZ.write(len | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
    epi.DIEPCTL.or(USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
    let mut count = len as i32;
    while count >= 4 {
        let data = unsafe { core::ptr::read_unaligned(src as *const u32) };
        writel(fifo, data);
        count -= 4;
        src = src.wrapping_add(4);
    }
    if count != 0 {
        let mut data = [0u8; 4];
        unsafe { core::ptr::copy_nonoverlapping(src, data.as_mut_ptr(), count as usize) };
        writel(fifo, u32::from_le_bytes(data));
    }
    len as i8
}

// Write a packet to a tx fifo (optimized for already aligned data)
fn fifo_write_packet_fast(ep: u32, src: &[u32], len: u32) -> i32 {
    let fifo = EPFIFO(ep);
    let epi = EPIN(ep);
    let ctl = epi.DIEPCTL.read();
    if ctl & USB_OTG_DIEPCTL_EPENA != 0 {
        return -1;
    }
    epi.DIEPINT.write(USB_OTG_DIEPINT_XFRC);
    epi.DIEPTSIZ.write(len | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
    epi.DIEPCTL.write(ctl | USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);
    for s in src.iter().take(len.div_ceil(4) as usize) {
        writel(fifo, *s);
    }
    0
}

// Read a packet from the rx queue
fn fifo_read_packet(mut dest: *mut u8, max_len: u32) -> i8 {
    // Transfer data
    let fifo = EPFIFO(0);
    let grx = OTG().GRXSTSP.read();
    let bcnt = (grx & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
    let xfer = if bcnt > max_len { max_len } else { bcnt };
    let mut count = xfer;
    while count >= 4 {
        let data = readl(fifo);
        unsafe { core::ptr::write_unaligned(dest as *mut u32, data) };
        count -= 4;
        dest = dest.wrapping_add(4);
    }
    if count != 0 {
        let data = readl(fifo).to_le_bytes();
        unsafe { core::ptr::copy_nonoverlapping(data.as_ptr(), dest, count as usize) };
    }
    let extra = bcnt.div_ceil(4) - xfer.div_ceil(4);
    for _ in 0..extra {
        readl(fifo);
    }
    xfer as i8
}

// Reenable packet reception if it got disabled by controller
fn enable_rx_endpoint(ep: u32) {
    let epo = EPOUT(ep);
    let ctl = epo.DOEPCTL.read();
    if ctl & USB_OTG_DOEPCTL_EPENA == 0 || ctl & USB_OTG_DOEPCTL_NAKSTS != 0 {
        epo.DOEPTSIZ.write(64 | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos));
        epo.DOEPCTL.write(ctl | USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);
    }
}

// Inspect the next packet on the rx queue
fn peek_rx_queue(ep: u32) -> u32 {
    let otg = OTG();
    loop {
        let sts = otg.GINTSTS.read();
        if sts & USB_OTG_GINTSTS_RXFLVL == 0 {
            // No packet ready
            return 0;
        }
        let grx = otg.GRXSTSR.read();
        let grx_ep = grx & USB_OTG_GRXSTSP_EPNUM_Msk;
        let pktsts = (grx & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
        let bulk_out = USB_CDC_EP_BULK_OUT as u32;
        if (grx_ep == 0 || grx_ep == bulk_out) && (pktsts == 2 || pktsts == 4 || pktsts == 6) {
            // A packet is ready
            if grx_ep != ep {
                return 0;
            }
            return grx;
        }
        if (grx_ep != 0 && grx_ep != bulk_out) || (pktsts != 1 && pktsts != 3 && pktsts != 4) {
            // Rx queue has bogus value - just pop it
            otg.GRXSTSP.read();
            continue;
        }
        // Discard informational entries from queue
        fifo_read_packet(core::ptr::null_mut(), 0);
    }
}

/****************************************************************
 * USB interface
 ****************************************************************/

pub fn usb_read_bulk_out(data: *mut u8, max_len: u8) -> i8 {
    usb_irq_disable();
    let grx = peek_rx_queue(USB_CDC_EP_BULK_OUT as u32);
    if grx == 0 {
        // Wait for packet
        OTG().GINTMSK.or(USB_OTG_GINTMSK_RXFLVLM);
        usb_irq_enable();
        return -1;
    }
    let ret = fifo_read_packet(data, max_len as u32);
    enable_rx_endpoint(USB_CDC_EP_BULK_OUT as u32);
    usb_irq_enable();
    ret
}

// Storage for "bulk in" transmissions for a kind of manual "double buffering"
struct TxBuf {
    len: u32,
    buf: [u32; USB_CDC_EP_BULK_IN_SIZE / 4],
}
static TX_BUF: Global<TxBuf> = Global::new(TxBuf { len: 0, buf: [0; USB_CDC_EP_BULK_IN_SIZE / 4] });

pub fn usb_send_bulk_in(data: *const u8, len: u8) -> i8 {
    usb_irq_disable();
    let ctl = EPIN(USB_CDC_EP_BULK_IN as u32).DIEPCTL.read();
    if ctl & USB_OTG_DIEPCTL_USBAEP == 0 {
        // Controller not enabled - discard data
        usb_irq_enable();
        return len as i8;
    }
    let tx = TX_BUF.get();
    let dbuf_busy = CONFIG_STM32_USB_DOUBLE_BUFFER_TX && tx.len != 0;
    if ctl & USB_OTG_DIEPCTL_EPENA != 0 || dbuf_busy {
        if !CONFIG_STM32_USB_DOUBLE_BUFFER_TX || tx.len != 0 || len == 0 {
            // Wait for space to transmit
            OTGD().DAINTMSK.or(1 << USB_CDC_EP_BULK_IN);
            usb_irq_enable();
            return -1;
        }
        // Buffer next packet for transmission from irq handler
        let len = if len as usize > USB_CDC_EP_BULK_IN_SIZE { USB_CDC_EP_BULK_IN_SIZE as u8 } else { len };
        let blocks = (len as usize).div_ceil(4);
        tx.buf[blocks - 1] = 0;
        unsafe { core::ptr::copy_nonoverlapping(data, tx.buf.as_mut_ptr() as *mut u8, len as usize) };
        tx.len = len as u32;
        OTGD().DAINTMSK.or(1 << USB_CDC_EP_BULK_IN);
        usb_irq_enable();
        return len as i8;
    }
    let ret = fifo_write_packet(USB_CDC_EP_BULK_IN as u32, data, len as u32);
    usb_irq_enable();
    ret
}

pub fn usb_read_ep0(data: *mut u8, max_len: u8) -> i8 {
    usb_irq_disable();
    let grx = peek_rx_queue(0);
    if grx == 0 {
        // Wait for packet
        OTG().GINTMSK.or(USB_OTG_GINTMSK_RXFLVLM);
        usb_irq_enable();
        return -1;
    }
    let pktsts = (grx & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
    if pktsts != 2 {
        // Transfer interrupted
        usb_irq_enable();
        return -2;
    }
    let ret = fifo_read_packet(data, max_len as u32);
    enable_rx_endpoint(0);
    usb_irq_enable();
    ret
}

static SETUP_BUF: Global<[u8; 8]> = Global::new([0; 8]);

pub fn usb_read_ep0_setup(data: *mut u8, max_len: u8) -> i8 {
    let setup_buf = SETUP_BUF.get();
    usb_irq_disable();
    loop {
        let grx = peek_rx_queue(0);
        if grx == 0 {
            // Wait for packet
            OTG().GINTMSK.or(USB_OTG_GINTMSK_RXFLVLM);
            usb_irq_enable();
            return -1;
        }
        let pktsts = (grx & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos;
        if pktsts == 6 {
            // Store setup packet
            fifo_read_packet(setup_buf.as_mut_ptr(), setup_buf.len() as u32);
        } else {
            // Discard other packets
            fifo_read_packet(core::ptr::null_mut(), 0);
        }
        if pktsts == 4 {
            // Setup complete
            break;
        }
    }
    let ctl = EPIN(0).DIEPCTL.read();
    if ctl & USB_OTG_DIEPCTL_EPENA != 0 {
        // Flush any pending tx packets
        EPIN(0).DIEPCTL.write(ctl | USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);
        while EPIN(0).DIEPCTL.read() & USB_OTG_DIEPCTL_EPENA != 0 {}
        OTG().GRSTCTL.write(USB_OTG_GRSTCTL_TXFFLSH);
        while OTG().GRSTCTL.read() & USB_OTG_GRSTCTL_TXFFLSH != 0 {}
    }
    enable_rx_endpoint(0);
    EPOUT(0).DOEPINT.write(USB_OTG_DOEPINT_STUP);
    usb_irq_enable();
    // Return previously read setup packet
    unsafe { core::ptr::copy_nonoverlapping(setup_buf.as_ptr(), data, max_len as usize) };
    max_len as i8
}

pub fn usb_send_ep0(data: *const u8, len: u8) -> i8 {
    usb_irq_disable();
    let grx = peek_rx_queue(0);
    if grx != 0 {
        // Transfer interrupted
        usb_irq_enable();
        return -2;
    }
    if EPIN(0).DIEPCTL.read() & USB_OTG_DIEPCTL_EPENA != 0 {
        // Wait for space to transmit
        OTG().GINTMSK.or(USB_OTG_GINTMSK_RXFLVLM);
        OTGD().DAINTMSK.or(1 << 0);
        usb_irq_enable();
        return -1;
    }
    let ret = fifo_write_packet(0, data, len as u32);
    usb_irq_enable();
    ret
}

pub fn usb_stall_ep0() {
    usb_irq_disable();
    EPIN(0).DIEPCTL.or(USB_OTG_DIEPCTL_STALL);
    usb_notify_ep0(); // XXX - wake from main usb_cdc.c code?
    usb_irq_enable();
}

pub fn usb_set_address(addr: u8) {
    let otgd = OTGD();
    otgd.DCFG.write((otgd.DCFG.read() & !USB_OTG_DCFG_DAD_Msk) | ((addr as u32) << USB_OTG_DCFG_DAD_Pos));
    usb_send_ep0(core::ptr::null(), 0);
    usb_notify_ep0();
}

pub fn usb_set_configure() {
    usb_irq_disable();
    // Configure and enable USB_CDC_EP_ACM
    let mut epi = EPIN(USB_CDC_EP_ACM as u32);
    epi.DIEPTSIZ.write(USB_CDC_EP_ACM_SIZE as u32 | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
    epi.DIEPCTL.write(
        USB_OTG_DIEPCTL_SNAK
            | USB_OTG_DIEPCTL_USBAEP
            | (0x03 << USB_OTG_DIEPCTL_EPTYP_Pos)
            | USB_OTG_DIEPCTL_SD0PID_SEVNFRM
            | ((USB_CDC_EP_ACM as u32) << USB_OTG_DIEPCTL_TXFNUM_Pos)
            | ((USB_CDC_EP_ACM_SIZE as u32) << USB_OTG_DIEPCTL_MPSIZ_Pos),
    );

    // Configure and enable USB_CDC_EP_BULK_OUT
    let epo = EPOUT(USB_CDC_EP_BULK_OUT as u32);
    epo.DOEPTSIZ.write(64 | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos));
    epo.DOEPCTL.write(
        USB_OTG_DOEPCTL_CNAK
            | USB_OTG_DOEPCTL_USBAEP
            | USB_OTG_DOEPCTL_EPENA
            | (0x02 << USB_OTG_DOEPCTL_EPTYP_Pos)
            | USB_OTG_DOEPCTL_SD0PID_SEVNFRM
            | ((USB_CDC_EP_BULK_OUT_SIZE as u32) << USB_OTG_DOEPCTL_MPSIZ_Pos),
    );

    // Configure and flush USB_CDC_EP_BULK_IN
    epi = EPIN(USB_CDC_EP_BULK_IN as u32);
    epi.DIEPTSIZ.write(USB_CDC_EP_BULK_IN_SIZE as u32 | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
    epi.DIEPCTL.write(
        USB_OTG_DIEPCTL_SNAK
            | USB_OTG_DIEPCTL_EPDIS
            | USB_OTG_DIEPCTL_USBAEP
            | (0x02 << USB_OTG_DIEPCTL_EPTYP_Pos)
            | USB_OTG_DIEPCTL_SD0PID_SEVNFRM
            | ((USB_CDC_EP_BULK_IN as u32) << USB_OTG_DIEPCTL_TXFNUM_Pos)
            | ((USB_CDC_EP_BULK_IN_SIZE as u32) << USB_OTG_DIEPCTL_MPSIZ_Pos),
    );
    while epi.DIEPCTL.read() & USB_OTG_DIEPCTL_EPENA != 0 {}
    OTG().GRSTCTL.write(((USB_CDC_EP_BULK_IN as u32) << USB_OTG_GRSTCTL_TXFNUM_Pos) | USB_OTG_GRSTCTL_TXFFLSH);
    while OTG().GRSTCTL.read() & USB_OTG_GRSTCTL_TXFFLSH != 0 {}
    if CONFIG_STM32_USB_DOUBLE_BUFFER_TX {
        TX_BUF.get().len = 0;
    }
    usb_irq_enable();
}

/****************************************************************
 * Setup and interrupts
 ****************************************************************/

// Main irq handler
#[unsafe(no_mangle)]
pub unsafe extern "C" fn OTG_FS_IRQHandler() {
    let (otg, otgd) = (OTG(), OTGD());
    let sts = otg.GINTSTS.read();
    if sts & USB_OTG_GINTSTS_RXFLVL != 0 {
        // Received data - disable irq and notify endpoint
        otg.GINTMSK.and(!USB_OTG_GINTMSK_RXFLVLM);
        let grx = otg.GRXSTSR.read();
        let ep = grx & USB_OTG_GRXSTSP_EPNUM_Msk;
        if ep == 0 {
            usb_notify_ep0();
        } else {
            usb_notify_bulk_out();
        }
    }
    if sts & USB_OTG_GINTSTS_IEPINT != 0 {
        // Can transmit data - disable irq and notify endpoint
        let (daint, msk) = (otgd.DAINT.read(), otgd.DAINTMSK.read());
        let pend = daint & msk;
        otgd.DAINTMSK.write(msk & !daint);
        if pend & (1 << 0) != 0 {
            usb_notify_ep0();
        }
        if pend & (1 << USB_CDC_EP_BULK_IN) != 0 {
            usb_notify_bulk_in();
            let tx = TX_BUF.get();
            if CONFIG_STM32_USB_DOUBLE_BUFFER_TX && tx.len != 0 {
                let ret = fifo_write_packet_fast(USB_CDC_EP_BULK_IN as u32, &tx.buf, tx.len);
                if ret == 0 {
                    tx.len = 0;
                }
            }
        }
    }
}

// Initialize the usb controller
pub fn usb_init() -> R {
    let (otg, otgd) = (OTG(), OTGD());
    // Enable USB clock
    // (stm32h7: its PWR / AHB1ENR setup comes with that board)
    unsafe { &*RCC }.AHB2ENR.or(RCC_AHB2ENR_OTGFSEN);
    while otg.GRSTCTL.read() & USB_OTG_GRSTCTL_AHBIDL == 0 {}

    // Configure USB in full-speed device mode
    otg.GUSBCFG.write(USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | (6 << USB_OTG_GUSBCFG_TRDT_Pos));
    otgd.DCFG.or(3 << USB_OTG_DCFG_DSPD_Pos);
    #[cfg(any(CONFIG_MACH_STM32F446, CONFIG_MACH_STM32H7, CONFIG_MACH_STM32F7))]
    otg.GOTGCTL.write(USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL);
    #[cfg(not(any(CONFIG_MACH_STM32F446, CONFIG_MACH_STM32H7, CONFIG_MACH_STM32F7)))]
    otg.GCCFG.or(USB_OTG_GCCFG_NOVBUSSENS);

    // Route pins
    gpio_peripheral(GPIO_D_NEG, GPIO_FUNC, 0);
    gpio_peripheral(GPIO_D_POS, GPIO_FUNC, 0);

    // Setup USB packet memory
    fifo_configure();

    // Configure and enable ep0
    let mpsize_ep0 = 2;
    let (epi, epo) = (EPIN(0), EPOUT(0));
    epi.DIEPCTL.write(mpsize_ep0 | USB_OTG_DIEPCTL_SNAK);
    epo.DOEPTSIZ.write(64 | (1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos));
    epo.DOEPCTL.write(mpsize_ep0 | USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_SNAK);

    // Enable interrupts
    otgd.DIEPMSK.write(USB_OTG_DIEPMSK_XFRCM);
    otg.GINTMSK.write(USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_IEPINT);
    otg.GAHBCFG.write(USB_OTG_GAHBCFG_GINT);
    armcm_enable_irq!(OTG_FS_IRQHandler, OTG_IRQn, 1);

    // Enable USB
    otg.GCCFG.or(USB_OTG_GCCFG_PWRDWN);
    otgd.DCTL.write(0);
    Ok(())
}
decl_init!(usb_init);
