// Hardware interface to USB on rp2040
//
// Copyright (C) 2021-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time};
use crate::ctr::Global;
use crate::generic::usb_cdc::{usb_notify_bulk_in, usb_notify_bulk_out, usb_notify_ep0};
use crate::generic::usb_cdc_ep::{USB_CDC_EP_ACM, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_OUT};
use crate::generic::usbstd::{USB_ENDPOINT_XFER_BULK, USB_ENDPOINT_XFER_INT};
use crate::sched::{sched_check_wake, sched_wake_task, TaskWake, R};
use crate::{armcm_enable_irq, decl_init, decl_task};
use core::mem::size_of;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{compiler_fence, Ordering};

fn barrier() {
    compiler_fence(Ordering::SeqCst);
}

fn usb() -> &'static usb_hw_t {
    unsafe { &*usb_hw }
}

fn dpram() -> &'static usb_device_dpram_t {
    unsafe { &*usb_dpram }
}

/****************************************************************
 * USB transfer memory
 ****************************************************************/

const DPBUF_SIZE: u32 = 64;

// Get the offset of a given endpoint's base buffer
fn usb_buf_offset(ep: u32) -> u32 {
    0x100 + ep * DPBUF_SIZE * 2
}

// Obtain a pointer to an endpoint buffer
fn usb_buf_addr(ep: u32, bufnum: u32) -> *mut u8 {
    (usb_dpram as *mut u8).wrapping_add((usb_buf_offset(ep) + bufnum * DPBUF_SIZE) as usize)
}

// Return a pointer to the ep_buf_ctrl register for an endpoint
fn lookup_epbufctrl(ep: u32, is_rx: bool, bufnum: u32) -> *mut u16 {
    let c = &dpram().ep_buf_ctrl[ep as usize];
    let epbp = if is_rx { c.out.ptr() } else { c.r#in.ptr() } as *mut u16;
    epbp.wrapping_add(bufnum as usize)
}

// Determine the next transfer PID id from the last PID
fn next_data_pid(epb: u32) -> u32 {
    (epb ^ USB_BUF_CTRL_DATA1_PID) & USB_BUF_CTRL_DATA1_PID
}

// Extract the number of bytes in an rx buffer
fn get_rx_count(epb: u32, max_len: u32) -> u32 {
    let c = epb & USB_BUF_CTRL_LEN_MASK;
    if c > max_len {
        max_len
    } else {
        c
    }
}

// Memcpy using 8-bit read/write (system memcpy may make unaligned accesses)
fn dpram_memcpy(dest: *mut u8, src: *const u8, n: usize) {
    for i in 0..n {
        unsafe { write_volatile(dest.add(i), *src.add(i)) };
    }
}

// Memset using only 8-bit writes (system memcpy may make unaligned accesses)
fn dpram_memset(s: *mut u8, c: u8, n: usize) {
    for i in 0..n {
        unsafe { write_volatile(s.add(i), c) };
    }
}

/****************************************************************
 * Interface
 ****************************************************************/

static BULK_OUT_PUSH_COUNT: Global<u32> = Global::new(0);

pub fn usb_read_bulk_out(data: *mut u8, max_len: u8) -> i8 {
    // Check if there is a packet ready
    let bopc = *BULK_OUT_PUSH_COUNT.get();
    let bufnum = bopc & 1;
    let ep = USB_CDC_EP_BULK_OUT as u32;
    let epbp = lookup_epbufctrl(ep, true, bufnum);
    let epb = unsafe { read_volatile(epbp) } as u32;
    if epb & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL) != USB_BUF_CTRL_FULL {
        return -1;
    }
    // Determine the next packet header
    *BULK_OUT_PUSH_COUNT.get() = bopc.wrapping_add(1);
    let pid = if bufnum != 0 { USB_BUF_CTRL_DATA1_PID } else { 0 };
    let new_epb = USB_BUF_CTRL_LAST | pid | DPBUF_SIZE;
    unsafe { write_volatile(epbp, new_epb as u16) };
    barrier();
    // Copy the packet to the given buffer
    let c = get_rx_count(epb, max_len as u32);
    dpram_memcpy(data, usb_buf_addr(ep, bufnum), c as usize);
    // Notify the USB hardware that the space is now available
    barrier();
    unsafe { write_volatile(epbp, (new_epb | USB_BUF_CTRL_AVAIL) as u16) };
    c as i8
}

static BULK_IN_POP_COUNT: Global<u32> = Global::new(0);

pub fn usb_send_bulk_in(data: *const u8, len: u8) -> i8 {
    // Check if there is room for this packet
    let bipc = *BULK_IN_POP_COUNT.get();
    let bufnum = bipc & 1;
    let ep = USB_CDC_EP_BULK_IN as u32;
    let epbp = lookup_epbufctrl(ep, false, bufnum);
    let epb = unsafe { read_volatile(epbp) } as u32;
    if epb & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL) != 0 {
        return -1;
    }
    // Determine the next packet header
    *BULK_IN_POP_COUNT.get() = bipc.wrapping_add(1);
    let pid = if bufnum != 0 { USB_BUF_CTRL_DATA1_PID } else { 0 };
    let new_epb = USB_BUF_CTRL_FULL | USB_BUF_CTRL_LAST | pid | len as u32;
    unsafe { write_volatile(epbp, new_epb as u16) };
    barrier();
    // Copy the packet to the hw buffer
    dpram_memcpy(usb_buf_addr(ep, bufnum), data, len as usize);
    // Inform the USB hardware of the available packet
    barrier();
    unsafe { write_volatile(epbp, (new_epb | USB_BUF_CTRL_AVAIL) as u16) };
    len as i8
}

pub fn usb_read_ep0_setup(data: *mut u8, max_len: u8) -> i8 {
    if usb().intr.read() & USB_INTR_SETUP_REQ_BITS == 0 {
        usb().inte.write(USB_INTE_BUFF_STATUS_BITS | USB_INTE_SETUP_REQ_BITS | USB_INTE_BUS_RESET_BITS);
        return -1;
    }
    dpram().ep_buf_ctrl[0].r#in.write(0);
    dpram().ep_buf_ctrl[0].out.write(USB_BUF_CTRL_DATA1_PID | USB_BUF_CTRL_LAST | USB_BUF_CTRL_AVAIL | DPBUF_SIZE);
    usb().sie_status.write(USB_SIE_STATUS_SETUP_REC_BITS);
    barrier();
    dpram_memcpy(data, dpram().setup_packet.as_ptr() as *const u8, max_len as usize);
    barrier();
    if usb().intr.read() & USB_INTR_SETUP_REQ_BITS != 0 {
        // Raced with next setup packet
        usb_notify_ep0();
        return -1;
    }
    max_len as i8
}

pub fn usb_read_ep0(data: *mut u8, max_len: u8) -> i8 {
    // Check if there is a packet ready
    let ep = 0;
    if usb().intr.read() & USB_INTR_SETUP_REQ_BITS != 0 {
        // Early end of transmission
        return -2;
    }
    let epbp = lookup_epbufctrl(ep, true, 0);
    let epb = unsafe { read_volatile(epbp) } as u32;
    if epb & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL) != USB_BUF_CTRL_FULL {
        return -1;
    }
    // Determine the next packet header
    let new_epb = USB_BUF_CTRL_LAST | next_data_pid(epb) | DPBUF_SIZE;
    unsafe { write_volatile(epbp, new_epb as u16) };
    barrier();
    // Copy the packet to the given buffer
    let c = get_rx_count(epb, max_len as u32);
    dpram_memcpy(data, usb_buf_addr(ep, 0), c as usize);
    // Notify the USB hardware that the space is now available
    barrier();
    unsafe { write_volatile(epbp, (new_epb | USB_BUF_CTRL_AVAIL) as u16) };
    c as i8
}

pub fn usb_send_ep0(data: *const u8, len: u8) -> i8 {
    // Check if there is room for this packet
    let ep = 0;
    if usb().intr.read() & USB_INTR_SETUP_REQ_BITS != 0 || usb().buf_status.read() & 2 != 0 {
        // Early end of transmission
        return -2;
    }
    let epbp = lookup_epbufctrl(ep, false, 0);
    let epb = unsafe { read_volatile(epbp) } as u32;
    if epb & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL) != 0 {
        return -1;
    }
    // Determine the next packet header
    let pid = next_data_pid(epb);
    let new_epb = USB_BUF_CTRL_FULL | USB_BUF_CTRL_LAST | pid | len as u32;
    unsafe { write_volatile(epbp, new_epb as u16) };
    barrier();
    // Copy the packet to the hw buffer
    dpram_memcpy(usb_buf_addr(ep, 0), data, len as usize);
    // Inform the USB hardware of the available packet
    barrier();
    unsafe { write_volatile(epbp, (new_epb | USB_BUF_CTRL_AVAIL) as u16) };
    len as i8
}

pub fn usb_stall_ep0() {
    dpram().ep_buf_ctrl[0].r#in.write(0);
    dpram().ep_buf_ctrl[0].out.write(0);
    usb().ep_stall_arm.write(USB_EP_STALL_ARM_BITS);
    dpram().ep_buf_ctrl[0].r#in.write(USB_BUF_CTRL_STALL);
    dpram().ep_buf_ctrl[0].out.write(USB_BUF_CTRL_STALL);
    usb_notify_ep0();
}

static SET_ADDRESS: Global<u8> = Global::new(0);

pub fn usb_set_address(addr: u8) {
    unsafe { write_volatile(SET_ADDRESS.ptr(), addr) };
    usb_send_ep0(core::ptr::null(), 0);
}

pub fn usb_set_configure() {
    *BULK_IN_POP_COUNT.get() = 0;
    dpram().ep_buf_ctrl[USB_CDC_EP_BULK_IN].r#in.write(0);

    *BULK_OUT_PUSH_COUNT.get() = 0;
    let epb0 = USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LAST | DPBUF_SIZE;
    let epb1 = epb0 | USB_BUF_CTRL_DATA1_PID;
    dpram().ep_buf_ctrl[USB_CDC_EP_BULK_OUT].out.write(epb0 | (epb1 << 16));
}

/****************************************************************
 * USB Errata workaround
 ****************************************************************/

// The rp2040 USB has an errata causing it to sometimes not connect
// after a reset.  The following code has extracts from the PICO SDK.

static NEED_ERRATA: Global<u8> = Global::new(0);
static USB_ERRATA_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

// Workaround for rp2040-e5 errata
pub fn usb_errata_task() -> R {
    if !sched_check_wake(USB_ERRATA_WAKE.ptr()) {
        return Ok(());
    }

    if usb().sie_status.read() & USB_SIE_STATUS_CONNECTED_BITS != 0 {
        // Already connected - workaround not needed
        return Ok(());
    }

    // Wait for not in SE0 state
    if usb().sie_status.read() & USB_SIE_STATUS_LINE_STATE_BITS == 0 {
        sched_wake_task(USB_ERRATA_WAKE.ptr());
        return Ok(());
    }

    // Backup GPIO15 pad state
    let dp = 15;
    let io = unsafe { &(*iobank0_hw).io[dp] };
    let pad = unsafe { &(*padsbank0_hw).io[dp] };
    let gpio_ctrl_prev = io.ctrl.read();
    let pad_ctrl_prev = pad.read();

    // Enable bus keep
    hw_write_masked(pad, PADS_BANK0_GPIO15_PUE_BITS | PADS_BANK0_GPIO15_PDE_BITS, PADS_BANK0_GPIO15_PUE_BITS | PADS_BANK0_GPIO15_PDE_BITS);
    // Disable pad output
    hw_write_masked(&io.ctrl, 0x2 << IO_BANK0_GPIO15_CTRL_OEOVER_LSB, IO_BANK0_GPIO15_CTRL_OEOVER_BITS);
    // Enable USB debug muxing function
    hw_write_masked(&io.ctrl, 8 << IO_BANK0_GPIO15_CTRL_FUNCSEL_LSB, IO_BANK0_GPIO15_CTRL_FUNCSEL_BITS);
    // Set input override
    hw_write_masked(&io.ctrl, 0x3 << IO_BANK0_GPIO15_CTRL_INOVER_LSB, IO_BANK0_GPIO15_CTRL_INOVER_BITS);
    // PHY pullups need to stay on
    hw_set_alias(usb_hw).phy_direct.write(USB_USBPHY_DIRECT_DP_PULLUP_EN_BITS);
    hw_set_alias(usb_hw).phy_direct_override.write(USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN_BITS);
    // Switch from USB PHY to GPIO PHY, now with J forced
    usb().muxing.write(USB_USB_MUXING_TO_DIGITAL_PAD_BITS | USB_USB_MUXING_SOFTCON_BITS);

    // Wait 1ms
    let mut endtime = timer_read_time().wrapping_add(timer_from_us(1000));
    while timer_is_before(timer_read_time(), endtime) {}

    // Verify in connected state
    endtime = endtime.wrapping_add(timer_from_us(1000));
    loop {
        if usb().sie_status.read() & USB_SIE_STATUS_CONNECTED_BITS != 0 {
            break;
        }
        if timer_is_before(endtime, timer_read_time()) {
            // Something went wrong - restore state and continue anyway
            break;
        }
    }

    // Switch back to USB phy
    usb().muxing.write(USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS);
    // Unset PHY pullup overrides
    hw_clear_alias(usb_hw).phy_direct_override.write(USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN_BITS);

    // Restore GPIO control states
    io.ctrl.write(gpio_ctrl_prev);
    pad.write(pad_ctrl_prev);
    Ok(())
}
decl_task!(usb_errata_task);

/****************************************************************
 * Setup and interrupts
 ****************************************************************/

#[unsafe(no_mangle)]
pub unsafe extern "C" fn USB_Handler() {
    let ints = usb().ints.read();
    if ints & USB_INTS_SETUP_REQ_BITS != 0 {
        usb().inte.write(USB_INTE_BUFF_STATUS_BITS | USB_INTE_BUS_RESET_BITS);
        usb_notify_ep0();
    }
    if ints & USB_INTS_BUFF_STATUS_BITS != 0 {
        let buf_status = usb().buf_status.read();
        usb().buf_status.write(buf_status);
        if buf_status & (1 << (USB_CDC_EP_BULK_OUT * 2 + 1)) != 0 {
            usb_notify_bulk_out();
        }
        if buf_status & (1 << (USB_CDC_EP_BULK_IN * 2)) != 0 {
            usb_notify_bulk_in();
        }
        if buf_status & 3 != 0 {
            usb_notify_ep0();
            let set_address = SET_ADDRESS.get();
            if buf_status & 1 != 0 && *set_address != 0 {
                usb().dev_addr_ctrl.write(*set_address as u32);
                *set_address = 0;
            }
        }
    }
    if ints & USB_INTS_BUS_RESET_BITS != 0 {
        usb().dev_addr_ctrl.write(0);
        usb().sie_status.write(USB_SIE_STATUS_BUS_RESET_BITS);
        if *NEED_ERRATA.get() != 0 {
            sched_wake_task(USB_ERRATA_WAKE.ptr());
        }
    }
}

fn endpoint_setup() {
    // ACM
    let ep_acm =
        EP_CTRL_ENABLE_BITS | usb_buf_offset(USB_CDC_EP_ACM as u32) | ((USB_ENDPOINT_XFER_INT as u32) << EP_CTRL_BUFFER_TYPE_LSB);
    dpram().ep_ctrl[USB_CDC_EP_ACM - 1].r#in.write(ep_acm);
    // BULK
    let ep_out = EP_CTRL_ENABLE_BITS
        | usb_buf_offset(USB_CDC_EP_BULK_OUT as u32)
        | EP_CTRL_DOUBLE_BUFFERED_BITS
        | EP_CTRL_INTERRUPT_PER_BUFFER
        | ((USB_ENDPOINT_XFER_BULK as u32) << EP_CTRL_BUFFER_TYPE_LSB);
    dpram().ep_ctrl[USB_CDC_EP_BULK_OUT - 1].out.write(ep_out);
    let ep_in = EP_CTRL_ENABLE_BITS
        | usb_buf_offset(USB_CDC_EP_BULK_IN as u32)
        | EP_CTRL_DOUBLE_BUFFERED_BITS
        | EP_CTRL_INTERRUPT_PER_BUFFER
        | ((USB_ENDPOINT_XFER_BULK as u32) << EP_CTRL_BUFFER_TYPE_LSB);
    dpram().ep_ctrl[USB_CDC_EP_BULK_IN - 1].r#in.write(ep_in);
    dpram().ep_buf_ctrl[USB_CDC_EP_BULK_IN].r#in.write(USB_BUF_CTRL_FULL);
}

pub fn usbserial_init() -> R {
    // Configure usb clock
    enable_pclock(RESETS_RESET_USBCTRL_BITS);

    // Setup shared memory area
    dpram_memset(usb_dpram as *mut u8, 0, size_of::<usb_device_dpram_t>());
    endpoint_setup();

    // Enable USB in device mode
    usb().muxing.write(USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS);
    usb().pwr.write(USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS);
    usb().main_ctrl.write(USB_MAIN_CTRL_CONTROLLER_EN_BITS);

    // Check if usb errata workaround needed
    enable_pclock(RESETS_RESET_SYSINFO_BITS);
    let chip_id = unsafe { read_volatile((SYSINFO_BASE + SYSINFO_CHIP_ID_OFFSET) as usize as *const u32) };
    let version = (chip_id & SYSINFO_CHIP_ID_REVISION_BITS) >> SYSINFO_CHIP_ID_REVISION_LSB;
    *NEED_ERRATA.get() = (version == 1) as u8;

    // Enable irqs
    usb().sie_ctrl.write(USB_SIE_CTRL_EP0_INT_1BUF_BITS);
    usb().inte.write(USB_INTE_BUFF_STATUS_BITS | USB_INTE_SETUP_REQ_BITS | USB_INTE_BUS_RESET_BITS);
    armcm_enable_irq!(USB_Handler, USBCTRL_IRQ_IRQn, 1);

    // Enable USB pullup
    usb().sie_ctrl.write(USB_SIE_CTRL_EP0_INT_1BUF_BITS | USB_SIE_CTRL_PULLUP_EN_BITS);
    Ok(())
}
decl_init!(usbserial_init);
