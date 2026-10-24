// Support for Linux "gs_usb" CANbus adapter emulation
//
// Copyright (C) 2018-2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, dead_code)]
use crate::board::{
    canhw_send, timer_from_us, timer_is_before, timer_read_time, usb_read_bulk_out, usb_read_ep0, usb_read_ep0_setup, usb_send_bulk_in,
    usb_send_ep0, usb_set_address, usb_set_configure, usb_stall_ep0, usbserial_get_serialid,
};
use crate::config::{CONFIG_USB_MANUFACTURER, CONFIG_USB_PRODUCT, CONFIG_USB_SERIAL_NUMBER, CONFIG_USB_SERIAL_NUMBER_CHIPID};
use crate::ctr::Global;
use crate::generic::canbus::CanbusMsg;
use crate::generic::canserial::{canserial_notify_tx, canserial_process_data, CANBUS_ID_ADMIN};
use crate::generic::usb_cdc_ep::{USB_CDC_EP0_SIZE, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_IN_SIZE, USB_CDC_EP_BULK_OUT, USB_CDC_EP_BULK_OUT_SIZE};
use crate::generic::usbstd::*;
use crate::sched::{sched_check_wake, sched_wake_task, TaskWake, R};
use crate::{decl_constant, decl_shutdown, decl_task, sendf};
use core::mem::size_of;
use core::ptr::{addr_of_mut, read_volatile};

decl_constant!("CANBUS_BRIDGE", 1);

/****************************************************************
 * Linux "gs_usb" definitions
 ****************************************************************/

const USB_GSUSB_1_VENDOR_ID: u16 = 0x1d50;
const USB_GSUSB_1_PRODUCT_ID: u16 = 0x606f;

// enum gs_usb_breq
const GS_USB_BREQ_HOST_FORMAT: u8 = 0;
const GS_USB_BREQ_BITTIMING: u8 = 1;
const GS_USB_BREQ_MODE: u8 = 2;
const GS_USB_BREQ_BERR: u8 = 3;
const GS_USB_BREQ_BT_CONST: u8 = 4;
const GS_USB_BREQ_DEVICE_CONFIG: u8 = 5;
const GS_USB_BREQ_TIMESTAMP: u8 = 6;
const GS_USB_BREQ_IDENTIFY: u8 = 7;
const GS_USB_BREQ_GET_USER_ID: u8 = 8;
const GS_USB_BREQ_SET_USER_ID: u8 = 9;
const GS_USB_BREQ_DATA_BITTIMING: u8 = 10;
const GS_USB_BREQ_BT_CONST_EXT: u8 = 11;

#[repr(C, packed)]
struct GsHostConfig {
    byte_order: u32,
}

#[repr(C, packed)]
struct GsDeviceConfig {
    reserved1: u8,
    reserved2: u8,
    reserved3: u8,
    icount: u8,
    sw_version: u32,
    hw_version: u32,
}

#[repr(C, packed)]
struct GsDeviceBtConst {
    feature: u32,
    fclk_can: u32,
    tseg1_min: u32,
    tseg1_max: u32,
    tseg2_min: u32,
    tseg2_max: u32,
    sjw_max: u32,
    brp_min: u32,
    brp_max: u32,
    brp_inc: u32,
}

#[repr(C, packed)]
struct GsDeviceBittiming {
    prop_seg: u32,
    phase_seg1: u32,
    phase_seg2: u32,
    sjw: u32,
    brp: u32,
}

#[repr(C, packed)]
struct GsDeviceMode {
    mode: u32,
    flags: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct GsHostFrame {
    echo_id: u32,
    can_id: u32,

    can_dlc: u8,
    channel: u8,
    flags: u8,
    reserved: u8,

    // (a union with uint32_t data32[2])
    data: [u8; 8],
}

impl GsHostFrame {
    const ZERO: GsHostFrame = GsHostFrame { echo_id: 0, can_id: 0, can_dlc: 0, channel: 0, flags: 0, reserved: 0, data: [0; 8] };
}

/****************************************************************
 * Main usbcan task (read requests from usb and send msgs to usb)
 ****************************************************************/

// Global storage
struct UsbcanData {
    wake: TaskWake,

    // Canbus data routed locally
    notify_local: u8,
    usb_send_busy: u8,
    assigned_id: u32,

    // State tracking for messages to be sent from host to canbus
    bus_send_discard_time: u32,
    bus_send_state: u8,

    // Canbus data from host
    host_status: u8,
    host_pull_pos: u32,
    host_push_pos: u32,
    host_frames: [GsHostFrame; 16],

    // Data from physical canbus interface
    canhw_pull_pos: u32,
    canhw_push_pos: u32,
    canhw_queue: [CanbusMsg; 32],
}

static USB_CAN: Global<UsbcanData> = Global::new(UsbcanData {
    wake: TaskWake { wake: 0 },
    notify_local: 0,
    usb_send_busy: 0,
    assigned_id: 0,
    bus_send_discard_time: 0,
    bus_send_state: 0,
    host_status: 0,
    host_pull_pos: 0,
    host_push_pos: 0,
    host_frames: [GsHostFrame::ZERO; 16],
    canhw_pull_pos: 0,
    canhw_push_pos: 0,
    canhw_queue: [CanbusMsg { id: 0, dlc: 0, data: [0; 8] }; 32],
});

fn uc() -> &'static mut UsbcanData {
    USB_CAN.get()
}

const BSS_READY: u8 = 0;
const BSS_BLOCKING: u8 = 1;
const BSS_DISCARDING: u8 = 2;

const HS_TX_ECHO: u8 = 1;
const HS_TX_HW: u8 = 2;
const HS_TX_LOCAL: u8 = 4;

// Send a message to the Linux host
fn send_frame(msg: &CanbusMsg) -> i8 {
    let mut gs = GsHostFrame::ZERO;
    gs.echo_id = 0xffffffff;
    gs.can_id = msg.id;
    gs.can_dlc = msg.dlc as u8;
    gs.data = msg.data;
    usb_send_bulk_in(&gs as *const _ as *const u8, size_of::<GsHostFrame>() as u8)
}

// Send any pending messages read from canbus hw to host
fn drain_canhw_queue() {
    let u = uc();
    let mut pull_pos = u.canhw_pull_pos;
    loop {
        let push_pos = unsafe { read_volatile(addr_of_mut!((*USB_CAN.ptr()).canhw_push_pos)) };
        if push_pos == pull_pos {
            // No more data to send
            u.usb_send_busy = 0;
            return;
        }
        let pos = (pull_pos % u.canhw_queue.len() as u32) as usize;
        let ret = send_frame(&u.canhw_queue[pos]);
        if ret < 0 {
            // USB is busy - retry later
            u.usb_send_busy = 1;
            return;
        }
        pull_pos += 1;
        u.canhw_pull_pos = pull_pos;
    }
}

// Fill local queue with any USB messages read from host
fn drain_usb_host_messages() {
    let u = uc();
    let pull_pos = u.host_pull_pos;
    let mut push_pos = u.host_push_pos;
    loop {
        if push_pos.wrapping_sub(pull_pos) >= u.host_frames.len() as u32 {
            // No more space in queue
            break;
        }
        let pushp = (push_pos % u.host_frames.len() as u32) as usize;
        let gs = &mut u.host_frames[pushp] as *mut GsHostFrame as *mut u8;
        let ret = usb_read_bulk_out(gs, size_of::<GsHostFrame>() as u8);
        if ret <= 0 {
            // No more messages ready
            break;
        }
        push_pos += 1;
        u.host_push_pos = push_pos;
    }
}

// Report bus stall state
fn note_discard_state(discard: u32) -> R {
    sendf!("usb_canbus_state discard=%u", discard)
}

// Check if canbus queue has gotten stuck
fn check_need_discard() -> bool {
    let u = uc();
    if u.bus_send_state != BSS_BLOCKING {
        return false;
    }
    timer_is_before(u.bus_send_discard_time, timer_read_time())
}

// Attempt to send a message on the canbus
fn try_canmsg_send(msg: &CanbusMsg) -> R<i32> {
    let u = uc();
    let ret = canhw_send(msg);
    if ret >= 0 {
        // Success
        if u.bus_send_state == BSS_DISCARDING {
            note_discard_state(0)?;
        }
        u.bus_send_state = BSS_READY;
        return Ok(ret);
    }

    // Unable to send message
    if check_need_discard() {
        // The canbus is stalled - start discarding messages
        note_discard_state(1)?;
        u.bus_send_state = BSS_DISCARDING;
    }
    if u.bus_send_state == BSS_DISCARDING {
        // Queue is stalled - just discard the message
        return Ok(0);
    }
    if u.bus_send_state == BSS_READY {
        // Just starting to block - setup stall detection after 50ms
        u.bus_send_state = BSS_BLOCKING;
        u.bus_send_discard_time = timer_read_time().wrapping_add(timer_from_us(50000));
    }
    Ok(ret)
}

// Process new requests arriving from the host
fn drain_host_queue() -> R {
    let u = uc();
    let mut pull_pos = u.host_pull_pos;
    let push_pos = u.host_push_pos;
    loop {
        let pullp = (pull_pos % u.host_frames.len() as u32) as usize;
        let gs = u.host_frames[pullp];
        let mut host_status = u.host_status;

        // Extract next frame from host
        if host_status == 0 {
            if pull_pos == push_pos {
                // No frame available - no more work to be done
                break;
            }
            let id = gs.can_id;
            host_status = HS_TX_ECHO | HS_TX_HW;
            if id == CANBUS_ID_ADMIN {
                host_status = HS_TX_ECHO | HS_TX_HW | HS_TX_LOCAL;
            } else if u.assigned_id != 0 && u.assigned_id == id {
                host_status = HS_TX_ECHO | HS_TX_LOCAL;
            }
            u.host_status = host_status;
        }

        // Send echo frames back to host
        if host_status & HS_TX_ECHO != 0 {
            if u.notify_local != 0 || u.usb_send_busy != 0 {
                // Don't send echo frame until other traffic is sent
                break;
            }
            let ret = usb_send_bulk_in(&u.host_frames[pullp] as *const _ as *const u8, size_of::<GsHostFrame>() as u8);
            if ret < 0 {
                break;
            }
            host_status &= !HS_TX_ECHO;
            u.host_status = host_status;
        }

        // See if host frame needs to be transmitted
        let msg = CanbusMsg { id: gs.can_id, dlc: gs.can_dlc as u32, data: gs.data };
        if host_status & HS_TX_LOCAL != 0 {
            canserial_process_data(&msg);
            host_status &= !HS_TX_LOCAL;
            u.host_status = host_status;
        }
        if host_status & HS_TX_HW != 0 {
            let ret = try_canmsg_send(&msg)?;
            if ret < 0 {
                break;
            }
            host_status &= !HS_TX_HW;
            u.host_status = host_status;
        }

        // Note message fully processed
        pull_pos += 1;
        u.host_pull_pos = pull_pos;
    }
    Ok(())
}

// Main message routing task
pub fn usbcan_task() -> R {
    if !sched_check_wake(&mut uc().wake) && !check_need_discard() {
        return Ok(());
    }

    // Send messages read from canbus hardware to host
    drain_canhw_queue();

    // Fill local queue with any USB messages arriving from host
    drain_usb_host_messages();

    // Route messages received from host
    drain_host_queue()?;

    // Wake up local message response handling (if usb is not busy)
    if uc().notify_local != 0 && uc().usb_send_busy == 0 {
        canserial_notify_tx();
    }
    Ok(())
}
decl_task!(usbcan_task);

// Helper function to wake usbcan_task()
fn wake_usbcan_task() {
    sched_wake_task(&mut uc().wake);
}

/****************************************************************
 * Interface to canbus hardware (read canbus hw msgs and tx notifications)
 ****************************************************************/

pub fn canbus_notify_tx() {
    wake_usbcan_task();
}

// Handle incoming data from hw canbus interface (called from IRQ handler)
pub fn canbus_process_data(msg: &CanbusMsg) {
    let u = uc();
    // Add to admin command queue
    let pushp = u.canhw_push_pos;
    if pushp.wrapping_sub(u.canhw_pull_pos) >= u.canhw_queue.len() as u32 {
        // No space - drop message
        return;
    }
    if u.assigned_id != 0 && (msg.id & !1) == u.assigned_id {
        // Id reserved for local
        return;
    }
    let pos = (pushp % u.canhw_queue.len() as u32) as usize;
    u.canhw_queue[pos] = *msg;
    u.canhw_push_pos = pushp + 1;
    wake_usbcan_task();
}

/****************************************************************
 * Handle messages routed locally (canserial.c interface)
 ****************************************************************/

pub fn canbus_send(msg: &CanbusMsg) -> i32 {
    let u = uc();
    'retry_later: {
        if u.usb_send_busy != 0 {
            break 'retry_later;
        }
        let ret = send_frame(msg);
        if ret < 0 {
            break 'retry_later;
        }
        if u.host_status != 0 {
            wake_usbcan_task();
        }
        u.notify_local = 0;
        return msg.dlc as i32;
    }
    u.notify_local = 1;
    -1
}

pub fn canbus_set_filter(id: u32) {
    uc().assigned_id = id;
}

/****************************************************************
 * USB bulk wakeup interface
 ****************************************************************/

pub fn usb_notify_bulk_out() {
    wake_usbcan_task();
}

pub fn usb_notify_bulk_in() {
    wake_usbcan_task();
}

/****************************************************************
 * USB descriptors
 ****************************************************************/

// (CONFIG_USB_MANUFACTURER: Kalico's autoconf.h always defines it, so the
// u"Klipper" default of the C code is not used)

// String descriptors
const USB_STR_ID_MANUFACTURER: u16 = 1;
const USB_STR_ID_PRODUCT: u16 = 2;
const USB_STR_ID_SERIAL: u16 = 3;

const MANUFACTURER_LEN: usize = utf16_len(CONFIG_USB_MANUFACTURER) + 1;
const PRODUCT_LEN: usize = utf16_len(CONFIG_USB_PRODUCT) + 1;
const SERIAL_LEN: usize = utf16_len(CONFIG_USB_SERIAL_NUMBER) + 1;

// sizeof(the descriptor) + sizeof(the u"" literal of N units) - 2
const fn string_size<const N: usize>() -> u8 {
    (size_of::<UsbStringDescriptor<0>>() + N * 2 - 2) as u8
}
const SIZE_CDC_STRING_LANGIDS: u8 = (size_of::<UsbStringDescriptor<0>>() + 2) as u8;
const SIZE_CDC_STRING_MANUFACTURER: u8 = string_size::<MANUFACTURER_LEN>();
const SIZE_CDC_STRING_PRODUCT: u8 = string_size::<PRODUCT_LEN>();
const SIZE_CDC_STRING_SERIAL: u8 = string_size::<SERIAL_LEN>();

static CDC_STRING_LANGIDS: UsbStringDescriptor<1> =
    UsbStringDescriptor { bLength: SIZE_CDC_STRING_LANGIDS, bDescriptorType: USB_DT_STRING, data: [USB_LANGID_ENGLISH_US] };

static CDC_STRING_MANUFACTURER: UsbStringDescriptor<MANUFACTURER_LEN> = UsbStringDescriptor {
    bLength: SIZE_CDC_STRING_MANUFACTURER,
    bDescriptorType: USB_DT_STRING,
    data: utf16(CONFIG_USB_MANUFACTURER),
};

static CDC_STRING_PRODUCT: UsbStringDescriptor<PRODUCT_LEN> =
    UsbStringDescriptor { bLength: SIZE_CDC_STRING_PRODUCT, bDescriptorType: USB_DT_STRING, data: utf16(CONFIG_USB_PRODUCT) };

static CDC_STRING_SERIAL: UsbStringDescriptor<SERIAL_LEN> =
    UsbStringDescriptor { bLength: SIZE_CDC_STRING_SERIAL, bDescriptorType: USB_DT_STRING, data: utf16(CONFIG_USB_SERIAL_NUMBER) };

// Device descriptor
static GS_DEVICE_DESCRIPTOR: UsbDeviceDescriptor = UsbDeviceDescriptor {
    bLength: size_of::<UsbDeviceDescriptor>() as u8,
    bDescriptorType: USB_DT_DEVICE,
    bcdUSB: 0x0200u16.to_le(),
    bDeviceClass: 0,
    bDeviceSubClass: 0,
    bDeviceProtocol: 0,
    bMaxPacketSize0: USB_CDC_EP0_SIZE as u8,
    idVendor: USB_GSUSB_1_VENDOR_ID.to_le(),
    idProduct: USB_GSUSB_1_PRODUCT_ID.to_le(),
    bcdDevice: 0,
    iManufacturer: USB_STR_ID_MANUFACTURER as u8,
    iProduct: USB_STR_ID_PRODUCT as u8,
    iSerialNumber: USB_STR_ID_SERIAL as u8,
    bNumConfigurations: 1,
};

// Config descriptor
#[repr(C, packed)]
struct ConfigS {
    config: UsbConfigDescriptor,
    iface0: UsbInterfaceDescriptor,
    ep1: UsbEndpointDescriptor,
    ep2: UsbEndpointDescriptor,
}

static GS_CONFIG_DESCRIPTOR: ConfigS = ConfigS {
    config: UsbConfigDescriptor {
        bLength: size_of::<UsbConfigDescriptor>() as u8,
        bDescriptorType: USB_DT_CONFIG,
        wTotalLength: (size_of::<ConfigS>() as u16).to_le(),
        bNumInterfaces: 1,
        bConfigurationValue: 1,
        iConfiguration: 0,
        bmAttributes: 0xC0,
        bMaxPower: 50,
    },
    iface0: UsbInterfaceDescriptor {
        bLength: size_of::<UsbInterfaceDescriptor>() as u8,
        bDescriptorType: USB_DT_INTERFACE,
        bInterfaceNumber: 0,
        bAlternateSetting: 0,
        bNumEndpoints: 2,
        bInterfaceClass: 255,
        bInterfaceSubClass: 255,
        bInterfaceProtocol: 255,
        iInterface: 0,
    },
    ep1: UsbEndpointDescriptor {
        bLength: size_of::<UsbEndpointDescriptor>() as u8,
        bDescriptorType: USB_DT_ENDPOINT,
        bEndpointAddress: USB_CDC_EP_BULK_OUT as u8,
        bmAttributes: USB_ENDPOINT_XFER_BULK,
        wMaxPacketSize: (USB_CDC_EP_BULK_OUT_SIZE as u16).to_le(),
        bInterval: 0,
    },
    ep2: UsbEndpointDescriptor {
        bLength: size_of::<UsbEndpointDescriptor>() as u8,
        bDescriptorType: USB_DT_ENDPOINT,
        bEndpointAddress: USB_CDC_EP_BULK_IN as u8 | USB_DIR_IN,
        bmAttributes: USB_ENDPOINT_XFER_BULK,
        wMaxPacketSize: (USB_CDC_EP_BULK_IN_SIZE as u16).to_le(),
        bInterval: 0,
    },
};

// List of available descriptors
struct DescriptorS {
    w_value: u16,
    w_index: u16,
    desc: *const u8,
    size: u8,
}
unsafe impl Sync for DescriptorS {}

macro_rules! desc {
    ($v:expr, $i:expr, $d:expr, $s:expr) => {
        DescriptorS { w_value: $v, w_index: $i, desc: &$d as *const _ as *const u8, size: $s }
    };
}

static USB_DESCRIPTORS_BASE: [DescriptorS; 5] = [
    desc!((USB_DT_DEVICE as u16) << 8, 0x0000, GS_DEVICE_DESCRIPTOR, size_of::<UsbDeviceDescriptor>() as u8),
    desc!((USB_DT_CONFIG as u16) << 8, 0x0000, GS_CONFIG_DESCRIPTOR, size_of::<ConfigS>() as u8),
    desc!((USB_DT_STRING as u16) << 8, 0x0000, CDC_STRING_LANGIDS, SIZE_CDC_STRING_LANGIDS),
    desc!(
        ((USB_DT_STRING as u16) << 8) | USB_STR_ID_MANUFACTURER,
        USB_LANGID_ENGLISH_US,
        CDC_STRING_MANUFACTURER,
        SIZE_CDC_STRING_MANUFACTURER
    ),
    desc!(((USB_DT_STRING as u16) << 8) | USB_STR_ID_PRODUCT, USB_LANGID_ENGLISH_US, CDC_STRING_PRODUCT, SIZE_CDC_STRING_PRODUCT),
];
#[cfg(not(CONFIG_USB_SERIAL_NUMBER_CHIPID))]
static USB_DESCRIPTORS_SERIAL: [DescriptorS; 1] = [desc!(
    ((USB_DT_STRING as u16) << 8) | USB_STR_ID_SERIAL,
    USB_LANGID_ENGLISH_US,
    CDC_STRING_SERIAL,
    SIZE_CDC_STRING_SERIAL
)];
#[cfg(CONFIG_USB_SERIAL_NUMBER_CHIPID)]
static USB_DESCRIPTORS_SERIAL: [DescriptorS; 0] = [];

// Fill in a USB serial string descriptor from a chip id
pub fn usb_fill_serial<const N: usize>(desc: &mut UsbStringDescriptor<N>, strlen: usize, id: *const u8) {
    desc.bLength = (size_of::<UsbStringDescriptor<0>>() + strlen * size_of::<u16>()) as u8;
    desc.bDescriptorType = USB_DT_STRING;

    let mut data = desc.data;
    for (i, d) in data.iter_mut().enumerate().take(strlen) {
        let b = unsafe { core::ptr::read_volatile(id.add(i / 2)) };
        let c = if i & 1 != 0 { b & 0x0f } else { b >> 4 };
        *d = (if c < 10 { c + b'0' } else { c - 10 + b'A' }) as u16;
    }
    desc.data = data;
}

/****************************************************************
 * USB endpoint 0 control message handling
 ****************************************************************/

// State tracking
const UX_READ: u8 = 1 << 0;
const UX_SEND: u8 = 1 << 1;
const UX_SEND_PROGMEM: u8 = 1 << 2;
const UX_SEND_ZLP: u8 = 1 << 3;

static USB_XFER_DATA: Global<*mut u8> = Global::new(core::ptr::null_mut());
static USB_XFER_SIZE: Global<u8> = Global::new(0);
static USB_XFER_FLAGS: Global<u8> = Global::new(0);

// Set the USB "stall" condition
fn usb_do_stall() {
    usb_stall_ep0();
    *USB_XFER_FLAGS.get() = 0;
}

// Transfer data on the usb endpoint 0
// (the UX_SEND_PROGMEM / usb_send_ep0_progmem() path is for AVR only:
// NEED_PROGMEM)
fn usb_do_xfer(mut data: *mut u8, mut size: u8, mut flags: u8) {
    loop {
        let mut xs = size;
        if xs as usize > USB_CDC_EP0_SIZE {
            xs = USB_CDC_EP0_SIZE as u8;
        }
        let ret = if flags & UX_READ != 0 { usb_read_ep0(data, xs) } else { usb_send_ep0(data, xs) };
        if ret as i32 == xs as i32 {
            // Success
            data = data.wrapping_add(xs as usize);
            size -= xs;
            if size == 0 {
                // Entire transfer completed successfully
                if flags & UX_READ != 0 {
                    // Send status packet at end of read
                    flags = UX_SEND;
                    continue;
                }
                if xs as usize == USB_CDC_EP0_SIZE && flags & UX_SEND_ZLP != 0 {
                    // Must send zero-length-packet
                    continue;
                }
                *USB_XFER_FLAGS.get() = 0;
                usb_notify_ep0();
                return;
            }
            continue;
        }
        if ret == -1 {
            // Interface busy - retry later
            *USB_XFER_DATA.get() = data;
            *USB_XFER_SIZE.get() = size;
            *USB_XFER_FLAGS.get() = flags;
            return;
        }
        // Error
        usb_do_stall();
        return;
    }
}

fn usb_req_get_descriptor(req: &UsbCtrlRequest) {
    'fail: {
        if req.bRequestType != USB_DIR_IN {
            break 'fail;
        }
        let mut desc: *const u8 = core::ptr::null();
        let (mut flags, mut size) = (0u8, 0u8);
        for d in USB_DESCRIPTORS_BASE.iter().chain(USB_DESCRIPTORS_SERIAL.iter()) {
            if d.w_value == { req.wValue } && d.w_index == { req.wIndex } {
                flags = UX_SEND;
                size = d.size;
                desc = d.desc;
            }
        }
        if CONFIG_USB_SERIAL_NUMBER_CHIPID
            && { req.wValue } == ((USB_DT_STRING as u16) << 8) | USB_STR_ID_SERIAL
            && { req.wIndex } == USB_LANGID_ENGLISH_US
        {
            let usbserial_serialid = usbserial_get_serialid();
            flags = UX_SEND;
            size = unsafe { *usbserial_serialid }; // its bLength
            desc = usbserial_serialid;
        }
        if !desc.is_null() {
            if size as u16 > { req.wLength } {
                size = req.wLength as u8;
            } else if (size as u16) < { req.wLength } {
                flags |= UX_SEND_ZLP;
            }
            usb_do_xfer(desc as *mut u8, size, flags);
            return;
        }
    }
    usb_do_stall();
}

fn usb_req_set_address(req: &UsbCtrlRequest) {
    if req.bRequestType != 0 || { req.wIndex } != 0 || { req.wLength } != 0 {
        usb_do_stall();
        return;
    }
    usb_set_address(req.wValue as u8);
}

fn usb_req_set_configuration(req: &UsbCtrlRequest) {
    if req.bRequestType != 0 || { req.wValue } != 1 || { req.wIndex } != 0 || { req.wLength } != 0 {
        usb_do_stall();
        return;
    }
    usb_set_configure();
    wake_usbcan_task();
    usb_do_xfer(core::ptr::null_mut(), 0, UX_SEND);
}

static HOST_CONFIG: Global<GsHostConfig> = Global::new(GsHostConfig { byte_order: 0 });

fn gs_breq_host_format(_req: &UsbCtrlRequest) {
    // Like candlightfw, little-endian is always used.  Read and ignore value.
    usb_do_xfer(HOST_CONFIG.ptr() as *mut u8, size_of::<GsHostConfig>() as u8, UX_READ);
}

static DEVICE_CONFIG: GsDeviceConfig =
    GsDeviceConfig { reserved1: 0, reserved2: 0, reserved3: 0, icount: 0, sw_version: 2, hw_version: 1 };

fn gs_breq_device_config(_req: &UsbCtrlRequest) {
    usb_do_xfer(&DEVICE_CONFIG as *const _ as *mut u8, size_of::<GsDeviceConfig>() as u8, UX_SEND);
}

static BT_CONST: GsDeviceBtConst = GsDeviceBtConst {
    // These are just dummy values for now
    feature: 0,
    fclk_can: 48000000,
    tseg1_min: 1,
    tseg1_max: 16,
    tseg2_min: 1,
    tseg2_max: 8,
    sjw_max: 4,
    brp_min: 1,
    brp_max: 1024,
    brp_inc: 1,
};

fn gs_breq_bt_const(_req: &UsbCtrlRequest) {
    usb_do_xfer(&BT_CONST as *const _ as *mut u8, size_of::<GsDeviceBtConst>() as u8, UX_SEND);
}

static DEVICE_BITTIMING: Global<GsDeviceBittiming> =
    Global::new(GsDeviceBittiming { prop_seg: 0, phase_seg1: 0, phase_seg2: 0, sjw: 0, brp: 0 });

fn gs_breq_bittiming(_req: &UsbCtrlRequest) {
    // Bit timing is ignored for now
    usb_do_xfer(DEVICE_BITTIMING.ptr() as *mut u8, size_of::<GsDeviceBittiming>() as u8, UX_READ);
}

static DEVICE_MODE: Global<GsDeviceMode> = Global::new(GsDeviceMode { mode: 0, flags: 0 });

fn gs_breq_mode(_req: &UsbCtrlRequest) {
    // Mode is ignored for now
    usb_do_xfer(DEVICE_MODE.ptr() as *mut u8, size_of::<GsDeviceMode>() as u8, UX_READ);
}

fn usb_state_ready() {
    let mut req = UsbCtrlRequest { bRequestType: 0, bRequest: 0, wValue: 0, wIndex: 0, wLength: 0 };
    let ret = usb_read_ep0_setup(&mut req as *mut _ as *mut u8, size_of::<UsbCtrlRequest>() as u8);
    if ret as usize != size_of::<UsbCtrlRequest>() {
        return;
    }
    let req_type = req.bRequestType & USB_TYPE_MASK;
    if req_type == USB_TYPE_STANDARD {
        match req.bRequest {
            USB_REQ_GET_DESCRIPTOR => usb_req_get_descriptor(&req),
            USB_REQ_SET_ADDRESS => usb_req_set_address(&req),
            USB_REQ_SET_CONFIGURATION => usb_req_set_configuration(&req),
            _ => usb_do_stall(),
        }
    } else if req_type == USB_TYPE_VENDOR {
        match req.bRequest {
            GS_USB_BREQ_HOST_FORMAT => gs_breq_host_format(&req),
            GS_USB_BREQ_DEVICE_CONFIG => gs_breq_device_config(&req),
            GS_USB_BREQ_BT_CONST => gs_breq_bt_const(&req),
            GS_USB_BREQ_BITTIMING => gs_breq_bittiming(&req),
            GS_USB_BREQ_MODE => gs_breq_mode(&req),
            _ => usb_do_stall(),
        }
    } else {
        usb_do_stall();
    }
}

// State tracking dispatch
static USB_EP0_WAKE: Global<TaskWake> = Global::new(TaskWake { wake: 0 });

pub fn usb_notify_ep0() {
    sched_wake_task(USB_EP0_WAKE.ptr());
}

pub fn usb_ep0_task() -> R {
    if !sched_check_wake(USB_EP0_WAKE.ptr()) {
        return Ok(());
    }
    if *USB_XFER_FLAGS.get() != 0 {
        usb_do_xfer(*USB_XFER_DATA.get(), *USB_XFER_SIZE.get(), *USB_XFER_FLAGS.get());
    } else {
        usb_state_ready();
    }
    Ok(())
}
decl_task!(usb_ep0_task);

pub fn usb_shutdown() -> R {
    wake_usbcan_task();
    usb_notify_ep0();
    Ok(())
}
decl_shutdown!(usb_shutdown);
