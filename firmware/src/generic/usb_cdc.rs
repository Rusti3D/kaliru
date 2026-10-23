// Support for standard serial port over USB emulation
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::{
    usb_read_bulk_out, usb_read_ep0, usb_read_ep0_setup, usb_send_bulk_in, usb_send_ep0, usb_set_address, usb_set_configure,
    usb_stall_ep0, usbserial_get_serialid,
};
use crate::command::{command_encode_and_frame, command_find_and_dispatch, CommandEncoder, SendArg};
use crate::config::{
    CONFIG_HAVE_BOOTLOADER_REQUEST, CONFIG_USB_DEVICE_ID, CONFIG_USB_MANUFACTURER, CONFIG_USB_PRODUCT, CONFIG_USB_SERIAL_NUMBER,
    CONFIG_USB_SERIAL_NUMBER_CHIPID, CONFIG_USB_VENDOR_ID,
};
use crate::ctr::Global;
use crate::generic::usb_cdc_ep::{
    USB_CDC_EP0_SIZE, USB_CDC_EP_ACM, USB_CDC_EP_ACM_SIZE, USB_CDC_EP_BULK_IN, USB_CDC_EP_BULK_IN_SIZE, USB_CDC_EP_BULK_OUT,
    USB_CDC_EP_BULK_OUT_SIZE,
};
use crate::generic::usbstd::*;
use crate::generic::usbstd_cdc::*;
use crate::sched::{sched_check_wake, sched_wake_task, TaskWake, R};
use crate::{decl_shutdown, decl_task};
use core::mem::size_of;

/****************************************************************
 * Message block sending
 ****************************************************************/

static USB_BULK_IN_WAKE: Global<TaskWake> = Global::new(TaskWake { wake: 0 });
static TRANSMIT_BUF: Global<[u8; 192]> = Global::new([0; 192]);
static TRANSMIT_POS: Global<u8> = Global::new(0);

pub fn usb_notify_bulk_in() {
    sched_wake_task(USB_BULK_IN_WAKE.ptr());
}

pub fn usb_bulk_in_task() -> R {
    if !sched_check_wake(USB_BULK_IN_WAKE.ptr()) {
        return Ok(());
    }
    let tpos = *TRANSMIT_POS.get() as usize;
    let mut max_tpos = tpos;
    if tpos == 0 {
        return Ok(());
    }
    if max_tpos > USB_CDC_EP_BULK_IN_SIZE {
        max_tpos = USB_CDC_EP_BULK_IN_SIZE;
    } else if max_tpos == USB_CDC_EP_BULK_IN_SIZE {
        max_tpos = USB_CDC_EP_BULK_IN_SIZE - 1; // Avoid zero-length-packets
    }
    let buf = TRANSMIT_BUF.get();
    let ret = usb_send_bulk_in(buf.as_ptr(), max_tpos as u8);
    if ret <= 0 {
        return Ok(());
    }
    let needcopy = tpos - ret as usize;
    if needcopy != 0 {
        buf.copy_within(ret as usize..tpos, 0);
        usb_notify_bulk_in();
    }
    *TRANSMIT_POS.get() = needcopy as u8;
    Ok(())
}
decl_task!(usb_bulk_in_task);

// Encode and transmit a "response" message
pub fn console_sendf(ce: &CommandEncoder, args: &[SendArg]) -> R {
    // Verify space for message
    let tpos = *TRANSMIT_POS.get() as usize;
    let max_size = ce.max_size as usize;
    let buf = TRANSMIT_BUF.get();
    if tpos + max_size > buf.len() {
        // Not enough space for message
        return Ok(());
    }

    // Generate message
    let msglen = command_encode_and_frame(&mut buf[tpos..], ce, args)?;

    // Start message transmit
    *TRANSMIT_POS.get() = (tpos + msglen) as u8;
    usb_notify_bulk_in();
    Ok(())
}

/****************************************************************
 * Message block reading
 ****************************************************************/

static USB_BULK_OUT_WAKE: Global<TaskWake> = Global::new(TaskWake { wake: 0 });
static RECEIVE_BUF: Global<[u8; 128]> = Global::new([0; 128]);
static RECEIVE_POS: Global<u8> = Global::new(0);

pub fn usb_notify_bulk_out() {
    sched_wake_task(USB_BULK_OUT_WAKE.ptr());
}

pub fn usb_bulk_out_task() -> R {
    if !sched_check_wake(USB_BULK_OUT_WAKE.ptr()) {
        return Ok(());
    }
    // Read data
    let buf = RECEIVE_BUF.get();
    let mut rpos = *RECEIVE_POS.get() as usize;
    let mut pop_count = 0;
    if rpos + USB_CDC_EP_BULK_OUT_SIZE <= buf.len() {
        let ret = usb_read_bulk_out(buf[rpos..].as_mut_ptr(), USB_CDC_EP_BULK_OUT_SIZE as u8);
        if ret > 0 {
            rpos += ret as usize;
            usb_notify_bulk_out();
        }
    } else {
        usb_notify_bulk_out();
    }
    // Process a message block
    let ret = command_find_and_dispatch(&buf[..rpos], &mut pop_count)?;
    if ret != 0 {
        // Move buffer
        let needcopy = rpos - pop_count;
        if needcopy != 0 {
            buf.copy_within(pop_count..rpos, 0);
            usb_notify_bulk_out();
        }
        rpos = needcopy;
    }
    *RECEIVE_POS.get() = rpos as u8;
    Ok(())
}
decl_task!(usb_bulk_out_task);

/****************************************************************
 * USB descriptors
 ****************************************************************/

// (CONFIG_USB_MANUFACTURER: Kalico's autoconf.h always defines it, so the
// u"Klipper" default of the C code is not used)

// String descriptors
const USB_STR_ID_MANUFACTURER: u16 = 1;
const USB_STR_ID_PRODUCT: u16 = 2;
const USB_STR_ID_SERIAL: u16 = 3;

// the number of UTF-16 units of a string (the u"" literal, without its 0)
const fn utf16_len(s: &str) -> usize {
    let b = s.as_bytes();
    let (mut i, mut n) = (0, 0);
    while i < b.len() {
        let c = b[i];
        let (len, units) = if c < 0x80 {
            (1, 1)
        } else if c < 0xe0 {
            (2, 1)
        } else if c < 0xf0 {
            (3, 1)
        } else {
            (4, 2)
        };
        i += len;
        n += units;
    }
    n
}

// a u"" literal: the UTF-16 units of a string and its terminating 0
const fn utf16<const N: usize>(s: &str) -> [u16; N] {
    let b = s.as_bytes();
    let mut out = [0u16; N];
    let (mut i, mut n) = (0, 0);
    while i < b.len() {
        let c = b[i] as u32;
        let (cp, len) = if c < 0x80 {
            (c, 1)
        } else if c < 0xe0 {
            (((c & 0x1f) << 6) | (b[i + 1] as u32 & 0x3f), 2)
        } else if c < 0xf0 {
            (((c & 0x0f) << 12) | ((b[i + 1] as u32 & 0x3f) << 6) | (b[i + 2] as u32 & 0x3f), 3)
        } else {
            (((c & 0x07) << 18) | ((b[i + 1] as u32 & 0x3f) << 12) | ((b[i + 2] as u32 & 0x3f) << 6) | (b[i + 3] as u32 & 0x3f), 4)
        };
        if cp >= 0x10000 {
            out[n] = (0xd800 + ((cp - 0x10000) >> 10)) as u16;
            out[n + 1] = (0xdc00 + ((cp - 0x10000) & 0x3ff)) as u16;
            n += 2;
        } else {
            out[n] = cp as u16;
            n += 1;
        }
        i += len;
    }
    out
}

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

static CDC_STRING_LANGIDS: UsbStringDescriptor<1> = UsbStringDescriptor {
    bLength: SIZE_CDC_STRING_LANGIDS,
    bDescriptorType: USB_DT_STRING,
    data: [USB_LANGID_ENGLISH_US],
};

static CDC_STRING_MANUFACTURER: UsbStringDescriptor<MANUFACTURER_LEN> = UsbStringDescriptor {
    bLength: SIZE_CDC_STRING_MANUFACTURER,
    bDescriptorType: USB_DT_STRING,
    data: utf16(CONFIG_USB_MANUFACTURER),
};

static CDC_STRING_PRODUCT: UsbStringDescriptor<PRODUCT_LEN> = UsbStringDescriptor {
    bLength: SIZE_CDC_STRING_PRODUCT,
    bDescriptorType: USB_DT_STRING,
    data: utf16(CONFIG_USB_PRODUCT),
};

#[allow(dead_code)]
static CDC_STRING_SERIAL: UsbStringDescriptor<SERIAL_LEN> = UsbStringDescriptor {
    bLength: SIZE_CDC_STRING_SERIAL,
    bDescriptorType: USB_DT_STRING,
    data: utf16(CONFIG_USB_SERIAL_NUMBER),
};

// Device descriptor
static CDC_DEVICE_DESCRIPTOR: UsbDeviceDescriptor = UsbDeviceDescriptor {
    bLength: size_of::<UsbDeviceDescriptor>() as u8,
    bDescriptorType: USB_DT_DEVICE,
    bcdUSB: 0x0200u16.to_le(),
    bDeviceClass: USB_CLASS_COMM,
    bDeviceSubClass: 0,
    bDeviceProtocol: 0,
    bMaxPacketSize0: USB_CDC_EP0_SIZE as u8,
    idVendor: (CONFIG_USB_VENDOR_ID as u16).to_le(),
    idProduct: (CONFIG_USB_DEVICE_ID as u16).to_le(),
    bcdDevice: 0x0100u16.to_le(),
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
    cdc_hdr: UsbCdcHeaderDescriptor,
    cdc_acm: UsbCdcAcmDescriptor,
    cdc_union: UsbCdcUnionDescriptor,
    ep1: UsbEndpointDescriptor,
    iface1: UsbInterfaceDescriptor,
    ep2: UsbEndpointDescriptor,
    ep3: UsbEndpointDescriptor,
}

static CDC_CONFIG_DESCRIPTOR: ConfigS = ConfigS {
    config: UsbConfigDescriptor {
        bLength: size_of::<UsbConfigDescriptor>() as u8,
        bDescriptorType: USB_DT_CONFIG,
        wTotalLength: (size_of::<ConfigS>() as u16).to_le(),
        bNumInterfaces: 2,
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
        bNumEndpoints: 1,
        bInterfaceClass: USB_CLASS_COMM,
        bInterfaceSubClass: USB_CDC_SUBCLASS_ACM,
        bInterfaceProtocol: USB_CDC_ACM_PROTO_AT_V25TER,
        iInterface: 0,
    },
    cdc_hdr: UsbCdcHeaderDescriptor {
        bLength: size_of::<UsbCdcHeaderDescriptor>() as u8,
        bDescriptorType: USB_CDC_CS_INTERFACE,
        bDescriptorSubType: USB_CDC_HEADER_TYPE,
        bcdCDC: 0x0110,
    },
    cdc_acm: UsbCdcAcmDescriptor {
        bLength: size_of::<UsbCdcAcmDescriptor>() as u8,
        bDescriptorType: USB_CDC_CS_INTERFACE,
        bDescriptorSubType: USB_CDC_ACM_TYPE,
        bmCapabilities: 0x06,
    },
    cdc_union: UsbCdcUnionDescriptor {
        bLength: size_of::<UsbCdcUnionDescriptor>() as u8,
        bDescriptorType: USB_CDC_CS_INTERFACE,
        bDescriptorSubType: USB_CDC_UNION_TYPE,
        bMasterInterface0: 0,
        bSlaveInterface0: 1,
    },
    ep1: UsbEndpointDescriptor {
        bLength: size_of::<UsbEndpointDescriptor>() as u8,
        bDescriptorType: USB_DT_ENDPOINT,
        bEndpointAddress: USB_CDC_EP_ACM as u8 | USB_DIR_IN,
        bmAttributes: USB_ENDPOINT_XFER_INT,
        wMaxPacketSize: (USB_CDC_EP_ACM_SIZE as u16).to_le(),
        bInterval: 255,
    },
    iface1: UsbInterfaceDescriptor {
        bLength: size_of::<UsbInterfaceDescriptor>() as u8,
        bDescriptorType: USB_DT_INTERFACE,
        bInterfaceNumber: 1,
        bAlternateSetting: 0,
        bNumEndpoints: 2,
        bInterfaceClass: 0x0A,
        bInterfaceSubClass: 0,
        bInterfaceProtocol: 0,
        iInterface: 0,
    },
    ep2: UsbEndpointDescriptor {
        bLength: size_of::<UsbEndpointDescriptor>() as u8,
        bDescriptorType: USB_DT_ENDPOINT,
        bEndpointAddress: USB_CDC_EP_BULK_OUT as u8,
        bmAttributes: USB_ENDPOINT_XFER_BULK,
        wMaxPacketSize: (USB_CDC_EP_BULK_OUT_SIZE as u16).to_le(),
        bInterval: 0,
    },
    ep3: UsbEndpointDescriptor {
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

static CDC_DESCRIPTORS_BASE: [DescriptorS; 5] = [
    desc!((USB_DT_DEVICE as u16) << 8, 0x0000, CDC_DEVICE_DESCRIPTOR, size_of::<UsbDeviceDescriptor>() as u8),
    desc!((USB_DT_CONFIG as u16) << 8, 0x0000, CDC_CONFIG_DESCRIPTOR, size_of::<ConfigS>() as u8),
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
static CDC_DESCRIPTORS_SERIAL: [DescriptorS; 1] = [desc!(
    ((USB_DT_STRING as u16) << 8) | USB_STR_ID_SERIAL,
    USB_LANGID_ENGLISH_US,
    CDC_STRING_SERIAL,
    SIZE_CDC_STRING_SERIAL
)];
#[cfg(CONFIG_USB_SERIAL_NUMBER_CHIPID)]
static CDC_DESCRIPTORS_SERIAL: [DescriptorS; 0] = [];

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
#[allow(dead_code)]
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
        for d in CDC_DESCRIPTORS_BASE.iter().chain(CDC_DESCRIPTORS_SERIAL.iter()) {
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
    usb_notify_bulk_in();
    usb_notify_bulk_out();
    usb_do_xfer(core::ptr::null_mut(), 0, UX_SEND);
}

static LINE_CODING: Global<UsbCdcLineCoding> =
    Global::new(UsbCdcLineCoding { dwDTERate: 0, bCharFormat: 0, bParityType: 0, bDataBits: 0 });
static LINE_CONTROL_STATE: Global<u8> = Global::new(0);

fn check_reboot() {
    if !CONFIG_HAVE_BOOTLOADER_REQUEST {
        return;
    }
    if { LINE_CODING.get().dwDTERate } == 1200 && *LINE_CONTROL_STATE.get() & 0x01 == 0 {
        // A baud of 1200 is an Arduino style request to enter the bootloader
        crate::board::bootloader_request();
    }
}

fn usb_req_set_line_coding(req: &UsbCtrlRequest) {
    if req.bRequestType != 0x21 || { req.wValue } != 0 || { req.wIndex } != 0 || { req.wLength } as usize != size_of::<UsbCdcLineCoding>() {
        usb_do_stall();
        return;
    }
    usb_do_xfer(LINE_CODING.ptr() as *mut u8, size_of::<UsbCdcLineCoding>() as u8, UX_READ);
    check_reboot();
}

fn usb_req_get_line_coding(req: &UsbCtrlRequest) {
    if req.bRequestType != 0xa1 || { req.wValue } != 0 || { req.wIndex } != 0 || ({ req.wLength } as usize) < size_of::<UsbCdcLineCoding>() {
        usb_do_stall();
        return;
    }
    usb_do_xfer(LINE_CODING.ptr() as *mut u8, size_of::<UsbCdcLineCoding>() as u8, UX_SEND);
}

fn usb_req_set_line(req: &UsbCtrlRequest) {
    if req.bRequestType != 0x21 || { req.wIndex } != 0 || { req.wLength } != 0 {
        usb_do_stall();
        return;
    }
    *LINE_CONTROL_STATE.get() = req.wValue as u8;
    usb_do_xfer(core::ptr::null_mut(), 0, UX_SEND);
    check_reboot();
}

fn usb_state_ready() {
    let mut req = UsbCtrlRequest { bRequestType: 0, bRequest: 0, wValue: 0, wIndex: 0, wLength: 0 };
    let ret = usb_read_ep0_setup(&mut req as *mut _ as *mut u8, size_of::<UsbCtrlRequest>() as u8);
    if ret as usize != size_of::<UsbCtrlRequest>() {
        return;
    }
    match req.bRequest {
        USB_REQ_GET_DESCRIPTOR => usb_req_get_descriptor(&req),
        USB_REQ_SET_ADDRESS => usb_req_set_address(&req),
        USB_REQ_SET_CONFIGURATION => usb_req_set_configuration(&req),
        USB_CDC_REQ_SET_LINE_CODING => usb_req_set_line_coding(&req),
        USB_CDC_REQ_GET_LINE_CODING => usb_req_get_line_coding(&req),
        USB_CDC_REQ_SET_CONTROL_LINE_STATE => usb_req_set_line(&req),
        _ => usb_do_stall(),
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
    usb_notify_bulk_in();
    usb_notify_bulk_out();
    usb_notify_ep0();
    Ok(())
}
decl_shutdown!(usb_shutdown);
