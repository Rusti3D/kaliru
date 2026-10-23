// Standard definitions for USB commands and data structures
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, dead_code)]

pub const USB_DIR_OUT: u8 = 0; // to device
pub const USB_DIR_IN: u8 = 0x80; // to host

pub const USB_TYPE_MASK: u8 = 0x03 << 5;
pub const USB_TYPE_STANDARD: u8 = 0x00 << 5;
pub const USB_TYPE_CLASS: u8 = 0x01 << 5;
pub const USB_TYPE_VENDOR: u8 = 0x02 << 5;
pub const USB_TYPE_RESERVED: u8 = 0x03 << 5;

pub const USB_REQ_GET_STATUS: u8 = 0x00;
pub const USB_REQ_CLEAR_FEATURE: u8 = 0x01;
pub const USB_REQ_SET_FEATURE: u8 = 0x03;
pub const USB_REQ_SET_ADDRESS: u8 = 0x05;
pub const USB_REQ_GET_DESCRIPTOR: u8 = 0x06;
pub const USB_REQ_SET_DESCRIPTOR: u8 = 0x07;
pub const USB_REQ_GET_CONFIGURATION: u8 = 0x08;
pub const USB_REQ_SET_CONFIGURATION: u8 = 0x09;
pub const USB_REQ_GET_INTERFACE: u8 = 0x0A;
pub const USB_REQ_SET_INTERFACE: u8 = 0x0B;
pub const USB_REQ_SYNCH_FRAME: u8 = 0x0C;

#[repr(C, packed)]
pub struct UsbCtrlRequest {
    pub bRequestType: u8,
    pub bRequest: u8,
    pub wValue: u16,
    pub wIndex: u16,
    pub wLength: u16,
}

pub const USB_DT_DEVICE: u8 = 0x01;
pub const USB_DT_CONFIG: u8 = 0x02;
pub const USB_DT_STRING: u8 = 0x03;
pub const USB_DT_INTERFACE: u8 = 0x04;
pub const USB_DT_ENDPOINT: u8 = 0x05;
pub const USB_DT_DEVICE_QUALIFIER: u8 = 0x06;
pub const USB_DT_OTHER_SPEED_CONFIG: u8 = 0x07;
pub const USB_DT_ENDPOINT_COMPANION: u8 = 0x30;

#[repr(C, packed)]
pub struct UsbDeviceDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bcdUSB: u16,
    pub bDeviceClass: u8,
    pub bDeviceSubClass: u8,
    pub bDeviceProtocol: u8,
    pub bMaxPacketSize0: u8,
    pub idVendor: u16,
    pub idProduct: u16,
    pub bcdDevice: u16,
    pub iManufacturer: u8,
    pub iProduct: u8,
    pub iSerialNumber: u8,
    pub bNumConfigurations: u8,
}

pub const USB_CLASS_PER_INTERFACE: u8 = 0; // for DeviceClass
pub const USB_CLASS_AUDIO: u8 = 1;
pub const USB_CLASS_COMM: u8 = 2;
pub const USB_CLASS_HID: u8 = 3;
pub const USB_CLASS_PHYSICAL: u8 = 5;
pub const USB_CLASS_STILL_IMAGE: u8 = 6;
pub const USB_CLASS_PRINTER: u8 = 7;
pub const USB_CLASS_MASS_STORAGE: u8 = 8;
pub const USB_CLASS_HUB: u8 = 9;

#[repr(C, packed)]
pub struct UsbConfigDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub wTotalLength: u16,
    pub bNumInterfaces: u8,
    pub bConfigurationValue: u8,
    pub iConfiguration: u8,
    pub bmAttributes: u8,
    pub bMaxPower: u8,
}

#[repr(C, packed)]
pub struct UsbInterfaceDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bInterfaceNumber: u8,
    pub bAlternateSetting: u8,
    pub bNumEndpoints: u8,
    pub bInterfaceClass: u8,
    pub bInterfaceSubClass: u8,
    pub bInterfaceProtocol: u8,
    pub iInterface: u8,
}

#[repr(C, packed)]
pub struct UsbEndpointDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bEndpointAddress: u8,
    pub bmAttributes: u8,
    pub wMaxPacketSize: u16,
    pub bInterval: u8,
}

pub const USB_ENDPOINT_NUMBER_MASK: u8 = 0x0f; // in bEndpointAddress
pub const USB_ENDPOINT_DIR_MASK: u8 = 0x80;

pub const USB_ENDPOINT_XFERTYPE_MASK: u8 = 0x03; // in bmAttributes
pub const USB_ENDPOINT_XFER_CONTROL: u8 = 0;
pub const USB_ENDPOINT_XFER_ISOC: u8 = 1;
pub const USB_ENDPOINT_XFER_BULK: u8 = 2;
pub const USB_ENDPOINT_XFER_INT: u8 = 3;
pub const USB_ENDPOINT_MAX_ADJUSTABLE: u8 = 0x80;

// (C: a flexible `data[]` array; N is its length here)
#[repr(C, packed)]
pub struct UsbStringDescriptor<const N: usize> {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub data: [u16; N],
}

pub const USB_LANGID_ENGLISH_US: u16 = 0x0409;
