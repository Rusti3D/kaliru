// Standard definitions for USB CDC devices
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, dead_code)]

pub const USB_CDC_SUBCLASS_ACM: u8 = 0x02;

pub const USB_CDC_ACM_PROTO_AT_V25TER: u8 = 1;

#[repr(C, packed)]
pub struct UsbCdcHeaderDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bDescriptorSubType: u8,
    pub bcdCDC: u16,
}

pub const USB_CDC_HEADER_TYPE: u8 = 0x00;
pub const USB_CDC_ACM_TYPE: u8 = 0x02;
pub const USB_CDC_UNION_TYPE: u8 = 0x06;

pub const USB_CDC_CS_INTERFACE: u8 = 0x24;
pub const USB_CDC_CS_ENDPOINT: u8 = 0x25;

#[repr(C, packed)]
pub struct UsbCdcAcmDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bDescriptorSubType: u8,
    pub bmCapabilities: u8,
}

#[repr(C, packed)]
pub struct UsbCdcUnionDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bDescriptorSubType: u8,
    pub bMasterInterface0: u8,
    pub bSlaveInterface0: u8,
}

pub const USB_CDC_REQ_SET_LINE_CODING: u8 = 0x20;
pub const USB_CDC_REQ_GET_LINE_CODING: u8 = 0x21;
pub const USB_CDC_REQ_SET_CONTROL_LINE_STATE: u8 = 0x22;

#[repr(C, packed)]
pub struct UsbCdcLineCoding {
    pub dwDTERate: u32,
    pub bCharFormat: u8,
    pub bParityType: u8,
    pub bDataBits: u8,
}
