// Support for extracting the hardware chip id on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::UID_BASE;
use crate::config::CONFIG_USB_SERIAL_NUMBER_CHIPID;
use crate::ctr::Global;
use crate::decl_init;
use crate::generic::usb_cdc::usb_fill_serial;
use crate::generic::usbstd::UsbStringDescriptor;
use crate::sched::R;

const CHIP_UID_LEN: usize = 12;

// (struct { struct usb_string_descriptor desc; uint16_t data[CHIP_UID_LEN * 2]; })
static CDC_CHIPID: Global<UsbStringDescriptor<{ CHIP_UID_LEN * 2 }>> =
    Global::new(UsbStringDescriptor { bLength: 0, bDescriptorType: 0, data: [0; CHIP_UID_LEN * 2] });

// the serial string descriptor (its first byte is its bLength)
pub fn usbserial_get_serialid() -> *const u8 {
    CDC_CHIPID.ptr() as *const u8
}

pub fn chipid_init() -> R {
    if CONFIG_USB_SERIAL_NUMBER_CHIPID {
        usb_fill_serial(CDC_CHIPID.get(), CHIP_UID_LEN * 2, UID_BASE as usize as *const u8);
    }

    // (CONFIG_CANBUS: canserial_set_uuid() comes with the canbus code)
    Ok(())
}
decl_init!(chipid_init);
