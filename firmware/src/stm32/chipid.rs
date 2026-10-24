// Support for extracting the hardware chip id on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#[allow(unused_imports)]
use super::internal::UID_BASE;
use crate::decl_init;
use crate::sched::R;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
use crate::{config::CONFIG_USB_SERIAL_NUMBER_CHIPID, ctr::Global, generic::usb_cdc::usb_fill_serial, generic::usbstd::UsbStringDescriptor};

const CHIP_UID_LEN: usize = 12;

// (struct { struct usb_string_descriptor desc; uint16_t data[CHIP_UID_LEN * 2]; })
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
static CDC_CHIPID: Global<UsbStringDescriptor<{ CHIP_UID_LEN * 2 }>> =
    Global::new(UsbStringDescriptor { bLength: 0, bDescriptorType: 0, data: [0; CHIP_UID_LEN * 2] });

// the serial string descriptor (its first byte is its bLength)
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub fn usbserial_get_serialid() -> *const u8 {
    CDC_CHIPID.ptr() as *const u8
}

pub fn chipid_init() -> R {
    // (usb_fill_serial() is part of the usb code)
    #[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
    if CONFIG_USB_SERIAL_NUMBER_CHIPID {
        usb_fill_serial(CDC_CHIPID.get(), CHIP_UID_LEN * 2, UID_BASE as usize as *const u8);
    }

    #[cfg(CONFIG_CANBUS)]
    {
        use crate::config::{CONFIG_CAN_UUID_CUSTOM, CONFIG_CAN_UUID_USE_CHIPID};
        use crate::generic::canserial::canserial_set_uuid;
        if CONFIG_CAN_UUID_USE_CHIPID {
            canserial_set_uuid(unsafe { core::slice::from_raw_parts(UID_BASE as usize as *const u8, CHIP_UID_LEN) });
        } else {
            // (C reads CHIP_UID_LEN bytes from the string, past its end when
            // it is shorter: here the rest is zeros)
            let mut raw = [0u8; CHIP_UID_LEN];
            let s = CONFIG_CAN_UUID_CUSTOM.as_bytes();
            let n = s.len().min(CHIP_UID_LEN);
            raw[..n].copy_from_slice(&s[..n]);
            canserial_set_uuid(&raw);
        }
    }
    Ok(())
}
decl_init!(chipid_init);
