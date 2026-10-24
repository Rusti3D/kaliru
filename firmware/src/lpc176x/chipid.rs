// Support for extracting the hardware chip id on lpc176x
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::{irq_disable, irq_enable};
use crate::config::CONFIG_USB_SERIAL_NUMBER_CHIPID;
use crate::decl_init;
use crate::sched::R;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
use crate::{ctr::Global, generic::usb_cdc::usb_fill_serial, generic::usbstd::UsbStringDescriptor};

// IAP interface
const IAP_LOCATION: usize = 0x1fff1ff1;
const IAP_CMD_READ_UID: u32 = 58;
const IAP_UID_LEN: usize = 16;
type Iap = extern "C" fn(*mut u32, *mut u32);

// (struct { struct usb_string_descriptor desc; uint16_t data[IAP_UID_LEN * 2]; })
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
static CDC_CHIPID: Global<UsbStringDescriptor<{ IAP_UID_LEN * 2 }>> =
    Global::new(UsbStringDescriptor { bLength: 0, bDescriptorType: 0, data: [0; IAP_UID_LEN * 2] });

// the serial string descriptor (its first byte is its bLength)
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub fn usbserial_get_serialid() -> *const u8 {
    CDC_CHIPID.ptr() as *const u8
}

pub fn chipid_init() -> R {
    if !CONFIG_USB_SERIAL_NUMBER_CHIPID && !cfg!(CONFIG_CANBUS) {
        return Ok(());
    }

    let mut iap_cmd_uid: [u32; 5] = [IAP_CMD_READ_UID, 0, 0, 0, 0];
    let mut iap_resp = [0u32; 5];
    let iap_entry: Iap = unsafe { core::mem::transmute(IAP_LOCATION) };
    irq_disable();
    iap_entry(iap_cmd_uid.as_mut_ptr(), iap_resp.as_mut_ptr());
    irq_enable();

    #[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
    if CONFIG_USB_SERIAL_NUMBER_CHIPID {
        usb_fill_serial(CDC_CHIPID.get(), IAP_UID_LEN * 2, iap_resp[1..].as_ptr() as *const u8);
    }

    #[cfg(CONFIG_CANBUS)]
    crate::generic::canserial::canserial_set_uuid(unsafe {
        core::slice::from_raw_parts(iap_resp[1..].as_ptr() as *const u8, IAP_UID_LEN)
    });
    Ok(())
}
decl_init!(chipid_init);
