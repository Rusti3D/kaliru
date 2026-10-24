// Support for extracting the hardware chip id on sam3/sam4
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::{irq_disable, irq_enable};
use crate::config::CONFIG_USB_SERIAL_NUMBER_CHIPID;
use crate::decl_init;
use crate::sched::R;
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
use crate::{ctr::Global, generic::usb_cdc::usb_fill_serial, generic::usbstd::UsbStringDescriptor};

const CHIP_UID_LEN: usize = 16;

// (struct { struct usb_string_descriptor desc; uint16_t data[CHIP_UID_LEN * 2]; })
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
static CDC_CHIPID: Global<UsbStringDescriptor<{ CHIP_UID_LEN * 2 }>> =
    Global::new(UsbStringDescriptor { bLength: 0, bDescriptorType: 0, data: [0; CHIP_UID_LEN * 2] });

// the serial string descriptor (its first byte is its bLength)
#[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
pub fn usbserial_get_serialid() -> *const u8 {
    CDC_CHIPID.ptr() as *const u8
}

// Compatibility definitions for sam4e8e
#[cfg(has_EFC0)]
const EFC0_: *mut Efc = EFC0;
#[cfg(has_EFC0)]
const IFLASH0_ADDR_: u32 = IFLASH0_ADDR;
#[cfg(not(has_EFC0))]
const EFC0_: *mut Efc = EFC;
#[cfg(not(has_EFC0))]
const IFLASH0_ADDR_: u32 = IFLASH_ADDR;

#[inline(never)]
#[unsafe(link_section = ".ramfunc.read_chip_id")]
pub fn read_chip_id(id: &mut [u32; 4]) {
    let efc = unsafe { &*EFC0_ };
    // Workaround sam3 errata
    let fmr = efc.EEFC_FMR.read();
    efc.EEFC_FMR.write(fmr | EEFC_FMR_SCOD);

    // Send the STUI command
    while efc.EEFC_FSR.read() & EEFC_FSR_FRDY == 0 {}
    efc.EEFC_FCR.write(EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_STUI);
    while efc.EEFC_FSR.read() & EEFC_FSR_FRDY != 0 {}

    // Copy the id
    for (i, v) in id.iter_mut().enumerate() {
        *v = unsafe { ((IFLASH0_ADDR_ as usize + i * 4) as *const u32).read_volatile() };
    }

    // Send the SPUI command
    efc.EEFC_FCR.write(EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SPUI);
    while efc.EEFC_FSR.read() & EEFC_FSR_FRDY == 0 {}

    // Restore fmr
    efc.EEFC_FMR.write(fmr);
}

pub fn chipid_init() -> R {
    if !CONFIG_USB_SERIAL_NUMBER_CHIPID && !cfg!(CONFIG_CANBUS) {
        return Ok(());
    }

    let mut id = [0u32; 4];
    irq_disable();
    read_chip_id(&mut id);
    irq_enable();

    #[cfg(any(CONFIG_USBSERIAL, CONFIG_USBCANBUS))]
    if CONFIG_USB_SERIAL_NUMBER_CHIPID {
        usb_fill_serial(CDC_CHIPID.get(), CHIP_UID_LEN * 2, id.as_ptr() as *const u8);
    }

    #[cfg(CONFIG_CANBUS)]
    {
        use crate::config::{CONFIG_CAN_UUID_CUSTOM, CONFIG_CAN_UUID_USE_CHIPID};
        use crate::generic::canserial::canserial_set_uuid;
        if CONFIG_CAN_UUID_USE_CHIPID {
            canserial_set_uuid(unsafe { core::slice::from_raw_parts(id.as_ptr() as *const u8, CHIP_UID_LEN) });
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
