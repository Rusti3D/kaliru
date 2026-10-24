// Wrapper functions connecting canserial.c to low-level can hardware
//
// Copyright (C) 2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#[cfg(CONFIG_CANSERIAL)]
use crate::board::{canhw_send, canhw_set_filter};
use crate::decl_constant;
#[cfg(CONFIG_CANSERIAL)]
use crate::generic::canserial::{canserial_notify_tx, canserial_process_data};

pub use crate::generic::canbus_h::*;

decl_constant!("CANBUS_FREQUENCY", CONFIG_CANBUS_FREQUENCY);

// (the functions of canbus.c; in the usb to canbus bridge mode they are
// usb_canbus.c's)
#[cfg(CONFIG_CANSERIAL)]
pub fn canbus_send(msg: &CanbusMsg) -> i32 {
    canhw_send(msg)
}

#[cfg(CONFIG_CANSERIAL)]
pub fn canbus_set_filter(id: u32) {
    canhw_set_filter(id);
}

#[cfg(CONFIG_CANSERIAL)]
pub fn canbus_notify_tx() {
    canserial_notify_tx();
}

#[cfg(CONFIG_CANSERIAL)]
pub fn canbus_process_data(msg: &CanbusMsg) {
    canserial_process_data(msg);
}
#[cfg(CONFIG_USBCANBUS)]
pub use crate::generic::usb_canbus::{canbus_notify_tx, canbus_process_data, canbus_send, canbus_set_filter};
