// Wrapper functions connecting canserial.c to low-level can hardware
//
// Copyright (C) 2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::{canhw_send, canhw_set_filter};
use crate::decl_constant;
use crate::generic::canserial::{canserial_notify_tx, canserial_process_data};

// canbus.h
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CanbusMsg {
    pub id: u32,
    pub dlc: u32,
    // (a union with uint32_t data32[2])
    pub data: [u8; 8],
}

impl CanbusMsg {
    pub fn data32(&self, i: usize) -> u32 {
        u32::from_ne_bytes([self.data[i * 4], self.data[i * 4 + 1], self.data[i * 4 + 2], self.data[i * 4 + 3]])
    }
    pub fn set_data32(&mut self, i: usize, v: u32) {
        self.data[i * 4..i * 4 + 4].copy_from_slice(&v.to_ne_bytes());
    }
}

pub const CANMSG_ID_RTR: u32 = 1 << 30;
pub const CANMSG_ID_EFF: u32 = 1 << 31;

#[allow(non_snake_case)]
pub fn CANMSG_DATA_LEN(msg: &CanbusMsg) -> u32 {
    if msg.dlc > 8 {
        8
    } else {
        msg.dlc
    }
}

#[derive(Default)]
pub struct CanbusStatus {
    pub rx_error: u32,
    pub tx_error: u32,
    pub tx_retries: u32,
    pub bus_state: u32,
}

pub const CANBUS_STATE_ACTIVE: u32 = 0;
pub const CANBUS_STATE_WARN: u32 = 1;
pub const CANBUS_STATE_PASSIVE: u32 = 2;
pub const CANBUS_STATE_OFF: u32 = 3;

decl_constant!("CANBUS_FREQUENCY", CONFIG_CANBUS_FREQUENCY);

pub fn canbus_send(msg: &CanbusMsg) -> i32 {
    canhw_send(msg)
}

pub fn canbus_set_filter(id: u32) {
    canhw_set_filter(id);
}

pub fn canbus_notify_tx() {
    canserial_notify_tx();
}

pub fn canbus_process_data(msg: &CanbusMsg) {
    canserial_process_data(msg);
}
