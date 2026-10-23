// Generic handling of serial over CAN support
//
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <liquidpontus@yahoo.se>
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::{canhw_get_status, irq_restore, irq_save};
use crate::command::{command_dispatch, command_encode_and_frame, command_find_block, command_send_ack, CommandEncoder, SendArg};
use crate::config::CONFIG_HAVE_BOOTLOADER_REQUEST;
use crate::ctr::Global;
use crate::generic::canbus::{canbus_send, canbus_set_filter, CanbusMsg, CanbusStatus, CANMSG_DATA_LEN};
use crate::generic::fasthash::fasthash64;
use crate::sched::{sched_check_wake, sched_wake_task, TaskWake, R};
use crate::{decl_command, decl_constant, decl_enumeration, decl_shutdown, decl_task, sendf, shutdown};
use core::ptr::{addr_of_mut, read_volatile, write_volatile};

// canserial.h
pub const CANBUS_ID_ADMIN: u32 = 0x3f0;
pub const CANBUS_ID_ADMIN_RESP: u32 = 0x3f1;

const CANBUS_UUID_LEN: usize = 6;
const RECEIVE_BUF_SIZE: usize = 192;
const ADMIN_QUEUE_SIZE: usize = 8;

// Global storage
struct CanbusData {
    assigned_id: u32,
    uuid: [u8; CANBUS_UUID_LEN],

    // Tx data
    tx_wake: TaskWake,
    transmit_pos: u8,
    transmit_max: u8,

    // Rx data
    rx_wake: TaskWake,
    receive_pos: u8,
    admin_pull_pos: u32,
    admin_push_pos: u32,

    // Transfer buffers
    admin_queue: [CanbusMsg; ADMIN_QUEUE_SIZE],
    transmit_buf: [u8; 96],
    receive_buf: [u8; RECEIVE_BUF_SIZE],
}

static CAN_DATA: Global<CanbusData> = Global::new(CanbusData {
    assigned_id: 0,
    uuid: [0; CANBUS_UUID_LEN],
    tx_wake: TaskWake { wake: 0 },
    transmit_pos: 0,
    transmit_max: 0,
    rx_wake: TaskWake { wake: 0 },
    receive_pos: 0,
    admin_pull_pos: 0,
    admin_push_pos: 0,
    admin_queue: [CanbusMsg { id: 0, dlc: 0, data: [0; 8] }; ADMIN_QUEUE_SIZE],
    transmit_buf: [0; 96],
    receive_buf: [0; RECEIVE_BUF_SIZE],
});

fn cd() -> &'static mut CanbusData {
    CAN_DATA.get()
}

/****************************************************************
 * Data transmission over CAN
 ****************************************************************/

pub fn canserial_notify_tx() {
    sched_wake_task(&mut cd().tx_wake);
}

pub fn canserial_tx_task() -> R {
    let d = cd();
    if !sched_check_wake(&mut d.tx_wake) {
        return Ok(());
    }
    let id = d.assigned_id;
    if id == 0 {
        d.transmit_pos = 0;
        d.transmit_max = 0;
        return Ok(());
    }
    let mut msg = CanbusMsg { id: id + 1, dlc: 0, data: [0; 8] };
    let (mut tpos, tmax) = (d.transmit_pos as i32, d.transmit_max as i32);
    loop {
        let avail = tmax - tpos;
        let now = if avail > 8 { 8 } else { avail };
        if avail <= 0 {
            break;
        }
        msg.dlc = now as u32;
        msg.data[..now as usize].copy_from_slice(&d.transmit_buf[tpos as usize..(tpos + now) as usize]);
        let ret = canbus_send(&msg);
        if ret <= 0 {
            break;
        }
        tpos += now;
    }
    d.transmit_pos = tpos as u8;
    Ok(())
}
decl_task!(canserial_tx_task);

// Encode and transmit a "response" message
pub fn console_sendf(ce: &CommandEncoder, args: &[SendArg]) -> R {
    let d = cd();
    // Verify space for message
    let (mut tpos, mut tmax) = (d.transmit_pos as usize, d.transmit_max as usize);
    if tpos >= tmax {
        tpos = 0;
        tmax = 0;
        d.transmit_pos = 0;
        d.transmit_max = 0;
    }
    let max_size = ce.max_size as usize;
    if tmax + max_size > d.transmit_buf.len() {
        if tmax + max_size - tpos > d.transmit_buf.len() {
            // Not enough space for message
            return Ok(());
        }
        // Move buffer
        tmax -= tpos;
        d.transmit_buf.copy_within(tpos..tpos + tmax, 0);
        d.transmit_pos = 0;
        d.transmit_max = tmax as u8;
    }

    // Generate message
    let msglen = command_encode_and_frame(&mut d.transmit_buf[tmax..], ce, args)?;

    // Start message transmit
    d.transmit_max = (tmax + msglen) as u8;
    canserial_notify_tx();
    Ok(())
}

/****************************************************************
 * CAN "admin" command handling
 ****************************************************************/

// Available commands and responses
const CANBUS_CMD_QUERY_UNASSIGNED: u8 = 0x00;
const CANBUS_CMD_QUERY_EXTENDED: u8 = 0x01;
const CANBUS_CMD_SET_KLIPPER_NODEID: u8 = 0x01;
const CANBUS_CMD_REQUEST_BOOTLOADER: u8 = 0x02;

const CANBUS_RESP_KLIPPER_NODEID: u8 = 0x01;
const CANBUS_RESP_KALICO_NODEID: u8 = 0x07;
const CANBUS_RESP_NEED_NODEID: u8 = 0x20;
const CANBUS_RESP_HAVE_NODEID: u8 = 0x21;

// Helper to verify a UUID in a command matches this chip's UUID
fn can_check_uuid(msg: &CanbusMsg) -> bool {
    msg.dlc >= 7 && msg.data[1..1 + CANBUS_UUID_LEN] == cd().uuid
}

// Helpers to encode/decode a CAN identifier to a 1-byte "nodeid"
fn can_get_nodeid() -> u32 {
    let id = cd().assigned_id;
    if id == 0 {
        return 0;
    }
    id.wrapping_sub(0x100) >> 1
}
fn can_decode_nodeid(nodeid: u32) -> u32 {
    (nodeid << 1) + 0x100
}

fn can_process_query_unassigned(msg: &CanbusMsg) {
    let d = cd();
    // Kalico addition
    let is_extended_query = if msg.dlc > 1 { msg.data[1] & CANBUS_CMD_QUERY_EXTENDED } else { 0 };
    if d.assigned_id != 0 && is_extended_query == 0 {
        return;
    }
    let mut send = CanbusMsg { id: CANBUS_ID_ADMIN_RESP, dlc: 8, data: [0; 8] };
    send.data[0] = CANBUS_RESP_NEED_NODEID;
    send.data[1..1 + CANBUS_UUID_LEN].copy_from_slice(&d.uuid);

    if is_extended_query != 0 {
        if d.assigned_id != 0 {
            send.data[0] = CANBUS_RESP_HAVE_NODEID;
            send.data[7] = can_get_nodeid() as u8;
        } else {
            send.data[7] = CANBUS_RESP_KALICO_NODEID;
        }
    } else {
        send.data[7] = CANBUS_RESP_KLIPPER_NODEID;
    }

    // Send with retry
    loop {
        let ret = canbus_send(&send);
        if ret >= 0 {
            return;
        }
    }
}

fn can_id_conflict() -> R {
    cd().assigned_id = 0;
    canbus_set_filter(cd().assigned_id);
    shutdown!("Another CAN node assigned this ID");
}

fn can_process_set_klipper_nodeid(msg: &CanbusMsg) -> R {
    if msg.dlc < 8 {
        return Ok(());
    }
    let newid = can_decode_nodeid(msg.data[7] as u32);
    if can_check_uuid(msg) {
        if newid != cd().assigned_id {
            cd().assigned_id = newid;
            canbus_set_filter(cd().assigned_id);
        }
    } else if newid == cd().assigned_id {
        can_id_conflict()?;
    }
    Ok(())
}

fn can_process_request_bootloader(msg: &CanbusMsg) {
    if !CONFIG_HAVE_BOOTLOADER_REQUEST || !can_check_uuid(msg) {
        return;
    }
    crate::board::bootloader_request();
}

// Handle an "admin" command
fn can_process_admin(msg: &CanbusMsg) -> R {
    if msg.dlc == 0 {
        return Ok(());
    }
    match msg.data[0] {
        CANBUS_CMD_QUERY_UNASSIGNED => can_process_query_unassigned(msg),
        CANBUS_CMD_SET_KLIPPER_NODEID => can_process_set_klipper_nodeid(msg)?,
        CANBUS_CMD_REQUEST_BOOTLOADER => can_process_request_bootloader(msg),
        _ => {}
    }
    Ok(())
}

/****************************************************************
 * CAN packet reading
 ****************************************************************/

fn canserial_notify_rx() {
    sched_wake_task(&mut cd().rx_wake);
}

decl_constant!("RECEIVE_WINDOW", RECEIVE_BUF_SIZE);

// readb() / readl() of the C code
fn receive_pos() -> usize {
    unsafe { read_volatile(addr_of_mut!((*CAN_DATA.ptr()).receive_pos)) as usize }
}

// Handle incoming data (called from IRQ handler)
pub fn canserial_process_data(msg: &CanbusMsg) {
    let d = cd();
    let id = msg.id;
    if d.assigned_id != 0 && id == d.assigned_id {
        // Add to incoming data buffer
        let rpos = d.receive_pos as usize;
        let len = CANMSG_DATA_LEN(msg) as usize;
        if len > d.receive_buf.len() - rpos {
            return;
        }
        d.receive_buf[rpos..rpos + len].copy_from_slice(&msg.data[..len]);
        d.receive_pos = (rpos + len) as u8;
        canserial_notify_rx();
    } else if id == CANBUS_ID_ADMIN || (d.assigned_id != 0 && id == d.assigned_id + 1) {
        // Add to admin command queue
        let pushp = d.admin_push_pos;
        if pushp >= d.admin_pull_pos.wrapping_add(ADMIN_QUEUE_SIZE as u32) {
            // No space - drop message
            return;
        }
        let pos = (pushp % ADMIN_QUEUE_SIZE as u32) as usize;
        d.admin_queue[pos] = *msg;
        d.admin_push_pos = pushp + 1;
        canserial_notify_rx();
    }
}

// Remove from the receive buffer the given number of bytes
fn console_pop_input(len: usize) {
    let mut copied = 0;
    loop {
        let rpos = receive_pos();
        let needcopy = rpos - len;
        if needcopy != 0 {
            let buf = &mut cd().receive_buf;
            buf.copy_within(copied + len..copied + len + (needcopy - copied), copied);
            copied = needcopy;
            canserial_notify_rx();
        }
        let flag = irq_save();
        if rpos != receive_pos() {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        unsafe { write_volatile(addr_of_mut!((*CAN_DATA.ptr()).receive_pos), needcopy as u8) };
        irq_restore(flag);
        break;
    }
}

// Task to process incoming commands and admin messages
pub fn canserial_rx_task() -> R {
    if !sched_check_wake(&mut cd().rx_wake) {
        return Ok(());
    }

    // Process pending admin messages
    loop {
        let pushp = unsafe { read_volatile(addr_of_mut!((*CAN_DATA.ptr()).admin_push_pos)) };
        let pullp = cd().admin_pull_pos;
        if pushp == pullp {
            break;
        }
        let pos = (pullp % ADMIN_QUEUE_SIZE as u32) as usize;
        let msg = cd().admin_queue[pos];
        let id = msg.id;
        if cd().assigned_id != 0 && id == cd().assigned_id + 1 {
            can_id_conflict()?;
        } else if id == CANBUS_ID_ADMIN {
            can_process_admin(&msg)?;
        }
        cd().admin_pull_pos = pullp + 1;
    }

    // Check for a complete message block and process it
    let rpos = receive_pos();
    let buf = unsafe { core::slice::from_raw_parts(addr_of_mut!((*CAN_DATA.ptr()).receive_buf) as *const u8, rpos) };
    let mut pop_count = 0;
    let ret = command_find_block(buf, &mut pop_count)?;
    if ret > 0 {
        command_dispatch(buf, pop_count)?;
    }
    if ret != 0 {
        console_pop_input(pop_count);
        if ret > 0 {
            command_send_ack()?;
        }
    }
    Ok(())
}
decl_task!(canserial_rx_task);

/****************************************************************
 * Setup and shutdown
 ****************************************************************/

decl_enumeration!("canbus_bus_state", "active", CANBUS_STATE_ACTIVE);
decl_enumeration!("canbus_bus_state", "warn", CANBUS_STATE_WARN);
decl_enumeration!("canbus_bus_state", "passive", CANBUS_STATE_PASSIVE);
decl_enumeration!("canbus_bus_state", "off", CANBUS_STATE_OFF);

pub fn command_get_canbus_status(_args: &[u32]) -> R {
    let mut status = CanbusStatus::default();
    canhw_get_status(&mut status);
    sendf!(
        "canbus_status rx_error=%u tx_error=%u tx_retries=%u canbus_bus_state=%u",
        status.rx_error,
        status.tx_error,
        status.tx_retries,
        status.bus_state
    )
}
decl_command!(command_get_canbus_status, HF_IN_SHUTDOWN, "get_canbus_status");

pub fn command_get_canbus_id(_args: &[u32]) -> R {
    sendf!("canbus_id canbus_uuid=%.*s canbus_nodeid=%u", &cd().uuid, can_get_nodeid())
}
decl_command!(command_get_canbus_id, HF_IN_SHUTDOWN, "get_canbus_id");

pub fn canserial_set_uuid(raw_uuid: &[u8]) {
    let hash = fasthash64(raw_uuid, 0xA16231A7);
    cd().uuid.copy_from_slice(&hash.to_ne_bytes()[..CANBUS_UUID_LEN]);
    canserial_notify_rx();
}

pub fn canserial_shutdown() -> R {
    canserial_notify_tx();
    canserial_notify_rx();
    Ok(())
}
decl_shutdown!(canserial_shutdown);
