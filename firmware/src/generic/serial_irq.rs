// Generic interrupt based serial uart helper code
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::{irq_restore, irq_save, serial_enable_tx_irq};
use crate::command::{command_dispatch, command_encode_and_frame, command_find_block, command_send_ack, CommandEncoder, SendArg, MESSAGE_SYNC};
use crate::ctr::Global;
use crate::sched::{sched_wake_tasks, R};
use crate::{decl_constant, decl_task};
use core::ptr::{read_volatile, write_volatile};

const RX_BUFFER_SIZE: usize = 192;

static RECEIVE_BUF: Global<[u8; RX_BUFFER_SIZE]> = Global::new([0; RX_BUFFER_SIZE]);
static RECEIVE_POS: Global<u8> = Global::new(0);
static TRANSMIT_BUF: Global<[u8; 96]> = Global::new([0; 96]);
static TRANSMIT_POS: Global<u8> = Global::new(0);
static TRANSMIT_MAX: Global<u8> = Global::new(0);

decl_constant!("SERIAL_BAUD", CONFIG_SERIAL_BAUD);
decl_constant!("RECEIVE_WINDOW", RX_BUFFER_SIZE);

// readb() / writeb() of the C code
fn readb(v: &Global<u8>) -> u8 {
    unsafe { read_volatile(v.ptr()) }
}
fn writeb(v: &Global<u8>, b: u8) {
    unsafe { write_volatile(v.ptr(), b) }
}

// Rx interrupt - store read data
pub fn serial_rx_byte(data: u8) {
    if data == MESSAGE_SYNC {
        sched_wake_tasks();
    }
    let pos = readb(&RECEIVE_POS) as usize;
    if pos >= RX_BUFFER_SIZE {
        // Serial overflow - ignore it as crc error will force retransmit
        return;
    }
    unsafe { write_volatile((RECEIVE_BUF.ptr() as *mut u8).add(pos), data) };
    writeb(&RECEIVE_POS, pos as u8 + 1);
}

// Tx interrupt - get next byte to transmit
pub fn serial_get_tx_byte(pdata: &mut u8) -> i32 {
    let pos = readb(&TRANSMIT_POS);
    if pos >= readb(&TRANSMIT_MAX) {
        return -1;
    }
    *pdata = unsafe { read_volatile((TRANSMIT_BUF.ptr() as *const u8).add(pos as usize)) };
    writeb(&TRANSMIT_POS, pos + 1);
    0
}

// Remove from the receive buffer the given number of bytes
fn console_pop_input(len: usize) {
    let buf = RECEIVE_BUF.ptr() as *mut u8;
    let mut copied = 0;
    loop {
        let rpos = readb(&RECEIVE_POS) as usize;
        let needcopy = rpos - len;
        if needcopy != 0 {
            unsafe { core::ptr::copy(buf.add(copied + len), buf.add(copied), needcopy - copied) };
            copied = needcopy;
            sched_wake_tasks();
        }
        let flag = irq_save();
        if rpos != readb(&RECEIVE_POS) as usize {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        writeb(&RECEIVE_POS, needcopy as u8);
        irq_restore(flag);
        break;
    }
}

// Process any incoming commands
pub fn console_task() -> R {
    let rpos = readb(&RECEIVE_POS) as usize;
    let buf = unsafe { core::slice::from_raw_parts(RECEIVE_BUF.ptr() as *const u8, rpos) };
    let mut pop_count = 0;
    let ret = command_find_block(buf, &mut pop_count)?;
    if ret > 0 {
        command_dispatch(buf, pop_count)?;
    }
    if ret != 0 {
        // (C: if (CONFIG_HAVE_BOOTLOADER_REQUEST && ...); boards without
        // it have no bootloader_request())
        #[cfg(CONFIG_HAVE_BOOTLOADER_REQUEST)]
        if ret < 0 && pop_count == 32 && buf[..32] == *b" \x1c Request Serial Bootloader!! ~" {
            crate::board::bootloader_request();
        }
        console_pop_input(pop_count);
        if ret > 0 {
            command_send_ack()?;
        }
    }
    Ok(())
}
decl_task!(console_task);

// Encode and transmit a "response" message
pub fn console_sendf(ce: &CommandEncoder, args: &[SendArg]) -> R {
    let buf = TRANSMIT_BUF.get();
    // Verify space for message
    let (mut tpos, mut tmax) = (readb(&TRANSMIT_POS) as usize, readb(&TRANSMIT_MAX) as usize);
    if tpos >= tmax {
        tpos = 0;
        tmax = 0;
        writeb(&TRANSMIT_MAX, 0);
        writeb(&TRANSMIT_POS, 0);
    }
    let max_size = ce.max_size as usize;
    if tmax + max_size > buf.len() {
        if tmax + max_size - tpos > buf.len() {
            // Not enough space for message
            return Ok(());
        }
        // Disable TX irq and move buffer
        writeb(&TRANSMIT_MAX, 0);
        tpos = readb(&TRANSMIT_POS) as usize;
        tmax -= tpos;
        buf.copy_within(tpos..tpos + tmax, 0);
        writeb(&TRANSMIT_POS, 0);
        writeb(&TRANSMIT_MAX, tmax as u8);
        serial_enable_tx_irq();
    }

    // Generate message
    let msglen = command_encode_and_frame(&mut buf[tmax..], ce, args)?;

    // Start message transmit
    writeb(&TRANSMIT_MAX, (tmax + msglen) as u8);
    serial_enable_tx_irq();
    Ok(())
}
