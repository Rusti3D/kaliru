// Example code for interacting with serial_irq.c
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::generic::serial_irq::serial_get_tx_byte;
use crate::sched::R;
use crate::decl_init;

pub fn serial_init() -> R {
    // Make stdin/stdout non-blocking
    unsafe {
        libc::fcntl(libc::STDIN_FILENO, libc::F_SETFL, libc::fcntl(libc::STDIN_FILENO, libc::F_GETFL, 0) | libc::O_NONBLOCK);
        libc::fcntl(libc::STDOUT_FILENO, libc::F_SETFL, libc::fcntl(libc::STDOUT_FILENO, libc::F_GETFL, 0) | libc::O_NONBLOCK);
    }
    Ok(())
}
decl_init!(serial_init);

pub fn console_receive_buffer() -> *mut u8 {
    core::ptr::null_mut()
}

fn do_uart() {
    loop {
        let mut data = 0u8;
        let ret = serial_get_tx_byte(&mut data);
        if ret != 0 {
            break;
        } else {
            unsafe { libc::write(libc::STDOUT_FILENO, &data as *const u8 as *const libc::c_void, 1) };
        }

        // XXX - Normally the code would check if input data is
        // available and call serial_rx_byte()
    }
}

pub fn serial_enable_tx_irq() {
    // Normally this would enable the hardware irq, but we just call
    // do_uart() directly in this demo code.
    do_uart();
}
