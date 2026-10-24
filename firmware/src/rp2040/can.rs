// Serial over CAN emulation for rp2040 using can2040 software canbus
//
// Copyright (C) 2022-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::can2040::*;
use super::internal::*;
use crate::config::{CONFIG_CANBUS_FREQUENCY, CONFIG_RPXXXX_CANBUS_GPIO_RX, CONFIG_RPXXXX_CANBUS_GPIO_TX};
use crate::ctr::Global;
use crate::generic::canbus::{canbus_notify_tx, canbus_process_data, CanbusMsg, CanbusStatus, CANBUS_STATE_ACTIVE, CANMSG_DATA_LEN};
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant_str, decl_init};

decl_constant_str!(
    "RESERVE_PINS_CAN",
    concat!("gpio", CONFIG_RPXXXX_CANBUS_GPIO_RX, ",", "gpio", CONFIG_RPXXXX_CANBUS_GPIO_TX)
);

static CBUS: Global<Can2040> = Global::new(Can2040::new());

// Transmit a packet
pub fn canhw_send(msg: &CanbusMsg) -> i32 {
    let ret = can2040_transmit(CBUS.get(), msg);
    if ret < 0 {
        return -1;
    }
    CANMSG_DATA_LEN(msg) as i32
}

// Setup the receive packet filter
pub fn canhw_set_filter(_id: u32) {
    // Filter not implemented (and not necessary)
}

static LAST_TX_RETRIES: Global<u32> = Global::new(0);

// Report interface status
pub fn canhw_get_status(status: &mut CanbusStatus) {
    let mut stats = Can2040Stats { rx_total: 0, tx_total: 0, tx_attempt: 0, parse_error: 0 };
    can2040_get_statistics(CBUS.get(), &mut stats);
    let tx_extra = stats.tx_attempt.wrapping_sub(stats.tx_total);
    let last_tx_retries = LAST_TX_RETRIES.get();
    if *last_tx_retries != tx_extra {
        *last_tx_retries = tx_extra.wrapping_sub(1);
    }

    status.rx_error = stats.parse_error;
    status.tx_retries = *last_tx_retries;
    status.bus_state = CANBUS_STATE_ACTIVE;
}

// can2040 callback function - handle rx and tx notifications
unsafe fn can2040_cb(_cd: *mut Can2040, notify: u32, msg: *mut Can2040Msg) {
    if notify & CAN2040_NOTIFY_TX != 0 {
        canbus_notify_tx();
        return;
    }
    if notify & CAN2040_NOTIFY_RX != 0 {
        canbus_process_data(&*msg);
    }
}

// Main PIO irq handler
#[unsafe(no_mangle)]
pub unsafe extern "C" fn PIOx_IRQHandler() {
    can2040_pio_irq_handler(CBUS.get());
}

pub fn can_init() -> R {
    // Setup canbus
    let cbus = CBUS.get();
    can2040_setup(cbus, 0);
    can2040_callback_config(cbus, can2040_cb);

    // Enable irqs
    armcm_enable_irq!(PIOx_IRQHandler, PIO0_IRQ_0_IRQn, 1);

    // Start canbus
    let pclk = get_pclock_frequency(RESETS_RESET_PIO0_RESET);
    can2040_start(cbus, pclk, CONFIG_CANBUS_FREQUENCY, CONFIG_RPXXXX_CANBUS_GPIO_RX, CONFIG_RPXXXX_CANBUS_GPIO_TX);
    Ok(())
}
decl_init!(can_init);
