// I2C functions for rp2040
//
// Copyright (C) 2022  Lasse Dalegaard <dalegaard@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::I2cConfig;
use super::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time};
use crate::i2ccmds::{I2C_BUS_NACK, I2C_BUS_START_NACK, I2C_BUS_START_READ_NACK, I2C_BUS_SUCCESS, I2C_BUS_TIMEOUT};
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

struct I2cInfo {
    i2c: *mut i2c_hw_t,
    sda_pin: u8,
    scl_pin: u8,
    pclk: u8,
}
unsafe impl Sync for I2cInfo {}

decl_enumeration!("i2c_bus", "i2c0a", 0);
decl_enumeration!("i2c_bus", "i2c0b", 1);
decl_enumeration!("i2c_bus", "i2c0c", 2);
decl_enumeration!("i2c_bus", "i2c0d", 3);
decl_enumeration!("i2c_bus", "i2c0e", 4);
decl_enumeration!("i2c_bus", "i2c0f", 5);
decl_enumeration!("i2c_bus", "i2c0g", 6);
decl_enumeration!("i2c_bus", "i2c0h", 7);
decl_enumeration!("i2c_bus", "i2c1a", 8);
decl_enumeration!("i2c_bus", "i2c1b", 9);
decl_enumeration!("i2c_bus", "i2c1c", 10);
decl_enumeration!("i2c_bus", "i2c1d", 11);
decl_enumeration!("i2c_bus", "i2c1e", 12);
decl_enumeration!("i2c_bus", "i2c1f", 13);
decl_enumeration!("i2c_bus", "i2c1g", 14);
decl_constant_str!("BUS_PINS_i2c0a", "gpio0,gpio1");
decl_constant_str!("BUS_PINS_i2c0b", "gpio4,gpio5");
decl_constant_str!("BUS_PINS_i2c0c", "gpio8,gpio9");
decl_constant_str!("BUS_PINS_i2c0d", "gpio12,gpio13");
decl_constant_str!("BUS_PINS_i2c0e", "gpio16,gpio17");
decl_constant_str!("BUS_PINS_i2c0f", "gpio20,gpio21");
decl_constant_str!("BUS_PINS_i2c0g", "gpio24,gpio25");
decl_constant_str!("BUS_PINS_i2c0h", "gpio28,gpio29");
decl_constant_str!("BUS_PINS_i2c1a", "gpio2,gpio3");
decl_constant_str!("BUS_PINS_i2c1b", "gpio6,gpio7");
decl_constant_str!("BUS_PINS_i2c1c", "gpio10,gpio11");
decl_constant_str!("BUS_PINS_i2c1d", "gpio14,gpio15");
decl_constant_str!("BUS_PINS_i2c1e", "gpio18,gpio19");
decl_constant_str!("BUS_PINS_i2c1f", "gpio22,gpio23");
decl_constant_str!("BUS_PINS_i2c1g", "gpio26,gpio27");

const fn info(i2c: *mut i2c_hw_t, sda_pin: u8, scl_pin: u8, pclk: u32) -> I2cInfo {
    I2cInfo { i2c, sda_pin, scl_pin, pclk: pclk as u8 }
}

static I2C_BUS: [I2cInfo; 15] = [
    info(i2c0_hw, 0, 1, RESETS_RESET_I2C0_BITS),
    info(i2c0_hw, 4, 5, RESETS_RESET_I2C0_BITS),
    info(i2c0_hw, 8, 9, RESETS_RESET_I2C0_BITS),
    info(i2c0_hw, 12, 13, RESETS_RESET_I2C0_BITS),
    info(i2c0_hw, 16, 17, RESETS_RESET_I2C0_BITS),
    info(i2c0_hw, 20, 21, RESETS_RESET_I2C0_BITS),
    info(i2c0_hw, 24, 25, RESETS_RESET_I2C0_BITS),
    info(i2c0_hw, 28, 29, RESETS_RESET_I2C0_BITS),
    info(i2c1_hw, 2, 3, RESETS_RESET_I2C1_BITS),
    info(i2c1_hw, 6, 7, RESETS_RESET_I2C1_BITS),
    info(i2c1_hw, 10, 11, RESETS_RESET_I2C1_BITS),
    info(i2c1_hw, 14, 15, RESETS_RESET_I2C1_BITS),
    info(i2c1_hw, 18, 19, RESETS_RESET_I2C1_BITS),
    info(i2c1_hw, 22, 23, RESETS_RESET_I2C1_BITS),
    info(i2c1_hw, 26, 27, RESETS_RESET_I2C1_BITS),
];

// DIV_ROUND_CLOSEST of unsigned values
fn div_round_closest(x: u32, d: u32) -> u32 {
    (x + d / 2) / d
}

pub fn i2c_setup(bus: u32, rate: u32, addr: u8) -> R<I2cConfig> {
    if bus as usize >= I2C_BUS.len() {
        shutdown!("Invalid i2c bus");
    }

    let info = &I2C_BUS[bus as usize];

    gpio_peripheral(info.sda_pin as u32, 3, 1)?;
    gpio_peripheral(info.scl_pin as u32, 3, 1)?;

    let pclk_bit = info.pclk as u32;
    if !is_enabled_pclock(pclk_bit) {
        enable_pclock(pclk_bit);

        let i2c = unsafe { &*info.i2c };

        i2c.enable.write(0);

        // We set up the bus in 400 kHz mode, but then set timings afterwards
        // to match either 100k or 400k mode. This simplifies the setup.

        i2c.con.write(
            (I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB)
                | I2C_IC_CON_MASTER_MODE_BITS
                | I2C_IC_CON_IC_SLAVE_DISABLE_BITS
                | I2C_IC_CON_IC_RESTART_EN_BITS,
        );

        i2c.tx_tl.write(0);
        i2c.rx_tl.write(0);

        let pclk = get_pclock_frequency(pclk_bit);

        // See `i2c_set_baudrate` in the Pico SDK `hardware_i2c/i2c.c` file
        // for details on the calculations here.
        let period = if rate >= 1000000 {
            div_round_closest(pclk, 1000000)
        } else if rate >= 400000 {
            div_round_closest(pclk, 400000)
        } else {
            div_round_closest(pclk, 100000)
        };
        let lcnt = div_round_closest(period * ((3 << 16) / 5), 1 << 16); // 60%
        let hcnt = period - lcnt;
        let sda_tx_hold_count = ((pclk * 3) / 10000000) + 1;

        i2c.fs_scl_hcnt.write(hcnt);
        i2c.fs_scl_lcnt.write(lcnt);
        i2c.fs_spklen.write(if lcnt < 16 { 1 } else { lcnt / 16 });
        hw_write_masked(&i2c.sda_hold, sda_tx_hold_count << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB, I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS);
    }

    Ok(I2cConfig { i2c: info.i2c, addr })
}

fn i2c_start(i2c: &i2c_hw_t, addr: u8) {
    i2c.enable.write(0);
    i2c.tar.write(addr as u32);
    i2c.enable.write(1);
}

fn i2c_stop(i2c: &i2c_hw_t) {
    i2c.enable.write(0);
}

fn i2c_do_write(i2c: &i2c_hw_t, write: &[u8], send_stop: bool, timeout: u32) -> i32 {
    let write_len = write.len();
    for (i, &b) in write.iter().enumerate() {
        let first = (i == 0) as u32;
        let last = (send_stop && i == write_len - 1) as u32;

        // Wait until there's a spot in the TX FIFO
        while i2c.txflr.read() == 16 {
            if !timer_is_before(timer_read_time(), timeout) {
                return I2C_BUS_TIMEOUT;
            }
        }

        i2c.data_cmd.write((first << I2C_IC_DATA_CMD_RESTART_LSB) | (last << I2C_IC_DATA_CMD_STOP_LSB) | b as u32);
    }

    if !send_stop {
        return I2C_BUS_SUCCESS;
    }

    // Drain the transmit buffer
    while i2c.txflr.read() != 0 {
        if !timer_is_before(timer_read_time(), timeout) {
            return I2C_BUS_TIMEOUT;
        }
    }

    if i2c.raw_intr_stat.read() & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS != 0 {
        let abort_source = i2c.tx_abrt_source.read();
        if abort_source & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS != 0 {
            i2c.clr_tx_abrt.read();
            return I2C_BUS_START_NACK;
        }
        if abort_source & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS != 0 {
            i2c.clr_tx_abrt.read();
            return I2C_BUS_NACK;
        }
    }
    I2C_BUS_SUCCESS
}

fn i2c_do_read(i2c: &i2c_hw_t, read: &mut [u8], timeout: u32) -> i32 {
    let read_len = read.len();
    let mut have_read = 0;
    let mut to_send = read_len;
    while have_read < read_len {
        if !timer_is_before(timer_read_time(), timeout) {
            return I2C_BUS_TIMEOUT;
        }

        if to_send > 0 && i2c.txflr.read() < 16 {
            let first = (to_send == read_len) as u32;
            let last = (to_send == 1) as u32;

            // Put a read command in the TX FIFO
            i2c.data_cmd.write((first << I2C_IC_DATA_CMD_RESTART_LSB) | (last << I2C_IC_DATA_CMD_STOP_LSB) | I2C_IC_DATA_CMD_CMD_BITS);
            to_send -= 1;
        }

        if have_read < read_len && i2c.rxflr.read() > 0 {
            read[have_read] = (i2c.data_cmd.read() & 0xFF) as u8;
            have_read += 1;
        }

        if i2c.raw_intr_stat.read() & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS != 0 {
            let abort_source = i2c.tx_abrt_source.read();
            if abort_source & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS != 0 {
                i2c.clr_tx_abrt.read();
                return I2C_BUS_START_READ_NACK;
            }
        }
    }

    I2C_BUS_SUCCESS
}

pub fn i2c_write(config: I2cConfig, write: &[u8]) -> R<i32> {
    let i2c = unsafe { &*config.i2c };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));

    i2c_start(i2c, config.addr);
    let ret = i2c_do_write(i2c, write, true, timeout);
    i2c_stop(i2c);
    Ok(ret)
}

pub fn i2c_read(config: I2cConfig, reg: &[u8], read: &mut [u8]) -> R<i32> {
    let i2c = unsafe { &*config.i2c };
    let timeout = timer_read_time().wrapping_add(timer_from_us(5000));
    let mut ret = I2C_BUS_SUCCESS;

    i2c_start(i2c, config.addr);
    if !reg.is_empty() {
        ret = i2c_do_write(i2c, reg, false, timeout);
    }
    if ret == I2C_BUS_SUCCESS {
        ret = i2c_do_read(i2c, read, timeout);
    }
    i2c_stop(i2c);
    Ok(ret)
}
