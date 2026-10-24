// Handling of sercom pins
//
// Copyright (C) 2019  Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::command::Args;
use crate::ctr::{concat, concat_len, Global};
use crate::sched::R;
use crate::{decl_command, decl_enumeration, decl_enumeration_range, shutdown};

/****************************************************************
 * Available sercom blocks
 ****************************************************************/

decl_enumeration_range!("bus", "sercom0", 0, 8);

#[derive(Clone, Copy)]
struct SercomBus {
    sercom: *mut Sercom,
    pclk_id: u32,
    pm_id: u32,
}
unsafe impl Sync for SercomBus {}

const fn sb(sercom: *mut Sercom, pclk_id: u32, pm_id: u32) -> SercomBus {
    SercomBus { sercom, pclk_id, pm_id }
}

const SERCOMS_0_3: &[SercomBus] = &[
    sb(SERCOM0, SERCOM0_GCLK_ID_CORE, ID_SERCOM0),
    sb(SERCOM1, SERCOM1_GCLK_ID_CORE, ID_SERCOM1),
    sb(SERCOM2, SERCOM2_GCLK_ID_CORE, ID_SERCOM2),
    sb(SERCOM3, SERCOM3_GCLK_ID_CORE, ID_SERCOM3),
];
#[cfg(has_SERCOM4)]
const SERCOMS_4_5: &[SercomBus] =
    &[sb(SERCOM4, SERCOM4_GCLK_ID_CORE, ID_SERCOM4), sb(SERCOM5, SERCOM5_GCLK_ID_CORE, ID_SERCOM5)];
#[cfg(not(has_SERCOM4))]
const SERCOMS_4_5: &[SercomBus] = &[];
#[cfg(has_SERCOM6)]
const SERCOMS_6_7: &[SercomBus] =
    &[sb(SERCOM6, SERCOM6_GCLK_ID_CORE, ID_SERCOM6), sb(SERCOM7, SERCOM7_GCLK_ID_CORE, ID_SERCOM7)];
#[cfg(not(has_SERCOM6))]
const SERCOMS_6_7: &[SercomBus] = &[];
const SERCOMS_PARTS: &[&[SercomBus]] = &[SERCOMS_0_3, SERCOMS_4_5, SERCOMS_6_7];
const NUM_SERCOMS: usize = concat_len(SERCOMS_PARTS);
static SERCOMS: [SercomBus; NUM_SERCOMS] = concat(SERCOMS_PARTS);

pub fn sercom_enable_pclock(sercom_id: u32) -> R<*mut Sercom> {
    if sercom_id as usize >= SERCOMS.len() {
        shutdown!("Invalid SERCOM bus");
    }
    let sb = &SERCOMS[sercom_id as usize];
    enable_pclock(sb.pclk_id, sb.pm_id as i32);
    Ok(sb.sercom)
}

pub fn sercom_get_pclock_frequency(sercom_id: u32) -> u32 {
    let sb = &SERCOMS[sercom_id as usize];
    get_pclock_frequency(sb.pclk_id)
}

/****************************************************************
 * Pin configurations
 ****************************************************************/

#[derive(Clone, Copy)]
struct SercomPad {
    sercom_id: u8,
    gpio: u8,
    pad: u8,
    ptype: u8,
}

const fn pad(sercom_id: u8, gpio: u32, pad: u8, ptype: char) -> SercomPad {
    SercomPad { sercom_id, gpio: gpio as u8, pad, ptype: ptype as u8 }
}

#[cfg(CONFIG_MACH_SAMX2)]
const SERCOM_PADS_BASE: &[SercomPad] = &[
    pad(0, GPIO('A', 8), 0, 'C'),
    pad(0, GPIO('A', 9), 1, 'C'),
    pad(0, GPIO('A', 10), 2, 'C'),
    pad(0, GPIO('A', 11), 3, 'C'),
    pad(0, GPIO('A', 4), 0, 'D'),
    pad(0, GPIO('A', 5), 1, 'D'),
    pad(0, GPIO('A', 6), 2, 'D'),
    pad(0, GPIO('A', 7), 3, 'D'),
    pad(1, GPIO('A', 16), 0, 'C'),
    pad(1, GPIO('A', 17), 1, 'C'),
    pad(1, GPIO('A', 18), 2, 'C'),
    pad(1, GPIO('A', 19), 3, 'C'),
    pad(1, GPIO('A', 0), 0, 'D'),
    pad(1, GPIO('A', 1), 1, 'D'),
    pad(1, GPIO('A', 30), 2, 'D'),
    pad(1, GPIO('A', 31), 3, 'D'),
    pad(2, GPIO('A', 12), 0, 'C'),
    pad(2, GPIO('A', 13), 1, 'C'),
    pad(2, GPIO('A', 14), 2, 'C'),
    pad(2, GPIO('A', 15), 3, 'C'),
    pad(2, GPIO('A', 8), 0, 'D'),
    pad(2, GPIO('A', 9), 1, 'D'),
    pad(2, GPIO('A', 10), 2, 'D'),
    pad(2, GPIO('A', 11), 3, 'D'),
    pad(3, GPIO('A', 22), 0, 'C'),
    pad(3, GPIO('A', 23), 1, 'C'),
    pad(3, GPIO('A', 24), 2, 'C'),
    pad(3, GPIO('A', 25), 3, 'C'),
    pad(3, GPIO('A', 16), 0, 'D'),
    pad(3, GPIO('A', 17), 1, 'D'),
    pad(3, GPIO('A', 18), 2, 'D'),
    pad(3, GPIO('A', 19), 3, 'D'),
    pad(3, GPIO('A', 20), 2, 'D'),
    pad(3, GPIO('A', 21), 3, 'D'),
    pad(4, GPIO('B', 12), 0, 'C'),
    pad(4, GPIO('B', 13), 1, 'C'),
    pad(4, GPIO('B', 14), 2, 'C'),
    pad(4, GPIO('B', 15), 3, 'C'),
    pad(4, GPIO('B', 8), 0, 'D'),
    pad(4, GPIO('B', 9), 1, 'D'),
    pad(4, GPIO('B', 10), 2, 'D'),
    pad(4, GPIO('B', 11), 3, 'D'),
    pad(4, GPIO('A', 12), 0, 'D'),
    pad(4, GPIO('A', 13), 1, 'D'),
    pad(4, GPIO('A', 14), 2, 'D'),
    pad(4, GPIO('A', 15), 3, 'D'),
    pad(5, GPIO('B', 16), 0, 'C'),
    pad(5, GPIO('B', 17), 1, 'C'),
    pad(5, GPIO('A', 20), 2, 'C'),
    pad(5, GPIO('A', 21), 3, 'C'),
    pad(5, GPIO('A', 22), 0, 'D'),
    pad(5, GPIO('A', 23), 1, 'D'),
    pad(5, GPIO('A', 24), 2, 'D'),
    pad(5, GPIO('A', 25), 3, 'D'),
    pad(5, GPIO('B', 30), 0, 'D'),
    pad(5, GPIO('B', 31), 1, 'D'),
    pad(5, GPIO('B', 22), 2, 'D'),
    pad(5, GPIO('B', 23), 3, 'D'),
    pad(5, GPIO('B', 2), 0, 'D'),
    pad(5, GPIO('B', 3), 1, 'D'),
    pad(5, GPIO('B', 0), 2, 'D'),
    pad(5, GPIO('B', 1), 3, 'D'),
];
#[cfg(CONFIG_MACH_SAMX5)]
const SERCOM_PADS_BASE: &[SercomPad] = &[
    pad(0, GPIO('A', 8), 0, 'C'),
    pad(0, GPIO('A', 9), 1, 'C'),
    pad(0, GPIO('A', 10), 2, 'C'),
    pad(0, GPIO('A', 11), 3, 'C'),
    pad(0, GPIO('B', 24), 0, 'C'),
    pad(0, GPIO('B', 25), 1, 'C'),
    pad(0, GPIO('C', 24), 2, 'C'),
    pad(0, GPIO('C', 25), 3, 'C'),
    pad(0, GPIO('A', 4), 0, 'D'),
    pad(0, GPIO('A', 5), 1, 'D'),
    pad(0, GPIO('A', 6), 2, 'D'),
    pad(0, GPIO('A', 7), 3, 'D'),
    pad(0, GPIO('C', 17), 0, 'D'),
    pad(0, GPIO('C', 16), 1, 'D'),
    pad(0, GPIO('C', 18), 2, 'D'),
    pad(0, GPIO('C', 19), 3, 'D'),
    pad(1, GPIO('A', 16), 0, 'C'),
    pad(1, GPIO('A', 17), 1, 'C'),
    pad(1, GPIO('A', 18), 2, 'C'),
    pad(1, GPIO('A', 19), 3, 'C'),
    pad(1, GPIO('C', 22), 0, 'C'),
    pad(1, GPIO('C', 23), 1, 'C'),
    pad(1, GPIO('D', 20), 2, 'C'),
    pad(1, GPIO('D', 21), 3, 'C'),
    pad(1, GPIO('C', 27), 0, 'C'),
    pad(1, GPIO('C', 28), 1, 'C'),
    pad(1, GPIO('B', 22), 2, 'C'),
    pad(1, GPIO('B', 23), 3, 'C'),
    pad(1, GPIO('A', 0), 0, 'D'),
    pad(1, GPIO('A', 1), 1, 'D'),
    pad(1, GPIO('A', 30), 2, 'D'),
    pad(1, GPIO('A', 31), 3, 'D'),
    pad(2, GPIO('A', 12), 0, 'C'),
    pad(2, GPIO('A', 13), 1, 'C'),
    pad(2, GPIO('A', 14), 2, 'C'),
    pad(2, GPIO('A', 15), 3, 'C'),
    pad(2, GPIO('B', 26), 0, 'C'),
    pad(2, GPIO('B', 27), 1, 'C'),
    pad(2, GPIO('B', 28), 2, 'C'),
    pad(2, GPIO('B', 29), 3, 'C'),
    pad(2, GPIO('A', 9), 0, 'D'),
    pad(2, GPIO('A', 8), 1, 'D'),
    pad(2, GPIO('A', 10), 2, 'D'),
    pad(2, GPIO('A', 11), 3, 'D'),
    pad(2, GPIO('B', 25), 0, 'D'),
    pad(2, GPIO('B', 24), 1, 'D'),
    pad(2, GPIO('C', 24), 2, 'D'),
    pad(2, GPIO('C', 25), 3, 'D'),
    pad(3, GPIO('A', 22), 0, 'C'),
    pad(3, GPIO('A', 23), 1, 'C'),
    pad(3, GPIO('A', 24), 2, 'C'),
    pad(3, GPIO('A', 25), 3, 'C'),
    pad(3, GPIO('B', 20), 0, 'C'),
    pad(3, GPIO('B', 21), 1, 'C'),
    pad(3, GPIO('A', 20), 2, 'D'),
    pad(3, GPIO('A', 21), 3, 'D'),
    pad(3, GPIO('A', 17), 0, 'D'),
    pad(3, GPIO('A', 16), 1, 'D'),
    pad(3, GPIO('A', 18), 2, 'D'),
    pad(3, GPIO('A', 19), 3, 'D'),
    pad(3, GPIO('C', 23), 0, 'D'),
    pad(3, GPIO('C', 22), 1, 'D'),
    pad(3, GPIO('D', 20), 2, 'D'),
    pad(3, GPIO('D', 21), 3, 'D'),
    pad(4, GPIO('B', 12), 0, 'C'),
    pad(4, GPIO('B', 13), 1, 'C'),
    pad(4, GPIO('B', 14), 2, 'C'),
    pad(4, GPIO('B', 15), 3, 'C'),
    pad(4, GPIO('B', 8), 0, 'D'),
    pad(4, GPIO('B', 9), 1, 'D'),
    pad(4, GPIO('B', 10), 2, 'D'),
    pad(4, GPIO('B', 11), 3, 'D'),
    pad(4, GPIO('A', 13), 0, 'D'),
    pad(4, GPIO('A', 12), 1, 'D'),
    pad(4, GPIO('A', 14), 2, 'D'),
    pad(4, GPIO('A', 15), 3, 'D'),
    pad(4, GPIO('B', 27), 0, 'D'),
    pad(4, GPIO('B', 26), 1, 'D'),
    pad(4, GPIO('B', 28), 2, 'D'),
    pad(4, GPIO('B', 29), 3, 'D'),
    pad(5, GPIO('B', 16), 0, 'C'),
    pad(5, GPIO('B', 17), 1, 'C'),
    pad(5, GPIO('B', 18), 2, 'C'),
    pad(5, GPIO('B', 19), 3, 'C'),
    pad(5, GPIO('A', 23), 0, 'D'),
    pad(5, GPIO('A', 22), 1, 'D'),
    pad(5, GPIO('A', 20), 2, 'D'),
    pad(5, GPIO('A', 21), 3, 'D'),
    pad(5, GPIO('A', 24), 2, 'D'),
    pad(5, GPIO('A', 25), 3, 'D'),
    pad(5, GPIO('B', 22), 2, 'D'),
    pad(5, GPIO('B', 23), 3, 'D'),
    pad(5, GPIO('B', 31), 0, 'D'),
    pad(5, GPIO('B', 30), 1, 'D'),
    pad(5, GPIO('B', 0), 2, 'D'),
    pad(5, GPIO('B', 1), 3, 'D'),
    pad(5, GPIO('B', 2), 0, 'D'),
    pad(5, GPIO('B', 3), 1, 'D'),
];
#[cfg(all(CONFIG_MACH_SAMX5, has_SERCOM6))]
const SERCOM_PADS_6_7: &[SercomPad] = &[
    pad(6, GPIO('C', 16), 0, 'C'),
    pad(6, GPIO('C', 17), 1, 'C'),
    pad(6, GPIO('C', 18), 2, 'C'),
    pad(6, GPIO('C', 19), 3, 'C'),
    pad(6, GPIO('C', 4), 0, 'C'),
    pad(6, GPIO('C', 5), 1, 'C'),
    pad(6, GPIO('C', 6), 2, 'C'),
    pad(6, GPIO('C', 7), 3, 'C'),
    pad(6, GPIO('D', 9), 0, 'D'),
    pad(6, GPIO('D', 8), 1, 'D'),
    pad(6, GPIO('D', 10), 2, 'D'),
    pad(6, GPIO('D', 11), 3, 'D'),
    pad(6, GPIO('C', 13), 0, 'D'),
    pad(6, GPIO('C', 12), 1, 'D'),
    pad(6, GPIO('C', 14), 2, 'D'),
    pad(6, GPIO('C', 15), 3, 'D'),
    pad(6, GPIO('C', 10), 2, 'C'),
    pad(6, GPIO('C', 11), 3, 'C'),
    pad(7, GPIO('C', 12), 0, 'C'),
    pad(7, GPIO('C', 13), 1, 'C'),
    pad(7, GPIO('C', 14), 2, 'C'),
    pad(7, GPIO('C', 15), 3, 'C'),
    pad(7, GPIO('D', 8), 0, 'C'),
    pad(7, GPIO('D', 9), 1, 'C'),
    pad(7, GPIO('D', 10), 2, 'C'),
    pad(7, GPIO('D', 11), 3, 'C'),
    pad(7, GPIO('C', 10), 2, 'D'),
    pad(7, GPIO('C', 11), 3, 'D'),
    pad(7, GPIO('B', 21), 0, 'D'),
    pad(7, GPIO('B', 20), 1, 'D'),
    pad(7, GPIO('B', 18), 2, 'D'),
    pad(7, GPIO('B', 19), 3, 'D'),
    pad(7, GPIO('B', 30), 0, 'C'),
    pad(7, GPIO('B', 31), 1, 'C'),
    pad(7, GPIO('A', 30), 2, 'C'),
    pad(7, GPIO('A', 31), 3, 'C'),
];
#[cfg(not(all(CONFIG_MACH_SAMX5, has_SERCOM6)))]
const SERCOM_PADS_6_7: &[SercomPad] = &[];
const SERCOM_PADS_PARTS: &[&[SercomPad]] = &[SERCOM_PADS_BASE, SERCOM_PADS_6_7];
static SERCOM_PADS: [SercomPad; concat_len(SERCOM_PADS_PARTS)] = concat(SERCOM_PADS_PARTS);

fn sercom_lookup_pad(sercom_id: u32, pin: u8) -> R<&'static SercomPad> {
    for sp in SERCOM_PADS.iter() {
        if sp.sercom_id as u32 == sercom_id && sp.gpio == pin {
            return Ok(sp);
        }
    }
    shutdown!("Invalid SERCOM configuration");
}

/****************************************************************
 * Runtime configuration
 ****************************************************************/

const TX_PIN: usize = 0;
const RX_PIN: usize = 1;
const CLK_PIN: usize = 2;
decl_enumeration!("sercom_pin_type", "tx", TX_PIN);
decl_enumeration!("sercom_pin_type", "rx", RX_PIN);
decl_enumeration!("sercom_pin_type", "clk", CLK_PIN);

// Runtime configuration
#[derive(Clone, Copy)]
struct SercomPin {
    pins: [u8; 3],
}

static SERCOM_PINS: Global<[SercomPin; NUM_SERCOMS]> = Global::new([SercomPin { pins: [0; 3] }; NUM_SERCOMS]);

pub fn command_set_sercom_pin(args: &Args) -> R {
    let (sercom_id, pin_type, pin) = (args[0] as u8 as usize, args[1] as u8 as usize, args[2] as u8);
    let sercom_pins = SERCOM_PINS.get();
    if sercom_id >= sercom_pins.len() || pin_type >= sercom_pins[0].pins.len() {
        shutdown!("Invalid SERCOM bus");
    }
    sercom_pins[sercom_id].pins[pin_type] = pin;
    Ok(())
}
decl_command!(command_set_sercom_pin, "set_sercom_pin bus=%u sercom_pin_type=%u pin=%u");

/****************************************************************
 * SPI dopo flag mapping
 ****************************************************************/

#[derive(Clone, Copy)]
struct SercomSpiMap {
    tx_pad: u8,
    clk_pad: u8,
    dopo: u8,
}

const fn sm(tx_pad: u8, clk_pad: u8, dopo: u8) -> SercomSpiMap {
    SercomSpiMap { tx_pad, clk_pad, dopo }
}

const SERCOM_SPI_BASE: &[SercomSpiMap] = &[sm(0, 1, 0), sm(3, 1, 2)];
#[cfg(CONFIG_MACH_SAMX2)]
const SERCOM_SPI_X2: &[SercomSpiMap] = &[sm(2, 3, 1), sm(0, 3, 3)];
#[cfg(not(CONFIG_MACH_SAMX2))]
const SERCOM_SPI_X2: &[SercomSpiMap] = &[];
const SERCOM_SPI_PARTS: &[&[SercomSpiMap]] = &[SERCOM_SPI_BASE, SERCOM_SPI_X2];
static SERCOM_SPI: [SercomSpiMap; concat_len(SERCOM_SPI_PARTS)] = concat(SERCOM_SPI_PARTS);

fn sercom_lookup_spi_dopo(tx_pad: u8, clk_pad: u8) -> R<u8> {
    for sm in SERCOM_SPI.iter() {
        if sm.tx_pad == tx_pad && sm.clk_pad == clk_pad {
            return Ok(sm.dopo);
        }
    }
    shutdown!("Invalid combination of TX pin and CLK pin");
}

/****************************************************************
 * Pin setup
 ****************************************************************/

pub fn sercom_spi_pins(sercom_id: u32) -> R<u32> {
    let sercom_pins = SERCOM_PINS.get();
    if sercom_id as usize >= sercom_pins.len() {
        shutdown!("Invalid SERCOM bus");
    }
    let pins = sercom_pins[sercom_id as usize].pins;
    let tx_pin = pins[TX_PIN];
    let tx_sp = sercom_lookup_pad(sercom_id, tx_pin)?;
    let rx_pin = pins[RX_PIN];
    let rx_sp = sercom_lookup_pad(sercom_id, rx_pin)?;
    let clk_pin = pins[CLK_PIN];
    let clk_sp = sercom_lookup_pad(sercom_id, clk_pin)?;

    let dopo = sercom_lookup_spi_dopo(tx_sp.pad, clk_sp.pad)?;
    if rx_sp.pad == tx_sp.pad || rx_sp.pad == clk_sp.pad {
        shutdown!("Sercom RX pad collides with TX or CLK pad");
    }

    gpio_peripheral(tx_pin as u32, tx_sp.ptype as char, 0);
    gpio_peripheral(rx_pin as u32, rx_sp.ptype as char, 0);
    gpio_peripheral(clk_pin as u32, clk_sp.ptype as char, 0);
    Ok(SERCOM_SPI_CTRLA_DIPO(rx_sp.pad as u32) | SERCOM_SPI_CTRLA_DOPO(dopo as u32))
}

pub fn sercom_i2c_pins(sercom_id: u32) -> R {
    let pins = SERCOM_PINS.get()[sercom_id as usize].pins;
    let tx_pin = pins[TX_PIN];
    let tx_sp = sercom_lookup_pad(sercom_id, tx_pin)?;
    let clk_pin = pins[CLK_PIN];
    let clk_sp = sercom_lookup_pad(sercom_id, clk_pin)?;

    if tx_sp.pad != 0 || clk_sp.pad != 1 {
        shutdown!("TX pin not on PAD0 or CLK pin not on PAD1");
    }

    gpio_peripheral(tx_pin as u32, tx_sp.ptype as char, 0);
    gpio_peripheral(clk_pin as u32, clk_sp.ptype as char, 0);
    Ok(())
}
