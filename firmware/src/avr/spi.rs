// Serial Peripheral Interface (SPI) support
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio::{gpio_in_setup, gpio_out_setup};
use super::gpio_h::SpiConfig;
use super::internal::*;
use crate::config::CONFIG_CLOCK_FREQ;
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

decl_enumeration!("spi_bus", "spi", 0);

#[cfg(any(CONFIG_MACH_atmega168, CONFIG_MACH_atmega328, CONFIG_MACH_atmega328p))]
mod pins {
    use super::*;
    pub const MISO: u8 = GPIO('B', 4);
    pub const MOSI: u8 = GPIO('B', 3);
    pub const SCK: u8 = GPIO('B', 5);
    pub const SS: u8 = GPIO('B', 2);
    decl_constant_str!("BUS_PINS_spi", "PB4,PB3,PB5");
}
#[cfg(any(CONFIG_MACH_atmega644p, CONFIG_MACH_atmega1284p))]
mod pins {
    use super::*;
    pub const MISO: u8 = GPIO('B', 6);
    pub const MOSI: u8 = GPIO('B', 5);
    pub const SCK: u8 = GPIO('B', 7);
    pub const SS: u8 = GPIO('B', 4);
    decl_constant_str!("BUS_PINS_spi", "PB6,PB5,PB7");
}
#[cfg(any(
    CONFIG_MACH_at90usb1286,
    CONFIG_MACH_at90usb646,
    CONFIG_MACH_atmega32u4,
    CONFIG_MACH_atmega1280,
    CONFIG_MACH_atmega2560
))]
mod pins {
    use super::*;
    pub const MISO: u8 = GPIO('B', 3);
    pub const MOSI: u8 = GPIO('B', 2);
    pub const SCK: u8 = GPIO('B', 1);
    pub const SS: u8 = GPIO('B', 0);
    decl_constant_str!("BUS_PINS_spi", "PB3,PB2,PB1");
}
use pins::*;

fn spi_init() -> R {
    if unsafe { (*GPIO2REGS(SS)).mode.read() } & GPIO2BIT(SS) == 0 {
        // The SS pin must be an output pin (but is otherwise unused)
        gpio_out_setup(SS as u32, 0)?;
    }
    gpio_out_setup(SCK as u32, 0)?;
    gpio_out_setup(MOSI as u32, 0)?;
    gpio_in_setup(MISO as u32, 0)?;

    unsafe {
        (*SPCR).write((1 << MSTR) | (1 << SPE));
        (*SPSR).write(0);
    }
    Ok(())
}

pub fn spi_setup(bus: u32, mode: u8, rate: u32) -> R<SpiConfig> {
    if bus != 0 {
        shutdown!("Invalid spi_setup parameters");
    }

    // Make sure the SPI interface is enabled
    spi_init()?;

    // Setup rate
    let mut config = SpiConfig { spcr: 0, spsr: 0 };
    if rate >= CONFIG_CLOCK_FREQ / 2 {
        config.spsr = 1 << SPI2X;
    } else if rate >= CONFIG_CLOCK_FREQ / 4 {
        config.spcr = 0;
    } else if rate >= CONFIG_CLOCK_FREQ / 8 {
        config.spcr = 1;
        config.spsr = 1 << SPI2X;
    } else if rate >= CONFIG_CLOCK_FREQ / 16 {
        config.spcr = 1;
    } else if rate >= CONFIG_CLOCK_FREQ / 32 {
        config.spcr = 2;
        config.spsr = 1 << SPI2X;
    } else if rate >= CONFIG_CLOCK_FREQ / 64 {
        config.spcr = 2;
    } else {
        config.spcr = 3;
    }

    // Setup mode
    config.spcr |= (1 << SPE) | (1 << MSTR) | (mode << CPHA);

    Ok(config)
}

pub fn spi_prepare(config: SpiConfig) {
    unsafe {
        (*SPCR).write(config.spcr);
        (*SPSR).write(config.spsr);
    }
}

pub fn spi_transfer(_config: SpiConfig, receive_data: u8, data: &mut [u8]) -> R {
    unsafe {
        if receive_data != 0 {
            for d in data.iter_mut() {
                (*SPDR).write(*d);
                while (*SPSR).read() & (1 << SPIF) == 0 {}
                *d = (*SPDR).read();
            }
        } else {
            for &d in data.iter() {
                (*SPDR).write(d);
                while (*SPSR).read() & (1 << SPIF) == 0 {}
            }
        }
    }
    Ok(())
}
