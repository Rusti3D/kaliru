// Analog to Digital Converter support
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioAdc;
use super::internal::*;
use super::pgm::readp;
use crate::ctr::Global;
use crate::sched::R;
use crate::{decl_constant, shutdown};
#[cfg(any(CONFIG_MACH_atmega168, CONFIG_MACH_atmega328))]
use crate::decl_enumeration_range;

#[cfg(any(CONFIG_MACH_atmega168, CONFIG_MACH_atmega328, CONFIG_MACH_atmega328p))]
const PINS: [u8; 8] = [
    GPIO('C', 0), GPIO('C', 1), GPIO('C', 2), GPIO('C', 3),
    GPIO('C', 4), GPIO('C', 5), GPIO('E', 2), GPIO('E', 3),
];
#[cfg(any(CONFIG_MACH_atmega644p, CONFIG_MACH_atmega1284p))]
const PINS: [u8; 8] = [
    GPIO('A', 0), GPIO('A', 1), GPIO('A', 2), GPIO('A', 3),
    GPIO('A', 4), GPIO('A', 5), GPIO('A', 6), GPIO('A', 7),
];
#[cfg(any(CONFIG_MACH_at90usb1286, CONFIG_MACH_at90usb646))]
const PINS: [u8; 8] = [
    GPIO('F', 0), GPIO('F', 1), GPIO('F', 2), GPIO('F', 3),
    GPIO('F', 4), GPIO('F', 5), GPIO('F', 6), GPIO('F', 7),
];
#[cfg(CONFIG_MACH_atmega32u4)]
const PINS: [u8; 12] = [
    GPIO('F', 0), GPIO('F', 1), GPIO('F', 2), GPIO('F', 3),
    GPIO('F', 4), GPIO('F', 5), GPIO('F', 6), GPIO('F', 7),
    GPIO('D', 4), GPIO('D', 6), GPIO('D', 7), GPIO('B', 4),
];
#[cfg(any(CONFIG_MACH_atmega1280, CONFIG_MACH_atmega2560))]
const PINS: [u8; 16] = [
    GPIO('F', 0), GPIO('F', 1), GPIO('F', 2), GPIO('F', 3),
    GPIO('F', 4), GPIO('F', 5), GPIO('F', 6), GPIO('F', 7),
    GPIO('K', 0), GPIO('K', 1), GPIO('K', 2), GPIO('K', 3),
    GPIO('K', 4), GPIO('K', 5), GPIO('K', 6), GPIO('K', 7),
];
#[link_section = ".progmem.data"]
static ADC_PINS: [u8; PINS.len()] = PINS;

// The atmega168/328 have two analog only pins
#[cfg(any(CONFIG_MACH_atmega168, CONFIG_MACH_atmega328))]
decl_enumeration_range!("pin", "PE2", GPIO('E', 2), 2);

const ADMUX_DEFAULT: u8 = 0x40;
const ADC_ENABLE: u8 = (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) | (1 << ADEN) | (1 << ADIF);

decl_constant!("ADC_MAX", 1023);

pub fn gpio_adc_setup(pin: u32) -> R<GpioAdc> {
    let pin = pin as u8;
    // Find pin in adc_pins table
    let mut chan: u8 = 0;
    loop {
        if chan as usize >= ADC_PINS.len() {
            shutdown!("Not a valid ADC pin");
        }
        if readp(&ADC_PINS[chan as usize]) == pin {
            break;
        }
        chan += 1;
    }

    unsafe {
        // Enable ADC
        (*ADCSRA).write(ADC_ENABLE);

        // Disable digital input for this pin
        #[cfg(has_DIDR2)]
        if chan >= 8 {
            (*DIDR2).write((*DIDR2).read() | 1 << (chan & 0x07));
            return Ok(GpioAdc { chan });
        }
        (*DIDR0).write((*DIDR0).read() | 1 << chan);
    }
    Ok(GpioAdc { chan })
}

const ADC_DUMMY: u8 = 0xff;
static LAST_ANALOG_READ: Global<u8> = Global::new(ADC_DUMMY);

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(g: GpioAdc) -> u32 {
    let last = LAST_ANALOG_READ.get();
    'need_delay: {
        if unsafe { (*ADCSRA).read() } & (1 << ADSC) != 0 {
            // Busy
            break 'need_delay;
        }
        if *last == g.chan {
            // Sample now ready
            return 0;
        }
        if *last != ADC_DUMMY {
            // Sample on another channel in progress
            break 'need_delay;
        }
        *last = g.chan;

        unsafe {
            // Set the channel to sample
            // (the MUX5 bit of ADCSRB selects whether we're reading from
            // channels 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high))
            #[cfg(all(has_ADCSRB, has_MUX5))]
            (*ADCSRB).write(((g.chan >> 3) & 0x01) << MUX5);
            (*ADMUX).write(ADMUX_DEFAULT | (g.chan & 0x07));

            // Start the sample
            (*ADCSRA).write(ADC_ENABLE | (1 << ADSC));
        }
    }
    // Schedule next attempt after sample is likely to be complete
    (13 + 1) * 128 + 200
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(_g: GpioAdc) -> R<u16> {
    *LAST_ANALOG_READ.get() = ADC_DUMMY;
    Ok(unsafe { (*ADC).read() })
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let last = LAST_ANALOG_READ.get();
    if *last == g.chan {
        *last = ADC_DUMMY;
    }
}
