// Analog to digital support
//
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioAdc;
use super::internal::*;
use crate::board::{gpio_in_setup, irq_restore, irq_save};
use crate::config::CONFIG_CLOCK_FREQ;
use crate::sched::R;
use crate::{decl_constant, decl_enumeration, shutdown};

const ADC_TEMPERATURE_PIN: u32 = 0xfe;
decl_enumeration!("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

const T: u8 = ADC_TEMPERATURE_PIN as u8;
const fn pin(port: char, num: u32) -> u8 {
    GPIO(port, num) as u8
}
#[cfg(CONFIG_MACH_SAM3X)]
static ADC_PINS: [u8; 16] = [
    pin('A', 2), pin('A', 3), pin('A', 4), pin('A', 6),
    pin('A', 22), pin('A', 23), pin('A', 24), pin('A', 16),
    pin('B', 12), pin('B', 13), pin('B', 17), pin('B', 18),
    pin('B', 19), pin('B', 20), pin('B', 21), T,
];
#[cfg(CONFIG_MACH_SAM4S)]
static ADC_PINS: [u8; 16] = [
    pin('A', 17), pin('A', 18), pin('A', 19), pin('A', 20),
    pin('B', 0), pin('B', 1), pin('B', 2), pin('B', 3),
    pin('A', 21), pin('A', 22), pin('C', 13), pin('C', 15),
    pin('C', 12), pin('C', 29), pin('C', 30), T,
];

const ADC_FREQ_MAX: u32 = 20000000;
decl_constant!("ADC_MAX", 4095);

fn adc() -> &'static Adc {
    unsafe { &*ADC }
}

pub fn gpio_adc_setup(pin: u32) -> R<GpioAdc> {
    // Find pin in adc_pins table
    let mut chan = 0;
    loop {
        if chan >= ADC_PINS.len() {
            shutdown!("Not a valid ADC pin");
        }
        if ADC_PINS[chan] as u32 == pin {
            break;
        }
        chan += 1;
    }

    if !is_enabled_pclock(ID_ADC) {
        // Setup ADC
        enable_pclock(ID_ADC);
        let prescal = get_pclock_frequency(ID_ADC) / (2 * ADC_FREQ_MAX) - 1;
        adc().ADC_MR.write(ADC_MR_PRESCAL(prescal) | ADC_MR_STARTUP_SUT768 | ADC_MR_TRANSFER(1));
    }

    if pin == ADC_TEMPERATURE_PIN {
        // Enable temperature sensor
        adc().ADC_ACR.or(ADC_ACR_TSON);
    } else {
        // Place pin in input floating mode
        gpio_in_setup(pin, 0)?;
    }
    Ok(GpioAdc { chan: 1 << chan })
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(g: GpioAdc) -> u32 {
    let chsr = adc().ADC_CHSR.read() & 0xffff;
    'need_delay: {
        if chsr == 0 {
            // Start sample
            adc().ADC_CHER.write(g.chan);
            adc().ADC_CR.write(ADC_CR_START);
            break 'need_delay;
        }
        if chsr != g.chan {
            // Sampling in progress on another channel
            break 'need_delay;
        }
        if adc().ADC_ISR.read() & ADC_ISR_DRDY == 0 {
            // Conversion still in progress
            break 'need_delay;
        }
        // Conversion ready
        return 0;
    }
    (ADC_FREQ_MAX as u64 * 1000 / CONFIG_CLOCK_FREQ as u64) as u32
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(g: GpioAdc) -> R<u16> {
    adc().ADC_CHDR.write(g.chan);
    Ok(adc().ADC_LCDR.read() as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let flag = irq_save();
    if (adc().ADC_CHSR.read() & 0xffff) == g.chan {
        let _ = gpio_adc_read(g);
    }
    irq_restore(flag);
}
