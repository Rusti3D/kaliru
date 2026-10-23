// Analog to digital (ADC) on stm32h7 and similar chips
//
// Copyright (C) 2020 Konstantin Vogel <konstantin.vogel@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_upper_case_globals)]
use super::internal::*;
use crate::board::{irq_restore, irq_save, timer_from_us, timer_is_before, timer_read_time, GpioAdc};
#[allow(unused_imports)]
use crate::config::{CONFIG_CLOCK_FREQ, CONFIG_MACH_STM32H723};
use crate::ctr::{concat, concat_len};
use crate::sched::R;
use crate::{decl_constant, decl_enumeration, shutdown};

const ADC_INVALID_PIN: u32 = 0xFF;

const ADC_TEMPERATURE_PIN: u32 = 0xfe;
decl_enumeration!("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

decl_constant!("ADC_MAX", 4095);

#[allow(dead_code)]
const ADCIN_BANK_SIZE: usize = 20;

const INV: u8 = ADC_INVALID_PIN as u8;
const T: u8 = ADC_TEMPERATURE_PIN as u8;
const fn pin(port: char, num: u32) -> u8 {
    GPIO(port, num) as u8
}

// GPIOs like A0_C are not covered!
// This always gives the pin connected to the positive channel
// (the C table, its #if blocks as parts: tools/port_table.py)
// ADC1
#[cfg(CONFIG_MACH_STM32H7)]
const ADC_PINS_0: &[u8] = &[
    INV,
    INV,
    pin('F', 11),
    pin('A', 6),
    pin('C', 4),
    pin('B', 1),
    pin('F', 12),
    pin('A', 7),
    pin('C', 5),
    pin('B', 0),
    pin('C', 0),
    pin('C', 1),
    pin('C', 2),
    pin('C', 3),
    pin('A', 2),
    pin('A', 3),
    pin('A', 0),
    pin('A', 1),
    pin('A', 4),
    pin('A', 5),
];
#[cfg(not(CONFIG_MACH_STM32H7))]
const ADC_PINS_0: &[u8] = &[];
// ADC2
#[cfg(CONFIG_MACH_STM32H7)]
const ADC_PINS_1: &[u8] = &[
    INV,
    INV,
    pin('F', 13),
    pin('A', 6),
    pin('C', 4),
    pin('B', 1),
    pin('F', 14),
    pin('A', 7),
    pin('C', 5),
    pin('B', 0),
    pin('C', 0),
    pin('C', 1),
    pin('C', 2),
    pin('C', 3),
    pin('A', 2),
    pin('A', 3),
    INV,
    INV,
    pin('A', 4),
    pin('A', 5),
];
#[cfg(not(CONFIG_MACH_STM32H7))]
const ADC_PINS_1: &[u8] = &[];
// ADC3
#[cfg(CONFIG_MACH_STM32H7)]
const ADC_PINS_2: &[u8] = &[
    INV,
    INV,
    pin('F', 9),
    pin('F', 7),
    pin('F', 5),
    pin('F', 3),
    pin('F', 10),
    pin('F', 8),
    pin('F', 6),
    pin('F', 4),
    pin('C', 0),
    pin('C', 1),
    pin('C', 2),
    pin('H', 2),
    pin('H', 3),
    pin('H', 4),
    pin('H', 5),
];
#[cfg(not(CONFIG_MACH_STM32H7))]
const ADC_PINS_2: &[u8] = &[];
#[cfg(all(CONFIG_MACH_STM32H7, CONFIG_MACH_STM32H723))]
const ADC_PINS_3: &[u8] = &[
    T,
    INV,
];
#[cfg(not(all(CONFIG_MACH_STM32H7, CONFIG_MACH_STM32H723)))]
const ADC_PINS_3: &[u8] = &[];
#[cfg(all(CONFIG_MACH_STM32H7, not(CONFIG_MACH_STM32H723)))]
const ADC_PINS_4: &[u8] = &[
    INV,
    T,
];
#[cfg(not(all(CONFIG_MACH_STM32H7, not(CONFIG_MACH_STM32H723))))]
const ADC_PINS_4: &[u8] = &[];
#[cfg(CONFIG_MACH_STM32H7)]
const ADC_PINS_5: &[u8] = &[
    INV,
];
#[cfg(not(CONFIG_MACH_STM32H7))]
const ADC_PINS_5: &[u8] = &[];
#[cfg(all(not(CONFIG_MACH_STM32H7), CONFIG_MACH_STM32G4))]
const ADC_PINS_6: &[u8] = &[
    INV,
    pin('A', 0),
    pin('A', 1),
    pin('A', 2),
    pin('A', 3),
    pin('B', 14),
    pin('C', 0),
    pin('C', 1),
    pin('C', 2),
    pin('C', 3),
    pin('F', 0),
    pin('B', 12),
    pin('B', 1),
    INV,
    pin('B', 11),
    pin('B', 0),
    T,
    INV,
    INV,
    INV,
    INV,
    pin('A', 0),
    pin('A', 1),
    pin('A', 6),
    pin('A', 7),
    pin('C', 4),
    pin('C', 0),
    pin('C', 1),
    pin('C', 2),
    pin('C', 3),
    pin('F', 1),
    pin('C', 5),
    pin('B', 2),
    pin('A', 5),
    pin('B', 11),
    pin('B', 15),
    INV,
    pin('A', 4),
    INV,
];
#[cfg(not(all(not(CONFIG_MACH_STM32H7), CONFIG_MACH_STM32G4)))]
const ADC_PINS_6: &[u8] = &[];
#[cfg(all(not(CONFIG_MACH_STM32H7), not(CONFIG_MACH_STM32G4)))]
const ADC_PINS_7: &[u8] = &[
    INV,
    pin('C', 0),
    pin('C', 1),
    pin('C', 2),
    pin('C', 3),
    pin('A', 0),
    pin('A', 1),
    pin('A', 2),
    pin('A', 3),
    pin('A', 4),
    pin('A', 5),
    pin('A', 6),
    pin('A', 7),
    pin('C', 4),
    pin('C', 5),
    pin('B', 0),
    pin('B', 1),
    T,
    INV,
];
#[cfg(not(all(not(CONFIG_MACH_STM32H7), not(CONFIG_MACH_STM32G4))))]
const ADC_PINS_7: &[u8] = &[];
const ADC_PINS_PARTS: &[&[u8]] = &[ADC_PINS_0, ADC_PINS_1, ADC_PINS_2, ADC_PINS_3, ADC_PINS_4, ADC_PINS_5, ADC_PINS_6, ADC_PINS_7];
static ADC_PINS: [u8; concat_len(ADC_PINS_PARTS)] = concat(ADC_PINS_PARTS);

// ADC timing
const ADC_CKMODE: u32 = 0b11;
const ADC_ATICKS: u32 = 0b110;
#[allow(dead_code)]
const ADC_ATICKS_H723_ADC3: u32 = 0b111;
// 400Mhz stm32h7: clock=25Mhz, Tsamp=387.5, Tconv=394, total=15.76us
// 520Mhz stm32h723: clock=32.5Mhz, Tsamp=387.5, Tconv=394, total=12.12us
// 520Mhz stm32h723 adc3: clock=65Mhz, Tsamp=640.5, Tconv=653, total=10.05us
// 80Mhz stm32l4: clock=20Mhz, Tsamp=247.5, Tconv=260, total=13.0us
// 170Mhz stm32g4: clock=42.5Mhz, Tsamp=247.5, Tconv=260, total=6.118us

// Handle register name differences between chips
#[cfg(CONFIG_MACH_STM32G4)]
const ADC_CCR_TSEN: u32 = ADC_CCR_VSENSESEL;

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

    // Determine which ADC block to use and enable its clock
    let (adc_p, adc_common_p): (*mut ADC_TypeDef, *mut ADC_Common_TypeDef) = 'pick: {
        #[cfg(has_ADC3)]
        if chan >= 2 * ADCIN_BANK_SIZE {
            chan -= 2 * ADCIN_BANK_SIZE;
            #[cfg(CONFIG_MACH_STM32G4)]
            break 'pick (ADC3, ADC345_COMMON);
            #[cfg(not(CONFIG_MACH_STM32G4))]
            break 'pick (ADC3, ADC3_COMMON);
        }
        #[cfg(has_ADC2)]
        if chan >= ADCIN_BANK_SIZE {
            chan -= ADCIN_BANK_SIZE;
            break 'pick (ADC2, ADC12_COMMON);
        }
        (ADC1, ADC12_COMMON)
    };
    let (adc, adc_common) = unsafe { (&*adc_p, &*adc_common_p) };
    if !is_enabled_pclock(adc_common_p as u32) {
        enable_pclock(adc_common_p as u32);
    }
    adc_common.CCR.modify(ADC_CCR_CKMODE_Msk, ADC_CKMODE << ADC_CCR_CKMODE_Pos);

    // Enable the ADC
    if adc.CR.read() & ADC_CR_ADEN == 0 {
        // Switch on voltage regulator and wait for it to stabilize
        #[allow(unused_mut)]
        let mut cr = ADC_CR_ADVREGEN;
        adc.CR.write(cr);
        let end = timer_read_time().wrapping_add(timer_from_us(20));
        while timer_is_before(timer_read_time(), end) {}

        // Setup chip specific flags
        #[allow(unused_mut)]
        let mut aticks = ADC_ATICKS;
        #[cfg(CONFIG_MACH_STM32H7)]
        if CONFIG_MACH_STM32H723 && adc_p == ADC3 {
            aticks = ADC_ATICKS_H723_ADC3;
        } else {
            // Use linear calibration on stm32h7
            cr |= ADC_CR_ADCALLIN;
            // Set boost mode on stm32h7
            let boost: u32 = if CONFIG_CLOCK_FREQ > 400000000 { 0b11 } else { 0b10 };
            cr |= boost << ADC_CR_BOOST_Pos;
            // Set 12bit samples on the stm32h7
            adc.CFGR.write(ADC_CFGR_JQDIS | (0b110 << ADC_CFGR_RES_Pos));
        }

        // Perform adc calibration
        adc.CR.write(cr | ADC_CR_ADCAL);
        while adc.CR.read() & ADC_CR_ADCAL != 0 {}

        // Enable ADC
        adc.ISR.write(ADC_ISR_ADRDY);
        adc.ISR.read(); // Dummy read to make sure write is flushed
        while adc.CR.read() & ADC_CR_ADEN == 0 {
            adc.CR.or(ADC_CR_ADEN);
        }
        while adc.ISR.read() & ADC_ISR_ADRDY == 0 {}

        // Set ADC clock cycles sample time for every channel
        let av = aticks
            | (aticks << 3)
            | (aticks << 6)
            | (aticks << 9)
            | (aticks << 12)
            | (aticks << 15)
            | (aticks << 18)
            | (aticks << 21)
            | (aticks << 24)
            | (aticks << 27);
        adc.SMPR1.write(av);
        adc.SMPR2.write(av);
    }

    if pin == ADC_TEMPERATURE_PIN {
        adc_common.CCR.or(ADC_CCR_TSEN);
    } else {
        gpio_peripheral(pin, GPIO_ANALOG, 0);
    }

    // Setup preselect (connect) channel on stm32h7
    #[cfg(all(CONFIG_MACH_STM32H7, CONFIG_MACH_STM32H723))]
    adc.PCSEL_RES0.or(1 << chan);
    #[cfg(all(CONFIG_MACH_STM32H7, not(CONFIG_MACH_STM32H723)))]
    adc.PCSEL.or(1 << chan);
    Ok(GpioAdc { adc: adc_p, chan: chan as u32 })
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(g: GpioAdc) -> u32 {
    let adc = unsafe { &*g.adc };
    let cr = adc.CR.read();
    'need_delay: {
        if cr & ADC_CR_ADSTART != 0 {
            break 'need_delay;
        }
        if adc.ISR.read() & ADC_ISR_EOC != 0 {
            if adc.SQR1.read() == (g.chan << ADC_SQR1_SQ1_Pos) {
                return 0;
            }
            break 'need_delay;
        }
        // Start sample
        adc.SQR1.write(g.chan << ADC_SQR1_SQ1_Pos);
        adc.CR.write(cr | ADC_CR_ADSTART);
    }
    timer_from_us(20)
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(g: GpioAdc) -> R<u16> {
    Ok(unsafe { &*g.adc }.DR.read() as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let adc = unsafe { &*g.adc };
    let flag = irq_save();
    if adc.SQR1.read() == (g.chan << ADC_SQR1_SQ1_Pos) {
        let cr = adc.CR.read();
        if cr & ADC_CR_ADSTART != 0 {
            adc.CR.write((cr & !ADC_CR_ADSTART) | ADC_CR_ADSTP);
        }
        if adc.ISR.read() & ADC_ISR_EOC != 0 {
            let _ = gpio_adc_read(g);
        }
    }
    irq_restore(flag);
}
