// ADC functions on STM32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::{irq_restore, irq_save, timer_from_us, GpioAdc};
#[allow(unused_imports)]
use crate::board::{timer_is_before, timer_read_time};
use crate::config::{CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0};
use crate::ctr::{concat, concat_len};
use crate::sched::R;
use crate::{decl_constant, decl_enumeration, shutdown};

decl_constant!("ADC_MAX", 4095);

const ADC_TEMPERATURE_PIN: u32 = 0xfe;
decl_enumeration!("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

const T: u8 = ADC_TEMPERATURE_PIN as u8;
const fn pin(port: char, num: u32) -> u8 {
    GPIO(port, num) as u8
}
const ADC_PINS_BASE: &[u8] = &[
    pin('A', 0), pin('A', 1), pin('A', 2), pin('A', 3),
    pin('A', 4), pin('A', 5), pin('A', 6), pin('A', 7),
    pin('B', 0), pin('B', 1),
];
#[cfg(CONFIG_MACH_STM32F0)]
const ADC_PINS_MORE: &[u8] = &[
    pin('C', 0), pin('C', 1),
    pin('C', 2), pin('C', 3), pin('C', 4), pin('C', 5),
    T,
];
#[cfg(all(not(CONFIG_MACH_STM32F0), CONFIG_MACH_STM32G0))]
const ADC_PINS_MORE: &[u8] = &[
    pin('B', 2), pin('B', 10),
    T, 0x00, 0x00,
    pin('B', 11), pin('B', 12), pin('C', 4), pin('C', 5),
];
#[cfg(not(any(CONFIG_MACH_STM32F0, CONFIG_MACH_STM32G0)))]
const ADC_PINS_MORE: &[u8] = &[];
const ADC_PINS_PARTS: &[&[u8]] = &[ADC_PINS_BASE, ADC_PINS_MORE];
static ADC_PINS: [u8; concat_len(ADC_PINS_PARTS)] = concat(ADC_PINS_PARTS);

#[cfg(CONFIG_MACH_STM32F0)]
const CR_FLAGS: u32 = 0;
#[cfg(CONFIG_MACH_STM32G0)]
const CR_FLAGS: u32 = ADC_CR_ADVREGEN;

fn adc1() -> &'static ADC_TypeDef {
    unsafe { &*ADC1 }
}

// Setup and calibrate ADC on stm32f0 chips
fn stm32f0_adc_setup() {
    #[cfg(CONFIG_MACH_STM32F0)]
    {
        // 100: 41.5 ADC clock cycles
        adc1().SMPR.write(4 << ADC_SMPR_SMP_Pos);
    }
}

// Setup and calibrate ADC on stm32g0 chips
fn stm32g0_adc_setup() {
    #[cfg(CONFIG_MACH_STM32G0)]
    {
        let adc = adc1();
        // 101: 39.5 ADC clock cycles
        adc.SMPR.write(5 << ADC_SMPR_SMP1_Pos);
        adc.CFGR2.write(2 << ADC_CFGR2_CKMODE_Pos); // 16Mhz

        // Enable voltage regulator
        adc.CR.write(CR_FLAGS);
        let end = timer_read_time().wrapping_add(timer_from_us(20));
        while timer_is_before(timer_read_time(), end) {}
    }
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

    // Enable the ADC
    if !is_enabled_pclock(ADC1_BASE) {
        enable_pclock(ADC1_BASE);
        if CONFIG_MACH_STM32F0 {
            stm32f0_adc_setup();
        } else if CONFIG_MACH_STM32G0 {
            stm32g0_adc_setup();
        }

        // Start calibration and wait for completion
        let adc = adc1();
        adc.CR.write(CR_FLAGS | ADC_CR_ADCAL);
        while adc.CR.read() & ADC_CR_ADCAL != 0 {}
        // Enable ADC
        adc.ISR.write(ADC_ISR_ADRDY);
        adc.ISR.read(); // Dummy read to make sure write is flushed
        adc.CR.write(CR_FLAGS | ADC_CR_ADEN);
        while adc.ISR.read() & ADC_ISR_ADRDY == 0 {}
    }

    if pin == ADC_TEMPERATURE_PIN {
        unsafe { &*ADC1_COMMON }.CCR.write(ADC_CCR_TSEN);
    } else {
        gpio_peripheral(pin, GPIO_ANALOG, 0);
    }

    Ok(GpioAdc { adc: core::ptr::null_mut(), chan: 1 << chan })
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(g: GpioAdc) -> u32 {
    let adc = adc1();
    'need_delay: {
        if adc.CR.read() & ADC_CR_ADSTART != 0 {
            break 'need_delay;
        }
        if adc.ISR.read() & ADC_ISR_EOC != 0 {
            if adc.CHSELR.read() == g.chan {
                return 0;
            }
            break 'need_delay;
        }
        #[cfg(CONFIG_MACH_STM32G0)]
        if adc.CHSELR.read() != g.chan {
            adc.ISR.write(ADC_ISR_CCRDY);
            adc.CHSELR.write(g.chan);
            while adc.ISR.read() & ADC_ISR_CCRDY == 0 {}
        }
        #[cfg(not(CONFIG_MACH_STM32G0))]
        adc.CHSELR.write(g.chan);
        adc.CR.write(CR_FLAGS | ADC_CR_ADSTART);
    }
    timer_from_us(10)
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(_g: GpioAdc) -> R<u16> {
    Ok(adc1().DR.read() as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let adc = adc1();
    let flag = irq_save();
    if adc.CHSELR.read() == g.chan {
        if adc.CR.read() & ADC_CR_ADSTART != 0 {
            adc.CR.write(CR_FLAGS | ADC_CR_ADSTP);
        }
        if adc.ISR.read() & ADC_ISR_EOC != 0 {
            let _ = gpio_adc_read(g);
        }
    }
    irq_restore(flag);
}
