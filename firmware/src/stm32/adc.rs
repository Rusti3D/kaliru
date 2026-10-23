// ADC functions on STM32
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::board::{irq_restore, irq_save, timer_from_us, GpioAdc};
use crate::ctr::{concat, concat_len};
#[allow(unused_imports)]
use crate::config::{CONFIG_MACH_STM32F1, CONFIG_MACH_STM32F4};
#[allow(unused_imports)]
use crate::generic::armcm_timer::udelay;
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
    pin('B', 0), pin('B', 1), pin('C', 0), pin('C', 1),
    pin('C', 2), pin('C', 3), pin('C', 4), pin('C', 5),
];
#[cfg(CONFIG_MACH_STM32F1)]
const ADC_PINS_TEMP: &[u8] = &[T];
#[cfg(all(not(CONFIG_MACH_STM32F1), any(CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4x5)))]
const ADC_PINS_TEMP: &[u8] = &[T, 0x00, 0x00];
#[cfg(all(
    not(any(CONFIG_MACH_STM32F1, CONFIG_MACH_STM32F2, CONFIG_MACH_STM32F4x5)),
    any(CONFIG_MACH_STM32F401, CONFIG_MACH_STM32F446, CONFIG_MACH_STM32F411)
))]
const ADC_PINS_TEMP: &[u8] = &[0x00, 0x00, T];
#[cfg(not(any(
    CONFIG_MACH_STM32F1,
    CONFIG_MACH_STM32F2,
    CONFIG_MACH_STM32F4x5,
    CONFIG_MACH_STM32F401,
    CONFIG_MACH_STM32F446,
    CONFIG_MACH_STM32F411
)))]
const ADC_PINS_TEMP: &[u8] = &[];
#[cfg(any(CONFIG_MACH_STM32F4x5, CONFIG_MACH_STM32F446))]
const ADC_PINS_ADC3: &[u8] = &[
    0x00, 0x00, 0x00, 0x00,
    pin('F', 6), pin('F', 7), pin('F', 8), pin('F', 9),
    pin('F', 10), pin('F', 3), 0x00, 0x00,
    0x00, 0x00, pin('F', 4), pin('F', 5),
];
#[cfg(not(any(CONFIG_MACH_STM32F4x5, CONFIG_MACH_STM32F446)))]
const ADC_PINS_ADC3: &[u8] = &[];
const ADC_PINS_PARTS: &[&[u8]] = &[ADC_PINS_BASE, ADC_PINS_TEMP, ADC_PINS_ADC3];
static ADC_PINS: [u8; concat_len(ADC_PINS_PARTS)] = concat(ADC_PINS_PARTS);

#[cfg(CONFIG_MACH_STM32F1)]
const CR2_FLAGS: u32 = ADC_CR2_ADON | (7 << ADC_CR2_EXTSEL_Pos) | ADC_CR2_EXTTRIG | ADC_CR2_TSVREFE;
#[cfg(not(CONFIG_MACH_STM32F1))]
const CR2_FLAGS: u32 = ADC_CR2_ADON;

// ADC timing:
// stm32f103: ADC clock=4.5Mhz, Tconv=12.5, Tsamp=41.5, total=12.000us
// stm32f407: ADC clock=21Mhz, Tconv=12, Tsamp=84, total=4.571us
// stm32f446: ADC clock=22.5Mhz, Tconv=12, Tsamp=84, total=4.267us

// Perform calibration on stm32f103
fn adc_calibrate(_adc: &ADC_TypeDef) {
    #[cfg(CONFIG_MACH_STM32F1)]
    {
        _adc.CR2.write(ADC_CR2_ADON);
        udelay(10);
        _adc.CR2.write(ADC_CR2_ADON | ADC_CR2_RSTCAL);
        while _adc.CR2.read() & ADC_CR2_RSTCAL != 0 {}
        _adc.CR2.write(ADC_CR2_ADON | ADC_CR2_CAL);
        while _adc.CR2.read() & ADC_CR2_CAL != 0 {}
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

    // Determine which ADC block to use
    #[allow(unused_mut)]
    let (mut adc_p, mut adc_base) = (ADC1, ADC1_BASE);
    #[cfg(any(CONFIG_MACH_STM32F4x5, CONFIG_MACH_STM32F446))]
    if chan >= 19 {
        // On the STM32F4, some ADC channels are only available from ADC3
        adc_p = ADC3;
        adc_base += 0x800;
        chan -= 19;
    }
    let adc = unsafe { &*adc_p };

    // Enable the ADC
    if !is_enabled_pclock(adc_base) {
        enable_pclock(adc_base);
        adc_calibrate(adc);
        let aticks: u32 = 4; // 4-12us sample time (depending on stm32 chip)
        adc.SMPR1.write(
            aticks
                | (aticks << 3)
                | (aticks << 6)
                | (aticks << 9)
                | (aticks << 12)
                | (aticks << 15)
                | (aticks << 18)
                | (aticks << 21)
                | (if CONFIG_MACH_STM32F4 { aticks << 24 } else { 0 }),
        );
        adc.SMPR2.write(
            aticks
                | (aticks << 3)
                | (aticks << 6)
                | (aticks << 9)
                | (aticks << 12)
                | (aticks << 15)
                | (aticks << 18)
                | (aticks << 21)
                | (aticks << 24)
                | (aticks << 27),
        );
        adc.CR2.write(CR2_FLAGS);
    }

    if pin == ADC_TEMPERATURE_PIN {
        #[cfg(any(CONFIG_MACH_STM32F401, CONFIG_MACH_STM32F411))]
        unsafe { &*ADC1_COMMON }.CCR.write(ADC_CCR_TSVREFE);
        #[cfg(not(any(CONFIG_MACH_STM32F401, CONFIG_MACH_STM32F411, CONFIG_MACH_STM32F1)))]
        unsafe { &*ADC123_COMMON }.CCR.write(ADC_CCR_TSVREFE);
    } else {
        gpio_peripheral(pin, GPIO_ANALOG, 0);
    }

    Ok(GpioAdc { adc: adc_p, chan: chan as u32 })
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(g: GpioAdc) -> u32 {
    let adc = unsafe { &*g.adc };
    let sr = adc.SR.read();
    'need_delay: {
        if sr & ADC_SR_STRT != 0 {
            if sr & ADC_SR_EOC == 0 || adc.SQR3.read() != g.chan {
                // Conversion still in progress or busy on another channel
                break 'need_delay;
            }
            // Conversion ready
            return 0;
        }
        // Start sample
        adc.SQR3.write(g.chan);
        adc.CR2.write(ADC_CR2_SWSTART | CR2_FLAGS);
    }
    timer_from_us(20)
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(g: GpioAdc) -> R<u16> {
    let adc = unsafe { &*g.adc };
    adc.SR.write(!ADC_SR_STRT);
    Ok(adc.DR.read() as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let adc = unsafe { &*g.adc };
    let flag = irq_save();
    if adc.SR.read() & ADC_SR_STRT != 0 && adc.SQR3.read() == g.chan {
        let _ = gpio_adc_read(g);
    }
    irq_restore(flag);
}
