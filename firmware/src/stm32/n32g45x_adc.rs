// ADC functions on N32G45x
//
// Copyright (C) 2022-2023  Alexey Golyshin <stas2z@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::n32g45x_adc_h::*;
use super::n32g45x_adc_lib::{ADC_ConfigRegularChannel, ADC_Init};
use crate::board::{irq_restore, irq_save, timer_from_us, GpioAdc};
use crate::ctr::{concat, concat_len};
use crate::generic::armcm_timer::udelay;
use crate::sched::R;
use crate::{decl_constant, decl_enumeration, shutdown};

const ADC_INVALID_PIN: u8 = 0xFF;

decl_constant!("ADC_MAX", 4095);

const ADC_TEMPERATURE_PIN: u32 = 0xfe;
decl_enumeration!("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

const X: u8 = ADC_INVALID_PIN;
const T: u8 = ADC_TEMPERATURE_PIN as u8;
const fn pin(port: char, num: u32) -> u8 {
    GPIO(port, num) as u8
}
const ADC_PINS_ADC12: &[u8] = &[
    // ADC1
    X, pin('A', 0), pin('A', 1), pin('A', 6),
    pin('A', 3), pin('F', 4), X, X,
    X, X, X, X,
    X, X, X, X,
    T, X, X, X,
    X, X, X, X,
    X, X, X, X,
    X, X, X, X,
    // ADC2
    X, pin('A', 4), pin('A', 5), pin('B', 1),
    pin('A', 7), pin('C', 4), pin('C', 0), pin('C', 1),
    pin('C', 2), pin('C', 3), pin('F', 2), pin('A', 2),
    pin('C', 5), pin('B', 2), X, X,
    X, X, X, X,
    X, X, X, X,
    X, X, X, X,
    X, X, X, X,
];
// ADC3/4 for G455 only
#[cfg(CONFIG_MACH_N32G455)]
const ADC_PINS_ADC34: &[u8] = &[
    // ADC3
    X, pin('B', 11), pin('E', 9), pin('E', 13),
    pin('E', 12), pin('B', 13), pin('E', 8), pin('D', 10),
    pin('D', 11), pin('D', 12), pin('D', 13), pin('D', 14),
    pin('B', 0), pin('E', 7), pin('E', 10), pin('E', 11),
    X, X, X, X,
    X, X, X, X,
    X, X, X, X,
    X, X, X, X,
    // ADC4
    X, pin('E', 14), pin('E', 15), pin('B', 12),
    pin('B', 14), pin('B', 15), X, X,
    X, X, X, X,
    pin('D', 8), pin('D', 9), X, X,
    X, X, X, X,
    X, X, X, X,
    X, X, X, X,
    X, X, X, X,
];
#[cfg(not(CONFIG_MACH_N32G455))]
const ADC_PINS_ADC34: &[u8] = &[];
const ADC_PINS_PARTS: &[&[u8]] = &[ADC_PINS_ADC12, ADC_PINS_ADC34];
static ADC_PINS: [u8; concat_len(ADC_PINS_PARTS)] = concat(ADC_PINS_PARTS);

// Perform calibration
fn adc_calibrate(adc: &ADC_Module) {
    adc.CTRL2.write(CTRL2_AD_ON_SET);
    while adc.CTRL3.read() & ADC_FLAG_RDY == 0 {}
    adc.CTRL3.and(!ADC_CTRL3_BPCAL_MSK);
    udelay(10);
    adc.CTRL2.write(CTRL2_AD_ON_SET | CTRL2_CAL_SET);
    while adc.CTRL2.read() & CTRL2_CAL_SET != 0 {}
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
    let adc_p = match chan >> 5 {
        0 => NS_ADC1,
        1 => NS_ADC2,
        2 => NS_ADC3,
        _ => NS_ADC4,
    };
    let adc = unsafe { &*adc_p };
    chan &= 0x1F;

    // Enable the ADC
    unsafe {
        let mut reg_temp = ADC_RCC_AHBPCLKEN.read_volatile();
        reg_temp |= RCC_AHB_PERIPH_ADC1 | RCC_AHB_PERIPH_ADC2 | RCC_AHB_PERIPH_ADC3 | RCC_AHB_PERIPH_ADC4;
        ADC_RCC_AHBPCLKEN.write_volatile(reg_temp);

        reg_temp = ADC_RCC_CFG2.read_volatile();
        reg_temp &= CFG2_ADCPLLPRES_RESET_MASK;
        reg_temp |= RCC_ADCPLLCLK_DIV1;
        reg_temp &= RCC_ADCPLLCLK_DISABLE;
        ADC_RCC_CFG2.write_volatile(reg_temp);

        reg_temp = ADC_RCC_CFG2.read_volatile();
        reg_temp &= CFG2_ADCHPRES_RESET_MASK;
        reg_temp |= RCC_ADCHCLK_DIV16;
        ADC_RCC_CFG2.write_volatile(reg_temp);
    }

    let ADC_InitStructure = ADC_InitType {
        WorkMode: ADC_WORKMODE_INDEPENDENT,
        MultiChEn: 0,
        ContinueConvEn: 0,
        ExtTrigSelect: ADC_EXT_TRIGCONV_NONE,
        DatAlign: ADC_DAT_ALIGN_R,
        ChsNumber: 1,
    };
    ADC_Init(adc, &ADC_InitStructure);

    adc_calibrate(adc);

    if pin == ADC_TEMPERATURE_PIN {
        unsafe { &*NS_ADC1 }.CTRL2.or(CTRL2_TSVREFE_SET);
        unsafe { VREF1P2_CTRL.write_volatile(VREF1P2_CTRL.read_volatile() | (1 << 10)) };
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
    let sr = adc.STS.read();
    'need_delay: {
        if sr & ADC_STS_STR != 0 {
            if sr & ADC_STS_ENDC == 0 || adc.RSEQ3.read() != g.chan {
                // Conversion still in progress or busy on another channel
                break 'need_delay;
            }
            // Conversion ready
            return 0;
        }
        // ADC timing: clock=4Mhz, Tconv=12.5, Tsamp=41.5, total=13.500us
        ADC_ConfigRegularChannel(adc, g.chan as u8, 1, ADC_SAMP_TIME_41CYCLES5 as u8);
        adc.CTRL2.or(CTRL2_AD_ON_SET);
        adc.CTRL2.or(CTRL2_EXT_TRIG_SWSTART_SET);
    }
    timer_from_us(20)
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(g: GpioAdc) -> R<u16> {
    let adc = unsafe { &*g.adc };
    adc.STS.and(!ADC_STS_ENDC);
    adc.STS.and(!ADC_STS_STR);
    adc.CTRL2.and(CTRL2_EXT_TRIG_SWSTART_RESET);
    Ok(adc.DAT.read() as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let adc = unsafe { &*g.adc };
    let flag = irq_save();
    if adc.STS.read() & ADC_STS_STR != 0 {
        let _ = gpio_adc_read(g);
    }
    irq_restore(flag);
}
