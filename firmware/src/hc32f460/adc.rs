// ADC functions on Huada HC32F460
//
// Copyright (C) 2022  Steven Gotthardt <gotthardt@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case, non_upper_case_globals)]
use super::gpio_h::GpioAdc;
use super::internal::*;
use super::lib::hc32f460_adc::*;
use super::lib::hc32f460_pwc::PWC_Fcg3PeriphClockCmd;
use crate::board::timer_from_us;
use crate::sched::R;
use crate::{decl_constant, decl_init, shutdown};

#[allow(dead_code)]
const ADC_RESOLUTION_12BIT: u32 = 12;
#[allow(dead_code)]
const ADC_RESOLUTION_10BIT: u32 = 10;
#[allow(dead_code)]
const ADC_RESOLUTION_8BIT: u32 = 8;

#[allow(dead_code)]
const ADC1_RESOLUTION: u32 = ADC_RESOLUTION_12BIT;
#[allow(dead_code)]
const ADC1_PRECISION: u32 = 1 << ADC1_RESOLUTION;

// (ADC1_RESOLUTION == ADC_RESOLUTION_12BIT)
const AdcResolution: u32 = AdcResolution_12Bit;

/* Timeout value definitions. Found in example code */
const TIMEOUT_VAL: u32 = 30;

decl_constant!("ADC_MAX", ADC1_PRECISION - 1);

// These pins can be used for ADC
static adc_gpio: [u8; 16] = [
    GPIO('A', 0) as u8, // Chan 0
    GPIO('A', 1) as u8, // Chan 1
    GPIO('A', 2) as u8, // Chan 2
    GPIO('A', 3) as u8, // Chan 3
    GPIO('A', 4) as u8, // Chan 4
    GPIO('A', 5) as u8, // Chan 5
    GPIO('A', 6) as u8, // Chan 6
    GPIO('A', 7) as u8, // Chan 7
    GPIO('B', 0) as u8, // Chan 8
    GPIO('B', 1) as u8, // Chan 9
    GPIO('C', 0) as u8, // Chan 10  // TBed  on TriGorilla
    GPIO('C', 1) as u8, // Chan 11  // THead on TriGorilla
    GPIO('C', 2) as u8, // Chan 12
    GPIO('C', 3) as u8, // Chan 13
    GPIO('C', 4) as u8, // Chan 14  // TBed  on aquilla
    GPIO('C', 5) as u8, // Chan 15  // THead on aquilla
];

fn adc1() -> &'static M4_ADC_TypeDef {
    unsafe { &*M4_ADC1 }
}

pub fn gpio_adc_setup(gpio: u32) -> R<GpioAdc> {
    // validate pin in adc_pins table
    let mut chan = 0;
    loop {
        if chan >= adc_gpio.len() {
            shutdown!("Not a valid ADC pin");
        }
        if adc_gpio[chan] as u32 == gpio {
            break;
        }
        chan += 1;
    }

    // set as analog
    gpio_peripheral(gpio, Pin_Mode_Ana, 0);

    let mut sampleTime = [0u8; 16]; // all chans
    sampleTime[0] = TIMEOUT_VAL as u8;
    let stcAdcChan = stc_adc_ch_cfg_t {
        u32Channel: 1 << chan,
        u8Sequence: ADC_SEQ_A as u8, // all conversions are in SEQ A
        pu8SampTime: &sampleTime,
    };
    ADC_AddAdcChannel(adc1(), &stcAdcChan);

    Ok(GpioAdc { chan: chan as u32 })
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(_g: GpioAdc) -> u32 {
    // true if the sequence is finished
    if ADC_GetEocFlag(adc1(), ADC_SEQ_A as u8) != Reset {
        // all conversions are done - clear the flag
        ADC_ClrEocFlag(adc1(), ADC_SEQ_A as u8);
        return 0;
    } else if adc1().STR.read() & 1 != 0 {
        // running but not done yet
        return timer_from_us(TIMEOUT_VAL / 2);
    } else {
        // not running - so start
        ADC_StartConvert(adc1());
    }

    timer_from_us(TIMEOUT_VAL)
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(g: GpioAdc) -> R<u16> {
    // return the one we want...
    Ok(ADC_GetValue(adc1(), g.chan as u8))
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(_g: GpioAdc) {
    ADC_StopConvert(adc1());
}

// The clocks are already set by the loader.
// There is ADC1 and ADC2. Sequences do all channels at once.
pub fn adc_init() -> R {
    //  PCLK2 (ADC clock) is 'divide by 4', Max ADC clock is 60MHz
    let stcAdcInit = stc_adc_init_t {
        enResolution: AdcResolution, // see define above
        enDataAlign: AdcDataAlign_Right,
        enAutoClear: AdcClren_Disable,
        enScanMode: AdcMode_SAOnce,
        enRschsel: 0,
    };

    // power-on ADC
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);

    // only using ADC1
    ADC_Init(adc1(), &stcAdcInit);
    Ok(())
}
decl_init!(adc_init);
