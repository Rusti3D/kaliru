// Analog to digital support on lpc176x
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioAdc;
use super::internal::*;
use crate::board::{irq_restore, irq_save, timer_from_us};
use crate::ctr::Global;
use crate::sched::R;
use crate::{armcm_enable_irq, decl_constant, shutdown};

static ADC_PINS: [u8; 8] = [
    GPIO(0, 23) as u8, GPIO(0, 24) as u8, GPIO(0, 25) as u8, GPIO(0, 26) as u8,
    GPIO(1, 30) as u8, GPIO(1, 31) as u8, GPIO(0, 3) as u8, GPIO(0, 2) as u8,
];

static ADC_PIN_FUNCS: [u8; 8] = [1, 1, 1, 1, 3, 3, 2, 2];

const ADC_FREQ_MAX: u32 = 13000000;
decl_constant!("ADC_MAX", 4095);

// The lpc176x adc is extremely noisy. Implement a 5 entry median
// filter to weed out obviously incorrect readings.
struct AdcStatus {
    adcr: u32,
    pos: u16,
    chan: u16,
    samples: [u16; 5],
}
static ADC_STATUS: Global<AdcStatus> = Global::new(AdcStatus { adcr: 0, pos: 0, chan: 0, samples: [0; 5] });

const ADC_DONE: u16 = 0x0100;

fn adc() -> &'static LPC_ADC_TypeDef {
    unsafe { &*LPC_ADC }
}

// (&LPC_ADC->ADDR0)[chan]
fn addr(chan: u32) -> u32 {
    unsafe { adc().ADDR0.ptr().add(chan as usize).read_volatile() }
}

// ADC hardware irq handler
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ADC_IRQHandler() {
    let st = ADC_STATUS.get();
    let (mut pos, chan) = (st.pos as usize, (st.chan & 0xff) as u32);
    let result = addr(chan);
    if pos >= st.samples.len() {
        // All samples complete
        return;
    }
    if pos >= st.samples.len() - 2 {
        // Turn off burst mode
        adc().ADCR.write(st.adcr | (1 << chan));
    }
    st.samples[pos] = ((result >> 4) & 0x0fff) as u16;
    pos += 1;
    st.pos = pos as u16;
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

    if !is_enabled_pclock(PCLK_ADC) {
        // Power up ADC
        enable_pclock(PCLK_ADC);
        let pclk = get_pclock_frequency(PCLK_ADC);
        let prescal = pclk.div_ceil(ADC_FREQ_MAX) - 1;
        let st = ADC_STATUS.get();
        st.adcr = (1 << 21) | ((prescal & 0xff) << 8);
        adc().ADCR.write(st.adcr);
        adc().ADINTEN.write(0xff);
        st.chan = ADC_DONE;
        armcm_enable_irq!(ADC_IRQHandler, ADC_IRQn, 0);
    }

    gpio_peripheral(pin, ADC_PIN_FUNCS[chan] as u32, 0);

    Ok(GpioAdc { chan: chan as u32 })
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(g: GpioAdc) -> u32 {
    let st = ADC_STATUS.get();
    let chan = st.chan as u32;
    'need_delay: {
        if chan == g.chan {
            // Sample already underway - check if it is ready
            if st.pos as usize >= st.samples.len() {
                // Sample ready
                return 0;
            }
            break 'need_delay;
        }
        if chan & ADC_DONE as u32 == 0 {
            // ADC busy on some other channel
            break 'need_delay;
        }

        // Start new sample
        st.pos = 0;
        st.chan = g.chan as u16;
        adc().ADCR.write(st.adcr | (1 << g.chan) | (1 << 16));
    }
    let pclk = get_pclock_frequency(PCLK_ADC);
    (64 * pclk.div_ceil(ADC_FREQ_MAX) * st.samples.len() as u32) / 4 + timer_from_us(10)
}

fn order(r1: &mut u32, r2: &mut u32) {
    if *r1 > *r2 {
        core::mem::swap(r1, r2);
    }
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(_g: GpioAdc) -> R<u16> {
    let st = ADC_STATUS.get();
    st.chan |= ADC_DONE;
    // The lpc176x adc has a defect that causes random reports near
    // 0xfff. Work around that with a 5 sample median filter.
    let p = &st.samples;
    let (mut v0, mut v4, mut v1, mut v3, mut v2) = (p[0] as u32, p[1] as u32, p[2] as u32, p[3] as u32, p[4] as u32);
    order(&mut v0, &mut v4);
    order(&mut v1, &mut v3);
    order(&mut v0, &mut v1);
    order(&mut v3, &mut v4);
    order(&mut v1, &mut v3);
    order(&mut v1, &mut v2);
    order(&mut v2, &mut v3);
    if v3 >= 0xff0 || v4 >= 0xff0 {
        order(&mut v0, &mut v1);
        if v2 >= 0xff0 {
            // At least 3 reports are clearly bogus - return the minimum sample
            return Ok(v0 as u16);
        }
        // 1 or 2 bogus reports - return the median of the minimum 3 samples
        return Ok(v1 as u16);
    }
    // Return the median of the 5 samples
    Ok(v2 as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let st = ADC_STATUS.get();
    let chan = st.chan as u32;
    if chan != g.chan {
        return;
    }
    let flag = irq_save();
    adc().ADCR.write(st.adcr);
    st.chan = (chan as u16) | ADC_DONE;
    st.pos = st.samples.len() as u16;
    addr(chan & 0xff);
    irq_restore(flag);
}
