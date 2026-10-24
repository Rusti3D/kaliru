// ADC functions on rp2040
//
// Copyright (C) 2021-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioAdc;
use super::internal::*;
use crate::board::timer_from_us;
use crate::ctr::Global;
use crate::sched::R;
use crate::{decl_constant, decl_enumeration, shutdown};

decl_constant!("ADC_MAX", 4095);

const ADC_TEMPERATURE_PIN: u32 = 0xfe;
decl_enumeration!("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

fn adc() -> &'static adc_hw_t {
    unsafe { &*adc_hw }
}

pub fn gpio_adc_setup(pin: u32) -> R<GpioAdc> {
    #[allow(unused_mut)]
    let (mut min_gpio, mut max_gpio, mut adc_temp_chan) = (26u32, 29u32, 4u32);
    #[cfg(CONFIG_MACH_RP2350)]
    {
        // Check for rp2350b package
        if !is_enabled_pclock(RESETS_RESET_SYSINFO_BITS) {
            enable_pclock(RESETS_RESET_SYSINFO_BITS);
        }
        if unsafe { &*sysinfo_hw }.package_sel.read() & SYSINFO_PACKAGE_SEL_BITS == 0 {
            min_gpio = 40;
            max_gpio = 47;
            adc_temp_chan = 8;
        }
    }

    if (pin < min_gpio || pin > max_gpio) && pin != ADC_TEMPERATURE_PIN {
        shutdown!("Not a valid ADC pin");
    }

    // Enable the ADC
    if !is_enabled_pclock(RESETS_RESET_ADC_BITS) {
        enable_pclock(RESETS_RESET_ADC_BITS);
        adc().cs.write(ADC_CS_EN_BITS);
    }

    let chan;
    if pin == ADC_TEMPERATURE_PIN {
        chan = adc_temp_chan as u8;
        adc().cs.or(ADC_CS_TS_EN_BITS);
    } else {
        chan = (pin - min_gpio) as u8;
        unsafe { &*padsbank0_hw }.io[pin as usize].write(PADS_BANK0_GPIO0_OD_BITS);
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
    let cs = adc().cs.read();
    'need_delay: {
        if cs & ADC_CS_READY_BITS == 0 {
            // ADC is busy
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

        // Begin sample
        *last = g.chan;
        adc().cs.write(
            (cs & ADC_CS_TS_EN_BITS) | ADC_CS_START_ONCE_BITS | ADC_CS_EN_BITS | ((g.chan as u32) << ADC_CS_AINSEL_LSB),
        );
    }
    timer_from_us(5) // Sample takes 2us but provide extra time
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(_g: GpioAdc) -> R<u16> {
    *LAST_ANALOG_READ.get() = ADC_DUMMY;
    Ok(adc().result.read() as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let last = LAST_ANALOG_READ.get();
    if *last == g.chan {
        *last = ADC_DUMMY;
    }
}
