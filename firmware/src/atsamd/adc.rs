// Analog to Digital Converter support
//
// Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::gpio_h::GpioAdc;
use super::internal::*;
use crate::ctr::{concat, concat_len, Global};
use crate::sched::R;
#[allow(unused_imports)]
use crate::{decl_constant, decl_constant_str, decl_enumeration, shutdown};

const ADC_TEMPERATURE_PIN: u32 = 0xfe;
decl_enumeration!("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

const fn pin(port: char, num: u32) -> u8 {
    GPIO(port, num) as u8
}

#[cfg(CONFIG_MACH_SAMC21)]
mod pins {
    use super::*;
    decl_constant_str!("RESERVE_PINS_adc", "PA3");
    pub const ADC_INPUTCTRL_MUXNEG_GND: u32 = ADC_INPUTCTRL_MUXNEG(0x18);
    pub const ADC_PINS_A: &[u8] = &[
        /* ADC0 */
        pin('A', 2), pin('A', 3), pin('B', 8), pin('B', 9), pin('A', 4),
        pin('A', 5), pin('A', 6), pin('A', 7), pin('A', 8), pin('A', 9),
        pin('A', 10), pin('A', 11),
        /* Padding to 16 */
        255, 255, 255, 255,
        /* ADC1 */
        pin('B', 0), pin('B', 1), pin('B', 2), pin('B', 3), pin('B', 8),
        pin('B', 9), pin('B', 4), pin('B', 5), pin('B', 6), pin('B', 7),
        pin('A', 8), pin('A', 9),
    ];
    pub const ADC_PINS_B: &[u8] = &[];
}
#[cfg(CONFIG_MACH_SAMD21)]
mod pins {
    use super::*;
    pub const ADC_PINS_A: &[u8] = &[
        pin('A', 2), pin('A', 3), pin('B', 8), pin('B', 9), pin('A', 4),
        pin('A', 5), pin('A', 6), pin('A', 7), pin('B', 0), pin('B', 1),
        pin('B', 2), pin('B', 3), pin('B', 4), pin('B', 5), pin('B', 6),
        pin('B', 7), pin('A', 8), pin('A', 9), pin('A', 10), pin('A', 11),
    ];
    pub const ADC_PINS_B: &[u8] = &[];
}
#[cfg(CONFIG_MACH_SAMX5)]
mod pins {
    use super::*;
    pub const ADC_PINS_A: &[u8] = &[
        /* ADC0 */
        pin('A', 2), pin('A', 3), pin('B', 8), pin('B', 9), pin('A', 4),
        pin('A', 5), pin('A', 6), pin('A', 7), pin('A', 8), pin('A', 9),
        pin('A', 10), pin('A', 11), pin('B', 0), pin('B', 1), pin('B', 2),
        pin('B', 3),
    ];
    #[cfg(not(CONFIG_WANT_INDX_HEATER))]
    pub const ADC_PINS_B: &[u8] = &[
        /* ADC1 */
        pin('B', 8), pin('B', 9), pin('A', 8), pin('A', 9), pin('C', 2),
        pin('C', 3), pin('B', 4), pin('B', 5), pin('B', 6), pin('B', 7),
        pin('C', 0), pin('C', 1), pin('C', 30), pin('C', 31), pin('D', 0),
        pin('D', 1),
    ];
    #[cfg(CONFIG_WANT_INDX_HEATER)]
    pub const ADC_PINS_B: &[u8] = &[];
}
use pins::*;
const ADC_PINS_PARTS: &[&[u8]] = &[ADC_PINS_A, ADC_PINS_B];
static ADC_PINS: [u8; concat_len(ADC_PINS_PARTS)] = concat(ADC_PINS_PARTS);

// SAMD51_ADC_SYNC(ADC, BIT)
#[allow(unused_variables)]
fn adc_sync(reg: &Adc, bit: u32) {
    #[cfg(not(CONFIG_MACH_SAMD21))]
    while reg.SYNCBUSY.reg.read() as u32 & bit != 0 {}
}

decl_constant!("ADC_MAX", 4095);

fn gpio_adc_pin_to_struct(pin: u32) -> R<GpioAdc> {
    // Find pin in adc_pins table
    let mut chan: u8 = 0;
    loop {
        if chan as usize >= ADC_PINS.len() {
            shutdown!("Not a valid ADC pin");
        }
        if ADC_PINS[chan as usize] as u32 == pin {
            break;
        }
        chan += 1;
    }
    #[cfg(CONFIG_MACH_SAMD21)]
    let reg = ADC;
    #[cfg(not(CONFIG_MACH_SAMD21))]
    let reg = if chan < 16 { ADC0 } else { ADC1 };
    #[cfg(not(CONFIG_MACH_SAMD21))]
    {
        chan %= 16;
    }
    Ok(GpioAdc { regs: reg, chan: chan as u32 })
}

static HAVE_RUN_INIT: Global<u8> = Global::new(0);

fn adc_init() {
    let have_run_init = HAVE_RUN_INIT.get();
    if *have_run_init != 0 {
        return;
    }
    *have_run_init = 1;

    #[cfg(CONFIG_MACH_SAMC21)]
    {
        let (adc0, adc1) = unsafe { (&*ADC0, &*ADC1) };
        // Enable adc clock
        enable_pclock(ADC0_GCLK_ID, ID_ADC0 as i32);
        enable_pclock(ADC1_GCLK_ID, ID_ADC1 as i32);

        // Set ADC-DAC VREFA pin to ADC mode
        gpio_peripheral(GPIO('A', 3), 'B', 0);

        // Reset
        adc0.CTRLA.reg.write(ADC_CTRLA_SWRST as _);
        while adc0.CTRLA.reg.read() as u32 & ADC_CTRLA_SWRST != 0 {}
        adc1.CTRLA.reg.write(ADC_CTRLA_SWRST as _);
        while adc1.CTRLA.reg.read() as u32 & ADC_CTRLA_SWRST != 0 {}

        // Load calibration info
        // ADC0
        let mut refbuf = GET_FUSE(ADC0_FUSES_BIASREFBUF_ADDR, ADC0_FUSES_BIASREFBUF_Msk, ADC0_FUSES_BIASREFBUF_Pos);
        let mut comp = GET_FUSE(ADC0_FUSES_BIASCOMP_ADDR, ADC0_FUSES_BIASCOMP_Msk, ADC0_FUSES_BIASCOMP_Pos);
        adc0.CALIB.reg.write((ADC0_FUSES_BIASREFBUF(refbuf) | ADC0_FUSES_BIASCOMP(comp)) as _);

        // ADC1
        refbuf = GET_FUSE(ADC1_FUSES_BIASREFBUF_ADDR, ADC1_FUSES_BIASREFBUF_Msk, ADC1_FUSES_BIASREFBUF_Pos);
        comp = GET_FUSE(ADC1_FUSES_BIASCOMP_ADDR, ADC1_FUSES_BIASCOMP_Msk, ADC1_FUSES_BIASCOMP_Pos);
        adc1.CALIB.reg.write((ADC0_FUSES_BIASREFBUF(refbuf) | ADC0_FUSES_BIASCOMP(comp)) as _);

        // Setup and enable
        for adc in [adc0, adc1] {
            adc.REFCTRL.reg.write((ADC_REFCTRL_REFSEL_AREFA | ADC_REFCTRL_REFCOMP) as _);
            adc.CTRLB.reg.write(ADC_CTRLB_PRESCALER_DIV128 as _);
            adc.SAMPCTRL.reg.write(ADC_SAMPCTRL_SAMPLEN(63) as _);
            while adc.SYNCBUSY.reg.read() as u32 & ADC_SYNCBUSY_SAMPCTRL != 0 {}
            adc.CTRLA.reg.write(ADC_CTRLA_ENABLE as _);
            while adc.SYNCBUSY.reg.read() as u32 & ADC_SYNCBUSY_ENABLE != 0 {}
        }
    }
    #[cfg(CONFIG_MACH_SAMD21)]
    {
        let adc = unsafe { &*ADC };
        // Enable adc clock
        enable_pclock(ADC_GCLK_ID, ID_ADC as i32);
        // Load calibraiton info
        let bias = GET_FUSE(ADC_FUSES_BIASCAL_ADDR, ADC_FUSES_BIASCAL_Msk, ADC_FUSES_BIASCAL_Pos);
        let li0 = GET_FUSE(ADC_FUSES_LINEARITY_0_ADDR, ADC_FUSES_LINEARITY_0_Msk, ADC_FUSES_LINEARITY_0_Pos);
        let li5 = GET_FUSE(ADC_FUSES_LINEARITY_1_ADDR, ADC_FUSES_LINEARITY_1_Msk, ADC_FUSES_LINEARITY_1_Pos);
        let lin = li0 | (li5 << 5);
        adc.CALIB.reg.write((ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(lin)) as _);

        // Setup and enable adc
        adc.REFCTRL.reg.write(ADC_REFCTRL_REFSEL_INTVCC1 as _);
        adc.CTRLB.reg.write(ADC_CTRLB_PRESCALER_DIV128 as _);
        adc.SAMPCTRL.reg.write(63);
        adc.CTRLA.reg.write(ADC_CTRLA_ENABLE as _);
    }
    #[cfg(CONFIG_MACH_SAMX5)]
    {
        let (adc0, adc1) = unsafe { (&*ADC0, &*ADC1) };
        // Enable adc clock
        enable_pclock(ADC0_GCLK_ID, ID_ADC0 as i32);
        enable_pclock(ADC1_GCLK_ID, ID_ADC1 as i32);

        // Load calibration info
        // ADC0
        let mut refbuf = GET_FUSE(ADC0_FUSES_BIASREFBUF_ADDR, ADC0_FUSES_BIASREFBUF_Msk, ADC0_FUSES_BIASREFBUF_Pos);
        let mut r2r = GET_FUSE(ADC0_FUSES_BIASR2R_ADDR, ADC0_FUSES_BIASR2R_Msk, ADC0_FUSES_BIASR2R_Pos);
        let mut comp = GET_FUSE(ADC0_FUSES_BIASCOMP_ADDR, ADC0_FUSES_BIASCOMP_Msk, ADC0_FUSES_BIASCOMP_Pos);
        adc0.CALIB.reg.write((ADC0_FUSES_BIASREFBUF(refbuf) | ADC0_FUSES_BIASR2R(r2r) | ADC0_FUSES_BIASCOMP(comp)) as _);

        // ADC1
        refbuf = GET_FUSE(ADC1_FUSES_BIASREFBUF_ADDR, ADC1_FUSES_BIASREFBUF_Msk, ADC1_FUSES_BIASREFBUF_Pos);
        r2r = GET_FUSE(ADC1_FUSES_BIASR2R_ADDR, ADC1_FUSES_BIASR2R_Msk, ADC1_FUSES_BIASR2R_Pos);
        comp = GET_FUSE(ADC1_FUSES_BIASCOMP_ADDR, ADC1_FUSES_BIASCOMP_Msk, ADC1_FUSES_BIASCOMP_Pos);
        adc1.CALIB.reg.write((ADC0_FUSES_BIASREFBUF(refbuf) | ADC0_FUSES_BIASR2R(r2r) | ADC0_FUSES_BIASCOMP(comp)) as _);

        // Setup and enable
        for adc in [adc0, adc1] {
            adc.REFCTRL.reg.write(ADC_REFCTRL_REFSEL_INTVCC1 as _);
            while adc.SYNCBUSY.reg.read() as u32 & ADC_SYNCBUSY_REFCTRL != 0 {}
            adc.SAMPCTRL.reg.write(ADC_SAMPCTRL_SAMPLEN(63) as _);
            while adc.SYNCBUSY.reg.read() as u32 & ADC_SYNCBUSY_SAMPCTRL != 0 {}
            adc.CTRLA.reg.write((ADC_CTRLA_PRESCALER(ADC_CTRLA_PRESCALER_DIV32_Val) | ADC_CTRLA_ENABLE) as _);
        }
    }
}

pub fn gpio_adc_setup(pin: u32) -> R<GpioAdc> {
    // Enable ADC
    adc_init();

    if pin == ADC_TEMPERATURE_PIN {
        #[cfg(CONFIG_MACH_SAMD21)]
        {
            unsafe { &*SYSCTRL }.VREF.reg.or(SYSCTRL_VREF_TSEN);
            return Ok(GpioAdc { regs: ADC, chan: ADC_INPUTCTRL_MUXPOS_TEMP_Val });
        }
        #[cfg(CONFIG_MACH_SAMX5)]
        {
            unsafe { &*SUPC }.VREF.reg.or(SUPC_VREF_TSEN);
            return Ok(GpioAdc { regs: ADC0, chan: ADC_INPUTCTRL_MUXPOS_PTAT_Val });
        }
    }

    // Set pin in ADC mode
    gpio_peripheral(pin, 'B', 0);

    gpio_adc_pin_to_struct(pin)
}

const ADC_DUMMY: u8 = 0xff;
static LAST_ANALOG_READ: Global<u8> = Global::new(ADC_DUMMY);

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(g: GpioAdc) -> u32 {
    let reg = unsafe { &*g.regs };
    let last = LAST_ANALOG_READ.get();
    'need_delay: {
        if *last as u32 == g.chan {
            if reg.INTFLAG.reg.read() as u32 & ADC_INTFLAG_RESRDY != 0 {
                // Sample now ready
                return 0;
            }
            // ADC is still busy
            break 'need_delay;
        }
        if *last != ADC_DUMMY {
            // Sample on another channel in progress
            break 'need_delay;
        }
        *last = g.chan as u8;

        // Set the channel to sample
        #[cfg(CONFIG_MACH_SAMD21)]
        let gain = ADC_INPUTCTRL_GAIN_DIV2;
        #[cfg(not(CONFIG_MACH_SAMD21))]
        let gain = 0;
        reg.INPUTCTRL.reg.write((ADC_INPUTCTRL_MUXPOS(g.chan) | ADC_INPUTCTRL_MUXNEG_GND | gain) as _);

        adc_sync(reg, adc_syncbusy::INPUTCTRL);
        // Start the sample
        reg.SWTRIG.reg.write(ADC_SWTRIG_START as _);
        adc_sync(reg, adc_syncbusy::SWTRIG);
    }

    // Schedule next attempt after sample is likely to be complete
    42 * 128 + 200 // 42 == 1 + (63+1)/2 + 1 + 12/2 + 1.5
}

// (the ADC_SYNCBUSY_ bits of SAMD51_ADC_SYNC; the samd21 has none)
mod adc_syncbusy {
    #[cfg(not(CONFIG_MACH_SAMD21))]
    pub use crate::board::chip::{ADC_SYNCBUSY_INPUTCTRL as INPUTCTRL, ADC_SYNCBUSY_SWTRIG as SWTRIG};
    #[cfg(CONFIG_MACH_SAMD21)]
    pub const INPUTCTRL: u32 = 0;
    #[cfg(CONFIG_MACH_SAMD21)]
    pub const SWTRIG: u32 = 0;
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(g: GpioAdc) -> R<u16> {
    *LAST_ANALOG_READ.get() = ADC_DUMMY;
    Ok(unsafe { &*g.regs }.RESULT.reg.read() as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let reg = unsafe { &*g.regs };
    let last = LAST_ANALOG_READ.get();
    if *last as u32 == g.chan {
        reg.SWTRIG.reg.write(ADC_SWTRIG_FLUSH as _);
        adc_sync(reg, adc_syncbusy::SWTRIG);
        reg.INTFLAG.reg.write(ADC_INTFLAG_RESRDY as _);
        *last = ADC_DUMMY;
    }
}
