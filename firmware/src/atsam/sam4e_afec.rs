// SAM4e8e Analog Front-End Converter (AFEC) support
//
// Copyright (C) 2018  Florian Heilmann <Florian.Heilmann@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#![allow(non_snake_case)]
use super::gpio_h::GpioAdc;
use super::internal::*;
use crate::config::CONFIG_CLOCK_FREQ;
use crate::ctr::Global;
use crate::sched::R;
use crate::{decl_constant, decl_enumeration, decl_init, shutdown};

// (same70_afec.h: the same70 header names; the registers as functions of
// the afec device)
#[cfg(CONFIG_MACH_SAM4E)]
mod afe {
    use super::*;
    pub fn ACR(a: &Afec) -> &Reg<u32> {
        &a.AFE_ACR
    }
    pub fn CDR(a: &Afec) -> &Reg<u32> {
        &a.AFE_CDR
    }
    pub fn CGR(a: &Afec) -> &Reg<u32> {
        &a.AFE_CGR
    }
    pub fn CHDR(a: &Afec) -> &Reg<u32> {
        &a.AFE_CHDR
    }
    pub fn CHER(a: &Afec) -> &Reg<u32> {
        &a.AFE_CHER
    }
    pub fn COCR(a: &Afec) -> &Reg<u32> {
        &a.AFE_COCR
    }
    pub fn CR(a: &Afec) -> &Reg<u32> {
        &a.AFE_CR
    }
    pub fn DIFFR(a: &Afec) -> &Reg<u32> {
        &a.AFE_DIFFR
    }
    pub fn EMR(a: &Afec) -> &Reg<u32> {
        &a.AFE_EMR
    }
    pub fn IDR(a: &Afec) -> &Reg<u32> {
        &a.AFE_IDR
    }
    pub fn ISR(a: &Afec) -> &Reg<u32> {
        &a.AFE_ISR
    }
    pub fn LCDR(a: &Afec) -> &Reg<u32> {
        &a.AFE_LCDR
    }
    pub fn MR(a: &Afec) -> &Reg<u32> {
        &a.AFE_MR
    }
    pub fn CSELR(a: &Afec) -> &Reg<u32> {
        &a.AFE_CSELR
    }
    pub fn CHSR(a: &Afec) -> &Reg<u32> {
        &a.AFE_CHSR
    }
    pub fn CDOR(a: &Afec) -> &Reg<u32> {
        &a.AFE_CDOR
    }
}
#[cfg(CONFIG_MACH_SAME70)]
mod afe {
    use super::*;
    pub use crate::board::chip::{
        AFEC_ACR_IBCTL as AFE_ACR_IBCTL, AFEC_COCR_AOFF_Msk as AFE_COCR_AOFF_Msk, AFEC_CR_START as AFE_CR_START,
        AFEC_CR_SWRST as AFE_CR_SWRST, AFEC_EMR_RES_NO_AVERAGE as AFE_EMR_RES_NO_AVERAGE, AFEC_EMR_STM as AFE_EMR_STM,
        AFEC_EMR_TAG as AFE_EMR_TAG, AFEC_ISR_DRDY as AFE_ISR_DRDY, AFEC_LCDR_LDATA_Msk as AFE_LCDR_LDATA_Msk,
        AFEC_MR_FREERUN_ON as AFE_MR_FREERUN_ON, AFEC_MR_PRESCAL as AFE_MR_PRESCAL, AFEC_MR_STARTUP_SUT64 as AFE_MR_STARTUP_SUT64,
        AFEC_MR_TRANSFER as AFE_MR_TRANSFER, AFEC_MR_TRGEN as AFE_MR_TRGEN, AFEC_MR_TRGEN_DIS as AFE_MR_TRGEN_DIS,
        AFEC_MR_TRGSEL_Msk as AFE_MR_TRGSEL_Msk,
    };
    pub fn ACR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_ACR
    }
    pub fn CDR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_CDR
    }
    pub fn CGR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_CGR
    }
    pub fn CHDR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_CHDR
    }
    pub fn CHER(a: &Afec) -> &Reg<u32> {
        &a.AFEC_CHER
    }
    pub fn COCR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_COCR
    }
    pub fn CR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_CR
    }
    pub fn DIFFR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_DIFFR
    }
    pub fn EMR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_EMR
    }
    pub fn IDR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_IDR
    }
    pub fn ISR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_ISR
    }
    pub fn LCDR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_LCDR
    }
    pub fn MR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_MR
    }
    pub fn CSELR(a: &Afec) -> &Reg<u32> {
        &a.AFEC_CSELR
    }
}
use afe::*;

const ADC_TEMPERATURE_PIN: u32 = 0xfe;
decl_enumeration!("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

const T: u8 = ADC_TEMPERATURE_PIN as u8;
const fn pin(port: char, num: u32) -> u8 {
    GPIO(port, num) as u8
}
//remove first channel, since it offsets the channel number: GPIO('A', 8),
#[cfg(CONFIG_MACH_SAM4E)]
static AFEC_PINS: [u8; 24] = [
    pin('A', 17), pin('A', 18), pin('A', 19),
    pin('A', 20), pin('B', 0), pin('B', 1), pin('C', 13),
    pin('C', 15), pin('C', 12), pin('C', 29), pin('C', 30),
    pin('C', 31), pin('C', 26), pin('C', 27), pin('C', 0),
    T,
    // AFEC1
    pin('B', 2), pin('B', 3), pin('A', 21), pin('A', 22),
    pin('C', 1), pin('C', 2), pin('C', 3), pin('C', 4),
];
#[cfg(CONFIG_MACH_SAME70)]
static AFEC_PINS: [u8; 24] = [
    pin('D', 30), pin('A', 21), pin('B', 3), pin('E', 5),
    pin('E', 4), pin('B', 2), pin('A', 17), pin('A', 18),
    pin('A', 19), pin('A', 20), pin('B', 0),
    T,
    // AFEC1
    pin('B', 1), pin('C', 13), pin('C', 15), pin('C', 12),
    pin('C', 29), pin('C', 30), pin('C', 31), pin('C', 26),
    pin('C', 27), pin('C', 0), pin('E', 3), pin('E', 0),
];

#[cfg(CONFIG_MACH_SAM4E)]
mod cfg {
    use super::*;
    pub const AFEC1_START: u32 = 16; // The first 16 pins are on afec0
    pub fn CFG_AFE_MR(pclk: u32) -> u32 {
        AFE_MR_ANACH_ALLOWED
            | AFE_MR_PRESCAL(pclk / (2 * ADC_FREQ_MAX) - 1)
            | AFE_MR_SETTLING_AST3
            | AFE_MR_TRACKTIM(2)
            | AFE_MR_TRANSFER(1)
            | AFE_MR_STARTUP_SUT64
    }
    pub const CFG_AFE_ACR: u32 = AFE_ACR_IBCTL(1);
    pub const CFG_AFE_IDR: u32 = 0xDF00FFFF;
    pub const CFG_AFE_COCR: u32 = 0x800 & AFE_COCR_AOFF_Msk;
}
#[cfg(CONFIG_MACH_SAME70)]
mod cfg {
    use super::*;
    pub const AFEC1_START: u32 = 12; // The first 12 pins are on afec0
    pub fn CFG_AFE_MR(pclk: u32) -> u32 {
        AFEC_MR_ONE | AFE_MR_PRESCAL(pclk / ADC_FREQ_MAX - 1) | AFE_MR_TRANSFER(2) | AFE_MR_STARTUP_SUT64
    }
    pub const CFG_AFE_ACR: u32 = AFE_ACR_IBCTL(1) | AFEC_ACR_PGA0EN | AFEC_ACR_PGA1EN;
    pub const CFG_AFE_IDR: u32 = 0x47000FFF;
    pub const CFG_AFE_COCR: u32 = 0x200 & AFE_COCR_AOFF_Msk;
}
use cfg::*;

fn pin_to_gpio_adc(pin: u32) -> R<GpioAdc> {
    let mut chan = 0;
    loop {
        if chan >= AFEC_PINS.len() {
            shutdown!("Not a valid ADC pin");
        }
        if AFEC_PINS[chan] as u32 == pin {
            break;
        }
        chan += 1;
    }
    Ok(GpioAdc { chan: chan as u32 })
}

fn gpio_adc_to_afec(g: GpioAdc) -> *mut Afec {
    if g.chan >= AFEC1_START {
        AFEC1
    } else {
        AFEC0
    }
}

fn gpio_adc_to_afec_chan(g: GpioAdc) -> u32 {
    if g.chan >= AFEC1_START {
        g.chan - AFEC1_START
    } else {
        g.chan
    }
}

const ADC_FREQ_MAX: u32 = 6000000;
decl_constant!("ADC_MAX", 4095);

fn init_afec(afec_p: *mut Afec) -> i32 {
    let afec = unsafe { &*afec_p };
    let id = if afec_p == AFEC0 { ID_AFEC0 } else { ID_AFEC1 };
    // Enable PMC
    enable_pclock(id);

    // If busy, return busy
    if (ISR(afec).read() & AFE_ISR_DRDY) == AFE_ISR_DRDY {
        return -1;
    }

    // Reset
    CR(afec).write(AFE_CR_SWRST);

    // Configure afec
    let pclk = get_pclock_frequency(id);
    MR(afec).write(CFG_AFE_MR(pclk));
    EMR(afec).write(AFE_EMR_TAG | AFE_EMR_RES_NO_AVERAGE | AFE_EMR_STM);
    ACR(afec).write(CFG_AFE_ACR);

    // Disable interrupts
    IDR(afec).write(CFG_AFE_IDR);

    // Disable SW triggering
    let mut mr = MR(afec).read();

    mr &= !(AFE_MR_TRGSEL_Msk | AFE_MR_TRGEN | AFE_MR_FREERUN_ON);
    mr |= AFE_MR_TRGEN_DIS;
    MR(afec).write(mr);

    0
}

pub fn gpio_afec_init() -> R {
    while init_afec(AFEC0) != 0 {
        let _ = LCDR(unsafe { &*AFEC0 }).read() & AFE_LCDR_LDATA_Msk;
    }
    while init_afec(AFEC1) != 0 {
        let _ = LCDR(unsafe { &*AFEC1 }).read() & AFE_LCDR_LDATA_Msk;
    }
    Ok(())
}
decl_init!(gpio_afec_init);

pub fn gpio_adc_setup(pin: u32) -> R<GpioAdc> {
    let adc_pin = pin_to_gpio_adc(pin)?;
    let afec = unsafe { &*gpio_adc_to_afec(adc_pin) };
    let afec_chan = gpio_adc_to_afec_chan(adc_pin);

    //config channel
    let mut reg = DIFFR(afec).read();
    reg &= !(1u32 << afec_chan);
    DIFFR(afec).write(reg);
    reg = CGR(afec).read();
    reg &= !(0x03u32 << (2 * afec_chan));
    CGR(afec).write(reg);

    // Configure channel
    // afec_ch_get_config_defaults(&ch_cfg);
    // afec_ch_set_config(afec, afec_chan, &ch_cfg);
    // Remove default internal offset from channel
    // See Atmel Appnote AT03078 Section 1.5 for SAM4E,
    // datasheet section 52.6.11 for SAME70
    CSELR(afec).write(afec_chan);
    COCR(afec).write(CFG_AFE_COCR);

    // Enable and calibrate Channel
    CHER(afec).write(1 << afec_chan);

    #[cfg(CONFIG_MACH_SAM4E)]
    {
        reg = CHSR(afec).read();
        CDOR(afec).write(reg);
        CR(afec).write(AFE_CR_AUTOCAL);
    }

    Ok(adc_pin)
}

const AFE_DUMMY: u8 = 0xff;
static ACTIVE_CHANNEL: Global<u8> = Global::new(AFE_DUMMY);

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
pub fn gpio_adc_sample(g: GpioAdc) -> u32 {
    let afec = unsafe { &*gpio_adc_to_afec(g) };
    let afec_chan = gpio_adc_to_afec_chan(g);
    let active_channel = ACTIVE_CHANNEL.get();
    'need_delay: {
        if *active_channel as u32 == g.chan {
            if ISR(afec).read() & AFE_ISR_DRDY != 0 && ISR(afec).read() & (1 << afec_chan) != 0 {
                // Sample now ready
                return 0;
            } else {
                // Busy
                break 'need_delay;
            }
        } else if *active_channel != AFE_DUMMY {
            break 'need_delay;
        }

        CHDR(afec).write(0x803F); // Disable all channels
        CHER(afec).write(1 << afec_chan);

        *active_channel = g.chan as u8;

        for chan in 0..16 {
            if ISR(afec).read() & (1 << chan) != 0 {
                CSELR(afec).write(chan);
                CDR(afec).read();
            }
        }
        CR(afec).write(AFE_CR_START);
    }
    // about 400 mcu clock cycles or 40 afec cycles
    (ADC_FREQ_MAX as u64 * 10000 / CONFIG_CLOCK_FREQ as u64) as u32
}

// Read a value; use only after gpio_adc_sample() returns zero
pub fn gpio_adc_read(g: GpioAdc) -> R<u16> {
    let afec = unsafe { &*gpio_adc_to_afec(g) };
    let afec_chan = gpio_adc_to_afec_chan(g);
    *ACTIVE_CHANNEL.get() = AFE_DUMMY;
    CSELR(afec).write(afec_chan);
    Ok(CDR(afec).read() as u16)
}

// Cancel a sample that may have been started with gpio_adc_sample()
pub fn gpio_adc_cancel_sample(g: GpioAdc) {
    let active_channel = ACTIVE_CHANNEL.get();
    if *active_channel as u32 == g.chan {
        *active_channel = AFE_DUMMY;
    }
}
