// lib/hc32f460/driver/src/hc32f460_adc.c: the ADC
//
// Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
// (BSD 3-Clause, see lib/hc32f460)
use super::super::internal::*;

// ADC initialization structure (hc32f460_adc.h)
pub struct stc_adc_init_t {
    pub enResolution: u32,
    pub enDataAlign: u32,
    pub enAutoClear: u32,
    pub enScanMode: u32,
    pub enRschsel: u32,
}

// ADC channel configuration structure (hc32f460_adc.h)
pub struct stc_adc_ch_cfg_t<'a> {
    pub u32Channel: u32,
    pub u8Sequence: u8,
    pub pu8SampTime: &'a [u8],
}

// Initializes an ADC instance.
pub fn ADC_Init(ADCx: &M4_ADC_TypeDef, pstcInit: &stc_adc_init_t) {
    // Stop ADC conversion.
    ADCx.STR.write(0);

    ADCx.CR0.set_field(stc_adc_cr0_field_t::ACCSEL, pstcInit.enResolution);
    ADCx.CR0.set_field(stc_adc_cr0_field_t::DFMT, pstcInit.enDataAlign);
    ADCx.CR0.set_field(stc_adc_cr0_field_t::CLREN, pstcInit.enAutoClear);
    ADCx.CR0.set_field(stc_adc_cr0_field_t::MS, pstcInit.enScanMode);
    ADCx.CR1.set_field(stc_adc_cr1_field_t::RSCHSEL, pstcInit.enRschsel);

    // Disable EOC(End Of Conversion) interrupts default.
    ADCx.ICR.write(0x0);
}

// Add ADC channels.
pub fn ADC_AddAdcChannel(ADCx: &M4_ADC_TypeDef, pstcChCfg: &stc_adc_ch_cfg_t) {
    if pstcChCfg.u8Sequence as u32 <= ADC_SEQ_B {
        let mut u32ChannelSel;
        if core::ptr::eq(M4_ADC1, ADCx) {
            u32ChannelSel = pstcChCfg.u32Channel & ADC1_CH_ALL;
            if ADC_SEQ_A == pstcChCfg.u8Sequence as u32 {
                ADCx.CHSELRA0.or(u32ChannelSel as u16);
                ADCx.CHSELRA1.or((u32ChannelSel >> 16) as u16);
            } else {
                ADCx.CHSELRB0.or(u32ChannelSel as u16);
                ADCx.CHSELRB1.or((u32ChannelSel >> 16) as u16);
            }
        } else {
            u32ChannelSel = pstcChCfg.u32Channel & ADC2_CH_ALL;
            if ADC_SEQ_A == pstcChCfg.u8Sequence as u32 {
                ADCx.CHSELRA0.or(u32ChannelSel as u16);
            } else {
                ADCx.CHSELRB0.or(u32ChannelSel as u16);
            }
        }

        // Set sampling time
        let mut i = 0usize;
        let mut j = 0usize;
        let io8Sstr = ADCx.SSTR0.ptr();
        while 0 != u32ChannelSel {
            if u32ChannelSel & 0x1 != 0 {
                unsafe { io8Sstr.add(i).write_volatile(pstcChCfg.pu8SampTime[j]) };
                j += 1;
            }
            u32ChannelSel >>= 1;
            i += 1;
        }
    }
}

// Start an ADC conversion.
pub fn ADC_StartConvert(ADCx: &M4_ADC_TypeDef) {
    ADCx.STR.write(1);
}

// Stop an ADC conversion.
pub fn ADC_StopConvert(ADCx: &M4_ADC_TypeDef) {
    let mut u32TimeCount = 0u32;

    // Make sure the ADC is really stopped.
    while ADCx.STR.read() == 1 {
        // Stop ADC conversion.
        ADCx.STR.write(0);
        u32TimeCount += 1;
        if u32TimeCount > 10000 {
            break;
        }
    }
}

// Get the end of conversion flag of a sequence.
pub fn ADC_GetEocFlag(ADCx: &M4_ADC_TypeDef, u8Seq: u8) -> u32 {
    let mut enFlag = Reset;

    if ADCx.ISR.read() & (1u32 << u8Seq) as u8 != 0 {
        enFlag = Set;
    }

    enFlag
}

// Clear the end of conversion flag of a sequence.
pub fn ADC_ClrEocFlag(ADCx: &M4_ADC_TypeDef, u8Seq: u8) {
    ADCx.ISR.and(!(1u32 << u8Seq) as u8);
}

// Get the conversion result of a channel.
pub fn ADC_GetValue(ADCx: &M4_ADC_TypeDef, u8ChIndex: u8) -> u16 {
    let pu16DataReg = ADCx.DR0.ptr();

    unsafe { pu16DataReg.add(u8ChIndex as usize).read_volatile() }
}
