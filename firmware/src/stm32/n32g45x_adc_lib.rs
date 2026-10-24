// The ADC functions of the N32G45x library (lib/n32g45x/n32g45x_adc.c)
//
// Copyright (c) 2019, Nations Technologies Inc.
use super::n32g45x_adc_h::*;

// Initializes the ADCx peripheral according to the specified parameters
// in the ADC_InitStruct.
pub fn ADC_Init(ADCx: &ADC_Module, ADC_InitStruct: &ADC_InitType) {
    let mut tmpreg2: u8 = 0;

    // ADCx CTRL1: clear DUALMOD and SCAN bits, then set them from WorkMode
    // and MultiChEn
    let mut tmpreg1 = ADCx.CTRL1.read();
    tmpreg1 &= CTRL1_CLR_MASK;
    tmpreg1 |= ADC_InitStruct.WorkMode | (ADC_InitStruct.MultiChEn << 8);
    ADCx.CTRL1.write(tmpreg1);

    // ADCx CTRL2: clear CONT, ALIGN and EXTSEL bits, then set them from
    // DatAlign, ExtTrigSelect and ContinueConvEn
    tmpreg1 = ADCx.CTRL2.read();
    tmpreg1 &= CTRL2_CLR_MASK;
    tmpreg1 |= ADC_InitStruct.DatAlign | ADC_InitStruct.ExtTrigSelect | (ADC_InitStruct.ContinueConvEn << 1);
    ADCx.CTRL2.write(tmpreg1);

    // ADCx RSEQ1: the regular channel sequence length (L bits) from ChsNumber
    tmpreg1 = ADCx.RSEQ1.read();
    tmpreg1 &= RSEQ1_CLR_MASK;
    tmpreg2 |= ADC_InitStruct.ChsNumber.wrapping_sub(1);
    tmpreg1 |= (tmpreg2 as u32) << 20;
    ADCx.RSEQ1.write(tmpreg1);
}

// Configures for the selected ADC regular channel its corresponding
// rank in the sequencer and its sample time.
pub fn ADC_ConfigRegularChannel(ADCx: &ADC_Module, ADC_Channel: u8, Rank: u8, ADC_SampleTime: u8) {
    let (ch, rank) = (ADC_Channel as u32, Rank as u32);
    if ch == ADC_CH_18 {
        let mut tmpreg1 = ADCx.SAMPT3.read();
        tmpreg1 &= !0x00000007;
        tmpreg1 |= ADC_SampleTime as u32;
        ADCx.SAMPT3.write(tmpreg1);
    } else if ch > ADC_CH_9 {
        // ADC_CH_10 ... ADC_CH_17
        let mut tmpreg1 = ADCx.SAMPT1.read();
        tmpreg1 &= !(SAMPT1_SMP_SET << (3 * (ch - 10)));
        tmpreg1 |= (ADC_SampleTime as u32) << (3 * (ch - 10));
        ADCx.SAMPT1.write(tmpreg1);
    } else {
        // ADC_CH_0 ... ADC_CH_9
        let mut tmpreg1 = ADCx.SAMPT2.read();
        tmpreg1 &= !(SAMPT2_SMP_SET << (3 * ch));
        tmpreg1 |= (ADC_SampleTime as u32) << (3 * ch);
        ADCx.SAMPT2.write(tmpreg1);
    }
    if rank < 7 {
        // Rank 1 to 6
        let mut tmpreg1 = ADCx.RSEQ3.read();
        tmpreg1 &= !(SQR3_SEQ_SET << (5 * (rank - 1)));
        tmpreg1 |= ch << (5 * (rank - 1));
        ADCx.RSEQ3.write(tmpreg1);
    } else if rank < 13 {
        // Rank 7 to 12
        let mut tmpreg1 = ADCx.RSEQ2.read();
        tmpreg1 &= !(SQR2_SEQ_SET << (5 * (rank - 7)));
        tmpreg1 |= ch << (5 * (rank - 7));
        ADCx.RSEQ2.write(tmpreg1);
    } else {
        // Rank 13 to 16
        let mut tmpreg1 = ADCx.RSEQ1.read();
        tmpreg1 &= !(SQR1_SEQ_SET << (5 * (rank - 13)));
        tmpreg1 |= ch << (5 * (rank - 13));
        ADCx.RSEQ1.write(tmpreg1);
    }
}
