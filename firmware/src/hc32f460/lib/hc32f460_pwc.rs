// lib/hc32f460/driver/src/hc32f460_pwc.c: the function clock gates
//
// Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
// (BSD 3-Clause, see lib/hc32f460)
use super::super::internal::*;

fn mstp() -> &'static M4_MSTP_TypeDef {
    unsafe { &*M4_MSTP }
}

pub fn PWC_Fcg1PeriphClockCmd(u32Fcg1Periph: u32, enNewState: u32) {
    if Enable == enNewState {
        mstp().FCG1.and(!u32Fcg1Periph);
    } else {
        mstp().FCG1.or(u32Fcg1Periph);
    }
}

pub fn PWC_Fcg2PeriphClockCmd(u32Fcg2Periph: u32, enNewState: u32) {
    if Enable == enNewState {
        mstp().FCG2.and(!u32Fcg2Periph);
    } else {
        mstp().FCG2.or(u32Fcg2Periph);
    }
}

pub fn PWC_Fcg3PeriphClockCmd(u32Fcg3Periph: u32, enNewState: u32) {
    if Enable == enNewState {
        mstp().FCG3.and(!u32Fcg3Periph);
    } else {
        mstp().FCG3.or(u32Fcg3Periph);
    }
}
