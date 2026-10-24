// lib/hc32f460/driver/src/hc32f460_gpio.c: port setup
//
// Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
// (BSD 3-Clause, see lib/hc32f460)
use super::super::internal::*;

// Port init configuration structure definition (hc32f460_gpio.h)
pub struct stc_port_init_t {
    pub enPinMode: u32,
    pub enLatch: u32,
    pub enExInt: u32,
    pub enInvert: u32,
    pub enPullUp: u32,
    pub enPinDrv: u32,
    pub enPinOType: u32,
    pub enPinSubFunc: u32,
}

fn port() -> &'static M4_PORT_TypeDef {
    unsafe { &*M4_PORT }
}

// the PCR / PFSR register of a pin (16 bits, 0x40 bytes per port, 4 per pin)
fn pin_reg(first: &Reg<u16>, enPort: u32, u8PinPos: u32) -> &'static Reg<u16> {
    unsafe { &*((first.ptr() as u32 + enPort * 0x40 + u8PinPos * 0x04) as *const Reg<u16>) }
}

// Port init
pub fn PORT_Init(enPort: u32, u16Pin: u16, pstcPortInit: &stc_port_init_t) {
    use stc_port_pcr_field_t as pcr;

    PORT_Unlock();
    for u8PinPos in 0..16u32 {
        if u16Pin as u32 & (1 << u8PinPos) != 0 {
            let PCRx = pin_reg(&port().PCRA0, enPort, u8PinPos);
            let PFSRx = pin_reg(&port().PFSRA0, enPort, u8PinPos);

            // Input latch function setting
            PCRx.set_field(pcr::LTE, pstcPortInit.enLatch);

            // External interrupt input enable setting
            PCRx.set_field(pcr::INTE, pstcPortInit.enExInt);

            // In_Out invert setting
            PCRx.set_field(pcr::INVE, pstcPortInit.enInvert);

            // Pin pull-up setting
            PCRx.set_field(pcr::PUU, pstcPortInit.enPullUp);

            // CMOS/OD output setting
            PCRx.set_field(pcr::NOD, pstcPortInit.enPinOType);

            // Pin drive mode setting
            PCRx.set_field(pcr::DRV, pstcPortInit.enPinDrv);

            // Pin mode setting
            match pstcPortInit.enPinMode {
                Pin_Mode_In => {
                    PCRx.set_field(pcr::DDIS, 0);
                    PCRx.set_field(pcr::POUTE, 0);
                }
                Pin_Mode_Out => {
                    PCRx.set_field(pcr::DDIS, 0);
                    PCRx.set_field(pcr::POUTE, 1);
                }
                Pin_Mode_Ana => {
                    PCRx.set_field(pcr::DDIS, 1);
                }
                _ => {}
            }
            // Sub function enable setting
            PFSRx.set_field(stc_port_pfsr_field_t::BFE, pstcPortInit.enPinSubFunc);
        }
    }
    PORT_Lock();
}

// Debug port setting
pub fn PORT_DebugPortSetting(u8DebugPort: u8, enFunc: u32) {
    PORT_Unlock();

    if Enable == enFunc {
        port().PSPCR.or((u8DebugPort as u16) & 0x1F);
    } else {
        port().PSPCR.and(!((u8DebugPort as u16) & 0x1F));
    }

    PORT_Lock();
}

// Unlock / lock the port registers
pub fn PORT_Unlock() {
    port().PWPR.write(0xA501);
}

pub fn PORT_Lock() {
    port().PWPR.write(0xA500);
}

// Set port pins' function
pub fn PORT_SetFunc(enPort: u32, u16Pin: u16, enFuncSel: u32, enSubFunc: u32) {
    PORT_Unlock();

    for u8PinPos in 0..16u32 {
        if u16Pin & (1u32 << u8PinPos) as u16 != 0 {
            let PFSRx = pin_reg(&port().PFSRA0, enPort, u8PinPos);

            // main function setting
            PFSRx.set_field(stc_port_pfsr_field_t::FSEL, enFuncSel);

            // sub function enable setting
            PFSRx.set_field(stc_port_pfsr_field_t::BFE, if Enable == enSubFunc { Enable } else { Disable });
        }
    }

    PORT_Lock();
}
