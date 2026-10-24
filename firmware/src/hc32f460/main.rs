// Code to setup clocks on Huada HC32F460
//
// Copyright (C) 2022  Steven Gotthardt <gotthardt@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::lib::hc32f460_gpio::PORT_DebugPortSetting;
use super::lib::system_hc32f460::SystemInit;
use crate::sched::sched_main;

/****************************************************************
 * Startup
 ****************************************************************/

// Main entry point - called from armcm_boot.c:ResetHandler()
pub fn armcm_main() -> ! {
    // sets the system clock speed variable for library use
    SystemInit();

    // disable JTAG/SWD on pins PA13, PA14, PA15, PB3, PB4
    // SWD still works until the relevant pins are reconfigured. Proprietary
    // flash program (XHSC ISP) must be used to reflash afterwards.
    PORT_DebugPortSetting(ALL_DBG_PIN as u8, Disable);

    // manage the system
    sched_main();
}
