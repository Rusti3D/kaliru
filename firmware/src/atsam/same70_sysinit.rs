// This code is from lib/sam4e/gcc/system_sam4e.c and modified for the SAM E70
#![allow(non_snake_case)]
use super::internal::*;
use crate::config::{CONFIG_ARMCM_ITCM_FLASH_MIRROR_START, CONFIG_FLASH_APPLICATION_ADDRESS};
use crate::generic::cmsis::{SCB_EnableDCache, __DSB, __ISB};
use core::ptr::addr_of_mut;

/* Clock Settings (300MHz) */
const SYS_BOARD_OSCOUNT: u32 = CKGR_MOR_MOSCXTST(0x8);
const SYS_BOARD_PLLAR: u32 = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0x18) | CKGR_PLLAR_PLLACOUNT(0x3f) | CKGR_PLLAR_DIVA_BYPASS;
const SYS_BOARD_MCKR: u32 = PMC_MCKR_MDIV_PCK_DIV2 | PMC_MCKR_CSS_PLLA_CLK;
const RST_PARAMS: u32 = (0xA5 << RSTC_CR_KEY_Pos) | RSTC_CR_PROCRST;
const GPNVM_TCM_MASK: u32 = (1 << 7) | (1 << 8);

/* Key to unlock MOR register */
const SYS_CKGR_MOR_KEY_VALUE: u32 = CKGR_MOR_KEY(0x37);

extern "C" {
    static mut _text_size: u32;
    static mut _dtcm_bss_start: u32;
    static mut _dtcm_bss_end: u32;
}

fn reg(r: *mut u32) -> &'static Reg<u32> {
    unsafe { &*(r as *const Reg<u32>) }
}

pub fn SystemInit() {
    let pmc = unsafe { &*PMC };
    let efc = unsafe { &*EFC };
    /* Set FWS according to SYS_BOARD_MCKR configuration */
    efc.EEFC_FMR.write(EEFC_FMR_FWS(6) | EEFC_FMR_CLOE);

    /* Initialize main oscillator */
    if pmc.CKGR_MOR.read() & CKGR_MOR_MOSCSEL == 0 {
        pmc.CKGR_MOR.write(SYS_CKGR_MOR_KEY_VALUE | SYS_BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN);

        while pmc.PMC_SR.read() & PMC_SR_MOSCXTS == 0 {}
    }

    /* Switch to 3-20MHz Xtal oscillator */
    pmc.CKGR_MOR.write(SYS_CKGR_MOR_KEY_VALUE | SYS_BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCSEL);

    while pmc.PMC_SR.read() & PMC_SR_MOSCSELS == 0 {}

    pmc.PMC_MCKR.write((pmc.PMC_MCKR.read() & !PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK);

    while pmc.PMC_SR.read() & PMC_SR_MCKRDY == 0 {}

    /* Initialize PLLA */
    pmc.CKGR_PLLAR.write(SYS_BOARD_PLLAR);
    while pmc.PMC_SR.read() & PMC_SR_LOCKA == 0 {}

    /* Switch to main clock */
    pmc.PMC_MCKR.write((SYS_BOARD_MCKR & !PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK);
    while pmc.PMC_SR.read() & PMC_SR_MCKRDY == 0 {}

    /* Switch to PLLA */
    pmc.PMC_MCKR.write(SYS_BOARD_MCKR);
    while pmc.PMC_SR.read() & PMC_SR_MCKRDY == 0 {}

    /* Configure PCK6 for TC use */
    pmc.PMC_PCK[6].write(PMC_PCK_CSS_MCK | PMC_PCK_PRES(2));
    while pmc.PMC_SR.read() & PMC_SR_PCKRDY6 == 0 {}
    pmc.PMC_SCER.or(PMC_SCER_PCK6);

    /* Check Tightly Coupled Memory (TCM) bits. */
    efc.EEFC_FCR.write(EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_GGPB);
    while (efc.EEFC_FSR.read() & EEFC_FSR_FRDY) == 0 {}
    if (efc.EEFC_FRR.read() & GPNVM_TCM_MASK) != GPNVM_TCM_MASK {
        /* Configure TCM sizes to 128KiB (set GPNVM7 and GPNVM8). */
        efc.EEFC_FCR.write(EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SGPB | EEFC_FCR_FARG(7));
        while (efc.EEFC_FSR.read() & EEFC_FSR_FRDY) == 0 {}
        efc.EEFC_FCR.write(EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SGPB | EEFC_FCR_FARG(8));
        while (efc.EEFC_FSR.read() & EEFC_FSR_FRDY) == 0 {}
        /* Reboot required, but bits are set now and we will not
         * return down this path. */
        __DSB();
        __ISB();
        unsafe { &*RSTC }.RSTC_CR.write(RST_PARAMS);
        #[allow(clippy::empty_loop)]
        loop {}
    }

    /* Clear Data Tightly Coupled Memory (DTCM) bss segment - this has to happen
     *  after we check that the DTCM is enabled. */
    unsafe {
        let start = addr_of_mut!(_dtcm_bss_start);
        let count = addr_of_mut!(_dtcm_bss_end) as usize - start as usize;
        core::ptr::write_bytes(start as *mut u8, 0, count);
    }

    /* DMA copy flash to Instruction Tightly Coupled Memory (ITCM). Just use
     *  channel 0 since we have not done anything yet. */
    enable_pclock(ID_XDMAC);
    /* Clear pending interrupts. */
    reg(REG_XDMAC_CIS0).read();
    reg(REG_XDMAC_CSA0).write(CONFIG_ARMCM_ITCM_FLASH_MIRROR_START);
    reg(REG_XDMAC_CDA0).write(CONFIG_FLASH_APPLICATION_ADDRESS);
    reg(REG_XDMAC_CUBC0).write(XDMAC_CUBC_UBLEN(addr_of_mut!(_text_size) as u32));

    reg(REG_XDMAC_CC0).write(
        XDMAC_CC_TYPE_MEM_TRAN
            | XDMAC_CC_MBSIZE_SINGLE
            | XDMAC_CC_CSIZE_CHK_1
            | XDMAC_CC_DWIDTH_WORD
            | XDMAC_CC_SAM_INCREMENTED_AM
            | XDMAC_CC_DAM_INCREMENTED_AM
            | XDMAC_CC_SIF_AHB_IF1
            | XDMAC_CC_DIF_AHB_IF0,
    );

    reg(REG_XDMAC_CNDA0).write(0);
    reg(REG_XDMAC_CNDC0).write(0);
    reg(REG_XDMAC_CBC0).write(0);
    reg(REG_XDMAC_CDS_MSP0).write(0);
    reg(REG_XDMAC_CSUS0).write(0);
    reg(REG_XDMAC_CDUS0).write(0);

    reg(REG_XDMAC_CIE0).write(XDMAC_CIE_BIE);

    __DSB();
    __ISB();
    let xdmac = unsafe { &*XDMAC };
    xdmac.XDMAC_GE.write(XDMAC_GE_EN0);
    while xdmac.XDMAC_GS.read() & XDMAC_GS_ST0 != 0 {}

    while reg(REG_XDMAC_CIS0).read() & XDMAC_CIS_BIS == 0 {}

    /* Enable ITCM. DTCM is enabled by default. */
    __DSB();
    __ISB();
    unsafe { &*SCB }.ITCMCR.write(SCB_ITCMCR_EN_Msk | SCB_ITCMCR_RMW_Msk | SCB_ITCMCR_RETEN_Msk);
    __DSB();
    __ISB();

    /* Use data cache rather than DTCM for two reasons:
     *  1. It is hard to flash this device with GPNVM bits enabled.
     *  2. It is about as fast. */
    SCB_EnableDCache();
}
