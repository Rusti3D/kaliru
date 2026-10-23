// SDIO functions on STM32
//
// Copyright (C) 2022  H. Gregor Molter <gregor.molter@secretlab.de>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use super::system_stm32f4xx::SystemCoreClock;
use crate::generic::armcm_timer::udelay;
use crate::sched::R;
use crate::{decl_constant_str, decl_enumeration, shutdown};

// struct sdio_config (sdio.h)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct SdioConfig {
    pub sdio: *mut SDIO_TypeDef,
}

struct SdioInfo {
    sdio: *mut SDIO_TypeDef,
    clk_pin: u8,
    cmd_pin: u8,
    dat0_pin: u8,
    dat1_pin: u8,
    dat2_pin: u8,
    dat3_pin: u8,
    function: u8,
}
unsafe impl Sync for SdioInfo {}

const SDIO_OK: u8 = 0;
#[allow(dead_code)]
const SDIO_ERROR: u8 = 1;
const SDIO_TIMEOUT: u8 = 2;
const SDIO_CMD_RESPONSE_TIMEOUT: u8 = 3;
const SDIO_CRC_FAIL: u8 = 4;
const SDIO_WRONG_CMD_RESPONSE: u8 = 5;
const SDIO_WRONG_BLOCKSIZE: u8 = 6;
const SDIO_DATA_TIMEOUT: u8 = 7;
const SDIO_READ_OVERRUN: u8 = 8;
const SDIO_WRITE_UNDERRUN: u8 = 9;
const SDIO_NO_DATA_MEMORY: u8 = 10;

#[allow(dead_code)]
const SDIO_CARDVER_UNKNOWN: u8 = 0;
#[allow(dead_code)]
const SDIO_CARDVER_1X: u8 = 1;
#[allow(dead_code)]
const SDIO_CARDVER_2X: u8 = 2;

#[allow(dead_code)]
const SDIO_CARDTYPE_UNKNOWN: u8 = 0;
#[allow(dead_code)]
const SDIO_CARDTYPE_SDHC_SDXC: u8 = 1;
#[allow(dead_code)]
const SDIO_CARDTYPE_SDSC: u8 = 2;

const SDIO_WAIT_NO_RESPONSE: u8 = 0;
const SDIO_WAIT_SHORT_RESPONSE: u8 = 1;
const SDIO_WAIT_LONG_RESPONSE: u8 = 2;

// PINS: CLK -> PC12 , CMD -> PD2,
//       DAT0 -> PC8, DAT1 -> PC9, DAT2 -> PC10, DAT3 -> PC11
decl_enumeration!("sdio_bus", "sdio", 0);
decl_constant_str!("BUS_PINS_sdio", "PC12,PD2,PC8,PC9,PC10,PC11");

const SDIO_FUNCTION: u8 = GPIO_FUNCTION(12) as u8;

static SDIO_BUS: &[SdioInfo] = &[SdioInfo {
    sdio: SDIO,
    clk_pin: GPIO('C', 12) as u8,
    cmd_pin: GPIO('D', 2) as u8,
    dat0_pin: GPIO('C', 8) as u8,
    dat1_pin: GPIO('C', 9) as u8,
    dat2_pin: GPIO('C', 10) as u8,
    dat3_pin: GPIO('C', 11) as u8,
    function: SDIO_FUNCTION,
}];

const SDIO_CLK_FREQ: u32 = 48000000;
const SDIO_INIT_CLK: u32 = 400000;
const SDIO_MAX_TIMEOUT: u32 = 500; // Wait for at least 500ms before a timeout occurs
const CLKCR_CLEAR_MASK: u32 =
    SDIO_CLKCR_CLKDIV | SDIO_CLKCR_PWRSAV | SDIO_CLKCR_BYPASS | SDIO_CLKCR_WIDBUS | SDIO_CLKCR_NEGEDGE | SDIO_CLKCR_HWFC_EN;
const DCTRL_CLEAR_MASK: u32 = SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE | SDIO_DCTRL_DBLOCKSIZE;
const CMD_CLEAR_MASK: u32 =
    SDIO_CMD_CMDINDEX | SDIO_CMD_WAITRESP | SDIO_CMD_WAITINT | SDIO_CMD_WAITPEND | SDIO_CMD_CPSMEN | SDIO_CMD_SDIOSUSPEND;
const SDIO_CLOCK_BYPASS_DISABLE: u32 = 0;
const SDIO_CLOCK_EDGE_RISING: u32 = 0;
const SDIO_CLOCK_POWER_SAVE_DISABLE: u32 = 0;
const SDIO_BUS_WIDE_1B: u32 = 0;
#[allow(dead_code)]
const SDIO_BUS_WIDE_4B: u32 = SDIO_CLKCR_WIDBUS_0;
const SDIO_HARDWARE_FLOW_CONTROL_DISABLE: u32 = 0;
#[allow(dead_code)]
const SDIO_HARDWARE_FLOW_CONTROL_ENABLE: u32 = SDIO_CLKCR_HWFC_EN;
const SDIO_CMD_FLAGS: u32 = SDIO_STA_CCRCFAIL | SDIO_STA_CTIMEOUT | SDIO_STA_CMDREND | SDIO_STA_CMDSENT;
const SDIO_STATIC_FLAGS: u32 = SDIO_STA_CCRCFAIL
    | SDIO_STA_DCRCFAIL
    | SDIO_STA_CTIMEOUT
    | SDIO_STA_DTIMEOUT
    | SDIO_STA_TXUNDERR
    | SDIO_STA_RXOVERR
    | SDIO_STA_CMDREND
    | SDIO_STA_CMDSENT
    | SDIO_STA_DATAEND
    | SDIO_STA_DBCKEND
    | SDIO_STA_SDIOIT;

pub fn sdio_setup(bus: u32) -> R<SdioConfig> {
    if bus as usize >= SDIO_BUS.len() {
        shutdown!("Invalid sdio bus");
    }

    // Enable SDIO
    let b = &SDIO_BUS[bus as usize];
    if !is_enabled_pclock(b.sdio as u32) {
        // Enable clock
        enable_pclock(b.sdio as u32);
        // Initialize pins
        let f = b.function as u32 | GPIO_HIGH_SPEED;
        gpio_peripheral(b.dat0_pin as u32, f, 1);
        gpio_peripheral(b.dat1_pin as u32, f, 1);
        gpio_peripheral(b.dat2_pin as u32, f, 1);
        gpio_peripheral(b.dat3_pin as u32, f, 1);
        gpio_peripheral(b.cmd_pin as u32, f, 1);
        gpio_peripheral(b.clk_pin as u32, f, 1);
    }

    let sdio_config = SdioConfig { sdio: b.sdio };
    let sdio = unsafe { &*b.sdio };

    // Setup SDIO with 1 bit width first and slow clock ~400 kHz
    sdio_set_speed(sdio_config, SDIO_INIT_CLK);

    // Disable clk
    sdio.CLKCR.and(!SDIO_CLKCR_CLKEN);
    // Set power state to _on_
    sdio.POWER.write(SDIO_POWER_PWRCTRL);
    // Wait for 2ms (standard: at least 1ms) to settle
    udelay(2000);
    // Enable Clk
    sdio.CLKCR.or(SDIO_CLKCR_CLKEN);

    Ok(sdio_config)
}

#[allow(dead_code)]
pub fn sdio_get_cmd_error(sdio_config: SdioConfig, flags: u32) -> u32 {
    let regs = unsafe { &*sdio_config.sdio };

    // wait for a timeout (max. SDIO_MAX_TIMEOUT) in msec.
    // 8 cycles is the instruction cycles for the loop below.
    let mut sta;
    let mut count = SDIO_MAX_TIMEOUT * (SystemCoreClock / 8 / 1000);
    loop {
        if count == 0 {
            return SDIO_TIMEOUT as u32;
        }
        count -= 1;
        sta = regs.STA.read();
        if !((sta & flags) == 0 || (sta & SDIO_STA_CMDACT) != 0) {
            break;
        }
    }

    regs.ICR.write(SDIO_CMD_FLAGS);

    if sta & SDIO_STA_CTIMEOUT != 0 {
        return SDIO_CMD_RESPONSE_TIMEOUT as u32;
    }
    if sta & SDIO_STA_CCRCFAIL != 0 {
        return SDIO_CRC_FAIL as u32;
    }

    SDIO_OK as u32
}

pub fn sdio_send_command(
    sdio_config: SdioConfig,
    cmd: u8,
    argument: u32,
    wait: u8,
    response_data: Option<&mut [u8; 16]>,
    response_data_len: &mut u8,
) -> u8 {
    let sdio = unsafe { &*sdio_config.sdio };
    let mut wait_flags = 0; // valid for SDIO_WAIT_NO_RESPONSE
    let sta_flags = if wait == SDIO_WAIT_NO_RESPONSE {
        SDIO_STA_CMDSENT
    } else {
        SDIO_STA_CCRCFAIL | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT
    };

    if wait == SDIO_WAIT_SHORT_RESPONSE {
        wait_flags = SDIO_CMD_WAITRESP_0;
    } else if wait == SDIO_WAIT_LONG_RESPONSE {
        wait_flags = SDIO_CMD_WAITRESP;
    }

    // Step 1: Send command and argument
    // CMD and State Machine enabled.
    // Wait for response like specified by wait_flags.
    let cmdreg = (cmd as u32 & 0x3F) | wait_flags | SDIO_CMD_CPSMEN;
    sdio.ARG.write(argument);
    sdio.CMD.modify(CMD_CLEAR_MASK, cmdreg);

    // Step 2: Wait until response
    // wait for a timeout (max. SDIO_MAX_TIMEOUT) in msec.
    // 8 cycles is the instruction cycles for the loop below.
    let mut sta;
    let mut count = SDIO_MAX_TIMEOUT * (SystemCoreClock / 8 / 1000);
    loop {
        if count == 0 {
            return SDIO_TIMEOUT;
        }
        count -= 1;
        sta = sdio.STA.read();
        if !((sta & sta_flags) == 0 || (sta & SDIO_STA_CMDACT) != 0) {
            break;
        }
    }

    sdio.ICR.write(SDIO_CMD_FLAGS);

    // Step 3: Store response_data and check for short and long responses
    // timeout and crc.
    if let Some(response_data) = response_data {
        if wait == SDIO_WAIT_SHORT_RESPONSE {
            response_data[..4].copy_from_slice(&sdio.RESP1.read().to_be_bytes());
            *response_data_len = 4;
        } else if wait == SDIO_WAIT_LONG_RESPONSE {
            // TODO Inverse?
            response_data[..4].copy_from_slice(&sdio.RESP1.read().to_be_bytes());
            response_data[4..8].copy_from_slice(&sdio.RESP2.read().to_be_bytes());
            response_data[8..12].copy_from_slice(&sdio.RESP3.read().to_be_bytes());
            response_data[12..16].copy_from_slice(&sdio.RESP4.read().to_be_bytes());
            *response_data_len = 16;
        }
    }

    if wait != SDIO_WAIT_NO_RESPONSE {
        // CTIMEOUT and CCRCFAIL check only for short or long responses.
        if sta & SDIO_STA_CTIMEOUT != 0 {
            return SDIO_CMD_RESPONSE_TIMEOUT;
        }
        if sta & SDIO_STA_CCRCFAIL != 0 {
            return SDIO_CRC_FAIL;
        }
    }

    // Step 4: For a short response check the response cmd field, too.
    if wait == SDIO_WAIT_SHORT_RESPONSE && sdio.RESPCMD.read() != cmd as u32 {
        return SDIO_WRONG_CMD_RESPONSE;
    }

    SDIO_OK
}

fn sdio_get_dctrl_blocksize(value: u32) -> u32 {
    match value {
        1 => 0,
        2 => SDIO_DCTRL_DBLOCKSIZE_0,
        4 => SDIO_DCTRL_DBLOCKSIZE_1,
        8 => SDIO_DCTRL_DBLOCKSIZE_1 | SDIO_DCTRL_DBLOCKSIZE_0,
        16 => SDIO_DCTRL_DBLOCKSIZE_2,
        32 => SDIO_DCTRL_DBLOCKSIZE_2 | SDIO_DCTRL_DBLOCKSIZE_0,
        64 => SDIO_DCTRL_DBLOCKSIZE_2 | SDIO_DCTRL_DBLOCKSIZE_1,
        128 => SDIO_DCTRL_DBLOCKSIZE_2 | SDIO_DCTRL_DBLOCKSIZE_1 | SDIO_DCTRL_DBLOCKSIZE_0,
        256 => SDIO_DCTRL_DBLOCKSIZE_3,
        512 => SDIO_DCTRL_DBLOCKSIZE_3 | SDIO_DCTRL_DBLOCKSIZE_0,
        1024 => SDIO_DCTRL_DBLOCKSIZE_3 | SDIO_DCTRL_DBLOCKSIZE_1,
        2048 => SDIO_DCTRL_DBLOCKSIZE_3 | SDIO_DCTRL_DBLOCKSIZE_1 | SDIO_DCTRL_DBLOCKSIZE_0,
        4096 => SDIO_DCTRL_DBLOCKSIZE_3 | SDIO_DCTRL_DBLOCKSIZE_2,
        8192 => SDIO_DCTRL_DBLOCKSIZE_3 | SDIO_DCTRL_DBLOCKSIZE_2 | SDIO_DCTRL_DBLOCKSIZE_0,
        16384 => SDIO_DCTRL_DBLOCKSIZE_3 | SDIO_DCTRL_DBLOCKSIZE_2 | SDIO_DCTRL_DBLOCKSIZE_1,
        _ => SDIO_DCTRL_DBLOCKSIZE_Msk,
    }
}

pub fn sdio_prepare_data_transfer(sdio_config: SdioConfig, read: u8, numblocks: u32, blocksize: u32, timeout: u32) -> u8 {
    let sdio = unsafe { &*sdio_config.sdio };
    let dctrl_blocksize = sdio_get_dctrl_blocksize(blocksize);
    let reg = dctrl_blocksize | (if read > 0 { SDIO_DCTRL_DTDIR } else { 0 }) | SDIO_DCTRL_DTEN;

    if dctrl_blocksize == SDIO_DCTRL_DBLOCKSIZE_Msk {
        return SDIO_WRONG_BLOCKSIZE;
    }

    sdio.DCTRL.write(0);
    sdio.DTIMER.write(timeout);
    sdio.DLEN.write(numblocks.wrapping_mul(blocksize));
    sdio.DCTRL.modify(DCTRL_CLEAR_MASK, reg);
    SDIO_OK
}

pub fn sdio_read_data(sdio_config: SdioConfig, data: Option<&mut [u8]>, numblocks: u32, blocksize: u32) -> u8 {
    // Read data by polling
    let sdio = unsafe { &*sdio_config.sdio };
    let mut data_remaining = numblocks.wrapping_mul(blocksize);

    let Some(buf) = data else {
        return SDIO_NO_DATA_MEMORY;
    };
    let mut pos = 0;

    while sdio.STA.read() & (SDIO_STA_RXOVERR | SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | SDIO_STA_DATAEND) == 0 {
        if sdio.STA.read() & SDIO_STA_RXDAVL != 0 {
            let mut tmp = sdio.FIFO.read();
            let mut i = 0;
            while i < 4 && data_remaining > 0 {
                buf[pos] = (tmp & 0xFF) as u8;
                pos += 1;
                data_remaining -= 1;
                tmp >>= 8;
                i += 1;
            }
        }
    }

    let sta = sdio.STA.read();
    sdio.ICR.write(SDIO_STATIC_FLAGS);

    if sta & SDIO_STA_DTIMEOUT != 0 {
        return SDIO_DATA_TIMEOUT;
    }
    if sta & SDIO_STA_DCRCFAIL != 0 {
        return SDIO_CRC_FAIL;
    }
    if sta & SDIO_STA_RXOVERR != 0 {
        return SDIO_READ_OVERRUN;
    }

    // Empty FIFO and clear flags again
    while (sdio.STA.read() & SDIO_STA_RXDAVL) != 0 && data_remaining > 0 {
        let mut tmp = sdio.FIFO.read();
        let mut i = 0;
        while i < 4 && data_remaining > 0 {
            buf[pos] = (tmp & 0xFF) as u8;
            pos += 1;
            data_remaining -= 1;
            tmp >>= 8;
            i += 1;
        }
    }

    sdio.ICR.write(SDIO_STATIC_FLAGS);
    SDIO_OK
}

pub fn sdio_write_data(sdio_config: SdioConfig, data: Option<&[u8]>, numblocks: u32, blocksize: u32) -> u8 {
    // Write data by polling
    let sdio = unsafe { &*sdio_config.sdio };
    let mut data_remaining = numblocks.wrapping_mul(blocksize);

    let Some(buf) = data else {
        return SDIO_NO_DATA_MEMORY;
    };
    let mut pos = 0;

    while sdio.STA.read() & (SDIO_STA_TXUNDERR | SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | SDIO_STA_DATAEND) == 0 {
        if sdio.STA.read() & SDIO_STA_TXFIFOF == 0 {
            let mut tmp = 0u32;
            let mut i = 0;
            while i < 4 && data_remaining > 0 {
                tmp |= (buf[pos] as u32) << (i << 3);
                pos += 1;
                data_remaining -= 1;
                i += 1;
            }
            sdio.FIFO.write(tmp);
        }
    }

    let sta = sdio.STA.read();
    sdio.ICR.write(SDIO_STATIC_FLAGS);

    if sta & SDIO_STA_DTIMEOUT != 0 {
        return SDIO_DATA_TIMEOUT;
    }
    if sta & SDIO_STA_DCRCFAIL != 0 {
        return SDIO_CRC_FAIL;
    }
    if sta & SDIO_STA_TXUNDERR != 0 {
        return SDIO_WRITE_UNDERRUN;
    }

    SDIO_OK
}

#[allow(dead_code)]
pub fn sdio_send_cmd(sdio_config: SdioConfig, cmd: u8, argument: u32, wait: u8) {
    let sdio = unsafe { &*sdio_config.sdio };

    sdio.ARG.write(argument);
    //CMD and State Machine enabled. Wait for response like specified.
    let cmdreg = (cmd as u32 & 0x3F) | (wait as u32 & 0xC0) | SDIO_CMD_CPSMEN;

    sdio.CMD.modify(CMD_CLEAR_MASK, cmdreg);
}

pub fn sdio_set_speed(sdio_config: SdioConfig, speed: u32) {
    let sdio = unsafe { &*sdio_config.sdio };

    // (a speed of 0: the Cortex-M udiv gives 0, no trap)
    let clkdiv = SDIO_CLK_FREQ.checked_div(speed).unwrap_or(0).wrapping_sub(2) as u8;

    let sdio_confreg = SDIO_CLOCK_EDGE_RISING
        | SDIO_CLOCK_BYPASS_DISABLE
        | SDIO_CLOCK_POWER_SAVE_DISABLE
        | SDIO_BUS_WIDE_1B
        | SDIO_HARDWARE_FLOW_CONTROL_DISABLE
        | (clkdiv as u32 & 0xFF);
    sdio.CLKCR.modify(CLKCR_CLEAR_MASK, sdio_confreg);
}
