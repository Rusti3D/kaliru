// Support for YHCB2004 (20x4 text) LCD displays based on AiP31068 controller
// (port of klippy/extras/display/aip31068_spi.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2021  Marc-Andre Denis <marcadenis@msn.com>
// Copyright (C) 2024  Alexander Bazarov <oss@bazarov.dev>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// This file is a modified version of hd44780_spi.py, introducing slightly
// different protocol as implemented in Marlin FW (based on
// https://github.com/red-scorp/LiquidCrystal_AIP31068 ).
// In addition, a hack is used to send 8 commands, each 9 bits, at once,
// allowing the transmission of a full 9 bytes.
// This helps avoid modifying the SW_SPI driver to handle non-8-bit data.
use super::hd44780::{TextFramebuffers, LINE_LENGTH_DEFAULT, LINE_LENGTH_OPTIONS};
use super::LcdChip;
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::extras::display_templates::Glyph;
use crate::mcu::Mcu;
use crate::printer::Printer;
use crate::IndexMap;
use std::rc::Rc;

// Each command is 9 bits long:
//     1 bit for RS (Register Select) - 0 for command, 1 for data
//     8 bits for the command/data
// Command is a bitwise OR of CMND(=opcode) and flg_CMND(=parameters) multiplied
// by 1 or 0 as En/Dis flag.
//     cmd = CMND | flg_CMND.param0*0 | flg_CMND.param1*1
// or just by OR with enabled flags:
//     cmd = CMND | flg_CMND.param1
const CMND_HOME: i64 = 2; // Return home
const CMND_ENTERY_MODE: i64 = 1 << 2; // Entry mode set
const CMND_DISPLAY: i64 = 1 << 3; // Display on/off control
const CMND_SHIFT: i64 = 1 << 4; // Cursor or display shift
const CMND_FUNCTION: i64 = 1 << 5; // Function set
const CMND_CGRAM: i64 = 1 << 6; // Character Generator RAM
const CMND_DDRAM: i64 = 1 << 7; // Display Data RAM
const CMND_WRITE_RAM: i64 = 1 << 8; // Write to RAM

const FLG_ENTERY_MODE_INC: i64 = 1 << 1; // Increment
const FLG_DISPLAY_ON: i64 = 1 << 2; // Display ON
const FLG_SHIFT_RIGHT: i64 = 1 << 2; // Shift right
const FLG_FUNCTION_TWO_LINES: i64 = 1 << 3; // 2-line display mode
const FLG_CGRAM_MASK: i64 = 0b00111111; // CGRAM address mask
const FLG_DDRAM_MASK: i64 = 0b01111111; // DDRAM address mask

const DISPLAY_INIT_CMNDS: [i64; 5] = [
    // CMND.CLR - no need as framebuffer will rewrite all
    CMND_HOME,                                // move cursor to home (0x00)
    CMND_ENTERY_MODE | FLG_ENTERY_MODE_INC,   // increment cursor and no shift
    CMND_DISPLAY | FLG_DISPLAY_ON,            // keep cursor and blinking off
    CMND_SHIFT | FLG_SHIFT_RIGHT,             // shift right cursor only
    CMND_FUNCTION | FLG_FUNCTION_TWO_LINES,   // 2-line display mode, 5x8 dots
];

pub struct Aip31068Spi {
    pub printer: Rc<Printer>,
    pub spi: Rc<McuSpi>,
    pub mcu: Rc<Mcu>,
    pub fb: TextFramebuffers,
}

pub fn new_aip31068_spi(config: &ConfigWrapper) -> KResult<Rc<dyn LcdChip>> {
    let printer = config.get_printer();
    // spi config
    let spi = mcu_spi_from_config(config, 0x00, "latch_pin", 100000, None, false)?; // (config, mode, cs_name)
    let mcu = spi.get_mcu();
    let line_length = config.getchoice_int("line_length", &LINE_LENGTH_OPTIONS, LINE_LENGTH_DEFAULT)?;
    // each controller's line is 2 lines on the display and hence twice
    // line length
    let fb = TextFramebuffers::new(
        line_length,
        [CMND_DDRAM | (FLG_DDRAM_MASK & 0x00), CMND_DDRAM | (FLG_DDRAM_MASK & 0x40), CMND_CGRAM | (FLG_CGRAM_MASK & 0x00)],
    );
    Ok(Rc::new(Aip31068Spi { printer, spi, mcu, fb }))
}

/// encode(data, width=9): pack the values msb first into bytes
pub fn encode(data: &[i64], width: u32) -> KResult<Vec<u8>> {
    let mut encoded_bytes = vec![];
    let mut accumulator: u64 = 0; // To accumulate bits
    let mut acc_bits: u32 = 0; // Count of bits in the accumulator
    for &num in data {
        // check that num will fit in width bits
        if num >= (1 << width) {
            return Err(KError::Other(format!("Number {} does not fit in {} bits", num, width)));
        }
        // Shift the current number into the accumulator from the right
        accumulator = (accumulator << width) | num as u64;
        acc_bits += width; // Update the count of bits in the accumulator
        // While we have at least 8 bits, form a byte and append it
        while acc_bits >= 8 {
            acc_bits -= 8; // Decrease bit count by 8
            // Extract the 8 most significant bits to form a byte
            let byte = (accumulator >> acc_bits) & 0xFF;
            // Remove msb 8 bits from the accumulator
            accumulator &= (1 << acc_bits) - 1;
            encoded_bytes.push(byte as u8);
        }
    }
    // Handle any remaining bits by padding them on the right to byte
    if acc_bits > 0 {
        encoded_bytes.push((accumulator << (8 - acc_bits)) as u8);
    }
    Ok(encoded_bytes)
}

impl Aip31068Spi {
    pub fn send(&self, data: &[i64], minclock: u64) -> KResult<()> {
        // different commands have different processing time
        // to avoid timing violation pad with some fast command, e.g. ENTRY_MODE
        // that has execution time of 39us (for comparison CLR is 1.53ms)
        let pad = CMND_ENTERY_MODE | FLG_ENTERY_MODE_INC;
        for group in data.chunks(8) {
            // Pad the group if it has fewer than 8 elements
            let mut group = group.to_vec();
            group.resize(8, pad);
            self.spi.spi_send(&encode(&group, 9)?, minclock, 0)?;
        }
        Ok(())
    }
}

impl LcdChip for Aip31068Spi {
    fn flush(&self) -> KResult<()> {
        self.fb.flush(|cmd, data| {
            self.send(&[cmd], 0)?;
            let data: Vec<i64> = data.iter().map(|b| CMND_WRITE_RAM | *b as i64).collect();
            self.send(&data, 0)
        })
    }

    fn init(&self) -> KResult<()> {
        let curtime = self.printer.get_reactor().monotonic();
        let print_time = self.mcu.estimated_print_time(curtime);
        for (i, cmds) in DISPLAY_INIT_CMNDS.iter().enumerate() {
            let minclock = self.mcu.print_time_to_clock(print_time + i as f64 * 0.100) as u64;
            self.send(&[*cmds], minclock)?;
        }
        self.flush()
    }

    fn write_text(&self, x: i64, y: i64, data: &[u8]) {
        self.fb.write_text(x, y, data)
    }

    fn set_glyphs(&self, glyphs: &IndexMap<String, Glyph>) {
        self.fb.set_glyphs(glyphs)
    }

    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> i64 {
        self.fb.write_glyph(x, y, glyph_name)
    }

    fn write_graphics(&self, _x: i64, _y: i64, _data: &[u8]) {
        // this display supports only hardcoded or 8 user defined glyphs
    }

    fn clear(&self) {
        self.fb.clear()
    }

    fn get_dimensions(&self) -> (i64, i64) {
        (self.fb.line_length, 4)
    }

    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }
}
