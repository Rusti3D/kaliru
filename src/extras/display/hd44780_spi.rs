// Support for HD44780 (20x4 text) LCD displays
// (port of klippy/extras/display/hd44780_spi.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2021  Marc-Andre Denis <marcadenis@msn.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::hd44780::{TextFramebuffers, LINE_LENGTH_DEFAULT, LINE_LENGTH_OPTIONS};
use super::LcdChip;
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::extras::display_templates::Glyph;
use crate::mcu::Mcu;
use crate::printer::Printer;
use indexmap::IndexMap;
use std::rc::Rc;

pub struct HD44780Spi {
    printer: Rc<Printer>,
    hd44780_protocol_init: bool,
    spi: Rc<McuSpi>,
    mcu: Rc<Mcu>,
    data_mask: u8,
    command_mask: u8,
    enable_mask: u8,
    fb: TextFramebuffers,
}

pub fn new_hd44780_spi(config: &ConfigWrapper) -> KResult<Rc<dyn LcdChip>> {
    let printer = config.get_printer();
    let hd44780_protocol_init = config.getboolean("hd44780_protocol_init", true)?;
    // spi config
    let spi = mcu_spi_from_config(config, 0x00, "latch_pin", 100000, None, false)?;
    let mcu = spi.get_mcu();
    let line_length = config.getchoice_int("line_length", &LINE_LENGTH_OPTIONS, LINE_LENGTH_DEFAULT)?;
    Ok(Rc::new(HD44780Spi {
        printer,
        hd44780_protocol_init,
        spi,
        mcu,
        data_mask: 1 << 1,
        command_mask: 0,
        enable_mask: 1 << 3,
        fb: TextFramebuffers::new(line_length, [0x80, 0xC0, 0x40]),
    }))
}

impl HD44780Spi {
    fn send_4_bits(&self, cmd: i64, is_data: bool, minclock: u64) -> KResult<()> {
        let mask = if is_data { self.data_mask } else { self.command_mask };
        let c = (cmd & 0xF0) as u8;
        self.spi.spi_send(&[c | mask], minclock, 0)?;
        self.spi.spi_send(&[c | mask | self.enable_mask], minclock, 0)?;
        self.spi.spi_send(&[c | mask], minclock, 0)
    }

    fn send(&self, cmds: &[u8], is_data: bool, minclock: u64) -> KResult<()> {
        for &data in cmds {
            self.send_4_bits(data as i64, is_data, minclock)?;
            self.send_4_bits((data as i64) << 4, is_data, minclock)?;
        }
        Ok(())
    }
}

impl LcdChip for HD44780Spi {
    fn flush(&self) -> KResult<()> {
        self.fb.flush(|cmd, data| {
            self.send(&[cmd as u8], false, 0)?;
            self.send(data, true, 0)
        })
    }

    fn init(&self) -> KResult<()> {
        let curtime = self.printer.get_reactor().monotonic();
        let print_time = self.mcu.estimated_print_time(curtime);
        // Program 4bit / 2-line mode and then issue 0x02 "Home" command
        let mut init: Vec<Vec<u8>> =
            if self.hd44780_protocol_init { vec![vec![0x33], vec![0x33], vec![0x32], vec![0x28, 0x28, 0x02]] } else { vec![vec![0x02]] };
        // Reset (set positive direction ; enable display and hide cursor)
        init.push(vec![0x06, 0x0C]);
        for (i, cmds) in init.iter().enumerate() {
            let minclock = self.mcu.print_time_to_clock(print_time + i as f64 * 0.100) as u64;
            self.send(cmds, false, minclock)?;
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

    fn write_graphics(&self, _x: i64, _y: i64, _data: &[u8]) {}

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
