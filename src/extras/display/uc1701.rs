// Support for UC1701 (and similar) 128x64 graphics LCD displays
// (port of klippy/extras/display/uc1701.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2018  Eric Callahan  <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::font8x14::VGA_FONT;
use super::st7920::BACKGROUND_PRIORITY_CLOCK;
use super::{framebuffer_diffs, set_slice, LcdChip};
use crate::chelper::serialqueue::CommandQueue;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::KResult;
use crate::extras::bus::{mcu_i2c_from_config, mcu_spi_from_config, McuBusDigitalOut, McuI2c, McuSpi};
use crate::extras::display_templates::Glyph;
use crate::mcu::Mcu;
use crate::printer::Printer;
use crate::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

fn text_glyph(name: &str) -> Option<&'static [u8]> {
    match name {
        "right_arrow" => Some(b"\x1a"),
        "degrees" => Some(b"\xf8"),
        _ => None,
    }
}

// Convert from "rows of pixels" format to "columns of pixels"
fn swizzle_bits(data: &[u8]) -> (Vec<u8>, Vec<u8>) {
    let (mut top, mut bot): (u128, u128) = (0, 0);
    for row in 0..8 {
        let spaced = (data[row] as u128 * 0x8040201008040201) & 0x8080808080808080;
        top |= spaced >> (7 - row);
        let spaced = (data[row + 8] as u128 * 0x8040201008040201) & 0x8080808080808080;
        bot |= spaced >> (7 - row);
    }
    let bits_top = (0..64).step_by(8).map(|s| ((top >> s) & 0xFF) as u8).collect();
    let bits_bot = (0..64).step_by(8).map(|s| ((bot >> s) & 0xFF) as u8).collect();
    (bits_top, bits_bot)
}

/// the io a DisplayBase sends through
enum Io {
    // IO wrapper for "4 wire" spi bus (spi bus with an extra data/control line)
    Spi4Wire { spi: Rc<McuSpi>, mcu_dc: Rc<McuBusDigitalOut> },
    // IO wrapper for i2c bus
    I2c { i2c: Rc<McuI2c> },
}

impl Io {
    fn spi4wire(config: &ConfigWrapper, data_pin_name: &str) -> KResult<Self> {
        let spi = mcu_spi_from_config(config, 0, "cs_pin", 10000000, None, false)?;
        let dc_pin = config.get(data_pin_name, Dflt::Required)?;
        let mcu_dc = McuBusDigitalOut::new(&spi.get_mcu(), &dc_pin, Some(spi.get_command_queue()), 0)?;
        Ok(Io::Spi4Wire { spi, mcu_dc })
    }

    fn i2c(config: &ConfigWrapper, default_addr: i64) -> KResult<Self> {
        Ok(Io::I2c { i2c: mcu_i2c_from_config(config, Some(default_addr), 400000)? })
    }

    fn send(&self, cmds: &[u8], is_data: bool) -> KResult<()> {
        match self {
            Io::Spi4Wire { spi, mcu_dc } => {
                mcu_dc.update_digital_out(is_data as i64, 0, BACKGROUND_PRIORITY_CLOCK)?;
                spi.spi_send(cmds, 0, BACKGROUND_PRIORITY_CLOCK)
            }
            Io::I2c { i2c } => {
                let hdr = if is_data { 0x40 } else { 0x00 };
                let mut c = vec![hdr];
                c.extend_from_slice(cmds);
                i2c.i2c_write_noack(&c, 0, BACKGROUND_PRIORITY_CLOCK)
            }
        }
    }

    /// (get_mcu(), get_command_queue()) of the bus
    fn bus(&self) -> (Rc<Mcu>, CommandQueue) {
        match self {
            Io::Spi4Wire { spi, .. } => (spi.get_mcu(), spi.get_command_queue()),
            Io::I2c { i2c } => (i2c.get_mcu(), i2c.get_command_queue()),
        }
    }
}

// Helper code for toggling a reset pin on startup
struct ResetHelper {
    printer: Rc<Printer>,
    mcu_reset: Option<Rc<McuBusDigitalOut>>,
}

impl ResetHelper {
    fn new(printer: &Rc<Printer>, pin_desc: Option<String>, io: &Io) -> KResult<Self> {
        let mcu_reset = match pin_desc {
            None => None,
            Some(pin) => {
                let (mcu, cq) = io.bus();
                Some(McuBusDigitalOut::new(&mcu, &pin, Some(cq), 0)?)
            }
        };
        Ok(ResetHelper { printer: printer.clone(), mcu_reset })
    }

    fn init(&self) -> KResult<()> {
        let Some(mcu_reset) = &self.mcu_reset else { return Ok(()) };
        let mcu = mcu_reset.get_mcu();
        let curtime = self.printer.get_reactor().monotonic();
        let print_time = mcu.estimated_print_time(curtime);
        // Toggle reset
        let minclock = mcu.print_time_to_clock(print_time + 0.100) as u64;
        mcu_reset.update_digital_out(0, minclock, 0)?;
        let minclock = mcu.print_time_to_clock(print_time + 0.200) as u64;
        mcu_reset.update_digital_out(1, minclock, 0)?;
        // Force a delay to any subsequent commands on the command queue
        let minclock = mcu.print_time_to_clock(print_time + 0.300) as u64;
        mcu_reset.update_digital_out(1, minclock, 0)
    }
}

enum Chip {
    UC1701 { contrast: i64 },
    SSD1306 { contrast: i64, vcomh: i64, invert: bool },
}

pub struct DisplayBase {
    io: Io,
    mcu: Rc<Mcu>,
    x_offset: i64,
    /// all_framebuffers: the 8 vram pages and the data on the display
    vram: RefCell<Vec<(Vec<u8>, Vec<u8>)>>,
    // Cache fonts and icons in display byte order
    font: Vec<(Vec<u8>, Vec<u8>)>,
    icons: RefCell<IndexMap<String, (Vec<u8>, Vec<u8>)>>,
    reset: ResetHelper,
    chip: Chip,
}

impl DisplayBase {
    fn new(io: Io, columns: usize, x_offset: i64, reset: ResetHelper, chip: Chip) -> Self {
        let mcu = io.bus().0;
        DisplayBase {
            io,
            mcu,
            x_offset,
            vram: RefCell::new((0..8).map(|_| (vec![0; columns], vec![b'~'; columns])).collect()),
            font: VGA_FONT.iter().map(|c| swizzle_bits(c)).collect(),
            icons: RefCell::new(IndexMap::default()),
            reset,
            chip,
        }
    }

    fn send(&self, cmds: &[u8], is_data: bool) -> KResult<()> {
        self.io.send(cmds, is_data)
    }
}

impl LcdChip for DisplayBase {
    fn flush(&self) -> KResult<()> {
        // Find all differences in the framebuffers and send them to the chip
        for page in 0..8 {
            let (new_data, old_data) = self.vram.borrow()[page].clone();
            if new_data == old_data {
                continue;
            }
            // Transmit changes
            for (col_pos, count) in framebuffer_diffs(&new_data, &old_data, 5) {
                // Set Position registers
                let ra = 0xB0 | (page as u8 & 0x0F);
                let ca_msb = 0x10 | ((col_pos >> 4) as u8 & 0x0F);
                let ca_lsb = col_pos as u8 & 0x0F;
                self.send(&[ra, ca_msb, ca_lsb], false)?;
                // Send Data
                let end = (col_pos + count).min(new_data.len());
                self.send(&new_data[col_pos..end], true)?;
            }
            self.vram.borrow_mut()[page].1 = new_data;
        }
        Ok(())
    }

    fn init(&self) -> KResult<()> {
        self.reset.init()?;
        match self.chip {
            Chip::UC1701 { contrast } => {
                let init_cmds = [
                    0xE2,           // System reset
                    0x40,           // Set display to start at line 0
                    0xA0,           // Set SEG direction
                    0xC8,           // Set COM Direction
                    0xA2,           // Set Bias = 1/9
                    0x2C,           // Boost ON
                    0x2E,           // Voltage regulator on
                    0x2F,           // Voltage follower on
                    0xF8,           // Set booster ratio
                    0x00,           // Booster ratio value (4x)
                    0x23,           // Set resistor ratio (3)
                    0x81,           // Set Electronic Volume
                    contrast as u8, // Electronic Volume value
                    0xAC,           // Set static indicator off
                    0x00,           // NOP
                    0xA6,           // Disable Inverse
                    0xAF,           // Set display enable
                ];
                self.send(&init_cmds, false)?;
                self.send(&[0xA5], false)?; // display all
                self.send(&[0xA4], false)?; // normal display
            }
            Chip::SSD1306 { contrast, vcomh, invert } => {
                let init_cmds = [
                    0xAE, // Display off
                    0xD5,
                    0x80, // Set oscillator frequency
                    0xA8,
                    0x3F, // Set multiplex ratio
                    0xD3,
                    0x00, // Set display offset
                    0x40, // Set display start line
                    0x8D,
                    0x14, // Charge pump setting
                    0x20,
                    0x02, // Set Memory addressing mode
                    0xA1, // Set Segment re-map
                    0xC8, // Set COM output scan direction
                    0xDA,
                    0x12, // Set COM pins hardware configuration
                    0x81,
                    contrast as u8, // Set contrast control
                    0xD9,
                    0xA1, // Set pre-charge period
                    0xDB,
                    vcomh as u8,                       // Set VCOMH deselect level
                    0x2E,                              // Deactivate scroll
                    0xA4,                              // Output ram to display
                    if invert { 0xA7 } else { 0xA6 }, // Set normal/invert
                    0xAF,                              // Display on
                ];
                self.send(&init_cmds, false)?;
            }
        }
        self.flush()
    }

    fn set_glyphs(&self, glyphs: &IndexMap<String, Glyph>) {
        for (glyph_name, glyph_data) in glyphs {
            if let Some(icon) = &glyph_data.icon16x16 {
                let b = |v: &Vec<i64>| v.iter().map(|x| *x as u8).collect::<Vec<u8>>();
                let (top1, bot1) = swizzle_bits(&b(&icon.0));
                let (top2, bot2) = swizzle_bits(&b(&icon.1));
                self.icons.borrow_mut().insert(glyph_name.clone(), ([top1, top2].concat(), [bot1, bot2].concat()));
            }
        }
    }

    fn write_text(&self, x: i64, y: i64, data: &[u8]) {
        let mut data = data;
        if x + data.len() as i64 > 16 {
            data = &data[..(16 - x.min(16)).max(0) as usize];
        }
        let mut pix_x = (x * 8 + self.x_offset) as usize;
        let mut vram = self.vram.borrow_mut();
        for &c in data {
            let (bits_top, bits_bot) = &self.font[c as usize];
            set_slice(&mut vram[(y * 2) as usize].0, pix_x, bits_top);
            set_slice(&mut vram[(y * 2 + 1) as usize].0, pix_x, bits_bot);
            pix_x += 8;
        }
    }

    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) {
        if x >= 16 || y >= 4 || data.len() != 16 {
            return;
        }
        let (bits_top, bits_bot) = swizzle_bits(data);
        let pix_x = (x * 8 + self.x_offset) as usize;
        let mut vram = self.vram.borrow_mut();
        for i in 0..8 {
            vram[(y * 2) as usize].0[pix_x + i] ^= bits_top[i];
            vram[(y * 2 + 1) as usize].0[pix_x + i] ^= bits_bot[i];
        }
    }

    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> i64 {
        let icon = self.icons.borrow().get(glyph_name).cloned();
        if let Some(icon) = icon {
            if x < 15 {
                // Draw icon in graphics mode
                let pix_x = (x * 8 + self.x_offset) as usize;
                let page_idx = (y * 2) as usize;
                let mut vram = self.vram.borrow_mut();
                set_slice(&mut vram[page_idx].0, pix_x, &icon.0);
                set_slice(&mut vram[page_idx + 1].0, pix_x, &icon.1);
                return 2;
            }
        }
        if let Some(c) = text_glyph(glyph_name) {
            // Draw character
            self.write_text(x, y, c);
            return 1;
        }
        0
    }

    fn clear(&self) {
        for page in self.vram.borrow_mut().iter_mut() {
            page.0.fill(0);
        }
    }

    fn get_dimensions(&self) -> (i64, i64) {
        (16, 4)
    }

    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }
}

// The UC1701 is a "4-wire" SPI display device
pub fn new_uc1701(config: &ConfigWrapper) -> KResult<Rc<dyn LcdChip>> {
    let printer = config.get_printer();
    let io = Io::spi4wire(config, "a0_pin")?;
    let contrast = config.getint("contrast", 40, Lim::new().min(0).max(63))?;
    let reset = ResetHelper::new(&printer, config.get_opt("rst_pin")?, &io)?;
    Ok(Rc::new(DisplayBase::new(io, 128, 0, reset, Chip::UC1701 { contrast })))
}

// The SSD1306 supports both i2c and "4-wire" spi
fn new_ssd1306_with(config: &ConfigWrapper, columns: usize, x_offset: i64) -> KResult<Rc<dyn LcdChip>> {
    let printer = config.get_printer();
    let io = match config.get_opt("cs_pin")? {
        None => Io::i2c(config, 60)?,
        Some(_) => Io::spi4wire(config, "dc_pin")?,
    };
    let reset = ResetHelper::new(&printer, config.get_opt("reset_pin")?, &io)?;
    let contrast = config.getint("contrast", 239, Lim::new().min(0).max(255))?;
    let vcomh = config.getint("vcomh", 0, Lim::new().min(0).max(63))?;
    let invert = config.getboolean("invert", false)?;
    Ok(Rc::new(DisplayBase::new(io, columns, x_offset, reset, Chip::SSD1306 { contrast, vcomh, invert })))
}

pub fn new_ssd1306(config: &ConfigWrapper) -> KResult<Rc<dyn LcdChip>> {
    new_ssd1306_with(config, 128, 0)
}

// the SH1106 is SSD1306 compatible with up to 132 columns
pub fn new_sh1106(config: &ConfigWrapper) -> KResult<Rc<dyn LcdChip>> {
    let x_offset = config.getint("x_offset", 0, Lim::new().min(0).max(3))?;
    new_ssd1306_with(config, 132, x_offset)
}
