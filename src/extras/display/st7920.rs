// Support for ST7920 (128x64 graphics) LCD displays
// (port of klippy/extras/display/st7920.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::font8x14::VGA_FONT;
use super::{framebuffer_diffs, set_slice, LcdChip};
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::{pin_mcu, same_rc, McuBusDigitalOut, McuSpi};
use crate::extras::display_templates::Glyph;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::pins::PinsObject;
use crate::printer::Printer;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const BACKGROUND_PRIORITY_CLOCK: u64 = 0x7FFFFFFF00000000;

// Spec says 72us, but faster is possible in practice
const ST7920_CMD_DELAY: f64 = 0.000020;
const ST7920_SYNC_DELAY: f64 = 0.000045;

fn text_glyph(name: &str) -> Option<&'static [u8]> {
    match name {
        "right_arrow" => Some(b"\x1a"),
        _ => None,
    }
}

fn char_glyph(name: &str) -> Option<&'static [u8; 16]> {
    match name {
        "degrees" => Some(&VGA_FONT[0xF8]),
        _ => None,
    }
}

/// how a DisplayBase sends: ST7920 (mcu commands) or EmulatedST7920 (spi)
enum Io {
    Native { mcu: Rc<Mcu>, oid: i64, pins: [String; 3], cmds: RefCell<Option<(CommandWrapper, CommandWrapper)>> },
    Emulated { spi: Rc<McuSpi>, en_helper: EnableHelper, en_set: Cell<bool> },
}

/// a framebuffer: (new data, data on the display, id)
type Framebuffer = (Vec<u8>, Vec<u8>, i64);

pub struct DisplayBase {
    io: Io,
    mcu: Rc<Mcu>,
    is_extended: Cell<bool>,
    /// all_framebuffers: text (0x80), glyph (0x40), then the 32 graphics
    /// framebuffers (0..31)
    fbs: RefCell<Vec<Framebuffer>>,
    cached_glyphs: RefCell<IndexMap<String, (String, Vec<u8>)>>,
    icons: RefCell<IndexMap<String, (Vec<i64>, Vec<i64>)>>,
}

const TEXT_FB: usize = 0;
const GLYPH_FB: usize = 1;
const GFX_FB: usize = 2;

impl DisplayBase {
    fn new(io: Io, mcu: Rc<Mcu>) -> Self {
        // framebuffers
        let mut fbs: Vec<Framebuffer> = vec![
            // Text framebuffer
            (vec![b' '; 64], vec![b'~'; 64], 0x80),
            // Glyph framebuffer
            (vec![0; 128], vec![b'~'; 128], 0x40),
        ];
        // Graphics framebuffers
        for i in 0..32 {
            fbs.push((vec![0; 32], vec![b'~'; 32], i));
        }
        DisplayBase {
            io,
            mcu,
            is_extended: Cell::new(false),
            fbs: RefCell::new(fbs),
            cached_glyphs: RefCell::new(IndexMap::default()),
            icons: RefCell::new(IndexMap::default()),
        }
    }

    fn send(&self, cmds: &[u8], is_data: bool, is_extended: bool) -> KResult<()> {
        match &self.io {
            Io::Native { oid, cmds: lookups, .. } => {
                let lookups = lookups.borrow();
                let (send_cmds_cmd, send_data_cmd) = lookups.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
                let mut cmds = cmds.to_vec();
                let cmd_type = if is_data {
                    send_data_cmd
                } else {
                    if self.is_extended.get() != is_extended {
                        let add_cmd = if is_extended { 0x26 } else { 0x22 };
                        cmds.insert(0, add_cmd);
                        self.is_extended.set(is_extended);
                    }
                    send_cmds_cmd
                };
                cmd_type.send(&[Val::Int(*oid), Val::Bytes(cmds)], 0, BACKGROUND_PRIORITY_CLOCK)
            }
            Io::Emulated { spi, en_helper, en_set } => {
                // setup sync byte and check for exten mode switch
                let mut cmds = cmds.to_vec();
                let mut sync_byte = 0xFA;
                if !is_data {
                    sync_byte = 0xF8;
                    if self.is_extended.get() != is_extended {
                        let add_cmd = if is_extended { 0x26 } else { 0x22 };
                        cmds.insert(0, add_cmd);
                        self.is_extended.set(is_extended);
                    }
                }
                // copy data to ST7920 data format
                let mut spi_data = vec![0u8; 2 * cmds.len() + 1];
                spi_data[0] = sync_byte;
                for (i, b) in cmds.iter().enumerate() {
                    spi_data[1 + 2 * i] = b & 0xF0;
                    spi_data[2 + 2 * i] = (b & 0x0F) << 4;
                }
                // check if enable pin has been set
                if !en_set.get() {
                    en_helper.init()?;
                    en_set.set(true);
                }
                // send data
                spi.spi_send(&spi_data, 0, BACKGROUND_PRIORITY_CLOCK)
            }
        }
    }

    fn cache_glyph(&self, glyph_name: &str, base_glyph_name: &str, glyph_id: usize) {
        let icons = self.icons.borrow();
        let (Some(icon), Some(base_icon)) = (icons.get(glyph_name), icons.get(base_glyph_name)) else { return };
        let mut fbs = self.fbs.borrow_mut();
        let n = icon.0.len().min(icon.1.len()).min(base_icon.0.len()).min(base_icon.1.len());
        for i in 0..n {
            let (x1, x2) = ((icon.0[i] ^ base_icon.0[i]) as u8, (icon.1[i] ^ base_icon.1[i]) as u8);
            let pos = glyph_id * 32 + i * 2;
            set_slice(&mut fbs[GLYPH_FB].0, pos, &[x1, x2]);
            set_slice(&mut fbs[GLYPH_FB].1, pos, &[x1 ^ 1, x2 ^ 1]);
        }
        self.cached_glyphs.borrow_mut().insert(glyph_name.to_string(), (base_glyph_name.to_string(), vec![0, (glyph_id * 2) as u8]));
    }
}

impl LcdChip for DisplayBase {
    fn flush(&self) -> KResult<()> {
        // Find all differences in the framebuffers and send them to the chip
        let n = self.fbs.borrow().len();
        for idx in 0..n {
            let (new_data, fb_id) = {
                let fbs = self.fbs.borrow();
                let (new_data, old_data, fb_id) = &fbs[idx];
                if new_data == old_data {
                    continue;
                }
                (new_data.clone(), *fb_id)
            };
            let old_data = self.fbs.borrow()[idx].1.clone();
            // Transmit changes
            for (pos, count) in framebuffer_diffs(&new_data, &old_data, 5) {
                let mut count = count + (pos & 0x01);
                count += count & 0x01;
                let pos = pos & !0x01;
                let chip_pos = (pos >> 1) as i64;
                if fb_id < 0x40 {
                    // Graphics framebuffer update
                    self.send(&[(0x80 + fb_id) as u8, (0x80 + chip_pos) as u8], false, true)?;
                } else {
                    self.send(&[(fb_id + chip_pos) as u8], false, false)?;
                }
                let end = (pos + count).min(new_data.len());
                self.send(&new_data[pos..end], true, false)?;
            }
            self.fbs.borrow_mut()[idx].1 = new_data;
        }
        Ok(())
    }

    fn init(&self) -> KResult<()> {
        let cmds = [
            0x24, // Enter extended mode
            0x40, // Clear vertical scroll address
            0x02, // Enable CGRAM access
            0x26, // Enable graphics
            0x22, // Leave extended mode
            0x02, // Home the display
            0x06, // Set positive update direction
            0x0C, // Enable display and hide cursor
        ];
        self.send(&cmds, false, false)?;
        self.flush()
    }

    fn set_glyphs(&self, glyphs: &IndexMap<String, Glyph>) {
        for (glyph_name, glyph_data) in glyphs {
            if let Some(icon) = &glyph_data.icon16x16 {
                self.icons.borrow_mut().insert(glyph_name.clone(), icon.clone());
            }
        }
        // Setup animated glyphs
        self.cache_glyph("fan2", "fan1", 0);
        self.cache_glyph("bed_heat2", "bed_heat1", 1);
    }

    fn write_text(&self, x: i64, y: i64, data: &[u8]) {
        let mut data = data;
        if x + data.len() as i64 > 16 {
            data = &data[..(16 - x.min(16)).max(0) as usize];
        }
        let Some(row) = [0, 32, 16, 48].get(y as usize) else { return };
        let pos = row + x;
        set_slice(&mut self.fbs.borrow_mut()[TEXT_FB].0, pos as usize, data);
    }

    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) {
        if x >= 16 || y >= 4 || data.len() != 16 {
            return;
        }
        let mut x = x;
        let mut gfx_fb = y * 16;
        if gfx_fb >= 32 {
            gfx_fb -= 32;
            x += 16;
        }
        let mut fbs = self.fbs.borrow_mut();
        for (i, bits) in data.iter().enumerate() {
            fbs[GFX_FB + gfx_fb as usize + i].0[x as usize] = *bits;
        }
    }

    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> i64 {
        let mut glyph_name = glyph_name.to_string();
        let cached = self.cached_glyphs.borrow().get(&glyph_name).cloned();
        if let Some((base, text)) = cached {
            if x & 1 == 0 {
                // Render cached icon using character generator
                glyph_name = base;
                self.write_text(x, y, &text);
            }
        }
        let icon = self.icons.borrow().get(&glyph_name).cloned();
        if let Some(icon) = icon {
            // Draw icon in graphics mode
            let b = |v: &Vec<i64>| v.iter().map(|x| *x as u8).collect::<Vec<u8>>();
            self.write_graphics(x, y, &b(&icon.0));
            self.write_graphics(x + 1, y, &b(&icon.1));
            return 2;
        }
        if let Some(c) = text_glyph(&glyph_name) {
            // Draw character
            self.write_text(x, y, c);
            return 1;
        }
        if let Some(font) = char_glyph(&glyph_name) {
            // Draw single width character
            self.write_graphics(x, y, font);
            return 1;
        }
        0
    }

    fn clear(&self) {
        let mut fbs = self.fbs.borrow_mut();
        fbs[TEXT_FB].0.fill(b' ');
        for fb in fbs.iter_mut().skip(GFX_FB) {
            fb.0.fill(0);
        }
    }

    fn get_dimensions(&self) -> (i64, i64) {
        (16, 4)
    }

    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }
}

// Display driver for stock ST7920 displays
pub fn new_st7920(config: &ConfigWrapper) -> KResult<Rc<dyn LcdChip>> {
    let printer = config.get_printer();
    // pin config
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let mut mcu: Option<Rc<Mcu>> = None;
    let mut pins: Vec<String> = vec![];
    for name in ["cs", "sclk", "sid"] {
        let desc = config.get(&format!("{}_pin", name), Dflt::Required)?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(&desc, false, false, None).map_err(|e| KError::Pins(e.0))?;
        if let Some(m) = &mcu {
            if !same_rc(&pin_params.borrow().chip, m) {
                return Err(KError::Pins("st7920 all pins must be on same mcu".into()));
            }
        }
        mcu = Some(pin_mcu(config, &pin_params)?);
        pins.push(pin_params.borrow().pin.clone());
    }
    let mcu = mcu.expect("st7920 mcu");
    // prepare send functions
    let oid = mcu.create_oid();
    let d = Rc::new(DisplayBase::new(
        Io::Native { mcu: mcu.clone(), oid, pins: [pins[0].clone(), pins[1].clone(), pins[2].clone()], cmds: RefCell::new(None) },
        mcu.clone(),
    ));
    let w = Rc::downgrade(&d);
    mcu.register_config_callback(Rc::new(move || match w.upgrade() {
        Some(d) => build_config(&d),
        None => Ok(()),
    }));
    Ok(d)
}

fn build_config(d: &DisplayBase) -> KResult<()> {
    let Io::Native { mcu, oid, pins, cmds } = &d.io else { return Ok(()) };
    // configure send functions
    mcu.add_config_cmd(
        &format!(
            "config_st7920 oid={} cs_pin={} sclk_pin={} sid_pin={} sync_delay_ticks={} cmd_delay_ticks={}",
            oid,
            pins[0],
            pins[1],
            pins[2],
            mcu.seconds_to_clock(ST7920_SYNC_DELAY),
            mcu.seconds_to_clock(ST7920_CMD_DELAY)
        ),
        false,
        false,
    );
    let cmd_queue = mcu.alloc_command_queue();
    let send_cmds_cmd = mcu.lookup_command("st7920_send_cmds oid=%c cmds=%*s", Some(cmd_queue.clone()))?;
    let send_data_cmd = mcu.lookup_command("st7920_send_data oid=%c data=%*s", Some(cmd_queue))?;
    *cmds.borrow_mut() = Some((send_cmds_cmd, send_data_cmd));
    Ok(())
}

// Helper code for toggling the en pin on startup
pub struct EnableHelper {
    printer: Rc<Printer>,
    en_pin: Rc<McuBusDigitalOut>,
}

impl EnableHelper {
    pub fn new(printer: &Rc<Printer>, pin_desc: &str, spi: &McuSpi) -> KResult<Self> {
        let en_pin = McuBusDigitalOut::new(&spi.get_mcu(), pin_desc, Some(spi.get_command_queue()), 0)?;
        Ok(EnableHelper { printer: printer.clone(), en_pin })
    }

    pub fn init(&self) -> KResult<()> {
        let mcu = self.en_pin.get_mcu();
        let curtime = self.printer.get_reactor().monotonic();
        let print_time = mcu.estimated_print_time(curtime);
        // Toggle enable pin
        let minclock = mcu.print_time_to_clock(print_time + 0.100) as u64;
        self.en_pin.update_digital_out(0, minclock, 0)?;
        let minclock = mcu.print_time_to_clock(print_time + 0.200) as u64;
        self.en_pin.update_digital_out(1, minclock, 0)?;
        // Force a delay to any subsequent commands on the command queue
        let minclock = mcu.print_time_to_clock(print_time + 0.300) as u64;
        self.en_pin.update_digital_out(1, minclock, 0)
    }
}

// Display driver for displays that emulate the ST7920 in software.
// These displays rely on the CS pin to be toggled in order to initialize the
// SPI correctly. This display driver uses a software SPI with an unused pin
// as the MISO pin.
pub fn new_emulated_st7920(config: &ConfigWrapper) -> KResult<Rc<dyn LcdChip>> {
    let printer = config.get_printer();
    // create software spi
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let mut mcu: Option<Rc<Mcu>> = None;
    let mut sw_pins: Vec<String> = vec![];
    for name in ["miso", "mosi", "sclk"] {
        let option = format!("spi_software_{}_pin", name);
        let desc = config.get(&option, Dflt::Required)?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(&desc, false, false, Some(&option)).map_err(|e| KError::Pins(e.0))?;
        if let Some(m) = &mcu {
            if !same_rc(&pin_params.borrow().chip, m) {
                return Err(KError::Pins(format!("{}: spi pins must be on same mcu", config.get_name())));
            }
        }
        mcu = Some(pin_mcu(config, &pin_params)?);
        sw_pins.push(pin_params.borrow().pin.clone());
    }
    let mcu = mcu.expect("st7920 mcu");
    let speed = config.getint("spi_speed", 1000000, Lim::new().min(100000))?;
    let spi = McuSpi::new(&mcu, None, None, 0, speed, Some([sw_pins[0].clone(), sw_pins[1].clone(), sw_pins[2].clone()]), false);
    // create enable helper
    let en_helper = EnableHelper::new(&printer, &config.get("en_pin", Dflt::Required)?, &spi)?;
    // init display base
    Ok(Rc::new(DisplayBase::new(Io::Emulated { spi, en_helper, en_set: Cell::new(false) }, mcu)))
}
