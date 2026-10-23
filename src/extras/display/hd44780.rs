// Support for HD44780 (20x4 text) LCD displays
// (port of klippy/extras/display/hd44780.py)
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::st7920::BACKGROUND_PRIORITY_CLOCK;
use super::{framebuffer_diffs, set_slice, LcdChip};
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::extras::bus::{pin_mcu, same_rc};
use crate::extras::display_templates::Glyph;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::pins::PinsObject;
use crate::printer::Printer;
use indexmap::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

pub const LINE_LENGTH_DEFAULT: i64 = 20;
pub const LINE_LENGTH_OPTIONS: [(i64, i64); 2] = [(16, 16), (20, 20)];

const HD44780_DELAY: f64 = 0.000040;

/// the framebuffers and glyph handling the hd44780 style text displays
/// (hd44780, hd44780_spi, aip31068_spi) share
pub struct TextFramebuffers {
    pub line_length: i64,
    /// all_framebuffers: (new data, data on the display, command)
    pub fbs: RefCell<Vec<(Vec<u8>, Vec<u8>, i64)>>,
    icons: RefCell<IndexMap<String, (i64, Vec<i64>)>>,
}

impl TextFramebuffers {
    pub fn new(line_length: i64, cmds: [i64; 3]) -> Self {
        let n = (2 * line_length) as usize;
        TextFramebuffers {
            line_length,
            fbs: RefCell::new(vec![
                // Text framebuffers
                (vec![b' '; n], vec![b'~'; n], cmds[0]),
                (vec![b' '; n], vec![b'~'; n], cmds[1]),
                // Glyph framebuffer
                (vec![0; 64], vec![b'~'; 64], cmds[2]),
            ]),
            icons: RefCell::new(IndexMap::new()),
        }
    }

    /// flush(): send(fb_cmd + pos) then send(data) for every changed run
    pub fn flush(&self, mut send: impl FnMut(i64, &[u8]) -> KResult<()>) -> KResult<()> {
        // Find all differences in the framebuffers and send them to the chip
        for idx in 0..3 {
            let (new_data, old_data, fb_id) = self.fbs.borrow()[idx].clone();
            if new_data == old_data {
                continue;
            }
            // Transmit changes
            for (pos, count) in framebuffer_diffs(&new_data, &old_data, 4) {
                let end = (pos + count).min(new_data.len());
                send(fb_id + pos as i64, &new_data[pos..end])?;
            }
            self.fbs.borrow_mut()[idx].1 = new_data;
        }
        Ok(())
    }

    pub fn write_text(&self, x: i64, y: i64, data: &[u8]) {
        let mut data = data;
        if x + data.len() as i64 > self.line_length {
            data = &data[..(self.line_length - x.min(self.line_length)).max(0) as usize];
        }
        let pos = x + ((y & 0x02) >> 1) * self.line_length;
        set_slice(&mut self.fbs.borrow_mut()[(y & 1) as usize].0, pos as usize, data);
    }

    pub fn set_glyphs(&self, glyphs: &IndexMap<String, Glyph>) {
        for (glyph_name, glyph_data) in glyphs {
            if let Some(data) = &glyph_data.icon5x8 {
                self.icons.borrow_mut().insert(glyph_name.clone(), data.clone());
            }
        }
    }

    pub fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> i64 {
        let data = self.icons.borrow().get(glyph_name).cloned();
        if let Some((slot, bits)) = data {
            self.write_text(x, y, &[slot as u8]);
            let bits: Vec<u8> = bits.iter().map(|b| *b as u8).collect();
            set_slice(&mut self.fbs.borrow_mut()[2].0, (slot * 8) as usize, &bits);
            return 1;
        }
        if glyph_name == "right_arrow" {
            // Draw character
            self.write_text(x, y, b"\x7e");
            return 1;
        }
        0
    }

    pub fn clear(&self) {
        let mut fbs = self.fbs.borrow_mut();
        fbs[0].0.fill(b' ');
        fbs[1].0.fill(b' ');
    }
}

pub struct HD44780 {
    printer: Rc<Printer>,
    hd44780_protocol_init: bool,
    pins: Vec<String>,
    mcu: Rc<Mcu>,
    oid: i64,
    cmds: RefCell<Option<(CommandWrapper, CommandWrapper)>>,
    fb: TextFramebuffers,
}

pub fn new_hd44780(config: &ConfigWrapper) -> KResult<Rc<dyn LcdChip>> {
    let printer = config.get_printer();
    // pin config
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let mut pin_params = vec![];
    for name in ["rs", "e", "d4", "d5", "d6", "d7"] {
        let desc = config.get(&format!("{}_pin", name), Dflt::Required)?;
        pin_params.push(ppins.0.borrow_mut().lookup_pin(&desc, false, false, None).map_err(|e| KError::Pins(e.0))?);
    }
    let hd44780_protocol_init = config.getboolean("hd44780_protocol_init", true)?;
    let line_length = config.getchoice_int("line_length", &LINE_LENGTH_OPTIONS, LINE_LENGTH_DEFAULT)?;
    let mut mcu: Option<Rc<Mcu>> = None;
    for pp in &pin_params {
        if let Some(m) = &mcu {
            if !same_rc(&pp.borrow().chip, m) {
                return Err(KError::Pins("hd44780 all pins must be on same mcu".into()));
            }
        }
        mcu = Some(pin_mcu(config, pp)?);
    }
    let mcu = mcu.expect("hd44780 mcu");
    let pins = pin_params.iter().map(|pp| pp.borrow().pin.clone()).collect();
    let oid = mcu.create_oid();
    let d = Rc::new(HD44780 {
        printer,
        hd44780_protocol_init,
        pins,
        mcu: mcu.clone(),
        oid,
        cmds: RefCell::new(None),
        fb: TextFramebuffers::new(line_length, [0x80, 0xC0, 0x40]),
    });
    let w = Rc::downgrade(&d);
    mcu.register_config_callback(Rc::new(move || match w.upgrade() {
        Some(d) => d.build_config(),
        None => Ok(()),
    }));
    Ok(d)
}

impl HD44780 {
    fn build_config(&self) -> KResult<()> {
        let p = &self.pins;
        self.mcu.add_config_cmd(
            &format!(
                "config_hd44780 oid={} rs_pin={} e_pin={} d4_pin={} d5_pin={} d6_pin={} d7_pin={} delay_ticks={}",
                self.oid,
                p[0],
                p[1],
                p[2],
                p[3],
                p[4],
                p[5],
                self.mcu.seconds_to_clock(HD44780_DELAY)
            ),
            false,
            false,
        );
        let cmd_queue = self.mcu.alloc_command_queue();
        let send_cmds_cmd = self.mcu.lookup_command("hd44780_send_cmds oid=%c cmds=%*s", Some(cmd_queue.clone()))?;
        let send_data_cmd = self.mcu.lookup_command("hd44780_send_data oid=%c data=%*s", Some(cmd_queue))?;
        *self.cmds.borrow_mut() = Some((send_cmds_cmd, send_data_cmd));
        Ok(())
    }

    fn send(&self, cmds: &[u8], is_data: bool, minclock: u64, reqclock: u64) -> KResult<()> {
        let c = self.cmds.borrow();
        let (send_cmds_cmd, send_data_cmd) = c.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        let cmd_type = if is_data { send_data_cmd } else { send_cmds_cmd };
        cmd_type.send(&[Val::Int(self.oid), Val::Bytes(cmds.to_vec())], minclock, reqclock)
    }
}

impl LcdChip for HD44780 {
    fn flush(&self) -> KResult<()> {
        self.fb.flush(|cmd, data| {
            self.send(&[cmd as u8], false, 0, BACKGROUND_PRIORITY_CLOCK)?;
            self.send(data, true, 0, BACKGROUND_PRIORITY_CLOCK)
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
            self.send(cmds, false, minclock, 0)?;
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
