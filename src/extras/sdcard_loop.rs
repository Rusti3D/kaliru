// Sdcard file looping support (port of klippy/extras/sdcard_loop.py)
//
// Copyright (C) 2021  Jason S. McMullan <jason.mcmullan@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::extras::virtual_sdcard::VirtualSD;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, PrinterObject};
use std::cell::RefCell;
use std::rc::Rc;

const CMD_SDCARD_LOOP_BEGIN_HELP: &str = "Begins a looped section in the SD file.";
const CMD_SDCARD_LOOP_END_HELP: &str = "Ends a looped section in the SD file.";
const CMD_SDCARD_LOOP_DESIST_HELP: &str = "Stops iterating the current loop stack.";

pub struct SDCardLoop {
    pub sdcard: Rc<VirtualSD>,
    pub loop_stack: RefCell<Vec<(i64, u64)>>,
}

impl SDCardLoop {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let sdcard = printer.load_as::<VirtualSD>(config, "virtual_sdcard")?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let s = Rc::new(SDCardLoop { sdcard, loop_stack: RefCell::new(vec![]) });
        type SCmd = fn(&SDCardLoop, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, SCmd, &str); 3] = [
            ("SDCARD_LOOP_BEGIN", SDCardLoop::cmd_sdcard_loop_begin, CMD_SDCARD_LOOP_BEGIN_HELP),
            ("SDCARD_LOOP_END", SDCardLoop::cmd_sdcard_loop_end, CMD_SDCARD_LOOP_END_HELP),
            ("SDCARD_LOOP_DESIST", SDCardLoop::cmd_sdcard_loop_desist, CMD_SDCARD_LOOP_DESIST_HELP),
        ];
        for (name, f, desc) in cmds {
            let w = Rc::downgrade(&s);
            gcode.register_command(
                name,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(s) => f(&s, g),
                    None => Ok(()),
                })),
                false,
                Some(desc),
            )?;
        }
        Ok(s)
    }

    pub fn cmd_sdcard_loop_begin(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let count = gcmd.get_int("COUNT", None, Lim::new().min(0))?;
        if !self.loop_begin(count) {
            return Err(gcmd.error("Only permitted in SD file."));
        }
        Ok(())
    }

    pub fn cmd_sdcard_loop_end(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if !self.loop_end() {
            return Err(gcmd.error("Only permitted in SD file."));
        }
        Ok(())
    }

    pub fn cmd_sdcard_loop_desist(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if !self.loop_desist() {
            return Err(gcmd.error("Only permitted outside of a SD file."));
        }
        Ok(())
    }

    pub fn loop_begin(&self, count: i64) -> bool {
        if !self.sdcard.is_cmd_from_sd() {
            // Can only run inside of an SD file
            return false;
        }
        self.loop_stack.borrow_mut().push((count, self.sdcard.get_file_position()));
        true
    }

    pub fn loop_end(&self) -> bool {
        if !self.sdcard.is_cmd_from_sd() {
            // Can only run inside of an SD file
            return false;
        }
        // If the stack is empty, no need to skip back
        let Some((count, position)) = self.loop_stack.borrow_mut().pop() else {
            return true;
        };
        // Get iteration count and return position
        if count == 0 {
            // Infinite loop
            self.sdcard.set_file_position(position);
            self.loop_stack.borrow_mut().push((0, position));
        } else if count == 1 {
            // Last repeat
            // Nothing to do
        } else {
            // At the next opportunity, seek back to the start of the loop
            self.sdcard.set_file_position(position);
            // Decrement the count by 1, and add the position back to the stack
            self.loop_stack.borrow_mut().push((count - 1, position));
        }
        true
    }

    pub fn loop_desist(&self) -> bool {
        if self.sdcard.is_cmd_from_sd() {
            // Can only run outside of an SD file
            return false;
        }
        logging::info("Desisting existing SD loops");
        self.loop_stack.borrow_mut().clear();
        true
    }
}

impl PrinterObject for SDCardLoop {
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(SDCardLoop::new(config)?)
}
