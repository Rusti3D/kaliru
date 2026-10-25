// Support for "neopixel" leds (port of klippy/extras/neopixel.py)
//
// Copyright (C) 2019-2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::led::{Color, LEDHelper};
use crate::logging;
use crate::mcu::{get_printer_mcu, CommandQueryWrapper, CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

const BACKGROUND_PRIORITY_CLOCK: u64 = 0x7FFFFFFF00000000;

const BIT_MAX_TIME: f64 = 0.000004;
const RESET_MIN_TIME: f64 = 0.000050;

const MAX_MCU_SIZE: usize = 500; // Sanity check on LED chain length

pub struct PrinterNeoPixel {
    printer: Rc<Printer>,
    mcu: Rc<Mcu>,
    oid: i64,
    pin: String,
    neopixel_update_cmd: RefCell<Option<CommandWrapper>>,
    neopixel_send_cmd: RefCell<Option<CommandQueryWrapper>>,
    color_map: Vec<(usize, (usize, usize))>,
    led_helper: RefCell<Option<Rc<LEDHelper>>>,
    color_data: RefCell<Vec<u8>>,
    old_color_data: RefCell<Vec<u8>>,
}

fn sorted(s: &str) -> Vec<char> {
    let mut v: Vec<char> = s.chars().collect();
    v.sort();
    v
}

impl PrinterNeoPixel {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Configure neopixel
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin_desc = config.get("pin", Dflt::Required)?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(&pin_desc, false, false, None).map_err(|e| KError::Pins(e.0))?;
        let (chip_name, pin) = {
            let p = pin_params.borrow();
            (p.chip_name.clone(), p.pin.clone())
        };
        let mcu = get_printer_mcu(&printer, &chip_name)?;
        let oid = mcu.create_oid();
        // Build color map
        let chain_count = config.getint("chain_count", 1, Lim::new().min(1))?;
        let mut color_order =
            config.getlist("color_order", PyVal::List(vec![PyVal::from("GRB")]), ",", None)?.unwrap_or_default();
        if color_order.len() == 1 {
            color_order = vec![color_order[0].clone(); chain_count as usize];
        }
        if color_order.len() != chain_count as usize {
            return Err(KError::config("color_order does not match chain_count"));
        }
        let mut color_indexes = vec![];
        for (lidx, co) in color_order.iter().enumerate() {
            if sorted(co) != sorted("RGB") && sorted(co) != sorted("RGBW") {
                return Err(KError::config(format!("Invalid color_order '{}'", co)));
            }
            color_indexes.extend(co.chars().map(|c| (lidx, "RGBW".find(c).unwrap_or(0))));
        }
        let color_map: Vec<(usize, (usize, usize))> = color_indexes.into_iter().enumerate().collect();
        if color_map.len() > MAX_MCU_SIZE {
            return Err(KError::config("neopixel chain too long"));
        }
        let n = Rc::new(PrinterNeoPixel {
            printer: printer.clone(),
            mcu: mcu.clone(),
            oid,
            pin,
            neopixel_update_cmd: RefCell::new(None),
            neopixel_send_cmd: RefCell::new(None),
            color_data: RefCell::new(vec![0; color_map.len()]),
            old_color_data: RefCell::new(vec![]),
            color_map,
            led_helper: RefCell::new(None),
        });
        let nn = n.clone();
        mcu.register_config_callback(Rc::new(move || nn.build_config()));
        // Initialize color data
        let w: Weak<Self> = Rc::downgrade(&n);
        let helper = LEDHelper::new(
            config,
            Rc::new(move |state, print_time| match w.upgrade() {
                Some(n) => n.update_leds(state, print_time),
                None => Ok(()),
            }),
            chain_count,
        )?;
        n.update_color_data(&helper.led_state());
        *n.old_color_data.borrow_mut() = n.color_data.borrow().iter().map(|d| d ^ 1).collect();
        *n.led_helper.borrow_mut() = Some(helper);
        // Register callbacks
        let nn = n.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| nn.send_data(None)));
        let nn = n.clone();
        printer.register_event_handler(mcu.get_non_critical_reconnect_event_name(), Rc::new(move |_| nn.send_data(None)));
        Ok(n)
    }

    fn build_config(&self) -> KResult<()> {
        let bmt = self.mcu.seconds_to_clock(BIT_MAX_TIME);
        let rmt = self.mcu.seconds_to_clock(RESET_MIN_TIME);
        self.mcu.add_config_cmd(
            &format!(
                "config_neopixel oid={} pin={} data_size={} bit_max_ticks={} reset_min_ticks={}",
                self.oid,
                self.pin,
                self.color_data.borrow().len(),
                bmt,
                rmt
            ),
            false,
            false,
        );
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.neopixel_update_cmd.borrow_mut() = Some(self.mcu.lookup_command("neopixel_update oid=%c pos=%hu data=%*s", Some(cmd_queue.clone()))?);
        *self.neopixel_send_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "neopixel_send oid=%c",
            "neopixel_result oid=%c success=%c",
            Some(self.oid),
            Some(cmd_queue),
            false,
        )?);
        Ok(())
    }

    fn update_color_data(&self, led_state: &[Color]) {
        let mut color_data = self.color_data.borrow_mut();
        for (cdidx, (lidx, cidx)) in &self.color_map {
            color_data[*cdidx] = (led_state[*lidx][*cidx] * 255.0 + 0.5).trunc() as i64 as u8;
        }
    }

    fn send_data(&self, print_time: Option<f64>) -> KResult<()> {
        if self.mcu.non_critical_disconnected.get() {
            return Ok(());
        }
        let new_data = self.color_data.borrow().clone();
        let old_data = self.old_color_data.borrow().clone();
        if new_data == old_data {
            return Ok(());
        }
        // Find the position of all changed bytes in this framebuffer
        let mut diffs: Vec<[usize; 2]> =
            new_data.iter().zip(&old_data).enumerate().filter(|(_, (n, o))| n != o).map(|(i, _)| [i, 1]).collect();
        // Batch together changes that are close to each other
        for i in (0..diffs.len().saturating_sub(1)).rev() {
            let [pos, _count] = diffs[i];
            let [nextpos, nextcount] = diffs[i + 1];
            if pos + 5 >= nextpos && nextcount < 16 {
                diffs[i][1] = nextcount + (nextpos - pos);
                diffs.remove(i + 1);
            }
        }
        // Transmit changes
        {
            let ucmd = self.neopixel_update_cmd.borrow();
            let ucmd = ucmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
            for [pos, count] in &diffs {
                let end = (pos + count).min(new_data.len());
                ucmd.send(&[Val::Int(self.oid), Val::Int(*pos as i64), Val::Bytes(new_data[*pos..end].to_vec())], 0, BACKGROUND_PRIORITY_CLOCK)?;
            }
        }
        *self.old_color_data.borrow_mut() = new_data;
        // Instruct mcu to update the LEDs
        let mut minclock = 0;
        if let Some(pt) = print_time {
            minclock = self.mcu.print_time_to_clock(pt);
        }
        if self.printer.start_arg("debugoutput").is_some() {
            return Ok(());
        }
        let scmd = self.neopixel_send_cmd.borrow();
        let scmd = scmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        for _ in 0..8 {
            let params = scmd.send(&[Val::Int(self.oid)], minclock as u64, BACKGROUND_PRIORITY_CLOCK, true)?;
            if matches!(params.get("success"), Some(Val::Int(v)) if *v != 0) {
                return Ok(());
            }
        }
        logging::info("Neopixel update did not succeed");
        Ok(())
    }

    fn update_leds(&self, led_state: &[Color], print_time: Option<f64>) -> KResult<()> {
        self.update_color_data(led_state);
        self.send_data(print_time)
    }
}

impl PrinterObject for PrinterNeoPixel {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        self.led_helper.borrow().as_ref().map(|h| h.get_status())
    }
    fn teardown(&self) {
        if let Some(h) = self.led_helper.borrow_mut().take() {
            h.teardown();
        }
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterNeoPixel::new(config)?)
}
