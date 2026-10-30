// Support for "dotstar" leds (port of klippy/extras/dotstar.py)
//
// Copyright (C) 2019-2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::McuSpi;
use crate::extras::led::{Color, LEDHelper};
use crate::mcu::get_printer_mcu;
use crate::pins::PinsObject;
use crate::printer::{PObj, PrinterObject};
use crate::pyval::PyVal;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

const BACKGROUND_PRIORITY_CLOCK: u64 = 0x7FFFFFFF00000000;

pub struct PrinterDotstar {
    pub spi: Rc<McuSpi>,
    pub(crate) led_helper: RefCell<Option<Rc<LEDHelper>>>,
    pub prev_data: RefCell<Option<Vec<Color>>>,
    pub chain_count: i64,
}

impl PrinterDotstar {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Configure a software spi bus
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let data_pin = config.get("data_pin", Dflt::Required)?;
        let data_pin_params = ppins.0.borrow_mut().lookup_pin(&data_pin, false, false, None).map_err(|e| KError::Pins(e.0))?;
        let clock_pin = config.get("clock_pin", Dflt::Required)?;
        let clock_pin_params = ppins.0.borrow_mut().lookup_pin(&clock_pin, false, false, None).map_err(|e| KError::Pins(e.0))?;
        let (dp, cp) = (data_pin_params.borrow(), clock_pin_params.borrow());
        if dp.chip_name != cp.chip_name {
            return Err(KError::config("Dotstar pins must be on same mcu"));
        }
        let mcu = get_printer_mcu(&printer, &dp.chip_name)?;
        let sw_spi_pins = [dp.pin.clone(), dp.pin.clone(), cp.pin.clone()];
        drop((dp, cp));
        let spi = McuSpi::new(&mcu, None, None, 0, 500000, Some(sw_spi_pins), false);
        // Initialize color data
        let chain_count = config.getint("chain_count", 1, Lim::new().min(1))?;
        let d = Rc::new(PrinterDotstar { spi, led_helper: RefCell::new(None), prev_data: RefCell::new(None), chain_count });
        let w: Weak<Self> = Rc::downgrade(&d);
        let helper = LEDHelper::new(
            config,
            Rc::new(move |state, print_time| match w.upgrade() {
                Some(d) => d.update_leds(state, print_time),
                None => Ok(()),
            }),
            chain_count,
        )?;
        *d.led_helper.borrow_mut() = Some(helper);
        // Register commands
        let dd = d.clone();
        crate::events::KlippyConnect::on(&printer, move || {
            let state = dd.led_helper.borrow().as_ref().map(|h| h.led_state()).unwrap_or_default();
            dd.update_leds(&state, None)
        });
        Ok(d)
    }

    pub fn update_leds(&self, led_state: &[Color], print_time: Option<f64>) -> KResult<()> {
        if self.prev_data.borrow().as_deref() == Some(led_state) {
            return Ok(());
        }
        *self.prev_data.borrow_mut() = Some(led_state.to_vec());
        // Build data to send
        let mut data = vec![0u8; (led_state.len() + 2) * 4];
        let byte = |v: f64| (v * 255.0 + 0.5).trunc() as i64 as u8;
        for (i, c) in led_state.iter().enumerate() {
            let (red, green, blue) = (c[0], c[1], c[2]);
            let idx = (i + 1) * 4;
            data[idx] = 0xFF;
            data[idx + 1] = byte(blue);
            data[idx + 2] = byte(green);
            data[idx + 3] = byte(red);
        }
        let n = data.len();
        data[n - 4..].copy_from_slice(&[0xFF; 4]);
        // Transmit update
        let mut minclock = 0;
        if let Some(pt) = print_time {
            minclock = self.spi.get_mcu().print_time_to_clock(pt);
        }
        for d in data.chunks(20) {
            self.spi.spi_send(d, minclock as u64, BACKGROUND_PRIORITY_CLOCK)?;
        }
        Ok(())
    }
}

impl PrinterObject for PrinterDotstar {
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
    Ok(PrinterDotstar::new(config)?)
}
