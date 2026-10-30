// HX711/HX717 Support (port of klippy/extras/load_cell/hx71x.py)
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::{adc_batch, adc_row, batch_helper, lookup_sensor_pin, no_cmd, BulkAdcSensor};
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, ClientCallback, FixedFreqReader};
use crate::logging;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::printer::Printer;
use crate::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

// Constants
const UPDATE_INTERVAL: f64 = 0.10;
const SAMPLE_ERROR_DESYNC: i64 = -0x80000000;
const SAMPLE_ERROR_LONG_READ: i64 = 0x40000000;

// Implementation of HX711 and HX717
pub struct HX71xBase {
    pub printer: Rc<Printer>,
    pub name: String,
    pub last_error_count: Cell<i64>,
    pub consecutive_fails: Cell<i64>,
    pub sensor_type: &'static str,
    pub mcu: Rc<Mcu>,
    pub oid: i64,
    pub sps: i64,
    ffreader: FixedFreqReader,
    pub batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    pub query_hx71x_cmd: RefCell<Option<CommandWrapper>>,
    pub attach_probe_cmd: RefCell<Option<CommandWrapper>>,
}

impl HX71xBase {
    fn new(
        config: &ConfigWrapper,
        sensor_type: &'static str,
        sample_rate_options: &[(i64, i64)],
        default_sample_rate: i64,
        gain_options: &[(&str, i64)],
        default_gain: &str,
    ) -> KResult<Rc<dyn BulkAdcSensor>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        // Chip options
        let (mcu, dout_pin) = lookup_sensor_pin(config, "dout_pin")?;
        let (sclk_mcu, sclk_pin) = lookup_sensor_pin(config, "sclk_pin")?;
        let oid = mcu.create_oid();
        if !Rc::ptr_eq(&sclk_mcu, &mcu) {
            return Err(KError::config(format!("{} config error: All pins must be connected to the same MCU", name)));
        }
        // Samples per second choices
        let sps = config.getchoice_int("sample_rate", sample_rate_options, default_sample_rate)?;
        // gain/channel choices
        let gain_channel = config.getchoice("gain", gain_options, default_gain)?;
        // Bulk Sensor Setup
        // Clock tracking
        let chip_smooth = sps as f64 * UPDATE_INTERVAL * 2.0;
        let ffreader = FixedFreqReader::new(&mcu, chip_smooth, "<i");
        let s = Rc::new(HX71xBase {
            printer: printer.clone(),
            name,
            last_error_count: Cell::new(0),
            consecutive_fails: Cell::new(0),
            sensor_type,
            mcu: mcu.clone(),
            oid,
            sps,
            ffreader,
            batch_bulk: RefCell::new(None),
            query_hx71x_cmd: RefCell::new(None),
            attach_probe_cmd: RefCell::new(None),
        });
        // Process messages in batches
        *s.batch_bulk.borrow_mut() = Some(batch_helper(
            &printer,
            &s,
            HX71xBase::process_batch,
            HX71xBase::start_measurements,
            HX71xBase::finish_measurements,
            UPDATE_INTERVAL,
        ));
        // Command Configuration
        mcu.add_config_cmd(
            &format!("config_hx71x oid={} gain_channel={} dout_pin={} sclk_pin={}", oid, gain_channel, dout_pin, sclk_pin),
            false,
            false,
        );
        mcu.add_config_cmd(&format!("query_hx71x oid={} rest_ticks=0", oid), false, true);
        let w = Rc::downgrade(&s);
        mcu.register_config_callback(Rc::new(move || match w.upgrade() {
            Some(s) => s.build_config(),
            None => Ok(()),
        }));
        Ok(s)
    }

    pub fn build_config(&self) -> KResult<()> {
        *self.query_hx71x_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_hx71x oid=%c rest_ticks=%u", None)?);
        *self.attach_probe_cmd.borrow_mut() =
            Some(self.mcu.lookup_command("hx71x_attach_load_cell_probe oid=%c load_cell_probe_oid=%c", None)?);
        self.ffreader.setup_query_command("query_hx71x_status oid=%c", self.oid, Some(self.mcu.alloc_command_queue()))
    }

    // Measurement decoding
    pub fn convert_samples(&self, samples: &[(f64, Vec<i64>)]) -> Vec<PyVal> {
        let mut out = vec![];
        for (ptime, f) in samples {
            let val = f[0];
            if val == SAMPLE_ERROR_DESYNC || val == SAMPLE_ERROR_LONG_READ {
                self.last_error_count.set(self.last_error_count.get() + 1);
                break; // additional errors are duplicates
            }
            out.push(adc_row(*ptime, val));
        }
        out
    }

    // Start, stop, and process message batches
    pub fn start_measurements(&self) -> KResult<()> {
        self.consecutive_fails.set(0);
        self.last_error_count.set(0);
        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps as f64));
        self.query_hx71x_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        logging::info(&format!("{} starting '{}' measurements", self.sensor_type, self.name));
        // Initialize clock tracking
        self.ffreader.note_start()
    }

    pub fn finish_measurements(&self) -> KResult<()> {
        // don't use serial connection after shutdown
        if self.printer.is_shutdown() {
            return Ok(());
        }
        // Halt bulk reading
        self.query_hx71x_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("{} finished '{}' measurements", self.sensor_type, self.name));
        Ok(())
    }

    pub fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let prev_overflows = self.ffreader.get_last_overflows();
        let prev_error_count = self.last_error_count.get();
        let samples = self.ffreader.pull_samples()?;
        let samples = self.convert_samples(&samples);
        let overflows = self.ffreader.get_last_overflows() - prev_overflows;
        let errors = self.last_error_count.get() - prev_error_count;
        if errors > 0 {
            logging::error(&format!("{}: Forced sensor restart due to error", self.name));
            self.finish_measurements()?;
            self.start_measurements()?;
        } else if overflows > 0 {
            self.consecutive_fails.set(self.consecutive_fails.get() + 1);
            if self.consecutive_fails.get() > 4 {
                logging::error(&format!("{}: Forced sensor restart due to overflows", self.name));
                self.finish_measurements()?;
                self.start_measurements()?;
            }
        } else {
            self.consecutive_fails.set(0);
        }
        Ok(adc_batch(samples, self.last_error_count.get(), self.ffreader.get_last_overflows()))
    }
}

impl BulkAdcSensor for HX71xBase {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    fn get_samples_per_second(&self) -> i64 {
        self.sps
    }

    // returns a tuple of the minimum and maximum value of the sensor, used to
    // detect if a data value is saturated
    fn get_range(&self) -> (i64, i64) {
        (-0x800000, 0x7FFFFF)
    }

    fn get_channel_count(&self) -> usize {
        1
    }

    // add_client interface, direct pass through to bulk_sensor API
    fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        self.batch_bulk.borrow().clone().ok_or_else(no_cmd)?.add_client(callback)
    }

    fn attach_load_cell_probe(&self, load_cell_probe_oid: i64) -> KResult<()> {
        self.attach_probe_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send(&[Val::Int(self.oid), Val::Int(load_cell_probe_oid)], 0, 0)
    }
}

pub fn new_hx711(config: &ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>> {
    HX71xBase::new(
        config,
        "hx711",
        // HX711 sps options
        &[(80, 80), (10, 10)],
        80,
        // HX711 gain/channel options
        &[("A-128", 1), ("B-32", 2), ("A-64", 3)],
        "A-128",
    )
}

pub fn new_hx717(config: &ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>> {
    HX71xBase::new(
        config,
        "hx717",
        // HX717 sps options
        &[(320, 320), (80, 80), (20, 20), (10, 10)],
        320,
        // HX717 gain/channel options
        &[("A-128", 1), ("B-64", 2), ("A-64", 3), ("B-8", 4)],
        "A-128",
    )
}
