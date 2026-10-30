// Chipsea CS1237 support (port of klippy/extras/load_cell/cs1237.py)
//
// Copyright (C) 2026  Xiaoyue Cui <2508041672@qq.com>
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
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const UPDATE_INTERVAL: f64 = 0.10;

pub struct CS1237 {
    pub printer: Rc<Printer>,
    pub name: String,
    pub last_error_count: Cell<i64>,
    sensor_errors: RefCell<IndexMap<i64, String>>,
    pub sps: i64,
    pub mcu: Rc<Mcu>,
    pub oid: i64,
    ffreader: FixedFreqReader,
    pub batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    pub query_cmd: RefCell<Option<CommandWrapper>>,
    pub attach_probe_cmd: RefCell<Option<CommandWrapper>>,
}

impl CS1237 {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();

        let rate_options = [("10", 0), ("40", 1), ("640", 2), ("1280", 3)];
        let gain_options = [("1", 0), ("2", 1), ("64", 2), ("128", 3)];
        let sps_options: Vec<(&str, i64)> = rate_options.iter().map(|(k, _)| (*k, k.parse::<i64>().unwrap_or(0))).collect();
        let sps = config.getchoice("sample_rate", &sps_options, "640")?;
        let rate = config.getchoice("sample_rate", &rate_options, "640")?;
        let gain = config.getchoice("gain", &gain_options, "128")?;
        let reference_output = config.getboolean("reference_output", true)?;
        let ref_off = if reference_output { 0 } else { 1 };
        let chip_config = (ref_off << 6) | (rate << 4) | (gain << 2);

        let (mcu, dout_pin) = lookup_sensor_pin(config, "dout_pin")?;
        let (sclk_mcu, sclk_pin) = lookup_sensor_pin(config, "sclk_pin")?;
        if !Rc::ptr_eq(&sclk_mcu, &mcu) {
            return Err(KError::config("CS1237 config error: All pins must be connected to the same MCU"));
        }
        let oid = mcu.create_oid();

        let chip_smooth = sps as f64 * UPDATE_INTERVAL * 2.0;
        let ffreader = FixedFreqReader::new(&mcu, chip_smooth, "<i");
        let s = Rc::new(CS1237 {
            printer: printer.clone(),
            name,
            last_error_count: Cell::new(0),
            sensor_errors: RefCell::new(IndexMap::default()),
            sps,
            mcu: mcu.clone(),
            oid,
            ffreader,
            batch_bulk: RefCell::new(None),
            query_cmd: RefCell::new(None),
            attach_probe_cmd: RefCell::new(None),
        });
        *s.batch_bulk.borrow_mut() =
            Some(batch_helper(&printer, &s, CS1237::process_batch, CS1237::start_measurements, CS1237::finish_measurements, UPDATE_INTERVAL));
        mcu.add_config_cmd(&format!("config_cs1237 oid={} config={} dout_pin={} sclk_pin={}", oid, chip_config, dout_pin, sclk_pin), false, false);
        mcu.add_config_cmd(&format!("query_cs1237 oid={} rest_ticks=0", oid), false, true);
        let w = Rc::downgrade(&s);
        mcu.register_config_callback(Rc::new(move || match w.upgrade() {
            Some(s) => s.build_config(),
            None => Ok(()),
        }));
        Ok(s)
    }

    pub fn build_config(&self) -> KResult<()> {
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.query_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_cs1237 oid=%c rest_ticks=%u", Some(cmd_queue.clone()))?);
        *self.attach_probe_cmd.borrow_mut() =
            Some(self.mcu.lookup_command("cs1237_attach_load_cell_probe oid=%c load_cell_probe_oid=%c", Some(cmd_queue.clone()))?);
        self.ffreader.setup_query_command("query_cs1237_status oid=%c", self.oid, Some(cmd_queue))?;
        let errors = self.mcu.get_enumerations().get("cs1237_error:").map(|t| t.enums.clone()).unwrap_or_default();
        *self.sensor_errors.borrow_mut() = errors.into_iter().map(|(k, v)| (v, k)).collect();
        Ok(())
    }

    pub fn get_status(&self, _eventtime: f64) -> PyVal {
        let d = crate::pydict! {
            "errors" => PyVal::Int(self.last_error_count.get()),
            "overflows" => PyVal::Int(self.ffreader.get_last_overflows()),
            "sample_rate" => PyVal::Int(self.sps),
        };
        PyVal::Dict(d)
    }

    pub fn lookup_sensor_error(&self, error_code: i64) -> String {
        self.sensor_errors.borrow().get(&error_code).cloned().unwrap_or_else(|| "Unknown CS1237 error".into())
    }

    pub fn convert_samples(&self, samples: &[(f64, Vec<i64>)]) -> Vec<PyVal> {
        let mut out = vec![];
        for (ptime, f) in samples {
            let value = f[0];
            let top_byte = (value >> 24) & 0xFF;
            if top_byte != 0x00 && top_byte != 0xFF {
                self.last_error_count.set(self.last_error_count.get() + 1);
                continue;
            }
            out.push(adc_row(*ptime, value));
        }
        out
    }

    pub fn start_measurements(&self) -> KResult<()> {
        let rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps as f64));
        self.query_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        logging::info(&format!("CS1237 starting '{}' measurements at {} SPS", self.name, self.sps));
        self.ffreader.note_start()
    }

    pub fn finish_measurements(&self) -> KResult<()> {
        if self.printer.is_shutdown() {
            return Ok(());
        }
        self.query_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("CS1237 finished '{}' measurements", self.name));
        Ok(())
    }

    pub fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let samples = self.ffreader.pull_samples()?;
        let samples = self.convert_samples(&samples);
        Ok(adc_batch(samples, self.last_error_count.get(), self.ffreader.get_last_overflows()))
    }
}

impl BulkAdcSensor for CS1237 {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    fn get_samples_per_second(&self) -> i64 {
        self.sps
    }

    fn get_range(&self) -> (i64, i64) {
        (-0x800000, 0x7FFFFF)
    }

    fn get_channel_count(&self) -> usize {
        1
    }

    fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        self.batch_bulk.borrow().clone().ok_or_else(no_cmd)?.add_client(callback)
    }

    fn attach_load_cell_probe(&self, load_cell_probe_oid: i64) -> KResult<()> {
        self.attach_probe_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send(&[Val::Int(self.oid), Val::Int(load_cell_probe_oid)], 0, 0)
    }
}

pub fn new_cs1237(config: &ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>> {
    Ok(CS1237::new(config)?)
}
