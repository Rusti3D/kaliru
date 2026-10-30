// Multi-sensor HX711/HX717 support (Pellcorp/COSMOS load_cell/hx711s.py)
// Reference: pellcorp/kalico 070a3066f2ffe1f0fdad291798263418b1c86004
//
// Copyright (C) 2026 James Turton <james.turton@gmx.com>
// Original HX711 driver Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::ADC_FACTOR;
use super::{adc_batch, batch_helper, no_cmd, BulkAdcSensor};
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, ClientCallback, FixedFreqReader};
use crate::logging;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::pins::PinsObject;
use crate::printer::Printer;
use crate::pyfmt::py_round;
use crate::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

// Constants
const UPDATE_INTERVAL: f64 = 0.10;
const SAMPLE_ERROR_DESYNC: i64 = -0x80000000;
const SAMPLE_ERROR_LONG_READ: i64 = 0x40000000;

// Implementation of HX711 and HX717
pub struct HX711SBase {
    printer: Rc<Printer>,
    name: String,
    last_error_count: Cell<i64>,
    consecutive_fails: Cell<i64>,
    sensor_type: &'static str,
    mcu: Rc<Mcu>,
    oid: i64,
    sps: i64,
    sensor_count: usize,
    ffreader: FixedFreqReader,
    batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    query_hx711s_cmd: RefCell<Option<CommandWrapper>>,
    attach_probe_cmd: RefCell<Option<CommandWrapper>>,
}

impl HX711SBase {
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
        let dout_names = config.get("dout_pins", crate::configfile::Dflt::Required)?;
        let sclk_names = config.get("sclk_pins", crate::configfile::Dflt::Required)?;
        let dout_names: Vec<_> = dout_names.split(',').map(str::trim).collect();
        let sclk_names: Vec<_> = sclk_names.split(',').map(str::trim).collect();
        validate_pin_count(sensor_type, dout_names.len(), sclk_names.len())?;
        let sensor_count = dout_names.len();
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let mut mcus = vec![];
        let mut pins = vec![];
        for pin in dout_names.iter().chain(&sclk_names) {
            let pp = ppins
                .0
                .borrow_mut()
                .lookup_pin(pin, false, false, None)
                .map_err(|e| KError::Pins(e.0))?;
            mcus.push(crate::extras::bus::pin_mcu(config, &pp)?);
            pins.push(pp.borrow().pin.clone());
        }
        let mcu = mcus[0].clone();
        if mcus.iter().any(|other| !Rc::ptr_eq(other, &mcu)) {
            return Err(KError::config(format!("{}: all pins must be on the same MCU", sensor_type)));
        }
        let oid = mcu.create_oid();
        // Samples per second choices
        let sps = config.getchoice_int("sample_rate", sample_rate_options, default_sample_rate)?;
        // gain/channel choices
        let gain_channel = config.getchoice("gain", gain_options, default_gain)?;
        // Bulk Sensor Setup
        // Clock tracking
        let chip_smooth = sps as f64 * UPDATE_INTERVAL * 2.0;
        let ffreader = FixedFreqReader::new(&mcu, chip_smooth, &format!("<{}", "i".repeat(sensor_count)));
        let s = Rc::new(HX711SBase {
            printer: printer.clone(),
            name,
            last_error_count: Cell::new(0),
            consecutive_fails: Cell::new(0),
            sensor_type,
            mcu: mcu.clone(),
            oid,
            sps,
            sensor_count,
            ffreader,
            batch_bulk: RefCell::new(None),
            query_hx711s_cmd: RefCell::new(None),
            attach_probe_cmd: RefCell::new(None),
        });
        // Process messages in batches
        *s.batch_bulk.borrow_mut() = Some(batch_helper(
            &printer,
            &s,
            HX711SBase::process_batch,
            HX711SBase::start_measurements,
            HX711SBase::finish_measurements,
            UPDATE_INTERVAL,
        ));
        // Command Configuration
        mcu.add_config_cmd(&format!("config_hx711s oid={} sensor_count={} gain_channel={}", oid, sensor_count, gain_channel), false, false);
        for i in 0..sensor_count {
            mcu.add_config_cmd(
                &format!("add_hx711s oid={} index={} dout_pin={} sclk_pin={}", oid, i, pins[i], pins[sensor_count + i]),
                false,
                false,
            );
        }
        mcu.add_config_cmd(&format!("query_hx711s oid={} rest_ticks=0", oid), false, true);
        let w = Rc::downgrade(&s);
        mcu.register_config_callback(Rc::new(move || match w.upgrade() {
            Some(s) => s.build_config(),
            None => Ok(()),
        }));
        Ok(s)
    }

    fn build_config(&self) -> KResult<()> {
        *self.query_hx711s_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_hx711s oid=%c rest_ticks=%u", None)?);
        *self.attach_probe_cmd.borrow_mut() = Some(
            self.mcu
                .lookup_command("hx711s_attach_load_cell_probe oid=%c load_cell_probe_oid=%c", None)?,
        );
        self.ffreader
            .setup_query_command("query_hx711s_status oid=%c", self.oid, Some(self.mcu.alloc_command_queue()))
    }

    // Start, stop, and process message batches
    fn start_measurements(&self) -> KResult<()> {
        self.consecutive_fails.set(0);
        self.last_error_count.set(0);
        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps as f64));
        self.query_hx711s_cmd
            .borrow()
            .as_ref()
            .ok_or_else(no_cmd)?
            .send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        logging::info(&format!("{} starting '{}' measurements", self.sensor_type, self.name));
        // Initialize clock tracking
        self.ffreader.note_start()
    }

    fn finish_measurements(&self) -> KResult<()> {
        // don't use serial connection after shutdown
        if self.printer.is_shutdown() {
            return Ok(());
        }
        // Halt bulk reading
        self.query_hx711s_cmd
            .borrow()
            .as_ref()
            .ok_or_else(no_cmd)?
            .send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("{} finished '{}' measurements", self.sensor_type, self.name));
        Ok(())
    }

    fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let prev_overflows = self.ffreader.get_last_overflows();
        let prev_error_count = self.last_error_count.get();
        let samples = self.ffreader.pull_samples()?;
        let (samples, errors) = decode_samples(&samples, &self.name);
        self.last_error_count.set(self.last_error_count.get() + errors);
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

impl BulkAdcSensor for HX711SBase {
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
        self.sensor_count
    }

    // add_client interface, direct pass through to bulk_sensor API
    fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        self.batch_bulk.borrow().clone().ok_or_else(no_cmd)?.add_client(callback)
    }

    fn attach_load_cell_probe(&self, load_cell_probe_oid: i64) -> KResult<()> {
        self.attach_probe_cmd
            .borrow()
            .as_ref()
            .ok_or_else(no_cmd)?
            .send(&[Val::Int(self.oid), Val::Int(load_cell_probe_oid)], 0, 0)
    }
}

pub fn new_hx711s(config: &ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>> {
    HX711SBase::new(
        config,
        "hx711s",
        // HX711 sps options
        &[(80, 80), (10, 10)],
        80,
        // HX711 gain/channel options
        &[("A-128", 1), ("B-32", 2), ("A-64", 3)],
        "A-128",
    )
}

pub fn new_hx717s(config: &ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>> {
    HX711SBase::new(
        config,
        "hx717s",
        // HX717 sps options
        &[(320, 320), (80, 80), (20, 20), (10, 10)],
        320,
        // HX717 gain/channel options
        &[("A-128", 1), ("B-64", 2), ("A-64", 3), ("B-8", 4)],
        "A-128",
    )
}

fn validate_pin_count(sensor_type: &str, dout: usize, sclk: usize) -> KResult<()> {
    if dout != sclk {
        return Err(KError::config(format!("{}: dout_pins and sclk_pins must have the same number of entries", sensor_type)));
    }
    if !(1..=4).contains(&dout) {
        return Err(KError::config(format!("{}: must specify 1 to 4 sensor pin pairs", sensor_type)));
    }
    Ok(())
}

// A frame is (time, counts0, fraction0, counts1, fraction1, ...).
// Firmware latches errors into every channel; stop at the first bad frame.
fn decode_samples(samples: &[(f64, Vec<i64>)], name: &str) -> (Vec<PyVal>, i64) {
    let mut out = vec![];
    for (ptime, channels) in samples {
        let error = match channels[0] {
            SAMPLE_ERROR_DESYNC => Some("DESYNC"),
            SAMPLE_ERROR_LONG_READ => Some("READ_TOO_LONG"),
            _ => None,
        };
        if let Some(error) = error {
            logging::error(&format!("{}: {} at t={:.3}", name, error, ptime));
            return (out, 1);
        }
        let mut row = vec![PyVal::Float(py_round(*ptime, 6))];
        for counts in channels {
            row.push(PyVal::Int(*counts));
            row.push(PyVal::Float(py_round(*counts as f64 * ADC_FACTOR, 9)));
        }
        out.push(PyVal::Tuple(row));
    }
    (out, 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_match_pellcorp_reference() {
        let cases: serde_json::Value = serde_json::from_str(include_str!("../../../tests/data/hx711s.json")).unwrap();
        for case in cases.as_array().unwrap() {
            let samples: Vec<(f64, Vec<i64>)> = serde_json::from_value(case["samples"].clone()).unwrap();
            let (rows, errors) = decode_samples(&samples, "test");
            assert_eq!(serde_json::from_str::<serde_json::Value>(&PyVal::List(rows).json()).unwrap(), case["expected"], "{}", case["name"]);
            assert_eq!(errors, case["errors"].as_i64().unwrap(), "{}", case["name"]);
        }
    }

    #[test]
    fn pin_counts_are_bounded_and_paired() {
        for count in 1..=4 {
            assert!(validate_pin_count("hx711s", count, count).is_ok());
        }
        for (dout, sclk) in [(0, 0), (5, 5), (1, 2), (4, 3)] {
            assert!(validate_pin_count("hx711s", dout, sclk).is_err());
        }
    }
}
