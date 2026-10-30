// ADS1220 Support (port of klippy/extras/load_cell/ads1220.py)
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::{adc_batch, adc_row, batch_helper, lookup_sensor_pin, no_cmd, BulkAdcSensor};
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, ClientCallback, FixedFreqReader};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::logging;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::printer::Printer;
use crate::pyval::PyVal;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

// Constants
const UPDATE_INTERVAL: f64 = 0.10;
const RESET_CMD: u8 = 0x06;
const START_SYNC_CMD: u8 = 0x08;
const RREG_CMD: u8 = 0x20;
const WREG_CMD: u8 = 0x40;
const NOOP_CMD: u8 = 0x0;
const RESET_STATE: [u8; 4] = [0x0, 0x0, 0x0, 0x0];

// turn bytearrays into pretty hex strings: [0xff, 0x1]
pub fn hexify(byte_array: &[u8]) -> String {
    format!("[{}]", byte_array.iter().map(|b| format!("{:#x}", b)).collect::<Vec<_>>().join(", "))
}

pub const SPS_NORMAL: [(&str, i64); 7] = [("20", 20), ("45", 45), ("90", 90), ("175", 175), ("330", 330), ("600", 600), ("1000", 1000)];
pub const SPS_TURBO: [(&str, i64); 7] = [("40", 40), ("90", 90), ("180", 180), ("350", 350), ("660", 660), ("1200", 1200), ("2000", 2000)];
pub const GAIN_OPTIONS: [(&str, i64); 8] = [("1", 0x0), ("2", 0x1), ("4", 0x2), ("8", 0x3), ("16", 0x4), ("32", 0x5), ("64", 0x6), ("128", 0x7)];
pub const VREF_OPTIONS: [(&str, i64); 4] = [("internal", 0b0), ("REF0", 0b01), ("REF1", 0b10), ("analog_supply", 0b11)];

/// sps_normal updated with sps_turbo
pub fn sps_options() -> Vec<(&'static str, i64)> {
    let mut sps_options: Vec<(&str, i64)> = SPS_NORMAL.to_vec();
    for (k, v) in SPS_TURBO {
        match sps_options.iter_mut().find(|(ok, _)| *ok == k) {
            Some(e) => e.1 = v,
            None => sps_options.push((k, v)),
        }
    }
    sps_options
}

pub struct ADS1220 {
    pub printer: Rc<Printer>,
    pub name: String,
    pub consecutive_fails: Cell<i64>,
    pub gain: i64,
    pub sps: i64,
    pub is_turbo: bool,
    pub mux: i64,
    pub pga_bypass: bool,
    pub vref: i64,
    pub spi: Rc<McuSpi>,
    pub mcu: Rc<Mcu>,
    pub oid: i64,
    pub data_ready_pin: String,
    pub ffreader: Rc<FixedFreqReader>,
    pub batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    pub attach_probe_cmd: RefCell<Option<CommandWrapper>>,
    pub query_ads1220_cmd: RefCell<Option<CommandWrapper>>,
}

impl ADS1220 {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        // Chip options
        // Gain
        let gain = config.getchoice("gain", &GAIN_OPTIONS, "128")?;
        // Sample rate
        let sps_options = sps_options();
        let sps = config.getchoice("sample_rate", &sps_options, "660")?;
        let is_turbo = SPS_TURBO.iter().any(|(k, _)| *k == sps.to_string());
        // Input multiplexer: AINP and AINN
        let mux_options = [
            ("AIN0_AIN1", 0b0000),
            ("AIN0_AIN2", 0b0001),
            ("AIN0_AIN3", 0b0010),
            ("AIN1_AIN2", 0b0011),
            ("AIN1_AIN3", 0b0100),
            ("AIN2_AIN3", 0b0101),
            ("AIN1_AIN0", 0b0110),
            ("AIN3_AIN2", 0b0111),
            ("AIN0_AVSS", 0b1000),
            ("AIN1_AVSS", 0b1001),
            ("AIN2_AVSS", 0b1010),
            ("AIN3_AVSS", 0b1011),
        ];
        let mux = config.getchoice("input_mux", &mux_options, "AIN0_AIN1")?;
        // PGA Bypass
        let pga_bypass = config.getboolean("pga_bypass", false)?;
        // bypass PGA when AVSS is the negative input
        let force_pga_bypass = mux >= 0b1000;
        let pga_bypass = force_pga_bypass || pga_bypass;
        // Voltage Reference
        let vref = config.getchoice("vref", &VREF_OPTIONS, "internal")?;
        // check for conflict between REF1 and AIN0/AIN3
        let mux_conflict = [0b0000, 0b0001, 0b0010, 0b0100, 0b0101, 0b0110, 0b0111, 0b1000, 0b1011];
        if vref == 0b10 && mux_conflict.contains(&mux) {
            return Err(KError::config(
                "ADS1220 config error: AIN0/REFP1 and AIN3/REFN1 cant be used as a voltage reference and an input at the same time",
            ));
        }
        // SPI Setup
        let spi_speed = if is_turbo { 512000 } else { 256000 };
        let spi = mcu_spi_from_config(config, 1, "cs_pin", spi_speed, None, false)?;
        let mcu = spi.get_mcu();
        let oid = mcu.create_oid();
        // Data Ready (DRDY) Pin
        let (drdy_pin_mcu, data_ready_pin) = lookup_sensor_pin(config, "data_ready_pin")?;
        if !Rc::ptr_eq(&drdy_pin_mcu, &mcu) {
            return Err(KError::config("ADS1220 config error: SPI communication and data_ready_pin must be on the same MCU"));
        }
        // Bulk Sensor Setup
        // Clock tracking
        let chip_smooth = sps as f64 * UPDATE_INTERVAL * 2.0;
        // Measurement conversion
        let ffreader = Rc::new(FixedFreqReader::new(&mcu, chip_smooth, "<i"));
        let s = Rc::new(ADS1220 {
            printer: printer.clone(),
            name,
            consecutive_fails: Cell::new(0),
            gain,
            sps,
            is_turbo,
            mux,
            pga_bypass,
            vref,
            spi: spi.clone(),
            mcu: mcu.clone(),
            oid,
            data_ready_pin: data_ready_pin.clone(),
            ffreader,
            batch_bulk: RefCell::new(None),
            attach_probe_cmd: RefCell::new(None),
            query_ads1220_cmd: RefCell::new(None),
        });
        // Process messages in batches
        *s.batch_bulk.borrow_mut() =
            Some(batch_helper(&printer, &s, ADS1220::process_batch, ADS1220::start_measurements, ADS1220::finish_measurements, UPDATE_INTERVAL));
        // Command Configuration
        mcu.add_config_cmd(&format!("config_ads1220 oid={} spi_oid={} data_ready_pin={}", oid, spi.get_oid(), data_ready_pin), false, false);
        mcu.add_config_cmd(&format!("query_ads1220 oid={} rest_ticks=0", oid), false, true);
        let w = Rc::downgrade(&s);
        mcu.register_config_callback(Rc::new(move || match w.upgrade() {
            Some(s) => s.build_config(),
            None => Ok(()),
        }));
        Ok(s)
    }

    pub fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.spi.get_command_queue();
        *self.query_ads1220_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_ads1220 oid=%c rest_ticks=%u", Some(cmdqueue.clone()))?);
        *self.attach_probe_cmd.borrow_mut() =
            Some(self.mcu.lookup_command("ads1220_attach_load_cell_probe oid=%c load_cell_probe_oid=%c", None)?);
        self.ffreader.setup_query_command("query_ads1220_status oid=%c", self.oid, Some(cmdqueue))
    }

    // Measurement decoding
    pub fn convert_samples(samples: &[(f64, Vec<i64>)]) -> Vec<PyVal> {
        samples.iter().map(|(ptime, f)| adc_row(*ptime, f[0])).collect()
    }

    // Start, stop, and process message batches
    pub fn start_measurements(&self) -> KResult<()> {
        self.consecutive_fails.set(0);
        // Start bulk reading
        self.reset_chip()?;
        self.setup_chip()?;
        let rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps as f64));
        self.query_ads1220_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        logging::info(&format!("ADS1220 starting '{}' measurements", self.name));
        // Initialize clock tracking
        self.ffreader.note_start()
    }

    pub fn finish_measurements(&self) -> KResult<()> {
        // don't use serial connection after shutdown
        if self.printer.is_shutdown() {
            return Ok(());
        }
        // Halt bulk reading
        self.query_ads1220_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("ADS1220 finished '{}' measurements", self.name));
        Ok(())
    }

    pub fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let (samples, overflows) = self.ffreader.pull_samples()?;
        let samples = Self::convert_samples(&samples);
        Ok(adc_batch(samples, 0, overflows))
    }

    pub fn reset_chip(&self) -> KResult<()> {
        // the reset command takes 50us to complete
        self.send_command(RESET_CMD)?;
        // read startup register state and validate
        let val = self.read_reg(0x0, 4)?;
        if val != RESET_STATE {
            if self.mcu.is_fileoutput() {
                return Ok(());
            }
            return Err(KError::command(format!(
                "Invalid ads1220 reset state (got {} vs {}).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty ADS1220 chip.",
                hexify(&val),
                hexify(&RESET_STATE)
            )));
        }
        Ok(())
    }

    pub fn setup_chip(&self) -> KResult<()> {
        let continuous = 0x1; // enable continuous conversions
        let mode = if self.is_turbo { 0x2 } else { 0x0 }; // turbo mode
        let sps_list: &[(&str, i64)] = if self.is_turbo { &SPS_TURBO } else { &SPS_NORMAL };
        let sps = self.sps.to_string();
        let data_rate = sps_list
            .iter()
            .position(|(k, _)| *k == sps)
            .ok_or_else(|| KError::Other(format!("'{}' is not in list", sps)))? as i64;
        let reg_values = [
            ((self.mux << 4) | (self.gain << 1) | self.pga_bypass as i64) as u8,
            ((data_rate << 5) | (mode << 3) | (continuous << 2)) as u8,
            (self.vref << 6) as u8,
            0x0,
        ];
        self.write_reg(0x0, &reg_values)?;
        // start measurements immediately
        self.send_command(START_SYNC_CMD)
    }

    pub fn read_reg(&self, reg: u8, byte_count: usize) -> KResult<Vec<u8>> {
        let mut read_command = vec![RREG_CMD | (reg << 2) | (byte_count as u8 - 1)];
        read_command.extend(std::iter::repeat_n(NOOP_CMD, byte_count));
        let params = self.spi.spi_transfer(&read_command, 0, 0)?;
        match params.get("response") {
            Some(Val::Bytes(b)) => Ok(b.get(1..).unwrap_or(&[]).to_vec()),
            _ => Ok(vec![]),
        }
    }

    pub fn send_command(&self, cmd: u8) -> KResult<()> {
        self.spi.spi_send(&[cmd], 0, 0)
    }

    pub fn write_reg(&self, reg: u8, register_bytes: &[u8]) -> KResult<()> {
        let mut write_command = vec![WREG_CMD | (reg << 2) | (register_bytes.len() as u8 - 1)];
        write_command.extend_from_slice(register_bytes);
        self.spi.spi_send(&write_command, 0, 0)?;
        let stored_val = self.read_reg(reg, register_bytes.len())?;
        if register_bytes != stored_val.as_slice() {
            if self.mcu.is_fileoutput() {
                return Ok(());
            }
            return Err(KError::command(format!(
                "Failed to set ADS1220 register [0x{:x}] to {}: got {}. This may be a connection problem (e.g. faulty wiring)",
                reg,
                hexify(register_bytes),
                hexify(&stored_val)
            )));
        }
        Ok(())
    }
}

impl BulkAdcSensor for ADS1220 {
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

pub fn new_ads1220(config: &ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>> {
    Ok(ADS1220::new(config)?)
}
