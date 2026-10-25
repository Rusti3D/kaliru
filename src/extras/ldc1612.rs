// Support for reading frequency samples from ldc1612
// (port of klippy/extras/ldc1612.py)
//
// Copyright (C) 2020-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, ClientCallback, FixedFreqReader};
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::extras::load_cell::{batch_helper, lookup_sensor_pin, no_cmd};
use crate::extras::probe_eddy_current::EddyCalibration;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::{CommandQueryWrapper, CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::printer::{ConfigFileObject, Printer, SubsystemComponentCollection};
use crate::pyfmt::py_round;
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const BATCH_UPDATES: f64 = 0.100;

const LDC1612_ADDR: i64 = 0x2A;

const DEFAULT_LDC1612_FREQ: i64 = 12000000;
const SETTLETIME: f64 = 0.005;
const DRIVECUR: i64 = 15;
const DEGLITCH: i64 = 0x05; // 10 Mhz

const LDC1612_MANUF_ID: i64 = 0x5449;
const LDC1612_DEV_ID: i64 = 0x3055;

const REG_RCOUNT0: i64 = 0x08;
const REG_OFFSET0: i64 = 0x0C;
const REG_SETTLECOUNT0: i64 = 0x10;
const REG_CLOCK_DIVIDERS0: i64 = 0x14;
const REG_ERROR_CONFIG: i64 = 0x19;
const REG_CONFIG: i64 = 0x1A;
const REG_MUX_CONFIG: i64 = 0x1B;
const REG_DRIVE_CURRENT0: i64 = 0x1E;
const REG_MANUFACTURER_ID: i64 = 0x7E;
const REG_DEVICE_ID: i64 = 0x7F;

const ERR2OUT_UR: i64 = 0x10; // Under-range
const ERR2OUT_OR: i64 = 0x08; // Over-range
const ERR2OUT_WD: i64 = 0x04; // Watchdog
const ERR2OUT_AH: i64 = 0x02; // Amplitude high
const ERR2OUT_AL: i64 = 0x01; // Amplitude low

/// a (time, frequency, z) sample
pub type FreqSample = (f64, f64, f64);

const CMD_LDC_CALIBRATE_HELP: &str = "Calibrate LDC1612 DRIVE_CURRENT register";

// Tool for determining appropriate DRIVE_CURRENT register
struct DriveCurrentCalibrate {
    printer: Rc<Printer>,
    sensor: RefCell<Weak<LDC1612>>,
    drive_cur: i64,
    name: String,
}

impl DriveCurrentCalibrate {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let drive_cur = config.getint("reg_drive_current", DRIVECUR, Lim::new().min(0).max(31))?;
        let name = config.get_name().to_string();
        let d = Rc::new(DriveCurrentCalibrate { printer: printer.clone(), sensor: RefCell::new(Weak::new()), drive_cur, name: name.clone() });
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let dd = d.clone();
        gcode.register_mux_command(
            "LDC_CALIBRATE_DRIVE_CURRENT",
            "CHIP",
            Some(name.split_whitespace().last().unwrap_or_default()),
            Rc::new(move |gcmd| dd.cmd_ldc_calibrate(gcmd)),
            Some(CMD_LDC_CALIBRATE_HELP),
        )?;
        Ok(d)
    }

    fn get_drive_current(&self) -> i64 {
        self.drive_cur
    }

    fn cmd_ldc_calibrate(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let sensor = self.sensor.borrow().upgrade().ok_or_else(|| KError::Other("ldc1612 sensor gone".into()))?;
        let is_in_progress = Rc::new(Cell::new(true));
        let p = is_in_progress.clone();
        sensor.add_client(Rc::new(move |_msg| p.get()))?;
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        toolhead.dwell(0.100)?;
        toolhead.wait_moves()?;
        let old_config = sensor.read_reg(REG_CONFIG)?;
        sensor.set_reg(REG_CONFIG, 0x001 | (1 << 9), 0)?;
        toolhead.wait_moves()?;
        toolhead.dwell(0.100)?;
        toolhead.wait_moves()?;
        let reg_drive_current0 = sensor.read_reg(REG_DRIVE_CURRENT0)?;
        sensor.set_reg(REG_CONFIG, old_config, 0)?;
        is_in_progress.set(false);
        // Report found value to user
        let drive_cur = (reg_drive_current0 >> 6) & 0x1F;
        gcmd.respond_info(
            &format!(
                "{}: reg_drive_current: {}\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
                self.name, drive_cur
            ),
            true,
        );
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        configfile.pconfig.borrow_mut().set(&self.name, "reg_drive_current", &drive_cur.to_string())?;
        Ok(())
    }
}

// Interface class to LDC1612 mcu support
pub struct LDC1612 {
    calibration: Option<Rc<EddyCalibration>>,
    dccal: Rc<DriveCurrentCalibrate>,
    data_rate: i64,
    i2c: Rc<McuI2c>,
    mcu: Rc<Mcu>,
    oid: i64,
    query_ldc1612_cmd: RefCell<Option<CommandWrapper>>,
    ldc1612_setup_home_cmd: RefCell<Option<CommandWrapper>>,
    query_ldc1612_home_state_cmd: RefCell<Option<CommandQueryWrapper>>,
    clock_freq: i64,
    error_mask_amplitude: i64,
    sensor_div: i64,
    freq_conv: f64,
    ffreader: FixedFreqReader,
    last_error_count: Cell<i64>,
    batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    name: String,
}

impl LDC1612 {
    pub fn new(config: &ConfigWrapper, calibration: Option<Rc<EddyCalibration>>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let slot: Rc<RefCell<Weak<LDC1612>>> = Rc::new(RefCell::new(Weak::new()));
        let dccal = DriveCurrentCalibrate::new(config)?;
        let data_rate = 250;
        // Setup mcu sensor_ldc1612 bulk query code
        let i2c = mcu_i2c_from_config(config, Some(LDC1612_ADDR), 400000)?;
        let mcu = i2c.get_mcu();
        let oid = mcu.create_oid();
        let clock_freq = config.getint("frequency", DEFAULT_LDC1612_FREQ, Lim::new().min(2000000).max(40000000))?;
        let amplitude_error_options = [("both", ERR2OUT_AH | ERR2OUT_AL), ("low", ERR2OUT_AL), ("high", ERR2OUT_AH), ("none", 0)];
        let error_mask_amplitude = config.getchoice("amplitude_errors", &amplitude_error_options, "both")?;
        // Coil frequency divider, assume 12MHz is BTT Eddy
        // BTT Eddy's coil frequency is > 1/4 of reference clock
        let sensor_div = if clock_freq != DEFAULT_LDC1612_FREQ { 1 } else { 2 };
        let freq_conv = (clock_freq * sensor_div) as f64 / (1u64 << 28) as f64;
        if config.get_opt("intb_pin")?.is_some() {
            let (pin_mcu, pin) = lookup_sensor_pin(config, "intb_pin")?;
            if !Rc::ptr_eq(&pin_mcu, &mcu) {
                return Err(KError::config("ldc1612 intb_pin must be on same mcu"));
            }
            mcu.add_config_cmd(&format!("config_ldc1612_with_intb oid={} i2c_oid={} intb_pin={}", oid, i2c.get_oid(), pin), false, false);
        } else {
            mcu.add_config_cmd(&format!("config_ldc1612 oid={} i2c_oid={}", oid, i2c.get_oid()), false, false);
        }
        mcu.add_config_cmd(&format!("query_ldc1612 oid={} rest_ticks=0", oid), false, true);
        let sl = slot.clone();
        mcu.register_config_callback(Rc::new(move || match sl.borrow().upgrade() {
            Some(s) => s.build_config(),
            None => Ok(()),
        }));
        // Bulk sample message reading
        let chip_smooth = data_rate as f64 * BATCH_UPDATES * 2.0;
        let ffreader = FixedFreqReader::new(&mcu, chip_smooth, ">I");
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let s = Rc::new(LDC1612 {
            calibration,
            dccal: dccal.clone(),
            data_rate,
            i2c,
            mcu,
            oid,
            query_ldc1612_cmd: RefCell::new(None),
            ldc1612_setup_home_cmd: RefCell::new(None),
            query_ldc1612_home_state_cmd: RefCell::new(None),
            clock_freq,
            error_mask_amplitude,
            sensor_div,
            freq_conv,
            ffreader,
            last_error_count: Cell::new(0),
            batch_bulk: RefCell::new(None),
            name: name.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&s);
        // DriveCurrentCalibrate(config, self): it finds the sensor once built
        *dccal.sensor.borrow_mut() = Rc::downgrade(&s);
        // Process messages in batches
        let batch_bulk = batch_helper(&printer, &s, Self::process_batch, Self::start_measurements, Self::finish_measurements, BATCH_UPDATES);
        let mut hdr = IndexMap::default();
        hdr.insert("header".into(), PyVal::Tuple(["time", "frequency", "z"].iter().map(|s| PyVal::Str(s.to_string())).collect()));
        batch_bulk.add_mux_endpoint("ldc1612/dump_ldc1612", "sensor", &name, PyVal::Dict(hdr))?;
        *s.batch_bulk.borrow_mut() = Some(batch_bulk);
        Ok(s)
    }

    fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.i2c.get_command_queue();
        *self.query_ldc1612_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_ldc1612 oid=%c rest_ticks=%u", Some(cmdqueue.clone()))?);
        self.ffreader.setup_query_command("query_status_ldc1612 oid=%c", self.oid, Some(cmdqueue.clone()))?;
        *self.ldc1612_setup_home_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "ldc1612_setup_home oid=%c clock=%u threshold=%u trsync_oid=%c trigger_reason=%c error_reason=%c",
            Some(cmdqueue.clone()),
        )?);
        *self.query_ldc1612_home_state_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "query_ldc1612_home_state oid=%c",
            "ldc1612_home_state oid=%c homing=%c trigger_clock=%u",
            Some(self.oid),
            Some(cmdqueue),
            false,
        )?);
        Ok(())
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.i2c.get_mcu()
    }

    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        let params = self.i2c.i2c_read(&[reg as u8], 2, true)?;
        match params.get("response") {
            Some(Val::Bytes(b)) if b.len() >= 2 => Ok(((b[0] as i64) << 8) | b[1] as i64),
            _ => Err(KError::Other("bytearray index out of range".into())),
        }
    }

    pub fn set_reg(&self, reg: i64, val: i64, minclock: u64) -> KResult<()> {
        self.i2c.i2c_write(&[reg as u8, ((val >> 8) & 0xFF) as u8, (val & 0xFF) as u8], minclock, 0)
    }

    pub fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        let bb = self.batch_bulk.borrow().clone().ok_or_else(|| KError::Other("ldc1612 batch helper missing".into()))?;
        bb.add_client(cb)
    }

    pub fn get_sample_rate(&self) -> i64 {
        self.data_rate
    }

    // Homing
    pub fn setup_home(&self, print_time: f64, trigger_freq: f64, trsync_oid: i64, hit_reason: i64, err_reason: i64) -> KResult<()> {
        let clock = self.mcu.print_time_to_clock(print_time);
        let tfreq = (trigger_freq / self.freq_conv + 0.5) as i64;
        let cmd = self.ldc1612_setup_home_cmd.borrow();
        let cmd = cmd.as_ref().ok_or_else(no_cmd)?;
        cmd.send(&[self.oid, clock, tfreq, trsync_oid, hit_reason, err_reason].map(Val::Int), 0, 0)
    }

    pub fn clear_home(&self) -> KResult<f64> {
        {
            let cmd = self.ldc1612_setup_home_cmd.borrow();
            cmd.as_ref().ok_or_else(no_cmd)?.send(&[self.oid, 0, 0, 0, 0, 0].map(Val::Int), 0, 0)?;
        }
        if self.mcu.is_fileoutput() {
            return Ok(0.0);
        }
        let params = {
            let q = self.query_ldc1612_home_state_cmd.borrow();
            q.as_ref().ok_or_else(no_cmd)?.send(&[Val::Int(self.oid)], 0, 0, true)?
        };
        let trigger_clock = match params.get("trigger_clock") {
            Some(Val::Int(c)) => *c,
            _ => 0,
        };
        let tclock = self.mcu.clock32_to_clock64(trigger_clock);
        Ok(self.mcu.clock_to_print_time(tclock as f64))
    }

    // Measurement decoding
    fn convert_samples(&self, samples: &[(f64, Vec<i64>)]) -> Vec<FreqSample> {
        let freq_conv = self.freq_conv;
        let mut out = Vec::with_capacity(samples.len());
        for (ptime, val) in samples {
            let val = val[0];
            let mv = val & 0x0FFFFFFF;
            if mv != val {
                self.last_error_count.set(self.last_error_count.get() + 1);
                continue;
            }
            out.push((py_round(*ptime, 6), py_round(freq_conv * mv as f64, 3), 999.9));
        }
        out
    }

    fn query_cmd(&self) -> KResult<std::cell::Ref<'_, CommandWrapper>> {
        let c = self.query_ldc1612_cmd.borrow();
        if c.is_none() {
            return Err(no_cmd());
        }
        Ok(std::cell::Ref::map(c, |c| c.as_ref().unwrap()))
    }

    // Start, stop, and process message batches
    fn start_measurements(&self) -> KResult<()> {
        // In case of miswiring, testing LDC1612 device ID prevents treating
        // noise or wrong signal as a correctly initialized device
        let manuf_id = self.read_reg(REG_MANUFACTURER_ID)?;
        let dev_id = self.read_reg(REG_DEVICE_ID)?;
        if manuf_id != LDC1612_MANUF_ID || dev_id != LDC1612_DEV_ID {
            return Err(KError::command(format!(
                "Invalid ldc1612 id (got {:x},{:x} vs {:x},{:x}).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty ldc1612 chip.",
                manuf_id, dev_id, LDC1612_MANUF_ID, LDC1612_DEV_ID
            )));
        }
        // Setup chip in requested query rate
        let rcount0 = self.clock_freq as f64 / (16.0 * self.data_rate as f64);
        self.set_reg(REG_RCOUNT0, (rcount0 + 0.5) as i64, 0)?;
        self.set_reg(REG_OFFSET0, 0, 0)?;
        self.set_reg(REG_SETTLECOUNT0, (SETTLETIME * self.clock_freq as f64 / 16.0 + 0.5) as i64, 0)?;
        self.set_reg(REG_CLOCK_DIVIDERS0, (self.sensor_div << 12) | 1, 0)?;
        let error_mask = ERR2OUT_UR | ERR2OUT_OR | ERR2OUT_WD | self.error_mask_amplitude;
        self.set_reg(REG_ERROR_CONFIG, (error_mask << 11) | 1, 0)?;
        self.set_reg(REG_MUX_CONFIG, 0x0208 | DEGLITCH, 0)?;
        self.set_reg(REG_CONFIG, 0x001 | (1 << 12) | (1 << 10) | (1 << 9), 0)?;
        self.set_reg(REG_DRIVE_CURRENT0, self.dccal.get_drive_current() << 11, 0)?;
        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(0.5 / self.data_rate as f64);
        self.query_cmd()?.send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        logging::info(&format!("LDC1612 starting '{}' measurements", self.name));
        // Initialize clock tracking
        self.ffreader.note_start()?;
        self.last_error_count.set(0);
        Ok(())
    }

    fn finish_measurements(&self) -> KResult<()> {
        // Halt bulk reading
        self.query_cmd()?.send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("LDC1612 finished '{}' measurements", self.name));
        Ok(())
    }

    fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let raw = self.ffreader.pull_samples()?;
        let mut samples = self.convert_samples(&raw);
        if samples.is_empty() {
            return Ok(PyVal::dict());
        }
        if let Some(cal) = &self.calibration {
            cal.apply_calibration(&mut samples);
        }
        let data = samples.iter().map(|(t, f, z)| PyVal::Tuple(vec![PyVal::Float(*t), PyVal::Float(*f), PyVal::Float(*z)])).collect();
        let d = crate::pydict! {
            "data" => PyVal::List(data),
            "errors" => PyVal::Int(self.last_error_count.get()),
            "overflows" => PyVal::Int(self.ffreader.get_last_overflows()),
        };
        Ok(PyVal::Dict(d))
    }
}

/// a sensor class of the eddy_current_sensors components
pub type EddySensorFactory = fn(&ConfigWrapper, Option<Rc<EddyCalibration>>) -> KResult<Rc<LDC1612>>;

pub fn register_components(subsystem: &SubsystemComponentCollection) -> KResult<()> {
    subsystem.register_component("eddy_current_sensors", "ldc1612", Rc::new(LDC1612::new as EddySensorFactory))
}
