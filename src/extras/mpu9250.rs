// Support for reading acceleration data from an mpu9250 chip
// (port of klippy/extras/mpu9250.py)
//
// Copyright (C) 2022  Harry Beyel <harry3b9@gmail.com>
// Copyright (C) 2020-2021 Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::adxl345::{accel_header, batch_msg, convert_sample, read_axes_map, AccelChip, AccelCommandHelper, AccelQueryHelper, AxisMap, BATCH_UPDATES};
use crate::extras::bulk_sensor::{BatchBulkHelper, FixedFreqReader, RawSample};
use crate::extras::bus::{mcu_i2c_from_config, McuI2c};
use crate::logging;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const MPU9250_ADDR: i64 = 0x68;

const MPU_DEV_IDS: [(i64, &str); 7] = [
    (0x74, "mpu-9515"),
    (0x73, "mpu-9255"),
    (0x71, "mpu-9250"),
    (0x70, "mpu-6500"),
    (0x68, "mpu-6050"),
    // everything above are normal MPU IDs
    (0x75, "mpu-unknown (DEFECTIVE! USE WITH CAUTION!)"),
    (0x69, "mpu-unknown (DEFECTIVE! USE WITH CAUTION!)"),
];

// MPU9250 registers
const REG_DEVID: i64 = 0x75;
const REG_FIFO_EN: i64 = 0x23;
const REG_SMPLRT_DIV: i64 = 0x19;
const REG_CONFIG: i64 = 0x1A;
const REG_ACCEL_CONFIG: i64 = 0x1C;
const REG_ACCEL_CONFIG2: i64 = 0x1D;
const REG_USER_CTRL: i64 = 0x6A;
const REG_PWR_MGMT_1: i64 = 0x6B;
const REG_PWR_MGMT_2: i64 = 0x6C;
const REG_INT_STATUS: i64 = 0x3A;

const SAMPLE_RATE_DIVS: [(i64, i64); 1] = [(4000, 0x00)];

const SET_CONFIG: i64 = 0x01; // FIFO mode 'stream' style
const SET_ACCEL_CONFIG: i64 = 0x10; // 8g full scale
const SET_ACCEL_CONFIG2: i64 = 0x08; // 1046Hz BW, 0.503ms delay 4kHz sample rate
const SET_PWR_MGMT_1_WAKE: i64 = 0x00;
const SET_PWR_MGMT_1_SLEEP: i64 = 0x40;
const SET_PWR_MGMT_2_ACCEL_ON: i64 = 0x07;
const SET_PWR_MGMT_2_OFF: i64 = 0x3F;
const SET_USER_FIFO_RESET: i64 = 0x04;
const SET_USER_FIFO_EN: i64 = 0x40;
const SET_ENABLE_FIFO: i64 = 0x08;
const SET_DISABLE_FIFO: i64 = 0x00;

const FREEFALL_ACCEL: f64 = 9.80665 * 1000.0;
// SCALE = 1/4096 g/LSB @8g scale * Earth gravity in mm/s**2
const SCALE: f64 = 0.000244140625 * FREEFALL_ACCEL;

// Printer class that controls MPU9250 chip
pub struct MPU9250 {
    printer: Rc<Printer>,
    cmd_helper: RefCell<Option<Rc<AccelCommandHelper>>>,
    axes_map: AxisMap,
    data_rate: i64,
    i2c: Rc<McuI2c>,
    mcu: Rc<Mcu>,
    oid: i64,
    query_mpu9250_cmd: RefCell<Option<CommandWrapper>>,
    ffreader: FixedFreqReader,
    last_error_count: Cell<i64>,
    batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    pub name: String,
}

impl MPU9250 {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // AccelCommandHelper(config, self): the chip is set once built
        let cmd_helper = AccelCommandHelper::new(config)?;
        let axes_map = read_axes_map(config, SCALE, SCALE, SCALE)?;
        let data_rate = config.getint("rate", 4000, Lim::new())?;
        if !SAMPLE_RATE_DIVS.iter().any(|(r, _)| *r == data_rate) {
            return Err(KError::config(format!("Invalid rate parameter: {}", data_rate)));
        }
        // Setup mcu sensor_mpu9250 bulk query code
        let i2c = mcu_i2c_from_config(config, Some(MPU9250_ADDR), 400000)?;
        let mcu = i2c.get_mcu();
        let oid = mcu.create_oid();
        let slot: Rc<RefCell<Weak<MPU9250>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        mcu.register_config_callback(Rc::new(move || match sl.borrow().upgrade() {
            Some(m) => m.build_config(),
            None => Ok(()),
        }));
        // Bulk sample message reading
        let chip_smooth = data_rate as f64 * BATCH_UPDATES * 2.0;
        let ffreader = FixedFreqReader::new(&mcu, chip_smooth, ">hhh");
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let m = Rc::new(MPU9250 {
            printer: printer.clone(),
            cmd_helper: RefCell::new(None),
            axes_map,
            data_rate,
            i2c,
            mcu,
            oid,
            query_mpu9250_cmd: RefCell::new(None),
            ffreader,
            last_error_count: Cell::new(0),
            batch_bulk: RefCell::new(None),
            name: name.clone(),
        });
        *slot.borrow_mut() = Rc::downgrade(&m);
        let chip: Rc<dyn AccelChip> = m.clone();
        cmd_helper.set_chip(Rc::downgrade(&chip));
        *m.cmd_helper.borrow_mut() = Some(cmd_helper);
        // Process messages in batches
        let (w1, w2, w3) = (Rc::downgrade(&m), Rc::downgrade(&m), Rc::downgrade(&m));
        let batch_bulk = BatchBulkHelper::new(
            &printer,
            Rc::new(move |eventtime| match w1.upgrade() {
                Some(m) => m.process_batch(eventtime),
                None => Ok(PyVal::dict()),
            }),
            Some(Rc::new(move || match w2.upgrade() {
                Some(m) => m.start_measurements(),
                None => Ok(()),
            })),
            Some(Rc::new(move || match w3.upgrade() {
                Some(m) => m.finish_measurements(),
                None => Ok(()),
            })),
            BATCH_UPDATES,
        );
        batch_bulk.add_mux_endpoint("mpu9250/dump_mpu9250", "sensor", &name, accel_header())?;
        *m.batch_bulk.borrow_mut() = Some(batch_bulk);
        Ok(m)
    }

    fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.i2c.get_command_queue();
        self.mcu.add_config_cmd(&format!("config_mpu9250 oid={} i2c_oid={}", self.oid, self.i2c.get_oid()), false, false);
        self.mcu.add_config_cmd(&format!("query_mpu9250 oid={} rest_ticks=0", self.oid), false, true);
        *self.query_mpu9250_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_mpu9250 oid=%c rest_ticks=%u", Some(cmdqueue.clone()))?);
        self.ffreader.setup_query_command("query_mpu9250_status oid=%c", self.oid, Some(cmdqueue))
    }

    fn check_connected(&self) -> KResult<()> {
        if self.mcu.non_critical_disconnected.get() {
            return Err(KError::command(format!(
                "MPU: {} could not connect because mcu: {} is non_critical_disconnected!",
                self.name,
                self.mcu.get_name()
            )));
        }
        Ok(())
    }

    fn query_cmd(&self) -> KResult<std::cell::Ref<'_, CommandWrapper>> {
        let c = self.query_mpu9250_cmd.borrow();
        if c.is_none() {
            return Err(KError::Other("'NoneType' object has no attribute 'send'".into()));
        }
        Ok(std::cell::Ref::map(c, |c| c.as_ref().unwrap()))
    }

    fn set_reg_at(&self, reg: i64, val: i64, minclock: u64) -> KResult<()> {
        self.i2c.i2c_write(&[reg as u8, (val & 0xFF) as u8], minclock, 0)
    }

    // Measurement decoding
    fn convert_samples(&self, samples: &[RawSample]) -> Vec<[f64; 4]> {
        samples.iter().map(|(ptime, f)| convert_sample(&self.axes_map, *ptime, [f[0], f[1], f[2]])).collect()
    }

    // Start, stop, and process message batches
    fn start_measurements(&self) -> KResult<()> {
        // In case of miswiring, testing MPU9250 device ID prevents treating
        // noise or wrong signal as a correctly initialized device
        let dev_id = self.read_reg(REG_DEVID)?;
        match MPU_DEV_IDS.iter().find(|(id, _)| *id == dev_id) {
            None => {
                return Err(KError::command(format!(
                    "Invalid mpu id (got {:x}).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty chip.",
                    dev_id
                )))
            }
            Some((_, name)) => logging::info(&format!("Found {} with id {:x}", name, dev_id)),
        }
        // Setup chip in requested query rate
        self.set_reg(REG_PWR_MGMT_1, SET_PWR_MGMT_1_WAKE)?;
        self.set_reg(REG_PWR_MGMT_2, SET_PWR_MGMT_2_ACCEL_ON)?;
        // Add 20ms pause for accelerometer chip wake up
        self.read_reg(REG_DEVID)?; // Dummy read to ensure queues flushed
        let systime = self.printer.get_reactor().monotonic();
        let next_time = self.mcu.estimated_print_time(systime) + 0.020;
        let div = SAMPLE_RATE_DIVS.iter().find(|(r, _)| *r == self.data_rate).map(|(_, d)| *d).unwrap_or(0);
        self.set_reg_at(REG_SMPLRT_DIV, div, self.mcu.print_time_to_clock(next_time) as u64)?;
        self.set_reg(REG_CONFIG, SET_CONFIG)?;
        self.set_reg(REG_ACCEL_CONFIG, SET_ACCEL_CONFIG)?;
        self.set_reg(REG_ACCEL_CONFIG2, SET_ACCEL_CONFIG2)?;
        // Reset fifo
        self.set_reg(REG_FIFO_EN, SET_DISABLE_FIFO)?;
        self.set_reg(REG_USER_CTRL, SET_USER_FIFO_RESET)?;
        self.set_reg(REG_USER_CTRL, SET_USER_FIFO_EN)?;
        self.read_reg(REG_INT_STATUS)?; // clear FIFO overflow flag

        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(4.0 / self.data_rate as f64);
        self.query_cmd()?.send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        self.set_reg(REG_FIFO_EN, SET_ENABLE_FIFO)?;
        logging::info(&format!("MPU9250 starting '{}' measurements", self.name));
        // Initialize clock tracking
        self.ffreader.note_start()?;
        self.last_error_count.set(0);
        Ok(())
    }

    fn finish_measurements(&self) -> KResult<()> {
        // Halt bulk reading
        self.set_reg(REG_FIFO_EN, SET_DISABLE_FIFO)?;
        self.query_cmd()?.send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("MPU9250 finished '{}' measurements", self.name));
        self.set_reg(REG_PWR_MGMT_1, SET_PWR_MGMT_1_SLEEP)?;
        self.set_reg(REG_PWR_MGMT_2, SET_PWR_MGMT_2_OFF)
    }

    fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let samples = self.ffreader.pull_samples()?;
        let samples = self.convert_samples(&samples);
        Ok(batch_msg(&samples, self.last_error_count.get(), self.ffreader.get_last_overflows()))
    }
}

impl AccelChip for MPU9250 {
    fn start_internal_client(&self) -> KResult<Rc<AccelQueryHelper>> {
        self.check_connected()?;
        let aqh = AccelQueryHelper::new(&self.printer)?;
        let a = aqh.clone();
        let bb = self.batch_bulk.borrow().clone().ok_or_else(|| KError::Other("mpu9250 batch helper missing".into()))?;
        bb.add_client(Rc::new(move |msg| a.handle_batch(msg)))?;
        Ok(aqh)
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        let params = self.i2c.i2c_read(&[reg as u8], 1, true)?;
        match params.get("response") {
            Some(Val::Bytes(b)) if !b.is_empty() => Ok(b[0] as i64),
            _ => Err(KError::Other("bytearray index out of range".into())),
        }
    }

    fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        self.set_reg_at(reg, val, 0)
    }

    fn chip_name(&self) -> String {
        self.name.clone()
    }
}

impl PrinterObject for MPU9250 {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        if let Some(bb) = self.batch_bulk.borrow_mut().take() {
            bb.teardown();
        }
        *self.cmd_helper.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(MPU9250::new(config)?)
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(MPU9250::new(config)?)
}
