// ADS131Mx Support (port of klippy/extras/load_cell/ads131m0x.py)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// ADS131M0x Chip Documentation:
// ADS131M02 - https://www.ti.com/lit/ds/symlink/ads131m02.pdf
// ADS131M04 - https://www.ti.com/lit/ds/symlink/ads131m04.pdf
use super::{adc_batch, batch_helper, lookup_sensor_pin, no_cmd, BulkAdcSensor, ADC_FACTOR};
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, ClientCallback, FixedFreqReader};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::logging;
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::printer::Printer;
use crate::pyfmt::py_round;
use crate::pyval::PyVal;
use crate::reactor::Reactor;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

// Constants
const WORD_SIZE: usize = 3; // there are 3 bytes in a word in the protocol
const UPDATE_INTERVAL: f64 = 0.10;
const NULL_CMD: i64 = 0x0000;
const RESET_CMD: i64 = 0x0011;
const RREG_CMD: i64 = 0b101 << 13;
const WREG_CMD: i64 = 0b011 << 13;
const STATUS_REG: i64 = 0x01;
const MODE_REG: i64 = 0x02;
const CLOCK_REG: i64 = 0x03;
const GAIN1_REG: i64 = 0x04;
const CFG_REG: i64 = 0x06;
const PWR_HR: i64 = 0b10; // High resolution mode
const GLOBAL_CHOP_ENABLED: i64 = 0b1 << 8;
const GLOBAL_CHOP_DELAY: [(i64, i64); 16] = [
    (2, 0b0000 << 9),
    (4, 0b0001 << 9),
    (8, 0b0010 << 9),
    (16, 0b0011 << 9),
    (32, 0b0100 << 9),
    (64, 0b0101 << 9),
    (128, 0b0110 << 9),
    (256, 0b0111 << 9),
    (512, 0b1000 << 9),
    (1024, 0b1001 << 9),
    (2048, 0b1010 << 9),
    (4096, 0b1011 << 9),
    (8192, 0b1100 << 9),
    (16384, 0b1101 << 9),
    (32768, 0b1110 << 9),
    (65536, 0b1111 << 9),
];
// Error codes from MCU (match sensor_ads131m02.c)
const SAMPLE_ERROR_CRC: i64 = -0x80000000; // 1 << 31 as signed
const SAMPLE_ERROR_RESET: i64 = 0x40000000; // 1 << 30
const GAIN_OPTIONS: [(i64, i64); 8] = [(1, 0), (2, 1), (4, 2), (8, 3), (16, 4), (32, 5), (64, 6), (128, 7)];
// SPS -> Oversampling Ration (OSR) code: OSR=fMOD/fDATA, fMOD=fCLKIN/2=4.096MHz
const OSR_LOOKUP: [(i64, i64); 8] = [
    (32000, 0 << 2), // OSR=128
    (16000, 1 << 2), // OSR=256
    (8000, 2 << 2),  // OSR=512
    (4000, 3 << 2),  // OSR=1024
    (2000, 4 << 2),  // OSR=2048
    (1000, 5 << 2),  // OSR=4096
    (500, 6 << 2),   // OSR=8192
    (250, 7 << 2),   // OSR=16384
];

fn lookup(table: &[(i64, i64)], key: i64) -> i64 {
    table.iter().find(|(k, _)| *k == key).map(|(_, v)| *v).unwrap_or(0)
}

pub fn channels_to_mask(channels: &[i64]) -> i64 {
    channels.iter().fold(0, |mask, channel| mask | (1 << channel))
}

pub struct ADS131MxBase {
    pub printer: Rc<Printer>,
    pub reactor: Rc<Reactor>,
    pub name: String,
    pub sensor_type: &'static str,
    pub channel_count: usize,
    pub error_count: Cell<i64>,
    pub consecutive_fails: Cell<i64>,
    pub sps: i64,
    pub gain: i64,
    pub enable_global_chop: bool,
    pub global_chop_delay: i64,
    pub spi: Rc<McuSpi>,
    pub mcu: Rc<Mcu>,
    pub oid: i64,
    pub data_ready_pin: String,
    pub channels: Vec<i64>,
    pub channel_mask: i64,
    pub ffreader: Rc<FixedFreqReader>,
    pub batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
    pub attach_probe_cmd: RefCell<Option<CommandWrapper>>,
    pub query_ads131m0x_cmd: RefCell<Option<CommandWrapper>>,
}

impl ADS131MxBase {
    fn new(config: &ConfigWrapper, sensor_type: &'static str, default_sample_rate: i64, channel_count: usize) -> KResult<Rc<dyn BulkAdcSensor>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or_default().to_string();
        let sample_rate_options: Vec<(i64, i64)> = OSR_LOOKUP.iter().map(|(k, _)| (*k, *k)).collect();
        let sps = config.getchoice_int("sample_rate", &sample_rate_options, default_sample_rate)?;
        let gain = config.getchoice_int("gain", &GAIN_OPTIONS, 128)?;
        let enable_global_chop = config.getboolean("enable_global_chop", false)?;
        let chop_settings: Vec<(i64, i64)> = GLOBAL_CHOP_DELAY.iter().map(|(k, _)| (*k, *k)).collect();
        let global_chop_delay = config.getchoice_int("global_chop_delay", &chop_settings, 16)?;
        let spi = mcu_spi_from_config(config, 1, "cs_pin", 8192000, None, false)?;
        let mcu = spi.get_mcu();
        let oid = mcu.create_oid();
        // Data Ready (DRDY) Pin
        let (drdy_mcu, data_ready_pin) = lookup_sensor_pin(config, "data_ready_pin")?;
        if !Rc::ptr_eq(&drdy_mcu, &mcu) {
            return Err(KError::config(format!("{} config error: SPI communication and data_ready_pin must be on the same MCU", sensor_type)));
        }
        let channels = read_channels(config, sensor_type, channel_count)?;
        let channel_mask = channels_to_mask(&channels);
        // Bulk Sensor Setup
        let samples_per_second = if enable_global_chop { (sps as f64 / 3.0).floor() as i64 } else { sps };
        let chip_smooth = samples_per_second as f64 * UPDATE_INTERVAL * 2.0;
        let unpack_format = format!("<{}", "i".repeat(channels.len()));
        let ffreader = Rc::new(FixedFreqReader::new(&mcu, chip_smooth, &unpack_format));
        let s = Rc::new(ADS131MxBase {
            printer: printer.clone(),
            reactor: printer.get_reactor(),
            name,
            sensor_type,
            channel_count,
            error_count: Cell::new(0),
            consecutive_fails: Cell::new(0),
            sps,
            gain,
            enable_global_chop,
            global_chop_delay,
            spi,
            mcu: mcu.clone(),
            oid,
            data_ready_pin,
            channels,
            channel_mask,
            ffreader,
            batch_bulk: RefCell::new(None),
            attach_probe_cmd: RefCell::new(None),
            query_ads131m0x_cmd: RefCell::new(None),
        });
        *s.batch_bulk.borrow_mut() = Some(batch_helper(
            &printer,
            &s,
            ADS131MxBase::process_batch,
            ADS131MxBase::start_measurements,
            ADS131MxBase::finish_measurements,
            UPDATE_INTERVAL,
        ));
        let w = Rc::downgrade(&s);
        mcu.register_config_callback(Rc::new(move || match w.upgrade() {
            Some(s) => s.build_config(),
            None => Ok(()),
        }));
        Ok(s)
    }

    pub fn build_config(&self) -> KResult<()> {
        let cq = self.spi.get_command_queue();
        self.mcu.add_config_cmd(
            &format!(
                "config_ads131m0x oid={} spi_oid={} data_ready_pin={} sensor_channel_count={} channel_mask={}",
                self.oid,
                self.spi.get_oid(),
                self.data_ready_pin,
                self.channel_count,
                self.channel_mask
            ),
            false,
            false,
        );
        self.mcu.add_config_cmd(&format!("query_ads131m0x oid={} rest_ticks=0", self.oid), false, true);
        *self.query_ads131m0x_cmd.borrow_mut() = Some(self.mcu.lookup_command("query_ads131m0x oid=%c rest_ticks=%u", Some(cq.clone()))?);
        *self.attach_probe_cmd.borrow_mut() =
            Some(self.mcu.lookup_command("ads131m0x_attach_load_cell_probe oid=%c load_cell_probe_oid=%c", None)?);
        self.ffreader.setup_query_command("query_ads131m0x_status oid=%c", self.oid, Some(cq))
    }

    // Measurement decoding: the samples and the batch's errors
    pub fn convert_samples(&self, samples: &[(f64, Vec<i64>)]) -> (Vec<PyVal>, i64) {
        let mut batch_errors = 0;
        let mut samples_out = vec![];
        for (ptime, channel_counts) in samples {
            let val = channel_counts[0];
            if val == SAMPLE_ERROR_CRC || val == SAMPLE_ERROR_RESET {
                batch_errors += 1;
                logging::error(&format!("{} sample error: {}", self.sensor_type, channel_counts[0]));
                break;
            }
            let mut converted_sample = vec![PyVal::Float(py_round(*ptime, 6))];
            for channel in channel_counts {
                converted_sample.push(PyVal::Int(*channel));
                converted_sample.push(PyVal::Float(*channel as f64 * ADC_FACTOR));
            }
            samples_out.push(PyVal::List(converted_sample));
        }
        (samples_out, batch_errors)
    }

    pub fn start_measurements(&self) -> KResult<()> {
        self.consecutive_fails.set(0);
        self.reset_chip()?;
        self.setup_chip()?;
        let rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.get_samples_per_second() as f64));
        self.query_ads131m0x_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send(&[Val::Int(self.oid), Val::Int(rest_ticks)], 0, 0)?;
        logging::info(&format!("{} starting '{}' measurements", self.sensor_type, self.name));
        self.ffreader.note_start()
    }

    pub fn finish_measurements(&self) -> KResult<()> {
        if self.printer.is_shutdown() {
            return Ok(());
        }
        self.query_ads131m0x_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send_wait_ack(&[Val::Int(self.oid), Val::Int(0)], 0, 0)?;
        self.ffreader.note_end()?;
        logging::info(&format!("{} finished '{}' measurements", self.sensor_type, self.name));
        Ok(())
    }

    pub fn process_batch(&self, _eventtime: f64) -> KResult<PyVal> {
        let (raw_samples, overflows) = self.ffreader.pull_samples()?;
        let (samples, batch_errors) = self.convert_samples(&raw_samples);
        self.error_count.set(self.error_count.get() + batch_errors);
        Ok(adc_batch(samples, batch_errors, overflows))
    }

    // --- SPI Communication Helpers ---
    // The ADS131M02 uses 24-bit SPI words. Commands and register data are
    // 16-bit values, MSB-aligned with zero padding: [MSB, LSB, 0x00]
    fn to_words(values_16bit: &[i64]) -> Vec<u8> {
        values_16bit.iter().flat_map(|val| [((val >> 8) & 0xFF) as u8, (val & 0xFF) as u8, 0x00]).collect()
    }

    // Send a frame of 16-bit values as 24-bit words.
    pub fn send_command(&self, cmd_16bit: i64) -> KResult<()> {
        let mut words = vec![cmd_16bit];
        words.extend(std::iter::repeat_n(NULL_CMD, 1 + self.channel_count));
        self.spi.spi_send_wait_ack(&Self::to_words(&words), 0, 0)
    }

    // Send frame and return response as list of 16-bit values.
    fn transfer_frame(&self, values_16bit: &[i64]) -> KResult<Vec<i64>> {
        // send 2 frames (8x24 bits), the response is in the second frame
        let msg_words = 2 + self.channel_count;
        let frame_size = 2 * msg_words * WORD_SIZE;
        let mut transfer_words = Self::to_words(values_16bit);
        // pad to target size with 0's
        while transfer_words.len() < frame_size {
            transfer_words.extend(Self::to_words(&[NULL_CMD]));
        }
        let params = self.spi.spi_transfer(&transfer_words, 0, 0)?;
        let resp: &[u8] = match params.get("response") {
            Some(Val::Bytes(b)) => b,
            _ => &[],
        };
        let mut result = vec![];
        for i in (0..resp.len()).step_by(3) {
            if i + 1 < resp.len() {
                result.push(((resp[i] as i64) << 8) | resp[i + 1] as i64);
            }
        }
        Ok(result)
    }

    // --- Command Helpers ---
    // WREG: 011a_aaaa_annn_nnnn (a=6-bit address, n=7-bit count-1)
    // RREG: 101a_aaaa_annn_nnnn

    fn build_register_command(cmd: i64, addr: i64, count: i64) -> i64 {
        cmd | ((addr & 0x3F) << 7) | ((count - 1) & 0x7F)
    }

    pub fn read_reg(&self, addr: i64) -> KResult<i64> {
        let msg_words = 2 + self.channel_count;
        let resp = self.transfer_frame(&[Self::build_register_command(RREG_CMD, addr, 1)])?;
        if resp.len() < msg_words + 1 {
            return Err(KError::command(format!("{} {}: no response reading reg 0x{:02x}", self.sensor_type, self.name, addr)));
        }
        Ok(resp[msg_words])
    }

    // Write a single register.
    pub fn write_reg(&self, addr: i64, value: i64) -> KResult<()> {
        self.transfer_frame(&[Self::build_register_command(WREG_CMD, addr, 1), value])?;
        Ok(())
    }

    // Write a register and verify the value was set.
    pub fn write_and_verify_reg(&self, addr: i64, value: i64) -> KResult<()> {
        self.write_reg(addr, value)?;
        let actual = self.read_reg(addr)?;
        if actual != value {
            return Err(KError::command(format!(
                "{} {}: reg 0x{:02x} write failed: wrote 0x{:04x}, read 0x{:04x}",
                self.sensor_type, self.name, addr, value, actual
            )));
        }
        Ok(())
    }

    pub fn reset_chip(&self) -> KResult<()> {
        self.send_command(RESET_CMD)?;
        self.reactor.delay_for_ms(20.0)?;
        let status = self.read_reg(STATUS_REG)?;
        logging::info(&format!("{} {}: reset complete, STATUS=0x{:04x}", self.sensor_type, self.name, status));
        Ok(())
    }

    pub fn gain_register_value(&self) -> i64 {
        // The gain setting for each channel is 3 bits, followed by a reserved
        // bits. Starting with Channel 0 is at the LSB.
        (0..self.channel_count).fold(0, |gain_val, channel| gain_val | (self.gain << (channel * 4)))
    }

    pub fn setup_chip(&self) -> KResult<()> {
        // MODE register (0x02): clear RESET bit (bit 10), set 24-bit word length (bits 9:8 = 01)
        // Reset default is 0x0510: RESET=1, WLENGTH=01, TIMEOUT=1
        let wlength_24 = 0b01 << 8;
        let timeout_en = 1 << 4;
        let mode_write_val = wlength_24 | timeout_en;
        self.write_reg(MODE_REG, mode_write_val)?;
        let mode_val = self.read_reg(MODE_REG)?;
        if (mode_val & 0x0300) != wlength_24 {
            return Err(KError::command(format!("{} {}: MODE reg write failed: got 0x{:04x}", self.sensor_type, self.name, mode_val)));
        }
        // CLOCK register (0x03): set OSR for sample rate, high resolution mode
        // OSR[2:0] at bits 4:2, PWR[1:0] at bits 1:0
        // PWR=10 for high-resolution mode
        let channel_enable = self.channel_mask << 8;
        let osr_code = lookup(&OSR_LOOKUP, self.sps);
        let clock_val = channel_enable | osr_code | PWR_HR;
        self.write_and_verify_reg(CLOCK_REG, clock_val)?;
        self.write_and_verify_reg(GAIN1_REG, self.gain_register_value())?;
        // CFG register settings for global chop mode, off by default:
        let mut cfg_val = lookup(&GLOBAL_CHOP_DELAY, self.global_chop_delay);
        if self.enable_global_chop {
            cfg_val |= GLOBAL_CHOP_ENABLED;
        }
        self.write_and_verify_reg(CFG_REG, cfg_val)?;
        // pause for settings to take effect
        self.reactor.delay_for_ms(50.0)?;
        let status = self.read_reg(STATUS_REG)?;
        logging::info(&format!("{} {}: post-WAKEUP: STATUS=0x{:04x}", self.sensor_type, self.name, status));
        Ok(())
    }
}

fn read_channels(config: &ConfigWrapper, sensor_type: &str, channel_count: usize) -> KResult<Vec<i64>> {
    let default = crate::pyval::PyVal::Tuple(vec![crate::pyval::PyVal::Int(0)]);
    let mut channels = config.getintlist("channels", default, ",", None)?.unwrap_or_default();
    channels.sort();
    channels.dedup();
    let max_channel = channel_count as i64 - 1;
    for channel in &channels {
        if *channel < 0 || *channel > max_channel {
            return Err(KError::config(format!("{} channels must be in range 0..{}", sensor_type.to_uppercase(), max_channel)));
        }
    }
    if channels.is_empty() {
        return Err(KError::config(format!("{} channels must contain at least one", sensor_type.to_uppercase())));
    }
    Ok(channels)
}

impl BulkAdcSensor for ADS131MxBase {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    fn get_samples_per_second(&self) -> i64 {
        if self.enable_global_chop {
            // global chop mode takes 3 cycles per sample
            return (self.sps as f64 / 3.0).floor() as i64;
        }
        self.sps
    }

    fn get_range(&self) -> (i64, i64) {
        // 24-bit signed range
        (-0x800000, 0x7FFFFF)
    }

    fn get_channel_count(&self) -> usize {
        self.channels.len()
    }

    fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        self.batch_bulk.borrow().clone().ok_or_else(no_cmd)?.add_client(callback)
    }

    fn attach_load_cell_probe(&self, load_cell_probe_oid: i64) -> KResult<()> {
        self.attach_probe_cmd.borrow().as_ref().ok_or_else(no_cmd)?.send(&[Val::Int(self.oid), Val::Int(load_cell_probe_oid)], 0, 0)
    }
}

pub fn new_ads131m02(config: &ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>> {
    ADS131MxBase::new(config, "ads131m02", 500, 2)
}

pub fn new_ads131m04(config: &ConfigWrapper) -> KResult<Rc<dyn BulkAdcSensor>> {
    ADS131MxBase::new(config, "ads131m04", 500, 4)
}
