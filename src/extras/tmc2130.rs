// TMC2130 configuration (port of klippy/extras/tmc2130.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::extras::tmc::{
    fields_from_table, tmc_stealthchop_helper, tmc_vcoolthrs_helper, tmc_vhigh_helper, tmc_wave_table_helper, BaseTMCCurrentHelper,
    CurrentInfo, FieldHelper, RawRead, TMCCommandHelper, TMCVirtualPinHelper, TmcCurrent, TmcDriver, TmcMcu, TmcMcuRef,
};
use crate::mcu::Mcu;
use crate::msgproto::{Params, Val};
use crate::pins::PinsObject;
use crate::printer::{PObj, Printer};
use crate::pyfmt::py_min;
use crate::reactor::ReactorMutex;
use crate::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

pub use crate::extras::tmc_tables::tmc2130::*;

pub fn field_formatters(field: &str, v: i64) -> Option<String> {
    let flag = |s: &str| Some(if v != 0 { s.to_string() } else { String::new() });
    match field {
        "i_scale_analog" => flag("1(ExtVREF)"),
        "shaft" => flag("1(Reverse)"),
        "reset" => flag("1(Reset)"),
        "drv_err" => flag("1(ErrorShutdown!)"),
        "uv_cp" => flag("1(Undervoltage!)"),
        "version" => Some(format!("{:#x}", v)),
        "mres" => Some(format!("{}({}usteps)", v, 0x100i64.checked_shr(v as u32).unwrap_or(0))),
        "otpw" => flag("1(OvertempWarning!)"),
        "ot" => flag("1(OvertempError!)"),
        "s2ga" => flag("1(ShortToGND_A!)"),
        "s2gb" => flag("1(ShortToGND_B!)"),
        "ola" => flag("1(OpenLoad_A!)"),
        "olb" => flag("1(OpenLoad_B!)"),
        "cs_actual" => Some(if v != 0 { v.to_string() } else { "0(Reset?)".to_string() }),
        _ => None,
    }
}

// ----------------------------------------------------------------------
// TMC stepper current config helper
// ----------------------------------------------------------------------

pub struct TMC2130CurrentHelper {
    base: BaseTMCCurrentHelper,
}

impl TMC2130CurrentHelper {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<Self> {
        let base = BaseTMCCurrentHelper::new(config, mcu_tmc, MAX_CURRENT, true)?;
        let h = TMC2130CurrentHelper { base };
        let (vsense, irun, ihold) = h.calc_current(h.base.req_run_current.get(), h.base.req_hold_current.get());
        h.base.fields.set_field("vsense", vsense as i64)?;
        h.base.fields.set_field("ihold", ihold)?;
        h.base.fields.set_field("irun", irun)?;
        Ok(h)
    }

    pub fn calc_current_bits(&self, current: f64, vsense: bool) -> i64 {
        let sense_resistor = self.base.sense_resistor + 0.020;
        let vref = if vsense { 0.18 } else { 0.32 };
        let cs = (32.0 * sense_resistor * current * 2f64.sqrt() / vref + 0.5).trunc() as i64 - 1;
        0.max(31.min(cs))
    }

    pub fn calc_current_from_bits(&self, cs: i64, vsense: bool) -> f64 {
        let sense_resistor = self.base.sense_resistor + 0.020;
        let vref = if vsense { 0.18 } else { 0.32 };
        (cs + 1) as f64 * vref / (32.0 * sense_resistor * 2f64.sqrt())
    }

    pub fn calc_current(&self, run_current: f64, hold_current: f64) -> (bool, i64, i64) {
        let mut vsense = true;
        let mut irun = self.calc_current_bits(run_current, true);
        if irun == 31 {
            let cur = self.calc_current_from_bits(irun, true);
            if cur < run_current {
                let irun2 = self.calc_current_bits(run_current, false);
                let cur2 = self.calc_current_from_bits(irun2, false);
                if (run_current - cur2).abs() < (run_current - cur).abs() {
                    vsense = false;
                    irun = irun2;
                }
            }
        }
        let ihold = self.calc_current_bits(py_min(hold_current, run_current), vsense);
        (vsense, irun, ihold)
    }
}

impl TmcCurrent for TMC2130CurrentHelper {
    fn base(&self) -> &BaseTMCCurrentHelper {
        &self.base
    }

    fn get_current(&self) -> CurrentInfo {
        let f = &self.base.fields;
        let irun = f.get_field("irun");
        let ihold = f.get_field("ihold");
        let vsense = f.get_field("vsense") != 0;
        let run_current = self.calc_current_from_bits(irun, vsense);
        let hold_current = self.calc_current_from_bits(ihold, vsense);
        (run_current, Some(hold_current), self.base.req_hold_current.get(), MAX_CURRENT, self.base.req_home_current.get())
    }

    fn apply_current(&self, print_time: Option<f64>) -> KResult<()> {
        let b = &self.base;
        let (vsense, irun, ihold) = self.calc_current(b.actual_current.get(), b.req_hold_current.get());
        if vsense as i64 != b.fields.get_field("vsense") {
            let val = b.fields.set_field("vsense", vsense as i64)?;
            b.mcu_tmc.set_register("CHOPCONF", val, print_time)?;
        }
        b.fields.set_field("ihold", ihold)?;
        let val = b.fields.set_field("irun", irun)?;
        b.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)
    }
}

// ----------------------------------------------------------------------
// TMC2130 SPI
// ----------------------------------------------------------------------

pub struct McuTmcSpiChain {
    pub printer: Rc<Printer>,
    pub chain_len: i64,
    pub mutex: Rc<ReactorMutex>,
    pub spi: Rc<McuSpi>,
    pub taken_chain_positions: RefCell<Vec<i64>>,
}

impl McuTmcSpiChain {
    pub fn new(config: &ConfigWrapper, chain_len: i64) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mutex = printer.get_reactor().mutex(false);
        let share = if chain_len > 1 { Some("tmc_spi_cs") } else { None };
        let spi = mcu_spi_from_config(config, 3, "cs_pin", 4000000, share, false)?;
        Ok(Rc::new(McuTmcSpiChain { printer, chain_len, mutex, spi, taken_chain_positions: RefCell::new(vec![]) }))
    }

    pub fn build_cmd(&self, data: &[u8], chain_pos: i64) -> Vec<u8> {
        let mut cmd = vec![0u8; ((self.chain_len - chain_pos) * 5) as usize];
        cmd.extend_from_slice(data);
        cmd.extend(std::iter::repeat_n(0u8, ((chain_pos - 1) * 5) as usize));
        cmd
    }

    /// the 5 bytes of a response at chain_pos
    fn chain_slice(&self, params: &Params, chain_pos: i64) -> Vec<u8> {
        let pr = match params.get("response") {
            Some(Val::Bytes(b)) => b.clone(),
            _ => vec![],
        };
        let start = (((self.chain_len - chain_pos) * 5) as usize).min(pr.len());
        let end = (((self.chain_len - chain_pos + 1) * 5) as usize).min(pr.len());
        pr[start..end].to_vec()
    }

    fn word(pr: &[u8]) -> KResult<i64> {
        if pr.len() < 5 {
            return Err(KError::Other("bytearray index out of range".into()));
        }
        Ok(((pr[1] as i64) << 24) | ((pr[2] as i64) << 16) | ((pr[3] as i64) << 8) | pr[4] as i64)
    }

    pub fn reg_read(&self, reg: i64, chain_pos: i64) -> KResult<RawRead> {
        let cmd = self.build_cmd(&[reg as u8, 0x00, 0x00, 0x00, 0x00], chain_pos);
        self.spi.spi_send(&cmd, 0, 0)?;
        if self.printer.start_arg("debugoutput").is_some() {
            return Ok(RawRead { data: 0, receive_time: 0.0, spi_status: Some(0) });
        }
        let params = self.spi.spi_transfer(&cmd, 0, 0)?;
        let pr = self.chain_slice(&params, chain_pos);
        let receive_time = match params.get("#receive_time") {
            Some(Val::Float(t)) => *t,
            _ => 0.0,
        };
        Ok(RawRead { data: Self::word(&pr)?, receive_time, spi_status: Some(pr[0] as i64) })
    }

    pub fn reg_write(&self, reg: i64, val: i64, chain_pos: i64, print_time: Option<f64>) -> KResult<i64> {
        let mut minclock = 0;
        if let Some(pt) = print_time {
            minclock = self.spi.get_mcu().print_time_to_clock(pt);
        }
        let data = [((reg | 0x80) & 0xFF) as u8, (val >> 24) as u8, (val >> 16) as u8, (val >> 8) as u8, val as u8];
        if self.printer.start_arg("debugoutput").is_some() {
            self.spi.spi_send(&self.build_cmd(&data, chain_pos), minclock as u64, 0)?;
            return Ok(val);
        }
        let write_cmd = self.build_cmd(&data, chain_pos);
        let dummy_read = self.build_cmd(&[0x00, 0x00, 0x00, 0x00, 0x00], chain_pos);
        let params = self.spi.spi_transfer_with_preface(&write_cmd, &dummy_read, minclock as u64, 0)?;
        let pr = self.chain_slice(&params, chain_pos);
        Self::word(&pr)
    }
}

// Helper to setup an spi daisy chain bus from settings in a config section
pub fn lookup_tmc_spi_chain(config: &ConfigWrapper) -> KResult<(Rc<McuTmcSpiChain>, i64)> {
    let Some(chain_len) = config.getint_opt("chain_length", Lim::new().min(2))? else {
        // Simple, non daisy chained SPI connection
        return Ok((McuTmcSpiChain::new(config, 1)?, 1));
    };
    // Shared SPI bus - lookup existing MCU_TMC_SPI_chain
    let printer = config.get_printer();
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let cs_pin = config.get("cs_pin", Dflt::Required)?;
    let cs_pin_params =
        ppins.0.borrow_mut().lookup_pin(&cs_pin, false, false, Some("tmc_spi_cs")).map_err(|e| KError::Pins(e.0))?;
    let existing = cs_pin_params.borrow().class.clone();
    let tmc_spi = match existing {
        Some(c) => c.downcast::<McuTmcSpiChain>().map_err(|_| KError::Other("tmc spi pin class has an unexpected type".into()))?,
        None => {
            let c = McuTmcSpiChain::new(config, chain_len)?;
            cs_pin_params.borrow_mut().class = Some(c.clone());
            c
        }
    };
    if chain_len != tmc_spi.chain_len {
        return Err(KError::config("TMC SPI chain must have same length"));
    }
    let chain_pos = config.getint("chain_position", Dflt::Required, Lim::new().min(1).max(chain_len))?;
    if tmc_spi.taken_chain_positions.borrow().contains(&chain_pos) {
        return Err(KError::config("TMC SPI chain can not have duplicate position"));
    }
    tmc_spi.taken_chain_positions.borrow_mut().push(chain_pos);
    Ok((tmc_spi, chain_pos))
}

// Helper code for working with TMC devices via SPI
pub struct McuTmcSpi {
    pub name: String,
    pub tmc_spi: Rc<McuTmcSpiChain>,
    pub chain_pos: i64,
    pub name_to_reg: IndexMap<String, i64>,
    pub fields: Rc<FieldHelper>,
    pub tmc_frequency: f64,
}

impl McuTmcSpi {
    pub fn new(config: &ConfigWrapper, name_to_reg: &[(&str, i64)], fields: Rc<FieldHelper>, tmc_frequency: f64) -> KResult<Rc<Self>> {
        let (tmc_spi, chain_pos) = lookup_tmc_spi_chain(config)?;
        Ok(Rc::new(McuTmcSpi {
            name: config.get_name().split_whitespace().last().unwrap_or("").to_string(),
            tmc_spi,
            chain_pos,
            name_to_reg: name_to_reg.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
            fields,
            tmc_frequency,
        }))
    }

    fn reg(&self, reg_name: &str) -> KResult<i64> {
        self.name_to_reg.get(reg_name).copied().ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(reg_name)))
    }

    fn with_mutex<R>(&self, f: impl FnOnce() -> KResult<R>) -> KResult<R> {
        self.tmc_spi.mutex.lock()?;
        let r = f();
        self.tmc_spi.mutex.unlock();
        r
    }
}

impl TmcMcu for McuTmcSpi {
    fn get_fields(&self) -> Rc<FieldHelper> {
        self.fields.clone()
    }

    fn tmc_spi(&self) -> Option<(Rc<McuTmcSpiChain>, i64)> {
        Some((self.tmc_spi.clone(), self.chain_pos))
    }

    fn get_register_raw(&self, reg_name: &str) -> KResult<RawRead> {
        let reg = self.reg(reg_name)?;
        self.with_mutex(|| self.tmc_spi.reg_read(reg, self.chain_pos))
    }

    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        let reg = self.reg(reg_name)?;
        self.with_mutex(|| {
            for _retry in 0..5 {
                let v = self.tmc_spi.reg_write(reg, val, self.chain_pos, print_time)?;
                if v == val {
                    return Ok(());
                }
            }
            Err(KError::command(format!("Unable to write tmc spi '{}' register {}", self.name, reg_name)))
        })
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(self.tmc_frequency)
    }

    fn get_mcu(&self) -> Rc<Mcu> {
        self.tmc_spi.spi.get_mcu()
    }

    fn name_to_reg(&self, reg_name: &str) -> i64 {
        self.name_to_reg.get(reg_name).copied().unwrap_or(0)
    }
}

// ----------------------------------------------------------------------
// TMC2130 printer object
// ----------------------------------------------------------------------

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    // Setup mcu communication
    let fields = Rc::new(FieldHelper::new(fields_from_table(FIELDS), SIGNED_FIELDS, Some(field_formatters)));
    let mcu_tmc: TmcMcuRef = McuTmcSpi::new(config, REGISTERS, fields.clone(), TMC_FREQUENCY)?;
    // Allow virtual pins to be created
    TMCVirtualPinHelper::new(config, &mcu_tmc)?;
    // Register commands
    let current_helper = Rc::new(TMC2130CurrentHelper::new(config, &mcu_tmc)?);
    let cmdhelper = TMCCommandHelper::new(config, &mcu_tmc, current_helper, None, None)?;
    cmdhelper.setup_register_dump(READ_REGISTERS, None)?;
    // Setup basic register values
    tmc_wave_table_helper(config, &mcu_tmc)?;
    tmc_stealthchop_helper(config, &mcu_tmc)?;
    tmc_vcoolthrs_helper(config, &mcu_tmc)?;
    tmc_vhigh_helper(config, &mcu_tmc)?;
    // Allow other registers to be set from the config
    let set = |f: &str, d: i64| fields.set_config_field(config, f, d);
    // CHOPCONF
    set("toff", 4)?;
    set("hstrt", 0)?;
    set("hend", 7)?;
    set("tbl", 1)?;
    set("vhighfs", 0)?;
    set("vhighchm", 0)?;
    // COOLCONF
    set("semin", 0)?;
    set("seup", 0)?;
    set("semax", 0)?;
    set("sedn", 0)?;
    set("seimin", 0)?;
    set("sgt", 0)?;
    set("sfilt", 0)?;
    // IHOLDIRUN
    set("iholddelay", 8)?;
    // PWMCONF
    set("pwm_ampl", 128)?;
    set("pwm_grad", 4)?;
    set("pwm_freq", 1)?;
    fields.set_config_field(config, "pwm_autoscale", true)?;
    set("freewheel", 0)?;
    // TPOWERDOWN
    set("tpowerdown", 0)?;
    Ok(Rc::new(TmcDriver { fields, mcu_tmc, cmdhelper, class: "TMC2130" }))
}
