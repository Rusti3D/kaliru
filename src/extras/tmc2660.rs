// TMC2660 configuration (port of klippy/extras/tmc2660.py)
//
// Copyright (C) 2018-2019  Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::{mcu_spi_from_config, McuSpi};
use crate::extras::tmc::{fields_from_table, BaseTMCCurrentHelper, CurrentInfo, FieldHelper, RawRead, TMCCommandHelper, TmcCurrent, TmcDriver, TmcMcu, TmcMcuRef};
use crate::extras::tmc2130;
use crate::logging;
use crate::mcu::Mcu;
use crate::msgproto::Val;
use crate::printer::{PObj, Printer};
use crate::reactor::{ReactorMutex, NOW};
use crate::IndexMap;
use std::cell::Cell;
use std::rc::{Rc, Weak};

pub use crate::extras::tmc_tables::tmc2660::*;

pub fn field_formatters(field: &str, v: i64) -> Option<String> {
    let pick = |a: &str, b: &str| Some(if v != 0 { a.to_string() } else { b.to_string() });
    match field {
        "chm" => pick("1(constant toff)", "0(spreadCycle)"),
        "vsense" => pick("1(165mV)", "0(305mV)"),
        "sdoff" => pick("1(Step/Dir disabled!)", ""),
        "diss2g" => pick("1(Short to GND disabled!)", ""),
        "se" => Some(if v != 0 { v.to_string() } else { "0(Reset?)".to_string() }),
        _ => tmc2130::field_formatters(field, v),
    }
}

// ----------------------------------------------------------------------
// TMC stepper current config helper
// ----------------------------------------------------------------------

pub struct TMC2660CurrentHelper {
    me: Weak<TMC2660CurrentHelper>,
    base: BaseTMCCurrentHelper,
    current: Cell<f64>,
    idle_current_percentage: i64,
}

impl TMC2660CurrentHelper {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<Rc<Self>> {
        let base = BaseTMCCurrentHelper::new(config, mcu_tmc, MAX_CURRENT, true)?;
        let current = base.req_run_current.get();
        let mut h = TMC2660CurrentHelper { me: Weak::new(), base, current: Cell::new(current), idle_current_percentage: 100 };
        let (vsense, cs) = h.calc_current(current);
        h.base.fields.set_field("cs", cs)?;
        h.base.fields.set_field("vsense", vsense as i64)?;
        // Register ready/printing handlers
        h.idle_current_percentage = config.getint("idle_current_percent", 100, Lim::new().min(0).max(100))?;
        let h = Rc::new_cyclic(|me| {
            h.me = me.clone();
            h
        });
        if h.idle_current_percentage < 100 {
            let w = h.me.clone();
            crate::events::IdleTimeoutPrinting::on(&h.base.printer, move |print_time| {
                if let Some(h) = w.upgrade() {
                    h.handle_printing(print_time);
                }
                Ok(())
            });
            let w = h.me.clone();
            crate::events::IdleTimeoutReady::on(&h.base.printer, move |print_time| {
                if let Some(h) = w.upgrade() {
                    h.handle_ready(print_time);
                }
                Ok(())
            });
        }
        Ok(h)
    }

    fn calc_current_bits(&self, current: f64, vsense: bool) -> i64 {
        let vref = if vsense { 0.165 } else { 0.310 };
        let sr = self.base.sense_resistor;
        let cs = (32.0 * sr * current * 2f64.sqrt() / vref + 0.5).trunc() as i64 - 1;
        0.max(31.min(cs))
    }

    fn calc_current_from_bits(&self, cs: i64, vsense: bool) -> f64 {
        let vref = if vsense { 0.165 } else { 0.310 };
        (cs + 1) as f64 * vref / (32.0 * self.base.sense_resistor * 2f64.sqrt())
    }

    fn calc_current(&self, run_current: f64) -> (bool, i64) {
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
        (vsense, irun)
    }

    fn schedule_update(&self, current: f64, print_time: f64) {
        let w = self.me.clone();
        self.base.printer.get_reactor().register_callback(
            move |_| {
                if let Some(h) = w.upgrade() {
                    if let Err(e) = h.update_current(current, Some(print_time)) {
                        logging::exception_err("tmc2660 current update", &e);
                    }
                }
            },
            NOW,
        );
    }

    fn handle_printing(&self, print_time: f64) {
        let print_time = print_time - 0.100; // Schedule slightly before deadline
        self.schedule_update(self.base.req_run_current.get(), print_time);
    }

    fn handle_ready(&self, print_time: f64) {
        let current = self.base.req_run_current.get() * self.idle_current_percentage as f64 / 100.0;
        self.schedule_update(current, print_time);
    }

    fn update_current(&self, current: f64, print_time: Option<f64>) -> KResult<()> {
        let b = &self.base;
        let (vsense, cs) = self.calc_current(current);
        let val = b.fields.set_field("cs", cs)?;
        b.mcu_tmc.set_register("SGCSCONF", val, print_time)?;
        // Only update VSENSE if we need to
        if vsense as i64 != b.fields.get_field("vsense") {
            let val = b.fields.set_field("vsense", vsense as i64)?;
            b.mcu_tmc.set_register("DRVCONF", val, print_time)?;
        }
        Ok(())
    }
}

impl TmcCurrent for TMC2660CurrentHelper {
    fn base(&self) -> &BaseTMCCurrentHelper {
        &self.base
    }

    /// (run, None, None, max, home): the req_hold slot is unused here
    fn get_current(&self) -> CurrentInfo {
        (self.base.req_run_current.get(), None, self.base.req_hold_current.get(), MAX_CURRENT, self.base.req_home_current.get())
    }

    fn set_current(&self, run_current: f64, _hold_current: f64, print_time: Option<f64>, force: bool) -> KResult<()> {
        if run_current == self.current.get() && !force {
            return Ok(());
        }
        self.current.set(run_current);
        self.update_current(run_current, print_time)
    }
}

// ----------------------------------------------------------------------
// TMC2660 SPI
// ----------------------------------------------------------------------

// Helper code for working with TMC2660 devices via SPI
pub struct McuTmc2660Spi {
    printer: Rc<Printer>,
    mutex: Rc<ReactorMutex>,
    spi: Rc<McuSpi>,
    name_to_reg: IndexMap<String, i64>,
    fields: Rc<FieldHelper>,
}

impl McuTmc2660Spi {
    fn new(config: &ConfigWrapper, name_to_reg: &[(&str, i64)], fields: Rc<FieldHelper>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let mutex = printer.get_reactor().mutex(false);
        let spi = mcu_spi_from_config(config, 0, "cs_pin", 4000000, None, false)?;
        Ok(Rc::new(McuTmc2660Spi {
            printer,
            mutex,
            spi,
            name_to_reg: name_to_reg.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
            fields,
        }))
    }

    fn reg(&self, reg_name: &str) -> KResult<i64> {
        self.name_to_reg.get(reg_name).copied().ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(reg_name)))
    }

    fn msg(reg: i64, val: i64) -> [u8; 3] {
        [(((val >> 16) | reg) & 0xFF) as u8, ((val >> 8) & 0xFF) as u8, (val & 0xFF) as u8]
    }
}

impl TmcMcu for McuTmc2660Spi {
    fn get_fields(&self) -> Rc<FieldHelper> {
        self.fields.clone()
    }

    fn get_register_raw(&self, reg_name: &str) -> KResult<RawRead> {
        let new_rdsel = READ_REGISTERS
            .iter()
            .position(|r| *r == reg_name)
            .ok_or_else(|| KError::Other(format!("{} is not in list", crate::pyfmt::py_str_repr(reg_name))))? as i64;
        let reg = self.reg("DRVCONF")?;
        if self.printer.start_arg("debugoutput").is_some() {
            return Ok(RawRead { data: 0, receive_time: 0.0, spi_status: None });
        }
        self.mutex.lock()?;
        let r = (|| {
            let old_rdsel = self.fields.get_field("rdsel");
            let val = self.fields.set_field("rdsel", new_rdsel)?;
            let msg = Self::msg(reg, val);
            if new_rdsel != old_rdsel {
                // Must set RDSEL value first
                self.spi.spi_send(&msg, 0, 0)?;
            }
            self.spi.spi_transfer(&msg, 0, 0)
        })();
        self.mutex.unlock();
        let params = r?;
        let pr = match params.get("response") {
            Some(Val::Bytes(b)) if b.len() >= 3 => b.clone(),
            _ => return Err(KError::Other("bytearray index out of range".into())),
        };
        let receive_time = match params.get("#receive_time") {
            Some(Val::Float(t)) => *t,
            _ => 0.0,
        };
        Ok(RawRead { data: ((pr[0] as i64) << 16) | ((pr[1] as i64) << 8) | pr[2] as i64, receive_time, spi_status: None })
    }

    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        let mut minclock = 0;
        if let Some(pt) = print_time {
            minclock = self.spi.get_mcu().print_time_to_clock(pt);
        }
        let reg = self.reg(reg_name)?;
        let msg = Self::msg(reg, val);
        self.mutex.lock()?;
        let r = self.spi.spi_send(&msg, minclock as u64, 0);
        self.mutex.unlock();
        r
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        None
    }

    fn get_mcu(&self) -> Rc<Mcu> {
        self.spi.get_mcu()
    }

    fn name_to_reg(&self, reg_name: &str) -> i64 {
        self.name_to_reg.get(reg_name).copied().unwrap_or(0)
    }
}

// ----------------------------------------------------------------------
// TMC2660 printer object
// ----------------------------------------------------------------------

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    // Setup mcu communication
    let fields = Rc::new(FieldHelper::new(fields_from_table(FIELDS), SIGNED_FIELDS, Some(field_formatters)));
    fields.set_field("sdoff", 0)?; // Access DRVCTRL in step/dir mode
    let mcu_tmc: TmcMcuRef = McuTmc2660Spi::new(config, REGISTERS, fields.clone())?;
    // Register commands
    let current_helper = TMC2660CurrentHelper::new(config, &mcu_tmc)?;
    let cmdhelper = TMCCommandHelper::new(config, &mcu_tmc, current_helper, None, None)?;
    cmdhelper.setup_register_dump(READ_REGISTERS, None)?;

    // CHOPCONF
    let set = |f: &str, d: i64| fields.set_config_field(config, f, d);
    set("tbl", 2)?;
    set("rndtf", 0)?;
    set("hdec", 0)?;
    set("chm", 0)?;
    set("hend", 3)?;
    set("hstrt", 3)?;
    set("toff", 4)?;
    if fields.get_field("chm") == 0 && (fields.get_field("hstrt") + fields.get_field("hend")) > 15 {
        return Err(KError::config("driver_HEND + driver_HSTRT must be <= 15"));
    }
    // SMARTEN
    set("seimin", 0)?;
    set("sedn", 0)?;
    set("semax", 0)?;
    set("seup", 0)?;
    set("semin", 0)?;

    // SGSCONF
    set("sfilt", 0)?;
    set("sgt", 0)?;

    // DRVCONF
    set("slph", 0)?;
    set("slpl", 0)?;
    set("diss2g", 0)?;
    set("ts2g", 3)?;
    Ok(Rc::new(TmcDriver { fields, mcu_tmc, cmdhelper }))
}
