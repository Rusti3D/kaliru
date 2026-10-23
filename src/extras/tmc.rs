// Common helper code for TMC stepper drivers (port of klippy/extras/tmc.py)
//
// Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bulk_sensor::{BatchBulkHelper, BATCH_INTERVAL};
use crate::extras::force_move::ForceMove;
use crate::extras::heaters::load_heaters;
use crate::extras::homing::HomingMove;
use crate::extras::stepper_enable::PrinterStepperEnable;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::mcu::{any_to_endstop, EndstopRef, Mcu};
use crate::pins::{PinChip, PinsObject, SharedPinParams};
use crate::printer::{downcast_rc, Printer};
use crate::pyfmt::{py_bool, py_float_repr, py_min, py_round};
use crate::pyval::PyVal;
use crate::reactor::{ReactorMutex, Timer, NEVER, NOW};
use crate::stepper::{parse_step_distance, McuStepper, TmcCurrentHelper};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

// ----------------------------------------------------------------------
// Field helpers
// ----------------------------------------------------------------------

// Return the position of the first bit set in a mask
pub fn ffs(mask: i64) -> u32 {
    if mask == 0 {
        // (0 & -0).bit_length() - 1
        return u32::MAX;
    }
    mask.trailing_zeros()
}

/// python int.bit_length()
fn bit_length(v: i64) -> u32 {
    64 - v.unsigned_abs().leading_zeros()
}

/// FieldFormatters: field -> pretty value (None: str(v))
pub type FieldFormatter = fn(&str, i64) -> Option<String>;

pub struct FieldHelper {
    pub all_fields: IndexMap<String, IndexMap<String, i64>>,
    signed_fields: Vec<String>,
    field_formatters: FieldFormatter,
    pub registers: RefCell<IndexMap<String, i64>>,
    field_to_register: IndexMap<String, String>,
}

/// a generated table as the all_fields dict
pub fn fields_from_table(t: &[(&str, &[(&str, i64)])]) -> IndexMap<String, IndexMap<String, i64>> {
    t.iter().map(|(r, fs)| (r.to_string(), fs.iter().map(|(f, m)| (f.to_string(), *m)).collect())).collect()
}

fn no_formatters(_f: &str, _v: i64) -> Option<String> {
    None
}

impl FieldHelper {
    pub fn new(all_fields: IndexMap<String, IndexMap<String, i64>>, signed_fields: &[&str], field_formatters: Option<FieldFormatter>) -> Self {
        let mut field_to_register = IndexMap::new();
        for (r, fields) in &all_fields {
            for f in fields.keys() {
                field_to_register.insert(f.clone(), r.clone());
            }
        }
        FieldHelper {
            all_fields,
            signed_fields: signed_fields.iter().map(|s| s.to_string()).collect(),
            field_formatters: field_formatters.unwrap_or(no_formatters),
            registers: RefCell::new(IndexMap::new()),
            field_to_register,
        }
    }

    pub fn lookup_register(&self, field_name: &str) -> Option<String> {
        self.field_to_register.get(field_name).cloned()
    }

    fn reg_of(&self, field_name: &str) -> KResult<String> {
        self.lookup_register(field_name).ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(field_name)))
    }

    fn mask_of(&self, reg_name: &str, field_name: &str) -> KResult<i64> {
        self.all_fields
            .get(reg_name)
            .and_then(|f| f.get(field_name))
            .copied()
            .ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(field_name)))
    }

    /// get_field(field_name, reg_value=None, reg_name=None)
    pub fn get_field_full(&self, field_name: &str, reg_value: Option<i64>, reg_name: Option<&str>) -> KResult<i64> {
        // Returns value of the register field
        let reg_name = match reg_name {
            Some(r) => r.to_string(),
            None => self.reg_of(field_name)?,
        };
        let reg_value = reg_value.unwrap_or_else(|| self.registers.borrow().get(&reg_name).copied().unwrap_or(0));
        let mask = self.mask_of(&reg_name, field_name)?;
        let mut field_value = (reg_value & mask) >> ffs(mask);
        if self.signed_fields.iter().any(|f| f == field_name) && ((reg_value & mask) << 1) > mask {
            field_value -= 1 << bit_length(field_value);
        }
        Ok(field_value)
    }

    pub fn get_field(&self, field_name: &str) -> i64 {
        self.get_field_full(field_name, None, None).unwrap_or(0)
    }

    pub fn get_field_of(&self, field_name: &str, reg_value: i64) -> i64 {
        self.get_field_full(field_name, Some(reg_value), None).unwrap_or(0)
    }

    /// set_field(field_name, field_value, reg_value=None, reg_name=None)
    pub fn set_field_full(&self, field_name: &str, field_value: i64, reg_value: Option<i64>, reg_name: Option<&str>) -> KResult<i64> {
        // Returns register value with field bits filled with supplied value
        let reg_name = match reg_name {
            Some(r) => r.to_string(),
            None => self.reg_of(field_name)?,
        };
        let reg_value = reg_value.unwrap_or_else(|| self.registers.borrow().get(&reg_name).copied().unwrap_or(0));
        let mask = self.mask_of(&reg_name, field_name)?;
        let new_value = (reg_value & !mask) | ((field_value << ffs(mask)) & mask);
        self.registers.borrow_mut().insert(reg_name, new_value);
        Ok(new_value)
    }

    pub fn set_field(&self, field_name: &str, field_value: i64) -> KResult<i64> {
        self.set_field_full(field_name, field_value, None, None)
    }

    /// set_config_field(config, field_name, default); a None default and
    /// a missing option leave the field alone. The default keeps its python
    /// type (True vs 1), as config access tracking records it.
    pub fn set_config_field(&self, config: &ConfigWrapper, field_name: &str, default: impl Into<Dflt>) -> KResult<Option<i64>> {
        // Allow a field to be set from the config file
        let default = default.into();
        let config_name = format!("driver_{}", field_name.to_uppercase());
        let reg_name = self.reg_of(field_name)?;
        let mask = self.mask_of(&reg_name, field_name)?;
        let maxval = mask >> ffs(mask);
        let no_default = matches!(default, Dflt::None);
        let lim = if maxval == 1 {
            None
        } else if self.signed_fields.iter().any(|f| f == field_name) {
            Some(Lim::new().min(-(maxval / 2 + 1)).max(maxval / 2))
        } else {
            Some(Lim::new().min(0).max(maxval))
        };
        let val = match (lim, no_default) {
            (None, true) => config.getboolean_opt(&config_name)?.map(|b| b as i64),
            (None, false) => Some(config.getboolean(&config_name, default)? as i64),
            (Some(lim), true) => config.getint_opt(&config_name, lim)?,
            (Some(lim), false) => Some(config.getint(&config_name, default, lim)?),
        };
        match val {
            None => Ok(None),
            Some(v) => Ok(Some(self.set_field(field_name, v)?)),
        }
    }

    pub fn pretty_format(&self, reg_name: &str, reg_value: i64) -> String {
        // Provide a string description of a register
        let mut reg_fields: Vec<(i64, String)> =
            self.all_fields.get(reg_name).map(|f| f.iter().map(|(n, m)| (*m, n.clone())).collect()).unwrap_or_default();
        reg_fields.sort();
        let mut fields = String::new();
        for (_mask, field_name) in reg_fields {
            let field_value = self.get_field_full(&field_name, Some(reg_value), Some(reg_name)).unwrap_or(0);
            let sval = (self.field_formatters)(&field_name, field_value).unwrap_or_else(|| field_value.to_string());
            if !sval.is_empty() && sval != "0" {
                fields += &format!(" {}={}", field_name, sval);
            }
        }
        format!("{:<11} {:08x}{}", format!("{}:", reg_name), reg_value, fields)
    }

    pub fn get_reg_fields(&self, reg_name: &str, reg_value: i64) -> IndexMap<String, i64> {
        // Provide fields found in a register
        self.all_fields
            .get(reg_name)
            .map(|f| f.keys().map(|n| (n.clone(), self.get_field_full(n, Some(reg_value), Some(reg_name)).unwrap_or(0))).collect())
            .unwrap_or_default()
    }
}

// ----------------------------------------------------------------------
// The mcu interface of a driver
// ----------------------------------------------------------------------

/// a raw register read: {"data": ..., "#receive_time": ..., "spi_status"}
#[derive(Clone, Default)]
pub struct RawRead {
    pub data: i64,
    pub receive_time: f64,
    pub spi_status: Option<i64>,
}

/// what tmc.py uses of the MCU_TMC_uart / MCU_TMC_SPI objects
pub trait TmcMcu {
    fn get_fields(&self) -> Rc<FieldHelper>;
    fn get_register_raw(&self, reg_name: &str) -> KResult<RawRead>;
    fn get_register(&self, reg_name: &str) -> KResult<i64> {
        Ok(self.get_register_raw(reg_name)?.data)
    }
    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()>;
    fn get_tmc_frequency(&self) -> Option<f64>;
    fn get_mcu(&self) -> Rc<Mcu>;
    /// self.name_to_reg.get(reg_name, 0)
    fn name_to_reg(&self, reg_name: &str) -> i64;
}

pub type TmcMcuRef = Rc<dyn TmcMcu>;

// ----------------------------------------------------------------------
// Periodic error checking
// ----------------------------------------------------------------------

/// reg_info: [last_value, reg_name, mask, err_mask, cs_actual_mask]
#[derive(Clone)]
struct RegInfo {
    last_value: i64,
    reg_name: String,
    mask: i64,
    err_mask: i64,
    cs_actual_mask: i64,
}

pub struct TMCErrorCheck {
    printer: Rc<Printer>,
    stepper_name: String,
    mcu_tmc: TmcMcuRef,
    fields: Rc<FieldHelper>,
    check_timer: RefCell<Option<Timer>>,
    last_drv_status: Cell<Option<i64>>,
    last_drv_fields: RefCell<Option<IndexMap<String, i64>>>,
    mcu_offline: Cell<bool>,
    gstat_reg_info: RefCell<Option<RegInfo>>,
    clear_gstat: bool,
    irun_field: &'static str,
    drv_status_reg_info: RefCell<RegInfo>,
    adc_temp: Cell<Option<i64>>,
    adc_temp_reg: Option<String>,
    temp_from_adc: Option<fn(i64) -> f64>,
}

impl TMCErrorCheck {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef, temp_from_adc: Option<fn(i64) -> f64>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name_parts: Vec<&str> = config.get_name().split_whitespace().collect();
        let stepper_name = name_parts[1..].join(" ");
        let fields = mcu_tmc.get_fields();
        let mcu = mcu_tmc.get_mcu();
        // Setup for GSTAT query
        let gstat_reg_info = fields.lookup_register("drv_err").map(|reg_name| RegInfo {
            last_value: 0,
            reg_name,
            mask: 0xFFFFFFFF,
            err_mask: 0xFFFFFFFF,
            cs_actual_mask: 0,
        });
        let mut clear_gstat = true;
        // Setup for DRV_STATUS query
        let mut irun_field = "irun";
        let mut reg_name = "DRV_STATUS".to_string();
        let (mut mask, mut err_mask, mut cs_actual_mask) = (0, 0, 0);
        let get = |r: &str, f: &str| fields.all_fields.get(r).and_then(|x| x.get(f)).copied();
        if name_parts[0] == "tmc2130" {
            // TMC2130 driver quirks
            clear_gstat = false;
            cs_actual_mask = get(&reg_name, "cs_actual").unwrap_or(0);
        } else if name_parts[0] == "tmc2660" {
            // TMC2660 driver quirks
            irun_field = "cs";
            reg_name = "READRSP@RDSEL2".into();
            cs_actual_mask = get(&reg_name, "se").unwrap_or(0);
        }
        let err_fields = ["ot", "s2ga", "s2gb", "s2vsa", "s2vsb"];
        let warn_fields = ["otpw", "t120", "t143", "t150", "t157"];
        for f in err_fields.iter().chain(warn_fields.iter()) {
            if let Some(m) = get(&reg_name, f) {
                mask |= m;
                if err_fields.contains(f) {
                    err_mask |= m;
                }
            }
        }
        let drv_status_reg_info = RegInfo { last_value: 0, reg_name, mask, err_mask, cs_actual_mask };
        // Setup for temperature query
        let adc_temp_reg = fields.lookup_register("adc_temp");
        let e = Rc::new(TMCErrorCheck {
            printer: printer.clone(),
            stepper_name,
            mcu_tmc: mcu_tmc.clone(),
            fields: fields.clone(),
            check_timer: RefCell::new(None),
            last_drv_status: Cell::new(None),
            last_drv_fields: RefCell::new(None),
            // A non-critical mcu only counts as back once its config is resent
            mcu_offline: Cell::new(false),
            gstat_reg_info: RefCell::new(gstat_reg_info),
            clear_gstat,
            irun_field,
            drv_status_reg_info: RefCell::new(drv_status_reg_info),
            adc_temp: Cell::new(None),
            adc_temp_reg: adc_temp_reg.clone(),
            temp_from_adc,
        });
        let ee = e.clone();
        printer.register_event_handler(
            mcu.get_non_critical_disconnect_event_name(),
            Rc::new(move |_| {
                ee.mcu_offline.set(true);
                Ok(())
            }),
        );
        let ee = e.clone();
        printer.register_event_handler(
            mcu.get_non_critical_reconnect_event_name(),
            Rc::new(move |_| {
                ee.mcu_offline.set(false);
                Ok(())
            }),
        );
        if adc_temp_reg.is_some() {
            let pheaters = load_heaters(config)?;
            pheaters.register_monitor(config);
        }
        Ok(e)
    }

    fn query_register(&self, info: &RefCell<RegInfo>, mut try_clear: bool) -> KResult<i64> {
        let RegInfo { mut last_value, reg_name, mask, err_mask, cs_actual_mask } = info.borrow().clone();
        let mut cleared_flags = 0;
        let mut count = 0;
        loop {
            let val = match self.mcu_tmc.get_register(&reg_name) {
                Ok(v) => v,
                Err(e) if e.is_command_error() => {
                    count += 1;
                    if count < 3 && e.message().starts_with("Unable to read tmc uart") {
                        // Allow more retries on a TMC UART read error
                        let reactor = self.printer.get_reactor();
                        reactor.pause(reactor.monotonic() + 0.050)?;
                        continue;
                    }
                    return Err(e);
                }
                Err(e) => return Err(e),
            };
            if val & mask != last_value & mask {
                let fmt = self.fields.pretty_format(&reg_name, val);
                logging::info(&format!("TMC '{}' reports {}", self.stepper_name, fmt));
            }
            info.borrow_mut().last_value = val;
            last_value = val;
            if val & err_mask == 0 {
                if cs_actual_mask == 0 || val & cs_actual_mask != 0 {
                    break;
                }
                let irun = self.fields.get_field(self.irun_field);
                if self.check_timer.borrow().is_none() || irun < 4 {
                    break;
                }
                if self.irun_field == "irun" && self.fields.get_field("ihold") == 0 {
                    break;
                }
                // CS_ACTUAL field of zero - indicates a driver reset
            }
            count += 1;
            if count >= 3 {
                let fmt = self.fields.pretty_format(&reg_name, val);
                return Err(KError::command(format!("TMC '{}' reports error: {}", self.stepper_name, fmt)));
            }
            if try_clear && val & err_mask != 0 {
                try_clear = false;
                cleared_flags |= val & err_mask;
                self.mcu_tmc.set_register(&reg_name, val & err_mask, None)?;
            }
        }
        Ok(cleared_flags)
    }

    fn query_temperature(&self) {
        let reg = self.adc_temp_reg.clone().unwrap_or_default();
        match self.mcu_tmc.get_register(&reg) {
            Ok(v) => self.adc_temp.set(Some(v)),
            // Ignore comms error for temperature
            Err(_) => self.adc_temp.set(None),
        }
    }

    fn do_periodic_check(&self, eventtime: f64) -> f64 {
        let mcu = self.mcu_tmc.get_mcu();
        if self.mcu_offline.get() || mcu.non_critical_disconnected.get() {
            return eventtime + 1.0;
        }
        let r = (|| -> KResult<()> {
            self.query_register(&self.drv_status_reg_info, false)?;
            if self.gstat_reg_info.borrow().is_some() {
                let gi = RefCell::new(self.gstat_reg_info.borrow().clone().unwrap());
                self.query_register(&gi, false)?;
                *self.gstat_reg_info.borrow_mut() = Some(gi.into_inner());
            }
            if self.adc_temp_reg.is_some() {
                self.query_temperature();
            }
            Ok(())
        })();
        match r {
            Ok(()) => eventtime + 1.0,
            Err(e) if e.is_command_error() => {
                // A non-critical mcu dropping mid-query is not a driver fault
                if mcu.non_critical_disconnected.get() {
                    return eventtime + 1.0;
                }
                self.printer.invoke_shutdown(e.message(), None);
                NEVER
            }
            Err(e) => {
                logging::exception_err("TMC periodic check", &e);
                NEVER
            }
        }
    }

    pub fn stop_checks(&self) {
        if let Some(t) = self.check_timer.borrow_mut().take() {
            self.printer.get_reactor().unregister_timer(&t);
        }
    }

    pub fn start_checks(self: &Rc<Self>) -> KResult<bool> {
        if self.check_timer.borrow().is_some() {
            self.stop_checks();
        }
        let mut cleared_flags = 0;
        self.query_register(&self.drv_status_reg_info, false)?;
        if self.gstat_reg_info.borrow().is_some() {
            let gi = RefCell::new(self.gstat_reg_info.borrow().clone().unwrap());
            cleared_flags = self.query_register(&gi, self.clear_gstat)?;
            *self.gstat_reg_info.borrow_mut() = Some(gi.into_inner());
        }
        let reactor = self.printer.get_reactor();
        let curtime = reactor.monotonic();
        let w = Rc::downgrade(self);
        let timer = reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(e) => e.do_periodic_check(eventtime),
                None => NEVER,
            }),
            curtime + 1.0,
        );
        *self.check_timer.borrow_mut() = Some(timer);
        if cleared_flags != 0 {
            let reset_mask = self.fields.all_fields.get("GSTAT").and_then(|f| f.get("reset")).copied().unwrap_or(0);
            if cleared_flags & reset_mask != 0 {
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub fn get_status(&self) -> IndexMap<String, PyVal> {
        let mut d = IndexMap::new();
        if self.check_timer.borrow().is_none() {
            d.insert("drv_status".to_string(), PyVal::None);
            d.insert("temperature".to_string(), PyVal::None);
            return d;
        }
        let mut temp = PyVal::None;
        if let (Some(adc_temp), Some(f)) = (self.adc_temp.get(), self.temp_from_adc) {
            let adc = self.fields.get_field_of("adc_temp", adc_temp);
            temp = PyVal::Float(py_round(f(adc), 2));
        }
        let info = self.drv_status_reg_info.borrow().clone();
        if Some(info.last_value) != self.last_drv_status.get() {
            self.last_drv_status.set(Some(info.last_value));
            let fields = self.fields.get_reg_fields(&info.reg_name, info.last_value);
            *self.last_drv_fields.borrow_mut() = Some(fields.into_iter().filter(|(_, v)| *v != 0).collect());
        }
        let drv: IndexMap<String, PyVal> =
            self.last_drv_fields.borrow().clone().unwrap_or_default().into_iter().map(|(k, v)| (k, PyVal::Int(v))).collect();
        d.insert("drv_status".to_string(), PyVal::Dict(drv));
        d.insert("temperature".to_string(), temp);
        d
    }

    pub fn teardown(&self) {
        self.stop_checks();
    }
}

// ----------------------------------------------------------------------
// Record driver status
// ----------------------------------------------------------------------

pub struct TMCStallguardDump {
    me: Weak<TMCStallguardDump>,
    printer: Rc<Printer>,
    mcu_tmc: TmcMcuRef,
    mcu: Rc<Mcu>,
    fields: Rc<FieldHelper>,
    sg4_reg_name: Option<&'static str>,
    optimized_spi: Cell<bool>,
    samples: RefCell<Vec<(f64, i64, i64)>>,
    query_timer: RefCell<Option<Timer>>,
    error: RefCell<Option<KError>>,
    batch_bulk: RefCell<Option<Rc<BatchBulkHelper>>>,
}

impl TMCStallguardDump {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let stepper_name = config.get_name().split_whitespace().skip(1).collect::<Vec<_>>().join(" ");
        let fields = mcu_tmc.get_fields();
        let mut sg4_reg_name = None;
        let mut active = true;
        // It is possible to support TMC2660, just disable it for now
        if fields.all_fields.get("DRV_STATUS").map(|f| f.is_empty()).unwrap_or(true) {
            active = false;
        } else {
            // New drivers have separate register for SG4 result
            if mcu_tmc.name_to_reg("SG_RESULT") != 0 {
                sg4_reg_name = Some("SG_RESULT");
            }
            // 2240 supports both SG2 & SG4
            if sg4_reg_name.is_none() && mcu_tmc.name_to_reg("SG4_RESULT") != 0 {
                sg4_reg_name = Some("SG4_RESULT");
            }
            // (python's "sg2_supp is None" check never matches: no early
            // return for the TMC2208)
        }
        let d = Rc::new_cyclic(|me| TMCStallguardDump {
            me: me.clone(),
            printer: printer.clone(),
            mcu_tmc: mcu_tmc.clone(),
            mcu: mcu_tmc.get_mcu(),
            fields,
            sg4_reg_name,
            optimized_spi: Cell::new(false),
            samples: RefCell::new(vec![]),
            query_timer: RefCell::new(None),
            error: RefCell::new(None),
            batch_bulk: RefCell::new(None),
        });
        if !active {
            return Ok(d);
        }
        // Bulk API
        let (w1, w2, w3) = (d.me.clone(), d.me.clone(), d.me.clone());
        let bb = BatchBulkHelper::new(
            &printer,
            Rc::new(move |et| match w1.upgrade() {
                Some(d) => d.dump(et),
                None => Ok(PyVal::dict()),
            }),
            Some(Rc::new(move || match w2.upgrade() {
                Some(d) => d.start(),
                None => Ok(()),
            })),
            Some(Rc::new(move || {
                if let Some(d) = w3.upgrade() {
                    d.stop();
                }
                Ok(())
            })),
            BATCH_INTERVAL,
        );
        let mut hdr = IndexMap::new();
        hdr.insert(
            "header".to_string(),
            PyVal::Tuple(vec![PyVal::from("time"), PyVal::from("sg_result"), PyVal::from("cs_actual")]),
        );
        bb.add_mux_endpoint("tmc/stallguard_dump", "name", &stepper_name, PyVal::Dict(hdr))?;
        *d.batch_bulk.borrow_mut() = Some(bb);
        Ok(d)
    }

    fn start(&self) -> KResult<()> {
        *self.error.borrow_mut() = None;
        let status = self.mcu_tmc.get_register_raw("DRV_STATUS")?;
        if status.spi_status.map(|s| s != 0).unwrap_or(false) {
            self.optimized_spi.set(true);
        }
        let reactor = self.printer.get_reactor();
        let w = self.me.clone();
        let timer = reactor.register_timer(
            Box::new(move |et| match w.upgrade() {
                Some(d) => d.query_tmc(et),
                None => NEVER,
            }),
            NOW,
        );
        *self.query_timer.borrow_mut() = Some(timer);
        Ok(())
    }

    fn stop(&self) {
        if let Some(t) = self.query_timer.borrow_mut().take() {
            self.printer.get_reactor().unregister_timer(&t);
        }
        self.samples.borrow_mut().clear();
    }

    fn query_tmc(&self, eventtime: f64) -> f64 {
        let mut sg_result = -1;
        let mut cs_actual = -1;
        let mut recv_time = eventtime;
        let r = (|| -> KResult<()> {
            if self.optimized_spi.get() || self.sg4_reg_name == Some("SG4_RESULT") {
                // TMC2130/TMC5160/TMC2240
                let status = self.mcu_tmc.get_register_raw("DRV_STATUS")?;
                let reg_val = status.data;
                cs_actual = self.fields.get_field_of("cs_actual", reg_val);
                sg_result = self.fields.get_field_of("sg_result", reg_val);
                let is_stealth = self.fields.get_field_of("stealth", reg_val);
                recv_time = status.receive_time;
                if is_stealth != 0 && self.sg4_reg_name == Some("SG4_RESULT") {
                    let sg4_ret = self.mcu_tmc.get_register_raw("SG4_RESULT")?;
                    sg_result = sg4_ret.data;
                    recv_time = sg4_ret.receive_time;
                }
            } else if self.sg4_reg_name == Some("SG_RESULT") {
                // TMC2209
                let sg4_ret = self.mcu_tmc.get_register_raw("SG_RESULT")?;
                sg_result = sg4_ret.data;
                recv_time = sg4_ret.receive_time;
            }
            Ok(())
        })();
        if let Err(e) = r {
            if e.is_command_error() {
                *self.error.borrow_mut() = Some(e);
                return NEVER;
            }
            logging::exception_err("TMC stallguard query", &e);
            return NEVER;
        }
        let print_time = self.mcu.estimated_print_time(recv_time);
        self.samples.borrow_mut().push((print_time, sg_result, cs_actual));
        if self.optimized_spi.get() {
            return eventtime + 0.001;
        }
        // UART queried as fast as possible
        eventtime + 0.005
    }

    fn dump(&self, _eventtime: f64) -> KResult<PyVal> {
        if let Some(e) = self.error.borrow().clone() {
            return Err(e);
        }
        let samples = std::mem::take(&mut *self.samples.borrow_mut());
        let data: Vec<PyVal> =
            samples.into_iter().map(|(t, s, c)| PyVal::Tuple(vec![PyVal::Float(t), PyVal::Int(s), PyVal::Int(c)])).collect();
        let mut d = IndexMap::new();
        d.insert("data".to_string(), PyVal::List(data));
        Ok(PyVal::Dict(d))
    }

    pub fn teardown(&self) {
        self.stop();
        if let Some(b) = self.batch_bulk.borrow_mut().take() {
            b.teardown();
        }
    }
}

// ----------------------------------------------------------------------
// Current helpers
// ----------------------------------------------------------------------

/// get_current(): (run, hold, req_hold, max, home); hold None where the
/// driver has no hold current
pub type CurrentInfo = (f64, Option<f64>, f64, f64, f64);

pub struct BaseTMCCurrentHelper {
    pub printer: Rc<Printer>,
    pub name: String,
    pub mcu_tmc: TmcMcuRef,
    pub fields: Rc<FieldHelper>,
    pub sense_resistor: f64,
    pub config_run_current: f64,
    pub config_hold_current: f64,
    pub config_home_current: f64,
    pub current_change_dwell_time: f64,
    pub req_run_current: Cell<f64>,
    pub req_hold_current: Cell<f64>,
    pub req_home_current: Cell<f64>,
    pub actual_current: Cell<f64>,
    pub max_current: f64,
}

impl BaseTMCCurrentHelper {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef, max_current: f64, has_sense_resistor: bool) -> KResult<Self> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let mut sense_resistor = 0.0;
        if has_sense_resistor {
            sense_resistor = config.getfloat("sense_resistor", crate::configfile::Dflt::Required, Lim::new().above(0.0))?;
        }
        // config_{run|hold|home}_current
        // represents an initial value set via config file
        let config_run_current =
            config.getfloat("run_current", crate::configfile::Dflt::Required, Lim::new().above(0.0).max(max_current))?;
        let config_hold_current = config.getfloat("hold_current", max_current, Lim::new().above(0.0).max(max_current))?;
        let config_home_current = config.getfloat("home_current", config_run_current, Lim::new().above(0.0).max(max_current))?;
        let current_change_dwell_time = config.getfloat("current_change_dwell_time", 0.5, Lim::new().above(0.0))?;
        Ok(BaseTMCCurrentHelper {
            printer,
            name,
            mcu_tmc: mcu_tmc.clone(),
            fields: mcu_tmc.get_fields(),
            sense_resistor,
            config_run_current,
            config_hold_current,
            config_home_current,
            current_change_dwell_time,
            // req_{run|hold|home}_current
            // represents a requested value, which starts with
            // the configured value but can change during runtime
            // e.g. SET_TMC_CURRENT
            req_run_current: Cell::new(config_run_current),
            req_hold_current: Cell::new(config_hold_current),
            req_home_current: Cell::new(config_home_current),
            // actual_current represents the actual current set to a stepper
            // It fluctuates between req_run_current and req_home_current
            // during homing
            actual_current: Cell::new(config_run_current),
            max_current,
        })
    }

    pub fn needs_home_current_change(&self) -> bool {
        let needs = self.actual_current.get() != self.req_home_current.get();
        logging::info(&format!("tmc {}: needs_home_current_change {}", self.name, py_bool(needs)));
        needs
    }

    pub fn needs_run_current_change(&self) -> bool {
        let needs = self.actual_current.get() != self.req_run_current.get();
        logging::info(&format!("tmc {}: needs_run_current_change {}", self.name, py_bool(needs)));
        needs
    }

    pub fn needs_hold_current_change(&self, hold_current: f64) -> bool {
        let needs = hold_current != self.req_hold_current.get();
        logging::info(&format!("tmc {}: needs_hold_current_change {}", self.name, py_bool(needs)));
        needs
    }

    pub fn set_home_current(&self, new_home_current: f64) {
        self.req_home_current.set(py_min(self.max_current, new_home_current));
    }

    pub fn set_run_current(&self, new_run_current: f64) {
        self.req_run_current.set(py_min(self.max_current, new_run_current));
    }

    pub fn set_hold_current(&self, new_hold_current: f64) {
        self.req_hold_current.set(new_hold_current);
    }

    pub fn set_actual_current(&self, current: f64) {
        self.actual_current.set(current);
        logging::info(&format!(
            "tmc {}: set_actual_current() new actual_current: {}",
            self.name,
            py_float_repr(self.actual_current.get())
        ));
    }

    pub fn needs_current_changes(&self, run_current: f64, hold_current: f64, force: bool) -> bool {
        !(run_current == self.actual_current.get() && hold_current == self.req_hold_current.get() && !force)
    }
}

/// a driver current helper (BaseTMCCurrentHelper subclasses)
pub trait TmcCurrent {
    fn base(&self) -> &BaseTMCCurrentHelper;
    fn get_current(&self) -> CurrentInfo;
    fn apply_current(&self, _print_time: Option<f64>) -> KResult<()> {
        Ok(())
    }
    fn set_current(&self, new_current: f64, hold_current: f64, print_time: Option<f64>, force: bool) -> KResult<()> {
        let b = self.base();
        if !b.needs_current_changes(new_current, hold_current, force) {
            return Ok(());
        }
        if b.needs_hold_current_change(hold_current) {
            b.set_hold_current(hold_current);
        }
        b.set_actual_current(new_current);
        self.apply_current(print_time)
    }
    fn set_current_for_homing_impl(&self, print_time: f64, pre_homing: bool) -> KResult<f64> {
        let b = self.base();
        if pre_homing && b.needs_home_current_change() {
            self.set_current(b.req_home_current.get(), b.req_hold_current.get(), Some(print_time), false)?;
            return Ok(b.current_change_dwell_time);
        } else if !pre_homing && b.needs_run_current_change() {
            self.set_current(b.req_run_current.get(), b.req_hold_current.get(), Some(print_time), false)?;
            return Ok(b.current_change_dwell_time);
        }
        Ok(0.0)
    }
}

pub type TmcCurrentRef = Rc<dyn TmcCurrent>;

/// the stepper's view of a current helper (homing.py)
pub struct StepperCurrentHelper(pub TmcCurrentRef);

impl TmcCurrentHelper for StepperCurrentHelper {
    fn set_current_for_homing(&self, print_time: f64, pre_homing: bool) -> KResult<f64> {
        self.0.set_current_for_homing_impl(print_time, pre_homing)
    }
}

// ----------------------------------------------------------------------
// G-Code command helpers
// ----------------------------------------------------------------------

const CMD_INIT_TMC_HELP: &str = "Initialize TMC stepper driver registers";
const CMD_SET_TMC_FIELD_HELP: &str = "Set a register field of a TMC driver";
const CMD_SET_TMC_CURRENT_HELP: &str = "Set the current of a TMC driver";
const CMD_DUMP_TMC_HELP: &str = "Read and display TMC stepper driver registers";

/// read_translate(reg_name, val) -> (reg_name, val)
pub type ReadTranslate = Rc<dyn Fn(&str, i64) -> (String, i64)>;

pub struct TMCCommandHelper {
    me: Weak<TMCCommandHelper>,
    printer: Rc<Printer>,
    stepper_name: String,
    name: String,
    mcu_tmc: TmcMcuRef,
    current_helper: TmcCurrentRef,
    init_callback: Option<Rc<dyn Fn() -> KResult<()>>>,
    fields: Rc<FieldHelper>,
    stepper: RefCell<Option<Rc<McuStepper>>>,
    mcu_phase_offset: Cell<Option<i64>>,
    toff: Cell<Option<i64>>,
    stepper_enable: Rc<PrinterStepperEnable>,
    enable_mutex: Rc<ReactorMutex>,
    read_registers: RefCell<Vec<String>>,
    read_translate: RefCell<Option<ReadTranslate>>,
    echeck_helper: Rc<TMCErrorCheck>,
    record_helper: Rc<TMCStallguardDump>,
}

impl TMCCommandHelper {
    pub fn new(
        config: &ConfigWrapper,
        mcu_tmc: &TmcMcuRef,
        current_helper: TmcCurrentRef,
        temp_from_adc: Option<fn(i64) -> f64>,
        init_callback: Option<Rc<dyn Fn() -> KResult<()>>>,
    ) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let stepper_name = config.get_name().split_whitespace().skip(1).collect::<Vec<_>>().join(" ");
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let fields = mcu_tmc.get_fields();
        // Stepper enable/disable tracking
        let stepper_enable = printer
            .load_object(config, "stepper_enable", true)?
            .and_then(downcast_rc::<PrinterStepperEnable>)
            .ok_or_else(|| KError::Other("stepper_enable has an unexpected type".into()))?;
        let enable_mutex = printer.get_reactor().mutex(false);
        // Common tmc helpers
        let echeck_helper = TMCErrorCheck::new(config, mcu_tmc, temp_from_adc)?;
        let record_helper = TMCStallguardDump::new(config, mcu_tmc)?;
        // Set microstep config options
        tmc_microstep_helper(config, mcu_tmc)?;
        let ch = Rc::new_cyclic(|me| TMCCommandHelper {
            me: me.clone(),
            printer: printer.clone(),
            stepper_name,
            name: name.clone(),
            mcu_tmc: mcu_tmc.clone(),
            current_helper,
            init_callback,
            fields,
            stepper: RefCell::new(None),
            // Stepper phase tracking
            mcu_phase_offset: Cell::new(None),
            toff: Cell::new(None),
            stepper_enable,
            enable_mutex,
            // DUMP_TMC support
            read_registers: RefCell::new(vec![]),
            read_translate: RefCell::new(None),
            echeck_helper,
            record_helper,
        });
        // Register callbacks
        for ev in ["stepper:sync_mcu_position", "stepper:set_dir_inverted"] {
            let c = ch.clone();
            printer.register_event_handler(
                ev,
                Rc::new(move |args| {
                    let stepper = args[0].downcast_ref::<Rc<McuStepper>>().expect("stepper").clone();
                    c.handle_sync_mcu_pos(&stepper)
                }),
            );
        }
        let c = ch.clone();
        printer.register_event_handler("klippy:mcu_identify", Rc::new(move |_| c.handle_mcu_identify()));
        let c = ch.clone();
        printer.register_event_handler("klippy:connect", Rc::new(move |_| c.handle_connect()));
        // Register commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        type TCmd = fn(&TMCCommandHelper, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, TCmd, &str); 3] = [
            ("SET_TMC_FIELD", TMCCommandHelper::cmd_set_tmc_field, CMD_SET_TMC_FIELD_HELP),
            ("INIT_TMC", TMCCommandHelper::cmd_init_tmc, CMD_INIT_TMC_HELP),
            ("SET_TMC_CURRENT", TMCCommandHelper::cmd_set_tmc_current, CMD_SET_TMC_CURRENT_HELP),
        ];
        for (cmd, f, desc) in cmds {
            let c = ch.clone();
            gcode.register_mux_command(cmd, "STEPPER", Some(&name), Rc::new(move |g| f(&c, g)), Some(desc))?;
        }
        Ok(ch)
    }

    fn init_registers(&self, print_time: Option<f64>) -> KResult<()> {
        if let Some(cb) = &self.init_callback {
            cb()?;
        }
        // Send registers
        let names: Vec<String> = self.fields.registers.borrow().keys().cloned().collect();
        for reg_name in names {
            let val = self.fields.registers.borrow()[&reg_name]; // Val may change during loop
            self.mcu_tmc.set_register(&reg_name, val, print_time)?;
        }
        Ok(())
    }

    fn last_move_time(&self) -> KResult<f64> {
        self.printer.lookup::<ToolHead>("toolhead")?.get_last_move_time()
    }

    fn cmd_init_tmc(&self, _gcmd: &GCodeCommand) -> KResult<()> {
        logging::info(&format!("INIT_TMC {}", self.name));
        let print_time = self.last_move_time()?;
        self.init_registers(Some(print_time))
    }

    fn cmd_set_tmc_field(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let field_name = gcmd.get("FIELD")?.to_lowercase();
        let Some(reg_name) = self.fields.lookup_register(&field_name) else {
            return Err(gcmd.error(format!("Unknown field name '{}'", field_name)));
        };
        let value = gcmd.get_int_opt("VALUE", Lim::new())?;
        let velocity = gcmd.get_float_opt("VELOCITY", Lim::new().min(0.0))?;
        if value.is_none() == velocity.is_none() {
            return Err(gcmd.error("Specify either VALUE or VELOCITY"));
        }
        let value = match velocity {
            Some(velocity) => {
                if self.mcu_tmc.get_tmc_frequency().is_none() {
                    return Err(gcmd.error("VELOCITY parameter not supported by this driver"));
                }
                let stepper = self.stepper.borrow().clone();
                tmc_tstep_helper(&*self.mcu_tmc, velocity, stepper.as_ref(), None)?
            }
            None => value.unwrap(),
        };
        let reg_val = self.fields.set_field(&field_name, value)?;
        let print_time = self.last_move_time()?;
        self.mcu_tmc.set_register(&reg_name, reg_val, Some(print_time))
    }

    fn cmd_set_tmc_current(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let ch = &self.current_helper;
        let (mut prev_cur, mut prev_hold_cur, req_hold_cur, max_cur, mut prev_home_cur) = ch.get_current();
        let run_current = gcmd.get_float_opt("CURRENT", Lim::new().min(0.0).max(max_cur))?;
        let hold_current = gcmd.get_float_opt("HOLDCURRENT", Lim::new().above(0.0).max(max_cur))?;
        let home_current = gcmd.get_float_opt("HOMECURRENT", Lim::new().above(0.0).max(max_cur))?;
        if run_current.is_some() || hold_current.is_some() || home_current.is_some() {
            let run_current = match run_current {
                Some(r) => {
                    ch.base().set_run_current(r);
                    r
                }
                None => prev_cur,
            };
            let hold_current = hold_current.unwrap_or(req_hold_cur);
            if let Some(h) = home_current {
                ch.base().set_home_current(h);
            }
            let print_time = self.last_move_time()?;
            ch.set_current(run_current, hold_current, Some(print_time), false)?;
            let c = ch.get_current();
            prev_cur = c.0;
            prev_hold_cur = c.1;
            prev_home_cur = c.4;
        }
        // Report values
        match prev_hold_cur {
            None => gcmd.respond_info(&format!("Run Current: {:.2}A Home Current: {:.2}A", prev_cur, prev_home_cur), true),
            Some(h) => gcmd.respond_info(
                &format!("Run Current: {:.2}A Hold Current: {:.2}A Home Current: {:.2}A", prev_cur, h, prev_home_cur),
                true,
            ),
        }
        Ok(())
    }

    // Stepper phase tracking
    fn get_phases(&self) -> i64 {
        (256i64.checked_shr(self.fields.get_field("mres") as u32).unwrap_or(0)) * 4
    }

    pub fn clear_mcu_phase_offset(&self) {
        self.mcu_phase_offset.set(None);
    }

    pub fn get_phase_offset(&self) -> (Option<i64>, i64) {
        (self.mcu_phase_offset.get(), self.get_phases())
    }

    fn query_phase(&self) -> KResult<i64> {
        let mut field_name = "mscnt";
        if self.fields.lookup_register(field_name).is_none() {
            // TMC2660 uses MSTEP
            field_name = "mstep";
        }
        let reg_name = self.fields.lookup_register(field_name).ok_or_else(|| KError::Other(format!("'{}'", field_name)))?;
        let reg = self.mcu_tmc.get_register(&reg_name)?;
        Ok(self.fields.get_field_of(field_name, reg))
    }

    fn handle_sync_mcu_pos(&self, stepper: &Rc<McuStepper>) -> KResult<()> {
        if stepper.get_name(false) != self.stepper_name {
            return Ok(());
        }
        let mut driver_phase = match self.query_phase() {
            Ok(p) => p,
            Err(e) if e.is_command_error() => {
                logging::info(&format!("Unable to obtain tmc {} phase", self.stepper_name));
                self.mcu_phase_offset.set(None);
                let enable_line = self.stepper_enable.lookup_enable(&self.stepper_name)?;
                if enable_line.is_motor_enabled() {
                    return Err(e);
                }
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        if !stepper.get_dir_inverted().0 {
            driver_phase = 1023 - driver_phase;
        }
        let phases = self.get_phases();
        let phase = ((driver_phase as f64 / 1024.0 * phases as f64 + 0.5).trunc() as i64).rem_euclid(phases);
        let moff = (phase - stepper.get_mcu_position(None)).rem_euclid(phases);
        if let Some(prev) = self.mcu_phase_offset.get() {
            if prev != moff {
                logging::warning(&format!("Stepper {} phase change (was {} now {})", self.stepper_name, prev, moff));
            }
        }
        self.mcu_phase_offset.set(Some(moff));
        Ok(())
    }

    // Stepper enable/disable tracking
    fn do_enable(&self, _print_time: f64) -> KResult<()> {
        if let Some(toff) = self.toff.get() {
            // Shared enable via comms handling
            self.fields.set_field("toff", toff)?;
        }
        self.init_registers(None)?;
        let did_reset = self.echeck_helper.start_checks()?;
        if did_reset {
            self.mcu_phase_offset.set(None);
        }
        // Calculate phase offset
        if self.mcu_phase_offset.get().is_some() {
            return Ok(());
        }
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        let mutex = gcode.get_mutex();
        mutex.lock()?;
        let r = (|| -> KResult<()> {
            if self.mcu_phase_offset.get().is_some() {
                return Ok(());
            }
            logging::info(&format!("Pausing toolhead to calculate {} phase offset", self.stepper_name));
            self.printer.lookup::<ToolHead>("toolhead")?.wait_moves()?;
            let stepper = self.stepper.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_name'".into()))?;
            self.handle_sync_mcu_pos(&stepper)
        })();
        mutex.unlock();
        r
    }

    fn do_disable(&self, print_time: f64) -> KResult<()> {
        if self.toff.get().is_some() {
            let val = self.fields.set_field("toff", 0)?;
            let reg_name = self.fields.lookup_register("toff").unwrap_or_default();
            self.mcu_tmc.set_register(&reg_name, val, Some(print_time))?;
        }
        self.echeck_helper.stop_checks();
        Ok(())
    }

    fn handle_stepper_enable(&self, print_time: f64, is_enable: bool) -> KResult<()> {
        let me = self.me.upgrade().expect("tmc command helper");
        self.printer.get_reactor().register_callback(
            move |_| {
                let r = me.enable_mutex.lock().map_err(KError::from).and_then(|_| {
                    let r = if is_enable { me.do_enable(print_time) } else { me.do_disable(print_time) };
                    me.enable_mutex.unlock();
                    r
                });
                match r {
                    Err(e) if e.is_command_error() => me.printer.invoke_shutdown(e.message(), None),
                    Err(e) => logging::exception_err("TMC enable/disable", &e),
                    Ok(()) => {}
                }
            },
            NOW,
        );
        Ok(())
    }

    // Initial startup handling
    fn handle_mcu_identify(&self) -> KResult<()> {
        // Lookup stepper object
        let force_move = self.printer.lookup::<ForceMove>("force_move")?;
        let stepper = force_move.lookup_stepper(&self.stepper_name)?;
        stepper.set_tmc_current_helper(Some(Rc::new(StepperCurrentHelper(self.current_helper.clone()))));
        // Note pulse duration and step_both_edge optimizations available
        stepper.setup_default_pulse_duration(0.000000100, true);
        *self.stepper.borrow_mut() = Some(stepper);
        Ok(())
    }

    fn handle_connect(&self) -> KResult<()> {
        // Check if using step on both edges optimization
        let stepper = self.stepper.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'get_pulse_duration'".into()))?;
        let (_pulse_duration, step_both_edge) = stepper.get_pulse_duration();
        if step_both_edge {
            self.fields.set_field("dedge", 1)?;
        }
        // Check for soft stepper enable/disable
        let enable_line = self.stepper_enable.lookup_enable(&self.stepper_name)?;
        let w = self.me.clone();
        enable_line.register_state_callback(Rc::new(move |print_time, is_enable| match w.upgrade() {
            Some(c) => c.handle_stepper_enable(print_time, is_enable),
            None => Ok(()),
        }));
        if !enable_line.has_dedicated_enable() {
            self.toff.set(Some(self.fields.get_field("toff")));
            self.fields.set_field("toff", 0)?;
            logging::info(&format!("Enabling TMC virtual enable for '{}'", self.stepper_name));
        }
        // Send init
        let mcu = self.mcu_tmc.get_mcu();
        let r = if mcu.non_critical_disconnected.get() {
            logging::info(&format!("TMC {} failed to init - non_critical_mcu: {} is disconnected!", self.name, mcu.get_name()));
            Ok(())
        } else {
            self.init_registers(None)
        };
        match r {
            Err(e) if e.is_command_error() => {
                logging::info(&format!("TMC {} failed to init: {}", self.name, e.message()));
                Ok(())
            }
            r => r,
        }
    }

    // get_status information export
    pub fn get_status(&self) -> PyVal {
        let mut cpos = PyVal::None;
        if let (Some(stepper), Some(off)) = (self.stepper.borrow().as_ref(), self.mcu_phase_offset.get()) {
            cpos = PyVal::Float(stepper.mcu_to_commanded_position(off));
        }
        let current = self.current_helper.get_current();
        let mut res = IndexMap::new();
        res.insert("mcu_phase_offset".to_string(), self.mcu_phase_offset.get().map(PyVal::Int).unwrap_or(PyVal::None));
        res.insert("phase_offset_position".to_string(), cpos);
        res.insert("run_current".to_string(), PyVal::Float(current.0));
        res.insert("hold_current".to_string(), current.1.map(PyVal::Float).unwrap_or(PyVal::None));
        res.extend(self.echeck_helper.get_status());
        PyVal::Dict(res)
    }

    // DUMP_TMC support
    pub fn setup_register_dump(&self, read_registers: &[&str], read_translate: Option<ReadTranslate>) -> KResult<()> {
        *self.read_registers.borrow_mut() = read_registers.iter().map(|s| s.to_string()).collect();
        *self.read_translate.borrow_mut() = read_translate;
        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;
        let c = self.me.upgrade().expect("tmc command helper");
        gcode.register_mux_command("DUMP_TMC", "STEPPER", Some(&self.name), Rc::new(move |g| c.cmd_dump_tmc(g)), Some(CMD_DUMP_TMC_HELP))
    }

    fn read_and_format(&self, reg_name: &str) -> KResult<String> {
        let mut val = self.mcu_tmc.get_register(reg_name)?;
        let mut reg_name = reg_name.to_string();
        if let Some(t) = self.read_translate.borrow().clone() {
            let (r, v) = t(&reg_name, val);
            reg_name = r;
            val = v;
        }
        Ok(self.fields.pretty_format(&reg_name, val))
    }

    fn cmd_dump_tmc(&self, gcmd: &GCodeCommand) -> KResult<()> {
        logging::info(&format!("DUMP_TMC {}", self.name));
        let read_registers = self.read_registers.borrow().clone();
        if let Some(reg_name) = gcmd.get_opt("REGISTER") {
            let reg_name = reg_name.to_uppercase();
            let val = self.fields.registers.borrow().get(&reg_name).copied();
            let readable = read_registers.contains(&reg_name);
            if let (Some(val), false) = (val, readable) {
                // write-only register
                gcmd.respond_info(&self.fields.pretty_format(&reg_name, val), true);
            } else if readable {
                // readable register
                gcmd.respond_info(&self.read_and_format(&reg_name)?, true);
            } else {
                return Err(gcmd.error(format!("Unknown register name '{}'", reg_name)));
            }
        } else {
            gcmd.respond_info("========== Write-only registers ==========", true);
            let regs: Vec<(String, i64)> = self.fields.registers.borrow().iter().map(|(k, v)| (k.clone(), *v)).collect();
            for (reg_name, val) in regs {
                if !read_registers.contains(&reg_name) {
                    gcmd.respond_info(&self.fields.pretty_format(&reg_name, val), true);
                }
            }
            gcmd.respond_info("========== Queried registers ==========", true);
            for reg_name in &read_registers {
                gcmd.respond_info(&self.read_and_format(reg_name)?, true);
            }
        }
        Ok(())
    }

    pub fn teardown(&self) {
        self.echeck_helper.teardown();
        self.record_helper.teardown();
        *self.stepper.borrow_mut() = None;
    }
}

// ----------------------------------------------------------------------
// TMC virtual pins
// ----------------------------------------------------------------------

// Helper class for "sensorless homing"
pub struct TMCVirtualPinHelper {
    me: Weak<TMCVirtualPinHelper>,
    printer: Rc<Printer>,
    mcu_tmc: TmcMcuRef,
    fields: Rc<FieldHelper>,
    diag_pin: Option<String>,
    diag_pin_field: Option<&'static str>,
    mcu_endstop: RefCell<Option<EndstopRef>>,
    dirty_regs: RefCell<IndexMap<String, i64>>,
    prev_state: RefCell<IndexMap<String, i64>>,
}

impl TMCVirtualPinHelper {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let fields = mcu_tmc.get_fields();
        let (diag_pin, diag_pin_field) = if fields.lookup_register("diag0_stall").is_some() {
            if let Some(p) = config.get_opt("diag0_pin")? {
                (Some(p), Some("diag0_stall"))
            } else {
                (config.get_opt("diag1_pin")?, Some("diag1_stall"))
            }
        } else {
            (config.get_opt("diag_pin")?, None)
        };
        let h = Rc::new_cyclic(|me| TMCVirtualPinHelper {
            me: me.clone(),
            printer: printer.clone(),
            mcu_tmc: mcu_tmc.clone(),
            fields,
            diag_pin,
            diag_pin_field,
            mcu_endstop: RefCell::new(None),
            dirty_regs: RefCell::new(IndexMap::new()),
            prev_state: RefCell::new(IndexMap::new()),
        });
        // Register virtual_endstop pin
        let name_parts: Vec<&str> = config.get_name().split_whitespace().collect();
        let ppins = printer.lookup::<PinsObject>("pins")?;
        ppins
            .0
            .borrow_mut()
            .register_chip(&format!("{}_{}", name_parts[0], name_parts[name_parts.len() - 1]), h.clone())
            .map_err(|e| KError::Pins(e.0))?;
        Ok(h)
    }

    fn set_field(&self, field_name: &str, value: i64) -> KResult<()> {
        self.prev_state.borrow_mut().insert(field_name.to_string(), self.fields.get_field(field_name));
        let reg_name = self.fields.lookup_register(field_name).unwrap_or_default();
        let v = self.fields.set_field(field_name, value)?;
        self.dirty_regs.borrow_mut().insert(reg_name, v);
        Ok(())
    }

    fn send_fields(&self) -> KResult<()> {
        let dirty: Vec<(String, i64)> = self.dirty_regs.borrow().iter().map(|(k, v)| (k.clone(), *v)).collect();
        for (reg, val) in dirty {
            self.mcu_tmc.set_register(&reg, val, None)?;
        }
        self.dirty_regs.borrow_mut().clear();
        Ok(())
    }

    fn uses_endstop(&self, hmove: &Rc<HomingMove>) -> bool {
        let me = self.mcu_endstop.borrow().clone();
        let Some(me) = me else { return false };
        hmove.get_mcu_endstops().iter().any(|e| std::ptr::eq(Rc::as_ptr(e) as *const (), Rc::as_ptr(&me) as *const ()))
    }

    fn handle_homing_move_begin(&self, hmove: &Rc<HomingMove>) -> KResult<()> {
        if !self.uses_endstop(hmove) {
            return Ok(());
        }
        let mut sg4_thrs = 0;
        if self.fields.lookup_register("sg4_thrs").is_some() {
            sg4_thrs = self.fields.get_field("sg4_thrs");
        }
        // Enable/disable stealthchop
        let diag = self.diag_pin_field.unwrap_or("");
        if self.fields.lookup_register("en_pwm_mode").is_none() {
            // On "stallguard4" drivers, "stealthchop" must be enabled
            self.set_field("tpwmthrs", 0)?;
            self.set_field("en_spreadcycle", 0)?;
        } else if sg4_thrs != 0 {
            // TMC2240 using SG4, "stealthchop" must be enabled
            self.set_field("en_pwm_mode", 1)?;
            self.set_field("tpwmthrs", 0)?;
            self.set_field(diag, 1)?;
        } else {
            // On earlier drivers, "stealthchop" must be disabled
            self.set_field("en_pwm_mode", 0)?;
            self.set_field(diag, 1)?;
        }
        // Enable tcoolthrs (if not already)
        if self.fields.get_field("tcoolthrs") == 0 {
            self.set_field("tcoolthrs", 0xFFFFF)?;
        }
        // Disable thigh
        if self.fields.lookup_register("thigh").is_some() {
            self.set_field("thigh", 0)?;
        }
        self.send_fields()
    }

    fn handle_homing_move_end(&self, hmove: &Rc<HomingMove>) -> KResult<()> {
        if !self.uses_endstop(hmove) {
            return Ok(());
        }
        // Restore previous state
        let prev: Vec<(String, i64)> = self.prev_state.borrow().iter().map(|(k, v)| (k.clone(), *v)).collect();
        for (field, val) in prev {
            self.set_field(&field, val)?;
        }
        self.send_fields()?;
        self.prev_state.borrow_mut().clear();
        Ok(())
    }
}

impl PinChip for TMCVirtualPinHelper {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        // Validate pin
        {
            let p = pin_params.borrow();
            if pin_type != "endstop" || p.pin != "virtual_endstop" {
                return Err("tmc virtual endstop only useful as endstop".into());
            }
            if p.invert != 0 || p.pullup != 0 {
                return Err("Can not pullup/invert tmc virtual pin".into());
            }
        }
        let Some(diag_pin) = self.diag_pin.clone() else {
            return Err("tmc virtual endstop requires diag pin config".into());
        };
        // Setup for sensorless homing
        let w = self.me.clone();
        self.printer.register_event_handler(
            "homing:homing_move_begin",
            Rc::new(move |args| match w.upgrade() {
                Some(h) => h.handle_homing_move_begin(args[0].downcast_ref::<Rc<HomingMove>>().expect("hmove")),
                None => Ok(()),
            }),
        );
        let w = self.me.clone();
        self.printer.register_event_handler(
            "homing:homing_move_end",
            Rc::new(move |args| match w.upgrade() {
                Some(h) => h.handle_homing_move_end(args[0].downcast_ref::<Rc<HomingMove>>().expect("hmove")),
                None => Ok(()),
            }),
        );
        let obj = crate::pins::setup_pin(&self.printer, "endstop", &diag_pin).map_err(|e| e.message().to_string())?;
        *self.mcu_endstop.borrow_mut() = any_to_endstop(obj.clone());
        Ok(obj)
    }
}

// ----------------------------------------------------------------------
// Config reading helpers
// ----------------------------------------------------------------------

// Helper to initialize the wave table from config or defaults
pub fn tmc_wave_table_helper(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    for (f, d) in [
        ("mslut0", 0xAAAAB554i64),
        ("mslut1", 0x4A9554AA),
        ("mslut2", 0x24492929),
        ("mslut3", 0x10104222),
        ("mslut4", 0xFBFFFFFF),
        ("mslut5", 0xB5BB777D),
        ("mslut6", 0x49295556),
        ("mslut7", 0x00404222),
        ("w0", 2),
        ("w1", 1),
        ("w2", 1),
        ("w3", 1),
        ("x1", 128),
        ("x2", 255),
        ("x3", 255),
        ("start_sin", 0),
        ("start_sin90", 247),
    ] {
        fields.set_config_field(config, f, d)?;
    }
    Ok(())
}

// Helper to configure the microstep settings
pub fn tmc_microstep_helper(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let stepper_name = config.get_name().split_whitespace().skip(1).collect::<Vec<_>>().join(" ");
    if !config.has_section(&stepper_name) {
        return Err(KError::config(format!("Could not find config section '[{}]' required by tmc driver", stepper_name)));
    }
    let sconfig = config.getsection(&stepper_name);
    let steps = [(256, 0), (128, 1), (64, 2), (32, 3), (16, 4), (8, 5), (4, 6), (2, 7), (1, 8)];
    let mres = sconfig.getchoice_int("microsteps", &steps, crate::configfile::Dflt::Required)?;
    fields.set_field("mres", mres)?;
    fields.set_field("intpol", config.getboolean("interpolate", true)? as i64)?;
    Ok(())
}

// Helper for calculating TSTEP based values from velocity
pub fn tmc_tstep_helper(mcu_tmc: &dyn TmcMcu, velocity: f64, pstepper: Option<&Rc<McuStepper>>, config: Option<&ConfigWrapper>) -> KResult<i64> {
    if velocity <= 0.0 {
        return Ok(0xFFFFF);
    }
    let step_dist = match pstepper {
        Some(s) => s.get_step_dist(),
        None => {
            let config = config.expect("config");
            let stepper_name = config.get_name().split_whitespace().skip(1).collect::<Vec<_>>().join(" ");
            let sconfig = config.getsection(&stepper_name);
            let (rotation_dist, steps_per_rotation) = parse_step_distance(&sconfig, None, false)?;
            rotation_dist / steps_per_rotation
        }
    };
    let mres = mcu_tmc.get_fields().get_field("mres");
    let step_dist_256 = step_dist / (1i64 << mres) as f64;
    let tmc_freq = mcu_tmc.get_tmc_frequency().unwrap_or(0.0);
    let threshold = (tmc_freq * step_dist_256 / velocity + 0.5).trunc() as i64;
    Ok(0.max(0xFFFFF.min(threshold)))
}

// Helper to configure stealthChop-spreadCycle transition velocity
pub fn tmc_stealthchop_helper(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let mut en_pwm_mode = false;
    let velocity = config.getfloat_opt("stealthchop_threshold", Lim::new().min(0.0))?;
    let mut tpwmthrs = 0xFFFFF;

    if let Some(velocity) = velocity {
        en_pwm_mode = true;
        tpwmthrs = tmc_tstep_helper(&**mcu_tmc, velocity, None, Some(config))?;
    }
    fields.set_field("tpwmthrs", tpwmthrs)?;

    if fields.lookup_register("en_pwm_mode").is_some() {
        fields.set_field("en_pwm_mode", en_pwm_mode as i64)?;
    } else {
        // TMC2208 uses en_spreadCycle
        fields.set_field("en_spreadcycle", (!en_pwm_mode) as i64)?;
    }
    Ok(())
}

// Helper to configure StallGuard and CoolStep minimum velocity
pub fn tmc_vcoolthrs_helper(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let velocity = config.getfloat_opt("coolstep_threshold", Lim::new().min(0.0))?;
    let mut tcoolthrs = 0;
    if let Some(velocity) = velocity {
        tcoolthrs = tmc_tstep_helper(&**mcu_tmc, velocity, None, Some(config))?;
    }
    fields.set_field("tcoolthrs", tcoolthrs)?;
    Ok(())
}

// Helper to configure StallGuard and CoolStep maximum velocity and
// SpreadCycle-FullStepping (High velocity) mode threshold.
pub fn tmc_vhigh_helper(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let velocity = config.getfloat_opt("high_velocity_threshold", Lim::new().min(0.0))?;
    let mut thigh = 0;
    if let Some(velocity) = velocity {
        thigh = tmc_tstep_helper(&**mcu_tmc, velocity, None, Some(config))?;
    }
    fields.set_field("thigh", thigh)?;
    Ok(())
}


// ----------------------------------------------------------------------
// The printer object of a driver
// ----------------------------------------------------------------------

/// a [tmcXXXX stepper] object: get_phase_offset and get_status of its
/// command helper
pub struct TmcDriver {
    pub fields: Rc<FieldHelper>,
    pub mcu_tmc: TmcMcuRef,
    pub cmdhelper: Rc<TMCCommandHelper>,
}

impl TmcDriver {
    pub fn get_phase_offset(&self) -> (Option<i64>, i64) {
        self.cmdhelper.get_phase_offset()
    }
}

impl crate::printer::PrinterObject for TmcDriver {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        Some(self.cmdhelper.get_status())
    }
    fn teardown(&self) {
        self.cmdhelper.teardown();
    }
}
