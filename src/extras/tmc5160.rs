// TMC5160 configuration (port of klippy/extras/tmc5160.py)
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::tmc::{
    fields_from_table, tmc_stealthchop_helper, tmc_vcoolthrs_helper, tmc_vhigh_helper, tmc_wave_table_helper, BaseTMCCurrentHelper,
    CurrentInfo, FieldHelper, TMCCommandHelper, TMCVirtualPinHelper, TmcCurrent, TmcDriver, TmcMcuRef,
};
use crate::extras::tmc2130::{self, McuTmcSpi};
use crate::printer::PObj;
use crate::pyfmt::py_min;
use std::rc::Rc;

pub use crate::extras::tmc_tables::tmc5160::*;

pub fn field_formatters(field: &str, v: i64) -> Option<String> {
    let flag = |s: &str| Some(if v != 0 { s.to_string() } else { String::new() });
    match field {
        "s2vsa" => flag("1(ShortToSupply_A!)"),
        "s2vsb" => flag("1(ShortToSupply_B!)"),
        _ => tmc2130::field_formatters(field, v),
    }
}

// ----------------------------------------------------------------------
// TMC stepper current config helper
// ----------------------------------------------------------------------

const VREF: f64 = 0.325;

fn globalscaler_error(name: &str, globalscaler: i64, cs: i64, cs_calculated: i64) -> String {
    format!(
        "[tmc5160 {}]\nGLOBALSCALER({}) calculation out of bounds.\nThe target current can't be achieved with the given CS({}) value. \
         Please adjust your configuration.\nPlease refer to the tmc5160.xlxs chopper tuning spreadsheet.\n\
         A value of {} may be a reasonable starting point.\n",
        name, globalscaler, cs, cs_calculated
    )
}

pub struct TMC5160CurrentHelper {
    base: BaseTMCCurrentHelper,
    pub cs: Option<i64>,
}

impl TMC5160CurrentHelper {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<Self> {
        let base = BaseTMCCurrentHelper::new(config, mcu_tmc, MAX_CURRENT, true)?;
        let cs = config.getint_opt("driver_CS", Lim::new().min(0).max(31))?;
        let h = TMC5160CurrentHelper { base, cs };
        let (gscaler, irun, ihold) = h.calc_current(h.base.req_run_current.get(), h.base.req_hold_current.get())?;
        h.base.fields.set_field("globalscaler", gscaler)?;
        h.base.fields.set_field("ihold", ihold)?;
        h.base.fields.set_field("irun", irun)?;
        Ok(h)
    }

    pub fn calc_globalscaler(&self, current: f64) -> i64 {
        let cs = self.cs.unwrap_or(31);
        let sr = self.base.sense_resistor;
        let globalscaler = ((current * 32.0 * 256.0 * sr * 2f64.sqrt()) / ((cs + 1) as f64 * VREF)).floor() as i64;
        if globalscaler == 256 {
            return 0;
        }
        if self.cs.is_none() && globalscaler < 32 {
            return 32;
        }
        if (1..=31).contains(&globalscaler) || globalscaler > 256 {
            let ipeak = current * 2f64.sqrt();
            let cs_calculated = (sr * 32.0 * ipeak / 0.32).floor() as i64 - 1;
            self.base.printer.invoke_shutdown(&globalscaler_error(&self.base.name, globalscaler, cs, cs_calculated), None);
        }
        globalscaler
    }

    pub fn calc_current_bits(&self, current: f64, globalscaler: i64) -> i64 {
        let globalscaler = if globalscaler == 0 { 256 } else { globalscaler };
        let cs = ((current * 256.0 * 32.0 * 2f64.sqrt() * self.base.sense_resistor) / (globalscaler as f64 * VREF) - 1.0 + 0.5).trunc()
            as i64;
        0.max(31.min(cs))
    }

    pub fn calc_current(&self, run_current: f64, hold_current: f64) -> KResult<(i64, i64, i64)> {
        let gscaler = self.calc_globalscaler(run_current);
        let irun = match self.cs {
            None => self.calc_current_bits(run_current, gscaler),
            Some(cs) => cs,
        };
        if run_current == 0.0 {
            return Err(KError::Other("float division by zero".into()));
        }
        let ihold = py_min((hold_current / run_current) * irun as f64, irun as f64).floor() as i64;
        Ok((gscaler, irun, ihold))
    }

    pub fn calc_current_from_field(&self, field_name: &str) -> f64 {
        let f = &self.base.fields;
        let mut globalscaler = f.get_field("globalscaler");
        if globalscaler == 0 {
            globalscaler = 256;
        }
        let bits = f.get_field(field_name);
        globalscaler as f64 * (bits + 1) as f64 * VREF / (256.0 * 32.0 * 2f64.sqrt() * self.base.sense_resistor)
    }
}

impl TmcCurrent for TMC5160CurrentHelper {
    fn base(&self) -> &BaseTMCCurrentHelper {
        &self.base
    }

    fn get_current(&self) -> CurrentInfo {
        let run_current = self.calc_current_from_field("irun");
        let hold_current = self.calc_current_from_field("ihold");
        (run_current, Some(hold_current), self.base.req_hold_current.get(), MAX_CURRENT, self.base.req_home_current.get())
    }

    fn apply_current(&self, print_time: Option<f64>) -> KResult<()> {
        let b = &self.base;
        let (gscaler, irun, ihold) = self.calc_current(b.actual_current.get(), b.req_hold_current.get())?;
        let val = b.fields.set_field("globalscaler", gscaler)?;
        b.mcu_tmc.set_register("GLOBALSCALER", val, print_time)?;
        b.fields.set_field("ihold", ihold)?;
        let val = b.fields.set_field("irun", irun)?;
        b.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)
    }
}

// ----------------------------------------------------------------------
// TMC5160 printer object
// ----------------------------------------------------------------------

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    // Setup mcu communication
    let fields = Rc::new(FieldHelper::new(fields_from_table(FIELDS), SIGNED_FIELDS, Some(field_formatters)));
    let mcu_tmc: TmcMcuRef = McuTmcSpi::new(config, REGISTERS, fields.clone(), TMC_FREQUENCY)?;
    // Allow virtual pins to be created
    TMCVirtualPinHelper::new(config, &mcu_tmc)?;
    // Register commands
    let current_helper = Rc::new(TMC5160CurrentHelper::new(config, &mcu_tmc)?);
    let cmdhelper = TMCCommandHelper::new(config, &mcu_tmc, current_helper, None, None)?;
    cmdhelper.setup_register_dump(READ_REGISTERS, None)?;
    // Setup basic register values
    tmc_wave_table_helper(config, &mcu_tmc)?;
    tmc_stealthchop_helper(config, &mcu_tmc)?;
    tmc_vcoolthrs_helper(config, &mcu_tmc)?;
    tmc_vhigh_helper(config, &mcu_tmc)?;
    // Allow other registers to be set from the config
    let set = |f: &str, d: i64| fields.set_config_field(config, f, d);
    //   GCONF
    fields.set_config_field(config, "multistep_filt", true)?;
    //   CHOPCONF
    set("toff", 3)?;
    set("hstrt", 5)?;
    set("hend", 2)?;
    set("fd3", 0)?;
    set("disfdcc", 0)?;
    set("chm", 0)?;
    set("tbl", 2)?;
    set("vhighfs", 0)?;
    set("vhighchm", 0)?;
    set("tpfd", 4)?;
    set("diss2g", 0)?;
    set("diss2vs", 0)?;
    //   COOLCONF
    set("semin", 0)?; // page 52
    set("seup", 0)?;
    set("semax", 0)?;
    set("sedn", 0)?;
    set("seimin", 0)?;
    set("sgt", 0)?;
    set("sfilt", 0)?;
    //   DRV_CONF
    set("drvstrength", 0)?;
    set("bbmclks", 4)?;
    set("bbmtime", 0)?;
    set("filt_isense", 0)?;
    //   SHORT_CONF, being write only we can't partially update
    let s2vs = config.getint_opt("driver_s2vs_level", Lim::new().min(4).max(15))?;
    let s2g = if s2vs.map(|v| v != 0).unwrap_or(false) {
        config.getint_opt("driver_s2g_level", Lim::new().min(2).max(15))?
    } else {
        None
    };
    if s2g.map(|v| v != 0).unwrap_or(false) {
        set("s2vs_level", 6)?;
        set("s2g_level", 6)?;
        set("short_filter", 1)?;
        set("shortdelay", 0)?;
    } else {
        let short_fields: Vec<String> = fields.all_fields.get("SHORT_CONF").map(|f| f.keys().cloned().collect()).unwrap_or_default();
        for field in short_fields {
            let v = config.get_value(&format!("driver_{}", field), Dflt::None, false)?;
            if v.map(|v| v.truthy()).unwrap_or(false) {
                return Err(KError::config("driver_s2vs_level and driver_s2g_level are required to update short_conf"));
            }
        }
    }
    //   IHOLDIRUN
    set("iholddelay", 6)?;
    //   PWMCONF
    set("pwm_ofs", 30)?;
    set("pwm_grad", 0)?;
    set("pwm_freq", 0)?;
    fields.set_config_field(config, "pwm_autoscale", true)?;
    fields.set_config_field(config, "pwm_autograd", true)?;
    set("freewheel", 0)?;
    set("pwm_reg", 4)?;
    set("pwm_lim", 12)?;
    //   TPOWERDOWN
    set("tpowerdown", 10)?;
    Ok(Rc::new(TmcDriver { fields, mcu_tmc, cmdhelper, class: "TMC5160" }))
}
