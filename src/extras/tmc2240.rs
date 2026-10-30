// TMC2240 configuration (port of klippy/extras/tmc2240.py)
//
// Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2023  Alex Voinea <voinea.dragos.alexandru@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::tmc::{
    fields_from_table, tmc_stealthchop_helper, tmc_vcoolthrs_helper, tmc_vhigh_helper, tmc_wave_table_helper, BaseTMCCurrentHelper,
    CurrentInfo, FieldHelper, TMCCommandHelper, TMCVirtualPinHelper, TmcCurrent, TmcDriver, TmcMcuRef,
};
use crate::extras::tmc2130::{self, McuTmcSpi};
use crate::extras::tmc_uart::McuTmcUart;
use crate::printer::PObj;
use crate::pyfmt::py_min;
use std::rc::Rc;

pub use crate::extras::tmc_tables::tmc2240::*;

pub fn field_formatters(field: &str, v: i64) -> Option<String> {
    let flag = |s: &str| Some(if v != 0 { s.to_string() } else { String::new() });
    let temp = || Some(format!("0x{:04x}({:.1}C)", v, (v as f64 - 2038.0) / 7.7));
    let volts = || Some(format!("0x{:04x}({:.3}V)", v, v as f64 * 0.009732));
    match field {
        "s2vsa" => flag("1(ShortToSupply_A!)"),
        "s2vsb" => flag("1(ShortToSupply_B!)"),
        "adc_temp" => temp(),
        "adc_vsupply" => volts(),
        "adc_ain" => Some(format!("0x{:04x}({:.3}mV)", v, v as f64 * 0.3052)),
        "overvoltage_vth" => volts(),
        "overtempprewarning_vth" => temp(),
        _ => tmc2130::field_formatters(field, v),
    }
}

// ----------------------------------------------------------------------
// TMC stepper current config helper
// ----------------------------------------------------------------------

const KIFS: [f64; 4] = [11750.0, 24000.0, 36000.0, 36000.0];

pub struct TMC2240CurrentHelper {
    base: BaseTMCCurrentHelper,
    pub rref: f64,
    pub current_range: i64,
    pub cs: Option<i64>,
}

fn ifs_rms(rref: f64, current_range: i64) -> f64 {
    (KIFS[current_range as usize] / rref) / 2f64.sqrt()
}

impl TMC2240CurrentHelper {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<Self> {
        let rref = config.getfloat("rref", Dflt::Required, Lim::new().min(12000.0).max(60000.0))?;
        let base = BaseTMCCurrentHelper::new(config, mcu_tmc, ifs_rms(rref, 3), false)?;
        let mut h = TMC2240CurrentHelper { base, rref, current_range: 0, cs: None };
        let current_range = h.calc_current_range(h.base.actual_current.get());
        let current_range = config.getint("current_range", current_range, Lim::new().min(current_range).max(3))?;
        h.base.fields.set_field("current_range", current_range)?;
        h.current_range = current_range;
        h.cs = config.getint_opt("driver_CS", Lim::new().min(0).max(31))?;
        let (gscaler, irun, ihold) = h.calc_current(h.base.req_run_current.get(), h.base.req_hold_current.get())?;
        h.base.fields.set_field("globalscaler", gscaler)?;
        h.base.fields.set_field("ihold", ihold)?;
        h.base.fields.set_field("irun", irun)?;
        Ok(h)
    }

    fn get_ifs_rms(&self) -> f64 {
        ifs_rms(self.rref, self.base.fields.get_field("current_range"))
    }

    pub fn calc_current_range(&self, current: f64) -> i64 {
        for current_range in 0..4 {
            if current <= ifs_rms(self.rref, current_range) {
                return current_range;
            }
        }
        3
    }

    pub fn calc_globalscaler(&self, current: f64) -> i64 {
        let ifs_rms = self.get_ifs_rms();
        let cs = self.cs.unwrap_or(31);
        let globalscaler = ((current * 256.0 * 32.0) / (ifs_rms * (cs + 1) as f64)).floor() as i64;
        if self.cs.is_none() && globalscaler == 256 {
            return 0;
        }
        if (1..=31).contains(&globalscaler) || globalscaler > 256 {
            let current_range = self.base.fields.get_field("current_range");
            let msg = format!(
                "[tmc2240 {}]\nGLOBALSCALER({}) calculation out of bounds.\nThe target current can't be achieved with the given RREF({:.6}) \
                 and CS({}). Please adjust your configuration.\nCalculated current_range bit: {:02b}\nCalculated KIFS: {:.2}",
                self.base.name,
                globalscaler,
                self.rref,
                cs,
                current_range,
                KIFS[current_range as usize] / 1000.0
            );
            self.base.printer.invoke_shutdown(&msg, None);
        }
        globalscaler
    }

    pub fn calc_current_bits(&self, current: f64, globalscaler: i64) -> i64 {
        let ifs_rms = self.get_ifs_rms();
        let globalscaler = if globalscaler == 0 { 256 } else { globalscaler };
        let cs = ((current * 256.0 * 32.0) / (globalscaler as f64 * ifs_rms) - 1.0 + 0.5).trunc() as i64;
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
        let ifs_rms = self.get_ifs_rms();
        let f = &self.base.fields;
        let mut globalscaler = f.get_field("globalscaler");
        if globalscaler == 0 {
            globalscaler = 256;
        }
        let bits = f.get_field(field_name);
        globalscaler as f64 * (bits + 1) as f64 * ifs_rms / (256.0 * 32.0)
    }
}

impl TmcCurrent for TMC2240CurrentHelper {
    fn base(&self) -> &BaseTMCCurrentHelper {
        &self.base
    }

    fn get_current(&self) -> CurrentInfo {
        let ifs_rms = self.get_ifs_rms();
        let run_current = self.calc_current_from_field("irun");
        let hold_current = self.calc_current_from_field("ihold");
        (run_current, Some(hold_current), self.base.req_hold_current.get(), ifs_rms, self.base.req_home_current.get())
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
// TMC2240 printer object
// ----------------------------------------------------------------------

fn adc_to_celsius(adc_value: i64) -> f64 {
    (adc_value as f64 - 2038.0) / 7.7
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    // Setup mcu communication
    let fields = Rc::new(FieldHelper::new(fields_from_table(FIELDS), SIGNED_FIELDS, Some(field_formatters)));
    let mcu_tmc: TmcMcuRef = if config.get_opt("uart_pin")?.is_some() {
        // use UART for communication
        McuTmcUart::new(config, REGISTERS, fields.clone(), 7, TMC_FREQUENCY)?
    } else {
        // Use SPI bus for communication
        McuTmcSpi::new(config, REGISTERS, fields.clone(), TMC_FREQUENCY)?
    };
    // Allow virtual pins to be created
    TMCVirtualPinHelper::new(config, &mcu_tmc)?;
    // Register commands
    let current_helper = Rc::new(TMC2240CurrentHelper::new(config, &mcu_tmc)?);
    let cmdhelper = TMCCommandHelper::new(config, &mcu_tmc, current_helper, Some(adc_to_celsius), None)?;
    cmdhelper.setup_register_dump(READ_REGISTERS, None)?;
    // Setup basic register values
    tmc_wave_table_helper(config, &mcu_tmc)?;
    fields.set_config_field(config, "offset_sin90", 0)?;
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
    set("semin", 0)?;
    set("seup", 0)?;
    set("semax", 0)?;
    set("sedn", 0)?;
    set("seimin", 0)?;
    set("sgt", 0)?;
    set("sfilt", 0)?;
    //   IHOLDIRUN
    set("iholddelay", 6)?;
    set("irundelay", 4)?;
    //   PWMCONF
    set("pwm_ofs", 29)?;
    set("pwm_grad", 0)?;
    set("pwm_freq", 0)?;
    fields.set_config_field(config, "pwm_autoscale", true)?;
    fields.set_config_field(config, "pwm_autograd", true)?;
    set("freewheel", 0)?;
    set("pwm_reg", 4)?;
    set("pwm_lim", 12)?;
    //   TPOWERDOWN
    set("tpowerdown", 10)?;
    //   SG4_THRS
    set("sg4_thrs", 0)?;
    set("sg4_angle_offset", 1)?;
    //   DRV_CONF
    set("slope_control", 0)?;
    Ok(Rc::new(TmcDriver { fields, mcu_tmc, cmdhelper, class: "TMC2240" }))
}
