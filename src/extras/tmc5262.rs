// TMC5262 configuration (port of klippy/extras/tmc5262.py)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::tmc::{
    fields_from_table, tmc_wave_table_helper, BaseTMCCurrentHelper, CurrentInfo, FieldHelper, TMCCommandHelper, TmcCurrent, TmcDriver,
    TmcMcuRef,
};
use crate::extras::tmc2130::{self, McuTmcSpi};
use crate::printer::{PObj, Printer};
use crate::pyfmt::py_min;
use indexmap::IndexMap;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

pub use crate::extras::tmc_tables::tmc5262::*;

fn adc_to_celsius(adc_value: i64) -> f64 {
    1.042 * adc_value as f64 - 264.6
}

fn adc_to_volts(adc_value: i64) -> f64 {
    adc_value as f64 * 0.1409
}

pub fn field_formatters(field: &str, v: i64) -> Option<String> {
    let pick = |a: &str, b: &str| Some(if v != 0 { a.to_string() } else { b.to_string() });
    let temp = || Some(format!("0x{:03x}({:.1}C)", v, adc_to_celsius(v)));
    let volts = || Some(format!("0x{:03x}({:.3}V)", v, adc_to_volts(v)));
    match field {
        "ov" => pick("1(Overvoltage!)", ""),
        "s2vsa" => pick("1(ShortToSupply_A!)", ""),
        "s2vsb" => pick("1(ShortToSupply_B!)", ""),
        "vm_uvlo" => pick("1(VMUndervoltage!)", ""),
        "vccio_uv" => pick("1(VCCIOUndervoltage!)", ""),
        "register_reset" => pick("1(RegisterReset!)", ""),
        "ext_clk" => pick("1(ExternalClock)", "0(Internal16MHz)"),
        "ext_res_det" => pick("1", "0(MissingRREF!)"),
        "drv_enn" => pick("1(Disabled)", "0(Enabled)"),
        "silicon_rv" => Some(v.to_string()),
        "clk_loss" => pick("1(ClockLoss!)", ""),
        "clk_is_stuck" => pick("1(ClockStuck!)", ""),
        "clk_1m0_tmo" => pick("1(ClockTimeout!)", ""),
        "adc_temp" => temp(),
        "adc_vsupply" => volts(),
        "angle_meas" => Some(format!("{}({:.1}deg)", v, v as f64 * 360.0 / 1024.0)),
        "overvoltage_vth" => volts(),
        "overtempprewarning_vth" => temp(),
        _ => tmc2130::field_formatters(field, v),
    }
}

// ----------------------------------------------------------------------
// TMC5262 current helper
// ----------------------------------------------------------------------

const DEFAULT_RREF: f64 = 12000.0;
const KIFS_BASE: f64 = 18.0;

fn ifs_rms_for(rref: f64, cr: i64, crs: i64) -> f64 {
    let i_peak = KIFS_BASE * (cr + 1) as f64 * (crs + 1) as f64 / (4.0 * rref / 1000.0);
    i_peak / 2f64.sqrt()
}

pub struct TMC5262CurrentHelper {
    base: BaseTMCCurrentHelper,
    rref: f64,
}

impl TMC5262CurrentHelper {
    pub fn new(config: &ConfigWrapper, mcu_tmc: &TmcMcuRef) -> KResult<Self> {
        let rref = config.getfloat("rref", DEFAULT_RREF, Lim::new().min(10000.0).max(14000.0))?;
        let base = BaseTMCCurrentHelper::new(config, mcu_tmc, ifs_rms_for(rref, 3, 3), false)?;
        let h = TMC5262CurrentHelper { base, rref };
        let (cr, crs) = h.calc_ranges(h.base.req_run_current.get());
        h.base.fields.set_field("current_range", cr)?;
        h.base.fields.set_field("current_range_scale", crs)?;
        let (irun, ihold) = h.calc_current(h.base.req_run_current.get(), h.base.req_hold_current.get());
        h.base.fields.set_field("irun", irun)?;
        h.base.fields.set_field("ihold", ihold)?;
        Ok(h)
    }

    fn ifs_rms(&self) -> f64 {
        let f = &self.base.fields;
        ifs_rms_for(self.rref, f.get_field("current_range"), f.get_field("current_range_scale"))
    }

    fn calc_ranges(&self, current: f64) -> (i64, i64) {
        for cr in 0..4 {
            if current <= ifs_rms_for(self.rref, cr, 3) {
                if cr == 0 {
                    for crs in 0..4 {
                        if current <= ifs_rms_for(self.rref, 0, crs) {
                            return (0, crs);
                        }
                    }
                }
                return (cr, 3);
            }
        }
        (3, 3)
    }

    fn calc_current(&self, run_current: f64, hold_current: f64) -> (i64, i64) {
        if run_current <= 0.0 {
            return (0, 0);
        }
        let ifs = self.ifs_rms();
        let irun = 1.max(250.min((run_current / ifs * 250.0).round_ties_even() as i64));
        let ihold = (py_min(hold_current, run_current) / run_current * irun as f64).round_ties_even() as i64;
        (irun, 0.max(irun.min(ihold)))
    }

    fn calc_current_from_field(&self, field_name: &str) -> f64 {
        self.ifs_rms() * self.base.fields.get_field(field_name) as f64 / 250.0
    }
}

impl TmcCurrent for TMC5262CurrentHelper {
    fn base(&self) -> &BaseTMCCurrentHelper {
        &self.base
    }

    fn get_current(&self) -> CurrentInfo {
        (
            self.calc_current_from_field("irun"),
            Some(self.calc_current_from_field("ihold")),
            self.base.req_hold_current.get(),
            self.ifs_rms(),
            self.base.req_home_current.get(),
        )
    }

    fn apply_current(&self, print_time: Option<f64>) -> KResult<()> {
        let b = &self.base;
        let (irun, ihold) = self.calc_current(b.actual_current.get(), b.req_hold_current.get());
        b.fields.set_field("ihold", ihold)?;
        let val = b.fields.set_field("irun", irun)?;
        b.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)
    }
}

// ----------------------------------------------------------------------
// TMC5262 printer object
// ----------------------------------------------------------------------

const PLL_DESIRED: [(&str, i64); 8] = [
    ("commit", 0),
    ("ext_not_int", 0),
    ("clk_sys_sel", 1),
    ("clk_fsm_ena", 1),
    ("clock_divider", 15),
    ("clk_1m0_tmo", 0),
    ("clk_loss", 0),
    ("clk_is_stuck", 0),
];

fn handle_pll_init(printer: &Printer, name: &str, mcu_tmc: &TmcMcuRef, cmdhelper: &Weak<TMCCommandHelper>) -> KResult<()> {
    // Check on every initialization, including re-enable after power loss.
    let pll_fields: IndexMap<String, IndexMap<String, i64>> =
        fields_from_table(FIELDS).into_iter().filter(|(r, _)| r == "PLL").collect();
    let fields = FieldHelper::new(pll_fields, &[], None);
    let pll = mcu_tmc.get_register("PLL")?;
    if PLL_DESIRED.iter().all(|(f, v)| fields.get_field_of(f, pll) == *v) {
        return Ok(());
    }
    for (field, value) in PLL_DESIRED {
        fields.set_field(field, value)?;
    }
    // PLL COMMIT resets the driver control logic, including MSCNT.
    // Clear the offset before a register access can fail after that reset.
    if let Some(c) = cmdhelper.upgrade() {
        c.clear_mcu_phase_offset();
    }
    // Disable the FSM before retrying configuration (datasheet p.106).
    mcu_tmc.set_register("PLL", fields.set_field("clk_fsm_ena", 0)?, None)?;
    fields.set_field("clk_fsm_ena", 1)?;
    mcu_tmc.set_register("PLL", fields.set_field("commit", 1)?, None)?;
    let reactor = printer.get_reactor();
    let deadline = reactor.monotonic() + 0.500;
    loop {
        let pll = mcu_tmc.get_register("PLL")?;
        if fields.get_field_of("commit", pll) == 0 {
            break;
        }
        if reactor.monotonic() >= deadline {
            return Err(KError::command(format!("TMC5262 {} PLL commit timed out", name)));
        }
        reactor.pause(reactor.monotonic() + 0.001)?;
    }
    fields.set_field("commit", 0)?;
    fields.set_field("clk_loss", 1)?;
    let val = fields.set_field("clk_is_stuck", 1)?;
    mcu_tmc.set_register("PLL", val, None)?;
    let pll = mcu_tmc.get_register("PLL")?;
    if ["clk_1m0_tmo", "clk_loss", "clk_is_stuck"].iter().any(|f| fields.get_field_of(f, pll) != 0) {
        return Err(KError::command(format!(
            "TMC5262 {} PLL fault after initialization: {}",
            name,
            fields.pretty_format("PLL", pll)
        )));
    }
    Ok(())
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
    let fields = Rc::new(FieldHelper::new(fields_from_table(FIELDS), SIGNED_FIELDS, Some(field_formatters)));
    let mcu_tmc: TmcMcuRef = McuTmcSpi::new(config, REGISTERS, fields.clone(), TMC_FREQUENCY)?;
    let current_helper = Rc::new(TMC5262CurrentHelper::new(config, &mcu_tmc)?);
    let slot: Rc<RefCell<Weak<TMCCommandHelper>>> = Rc::new(RefCell::new(Weak::new()));
    let (p, s, m) = (Rc::downgrade(&printer), slot.clone(), mcu_tmc.clone());
    let init_callback: Rc<dyn Fn() -> KResult<()>> = Rc::new(move || match p.upgrade() {
        Some(p) => handle_pll_init(&p, &name, &m, &s.borrow()),
        None => Ok(()),
    });
    let cmdhelper = TMCCommandHelper::new(config, &mcu_tmc, current_helper, Some(adc_to_celsius), Some(init_callback))?;
    *slot.borrow_mut() = Rc::downgrade(&cmdhelper);
    cmdhelper.setup_register_dump(READ_REGISTERS, None)?;

    tmc_wave_table_helper(config, &mcu_tmc)?;
    fields.set_config_field(config, "offset_sin90", 0)?;
    fields.set_field("en_pwm_mode", 0)?;
    fields.set_field("tpwmthrs", 0)?;
    fields.set_field("tcoolthrs", 0)?;
    fields.set_field("thigh", 0)?;
    fields.registers.borrow_mut().insert("DO_CONF".into(), 0);
    fields.registers.borrow_mut().insert("DO_SCOPE_CONF".into(), 0);

    let set = |f: &str, d: i64| fields.set_config_field(config, f, d);
    fields.set_field("clock_divider", 15)?;
    fields.set_field("clk_sys_sel", 1)?;
    fields.set_field("clk_fsm_ena", 1)?;

    fields.set_field("step_dir", 1)?;
    fields.set_config_field(config, "multistep_filt", true)?;

    set("toff", 3)?;
    set("hstrt", 5)?;
    set("hend", 2)?;
    set("fd3", 0)?;
    set("disfdcc", 0)?;
    set("chm", 0)?;
    set("tbl", 2)?;
    set("tpfd", 4)?;

    fields.registers.borrow_mut().insert("COOLCONF".into(), 0);

    set("iholddelay", 7)?;
    set("irundelay", 4)?;

    set("tpowerdown", 10)?;
    set("slope_control", 3)?;

    // Write the chopper configuration last.
    for register in ["CHOPCONF"] {
        let mut regs = fields.registers.borrow_mut();
        let value = regs.shift_remove(register).ok_or_else(|| KError::Other(format!("'{}'", register)))?;
        regs.insert(register.to_string(), value);
    }
    Ok(Rc::new(TmcDriver { fields, mcu_tmc, cmdhelper }))
}
