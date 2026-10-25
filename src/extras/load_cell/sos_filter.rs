// Second Order Sections Filter
// (port of klippy/extras/load_cell/sos_filter.py)
//
// Copyright (C) 2025 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::error::{KError, KResult};
use crate::mcu::{CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::chelper::serialqueue::CommandQueue;
use crate::scipy_signal;
use std::cell::RefCell;
use std::rc::Rc;

const MAX_INT32: i64 = 1 << 31;
const MIN_INT32: i64 = -(1 << 31) - 1;

fn assert_is_int32(value: i64, error: String) -> KResult<i64> {
    if !(MIN_INT32..=MAX_INT32).contains(&value) {
        return Err(KError::Py("OverflowError".into(), error));
    }
    Ok(value)
}

// convert a floating point value to a 32 bit fixed point representation
// checks for overflow
pub fn to_fixed_32(value: f64, int_bits: i64) -> KResult<i64> {
    let fractional_bits = 32 - (1 + int_bits);
    let scaled = value * (1i64 << fractional_bits) as f64;
    if !scaled.is_finite() {
        let msg = if scaled.is_nan() { "cannot convert float NaN to integer" } else { "cannot convert float infinity to integer" };
        return Err(KError::Py(if scaled.is_nan() { "ValueError" } else { "OverflowError" }.into(), msg.into()));
    }
    // int() truncates; values past i64 overflow the int32 check anyway
    let fixed_val = if scaled.abs() >= 9.2e18 { i64::MAX } else { scaled.trunc() as i64 };
    assert_is_int32(fixed_val, format!("Fixed point Q{} overflow", int_bits))
}

/// a scipy.signal design error: python raises it as ValueError
fn design_error(msg: String) -> KError {
    KError::Py("ValueError".into(), msg)
}

// Design a SOS filter from high-level parameters
#[derive(Default)]
pub struct DigitalFilter {
    filter_sections: Vec<[f64; 6]>,
    initial_state: Vec<[f64; 2]>,
}

impl DigitalFilter {
    /// DigitalFilter(sps, cfg_error, highpass, highpass_order, lowpass,
    /// lowpass_order, notches, notch_quality). Kaliru designs the filters
    /// itself: the "requires the SciPy module" error never happens.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        sps: f64,
        highpass: Option<f64>,
        highpass_order: i64,
        lowpass: Option<f64>,
        lowpass_order: i64,
        notches: &[f64],
        notch_quality: f64,
    ) -> KResult<Self> {
        let mut f = DigitalFilter::default();
        let sample_frequency = sps;
        // an empty filter can be created without SciPi/numpy
        let truthy = |v: Option<f64>| v.is_some_and(|v| v != 0.0);
        if !(truthy(highpass) || truthy(lowpass) || !notches.is_empty()) {
            return Ok(f);
        }
        if let Some(hp) = highpass.filter(|v| *v != 0.0) {
            f.filter_sections.push(scipy_signal::butter_sos(highpass_order, hp, true, sample_frequency).map_err(design_error)?[0]);
        }
        if let Some(lp) = lowpass.filter(|v| *v != 0.0) {
            f.filter_sections.push(scipy_signal::butter_sos(lowpass_order, lp, false, sample_frequency).map_err(design_error)?[0]);
        }
        for notch_freq in notches {
            let (b, a) = scipy_signal::iirnotch(*notch_freq, notch_quality, sample_frequency).map_err(design_error)?;
            f.filter_sections.push(scipy_signal::tf2sos(&b, &a)[0]);
        }
        if !f.filter_sections.is_empty() {
            f.initial_state = scipy_signal::sosfilt_zi(&f.filter_sections);
        }
        Ok(f)
    }

    pub fn get_filter_sections(&self) -> &[[f64; 6]] {
        &self.filter_sections
    }

    pub fn get_initial_state(&self) -> &[[f64; 2]] {
        &self.initial_state
    }
}

// container that accepts SciPy formatted SOS filter data and converts it to a
// selected fixed point representation. This data could come from DigitalFilter,
// static data, config etc.
#[derive(Clone, Debug, PartialEq)]
pub struct FixedPointSosFilter {
    coeff_int_bits: i64,
    value_int_bits: i64,
    filter: Vec<[i64; 5]>,
    state: Vec<[i64; 2]>,
}

impl FixedPointSosFilter {
    // filter_sections is an array of SciPy formatted SOS filter sections (sos)
    // initial_state is an array of SciPy formatted SOS state sections (zi)
    pub fn new(filter_sections: &[[f64; 6]], initial_state: &[[f64; 2]], coeff_int_bits: i64, value_int_bits: i64) -> KResult<Self> {
        let num_sections = filter_sections.len();
        let num_state = initial_state.len();
        if num_state != num_sections {
            return Err(KError::Py(
                "ValueError".into(),
                format!("The number of filter sections ({}) and state sections ({}) must be equal", num_sections, num_state),
            ));
        }
        let coeff_int_bits = Self::validate_int_bits(coeff_int_bits)?;
        let value_int_bits = Self::validate_int_bits(value_int_bits)?;
        // convert the SciPi SOS filters to fixed point format
        let mut filter = vec![];
        for section in filter_sections {
            let mut fixed_section = [0i64; 5];
            let mut n = 0;
            for (col, coeff) in section.iter().enumerate() {
                if col != 3 {
                    // omit column 3
                    fixed_section[n] = to_fixed_32(*coeff, coeff_int_bits)?;
                    n += 1;
                } else if *coeff != 1.0 {
                    // double check colum 3 is always 1.0
                    return Err(KError::Py("ValueError".into(), format!("Coefficient 3 is expected to be 1.0 but was {:.6}", coeff)));
                }
            }
            filter.push(fixed_section);
        }
        // convert the SOS filter state matrix (zi) to fixed point format
        let mut state = vec![];
        for section in initial_state {
            state.push([to_fixed_32(section[0], value_int_bits)?, to_fixed_32(section[1], value_int_bits)?]);
        }
        Ok(FixedPointSosFilter { coeff_int_bits, value_int_bits, filter, state })
    }

    pub fn get_filter_sections(&self) -> &[[i64; 5]] {
        &self.filter
    }

    pub fn get_initial_state(&self) -> &[[i64; 2]] {
        &self.state
    }

    pub fn get_coeff_int_bits(&self) -> i64 {
        self.coeff_int_bits
    }

    pub fn get_value_int_bits(&self) -> i64 {
        self.value_int_bits
    }

    pub fn get_num_sections(&self) -> usize {
        self.filter.len()
    }

    fn validate_int_bits(int_bits: i64) -> KResult<i64> {
        if !(1..=30).contains(&int_bits) {
            return Err(KError::Py("ValueError".into(), format!("The number of integer bits ({}) must be a value between 1 and 30", int_bits)));
        }
        Ok(int_bits)
    }
}

/// one of the filter's commands: (config format, mcu format, looked up command)
struct SosCmd {
    mcu_fmt: &'static str,
    cmd: RefCell<Option<CommandWrapper>>,
}

impl SosCmd {
    fn new(mcu_fmt: &'static str) -> Self {
        SosCmd { mcu_fmt, cmd: RefCell::new(None) }
    }
}

// Control an `sos_filter` object on the MCU
pub struct SosFilter {
    mcu: Rc<Mcu>,
    cmd_queue: CommandQueue,
    oid: i64,
    filter: RefCell<FixedPointSosFilter>,
    max_sections: usize,
    cmd_set_section: SosCmd,
    cmd_config_state: SosCmd,
    cmd_activate: SosCmd,
}

impl SosFilter {
    // fixed_point_filter should be an FixedPointSosFilter instance. A filter of
    // size 0 will create a passthrough filter.
    // max_sections should be the largest number of sections you expect
    // to use at runtime. The default is the size of the fixed_point_filter.
    pub fn new(mcu: &Rc<Mcu>, cmd_queue: CommandQueue, fixed_point_filter: FixedPointSosFilter, max_sections: Option<usize>) -> Rc<Self> {
        let oid = mcu.create_oid();
        let max_sections = max_sections.unwrap_or(fixed_point_filter.get_num_sections());
        let f = Rc::new(SosFilter {
            mcu: mcu.clone(),
            cmd_queue,
            oid,
            filter: RefCell::new(fixed_point_filter),
            max_sections,
            cmd_set_section: SosCmd::new("sos_filter_set_section oid=%c section_idx=%c sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i"),
            cmd_config_state: SosCmd::new("sos_filter_set_state oid=%c section_idx=%c state0=%i state1=%i"),
            cmd_activate: SosCmd::new("sos_filter_set_active oid=%c n_sections=%c coeff_int_bits=%c"),
        });
        let ff = f.clone();
        mcu.register_config_callback(Rc::new(move || ff.build_config()));
        f
    }

    fn build_config(&self) -> KResult<()> {
        for cmd in [&self.cmd_set_section, &self.cmd_config_state, &self.cmd_activate] {
            *cmd.cmd.borrow_mut() = Some(self.mcu.lookup_command(cmd.mcu_fmt, Some(self.cmd_queue.clone()))?);
        }
        Ok(())
    }

    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    // create an uninitialized filter object on the MCU
    pub fn create_filter(&self) -> KResult<()> {
        self.mcu.add_config_cmd(&format!("config_sos_filter oid={} max_sections={}", self.oid, self.max_sections), false, false);
        self.configure_filter(true)
    }

    // either setup an init command or send the command based on a flag
    fn cmd(&self, command: &SosCmd, config_line: String, args: &[i64], is_init: bool) -> KResult<()> {
        if is_init {
            self.mcu.add_config_cmd(&config_line, true, false);
            return Ok(());
        }
        let c = command.cmd.borrow();
        let c = c.as_ref().ok_or_else(|| KError::Py("IndexError".into(), "list index out of range".into()))?;
        c.send(&args.iter().map(|v| Val::Int(*v)).collect::<Vec<_>>(), 0, 0)
    }

    fn set_filter_sections(&self, is_init: bool) -> KResult<()> {
        let sections = self.filter.borrow().get_filter_sections().to_vec();
        for (i, s) in sections.iter().enumerate() {
            let line = format!(
                "sos_filter_set_section oid={} section_idx={} sos0={} sos1={} sos2={} sos3={} sos4={}",
                self.oid, i, s[0], s[1], s[2], s[3], s[4]
            );
            self.cmd(&self.cmd_set_section, line, &[self.oid, i as i64, s[0], s[1], s[2], s[3], s[4]], is_init)?;
        }
        Ok(())
    }

    fn set_filter_state(&self, is_init: bool) -> KResult<()> {
        let states = self.filter.borrow().get_initial_state().to_vec();
        for (i, s) in states.iter().enumerate() {
            let line = format!("sos_filter_set_state oid={} section_idx={} state0={} state1={}", self.oid, i, s[0], s[1]);
            self.cmd(&self.cmd_config_state, line, &[self.oid, i as i64, s[0], s[1]], is_init)?;
        }
        Ok(())
    }

    fn activate_filter(&self, is_init: bool) -> KResult<()> {
        let (n, bits) = {
            let f = self.filter.borrow();
            (f.get_num_sections() as i64, f.get_coeff_int_bits())
        };
        let line = format!("sos_filter_set_active oid={} n_sections={} coeff_int_bits={}", self.oid, n, bits);
        self.cmd(&self.cmd_activate, line, &[self.oid, n, bits], is_init)
    }

    // configure the filter sections on the mcu
    fn configure_filter(&self, is_init: bool) -> KResult<()> {
        let num_sections = self.filter.borrow().get_num_sections();
        if num_sections > self.max_sections {
            return Err(KError::Py("ValueError".into(), format!("Too many filter sections: {}, The max is {}", num_sections, self.max_sections)));
        }
        // convert to fixed point to find errors
        // no errors, state is accepted
        // configure MCU filter and activate
        self.set_filter_sections(is_init)?;
        self.set_filter_state(is_init)?;
        self.activate_filter(is_init)
    }

    // Change the filter coefficients and state at runtime
    pub fn change_filter(&self, fixed_point_filter: FixedPointSosFilter) -> KResult<()> {
        *self.filter.borrow_mut() = fixed_point_filter;
        self.configure_filter(false)
    }

    // Resets the filter state back to initial conditions at runtime
    pub fn reset_filter(&self) -> KResult<()> {
        self.set_filter_state(false)?;
        self.activate_filter(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mathutil::numpy_tests::parse_hex;

    /// RK_SOS_CASES: tools/gen_sos_cases.py output (scipy reference)
    #[test]
    fn sos_filter_vs_scipy() {
        let Ok(path) = std::env::var("RK_SOS_CASES") else { return };
        let text = std::fs::read_to_string(path).unwrap();
        let (mut bad, mut n) = (0, 0);
        for line in text.lines() {
            let (lhs, rhs) = line.split_once(" = ").unwrap();
            let a: Vec<&str> = lhs.split_whitespace().collect();
            let opt = |s: &str| if s == "None" { None } else { Some(parse_hex(s)) };
            let notches: Vec<f64> = a[5..a.len() - 1].iter().map(|s| parse_hex(s)).collect();
            let got = DigitalFilter::new(parse_hex(a[0]), opt(a[1]), a[2].parse().unwrap(), opt(a[3]), a[4].parse().unwrap(), &notches, parse_hex(a[a.len() - 1]))
                .and_then(|d| FixedPointSosFilter::new(d.get_filter_sections(), d.get_initial_state(), 2, 16));
            let got = match got {
                Ok(f) => format!("{:?} {:?}", f.get_filter_sections(), f.get_initial_state()),
                Err(e) => format!("err {}", e.py_class()),
            };
            n += 1;
            if got != rhs {
                bad += 1;
                if bad <= 5 {
                    eprintln!("mismatch {}\n  got  {}\n  want {}", lhs, got, rhs);
                }
            }
        }
        eprintln!("sos filter: {}/{} mismatches", bad, n);
        assert_eq!(bad, 0);
    }
}
