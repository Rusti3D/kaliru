// Kinematic input shaper to minimize motion vibrations in XY plane
// (port of klippy/extras/input_shaper.py)
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::itersolve::SkRef;
use crate::chelper::kin_shaper::{input_shaper_alloc, input_shaper_set_shaper_params, input_shaper_set_sk, input_shaper_update_sk};
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::motion_queuing::PrinterMotionQueuing;
use crate::extras::shaper_defs::{get_none_shaper, DEFAULT_DAMPING_RATIO, INPUT_SHAPERS};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::stepper::McuStepper;
use crate::toolhead::ToolHead;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

fn find_shaper(name: &str) -> Option<fn(f64, f64) -> (Vec<f64>, Vec<f64>)> {
    INPUT_SHAPERS.iter().find(|s| s.name == name).map(|s| s.init_func)
}

pub struct InputShaperParams {
    axis: &'static str,
    shaper_type: RefCell<String>,
    damping_ratio: Cell<f64>,
    shaper_freq: Cell<f64>,
}

impl InputShaperParams {
    fn new(axis: &'static str, config: &ConfigWrapper) -> KResult<Self> {
        let shaper_type = config.get("shaper_type", "mzv")?;
        let shaper_type = config.get(&format!("shaper_type_{}", axis), shaper_type)?;
        if find_shaper(&shaper_type).is_none() {
            return Err(KError::config(format!("Unsupported shaper type: {}", shaper_type)));
        }
        let damping_ratio = config.getfloat(&format!("damping_ratio_{}", axis), DEFAULT_DAMPING_RATIO, Lim::new().min(0.0).max(1.0))?;
        let shaper_freq = config.getfloat(&format!("shaper_freq_{}", axis), 0.0, Lim::new().min(0.0))?;
        Ok(InputShaperParams {
            axis,
            shaper_type: RefCell::new(shaper_type),
            damping_ratio: Cell::new(damping_ratio),
            shaper_freq: Cell::new(shaper_freq),
        })
    }

    fn update(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let axis = self.axis.to_uppercase();
        self.damping_ratio.set(gcmd.get_float(
            &format!("DAMPING_RATIO_{}", axis),
            Some(self.damping_ratio.get()),
            Lim::new().min(0.0).max(1.0),
        )?);
        self.shaper_freq.set(gcmd.get_float(&format!("SHAPER_FREQ_{}", axis), Some(self.shaper_freq.get()), Lim::new().min(0.0))?);
        let shaper_type = match gcmd.get_opt("SHAPER_TYPE") {
            Some(s) => s,
            None => gcmd.get_or(&format!("SHAPER_TYPE_{}", axis), &self.shaper_type.borrow()),
        };
        if find_shaper(&shaper_type.to_lowercase()).is_none() {
            return Err(gcmd.error(format!("Unsupported shaper type: {}", shaper_type)));
        }
        *self.shaper_type.borrow_mut() = shaper_type.to_lowercase();
        Ok(())
    }

    fn get_shaper(&self) -> (i32, Vec<f64>, Vec<f64>) {
        let (a, t) = if self.shaper_freq.get() == 0.0 {
            get_none_shaper()
        } else {
            find_shaper(&self.shaper_type.borrow()).expect("shaper")(self.shaper_freq.get(), self.damping_ratio.get())
        };
        (a.len() as i32, a, t)
    }

    fn get_status(&self) -> Vec<(&'static str, String)> {
        vec![
            ("shaper_type", self.shaper_type.borrow().clone()),
            ("shaper_freq", format!("{:.3}", self.shaper_freq.get())),
            ("damping_ratio", format!("{:.6}", self.damping_ratio.get())),
        ]
    }
}

type ShaperState = (i32, Vec<f64>, Vec<f64>);

pub struct AxisInputShaper {
    axis: &'static str,
    params: InputShaperParams,
    state: RefCell<ShaperState>,
    saved: RefCell<Option<ShaperState>>,
}

impl AxisInputShaper {
    fn new(axis: &'static str, config: &ConfigWrapper) -> KResult<Self> {
        let params = InputShaperParams::new(axis, config)?;
        let state = params.get_shaper();
        Ok(AxisInputShaper { axis, params, state: RefCell::new(state), saved: RefCell::new(None) })
    }

    pub fn get_name(&self) -> String {
        format!("shaper_{}", self.axis)
    }

    fn update(&self, gcmd: &GCodeCommand) -> KResult<()> {
        self.params.update(gcmd)?;
        *self.state.borrow_mut() = self.params.get_shaper();
        Ok(())
    }

    fn set_shaper_kinematics(&self, sk: &SkRef) -> bool {
        let axis = self.axis.chars().next().unwrap();
        let (n, a, t) = self.state.borrow().clone();
        let success = input_shaper_set_shaper_params(sk, axis, n, &a, &t) == 0;
        if !success {
            self.disable_shaping();
            let (n, a, t) = self.state.borrow().clone();
            input_shaper_set_shaper_params(sk, axis, n, &a, &t);
        }
        success
    }

    fn disable_shaping(&self) {
        let st = self.state.borrow().clone();
        if self.saved.borrow().is_none() && st.0 != 0 {
            *self.saved.borrow_mut() = Some(st);
        }
        let (a, t) = get_none_shaper();
        *self.state.borrow_mut() = (a.len() as i32, a, t);
    }

    fn enable_shaping(&self) {
        let Some(saved) = self.saved.borrow_mut().take() else {
            // Input shaper was not disabled
            return;
        };
        *self.state.borrow_mut() = saved;
    }

    fn report(&self, gcmd: &GCodeCommand) {
        let info: Vec<String> = self.params.get_status().iter().map(|(key, value)| format!("{}_{}:{}", key, self.axis, value)).collect();
        gcmd.respond_info(&info.join(" "), true);
    }
}

const CMD_SET_INPUT_SHAPER_HELP: &str = "Set cartesian parameters for input shaper";

pub struct InputShaper {
    printer: Rc<Printer>,
    toolhead: RefCell<Option<Rc<ToolHead>>>,
    shapers: Vec<AxisInputShaper>,
    input_shaper_stepper_kinematics: RefCell<Vec<SkRef>>,
    orig_stepper_kinematics: RefCell<Vec<SkRef>>,
}

impl InputShaper {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // klippy:connect is registered before the shapers read the config
        let slot: Rc<RefCell<std::rc::Weak<InputShaper>>> = Rc::new(RefCell::new(std::rc::Weak::new()));
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match sl.borrow().upgrade() {
                Some(is) => is.connect(),
                None => Ok(()),
            }),
        );
        let shapers = vec![AxisInputShaper::new("x", config)?, AxisInputShaper::new("y", config)?];
        let is = Rc::new(InputShaper {
            printer: printer.clone(),
            toolhead: RefCell::new(None),
            shapers,
            input_shaper_stepper_kinematics: RefCell::new(vec![]),
            orig_stepper_kinematics: RefCell::new(vec![]),
        });
        *slot.borrow_mut() = Rc::downgrade(&is);
        // Register gcode commands
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let i = is.clone();
        gcode.register_command("SET_INPUT_SHAPER", Some(Rc::new(move |g| i.cmd_set_input_shaper(g))), false, Some(CMD_SET_INPUT_SHAPER_HELP))?;
        Ok(is)
    }

    pub fn get_shapers(&self) -> &[AxisInputShaper] {
        &self.shapers
    }

    fn connect(&self) -> KResult<()> {
        *self.toolhead.borrow_mut() = Some(self.printer.lookup::<ToolHead>("toolhead")?);
        // Configure initial values
        self.update_input_shaping(true)
    }

    fn get_input_shaper_stepper_kinematics(&self, stepper: &Rc<McuStepper>) -> Option<SkRef> {
        // Lookup stepper kinematics
        let sk = stepper.get_stepper_kinematics()?;
        if self.orig_stepper_kinematics.borrow().iter().any(|s| Rc::ptr_eq(s, &sk)) {
            // Already processed this stepper kinematics unsuccessfully
            return None;
        }
        if self.input_shaper_stepper_kinematics.borrow().iter().any(|s| Rc::ptr_eq(s, &sk)) {
            return Some(sk);
        }
        self.orig_stepper_kinematics.borrow_mut().push(sk.clone());
        let is_sk = input_shaper_alloc();
        stepper.set_stepper_kinematics(Some(is_sk.clone()));
        let res = input_shaper_set_sk(&is_sk, &sk);
        if res < 0 {
            stepper.set_stepper_kinematics(Some(sk));
            return None;
        }
        self.input_shaper_stepper_kinematics.borrow_mut().push(is_sk.clone());
        Some(is_sk)
    }

    fn toolhead(&self) -> KResult<Rc<ToolHead>> {
        self.toolhead.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'flush_step_generation'".into()))
    }

    pub fn update_kinematics(&self) -> KResult<()> {
        let Some(toolhead) = self.toolhead.borrow().clone() else {
            // Klipper initialization is not yet completed
            return Ok(());
        };
        toolhead.flush_step_generation()?;
        let kin = toolhead.get_kinematics();
        for s in kin.get_steppers() {
            if s.get_trapq().is_none() {
                continue;
            }
            let Some(is_sk) = self.get_input_shaper_stepper_kinematics(&s) else { continue };
            input_shaper_update_sk(&is_sk);
        }
        self.printer.lookup::<PrinterMotionQueuing>("motion_queuing")?.check_step_generation_scan_windows();
        Ok(())
    }

    /// _update_input_shaping(error): config errors at connect time
    fn update_input_shaping(&self, config_error: bool) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let kin = toolhead.get_kinematics();
        let mut failed_shapers: Vec<usize> = vec![];
        for s in kin.get_steppers() {
            if s.get_trapq().is_none() {
                continue;
            }
            let Some(is_sk) = self.get_input_shaper_stepper_kinematics(&s) else { continue };
            for (i, shaper) in self.shapers.iter().enumerate() {
                if failed_shapers.contains(&i) {
                    continue;
                }
                if !shaper.set_shaper_kinematics(&is_sk) {
                    failed_shapers.push(i);
                }
            }
        }
        self.printer.lookup::<PrinterMotionQueuing>("motion_queuing")?.check_step_generation_scan_windows();
        if !failed_shapers.is_empty() {
            let names: Vec<String> = failed_shapers.iter().map(|i| self.shapers[*i].get_name()).collect();
            let msg = format!("Failed to configure shaper(s) {} with given parameters", names.join(", "));
            return Err(if config_error { KError::config(msg) } else { KError::command(msg) });
        }
        Ok(())
    }

    pub fn disable_shaping(&self) -> KResult<()> {
        for shaper in &self.shapers {
            shaper.disable_shaping();
        }
        self.update_input_shaping(false)
    }

    pub fn enable_shaping(&self) -> KResult<()> {
        for shaper in &self.shapers {
            shaper.enable_shaping();
        }
        self.update_input_shaping(false)
    }

    pub fn cmd_set_input_shaper(&self, gcmd: &GCodeCommand) -> KResult<()> {
        if !gcmd.get_command_parameters().is_empty() {
            for shaper in &self.shapers {
                shaper.update(gcmd)?;
            }
            self.update_input_shaping(false)?;
        }
        for shaper in &self.shapers {
            shaper.report(gcmd);
        }
        Ok(())
    }
}

impl PrinterObject for InputShaper {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn teardown(&self) {
        *self.toolhead.borrow_mut() = None;
        self.input_shaper_stepper_kinematics.borrow_mut().clear();
        self.orig_stepper_kinematics.borrow_mut().clear();
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(InputShaper::new(config)?)
}
