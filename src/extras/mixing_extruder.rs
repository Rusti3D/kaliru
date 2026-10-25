// Code for supporting mixing extruders (port of klippy/extras/mixing_extruder.py)
//
// Copyright (C) 2022  Peter Gruber <peter@gruber.co.at>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::{KError, KResult};
use crate::extras::extruder_stepper::PrinterExtruderStepper;
use crate::extras::gcode_move::{GCodeMove, MoveTransform, MoveTransformRef};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::kinematics::extruder::{ExtruderStepper, PrinterExtruder};
use crate::chelper::trapq::TrapQRef;
use crate::printer::{downcast_rc, PObj, Printer, PrinterObject};
use crate::pyfmt::{py_float, py_strip, py_sum};
use crate::pyval::PyVal;
use crate::stepper::McuStepper;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const EXTRUSION_FACTOR_THRESHOLD: f64 = 0.01;

// Remove scales which are to close to zero to be used as
// recipocal of the rotation_distance. Rescale others to compensate.
fn normalize_mixing(mixing: &[f64]) -> Vec<f64> {
    let normalized: Vec<f64> = mixing.iter().map(|m| if *m < EXTRUSION_FACTOR_THRESHOLD { 0.0 } else { *m }).collect();
    let s = py_sum(normalized.iter().copied());
    normalized.iter().map(|m| m / s).collect()
}

fn zero_division() -> KError {
    KError::Other("float division by zero".into())
}

fn mixing_preset(idx: i64, steppers_count: usize) -> KResult<Vec<f64>> {
    let n = steppers_count as i64;
    if idx == 0 {
        return Ok(vec![1.0 / n as f64; steppers_count]);
    }
    let mut idx = idx - 1;
    if idx < n {
        return Ok((0..n).map(|p| if p == idx { 1.0 } else { 0.0 }).collect());
    }
    idx -= n;
    if idx < n {
        if n == 1 {
            return Err(zero_division());
        }
        return Ok((0..n).map(|p| if p == idx { 0.0 } else { 1.0 / (n - 1) as f64 }).collect());
    }
    idx -= n;
    if n == 3 {
        if idx < 2 * n {
            let mut all = vec![];
            for x in 0..n {
                for y in [0, 1] {
                    all.push((0..n).map(|p| if p == x { 0.0 } else { (1 + (((p - x).rem_euclid(3) + y) % 2)) as f64 / 3.0 }).collect::<Vec<f64>>());
                }
            }
            return Ok(all.swap_remove(idx as usize));
        }
    } else if n > 3 && (idx as f64) < (n * (n - 1)) as f64 / 2.0 {
        let mut all = vec![];
        for x in 0..n {
            for y in x + 1..n {
                all.push((0..n).map(|p| if p == x || p == y { 0.0 } else { 1.0 / (n - 2) as f64 }).collect::<Vec<f64>>());
            }
        }
        return Ok(all.swap_remove(idx as usize));
    }
    Ok(vec![1.0 / n as f64; steppers_count])
}

fn pct_list(mixing: &[f64], suffix: &str) -> String {
    mixing.iter().map(|m| format!("{:.1}{}", m * 100.0, suffix)).collect::<Vec<_>>().join(",")
}

/// a gradient: (start, middle, end) heights and (start, end) mixings
type Gradient = ((f64, f64, f64), (Vec<f64>, Vec<f64>));

const CMD_SET_MIXING_EXTRUDER_HELP: &str = "Set scale on stepper";
const CMD_SET_MIXING_EXTRUDER_GRADIENT_HELP: &str = "Configure gradient mixing";

pub struct MixingExtruder {
    me: Weak<MixingExtruder>,
    printer: Rc<Printer>,
    activated: Cell<bool>,
    name: String,
    stepper_names: Vec<String>,
    extruder_steppers: RefCell<Vec<(Rc<ExtruderStepper>, Rc<McuStepper>)>>,
    rotation_distances: RefCell<Vec<f64>>,
    trapqs: RefCell<Vec<Option<TrapQRef>>>,
    extruder_name: String,
    mixing: RefCell<Vec<f64>>,
    /// self._active_mixing; None while it is the same list as self._mixing
    active_mixing: RefCell<Option<Vec<f64>>>,
    enabled: Cell<bool>,
    // GradientMixingExtruder
    gradient_enabled: Cell<bool>,
    gradients: RefCell<Vec<Gradient>>,
    gradient_method: RefCell<String>,
    gradient_vector: Cell<(f64, f64, f64)>,
    gcode_move: Rc<GCodeMove>,
    normal_transform: RefCell<Option<MoveTransformRef>>,
}

impl MixingExtruder {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let stepper_names: Vec<String> =
            config.get("steppers", "")?.split(',').filter(|e| !e.is_empty()).map(|e| py_strip(e).to_string()).collect();
        if stepper_names.is_empty() {
            return Err(KError::config("No steppers configured for mixing"));
        }
        let extruder_name = config.get("extruder_name", stepper_names[0].as_str())?;
        let mixing = mixing_preset(0, stepper_names.len())?;
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        // (GradientMixingExtruder) gcode_move is loaded after the base
        // class registered its handlers and command
        let slot: Rc<RefCell<Weak<MixingExtruder>>> = Rc::new(RefCell::new(Weak::new()));
        let sl = slot.clone();
        printer.register_event_handler(
            "klippy:connect",
            Rc::new(move |_| match sl.borrow().upgrade() {
                Some(m) => m.handle_connect(),
                None => Ok(()),
            }),
        );
        // Register commands
        let sl = slot.clone();
        gcode.register_command(
            "SET_MIXING_EXTRUDER",
            Some(Rc::new(move |g| match sl.borrow().upgrade() {
                Some(m) => m.cmd_set_mixing_extruder(g),
                None => Ok(()),
            })),
            false,
            Some(CMD_SET_MIXING_EXTRUDER_HELP),
        )?;
        let gcode_move = printer.load_as::<GCodeMove>(config, "gcode_move")?;
        let m = Rc::new_cyclic(|me| MixingExtruder {
            me: me.clone(),
            printer: printer.clone(),
            activated: Cell::new(false),
            name: "mixingextruder".to_string(),
            stepper_names,
            extruder_steppers: RefCell::new(vec![]),
            rotation_distances: RefCell::new(vec![]),
            trapqs: RefCell::new(vec![]),
            extruder_name,
            mixing: RefCell::new(mixing),
            active_mixing: RefCell::new(None),
            enabled: Cell::new(false),
            gradient_enabled: Cell::new(false),
            gradients: RefCell::new(vec![]),
            gradient_method: RefCell::new("linear".to_string()),
            gradient_vector: Cell::new((0.0, 0.0, 1.0)),
            gcode_move,
            normal_transform: RefCell::new(None),
        });
        *slot.borrow_mut() = Rc::downgrade(&m);
        let w = m.me.clone();
        gcode.register_command(
            "SET_MIXING_EXTRUDER_GRADIENT",
            Some(Rc::new(move |g| match w.upgrade() {
                Some(m) => m.cmd_set_mixing_extruder_gradient(g),
                None => Ok(()),
            })),
            false,
            Some(CMD_SET_MIXING_EXTRUDER_GRADIENT_HELP),
        )?;
        Ok(m)
    }

    pub fn get_name(&self) -> &str {
        &self.name
    }

    fn handle_connect(&self) -> KResult<()> {
        if self.activated.get() {
            return Ok(());
        }
        self.activated.set(true);
        for name in &self.stepper_names {
            let printer_extruder_name = if name != "extruder" { format!("extruder_stepper {}", name) } else { name.clone() };
            let obj = self.printer.lookup_object(&printer_extruder_name)?;
            let es = match downcast_rc::<PrinterExtruderStepper>(obj.clone()) {
                Some(p) => Some(p.extruder_stepper.clone()),
                None => downcast_rc::<PrinterExtruder>(obj).and_then(|p| p.extruder_stepper.clone()),
            };
            let es = es.ok_or_else(|| KError::Other("object has no attribute 'extruder_stepper'".into()))?;
            let stepper = es.stepper.clone();
            self.extruder_steppers.borrow_mut().push((es, stepper));
        }
        let es = self.extruder_steppers.borrow();
        *self.rotation_distances.borrow_mut() = es.iter().map(|(_, s)| s.get_rotation_distance().0).collect();
        *self.trapqs.borrow_mut() = es.iter().map(|(_, s)| s.get_trapq()).collect();
        Ok(())
    }

    fn active_mixing(&self) -> Vec<f64> {
        self.active_mixing.borrow().clone().unwrap_or_else(|| self.mixing.borrow().clone())
    }

    fn ticks(&self) -> String {
        self.extruder_steppers.borrow().iter().map(|(_, s)| s.get_mcu_position(None).to_string()).collect::<Vec<_>>().join(",")
    }

    fn init_mixing(&self) -> KResult<()> {
        for (es, _) in self.extruder_steppers.borrow().iter() {
            es.sync_to_extruder(&self.extruder_name)?;
        }
        Ok(())
    }

    fn restore_extruder(&self) -> KResult<()> {
        self.printer.lookup::<ToolHead>("toolhead")?.flush_step_generation()?;
        let rds = self.rotation_distances.borrow();
        let tqs = self.trapqs.borrow();
        for (i, (es, _)) in self.extruder_steppers.borrow().iter().enumerate() {
            es.stepper.set_rotation_distance(rds[i]);
            es.stepper.set_trapq(tqs[i].clone());
        }
        Ok(())
    }

    fn apply_mixing(&self, mixing: Option<Vec<f64>>) -> KResult<()> {
        let has_mixing = mixing.as_ref().map(|m| !m.is_empty()).unwrap_or(false);
        if has_mixing && !self.enabled.get() {
            self.init_mixing()?;
        } else if self.enabled.get() && !has_mixing {
            return self.restore_extruder();
        }
        if !self.enabled.get() {
            return Ok(());
        }
        let active = normalize_mixing(&mixing.unwrap_or_default());
        *self.active_mixing.borrow_mut() = Some(active.clone());
        let rds = self.rotation_distances.borrow().clone();
        for (i, (es, stepper)) in self.extruder_steppers.borrow().iter().enumerate() {
            let has_trapq = stepper.get_trapq().is_some();
            if active[i] > 0.0 && has_trapq {
                es.set_rotation_distance(rds[i] / active[i]);
            } else if active[i] > 0.0 && !has_trapq {
                es.sync_to_extruder(&self.extruder_name)?;
                es.set_rotation_distance(rds[i] / active[i]);
            } else if active[i] == 0.0 && has_trapq {
                es.sync_to_extruder("")?;
            }
        }
        Ok(())
    }

    fn cmd_set_mixing_extruder(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let truthy = |k: &str| gcmd.get_opt(k).map(|v| !v.is_empty()).unwrap_or(false);
        if truthy("FACTORS") || truthy("PRESET") || truthy("ENABLE") {
            let factors = gcmd.get_opt("FACTORS");
            let preset = gcmd.get_int("PRESET", Some(0), Lim::new().min(0))?;
            let enable = gcmd.get_int("ENABLE", Some(1), Lim::new().min(0).max(1))?;
            let n = self.stepper_names.len();
            let mut ratios: Vec<f64> = (0..n).map(|p| if p == 0 { 1.0 } else { 0.0 }).collect();
            match factors.filter(|f| !f.is_empty()) {
                Some(factors) => {
                    let mut parsed = vec![];
                    for s in factors.replace(',', ":").split(':') {
                        match py_float(s) {
                            Some(v) => parsed.push(v),
                            // python formats e.message, which py3 exceptions lack
                            None => return Err(KError::Other("'ValueError' object has no attribute 'message'".into())),
                        }
                    }
                    if parsed.len() > n {
                        return Err(KError::Other("'Exception' object has no attribute 'message'".into()));
                    }
                    for (index, factor) in parsed.into_iter().enumerate() {
                        ratios[index] = factor;
                    }
                }
                None => ratios = mixing_preset(preset, n)?,
            }
            if enable != 0 {
                let s = py_sum(ratios.iter().copied());
                if s <= 0.0 {
                    return Err(gcmd.error("Could not save ratio: its empty"));
                }
                {
                    let mut mixing = self.mixing.borrow_mut();
                    for (i, v) in ratios.iter().enumerate() {
                        mixing[i] = v / s;
                    }
                }
                self.enabled.set(true);
                let mixing = self.mixing.borrow().clone();
                self.apply_mixing(Some(mixing))?;
            } else {
                self.apply_mixing(None)?;
                self.enabled.set(false);
            }
        } else {
            let mix = "mixing ";
            gcmd.respond_info(&format!("{}{}", mix, if self.enabled.get() { "enabled" } else { "disabled" }), true);
            gcmd.respond_info(&format!("{}{}", mix, pct_list(&self.active_mixing(), "%")), true);
            gcmd.respond_info(&format!("{}{}", mix, self.ticks()), true);
            let names: Vec<String> = self.extruder_steppers.borrow().iter().map(|(es, _)| es.name.clone()).collect();
            gcmd.respond_info(&format!("{}{}", mix, names.join(",")), true);
        }
        Ok(())
    }

    // GradientMixingExtruder

    fn enable_gradient(&self) -> KResult<()> {
        if self.normal_transform.borrow().is_some() {
            return Ok(());
        }
        if !self.enabled.get() {
            self.init_mixing()?;
        }
        let me: MoveTransformRef = self.me.upgrade().expect("mixing extruder");
        let nt = self.gcode_move.set_move_transform(me, true)?;
        *self.normal_transform.borrow_mut() = nt;
        let gcode_pos = self.gcode_position();
        let g = self.get_gradient(&gcode_pos)?;
        self.apply_mixing(Some(g))
    }

    fn disable_gradient(&self) -> KResult<()> {
        let nt = self.normal_transform.borrow_mut().take();
        if let Some(nt) = nt {
            self.gcode_move.set_move_transform(nt, true)?;
        }
        Ok(())
    }

    fn gcode_position(&self) -> Vec<f64> {
        match self.gcode_move.get_status(0.0) {
            Some(PyVal::Dict(d)) => match d.get("gcode_position") {
                Some(PyVal::NamedTuple(_, _, v)) | Some(PyVal::List(v)) | Some(PyVal::Tuple(v)) => v.iter().map(|x| x.as_f64().unwrap_or(0.0)).collect(),
                _ => vec![],
            },
            _ => vec![],
        }
    }

    fn get_gradient(&self, pos: &[f64]) -> KResult<Vec<f64>> {
        let mut mixing = self.mixing.borrow().clone();
        let vec = self.gradient_vector.get();
        let gv = [vec.0, vec.1, vec.2];
        let method = self.gradient_method.borrow().clone();
        for (heights, refs) in self.gradients.borrow().iter() {
            let (start, _, end) = *heights;
            let (start_mix, end_mix) = refs;
            let lerp = |w: f64| start_mix.iter().zip(end_mix).map(|(s, e)| (1.0 - w) * s + w * e).collect::<Vec<f64>>();
            if method == "linear" {
                let zpos = py_sum((0..3).map(|i| pos[i] * gv[i]));
                if zpos <= start {
                    return Ok(start_mix.clone());
                }
                if zpos >= end {
                    mixing = end_mix.clone();
                    continue;
                }
                return Ok(lerp((zpos - start) / (end - start)));
            }
            if method == "spherical" {
                if pos.len() > 3 {
                    // enumerate(pos) runs past the 3 item gradient vector
                    return Err(KError::Other("tuple index out of range".into()));
                }
                let dist = py_sum(pos.iter().enumerate().map(|(i, x)| (x - gv[i]).powf(2.0))).sqrt();
                if dist <= start {
                    return Ok(start_mix.clone());
                }
                if dist >= end {
                    mixing = end_mix.clone();
                    continue;
                }
                return Ok(lerp((dist - start) / (end - start)));
            }
        }
        Ok(mixing)
    }

    fn check_scales(&self, gcmd: &GCodeCommand, s: &str) -> KResult<Vec<f64>> {
        let scales: Vec<String> = s.replace(',', ":").split(':').map(|x| x.to_string()).collect();
        if scales.len() != self.mixing.borrow().len() {
            return Err(gcmd.error("Could not configure gradient: invalid scales"));
        }
        scales.iter().map(|x| py_float(x).ok_or_else(|| gcmd.error("Could not configure gradient: could not parse scales"))).collect()
    }

    fn cmd_set_mixing_extruder_gradient(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let get = |k: &str| gcmd.get_opt(k).unwrap_or_default();
        if !get("START_FACTORS").is_empty() {
            let mut start_scales = self.check_scales(gcmd, &gcmd.get("START_FACTORS")?)?;
            let mut end_scales = self.check_scales(gcmd, &gcmd.get("END_FACTORS")?)?;
            let mut start_height = gcmd.get_float("START_HEIGHT", None, Lim::new().min(0.0))?;
            let mut end_height = gcmd.get_float("END_HEIGHT", None, Lim::new().min(0.0))?;
            if start_height > end_height {
                std::mem::swap(&mut start_height, &mut end_height);
                std::mem::swap(&mut start_scales, &mut end_scales);
            }
            for ((s, _, e), _) in self.gradients.borrow().iter() {
                if *e > start_height && end_height > *s {
                    return Err(gcmd.error("Could not configure gradient: overlapping starts/ends"));
                }
            }
            let mut gradients = self.gradients.borrow_mut();
            gradients.push(((start_height, (start_height + end_height) / 2.0, end_height), (start_scales, end_scales)));
            gradients.sort_by(|a, b| a.0 .0.partial_cmp(&b.0 .0).unwrap_or(std::cmp::Ordering::Equal));
        } else if get("ENABLE") == "RESET" {
            self.gradient_enabled.set(false);
            self.gradients.borrow_mut().clear();
            *self.gradient_method.borrow_mut() = "linear".into();
            self.disable_gradient()?;
        } else if !get("ENABLE").is_empty() || !get("METHOD").is_empty() {
            let method = get("METHOD");
            if method == "linear" || method == "spherical" {
                *self.gradient_method.borrow_mut() = method;
            }
            let vector = gcmd.get_or("VECTOR", "0,0,1");
            let parts: Vec<&str> = vector.split(',').collect();
            let v: Option<Vec<f64>> = (0..3).map(|i| parts.get(i).and_then(|p| py_float(p))).collect();
            let Some(v) = v else {
                return Err(gcmd.error("Could not configure gradient: invalid vector"));
            };
            self.gradient_vector.set((v[0], v[1], v[2]));
            let enable = match gcmd.get_int("ENABLE", Some(1), Lim::new().min(0).max(1)) {
                Ok(e) => e == 1,
                Err(_) => get("ENABLE").to_lowercase() == "true",
            };
            if enable {
                self.enable_gradient()?;
            } else {
                self.disable_gradient()?;
            }
            self.gradient_enabled.set(enable);
        } else {
            let method = self.gradient_method.borrow().clone();
            let gv = self.gradient_vector.get();
            for (i, ((s, m, e), (a, b))) in self.gradients.borrow().iter().enumerate() {
                let grad = format!("gradient[{}] ", i);
                let mix = |v: &[f64]| v.iter().map(|x| format!("{:.1}", x)).collect::<Vec<_>>().join("/");
                gcmd.respond_info(&format!("{}heights={:.1}-({:.1})-{:.1}", grad, s, m, e), true);
                gcmd.respond_info(&format!("{}mixing={}-{}", grad, mix(a), mix(b)), true);
                gcmd.respond_info(&format!("{}method={}", grad, method), true);
                gcmd.respond_info(&format!("{}vector={}", grad, mix(&[gv.0, gv.1, gv.2])), true);
                gcmd.respond_info(&format!("{}enabled={}", grad, if self.gradient_enabled.get() { "True" } else { "False" }), true);
            }
        }
        Ok(())
    }
}

impl MoveTransform for MixingExtruder {
    /// (python's get_position() returns None, which breaks the next gcode
    /// position reset; the normal transform's position is returned here)
    fn get_position(&self) -> Vec<f64> {
        self.normal_transform.borrow().as_ref().map(|t| t.get_position()).unwrap_or_default()
    }

    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        let gcode_pos = self.gcode_position();
        let g = self.get_gradient(&gcode_pos)?;
        self.apply_mixing(Some(g))?;
        let nt = self.normal_transform.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'move'".into()))?;
        nt.move_to(newpos, speed)
    }
}

impl PrinterObject for MixingExtruder {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("mixing".to_string(), PyVal::Str(pct_list(&self.active_mixing(), "")));
        d.insert("ticks".to_string(), PyVal::Str(self.ticks()));
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        self.extruder_steppers.borrow_mut().clear();
        *self.normal_transform.borrow_mut() = None;
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(MixingExtruder::new(config)?)
}
