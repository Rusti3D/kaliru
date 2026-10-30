// klippy/stepper.py: MCU_stepper, PrinterRail, PrinterStepper,
// LookupMultiRail. A stepper's kinematics and trapq are chelper objects
// (impls/chelper.rs); its endstops kaliru's own or a plugin's (impls/pins.rs).
use super::chelper::{ffi_fn, lib_fn, sk_from_py, sk_to_py, trapq_from_py, trapq_to_py};
use super::pins::{endstop_from_py, endstop_to_py, pin_params_from_py};
use super::*;
use crate::configfile::ConfigWrapper;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::{Args, DictField, ListField};
use crate::stepper::{McuStepper, PrinterRail, Rail, TmcCurrentHelper};
use crate::{cell_attr, function, method, ro_attr};

fn cfg(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
    super::configfile::config_from_py(o)
}

fn st(o: &PyRef) -> KResult<Rc<McuStepper>> {
    objects::unwrap_as::<McuStepper>(o)
}

fn rl(o: &PyRef) -> KResult<Rc<PrinterRail>> {
    objects::unwrap_as::<PrinterRail>(o)
}

/// A plugin's tmc current helper (stepper.set_tmc_current_helper(obj))
struct PyTmcCurrentHelper {
    obj: PyRef,
}

impl TmcCurrentHelper for PyTmcCurrentHelper {
    fn set_current_for_homing(&self, print_time: f64, pre_homing: bool) -> KResult<f64> {
        self.obj.call_method("set_current_for_homing", &[&py_float(print_time), &py_bool(pre_homing)])?.as_f64()
    }
}

fn tmc_to_py(h: Option<Rc<dyn TmcCurrentHelper>>) -> KResult<PyRef> {
    let Some(h) = h else { return Ok(py_none()) };
    if let Some(p) = (h.as_ref() as &dyn Any).downcast_ref::<PyTmcCurrentHelper>() {
        return Ok(p.obj.clone());
    }
    let any: Rc<dyn Any> = h.clone();
    let name = h.type_name();
    objects::wrap_view(any, h, name)
}

fn tmc_from_py(o: &PyRef) -> KResult<Option<Rc<dyn TmcCurrentHelper>>> {
    if o.is_none() {
        return Ok(None);
    }
    if is_mirror_instance(o)? {
        if let Some(h) = objects::unwrap_view::<dyn TmcCurrentHelper>(o) {
            return Ok(Some(h));
        }
    }
    Ok(Some(Rc::new(PyTmcCurrentHelper { obj: o.clone() })))
}

/// setup_itersolve(alloc_func, *params): getattr(ffi_lib, alloc_func)(*params)
fn alloc_sk(a: &Args) -> KResult<crate::chelper::itersolve::SkRef> {
    let params: Vec<PyRef> = FromPy::from_py(a.arg(1))?;
    let refs: Vec<&PyRef> = params.iter().collect();
    let sk = ffi_fn(&a.string(0)?)?.call(&refs)?;
    sk_from_py(&sk)?.ok_or_else(|| KError::Py("TypeError".into(), format!("kaliru: {} returned NULL", a.string(0).unwrap_or_default())))
}

fn steppers_to_py(steppers: Vec<Rc<McuStepper>>) -> KResult<PyRef> {
    Ok(py_list(steppers.into_iter().map(objects::wrap).collect::<KResult<Vec<_>>>()?))
}

fn endstops_to_py(endstops: Vec<(crate::mcu::EndstopRef, String)>) -> KResult<PyRef> {
    let items = endstops.iter().map(|(e, n)| Ok(py_tuple(vec![endstop_to_py(e)?, py_str(n)]))).collect::<KResult<Vec<_>>>()?;
    Ok(py_list(items))
}

pub fn register(r: &mut Registry) {
    objects::register_class::<crate::chelper::steppersync::SyncEmitter>("_k", "CData");

    // ------------------------------------------------------- MCU_stepper
    r.ctor("klippy.stepper.MCU_stepper.__init__", |a| {
        let s: Rc<dyn Any> = McuStepper::new(
            &*cfg(a.arg(0))?,
            &pin_params_from_py(a.arg(1))?,
            &pin_params_from_py(a.arg(2))?,
            a.f64(3)?,
            a.f64(4)?,
            a.opt_f64(5)?,
            a.bool(6)?,
        )?;
        Ok(s)
    });
    r.method("klippy.stepper.MCU_stepper.get_tmc_current_helper", |a| tmc_to_py(a.this::<McuStepper>()?.get_tmc_current_helper()));
    r.method("klippy.stepper.MCU_stepper.set_tmc_current_helper", |a| {
        a.this::<McuStepper>()?.set_tmc_current_helper(tmc_from_py(a.arg(0))?);
        Ok(py_none())
    });
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_mcu(s) => s.get_mcu());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_name(s, short: bool) => s.get_name(short));
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, units_in_radians(s) => s.units_in_radians());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_pulse_duration(s) => s.get_pulse_duration());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, setup_default_pulse_duration(s, pulse_duration: f64, step_both_edge: bool) =>
        s.setup_default_pulse_duration(pulse_duration, step_both_edge));
    r.method("klippy.stepper.MCU_stepper.setup_itersolve", |a| {
        a.this::<McuStepper>()?.setup_itersolve(alloc_sk(a)?);
        Ok(py_none())
    });
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, _build_config(s) => s.build_config());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_oid(s) => s.get_oid());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_step_dist(s) => s.get_step_dist());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_rotation_distance(s) => s.get_rotation_distance());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, set_rotation_distance(s, rotation_dist: f64) => s.set_rotation_distance(rotation_dist));
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_dir_inverted(s) => s.get_dir_inverted());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, set_dir_inverted(s, invert_dir: bool) => s.set_dir_inverted(invert_dir));
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, calc_position_from_coord(s, coord: Vec<f64>) => s.calc_position_from_coord(&coord));
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, set_position(s, coord: Vec<f64>) => s.set_position(&coord));
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_commanded_position(s) => s.get_commanded_position());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_mcu_position(s, cmd_pos: Option<f64>) => s.get_mcu_position(cmd_pos));
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, _set_mcu_position(s, mcu_pos: i64) => s.set_mcu_position(mcu_pos));
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, get_past_mcu_position(s, print_time: f64) => s.get_past_mcu_position(print_time));
    // mcu_pos * step_dist - offset: an int or a float position
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, mcu_to_commanded_position(s, mcu_pos: f64) => s.mcu_to_commanded_position_f(mcu_pos));
    // dump_steps(count, start_clock, end_clock) -> (data, count)
    r.method("klippy.stepper.MCU_stepper.dump_steps", |a| {
        let count = a.arg(0).as_i64()?.max(0) as usize;
        let steps = a.this::<McuStepper>()?.dump_steps(count, a.arg(1).as_i64()? as u64, a.arg(2).as_i64()? as u64);
        let ffi = lookup("_k", "get_ffi")?.call(&[])?.call_method("__getitem__", &[&py_int(0)])?;
        let data = ffi.call_method("new", &[&py_str("struct pull_history_steps[]"), &py_int(count as i64)])?;
        for (i, s) in steps.iter().enumerate() {
            let p = data.call_method("__getitem__", &[&py_int(i as i64)])?;
            p.setattr("first_clock", &py_int(s.first_clock as i64))?;
            p.setattr("last_clock", &py_int(s.last_clock as i64))?;
            p.setattr("start_position", &py_int(s.start_position))?;
            p.setattr("step_count", &py_int(s.step_count as i64))?;
            p.setattr("interval", &py_int(s.interval as i64))?;
            p.setattr("add", &py_int(s.add as i64))?;
        }
        Ok(py_tuple(vec![data, py_int(steps.len() as i64)]))
    });
    r.method("klippy.stepper.MCU_stepper.get_stepper_kinematics", |a| sk_to_py(a.this::<McuStepper>()?.get_stepper_kinematics()));
    r.method("klippy.stepper.MCU_stepper.set_stepper_kinematics", |a| {
        sk_to_py(a.this::<McuStepper>()?.set_stepper_kinematics(sk_from_py(a.arg(0))?))
    });
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, note_homing_end(s) => s.note_homing_end());
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, _query_mcu_position(s) => s.query_mcu_position());
    r.method("klippy.stepper.MCU_stepper.get_trapq", |a| trapq_to_py(a.this::<McuStepper>()?.get_trapq()));
    r.method("klippy.stepper.MCU_stepper.set_trapq", |a| trapq_to_py(a.this::<McuStepper>()?.set_trapq(trapq_from_py(a.arg(0))?)));
    // add_active_callback(cb): cb(print_time)
    r.method("klippy.stepper.MCU_stepper.add_active_callback", |a| {
        let cb = a.arg(0).clone();
        a.this::<McuStepper>()?.add_active_callback(Rc::new(move |pt| {
            cb.call(&[&py_float(pt)])?;
            Ok(())
        }))?;
        Ok(py_none())
    });
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, _check_active(s, must_flush_time: f64, max_step_gen_time: f64) =>
        s.check_active(must_flush_time, max_step_gen_time));
    method!(r, "klippy.stepper.MCU_stepper", McuStepper, is_active_axis(s, axis: String) =>
        s.is_active_axis(axis.chars().next().unwrap_or('\0')) as i64);
    // attributes
    ro_attr!(r, "klippy.stepper.MCU_stepper._name", McuStepper, |s| s.name.clone());
    ro_attr!(r, "klippy.stepper.MCU_stepper._oid", McuStepper, |s| s.oid);
    ro_attr!(r, "klippy.stepper.MCU_stepper._mcu", McuStepper, |s| s.mcu.clone());
    ro_attr!(r, "klippy.stepper.MCU_stepper._steps_per_rotation", McuStepper, |s| s.steps_per_rotation);
    ro_attr!(r, "klippy.stepper.MCU_stepper._units_in_radians", McuStepper, |s| s.units_in_radians);
    ro_attr!(r, "klippy.stepper.MCU_stepper._step_pin", McuStepper, |s| s.step_pin.clone());
    ro_attr!(r, "klippy.stepper.MCU_stepper._invert_step", McuStepper, |s| s.invert_step);
    ro_attr!(r, "klippy.stepper.MCU_stepper._dir_pin", McuStepper, |s| s.dir_pin.clone());
    ro_attr!(r, "klippy.stepper.MCU_stepper._orig_invert_dir", McuStepper, |s| s.orig_invert_dir);
    cell_attr!(r, "klippy.stepper.MCU_stepper._rotation_dist", McuStepper, rotation_dist);
    cell_attr!(r, "klippy.stepper.MCU_stepper._step_dist", McuStepper, step_dist);
    cell_attr!(r, "klippy.stepper.MCU_stepper._step_pulse_duration", McuStepper, step_pulse_duration);
    cell_attr!(r, "klippy.stepper.MCU_stepper._invert_dir", McuStepper, invert_dir);
    cell_attr!(r, "klippy.stepper.MCU_stepper._step_both_edge", McuStepper, step_both_edge);
    cell_attr!(r, "klippy.stepper.MCU_stepper._req_step_both_edge", McuStepper, req_step_both_edge);
    cell_attr!(r, "klippy.stepper.MCU_stepper._mcu_position_offset", McuStepper, mcu_position_offset);
    cell_attr!(r, "klippy.stepper.MCU_stepper._reset_cmd_tag", McuStepper, reset_cmd_tag);
    r.attr("klippy.stepper.MCU_stepper._stepper_kinematics", |o| sk_to_py(st(o)?.get_stepper_kinematics()), None);
    r.attr("klippy.stepper.MCU_stepper._trapq", |o| trapq_to_py(st(o)?.get_trapq()), None);
    r.attr("klippy.stepper.MCU_stepper._tmc_current_helper", |o| tmc_to_py(st(o)?.get_tmc_current_helper()), Some(|o, v| {
        st(o)?.set_tmc_current_helper(tmc_from_py(v)?);
        Ok(())
    }));
    r.attr("klippy.stepper.MCU_stepper._syncemitter", |o| st(o)?.syncemitter.clone().to_py(), None);
    // self._itersolve_check_active = ffi_lib.itersolve_check_active
    r.attr("klippy.stepper.MCU_stepper._itersolve_check_active", |_o| lib_fn("itersolve_check_active"), None);
    // (a snapshot: kaliru's callbacks are its own closures)
    r.attr("klippy.stepper.MCU_stepper._active_callbacks", |o| {
        let n = st(o)?.active_callbacks.borrow().len();
        let cbs = (0..n)
            .map(|i| {
                let s = st(o)?;
                py_function_of(move |a| {
                    let cb = s.active_callbacks.borrow().get(i).cloned();
                    if let Some(cb) = cb {
                        cb(a[0].as_f64()?)?;
                    }
                    Ok(py_none())
                })
            })
            .collect::<KResult<Vec<_>>>()?;
        Ok(py_list(cbs))
    }, None);

    // ----------------------------------------------------- module functions
    r.method("klippy.stepper.PrinterStepper", |a| crate::stepper::printer_stepper(&*cfg(a.arg(0))?, a.bool(1)?)?.to_py());
    function!(r, "klippy.stepper", parse_gear_ratio(config: PyRef, note_valid: bool) =>
        crate::stepper::parse_gear_ratio(&*cfg(&config)?, note_valid));
    function!(r, "klippy.stepper", parse_step_distance(config: PyRef, units_in_radians: Option<bool>, note_valid: bool) =>
        crate::stepper::parse_step_distance(&*cfg(&config)?, units_in_radians, note_valid));
    r.method("klippy.stepper.LookupMultiRail", |a| {
        crate::stepper::lookup_multi_rail(&*cfg(a.arg(0))?, a.bool(1)?, a.opt_f64(2)?, a.bool(3)?)?.to_py()
    });

    // ------------------------------------------------------- PrinterRail
    r.ctor("klippy.stepper.PrinterRail.__init__", |a| {
        let rail: Rc<dyn Any> = PrinterRail::new(&*cfg(a.arg(0))?, a.bool(1)?, a.opt_f64(2)?, a.bool(3)?)?;
        Ok(rail)
    });
    r.method("klippy.stepper.PrinterRail.get_tmc_current_helpers", |a| {
        let hs = Rail::get_tmc_current_helpers(&*a.this::<PrinterRail>()?);
        Ok(py_list(hs.into_iter().map(tmc_to_py).collect::<KResult<Vec<_>>>()?))
    });
    method!(r, "klippy.stepper.PrinterRail", PrinterRail, get_range(p) => p.get_range());
    // get_homing_info(): Kalico's namedtuple "homing_info"
    r.method("klippy.stepper.PrinterRail.get_homing_info", |a| {
        let h = Rail::get_homing_info(&*a.this::<PrinterRail>()?);
        const FIELDS: &[&str] = &[
            "speed",
            "position_endstop",
            "retract_speed",
            "retract_dist",
            "positive_dir",
            "second_homing_speed",
            "use_sensorless_homing",
            "min_home_dist",
            "accel",
        ];
        let values = vec![
            PyVal::Float(h.speed),
            PyVal::Float(h.position_endstop),
            PyVal::Float(h.retract_speed),
            PyVal::Float(h.retract_dist),
            PyVal::Bool(h.positive_dir),
            PyVal::Float(h.second_homing_speed),
            PyVal::Bool(h.use_sensorless_homing),
            PyVal::Float(h.min_home_dist),
            h.accel.map(PyVal::Float).unwrap_or(PyVal::None),
        ];
        Ok(from_pyval(&PyVal::NamedTuple("homing_info", FIELDS, values)))
    });
    r.method("klippy.stepper.PrinterRail.get_steppers", |a| steppers_to_py(Rail::get_steppers(&*a.this::<PrinterRail>()?)));
    r.method("klippy.stepper.PrinterRail.get_endstops", |a| endstops_to_py(Rail::get_endstops(&*a.this::<PrinterRail>()?)));
    method!(r, "klippy.stepper.PrinterRail", PrinterRail, add_extra_stepper(p, config: PyRef) => p.add_extra_stepper(&*cfg(&config)?));
    r.method("klippy.stepper.PrinterRail.setup_itersolve", |a| {
        // (each stepper gets its own sk: alloc_func called once per stepper)
        let rail = a.this::<PrinterRail>()?;
        for s in rail.steppers.borrow().iter() {
            s.setup_itersolve(alloc_sk(a)?);
        }
        Ok(py_none())
    });
    r.method("klippy.stepper.PrinterRail.set_trapq", |a| {
        a.this::<PrinterRail>()?.set_trapq(trapq_from_py(a.arg(0))?);
        Ok(py_none())
    });
    method!(r, "klippy.stepper.PrinterRail", PrinterRail, set_position(p, coord: Vec<f64>) => p.set_position(&coord));
    // self.get_name / get_commanded_position / calc_position_from_coord =
    // the primary stepper's bound methods
    r.attr("klippy.stepper.PrinterRail.get_name", |o| objects::wrap(rl(o)?.steppers.borrow()[0].clone())?.getattr("get_name"), None);
    r.attr("klippy.stepper.PrinterRail.get_commanded_position", |o| {
        objects::wrap(rl(o)?.steppers.borrow()[0].clone())?.getattr("get_commanded_position")
    }, None);
    r.attr("klippy.stepper.PrinterRail.calc_position_from_coord", |o| {
        objects::wrap(rl(o)?.steppers.borrow()[0].clone())?.getattr("calc_position_from_coord")
    }, None);
    ro_attr!(r, "klippy.stepper.PrinterRail.stepper_units_in_radians", PrinterRail, |p| p.stepper_units_in_radians);
    cell_attr!(r, "klippy.stepper.PrinterRail.position_endstop", PrinterRail, position_endstop);
    cell_attr!(r, "klippy.stepper.PrinterRail.position_min", PrinterRail, position_min);
    cell_attr!(r, "klippy.stepper.PrinterRail.position_max", PrinterRail, position_max);
    cell_attr!(r, "klippy.stepper.PrinterRail.use_sensorless_homing", PrinterRail, use_sensorless_homing);
    cell_attr!(r, "klippy.stepper.PrinterRail.homing_speed", PrinterRail, homing_speed);
    cell_attr!(r, "klippy.stepper.PrinterRail.second_homing_speed", PrinterRail, second_homing_speed);
    cell_attr!(r, "klippy.stepper.PrinterRail.homing_retract_speed", PrinterRail, homing_retract_speed);
    cell_attr!(r, "klippy.stepper.PrinterRail.homing_retract_dist", PrinterRail, homing_retract_dist);
    cell_attr!(r, "klippy.stepper.PrinterRail.homing_positive_dir", PrinterRail, homing_positive_dir);
    cell_attr!(r, "klippy.stepper.PrinterRail.min_home_dist", PrinterRail, min_home_dist);
    cell_attr!(r, "klippy.stepper.PrinterRail.homing_accel", PrinterRail, homing_accel);
    r.list("klippy.stepper.PrinterRail.steppers", ListField {
        get: |o, _f| rl(o)?.steppers.borrow().clone().into_iter().map(objects::wrap).collect(),
        set: |o, _f, items| {
            let v = items.iter().map(st).collect::<KResult<Vec<_>>>()?;
            *rl(o)?.steppers.borrow_mut() = v;
            Ok(())
        },
    });
    r.list("klippy.stepper.PrinterRail.endstops", ListField {
        get: |o, _f| {
            let es = rl(o)?.endstops.borrow().clone();
            es.iter().map(|(e, n)| Ok(py_tuple(vec![endstop_to_py(e)?, py_str(n)]))).collect()
        },
        set: |o, _f, items| {
            let v = items
                .iter()
                .map(|i| {
                    let (e, n): (PyRef, String) = FromPy::from_py(i)?;
                    Ok((endstop_from_py(&e)?, n))
                })
                .collect::<KResult<Vec<_>>>()?;
            *rl(o)?.endstops.borrow_mut() = v;
            Ok(())
        },
    });
    r.attr("klippy.stepper.PrinterRail._tmc_current_helpers", |o| {
        let hs = rl(o)?.tmc_current_helpers.borrow().clone();
        match hs {
            None => Ok(py_none()),
            Some(hs) => Ok(py_list(hs.into_iter().map(tmc_to_py).collect::<KResult<Vec<_>>>()?)),
        }
    }, None);
    // endstop_map: pin name -> {"endstop", "invert", "pullup"}
    r.dict("klippy.stepper.PrinterRail.endstop_map", DictField {
        get: |o, _f, key| match rl(o)?.endstop_map.borrow().get(&key.as_string()?) {
            Some((e, invert, pullup)) => {
                let d = from_pyval(&PyVal::Dict(crate::pydict! { "invert" => PyVal::Int(*invert), "pullup" => PyVal::Int(*pullup) }));
                d.call_method("__setitem__", &[&py_str("endstop"), &endstop_to_py(e)?])?;
                Ok(d)
            }
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |o, _f, key, value| {
            let e = endstop_from_py(&value.call_method("__getitem__", &[&py_str("endstop")])?)?;
            let invert = value.call_method("__getitem__", &[&py_str("invert")])?.as_i64()?;
            let pullup = value.call_method("__getitem__", &[&py_str("pullup")])?.as_i64()?;
            rl(o)?.endstop_map.borrow_mut().insert(key.as_string()?, (e, invert, pullup));
            Ok(())
        },
        del: |o, _f, key| match rl(o)?.endstop_map.borrow_mut().shift_remove(&key.as_string()?) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(rl(o)?.endstop_map.borrow().keys().map(|k| py_str(k)).collect()),
    });
}

/// A python callable running `f`
fn py_function_of(f: impl Fn(&[PyRef]) -> KResult<PyRef> + 'static) -> KResult<PyRef> {
    crate::plugins::callable::py_function(Rc::new(f))
}
