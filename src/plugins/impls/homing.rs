// klippy/extras/homing.py (PrinterHoming, Homing, HomingMove, StepperPosition,
// multi_complete) and force_move.py (ForceMove, calc_move_time). Plugins home
// their own steppers with a python "toolhead" of their own (AFC's and Happy
// Hare's manual_home(toolhead=self, ...) / HomingMove(printer, endstops,
// self)): kaliru's homing calls it through (PyHomingToolhead).
use super::chelper::{lib_fn, sk_to_py, trapq_to_py};
use super::pins::{endstop_from_py, endstop_to_py};
use super::*;
use crate::configfile::ConfigWrapper;
use crate::extras::force_move::ForceMove;
use crate::extras::homing::{Homing, HomingMove, HomingToolhead, PrinterHoming, StepperPosition};
use crate::extras::manual_stepper::ManualStepper;
use crate::gcode::GCodeCommand;
use crate::mcu::EndstopRef;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::{DictField, ListField};
use crate::reactor::{ReactorCompletion, NOW};
use crate::stepper::{McuStepper, PrinterRail, Rail, RailRef};
use crate::toolhead::ToolHead;
use crate::IndexMap;
use crate::{cell_attr, function, method, ro_attr};

type SpCell = RefCell<StepperPosition>;

fn cfg(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
    super::configfile::config_from_py(o)
}

fn gcmd(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    super::gcode::gcmd_from_py(o)
}

fn hm(o: &PyRef) -> KResult<Rc<Homing>> {
    objects::unwrap_as::<Homing>(o)
}

fn hmv(o: &PyRef) -> KResult<Rc<HomingMove>> {
    objects::unwrap_as::<HomingMove>(o)
}

fn sp(o: &PyRef) -> KResult<Rc<SpCell>> {
    objects::unwrap_as::<SpCell>(o)
}

fn fm(o: &PyRef) -> KResult<Rc<ForceMove>> {
    objects::unwrap_as::<ForceMove>(o)
}

/// a value kaliru's HomingToolhead returns without a KResult: a python
/// error is reported as unhandled
fn reported<T>(r: KResult<T>, default: T) -> T {
    r.unwrap_or_else(|e| {
        report_unhandled(&e);
        default
    })
}

// ------------------------------------------------------- python toolheads

/// A plugin's toolhead-like object (AFC_stepper, Happy Hare's MmuToolHead)
struct PyHomingToolhead {
    obj: PyRef,
}

impl HomingToolhead for PyHomingToolhead {
    fn get_position(&self) -> Vec<f64> {
        reported(self.obj.call_method("get_position", &[]).and_then(|p| FromPy::from_py(&p)), vec![])
    }
    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()> {
        // (as Kalico's homing calls it: the axes only where there are some)
        let pos = newpos.to_vec().to_py()?;
        if homing_axes.is_empty() {
            self.obj.call_method("set_position", &[&pos])?;
        } else {
            self.obj.call_method("set_position", &[&pos, &py_str(homing_axes)])?;
        }
        Ok(())
    }
    fn flush_step_generation(&self) -> KResult<()> {
        self.obj.call_method("flush_step_generation", &[])?;
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        self.obj.call_method("get_last_move_time", &[])?.as_f64()
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        self.obj.call_method("dwell", &[&py_float(delay)])?;
        Ok(())
    }
    fn drip_move(&self, newpos: &[f64], speed: f64, drip_completion: &Rc<ReactorCompletion<bool>>) -> KResult<()> {
        let c = objects::wrap(drip_completion.clone())?;
        self.obj.call_method("drip_move", &[&newpos.to_vec().to_py()?, &py_float(speed), &c])?;
        Ok(())
    }
    fn kin_steppers(&self) -> Vec<Rc<McuStepper>> {
        let r = (|| -> KResult<Vec<Rc<McuStepper>>> {
            let steppers = self.obj.call_method("get_kinematics", &[])?.call_method("get_steppers", &[])?;
            let list: Vec<PyRef> = FromPy::from_py(&steppers)?;
            list.iter().map(objects::unwrap_as::<McuStepper>).collect()
        })();
        reported(r, vec![])
    }
    fn kin_calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64> {
        let r = (|| -> KResult<Vec<f64>> {
            let d = from_pyval(&PyVal::Dict(stepper_positions.iter().map(|(k, v)| (k.clone().into(), PyVal::Float(*v))).collect()));
            let pos = self.obj.call_method("get_kinematics", &[])?.call_method("calc_position", &[&d])?;
            FromPy::from_py(&pos)
        })();
        reported(r, vec![])
    }
}

fn homing_toolhead_to_py(t: &Rc<dyn HomingToolhead>) -> KResult<PyRef> {
    let any: Rc<dyn Any> = t.clone();
    if let Some(p) = any.downcast_ref::<PyHomingToolhead>() {
        return Ok(p.obj.clone());
    }
    let any = match any.downcast::<ToolHead>() {
        Ok(th) => return objects::wrap(th),
        Err(a) => a,
    };
    match any.downcast::<ManualStepper>() {
        Ok(ms) => objects::wrap(ms),
        Err(_) => Err(KError::Py("TypeError".into(), "kaliru: a homing toolhead without a Kalico class".into())),
    }
}

/// toolhead=None: the printer's toolhead (Some(None) here)
fn homing_toolhead_from_py(o: &PyRef) -> KResult<Option<Rc<dyn HomingToolhead>>> {
    if o.is_none() {
        return Ok(None);
    }
    if is_mirror_instance(o)? {
        if let Ok(th) = objects::unwrap_as::<ToolHead>(o) {
            return Ok(Some(th as Rc<dyn HomingToolhead>));
        }
        if let Ok(ms) = objects::unwrap_as::<ManualStepper>(o) {
            return Ok(Some(ms as Rc<dyn HomingToolhead>));
        }
    }
    Ok(Some(Rc::new(PyHomingToolhead { obj: o.clone() }) as Rc<dyn HomingToolhead>))
}

// ----------------------------------------------------------------- helpers

/// endstops: [(endstop, name), ...]
fn endstops_from_py(o: &PyRef) -> KResult<Vec<(EndstopRef, String)>> {
    let items: Vec<(PyRef, String)> = FromPy::from_py(o)?;
    items.iter().map(|(e, n)| Ok((endstop_from_py(e)?, n.clone()))).collect()
}

fn endstops_to_py(endstops: &[(EndstopRef, String)]) -> KResult<PyRef> {
    Ok(py_list(endstops.iter().map(|(e, n)| Ok(py_tuple(vec![endstop_to_py(e)?, py_str(n)]))).collect::<KResult<Vec<_>>>()?))
}

/// A completion as a bool completion kaliru can wait on: kaliru's own, or
/// a python one (done when it is, with the truth of its result)
fn bool_completion(o: &PyRef) -> KResult<Rc<ReactorCompletion<bool>>> {
    if let Ok(c) = objects::unwrap_as::<ReactorCompletion<bool>>(o) {
        return Ok(c);
    }
    let reactor = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.get_reactor();
    let done = reactor.completion::<bool>();
    let (py, d) = (o.clone(), done.clone());
    reactor.register_callback(
        move |_| {
            let v = py.call_method("wait", &[]).and_then(|r| r.is_true()).unwrap_or_else(|e| {
                report_unhandled(&e);
                false
            });
            d.complete(v);
        },
        NOW,
    );
    Ok(done)
}

pub fn rails_from_py(o: &PyRef) -> KResult<Vec<RailRef>> {
    let items: Vec<PyRef> = FromPy::from_py(o)?;
    items
        .iter()
        .map(|r| objects::unwrap_view::<dyn Rail>(r).ok_or_else(|| KError::Py("TypeError".into(), "kaliru: expected a stepper rail".into())))
        .collect()
}

pub fn f64_map_from_py(o: &PyRef) -> KResult<IndexMap<String, f64>> {
    let items: Vec<(String, f64)> = FromPy::from_py(&o.call_method("items", &[])?)?;
    Ok(items.into_iter().collect())
}

pub fn register(r: &mut Registry) {
    objects::register_class::<SpCell>("klippy.extras.homing", "StepperPosition");
    objects::register_view::<PrinterRail>(|a| -> Box<dyn Any> { Box::new(a.downcast::<PrinterRail>().expect("type") as RailRef) });

    // multi_complete(printer, completions)
    r.method("klippy.extras.homing.multi_complete", |a| {
        let printer = objects::unwrap_as::<Printer>(a.arg(0))?;
        let items: Vec<PyRef> = FromPy::from_py(a.arg(1))?;
        let completions = items.iter().map(bool_completion).collect::<KResult<Vec<_>>>()?;
        objects::wrap(crate::extras::homing::multi_complete(&printer, &completions))
    });

    // --------------------------------------------------- StepperPosition
    r.ctor("klippy.extras.homing.StepperPosition.__init__", |a| {
        let stepper = objects::unwrap_as::<McuStepper>(a.arg(0))?;
        let s: Rc<dyn Any> = Rc::new(RefCell::new(StepperPosition::new(&stepper, &a.string(1)?)));
        Ok(s)
    });
    r.method("klippy.extras.homing.StepperPosition.note_home_end", |a| {
        sp(a.slf.as_ref().expect("method"))?.borrow_mut().note_home_end(a.f64(0)?);
        Ok(py_none())
    });
    // verify_no_probe_skew(haltpos): (haltpos unused, as in Kalico)
    r.method("klippy.extras.homing.StepperPosition.verify_no_probe_skew", |a| {
        sp(a.slf.as_ref().expect("method"))?.borrow().verify_no_probe_skew();
        Ok(py_none())
    });
    r.attr("klippy.extras.homing.StepperPosition.stepper", |o| sp(o)?.borrow().stepper.clone().to_py(), None);
    r.attr("klippy.extras.homing.StepperPosition.endstop_name", |o| Ok(py_str(&sp(o)?.borrow().endstop_name)), None);
    r.attr("klippy.extras.homing.StepperPosition.stepper_name", |o| Ok(py_str(&sp(o)?.borrow().stepper_name)), None);
    r.attr("klippy.extras.homing.StepperPosition.start_pos", |o| Ok(py_int(sp(o)?.borrow().start_pos)), None);
    r.attr("klippy.extras.homing.StepperPosition.start_cmd_pos", |o| Ok(py_float(sp(o)?.borrow().start_cmd_pos)), None);
    r.attr("klippy.extras.homing.StepperPosition.halt_pos", |o| sp(o)?.borrow().halt_pos.to_py(), Some(|o, v| {
        sp(o)?.borrow_mut().halt_pos = FromPy::from_py(v)?;
        Ok(())
    }));
    r.attr("klippy.extras.homing.StepperPosition.trig_pos", |o| sp(o)?.borrow().trig_pos.to_py(), Some(|o, v| {
        sp(o)?.borrow_mut().trig_pos = FromPy::from_py(v)?;
        Ok(())
    }));

    // -------------------------------------------------------- HomingMove
    r.ctor("klippy.extras.homing.HomingMove.__init__", |a| {
        let printer = objects::unwrap_as::<Printer>(a.arg(0))?;
        let h: Rc<dyn Any> = HomingMove::new(&printer, endstops_from_py(a.arg(1))?, homing_toolhead_from_py(a.arg(2))?)?;
        Ok(h)
    });
    r.method("klippy.extras.homing.HomingMove.get_mcu_endstops", |a| {
        let es = a.this::<HomingMove>()?.get_mcu_endstops();
        Ok(py_list(es.iter().map(endstop_to_py).collect::<KResult<Vec<_>>>()?))
    });
    r.method("klippy.extras.homing.HomingMove._calc_endstop_rate", |a| {
        let movepos: Vec<f64> = FromPy::from_py(a.arg(1))?;
        Ok(py_float(a.this::<HomingMove>()?.calc_endstop_rate(&endstop_from_py(a.arg(0))?, &movepos, a.f64(2)?)?))
    });
    r.method("klippy.extras.homing.HomingMove.calc_toolhead_pos", |a| {
        let kin_spos = f64_map_from_py(a.arg(0))?;
        let items: Vec<(String, i64)> = FromPy::from_py(&a.arg(1).call_method("items", &[])?)?;
        let offsets: IndexMap<String, i64> = items.into_iter().collect();
        a.this::<HomingMove>()?.calc_toolhead_pos(&kin_spos, &offsets).to_py()
    });
    // homing_move(movepos, speed, probe_pos=False, triggered=True, check_triggered=True)
    r.method("klippy.extras.homing.HomingMove.homing_move", |a| {
        let movepos: Vec<f64> = FromPy::from_py(a.arg(0))?;
        a.this::<HomingMove>()?.homing_move(&movepos, a.f64(1)?, a.bool(2)?, a.bool(3)?, a.bool(4)?).to_py()
    });
    method!(r, "klippy.extras.homing.HomingMove", HomingMove, check_no_movement(h) => h.check_no_movement());
    method!(r, "klippy.extras.homing.HomingMove", HomingMove, moved_less_than_dist(h, min_dist: f64, homing_axes: Vec<usize>) =>
        h.moved_less_than_dist(min_dist, &homing_axes));
    ro_attr!(r, "klippy.extras.homing.HomingMove.printer", HomingMove, |h| h.printer.clone());
    r.attr("klippy.extras.homing.HomingMove.toolhead", |o| homing_toolhead_to_py(&hmv(o)?.toolhead), None);
    r.attr("klippy.extras.homing.HomingMove.endstops", |o| {
        let h = hmv(o)?;
        endstops_to_py(&h.endstops)
    }, None);
    crate::vec_attr!(r, "klippy.extras.homing.HomingMove.distance_elapsed", HomingMove, distance_elapsed);
    // (copies: python reads them after the move)
    r.list("klippy.extras.homing.HomingMove.stepper_positions", ListField {
        get: |o, _f| {
            let sps = hmv(o)?.stepper_positions.borrow().clone();
            sps.into_iter().map(|s| objects::wrap(Rc::new(RefCell::new(s)))).collect()
        },
        set: |o, _f, items| {
            let v = items.iter().map(|i| Ok(sp(i)?.borrow().clone())).collect::<KResult<Vec<_>>>()?;
            *hmv(o)?.stepper_positions.borrow_mut() = v;
            Ok(())
        },
    });

    // ------------------------------------------------------------ Homing
    r.ctor("klippy.extras.homing.Homing.__init__", |a| {
        let h: Rc<dyn Any> = Homing::new(&objects::unwrap_as::<Printer>(a.arg(0))?)?;
        Ok(h)
    });
    method!(r, "klippy.extras.homing.Homing", Homing, set_axes(h, axes: Vec<usize>) => h.set_axes(axes));
    method!(r, "klippy.extras.homing.Homing", Homing, get_axes(h) => h.get_axes());
    method!(r, "klippy.extras.homing.Homing", Homing, get_trigger_position(h, stepper_name: String) => h.get_trigger_position(&stepper_name));
    method!(r, "klippy.extras.homing.Homing", Homing, set_stepper_adjustment(h, stepper_name: String, adjustment: f64) =>
        h.set_stepper_adjustment(&stepper_name, adjustment));
    method!(r, "klippy.extras.homing.Homing", Homing, _fill_coord(h, coord: Vec<Option<f64>>) => h.fill_coord(&coord));
    method!(r, "klippy.extras.homing.Homing", Homing, set_homed_position(h, pos: Vec<Option<f64>>) => h.set_homed_position(&pos));
    method!(r, "klippy.extras.homing.Homing", Homing, _set_homing_accel(h, accel: Option<f64>, pre_homing: bool) =>
        h.set_homing_accel(accel, pre_homing));
    method!(r, "klippy.extras.homing.Homing", Homing, _set_homing_current(h, homing_axes: String, pre_homing: bool) =>
        h.set_homing_current(&homing_axes, pre_homing));
    r.method("klippy.extras.homing.Homing._reset_endstop_states", |a| {
        let endstops = endstops_from_py(a.arg(0))?;
        a.this::<Homing>()?.reset_endstop_states(&endstops)?;
        Ok(py_none())
    });
    r.method("klippy.extras.homing.Homing.home_rails", |a| {
        let rails = rails_from_py(a.arg(0))?;
        let forcepos: Vec<Option<f64>> = FromPy::from_py(a.arg(1))?;
        let movepos: Vec<Option<f64>> = FromPy::from_py(a.arg(2))?;
        a.this::<Homing>()?.home_rails(&rails, &forcepos, &movepos)?;
        Ok(py_none())
    });
    ro_attr!(r, "klippy.extras.homing.Homing.printer", Homing, |h| h.printer.clone());
    ro_attr!(r, "klippy.extras.homing.Homing.toolhead", Homing, |h| h.toolhead.clone());
    cell_attr!(r, "klippy.extras.homing.Homing.saved_accel", Homing, saved_accel);
    crate::vec_attr!(r, "klippy.extras.homing.Homing.changed_axes", Homing, changed_axes);
    r.dict("klippy.extras.homing.Homing.trigger_mcu_pos", DictField {
        get: |o, _f, key| match hm(o)?.trigger_mcu_pos.borrow().get(&key.as_string()?) {
            Some(v) => Ok(py_int(*v)),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |o, _f, key, value| {
            hm(o)?.trigger_mcu_pos.borrow_mut().insert(key.as_string()?, value.as_i64()?);
            Ok(())
        },
        del: |o, _f, key| match hm(o)?.trigger_mcu_pos.borrow_mut().shift_remove(&key.as_string()?) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(hm(o)?.trigger_mcu_pos.borrow().keys().map(|k| py_str(k)).collect()),
    });
    r.dict("klippy.extras.homing.Homing.adjust_pos", DictField {
        get: |o, _f, key| match hm(o)?.adjust_pos.borrow().get(&key.as_string()?) {
            Some(v) => Ok(py_float(*v)),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |o, _f, key, value| {
            hm(o)?.adjust_pos.borrow_mut().insert(key.as_string()?, value.as_f64()?);
            Ok(())
        },
        del: |o, _f, key| match hm(o)?.adjust_pos.borrow_mut().shift_remove(&key.as_string()?) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(hm(o)?.adjust_pos.borrow().keys().map(|k| py_str(k)).collect()),
    });

    // ----------------------------------------------------- PrinterHoming
    r.ctor("klippy.extras.homing.PrinterHoming.__init__", |a| {
        let p: Rc<dyn Any> = PrinterHoming::new(&*cfg(a.arg(0))?)?;
        Ok(p)
    });
    // manual_home(toolhead, endstops, pos, speed, triggered, check_triggered)
    r.method("klippy.extras.homing.PrinterHoming.manual_home", |a| {
        let ph = a.this::<PrinterHoming>()?;
        let toolhead = match homing_toolhead_from_py(a.arg(0))? {
            Some(t) => t,
            None => ph.printer.lookup::<ToolHead>("toolhead")? as Rc<dyn HomingToolhead>,
        };
        let pos: Vec<f64> = FromPy::from_py(a.arg(2))?;
        ph.manual_home(&toolhead, endstops_from_py(a.arg(1))?, &pos, a.f64(3)?, a.bool(4)?, a.bool(5)?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.homing.PrinterHoming.probing_move", |a| {
        let pos: Vec<f64> = FromPy::from_py(a.arg(1))?;
        a.this::<PrinterHoming>()?.probing_move(endstop_from_py(a.arg(0))?, &pos, a.f64(2)?).to_py()
    });
    method!(r, "klippy.extras.homing.PrinterHoming", PrinterHoming, cmd_G28(p, g: PyRef) => p.cmd_g28(&*gcmd(&g)?));
    ro_attr!(r, "klippy.extras.homing.PrinterHoming.printer", PrinterHoming, |p| p.printer.clone());
    r.method("klippy.extras.homing.load_config", |a| crate::extras::homing::load_config(&*cfg(a.arg(0))?)?.to_py());

    // --------------------------------------------------------- ForceMove
    function!(r, "klippy.extras.force_move", calc_move_time(dist: f64, speed: f64, accel: f64) =>
        crate::extras::force_move::calc_move_time(dist, speed, accel));
    r.ctor("klippy.extras.force_move.ForceMove.__init__", |a| {
        let f: Rc<dyn Any> = ForceMove::new(&*cfg(a.arg(0))?)?;
        Ok(f)
    });
    r.method("klippy.extras.force_move.ForceMove.register_stepper", |a| {
        let s = objects::unwrap_as::<McuStepper>(a.arg(1))?;
        a.this::<ForceMove>()?.register_stepper(&*cfg(a.arg(0))?, &s);
        Ok(py_none())
    });
    method!(r, "klippy.extras.force_move.ForceMove", ForceMove, lookup_stepper(f, name: String) => f.lookup_stepper(&name));
    method!(r, "klippy.extras.force_move.ForceMove", ForceMove, _force_enable(f, stepper: Rc<McuStepper>) => f.force_enable(&stepper));
    method!(r, "klippy.extras.force_move.ForceMove", ForceMove, _restore_enable(f, stepper: Rc<McuStepper>, did_enable: bool) =>
        f.restore_enable(&stepper, did_enable));
    // manual_move(stepper, dist, speed, accel=0.)
    method!(r, "klippy.extras.force_move.ForceMove", ForceMove, manual_move(f, stepper: Rc<McuStepper>, dist: f64, speed: f64, accel: f64) =>
        f.manual_move(&stepper, dist, speed, accel));
    method!(r, "klippy.extras.force_move.ForceMove", ForceMove, _lookup_stepper(f, g: PyRef) => f.lookup_gcmd_stepper(&*gcmd(&g)?));
    method!(r, "klippy.extras.force_move.ForceMove", ForceMove, cmd_STEPPER_BUZZ(f, g: PyRef) => f.cmd_stepper_buzz(&*gcmd(&g)?));
    method!(r, "klippy.extras.force_move.ForceMove", ForceMove, cmd_FORCE_MOVE(f, g: PyRef) => f.cmd_force_move(&*gcmd(&g)?));
    method!(r, "klippy.extras.force_move.ForceMove", ForceMove, cmd_SET_KINEMATIC_POSITION(f, g: PyRef) =>
        f.cmd_set_kinematic_position(&*gcmd(&g)?));
    ro_attr!(r, "klippy.extras.force_move.ForceMove.printer", ForceMove, |f| f.printer.clone());
    ro_attr!(r, "klippy.extras.force_move.ForceMove.motion_queuing", ForceMove, |f| f.motion_queuing.clone());
    r.attr("klippy.extras.force_move.ForceMove.trapq", |o| trapq_to_py(Some(fm(o)?.trapq.clone())), None);
    r.attr("klippy.extras.force_move.ForceMove.stepper_kinematics", |o| sk_to_py(Some(fm(o)?.stepper_kinematics.clone())), None);
    // self.trapq_append = ffi_lib.trapq_append
    r.attr("klippy.extras.force_move.ForceMove.trapq_append", |_o| lib_fn("trapq_append"), None);
    r.dict("klippy.extras.force_move.ForceMove.steppers", DictField {
        get: |o, _f, key| match fm(o)?.steppers.borrow().get(&key.as_string()?) {
            Some(s) => objects::wrap(s.clone()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |o, _f, key, value| {
            fm(o)?.steppers.borrow_mut().insert(key.as_string()?, objects::unwrap_as::<McuStepper>(value)?);
            Ok(())
        },
        del: |o, _f, key| match fm(o)?.steppers.borrow_mut().shift_remove(&key.as_string()?) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(fm(o)?.steppers.borrow().keys().map(|k| py_str(k)).collect()),
    });
    r.method("klippy.extras.force_move.load_config", |a| crate::extras::force_move::load_config(&*cfg(a.arg(0))?)?.to_py());
}
