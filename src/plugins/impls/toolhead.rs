// klippy/toolhead.py: ToolHead, Move, LookAheadQueue, ToolHeadCommandHelper.
// A Move python sees is a copy (kaliru's moves are values in its lookahead
// queue): one passed to a plugin's extra axis is copied back after the call.
use super::chelper::{lib_fn, trapq_to_py};
use super::*;
use crate::chelper::trapq::TrapQRef;
use crate::extras::homing::HomingToolhead;
use crate::extras::manual_stepper::ManualStepper;
use crate::gcode::GCodeCommand;
use crate::kinematics::extruder::{DummyExtruder, PrinterExtruder};
use crate::kinematics::Kinematics;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::gil::Gil;
use crate::plugins::native::{DictField, ListField};
use crate::stepper::RailRef;
use crate::toolhead::{ExtraAxis, ExtraAxisRef, LookAheadQueue, Move, ToolHead, ToolHeadCommandHelper};
use crate::{cell_attr, function, method, ro_attr, vec_attr};
use std::collections::HashMap;

type MoveCell = RefCell<Move>;

fn th(o: &PyRef) -> KResult<Rc<ToolHead>> {
    objects::unwrap_as::<ToolHead>(o)
}

fn mv(o: &PyRef) -> KResult<Rc<MoveCell>> {
    objects::unwrap_as::<MoveCell>(o)
}

fn gcmd(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    super::gcode::gcmd_from_py(o)
}

/// A move as python sees it (a copy)
pub fn move_to_py(m: &Move) -> KResult<PyRef> {
    objects::wrap(Rc::new(RefCell::new(m.clone())))
}

/// the printer's toolhead (Move.toolhead: kaliru's moves do not keep it)
fn toolhead() -> KResult<Rc<ToolHead>> {
    printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.lookup::<ToolHead>("toolhead")
}

// ------------------------------------------------------------ extra axes

/// A plugin's extra axis (toolhead.add_extra_axis(obj, pos))
struct PyExtraAxis {
    obj: PyRef,
}

/// a value kaliru's ExtraAxis returns without a KResult: a python error
/// is reported as unhandled
fn reported<T>(r: KResult<T>, default: T) -> T {
    r.unwrap_or_else(|e| {
        report_unhandled(&e);
        default
    })
}

impl ExtraAxis for PyExtraAxis {
    fn check_move(&self, m: &mut Move, ea_index: usize) -> KResult<()> {
        let cell = Rc::new(RefCell::new(m.clone()));
        let r = self.obj.call_method("check_move", &[&objects::wrap(cell.clone())?, &py_int(ea_index as i64)]);
        *m = cell.borrow().clone();
        r.map(|_| ())
    }
    fn find_past_position(&self, print_time: f64) -> f64 {
        reported(self.obj.call_method("find_past_position", &[&py_float(print_time)]).and_then(|v| v.as_f64()), 0.0)
    }
    fn calc_junction(&self, prev_move: &Move, m: &Move, ea_index: usize) -> KResult<f64> {
        self.obj.call_method("calc_junction", &[&move_to_py(prev_move)?, &move_to_py(m)?, &py_int(ea_index as i64)])?.as_f64()
    }
    fn process_move(&self, print_time: f64, m: &Move, ea_index: usize) -> KResult<()> {
        self.obj.call_method("process_move", &[&py_float(print_time), &move_to_py(m)?, &py_int(ea_index as i64)])?;
        Ok(())
    }
    fn get_name(&self) -> String {
        reported(self.obj.call_method("get_name", &[]).and_then(|v| v.as_string()), String::new())
    }
    fn get_trapq(&self) -> Option<TrapQRef> {
        reported(self.obj.call_method("get_trapq", &[]).and_then(|t| super::chelper::trapq_from_py(&t)), None)
    }
    fn get_axis_gcode_id(&self) -> String {
        reported(self.obj.call_method("get_axis_gcode_id", &[]).and_then(|v| v.as_string()), String::new())
    }
    fn get_heater(&self) -> KResult<Rc<crate::extras::heaters::Heater>> {
        objects::unwrap_as(&self.obj.call_method("get_heater", &[])?)
    }
}

/// An extra axis as python sees it: an extruder is the printer object of
/// its name (the object lookup_object() returns)
fn extra_axis_to_py(ea: &ExtraAxisRef) -> KResult<PyRef> {
    let any: Rc<dyn Any> = ea.clone();
    if let Some(p) = any.downcast_ref::<PyExtraAxis>() {
        return Ok(p.obj.clone());
    }
    let name = ea.type_name();
    objects::wrap_view(any, ea.clone(), name)
}

/// An extra axis python hands back: kaliru's own, else called through
fn extra_axis_from_py(o: &PyRef) -> KResult<ExtraAxisRef> {
    if is_mirror_instance(o)? {
        if let Some(ea) = objects::unwrap_view::<dyn ExtraAxis>(o) {
            return Ok(ea);
        }
    }
    Ok(Rc::new(PyExtraAxis { obj: o.clone() }))
}

pub fn rail_to_py(rail: RailRef) -> KResult<PyRef> {
    let any: Rc<dyn Any> = rail.clone();
    let name = rail.type_name();
    objects::wrap_view(any, rail, name)
}

// ------------------------------------------------------------ kinematics

/// A kinematics object as python sees it: the kinematics itself, its
/// trait a view (impls/kinematics.rs)
pub fn kin_to_py(kin: Rc<dyn Kinematics>) -> KResult<PyRef> {
    let any: Rc<dyn Any> = kin.clone();
    let name = kin.type_name();
    objects::wrap_view(any, kin, name)
}

// -------------------------------------------------------------- lookahead

/// toolhead.lookahead: the toolhead's LookAheadQueue (one per toolhead,
/// kept for `is`)
pub struct LookaheadRef(pub Rc<ToolHead>);

static LOOKAHEADS: Gil<RefCell<HashMap<usize, Rc<LookaheadRef>>>> = Gil::new(|| RefCell::new(HashMap::new()));

fn lookahead_to_py(t: &Rc<ToolHead>) -> KResult<PyRef> {
    let key = Rc::as_ptr(t) as usize;
    let l = LOOKAHEADS.with(|m| m.borrow_mut().entry(key).or_insert_with(|| Rc::new(LookaheadRef(t.clone()))).clone());
    objects::wrap(l)
}

/// The queue behind a python LookAheadQueue: a toolhead's, or one a plugin
/// built (LookAheadQueue())
fn with_queue<R>(o: &PyRef, f: impl FnOnce(&mut LookAheadQueue) -> R) -> KResult<R> {
    if let Ok(l) = objects::unwrap_as::<LookaheadRef>(o) {
        return Ok(f(&mut l.0.lookahead.borrow_mut()));
    }
    let q = objects::unwrap_as::<RefCell<LookAheadQueue>>(o)?;
    let r = f(&mut q.borrow_mut());
    Ok(r)
}

fn extra_axes_of(o: &PyRef) -> Vec<ExtraAxisRef> {
    match objects::unwrap_as::<LookaheadRef>(o) {
        Ok(l) => l.0.extra_axes.borrow().clone(),
        Err(_) => toolhead().map(|t| t.extra_axes.borrow().clone()).unwrap_or_default(),
    }
}

/// Move attributes: f64 fields read and written on the move
macro_rules! move_f64 {
    ($r:expr, $($field:ident),*) => {
        $(
            $r.attr(
                concat!("klippy.toolhead.Move.", stringify!($field)),
                |o| Ok(py_float(mv(o)?.borrow().$field)),
                Some(|o, v| {
                    mv(o)?.borrow_mut().$field = v.as_f64()?;
                    Ok(())
                }),
            );
        )*
    };
}

/// Move attributes: list fields (a copy; assignment replaces)
macro_rules! move_vec {
    ($r:expr, $($field:ident),*) => {
        $(
            $r.attr(
                concat!("klippy.toolhead.Move.", stringify!($field)),
                |o| mv(o)?.borrow().$field.clone().to_py(),
                Some(|o, v| {
                    mv(o)?.borrow_mut().$field = FromPy::from_py(v)?;
                    Ok(())
                }),
            );
        )*
    };
}

pub fn register(r: &mut Registry) {
    objects::register_class::<MoveCell>("klippy.toolhead", "Move");
    objects::register_class::<LookaheadRef>("klippy.toolhead", "LookAheadQueue");
    objects::register_class::<RefCell<LookAheadQueue>>("klippy.toolhead", "LookAheadQueue");
    objects::register_pobj::<PrinterExtruder>();
    objects::register_pobj::<ManualStepper>();
    objects::register_view::<PrinterExtruder>(|a| -> Box<dyn Any> { Box::new(a.downcast::<PrinterExtruder>().expect("type") as ExtraAxisRef) });
    objects::register_view::<DummyExtruder>(|a| -> Box<dyn Any> { Box::new(a.downcast::<DummyExtruder>().expect("type") as ExtraAxisRef) });
    objects::register_view::<ManualStepper>(|a| -> Box<dyn Any> { Box::new(a.downcast::<ManualStepper>().expect("type") as ExtraAxisRef) });
    // ManualStepper: its extra axis and homing toolhead interfaces
    r.method("klippy.extras.manual_stepper.ManualStepper.process_move", |a| {
        let m = mv(a.arg(1))?;
        ExtraAxis::process_move(&*a.this::<ManualStepper>()?, a.f64(0)?, &m.borrow(), a.arg(2).as_i64()? as usize)?;
        Ok(py_none())
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.check_move", |a| {
        let m = mv(a.arg(0))?;
        ExtraAxis::check_move(&*a.this::<ManualStepper>()?, &mut m.borrow_mut(), a.arg(1).as_i64()? as usize)?;
        Ok(py_none())
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.calc_junction", |a| {
        let (prev, m) = (mv(a.arg(0))?, mv(a.arg(1))?);
        let v = ExtraAxis::calc_junction(&*a.this::<ManualStepper>()?, &prev.borrow(), &m.borrow(), a.arg(2).as_i64()? as usize)?;
        Ok(py_float(v))
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.get_axis_gcode_id", |a| {
        Ok(py_str(&ExtraAxis::get_axis_gcode_id(&*a.this::<ManualStepper>()?)))
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.get_trapq", |a| trapq_to_py(ExtraAxis::get_trapq(&*a.this::<ManualStepper>()?)));
    r.method("klippy.extras.manual_stepper.ManualStepper.flush_step_generation", |a| {
        HomingToolhead::flush_step_generation(&*a.this::<ManualStepper>()?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.get_position", |a| HomingToolhead::get_position(&*a.this::<ManualStepper>()?).to_py());
    r.method("klippy.extras.manual_stepper.ManualStepper.set_position", |a| {
        let newpos: Vec<f64> = FromPy::from_py(a.arg(0))?;
        HomingToolhead::set_position(&*a.this::<ManualStepper>()?, &newpos, "")?;
        Ok(py_none())
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.get_last_move_time", |a| {
        Ok(py_float(HomingToolhead::get_last_move_time(&*a.this::<ManualStepper>()?)?))
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.dwell", |a| {
        HomingToolhead::dwell(&*a.this::<ManualStepper>()?, a.f64(0)?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.drip_move", |a| {
        let newpos: Vec<f64> = FromPy::from_py(a.arg(0))?;
        let c = objects::unwrap_as::<crate::reactor::ReactorCompletion<bool>>(a.arg(2))
            .map_err(|_| KError::Py("TypeError".into(), "kaliru: drip_move with a completion kaliru did not make".into()))?;
        HomingToolhead::drip_move(&*a.this::<ManualStepper>()?, &newpos, a.f64(1)?, &c)?;
        Ok(py_none())
    });
    // get_kinematics(): the manual stepper itself
    r.method("klippy.extras.manual_stepper.ManualStepper.get_kinematics", |a| Ok(a.slf.clone().expect("method")));
    r.method("klippy.extras.manual_stepper.ManualStepper.get_steppers", |a| {
        let steppers: KResult<Vec<PyRef>> = a.this::<ManualStepper>()?.steppers.iter().cloned().map(objects::wrap).collect();
        Ok(py_list(steppers?))
    });
    r.method("klippy.extras.manual_stepper.ManualStepper.calc_position", |a| {
        HomingToolhead::kin_calc_position(&*a.this::<ManualStepper>()?, &super::homing::f64_map_from_py(a.arg(0))?).to_py()
    });
    // self.rail: the PrinterRail, or the stepper of an endstop-less one
    r.attr("klippy.extras.manual_stepper.ManualStepper.rail", |o| match &objects::unwrap_as::<ManualStepper>(o)?.rail {
        crate::extras::manual_stepper::RailOrStepper::Rail(r) => rail_to_py(r.clone()),
        crate::extras::manual_stepper::RailOrStepper::Stepper(s) => objects::wrap(s.clone()),
    }, None);
    r.attr("klippy.extras.manual_stepper.ManualStepper.trapq_append", |_o| lib_fn("trapq_append"), None);

    // --------------------------------------------------------------- Move
    r.ctor("klippy.toolhead.Move.__init__", |a| {
        let t = th(a.arg(0))?;
        let (start_pos, end_pos): (Vec<f64>, Vec<f64>) = (FromPy::from_py(a.arg(1))?, FromPy::from_py(a.arg(2))?);
        let m: Rc<dyn Any> = Rc::new(RefCell::new(Move::new(&t, &start_pos, &end_pos, a.f64(3)?)));
        Ok(m)
    });
    r.method("klippy.toolhead.Move.limit_speed", |a| {
        mv(a.slf.as_ref().expect("method"))?.borrow_mut().limit_speed(a.f64(0)?, a.f64(1)?);
        Ok(py_none())
    });
    r.method("klippy.toolhead.Move.limit_next_junction_speed", |a| {
        mv(a.slf.as_ref().expect("method"))?.borrow_mut().limit_next_junction_speed(a.f64(0)?);
        Ok(py_none())
    });
    // move_error(msg="Move out of range"): the command_error, not raised
    r.method("klippy.toolhead.Move.move_error", |a| {
        let m = mv(a.slf.as_ref().expect("method"))?;
        let ep = m.borrow().end_pos.clone();
        let msg = format!("{}: {:.3} {:.3} {:.3} [{:.3}]", a.string(0)?, ep[0], ep[1], ep[2], ep[3]);
        lookup("klippy.gcode", "CommandError")?.call(&[&py_str(&msg)])
    });
    r.method("klippy.toolhead.Move.calc_junction", |a| {
        let m = mv(a.slf.as_ref().expect("method"))?;
        let prev = mv(a.arg(0))?.borrow().clone();
        let extra_axes = toolhead()?.extra_axes.borrow().clone();
        m.borrow_mut().calc_junction(&prev, &extra_axes)?;
        Ok(py_none())
    });
    r.method("klippy.toolhead.Move.set_junction", |a| {
        mv(a.slf.as_ref().expect("method"))?.borrow_mut().set_junction(a.f64(0)?, a.f64(1)?, a.f64(2)?);
        Ok(py_none())
    });
    move_f64!(r, accel, junction_deviation, move_d, min_move_t, max_start_v2, max_cruise_v2, delta_v2, max_smoothed_v2,
        smooth_delta_v2, next_junction_v2, start_v, cruise_v, end_v, accel_t, cruise_t, decel_t);
    move_vec!(r, start_pos, end_pos, axes_d, axes_r);
    r.attr("klippy.toolhead.Move.is_kinematic_move", |o| Ok(py_bool(mv(o)?.borrow().is_kinematic_move)), Some(|o, v| {
        mv(o)?.borrow_mut().is_kinematic_move = v.is_true()?;
        Ok(())
    }));
    r.attr("klippy.toolhead.Move.toolhead", |_o| toolhead()?.to_py(), None);
    // timing_callbacks: a list python may append to (cb(next_move_time))
    r.list("klippy.toolhead.Move.timing_callbacks", ListField {
        get: |o, _f| {
            let cbs = mv(o)?.borrow().timing_callbacks.clone();
            cbs.into_iter()
                .map(|cb| {
                    crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| {
                        cb(a[0].as_f64()?)?;
                        Ok(py_none())
                    }))
                })
                .collect()
        },
        set: |o, _f, items| {
            let cbs: Vec<crate::toolhead::TimingCallback> = items
                .into_iter()
                .map(|cb| -> crate::toolhead::TimingCallback {
                    Rc::new(move |t: f64| {
                        cb.call(&[&py_float(t)])?;
                        Ok(())
                    })
                })
                .collect();
            mv(o)?.borrow_mut().timing_callbacks = cbs;
            Ok(())
        },
    });

    // ----------------------------------------------------- LookAheadQueue
    r.ctor("klippy.toolhead.LookAheadQueue.__init__", |_a| {
        let q: Rc<dyn Any> = Rc::new(RefCell::new(LookAheadQueue::new()));
        Ok(q)
    });
    r.method("klippy.toolhead.LookAheadQueue.reset", |a| {
        with_queue(a.slf.as_ref().expect("method"), |q| q.reset())?;
        Ok(py_none())
    });
    r.method("klippy.toolhead.LookAheadQueue.set_flush_time", |a| {
        let t = a.f64(0)?;
        with_queue(a.slf.as_ref().expect("method"), |q| q.set_flush_time(t))?;
        Ok(py_none())
    });
    r.method("klippy.toolhead.LookAheadQueue.get_last", |a| {
        match with_queue(a.slf.as_ref().expect("method"), |q| q.get_last().map(|m| m.clone()))? {
            Some(m) => move_to_py(&m),
            None => Ok(py_none()),
        }
    });
    r.method("klippy.toolhead.LookAheadQueue.flush", |a| {
        let lazy = a.bool(0)?;
        let moves = with_queue(a.slf.as_ref().expect("method"), |q| q.flush(lazy))?;
        Ok(py_list(moves.iter().map(move_to_py).collect::<KResult<Vec<_>>>()?))
    });
    r.method("klippy.toolhead.LookAheadQueue.add_move", |a| {
        let slf = a.slf.as_ref().expect("method");
        let m = mv(a.arg(0))?.borrow().clone();
        let extra_axes = extra_axes_of(slf);
        let want_flush = with_queue(slf, |q| q.add_move(m, &extra_axes))??;
        Ok(py_bool(want_flush))
    });
    r.attr("klippy.toolhead.LookAheadQueue.junction_flush", |o| Ok(py_float(with_queue(o, |q| q.junction_flush)?)), Some(|o, v| {
        let t = v.as_f64()?;
        with_queue(o, |q| q.junction_flush = t)
    }));
    r.attr("klippy.toolhead.LookAheadQueue.queue", |o| {
        let moves = with_queue(o, |q| q.queue.clone())?;
        Ok(py_list(moves.iter().map(move_to_py).collect::<KResult<Vec<_>>>()?))
    }, None);

    // ----------------------------------------------------------- ToolHead
    r.ctor("klippy.toolhead.ToolHead.__init__", |a| {
        let t: Rc<dyn Any> = ToolHead::new(&*super::configfile::config_from_py(a.arg(0))?)?;
        Ok(t)
    });
    r.method("klippy.toolhead.ToolHead.get_kinematics", |a| kin_to_py(a.this::<ToolHead>()?.get_kinematics()));
    r.method("klippy.toolhead.ToolHead.get_active_rails_for_axis", |a| {
        let axis = a.string(0)?.chars().next().unwrap_or('\0');
        let rails = a.this::<ToolHead>()?.get_active_rails_for_axis(axis);
        Ok(py_list(rails.into_iter().map(rail_to_py).collect::<KResult<Vec<_>>>()?))
    });
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _advance_move_time(t, next_print_time: f64) => t.advance_move_time(next_print_time));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _calc_print_time(t) => t.calc_print_time());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _process_lookahead(t, lazy: bool) => t.process_lookahead(lazy));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _flush_lookahead(t, is_runout: bool) => t.flush_lookahead(is_runout));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _check_pause(t) => t.check_pause());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _priming_handler(t, eventtime: f64) => t.priming_handler(eventtime));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _handle_step_flush(t, flush_time: f64, step_gen_time: f64) =>
        t.handle_step_flush(flush_time, step_gen_time));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, get_position(t) => t.get_position());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, set_position(t, newpos: Vec<f64>, homing_axes: String) => t.set_position(&newpos, &homing_axes));
    // move(newpos, speed) (`move` is a Rust keyword: registered by name)
    r.method("klippy.toolhead.ToolHead.move", |a| {
        let newpos: Vec<f64> = FromPy::from_py(a.arg(0))?;
        a.this::<ToolHead>()?.move_to(&newpos, a.f64(1)?)?;
        Ok(py_none())
    });
    method!(r, "klippy.toolhead.ToolHead", ToolHead, manual_move(t, coord: Vec<Option<f64>>, speed: f64) => t.manual_move(&coord, speed));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, dwell(t, delay: f64) => t.dwell(delay));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, wait_moves(t) => t.wait_moves());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, get_last_move_time(t) => t.get_last_move_time());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, flush_step_generation(t) => t.flush_step_generation());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, limit_next_junction_speed(t, speed: f64) => t.limit_next_junction_speed(speed));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, check_busy(t, eventtime: f64) => t.check_busy(eventtime));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, get_max_velocity(t) => t.get_max_velocity());
    method!(r, "klippy.toolhead.ToolHead", ToolHead, set_accel(t, accel: f64) => t.set_accel(accel));
    method!(r, "klippy.toolhead.ToolHead", ToolHead, set_max_velocities(
        t, max_velocity: Option<f64>, max_accel: Option<f64>, square_corner_velocity: Option<f64>, min_cruise_ratio: Option<f64>) => {
        let (a, b, c, d) = t.set_max_velocities(max_velocity, max_accel, square_corner_velocity, min_cruise_ratio);
        crate::plugins::py::py_tuple(vec![py_float(a), py_float(b), py_float(c), py_float(d)])
    });
    r.method("klippy.toolhead.ToolHead.get_extruder", |a| extra_axis_to_py(&a.this::<ToolHead>()?.get_extruder()));
    r.method("klippy.toolhead.ToolHead.set_extruder", |a| {
        let ea = extra_axis_from_py(a.arg(0))?;
        a.this::<ToolHead>()?.set_extruder(ea, a.f64(1)?);
        Ok(py_none())
    });
    r.method("klippy.toolhead.ToolHead.add_extra_axis", |a| {
        a.this::<ToolHead>()?.add_extra_axis(extra_axis_from_py(a.arg(0))?, a.f64(1)?)?;
        Ok(py_none())
    });
    r.method("klippy.toolhead.ToolHead.remove_extra_axis", |a| {
        a.this::<ToolHead>()?.remove_extra_axis(&extra_axis_from_py(a.arg(0))?)?;
        Ok(py_none())
    });
    r.method("klippy.toolhead.ToolHead.get_extra_axes", |a| {
        let axes = a.this::<ToolHead>()?.get_extra_axes();
        let items = axes
            .iter()
            .map(|ea| match ea {
                Some(ea) => extra_axis_to_py(ea),
                None => Ok(py_none()),
            })
            .collect::<KResult<Vec<_>>>()?;
        Ok(py_list(items))
    });
    r.method("klippy.toolhead.ToolHead._drip_load_trapq", |a| {
        let m = mv(a.arg(0))?.borrow().clone();
        a.this::<ToolHead>()?.drip_load_trapq(m).to_py()
    });
    // drip_move(newpos, speed, drip_completion)
    r.method("klippy.toolhead.ToolHead.drip_move", |a| {
        let t = a.this::<ToolHead>()?;
        let newpos: Vec<f64> = FromPy::from_py(a.arg(0))?;
        if let Ok(c) = objects::unwrap_as::<crate::reactor::ReactorCompletion<bool>>(a.arg(2)) {
            t.drip_move(&newpos, a.f64(1)?, &*c)?;
        } else if let Ok(c) = objects::unwrap_as::<crate::reactor::ReactorCompletion<PyRef>>(a.arg(2)) {
            t.drip_move(&newpos, a.f64(1)?, &*c)?;
        } else {
            t.drip_move(&newpos, a.f64(1)?, &PyDrip(a.arg(2).clone()))?;
        }
        Ok(py_none())
    });
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _handle_shutdown(t) => t.handle_shutdown());
    r.method("klippy.toolhead.ToolHead.get_trapq", |a| trapq_to_py(Some(a.this::<ToolHead>()?.get_trapq())));
    // register_lookahead_callback(callback): callback(print_time)
    r.method("klippy.toolhead.ToolHead.register_lookahead_callback", |a| {
        let cb = a.arg(0).clone();
        a.this::<ToolHead>()?.register_lookahead_callback(Rc::new(move |pt| {
            cb.call(&[&py_float(pt)])?;
            Ok(())
        }))?;
        Ok(py_none())
    });
    method!(r, "klippy.toolhead.ToolHead", ToolHead, _calc_junction_deviation(t) => t.calc_junction_deviation());
    // attributes
    ro_attr!(r, "klippy.toolhead.ToolHead.mcu", ToolHead, |t| t.mcu.clone());
    ro_attr!(r, "klippy.toolhead.ToolHead.printer", ToolHead, |t| t.printer.clone());
    ro_attr!(r, "klippy.toolhead.ToolHead.reactor", ToolHead, |t| t.reactor.clone());
    ro_attr!(r, "klippy.toolhead.ToolHead.motion_queuing", ToolHead, |t| t.motion_queuing.clone());
    ro_attr!(r, "klippy.toolhead.ToolHead.priming_timer", ToolHead, |t| t.priming_timer.borrow().clone());
    r.attr("klippy.toolhead.ToolHead.kin", |o| kin_to_py(th(o)?.get_kinematics()), None);
    r.attr("klippy.toolhead.ToolHead.lookahead", |o| lookahead_to_py(&th(o)?), None);
    r.attr("klippy.toolhead.ToolHead.trapq", |o| trapq_to_py(Some(th(o)?.get_trapq())), None);
    // self.trapq_append = ffi_lib.trapq_append
    r.attr("klippy.toolhead.ToolHead.trapq_append", |_o| lib_fn("trapq_append"), None);
    // self.Coord = gcode.Coord
    r.attr("klippy.toolhead.ToolHead.Coord", |_o| lookup("klippy.gcode", "Coord"), None);
    cell_attr!(r, "klippy.toolhead.ToolHead.print_time", ToolHead, print_time);
    cell_attr!(r, "klippy.toolhead.ToolHead.can_pause", ToolHead, can_pause);
    cell_attr!(r, "klippy.toolhead.ToolHead.check_stall_time", ToolHead, check_stall_time);
    cell_attr!(r, "klippy.toolhead.ToolHead.need_check_pause", ToolHead, need_check_pause);
    cell_attr!(r, "klippy.toolhead.ToolHead.max_velocity", ToolHead, max_velocity);
    cell_attr!(r, "klippy.toolhead.ToolHead.max_accel", ToolHead, max_accel);
    cell_attr!(r, "klippy.toolhead.ToolHead.min_cruise_ratio", ToolHead, min_cruise_ratio);
    cell_attr!(r, "klippy.toolhead.ToolHead.square_corner_velocity", ToolHead, square_corner_velocity);
    cell_attr!(r, "klippy.toolhead.ToolHead.junction_deviation", ToolHead, junction_deviation);
    cell_attr!(r, "klippy.toolhead.ToolHead.max_accel_to_decel", ToolHead, max_accel_to_decel);
    cell_attr!(r, "klippy.toolhead.ToolHead.print_stall", ToolHead, print_stall);
    ro_attr!(r, "klippy.toolhead.ToolHead.special_queuing_state", ToolHead, |t| t.special_queuing_state.borrow().to_string());
    vec_attr!(r, "klippy.toolhead.ToolHead.commanded_pos", ToolHead, commanded_pos);
    r.list("klippy.toolhead.ToolHead.extra_axes", ListField {
        get: |o, _f| th(o)?.extra_axes.borrow().clone().iter().map(extra_axis_to_py).collect(),
        set: |o, _f, items| {
            let v = items.iter().map(extra_axis_from_py).collect::<KResult<Vec<_>>>()?;
            *th(o)?.extra_axes.borrow_mut() = v;
            Ok(())
        },
    });
    r.dict("klippy.toolhead.ToolHead.orig_cfg", DictField {
        get: |o, _f, key| match th(o)?.orig_cfg.borrow().get(&key.as_string()?) {
            Some(v) => Ok(py_float(*v)),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |o, _f, key, value| {
            th(o)?.orig_cfg.borrow_mut().insert(key.as_string()?, value.as_f64()?);
            Ok(())
        },
        del: |o, _f, key| match th(o)?.orig_cfg.borrow_mut().shift_remove(&key.as_string()?) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(th(o)?.orig_cfg.borrow().keys().map(|k| py_str(k)).collect()),
    });

    // ---------------------------------------------- ToolHeadCommandHelper
    r.ctor("klippy.toolhead.ToolHeadCommandHelper.__init__", |a| {
        let h: Rc<dyn Any> = ToolHeadCommandHelper::new(&*super::configfile::config_from_py(a.arg(0))?)?;
        Ok(h)
    });
    method!(r, "klippy.toolhead.ToolHeadCommandHelper", ToolHeadCommandHelper, cmd_G4(h, g: PyRef) => h.cmd_g4(&*gcmd(&g)?));
    method!(r, "klippy.toolhead.ToolHeadCommandHelper", ToolHeadCommandHelper, cmd_M400(h, g: PyRef) => h.cmd_m400(&*gcmd(&g)?));
    method!(r, "klippy.toolhead.ToolHeadCommandHelper", ToolHeadCommandHelper, cmd_SET_VELOCITY_LIMIT(h, g: PyRef) =>
        h.cmd_set_velocity_limit(&*gcmd(&g)?));
    method!(r, "klippy.toolhead.ToolHeadCommandHelper", ToolHeadCommandHelper, cmd_RESET_VELOCITY_LIMIT(h, g: PyRef) =>
        h.cmd_reset_velocity_limit(&*gcmd(&g)?));
    method!(r, "klippy.toolhead.ToolHeadCommandHelper", ToolHeadCommandHelper, cmd_M204(h, g: PyRef) => h.cmd_m204(&*gcmd(&g)?));
    ro_attr!(r, "klippy.toolhead.ToolHeadCommandHelper.printer", ToolHeadCommandHelper, |h| h.printer.clone());
    ro_attr!(r, "klippy.toolhead.ToolHeadCommandHelper.toolhead", ToolHeadCommandHelper, |h| h.toolhead.clone());

    function!(r, "klippy.toolhead", add_printer_objects(config: PyRef) => {
        let c = super::configfile::config_from_py(&config)?;
        crate::toolhead::add_printer_objects(&c.get_printer(), &c)
    });
}

/// A plugin's drip completion (test() / wait())
struct PyDrip(PyRef);

impl crate::extras::motion_queuing::DripCompletion for PyDrip {
    fn test(&self) -> KResult<bool> {
        self.0.call_method("test", &[])?.is_true()
    }
    fn wait(&self, waketime: f64) -> KResult<()> {
        self.0.call_method("wait", &[&py_float(waketime)])?;
        Ok(())
    }
}
