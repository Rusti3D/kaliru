// klippy/extras/probe.py: PrinterProbe, ProbeEndstopWrapper, ProbePointsHelper,
// ProbeList, RetryPolicy, RetrySession. A probe a plugin hands
// PrinterProbe(config, mcu_probe) is called through (PyMcuProbe); a
// finalize_callback a plugin gives ProbePointsHelper likewise.
use super::pins::{endstop_from_py, pin_params_from_py, pin_to_py};
use super::*;
use super::homing::rails_from_py;
use crate::extras::homing::HomingMove;
use crate::extras::probe::{
    FinalizeResult, McuProbe, Multi, PointsFinalize, PrinterProbe, ProbeDistanceCollector, ProbeDistanceResult, ProbeEndstopWrapper, ProbeList,
    ProbePointsHelper, ProbeRetryState, RetryPolicy, RetrySession, RetryStrategy,
};
use crate::gcode::GCodeCommand;
use crate::mcu::{Endstop, EndstopRef, Mcu, StepperRef};
use crate::plugins::callable::py_function;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::DictField;
use crate::reactor::ReactorCompletion;
use crate::{cell_attr, method, ro_attr};

fn cfg(o: &PyRef) -> KResult<Rc<crate::configfile::ConfigWrapper>> {
    super::configfile::config_from_py(o)
}

fn gcmd(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    super::gcode::gcmd_from_py(o)
}

fn opt_gcmd(o: &PyRef) -> KResult<Option<Rc<GCodeCommand>>> {
    if o.is_none() {
        return Ok(None);
    }
    gcmd(o).map(Some)
}

fn pp(o: &PyRef) -> KResult<Rc<PrinterProbe>> {
    objects::unwrap_as::<PrinterProbe>(o)
}

fn pew(o: &PyRef) -> KResult<Rc<ProbeEndstopWrapper>> {
    objects::unwrap_as::<ProbeEndstopWrapper>(o)
}

fn pph(o: &PyRef) -> KResult<Rc<ProbePointsHelper>> {
    objects::unwrap_as::<ProbePointsHelper>(o)
}

fn rs(o: &PyRef) -> KResult<Rc<RetrySession>> {
    objects::unwrap_as::<RetrySession>(o)
}

fn is_type_error(e: &KError) -> bool {
    matches!(e, KError::Py(c, _) if c == "TypeError")
}

// --------------------------------------------------------- plugin probes

/// A plugin's probe (PrinterProbe(config, obj)): an endstop plus the probe hooks
struct PyMcuProbe {
    obj: PyRef,
    endstop: EndstopRef,
}

/// A plugin's distance collector (begin_collect_distance())
struct PyCollector(PyRef);

impl ProbeDistanceCollector for PyCollector {
    fn finish(&self, before_time: f64) -> KResult<Vec<ProbeDistanceResult>> {
        FromPy::from_py(&self.0.call_method("finish", &[&py_float(before_time)])?)
    }
    fn sample_rate(&self) -> i64 {
        let r = self.0.getattr("sample_rate").and_then(|v| if v.hasattr("__call__")? { v.call(&[]) } else { Ok(v) });
        match r.and_then(|v| v.as_f64()) {
            Ok(v) => v as i64,
            Err(e) => {
                report_unhandled(&e);
                0
            }
        }
    }
}

impl Endstop for PyMcuProbe {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.endstop.get_mcu()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.endstop.add_stepper(stepper)
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        self.endstop.get_steppers()
    }
    fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        self.endstop.home_start(print_time, sample_time, sample_count, rest_time, triggered)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.endstop.home_wait(home_end_time)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        self.endstop.query_endstop(print_time)
    }
    fn get_position_endstop(&self) -> Option<f64> {
        if !self.obj.hasattr("get_position_endstop").unwrap_or(false) {
            return None;
        }
        match self.obj.call_method("get_position_endstop", &[]).and_then(|v| v.as_f64()) {
            Ok(v) => Some(v),
            Err(e) => {
                report_unhandled(&e);
                None
            }
        }
    }
}

impl McuProbe for PyMcuProbe {
    // try: multi_probe_begin(always_restore_toolhead) except TypeError: multi_probe_begin()
    fn multi_probe_begin(&self, always_restore_toolhead: bool) -> KResult<()> {
        match self.obj.call_method("multi_probe_begin", &[&py_bool(always_restore_toolhead)]) {
            Err(e) if is_type_error(&e) => self.obj.call_method("multi_probe_begin", &[]).map(|_| ()),
            r => r.map(|_| ()),
        }
    }
    fn multi_probe_end(&self) -> KResult<()> {
        self.obj.call_method("multi_probe_end", &[])?;
        Ok(())
    }
    // a (epos, is_good) tuple, else epos (good)
    fn probing_move(&self, pos: &[f64], speed: f64, gcmd: &GCodeCommand) -> KResult<(Vec<f64>, Option<bool>)> {
        let g = super::gcode::gcmd_to_py(gcmd)?;
        let r = self.obj.call_method("probing_move", &[&pos.to_vec().to_py()?, &py_float(speed), &g])?;
        if type_name(&r)? == "tuple" && r.call_method("__len__", &[])?.as_i64()? == 2 {
            let epos: Vec<f64> = FromPy::from_py(&r.call_method("__getitem__", &[&py_int(0)])?)?;
            let is_good: Option<bool> = FromPy::from_py(&r.call_method("__getitem__", &[&py_int(1)])?)?;
            return Ok((epos, is_good));
        }
        Ok((FromPy::from_py(&r)?, Some(true)))
    }
    fn probe_prepare(&self, hmove: &Rc<HomingMove>) -> KResult<()> {
        self.obj.call_method("probe_prepare", &[&objects::wrap(hmove.clone())?])?;
        Ok(())
    }
    fn probe_finish(&self, hmove: &Rc<HomingMove>) -> KResult<()> {
        self.obj.call_method("probe_finish", &[&objects::wrap(hmove.clone())?])?;
        Ok(())
    }
    fn can_collect_distance(&self) -> bool {
        self.obj.hasattr("begin_collect_distance").unwrap_or(false)
    }
    fn begin_collect_distance(&self) -> KResult<Rc<dyn ProbeDistanceCollector>> {
        Ok(Rc::new(PyCollector(self.obj.call_method("begin_collect_distance", &[])?)))
    }
}

fn mcu_probe_to_py(p: &Rc<dyn McuProbe>) -> KResult<PyRef> {
    let any: Rc<dyn Any> = p.clone();
    if let Some(py) = any.downcast_ref::<PyMcuProbe>() {
        return Ok(py.obj.clone());
    }
    let name = Endstop::type_name(&**p);
    objects::wrap_view(any, p.clone(), name)
}

fn mcu_probe_from_py(o: &PyRef) -> KResult<Rc<dyn McuProbe>> {
    if is_mirror_instance(o)? {
        if let Some(p) = objects::unwrap_view::<dyn McuProbe>(o) {
            return Ok(p);
        }
    }
    Ok(Rc::new(PyMcuProbe { obj: o.clone(), endstop: endstop_from_py(o)? }))
}

/// A python finalize_callback(probe_offsets, results): a number, "retry"
/// or anything else (done)
fn finalize_from_py(cb: PyRef) -> PointsFinalize {
    Rc::new(move |offsets: (f64, f64, f64), results: &[Vec<f64>]| {
        let r = cb.call(&[&offsets.to_py()?, &results.to_vec().to_py()?])?;
        Ok(match type_name(&r)?.as_str() {
            "int" | "float" | "bool" => FinalizeResult::Num(r.as_f64()?),
            "str" if r.as_string()? == "retry" => FinalizeResult::Retry,
            _ => FinalizeResult::Done,
        })
    })
}

fn strategy_to_py(s: RetryStrategy) -> KResult<PyRef> {
    lookup("klippy.extras.probe", "RetryStrategy")?.call(&[&py_int(s as i64)])
}

fn strategy_from_py(o: &PyRef) -> KResult<RetryStrategy> {
    Ok(match o.as_i64()? {
        0 => RetryStrategy::Fail,
        1 => RetryStrategy::Ignore,
        2 => RetryStrategy::Retry,
        3 => RetryStrategy::Circle,
        n => return Err(KError::Py("ValueError".into(), format!("{} is not a valid RetryStrategy", n))),
    })
}

fn multi_str(m: &Multi) -> &'static str {
    match m {
        Multi::Off => "OFF",
        Multi::First => "FIRST",
        Multi::On => "ON",
    }
}

/// default=sentinel: the probe module's sentinel class means "no default"
fn probe_default(o: &PyRef) -> KResult<Option<Option<PObj>>> {
    if o.is(lookup("klippy.extras.probe", "sentinel")?.as_ptr()) {
        return Ok(None);
    }
    Ok(Some(if o.is_none() { None } else { Some(crate::plugins::py_to_pobj(o)) }))
}

fn opt_pobj_to_py(p: Option<PObj>) -> KResult<PyRef> {
    match p {
        Some(p) => pobj_to_py(&p),
        None => Ok(py_none()),
    }
}

fn endstop_of(o: &PyRef) -> KResult<EndstopRef> {
    objects::unwrap_view::<dyn Endstop>(o).ok_or_else(|| KError::Py("TypeError".into(), "kaliru: not an endstop".into()))
}

/// An endstop method by name, with python's arguments
fn endstop_call(e: &EndstopRef, name: &str, a: &[PyRef]) -> KResult<PyRef> {
    match name {
        "get_mcu" => e.get_mcu().to_py(),
        "add_stepper" => {
            e.add_stepper(&objects::unwrap_as::<crate::stepper::McuStepper>(&a[0])?)?;
            Ok(py_none())
        }
        "get_steppers" => {
            let steppers: KResult<Vec<PyRef>> = e.get_steppers().into_iter().map(objects::wrap).collect();
            Ok(py_list(steppers?))
        }
        // home_start(print_time, sample_time, sample_count, rest_time, triggered=True)
        "home_start" => {
            let triggered = match a.get(4) {
                Some(t) => t.is_true()?,
                None => true,
            };
            objects::wrap(e.home_start(a[0].as_f64()?, a[1].as_f64()?, a[2].as_i64()?, a[3].as_f64()?, triggered)?)
        }
        "home_wait" => Ok(py_float(e.home_wait(a[0].as_f64()?)?)),
        _ => Ok(py_int(e.query_endstop(a[0].as_f64()?)?)),
    }
}

// self.<name> = self.mcu_endstop.<name>: the method as an attribute
macro_rules! endstop_attr {
    ($r:ident, $class:literal, $name:ident) => {
        $r.attr(concat!($class, ".", stringify!($name)), |o| {
            let e = endstop_of(o)?;
            py_function(Rc::new(move |a: &[PyRef]| endstop_call(&e, stringify!($name), a)))
        }, None);
    };
}

macro_rules! endstop_method {
    ($r:ident, $class:literal, $name:ident) => {
        $r.method(concat!($class, ".", stringify!($name)), |a| endstop_call(&endstop_of(a.slf.as_ref().expect("method"))?, stringify!($name), &a.args));
    };
}

pub fn register(r: &mut Registry) {
    objects::register_view::<ProbeEndstopWrapper>(|a| -> Box<dyn Any> {
        Box::new(a.downcast::<ProbeEndstopWrapper>().expect("type") as Rc<dyn McuProbe>)
    });
    // endstop wrappers: Kalico assigns the mcu endstop's methods
    // (self.get_mcu = self.mcu_endstop.get_mcu, ...) or defines its own;
    // kaliru's wrappers do either through their Endstop interface
    macro_rules! endstop_view {
        ($($ty:ty),*) => {$(
            objects::register_view::<$ty>(|a| -> Box<dyn Any> { Box::new(a.downcast::<$ty>().expect("type") as EndstopRef) });
        )*};
    }
    endstop_view!(
        ProbeEndstopWrapper,
        crate::extras::bltouch::BLTouchEndstopWrapper,
        crate::extras::dockable_probe::DockableProbe,
        crate::extras::load_cell::load_cell_probe::TappingMove,
        crate::extras::load_cell::load_cell_probe::LoadCellEndstopWrapper,
        crate::extras::probe_eddy_current::EddyEndstopWrapper,
        crate::extras::smart_effector::SmartEffectorEndstopWrapper,
        crate::extras::tools_calibrate::ProbeEndstopWrapper,
        crate::extras::tmc::TMCSpiEndstop
    );
    endstop_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper", add_stepper);
    endstop_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper", get_mcu);
    endstop_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper", get_steppers);
    endstop_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper", home_start);
    endstop_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper", home_wait);
    endstop_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper", query_endstop);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.LoadCellEndstopWrapper", add_stepper);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.LoadCellEndstopWrapper", get_mcu);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.LoadCellEndstopWrapper", get_steppers);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.LoadCellEndstopWrapper", home_start);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.LoadCellEndstopWrapper", home_wait);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.LoadCellEndstopWrapper", query_endstop);
    endstop_attr!(r, "klippy.extras.smart_effector.SmartEffectorEndstopWrapper", add_stepper);
    endstop_attr!(r, "klippy.extras.smart_effector.SmartEffectorEndstopWrapper", get_mcu);
    endstop_attr!(r, "klippy.extras.smart_effector.SmartEffectorEndstopWrapper", get_steppers);
    endstop_attr!(r, "klippy.extras.smart_effector.SmartEffectorEndstopWrapper", home_start);
    endstop_attr!(r, "klippy.extras.smart_effector.SmartEffectorEndstopWrapper", home_wait);
    endstop_attr!(r, "klippy.extras.smart_effector.SmartEffectorEndstopWrapper", query_endstop);
    endstop_attr!(r, "klippy.extras.tools_calibrate.ProbeEndstopWrapper", add_stepper);
    endstop_attr!(r, "klippy.extras.tools_calibrate.ProbeEndstopWrapper", get_mcu);
    endstop_attr!(r, "klippy.extras.tools_calibrate.ProbeEndstopWrapper", get_steppers);
    endstop_attr!(r, "klippy.extras.tools_calibrate.ProbeEndstopWrapper", home_start);
    endstop_attr!(r, "klippy.extras.tools_calibrate.ProbeEndstopWrapper", home_wait);
    endstop_attr!(r, "klippy.extras.tools_calibrate.ProbeEndstopWrapper", query_endstop);
    // TMCSpiEndstop(printer, mcu_tmc): sensorless homing over spi
    r.ctor("klippy.extras.tmc.TMCSpiEndstop.__init__", |a| {
        let printer = objects::unwrap_as::<crate::printer::Printer>(a.arg(0))?;
        let mcu_tmc = super::tmc::mcu_tmc_from_py(a.arg(1))?;
        let e: Rc<dyn Any> = crate::extras::tmc::TMCSpiEndstop::new(&printer, &mcu_tmc).map_err(KError::Config)?;
        Ok(e)
    });
    endstop_method!(r, "klippy.extras.tmc.TMCSpiEndstop", add_stepper);
    endstop_method!(r, "klippy.extras.tmc.TMCSpiEndstop", get_steppers);
    endstop_method!(r, "klippy.extras.tmc.TMCSpiEndstop", home_start);
    endstop_method!(r, "klippy.extras.tmc.TMCSpiEndstop", home_wait);
    endstop_method!(r, "klippy.extras.tmc.TMCSpiEndstop", query_endstop);
    endstop_method!(r, "klippy.extras.probe_eddy_current.EddyEndstopWrapper", add_stepper);
    endstop_method!(r, "klippy.extras.probe_eddy_current.EddyEndstopWrapper", get_mcu);
    endstop_method!(r, "klippy.extras.probe_eddy_current.EddyEndstopWrapper", get_steppers);
    endstop_method!(r, "klippy.extras.probe_eddy_current.EddyEndstopWrapper", home_start);
    endstop_method!(r, "klippy.extras.probe_eddy_current.EddyEndstopWrapper", home_wait);
    endstop_method!(r, "klippy.extras.probe_eddy_current.EddyEndstopWrapper", query_endstop);
    // home_start their own, the rest the mcu endstop's
    endstop_method!(r, "klippy.extras.bltouch.BLTouchEndstopWrapper", home_start);
    endstop_attr!(r, "klippy.extras.bltouch.BLTouchEndstopWrapper", add_stepper);
    endstop_attr!(r, "klippy.extras.bltouch.BLTouchEndstopWrapper", get_mcu);
    endstop_attr!(r, "klippy.extras.bltouch.BLTouchEndstopWrapper", get_steppers);
    endstop_attr!(r, "klippy.extras.bltouch.BLTouchEndstopWrapper", home_wait);
    endstop_attr!(r, "klippy.extras.bltouch.BLTouchEndstopWrapper", query_endstop);
    endstop_method!(r, "klippy.extras.dockable_probe.DockableProbe", home_start);
    endstop_attr!(r, "klippy.extras.dockable_probe.DockableProbe", add_stepper);
    endstop_attr!(r, "klippy.extras.dockable_probe.DockableProbe", get_mcu);
    endstop_attr!(r, "klippy.extras.dockable_probe.DockableProbe", get_steppers);
    endstop_attr!(r, "klippy.extras.dockable_probe.DockableProbe", home_wait);
    endstop_attr!(r, "klippy.extras.dockable_probe.DockableProbe", query_endstop);
    endstop_method!(r, "klippy.extras.load_cell.load_cell_probe.TappingMove", home_start);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.TappingMove", add_stepper);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.TappingMove", get_mcu);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.TappingMove", get_steppers);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.TappingMove", home_wait);
    endstop_attr!(r, "klippy.extras.load_cell.load_cell_probe.TappingMove", query_endstop);

    // ---------------------------------------------------- ProbeRetryState
    r.ctor("klippy.extras.probe.ProbeRetryState.__init__", |a| {
        let pos: (f64, f64) = FromPy::from_py(a.arg(0))?;
        let s: Rc<dyn Any> = Rc::new(ProbeRetryState::new(pos, &objects::unwrap_as::<RetryPolicy>(a.arg(1))?));
        Ok(s)
    });
    fn prs(a: &crate::plugins::native::Args) -> KResult<Rc<ProbeRetryState>> {
        a.this::<ProbeRetryState>()
    }
    r.method("klippy.extras.probe.ProbeRetryState.has_retries_remaining", |a| {
        let s = prs(a)?;
        Ok(py_bool(s.has_retries_remaining(&s.retry_policy)))
    });
    r.method("klippy.extras.probe.ProbeRetryState.get_attempt", |a| Ok(py_int(prs(a)?.bad_probe_count.get())));
    r.method("klippy.extras.probe.ProbeRetryState.get_bad_probe_count", |a| Ok(py_int(prs(a)?.bad_probe_count.get())));
    r.method("klippy.extras.probe.ProbeRetryState.evaluate_probe", |a| {
        let s = prs(a)?;
        let is_good: Option<bool> = if a.arg(0).is_none() { None } else { Some(a.bool(0)?) };
        Ok(py_bool(s.evaluate_probe(&s.retry_policy, is_good, &*gcmd(a.arg(1))?)?))
    });
    // get_position(): (x, y, None)
    r.method("klippy.extras.probe.ProbeRetryState.get_position", |a| {
        let s = prs(a)?;
        let p = s.get_position(&s.retry_policy)?;
        Ok(py_tuple(p.iter().map(|v| v.map_or_else(py_none, py_float)).collect()))
    });
    r.attr("klippy.extras.probe.ProbeRetryState.retry_policy", |o| objects::wrap(objects::unwrap_as::<ProbeRetryState>(o)?.retry_policy.clone()), None);

    // ------------------------------------------------------- RetryPolicy
    r.ctor("klippy.extras.probe.RetryPolicy.__init__", |a| {
        let p: Rc<dyn Any> = Rc::new(RetryPolicy::new(&*cfg(a.arg(0))?)?);
        Ok(p)
    });
    method!(r, "klippy.extras.probe.RetryPolicy", RetryPolicy, customize(p, g: PyRef) => p.customize(&*gcmd(&g)?));
    ro_attr!(r, "klippy.extras.probe.RetryPolicy._cfg_retry_speed", RetryPolicy, |p| p.cfg_retry_speed);
    ro_attr!(r, "klippy.extras.probe.RetryPolicy._cfg_bad_probe_retries", RetryPolicy, |p| p.cfg_bad_probe_retries);
    ro_attr!(r, "klippy.extras.probe.RetryPolicy._cfg_pattern_spacing", RetryPolicy, |p| p.cfg_pattern_spacing);
    r.attr("klippy.extras.probe.RetryPolicy._cfg_bad_probe_strategy", |o| {
        strategy_to_py(objects::unwrap_as::<RetryPolicy>(o)?.cfg_bad_probe_strategy)
    }, None);
    cell_attr!(r, "klippy.extras.probe.RetryPolicy.retry_speed", RetryPolicy, retry_speed);
    cell_attr!(r, "klippy.extras.probe.RetryPolicy.bad_probe_retries", RetryPolicy, bad_probe_retries);
    cell_attr!(r, "klippy.extras.probe.RetryPolicy.pattern_spacing", RetryPolicy, pattern_spacing);
    r.attr("klippy.extras.probe.RetryPolicy.bad_probe_strategy", |o| {
        strategy_to_py(objects::unwrap_as::<RetryPolicy>(o)?.bad_probe_strategy.get())
    }, Some(|o, v| {
        objects::unwrap_as::<RetryPolicy>(o)?.bad_probe_strategy.set(strategy_from_py(v)?);
        Ok(())
    }));

    // ------------------------------------------------------ RetrySession
    r.ctor("klippy.extras.probe.RetrySession.__init__", |a| {
        let s: Rc<dyn Any> = Rc::new(RetrySession::new(&*cfg(a.arg(0))?)?);
        Ok(s)
    });
    r.method("klippy.extras.probe.RetrySession._quantize_position", |a| {
        let pos: (f64, f64) = FromPy::from_py(a.arg(0))?;
        let (x, y) = RetrySession::quantize_position(pos);
        Ok(py_tuple(vec![py_int(x as i64), py_int(y as i64)]))
    });
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, start(s, g: PyRef) => s.start(&*gcmd(&g)?));
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, end(s) => s.end());
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, set_position(s, pos: Vec<f64>) => s.set_position(&pos));
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, get_position(s) => s.get_position());
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, get_probe_position(s) => s.get_probe_position().map(|p| p.to_vec()));
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, can_retry(s) => s.can_retry());
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, get_bad_probe_count(s) => s.get_bad_probe_count());
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, evaluate_probe(s, is_good: Option<bool>) => s.evaluate_probe(is_good));
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, scrub_nozzle(s) => s.scrub_nozzle());
    method!(r, "klippy.extras.probe.RetrySession", RetrySession, reset_all(s) => s.reset_all());
    r.method("klippy.extras.probe.RetrySession.set_retry_strategy", |a| {
        a.this::<RetrySession>()?.set_retry_strategy(strategy_from_py(a.arg(0))?);
        Ok(py_none())
    });
    ro_attr!(r, "klippy.extras.probe.RetrySession.retry_policy", RetrySession, |s| s.retry_policy.clone());
    ro_attr!(r, "klippy.extras.probe.RetrySession._pos", RetrySession, |s| s.pos.get());

    // ---------------------------------------------------------- ProbeList
    r.method("klippy.extras.probe.ProbeList.get_list", |a| {
        crate::extras::probe::probe_list_get_list(&objects::unwrap_as::<Printer>(a.arg(0))?)?.to_py()
    });
    r.ctor("klippy.extras.probe.ProbeList.__init__", |a| {
        let l: Rc<dyn Any> = ProbeList::new(&objects::unwrap_as::<Printer>(a.arg(0))?)?;
        Ok(l)
    });
    method!(r, "klippy.extras.probe.ProbeList", ProbeList, _handle_connect(l) => l.handle_connect());
    r.method("klippy.extras.probe.ProbeList.get_all", |a| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in a.this::<ProbeList>()?.get_all() {
            d.call_method("__setitem__", &[&py_str(&k), &pobj_to_py(&v)?])?;
        }
        Ok(d)
    });
    r.method("klippy.extras.probe.ProbeList.get_probe", |a| opt_pobj_to_py(a.this::<ProbeList>()?.get_probe(&a.string(0)?)));
    r.method("klippy.extras.probe.ProbeList.get_default_probe", |a| opt_pobj_to_py(a.this::<ProbeList>()?.get_default_probe()));
    // get_command_probe(gcmd, default=sentinel, no_default_error=...)
    r.method("klippy.extras.probe.ProbeList.get_command_probe", |a| {
        let g = gcmd(a.arg(0))?;
        let err: Option<String> = FromPy::from_py(a.arg(2))?;
        opt_pobj_to_py(a.this::<ProbeList>()?.get_command_probe(&g, probe_default(a.arg(1))?, err.as_deref())?)
    });
    r.method("klippy.extras.probe.ProbeList.get_config_probe", |a| {
        let c = cfg(a.arg(0))?;
        opt_pobj_to_py(a.this::<ProbeList>()?.get_config_probe(&c, probe_default(a.arg(1))?)?)
    });
    // add_probe_object(obj, config): obj.probe_name / obj.is_default_probe
    r.method("klippy.extras.probe.ProbeList.add_probe_object", |a| {
        let obj = a.arg(0);
        let name = obj.getattr("probe_name")?.as_string()?;
        let is_default = obj.getattr("is_default_probe")?.is_true()?;
        let c = cfg(a.arg(1))?;
        pobj_to_py(&a.this::<ProbeList>()?.add_probe_object(&name, is_default, crate::plugins::py_to_pobj(obj), &c)?)
    });
    method!(r, "klippy.extras.probe.ProbeList", ProbeList, cmd_LIST_PROBES(l, g: PyRef) => l.cmd_list_probes(&*gcmd(&g)?));
    ro_attr!(r, "klippy.extras.probe.ProbeList.printer", ProbeList, |l| l.printer.clone());
    r.attr("klippy.extras.probe.ProbeList.default_probe", |o| match objects::unwrap_as::<ProbeList>(o)?.default_probe.borrow().clone() {
        Some((n, p)) => Ok(py_tuple(vec![py_str(&n), pobj_to_py(&p)?])),
        None => Ok(py_none()),
    }, None);
    r.dict("klippy.extras.probe.ProbeList.probes", DictField {
        get: |o, _f, key| match objects::unwrap_as::<ProbeList>(o)?.probes.borrow().get(&key.as_string()?) {
            Some((p, _)) => pobj_to_py(p),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |o, _f, key, value| {
            let is_default = value.getattr("is_default_probe").and_then(|v| v.is_true()).unwrap_or(false);
            objects::unwrap_as::<ProbeList>(o)?.probes.borrow_mut().insert(key.as_string()?, (crate::plugins::py_to_pobj(value), is_default));
            Ok(())
        },
        del: |o, _f, key| match objects::unwrap_as::<ProbeList>(o)?.probes.borrow_mut().shift_remove(&key.as_string()?) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(objects::unwrap_as::<ProbeList>(o)?.probes.borrow().keys().map(|k| py_str(k)).collect()),
    });

    // ------------------------------------------------------- PrinterProbe
    r.ctor("klippy.extras.probe.PrinterProbe.__init__", |a| {
        let p: Rc<dyn Any> = PrinterProbe::new(&*cfg(a.arg(0))?, mcu_probe_from_py(a.arg(1))?)?;
        Ok(p)
    });
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, register_commands(p, key: Option<String>) => p.register_commands(key.as_deref()));
    r.method("klippy.extras.probe.PrinterProbe._handle_homing_move_begin", |a| {
        let p = a.this::<PrinterProbe>()?;
        let hmove = objects::unwrap_as::<HomingMove>(a.arg(0))?;
        let me = p.endstop();
        if hmove.get_mcu_endstops().iter().any(|e| Rc::ptr_eq(e, &me)) {
            p.mcu_probe.probe_prepare(&hmove)?;
        }
        Ok(py_none())
    });
    r.method("klippy.extras.probe.PrinterProbe._handle_homing_move_end", |a| {
        let p = a.this::<PrinterProbe>()?;
        let hmove = objects::unwrap_as::<HomingMove>(a.arg(0))?;
        let me = p.endstop();
        if hmove.get_mcu_endstops().iter().any(|e| Rc::ptr_eq(e, &me)) {
            p.mcu_probe.probe_finish(&hmove)?;
        }
        Ok(py_none())
    });
    r.method("klippy.extras.probe.PrinterProbe._handle_home_rails_begin", |a| {
        let p = a.this::<PrinterProbe>()?;
        if p.rails_use_probe(&rails_from_py(a.arg(1))?) {
            p.multi_probe_begin(true)?;
        }
        Ok(py_none())
    });
    r.method("klippy.extras.probe.PrinterProbe._handle_home_rails_end", |a| {
        let p = a.this::<PrinterProbe>()?;
        if p.rails_use_probe(&rails_from_py(a.arg(1))?) {
            p.multi_probe_end()?;
        }
        Ok(py_none())
    });
    r.method("klippy.extras.probe.PrinterProbe._handle_command_error", |a| {
        if let Err(e) = a.this::<PrinterProbe>()?.multi_probe_end() {
            crate::logging::exception_err("Multi-probe end", &e);
        }
        Ok(py_none())
    });
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, multi_probe_begin(p, always_restore_toolhead: bool) =>
        p.multi_probe_begin(always_restore_toolhead));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, multi_probe_end(p) => p.multi_probe_end());
    r.method("klippy.extras.probe.PrinterProbe.setup_pin", |a| {
        let p = a.this::<PrinterProbe>()?;
        let pin_type = a.string(0)?;
        let params = pin_params_from_py(a.arg(1))?;
        let pin = crate::pins::PinChip::setup_pin(&*p, &pin_type, &params).map_err(KError::Pins)?;
        pin_to_py(pin, &pin_type)
    });
    r.method("klippy.extras.probe.PrinterProbe.get_lift_speed", |a| {
        let g = opt_gcmd(a.arg(0))?;
        Ok(py_float(a.this::<PrinterProbe>()?.get_lift_speed(g.as_deref())?))
    });
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, get_offsets(p) => p.get_offsets());
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, probing_move(p, speed: f64, g: PyRef) => p.probing_move(speed, &*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, _probe(p, speed: f64, g: PyRef) => p.probe(speed, &*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, _move(p, coord: Vec<Option<f64>>, speed: f64) => p.move_to(&coord, speed));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, _calc_mean(_p, positions: Vec<Vec<f64>>) => PrinterProbe::calc_mean(&positions));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, _calc_median(_p, positions: Vec<Vec<f64>>) => PrinterProbe::calc_median(&positions));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, _drop_first_result(p) => p.drop_first_result);
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, _discard_first_result(p, speed: f64, retry_session: PyRef, g: PyRef) =>
        p.discard_first_result(speed, &*rs(&retry_session)?, &*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, _retract(p, g: PyRef) => p.retract(&*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, _run_probe_with_retries(p, speed: f64, retry_session: PyRef, g: PyRef) =>
        p.run_probe_with_retries(speed, &*rs(&retry_session)?, &*gcmd(&g)?));
    // run_probe(gcmd, retry_session=None)
    r.method("klippy.extras.probe.PrinterProbe.run_probe", |a| {
        let g = gcmd(a.arg(0))?;
        let session = if a.arg(1).is_none() { None } else { Some(rs(a.arg(1))?) };
        a.this::<PrinterProbe>()?.run_probe(&g, session.as_deref()).to_py()
    });
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, cmd_PROBE(p, g: PyRef) => p.cmd_probe(&*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, cmd_QUERY_PROBE(p, g: PyRef) => p.cmd_query_probe(&*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, cmd_PROBE_ACCURACY(p, g: PyRef) => p.cmd_probe_accuracy(&*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, cmd_PROBE_CALIBRATE(p, g: PyRef) => p.cmd_probe_calibrate(&*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, cmd_Z_OFFSET_APPLY_PROBE(p, g: PyRef) => p.cmd_z_offset_apply_probe(&*gcmd(&g)?));
    method!(r, "klippy.extras.probe.PrinterProbe", PrinterProbe, probe_calibrate_finalize(p, kin_pos: Option<Vec<f64>>) =>
        p.probe_calibrate_finalize(kin_pos));
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.printer", PrinterProbe, |p| p.printer.clone());
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.name", PrinterProbe, |p| p.name.clone());
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.probe_name", PrinterProbe, |p| p.probe_name.clone());
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.speed", PrinterProbe, |p| p.speed);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.retry_speed", PrinterProbe, |p| p.retry_speed);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.lift_speed", PrinterProbe, |p| p.lift_speed);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.x_offset", PrinterProbe, |p| p.x_offset);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.y_offset", PrinterProbe, |p| p.y_offset);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.z_offset", PrinterProbe, |p| p.z_offset);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.z_position", PrinterProbe, |p| p.z_position);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.drop_first_result", PrinterProbe, |p| p.drop_first_result);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.sample_count", PrinterProbe, |p| p.sample_count);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.sample_retract_dist", PrinterProbe, |p| p.sample_retract_dist);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.samples_result", PrinterProbe, |p| p.samples_result.clone());
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.samples_tolerance", PrinterProbe, |p| p.samples_tolerance);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.samples_retries", PrinterProbe, |p| p.samples_retries);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.is_default_probe", PrinterProbe, |p| p.is_default_probe);
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.gcode", PrinterProbe, |p| p.gcode.clone());
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.gcode_move", PrinterProbe, |p| p.gcode_move.clone());
    ro_attr!(r, "klippy.extras.probe.PrinterProbe.retry_session", PrinterProbe, |p| p.retry_session.clone());
    cell_attr!(r, "klippy.extras.probe.PrinterProbe.probe_calibrate_z", PrinterProbe, probe_calibrate_z);
    cell_attr!(r, "klippy.extras.probe.PrinterProbe.multi_probe_pending", PrinterProbe, multi_probe_pending);
    cell_attr!(r, "klippy.extras.probe.PrinterProbe.last_z_result", PrinterProbe, last_z_result);
    cell_attr!(r, "klippy.extras.probe.PrinterProbe.was_last_result_good", PrinterProbe, was_last_result_good);
    r.attr("klippy.extras.probe.PrinterProbe.mcu_probe", |o| mcu_probe_to_py(&pp(o)?.mcu_probe), None);
    r.attr("klippy.extras.probe.PrinterProbe.last_state", |o| Ok(from_pyval(&pp(o)?.last_state.borrow())), Some(|o, v| {
        *pp(o)?.last_state.borrow_mut() = to_pyval(v)?;
        Ok(())
    }));

    // ------------------------------------------------ ProbeEndstopWrapper
    r.ctor("klippy.extras.probe.ProbeEndstopWrapper.__init__", |a| {
        let p: Rc<dyn Any> = ProbeEndstopWrapper::new(&*cfg(a.arg(0))?)?;
        Ok(p)
    });
    method!(r, "klippy.extras.probe.ProbeEndstopWrapper", ProbeEndstopWrapper, _handle_mcu_identify(p) => p.handle_mcu_identify());
    method!(r, "klippy.extras.probe.ProbeEndstopWrapper", ProbeEndstopWrapper, _raise_probe(p) => p.raise_probe());
    method!(r, "klippy.extras.probe.ProbeEndstopWrapper", ProbeEndstopWrapper, _lower_probe(p) => p.lower_probe());
    method!(r, "klippy.extras.probe.ProbeEndstopWrapper", ProbeEndstopWrapper, multi_probe_begin(p) => McuProbe::multi_probe_begin(&*p, false));
    method!(r, "klippy.extras.probe.ProbeEndstopWrapper", ProbeEndstopWrapper, multi_probe_end(p) => McuProbe::multi_probe_end(&*p));
    // probing_move(pos, speed, gcmd): phoming.probing_move(self, pos, speed)
    r.method("klippy.extras.probe.ProbeEndstopWrapper.probing_move", |a| {
        let pos: Vec<f64> = FromPy::from_py(a.arg(0))?;
        let g = gcmd(a.arg(2))?;
        McuProbe::probing_move(&*a.this::<ProbeEndstopWrapper>()?, &pos, a.f64(1)?, &g)?.0.to_py()
    });
    r.method("klippy.extras.probe.ProbeEndstopWrapper.probe_prepare", |a| {
        McuProbe::probe_prepare(&*a.this::<ProbeEndstopWrapper>()?, &objects::unwrap_as::<HomingMove>(a.arg(0))?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.probe.ProbeEndstopWrapper.probe_finish", |a| {
        McuProbe::probe_finish(&*a.this::<ProbeEndstopWrapper>()?, &objects::unwrap_as::<HomingMove>(a.arg(0))?)?;
        Ok(py_none())
    });
    method!(r, "klippy.extras.probe.ProbeEndstopWrapper", ProbeEndstopWrapper, get_position_endstop(p) => p.position_endstop);
    ro_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper.printer", ProbeEndstopWrapper, |p| p.printer.clone());
    ro_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper.position_endstop", ProbeEndstopWrapper, |p| p.position_endstop);
    ro_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper.stow_on_each_sample", ProbeEndstopWrapper, |p| p.stow_on_each_sample);
    ro_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper.activate_gcode", ProbeEndstopWrapper, |p| p.activate_gcode.clone());
    ro_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper.deactivate_gcode", ProbeEndstopWrapper, |p| p.deactivate_gcode.clone());
    ro_attr!(r, "klippy.extras.probe.ProbeEndstopWrapper.mcu_endstop", ProbeEndstopWrapper, |p| p.mcu_endstop.clone());
    r.attr("klippy.extras.probe.ProbeEndstopWrapper.multi", |o| Ok(py_str(multi_str(&pew(o)?.multi.get()))), Some(|o, v| {
        let m = match v.as_string()?.as_str() {
            "OFF" => Multi::Off,
            "FIRST" => Multi::First,
            "ON" => Multi::On,
            other => return Err(KError::Py("ValueError".into(), format!("kaliru: probe multi state {:?}", other))),
        };
        pew(o)?.multi.set(m);
        Ok(())
    }));
    // bltouch: the same "OFF" / "FIRST" / "ON" strings
    r.attr("klippy.extras.bltouch.BLTouchEndstopWrapper.multi", |o| {
        use crate::extras::bltouch::{BLTouchEndstopWrapper, Multi as B};
        Ok(py_str(match objects::unwrap_as::<BLTouchEndstopWrapper>(o)?.multi.get() {
            B::Off => "OFF",
            B::First => "FIRST",
            B::On => "ON",
        }))
    }, None);
    // dockable_probe: MULTI_OFF / MULTI_FIRST / MULTI_ON are 0 / 1 / 2
    {
        use crate::extras::dockable_probe::DockableProbe;
        fn dp(o: &PyRef) -> KResult<Rc<DockableProbe>> {
            objects::unwrap_as::<DockableProbe>(o)
        }
        // a position: [x, y, z] with None for what the config left out
        fn coord(c: &[Option<f64>; 3]) -> KResult<PyRef> {
            c.to_vec().to_py()
        }
        r.attr("klippy.extras.dockable_probe.DockableProbe.multi", |o| {
            Ok(py_int(match dp(o)?.multi.get() {
                Multi::Off => 0,
                Multi::First => 1,
                Multi::On => 2,
            }))
        }, None);
        r.attr("klippy.extras.dockable_probe.DockableProbe.approach_position", |o| coord(&dp(o)?.approach_position), None);
        r.attr("klippy.extras.dockable_probe.DockableProbe.detach_position", |o| coord(&dp(o)?.detach_position), None);
        r.attr("klippy.extras.dockable_probe.DockableProbe.extract_position", |o| coord(&dp(o)?.extract_position), None);
        r.attr("klippy.extras.dockable_probe.DockableProbe.insert_position", |o| coord(&dp(o)?.insert_position), None);
        r.attr("klippy.extras.dockable_probe.DockableProbe.dock_position", |o| coord(&dp(o)?.dock_position), None);
        r.attr("klippy.extras.dockable_probe.DockableProbe.safe_position", |o| coord(&dp(o)?.safe_position), None);
        r.attr("klippy.extras.dockable_probe.DockableProbe.safe_points", |o| dp(o)?.safe_points.clone().to_py(), None);
        r.attr("klippy.extras.dockable_probe.DockableProbe.probe_states", |o| {
            dp(o)?;
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in [(1, "ATTACHED"), (2, "DOCKED"), (0, "UNKNOWN")] {
                d.call_method("__setitem__", &[&py_int(k), &py_str(v)])?;
            }
            Ok(d)
        }, None);
        // self.get_probe_state = pstate.get_probe_state
        r.attr("klippy.extras.dockable_probe.DockableProbe.get_probe_state", |o| {
            let p = dp(o)?;
            py_function(Rc::new(move |_a: &[PyRef]| Ok(py_int(p.pstate.get_probe_state()?))))
        }, None);
    }
    // self.get_mcu = mcu_endstop.get_mcu, ... (the mcu endstop's bound methods)
    macro_rules! endstop_method_attr {
        ($($name:literal),*) => {
            $(
                r.attr(concat!("klippy.extras.probe.ProbeEndstopWrapper.", $name), |o| {
                    objects::wrap(pew(o)?.mcu_endstop.clone())?.getattr($name)
                }, None);
            )*
        };
    }
    endstop_method_attr!("get_mcu", "add_stepper", "get_steppers", "home_start", "home_wait", "query_endstop");

    // -------------------------------------------------- ProbePointsHelper
    // ProbePointsHelper(config, finalize_callback, default_points=None,
    //   option_name="points", use_offsets=False, enable_horizontal_z_clearance=False)
    r.ctor("klippy.extras.probe.ProbePointsHelper.__init__", |a| {
        let default_points: Option<Vec<(f64, f64)>> = FromPy::from_py(a.arg(2))?;
        let cb = a.arg(1).clone();
        // (python's own callback kept for self.finalize_callback)
        if let Some(slf) = a.slf.as_ref() {
            lookup("builtins", "object")?.getattr("__setattr__")?.call(&[slf, &py_str("_k_finalize"), &cb])?;
        }
        let h: Rc<dyn Any> = ProbePointsHelper::new(&*cfg(a.arg(0))?, finalize_from_py(cb), default_points, &a.string(3)?, a.bool(4)?, a.bool(5)?)?;
        Ok(h)
    });
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, get_probe_points(h) => h.get_probe_points());
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, minimum_points(h, n: usize) => h.minimum_points(n));
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, update_probe_points(h, points: Vec<(f64, f64)>, min_points: usize) =>
        h.update_probe_points(points, min_points));
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, use_xy_offsets(h, use_offsets: bool) => h.use_xy_offsets(use_offsets));
    r.method("klippy.extras.probe.ProbePointsHelper.get_lift_speed", |a| {
        let g = opt_gcmd(a.arg(0))?;
        Ok(py_float(a.this::<ProbePointsHelper>()?.get_lift_speed(g.as_deref())?))
    });
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, _store_probe_result(h, position: Vec<f64>) => h.store_probe_result(position));
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, _lift_toolhead(h) => h.lift_toolhead());
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, _next_pos(h) => h.next_pos().map(|p| p.to_vec()));
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, _move_next(h) => h.move_next());
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, start_probe(h, g: PyRef) => h.start_probe(&*gcmd(&g)?));
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, _start_rapid_scan(h, g: PyRef, probe: PyRef) =>
        h.start_rapid_scan(&*gcmd(&g)?, &pp(&probe)?));
    r.method("klippy.extras.probe.ProbePointsHelper._generate_rapid_path", |a| {
        let path = a.this::<ProbePointsHelper>()?.generate_rapid_path();
        let items = path.into_iter().map(|(pos, b)| Ok(py_tuple(vec![pos.to_py()?, py_bool(b)]))).collect::<KResult<Vec<_>>>()?;
        Ok(py_list(items))
    });
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, _manual_probe_start(h) => h.manual_probe_start());
    method!(r, "klippy.extras.probe.ProbePointsHelper", ProbePointsHelper, _manual_probe_finalize(h, kin_pos: Option<Vec<f64>>) =>
        h.manual_probe_finalize(kin_pos));
    r.attr("klippy.extras.probe.ProbePointsHelper.finalize_callback", |o| {
        let d = o.getattr("__dict__")?;
        if let Ok(cb) = d.call_method("__getitem__", &[&py_str("_k_finalize")]) {
            return Ok(cb);
        }
        let f = pph(o)?.finalize_callback.clone();
        crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| {
            let offsets: (f64, f64, f64) = {
                let v: Vec<f64> = FromPy::from_py(&a[0])?;
                (v[0], v[1], v[2])
            };
            let results: Vec<Vec<f64>> = FromPy::from_py(&a[1])?;
            Ok(match f(offsets, &results)? {
                FinalizeResult::Num(n) => py_float(n),
                FinalizeResult::Retry => py_str("retry"),
                FinalizeResult::Done => py_none(),
            })
        }))
    }, None);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.printer", ProbePointsHelper, |h| h.printer.clone());
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.name", ProbePointsHelper, |h| h.name.clone());
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.gcode", ProbePointsHelper, |h| h.gcode.clone());
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.default_horizontal_move_z", ProbePointsHelper, |h| h.default_horizontal_move_z);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.enable_horizontal_z_clearance", ProbePointsHelper, |h| h.enable_horizontal_z_clearance);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.default_horizontal_z_clearance", ProbePointsHelper, |h| h.default_horizontal_z_clearance);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.adaptive_horizontal_move_z", ProbePointsHelper, |h| h.adaptive_horizontal_move_z);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.min_horizontal_move_z", ProbePointsHelper, |h| h.min_horizontal_move_z);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.speed", ProbePointsHelper, |h| h.speed);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.alternate_probe_direction", ProbePointsHelper, |h| h.alternate_probe_direction);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.start_reverse", ProbePointsHelper, |h| h.start_reverse);
    ro_attr!(r, "klippy.extras.probe.ProbePointsHelper.retry_session", ProbePointsHelper, |h| h.retry_session.clone());
    cell_attr!(r, "klippy.extras.probe.ProbePointsHelper.horizontal_move_z", ProbePointsHelper, horizontal_move_z);
    cell_attr!(r, "klippy.extras.probe.ProbePointsHelper.horizontal_z_clearance", ProbePointsHelper, horizontal_z_clearance);
    cell_attr!(r, "klippy.extras.probe.ProbePointsHelper.use_offsets", ProbePointsHelper, use_offsets);
    cell_attr!(r, "klippy.extras.probe.ProbePointsHelper.enforce_lift_speed", ProbePointsHelper, enforce_lift_speed);
    cell_attr!(r, "klippy.extras.probe.ProbePointsHelper.lift_speed", ProbePointsHelper, lift_speed);
    cell_attr!(r, "klippy.extras.probe.ProbePointsHelper._probe_pass_direction_reversed", ProbePointsHelper, probe_pass_direction_reversed);
    r.attr("klippy.extras.probe.ProbePointsHelper.probe_offsets", |o| pph(o)?.probe_offsets.get().to_py(), Some(|o, v| {
        let t: Vec<f64> = FromPy::from_py(v)?;
        if t.len() != 3 {
            return Err(KError::Py("ValueError".into(), "kaliru: probe_offsets needs 3 values".into()));
        }
        pph(o)?.probe_offsets.set((t[0], t[1], t[2]));
        Ok(())
    }));
    crate::vec_attr!(r, "klippy.extras.probe.ProbePointsHelper.probe_points", ProbePointsHelper, probe_points);
    crate::vec_attr!(r, "klippy.extras.probe.ProbePointsHelper.results", ProbePointsHelper, results);

    r.method("klippy.extras.probe.load_config", |a| crate::extras::probe::load_config(&*cfg(a.arg(0))?)?.to_py());
    r.method("klippy.extras.probe.load_config_prefix", |a| crate::extras::probe::load_config_prefix(&*cfg(a.arg(0))?)?.to_py());
}
