// klippy/extras/trad_rack: its selector toolhead (a kaliru ToolHead python
// sees as Kalico's TradRackToolHead subclass; native.rs ALIASES serves the
// inherited names) and what kaliru keeps as fields / closures
use super::*;
use crate::extras::trad_rack::{MovingAverageFilter, SyncState, TradRack, TradRackExtruderSyncManager, TradRackKinematics, TradRackRunoutSensor};
use crate::plugins::callable::py_function;
use crate::plugins::convert::ToPy;
use crate::ro_attr;
use crate::toolhead::ToolHead;

/// the trad_rack whose selector toolhead this is
fn tr_of(toolhead: &dyn Any) -> Option<Rc<TradRack>> {
    let tr = crate::printer::downcast_rc::<TradRack>(printer()?.lookup_object_opt("trad_rack")?)?;
    std::ptr::eq(Rc::as_ptr(&tr.tr_toolhead) as *const (), toolhead as *const dyn Any as *const ()).then_some(tr)
}

fn tr(o: &PyRef) -> KResult<Rc<TradRack>> {
    let th = objects::unwrap_as::<ToolHead>(o)?;
    tr_of(&*th).ok_or_else(|| KError::Py("AttributeError".into(), "kaliru: not trad_rack's toolhead".into()))
}

fn endstops(v: &[(crate::mcu::EndstopRef, String)]) -> KResult<PyRef> {
    let items: KResult<Vec<PyRef>> = v.iter().map(|(e, n)| Ok(py_tuple(vec![super::pins::endstop_to_py(e)?, py_str(n)]))).collect();
    Ok(py_list(items?))
}

pub fn register(r: &mut Registry) {
    objects::register_class_fn::<ToolHead>(|a| match tr_of(a) {
        Some(_) => ("klippy.extras.trad_rack", "TradRackToolHead"),
        None => ("klippy.toolhead", "ToolHead"),
    });
    // the selector / filament driver limits (the kinematics took them from the toolhead)
    r.attr("klippy.extras.trad_rack.TradRackToolHead.sel_max_velocity", |o| Ok(py_float(tr(o)?.tr_kinematics.sel_max_velocity)), None);
    r.attr("klippy.extras.trad_rack.TradRackToolHead.sel_max_accel", |o| Ok(py_float(tr(o)?.tr_kinematics.sel_max_accel)), None);
    r.attr("klippy.extras.trad_rack.TradRackToolHead.fil_max_velocity", |o| Ok(py_float(tr(o)?.tr_kinematics.fil_max_velocity)), None);
    r.attr("klippy.extras.trad_rack.TradRackToolHead.fil_max_accel", |o| Ok(py_float(tr(o)?.tr_kinematics.fil_max_accel)), None);
    r.method("klippy.extras.trad_rack.TradRackToolHead.get_sel_max_velocity", |a| {
        let k = tr(a.slf.as_ref().expect("method"))?.tr_kinematics.clone();
        Ok(py_tuple(vec![py_float(k.sel_max_velocity), py_float(k.sel_max_accel)]))
    });
    r.method("klippy.extras.trad_rack.TradRackToolHead.get_fil_max_velocity", |a| {
        let k = tr(a.slf.as_ref().expect("method"))?.tr_kinematics.clone();
        Ok(py_tuple(vec![py_float(k.fil_max_velocity), py_float(k.fil_max_accel)]))
    });
    // (set to 0.0 in __init__ and never changed)
    r.attr("klippy.extras.trad_rack.TradRackToolHead.mcr_pseudo_accel", |o| {
        tr(o)?;
        Ok(py_float(0.0))
    }, None);
    r.attr("klippy.extras.trad_rack.TradRackToolHead.danger_options", |o| tr(o)?.printer.lookup_object("danger_options")?.to_py(), None);

    // ---- TradRack
    r.attr("klippy.extras.trad_rack.TradRack.fil_driver_endstops", |o| endstops(&objects::unwrap_as::<TradRack>(o)?.fil_driver_endstops), None);
    r.attr("klippy.extras.trad_rack.TradRack.toolhead_fil_endstops", |o| endstops(&objects::unwrap_as::<TradRack>(o)?.toolhead_fil_endstops), None);
    r.attr("klippy.extras.trad_rack.TradRack.selector_endstops", |o| {
        use crate::stepper::Rail;
        endstops(&objects::unwrap_as::<TradRack>(o)?.tr_kinematics.get_selector_rail().get_endstops())
    }, None);
    r.attr("klippy.extras.trad_rack.TradRack.variables", |o| {
        match objects::unwrap_as::<TradRack>(o)?.variables.borrow().as_ref() {
            Some(v) => {
                let d = lookup("builtins", "dict")?.call(&[])?;
                for (k, val) in v {
                    d.call_method("__setitem__", &[&py_str(k), &crate::plugins::values::value_to_py(val)?])?;
                }
                Ok(d)
            }
            None => Ok(py_none()),
        }
    }, None);
    // {"load toolhead", "check condition", "runout": the bound resume methods}
    r.attr("klippy.extras.trad_rack.TradRack.resume_callbacks", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, m) in [("load toolhead", "_resume_load_toolhead"), ("check condition", "_resume_check_condition"), ("runout", "_resume_runout")] {
            d.call_method("__setitem__", &[&py_str(k), &o.getattr(m)?])?;
        }
        Ok(d)
    }, None);

    // ---- TradRackKinematics: stepper_count (selector and filament driver
    // rails), is_extruder_synced()
    ro_attr!(r, "klippy.extras.trad_rack.TradRackKinematics.stepper_count", TradRackKinematics, |s| s.rails.len() as i64);
    r.attr("klippy.extras.trad_rack.TradRackKinematics.is_extruder_synced", |o| {
        let f = objects::unwrap_as::<TradRackKinematics>(o)?.is_extruder_synced.clone();
        py_function(Rc::new(move |_a: &[PyRef]| Ok(py_bool(f()))))
    }, None);

    // ---- TradRackExtruderSyncManager
    r.method("klippy.extras.trad_rack.TradRackExtruderSyncManager.handle_connect", |a| {
        let m = a.this::<TradRackExtruderSyncManager>()?;
        *m.toolhead.borrow_mut() = Some(m.printer.lookup::<ToolHead>("toolhead")?);
        Ok(py_none())
    });
    // EXTRUDER_TO_FIL_DRIVER = 0, FIL_DRIVER_TO_EXTRUDER = 1
    r.attr("klippy.extras.trad_rack.TradRackExtruderSyncManager.sync_state", |o| {
        Ok(match objects::unwrap_as::<TradRackExtruderSyncManager>(o)?.sync_state.get() {
            Some(SyncState::ExtruderToFilDriver) => py_int(0),
            Some(SyncState::FilDriverToExtruder) => py_int(1),
            None => py_none(),
        })
    }, None);

    // ---- MovingAverageFilter.queue: collections.deque (a snapshot)
    r.attr("klippy.extras.trad_rack.MovingAverageFilter.queue", |o| {
        let q: Vec<f64> = objects::unwrap_as::<MovingAverageFilter>(o)?.queue.borrow().iter().copied().collect();
        lookup("collections", "deque")?.call(&[&q.to_py()?])
    }, None);

    // ---- TradRackRunoutSensor
    ro_attr!(r, "klippy.extras.trad_rack.TradRackRunoutSensor.active", TradRackRunoutSensor, |s| s.active.get());
    r.attr("klippy.extras.trad_rack.TradRackRunoutSensor.runout_callback", |o| {
        let cb = objects::unwrap_as::<TradRackRunoutSensor>(o)?.runout_callback.clone();
        py_function(Rc::new(move |a: &[PyRef]| {
            cb(a[0].as_f64()?)?;
            Ok(py_none())
        }))
    }, None);
}
