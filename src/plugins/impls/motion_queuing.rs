// klippy/extras/motion_queuing.py: PrinterMotionQueuing. Plugins moving
// their own steppers (AFC, Happy Hare) allocate a trapq here, append to it
// through ffi_lib.trapq_append and drip it out.
use super::chelper::{lib_fn, trapq_from_py, trapq_to_py};
use super::*;
use crate::chelper::steppersync::StepperSync;
use crate::extras::motion_queuing::{DripCompletion, FlushCallback, PrinterMotionQueuing};
use crate::plugins::gil::Gil;
use crate::{cell_attr, method, ro_attr, vec_attr};

fn mq(o: &PyRef) -> KResult<Rc<PrinterMotionQueuing>> {
    objects::unwrap_as::<PrinterMotionQueuing>(o)
}

// the flush callbacks python registered: unregister_flush_callback(cb)
// finds kaliru's callback by the python one
static PY_FLUSH: Gil<RefCell<Vec<(PyRef, FlushCallback)>>> = Gil::new(|| RefCell::new(Vec::new()));

/// A python drip completion (a plugin's own: test() / wait())
struct PyDrip(PyRef);

impl DripCompletion for PyDrip {
    fn test(&self) -> KResult<bool> {
        self.0.call_method("test", &[])?.is_true()
    }
    fn wait(&self, waketime: f64) -> KResult<()> {
        self.0.call_method("wait", &[&py_float(waketime)])?;
        Ok(())
    }
}

fn mcu_list(mcus: &[Rc<crate::mcu::Mcu>]) -> KResult<PyRef> {
    Ok(py_list(mcus.iter().map(|m| objects::wrap(m.clone())).collect::<KResult<Vec<_>>>()?))
}

pub fn register(r: &mut Registry) {
    objects::register_class::<RefCell<StepperSync>>("_k", "CData");
    r.method("klippy.extras.motion_queuing.PrinterMotionQueuing.allocate_trapq", |a| {
        objects::wrap(a.this::<PrinterMotionQueuing>()?.allocate_trapq())
    });
    r.method("klippy.extras.motion_queuing.PrinterMotionQueuing.wipe_trapq", |a| {
        let tq = trapq_from_py(a.arg(0))?.ok_or_else(|| KError::Py("TypeError".into(), "kaliru: NULL trapq".into()))?;
        a.this::<PrinterMotionQueuing>()?.wipe_trapq(&tq);
        Ok(py_none())
    });
    // lookup_trapq_append(): ffi_lib.trapq_append
    r.method("klippy.extras.motion_queuing.PrinterMotionQueuing.lookup_trapq_append", |_a| lib_fn("trapq_append"));
    r.method("klippy.extras.motion_queuing.PrinterMotionQueuing._lookup_steppersync", |a| {
        let mcu = objects::unwrap_as::<crate::mcu::Mcu>(a.arg(0))?;
        objects::wrap(a.this::<PrinterMotionQueuing>()?.lookup_steppersync(&mcu))
    });
    r.method("klippy.extras.motion_queuing.PrinterMotionQueuing.allocate_syncemitter", |a| {
        let mcu = objects::unwrap_as::<crate::mcu::Mcu>(a.arg(0))?;
        objects::wrap(a.this::<PrinterMotionQueuing>()?.allocate_syncemitter(&mcu, &a.string(1)?, a.bool(2)?))
    });
    // register_flush_callback(callback, can_add_trapq=False): callback(must_flush_time, max_step_gen_time)
    r.method("klippy.extras.motion_queuing.PrinterMotionQueuing.register_flush_callback", |a| {
        let cb = a.arg(0).clone();
        let pcb = cb.clone();
        let fcb: FlushCallback = Rc::new(move |ft, sgt| {
            pcb.call(&[&py_float(ft), &py_float(sgt)])?;
            Ok(())
        });
        PY_FLUSH.with(|l| l.borrow_mut().push((cb, fcb.clone())));
        a.this::<PrinterMotionQueuing>()?.register_flush_callback(fcb, a.bool(1)?);
        Ok(py_none())
    });
    r.method("klippy.extras.motion_queuing.PrinterMotionQueuing.unregister_flush_callback", |a| {
        let cb = a.arg(0).clone();
        let found = PY_FLUSH.with(|l| {
            let mut l = l.borrow_mut();
            let pos = l.iter().position(|(p, _)| p.is(cb.as_ptr()))?;
            Some(l.remove(pos).1)
        });
        if let Some(fcb) = found {
            a.this::<PrinterMotionQueuing>()?.unregister_flush_callback(&fcb);
        }
        Ok(py_none())
    });
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, get_kin_flush_delay(m) => m.get_kin_flush_delay());
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, check_step_generation_scan_windows(m) =>
        m.check_step_generation_scan_windows());
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, _handle_shutdown(m) => m.can_pause.set(false));
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, _advance_flush_time(
        m, want_flush_time: f64, want_step_gen_time: f64) => m.advance_flush_time(want_flush_time, want_step_gen_time));
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, _await_flush_time(m, want_flush_time: f64) =>
        m.await_flush_time(want_flush_time));
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, flush_all_steps(m) => m.flush_all_steps());
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, calc_step_gen_restart(m, est_print_time: f64) =>
        m.calc_step_gen_restart(est_print_time));
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, _flush_handler(m, eventtime: f64) =>
        m.flush_handler(eventtime));
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, _flush_handler_debug(m, eventtime: f64) =>
        m.flush_handler_debug(eventtime));
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, note_mcu_movequeue_activity(
        m, mq_time: f64, is_step_gen: bool) => m.note_mcu_movequeue_activity(mq_time, is_step_gen));
    // drip_update_time(start_time, end_time, drip_completion)
    r.method("klippy.extras.motion_queuing.PrinterMotionQueuing.drip_update_time", |a| {
        let m = a.this::<PrinterMotionQueuing>()?;
        let (start, end) = (a.f64(0)?, a.f64(1)?);
        if let Ok(c) = objects::unwrap_as::<crate::reactor::ReactorCompletion<bool>>(a.arg(2)) {
            m.drip_update_time(start, end, &*c)?;
        } else if let Ok(c) = objects::unwrap_as::<crate::reactor::ReactorCompletion<PyRef>>(a.arg(2)) {
            m.drip_update_time(start, end, &*c)?;
        } else {
            m.drip_update_time(start, end, &PyDrip(a.arg(2).clone()))?;
        }
        Ok(py_none())
    });
    method!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing", PrinterMotionQueuing, check_drip_timing(m) => m.check_drip_timing());
    // attributes
    r.attr("klippy.extras.motion_queuing.PrinterMotionQueuing.all_mcus", |o| mcu_list(&mq(o)?.all_mcus), None);
    ro_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.mcu", PrinterMotionQueuing, |m| m.mcu.clone());
    ro_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.printer", PrinterMotionQueuing, |m| m.printer.clone());
    ro_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.reactor", PrinterMotionQueuing, |m| m.reactor.clone());
    cell_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.can_pause", PrinterMotionQueuing, can_pause);
    cell_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.clear_history_time", PrinterMotionQueuing, clear_history_time);
    cell_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.do_kick_flush_timer", PrinterMotionQueuing, do_kick_flush_timer);
    cell_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.kin_flush_delay", PrinterMotionQueuing, kin_flush_delay);
    cell_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.last_flush_time", PrinterMotionQueuing, last_flush_time);
    cell_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.last_step_gen_time", PrinterMotionQueuing, last_step_gen_time);
    cell_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.need_flush_time", PrinterMotionQueuing, need_flush_time);
    cell_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.need_step_gen_time", PrinterMotionQueuing, need_step_gen_time);
    vec_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.drip_start_times", PrinterMotionQueuing, drip_start_times);
    ro_attr!(r, "klippy.extras.motion_queuing.PrinterMotionQueuing.flush_timer", PrinterMotionQueuing, |m| m.flush_timer.borrow().clone());
    r.attr("klippy.extras.motion_queuing.PrinterMotionQueuing.trapqs", |o| {
        let tqs = mq(o)?.trapqs.borrow().clone();
        Ok(py_list(tqs.into_iter().map(|t| trapq_to_py(Some(t))).collect::<KResult<Vec<_>>>()?))
    }, None);
    r.attr("klippy.extras.motion_queuing.PrinterMotionQueuing.syncemitters", |o| {
        let ses = mq(o)?.syncemitters.borrow().clone();
        Ok(py_list(ses.into_iter().map(objects::wrap).collect::<KResult<Vec<_>>>()?))
    }, None);
    // steppersyncs: [(mcu, steppersync), ...]
    r.attr("klippy.extras.motion_queuing.PrinterMotionQueuing.steppersyncs", |o| {
        let sss = mq(o)?.steppersyncs.borrow().clone();
        let items = sss.into_iter().map(|(m, s)| Ok(py_tuple(vec![objects::wrap(m)?, objects::wrap(s)?]))).collect::<KResult<Vec<_>>>()?;
        Ok(py_list(items))
    }, None);
    r.attr("klippy.extras.motion_queuing.PrinterMotionQueuing.trapq_finalize_moves", |_o| lib_fn("trapq_finalize_moves"), None);
    // (a snapshot of kaliru's callbacks: python's own as registered)
    r.attr("klippy.extras.motion_queuing.PrinterMotionQueuing.flush_callbacks", |o| {
        let fcbs = mq(o)?.flush_callbacks.borrow().clone();
        let items = fcbs
            .into_iter()
            .map(|f| {
                if let Some(p) = PY_FLUSH.with(|l| l.borrow().iter().find(|(_, c)| Rc::ptr_eq(c, &f)).map(|(p, _)| p.clone())) {
                    return Ok(p);
                }
                crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| {
                    f(a[0].as_f64()?, a[1].as_f64()?)?;
                    Ok(py_none())
                }))
            })
            .collect::<KResult<Vec<_>>>()?;
        Ok(py_list(items))
    }, None);
}
