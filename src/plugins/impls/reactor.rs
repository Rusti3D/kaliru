// klippy/reactor.py: PollReactor (the class Kalico's `reactor.Reactor` is on
// Linux), ReactorTimer
use super::*;
use crate::plugins::callable::py_function;
use crate::plugins::native::Args;
use crate::reactor::{Reactor, ReactorTimer, NEVER};

pub fn register(r: &mut Registry) {
    objects::register_class::<Reactor>("klippy.reactor", "PollReactor");
    objects::register_class::<ReactorTimer>("klippy.reactor", "ReactorTimer");
    r.method("klippy.reactor.SelectReactor.register_timer", register_timer);
    r.method("klippy.reactor.SelectReactor.update_timer", |a| {
        let t = objects::unwrap_as::<ReactorTimer>(a.arg(0))?;
        a.this::<Reactor>()?.update_timer(&t, a.f64(1)?);
        Ok(py_none())
    });
    r.method("klippy.reactor.SelectReactor.unregister_timer", |a| {
        let t = objects::unwrap_as::<ReactorTimer>(a.arg(0))?;
        a.this::<Reactor>()?.unregister_timer(&t);
        Ok(py_none())
    });
    r.method("klippy.reactor.SelectReactor.pause", |a| {
        let v = a.this::<Reactor>()?.pause(a.f64(0)?).map_err(|e| KError::Reactor(e.0))?;
        Ok(py_float(v))
    });
    // self.monotonic = chelper...get_monotonic (a function attribute)
    r.attr("klippy.reactor.SelectReactor.monotonic", |o| {
        let r = objects::unwrap_as::<Reactor>(o)?;
        py_function(Rc::new(move |_args: &[PyRef]| Ok(py_float(r.monotonic()))))
    }, None);
}

// register_timer(callback, waketime=NEVER)
fn register_timer(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let cb = a.arg(0).clone();
    let timer = reactor.register_timer(
        Box::new(move |eventtime| match cb.call(&[&py_float(eventtime)]).and_then(|r| r.as_f64()) {
            Ok(w) => w,
            Err(e) => {
                // python leaves reactor.run(); the timer does not run again
                report_unhandled(&e);
                NEVER
            }
        }),
        a.f64(1)?,
    );
    wrap(timer)
}
