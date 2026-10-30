// klippy/reactor.py: PollReactor (the class Kalico's `reactor.Reactor` is on
// Linux), ReactorTimer, ReactorCompletion, ReactorMutex, ReactorFileHandler.
// Completions python makes carry python objects (ReactorCompletion<PyRef>).
use super::*;
use crate::method;
use crate::plugins::callable::py_function;
use crate::plugins::convert::ToPy;
use crate::plugins::native::Args;
use crate::reactor::{FileHandler, Reactor, ReactorCompletion, ReactorMutex, ReactorTimer, NEVER};

type PyCompletion = ReactorCompletion<PyRef>;

/// ReactorPreventPause(reactor) as Kalico has it: __enter__ / __exit__
/// count the pauses off (kaliru's own guard counts from creation to drop)
pub struct PyPreventPause {
    pub reactor: Rc<Reactor>,
}

/// completion.result: the result, else the class's sentinel
fn completion_result(o: &PyRef) -> KResult<PyRef> {
    if let Ok(c) = objects::unwrap_as::<ReactorCompletion<bool>>(o) {
        return Ok(match *c.result.borrow() {
            Some(v) => py_bool(v),
            None => lookup("klippy.reactor", "ReactorCompletion")?.getattr("sentinel")?,
        });
    }
    let c = objects::unwrap_as::<PyCompletion>(o)?;
    let r = c.result.borrow().clone();
    match r {
        Some(v) => Ok(v),
        None => lookup("klippy.reactor", "ReactorCompletion")?.getattr("sentinel"),
    }
}

pub fn register(r: &mut Registry) {
    objects::register_class::<PyPreventPause>("klippy.reactor", "ReactorPreventPause");
    r.ctor("klippy.reactor.ReactorPreventPause.__init__", |a| {
        let p: Rc<dyn Any> = Rc::new(PyPreventPause { reactor: objects::unwrap_as::<Reactor>(a.arg(0))? });
        Ok(p)
    });
    r.method("klippy.reactor.ReactorPreventPause.__enter__", |a| {
        let c = &a.this::<PyPreventPause>()?.reactor.prevent_pause_count;
        c.set(c.get() + 1);
        Ok(py_none())
    });
    r.method("klippy.reactor.ReactorPreventPause.__exit__", |a| {
        let c = &a.this::<PyPreventPause>()?.reactor.prevent_pause_count;
        c.set(c.get() - 1);
        Ok(py_none())
    });
    r.attr("klippy.reactor.ReactorPreventPause.reactor", |o| objects::unwrap_as::<PyPreventPause>(o)?.reactor.clone().to_py(), None);
    r.method("klippy.reactor.SelectReactor.assert_no_pause", |a| objects::wrap(Rc::new(PyPreventPause { reactor: a.this::<Reactor>()? })));
    method!(r, "klippy.reactor.SelectReactor", Reactor, verify_can_pause(re) => re.verify_can_pause().map_err(|e| KError::Reactor(e.0)));
    // _READ / _WRITE: the poll flags of each reactor class
    r.attr("klippy.reactor.SelectReactor._READ", |_o| Ok(py_int(1)), None);
    r.attr("klippy.reactor.SelectReactor._WRITE", |_o| Ok(py_int(2)), None);
    r.attr("klippy.reactor.PollReactor._READ", |_o| Ok(py_int((libc::POLLIN | libc::POLLHUP) as i64)), None);
    r.attr("klippy.reactor.PollReactor._WRITE", |_o| Ok(py_int(libc::POLLOUT as i64)), None);
    r.attr("klippy.reactor.EPollReactor._READ", |_o| Ok(py_int((libc::EPOLLIN | libc::EPOLLHUP) as i64)), None);
    r.attr("klippy.reactor.EPollReactor._WRITE", |_o| Ok(py_int(libc::EPOLLOUT as i64)), None);
    // ReactorCompletion(reactor) / ReactorMutex(reactor, is_locked=False)
    r.ctor("klippy.reactor.ReactorCompletion.__init__", |a| {
        let c: Rc<dyn Any> = objects::unwrap_as::<Reactor>(a.arg(0))?.completion::<PyRef>();
        Ok(c)
    });
    r.attr("klippy.reactor.ReactorCompletion.result", completion_result, None);
    r.attr("klippy.reactor.ReactorCompletion.reactor", |o| {
        if let Ok(c) = objects::unwrap_as::<ReactorCompletion<bool>>(o) {
            return c.reactor.clone().to_py();
        }
        objects::unwrap_as::<PyCompletion>(o)?.reactor.clone().to_py()
    }, None);
    r.attr("klippy.reactor.ReactorCompletion.waiting", |o| {
        let w = match objects::unwrap_as::<ReactorCompletion<bool>>(o) {
            Ok(c) => c.waiting.borrow().clone(),
            Err(_) => objects::unwrap_as::<PyCompletion>(o)?.waiting.borrow().clone(),
        };
        w.to_py()
    }, None);
    r.ctor("klippy.reactor.ReactorMutex.__init__", |a| {
        let m: Rc<dyn Any> = objects::unwrap_as::<Reactor>(a.arg(0))?.mutex(a.bool(1)?);
        Ok(m)
    });
    r.method("klippy.reactor.SelectReactor.register_timer", register_timer);
    method!(r, "klippy.reactor.SelectReactor", Reactor, update_timer(re, t: Rc<ReactorTimer>, waketime: f64) => re.update_timer(&t, waketime));
    // timers.pop(timers.index(timer_handler)): ValueError for an unknown timer
    method!(r, "klippy.reactor.SelectReactor", Reactor, unregister_timer(re, t: Rc<ReactorTimer>) => {
        if !re.has_timer(&t) {
            return Err(KError::Py("ValueError".into(), "list.index(x): x not in list".into()));
        }
        re.unregister_timer(&t)
    });
    method!(r, "klippy.reactor.SelectReactor", Reactor, pause(re, waketime: f64) => re.pause(waketime).map_err(|e| KError::Reactor(e.0)));
    method!(r, "klippy.reactor.SelectReactor", Reactor, delay_for_ms(re, ms: f64) => re.delay_for_ms(ms).map_err(|e| KError::Reactor(e.0)));
    method!(r, "klippy.reactor.SelectReactor", Reactor, get_gc_stats(re) => re.get_gc_stats());
    method!(r, "klippy.reactor.SelectReactor", Reactor, completion(re) => re.completion::<PyRef>());
    r.method("klippy.reactor.SelectReactor.register_callback", register_callback);
    r.method("klippy.reactor.SelectReactor.register_async_callback", register_async_callback);
    method!(r, "klippy.reactor.SelectReactor", Reactor, async_complete(re, c: Rc<PyCompletion>, result: PyRef) => re.async_complete(&c, result));
    method!(r, "klippy.reactor.SelectReactor", Reactor, mutex(re, is_locked: bool) => re.mutex(is_locked));
    // (PollReactor / EPollReactor override these; kaliru's reactor is klippy.reactor.Reactor = PollReactor)
    r.method("klippy.reactor.SelectReactor.register_fd", register_fd);
    method!(r, "klippy.reactor.SelectReactor", Reactor, unregister_fd(re, h: Rc<crate::reactor::ReactorFileHandler>) => re.unregister_fd(&h));
    method!(r, "klippy.reactor.SelectReactor", Reactor, set_fd_wake(re, h: Rc<crate::reactor::ReactorFileHandler>, readable: bool, writeable: bool) =>
        re.set_fd_wake(&h, readable, writeable));
    r.method("klippy.reactor.PollReactor.register_fd", register_fd);
    method!(r, "klippy.reactor.PollReactor", Reactor, unregister_fd(re, h: Rc<crate::reactor::ReactorFileHandler>) => re.unregister_fd(&h));
    method!(r, "klippy.reactor.PollReactor", Reactor, set_fd_wake(re, h: Rc<crate::reactor::ReactorFileHandler>, readable: bool, writeable: bool) =>
        re.set_fd_wake(&h, readable, writeable));
    r.method("klippy.reactor.EPollReactor.register_fd", register_fd);
    method!(r, "klippy.reactor.EPollReactor", Reactor, unregister_fd(re, h: Rc<crate::reactor::ReactorFileHandler>) => re.unregister_fd(&h));
    method!(r, "klippy.reactor.EPollReactor", Reactor, set_fd_wake(re, h: Rc<crate::reactor::ReactorFileHandler>, readable: bool, writeable: bool) =>
        re.set_fd_wake(&h, readable, writeable));
    method!(r, "klippy.reactor.SelectReactor", Reactor, end(re) => re.end());
    // self.monotonic = chelper...get_monotonic (a function attribute)
    r.attr("klippy.reactor.SelectReactor.monotonic", |o| {
        let re = objects::unwrap_as::<Reactor>(o)?;
        py_function(Rc::new(move |_args: &[PyRef]| Ok(py_float(re.monotonic()))))
    }, None);

    r.attr("klippy.reactor.ReactorTimer.waketime", |o| Ok(py_float(objects::unwrap_as::<ReactorTimer>(o)?.waketime.get())), None);

    // a completion python made carries python objects; kaliru's own (a
    // homing move's endstop triggers) a bool
    type BoolCompletion = ReactorCompletion<bool>;
    r.method("klippy.reactor.ReactorCompletion.test", |a| {
        let o = a.slf.as_ref().expect("method");
        if let Ok(c) = objects::unwrap_as::<BoolCompletion>(o) {
            return Ok(py_bool(c.test()));
        }
        Ok(py_bool(objects::unwrap_as::<PyCompletion>(o)?.test()))
    });
    r.method("klippy.reactor.ReactorCompletion.complete", |a| {
        let o = a.slf.as_ref().expect("method");
        if let Ok(c) = objects::unwrap_as::<BoolCompletion>(o) {
            c.complete(a.bool(0)?);
            return Ok(py_none());
        }
        objects::unwrap_as::<PyCompletion>(o)?.complete(a.arg(0).clone());
        Ok(py_none())
    });
    r.method("klippy.reactor.ReactorCompletion.wait", |a| {
        let o = a.slf.as_ref().expect("method");
        let waketime = a.f64(0)?;
        if let Ok(c) = objects::unwrap_as::<BoolCompletion>(o) {
            let wr = if a.arg(1).is_none() { None } else { Some(a.bool(1)?) };
            return match c.wait(waketime, wr).map_err(|e| KError::Reactor(e.0))? {
                Some(v) => Ok(py_bool(v)),
                None => Ok(py_none()),
            };
        }
        objects::unwrap_as::<PyCompletion>(o)?.wait(waketime, Some(a.arg(1).clone())).map_err(|e| KError::Reactor(e.0)).to_py()
    });

    method!(r, "klippy.reactor.ReactorMutex", ReactorMutex, test(m) => m.test());
    method!(r, "klippy.reactor.ReactorMutex", ReactorMutex, __enter__(m) => m.lock().map_err(|e| KError::Reactor(e.0)));
    method!(r, "klippy.reactor.ReactorMutex", ReactorMutex, __exit__(m, _t: PyRef, _v: PyRef, _tb: PyRef) => m.unlock());
    // self.lock = self.__enter__; self.unlock = self.__exit__
    r.attr("klippy.reactor.ReactorMutex.lock", |o| {
        let m = objects::unwrap_as::<ReactorMutex>(o)?;
        py_function(Rc::new(move |_a: &[PyRef]| m.lock().map_err(|e| KError::Reactor(e.0)).map(|_| py_none())))
    }, None);
    r.attr("klippy.reactor.ReactorMutex.unlock", |o| {
        let m = objects::unwrap_as::<ReactorMutex>(o)?;
        py_function(Rc::new(move |_a: &[PyRef]| {
            m.unlock();
            Ok(py_none())
        }))
    }, None);

    r.attr("klippy.reactor.ReactorFileHandler.fd", |o| Ok(py_int(objects::unwrap_as::<crate::reactor::ReactorFileHandler>(o)?.fd as i64)), None);
    // read_callback(eventtime) / write_callback(eventtime) or None
    fn fd_callback(cb: Rc<dyn Fn(f64)>) -> KResult<PyRef> {
        crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| {
            cb(a[0].as_f64()?);
            Ok(py_none())
        }))
    }
    r.attr("klippy.reactor.ReactorFileHandler.read_callback", |o| fd_callback(objects::unwrap_as::<crate::reactor::ReactorFileHandler>(o)?.read_callback.clone()), None);
    r.attr("klippy.reactor.ReactorFileHandler.write_callback", |o| match objects::unwrap_as::<crate::reactor::ReactorFileHandler>(o)?.write_callback.clone() {
        Some(cb) => fd_callback(cb),
        None => Ok(py_none()),
    }, None);
}

/// A python callable called with eventtime; a raise is reactor.run()'s
/// "Unhandled exception during run"
fn call_reporting(cb: &PyRef, eventtime: f64) -> Option<PyRef> {
    match cb.call(&[&py_float(eventtime)]) {
        Ok(v) => Some(v),
        Err(e) => {
            report_unhandled(&e);
            None
        }
    }
}

// register_timer(callback, waketime=NEVER)
fn register_timer(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let cb = a.arg(0).clone();
    let timer = reactor.register_timer(
        Box::new(move |eventtime| match call_reporting(&cb, eventtime).map(|r| r.as_f64()) {
            Some(Ok(w)) => w,
            Some(Err(e)) => {
                report_unhandled(&e);
                NEVER
            }
            // python leaves reactor.run(); the timer does not run again
            None => NEVER,
        }),
        a.f64(1)?,
    );
    timer.to_py()
}

// register_callback(callback, waketime=NOW) -> completion of its result
fn register_callback(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let cb = a.arg(0).clone();
    let completion: Rc<PyCompletion> = reactor.register_callback(
        move |eventtime| call_reporting(&cb, eventtime).unwrap_or_else(py_none),
        a.f64(1)?,
    );
    completion.to_py()
}

/// A python object carried to the reactor thread (it runs there, holding
/// the GIL: docs/PLUGINS.md "GIL and threads")
struct ToReactor(PyRef);
// SAFETY: created and dropped with the GIL held; used on the reactor thread
unsafe impl Send for ToReactor {}

// register_async_callback(callback, waketime=NOW): from any thread
fn register_async_callback(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let cb = ToReactor(a.arg(0).clone());
    reactor.register_async_callback(
        Box::new(move |eventtime| {
            let cb = cb;
            call_reporting(&cb.0, eventtime);
        }),
        a.f64(1)?,
    );
    Ok(py_none())
}

// register_fd(fd, read_callback, write_callback=None)
fn register_fd(a: &Args) -> KResult<PyRef> {
    let reactor = a.this::<Reactor>()?;
    let fd = a.arg(0).as_i64()? as std::os::fd::RawFd;
    let read = a.arg(1).clone();
    let write = if a.arg(2).is_none() { None } else { Some(a.arg(2).clone()) };
    let h: FileHandler = reactor.register_fd(
        fd,
        Box::new(move |et| {
            call_reporting(&read, et);
        }),
        write.map(|w| -> Box<dyn Fn(f64)> {
            Box::new(move |et| {
                call_reporting(&w, et);
            })
        }),
    );
    h.to_py()
}
