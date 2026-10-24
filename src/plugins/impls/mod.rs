// What each mirrored Kalico method, constructor and attribute does in
// kaliru, one file per klippy module. Registered by qualified name
// (native::Registry); argument order is Kalico's signature with its
// defaults filled in by the generated python.
mod configfile;
mod gcode;
mod printer;
mod reactor;

use super::native::Registry;
use super::py::*;
use super::{objects, pobj_to_py};
use crate::configfile::{Dflt, Lim};
use crate::error::{KError, KResult};
use crate::printer::{PObj, Printer};
use crate::pyval::PyVal;
use std::any::Any;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

pub fn register(r: &mut Registry) {
    printer::register(r);
    configfile::register(r);
    gcode::register(r);
    reactor::register(r);
}

thread_local! {
    // the printer of the running instance (callbacks report through it)
    static PRINTER: RefCell<Weak<Printer>> = const { RefCell::new(Weak::new()) };
}

pub fn set_printer(p: &Rc<Printer>) {
    PRINTER.with(|c| *c.borrow_mut() = Rc::downgrade(p));
}

pub fn printer() -> Option<Rc<Printer>> {
    PRINTER.with(|c| c.borrow().upgrade())
}

/// A python callback failed where Kalico would leave reactor.run()
pub fn report_unhandled(e: &KError) {
    if let Some(p) = printer() {
        p.unhandled_exception(e);
    }
}

/// `default=sentinel` handling of the config / gcode getters: Required, or
/// the default as kaliru's Dflt plus the python object to return as is
pub fn dflt(default: &PyRef, sentinel: &PyRef) -> KResult<(Dflt, Option<PyRef>)> {
    if default.is(sentinel.as_ptr()) {
        return Ok((Dflt::Required, None));
    }
    if default.is_none() {
        return Ok((Dflt::None, Some(default.clone())));
    }
    Ok((Dflt::Val(to_pyval(default)?), Some(default.clone())))
}

/// minval / maxval / above / below as kaliru's Lim (None: no bound)
pub fn lim(bounds: [&PyRef; 4], note_valid: bool) -> KResult<Lim> {
    let v = |o: &PyRef| -> KResult<Option<PyVal>> { if o.is_none() { Ok(None) } else { to_pyval(o).map(Some) } };
    Ok(Lim { minval: v(bounds[0])?, maxval: v(bounds[1])?, above: v(bounds[2])?, below: v(bounds[3])?, note_valid })
}

/// A value from a kaliru getter, or the python default it fell back to
pub fn value_or_default(v: Option<PyVal>, missing: bool, default: Option<PyRef>) -> PyRef {
    if missing {
        if let Some(d) = default {
            return d;
        }
    }
    v.map(|v| from_pyval(&v)).unwrap_or_else(py_none)
}

/// An event / callback argument as python sees it
pub fn any_to_py(a: &dyn Any) -> KResult<PyRef> {
    if let Some(v) = a.downcast_ref::<f64>() {
        return Ok(py_float(*v));
    }
    if let Some(v) = a.downcast_ref::<i64>() {
        return Ok(py_int(*v));
    }
    if let Some(v) = a.downcast_ref::<bool>() {
        return Ok(py_bool(*v));
    }
    if let Some(v) = a.downcast_ref::<String>() {
        return Ok(py_str(v));
    }
    if let Some(v) = a.downcast_ref::<&str>() {
        return Ok(py_str(v));
    }
    if let Some(v) = a.downcast_ref::<PyVal>() {
        return Ok(from_pyval(v));
    }
    if let Some(v) = a.downcast_ref::<PObj>() {
        return pobj_to_py(v);
    }
    Err(KError::Py("TypeError".into(), format!("kaliru: event argument of type {:?} is not mirrored yet", a.type_id())))
}

/// The python object of a kaliru object that is not a printer object
pub fn wrap<T: 'static>(rc: Rc<T>) -> KResult<PyRef> {
    objects::wrap(rc)
}
