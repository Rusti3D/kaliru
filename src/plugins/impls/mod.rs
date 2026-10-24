// What each mirrored Kalico method, constructor and attribute does in
// kaliru, one file per klippy module. Registered by qualified name
// (native::Registry); argument order is Kalico's signature with its
// defaults filled in by the generated python.
mod configfile;
mod gcode;
mod heaters;
mod pins;
mod printer;
mod reactor;
mod webhooks;

use crate::plugins::gil::Gil;
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
    heaters::register(r);
    pins::register(r);
    reactor::register(r);
    webhooks::register(r);
}

/// get_status / stats / get_temp of any printer object (kaliru's
/// PrinterObject methods), for every Kalico class that has them
pub fn generic_methods(name: &str) -> Option<super::native::Method> {
    if name.ends_with(".get_status") {
        Some(|a| {
            let o = pobj_of(a)?;
            match o.try_get_status(a.f64(0)?)? {
                Some(v) => Ok(from_pyval(&v)),
                None => Err(KError::Py("AttributeError".into(), "get_status".into())),
            }
        })
    } else if name.ends_with(".stats") {
        Some(|a| {
            let o = pobj_of(a)?;
            match o.stats(a.f64(0)?) {
                Some((busy, msg)) => Ok(py_tuple(vec![py_bool(busy), py_str(&msg)])),
                None => Err(KError::Py("AttributeError".into(), "stats".into())),
            }
        })
    } else if name.ends_with(".get_temp") {
        Some(|a| {
            let o = pobj_of(a)?;
            match o.get_temp(a.f64(0)?) {
                Some((cur, target)) => Ok(py_tuple(vec![py_float(cur), py_float(target)])),
                None => Err(KError::Py("AttributeError".into(), "get_temp".into())),
            }
        })
    } else {
        None
    }
}

fn pobj_of(a: &super::native::Args) -> KResult<PObj> {
    let slf = a.slf.as_ref().expect("method without self");
    objects::bound_pobj(slf).ok_or_else(|| KError::Py("NotImplementedError".into(), "kaliru: not a printer object".into()))
}

// the printer of the running instance (callbacks report through it)
static PRINTER: Gil<RefCell<Weak<Printer>>> = Gil::new(|| RefCell::new(Weak::new()));

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
    if let Some(v) = a.downcast_ref::<PyRef>() {
        return Ok(v.clone());
    }
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
