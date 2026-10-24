// Kaliru objects seen from python: each Rust object handed to a plugin gets
// one python object of its mirrored class (so `is` holds), created without
// running __init__ and carrying a handle (`_k_h`) back to the Rust object.
// Objects a plugin constructs (`ConfigWrapper(...)`, a subclass of a
// mirrored class) are bound to their Rust object by _k.new.
//
// The table holds both sides until teardown() (the end of a printer
// instance, as python klippy drops all printer objects on RESTART).
use super::py::{lookup, py_int, PyRef};
use crate::error::{KError, KResult};
use crate::printer::PObj;
use std::any::{Any, TypeId};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

struct Entry {
    rust: Rc<dyn Any>,
    py: PyRef,
    // the same object as a printer object, when it is one
    pobj: Option<PObj>,
}

#[derive(Default)]
struct Table {
    entries: Vec<Option<Entry>>,
    // Rust object address -> handle
    by_addr: HashMap<usize, usize>,
    // Rust type -> mirrored class (module, class name)
    classes: HashMap<TypeId, (&'static str, &'static str)>,
}

thread_local! {
    static TABLE: RefCell<Table> = RefCell::new(Table::default());
}

fn addr(rust: &Rc<dyn Any>) -> usize {
    Rc::as_ptr(rust) as *const () as usize
}

/// Register the mirrored class of a Rust type ("klippy.gcode", "GCodeDispatch")
pub fn register_class<T: 'static>(module: &'static str, class: &'static str) {
    TABLE.with(|t| t.borrow_mut().classes.insert(TypeId::of::<T>(), (module, class)));
}

/// The python object of a Rust object (created on first use)
pub fn wrap(rust: Rc<dyn Any>) -> KResult<PyRef> {
    wrap_impl(rust, None)
}

/// wrap() of a printer object (keeps it usable as one when python hands it back)
pub fn wrap_pobj(rust: Rc<dyn Any>, pobj: PObj) -> KResult<PyRef> {
    wrap_impl(rust, Some(pobj))
}

fn wrap_impl(rust: Rc<dyn Any>, pobj: Option<PObj>) -> KResult<PyRef> {
    if let Some(py) = TABLE.with(|t| {
        let t = t.borrow();
        t.by_addr.get(&addr(&rust)).and_then(|h| t.entries[*h].as_ref().map(|e| e.py.clone()))
    }) {
        return Ok(py);
    }
    let tid = (*rust).type_id();
    let Some((module, class)) = TABLE.with(|t| t.borrow().classes.get(&tid).copied()) else {
        return Err(KError::Py(
            "TypeError".into(),
            format!("kaliru: this object has no python class yet ({:?})", tid),
        ));
    };
    let cls = lookup(module, class)?;
    // cls.__new__(cls): the object exists already, __init__ must not run
    let py = cls.getattr("__new__")?.call(&[&cls])?;
    bind_impl(&py, rust, pobj)?;
    Ok(py)
}

/// Make `py` the python object of `rust` (_k.new)
pub fn bind(py: &PyRef, rust: Rc<dyn Any>) -> KResult<()> {
    bind_impl(py, rust, None)
}

fn bind_impl(py: &PyRef, rust: Rc<dyn Any>, pobj: Option<PObj>) -> KResult<()> {
    let handle = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        let h = t.entries.len();
        t.by_addr.insert(addr(&rust), h);
        t.entries.push(Some(Entry { rust, py: py.clone(), pobj }));
        h
    });
    // object.__setattr__: Object.__setattr__ would report a patch
    lookup("builtins", "object")?.getattr("__setattr__")?.call(&[py, &super::py::py_str("_k_h"), &py_int(handle as i64)])?;
    Ok(())
}

/// The Rust object behind a python object of a mirrored class
pub fn unwrap(py: &PyRef) -> KResult<Rc<dyn Any>> {
    let h = match py.getattr("_k_h") {
        Ok(h) => h.as_i64()? as usize,
        Err(_) => {
            return Err(KError::Py(
                "TypeError".into(),
                "kaliru: the object is not bound to a kaliru object (its __init__ did not run)".into(),
            ))
        }
    };
    TABLE.with(|t| t.borrow().entries.get(h).and_then(|e| e.as_ref().map(|e| e.rust.clone())))
        .ok_or_else(|| KError::Py("ReferenceError".into(), "kaliru: the object no longer exists".into()))
}

/// The Rust object of a given type behind a python object
pub fn unwrap_as<T: 'static>(py: &PyRef) -> KResult<Rc<T>> {
    unwrap(py)?.downcast::<T>().map_err(|_| {
        KError::Py("TypeError".into(), format!("kaliru: expected a {}", std::any::type_name::<T>()))
    })
}

/// The printer object behind a python object, if it is a kaliru one
pub fn bound_pobj(py: &PyRef) -> Option<PObj> {
    let h = py.getattr("_k_h").ok()?.as_i64().ok()? as usize;
    TABLE.with(|t| t.borrow().entries.get(h).and_then(|e| e.as_ref().and_then(|e| e.pobj.clone())))
}

/// The python object already bound to a Rust object, if any
pub fn existing(rust: &Rc<dyn Any>) -> Option<PyRef> {
    TABLE.with(|t| {
        let t = t.borrow();
        t.by_addr.get(&addr(rust)).and_then(|h| t.entries[*h].as_ref().map(|e| e.py.clone()))
    })
}

/// Drop every binding (end of a printer instance)
pub fn teardown() {
    let entries = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        t.by_addr.clear();
        std::mem::take(&mut t.entries)
    });
    drop(entries);
}
