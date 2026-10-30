// Kaliru objects seen from python: each Rust object handed to a plugin gets
// one python object of its mirrored class (so `is` holds), created without
// running __init__ and carrying a handle (`_k_h`) back to the Rust object.
// Objects a plugin constructs (`ConfigWrapper(...)`, a subclass of a
// mirrored class) are bound to their Rust object by _k.new.
//
// The table holds both sides while either can still be reached: a sweep
// (each time the table doubles) releases the pairs only the table holds
// (per-call objects: a GCodeCommand, a WebRequest), and teardown() (the end
// of a printer instance) releases everything.
use crate::plugins::gil::Gil;
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
    // the same object as trait objects (Rc<dyn TempSensor>, ...): a trait
    // object cannot be recovered from Rc<dyn Any>
    views: Vec<Box<dyn Any>>,
}

#[derive(Default)]
struct Table {
    // handle -> binding; handles are never reused
    entries: HashMap<usize, Entry>,
    next_handle: usize,
    // the size at which the next sweep runs
    sweep_at: usize,
    // Rust object address -> handle
    by_addr: HashMap<usize, usize>,
    // Rust type -> mirrored class (module, class name)
    classes: HashMap<TypeId, (&'static str, &'static str)>,
    // Rust type -> its mirrored class chosen per object (a Heater with a
    // secondary sensor is a DualSensorHeater)
    class_fns: HashMap<TypeId, fn(&dyn Any) -> (&'static str, &'static str)>,
    // printer object types that also travel as plain Rc (a heater as
    // extruder.heater): how to see one as a printer object
    pobj_casts: HashMap<TypeId, fn(Rc<dyn Any>) -> Option<PObj>>,
    // Rust type -> its trait object views (each a Box<Rc<dyn Trait>>), for
    // objects no view was stored with (a plugin constructed them)
    view_casts: HashMap<TypeId, Vec<fn(Rc<dyn Any>) -> Box<dyn Any>>>,
    // Rust type -> the object python sees in its place (kaliru's stand-in
    // for a Kalico object it cannot hold directly, e.g. a weak handle)
    redirects: HashMap<TypeId, fn(&Rc<dyn Any>) -> Option<(Rc<dyn Any>, &'static str)>>,
}

static TABLE: Gil<RefCell<Table>> = Gil::new(|| RefCell::new(Table::default()));

fn addr(rust: &Rc<dyn Any>) -> usize {
    Rc::as_ptr(rust) as *const () as usize
}

/// Register the mirrored class of a Rust type whose path does not name it
/// ("klippy.mcu", "MCU" for kaliru::mcu::Mcu); others are found by path
pub fn register_class<T: 'static>(module: &'static str, class: &'static str) {
    TABLE.with(|t| t.borrow_mut().classes.insert(TypeId::of::<T>(), (module, class)));
}

/// Register the mirrored class of a Rust type that depends on the object
pub fn register_class_fn<T: 'static>(f: fn(&dyn Any) -> (&'static str, &'static str)) {
    TABLE.with(|t| t.borrow_mut().class_fns.insert(TypeId::of::<T>(), f));
}

/// A printer object type that kaliru also hands out as a plain Rc<T>: its
/// python object stays usable where kaliru wants the printer object
pub fn register_pobj<T: crate::printer::PrinterObject + 'static>() {
    fn cast<T: crate::printer::PrinterObject + 'static>(a: Rc<dyn Any>) -> Option<PObj> {
        a.downcast::<T>().ok().map(|t| t as PObj)
    }
    TABLE.with(|t| t.borrow_mut().pobj_casts.insert(TypeId::of::<T>(), cast::<T>));
}

/// How to see a Rust type as a trait object (`Box::new(rc as Rc<dyn Trait>)`
/// of the downcast object): unwrap_view() of objects a plugin constructed
pub fn register_view<T: 'static>(f: fn(Rc<dyn Any>) -> Box<dyn Any>) {
    TABLE.with(|t| t.borrow_mut().view_casts.entry(TypeId::of::<T>()).or_default().push(f));
}

/// The Kalico class of a Rust type: registered, else by its path
/// (kaliru::extras::heaters::Heater is klippy.extras.heaters.Heater)
fn class_of(rust: &Rc<dyn Any>, type_name: &str) -> KResult<PyRef> {
    let tid = (**rust).type_id();
    if let Some(f) = TABLE.with(|t| t.borrow().class_fns.get(&tid).copied()) {
        let (module, class) = f(&**rust);
        return lookup(module, class);
    }
    if let Some((module, class)) = TABLE.with(|t| t.borrow().classes.get(&tid).copied()) {
        return lookup(module, class);
    }
    let path = type_name.split('<').next().unwrap_or(type_name);
    let mut parts: Vec<&str> = path.split("::").collect();
    let class = parts.pop().unwrap_or("");
    if parts.first() == Some(&"kaliru") && parts.len() > 1 {
        parts[0] = "klippy";
        if let Ok(c) = lookup(&parts.join("."), class) {
            return Ok(c);
        }
    }
    Err(KError::Py(
        "TypeError".into(),
        format!("kaliru: {} has no Kalico class yet (plugins/objects.rs register_class)", type_name),
    ))
}

/// The python object of a Rust object (created on first use)
pub fn wrap<T: 'static>(rust: Rc<T>) -> KResult<PyRef> {
    wrap_impl(rust, std::any::type_name::<T>(), None)
}

/// wrap() of an object known by its Rust type's path only
pub fn wrap_any(rust: Rc<dyn Any>, type_name: &str) -> KResult<PyRef> {
    wrap_impl(rust, type_name, None)
}

/// wrap() of a printer object (keeps it usable as one when python hands it back)
pub fn wrap_pobj(pobj: PObj) -> KResult<PyRef> {
    let type_name = pobj.type_name();
    let rust: Rc<dyn Any> = pobj.clone();
    wrap_impl(rust, type_name, Some(pobj))
}

/// Python sees `f(obj)` (object, Rust type path) wherever kaliru hands it a T
pub fn register_redirect<T: 'static>(f: fn(&Rc<dyn Any>) -> Option<(Rc<dyn Any>, &'static str)>) {
    TABLE.with(|t| t.borrow_mut().redirects.insert(TypeId::of::<T>(), f));
}

fn wrap_impl(rust: Rc<dyn Any>, type_name: &str, pobj: Option<PObj>) -> KResult<PyRef> {
    if let Some(f) = TABLE.with(|t| t.borrow().redirects.get(&(*rust).type_id()).copied()) {
        if let Some((target, name)) = f(&rust) {
            return wrap_impl(target, name, None);
        }
    }
    let pobj = pobj.or_else(|| {
        let cast = TABLE.with(|t| t.borrow().pobj_casts.get(&(*rust).type_id()).copied());
        cast.and_then(|c| c(rust.clone()))
    });
    if let Some(py) = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        let h = *t.by_addr.get(&addr(&rust))?;
        let e = t.entries.get_mut(&h)?;
        // first met as a plain object, now known as a printer object too
        if e.pobj.is_none() {
            e.pobj = pobj.clone();
        }
        Some(e.py.clone())
    }) {
        return Ok(py);
    }
    let cls = class_of(&rust, type_name)?;
    // cls.__new__(cls): the object exists already, __init__ must not run
    let py = cls.getattr("__new__")?.call(&[&cls])?;
    bind_impl(&py, rust, pobj)?;
    Ok(py)
}

/// wrap() of a trait object (`Rc<dyn TempSensor>`): `rust` is the same
/// object as Rc<dyn Any>, `type_name` its concrete type's path; the view
/// comes back from unwrap_view()
pub fn wrap_view<D: ?Sized + 'static>(rust: Rc<dyn Any>, view: Rc<D>, type_name: &str) -> KResult<PyRef> {
    let a = addr(&rust);
    let py = wrap_impl(rust, type_name, None)?;
    add_view(a, view);
    Ok(py)
}

fn add_view<D: ?Sized + 'static>(a: usize, view: Rc<D>) {
    TABLE.with(|t| {
        let mut t = t.borrow_mut();
        let Some(h) = t.by_addr.get(&a).copied() else { return };
        if let Some(e) = t.entries.get_mut(&h) {
            if !e.views.iter().any(|v| v.is::<Rc<D>>()) {
                e.views.push(Box::new(view));
            }
        }
    });
}

/// The trait object view of a python object's kaliru object, if it has one
pub fn unwrap_view<D: ?Sized + 'static>(py: &PyRef) -> Option<Rc<D>> {
    let h = py.getattr("_k_h").ok()?.as_i64().ok()? as usize;
    let (stored, rust) = TABLE.with(|t| {
        let t = t.borrow();
        let e = t.entries.get(&h)?;
        Some((e.views.iter().find_map(|v| v.downcast_ref::<Rc<D>>().cloned()), e.rust.clone()))
    })?;
    if stored.is_some() {
        return stored;
    }
    let casts = TABLE.with(|t| t.borrow().view_casts.get(&(*rust).type_id()).cloned())?;
    casts.into_iter().find_map(|c| c(rust.clone()).downcast::<Rc<D>>().ok().map(|b| *b))
}

/// Make `py` the python object of `rust` (_k.new)
pub fn bind(py: &PyRef, rust: Rc<dyn Any>) -> KResult<()> {
    // (a printer object type: usable as one where kaliru wants it)
    let cast = TABLE.with(|t| t.borrow().pobj_casts.get(&(*rust).type_id()).copied());
    let pobj = cast.and_then(|c| c(rust.clone()));
    bind_impl(py, rust, pobj)
}

fn bind_impl(py: &PyRef, rust: Rc<dyn Any>, pobj: Option<PObj>) -> KResult<()> {
    let (handle, sweep) = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        let h = t.next_handle;
        t.next_handle += 1;
        t.by_addr.insert(addr(&rust), h);
        t.entries.insert(h, Entry { rust, py: py.clone(), pobj, views: vec![] });
        (h, t.entries.len() >= t.sweep_at.max(64))
    });
    if sweep {
        sweep_unreachable()?;
    }
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
    TABLE.with(|t| t.borrow().entries.get(&h).map(|e| e.rust.clone()))
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
    TABLE.with(|t| t.borrow().entries.get(&h).and_then(|e| e.pobj.clone()))
}

/// The python object bound to the kaliru object at this address, if any
pub fn existing_addr(addr: usize) -> Option<PyRef> {
    TABLE.with(|t| {
        let t = t.borrow();
        t.by_addr.get(&addr).and_then(|h| t.entries.get(h).map(|e| e.py.clone()))
    })
}

/// The python object already bound to a Rust object, if any
pub fn existing(rust: &Rc<dyn Any>) -> Option<PyRef> {
    TABLE.with(|t| {
        let t = t.borrow();
        t.by_addr.get(&addr(rust)).and_then(|h| t.entries.get(h).map(|e| e.py.clone()))
    })
}

/// Release the bindings nothing else holds: the Rust object's only owner
/// is the table (its Rc, its printer object and trait object views) and the python
/// object's only reference is the table's (sys.getrefcount against that of
/// a fresh object held the same way). Neither side can be reached again, so
/// nothing can tell; the next sweep runs when the table has doubled.
fn sweep_unreachable() -> KResult<()> {
    let getrefcount = lookup("sys", "getrefcount")?;
    let fresh = lookup("builtins", "object")?.call(&[])?;
    let baseline = getrefcount.call(&[&fresh])?.as_i64()?;
    let candidates: Vec<(usize, PyRef)> = TABLE.with(|t| {
        t.borrow()
            .entries
            .iter()
            .filter(|(_, e)| Rc::strong_count(&e.rust) == 1 + e.pobj.is_some() as usize + e.views.len())
            .map(|(h, e)| (*h, e.py.clone()))
            .collect()
    });
    let mut dead = vec![];
    for (h, py) in candidates {
        // (`py` here is one more reference than the table's)
        if getrefcount.call(&[&py])?.as_i64()? == baseline + 1 {
            dead.push(h);
        }
    }
    let released: Vec<Entry> = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        let out: Vec<Entry> = dead.iter().filter_map(|h| t.entries.remove(h)).collect();
        for e in &out {
            t.by_addr.remove(&addr(&e.rust));
        }
        t.sweep_at = 2 * t.entries.len();
        out
    });
    super::native::forget_instances(&dead);
    // (dropped outside the table: a python __del__ may call back in)
    drop(released);
    Ok(())
}

/// Number of bindings (tests)
pub fn table_len() -> usize {
    TABLE.with(|t| t.borrow().entries.len())
}

/// Drop every binding (end of a printer instance)
pub fn teardown() {
    let entries = TABLE.with(|t| {
        let mut t = t.borrow_mut();
        t.by_addr.clear();
        t.sweep_at = 0;
        std::mem::take(&mut t.entries)
    });
    drop(entries);
}
