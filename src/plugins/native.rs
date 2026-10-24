// The native module `_kaliru` the mirror modules call (prelude.py binds it as
// _k.call / _k.fcall / _k.new ...), and the registry of what each mirrored
// method, constructor and attribute does in kaliru.
//
// Handlers are registered by Kalico qualified name ("klippy.gcode.
// GCodeDispatch.register_command"); mirror_ids.rs gives each name the ID the
// generated python passes. An ID without a handler raises
// NotImplementedError naming it.
use super::capi::*;
use super::mirror_ids::{IDS, MODULES};
use super::objects;
use super::py::*;
use crate::error::{KError, KResult};
use std::any::Any;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::ffi::c_char;
use std::rc::Rc;

/// The arguments of a mirrored call: the python object (methods) and the
/// arguments in the order of Kalico's signature (defaults filled in)
pub struct Args {
    pub slf: Option<PyRef>,
    pub args: Vec<PyRef>,
}

impl Args {
    pub fn arg(&self, i: usize) -> &PyRef {
        &self.args[i]
    }

    pub fn this<T: 'static>(&self) -> KResult<Rc<T>> {
        objects::unwrap_as::<T>(self.slf.as_ref().expect("method without self"))
    }

    pub fn f64(&self, i: usize) -> KResult<f64> {
        self.args[i].as_f64()
    }

    pub fn opt_f64(&self, i: usize) -> KResult<Option<f64>> {
        if self.args[i].is_none() { Ok(None) } else { self.f64(i).map(Some) }
    }

    pub fn string(&self, i: usize) -> KResult<String> {
        self.args[i].as_string()
    }

    pub fn opt_string(&self, i: usize) -> KResult<Option<String>> {
        if self.args[i].is_none() { Ok(None) } else { self.string(i).map(Some) }
    }

    pub fn bool(&self, i: usize) -> KResult<bool> {
        self.args[i].is_true()
    }
}

pub type Method = fn(&Args) -> KResult<PyRef>;
pub type Ctor = fn(&Args) -> KResult<Rc<dyn Any>>;
pub type Getter = fn(&PyRef) -> KResult<PyRef>;
pub type Setter = fn(&PyRef, &PyRef) -> KResult<()>;

#[derive(Clone, Copy)]
pub enum Handler {
    Method(Method),
    Ctor(Ctor),
    Attr(Getter, Option<Setter>),
}

/// Collects handlers by qualified name (src/plugins/impls/*)
#[derive(Default)]
pub struct Registry(HashMap<&'static str, Handler>);

impl Registry {
    pub fn method(&mut self, name: &'static str, f: Method) {
        self.0.insert(name, Handler::Method(f));
    }
    pub fn ctor(&mut self, name: &'static str, f: Ctor) {
        self.0.insert(name, Handler::Ctor(f));
    }
    pub fn attr(&mut self, name: &'static str, get: Getter, set: Option<Setter>) {
        self.0.insert(name, Handler::Attr(get, set));
    }
}

thread_local! {
    static HANDLERS: RefCell<Vec<Option<Handler>>> = const { RefCell::new(Vec::new()) };
    // (class qualname or "@<object handle>", name) a plugin replaced
    static PATCHED: RefCell<HashSet<(String, String)>> = RefCell::new(HashSet::new());
}

/// Resolve the registry against the generated IDs; names that are not
/// Kalico's are a bug in the impls
pub fn install_handlers(reg: Registry) {
    let mut by_id = vec![None; IDS.len()];
    let mut unknown: Vec<&str> = vec![];
    let index: HashMap<&str, usize> = IDS.iter().enumerate().map(|(i, n)| (*n, i)).collect();
    for (name, h) in reg.0 {
        match index.get(name) {
            Some(&i) => by_id[i] = Some(h),
            None => unknown.push(name),
        }
    }
    assert!(unknown.is_empty(), "plugin handlers for names Kalico does not have: {:?}", unknown);
    HANDLERS.with(|h| *h.borrow_mut() = by_id);
}

fn handler(id: usize) -> KResult<Handler> {
    HANDLERS
        .with(|h| h.borrow().get(id).copied().flatten())
        .ok_or_else(|| KError::Py("NotImplementedError".into(), format!("kaliru: {} is not mirrored yet", IDS.get(id).unwrap_or(&"?"))))
}

/// Whether a plugin replaced `name` on this object's class (or a subclass)
/// or on the object (hook points consult this; docs/PLUGINS.md)
pub fn is_patched(class_qualname: &str, name: &str) -> bool {
    PATCHED.with(|p| p.borrow().contains(&(class_qualname.to_string(), name.to_string())))
}

// ------------------------------------------------------------------ module

unsafe fn args_vec(args: *const *mut PyObject, nargs: Py_ssize_t, skip: usize) -> Vec<PyRef> {
    let n = (nargs as usize) & !(1usize << (usize::BITS - 1)); // PY_VECTORCALL_ARGUMENTS_OFFSET
    (skip..n).map(|i| PyRef::borrowed(*args.add(i))).collect()
}

fn result(r: KResult<PyRef>) -> *mut PyObject {
    match r {
        Ok(v) => v.into_ptr(),
        Err(e) => {
            raise(&e);
            std::ptr::null_mut()
        }
    }
}

macro_rules! native {
    ($name:ident, |$a:ident| $body:expr) => {
        unsafe extern "C" fn $name(
            _m: *mut PyObject,
            args: *const *mut PyObject,
            nargs: Py_ssize_t,
            _kw: *mut PyObject,
        ) -> *mut PyObject {
            let $a = args_vec(args, nargs, 0);
            result((|| -> KResult<PyRef> { $body })())
        }
    };
}

fn id_of(o: &PyRef) -> KResult<usize> {
    Ok(o.as_i64()? as usize)
}

// call(obj, id, *args)
native!(k_call, |a| {
    let h = handler(id_of(&a[1])?)?;
    let Handler::Method(f) = h else { return Err(KError::Other("kaliru: not a method".into())) };
    f(&Args { slf: Some(a[0].clone()), args: a[2..].to_vec() })
});

// fcall(id, *args)
native!(k_fcall, |a| {
    let Handler::Method(f) = handler(id_of(&a[0])?)? else { return Err(KError::Other("kaliru: not a function".into())) };
    f(&Args { slf: None, args: a[1..].to_vec() })
});

// new(obj, id, *args): obj.__init__
native!(k_new, |a| {
    let Handler::Ctor(f) = handler(id_of(&a[1])?)? else { return Err(KError::Other("kaliru: not a constructor".into())) };
    let rust = f(&Args { slf: Some(a[0].clone()), args: a[2..].to_vec() })?;
    objects::bind(&a[0], rust)?;
    Ok(py_none())
});

// get(obj, id)
native!(k_get, |a| {
    let Handler::Attr(get, _) = handler(id_of(&a[1])?)? else { return Err(KError::Other("kaliru: not an attribute".into())) };
    get(&a[0])
});

// set(obj, id, value)
native!(k_set, |a| {
    let id = id_of(&a[1])?;
    let Handler::Attr(_, set) = handler(id)? else { return Err(KError::Other("kaliru: not an attribute".into())) };
    let Some(set) = set else {
        return Err(KError::Py("NotImplementedError".into(), format!("kaliru: setting {} is not mirrored yet", IDS[id])));
    };
    set(&a[0], &a[2])?;
    Ok(py_none())
});

native!(k_delete, |a| {
    Err(KError::Py("NotImplementedError".into(), format!("kaliru: deleting {} is not mirrored", IDS[id_of(&a[1])?])))
});

fn qualname(cls: &PyRef) -> KResult<String> {
    Ok(format!("{}.{}", cls.getattr("__module__")?.as_string()?, cls.getattr("__qualname__")?.as_string()?))
}

// patched_class(cls, name): a mirrored class's attribute set, or a subclass's
native!(k_patched_class, |a| {
    let (q, name) = (qualname(&a[0])?, a[1].as_string()?);
    PATCHED.with(|p| p.borrow_mut().insert((q, name)));
    Ok(py_none())
});

// patched_instance(obj, name)
native!(k_patched_instance, |a| {
    if let Ok(h) = a[0].getattr("_k_h") {
        let (key, name) = (format!("@{}", h.as_i64()?), a[1].as_string()?);
        PATCHED.with(|p| p.borrow_mut().insert((key, name)));
    }
    Ok(py_none())
});

// module_value(module, name): module names not copied into a mirror
native!(k_module_value, |a| {
    let (m, n) = (a[0].as_string()?, a[1].as_string()?);
    Err(KError::Py("AttributeError".into(), format!("module '{}' has no attribute '{}'", m, n)))
});

// source(module): the generated mirror source
native!(k_source, |a| {
    let name = a[0].as_string()?;
    let src = super::sources::source(&name)
        .ok_or_else(|| KError::Py("ImportError".into(), format!("kaliru: no mirror of {}", name)))?;
    Ok(py_str(&src))
});

// log(levelno, text): python logging records into kaliru's log
native!(k_log, |a| {
    crate::logging::log(a[0].as_i64()? as i32, &a[1].as_string()?);
    Ok(py_none())
});

const FLAGS: std::ffi::c_int = METH_FASTCALL | METH_KEYWORDS;

macro_rules! def {
    ($name:literal, $f:ident) => {
        PyMethodDef { ml_name: concat!($name, "\0").as_ptr() as *const c_char, ml_meth: $f, ml_flags: FLAGS, ml_doc: std::ptr::null() }
    };
}

static DEFS: [PyMethodDef; 11] = [
    def!("log", k_log),
    def!("call", k_call),
    def!("fcall", k_fcall),
    def!("new", k_new),
    def!("get", k_get),
    def!("set", k_set),
    def!("delete", k_delete),
    def!("patched_class", k_patched_class),
    def!("patched_instance", k_patched_instance),
    def!("module_value", k_module_value),
    def!("source", k_source),
];

/// Create `_kaliru` in sys.modules
pub fn create_module() -> KResult<()> {
    unsafe {
        let m = PyRef::owned_or_err(PyImport_AddModuleRef(c"_kaliru".as_ptr()))?;
        for d in DEFS.iter() {
            let f = PyRef::owned_or_err(PyCMethod_New(d, std::ptr::null_mut(), m.as_ptr(), std::ptr::null_mut()))?;
            if PyModule_AddObjectRef(m.as_ptr(), d.ml_name, f.as_ptr()) != 0 {
                return Err(fetch_error());
            }
        }
    }
    Ok(())
}

/// (module name, is package) of every mirror module, as the prelude's finder wants
pub fn mirror_modules() -> PyRef {
    py_list(MODULES.iter().map(|(n, p)| py_tuple(vec![py_str(n), py_bool(*p)])).collect())
}
