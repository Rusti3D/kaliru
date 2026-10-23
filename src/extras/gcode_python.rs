// Python g-code macros: TemplateWrapperPython of
// klippy/extras/gcode_macro.py, run on an embedded RustPython interpreter
// (cargo feature "python").
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// GetStatusWrapperPython and TemplateVariableWrapperPython are the python
// classes themselves (PRELUDE), backed by native functions.
use crate::error::{KError, KResult};
use crate::extras::gcode_macro::{kerror_exc, Action, GCodeMacro};
use crate::gcode::GCodeDispatch;
use crate::jinja::value::{Exc, Key, Value};
use crate::logging;
use crate::printer::{downcast_rc, Printer};
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use rustpython_vm::builtins::{PyBaseException, PyBaseExceptionRef, PyCode, PyDict, PyDictRef, PyFloat, PyInt, PyList, PyStr, PyTuple};
use rustpython_vm::compiler::Mode;
use rustpython_vm::function::{ArgMapping, FuncArgs};
use rustpython_vm::scope::Scope;
use rustpython_vm::{AsObject, Interpreter, PyObjectRef, PyRef, PyResult, VirtualMachine};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const PRELUDE: &str = r#"
import builtins

class _ConfigSentinel:
    pass

class GetStatusWrapperPython:
    def __getitem__(self, val):
        sval = str(val).strip()
        found, status = _kaliru_status(sval)
        if not found:
            raise KeyError(val)
        return status

    def __getattr__(self, val):
        return self.__getitem__(val)

    def __contains__(self, val):
        try:
            self.__getitem__(val)
        except KeyError as e:
            return False
        return True

    def __iter__(self):
        for name in _kaliru_object_names():
            if self.__contains__(name):
                yield name

    def get(self, key, default):
        try:
            return self[key]
        except KeyError:
            if default is not _ConfigSentinel:
                return default
            raise


class TemplateVariableWrapperPython:
    def __init__(self, macro):
        self.__dict__["__macro"] = macro

    def __setattr__(self, name, value):
        v = dict(_kaliru_get_vars(self.__dict__["__macro"]))
        v[name] = value
        _kaliru_set_vars(self.__dict__["__macro"], v)

    def __getattr__(self, name):
        variables = _kaliru_get_vars(self.__dict__["__macro"])
        if name not in variables:
            raise KeyError(name)
        return variables[name]

    def __contains__(self, val):
        try:
            self.__getattr__(val)
        except KeyError as e:
            return False
        return True

    def __iter__(self):
        for name, obj in _kaliru_get_vars(self.__dict__["__macro"]):
            yield name


_exc_classes = {}

def _kaliru_exception(qualname, msg):
    # an exception of python class `qualname` ("klippy.gcode.CommandError")
    module, _, name = qualname.rpartition(".")
    if not module:
        cls = getattr(builtins, name, None)
        if isinstance(cls, type) and issubclass(cls, BaseException):
            return cls(msg)
        module = "builtins"
    cls = _exc_classes.get(qualname)
    if cls is None:
        cls = type(name, (Exception,), {"__module__": module})
        _exc_classes[qualname] = cls
    return cls(msg)


_tuple_classes = {}

def _kaliru_namedtuple(name, fields, values):
    key = (name, tuple(fields))
    cls = _tuple_classes.get(key)
    if cls is None:
        def __new__(c, *vals):
            return tuple.__new__(c, vals)
        def __repr__(self):
            return name + "(" + ", ".join("%s=%r" % (f, v) for f, v in zip(fields, self)) + ")"
        ns = {"__new__": __new__, "__repr__": __repr__, "_fields": tuple(fields), "__slots__": ()}
        for i, f in enumerate(fields):
            ns[f] = property(lambda self, i=i: tuple.__getitem__(self, i))
        cls = type(name, (tuple,), ns)
        _tuple_classes[key] = cls
    return cls(*values)


def _kaliru_format_exc(e):
    # traceback.format_exception_only(type(e), e)[-1] as (class, message)
    t = type(e)
    qualname = t.__qualname__
    if t.__module__ not in ("__main__", "builtins"):
        qualname = t.__module__ + "." + qualname
    if isinstance(e, SyntaxError):
        return qualname, str(e.msg), e.lineno or 0, e.offset or 0
    return qualname, str(e), 0, 0
"#;

/// the interpreter and the prelude's globals
struct Py {
    interp: Interpreter,
    prelude: PyDictRef,
}

thread_local! {
    static PY: RefCell<Option<Rc<Py>>> = const { RefCell::new(None) };
}

fn py() -> Rc<Py> {
    PY.with(|p| {
        if let Some(py) = p.borrow().as_ref() {
            return py.clone();
        }
        let interp = Interpreter::with_init(Default::default(), |vm| {
            vm.add_native_modules(rustpython_stdlib::get_module_inits());
            vm.add_frozen(rustpython_pylib::FROZEN_STDLIB);
        });
        let prelude = interp.enter(|vm| {
            let scope = vm.new_scope_with_builtins();
            let code = vm.compile(PRELUDE, Mode::Exec, "<kaliru>".to_owned()).expect("prelude compiles");
            vm.run_code_obj(code, scope.clone()).expect("prelude runs");
            scope.globals
        });
        let py = Rc::new(Py { interp, prelude });
        *p.borrow_mut() = Some(py.clone());
        py
    })
}

fn prelude_call(vm: &VirtualMachine, name: &str, args: impl rustpython_vm::function::IntoFuncArgs) -> PyResult {
    py().prelude.get_item(name, vm)?.call(args, vm)
}

/// a template exception as the python exception
fn exc_to_py(vm: &VirtualMachine, e: Exc) -> PyBaseExceptionRef {
    match prelude_call(vm, "_kaliru_exception", (e.cls, e.msg)) {
        Ok(obj) => obj.downcast::<PyBaseException>().unwrap_or_else(|o| vm.new_type_error(format!("{:?}", o.class().name()))),
        Err(exc) => exc,
    }
}

fn kres<T>(vm: &VirtualMachine, r: KResult<T>) -> PyResult<T> {
    r.map_err(|e| exc_to_py(vm, kerror_exc(e)))
}

/// format_exception_only(type(e), e)[-1] parts: class, message, and a
/// SyntaxError's line and column
fn format_exc(vm: &VirtualMachine, e: &PyBaseExceptionRef) -> (String, String, i64, i64) {
    let r = prelude_call(vm, "_kaliru_format_exc", (e.clone(),)).ok().and_then(|t| t.downcast::<PyTuple>().ok());
    let s = |o: &rustpython_vm::PyObject| o.str(vm).map(|s| s.as_str().to_owned()).unwrap_or_default();
    let i = |o: &PyObjectRef| o.downcast_ref::<PyInt>().and_then(|n| n.try_to_primitive::<i64>(vm).ok()).unwrap_or(0);
    match r {
        Some(t) if t.len() == 4 => {
            let t = t.as_slice();
            (s(&t[0]), s(&t[1]), i(&t[2]), i(&t[3]))
        }
        _ => ("Exception".to_string(), s(e.as_object()), 0, 0),
    }
}

// Object identity. Python shares list and dict objects between macro
// variables, status dicts and the script (own_vars.lst.append(1) changes
// the variable). Kaliru keeps them as Rc'd values; while scripts run, each
// Rc list/dict that crossed over is paired with its python object. The
// python side is the current one while python runs; both sides are synced
// around helpers that run klippy code (emit, waits, actions).
enum Pair {
    List(Rc<RefCell<Vec<Value>>>, PyObjectRef),
    Dict(Rc<RefCell<IndexMap<Key, Value>>>, PyObjectRef),
}

thread_local! {
    static PAIRS: RefCell<Vec<Pair>> = const { RefCell::new(Vec::new()) };
    static DEPTH: Cell<usize> = const { Cell::new(0) };
}

fn paired_py<T>(rc: &Rc<RefCell<T>>) -> Option<PyObjectRef> {
    let ptr = Rc::as_ptr(rc) as *const u8;
    PAIRS.with(|p| {
        p.borrow().iter().find_map(|e| match e {
            Pair::List(r, o) if Rc::as_ptr(r) as *const u8 == ptr => Some(o.clone()),
            Pair::Dict(r, o) if Rc::as_ptr(r) as *const u8 == ptr => Some(o.clone()),
            _ => None,
        })
    })
}

fn paired_list(obj: &PyObjectRef) -> Option<Rc<RefCell<Vec<Value>>>> {
    PAIRS.with(|p| {
        p.borrow().iter().find_map(|e| match e {
            Pair::List(r, o) if o.is(obj) => Some(r.clone()),
            _ => None,
        })
    })
}

fn paired_dict(obj: &PyObjectRef) -> Option<Rc<RefCell<IndexMap<Key, Value>>>> {
    PAIRS.with(|p| {
        p.borrow().iter().find_map(|e| match e {
            Pair::Dict(r, o) if o.is(obj) => Some(r.clone()),
            _ => None,
        })
    })
}

fn pairs() -> Vec<(Value, PyObjectRef)> {
    PAIRS.with(|p| {
        p.borrow()
            .iter()
            .map(|e| match e {
                Pair::List(r, o) => (Value::List(r.clone()), o.clone()),
                Pair::Dict(r, o) => (Value::Dict(r.clone()), o.clone()),
            })
            .collect()
    })
}

/// python changes into the paired values
fn sync_to_rust(vm: &VirtualMachine) {
    for (_, o) in pairs() {
        py_to_value(vm, &o);
    }
}

/// klippy changes into the paired python objects
fn sync_to_py(vm: &VirtualMachine) -> PyResult<()> {
    for (v, o) in pairs() {
        match v {
            Value::List(r) => {
                let items = r.borrow().clone();
                let items = items.iter().map(|x| value_to_py(vm, x)).collect::<PyResult<Vec<_>>>()?;
                if let Some(l) = o.downcast_ref::<PyList>() {
                    *l.borrow_vec_mut() = items;
                }
            }
            Value::Dict(r) => {
                let items = r.borrow().clone();
                vm.call_method(&o, "clear", ())?;
                for (k, x) in items.iter() {
                    o.set_item(&*value_to_py(vm, &k.0)?, value_to_py(vm, x)?, vm)?;
                }
            }
            _ => {}
        }
    }
    Ok(())
}

/// a helper that runs klippy code: values synced before and after
fn klippy_call<T>(vm: &VirtualMachine, f: impl FnOnce() -> PyResult<T>) -> PyResult<T> {
    sync_to_rust(vm);
    let r = f();
    sync_to_py(vm)?;
    r
}

/// a python object kept in a template value (macro variable)
struct PyOpaque {
    obj: PyObjectRef,
    type_name: String,
    module: String,
    repr: String,
    str: String,
}

impl crate::jinja::value::Object for PyOpaque {
    fn type_name(&self) -> String {
        self.type_name.clone()
    }
    fn module(&self) -> String {
        self.module.clone()
    }
    fn repr(&self) -> String {
        self.repr.clone()
    }
    fn str(&self) -> String {
        self.str.clone()
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

/// a template value (status, macro variable, param) as a python object
fn value_to_py(vm: &VirtualMachine, v: &Value) -> PyResult {
    Ok(match v {
        Value::Bool(b) => vm.ctx.new_bool(*b).into(),
        Value::Int(i) => vm.ctx.new_int(*i).into(),
        Value::Float(f) => vm.ctx.new_float(*f).into(),
        Value::Str(s) => vm.ctx.new_str(&**s).into(),
        Value::List(l) => match paired_py(l) {
            Some(o) => o,
            None => {
                let items = l.borrow().clone();
                let o: PyObjectRef = vm.ctx.new_list(items.iter().map(|x| value_to_py(vm, x)).collect::<PyResult<_>>()?).into();
                PAIRS.with(|p| p.borrow_mut().push(Pair::List(l.clone(), o.clone())));
                o
            }
        },
        Value::Tuple(t) => vm.ctx.new_tuple(t.iter().map(|x| value_to_py(vm, x)).collect::<PyResult<_>>()?).into(),
        Value::NamedTuple(nt) => {
            let fields: Vec<PyObjectRef> = nt.1.iter().map(|s| vm.ctx.new_str(s.as_str()).into()).collect();
            let vals = nt.2.iter().map(|x| value_to_py(vm, x)).collect::<PyResult<_>>()?;
            prelude_call(vm, "_kaliru_namedtuple", (nt.0.clone(), vm.ctx.new_list(fields), vm.ctx.new_list(vals)))?
        }
        Value::Dict(d) => match paired_py(d) {
            Some(o) => o,
            None => {
                let dict = vm.ctx.new_dict();
                let items = d.borrow().clone();
                for (k, x) in items.iter() {
                    dict.set_item(&*value_to_py(vm, &k.0)?, value_to_py(vm, x)?, vm)?;
                }
                let o: PyObjectRef = dict.into();
                PAIRS.with(|p| p.borrow_mut().push(Pair::Dict(d.clone(), o.clone())));
                o
            }
        },
        Value::Object(o) => {
            if let Some(a) = o.as_any().downcast_ref::<Action>() {
                action_fn(vm, a.clone())
            } else if let Some(p) = o.as_any().downcast_ref::<PyOpaque>() {
                p.obj.clone()
            } else {
                vm.ctx.none()
            }
        }
        _ => vm.ctx.none(),
    })
}

/// a python object as a template value
fn py_to_value(vm: &VirtualMachine, obj: &PyObjectRef) -> Value {
    if vm.is_none(obj) {
        return Value::None;
    }
    let cls = obj.class();
    if cls.is(vm.ctx.types.bool_type) {
        return Value::Bool(obj.is(&vm.ctx.true_value));
    }
    if cls.is(vm.ctx.types.int_type) {
        if let Some(i) = obj.downcast_ref::<PyInt>().and_then(|i| i.try_to_primitive::<i64>(vm).ok()) {
            return Value::Int(i);
        }
    } else if cls.is(vm.ctx.types.float_type) {
        if let Some(f) = obj.downcast_ref::<PyFloat>() {
            return Value::Float(f.to_f64());
        }
    } else if cls.is(vm.ctx.types.str_type) {
        if let Some(s) = obj.downcast_ref::<PyStr>() {
            return Value::from(s.as_str());
        }
    } else if cls.is(vm.ctx.types.list_type) {
        if let Some(l) = obj.downcast_ref::<PyList>() {
            let items: Vec<PyObjectRef> = l.borrow_vec().to_vec();
            let rc = paired_list(obj).unwrap_or_else(|| {
                let rc = Rc::new(RefCell::new(Vec::new()));
                PAIRS.with(|p| p.borrow_mut().push(Pair::List(rc.clone(), obj.clone())));
                rc
            });
            let vals = items.iter().map(|x| py_to_value(vm, x)).collect();
            *rc.borrow_mut() = vals;
            return Value::List(rc);
        }
    } else if cls.is(vm.ctx.types.tuple_type) {
        if let Some(t) = obj.downcast_ref::<PyTuple>() {
            return Value::Tuple(Rc::new(t.as_slice().iter().map(|x| py_to_value(vm, x)).collect()));
        }
    } else if cls.is(vm.ctx.types.dict_type) {
        if let Some(d) = obj.downcast_ref::<PyDict>() {
            let items: Vec<(PyObjectRef, PyObjectRef)> = d.into_iter().collect();
            let rc = paired_dict(obj).unwrap_or_else(|| {
                let rc = Rc::new(RefCell::new(IndexMap::new()));
                PAIRS.with(|p| p.borrow_mut().push(Pair::Dict(rc.clone(), obj.clone())));
                rc
            });
            let vals = items.iter().map(|(k, x)| (Key(py_to_value(vm, k)), py_to_value(vm, x))).collect();
            *rc.borrow_mut() = vals;
            return Value::Dict(rc);
        }
    }
    // any other object (a set, a function, a big int) is kept as itself
    let s = |r: PyResult<rustpython_vm::builtins::PyStrRef>| r.map(|s| s.as_str().to_owned()).unwrap_or_default();
    let module = obj
        .class()
        .get_attr(vm.ctx.intern_str("__module__"))
        .and_then(|m| m.downcast::<PyStr>().ok())
        .map(|m| m.as_str().to_owned())
        .unwrap_or_default();
    Value::Object(Rc::new(PyOpaque {
        obj: obj.clone(),
        type_name: obj.class().name().to_string(),
        module,
        repr: s(obj.repr(vm)),
        str: s(obj.str(vm)),
    }))
}

/// compile(script, name, "exec"); a SyntaxError as its class, msg,
/// lineno and offset (CPython's wording, see pysyntax.rs)
fn compile(script: &str, name: &str) -> Result<PyRef<PyCode>, (String, String, i64, i64)> {
    py().interp.enter(|vm| {
        let code = vm.compile(script, Mode::Exec, name.to_owned()).map_err(|e| {
            let (cls, emsg, line, col) = format_exc(vm, &vm.new_syntax_error(&e, Some(script)));
            crate::pysyntax::cpython_error(script, cls, emsg, line, col)
        })?;
        match crate::pysyntax::missed_error(script) {
            Some(e) => Err(e),
            None => Ok(code),
        }
    })
}

/// the SyntaxError compiling `script` raises, if any (tools/gen_pysyntax_cases.py)
pub fn syntax_error(script: &str) -> Option<(String, String, i64, i64)> {
    compile(script, "<test>").err()
}

/// the exception running `script` raises, as format_exception_only's
/// class and message (tools/gen_pysyntax_cases.py --exec)
pub fn exec_error(script: &str) -> Option<(String, String)> {
    let code = compile(script, "<test>").ok()?;
    py().interp.enter(|vm| {
        let scope = vm.new_scope_with_builtins();
        vm.run_code_obj(code, scope).err().map(|e| {
            let (cls, msg, _, _) = format_exc(vm, &e);
            (cls, msg)
        })
    })
}

/// a PrinterGCodeMacro._action_* bound method as a python callable
fn action_fn(vm: &VirtualMachine, action: Action) -> PyObjectRef {
    vm.new_function("action", move |args: FuncArgs, vm: &VirtualMachine| -> PyResult {
        let pos = args.args.iter().map(|a| py_to_value(vm, a)).collect();
        let kw = args.kwargs.iter().map(|(k, a)| (k.clone(), py_to_value(vm, a))).collect();
        match klippy_call(vm, || Ok(action.run(pos, kw)))? {
            Ok(v) => value_to_py(vm, &v),
            Err(e) => Err(exc_to_py(vm, e)),
        }
    })
    .into()
}

fn lookup_macro(printer: &Printer, vm: &VirtualMachine, name: &str) -> PyResult<Rc<GCodeMacro>> {
    let obj = kres(vm, printer.lookup_object(name))?;
    downcast_rc::<GCodeMacro>(obj).ok_or_else(|| vm.new_attribute_error("object has no attribute 'variables'".to_owned()))
}

/// printer.wait_while(cb) with a python condition
fn wait_while(printer: &Printer, vm: &VirtualMachine, mut cond: impl FnMut(f64) -> PyResult<bool>) -> PyResult<()> {
    let mut err = None;
    let r = printer.wait_while(
        |eventtime| match cond(eventtime) {
            Ok(b) => Ok(b),
            Err(e) => {
                err = Some(e);
                Ok(false)
            }
        },
        true,
        1.0,
    );
    match err {
        Some(e) => Err(e),
        None => kres(vm, r),
    }
}

// Wrapper around a python script
pub struct TemplateWrapperPython {
    printer: Rc<Printer>,
    name: String,
    gcode: Rc<GCodeDispatch>,
    func: PyRef<PyCode>,
    checked_own_macro: Cell<bool>,
    has_vars: Cell<bool>,
}

impl TemplateWrapperPython {
    pub fn new(printer: &Rc<Printer>, name: &str, script: &str) -> KResult<Self> {
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let func = compile(script, name).map_err(|(cls, emsg, line, col)| {
            let msg = format!("Error compiling expression '{}': {}: {}\n at line {} column {}", name, cls, emsg, line, col);
            logging::exception_tb(&msg, &cls, &emsg);
            KError::command(msg).with_context(KError::Py(cls, emsg))
        })?;
        Ok(TemplateWrapperPython {
            printer: printer.clone(),
            name: name.to_string(),
            gcode,
            func,
            checked_own_macro: Cell::new(false),
            has_vars: Cell::new(false),
        })
    }

    /// the native functions the prelude classes call
    fn install_natives(&self, vm: &VirtualMachine, g: &PyDictRef) -> PyResult<()> {
        let p = self.printer.clone();
        let f = vm.new_function("_kaliru_status", move |name: String, vm: &VirtualMachine| -> PyResult {
            let (found, status) = match p.lookup_object_opt(&name).filter(|po| po.has_status()) {
                None => (false, vm.ctx.none()),
                Some(po) => {
                    let eventtime = p.get_reactor().monotonic();
                    let st = match downcast_rc::<GCodeMacro>(po.clone()) {
                        // dict(self.variables): the values themselves
                        Some(m) => Value::dict(m.variables.borrow().iter().map(|(k, v)| (Key::str(k), v.clone())).collect()),
                        None => Value::from_pyval(&kres(vm, po.try_get_status(eventtime))?.unwrap_or(crate::pyval::PyVal::None)),
                    };
                    (true, value_to_py(vm, &st)?)
                }
            };
            Ok(vm.ctx.new_tuple(vec![vm.ctx.new_bool(found).into(), status]).into())
        });
        g.set_item("_kaliru_status", f.into(), vm)?;
        let p = self.printer.clone();
        let f = vm.new_function("_kaliru_object_names", move |vm: &VirtualMachine| -> PyObjectRef {
            vm.ctx.new_list(p.lookup_objects(None).into_iter().map(|(n, _)| vm.ctx.new_str(n).into()).collect()).into()
        });
        g.set_item("_kaliru_object_names", f.into(), vm)?;
        let p = self.printer.clone();
        let f = vm.new_function("_kaliru_get_vars", move |name: String, vm: &VirtualMachine| -> PyResult {
            let m = lookup_macro(&p, vm, &name)?;
            let vars: IndexMap<Key, Value> = m.variables.borrow().iter().map(|(k, v)| (Key::str(k), v.clone())).collect();
            value_to_py(vm, &Value::dict(vars))
        });
        g.set_item("_kaliru_get_vars", f.into(), vm)?;
        let p = self.printer.clone();
        let f = vm.new_function("_kaliru_set_vars", move |name: String, vars: PyObjectRef, vm: &VirtualMachine| -> PyResult<()> {
            let m = lookup_macro(&p, vm, &name)?;
            if let Value::Dict(d) = py_to_value(vm, &vars) {
                *m.variables.borrow_mut() = d.borrow().iter().map(|(k, v)| (k.0.to_str(), v.clone())).collect();
            }
            Ok(())
        });
        g.set_item("_kaliru_set_vars", f.into(), vm)?;
        Ok(())
    }

    fn helpers(&self, vm: &VirtualMachine, g: &PyDictRef) -> PyResult<()> {
        g.set_item("printer", prelude_call(vm, "GetStatusWrapperPython", ())?, vm)?;
        let gcode = self.gcode.clone();
        let f = vm.new_function("_action_emit", move |script: String, vm: &VirtualMachine| -> PyResult<()> {
            klippy_call(vm, || kres(vm, gcode.run_script_from_command(&script)))
        });
        g.set_item("emit", f.into(), vm)?;
        let p = self.printer.clone();
        let f = vm.new_function("_action_wait_while", move |check: PyObjectRef, vm: &VirtualMachine| -> PyResult<()> {
            klippy_call(vm, || wait_while(&p, vm, |eventtime| check.call((eventtime,), vm)?.try_to_bool(vm)))
        });
        g.set_item("wait_while", f.into(), vm)?;
        let p = self.printer.clone();
        let f = vm.new_function("_action_wait_until", move |check: PyObjectRef, vm: &VirtualMachine| -> PyResult<()> {
            klippy_call(vm, || wait_while(&p, vm, |_| Ok(!check.call((), vm)?.try_to_bool(vm)?)))
        });
        g.set_item("wait_until", f.into(), vm)?;
        let p = self.printer.clone();
        let f = vm.new_function("_action_wait_moves", move |vm: &VirtualMachine| -> PyResult<()> {
            let toolhead = kres(vm, p.lookup::<ToolHead>("toolhead"))?;
            klippy_call(vm, || kres(vm, toolhead.wait_moves()))
        });
        g.set_item("wait_moves", f.into(), vm)?;
        // python runs func on a thread while the reactor waits for it; the
        // interpreter here is single threaded, so func runs in place
        let f = vm.new_function("_action_blocking", move |func: PyObjectRef, vm: &VirtualMachine| -> PyResult { func.call((), vm) });
        g.set_item("blocking", f.into(), vm)?;
        let p = self.printer.clone();
        let f = vm.new_function("_action_sleep", move |timeout: f64, vm: &VirtualMachine| -> PyResult<()> {
            let reactor = p.get_reactor();
            let deadline = reactor.monotonic() + timeout;
            klippy_call(vm, || wait_while(&p, vm, |_| Ok(deadline > reactor.monotonic())))
        });
        g.set_item("sleep", f.into(), vm)?;
        let p = self.printer.clone();
        let f = vm.new_function("_action_set_gcode_variable", move |macro_name: String, variable: String, value: PyObjectRef, vm: &VirtualMachine| -> PyResult<()> {
            let m = lookup_macro(&p, vm, &format!("gcode_macro {}", macro_name))?;
            let mut v = m.variables.borrow().clone();
            v.insert(variable, py_to_value(vm, &value));
            *m.variables.borrow_mut() = v;
            Ok(())
        });
        g.set_item("set_gcode_variable", f.into(), vm)?;
        for (names, method) in [
            (["emergency_stop", "action_emergency_stop"], "_action_emergency_stop"),
            (["respond_info", "action_respond_info"], "_action_respond_info"),
            (["raise_error", "action_raise_error"], "_action_raise_error"),
            (["call_remote_method", "action_call_remote_method"], "_action_call_remote_method"),
        ] {
            for name in names {
                g.set_item(name, action_fn(vm, Action::new(&self.printer, method)), vm)?;
            }
        }
        g.set_item("math", vm.import("math", 0)?, vm)?;
        if !self.checked_own_macro.get() {
            self.checked_own_macro.set(true);
            let own = self.printer.lookup_object_opt(self.own_name()).and_then(downcast_rc::<GCodeMacro>);
            self.has_vars.set(own.is_some());
        }
        if self.has_vars.get() {
            g.set_item("own_vars", prelude_call(vm, "TemplateVariableWrapperPython", (self.own_name().to_owned(),))?, vm)?;
        }
        Ok(())
    }

    fn own_name(&self) -> &str {
        self.name.split(':').next().unwrap_or("")
    }

    pub fn run_gcode_from_command(&self, context: Option<IndexMap<String, Value>>) -> KResult<()> {
        let py = py();
        py.interp.enter(|vm| {
            DEPTH.with(|d| d.set(d.get() + 1));
            let run = || -> PyResult<()> {
                self.install_natives(vm, &py.prelude)?;
                let g = vm.ctx.new_dict();
                for (k, v) in context.iter().flatten() {
                    g.set_item(k.as_str(), value_to_py(vm, v)?, vm)?;
                }
                self.helpers(vm, &g)?;
                g.set_item("__builtins__", vm.builtins.clone().into(), vm)?;
                let scope = Scope::new(Some(ArgMapping::from_dict_exact(vm.ctx.new_dict())), g);
                vm.run_code_obj(self.func.clone(), scope)?;
                Ok(())
            };
            let r = run();
            sync_to_rust(vm);
            if DEPTH.with(|d| {
                d.set(d.get() - 1);
                d.get() == 0
            }) {
                PAIRS.with(|p| p.borrow_mut().clear());
            }
            r.map_err(|e| {
                let (cls, emsg, _, _) = format_exc(vm, &e);
                let last = if emsg.is_empty() { cls.clone() } else { format!("{}: {}", cls, emsg) };
                let msg = format!("Error evaluating '{}': {}\n", self.name, last);
                logging::exception_tb(&msg, &cls, &emsg);
                KError::command(msg)
            })
        })
    }
}
