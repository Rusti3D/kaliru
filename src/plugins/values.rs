// kaliru's template values (jinja Value) seen from python. A Value list or
// dict is shared by reference (Rc), as python shares its objects between a
// macro's variables, its status and a running script: python gets one
// object per Rc, a list / dict subclass (prelude.py ValueList / ValueDict)
// paired through the object table, so `is` holds and json, isinstance and
// repr see a real list / dict. Its contents are refreshed from the Rc each
// time kaliru hands it over, and each python mutation writes the new
// contents back, so a jinja template run in between (emit()) sees it.
use super::objects;
use super::py::{self, PyRef};
use crate::error::KResult;
use crate::jinja::value::{Key, Value};
use indexmap::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

pub type ListRc = Rc<RefCell<Vec<Value>>>;
pub type DictRc = Rc<RefCell<IndexMap<Key, Value>>>;

/// A template value as a python object
pub fn value_to_py(v: &Value) -> KResult<PyRef> {
    Ok(match v {
        Value::None => py::py_none(),
        Value::Bool(b) => py::py_bool(*b),
        Value::Int(i) => py::py_int(*i),
        Value::Float(f) => py::py_float(*f),
        Value::Str(s) => py::py_str(s),
        Value::Tuple(t) => py::py_tuple(t.iter().map(value_to_py).collect::<KResult<_>>()?),
        Value::List(rc) => {
            let obj = objects::wrap(rc.clone())?;
            let items = rc.borrow().iter().map(value_to_py).collect::<KResult<Vec<_>>>()?;
            py::lookup("_k", "_vl_fill")?.call(&[&obj, &py::py_list(items)])?;
            obj
        }
        Value::Dict(rc) => {
            let obj = objects::wrap(rc.clone())?;
            let pairs = rc
                .borrow()
                .iter()
                .map(|(k, v)| Ok(py::py_tuple(vec![value_to_py(&k.0)?, value_to_py(v)?])))
                .collect::<KResult<Vec<_>>>()?;
            py::lookup("_k", "_vd_fill")?.call(&[&obj, &py::py_list(pairs)])?;
            obj
        }
        v => py::from_pyval(&v.to_pyval()),
    })
}

/// A python object as a template value: a paired list / dict is its Rc
pub fn py_to_value(o: &PyRef) -> KResult<Value> {
    if let Ok(rc) = objects::unwrap_as::<RefCell<Vec<Value>>>(o) {
        return Ok(Value::List(rc));
    }
    if let Ok(rc) = objects::unwrap_as::<RefCell<IndexMap<Key, Value>>>(o) {
        return Ok(Value::Dict(rc));
    }
    let t = py::type_name(o)?;
    if t == "list" {
        return Ok(Value::list(seq(o)?));
    }
    if t == "tuple" {
        return Ok(Value::tuple(seq(o)?));
    }
    if t == "dict" {
        return Ok(Value::dict(pairs(o)?));
    }
    Ok(Value::from_pyval(&py::to_pyval(o)?))
}

fn seq(o: &PyRef) -> KResult<Vec<Value>> {
    let items = py::lookup("builtins", "list")?.call(&[o])?;
    let n = py::lookup("builtins", "len")?.call(&[&items])?.as_i64()?;
    (0..n).map(|i| py_to_value(&items.call_method("__getitem__", &[&py::py_int(i)])?)).collect()
}

fn pairs(o: &PyRef) -> KResult<IndexMap<Key, Value>> {
    let items = py::lookup("builtins", "list")?.call(&[&o.call_method("items", &[])?])?;
    let n = py::lookup("builtins", "len")?.call(&[&items])?.as_i64()?;
    let mut out = IndexMap::new();
    for i in 0..n {
        let kv = items.call_method("__getitem__", &[&py::py_int(i)])?;
        let k = py_to_value(&kv.call_method("__getitem__", &[&py::py_int(0)])?)?;
        let v = py_to_value(&kv.call_method("__getitem__", &[&py::py_int(1)])?)?;
        out.insert(Key::hashable(&k).map_err(|e| crate::error::KError::Py("TypeError".into(), e.msg))?, v);
    }
    Ok(out)
}

/// _k ValueList / ValueDict after a mutation: the contents back into the Rc
pub fn store_list(obj: &PyRef) -> KResult<()> {
    let rc = objects::unwrap_as::<RefCell<Vec<Value>>>(obj)?;
    let items = seq(&py::lookup("builtins", "list")?.call(&[obj])?)?;
    *rc.borrow_mut() = items;
    Ok(())
}

pub fn store_dict(obj: &PyRef) -> KResult<()> {
    let rc = objects::unwrap_as::<RefCell<IndexMap<Key, Value>>>(obj)?;
    let d = pairs(&py::lookup("builtins", "dict")?.call(&[obj])?)?;
    *rc.borrow_mut() = d;
    Ok(())
}
