// Jinja2 3.1.6 builtin tests (port of jinja2/tests.py)
use super::eval::Renderer;
use super::filters::{bind, FILTER_NAMES};
use super::runtime::Markup;
use super::value::{Exc, VResult, Value};
use crate::pyfmt::py_str_repr;
use indexmap::IndexMap;

pub const TEST_NAMES: &[&str] = &[
    "odd", "even", "divisibleby", "defined", "undefined", "filter", "test", "none", "boolean", "false", "true", "integer",
    "float", "lower", "upper", "string", "mapping", "number", "sequence", "iterable", "callable", "sameas", "escaped", "in", "==",
    "eq", "equalto", "!=", "ne", ">", "gt", "greaterthan", "ge", ">=", "<", "lt", "lessthan", "<=", "le",
];

/// `value is sameas other`
fn is_same(a: &Value, b: &Value) -> bool {
    use std::rc::Rc;
    match (a, b) {
        (Value::None, Value::None) => true,
        (Value::Bool(x), Value::Bool(y)) => x == y,
        // small ints are cached by python
        (Value::Int(x), Value::Int(y)) => x == y && (-5..=256).contains(x),
        (Value::Str(x), Value::Str(y)) => Rc::ptr_eq(x, y),
        (Value::List(x), Value::List(y)) => Rc::ptr_eq(x, y),
        (Value::Dict(x), Value::Dict(y)) => Rc::ptr_eq(x, y),
        (Value::Tuple(x), Value::Tuple(y)) => Rc::ptr_eq(x, y),
        (Value::Object(x), Value::Object(y)) => std::ptr::eq(Rc::as_ptr(x) as *const (), Rc::as_ptr(y) as *const ()),
        (Value::Undefined(x), Value::Undefined(y)) => Rc::ptr_eq(x, y),
        _ => false,
    }
}

fn callable(v: &Value) -> bool {
    match v {
        Value::Object(o) => {
            o.as_any().is::<super::methods::BoundMethod>()
                || o.as_any().is::<super::eval::MacroObj>()
                || o.as_any().is::<super::runtime::BuiltinFn>()
                || o.as_any().is::<super::runtime::Joiner>()
                || o.type_name() == "method"
                || o.type_name().ends_with("function")
        }
        Value::Undefined(_) => false,
        _ => false,
    }
}

/// environment.call_test(name, value, args, kwargs)
pub fn call_test(_r: &mut Renderer, name: &str, value: Value, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<bool> {
    let mut all = vec![value];
    all.extend(args);
    macro_rules! b {
        ($fname:expr, [$($p:expr),*]) => {
            bind($fname, &[$(($p, None)),*], all, kwargs, false, false)?.0
        };
    }
    let mod2 = |v: &Value| v.modulo(&Value::Int(2));
    match name {
        "odd" => {
            let a = b!("test_odd", ["value"]);
            Ok(mod2(&a[0])?.py_eq(&Value::Int(1)))
        }
        "even" => {
            let a = b!("test_even", ["value"]);
            Ok(mod2(&a[0])?.py_eq(&Value::Int(0)))
        }
        "divisibleby" => {
            let a = b!("test_divisibleby", ["value", "num"]);
            Ok(a[0].modulo(&a[1])?.py_eq(&Value::Int(0)))
        }
        "defined" => {
            let a = b!("test_defined", ["value"]);
            Ok(!a[0].is_undefined())
        }
        "undefined" => {
            let a = b!("test_undefined", ["value"]);
            Ok(a[0].is_undefined())
        }
        "filter" => {
            let a = b!("test_filter", ["value"]);
            let v = a[0].to_str();
            Ok(matches!(a[0], Value::Str(_)) && FILTER_NAMES.contains(&v.as_str()))
        }
        "test" => {
            let a = b!("test_test", ["value"]);
            let v = a[0].to_str();
            Ok(matches!(a[0], Value::Str(_)) && TEST_NAMES.contains(&v.as_str()))
        }
        "none" => {
            let a = b!("test_none", ["value"]);
            Ok(matches!(a[0], Value::None))
        }
        "boolean" => {
            let a = b!("test_boolean", ["value"]);
            Ok(matches!(a[0], Value::Bool(_)))
        }
        "false" => {
            let a = b!("test_false", ["value"]);
            Ok(matches!(a[0], Value::Bool(false)))
        }
        "true" => {
            let a = b!("test_true", ["value"]);
            Ok(matches!(a[0], Value::Bool(true)))
        }
        "integer" => {
            let a = b!("test_integer", ["value"]);
            Ok(matches!(a[0], Value::Int(_)))
        }
        "float" => {
            let a = b!("test_float", ["value"]);
            Ok(matches!(a[0], Value::Float(_)))
        }
        "lower" | "upper" => {
            let fname = if name == "lower" { "test_lower" } else { "test_upper" };
            let a = b!(fname, ["value"]);
            let s = a[0].to_str();
            let method = if name == "lower" { "islower" } else { "isupper" };
            Ok(super::methods::call_method(&Value::from(s), method, vec![], IndexMap::new())?.truthy())
        }
        "string" => {
            let a = b!("test_string", ["value"]);
            Ok(matches!(a[0], Value::Str(_)) || a[0].as_object::<Markup>().is_some())
        }
        "mapping" => {
            let a = b!("test_mapping", ["value"]);
            Ok(matches!(a[0], Value::Dict(_)))
        }
        "number" => {
            let a = b!("test_number", ["value"]);
            Ok(matches!(a[0], Value::Bool(_) | Value::Int(_) | Value::Float(_)))
        }
        "sequence" => {
            let a = b!("test_sequence", ["value"]);
            Ok(match &a[0] {
                Value::Str(_) | Value::List(_) | Value::Tuple(_) | Value::NamedTuple(_) | Value::Dict(_) | Value::Range(..) => true,
                Value::Object(o) => o.len().is_some() && o.getitem(&Value::Int(0)).is_some(),
                _ => false,
            })
        }
        "iterable" => {
            let a = b!("test_iterable", ["value"]);
            Ok(match &a[0] {
                Value::Object(o) => o.iter().is_some(),
                v => v.iter_items().is_ok(),
            })
        }
        "callable" => {
            let a = b!("callable", ["obj"]);
            Ok(callable(&a[0]))
        }
        "sameas" => {
            let a = b!("test_sameas", ["value", "other"]);
            Ok(is_same(&a[0], &a[1]))
        }
        "escaped" => {
            let a = b!("test_escaped", ["value"]);
            Ok(a[0].as_object::<Markup>().is_some())
        }
        "in" => {
            let a = b!("test_in", ["value", "seq"]);
            a[1].contains(&a[0])
        }
        "==" | "eq" | "equalto" => {
            let a = b!("eq", ["a", "b"]);
            Ok(a[0].py_eq(&a[1]))
        }
        "!=" | "ne" => {
            let a = b!("ne", ["a", "b"]);
            Ok(!a[0].py_eq(&a[1]))
        }
        ">" | "gt" | "greaterthan" => {
            let a = b!("gt", ["a", "b"]);
            a[0].compare(&a[1], "gt")
        }
        ">=" | "ge" => {
            let a = b!("ge", ["a", "b"]);
            a[0].compare(&a[1], "ge")
        }
        "<" | "lt" | "lessthan" => {
            let a = b!("lt", ["a", "b"]);
            a[0].compare(&a[1], "lt")
        }
        "<=" | "le" => {
            let a = b!("le", ["a", "b"]);
            a[0].compare(&a[1], "le")
        }
        _ => Err(Exc::runtime(format!("No test named {} found.", py_str_repr(name)))),
    }
}
