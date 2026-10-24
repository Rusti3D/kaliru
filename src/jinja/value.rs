// Template values with python object semantics: what jinja2's generated
// code does with python objects (str(), repr(), ==, <, +, in, len(), iter(),
// hash()), for the object types templates meet in klippy.
use crate::pyfmt::{py_float_repr, py_str_repr};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

/// a python exception: class (as format_exception_only prints it) and str(e)
#[derive(Clone, Debug)]
pub struct Exc {
    pub cls: String,
    pub msg: String,
    /// a klippy error raised through the template (action_raise_error, a
    /// failing command), kept so its type survives
    pub kerror: Option<crate::error::KError>,
}

impl Exc {
    pub fn new(cls: &str, msg: impl Into<String>) -> Exc {
        Exc { cls: cls.to_string(), msg: msg.into(), kerror: None }
    }
    pub fn type_error(msg: impl Into<String>) -> Exc {
        Exc::new("TypeError", msg)
    }
    pub fn value_error(msg: impl Into<String>) -> Exc {
        Exc::new("ValueError", msg)
    }
    pub fn key_error(key: &Value) -> Exc {
        Exc::new("KeyError", key.repr())
    }
    pub fn attribute_error(msg: impl Into<String>) -> Exc {
        Exc::new("AttributeError", msg)
    }
    pub fn index_error(msg: impl Into<String>) -> Exc {
        Exc::new("IndexError", msg)
    }
    pub fn zero_division(msg: &str) -> Exc {
        Exc::new("ZeroDivisionError", msg)
    }
    pub fn undefined(msg: impl Into<String>) -> Exc {
        Exc::new("jinja2.exceptions.UndefinedError", msg)
    }
    pub fn runtime(msg: impl Into<String>) -> Exc {
        Exc::new("jinja2.exceptions.TemplateRuntimeError", msg)
    }
    /// is this a LookupError (KeyError / IndexError)?
    pub fn is_lookup(&self) -> bool {
        matches!(self.cls.as_str(), "KeyError" | "IndexError")
    }
    /// traceback.format_exception_only(type(e), e)[-1]
    pub fn format_only(&self) -> String {
        if self.msg.is_empty() {
            format!("{}\n", self.cls)
        } else {
            format!("{}: {}\n", self.cls, self.msg)
        }
    }
}

pub type VResult<T> = Result<T, Exc>;

/// jinja2 Undefined
#[derive(Debug)]
pub struct Undefined {
    pub hint: Option<String>,
    /// the object the attribute / item was looked up on (None: missing)
    pub obj: Option<Value>,
    pub name: Option<Value>,
}

impl Undefined {
    /// Undefined._undefined_message
    pub fn message(&self) -> String {
        if let Some(h) = &self.hint {
            return h.clone();
        }
        let name = self.name.clone().unwrap_or(Value::None);
        match &self.obj {
            None => format!("{} is undefined", name.repr()),
            Some(obj) => {
                if !matches!(name, Value::Str(_)) {
                    format!("{} has no element {}", object_type_repr(obj), name.repr())
                } else {
                    format!("{} has no attribute {}", py_str_repr(&object_type_repr(obj)), name.repr())
                }
            }
        }
    }
}

/// jinja2.utils.object_type_repr
pub fn object_type_repr(obj: &Value) -> String {
    if matches!(obj, Value::None) {
        return "None".into();
    }
    let (module, name) = obj.type_module_name();
    if module == "builtins" {
        format!("{} object", name)
    } else {
        format!("{}.{} object", module, name)
    }
}

/// python objects other than the builtin containers and numbers
pub trait Object: Any {
    /// type(o).__name__
    fn type_name(&self) -> String;
    /// type(o).__module__
    fn module(&self) -> String {
        "builtins".into()
    }
    fn repr(&self) -> String {
        format!("<{}.{} object>", self.module(), self.type_name())
    }
    fn str(&self) -> String {
        self.repr()
    }
    /// getattr(o, name); None is an AttributeError
    fn getattr(&self, _me: &Value, _name: &str) -> Option<Value> {
        None
    }
    /// o[key]; None when the type is not subscriptable
    fn getitem(&self, _key: &Value) -> Option<VResult<Value>> {
        None
    }
    /// o(*args, **kwargs); None when not callable
    fn call(&self, _ctx: &mut dyn CallCtx, _args: Vec<Value>, _kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        None
    }
    /// list(iter(o)); None when not iterable
    fn iter(&self) -> Option<VResult<Vec<Value>>> {
        None
    }
    fn len(&self) -> Option<usize> {
        None
    }
    fn truthy(&self) -> bool {
        self.len().map(|l| l != 0).unwrap_or(true)
    }
    /// `v in o`; None falls back to iteration
    fn contains(&self, _v: &Value) -> Option<VResult<bool>> {
        None
    }
    /// o[key] = value; None when unsupported
    fn setitem(&self, _key: &str, _value: Value) -> Option<VResult<()>> {
        None
    }
    fn as_any(&self) -> &dyn Any;
}

/// what a called object can reach of the running template (macros render
/// with it, context functions read it)
pub trait CallCtx {
    fn call_macro(&mut self, m: &super::eval::MacroObj, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value>;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

#[derive(Clone)]
pub enum Value {
    None,
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(Rc<str>),
    List(Rc<RefCell<Vec<Value>>>),
    Tuple(Rc<Vec<Value>>),
    Dict(Rc<RefCell<IndexMap<Key, Value>>>),
    /// collections.namedtuple instance: (type name, field names, values)
    NamedTuple(Rc<(String, Vec<String>, Vec<Value>)>),
    /// range(start, stop, step)
    Range(i64, i64, i64),
    /// a slice object (start, stop, step)
    Slice(Rc<(Value, Value, Value)>),
    Undefined(Rc<Undefined>),
    Object(Rc<dyn Object>),
}

impl std::fmt::Debug for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str(&self.repr())
    }
}

/// a hashable value (dict key / set member) with python's equality rules
#[derive(Clone, Debug)]
pub struct Key(pub Value);

impl PartialEq for Key {
    fn eq(&self, other: &Key) -> bool {
        self.0.py_eq(&other.0)
    }
}

impl Eq for Key {}

impl Hash for Key {
    fn hash<H: Hasher>(&self, state: &mut H) {
        hash_value(&self.0, state)
    }
}

fn hash_value<H: Hasher>(v: &Value, state: &mut H) {
    match v {
        Value::None => 0u8.hash(state),
        Value::Bool(b) => (*b as i64).hash(state),
        Value::Int(i) => i.hash(state),
        Value::Float(f) => {
            // equal numbers hash equal (1 == 1.0)
            if f.fract() == 0.0 && f.is_finite() && f.abs() < 9.2e18 {
                (*f as i64).hash(state)
            } else {
                f.to_bits().hash(state)
            }
        }
        Value::Str(s) => s.hash(state),
        Value::Tuple(t) => {
            for x in t.iter() {
                hash_value(x, state);
            }
        }
        Value::NamedTuple(nt) => {
            for x in nt.2.iter() {
                hash_value(x, state);
            }
        }
        Value::Range(a, b, c) => (a, b, c).hash(state),
        Value::Object(o) => (Rc::as_ptr(o) as *const () as usize).hash(state),
        Value::Undefined(_) => 1u8.hash(state),
        _ => 2u8.hash(state),
    }
}

impl From<bool> for Value {
    fn from(b: bool) -> Self {
        Value::Bool(b)
    }
}
impl From<i64> for Value {
    fn from(i: i64) -> Self {
        Value::Int(i)
    }
}
impl From<f64> for Value {
    fn from(f: f64) -> Self {
        Value::Float(f)
    }
}
impl From<&str> for Value {
    fn from(s: &str) -> Self {
        Value::Str(s.into())
    }
}
impl From<String> for Value {
    fn from(s: String) -> Self {
        Value::Str(s.into())
    }
}

fn int_pow(base: i64, exp: u32) -> Option<i64> {
    base.checked_pow(exp)
}

/// python's float floor division and modulo (divmod)
pub fn float_divmod(x: f64, y: f64) -> (f64, f64) {
    let mut m = x % y;
    let mut div = (x - m) / y;
    if m != 0.0 {
        if (y < 0.0) != (m < 0.0) {
            m += y;
            div -= 1.0;
        }
    } else {
        m = 0.0f64.copysign(y);
    }
    let floordiv = if div != 0.0 {
        let f = div.floor();
        if div - f > 0.5 {
            f + 1.0
        } else {
            f
        }
    } else {
        0.0f64.copysign(x / y)
    };
    (floordiv, m)
}

impl Value {
    pub fn str(s: &str) -> Value {
        Value::Str(s.into())
    }

    pub fn list(items: Vec<Value>) -> Value {
        Value::List(Rc::new(RefCell::new(items)))
    }

    pub fn tuple(items: Vec<Value>) -> Value {
        Value::Tuple(Rc::new(items))
    }

    pub fn dict(items: IndexMap<Key, Value>) -> Value {
        Value::Dict(Rc::new(RefCell::new(items)))
    }

    pub fn undefined(hint: Option<String>, obj: Option<Value>, name: Option<Value>) -> Value {
        Value::Undefined(Rc::new(Undefined { hint, obj, name }))
    }

    pub fn object(o: impl Object + 'static) -> Value {
        Value::Object(Rc::new(o))
    }

    pub fn is_undefined(&self) -> bool {
        matches!(self, Value::Undefined(_))
    }

    pub fn as_object<T: 'static>(&self) -> Option<&T> {
        match self {
            Value::Object(o) => o.as_any().downcast_ref::<T>(),
            _ => None,
        }
    }

    /// (type(v).__module__, type(v).__name__)
    pub fn type_module_name(&self) -> (String, String) {
        let b = |n: &str| ("builtins".to_string(), n.to_string());
        match self {
            Value::None => b("NoneType"),
            Value::Bool(_) => b("bool"),
            Value::Int(_) => b("int"),
            Value::Float(_) => b("float"),
            Value::Str(_) => b("str"),
            Value::List(_) => b("list"),
            Value::Tuple(_) => b("tuple"),
            Value::Dict(_) => b("dict"),
            Value::NamedTuple(nt) => ("klippy.gcode".to_string(), nt.0.clone()),
            Value::Range(..) => b("range"),
            Value::Slice(_) => b("slice"),
            Value::Undefined(_) => ("jinja2.runtime".to_string(), "Undefined".to_string()),
            Value::Object(o) => (o.module(), o.type_name()),
        }
    }

    /// type(v).__name__
    pub fn type_name(&self) -> String {
        self.type_module_name().1
    }

    /// python repr()
    pub fn repr(&self) -> String {
        match self {
            Value::None => "None".into(),
            Value::Bool(b) => if *b { "True" } else { "False" }.into(),
            Value::Int(i) => i.to_string(),
            Value::Float(f) => py_float_repr(*f),
            Value::Str(s) => py_str_repr(s),
            Value::List(l) => {
                let items: Vec<String> = l.borrow().iter().map(|v| v.repr()).collect();
                format!("[{}]", items.join(", "))
            }
            Value::Tuple(t) => {
                let items: Vec<String> = t.iter().map(|v| v.repr()).collect();
                if items.len() == 1 {
                    format!("({},)", items[0])
                } else {
                    format!("({})", items.join(", "))
                }
            }
            Value::Dict(d) => {
                let items: Vec<String> = d.borrow().iter().map(|(k, v)| format!("{}: {}", k.0.repr(), v.repr())).collect();
                format!("{{{}}}", items.join(", "))
            }
            Value::NamedTuple(nt) => {
                let items: Vec<String> = nt.1.iter().zip(&nt.2).map(|(f, v)| format!("{}={}", f, v.repr())).collect();
                format!("{}({})", nt.0, items.join(", "))
            }
            Value::Range(a, b, c) => {
                if *c == 1 {
                    format!("range({}, {})", a, b)
                } else {
                    format!("range({}, {}, {})", a, b, c)
                }
            }
            Value::Slice(s) => format!("slice({}, {}, {})", s.0.repr(), s.1.repr(), s.2.repr()),
            Value::Undefined(_) => "Undefined".into(),
            Value::Object(o) => o.repr(),
        }
    }

    /// python str()
    pub fn to_str(&self) -> String {
        match self {
            Value::Str(s) => s.to_string(),
            Value::Undefined(_) => String::new(),
            Value::Object(o) => o.str(),
            v => v.repr(),
        }
    }

    /// python bool()
    pub fn truthy(&self) -> bool {
        match self {
            Value::None => false,
            Value::Bool(b) => *b,
            Value::Int(i) => *i != 0,
            Value::Float(f) => *f != 0.0,
            Value::Str(s) => !s.is_empty(),
            Value::List(l) => !l.borrow().is_empty(),
            Value::Tuple(t) => !t.is_empty(),
            Value::Dict(d) => !d.borrow().is_empty(),
            Value::NamedTuple(nt) => !nt.2.is_empty(),
            Value::Range(..) => self.range_len() != 0,
            Value::Slice(_) => true,
            Value::Undefined(_) => false,
            Value::Object(o) => o.truthy(),
        }
    }

    fn range_len(&self) -> i64 {
        match self {
            Value::Range(a, b, c) => {
                let (a, b, c) = (*a as i128, *b as i128, *c as i128);
                let n = if c > 0 { (b - a + c - 1) / c } else { (a - b - c - 1) / -c };
                n.max(0) as i64
            }
            _ => 0,
        }
    }

    /// the number value of bool / int / float
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Bool(b) => Some(*b as i64 as f64),
            Value::Int(i) => Some(*i as f64),
            Value::Float(f) => Some(*f),
            _ => None,
        }
    }

    pub fn as_int(&self) -> Option<i64> {
        match self {
            Value::Bool(b) => Some(*b as i64),
            Value::Int(i) => Some(*i),
            _ => None,
        }
    }

    fn is_num(&self) -> bool {
        matches!(self, Value::Bool(_) | Value::Int(_) | Value::Float(_))
    }

    /// python ==
    pub fn py_eq(&self, other: &Value) -> bool {
        use Value::*;
        match (self, other) {
            (None, None) => true,
            (Str(a), Str(b)) => a == b,
            (Bool(_) | Int(_), Bool(_) | Int(_)) => self.as_int() == other.as_int(),
            (a, b) if a.is_num() && b.is_num() => a.as_f64() == b.as_f64(),
            (List(a), List(b)) => {
                if Rc::ptr_eq(a, b) {
                    return true;
                }
                let (a, b) = (a.borrow(), b.borrow());
                a.len() == b.len() && a.iter().zip(b.iter()).all(|(x, y)| x.py_eq(y))
            }
            (Tuple(_) | NamedTuple(_), Tuple(_) | NamedTuple(_)) => {
                let (a, b) = (self.tuple_items(), other.tuple_items());
                a.len() == b.len() && a.iter().zip(b.iter()).all(|(x, y)| x.py_eq(y))
            }
            (Dict(a), Dict(b)) => {
                let (a, b) = (a.borrow(), b.borrow());
                a.len() == b.len() && a.iter().all(|(k, v)| b.get(k).map(|w| v.py_eq(w)).unwrap_or(false))
            }
            (Range(..), Range(..)) => {
                let (a, b) = (self.iter_items().unwrap_or_default(), other.iter_items().unwrap_or_default());
                a.len() == b.len() && a.iter().zip(b.iter()).all(|(x, y)| x.py_eq(y))
            }
            // Undefined.__eq__: type(self) is type(other)
            (Undefined(_), Undefined(_)) => true,
            (Object(a), Object(b)) => std::ptr::eq(Rc::as_ptr(a) as *const (), Rc::as_ptr(b) as *const ()),
            _ => false,
        }
    }

    fn tuple_items(&self) -> Vec<Value> {
        match self {
            Value::Tuple(t) => t.to_vec(),
            Value::NamedTuple(nt) => nt.2.clone(),
            _ => vec![],
        }
    }

    /// python <, <=, >, >= ("lt", "le", "gt", "ge")
    pub fn compare(&self, other: &Value, op: &str) -> VResult<bool> {
        use std::cmp::Ordering;
        let sym = match op {
            "lt" => "<",
            "le" => "<=",
            "gt" => ">",
            _ => ">=",
        };
        let ord: Option<Ordering> = match (self, other) {
            (a, b) if a.is_num() && b.is_num() => {
                if let (Some(x), Some(y)) = (a.as_int(), b.as_int()) {
                    Some(x.cmp(&y))
                } else {
                    // nan compares false
                    match a.as_f64().unwrap().partial_cmp(&b.as_f64().unwrap()) {
                        Some(o) => Some(o),
                        None => return Ok(false),
                    }
                }
            }
            (Value::Str(a), Value::Str(b)) => Some(a.cmp(b)),
            (Value::List(_), Value::List(_)) | (Value::Tuple(_) | Value::NamedTuple(_), Value::Tuple(_) | Value::NamedTuple(_)) => {
                let (a, b) = match (self, other) {
                    (Value::List(a), Value::List(b)) => (a.borrow().clone(), b.borrow().clone()),
                    _ => (self.tuple_items(), other.tuple_items()),
                };
                for (x, y) in a.iter().zip(b.iter()) {
                    if !x.py_eq(y) {
                        return x.compare(y, op);
                    }
                }
                Some(a.len().cmp(&b.len()))
            }
            (Value::Undefined(u), _) | (_, Value::Undefined(u)) => return Err(Exc::undefined(u.message())),
            _ => None,
        };
        let Some(ord) = ord else {
            return Err(Exc::type_error(format!(
                "'{}' not supported between instances of '{}' and '{}'",
                sym,
                self.type_name(),
                other.type_name()
            )));
        };
        Ok(match op {
            "lt" => ord == Ordering::Less,
            "le" => ord != Ordering::Greater,
            "gt" => ord == Ordering::Greater,
            _ => ord != Ordering::Less,
        })
    }

    fn binop_error(&self, other: &Value, sym: &str) -> Exc {
        if let Value::Undefined(u) = self {
            return Exc::undefined(u.message());
        }
        if let Value::Undefined(u) = other {
            return Exc::undefined(u.message());
        }
        Exc::type_error(format!(
            "unsupported operand type(s) for {}: '{}' and '{}'",
            sym,
            self.type_name(),
            other.type_name()
        ))
    }

    /// python a + b
    pub fn add(&self, other: &Value) -> VResult<Value> {
        use Value::*;
        Ok(match (self, other) {
            (a, b) if a.as_int().is_some() && b.as_int().is_some() => match a.as_int().unwrap().checked_add(b.as_int().unwrap()) {
                Some(v) => Int(v),
                Option::None => Float(a.as_f64().unwrap() + b.as_f64().unwrap()),
            },
            (a, b) if a.is_num() && b.is_num() => Float(a.as_f64().unwrap() + b.as_f64().unwrap()),
            (Str(a), Str(b)) => Value::from(format!("{}{}", a, b)),
            (List(a), List(b)) => {
                let mut v = a.borrow().clone();
                v.extend(b.borrow().iter().cloned());
                Value::list(v)
            }
            (Tuple(_) | NamedTuple(_), Tuple(_) | NamedTuple(_)) => {
                let mut v = self.tuple_items();
                v.extend(other.tuple_items());
                Value::tuple(v)
            }
            (Str(_), b) if !matches!(b, Undefined(_)) => {
                return Err(Exc::type_error(format!("can only concatenate str (not \"{}\") to str", b.type_name())))
            }
            (List(_), b) if !matches!(b, Undefined(_)) => {
                return Err(Exc::type_error(format!("can only concatenate list (not \"{}\") to list", b.type_name())))
            }
            (Tuple(_), b) if !matches!(b, Undefined(_)) => {
                return Err(Exc::type_error(format!("can only concatenate tuple (not \"{}\") to tuple", b.type_name())))
            }
            _ => return Err(self.binop_error(other, "+")),
        })
    }

    pub fn sub(&self, other: &Value) -> VResult<Value> {
        Ok(match (self, other) {
            (a, b) if a.as_int().is_some() && b.as_int().is_some() => match a.as_int().unwrap().checked_sub(b.as_int().unwrap()) {
                Some(v) => Value::Int(v),
                None => Value::Float(a.as_f64().unwrap() - b.as_f64().unwrap()),
            },
            (a, b) if a.is_num() && b.is_num() => Value::Float(a.as_f64().unwrap() - b.as_f64().unwrap()),
            _ => return Err(self.binop_error(other, "-")),
        })
    }

    pub fn mul(&self, other: &Value) -> VResult<Value> {
        use Value::*;
        let repeat = |items: Vec<Value>, n: i64| -> Vec<Value> {
            let mut out = vec![];
            for _ in 0..n.max(0) {
                out.extend(items.iter().cloned());
            }
            out
        };
        Ok(match (self, other) {
            (a, b) if a.as_int().is_some() && b.as_int().is_some() => match a.as_int().unwrap().checked_mul(b.as_int().unwrap()) {
                Some(v) => Int(v),
                Option::None => Float(a.as_f64().unwrap() * b.as_f64().unwrap()),
            },
            (a, b) if a.is_num() && b.is_num() => Float(a.as_f64().unwrap() * b.as_f64().unwrap()),
            (Str(s), n) | (n, Str(s)) if n.as_int().is_some() => Value::from(s.repeat(n.as_int().unwrap().max(0) as usize)),
            (List(l), n) | (n, List(l)) if n.as_int().is_some() => Value::list(repeat(l.borrow().clone(), n.as_int().unwrap())),
            (t @ (Tuple(_) | NamedTuple(_)), n) | (n, t @ (Tuple(_) | NamedTuple(_))) if n.as_int().is_some() => {
                Value::tuple(repeat(t.tuple_items(), n.as_int().unwrap()))
            }
            (Str(_) | List(_) | Tuple(_), Float(_)) | (Float(_), Str(_) | List(_) | Tuple(_)) => {
                return Err(Exc::type_error("can't multiply sequence by non-int of type 'float'"))
            }
            _ => return Err(self.binop_error(other, "*")),
        })
    }

    pub fn truediv(&self, other: &Value) -> VResult<Value> {
        if self.is_num() && other.is_num() {
            let (a, b) = (self.as_f64().unwrap(), other.as_f64().unwrap());
            if b == 0.0 {
                let msg = if matches!(self, Value::Float(_)) || matches!(other, Value::Float(_)) {
                    "float division by zero"
                } else {
                    "division by zero"
                };
                return Err(Exc::zero_division(msg));
            }
            if let (Some(x), Some(y)) = (self.as_int(), other.as_int()) {
                // int / int: correctly rounded
                return Ok(Value::Float(x as f64 / y as f64));
            }
            return Ok(Value::Float(a / b));
        }
        Err(self.binop_error(other, "/"))
    }

    pub fn floordiv(&self, other: &Value) -> VResult<Value> {
        if let (Some(x), Some(y)) = (self.as_int(), other.as_int()) {
            if y == 0 {
                return Err(Exc::zero_division("integer division or modulo by zero"));
            }
            let q = x.wrapping_div(y);
            let r = x.wrapping_rem(y);
            return Ok(Value::Int(if r != 0 && ((r < 0) != (y < 0)) { q - 1 } else { q }));
        }
        if self.is_num() && other.is_num() {
            let (a, b) = (self.as_f64().unwrap(), other.as_f64().unwrap());
            if b == 0.0 {
                return Err(Exc::zero_division("float floor division by zero"));
            }
            return Ok(Value::Float(float_divmod(a, b).0));
        }
        Err(self.binop_error(other, "//"))
    }

    pub fn modulo(&self, other: &Value) -> VResult<Value> {
        if let Value::Str(s) = self {
            // printf style formatting
            let args = other.to_pyval_shallow();
            return crate::pyformat::percent_format(s, &args).map(Value::from).map_err(|(cls, msg)| Exc::new(cls, msg));
        }
        if let (Some(x), Some(y)) = (self.as_int(), other.as_int()) {
            if y == 0 {
                return Err(Exc::zero_division("integer modulo by zero"));
            }
            let m = x % y;
            return Ok(Value::Int(if m != 0 && ((m < 0) != (y < 0)) { m + y } else { m }));
        }
        if self.is_num() && other.is_num() {
            let (a, b) = (self.as_f64().unwrap(), other.as_f64().unwrap());
            if b == 0.0 {
                return Err(Exc::zero_division("float modulo by zero"));
            }
            return Ok(Value::Float(float_divmod(a, b).1));
        }
        Err(self.binop_error(other, "%"))
    }

    pub fn pow(&self, other: &Value) -> VResult<Value> {
        if let (Some(x), Some(y)) = (self.as_int(), other.as_int()) {
            if y >= 0 {
                if let Some(v) = u32::try_from(y).ok().and_then(|e| int_pow(x, e)) {
                    return Ok(Value::Int(v));
                }
                return Ok(Value::Float((x as f64).powf(y as f64)));
            }
            if x == 0 {
                return Err(Exc::zero_division("0.0 cannot be raised to a negative power"));
            }
            return Ok(Value::Float((x as f64).powf(y as f64)));
        }
        if self.is_num() && other.is_num() {
            let (a, b) = (self.as_f64().unwrap(), other.as_f64().unwrap());
            if a == 0.0 && b < 0.0 {
                return Err(Exc::zero_division("zero to a negative power"));
            }
            if a < 0.0 && b.fract() != 0.0 && b.is_finite() {
                // python returns a complex number here
                return Err(Exc::value_error("math domain error"));
            }
            return Ok(Value::Float(a.powf(b)));
        }
        Err(self.binop_error(other, "** or pow()"))
    }

    pub fn neg(&self) -> VResult<Value> {
        match self {
            Value::Bool(b) => Ok(Value::Int(-(*b as i64))),
            Value::Int(i) => Ok(i.checked_neg().map(Value::Int).unwrap_or(Value::Float(-(*i as f64)))),
            Value::Float(f) => Ok(Value::Float(-f)),
            Value::Undefined(u) => Err(Exc::undefined(u.message())),
            v => Err(Exc::type_error(format!("bad operand type for unary -: '{}'", v.type_name()))),
        }
    }

    pub fn pos(&self) -> VResult<Value> {
        match self {
            Value::Bool(b) => Ok(Value::Int(*b as i64)),
            Value::Int(_) | Value::Float(_) => Ok(self.clone()),
            Value::Undefined(u) => Err(Exc::undefined(u.message())),
            v => Err(Exc::type_error(format!("bad operand type for unary +: '{}'", v.type_name()))),
        }
    }

    /// len(v)
    pub fn len(&self) -> VResult<usize> {
        match self {
            Value::Str(s) => Ok(s.chars().count()),
            Value::List(l) => Ok(l.borrow().len()),
            Value::Tuple(t) => Ok(t.len()),
            Value::NamedTuple(nt) => Ok(nt.2.len()),
            Value::Dict(d) => Ok(d.borrow().len()),
            Value::Range(..) => Ok(self.range_len() as usize),
            Value::Undefined(_) => Ok(0),
            Value::Object(o) => o.len().ok_or_else(|| Exc::type_error(format!("object of type '{}' has no len()", o.type_name()))),
            v => Err(Exc::type_error(format!("object of type '{}' has no len()", v.type_name()))),
        }
    }

    /// list(iter(v))
    pub fn iter_items(&self) -> VResult<Vec<Value>> {
        match self {
            Value::Str(s) => Ok(s.chars().map(|c| Value::from(c.to_string())).collect()),
            Value::List(l) => Ok(l.borrow().clone()),
            Value::Tuple(t) => Ok(t.to_vec()),
            Value::NamedTuple(nt) => Ok(nt.2.clone()),
            Value::Dict(d) => Ok(d.borrow().keys().map(|k| k.0.clone()).collect()),
            Value::Range(a, b, c) => {
                let mut out = vec![];
                let mut i = *a;
                while (*c > 0 && i < *b) || (*c < 0 && i > *b) {
                    out.push(Value::Int(i));
                    i += c;
                }
                Ok(out)
            }
            Value::Undefined(_) => Ok(vec![]),
            Value::Object(o) => o.iter().unwrap_or_else(|| Err(Exc::type_error(format!("'{}' object is not iterable", o.type_name())))),
            v => Err(Exc::type_error(format!("'{}' object is not iterable", v.type_name()))),
        }
    }

    /// `needle in self`
    pub fn contains(&self, needle: &Value) -> VResult<bool> {
        match self {
            Value::Str(s) => match needle {
                Value::Str(n) => Ok(s.contains(&**n)),
                Value::Undefined(u) => Err(Exc::undefined(u.message())),
                n => Err(Exc::type_error(format!("'in <string>' requires string as left operand, not {}", n.type_name()))),
            },
            Value::Dict(d) => {
                let key = Key::hashable(needle)?;
                Ok(d.borrow().contains_key(&key))
            }
            Value::Object(o) => match o.contains(needle) {
                Some(r) => r,
                None => Ok(self.iter_items()?.iter().any(|v| v.py_eq(needle))),
            },
            Value::Undefined(u) => Err(Exc::undefined(u.message())),
            Value::None | Value::Bool(_) | Value::Int(_) | Value::Float(_) => {
                Err(Exc::type_error(format!("argument of type '{}' is not iterable", self.type_name())))
            }
            _ => Ok(self.iter_items()?.iter().any(|v| v.py_eq(needle))),
        }
    }

    /// a python value (str() / repr() / formatting argument) of this value
    pub fn to_pyval_shallow(&self) -> PyVal {
        match self {
            Value::None => PyVal::None,
            Value::Bool(b) => PyVal::Bool(*b),
            Value::Int(i) => PyVal::Int(*i),
            Value::Float(f) => PyVal::Float(*f),
            Value::Str(s) => PyVal::Str(s.to_string()),
            Value::Tuple(t) => PyVal::Tuple(t.iter().map(|v| v.to_pyval_shallow()).collect()),
            Value::List(l) => PyVal::List(l.borrow().iter().map(|v| v.to_pyval_shallow()).collect()),
            Value::Dict(d) => PyVal::Dict(d.borrow().iter().map(|(k, v)| (k.0.to_str(), v.to_pyval_shallow())).collect()),
            // other objects format through their str() / repr()
            v => PyVal::Str(v.to_str()),
        }
    }

    /// convert a status value (get_status() output) to a template value
    pub fn from_pyval(v: &PyVal) -> Value {
        match v {
            PyVal::None => Value::None,
            PyVal::Bool(b) => Value::Bool(*b),
            PyVal::Int(i) => Value::Int(*i),
            PyVal::Float(f) => Value::Float(*f),
            PyVal::Str(s) => Value::from(s.as_str()),
            PyVal::List(l) => Value::list(l.iter().map(Value::from_pyval).collect()),
            PyVal::Tuple(t) => Value::tuple(t.iter().map(Value::from_pyval).collect()),
            PyVal::NamedTuple(name, fields, values) => Value::NamedTuple(Rc::new((
                name.to_string(),
                fields.iter().map(|f| f.to_string()).collect(),
                values.iter().map(Value::from_pyval).collect(),
            ))),
            PyVal::Dict(d) => {
                Value::dict(d.iter().map(|(k, v)| (Key(Value::from(k.as_str())), Value::from_pyval(v))).collect())
            }
            PyVal::Shared(v) => Value::from_pyval(v),
        }
    }

    /// a template value as a python value (for klippy: variables, call
    /// arguments); objects become their str()
    pub fn to_pyval(&self) -> PyVal {
        match self {
            Value::NamedTuple(nt) => PyVal::Tuple(nt.2.iter().map(|v| v.to_pyval()).collect()),
            Value::Range(..) => PyVal::List(self.iter_items().unwrap_or_default().iter().map(|v| v.to_pyval()).collect()),
            v => v.to_pyval_shallow(),
        }
    }
}

impl Key {
    /// a dict key; TypeError for unhashable values
    pub fn hashable(v: &Value) -> VResult<Key> {
        match v {
            Value::List(_) | Value::Dict(_) | Value::Slice(_) => Err(Exc::type_error(format!("unhashable type: '{}'", v.type_name()))),
            Value::Tuple(t) => {
                for x in t.iter() {
                    Key::hashable(x)?;
                }
                Ok(Key(v.clone()))
            }
            _ => Ok(Key(v.clone())),
        }
    }

    pub fn str(s: &str) -> Key {
        Key(Value::from(s))
    }
}
