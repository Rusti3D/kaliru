// Runtime helper objects: generators, markupsafe.Markup, the environment
// globals (range, dict, lipsum, cycler, joiner, namespace) and builtin
// python callables
use super::eval::Namespace;
use super::value::{CallCtx, Exc, Key, Object, VResult, Value};
use indexmap::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

/// a python generator / iterator object: iterates once
pub struct Generator {
    /// the name in "<generator object NAME at 0x...>"
    pub name: String,
    /// the type name ("generator", "list_reverseiterator", ...)
    pub tname: String,
    pub items: RefCell<Option<Vec<Value>>>,
}

impl Generator {
    pub fn new(name: &str, items: Vec<Value>) -> Value {
        Value::object(Generator { name: name.into(), tname: "generator".into(), items: RefCell::new(Some(items)) })
    }
    pub fn iterator(tname: &str, items: Vec<Value>) -> Value {
        Value::object(Generator { name: String::new(), tname: tname.into(), items: RefCell::new(Some(items)) })
    }
}

impl Object for Generator {
    fn type_name(&self) -> String {
        self.tname.clone()
    }
    fn repr(&self) -> String {
        let addr = self as *const Generator as usize;
        if self.tname == "generator" {
            format!("<generator object {} at 0x{:x}>", self.name, addr)
        } else {
            format!("<{} object at 0x{:x}>", self.tname, addr)
        }
    }
    fn iter(&self) -> Option<VResult<Vec<Value>>> {
        Some(Ok(self.items.borrow_mut().take().unwrap_or_default()))
    }
    fn truthy(&self) -> bool {
        true
    }
}

/// markupsafe.Markup (a str subclass)
pub struct Markup(pub String);

impl Object for Markup {
    fn type_name(&self) -> String {
        "Markup".into()
    }
    fn module(&self) -> String {
        "markupsafe".into()
    }
    fn repr(&self) -> String {
        format!("Markup({})", crate::pyfmt::py_str_repr(&self.0))
    }
    fn str(&self) -> String {
        self.0.clone()
    }
    fn len(&self) -> Option<usize> {
        Some(self.0.chars().count())
    }
    fn iter(&self) -> Option<VResult<Vec<Value>>> {
        Some(Ok(self.0.chars().map(|c| Value::from(c.to_string())).collect()))
    }
    fn contains(&self, v: &Value) -> Option<VResult<bool>> {
        Some(Ok(self.0.contains(&v.to_str())))
    }
    fn getattr(&self, _me: &Value, name: &str) -> Option<Value> {
        super::methods::getattr(&Value::from(self.0.as_str()), name)
    }
}

/// markupsafe.escape()
pub fn escape(v: &Value) -> Value {
    if let Some(m) = v.as_object::<Markup>() {
        return Value::object(Markup(m.0.clone()));
    }
    let s = v.to_str();
    let mut out = String::new();
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '>' => out.push_str("&gt;"),
            '<' => out.push_str("&lt;"),
            '\'' => out.push_str("&#39;"),
            '"' => out.push_str("&#34;"),
            c => out.push(c),
        }
    }
    Value::object(Markup(out))
}

pub type BuiltinFnPtr = fn(&mut dyn CallCtx, Vec<Value>, IndexMap<String, Value>) -> VResult<Value>;

/// a builtin function / class (range, dict, math.sqrt, ...)
pub struct BuiltinFn {
    pub name: &'static str,
    /// "builtin_function_or_method" or "type"
    pub tname: &'static str,
    pub f: BuiltinFnPtr,
}

impl Object for BuiltinFn {
    fn type_name(&self) -> String {
        self.tname.into()
    }
    fn repr(&self) -> String {
        if self.tname == "type" {
            format!("<class '{}'>", self.name)
        } else {
            format!("<built-in function {}>", self.name)
        }
    }
    fn call(&self, ctx: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        Some((self.f)(ctx, args, kwargs))
    }
}

pub fn builtin(name: &'static str, tname: &'static str, f: BuiltinFnPtr) -> Value {
    Value::object(BuiltinFn { name, tname, f })
}

fn range_fn(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    if !kwargs.is_empty() {
        return Err(Exc::type_error("range() takes no keyword arguments"));
    }
    let ints: Vec<i64> = args
        .iter()
        .map(|a| {
            a.as_int()
                .ok_or_else(|| Exc::type_error(format!("'{}' object cannot be interpreted as an integer", a.type_name())))
        })
        .collect::<VResult<_>>()?;
    let (start, stop, step) = match ints.len() {
        1 => (0, ints[0], 1),
        2 => (ints[0], ints[1], 1),
        3 => (ints[0], ints[1], ints[2]),
        0 => return Err(Exc::type_error("range expected at least 1 argument, got 0")),
        n => return Err(Exc::type_error(format!("range expected at most 3 arguments, got {}", n))),
    };
    if step == 0 {
        return Err(Exc::value_error("range() arg 3 must not be zero"));
    }
    Ok(Value::Range(start, stop, step))
}

fn dict_fn(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    if args.len() > 1 {
        return Err(Exc::type_error(format!("dict expected at most 1 argument, got {}", args.len())));
    }
    let d = Value::dict(IndexMap::new());
    super::methods::call_method(&d, "update", args, kwargs)?;
    Ok(d)
}

fn namespace_fn(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let mut attrs = IndexMap::new();
    for a in args {
        match a {
            Value::Dict(d) => {
                for (k, v) in d.borrow().iter() {
                    attrs.insert(k.0.to_str(), v.clone());
                }
            }
            other => {
                for item in other.iter_items()? {
                    let pair = item.iter_items()?;
                    if pair.len() == 2 {
                        attrs.insert(pair[0].to_str(), pair[1].clone());
                    }
                }
            }
        }
    }
    attrs.extend(kwargs);
    Ok(Value::object(Namespace { attrs: RefCell::new(attrs) }))
}

/// utils.Cycler
pub struct Cycler {
    items: Vec<Value>,
    pos: RefCell<usize>,
}

impl Object for Cycler {
    fn type_name(&self) -> String {
        "Cycler".into()
    }
    fn module(&self) -> String {
        "jinja2.utils".into()
    }
    fn getattr(&self, me: &Value, name: &str) -> Option<Value> {
        match name {
            "current" => Some(self.items[*self.pos.borrow()].clone()),
            "items" => Some(Value::tuple(self.items.clone())),
            "next" | "reset" | "__next__" => Some(Value::object(CyclerMethod { cy: me.clone(), name: name.to_string() })),
            _ => None,
        }
    }
}

struct CyclerMethod {
    cy: Value,
    name: String,
}

impl Object for CyclerMethod {
    fn type_name(&self) -> String {
        "method".into()
    }
    fn call(&self, _ctx: &mut dyn CallCtx, _args: Vec<Value>, _kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        let c = self.cy.as_object::<Cycler>()?;
        if self.name == "reset" {
            *c.pos.borrow_mut() = 0;
            return Some(Ok(Value::None));
        }
        let mut pos = c.pos.borrow_mut();
        let rv = c.items[*pos].clone();
        *pos = (*pos + 1) % c.items.len();
        Some(Ok(rv))
    }
}

fn cycler_fn(_: &mut dyn CallCtx, args: Vec<Value>, _kwargs: IndexMap<String, Value>) -> VResult<Value> {
    if args.is_empty() {
        return Err(Exc::runtime("at least one item has to be provided"));
    }
    Ok(Value::object(Cycler { items: args, pos: RefCell::new(0) }))
}

/// utils.Joiner
pub struct Joiner {
    sep: String,
    used: RefCell<bool>,
}

impl Object for Joiner {
    fn type_name(&self) -> String {
        "Joiner".into()
    }
    fn module(&self) -> String {
        "jinja2.utils".into()
    }
    fn call(&self, _ctx: &mut dyn CallCtx, _args: Vec<Value>, _kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        let mut used = self.used.borrow_mut();
        if !*used {
            *used = true;
            return Some(Ok(Value::from("")));
        }
        Some(Ok(Value::from(self.sep.as_str())))
    }
}

fn joiner_fn(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let sep = args.first().or_else(|| kwargs.get("sep")).map(|v| v.to_str()).unwrap_or_else(|| ", ".into());
    Ok(Value::object(Joiner { sep, used: RefCell::new(false) }))
}

fn lipsum_fn(_: &mut dyn CallCtx, _args: Vec<Value>, _kwargs: IndexMap<String, Value>) -> VResult<Value> {
    // generate_lorem_ipsum() output is random
    Ok(Value::object(Markup("Lorem ipsum dolor sit amet.".into())))
}

/// jinja2.defaults.DEFAULT_NAMESPACE
pub fn default_namespace() -> IndexMap<String, Value> {
    let mut g = IndexMap::new();
    g.insert("range".into(), builtin("range", "type", range_fn));
    g.insert("dict".into(), builtin("dict", "type", dict_fn));
    g.insert("lipsum".into(), builtin("generate_lorem_ipsum", "function", lipsum_fn));
    g.insert("cycler".into(), builtin("Cycler", "type", cycler_fn));
    g.insert("joiner".into(), builtin("Joiner", "type", joiner_fn));
    g.insert("namespace".into(), builtin("Namespace", "type", namespace_fn));
    g
}

/// a python module object (math)
pub struct Module {
    pub name: &'static str,
    pub attrs: IndexMap<&'static str, Value>,
}

impl Object for Module {
    fn type_name(&self) -> String {
        "module".into()
    }
    fn repr(&self) -> String {
        format!("<module '{}' (built-in)>", self.name)
    }
    fn getattr(&self, _me: &Value, name: &str) -> Option<Value> {
        self.attrs.get(name).cloned()
    }
}

/// a dict key for a string
pub fn skey(s: &str) -> Key {
    Key::str(s)
}

pub fn rc_list(items: Vec<Value>) -> Rc<RefCell<Vec<Value>>> {
    Rc::new(RefCell::new(items))
}
