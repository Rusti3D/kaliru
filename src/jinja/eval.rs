// Template evaluation: what the code jinja2's compiler generates does,
// run directly on the AST.
//
// Scoping follows the compiler's frames: the template body is the root
// frame; for-loop iterations, macro calls, with / filter / set blocks get a
// fresh inner frame (stores stay local, loads fall through to the outer
// frames and then the context); if blocks share their frame.
use super::nodes::*;
use super::value::{CallCtx, Exc, Key, Object, VResult, Value};
use super::{filters, tests};
use indexmap::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

pub type Frame = Rc<RefCell<IndexMap<String, Value>>>;

fn new_frame() -> Frame {
    Rc::new(RefCell::new(IndexMap::new()))
}

/// a macro (or the caller of a call block)
pub struct MacroObj {
    pub def: Rc<MacroDef>,
    /// the frames visible where the macro was defined
    pub scopes: Vec<Frame>,
    pub catch_kwargs: bool,
    pub catch_varargs: bool,
    pub caller: bool,
    pub explicit_caller: bool,
}

impl Object for MacroObj {
    fn type_name(&self) -> String {
        "Macro".into()
    }
    fn module(&self) -> String {
        "jinja2.runtime".into()
    }
    fn repr(&self) -> String {
        format!("<Macro {}>", crate::pyfmt::py_str_repr(&self.def.name))
    }
    fn getattr(&self, _me: &Value, name: &str) -> Option<Value> {
        match name {
            "name" => Some(Value::from(self.def.name.as_str())),
            "arguments" => Some(Value::tuple(self.def.args.iter().map(|a| Value::from(a.as_str())).collect())),
            "catch_kwargs" => Some(Value::Bool(self.catch_kwargs)),
            "catch_varargs" => Some(Value::Bool(self.catch_varargs)),
            "caller" => Some(Value::Bool(self.caller)),
            _ => None,
        }
    }
    fn call(&self, ctx: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        Some(ctx.call_macro(self, args, kwargs))
    }
}

/// runtime.LoopContext
pub struct LoopCtx {
    items: Vec<Value>,
    index0: RefCell<usize>,
    depth0: usize,
    last_changed: RefCell<Option<Vec<Value>>>,
    /// the loop body for recursive loops: (target, body, test)
    recurse: Option<Rc<(Expr, Vec<Stmt>, Option<Expr>, Vec<Frame>)>>,
}

impl Object for LoopCtx {
    fn type_name(&self) -> String {
        "LoopContext".into()
    }
    fn module(&self) -> String {
        "jinja2.runtime".into()
    }
    fn repr(&self) -> String {
        format!("<LoopContext {}/{}>", *self.index0.borrow() + 1, self.items.len())
    }
    fn getattr(&self, me: &Value, name: &str) -> Option<Value> {
        let i = *self.index0.borrow();
        let n = self.items.len();
        Some(match name {
            "index0" => Value::Int(i as i64),
            "index" => Value::Int(i as i64 + 1),
            "revindex0" => Value::Int((n - i - 1) as i64),
            "revindex" => Value::Int((n - i) as i64),
            "first" => Value::Bool(i == 0),
            "last" => Value::Bool(i + 1 == n),
            "length" => Value::Int(n as i64),
            "depth0" => Value::Int(self.depth0 as i64),
            "depth" => Value::Int(self.depth0 as i64 + 1),
            "previtem" => {
                if i == 0 {
                    Value::undefined(Some("there is no previous item".into()), None, Some(Value::from("previtem")))
                } else {
                    self.items[i - 1].clone()
                }
            }
            "nextitem" => {
                if i + 1 >= n {
                    Value::undefined(Some("there is no next item".into()), None, Some(Value::from("nextitem")))
                } else {
                    self.items[i + 1].clone()
                }
            }
            "cycle" | "changed" => Value::object(LoopMethod { lp: me.clone(), name: name.to_string() }),
            _ => return None,
        })
    }
    fn call(&self, ctx: &mut dyn CallCtx, args: Vec<Value>, _kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        let Some(rec) = &self.recurse else {
            return Some(Err(Exc::type_error(
                "The loop must have the 'recursive' marker to be called recursively.",
            )));
        };
        if args.len() != 1 {
            return Some(Err(Exc::type_error(format!("loop() takes 1 positional argument but {} were given", args.len()))));
        }
        let Some(r) = ctx.downcast_mut::<Renderer>() else {
            return Some(Err(Exc::runtime("loop called outside of a template")));
        };
        Some(r.render_loop_recursive(rec.clone(), &args[0], self.depth0 + 1))
    }
}

struct LoopMethod {
    lp: Value,
    name: String,
}

impl Object for LoopMethod {
    fn type_name(&self) -> String {
        "method".into()
    }
    fn call(&self, _ctx: &mut dyn CallCtx, args: Vec<Value>, _kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        let lp = self.lp.as_object::<LoopCtx>()?;
        if self.name == "cycle" {
            if args.is_empty() {
                return Some(Err(Exc::type_error("no items for cycling given")));
            }
            let i = *lp.index0.borrow();
            return Some(Ok(args[i % args.len()].clone()));
        }
        // changed(*value)
        let mut last = lp.last_changed.borrow_mut();
        let changed = match &*last {
            Some(prev) => !(prev.len() == args.len() && prev.iter().zip(&args).all(|(a, b)| a.py_eq(b))),
            None => true,
        };
        if changed {
            *last = Some(args);
        }
        Some(Ok(Value::Bool(changed)))
    }
}

/// runtime's Namespace
pub struct Namespace {
    pub attrs: RefCell<IndexMap<String, Value>>,
}

impl Object for Namespace {
    fn type_name(&self) -> String {
        "Namespace".into()
    }
    fn module(&self) -> String {
        "jinja2.utils".into()
    }
    fn repr(&self) -> String {
        let items: Vec<String> = self.attrs.borrow().iter().map(|(k, v)| format!("{}={}", k, v.repr())).collect();
        format!("<Namespace {{{}}}>", items.join(", "))
    }
    fn getattr(&self, _me: &Value, name: &str) -> Option<Value> {
        self.attrs.borrow().get(name).cloned()
    }
    fn setitem(&self, key: &str, value: Value) -> Option<VResult<()>> {
        self.attrs.borrow_mut().insert(key.to_string(), value);
        Some(Ok(()))
    }
}

/// getattr(obj, name) (None: AttributeError)
pub fn py_getattr(obj: &Value, name: &str) -> Option<Value> {
    match obj {
        Value::Object(o) => o.getattr(obj, name),
        _ => super::methods::getattr(obj, name),
    }
}

fn index_of(len: usize, key: &Value, what: &str) -> VResult<usize> {
    let i = key.as_int().ok_or_else(|| {
        if what == "string" {
            Exc::type_error(format!("string indices must be integers, not '{}'", key.type_name()))
        } else {
            Exc::type_error(format!("{} indices must be integers or slices, not {}", what, key.type_name()))
        }
    })?;
    let n = len as i64;
    let j = if i < 0 { i + n } else { i };
    if j < 0 || j >= n {
        return Err(Exc::index_error(format!("{} index out of range", what)));
    }
    Ok(j as usize)
}

/// slice indices (start, stop, step) of a sequence of `len` items
pub fn slice_indices(len: usize, s: &(Value, Value, Value)) -> VResult<Vec<usize>> {
    let get = |v: &Value| -> VResult<Option<i64>> {
        match v {
            Value::None => Ok(None),
            v => v
                .as_int()
                .map(Some)
                .ok_or_else(|| Exc::type_error("slice indices must be integers or None or have an __index__ method")),
        }
    };
    let step = get(&s.2)?.unwrap_or(1);
    if step == 0 {
        return Err(Exc::value_error("slice step cannot be zero"));
    }
    let n = len as i64;
    let clamp = |v: Option<i64>, default: i64, lo: i64, hi: i64| -> i64 {
        match v {
            None => default,
            Some(i) => {
                let i = if i < 0 { i + n } else { i };
                i.clamp(lo, hi)
            }
        }
    };
    let (start, stop) = if step > 0 {
        (clamp(get(&s.0)?, 0, 0, n), clamp(get(&s.1)?, n, 0, n))
    } else {
        (clamp(get(&s.0)?, n - 1, -1, n - 1), clamp(get(&s.1)?, -1, -1, n - 1))
    };
    let mut out = vec![];
    let mut i = start;
    while (step > 0 && i < stop) || (step < 0 && i > stop) {
        out.push(i as usize);
        i += step;
    }
    Ok(out)
}

/// obj[key]
pub fn py_getitem(obj: &Value, key: &Value) -> VResult<Value> {
    if let Value::Undefined(u) = obj {
        return Err(Exc::undefined(u.message()));
    }
    if let Value::Slice(s) = key {
        let pick = |items: &[Value]| -> VResult<Vec<Value>> { Ok(slice_indices(items.len(), s)?.into_iter().map(|i| items[i].clone()).collect()) };
        return match obj {
            Value::Str(st) => {
                let chars: Vec<Value> = st.chars().map(|c| Value::from(c.to_string())).collect();
                Ok(Value::from(pick(&chars)?.iter().map(|v| v.to_str()).collect::<String>()))
            }
            Value::List(l) => Ok(Value::list(pick(&l.borrow())?)),
            Value::Tuple(t) => Ok(Value::tuple(pick(t)?)),
            Value::NamedTuple(nt) => Ok(Value::tuple(pick(&nt.2)?)),
            Value::Range(..) => Ok(Value::list(pick(&obj.iter_items()?)?)),
            Value::Dict(_) => Err(Exc::type_error("unhashable type: 'slice'")),
            v => Err(Exc::type_error(format!("'{}' object is not subscriptable", v.type_name()))),
        };
    }
    match obj {
        Value::List(l) => {
            let l = l.borrow();
            Ok(l[index_of(l.len(), key, "list")?].clone())
        }
        Value::Tuple(t) => Ok(t[index_of(t.len(), key, "tuple")?].clone()),
        Value::NamedTuple(nt) => Ok(nt.2[index_of(nt.2.len(), key, "tuple")?].clone()),
        Value::Str(s) => {
            let chars: Vec<char> = s.chars().collect();
            Ok(Value::from(chars[index_of(chars.len(), key, "string")?].to_string()))
        }
        Value::Range(..) => {
            let items = obj.iter_items()?;
            Ok(items[index_of(items.len(), key, "range object")?].clone())
        }
        Value::Dict(d) => {
            let k = Key::hashable(key)?;
            d.borrow().get(&k).cloned().ok_or_else(|| Exc::key_error(key))
        }
        Value::Object(o) => match o.getitem(key) {
            Some(r) => r,
            None => Err(Exc::type_error(format!("'{}' object is not subscriptable", o.type_name()))),
        },
        v => Err(Exc::type_error(format!("'{}' object is not subscriptable", v.type_name()))),
    }
}

/// environment.getattr(obj, attribute)
pub fn env_getattr(obj: &Value, attr: &str) -> VResult<Value> {
    if let Value::Undefined(u) = obj {
        return Err(Exc::undefined(u.message()));
    }
    if let Some(v) = py_getattr(obj, attr) {
        return Ok(v);
    }
    match py_getitem(obj, &Value::from(attr)) {
        Ok(v) => Ok(v),
        Err(e) if e.cls == "TypeError" || e.is_lookup() || e.cls == "AttributeError" => {
            Ok(Value::undefined(None, Some(obj.clone()), Some(Value::from(attr))))
        }
        Err(e) => Err(e),
    }
}

/// environment.getitem(obj, argument)
pub fn env_getitem(obj: &Value, arg: &Value) -> VResult<Value> {
    if let Value::Undefined(u) = obj {
        return Err(Exc::undefined(u.message()));
    }
    match py_getitem(obj, arg) {
        Ok(v) => Ok(v),
        Err(e) if e.cls == "TypeError" || e.is_lookup() || e.cls == "AttributeError" => {
            if let Value::Str(s) = arg {
                if let Some(v) = py_getattr(obj, s) {
                    return Ok(v);
                }
            }
            Ok(Value::undefined(None, Some(obj.clone()), Some(arg.clone())))
        }
        Err(e) => Err(e),
    }
}

enum Flow {
    Normal,
    Break,
    Continue,
}

/// a template render in progress
pub struct Renderer {
    /// the render context (template arguments + environment globals)
    pub context: Rc<IndexMap<String, Value>>,
    scopes: Vec<Frame>,
    out: Vec<String>,
}

fn uses_name(stmts: &[Stmt], name: &str) -> bool {
    let mut names = vec![];
    for s in stmts {
        stmt_names(s, &mut names);
    }
    names.iter().any(|n| n == name)
}

fn expr_names(e: &Expr, out: &mut Vec<String>) {
    match &e.kind {
        ExprKind::Name(n) => out.push(n.clone()),
        ExprKind::NSRef(n, _) => out.push(n.clone()),
        ExprKind::Const(_) => {}
        ExprKind::Tuple(v) | ExprKind::List(v) | ExprKind::Concat(v) => v.iter().for_each(|x| expr_names(x, out)),
        ExprKind::Dict(v) => v.iter().for_each(|(k, x)| {
            expr_names(k, out);
            expr_names(x, out)
        }),
        ExprKind::BinOp(_, a, b) | ExprKind::And(a, b) | ExprKind::Or(a, b) => {
            expr_names(a, out);
            expr_names(b, out)
        }
        ExprKind::Not(a) | ExprKind::Neg(a) | ExprKind::Pos(a) => expr_names(a, out),
        ExprKind::Compare(a, ops) => {
            expr_names(a, out);
            ops.iter().for_each(|(_, x)| expr_names(x, out))
        }
        ExprKind::Getattr(a, _) => expr_names(a, out),
        ExprKind::Getitem(a, b) => {
            expr_names(a, out);
            expr_names(b, out)
        }
        ExprKind::Slice(a, b, c) => [a, b, c].iter().for_each(|x| {
            if let Some(x) = x {
                expr_names(x, out)
            }
        }),
        ExprKind::Filter(a, _, ca) => {
            if let Some(a) = a {
                expr_names(a, out);
            }
            ca_names(ca, out)
        }
        ExprKind::Test(a, _, ca) => {
            expr_names(a, out);
            ca_names(ca, out)
        }
        ExprKind::CondExpr(a, b, c) => {
            expr_names(a, out);
            expr_names(b, out);
            if let Some(c) = c {
                expr_names(c, out)
            }
        }
        ExprKind::Call(a, ca) => {
            expr_names(a, out);
            ca_names(ca, out)
        }
    }
}

fn ca_names(ca: &CallArgs, out: &mut Vec<String>) {
    ca.args.iter().for_each(|x| expr_names(x, out));
    ca.kwargs.iter().for_each(|k| expr_names(&k.value, out));
    if let Some(x) = &ca.dyn_args {
        expr_names(x, out);
    }
    if let Some(x) = &ca.dyn_kwargs {
        expr_names(x, out);
    }
}

fn stmt_names(s: &Stmt, out: &mut Vec<String>) {
    let block = |b: &[Stmt], out: &mut Vec<String>| b.iter().for_each(|x| stmt_names(x, out));
    match &s.kind {
        StmtKind::Output(nodes) => nodes.iter().for_each(|n| {
            if let OutputNode::Expr(e) = n {
                expr_names(e, out)
            }
        }),
        StmtKind::For { target, iter, body, else_, test, .. } => {
            expr_names(target, out);
            expr_names(iter, out);
            block(body, out);
            block(else_, out);
            if let Some(t) = test {
                expr_names(t, out);
            }
        }
        StmtKind::If { test, body, elif_, else_ } => {
            expr_names(test, out);
            block(body, out);
            for (t, b) in elif_ {
                expr_names(t, out);
                block(b, out);
            }
            block(else_, out);
        }
        // macro bodies are their own scope
        StmtKind::Macro(_) => {}
        StmtKind::CallBlock { call, .. } => expr_names(call, out),
        StmtKind::FilterBlock { filter, body } => {
            expr_names(filter, out);
            block(body, out)
        }
        StmtKind::With { targets, values, body } => {
            targets.iter().chain(values.iter()).for_each(|x| expr_names(x, out));
            block(body, out)
        }
        StmtKind::Assign { target, node } => {
            expr_names(target, out);
            expr_names(node, out)
        }
        StmtKind::AssignBlock { target, filter, body } => {
            expr_names(target, out);
            if let Some(f) = filter {
                expr_names(f, out);
            }
            block(body, out)
        }
        StmtKind::ExprStmt(e) => expr_names(e, out),
        StmtKind::Scope(b) | StmtKind::Block { body: b, .. } => block(b, out),
        StmtKind::Extends(e) | StmtKind::Include { template: e, .. } | StmtKind::Import { template: e, .. } => expr_names(e, out),
        StmtKind::FromImport { template, .. } => expr_names(template, out),
        StmtKind::Break | StmtKind::Continue => {}
    }
}

impl CallCtx for Renderer {
    fn call_macro(&mut self, m: &MacroObj, args: Vec<Value>, mut kwargs: IndexMap<String, Value>) -> VResult<Value> {
        let def = &m.def;
        let argument_count = def.args.len();
        let mut arguments: Vec<Option<Value>> = args.iter().take(argument_count).cloned().map(Some).collect();
        let off = arguments.len();
        let mut found_caller = false;
        if off != argument_count {
            for name in &def.args[off..] {
                let value = kwargs.shift_remove(name);
                if name == "caller" {
                    found_caller = true;
                }
                arguments.push(value);
            }
        } else {
            found_caller = m.explicit_caller;
        }
        let mut caller = None;
        if m.caller && !found_caller {
            caller = Some(kwargs.shift_remove("caller").filter(|c| !matches!(c, Value::None)).unwrap_or_else(|| {
                Value::undefined(Some("No caller defined".into()), None, Some(Value::from("caller")))
            }));
        }
        let mut kw_value = None;
        if m.catch_kwargs {
            kw_value = Some(Value::dict(kwargs.into_iter().map(|(k, v)| (Key::str(&k), v)).collect()));
        } else if let Some((k, _)) = kwargs.first() {
            if kwargs.contains_key("caller") {
                return Err(Exc::type_error(format!(
                    "macro {} was invoked with two values for the special caller argument. This is most likely a bug.",
                    crate::pyfmt::py_str_repr(&def.name)
                )));
            }
            return Err(Exc::type_error(format!(
                "macro {} takes no keyword argument {}",
                crate::pyfmt::py_str_repr(&def.name),
                crate::pyfmt::py_str_repr(k)
            )));
        }
        let mut varargs = None;
        if m.catch_varargs {
            varargs = Some(Value::tuple(args.iter().skip(argument_count).cloned().collect()));
        } else if args.len() > argument_count {
            return Err(Exc::type_error(format!(
                "macro {} takes not more than {} argument(s)",
                crate::pyfmt::py_str_repr(&def.name),
                def.args.len()
            )));
        }
        // the macro body frame
        let frame = new_frame();
        let saved = std::mem::replace(&mut self.scopes, m.scopes.clone());
        self.scopes.push(frame.clone());
        let res = (|| -> VResult<String> {
            if let Some(c) = caller {
                frame.borrow_mut().insert("caller".into(), c);
            }
            if let Some(k) = kw_value {
                frame.borrow_mut().insert("kwargs".into(), k);
            }
            if let Some(v) = varargs {
                frame.borrow_mut().insert("varargs".into(), v);
            }
            let ndefaults = def.defaults.len();
            for (i, name) in def.args.iter().enumerate() {
                let v = match arguments.get(i).cloned().flatten() {
                    Some(v) => v,
                    None => {
                        let di = i as i64 - (def.args.len() - ndefaults) as i64;
                        if di >= 0 {
                            self.eval(&def.defaults[di as usize])?
                        } else {
                            Value::undefined(Some(format!("parameter {} was not provided", crate::pyfmt::py_str_repr(name))), None, Some(Value::from(name.as_str())))
                        }
                    }
                };
                frame.borrow_mut().insert(name.clone(), v);
            }
            self.out.push(String::new());
            let r = self.block(&def.body);
            let text = self.out.pop().unwrap_or_default();
            r?;
            Ok(text)
        })();
        self.scopes = saved;
        Ok(Value::from(res?))
    }

}

impl Renderer {
    pub fn new(context: IndexMap<String, Value>) -> Self {
        Renderer { context: Rc::new(context), scopes: vec![new_frame()], out: vec![String::new()] }
    }

    /// render the template body; the output text
    pub fn render(&mut self, body: &[Stmt]) -> VResult<String> {
        self.block(body)?;
        Ok(self.out.pop().unwrap_or_default())
    }

    fn write(&mut self, s: &str) {
        if let Some(o) = self.out.last_mut() {
            o.push_str(s);
        }
    }

    fn lookup(&self, name: &str) -> Value {
        for f in self.scopes.iter().rev() {
            if let Some(v) = f.borrow().get(name) {
                return v.clone();
            }
        }
        if let Some(v) = self.context.get(name) {
            return v.clone();
        }
        Value::undefined(None, None, Some(Value::from(name)))
    }

    fn set_local(&mut self, name: &str, v: Value) {
        if let Some(f) = self.scopes.last() {
            f.borrow_mut().insert(name.to_string(), v);
        }
    }

    fn block(&mut self, stmts: &[Stmt]) -> VResult<Flow> {
        for s in stmts {
            match self.stmt(s)? {
                Flow::Normal => {}
                f => return Ok(f),
            }
        }
        Ok(Flow::Normal)
    }

    /// run `stmts` in a fresh inner frame; captured output when `capture`
    fn inner(&mut self, stmts: &[Stmt], capture: bool) -> VResult<(Flow, String)> {
        self.scopes.push(new_frame());
        if capture {
            self.out.push(String::new());
        }
        let r = self.block(stmts);
        let text = if capture { self.out.pop().unwrap_or_default() } else { String::new() };
        self.scopes.pop();
        Ok((r?, text))
    }

    fn make_macro(&self, def: &Rc<MacroDef>, explicit_caller: bool) -> Value {
        let body_uses = |n: &str| uses_name(&def.body, n) || def.defaults.iter().any(|d| {
            let mut v = vec![];
            expr_names(d, &mut v);
            v.iter().any(|x| x == n)
        });
        let declared = |n: &str| def.args.iter().any(|a| a == n);
        Value::object(MacroObj {
            def: def.clone(),
            scopes: self.scopes.clone(),
            catch_kwargs: body_uses("kwargs") && !declared("kwargs"),
            catch_varargs: body_uses("varargs") && !declared("varargs"),
            caller: body_uses("caller") && !declared("caller"),
            explicit_caller,
        })
    }

    fn assign(&mut self, target: &Expr, value: Value) -> VResult<()> {
        match &target.kind {
            ExprKind::Name(n) => {
                self.set_local(n, value);
                Ok(())
            }
            ExprKind::NSRef(n, attr) => {
                let ns = self.lookup(n);
                match &ns {
                    Value::Object(o) if o.is::<Namespace>() => {
                        o.setitem(attr, value);
                        Ok(())
                    }
                    _ => Err(Exc::runtime("cannot assign attribute on non-namespace object")),
                }
            }
            ExprKind::Tuple(items) => {
                let values = value.iter_items()?;
                if values.len() < items.len() {
                    return Err(Exc::value_error(format!(
                        "not enough values to unpack (expected {}, got {})",
                        items.len(),
                        values.len()
                    )));
                }
                if values.len() > items.len() {
                    return Err(Exc::value_error(format!("too many values to unpack (expected {})", items.len())));
                }
                for (t, v) in items.iter().zip(values) {
                    self.assign(t, v)?;
                }
                Ok(())
            }
            _ => Err(Exc::runtime("invalid assignment target")),
        }
    }

    fn stmt(&mut self, s: &Stmt) -> VResult<Flow> {
        match &s.kind {
            StmtKind::Output(nodes) => {
                for n in nodes {
                    match n {
                        OutputNode::Data(d) => self.write(d),
                        OutputNode::Expr(e) => {
                            let v = self.eval(e)?;
                            let s = v.to_str();
                            self.write(&s);
                        }
                    }
                }
            }
            StmtKind::For { target, iter, body, else_, test, recursive } => {
                let it = self.eval(iter)?;
                let rec = recursive.then(|| Rc::new((target.clone(), body.clone(), test.clone(), self.scopes.clone())));
                let iterated = self.run_loop(target, &it, body, test.as_ref(), rec, 0)?;
                if !iterated && !else_.is_empty() {
                    let (flow, _) = self.inner(else_, false)?;
                    if !matches!(flow, Flow::Normal) {
                        return Ok(flow);
                    }
                }
            }
            StmtKind::If { test, body, elif_, else_ } => {
                if self.eval(test)?.truthy() {
                    return self.block(body);
                }
                for (t, b) in elif_ {
                    if self.eval(t)?.truthy() {
                        return self.block(b);
                    }
                }
                return self.block(else_);
            }
            StmtKind::Macro(def) => {
                let m = self.make_macro(def, false);
                self.set_local(&def.name, m);
            }
            StmtKind::CallBlock { call, def } => {
                let caller = self.make_macro(def, false);
                let v = self.eval_call(call, Some(caller))?;
                let s = v.to_str();
                self.write(&s);
            }
            StmtKind::FilterBlock { filter, body } => {
                self.scopes.push(new_frame());
                self.out.push(String::new());
                let r = self.block(body);
                let text = self.out.pop().unwrap_or_default();
                let res = r.and_then(|_| self.eval_filter(filter, Some(Value::from(text))));
                self.scopes.pop();
                let s = res?.to_str();
                self.write(&s);
            }
            StmtKind::With { targets, values, body } => {
                let mut vals = vec![];
                for v in values {
                    vals.push(self.eval(v)?);
                }
                self.scopes.push(new_frame());
                let r = (|| {
                    for (t, v) in targets.iter().zip(vals) {
                        self.assign(t, v)?;
                    }
                    self.block(body)
                })();
                self.scopes.pop();
                let flow = r?;
                if !matches!(flow, Flow::Normal) {
                    return Ok(flow);
                }
            }
            StmtKind::Assign { target, node } => {
                let v = self.eval(node)?;
                self.assign(target, v)?;
            }
            StmtKind::AssignBlock { target, filter, body } => {
                self.scopes.push(new_frame());
                self.out.push(String::new());
                let r = self.block(body);
                let text = self.out.pop().unwrap_or_default();
                let res = r.and_then(|_| match filter {
                    Some(f) => self.eval_filter(f, Some(Value::from(text))),
                    None => Ok(Value::from(text)),
                });
                self.scopes.pop();
                self.assign(target, res?)?;
            }
            StmtKind::ExprStmt(e) => {
                self.eval(e)?;
            }
            StmtKind::Break => return Ok(Flow::Break),
            StmtKind::Continue => return Ok(Flow::Continue),
            StmtKind::Scope(body) => {
                let (flow, _) = self.inner(body, false)?;
                return Ok(flow);
            }
            StmtKind::Block { body, .. } => {
                let (flow, _) = self.inner(body, false)?;
                return Ok(flow);
            }
            StmtKind::Extends(_) | StmtKind::Include { .. } | StmtKind::Import { .. } | StmtKind::FromImport { .. } => {
                // the environment has no loader
                return Err(Exc::new("TypeError", "no loader for this environment specified"));
            }
        }
        Ok(Flow::Normal)
    }

    /// run a for loop; whether the body ran at least once
    fn run_loop(
        &mut self,
        target: &Expr,
        it: &Value,
        body: &[Stmt],
        test: Option<&Expr>,
        rec: Option<Rc<(Expr, Vec<Stmt>, Option<Expr>, Vec<Frame>)>>,
        depth0: usize,
    ) -> VResult<bool> {
        let mut items = it.iter_items()?;
        if let Some(t) = test {
            // the loop filter runs in its own frame per item
            let mut kept = vec![];
            for item in items {
                self.scopes.push(new_frame());
                let r = self.assign(target, item.clone()).and_then(|_| self.eval(t));
                self.scopes.pop();
                if r?.truthy() {
                    kept.push(item);
                }
            }
            items = kept;
        }
        let lp = Rc::new(LoopCtx {
            items: items.clone(),
            index0: RefCell::new(0),
            depth0,
            last_changed: RefCell::new(None),
            recurse: rec,
        });
        let loop_value = Value::Object(lp.clone());
        let mut iterated = false;
        for (i, item) in items.into_iter().enumerate() {
            iterated = true;
            *lp.index0.borrow_mut() = i;
            self.scopes.push(new_frame());
            let r = (|| {
                self.set_local("loop", loop_value.clone());
                self.assign(target, item)?;
                self.block(body)
            })();
            self.scopes.pop();
            match r? {
                Flow::Break => break,
                Flow::Continue | Flow::Normal => {}
            }
        }
        Ok(iterated)
    }

    fn render_loop_recursive(&mut self, rec: Rc<(Expr, Vec<Stmt>, Option<Expr>, Vec<Frame>)>, it: &Value, depth0: usize) -> VResult<Value> {
        let saved = std::mem::replace(&mut self.scopes, rec.3.clone());
        self.out.push(String::new());
        let r = self.run_loop(&rec.0, it, &rec.1, rec.2.as_ref(), Some(rec.clone()), depth0);
        let text = self.out.pop().unwrap_or_default();
        self.scopes = saved;
        r?;
        Ok(Value::from(text))
    }

    // ------------------------------------------------------------ expressions

    pub fn eval(&mut self, e: &Expr) -> VResult<Value> {
        match &e.kind {
            ExprKind::Name(n) => Ok(self.lookup(n)),
            ExprKind::NSRef(n, a) => env_getattr(&self.lookup(n), a),
            ExprKind::Const(v) => Ok(v.clone()),
            ExprKind::Tuple(items) => {
                let mut v = vec![];
                for i in items {
                    v.push(self.eval(i)?);
                }
                Ok(Value::tuple(v))
            }
            ExprKind::List(items) => {
                let mut v = vec![];
                for i in items {
                    v.push(self.eval(i)?);
                }
                Ok(Value::list(v))
            }
            ExprKind::Dict(items) => {
                let mut d = IndexMap::new();
                for (k, v) in items {
                    let k = self.eval(k)?;
                    let v = self.eval(v)?;
                    d.insert(Key::hashable(&k)?, v);
                }
                Ok(Value::dict(d))
            }
            ExprKind::BinOp(op, a, b) => {
                let a = self.eval(a)?;
                let b = self.eval(b)?;
                match op {
                    BinOp::Add => a.add(&b),
                    BinOp::Sub => a.sub(&b),
                    BinOp::Mul => a.mul(&b),
                    BinOp::Div => a.truediv(&b),
                    BinOp::FloorDiv => a.floordiv(&b),
                    BinOp::Mod => a.modulo(&b),
                    BinOp::Pow => a.pow(&b),
                }
            }
            ExprKind::And(a, b) => {
                let a = self.eval(a)?;
                if !a.truthy() {
                    return Ok(a);
                }
                self.eval(b)
            }
            ExprKind::Or(a, b) => {
                let a = self.eval(a)?;
                if a.truthy() {
                    return Ok(a);
                }
                self.eval(b)
            }
            ExprKind::Not(a) => Ok(Value::Bool(!self.eval(a)?.truthy())),
            ExprKind::Neg(a) => self.eval(a)?.neg(),
            ExprKind::Pos(a) => self.eval(a)?.pos(),
            ExprKind::Concat(items) => {
                let mut s = String::new();
                for i in items {
                    s.push_str(&self.eval(i)?.to_str());
                }
                Ok(Value::from(s))
            }
            ExprKind::Compare(first, ops) => {
                let mut left = self.eval(first)?;
                for (op, right) in ops {
                    let right = self.eval(right)?;
                    let r = match op {
                        CmpOp::Eq => left.py_eq(&right),
                        CmpOp::Ne => !left.py_eq(&right),
                        CmpOp::Lt => left.compare(&right, "lt")?,
                        CmpOp::LtEq => left.compare(&right, "le")?,
                        CmpOp::Gt => left.compare(&right, "gt")?,
                        CmpOp::GtEq => left.compare(&right, "ge")?,
                        CmpOp::In => right.contains(&left)?,
                        CmpOp::NotIn => !right.contains(&left)?,
                    };
                    if !r {
                        return Ok(Value::Bool(false));
                    }
                    left = right;
                }
                Ok(Value::Bool(true))
            }
            ExprKind::Getattr(obj, attr) => {
                let o = self.eval(obj)?;
                env_getattr(&o, attr)
            }
            ExprKind::Getitem(obj, arg) => {
                let o = self.eval(obj)?;
                if let ExprKind::Slice(..) = arg.kind {
                    let s = self.eval(arg)?;
                    return py_getitem(&o, &s);
                }
                let a = self.eval(arg)?;
                env_getitem(&o, &a)
            }
            ExprKind::Slice(a, b, c) => {
                let mut ev = |x: &Option<Box<Expr>>| -> VResult<Value> {
                    match x {
                        Some(x) => self.eval(x),
                        None => Ok(Value::None),
                    }
                };
                let s = (ev(a)?, ev(b)?, ev(c)?);
                Ok(Value::Slice(Rc::new(s)))
            }
            ExprKind::Filter(..) => self.eval_filter(e, None),
            ExprKind::Test(node, name, ca) => {
                let v = self.eval(node)?;
                let (args, kwargs) = self.eval_args(ca)?;
                tests::call_test(self, name, v, args, kwargs).map(Value::Bool)
            }
            ExprKind::CondExpr(test, expr1, expr2) => {
                if self.eval(test)?.truthy() {
                    return self.eval(expr1);
                }
                match expr2 {
                    Some(x) => self.eval(x),
                    None => Ok(Value::undefined(
                        Some(format!(
                            "the inline if-expression on line {} evaluated to false and no else section was defined.",
                            e.lineno
                        )),
                        None,
                        None,
                    )),
                }
            }
            ExprKind::Call(..) => self.eval_call(e, None),
        }
    }

    fn eval_args(&mut self, ca: &CallArgs) -> VResult<(Vec<Value>, IndexMap<String, Value>)> {
        let mut args = vec![];
        for a in &ca.args {
            args.push(self.eval(a)?);
        }
        let mut kwargs = IndexMap::new();
        for k in &ca.kwargs {
            let v = self.eval(&k.value)?;
            kwargs.insert(k.key.clone(), v);
        }
        if let Some(d) = &ca.dyn_args {
            let v = self.eval(d)?;
            args.extend(v.iter_items()?);
        }
        if let Some(d) = &ca.dyn_kwargs {
            match self.eval(d)? {
                Value::Dict(m) => {
                    for (k, v) in m.borrow().iter() {
                        match &k.0 {
                            Value::Str(s) => {
                                kwargs.insert(s.to_string(), v.clone());
                            }
                            _ => return Err(Exc::type_error("keywords must be strings")),
                        }
                    }
                }
                Value::Undefined(_) => {}
                v => return Err(Exc::type_error(format!("argument after ** must be a mapping, not {}", v.type_name()))),
            }
        }
        Ok((args, kwargs))
    }

    fn eval_filter(&mut self, e: &Expr, block_value: Option<Value>) -> VResult<Value> {
        let ExprKind::Filter(node, name, ca) = &e.kind else { return self.eval(e) };
        let v = match node {
            Some(n) => self.eval(n)?,
            None => block_value.unwrap_or(Value::from("")),
        };
        let (args, kwargs) = self.eval_args(ca)?;
        filters::call_filter(self, name, v, args, kwargs)
    }

    fn eval_call(&mut self, e: &Expr, caller: Option<Value>) -> VResult<Value> {
        let ExprKind::Call(func, ca) = &e.kind else { return self.eval(e) };
        let f = self.eval(func)?;
        let (args, mut kwargs) = self.eval_args(ca)?;
        if let Some(c) = caller {
            kwargs.insert("caller".into(), c);
        }
        self.call(&f, args, kwargs)
    }

    /// context.call(func, *args, **kwargs)
    pub fn call(&mut self, f: &Value, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
        match f {
            Value::Undefined(u) => Err(Exc::undefined(u.message())),
            Value::Object(o) => {
                let o = o.clone();
                match o.call(self, args, kwargs) {
                    Some(r) => r,
                    None => Err(Exc::type_error(format!("'{}' object is not callable", o.type_name()))),
                }
            }
            v => Err(Exc::type_error(format!("'{}' object is not callable", v.type_name()))),
        }
    }
}
