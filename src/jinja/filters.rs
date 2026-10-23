// Jinja2 3.1.6 builtin filters (port of jinja2/filters.py)
use super::eval::{env_getattr, env_getitem, py_getattr, Renderer};
use super::methods::{str_capitalize, str_splitlines};
use super::runtime::{escape, Generator, Markup};
use super::value::{Exc, Key, VResult, Value};
use crate::pyfmt::{py_float, py_isspace, py_str_repr};
use indexmap::IndexMap;

pub const FILTER_NAMES: &[&str] = &[
    "abs", "attr", "batch", "capitalize", "center", "count", "d", "default", "dictsort", "e", "escape", "filesizeformat", "first",
    "float", "forceescape", "format", "groupby", "indent", "int", "join", "last", "length", "list", "lower", "items", "map", "min",
    "max", "pprint", "random", "reject", "rejectattr", "replace", "reverse", "round", "safe", "select", "selectattr", "slice",
    "sort", "string", "striptags", "sum", "title", "trim", "truncate", "unique", "upper", "urlencode", "urlize", "wordcount",
    "wordwrap", "xmlattr", "tojson",
];

/// python argument binding: (name, default) parameters; the bound values
/// plus the extra positional / keyword arguments when accepted
pub fn bind(
    fname: &str,
    params: &[(&str, Option<Value>)],
    args: Vec<Value>,
    mut kwargs: IndexMap<String, Value>,
    varargs: bool,
    varkw: bool,
) -> VResult<(Vec<Value>, Vec<Value>, IndexMap<String, Value>)> {
    let n = params.len();
    let mut bound: Vec<Option<Value>> = vec![None; n];
    let mut extra_args = vec![];
    let npos = args.len();
    for (i, a) in args.into_iter().enumerate() {
        if i < n {
            bound[i] = Some(a);
        } else {
            extra_args.push(a);
        }
    }
    let mut extra_kwargs = IndexMap::new();
    for (k, v) in kwargs.drain(..) {
        match params.iter().position(|(p, _)| *p == k) {
            Some(i) => {
                if bound[i].is_some() {
                    return Err(Exc::type_error(format!("{}() got multiple values for argument '{}'", fname, k)));
                }
                bound[i] = Some(v);
            }
            None if varkw => {
                extra_kwargs.insert(k, v);
            }
            None => return Err(Exc::type_error(format!("{}() got an unexpected keyword argument '{}'", fname, k))),
        }
    }
    if !varargs && npos > n {
        let required = params.iter().filter(|(_, d)| d.is_none()).count();
        let s = |c: usize| if c == 1 { "" } else { "s" };
        let msg = if required == n {
            format!("{}() takes {} positional argument{} but {} {} given", fname, n, s(n), npos, if npos == 1 { "was" } else { "were" })
        } else {
            format!("{}() takes from {} to {} positional arguments but {} {} given", fname, required, n, npos, if npos == 1 { "was" } else { "were" })
        };
        return Err(Exc::type_error(msg));
    }
    let mut missing = vec![];
    let mut out = vec![];
    for (i, (p, d)) in params.iter().enumerate() {
        match bound[i].take().or_else(|| d.clone()) {
            Some(v) => out.push(v),
            None => {
                missing.push(format!("'{}'", p));
                out.push(Value::None);
            }
        }
    }
    if !missing.is_empty() {
        let names = match missing.len() {
            1 => missing[0].clone(),
            2 => format!("{} and {}", missing[0], missing[1]),
            _ => format!("{}, and {}", missing[..missing.len() - 1].join(", "), missing[missing.len() - 1]),
        };
        return Err(Exc::type_error(format!(
            "{}() missing {} required positional argument{}: {}",
            fname,
            missing.len(),
            if missing.len() == 1 { "" } else { "s" },
            names
        )));
    }
    Ok((out, extra_args, extra_kwargs))
}

fn filter_argument_error(msg: impl Into<String>) -> Exc {
    Exc::new("jinja2.exceptions.FilterArgumentError", msg)
}

/// python int(v) / int(s, base); Err as python raises
pub fn py_int_value(v: &Value, base: Option<i64>) -> VResult<i64> {
    match v {
        Value::Str(s) => {
            let base = base.unwrap_or(10);
            let t = crate::pyfmt::py_strip(s);
            let r = if base == 10 {
                crate::pyfmt::py_int(t)
            } else {
                crate::pyfmt::py_int_base(t, base as u32)
            };
            r.ok_or_else(|| Exc::value_error(format!("invalid literal for int() with base {}: {}", base, py_str_repr(s))))
        }
        _ if base.is_some() => Err(Exc::type_error("int() can't convert non-string with explicit base")),
        Value::Bool(b) => Ok(*b as i64),
        Value::Int(i) => Ok(*i),
        Value::Float(f) => {
            if f.is_nan() {
                return Err(Exc::value_error("cannot convert float NaN to integer"));
            }
            if f.is_infinite() {
                return Err(Exc::new("OverflowError", "cannot convert float infinity to integer"));
            }
            Ok(f.trunc() as i64)
        }
        other => Err(Exc::type_error(format!(
            "int() argument must be a string, a bytes-like object or a real number, not '{}'",
            other.type_name()
        ))),
    }
}

/// python float(v)
pub fn py_float_value(v: &Value) -> VResult<f64> {
    match v {
        Value::Str(s) => py_float(s).ok_or_else(|| Exc::value_error(format!("could not convert string to float: {}", py_str_repr(s)))),
        Value::Bool(_) | Value::Int(_) | Value::Float(_) => Ok(v.as_f64().unwrap()),
        Value::Object(o) => match o.as_any().downcast_ref::<Markup>() {
            Some(m) => py_float(&m.0).ok_or_else(|| Exc::value_error(format!("could not convert string to float: {}", py_str_repr(&m.0)))),
            None => Err(Exc::type_error(format!("float() argument must be a string or a real number, not '{}'", o.type_name()))),
        },
        other => Err(Exc::type_error(format!("float() argument must be a string or a real number, not '{}'", other.type_name()))),
    }
}

/// ignore_case postprocess
fn ignore_case(v: Value) -> Value {
    match &v {
        Value::Str(s) => Value::from(s.to_lowercase()),
        _ => v,
    }
}

/// _prepare_attribute_parts
fn attribute_parts(attr: &Value) -> Vec<Value> {
    match attr {
        Value::None => vec![],
        Value::Str(s) => s
            .split('.')
            .map(|x| if !x.is_empty() && x.chars().all(|c| c.is_ascii_digit()) { Value::Int(x.parse().unwrap_or(0)) } else { Value::from(x) })
            .collect(),
        v => vec![v.clone()],
    }
}

/// make_attrgetter(environment, attribute, postprocess, default)
fn attrgetter(item: &Value, parts: &[Value], postprocess: bool, default: Option<&Value>) -> VResult<Value> {
    let mut item = item.clone();
    for part in parts {
        item = env_getitem(&item, part)?;
        if let Some(d) = default {
            if !matches!(d, Value::None) && item.is_undefined() {
                item = d.clone();
            }
        }
    }
    if postprocess {
        item = ignore_case(item);
    }
    Ok(item)
}

/// make_multi_attrgetter
fn multi_attrgetter(item: &Value, attribute: &Value, postprocess: bool) -> VResult<Value> {
    let split: Vec<Value> = match attribute {
        Value::Str(s) => s.split(',').map(Value::from).collect(),
        v => vec![v.clone()],
    };
    let mut items = vec![];
    for a in &split {
        let parts = attribute_parts(a);
        let mut it = item.clone();
        for part in &parts {
            it = env_getitem(&it, part)?;
        }
        if postprocess {
            it = ignore_case(it);
        }
        items.push(it);
    }
    Ok(Value::list(items))
}

fn sorted_by_key(items: Vec<Value>, keys: Vec<Value>, reverse: bool) -> VResult<Vec<Value>> {
    // decorate-sort-undecorate with a stable sort on the keys
    let mut idx: Vec<usize> = (0..items.len()).collect();
    let mut err = None;
    idx.sort_by(|&a, &b| {
        if err.is_some() {
            return std::cmp::Ordering::Equal;
        }
        let (x, y) = if reverse { (&keys[b], &keys[a]) } else { (&keys[a], &keys[b]) };
        match x.compare(y, "lt") {
            Ok(true) => std::cmp::Ordering::Less,
            Ok(false) => match y.compare(x, "lt") {
                Ok(true) => std::cmp::Ordering::Greater,
                Ok(false) => std::cmp::Ordering::Equal,
                Err(e) => {
                    err = Some(e);
                    std::cmp::Ordering::Equal
                }
            },
            Err(e) => {
                err = Some(e);
                std::cmp::Ordering::Equal
            }
        }
    });
    if let Some(e) = err {
        return Err(e);
    }
    Ok(idx.into_iter().map(|i| items[i].clone()).collect())
}

fn str_of(v: &Value) -> String {
    v.to_str()
}

fn want_int_arg(v: &Value) -> VResult<i64> {
    v.as_int().ok_or_else(|| Exc::type_error(format!("'{}' object cannot be interpreted as an integer", v.type_name())))
}

/// environment.call_filter(name, value, args, kwargs, context)
pub fn call_filter(r: &mut Renderer, name: &str, value: Value, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let none = || Some(Value::None);
    let mut all = vec![value];
    all.extend(args);
    macro_rules! b {
        ($fname:expr, [$(($p:expr, $d:expr)),*]) => {
            bind($fname, &[$(($p, $d)),*], all, kwargs, false, false)?.0
        };
    }
    match name {
        "abs" => {
            let a = b!("abs", [("x", None)]);
            match &a[0] {
                Value::Bool(x) => Ok(Value::Int(*x as i64)),
                Value::Int(i) => Ok(Value::Int(i.abs())),
                Value::Float(f) => Ok(Value::Float(f.abs())),
                v => Err(Exc::type_error(format!("bad operand type for abs(): '{}'", v.type_name()))),
            }
        }
        "attr" => {
            let a = b!("do_attr", [("obj", None), ("name", None)]);
            let name = a[1].to_str();
            match py_getattr(&a[0], &name) {
                Some(v) => Ok(v),
                None => Ok(Value::undefined(None, Some(a[0].clone()), Some(Value::from(name)))),
            }
        }
        "batch" => {
            let a = b!("do_batch", [("value", None), ("linecount", None), ("fill_with", none())]);
            let linecount = want_int_arg(&a[1])? as usize;
            let mut out = vec![];
            let mut tmp: Vec<Value> = vec![];
            for item in a[0].iter_items()? {
                if tmp.len() == linecount {
                    out.push(Value::list(std::mem::take(&mut tmp)));
                }
                tmp.push(item);
            }
            if !tmp.is_empty() {
                if !matches!(a[2], Value::None) && tmp.len() < linecount {
                    while tmp.len() < linecount {
                        tmp.push(a[2].clone());
                    }
                }
                out.push(Value::list(tmp));
            }
            Ok(Generator::new("do_batch", out))
        }
        "capitalize" => {
            let a = b!("do_capitalize", [("s", None)]);
            Ok(Value::from(str_capitalize(&str_of(&a[0]))))
        }
        "center" => {
            let a = b!("do_center", [("value", None), ("width", Some(Value::Int(80)))]);
            super::methods::call_method(&Value::from(str_of(&a[0])), "center", vec![a[1].clone()], IndexMap::new())
        }
        "count" | "length" => {
            let a = b!("len", [("obj", None)]);
            Ok(Value::Int(a[0].len()? as i64))
        }
        "d" | "default" => {
            let a = b!("do_default", [("value", None), ("default_value", Some(Value::from(""))), ("boolean", Some(Value::Bool(false)))]);
            if a[0].is_undefined() || (a[2].truthy() && !a[0].truthy()) {
                return Ok(a[1].clone());
            }
            Ok(a[0].clone())
        }
        "dictsort" => {
            let a = b!(
                "do_dictsort",
                [("value", None), ("case_sensitive", Some(Value::Bool(false))), ("by", Some(Value::from("key"))), ("reverse", Some(Value::Bool(false)))]
            );
            let pos = match a[2].to_str().as_str() {
                "key" => 0,
                "value" => 1,
                _ => return Err(filter_argument_error("You can only sort by either \"key\" or \"value\"")),
            };
            let items = super::methods::call_method(&a[0], "items", vec![], IndexMap::new())?.iter_items()?;
            let keys = items
                .iter()
                .map(|it| {
                    let v = it.iter_items().map(|p| p[pos].clone()).unwrap_or(Value::None);
                    if a[1].truthy() {
                        v
                    } else {
                        ignore_case(v)
                    }
                })
                .collect();
            Ok(Value::list(sorted_by_key(items, keys, a[3].truthy())?))
        }
        "e" | "escape" => {
            let a = b!("escape", [("s", None)]);
            Ok(escape(&a[0]))
        }
        "filesizeformat" => {
            let a = b!("do_filesizeformat", [("value", None), ("binary", Some(Value::Bool(false)))]);
            let bytes = py_float_value(&a[0])?;
            let binary = a[1].truthy();
            let base: f64 = if binary { 1024.0 } else { 1000.0 };
            let prefixes = if binary {
                ["KiB", "MiB", "GiB", "TiB", "PiB", "EiB", "ZiB", "YiB"]
            } else {
                ["kB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"]
            };
            if bytes == 1.0 {
                return Ok(Value::from("1 Byte"));
            } else if bytes < base {
                return Ok(Value::from(format!("{} Bytes", bytes.trunc() as i64)));
            }
            let mut unit = 0.0;
            let mut prefix = "";
            for (i, p) in prefixes.iter().enumerate() {
                unit = base.powi(i as i32 + 2);
                prefix = p;
                if bytes < unit {
                    break;
                }
            }
            Ok(Value::from(format!("{:.1} {}", base * bytes / unit, prefix)))
        }
        "first" => {
            let a = b!("sync_do_first", [("seq", None)]);
            Ok(a[0].iter_items()?.into_iter().next().unwrap_or_else(|| Value::undefined(Some("No first item, sequence was empty.".into()), None, None)))
        }
        "float" => {
            let a = b!("do_float", [("value", None), ("default", Some(Value::Float(0.0)))]);
            match py_float_value(&a[0]) {
                Ok(f) => Ok(Value::Float(f)),
                Err(e) if e.cls == "TypeError" || e.cls == "ValueError" => Ok(a[1].clone()),
                Err(e) => Err(e),
            }
        }
        "forceescape" => {
            let a = b!("do_forceescape", [("value", None)]);
            Ok(escape(&Value::from(a[0].to_str())))
        }
        "format" => {
            let (a, extra, extra_kw) = bind("do_format", &[("value", None)], all, kwargs, true, true)?;
            if !extra.is_empty() && !extra_kw.is_empty() {
                return Err(filter_argument_error("can't handle positional and keyword arguments at the same time"));
            }
            let fmt = Value::from(a[0].to_str());
            let arg = if !extra_kw.is_empty() {
                Value::dict(extra_kw.into_iter().map(|(k, v)| (Key::str(&k), v)).collect())
            } else {
                Value::tuple(extra)
            };
            fmt.modulo(&arg)
        }
        "groupby" => {
            let a = b!(
                "sync_do_groupby",
                [("value", None), ("attribute", None), ("default", none()), ("case_sensitive", Some(Value::Bool(false)))]
            );
            let parts = attribute_parts(&a[1]);
            let case_sensitive = a[3].truthy();
            let items = a[0].iter_items()?;
            let keys = items.iter().map(|it| attrgetter(it, &parts, !case_sensitive, Some(&a[2]))).collect::<VResult<Vec<_>>>()?;
            let sorted = sorted_by_key(items, keys, false)?;
            let mut groups: Vec<(Value, Vec<Value>)> = vec![];
            for it in sorted {
                let k = attrgetter(&it, &parts, !case_sensitive, Some(&a[2]))?;
                match groups.last_mut() {
                    Some((gk, gv)) if gk.py_eq(&k) => gv.push(it),
                    _ => groups.push((k, vec![it])),
                }
            }
            let mut out = vec![];
            for (k, values) in groups {
                let key = if case_sensitive { k } else { attrgetter(&values[0], &parts, false, Some(&a[2]))? };
                out.push(Value::NamedTuple(std::rc::Rc::new((
                    "_GroupTuple".into(),
                    vec!["grouper".into(), "list".into()],
                    vec![key, Value::list(values)],
                ))));
            }
            Ok(Value::list(out))
        }
        "indent" => {
            let a = b!(
                "do_indent",
                [("s", None), ("width", Some(Value::Int(4))), ("first", Some(Value::Bool(false))), ("blank", Some(Value::Bool(false)))]
            );
            let indention = match &a[1] {
                Value::Str(s) => s.to_string(),
                w => " ".repeat(want_int_arg(w)?.max(0) as usize),
            };
            let s = format!("{}\n", a[0].to_str());
            let mut rv = if a[3].truthy() {
                str_splitlines(&s, false).join(&format!("\n{}", indention))
            } else {
                let mut lines = str_splitlines(&s, false);
                let mut rv = lines.remove(0);
                if !lines.is_empty() {
                    let rest: Vec<String> =
                        lines.iter().map(|l| if l.is_empty() { l.clone() } else { format!("{}{}", indention, l) }).collect();
                    rv = format!("{}\n{}", rv, rest.join("\n"));
                }
                rv
            };
            if a[2].truthy() {
                rv = format!("{}{}", indention, rv);
            }
            Ok(Value::from(rv))
        }
        "int" => {
            let a = b!("do_int", [("value", None), ("default", Some(Value::Int(0))), ("base", Some(Value::Int(10)))]);
            let base = want_int_arg(&a[2])?;
            let first = match &a[0] {
                Value::Str(_) => py_int_value(&a[0], Some(base)),
                v => py_int_value(v, None),
            };
            match first {
                Ok(i) => Ok(Value::Int(i)),
                Err(e) if e.cls == "TypeError" || e.cls == "ValueError" => match py_float_value(&a[0]).and_then(|f| py_int_value(&Value::Float(f), None)) {
                    Ok(i) => Ok(Value::Int(i)),
                    Err(e) if matches!(e.cls.as_str(), "TypeError" | "ValueError" | "OverflowError") => Ok(a[1].clone()),
                    Err(e) => Err(e),
                },
                Err(e) => Err(e),
            }
        }
        "join" => {
            let a = b!("sync_do_join", [("value", None), ("d", Some(Value::from(""))), ("attribute", none())]);
            let mut items = a[0].iter_items()?;
            if !matches!(a[2], Value::None) {
                let parts = attribute_parts(&a[2]);
                items = items.iter().map(|it| attrgetter(it, &parts, false, None)).collect::<VResult<_>>()?;
            }
            Ok(Value::from(items.iter().map(|v| v.to_str()).collect::<Vec<_>>().join(&a[1].to_str())))
        }
        "last" => {
            let a = b!("do_last", [("seq", None)]);
            let items = match &a[0] {
                Value::Object(o) if o.as_any().is::<Generator>() => {
                    return Err(Exc::type_error(format!("'{}' object is not reversible", o.type_name())))
                }
                v => v.iter_items()?,
            };
            Ok(items.into_iter().last().unwrap_or_else(|| Value::undefined(Some("No last item, sequence was empty.".into()), None, None)))
        }
        "list" => {
            let a = b!("sync_do_list", [("value", None)]);
            Ok(Value::list(a[0].iter_items()?))
        }
        "lower" => {
            let a = b!("do_lower", [("s", None)]);
            Ok(Value::from(str_of(&a[0]).to_lowercase()))
        }
        "upper" => {
            let a = b!("do_upper", [("s", None)]);
            Ok(Value::from(str_of(&a[0]).to_uppercase()))
        }
        "items" => {
            let a = b!("do_items", [("value", None)]);
            match &a[0] {
                Value::Undefined(_) => Ok(Generator::new("do_items", vec![])),
                Value::Dict(_) => Ok(Generator::new("do_items", super::methods::call_method(&a[0], "items", vec![], IndexMap::new())?.iter_items()?)),
                _ => Err(Exc::type_error("Can only get item pairs from a mapping.")),
            }
        }
        "map" => {
            let (a, args, mut kwargs) = bind("sync_do_map", &[("value", None)], all, kwargs, true, true)?;
            let mut out = vec![];
            if a[0].truthy() {
                if args.is_empty() && kwargs.contains_key("attribute") {
                    let attribute = kwargs.shift_remove("attribute").unwrap();
                    let default = kwargs.shift_remove("default").unwrap_or(Value::None);
                    if let Some((k, _)) = kwargs.first() {
                        return Err(filter_argument_error(format!("Unexpected keyword argument {}", py_str_repr(k))));
                    }
                    let parts = attribute_parts(&attribute);
                    for item in a[0].iter_items()? {
                        out.push(attrgetter(&item, &parts, false, Some(&default))?);
                    }
                } else {
                    let Some(fname) = args.first().cloned() else {
                        return Err(filter_argument_error("map requires a filter argument"));
                    };
                    let rest: Vec<Value> = args[1..].to_vec();
                    for item in a[0].iter_items()? {
                        out.push(call_filter_by_name(r, &fname.to_str(), item, rest.clone(), kwargs.clone())?);
                    }
                }
            }
            Ok(Generator::new("sync_do_map", out))
        }
        "min" | "max" => {
            let fname = if name == "min" { "do_min" } else { "do_max" };
            let a = b!(fname, [("value", None), ("case_sensitive", Some(Value::Bool(false))), ("attribute", none())]);
            let items = a[0].iter_items()?;
            if items.is_empty() {
                return Ok(Value::undefined(Some("No aggregated item, sequence was empty.".into()), None, None));
            }
            let parts = attribute_parts(&a[2]);
            let keys = items.iter().map(|it| attrgetter(it, &parts, !a[1].truthy(), None)).collect::<VResult<Vec<_>>>()?;
            let mut best = 0;
            for i in 1..items.len() {
                let better = if name == "min" { keys[i].compare(&keys[best], "lt")? } else { keys[i].compare(&keys[best], "gt")? };
                if better {
                    best = i;
                }
            }
            Ok(items[best].clone())
        }
        "pprint" => {
            let a = b!("do_pprint", [("value", None)]);
            Ok(Value::from(a[0].repr()))
        }
        "random" => {
            let a = b!("do_random", [("seq", None)]);
            let items = a[0].iter_items()?;
            if items.is_empty() {
                return Ok(Value::undefined(Some("No random item, sequence was empty.".into()), None, None));
            }
            let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.subsec_nanos()).unwrap_or(0);
            Ok(items[t as usize % items.len()].clone())
        }
        "reject" | "select" | "rejectattr" | "selectattr" => {
            let fname = match name {
                "reject" => "sync_do_reject",
                "select" => "sync_do_select",
                "rejectattr" => "sync_do_rejectattr",
                _ => "sync_do_selectattr",
            };
            let (a, args, kwargs) = bind(fname, &[("value", None)], all, kwargs, true, true)?;
            let lookup_attr = name.ends_with("attr");
            let negate = name.starts_with("reject");
            let mut out = vec![];
            if a[0].truthy() {
                let off = if lookup_attr { 1 } else { 0 };
                let parts = if lookup_attr {
                    let Some(attr) = args.first() else {
                        return Err(filter_argument_error("Missing parameter for attribute name"));
                    };
                    attribute_parts(attr)
                } else {
                    vec![]
                };
                let test = args.get(off).cloned();
                let rest: Vec<Value> = args.iter().skip(off + 1).cloned().collect();
                for item in a[0].iter_items()? {
                    let v = if lookup_attr { attrgetter(&item, &parts, false, None)? } else { item.clone() };
                    let ok = match &test {
                        Some(t) => super::tests::call_test(r, &t.to_str(), v, rest.clone(), kwargs.clone())?,
                        None => v.truthy(),
                    };
                    if ok != negate {
                        out.push(item);
                    }
                }
            }
            Ok(Generator::new(fname, out))
        }
        "replace" => {
            let a = b!("do_replace", [("s", None), ("old", None), ("new", None), ("count", none())]);
            let count = if matches!(a[3], Value::None) { Value::Int(-1) } else { a[3].clone() };
            super::methods::call_method(&Value::from(a[0].to_str()), "replace", vec![Value::from(a[1].to_str()), Value::from(a[2].to_str()), count], IndexMap::new())
        }
        "reverse" => {
            let a = b!("do_reverse", [("value", None)]);
            match &a[0] {
                Value::Str(s) => Ok(Value::from(s.chars().rev().collect::<String>())),
                Value::List(l) => Ok(Generator::iterator("list_reverseiterator", l.borrow().iter().rev().cloned().collect())),
                Value::Tuple(_) | Value::NamedTuple(_) | Value::Range(..) => {
                    let mut items = a[0].iter_items()?;
                    items.reverse();
                    let tname = if matches!(a[0], Value::Range(..)) { "range_iterator" } else { "reversed" };
                    Ok(Generator::iterator(tname, items))
                }
                Value::Dict(d) => Ok(Generator::iterator("dict_reversekeyiterator", d.borrow().keys().rev().map(|k| k.0.clone()).collect())),
                v => match v.iter_items() {
                    Ok(mut items) => {
                        items.reverse();
                        Ok(Value::list(items))
                    }
                    Err(_) => Err(filter_argument_error("argument must be iterable")),
                },
            }
        }
        "round" => {
            let a = b!("do_round", [("value", None), ("precision", Some(Value::Int(0))), ("method", Some(Value::from("common")))]);
            let method = a[2].to_str();
            if !matches!(method.as_str(), "common" | "ceil" | "floor") {
                return Err(filter_argument_error("method must be common, ceil or floor"));
            }
            let precision = want_int_arg(&a[1])?;
            if method == "common" {
                return py_round(&a[0], Some(precision));
            }
            let v = a[0].as_f64().ok_or_else(|| Exc::type_error(format!("must be real number, not {}", a[0].type_name())))?;
            let scale = Value::Int(10).pow(&Value::Int(precision))?.as_f64().unwrap();
            let x = v * scale;
            let r = if method == "ceil" { x.ceil() } else { x.floor() };
            Ok(Value::Float(r / scale))
        }
        "safe" => {
            let a = b!("do_mark_safe", [("value", None)]);
            Ok(Value::object(Markup(a[0].to_str())))
        }
        "slice" => {
            let a = b!("sync_do_slice", [("value", None), ("slices", None), ("fill_with", none())]);
            let seq = a[0].iter_items()?;
            let slices = want_int_arg(&a[1])?;
            if slices == 0 {
                return Err(Exc::zero_division("integer division or modulo by zero"));
            }
            let length = seq.len() as i64;
            let items_per_slice = length.div_euclid(slices);
            let slices_with_extra = length.rem_euclid(slices);
            let mut offset = 0;
            let mut out = vec![];
            for slice_number in 0..slices.max(0) {
                let start = offset + slice_number * items_per_slice;
                if slice_number < slices_with_extra {
                    offset += 1;
                }
                let end = offset + (slice_number + 1) * items_per_slice;
                let mut tmp: Vec<Value> = seq[(start.max(0) as usize).min(seq.len())..(end.max(0) as usize).min(seq.len())].to_vec();
                if !matches!(a[2], Value::None) && slice_number >= slices_with_extra {
                    tmp.push(a[2].clone());
                }
                out.push(Value::list(tmp));
            }
            Ok(Generator::new("sync_do_slice", out))
        }
        "sort" => {
            let a = b!(
                "do_sort",
                [("value", None), ("reverse", Some(Value::Bool(false))), ("case_sensitive", Some(Value::Bool(false))), ("attribute", none())]
            );
            let items = a[0].iter_items()?;
            let keys = items.iter().map(|it| multi_attrgetter(it, &a[3], !a[2].truthy())).collect::<VResult<Vec<_>>>()?;
            Ok(Value::list(sorted_by_key(items, keys, a[1].truthy())?))
        }
        "string" => {
            let a = b!("soft_str", [("s", None)]);
            match &a[0] {
                Value::Object(o) if o.as_any().is::<Markup>() => Ok(a[0].clone()),
                v => Ok(Value::from(v.to_str())),
            }
        }
        "striptags" => {
            let a = b!("do_striptags", [("value", None)]);
            Ok(Value::from(striptags(&a[0].to_str())))
        }
        "sum" => {
            let a = b!("sync_do_sum", [("iterable", None), ("attribute", none()), ("start", Some(Value::Int(0)))]);
            let mut items = a[0].iter_items()?;
            if !matches!(a[1], Value::None) {
                let parts = attribute_parts(&a[1]);
                items = items.iter().map(|it| attrgetter(it, &parts, false, None)).collect::<VResult<_>>()?;
            }
            if matches!(a[2], Value::Str(_)) {
                return Err(Exc::type_error("sum() can't sum strings [use ''.join(seq) instead]"));
            }
            py_builtin_sum(a[2].clone(), items)
        }
        "title" => {
            let a = b!("do_title", [("s", None)]);
            Ok(Value::from(do_title(&a[0].to_str())))
        }
        "trim" => {
            let a = b!("do_trim", [("value", None), ("chars", none())]);
            super::methods::call_method(&Value::from(a[0].to_str()), "strip", vec![a[1].clone()], IndexMap::new())
        }
        "truncate" => {
            let a = b!(
                "do_truncate",
                [("s", None), ("length", Some(Value::Int(255))), ("killwords", Some(Value::Bool(false))), ("end", Some(Value::from("..."))), ("leeway", none())]
            );
            let s = a[0].to_str();
            let length = want_int_arg(&a[1])?;
            let end = a[3].to_str();
            let leeway = if matches!(a[4], Value::None) { 5 } else { want_int_arg(&a[4])? };
            let endlen = end.chars().count() as i64;
            if length < endlen {
                return Err(Exc::new("AssertionError", format!("expected length >= {}, got {}", endlen, length)));
            }
            if leeway < 0 {
                return Err(Exc::new("AssertionError", format!("expected leeway >= 0, got {}", leeway)));
            }
            let chars: Vec<char> = s.chars().collect();
            if chars.len() as i64 <= length + leeway {
                return Ok(Value::from(s));
            }
            let cut: String = chars[..(length - endlen) as usize].iter().collect();
            if a[2].truthy() {
                return Ok(Value::from(format!("{}{}", cut, end)));
            }
            let result = match cut.rfind(' ') {
                Some(p) => cut[..p].to_string(),
                None => cut,
            };
            Ok(Value::from(format!("{}{}", result, end)))
        }
        "unique" => {
            let a = b!("sync_do_unique", [("value", None), ("case_sensitive", Some(Value::Bool(false))), ("attribute", none())]);
            let parts = attribute_parts(&a[2]);
            let mut seen: Vec<Key> = vec![];
            let mut out = vec![];
            for item in a[0].iter_items()? {
                let key = Key::hashable(&attrgetter(&item, &parts, !a[1].truthy(), None)?)?;
                if !seen.contains(&key) {
                    seen.push(key);
                    out.push(item);
                }
            }
            Ok(Generator::new("sync_do_unique", out))
        }
        "urlencode" => {
            let a = b!("do_urlencode", [("value", None)]);
            match &a[0] {
                Value::Str(_) | Value::None | Value::Bool(_) | Value::Int(_) | Value::Float(_) => Ok(Value::from(url_quote(&a[0].to_str(), false))),
                v => {
                    let items = match v {
                        Value::Dict(_) => super::methods::call_method(v, "items", vec![], IndexMap::new())?.iter_items()?,
                        _ => v.iter_items()?,
                    };
                    let mut parts = vec![];
                    for it in items {
                        let p = it.iter_items()?;
                        if p.len() != 2 {
                            return Err(Exc::value_error(format!("too many values to unpack (expected 2)")));
                        }
                        parts.push(format!("{}={}", url_quote(&p[0].to_str(), true), url_quote(&p[1].to_str(), true)));
                    }
                    Ok(Value::from(parts.join("&")))
                }
            }
        }
        "urlize" => {
            let a = b!(
                "do_urlize",
                [("value", None), ("trim_url_limit", none()), ("nofollow", Some(Value::Bool(false))), ("target", none()), ("rel", none()), ("extra_schemes", none())]
            );
            // links are not detected: the text comes back escaped
            Ok(escape(&a[0]))
        }
        "wordcount" => {
            let a = b!("do_wordcount", [("s", None)]);
            let s = a[0].to_str();
            let mut count = 0;
            let mut in_word = false;
            for c in s.chars() {
                let w = c == '_' || c.is_alphanumeric();
                if w && !in_word {
                    count += 1;
                }
                in_word = w;
            }
            Ok(Value::Int(count))
        }
        "wordwrap" => {
            let a = b!(
                "do_wordwrap",
                [("s", None), ("width", Some(Value::Int(79))), ("break_long_words", Some(Value::Bool(true))), ("wrapstring", none()), ("break_on_hyphens", Some(Value::Bool(true)))]
            );
            let width = want_int_arg(&a[1])? as usize;
            let wrapstring = if matches!(a[3], Value::None) { "\n".to_string() } else { a[3].to_str() };
            let paragraphs: Vec<String> = str_splitlines(&a[0].to_str(), false)
                .iter()
                .map(|line| textwrap(line, width, a[2].truthy()).join(&wrapstring))
                .collect();
            Ok(Value::from(paragraphs.join(&wrapstring)))
        }
        "xmlattr" => {
            let a = b!("do_xmlattr", [("d", None), ("autospace", Some(Value::Bool(true)))]);
            let items = super::methods::call_method(&a[0], "items", vec![], IndexMap::new())?.iter_items()?;
            let mut parts = vec![];
            for it in items {
                let p = it.iter_items()?;
                if matches!(p[1], Value::None) || p[1].is_undefined() {
                    continue;
                }
                let key = p[0].to_str();
                if key.chars().any(|c| py_isspace(c) && c.is_ascii() || matches!(c, '/' | '>' | '=')) {
                    return Err(Exc::value_error(format!("Invalid character in attribute name: {}", py_str_repr(&key))));
                }
                parts.push(format!("{}=\"{}\"", escape(&p[0]).to_str(), escape(&p[1]).to_str()));
            }
            let mut rv = parts.join(" ");
            if a[1].truthy() && !rv.is_empty() {
                rv = format!(" {}", rv);
            }
            Ok(Value::from(rv))
        }
        "tojson" => {
            let a = b!("do_tojson", [("value", None), ("indent", none())]);
            let json = json_dumps_sorted(&a[0])?;
            let json = json.replace('<', "\\u003c").replace('>', "\\u003e").replace('&', "\\u0026").replace('\'', "\\u0027");
            Ok(Value::object(Markup(json)))
        }
        _ => Err(Exc::runtime(format!("No filter named {} found.", py_str_repr(name)))),
    }
}

/// call_filter for a filter named at runtime (map('name'))
fn call_filter_by_name(r: &mut Renderer, name: &str, value: Value, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    if !FILTER_NAMES.contains(&name) {
        return Err(Exc::runtime(format!("No filter named {}.", py_str_repr(name))));
    }
    call_filter(r, name, value, args, kwargs)
}

/// python round(number, ndigits)
pub fn py_round(v: &Value, ndigits: Option<i64>) -> VResult<Value> {
    match v {
        Value::Bool(_) | Value::Int(_) => {
            let i = v.as_int().unwrap();
            match ndigits {
                None => Ok(Value::Int(i)),
                Some(n) if n >= 0 => Ok(Value::Int(i)),
                Some(n) => {
                    let p = 10i64.checked_pow((-n) as u32).unwrap_or(i64::MAX);
                    let q = (i as f64 / p as f64).round_ties_even() as i64;
                    Ok(Value::Int(q * p))
                }
            }
        }
        Value::Float(f) => match ndigits {
            None => {
                if f.is_nan() {
                    return Err(Exc::value_error("cannot convert float NaN to integer"));
                }
                if f.is_infinite() {
                    return Err(Exc::new("OverflowError", "cannot convert float infinity to integer"));
                }
                Ok(Value::Int(f.round_ties_even() as i64))
            }
            Some(n) if n >= 0 => Ok(Value::Float(crate::pyfmt::py_round(*f, n as usize))),
            Some(n) => {
                let p = 10f64.powi((-n) as i32);
                Ok(Value::Float((f / p).round_ties_even() * p))
            }
        },
        other => Err(Exc::type_error(format!("type {} doesn't define __round__ method", other.type_name()))),
    }
}

fn do_title(s: &str) -> String {
    // re.split(r"([-\s({\[<]+)") then capitalize the first char of each item
    let is_sep = |c: char| c == '-' || py_isspace(c) || matches!(c, '(' | '{' | '[' | '<');
    let mut out = String::new();
    let mut item = String::new();
    let mut item_is_sep = false;
    let flush = |item: &mut String, out: &mut String| {
        let mut cs = item.chars();
        if let Some(f) = cs.next() {
            out.extend(f.to_uppercase());
            out.push_str(&cs.as_str().to_lowercase());
        }
        item.clear();
    };
    for c in s.chars() {
        let sep = is_sep(c);
        if sep != item_is_sep && !item.is_empty() {
            flush(&mut item, &mut out);
        }
        item_is_sep = sep;
        item.push(c);
    }
    flush(&mut item, &mut out);
    out
}

/// Markup.striptags()
fn striptags(s: &str) -> String {
    // remove comments and tags, unescape, collapse whitespace
    let mut out = String::new();
    let mut rest = s;
    while !rest.is_empty() {
        if rest.starts_with("<!--") {
            match rest.find("-->") {
                Some(e) => {
                    rest = &rest[e + 3..];
                    continue;
                }
                None => {}
            }
        }
        if rest.starts_with('<') {
            if let Some(e) = rest.find('>') {
                rest = &rest[e + 1..];
                continue;
            }
        }
        let c = rest.chars().next().unwrap();
        out.push(c);
        rest = &rest[c.len_utf8()..];
    }
    let words: Vec<&str> = out.split(py_isspace).filter(|w| !w.is_empty()).collect();
    let joined = words.join(" ");
    joined.replace("&lt;", "<").replace("&gt;", ">").replace("&#34;", "\"").replace("&#39;", "'").replace("&quot;", "\"").replace("&amp;", "&")
}

/// utils.url_quote(obj, for_qs)
fn url_quote(s: &str, for_qs: bool) -> String {
    let mut out = String::new();
    for b in s.as_bytes() {
        let c = *b as char;
        if c.is_ascii_alphanumeric() || "_.-~".contains(c) || (!for_qs && c == '/') {
            out.push(c);
        } else {
            out.push_str(&format!("%{:02X}", b));
        }
    }
    if for_qs {
        out = out.replace("%20", "+");
    }
    out
}

/// textwrap.wrap(line, width, expand_tabs=False, replace_whitespace=False,
/// break_long_words) (hyphen breaks are not split)
fn textwrap(text: &str, width: usize, break_long_words: bool) -> Vec<String> {
    // chunks: alternating whitespace / words
    let mut chunks: Vec<String> = vec![];
    let mut cur = String::new();
    let mut cur_ws = None;
    for c in text.chars() {
        let ws = py_isspace(c);
        if cur_ws.is_some() && cur_ws != Some(ws) {
            chunks.push(std::mem::take(&mut cur));
        }
        cur_ws = Some(ws);
        cur.push(c);
    }
    if !cur.is_empty() {
        chunks.push(cur);
    }
    chunks.reverse();
    let mut lines = vec![];
    while !chunks.is_empty() {
        let mut cur_line: Vec<String> = vec![];
        let mut cur_len = 0;
        // drop leading whitespace (except on the first line)
        if !lines.is_empty() && chunks.last().map(|c| c.trim_matches(py_isspace).is_empty()).unwrap_or(false) {
            chunks.pop();
        }
        while let Some(chunk) = chunks.last() {
            let l = chunk.chars().count();
            if cur_len + l <= width {
                cur_len += l;
                cur_line.push(chunks.pop().unwrap());
            } else {
                break;
            }
        }
        if let Some(chunk) = chunks.last() {
            if chunk.chars().count() > width {
                let space_left = if width > cur_len { width - cur_len } else { 1 };
                if break_long_words {
                    let c = chunks.pop().unwrap();
                    let head: String = c.chars().take(space_left).collect();
                    let tail: String = c.chars().skip(space_left).collect();
                    cur_line.push(head);
                    chunks.push(tail);
                } else if cur_line.is_empty() {
                    cur_line.push(chunks.pop().unwrap());
                }
            }
        }
        // drop trailing whitespace
        if cur_line.last().map(|c| c.trim_matches(py_isspace).is_empty()).unwrap_or(false) {
            cur_line.pop();
        }
        if !cur_line.is_empty() {
            lines.push(cur_line.concat());
        }
    }
    lines
}

/// json.dumps(value, sort_keys=True)
fn json_dumps_sorted(v: &Value) -> VResult<String> {
    Ok(match v {
        Value::None => "null".into(),
        Value::Bool(b) => if *b { "true" } else { "false" }.into(),
        Value::Int(i) => i.to_string(),
        Value::Float(f) => crate::pyfmt::py_json_float(*f),
        Value::Str(s) => crate::pyval::PyVal::Str(s.to_string()).json(),
        Value::List(_) | Value::Tuple(_) | Value::NamedTuple(_) => {
            let items = v.iter_items()?;
            let parts: Vec<String> = items.iter().map(json_dumps_sorted).collect::<VResult<_>>()?;
            format!("[{}]", parts.join(", "))
        }
        Value::Dict(d) => {
            let mut items: Vec<(String, Value)> = d.borrow().iter().map(|(k, v)| (k.0.to_str(), v.clone())).collect();
            items.sort_by(|a, b| a.0.cmp(&b.0));
            let parts: Vec<String> = items
                .iter()
                .map(|(k, v)| Ok(format!("{}: {}", crate::pyval::PyVal::Str(k.clone()).json(), json_dumps_sorted(v)?)))
                .collect::<VResult<_>>()?;
            format!("{{{}}}", parts.join(", "))
        }
        Value::Object(o) if o.as_any().is::<Markup>() => crate::pyval::PyVal::Str(v.to_str()).json(),
        other => return Err(Exc::type_error(format!("Object of type {} is not JSON serializable", other.type_name()))),
    })
}

/// filters used by env_getattr fallbacks elsewhere
pub fn getattr_or_item(obj: &Value, name: &str) -> VResult<Value> {
    env_getattr(obj, name)
}

/// python's builtin sum(iterable, start): exact ints, Neumaier compensated
/// floats (int items join a float sum uncompensated), generic + otherwise
/// (Python/bltinmodule.c builtin_sum_impl)
pub fn py_builtin_sum(start: Value, items: Vec<Value>) -> VResult<Value> {
    let mut rv = start;
    let mut it = items.into_iter().peekable();
    loop {
        match rv {
            Value::Float(start_f) => {
                let (mut f, mut c) = (start_f, 0.0f64);
                while let Some(item) = it.peek() {
                    match item {
                        Value::Float(x) => {
                            let x = *x;
                            let t = f + x;
                            if f.abs() >= x.abs() {
                                c += (f - t) + x;
                            } else {
                                c += (x - t) + f;
                            }
                            f = t;
                        }
                        Value::Int(i) => f += *i as f64,
                        Value::Bool(b) => f += *b as i64 as f64,
                        _ => break,
                    }
                    it.next();
                }
                if c != 0.0 && c.is_finite() {
                    f += c;
                }
                rv = Value::Float(f);
                match it.next() {
                    None => return Ok(rv),
                    Some(item) => rv = rv.add(&item)?,
                }
            }
            _ => match it.next() {
                None => return Ok(rv),
                Some(item) => rv = rv.add(&item)?,
            },
        }
    }
}
