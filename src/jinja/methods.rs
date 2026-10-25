// Attributes and methods of python's builtin types (str, list, dict, tuple,
// int, float, namedtuple) as templates reach them: getattr(obj, name)
// and the bound method calls.
use super::value::{CallCtx, Exc, Key, Object, VResult, Value};
use crate::pyfmt::py_isspace;
use crate::IndexMap;
use std::rc::Rc;

/// a bound method of a builtin object
pub struct BoundMethod {
    pub recv: Value,
    pub name: String,
}

impl Object for BoundMethod {
    fn type_name(&self) -> String {
        "builtin_function_or_method".into()
    }
    fn repr(&self) -> String {
        format!("<built-in method {} of {} object>", self.name, self.recv.type_name())
    }
    fn call(&self, _ctx: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        Some(call_method(&self.recv, &self.name, args, kwargs))
    }
}

/// dict_keys / dict_values / dict_items
pub struct DictView {
    pub kind: &'static str,
    pub items: Vec<Value>,
}

impl Object for DictView {
    fn type_name(&self) -> String {
        self.kind.into()
    }
    fn repr(&self) -> String {
        format!("{}([{}])", self.kind, self.items.iter().map(|v| v.repr()).collect::<Vec<_>>().join(", "))
    }
    fn iter(&self) -> Option<VResult<Vec<Value>>> {
        Some(Ok(self.items.clone()))
    }
    fn len(&self) -> Option<usize> {
        Some(self.items.len())
    }
}

const STR_METHODS: &[&str] = &[
    "capitalize", "casefold", "center", "count", "endswith", "find", "format", "format_map", "index", "isalnum", "isalpha",
    "isascii", "isdecimal", "isdigit", "isidentifier", "islower", "isnumeric", "isspace", "istitle", "isupper", "join", "ljust",
    "lower", "lstrip", "partition", "removeprefix", "removesuffix", "replace", "rfind", "rindex", "rjust", "rpartition",
    "rsplit", "rstrip", "split", "splitlines", "startswith", "strip", "swapcase", "title", "upper", "zfill",
];
const LIST_METHODS: &[&str] = &["append", "clear", "copy", "count", "extend", "index", "insert", "pop", "remove", "reverse", "sort"];
const DICT_METHODS: &[&str] = &["clear", "copy", "get", "items", "keys", "pop", "popitem", "setdefault", "update", "values"];
const TUPLE_METHODS: &[&str] = &["count", "index"];

/// getattr(obj, name) for builtin values (None: AttributeError)
pub fn getattr(v: &Value, name: &str) -> Option<Value> {
    let bound = || Some(Value::object(BoundMethod { recv: v.clone(), name: name.to_string() }));
    match v {
        Value::Str(_) if STR_METHODS.contains(&name) => bound(),
        Value::List(_) if LIST_METHODS.contains(&name) => bound(),
        Value::Dict(_) if DICT_METHODS.contains(&name) => bound(),
        Value::Tuple(_) if TUPLE_METHODS.contains(&name) => bound(),
        Value::NamedTuple(nt) => {
            if let Some(i) = nt.1.iter().position(|f| f == name) {
                return Some(nt.2[i].clone());
            }
            match name {
                "_fields" => Some(Value::tuple(nt.1.iter().map(|f| Value::from(f.as_str())).collect())),
                "count" | "index" | "_asdict" | "_replace" => bound(),
                _ => None,
            }
        }
        Value::Bool(_) | Value::Int(_) => match name {
            "real" | "numerator" => Some(Value::Int(v.as_int().unwrap())),
            "imag" => Some(Value::Int(0)),
            "denominator" => Some(Value::Int(1)),
            "bit_length" | "conjugate" | "is_integer" | "bit_count" => bound(),
            _ => None,
        },
        Value::Float(f) => match name {
            "real" => Some(Value::Float(*f)),
            "imag" => Some(Value::Float(0.0)),
            "is_integer" | "conjugate" | "hex" => bound(),
            _ => None,
        },
        Value::Range(a, b, c) => match name {
            "start" => Some(Value::Int(*a)),
            "stop" => Some(Value::Int(*b)),
            "step" => Some(Value::Int(*c)),
            "count" | "index" => bound(),
            _ => None,
        },
        _ => None,
    }
}

fn nargs(name: &str, args: &[Value], min: usize, max: usize) -> VResult<()> {
    if args.len() < min || args.len() > max {
        let msg = if min == max {
            if min == 0 {
                format!("{}() takes no arguments ({} given)", name, args.len())
            } else {
                format!("{}() takes exactly one argument ({} given)", name, args.len())
            }
        } else if args.len() < min {
            format!("{} expected at least {} argument{}, got {}", name, min, if min == 1 { "" } else { "s" }, args.len())
        } else {
            format!("{} expected at most {} argument{}, got {}", name, max, if max == 1 { "" } else { "s" }, args.len())
        };
        return Err(Exc::type_error(msg));
    }
    Ok(())
}

fn no_kwargs(name: &str, kwargs: &IndexMap<String, Value>) -> VResult<()> {
    if !kwargs.is_empty() {
        return Err(Exc::type_error(format!("{}() takes no keyword arguments", name)));
    }
    Ok(())
}

fn want_str<'a>(v: &'a Value, what: &str) -> VResult<&'a str> {
    match v {
        Value::Str(s) => Ok(s),
        other => Err(Exc::type_error(format!("{} must be str, not {}", what, other.type_name()))),
    }
}

fn want_int(v: &Value) -> VResult<i64> {
    v.as_int()
        .ok_or_else(|| Exc::type_error(format!("'{}' object cannot be interpreted as an integer", v.type_name())))
}

/// python index normalization for start/end arguments (None allowed)
fn slice_bounds(len: usize, start: Option<&Value>, end: Option<&Value>) -> VResult<(usize, usize)> {
    let norm = |v: Option<&Value>, default: i64| -> VResult<i64> {
        match v {
            None | Some(Value::None) => Ok(default),
            Some(v) => {
                let i = v.as_int().ok_or_else(|| {
                    Exc::type_error("slice indices must be integers or None or have an __index__ method")
                })?;
                let len = len as i64;
                Ok(if i < 0 { (i + len).max(0) } else { i.min(len) })
            }
        }
    };
    let s = norm(start, 0)? as usize;
    let e = norm(end, len as i64)? as usize;
    Ok((s, e))
}

/// str.split() / str.rsplit()
fn str_split(s: &str, sep: Option<&str>, maxsplit: i64, right: bool) -> VResult<Vec<String>> {
    let chars: Vec<char> = s.chars().collect();
    match sep {
        None => {
            // runs of whitespace
            let mut words: Vec<String> = vec![];
            if !right {
                let mut i = 0;
                let mut splits = 0;
                while i < chars.len() {
                    while i < chars.len() && py_isspace(chars[i]) {
                        i += 1;
                    }
                    if i >= chars.len() {
                        break;
                    }
                    if maxsplit >= 0 && splits >= maxsplit {
                        let rest: String = chars[i..].iter().collect();
                        let rest = rest.trim_end_matches(py_isspace).to_string();
                        words.push(rest);
                        break;
                    }
                    let st = i;
                    while i < chars.len() && !py_isspace(chars[i]) {
                        i += 1;
                    }
                    words.push(chars[st..i].iter().collect());
                    splits += 1;
                }
            } else {
                let mut i = chars.len();
                let mut splits = 0;
                while i > 0 {
                    while i > 0 && py_isspace(chars[i - 1]) {
                        i -= 1;
                    }
                    if i == 0 {
                        break;
                    }
                    if maxsplit >= 0 && splits >= maxsplit {
                        let rest: String = chars[..i].iter().collect();
                        words.push(rest.trim_start_matches(py_isspace).to_string());
                        break;
                    }
                    let en = i;
                    while i > 0 && !py_isspace(chars[i - 1]) {
                        i -= 1;
                    }
                    words.push(chars[i..en].iter().collect());
                    splits += 1;
                }
                words.reverse();
            }
            Ok(words)
        }
        Some("") => Err(Exc::value_error("empty separator")),
        Some(sep) => {
            if maxsplit < 0 {
                return Ok(s.split(sep).map(|x| x.to_string()).collect());
            }
            if !right {
                Ok(s.splitn(maxsplit as usize + 1, sep).map(|x| x.to_string()).collect())
            } else {
                let mut v: Vec<String> = s.rsplitn(maxsplit as usize + 1, sep).map(|x| x.to_string()).collect();
                v.reverse();
                Ok(v)
            }
        }
    }
}

fn strip_chars<'a>(s: &'a str, chars: Option<&Value>, left: bool, right: bool) -> VResult<&'a str> {
    let set: Option<Vec<char>> = match chars {
        None | Some(Value::None) => None,
        Some(Value::Str(c)) => Some(c.chars().collect()),
        Some(v) => return Err(Exc::type_error(format!("strip arg must be None or str, not {}", v.type_name()))),
    };
    let pred = |c: char| match &set {
        None => py_isspace(c),
        Some(s) => s.contains(&c),
    };
    let mut r = s;
    if left {
        r = r.trim_start_matches(pred);
    }
    if right {
        r = r.trim_end_matches(pred);
    }
    Ok(r)
}

fn pad(s: &str, width: i64, fill: char, align: char) -> String {
    let len = s.chars().count() as i64;
    if width <= len {
        return s.to_string();
    }
    let total = (width - len) as usize;
    let (l, r) = match align {
        '<' => (0, total),
        '>' => (total, 0),
        _ => {
            // str.center: extra goes right unless both odd
            let left = total / 2 + (total & (width as usize) & 1);
            (left, total - left)
        }
    };
    format!("{}{}{}", fill.to_string().repeat(l), s, fill.to_string().repeat(r))
}

fn fill_char(v: Option<&Value>) -> VResult<char> {
    match v {
        None => Ok(' '),
        Some(Value::Str(s)) if s.chars().count() == 1 => Ok(s.chars().next().unwrap()),
        Some(Value::Str(_)) => Err(Exc::type_error("The fill character must be exactly one character long")),
        Some(v) => Err(Exc::type_error(format!("The fill character must be a unicode character, not {}", v.type_name()))),
    }
}

/// str.title()
pub fn str_title(s: &str) -> String {
    let mut out = String::new();
    let mut prev_cased = false;
    for c in s.chars() {
        if prev_cased {
            out.extend(c.to_lowercase());
        } else {
            out.extend(c.to_uppercase());
        }
        prev_cased = c.is_alphabetic();
    }
    out
}

/// str.capitalize()
pub fn str_capitalize(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        None => String::new(),
        Some(f) => f.to_uppercase().collect::<String>() + &chars.as_str().to_lowercase(),
    }
}

/// str.splitlines(keepends)
pub fn str_splitlines(s: &str, keepends: bool) -> Vec<String> {
    let chars: Vec<char> = s.chars().collect();
    let mut out = vec![];
    let mut start = 0;
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        let is_break = matches!(c, '\n' | '\r' | '\x0b' | '\x0c' | '\x1c' | '\x1d' | '\x1e' | '\u{85}' | '\u{2028}' | '\u{2029}');
        if is_break {
            let mut end = i + 1;
            if c == '\r' && chars.get(i + 1) == Some(&'\n') {
                end += 1;
            }
            let line: String = if keepends { chars[start..end].iter().collect() } else { chars[start..i].iter().collect() };
            out.push(line);
            start = end;
            i = end;
        } else {
            i += 1;
        }
    }
    if start < chars.len() {
        out.push(chars[start..].iter().collect());
    }
    out
}

fn kw_or_arg<'a>(args: &'a [Value], kwargs: &'a IndexMap<String, Value>, idx: usize, name: &str) -> Option<&'a Value> {
    args.get(idx).or_else(|| kwargs.get(name))
}

pub fn call_method(recv: &Value, name: &str, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    match recv {
        Value::Str(s) => str_method(s, name, args, kwargs),
        Value::List(_) => list_method(recv, name, args, kwargs),
        Value::Dict(_) => dict_method(recv, name, args, kwargs),
        Value::Tuple(t) => seq_method(&t.to_vec(), "tuple", name, args),
        Value::NamedTuple(nt) => match name {
            "_asdict" => Ok(Value::dict(nt.1.iter().zip(&nt.2).map(|(f, v)| (Key::str(f), v.clone())).collect())),
            "_replace" => {
                let mut values = nt.2.clone();
                for (k, v) in kwargs {
                    match nt.1.iter().position(|f| *f == k) {
                        Some(i) => values[i] = v,
                        None => return Err(Exc::type_error(format!("Got unexpected field names: [{}]", crate::pyfmt::py_str_repr(&k)))),
                    }
                }
                Ok(Value::NamedTuple(Rc::new((nt.0.clone(), nt.1.clone(), values))))
            }
            _ => seq_method(&nt.2, "tuple", name, args),
        },
        Value::Bool(_) | Value::Int(_) => {
            let i = recv.as_int().unwrap();
            match name {
                "bit_length" => Ok(Value::Int(64 - i.unsigned_abs().leading_zeros() as i64)),
                "bit_count" => Ok(Value::Int(i.unsigned_abs().count_ones() as i64)),
                "conjugate" => Ok(Value::Int(i)),
                _ => Ok(Value::Bool(true)),
            }
        }
        Value::Float(f) => match name {
            "is_integer" => Ok(Value::Bool(f.is_finite() && f.fract() == 0.0)),
            "conjugate" => Ok(Value::Float(*f)),
            _ => Err(Exc::attribute_error(format!("'float' object has no attribute '{}'", name))),
        },
        Value::Range(..) => seq_method(&recv.iter_items()?, "range", name, args),
        _ => Err(Exc::attribute_error(format!("'{}' object has no attribute '{}'", recv.type_name(), name))),
    }
}

fn seq_method(items: &[Value], tname: &str, name: &str, args: Vec<Value>) -> VResult<Value> {
    match name {
        "count" => {
            nargs("count", &args, 1, 1)?;
            Ok(Value::Int(items.iter().filter(|v| v.py_eq(&args[0])).count() as i64))
        }
        "index" => {
            nargs("index", &args, 1, 3)?;
            let (s, e) = slice_bounds(items.len(), args.get(1), args.get(2))?;
            for (i, v) in items.iter().enumerate().take(e).skip(s) {
                if v.py_eq(&args[0]) {
                    return Ok(Value::Int(i as i64));
                }
            }
            if tname == "range" {
                return Err(Exc::value_error(format!("{} is not in range", args[0].repr())));
            }
            Err(Exc::value_error(format!("{}.index(x): x not in {}", tname, tname)))
        }
        _ => Err(Exc::attribute_error(format!("'{}' object has no attribute '{}'", tname, name))),
    }
}

fn str_method(s: &str, name: &str, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let ok_str = |v: String| Ok(Value::from(v));
    match name {
        "format" => return super::strformat::str_format(s, &args, &kwargs).map(Value::from),
        "format_map" => {
            nargs("format_map", &args, 1, 1)?;
            let mut kw = IndexMap::default();
            if let Value::Dict(d) = &args[0] {
                for (k, v) in d.borrow().iter() {
                    kw.insert(k.0.to_str(), v.clone());
                }
            }
            return super::strformat::str_format(s, &[], &kw).map(Value::from);
        }
        "split" | "rsplit" => {
            let sep = kw_or_arg(&args, &kwargs, 0, "sep");
            let maxsplit = kw_or_arg(&args, &kwargs, 1, "maxsplit").map(want_int).transpose()?.unwrap_or(-1);
            let sep = match sep {
                None | Some(Value::None) => None,
                Some(v) => Some(want_str(v, "sep")?.to_string()),
            };
            let parts = str_split(s, sep.as_deref(), maxsplit, name == "rsplit")?;
            return Ok(Value::list(parts.into_iter().map(Value::from).collect()));
        }
        _ => {}
    }
    no_kwargs(name, &kwargs)?;
    match name {
        "upper" => ok_str(s.to_uppercase()),
        "lower" => ok_str(s.to_lowercase()),
        "casefold" => ok_str(s.to_lowercase()),
        "swapcase" => ok_str(s.chars().map(|c| if c.is_uppercase() { c.to_lowercase().collect::<String>() } else { c.to_uppercase().collect() }).collect()),
        "title" => ok_str(str_title(s)),
        "capitalize" => ok_str(str_capitalize(s)),
        "strip" | "lstrip" | "rstrip" => {
            nargs(name, &args, 0, 1)?;
            let r = strip_chars(s, args.first(), name != "rstrip", name != "lstrip")?;
            ok_str(r.to_string())
        }
        "startswith" | "endswith" => {
            nargs(name, &args, 1, 3)?;
            let chars: Vec<char> = s.chars().collect();
            let (st, en) = slice_bounds(chars.len(), args.get(1), args.get(2))?;
            let sub: String = if st <= en { chars[st..en].iter().collect() } else { String::new() };
            let test = |p: &str| if name == "startswith" { sub.starts_with(p) } else { sub.ends_with(p) };
            let res = match &args[0] {
                Value::Str(p) => test(p),
                Value::Tuple(t) => {
                    let mut r = false;
                    for x in t.iter() {
                        if test(want_str(x, "tuple for startswith")?) {
                            r = true;
                        }
                    }
                    r
                }
                v => {
                    return Err(Exc::type_error(format!(
                        "{} first arg must be str or a tuple of str, not {}",
                        name,
                        v.type_name()
                    )))
                }
            };
            Ok(Value::Bool(res))
        }
        "replace" => {
            nargs("replace", &args, 2, 3)?;
            let old = want_str(&args[0], "replace() argument 1")?;
            let new = want_str(&args[1], "replace() argument 2")?;
            let count = args.get(2).map(want_int).transpose()?.unwrap_or(-1);
            if count < 0 {
                if old.is_empty() {
                    // python inserts between every character
                    let mut out = new.to_string();
                    for c in s.chars() {
                        out.push(c);
                        out.push_str(new);
                    }
                    return ok_str(out);
                }
                return ok_str(s.replace(old, new));
            }
            ok_str(s.replacen(old, new, count as usize))
        }
        "find" | "rfind" | "index" | "rindex" | "count" => {
            nargs(name, &args, 1, 3)?;
            let sub = want_str(&args[0], "must be str")
                .map_err(|_| Exc::type_error(format!("must be str, not {}", args[0].type_name())))?;
            let chars: Vec<char> = s.chars().collect();
            let (st, en) = slice_bounds(chars.len(), args.get(1), args.get(2))?;
            let hay: String = if st <= en { chars[st..en].iter().collect() } else { String::new() };
            if name == "count" {
                if st > en {
                    return Ok(Value::Int(0));
                }
                if sub.is_empty() {
                    return Ok(Value::Int(hay.chars().count() as i64 + 1));
                }
                return Ok(Value::Int(hay.matches(sub).count() as i64));
            }
            let pos = if st > en {
                None
            } else if name.starts_with('r') {
                hay.rfind(sub)
            } else {
                hay.find(sub)
            };
            match pos {
                Some(byte) => Ok(Value::Int((hay[..byte].chars().count() + st) as i64)),
                None if name.ends_with("index") => Err(Exc::value_error("substring not found")),
                None => Ok(Value::Int(-1)),
            }
        }
        "join" => {
            nargs("join", &args, 1, 1)?;
            let items = args[0].iter_items()?;
            let mut parts = vec![];
            for (i, v) in items.iter().enumerate() {
                match v {
                    Value::Str(x) => parts.push(x.to_string()),
                    other => {
                        return Err(Exc::type_error(format!(
                            "sequence item {}: expected str instance, {} found",
                            i,
                            other.type_name()
                        )))
                    }
                }
            }
            ok_str(parts.join(s))
        }
        "zfill" => {
            nargs("zfill", &args, 1, 1)?;
            let width = want_int(&args[0])?;
            let len = s.chars().count() as i64;
            if width <= len {
                return ok_str(s.to_string());
            }
            let fill = "0".repeat((width - len) as usize);
            if s.starts_with('+') || s.starts_with('-') {
                return ok_str(format!("{}{}{}", &s[..1], fill, &s[1..]));
            }
            ok_str(format!("{}{}", fill, s))
        }
        "center" | "ljust" | "rjust" => {
            nargs(name, &args, 1, 2)?;
            let width = want_int(&args[0])?;
            let fill = fill_char(args.get(1))?;
            let align = match name {
                "ljust" => '<',
                "rjust" => '>',
                _ => '^',
            };
            ok_str(pad(s, width, fill, align))
        }
        "splitlines" => {
            nargs("splitlines", &args, 0, 1)?;
            let keepends = args.first().map(|v| v.truthy()).unwrap_or(false);
            Ok(Value::list(str_splitlines(s, keepends).into_iter().map(Value::from).collect()))
        }
        "partition" | "rpartition" => {
            nargs(name, &args, 1, 1)?;
            let sep = want_str(&args[0], "must be str")?;
            if sep.is_empty() {
                return Err(Exc::value_error("empty separator"));
            }
            let found = if name == "partition" { s.find(sep) } else { s.rfind(sep) };
            let t = match found {
                Some(p) => vec![Value::from(&s[..p]), Value::from(sep), Value::from(&s[p + sep.len()..])],
                None if name == "partition" => vec![Value::from(s), Value::from(""), Value::from("")],
                None => vec![Value::from(""), Value::from(""), Value::from(s)],
            };
            Ok(Value::tuple(t))
        }
        "removeprefix" => {
            nargs(name, &args, 1, 1)?;
            let p = want_str(&args[0], "removeprefix() argument")?;
            ok_str(s.strip_prefix(p).unwrap_or(s).to_string())
        }
        "removesuffix" => {
            nargs(name, &args, 1, 1)?;
            let p = want_str(&args[0], "removesuffix() argument")?;
            ok_str(s.strip_suffix(p).unwrap_or(s).to_string())
        }
        "isdigit" | "isdecimal" | "isnumeric" => Ok(Value::Bool(!s.is_empty() && s.chars().all(|c| c.is_numeric()))),
        "isalpha" => Ok(Value::Bool(!s.is_empty() && s.chars().all(|c| c.is_alphabetic()))),
        "isalnum" => Ok(Value::Bool(!s.is_empty() && s.chars().all(|c| c.is_alphanumeric()))),
        "isascii" => Ok(Value::Bool(s.is_ascii())),
        "isspace" => Ok(Value::Bool(!s.is_empty() && s.chars().all(py_isspace))),
        "islower" => Ok(Value::Bool(s.chars().any(|c| c.is_lowercase()) && !s.chars().any(|c| c.is_uppercase()))),
        "isupper" => Ok(Value::Bool(s.chars().any(|c| c.is_uppercase()) && !s.chars().any(|c| c.is_lowercase()))),
        "istitle" => Ok(Value::Bool(str_title(s) == s && s.chars().any(|c| c.is_alphabetic()))),
        "isidentifier" => {
            let mut cs = s.chars();
            Ok(Value::Bool(match cs.next() {
                Some(c) if c == '_' || c.is_alphabetic() => cs.all(|c| c == '_' || c.is_alphanumeric()),
                _ => false,
            }))
        }
        _ => Err(Exc::attribute_error(format!("'str' object has no attribute '{}'", name))),
    }
}

fn list_method(recv: &Value, name: &str, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let Value::List(l) = recv else { unreachable!() };
    if name == "sort" {
        nargs("sort", &args, 0, 0)?;
        let reverse = kwargs.get("reverse").map(|v| v.truthy()).unwrap_or(false);
        let key = kwargs.get("key").cloned();
        if key.as_ref().map(|k| !matches!(k, Value::None)).unwrap_or(false) {
            return Err(Exc::type_error("list.sort() with a key function is not supported in templates"));
        }
        let mut items = l.borrow().clone();
        sort_values(&mut items, reverse)?;
        *l.borrow_mut() = items;
        return Ok(Value::None);
    }
    no_kwargs(name, &kwargs)?;
    match name {
        "append" => {
            nargs("list.append", &args, 1, 1).map_err(|_| Exc::type_error(format!("list.append() takes exactly one argument ({} given)", args.len())))?;
            l.borrow_mut().push(args[0].clone());
            Ok(Value::None)
        }
        "extend" => {
            nargs("list.extend", &args, 1, 1)?;
            let items = args[0].iter_items()?;
            l.borrow_mut().extend(items);
            Ok(Value::None)
        }
        "insert" => {
            nargs("insert", &args, 2, 2)?;
            let len = l.borrow().len() as i64;
            let mut i = want_int(&args[0])?;
            if i < 0 {
                i = (i + len).max(0);
            }
            let i = i.min(len) as usize;
            l.borrow_mut().insert(i, args[1].clone());
            Ok(Value::None)
        }
        "pop" => {
            nargs("pop", &args, 0, 1)?;
            let len = l.borrow().len() as i64;
            if len == 0 {
                return Err(Exc::index_error("pop from empty list"));
            }
            let mut i = args.first().map(want_int).transpose()?.unwrap_or(-1);
            if i < 0 {
                i += len;
            }
            if i < 0 || i >= len {
                return Err(Exc::index_error("pop index out of range"));
            }
            Ok(l.borrow_mut().remove(i as usize))
        }
        "remove" => {
            nargs("list.remove", &args, 1, 1)?;
            let pos = l.borrow().iter().position(|v| v.py_eq(&args[0]));
            match pos {
                Some(p) => {
                    l.borrow_mut().remove(p);
                    Ok(Value::None)
                }
                None => Err(Exc::value_error("list.remove(x): x not in list")),
            }
        }
        "reverse" => {
            l.borrow_mut().reverse();
            Ok(Value::None)
        }
        "copy" => Ok(Value::list(l.borrow().clone())),
        "clear" => {
            l.borrow_mut().clear();
            Ok(Value::None)
        }
        _ => {
            let items = l.borrow().clone();
            seq_method(&items, "list", name, args)
        }
    }
}

/// list.sort() / sorted() with python's comparisons (a stable sort)
pub fn sort_values(items: &mut [Value], reverse: bool) -> VResult<()> {
    let mut err = None;
    items.sort_by(|a, b| {
        if err.is_some() {
            return std::cmp::Ordering::Equal;
        }
        let (x, y) = if reverse { (b, a) } else { (a, b) };
        match y.compare(x, "lt") {
            Ok(true) => std::cmp::Ordering::Greater,
            Ok(false) => match x.compare(y, "lt") {
                Ok(true) => std::cmp::Ordering::Less,
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
    match err {
        Some(e) => Err(e),
        None => Ok(()),
    }
}

fn dict_method(recv: &Value, name: &str, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let Value::Dict(d) = recv else { unreachable!() };
    if name == "update" {
        nargs("update", &args, 0, 1)?;
        if let Some(a) = args.first() {
            match a {
                Value::Dict(o) => {
                    let items: Vec<(Key, Value)> = o.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect();
                    d.borrow_mut().extend(items);
                }
                other => {
                    for item in other.iter_items()? {
                        let pair = item.iter_items()?;
                        if pair.len() != 2 {
                            return Err(Exc::value_error(format!(
                                "dictionary update sequence element #0 has length {}; 2 is required",
                                pair.len()
                            )));
                        }
                        d.borrow_mut().insert(Key::hashable(&pair[0])?, pair[1].clone());
                    }
                }
            }
        }
        for (k, v) in kwargs {
            d.borrow_mut().insert(Key::str(&k), v);
        }
        return Ok(Value::None);
    }
    no_kwargs(name, &kwargs)?;
    match name {
        "get" => {
            nargs("get", &args, 1, 2)?;
            let key = Key::hashable(&args[0])?;
            Ok(d.borrow().get(&key).cloned().unwrap_or_else(|| args.get(1).cloned().unwrap_or(Value::None)))
        }
        "keys" => Ok(Value::object(DictView { kind: "dict_keys", items: d.borrow().keys().map(|k| k.0.clone()).collect() })),
        "values" => Ok(Value::object(DictView { kind: "dict_values", items: d.borrow().values().cloned().collect() })),
        "items" => Ok(Value::object(DictView {
            kind: "dict_items",
            items: d.borrow().iter().map(|(k, v)| Value::tuple(vec![k.0.clone(), v.clone()])).collect(),
        })),
        "pop" => {
            nargs("pop", &args, 1, 2)?;
            let key = Key::hashable(&args[0])?;
            let removed = d.borrow_mut().shift_remove(&key);
            match removed {
                Some(v) => Ok(v),
                None => args.get(1).cloned().ok_or_else(|| Exc::key_error(&args[0])),
            }
        }
        "popitem" => {
            let item = d.borrow_mut().pop();
            match item {
                Some((k, v)) => Ok(Value::tuple(vec![k.0, v])),
                None => Err(Exc::new("KeyError", "'popitem(): dictionary is empty'")),
            }
        }
        "setdefault" => {
            nargs("setdefault", &args, 1, 2)?;
            let key = Key::hashable(&args[0])?;
            let default = args.get(1).cloned().unwrap_or(Value::None);
            Ok(d.borrow_mut().entry(key).or_insert(default).clone())
        }
        "copy" => Ok(Value::dict(d.borrow().clone())),
        "clear" => {
            d.borrow_mut().clear();
            Ok(Value::None)
        }
        _ => Err(Exc::attribute_error(format!("'dict' object has no attribute '{}'", name))),
    }
}
