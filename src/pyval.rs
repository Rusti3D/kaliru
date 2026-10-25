// A dynamically typed value with python semantics, for the places klippy
// passes python objects around: config access tracking, get_status() data,
// webhooks payloads. Unlike serde_json::Value it keeps int vs float, allows
// inf / nan and tells tuples from lists, and it formats like python's str(),
// repr() and json.dumps().
use crate::pyfmt::{py_float_repr, py_json_float_into, py_str_repr};
use crate::IndexMap;

/// a dict key: field names are static, so a status dict built every poll
/// allocates no key strings (and clones of it copy none)
pub type PyKey = std::borrow::Cow<'static, str>;
pub type PyDict = IndexMap<PyKey, PyVal>;

/// A dict of static field names, sized up front (a status built every poll
/// allocates its table once instead of growing it):
/// `pydict! { "power" => v, "value" => w }` inserts in that order
#[macro_export]
macro_rules! pydict {
    ($($k:literal => $v:expr),* $(,)?) => {{
        let mut d = $crate::pyval::PyDict::with_capacity_and_hasher(
            <[()]>::len(&[$($crate::pydict!(@one $k)),*]),
            ::std::default::Default::default(),
        );
        $( d.insert(::std::borrow::Cow::Borrowed($k), $v); )*
        d
    }};
    (@one $k:literal) => { () };
}

#[derive(Clone, Debug, PartialEq)]
pub enum PyVal {
    None,
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
    List(Vec<PyVal>),
    Tuple(Vec<PyVal>),
    /// a collections.namedtuple instance: (type name, field names, values)
    NamedTuple(&'static str, &'static [&'static str], Vec<PyVal>),
    Dict(PyDict),
    /// a value handed out by reference, as python hands out its objects:
    /// a big status dict kept by its owner (configfile, gcode commands);
    /// copies share it and py_eq of the same object is immediate
    Shared(std::rc::Rc<PyVal>),
}

impl PyVal {
    /// the value itself (through Shared)
    pub fn get(&self) -> &PyVal {
        match self {
            PyVal::Shared(v) => v.get(),
            v => v,
        }
    }

    /// a Shared value
    pub fn shared(v: PyVal) -> PyVal {
        PyVal::Shared(std::rc::Rc::new(v))
    }

    /// python repr()
    pub fn repr(&self) -> String {
        match self {
            PyVal::None => "None".into(),
            PyVal::Bool(b) => if *b { "True" } else { "False" }.into(),
            PyVal::Int(i) => i.to_string(),
            PyVal::Float(f) => py_float_repr(*f),
            PyVal::Str(s) => py_str_repr(s),
            PyVal::List(l) => format!("[{}]", l.iter().map(|v| v.repr()).collect::<Vec<_>>().join(", ")),
            PyVal::Tuple(t) => {
                let items: Vec<String> = t.iter().map(|v| v.repr()).collect();
                if items.len() == 1 {
                    format!("({},)", items[0])
                } else {
                    format!("({})", items.join(", "))
                }
            }
            PyVal::NamedTuple(name, fields, values) => format!(
                "{}({})",
                name,
                fields.iter().zip(values).map(|(f, v)| format!("{}={}", f, v.repr())).collect::<Vec<_>>().join(", ")
            ),
            PyVal::Dict(d) => format!(
                "{{{}}}",
                d.iter().map(|(k, v)| format!("{}: {}", py_str_repr(k), v.repr())).collect::<Vec<_>>().join(", ")
            ),
            PyVal::Shared(v) => v.repr(),
        }
    }

    /// python str()
    pub fn str(&self) -> String {
        match self.get() {
            PyVal::Str(s) => s.clone(),
            v => v.repr(),
        }
    }

    /// python json.dumps() with default arguments
    pub fn json(&self) -> String {
        let mut out = String::new();
        self.json_into(&mut out);
        out
    }

    fn json_into(&self, out: &mut String) {
        match self {
            PyVal::None => out.push_str("null"),
            PyVal::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
            PyVal::Int(i) => {
                use std::fmt::Write;
                let _ = write!(out, "{}", i);
            }
            PyVal::Float(f) => py_json_float_into(out, *f),
            PyVal::Str(s) => json_str_into(out, s),
            PyVal::List(a) | PyVal::Tuple(a) | PyVal::NamedTuple(_, _, a) => {
                out.push('[');
                for (i, x) in a.iter().enumerate() {
                    if i > 0 {
                        out.push_str(", ");
                    }
                    x.json_into(out);
                }
                out.push(']');
            }
            PyVal::Dict(o) => {
                out.push('{');
                for (i, (k, x)) in o.iter().enumerate() {
                    if i > 0 {
                        out.push_str(", ");
                    }
                    json_str_into(out, k);
                    out.push_str(": ");
                    x.json_into(out);
                }
                out.push('}');
            }
            PyVal::Shared(v) => v.json_into(out),
        }
    }

    /// float(v) for numbers and bools
    pub fn as_f64(&self) -> Option<f64> {
        match self.get() {
            PyVal::Bool(b) => Some(*b as i64 as f64),
            PyVal::Int(i) => Some(*i as f64),
            PyVal::Float(f) => Some(*f),
            _ => None,
        }
    }

    /// int(v) for numbers and bools (floats truncate)
    pub fn as_i64(&self) -> Option<i64> {
        match self.get() {
            PyVal::Bool(b) => Some(*b as i64),
            PyVal::Int(i) => Some(*i),
            PyVal::Float(f) => Some(f.trunc() as i64),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self.get() {
            PyVal::Str(s) => Some(s),
            _ => None,
        }
    }

    pub fn items(&self) -> Option<&[PyVal]> {
        match self.get() {
            PyVal::List(a) | PyVal::Tuple(a) | PyVal::NamedTuple(_, _, a) => Some(a),
            _ => None,
        }
    }

    /// python bool(v)
    pub fn truthy(&self) -> bool {
        match self {
            PyVal::None => false,
            PyVal::Bool(b) => *b,
            PyVal::Int(i) => *i != 0,
            PyVal::Float(f) => *f != 0.,
            PyVal::Str(s) => !s.is_empty(),
            PyVal::List(a) | PyVal::Tuple(a) | PyVal::NamedTuple(_, _, a) => !a.is_empty(),
            PyVal::Dict(d) => !d.is_empty(),
            PyVal::Shared(v) => v.truthy(),
        }
    }

    pub fn dict() -> Self {
        PyVal::Dict(IndexMap::default())
    }

    /// json.dump(v, fp, indent=n)
    pub fn json_indent(&self, indent: usize) -> String {
        self.json_indent_at(indent, 0)
    }

    fn json_indent_at(&self, indent: usize, level: usize) -> String {
        let pad = |l: usize| " ".repeat(indent * l);
        match self.get() {
            PyVal::List(a) | PyVal::Tuple(a) | PyVal::NamedTuple(_, _, a) if !a.is_empty() => {
                let items: Vec<String> = a.iter().map(|x| format!("{}{}", pad(level + 1), x.json_indent_at(indent, level + 1))).collect();
                format!("[\n{}\n{}]", items.join(",\n"), pad(level))
            }
            PyVal::Dict(d) if !d.is_empty() => {
                let items: Vec<String> = d
                    .iter()
                    .map(|(k, x)| format!("{}{}: {}", pad(level + 1), PyVal::Str(k.to_string()).json(), x.json_indent_at(indent, level + 1)))
                    .collect();
                format!("{{\n{}\n{}}}", items.join(",\n"), pad(level))
            }
            v => v.json(),
        }
    }

    /// json.dumps(v, separators=(",", ":")) (webhooks responses)
    pub fn json_compact(&self) -> String {
        let mut out = String::new();
        self.json_compact_into(&mut out);
        out
    }

    /// json_compact() appended to `out`
    pub fn json_compact_into(&self, out: &mut String) {
        self.json_sep(out, ",", ":");
    }

    fn json_sep(&self, out: &mut String, item: &str, key: &str) {
        match self {
            PyVal::List(a) | PyVal::Tuple(a) | PyVal::NamedTuple(_, _, a) => {
                out.push('[');
                for (i, x) in a.iter().enumerate() {
                    if i > 0 {
                        out.push_str(item);
                    }
                    x.json_sep(out, item, key);
                }
                out.push(']');
            }
            PyVal::Dict(o) => {
                out.push('{');
                for (i, (k, x)) in o.iter().enumerate() {
                    if i > 0 {
                        out.push_str(item);
                    }
                    json_str_into(out, k);
                    out.push_str(key);
                    x.json_sep(out, item, key);
                }
                out.push('}');
            }
            PyVal::Shared(v) => v.json_sep(out, item, key),
            v => v.json_into(out),
        }
    }

    /// python ==
    pub fn py_eq(&self, other: &PyVal) -> bool {
        use PyVal::*;
        if let (Shared(a), Shared(b)) = (self, other) {
            // python compares the same object without looking inside
            if std::rc::Rc::ptr_eq(a, b) {
                return true;
            }
        }
        let (this, other) = (self.get(), other.get());
        match (this, other) {
            (None, None) => true,
            (Str(a), Str(b)) => a == b,
            (List(a), List(b)) | (Tuple(a) | NamedTuple(_, _, a), Tuple(b) | NamedTuple(_, _, b)) => {
                a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.py_eq(y))
            }
            (Dict(a), Dict(b)) => a.len() == b.len() && a.iter().all(|(k, v)| b.get(k).map(|w| v.py_eq(w)).unwrap_or(false)),
            (Bool(_) | Int(_), Bool(_) | Int(_)) => this.as_i64() == other.as_i64(),
            (Bool(_) | Int(_) | Float(_), Bool(_) | Int(_) | Float(_)) => this.as_f64() == other.as_f64(),
            _ => false,
        }
    }

    /// json.loads(s) (python 3.13 defaults: NaN / Infinity accepted, strict
    /// strings, duplicate keys keep the last value); Err is the error text
    pub fn json_loads(s: &str) -> Result<PyVal, String> {
        let chars: Vec<char> = s.chars().collect();
        let mut p = JsonParser { s: &chars, pos: 0 };
        p.ws();
        let v = p.value()?;
        p.ws();
        if p.pos != chars.len() {
            return Err(p.err("Extra data"));
        }
        Ok(v)
    }
}

struct JsonParser<'a> {
    s: &'a [char],
    pos: usize,
}

impl JsonParser<'_> {
    fn err(&self, what: &str) -> String {
        // python: "<msg>: line L column C (char P)"
        let before = &self.s[..self.pos.min(self.s.len())];
        let line = before.iter().filter(|c| **c == '\n').count() + 1;
        let col = self.pos - before.iter().rposition(|c| *c == '\n').map(|p| p + 1).unwrap_or(0) + 1;
        format!("{}: line {} column {} (char {})", what, line, col, self.pos)
    }
    fn ws(&mut self) {
        while matches!(self.s.get(self.pos), Some(' ' | '\t' | '\n' | '\r')) {
            self.pos += 1;
        }
    }
    fn lit(&mut self, word: &str) -> bool {
        let w: Vec<char> = word.chars().collect();
        if self.s[self.pos..].starts_with(&w) {
            self.pos += w.len();
            true
        } else {
            false
        }
    }
    fn value(&mut self) -> Result<PyVal, String> {
        match self.s.get(self.pos) {
            Some('{') => self.object(),
            Some('[') => self.array(),
            Some('"') => {
                self.pos += 1;
                Ok(PyVal::Str(self.string()?))
            }
            Some('n') if self.lit("null") => Ok(PyVal::None),
            Some('t') if self.lit("true") => Ok(PyVal::Bool(true)),
            Some('f') if self.lit("false") => Ok(PyVal::Bool(false)),
            Some('N') if self.lit("NaN") => Ok(PyVal::Float(f64::NAN)),
            Some('I') if self.lit("Infinity") => Ok(PyVal::Float(f64::INFINITY)),
            Some('-') if self.lit("-Infinity") => Ok(PyVal::Float(f64::NEG_INFINITY)),
            Some(c) if *c == '-' || c.is_ascii_digit() => self.number(),
            _ => Err(self.err("Expecting value")),
        }
    }
    fn number(&mut self) -> Result<PyVal, String> {
        // (-?(?:0|[1-9]\d*))(\.\d+)?([eE][-+]?\d+)?
        let start = self.pos;
        let digits = |p: &mut Self| {
            let s = p.pos;
            while p.s.get(p.pos).map(|c| c.is_ascii_digit()).unwrap_or(false) {
                p.pos += 1;
            }
            p.pos - s
        };
        if self.s.get(self.pos) == Some(&'-') {
            self.pos += 1;
        }
        if self.s.get(self.pos) == Some(&'0') {
            self.pos += 1;
        } else if digits(self) == 0 {
            self.pos = start;
            return Err(self.err("Expecting value"));
        }
        let int_end = self.pos;
        let mut is_float = false;
        if self.s.get(self.pos) == Some(&'.') && self.s.get(self.pos + 1).map(|c| c.is_ascii_digit()).unwrap_or(false) {
            self.pos += 1;
            digits(self);
            is_float = true;
        }
        if matches!(self.s.get(self.pos), Some('e' | 'E')) {
            let save = self.pos;
            self.pos += 1;
            if matches!(self.s.get(self.pos), Some('+' | '-')) {
                self.pos += 1;
            }
            if digits(self) == 0 {
                self.pos = save;
            } else {
                is_float = true;
            }
        }
        let text: String = self.s[start..self.pos].iter().collect();
        if is_float {
            Ok(PyVal::Float(text.parse().unwrap_or(f64::NAN)))
        } else {
            // ponytail: python ints are unbounded; beyond i64 parsed as float
            let _ = int_end;
            Ok(text.parse::<i64>().map(PyVal::Int).unwrap_or_else(|_| PyVal::Float(text.parse().unwrap_or(0.))))
        }
    }
    fn hex4(&mut self) -> Result<u32, String> {
        let h: String = self.s.get(self.pos..self.pos + 4).map(|c| c.iter().collect()).unwrap_or_default();
        let v = u32::from_str_radix(&h, 16).map_err(|_| {
            let mut e = self.err("Invalid \\uXXXX escape");
            e = e.replace(&format!("(char {})", self.pos), &format!("(char {})", self.pos - 1));
            e
        })?;
        self.pos += 4;
        Ok(v)
    }
    fn string(&mut self) -> Result<String, String> {
        let start = self.pos - 1;
        let mut out = String::new();
        loop {
            let Some(&c) = self.s.get(self.pos) else {
                self.pos = start;
                return Err(self.err("Unterminated string starting at"));
            };
            self.pos += 1;
            match c {
                '"' => return Ok(out),
                '\\' => {
                    let Some(&e) = self.s.get(self.pos) else {
                        self.pos = start;
                        return Err(self.err("Unterminated string starting at"));
                    };
                    self.pos += 1;
                    match e {
                        '"' => out.push('"'),
                        '\\' => out.push('\\'),
                        '/' => out.push('/'),
                        'b' => out.push('\x08'),
                        'f' => out.push('\x0c'),
                        'n' => out.push('\n'),
                        'r' => out.push('\r'),
                        't' => out.push('\t'),
                        'u' => {
                            let mut u = self.hex4()?;
                            if (0xd800..0xdc00).contains(&u) && self.s.get(self.pos..self.pos + 2) == Some(&['\\', 'u']) {
                                let save = self.pos;
                                self.pos += 2;
                                let lo = self.hex4()?;
                                if (0xdc00..0xe000).contains(&lo) {
                                    u = 0x10000 + ((u - 0xd800) << 10) + (lo - 0xdc00);
                                } else {
                                    self.pos = save;
                                }
                            }
                            // python keeps lone surrogates; rust strings can't
                            out.push(char::from_u32(u).unwrap_or('\u{fffd}'));
                        }
                        _ => {
                            self.pos -= 2;
                            return Err(self.err("Invalid \\escape"));
                        }
                    }
                }
                c if (c as u32) < 0x20 => {
                    self.pos -= 1;
                    return Err(self.err("Invalid control character at"));
                }
                c => out.push(c),
            }
        }
    }
    fn array(&mut self) -> Result<PyVal, String> {
        self.pos += 1;
        let mut out = vec![];
        self.ws();
        if self.s.get(self.pos) == Some(&']') {
            self.pos += 1;
            return Ok(PyVal::List(out));
        }
        loop {
            self.ws();
            out.push(self.value()?);
            self.ws();
            match self.s.get(self.pos) {
                Some(',') => self.pos += 1,
                Some(']') => {
                    self.pos += 1;
                    return Ok(PyVal::List(out));
                }
                _ => return Err(self.err("Expecting ',' delimiter")),
            }
        }
    }
    fn object(&mut self) -> Result<PyVal, String> {
        self.pos += 1;
        let mut out = IndexMap::default();
        self.ws();
        if self.s.get(self.pos) == Some(&'}') {
            self.pos += 1;
            return Ok(PyVal::Dict(out));
        }
        loop {
            self.ws();
            if self.s.get(self.pos) != Some(&'"') {
                return Err(self.err("Expecting property name enclosed in double quotes"));
            }
            self.pos += 1;
            let k = self.string()?;
            self.ws();
            if self.s.get(self.pos) != Some(&':') {
                return Err(self.err("Expecting ':' delimiter"));
            }
            self.pos += 1;
            self.ws();
            let v = self.value()?;
            out.insert(k.into(), v);
            self.ws();
            match self.s.get(self.pos) {
                Some(',') => self.pos += 1,
                Some('}') => {
                    self.pos += 1;
                    return Ok(PyVal::Dict(out));
                }
                _ => return Err(self.err("Expecting ',' delimiter")),
            }
        }
    }
}

fn json_str_into(out: &mut String, s: &str) {
    out.push('"');
    // (the common case: printable ascii without quotes or backslashes, as is)
    if s.bytes().all(|b| (0x20..=0x7e).contains(&b) && b != b'"' && b != b'\\') {
        out.push_str(s);
        out.push('"');
        return;
    }
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\x08' => out.push_str("\\b"),
            '\x0c' => out.push_str("\\f"),
            // ensure_ascii: everything outside printable ascii as \uXXXX
            c if (c as u32) < 0x20 || (c as u32) > 0x7e => {
                let mut buf = [0u16; 2];
                for u in c.encode_utf16(&mut buf) {
                    out.push_str(&format!("\\u{:04x}", u));
                }
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

impl From<bool> for PyVal {
    fn from(v: bool) -> Self {
        PyVal::Bool(v)
    }
}
impl From<i64> for PyVal {
    fn from(v: i64) -> Self {
        PyVal::Int(v)
    }
}
impl From<i32> for PyVal {
    fn from(v: i32) -> Self {
        PyVal::Int(v as i64)
    }
}
impl From<f64> for PyVal {
    fn from(v: f64) -> Self {
        PyVal::Float(v)
    }
}
impl From<&str> for PyVal {
    fn from(v: &str) -> Self {
        PyVal::Str(v.to_string())
    }
}
impl From<String> for PyVal {
    fn from(v: String) -> Self {
        PyVal::Str(v)
    }
}
impl<T: Into<PyVal>> From<Option<T>> for PyVal {
    fn from(v: Option<T>) -> Self {
        v.map(Into::into).unwrap_or(PyVal::None)
    }
}

/// a json value (as python's json.loads() would have made it)
impl From<&serde_json::Value> for PyVal {
    fn from(v: &serde_json::Value) -> Self {
        use serde_json::Value;
        match v {
            Value::Null => PyVal::None,
            Value::Bool(b) => PyVal::Bool(*b),
            Value::Number(n) => match n.as_i64() {
                Some(i) => PyVal::Int(i),
                None => PyVal::Float(n.as_f64().unwrap_or(0.)),
            },
            Value::String(s) => PyVal::Str(s.clone()),
            Value::Array(a) => PyVal::List(a.iter().map(PyVal::from).collect()),
            Value::Object(o) => PyVal::Dict(o.iter().map(|(k, v)| (k.clone().into(), PyVal::from(v))).collect()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // the per-character escaping without the ascii fast path (rustify): the
    // oracle for json_str_into
    fn old_json_str(s: &str) -> String {
        let mut out = String::from('"');
        for c in s.chars() {
            match c {
                '"' => out.push_str("\\\""),
                '\\' => out.push_str("\\\\"),
                '\n' => out.push_str("\\n"),
                '\r' => out.push_str("\\r"),
                '\t' => out.push_str("\\t"),
                '\x08' => out.push_str("\\b"),
                '\x0c' => out.push_str("\\f"),
                c if (c as u32) < 0x20 || (c as u32) > 0x7e => {
                    let mut buf = [0u16; 2];
                    for u in c.encode_utf16(&mut buf) {
                        out.push_str(&format!("\\u{:04x}", u));
                    }
                }
                c => out.push(c),
            }
        }
        out.push('"');
        out
    }

    #[test]
    fn json_str_matches_the_per_char_escaping() {
        let mut cases: Vec<String> = ["", "status", "a b", "q\"x", "back\\slash", "tab\t", "nl\n", "~", "\x7f", "é", "日本", "😀", "\x00\x1f"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        // every single ascii character and pairs of them
        for a in 0u8..128 {
            cases.push((a as char).to_string());
            for b in [b'a', b'"', b'\\', 0x1f, 0x7e, 0x7f] {
                cases.push(format!("{}{}", a as char, b as char));
            }
        }
        for s in &cases {
            let mut out = String::new();
            json_str_into(&mut out, s);
            assert_eq!(out, old_json_str(s), "{:?}", s);
        }
    }

    #[test]
    fn formats_like_python() {
        let mut d = IndexMap::default();
        d.insert("a".into(), PyVal::Tuple(vec![PyVal::Int(1), PyVal::Float(2.0)]));
        d.insert("b".into(), PyVal::Float(f64::INFINITY));
        d.insert("é".into(), PyVal::None);
        let v = PyVal::Dict(d);
        assert_eq!(v.repr(), "{'a': (1, 2.0), 'b': inf, 'é': None}");
        assert_eq!(v.json(), "{\"a\": [1, 2.0], \"b\": Infinity, \"\\u00e9\": null}");
        assert_eq!(PyVal::Tuple(vec![PyVal::Str("x".into())]).repr(), "('x',)");
    }
}
