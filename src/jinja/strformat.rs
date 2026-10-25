// python str.format() and format(value, spec) (PEP 3101) for template
// values (port of CPython's Objects/stringlib/unicode_format.h and
// Python/formatter_unicode.c behaviour)
use super::value::{Exc, Key, VResult, Value};
use crate::pyfmt::py_float_repr;
use crate::pyformat::float_body;
use crate::IndexMap;

/// str.format(*args, **kwargs)
pub fn str_format(fmt: &str, args: &[Value], kwargs: &IndexMap<String, Value>) -> VResult<String> {
    let chars: Vec<char> = fmt.chars().collect();
    let mut out = String::new();
    let mut auto_idx: Option<usize> = None;
    let mut manual = false;
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c == '}' {
            if chars.get(i + 1) == Some(&'}') {
                out.push('}');
                i += 2;
                continue;
            }
            return Err(Exc::value_error("Single '}' encountered in format string"));
        }
        if c != '{' {
            out.push(c);
            i += 1;
            continue;
        }
        if chars.get(i + 1) == Some(&'{') {
            out.push('{');
            i += 2;
            continue;
        }
        // a replacement field up to the matching '}'
        let start = i + 1;
        let mut depth = 1;
        let mut j = start;
        while j < chars.len() {
            match chars[j] {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                _ => {}
            }
            j += 1;
        }
        if j >= chars.len() {
            return Err(Exc::value_error(if depth > 1 { "unmatched '{' in format spec" } else { "Single '{' encountered in format string" }));
        }
        let field: String = chars[start..j].iter().collect();
        i = j + 1;
        // field_name [!conversion] [:format_spec]
        let (name_conv, spec) = split_field(&field);
        let (name, conv) = match name_conv.find('!') {
            Some(p) => {
                let conv = &name_conv[p + 1..];
                if conv.chars().count() != 1 {
                    return Err(Exc::value_error(if conv.is_empty() {
                        "end of string while looking for conversion specifier"
                    } else {
                        "expected ':' after conversion specifier"
                    }));
                }
                (&name_conv[..p], conv.chars().next())
            }
            None => (name_conv.as_str(), None),
        };
        let value = get_field(name, args, kwargs, &mut auto_idx, &mut manual)?;
        let value = match conv {
            None => value,
            Some('s') => Value::from(value.to_str()),
            Some('r') => Value::from(value.repr()),
            Some('a') => Value::from(value.repr().chars().map(|c| if (c as u32) < 128 { c.to_string() } else { format!("\\u{:04x}", c as u32) }).collect::<String>()),
            Some(c) => return Err(Exc::value_error(format!("Unknown conversion specifier {}", c))),
        };
        // nested replacement fields in the spec
        let spec = if spec.contains('{') { str_format_nested(&spec, args, kwargs, &mut auto_idx, &mut manual)? } else { spec };
        out.push_str(&format_value(&value, &spec)?);
    }
    Ok(out)
}

fn split_field(field: &str) -> (String, String) {
    // the first ':' outside brackets starts the spec
    let mut bracket = false;
    for (i, c) in field.char_indices() {
        match c {
            '[' => bracket = true,
            ']' => bracket = false,
            ':' if !bracket => return (field[..i].to_string(), field[i + 1..].to_string()),
            _ => {}
        }
    }
    (field.to_string(), String::new())
}

fn str_format_nested(
    spec: &str,
    args: &[Value],
    kwargs: &IndexMap<String, Value>,
    auto_idx: &mut Option<usize>,
    manual: &mut bool,
) -> VResult<String> {
    let mut out = String::new();
    let mut rest = spec;
    while let Some(p) = rest.find('{') {
        out.push_str(&rest[..p]);
        let Some(e) = rest[p..].find('}') else {
            return Err(Exc::value_error("unmatched '{' in format spec"));
        };
        let name = &rest[p + 1..p + e];
        let v = get_field(name, args, kwargs, auto_idx, manual)?;
        out.push_str(&v.to_str());
        rest = &rest[p + e + 1..];
    }
    out.push_str(rest);
    Ok(out)
}

fn get_field(
    name: &str,
    args: &[Value],
    kwargs: &IndexMap<String, Value>,
    auto_idx: &mut Option<usize>,
    manual: &mut bool,
) -> VResult<Value> {
    // first part up to '.' or '['
    let end = name.find(['.', '[']).unwrap_or(name.len());
    let first = &name[..end];
    let mut value = if first.is_empty() {
        if *manual {
            return Err(Exc::value_error("cannot switch from manual field specification to automatic field numbering"));
        }
        let idx = auto_idx.map(|i| i + 1).unwrap_or(0);
        *auto_idx = Some(idx);
        args.get(idx)
            .cloned()
            .ok_or_else(|| Exc::index_error(format!("Replacement index {} out of range for positional args tuple", idx)))?
    } else if first.chars().all(|c| c.is_ascii_digit()) {
        if auto_idx.is_some() {
            return Err(Exc::value_error("cannot switch from automatic field numbering to manual field specification"));
        }
        *manual = true;
        let idx: usize = first.parse().unwrap_or(usize::MAX);
        args.get(idx)
            .cloned()
            .ok_or_else(|| Exc::index_error(format!("Replacement index {} out of range for positional args tuple", idx)))?
    } else {
        kwargs.get(first).cloned().ok_or_else(|| Exc::key_error(&Value::from(first)))?
    };
    // .attr and [key] accessors
    let mut rest = &name[end..];
    while !rest.is_empty() {
        if let Some(r) = rest.strip_prefix('.') {
            let e = r.find(['.', '[']).unwrap_or(r.len());
            let attr = &r[..e];
            if attr.is_empty() {
                return Err(Exc::value_error("Empty attribute in format string"));
            }
            value = super::eval::py_getattr(&value, attr)
                .ok_or_else(|| Exc::attribute_error(format!("'{}' object has no attribute '{}'", value.type_name(), attr)))?;
            rest = &r[e..];
        } else if let Some(r) = rest.strip_prefix('[') {
            let Some(e) = r.find(']') else {
                return Err(Exc::value_error("Missing ']' in format string"));
            };
            let key = &r[..e];
            let k = if !key.is_empty() && key.chars().all(|c| c.is_ascii_digit()) {
                Value::Int(key.parse().unwrap_or(0))
            } else {
                Value::from(key)
            };
            value = super::eval::py_getitem(&value, &k)?;
            rest = &r[e + 1..];
        } else {
            return Err(Exc::value_error("Only '.' or '[' may follow ']' in format field specifier"));
        }
    }
    let _ = Key::str("");
    Ok(value)
}

struct Spec {
    fill: char,
    align: Option<char>,
    sign: char,
    alt: bool,
    zero: bool,
    width: usize,
    grouping: Option<char>,
    precision: Option<usize>,
    ty: Option<char>,
}

fn parse_spec(spec: &str) -> VResult<Spec> {
    let c: Vec<char> = spec.chars().collect();
    let mut i = 0;
    let mut s = Spec { fill: ' ', align: None, sign: '-', alt: false, zero: false, width: 0, grouping: None, precision: None, ty: None };
    let is_align = |ch: char| matches!(ch, '<' | '>' | '=' | '^');
    if c.len() >= 2 && is_align(c[1]) {
        s.fill = c[0];
        s.align = Some(c[1]);
        i = 2;
    } else if !c.is_empty() && is_align(c[0]) {
        s.align = Some(c[0]);
        i = 1;
    }
    if i < c.len() && matches!(c[i], '+' | '-' | ' ') {
        s.sign = c[i];
        i += 1;
    }
    if i < c.len() && c[i] == 'z' {
        i += 1;
    }
    if i < c.len() && c[i] == '#' {
        s.alt = true;
        i += 1;
    }
    if i < c.len() && c[i] == '0' {
        s.zero = true;
        i += 1;
    }
    while i < c.len() && c[i].is_ascii_digit() {
        s.width = s.width * 10 + c[i].to_digit(10).unwrap() as usize;
        i += 1;
    }
    if i < c.len() && (c[i] == ',' || c[i] == '_') {
        s.grouping = Some(c[i]);
        i += 1;
    }
    if i < c.len() && c[i] == '.' {
        i += 1;
        let st = i;
        let mut p = 0;
        while i < c.len() && c[i].is_ascii_digit() {
            p = p * 10 + c[i].to_digit(10).unwrap() as usize;
            i += 1;
        }
        if i == st {
            return Err(Exc::value_error("Format specifier missing precision"));
        }
        s.precision = Some(p);
    }
    if i < c.len() {
        s.ty = Some(c[i]);
        i += 1;
    }
    if i < c.len() {
        return Err(Exc::value_error("Invalid format specifier"));
    }
    Ok(s)
}

fn group_digits(digits: &str, sep: char, every: usize) -> String {
    let chars: Vec<char> = digits.chars().collect();
    let mut out = String::new();
    for (i, ch) in chars.iter().enumerate() {
        if i > 0 && (chars.len() - i).is_multiple_of(every) {
            out.push(sep);
        }
        out.push(*ch);
    }
    out
}

/// apply fill / align / width to (sign+prefix, body)
fn finish(s: &Spec, prefix: &str, body: &str, numeric: bool) -> String {
    let len = prefix.chars().count() + body.chars().count();
    let (fill, align) = match (s.align, s.zero) {
        (Some(a), _) => (s.fill, a),
        (None, true) if numeric => ('0', '='),
        (None, _) => (s.fill, if numeric { '>' } else { '<' }),
    };
    let fill = if s.align.is_none() && s.zero && numeric { '0' } else { fill };
    if s.width <= len {
        return format!("{}{}", prefix, body);
    }
    let pad = s.width - len;
    let f = |n: usize| fill.to_string().repeat(n);
    match align {
        '<' => format!("{}{}{}", prefix, body, f(pad)),
        '>' => format!("{}{}{}", f(pad), prefix, body),
        '=' => format!("{}{}{}", prefix, f(pad), body),
        _ => format!("{}{}{}{}", f(pad / 2), prefix, body, f(pad - pad / 2)),
    }
}

fn sign_str(neg: bool, sign: char) -> &'static str {
    if neg {
        "-"
    } else {
        match sign {
            '+' => "+",
            ' ' => " ",
            _ => "",
        }
    }
}

/// format(value, spec)
pub fn format_value(v: &Value, spec: &str) -> VResult<String> {
    match v {
        Value::Str(s) => {
            if spec.is_empty() {
                return Ok(s.to_string());
            }
            let sp = parse_spec(spec)?;
            if let Some(t) = sp.ty.filter(|t| *t != 's') {
                return Err(Exc::value_error(format!("Unknown format code '{}' for object of type 'str'", t)));
            }
            if sp.sign != '-' {
                return Err(Exc::value_error("Sign not allowed in string format specifier"));
            }
            if sp.align == Some('=') {
                return Err(Exc::value_error("'=' alignment not allowed in string format specifier"));
            }
            let body: String = match sp.precision {
                Some(p) => s.chars().take(p).collect(),
                None => s.to_string(),
            };
            Ok(finish(&sp, "", &body, false))
        }
        Value::Bool(_) | Value::Int(_) => {
            if spec.is_empty() {
                return Ok(v.to_str());
            }
            let sp = parse_spec(spec)?;
            let i = v.as_int().unwrap();
            if let Some('e' | 'E' | 'f' | 'F' | 'g' | 'G' | '%') = sp.ty { return format_float(i as f64, &sp) }
            if sp.precision.is_some() {
                return Err(Exc::value_error("Precision not allowed in integer format specifier"));
            }
            let a = i.unsigned_abs();
            let (digits, prefix) = match sp.ty {
                None | Some('d') | Some('n') => (a.to_string(), ""),
                Some('b') => (format!("{:b}", a), "0b"),
                Some('o') => (format!("{:o}", a), "0o"),
                Some('x') => (format!("{:x}", a), "0x"),
                Some('X') => (format!("{:X}", a), "0X"),
                Some('c') => {
                    let ch = char::from_u32(i as u32).ok_or_else(|| Exc::new("OverflowError", "%c arg not in range(0x110000)"))?;
                    return Ok(finish(&sp, "", &ch.to_string(), false));
                }
                Some(t) => return Err(Exc::value_error(format!("Unknown format code '{}' for object of type '{}'", t, v.type_name()))),
            };
            let digits = match sp.grouping {
                Some(g) => group_digits(&digits, g, if matches!(sp.ty, Some('b' | 'o' | 'x' | 'X')) { 4 } else { 3 }),
                None => digits,
            };
            let mut pre = sign_str(i < 0, sp.sign).to_string();
            if sp.alt {
                pre.push_str(prefix);
            }
            Ok(finish(&sp, &pre, &digits, true))
        }
        Value::Float(f) => {
            if spec.is_empty() {
                return Ok(py_float_repr(*f));
            }
            let sp = parse_spec(spec)?;
            format_float(*f, &sp)
        }
        Value::Undefined(_) if spec.is_empty() => Ok(String::new()),
        other => {
            if spec.is_empty() {
                return Ok(other.to_str());
            }
            Err(Exc::type_error(format!("unsupported format string passed to {}.__format__", other.type_name())))
        }
    }
}

fn format_float(f: f64, sp: &Spec) -> VResult<String> {
    let neg = f.is_sign_negative() && !f.is_nan();
    let ty = sp.ty;
    let mut body = if f.is_nan() {
        "nan".to_string()
    } else if f.is_infinite() {
        "inf".to_string()
    } else {
        match ty {
            Some(t @ ('e' | 'E' | 'f' | 'F' | 'g' | 'G')) => float_body(f, t, sp.precision.map(|p| p as i64).unwrap_or(6), sp.alt),
            Some('n') => float_body(f, 'g', sp.precision.map(|p| p as i64).unwrap_or(6), sp.alt),
            Some('%') => {
                let mut b = float_body(f * 100.0, 'f', sp.precision.map(|p| p as i64).unwrap_or(6), sp.alt);
                b.push('%');
                b
            }
            None => match sp.precision {
                None => {
                    
                    py_float_repr(f.abs())
                }
                Some(p) => {
                    // like 'g' but keeps a ".0" in fixed notation
                    let g = float_body(f, 'g', if p == 0 { 1 } else { p as i64 }, sp.alt);
                    if !g.contains(['.', 'e', 'n', 'i']) {
                        format!("{}.0", g)
                    } else {
                        g
                    }
                }
            },
            Some(t) => return Err(Exc::value_error(format!("Unknown format code '{}' for object of type 'float'", t))),
        }
    };
    if matches!(ty, Some('E' | 'F' | 'G')) {
        body = body.to_uppercase();
    }
    if let Some(g) = sp.grouping {
        let (int_part, rest) = match body.find(|c: char| !c.is_ascii_digit()) {
            Some(p) => (body[..p].to_string(), body[p..].to_string()),
            None => (body.clone(), String::new()),
        };
        body = format!("{}{}", group_digits(&int_part, g, 3), rest);
    }
    let pre = sign_str(neg, sp.sign);
    Ok(finish(sp, pre, &body, f.is_finite()))
}
