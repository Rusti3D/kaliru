// Python's printf-style string formatting: str % args
// (port of CPython 3.13 Objects/unicodeobject.c PyUnicode_Format and the
// float / int helpers it calls)
//
// Errors are (python exception class, message).
use crate::pyfmt::py_str_repr;
use crate::pyval::PyVal;

pub type FmtError = (&'static str, String);

fn type_error(msg: String) -> FmtError {
    ("TypeError", msg)
}

fn value_error(msg: String) -> FmtError {
    ("ValueError", msg)
}

/// python type(v).__name__
pub fn type_name(v: &PyVal) -> &'static str {
    match v {
        PyVal::None => "NoneType",
        PyVal::Bool(_) => "bool",
        PyVal::Int(_) => "int",
        PyVal::Float(_) => "float",
        PyVal::Str(_) => "str",
        PyVal::List(_) => "list",
        PyVal::Tuple(_) => "tuple",
        PyVal::NamedTuple(n, _, _) => n,
        PyVal::Dict(_) => "dict",
    }
}

#[derive(Default, Clone, Copy)]
struct Flags {
    ljust: bool,
    sign: bool,
    blank: bool,
    alt: bool,
    zero: bool,
}

/// format % args
pub fn percent_format(format: &str, args: &PyVal) -> Result<String, FmtError> {
    let fmt: Vec<char> = format.chars().collect();
    let (arglist, dict): (Vec<PyVal>, Option<&PyVal>) = match args {
        PyVal::Tuple(t) | PyVal::NamedTuple(_, _, t) => (t.clone(), None),
        PyVal::Dict(_) => (vec![args.clone()], Some(args)),
        v => (vec![v.clone()], None),
    };
    // PyMapping_Check(): dicts and lists (not tuples or strings)
    let mapping_like = matches!(args, PyVal::Dict(_) | PyVal::List(_));
    let mut argidx = 0usize;
    let mut out = String::new();
    let mut i = 0;
    let next_arg = |argidx: &mut usize| -> Result<PyVal, FmtError> {
        if *argidx < arglist.len() {
            *argidx += 1;
            Ok(arglist[*argidx - 1].clone())
        } else {
            Err(type_error("not enough arguments for format string".into()))
        }
    };
    while i < fmt.len() {
        if fmt[i] != '%' {
            out.push(fmt[i]);
            i += 1;
            continue;
        }
        i += 1;
        // Get argument value from a dictionary
        let mut key_arg = None;
        if i < fmt.len() && fmt[i] == '(' {
            let Some(d) = dict else {
                return Err(type_error("format requires a mapping".into()));
            };
            i += 1;
            let keystart = i;
            let mut pcount = 1;
            while pcount > 0 && i < fmt.len() {
                if fmt[i] == ')' {
                    pcount -= 1;
                } else if fmt[i] == '(' {
                    pcount += 1;
                }
                i += 1;
            }
            if pcount > 0 {
                return Err(value_error("incomplete format key".into()));
            }
            let key: String = fmt[keystart..i - 1].iter().collect();
            let PyVal::Dict(m) = d else { unreachable!() };
            match m.get(&key) {
                Some(v) => key_arg = Some(v.clone()),
                None => return Err(("KeyError", py_str_repr(&key))),
            }
        }
        // Parse flags
        let mut flags = Flags::default();
        while i < fmt.len() {
            match fmt[i] {
                '-' => flags.ljust = true,
                '+' => flags.sign = true,
                ' ' => flags.blank = true,
                '#' => flags.alt = true,
                '0' => flags.zero = true,
                _ => break,
            }
            i += 1;
        }
        // Parse width. Example: "%10s" => width=10
        let mut width: i64 = -1;
        if i < fmt.len() && fmt[i] == '*' {
            let v = next_arg(&mut argidx)?;
            width = match v {
                PyVal::Int(n) => n,
                PyVal::Bool(b) => b as i64,
                _ => return Err(type_error("* wants int".into())),
            };
            if width < 0 {
                flags.ljust = true;
                width = -width;
            }
            i += 1;
        } else {
            while i < fmt.len() && fmt[i].is_ascii_digit() {
                width = width.max(0) * 10 + fmt[i].to_digit(10).unwrap() as i64;
                i += 1;
            }
        }
        // Parse precision. Example: "%.3f" => prec=3
        let mut prec: i64 = -1;
        if i < fmt.len() && fmt[i] == '.' {
            i += 1;
            prec = 0;
            if i < fmt.len() && fmt[i] == '*' {
                let v = next_arg(&mut argidx)?;
                prec = match v {
                    PyVal::Int(n) => n,
                    PyVal::Bool(b) => b as i64,
                    _ => return Err(type_error("* wants int".into())),
                };
                if prec < 0 {
                    prec = 0;
                }
                i += 1;
            } else {
                while i < fmt.len() && fmt[i].is_ascii_digit() {
                    prec = prec * 10 + fmt[i].to_digit(10).unwrap() as i64;
                    i += 1;
                }
            }
        }
        // Ignore "h", "l" and "L" format prefix (ex: "%hi" or "%ls")
        while i < fmt.len() && matches!(fmt[i], 'h' | 'l' | 'L') {
            i += 1;
        }
        if i >= fmt.len() {
            return Err(value_error("incomplete format".into()));
        }
        let c = fmt[i];
        i += 1;
        if c == '%' {
            out.push('%');
            continue;
        }
        let v = match key_arg.take() {
            Some(v) => v,
            None => next_arg(&mut argidx)?,
        };
        let piece = format_arg(c, &v, flags, prec, i - 1)?;
        pad_into(&mut out, &piece, c, flags, width);
    }
    if argidx < arglist.len() && !mapping_like {
        return Err(type_error("not all arguments converted during string formatting".into()));
    }
    Ok(out)
}

/// a converted argument: (sign/prefix part, digits/body part, is_number)
struct Piece {
    sign: String,
    body: String,
    numeric: bool,
}

fn format_arg(c: char, v: &PyVal, flags: Flags, prec: i64, index: usize) -> Result<Piece, FmtError> {
    let text = |s: String| {
        let s = if prec >= 0 { s.chars().take(prec as usize).collect() } else { s };
        Piece { sign: String::new(), body: s, numeric: false }
    };
    match c {
        's' => Ok(text(v.str())),
        'r' => Ok(text(v.repr())),
        'a' => Ok(text(ascii_repr(&v.repr()))),
        'i' | 'd' | 'u' | 'o' | 'x' | 'X' => {
            let n: i128 = match v {
                PyVal::Int(n) => *n as i128,
                PyVal::Bool(b) => *b as i128,
                PyVal::Float(f) if matches!(c, 'i' | 'd' | 'u') => {
                    if f.is_nan() {
                        return Err(value_error("cannot convert float NaN to integer".into()));
                    }
                    if f.is_infinite() {
                        return Err(("OverflowError", "cannot convert float infinity to integer".into()));
                    }
                    f.trunc() as i128
                }
                _ => {
                    let what = if matches!(c, 'i' | 'd' | 'u') { "a real number" } else { "an integer" };
                    return Err(type_error(format!("%{} format: {} is required, not {}", c, what, type_name(v))));
                }
            };
            Ok(format_long(n, c, flags, prec))
        }
        'e' | 'E' | 'f' | 'F' | 'g' | 'G' => {
            let f = match v {
                PyVal::Float(f) => *f,
                PyVal::Int(n) => *n as f64,
                PyVal::Bool(b) => *b as i64 as f64,
                _ => return Err(type_error(format!("must be real number, not {}", type_name(v)))),
            };
            Ok(format_float(f, c, flags, prec))
        }
        'c' => {
            let ch = match v {
                PyVal::Str(s) if s.chars().count() == 1 => s.chars().next().unwrap(),
                PyVal::Str(s) => {
                    return Err(type_error(format!(
                        "%c requires an int or a unicode character, not a string of length {}",
                        s.chars().count()
                    )))
                }
                PyVal::Int(n) => char::from_u32(*n as u32)
                    .filter(|_| (0..=0x10ffff).contains(n))
                    .ok_or_else(|| ("OverflowError", "%c arg not in range(0x110000)".to_string()))?,
                PyVal::Bool(b) => char::from_u32(*b as u32).unwrap(),
                _ => return Err(type_error(format!("%c requires an int or a unicode character, not {}", type_name(v)))),
            };
            Ok(Piece { sign: String::new(), body: ch.to_string(), numeric: false })
        }
        _ => Err(value_error(format!("unsupported format character '{}' (0x{:x}) at index {}", c, c as u32, index))),
    }
}

/// ascii(): repr() with non-ascii characters escaped
fn ascii_repr(s: &str) -> String {
    let mut out = String::new();
    for ch in s.chars() {
        let n = ch as u32;
        if n < 0x80 {
            out.push(ch);
        } else if n < 0x100 {
            out.push_str(&format!("\\x{:02x}", n));
        } else if n < 0x10000 {
            out.push_str(&format!("\\u{:04x}", n));
        } else {
            out.push_str(&format!("\\U{:08x}", n));
        }
    }
    out
}

// formatlong(): %d %i %u %o %x %X
fn format_long(n: i128, c: char, flags: Flags, prec: i64) -> Piece {
    let neg = n < 0;
    let a = n.unsigned_abs();
    let mut digits = match c {
        'o' => format!("{:o}", a),
        'x' => format!("{:x}", a),
        'X' => format!("{:X}", a),
        _ => a.to_string(),
    };
    if prec >= 0 && (digits.len() as i64) < prec {
        digits = format!("{}{}", "0".repeat(prec as usize - digits.len()), digits);
    }
    let mut sign = String::new();
    if neg {
        sign.push('-');
    } else if flags.sign {
        sign.push('+');
    } else if flags.blank {
        sign.push(' ');
    }
    if flags.alt {
        match c {
            'o' => sign.push_str("0o"),
            'x' => sign.push_str("0x"),
            'X' => sign.push_str("0X"),
            _ => {}
        }
    }
    Piece { sign, body: digits, numeric: true }
}

/// C printf style "%.*e" of a finite value: d.ddde+XX
fn fmt_exp(f: f64, prec: usize, upper: bool) -> String {
    let s = format!("{:.*e}", prec, f.abs());
    let (mant, exp) = s.split_once('e').unwrap();
    let exp: i32 = exp.parse().unwrap();
    let e = if upper { 'E' } else { 'e' };
    let sign = if exp < 0 { '-' } else { '+' };
    format!("{}{}{}{:02}", mant, e, sign, exp.abs())
}

/// the body of a finite float for %e %f %g (no sign); PyOS_double_to_string
pub fn float_body(f: f64, c: char, prec: i64, alt: bool) -> String {
    let upper = c.is_ascii_uppercase();
    let prec = if prec < 0 { 6 } else { prec as usize };
    match c.to_ascii_lowercase() {
        'f' => {
            let mut s = format!("{:.*}", prec, f.abs());
            if alt && prec == 0 {
                s.push('.');
            }
            s
        }
        'e' => {
            let mut s = fmt_exp(f, prec, upper);
            if alt && prec == 0 {
                let pos = s.find(['e', 'E']).unwrap();
                s.insert(pos, '.');
            }
            s
        }
        _ => {
            // %g
            let p = if prec == 0 { 1 } else { prec };
            let exp = if f == 0.0 {
                0
            } else {
                let es = fmt_exp(f, p - 1, false);
                es.split_once('e').unwrap().1.parse::<i32>().unwrap()
            };
            let mut s = if exp < -4 || exp >= p as i32 {
                fmt_exp(f, p - 1, upper)
            } else {
                format!("{:.*}", (p as i32 - 1 - exp) as usize, f.abs())
            };
            if !alt {
                // remove trailing zeros (and a trailing point)
                let (mant, rest) = match s.find(['e', 'E']) {
                    Some(pos) => (s[..pos].to_string(), s[pos..].to_string()),
                    None => (s.clone(), String::new()),
                };
                let mant = if mant.contains('.') { mant.trim_end_matches('0').trim_end_matches('.').to_string() } else { mant };
                s = mant + &rest;
            } else if !s.contains('.') {
                match s.find(['e', 'E']) {
                    Some(pos) => s.insert(pos, '.'),
                    None => s.push('.'),
                }
            }
            s
        }
    }
}

// formatfloat(): %e %E %f %F %g %G
fn format_float(f: f64, c: char, flags: Flags, prec: i64) -> Piece {
    let neg = f.is_sign_negative() && !f.is_nan();
    let body = if f.is_nan() {
        if c.is_ascii_uppercase() { "NAN" } else { "nan" }.to_string()
    } else if f.is_infinite() {
        if c.is_ascii_uppercase() { "INF" } else { "inf" }.to_string()
    } else {
        float_body(f, c, prec, flags.alt)
    };
    let mut sign = String::new();
    if neg {
        sign.push('-');
    } else if flags.sign {
        sign.push('+');
    } else if flags.blank {
        sign.push(' ');
    }
    Piece { sign, body, numeric: f.is_finite() }
}

// unicode_format_arg_output(): apply the width
fn pad_into(out: &mut String, p: &Piece, c: char, flags: Flags, width: i64) {
    let len = (p.sign.chars().count() + p.body.chars().count()) as i64;
    let fill = (width - len).max(0) as usize;
    if flags.ljust {
        out.push_str(&p.sign);
        out.push_str(&p.body);
        out.push_str(&" ".repeat(fill));
    } else if flags.zero && (p.numeric || matches!(c, 'd' | 'i' | 'u' | 'o' | 'x' | 'X')) {
        out.push_str(&p.sign);
        out.push_str(&"0".repeat(fill));
        out.push_str(&p.body);
    } else {
        out.push_str(&" ".repeat(fill));
        out.push_str(&p.sign);
        out.push_str(&p.body);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(fmt: &str, args: PyVal) -> String {
        percent_format(fmt, &args).unwrap()
    }

    #[test]
    fn basics() {
        assert_eq!(f("%.3f", PyVal::Float(1.23456)), "1.235");
        assert_eq!(f("%d %s", PyVal::Tuple(vec![PyVal::Float(3.7), PyVal::Str("x".into())])), "3 x");
        assert_eq!(f("%5.1f|%-5d|%05d", PyVal::Tuple(vec![PyVal::Float(2.25), PyVal::Int(3), PyVal::Int(-42)])), "  2.2|3    |-0042");
        assert_eq!(f("%g %g %g", PyVal::Tuple(vec![PyVal::Float(0.0001), PyVal::Float(1e-5), PyVal::Float(123456789.0)])), "0.0001 1e-05 1.23457e+08");
        assert_eq!(f("%x %#o %e", PyVal::Tuple(vec![PyVal::Int(255), PyVal::Int(8), PyVal::Float(12345.678)])), "ff 0o10 1.234568e+04");
        assert_eq!(f("%r", PyVal::Str("a'b".into())), "\"a'b\"");
        assert_eq!(f("%%", PyVal::Tuple(vec![])), "%");
        assert_eq!(percent_format("%d", &PyVal::Str("x".into())).unwrap_err().1, "%d format: a real number is required, not str");
        assert_eq!(percent_format("%d %d", &PyVal::Int(1)).unwrap_err().1, "not enough arguments for format string");
        assert_eq!(percent_format("x", &PyVal::Int(1)).unwrap_err().1, "not all arguments converted during string formatting");
    }
}
