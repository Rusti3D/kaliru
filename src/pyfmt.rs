// Python compatible string formatting and conversion helpers (from
// Moonruster/src/pyfmt.rs). klippy embeds str()/repr() output of python
// objects in messages and logs; these reproduce it.

use crate::pyfmt_tables::{DIGIT_ZEROS, NONPRINTABLE, WHITESPACE};
use serde_json::Value;
use std::borrow::Cow;

/// is `cp` in one of the sorted, disjoint (first, last) ranges
fn in_ranges(table: &[(u32, u32)], cp: u32) -> bool {
    table
        .binary_search_by(|&(a, b)| {
            if b < cp {
                std::cmp::Ordering::Less
            } else if a > cp {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        })
        .is_ok()
}

/// The value of a Unicode decimal digit (unicodedata.decimal)
fn py_decimal(c: char) -> Option<u32> {
    let cp = c as u32;
    let i = DIGIT_ZEROS.partition_point(|&z| z <= cp);
    let zero = *DIGIT_ZEROS.get(i.checked_sub(1)?)?;
    (cp - zero < 10).then_some(cp - zero)
}

/// `s` with its Unicode decimal digits as ASCII ones (int() and float()
/// accept any Nd digit)
fn ascii_decimals(s: &str) -> Cow<'_, str> {
    if s.is_ascii() {
        return Cow::Borrowed(s);
    }
    Cow::Owned(
        s.chars()
            .map(|c| match py_decimal(c) {
                Some(d) if !c.is_ascii() => char::from_digit(d, 10).unwrap_or(c),
                _ => c,
            })
            .collect(),
    )
}

/// Python's repr() for a float
pub fn py_float_repr(f: f64) -> String {
    let mut out = String::new();
    py_float_repr_into(&mut out, f);
    out
}

/// A fixed buffer for a formatted number (no heap allocation)
struct NumBuf {
    buf: [u8; 32],
    len: usize,
}

impl std::fmt::Write for NumBuf {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        let b = s.as_bytes();
        let end = self.len + b.len();
        if end > self.buf.len() {
            return Err(std::fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(b);
        self.len = end;
        Ok(())
    }
}

/// Python's repr() for a float, appended to `out`
pub fn py_float_repr_into(out: &mut String, f: f64) {
    use std::fmt::Write;
    if f.is_nan() {
        return out.push_str("nan");
    }
    if f.is_infinite() {
        return out.push_str(if f > 0.0 { "inf" } else { "-inf" });
    }
    if f == 0.0 {
        return out.push_str(if f.is_sign_negative() { "-0.0" } else { "0.0" });
    }
    // Rust's {:e} produces the shortest round trip digits: "-d.ddde-7"
    let mut sci = NumBuf { buf: [0; 32], len: 0 };
    write!(sci, "{:e}", f).expect("a float fits 32 bytes");
    let sci = &sci.buf[..sci.len];
    let e = sci.iter().position(|&c| c == b'e').expect("exponent");
    let (mantissa, exp) = (&sci[..e], &sci[e + 1..]);
    let exp: i32 = std::str::from_utf8(exp).expect("ascii").parse().expect("exponent");
    let neg = mantissa[0] == b'-';
    let mut digits = [0u8; 24];
    let mut nd = 0;
    for &c in mantissa {
        if c.is_ascii_digit() {
            digits[nd] = c;
            nd += 1;
        }
    }
    let digits = std::str::from_utf8(&digits[..nd]).expect("ascii");
    if neg {
        out.push('-');
    }
    let zeros = |out: &mut String, n: i32| {
        for _ in 0..n {
            out.push('0');
        }
    };
    if (-4..16).contains(&exp) {
        // fixed notation
        let ndigits = nd as i32;
        if exp >= 0 {
            let int_len = exp + 1;
            if ndigits <= int_len {
                out.push_str(digits);
                zeros(out, int_len - ndigits);
                out.push_str(".0");
            } else {
                out.push_str(&digits[..int_len as usize]);
                out.push('.');
                out.push_str(&digits[int_len as usize..]);
            }
        } else {
            out.push_str("0.");
            zeros(out, -exp - 1);
            out.push_str(digits);
        }
    } else {
        out.push_str(&digits[..1]);
        if nd > 1 {
            out.push('.');
            out.push_str(&digits[1..]);
        }
        out.push('e');
        out.push(if exp < 0 { '-' } else { '+' });
        let _ = write!(out, "{:02}", exp.abs());
    }
}

pub fn py_str_repr(s: &str) -> String {
    // Python prefers single quotes unless the string contains single quotes
    // and no double quotes
    let quote = if s.contains('\'') && !s.contains('"') { '"' } else { '\'' };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(quote);
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c == quote => {
                out.push('\\');
                out.push(c);
            }
            // what str.isprintable() rejects, escaped as repr() does
            c if in_ranges(NONPRINTABLE, c as u32) => {
                let cp = c as u32;
                if cp < 0x100 {
                    out.push_str(&format!("\\x{:02x}", cp));
                } else if cp < 0x10000 {
                    out.push_str(&format!("\\u{:04x}", cp));
                } else {
                    out.push_str(&format!("\\U{:08x}", cp));
                }
            }
            c => out.push(c),
        }
    }
    out.push(quote);
    out
}

/// Python's repr() for a json value
pub fn py_repr(v: &Value) -> String {
    match v {
        Value::Null => "None".into(),
        Value::Bool(b) => if *b { "True".into() } else { "False".into() },
        Value::Number(n) => {
            if n.is_f64() {
                py_float_repr(n.as_f64().unwrap())
            } else {
                n.to_string()
            }
        }
        Value::String(s) => py_str_repr(s),
        Value::Array(a) => format!("[{}]", a.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        Value::Object(o) => format!(
            "{{{}}}",
            o.iter().map(|(k, v)| format!("{}: {}", py_str_repr(k), py_repr(v))).collect::<Vec<_>>().join(", ")
        ),
    }
}

/// Python's str() for a json value
pub fn py_str(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        other => py_repr(other),
    }
}

pub fn py_bool(b: bool) -> &'static str {
    if b { "True" } else { "False" }
}

/// Python type name of a json value
pub fn py_type_name(v: &Value) -> &'static str {
    match v {
        Value::Null => "NoneType",
        Value::Bool(_) => "bool",
        Value::Number(n) => if n.is_f64() { "float" } else { "int" },
        Value::String(_) => "str",
        Value::Array(_) => "list",
        Value::Object(_) => "dict",
    }
}

/// Python's min(a, b) for floats (a unless b < a: NaN handling included)
pub fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python's max(a, b) for floats (a unless b > a)
pub fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python 3.12+ builtin sum() of floats (start 0): Neumaier compensated
/// summation (Python/bltinmodule.c builtin_sum_impl)
pub fn py_sum(items: impl IntoIterator<Item = f64>) -> f64 {
    let mut f_result = 0.0f64;
    let mut c = 0.0f64;
    for x in items {
        // Improved Kahan-Babuska algorithm by Arnold Neumaier
        let t = f_result + x;
        if f_result.abs() >= x.abs() {
            c += (f_result - t) + x;
        } else {
            c += (x - t) + f_result;
        }
        f_result = t;
    }
    // Avoid losing the sign on a negative result, and don't let adding the
    // compensation convert an infinite or overflowed sum to a NaN.
    if c != 0.0 && c.is_finite() {
        f_result += c;
    }
    f_result
}

/// Python's round(x, ndigits) (correctly rounded, ties to even)
pub fn py_round(x: f64, ndigits: usize) -> f64 {
    if !x.is_finite() {
        return x;
    }
    format!("{:.*}", ndigits, x).parse().unwrap_or(x)
}

/// Python's str.isspace() for one char (also the `\s` class of `re`)
pub fn py_isspace(c: char) -> bool {
    in_ranges(WHITESPACE, c as u32)
}

/// Python's str.strip() / lstrip() / rstrip() without arguments
pub fn py_strip(s: &str) -> &str {
    s.trim_matches(py_isspace)
}
pub fn py_lstrip(s: &str) -> &str {
    s.trim_start_matches(py_isspace)
}
pub fn py_rstrip(s: &str) -> &str {
    s.trim_end_matches(py_isspace)
}

// underscores in python number literals must sit between two digits
fn underscores_ok(s: &str) -> bool {
    let b = s.as_bytes();
    (0..b.len()).all(|i| b[i] != b'_' || (i > 0 && i + 1 < b.len() && b[i - 1].is_ascii_digit() && b[i + 1].is_ascii_digit()))
}

/// Python's int(str) conversion
pub fn py_int(s: &str) -> Option<i64> {
    let t = py_strip(s);
    if t.is_empty() {
        return None;
    }
    let (sign, body) = match t.as_bytes()[0] {
        b'+' => (1, &t[1..]),
        b'-' => (-1, &t[1..]),
        _ => (1, t),
    };
    let body = ascii_decimals(body);
    if body.is_empty() || body.starts_with('_') || body.ends_with('_') || body.contains("__") {
        return None;
    }
    let clean: String = body.chars().filter(|c| *c != '_').collect();
    if !clean.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    clean.parse::<i64>().ok().map(|v| v * sign)
}

/// Python's int(str, 0): a 0x / 0o / 0b prefix picks the base, and a
/// decimal may not have leading zeros
pub fn py_int_base0(s: &str) -> Option<i64> {
    let t = py_strip(s);
    let (sign, body) = match t.as_bytes().first()? {
        b'+' => (1, &t[1..]),
        b'-' => (-1, &t[1..]),
        _ => (1, t),
    };
    let lower = body.to_ascii_lowercase();
    let (radix, digits) = match lower.get(..2) {
        Some("0x") => (16, &lower[2..]),
        Some("0o") => (8, &lower[2..]),
        Some("0b") => (2, &lower[2..]),
        _ => (10, lower.as_str()),
    };
    // an underscore may follow the prefix or separate digits
    let digits = if radix != 10 { digits.strip_prefix('_').unwrap_or(digits) } else { digits };
    if digits.is_empty() || digits.starts_with('_') || digits.ends_with('_') || digits.contains("__") {
        return None;
    }
    let clean: String = digits.chars().filter(|c| *c != '_').collect();
    if radix == 10 && clean.len() > 1 && clean.starts_with('0') && clean.chars().any(|c| c != '0') {
        return None;
    }
    i64::from_str_radix(&clean, radix).ok().filter(|_| clean.chars().all(|c| c.is_digit(radix))).map(|v| v * sign)
}

/// Python's int(str, 16): an optional 0x prefix, then hex digits
pub fn py_int_base16(s: &str) -> Option<i64> {
    let t = py_strip(s);
    let (sign, body) = match t.as_bytes().first()? {
        b'+' => (1, &t[1..]),
        b'-' => (-1, &t[1..]),
        _ => (1, t),
    };
    let lower = body.to_ascii_lowercase();
    let digits = match lower.strip_prefix("0x") {
        // an underscore may follow the prefix
        Some(d) => d.strip_prefix('_').unwrap_or(d),
        None => lower.as_str(),
    };
    if digits.is_empty() || digits.starts_with('_') || digits.ends_with('_') || digits.contains("__") {
        return None;
    }
    let clean: String = digits.chars().filter(|c| *c != '_').collect();
    if !clean.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    i64::from_str_radix(&clean, 16).ok().map(|v| v * sign)
}

/// Python's float(str) conversion
pub fn py_float(s: &str) -> Option<f64> {
    let t = py_strip(s);
    if t.is_empty() {
        return None;
    }
    let lower = ascii_decimals(t).to_lowercase();
    let (sign, body) = match lower.as_bytes()[0] {
        b'+' => (1.0, &lower[1..]),
        b'-' => (-1.0, &lower[1..]),
        _ => (1.0, lower.as_str()),
    };
    match body {
        "inf" | "infinity" => return Some(sign * f64::INFINITY),
        "nan" => return Some(f64::NAN),
        _ => {}
    }
    if !underscores_ok(body) {
        return None;
    }
    let clean: String = body.chars().filter(|c| *c != '_').collect();
    if !clean.chars().all(|c| c.is_ascii_digit() || c == '.' || c == 'e' || c == '+' || c == '-') {
        return None;
    }
    // rust also takes "inf"/"nan" spellings, handled above; signs only
    // directly after 'e' are valid
    if clean.starts_with(['+', '-']) {
        return None;
    }
    clean.parse::<f64>().ok().map(|v| v * sign)
}

/// Python bool(value)
pub fn py_truthy(v: &Value) -> bool {
    match v {
        Value::Null => false,
        Value::Bool(b) => *b,
        Value::Number(n) => n.as_f64().map(|f| f != 0.0).unwrap_or(true),
        Value::String(s) => !s.is_empty(),
        Value::Array(a) => !a.is_empty(),
        Value::Object(o) => !o.is_empty(),
    }
}

/// list(shlex.shlex(s, posix=True)) with whitespace_split=True and the given
/// commenters (port of the python 3.13 state machine for that configuration:
/// no punctuation_chars, no source inclusion). Err is the ValueError text.
pub fn py_shlex_split(s: &str, commenters: &str) -> Result<Vec<String>, String> {
    const WHITESPACE: &str = " \t\r\n";
    const QUOTES: &str = "'\"";
    const ESCAPE: char = '\\';
    const ESCAPEDQUOTES: &str = "\"";
    let chars: Vec<char> = s.chars().collect();
    let mut pos = 0;
    let mut out = vec![];
    // state: ' ' (whitespace), 'a' (word), a quote char, the escape char, None (eof)
    let mut state: Option<char> = Some(' ');
    loop {
        // read_token()
        let mut quoted = false;
        let mut escapedstate = ' ';
        let mut token = String::new();
        loop {
            let nextchar = chars.get(pos).copied();
            if nextchar.is_some() {
                pos += 1;
            }
            let Some(st) = state else {
                token.clear(); // past end of file
                break;
            };
            if st == ' ' {
                match nextchar {
                    None => {
                        state = None; // end of file
                        break;
                    }
                    Some(c) if WHITESPACE.contains(c) => {
                        if !token.is_empty() || quoted {
                            break; // emit current token
                        }
                        continue;
                    }
                    Some(c) if commenters.contains(c) => {
                        // instream.readline()
                        while let Some(&c) = chars.get(pos) {
                            pos += 1;
                            if c == '\n' {
                                break;
                            }
                        }
                    }
                    Some(c) if c == ESCAPE => {
                        escapedstate = 'a';
                        state = Some(c);
                    }
                    Some(c) if QUOTES.contains(c) => state = Some(c),
                    Some(c) => {
                        // wordchars or whitespace_split
                        token.push(c);
                        state = Some('a');
                    }
                }
            } else if QUOTES.contains(st) {
                quoted = true;
                match nextchar {
                    None => return Err("No closing quotation".into()),
                    Some(c) if c == st => state = Some('a'),
                    Some(c) if c == ESCAPE && ESCAPEDQUOTES.contains(st) => {
                        escapedstate = st;
                        state = Some(c);
                    }
                    Some(c) => token.push(c),
                }
            } else if st == ESCAPE {
                let Some(c) = nextchar else {
                    return Err("No escaped character".into());
                };
                // In posix shells, only the quote itself or the escape
                // character may be escaped within quotes.
                if QUOTES.contains(escapedstate) && c != st && c != escapedstate {
                    token.push(st);
                }
                token.push(c);
                state = Some(escapedstate);
            } else {
                // state 'a'
                match nextchar {
                    None => {
                        state = None; // end of file
                        break;
                    }
                    Some(c) if WHITESPACE.contains(c) => {
                        state = Some(' ');
                        if !token.is_empty() || quoted {
                            break; // emit current token
                        }
                        continue;
                    }
                    Some(c) if commenters.contains(c) => {
                        while let Some(&c) = chars.get(pos) {
                            pos += 1;
                            if c == '\n' {
                                break;
                            }
                        }
                        state = Some(' ');
                        if !token.is_empty() || quoted {
                            break; // emit current token
                        }
                        continue;
                    }
                    Some(c) if QUOTES.contains(c) => state = Some(c),
                    Some(c) if c == ESCAPE => {
                        escapedstate = 'a';
                        state = Some(c);
                    }
                    Some(c) => token.push(c),
                }
            }
        }
        // posix: an unquoted empty token is EOF
        if !quoted && token.is_empty() {
            return Ok(out);
        }
        out.push(token);
    }
}

/// str() of the python OSError an io failure on `path` raises:
/// "[Errno 2] No such file or directory: 'path'"
/// os.strerror(errno) (thread-safe: the serial thread reports errors too)
pub fn strerror(errno: i32) -> String {
    let mut buf = [0 as libc::c_char; 256];
    // SAFETY: buf is a live buffer of buf.len() bytes (the XSI strerror_r:
    // glibc's __xpg_strerror_r, musl's own), NUL-terminated on success
    if unsafe { libc::strerror_r(errno, buf.as_mut_ptr(), buf.len()) } != 0 {
        return format!("Unknown error {}", errno);
    }
    // SAFETY: NUL-terminated by strerror_r
    unsafe { std::ffi::CStr::from_ptr(buf.as_ptr()) }.to_string_lossy().into_owned()
}

pub fn py_oserror(e: &std::io::Error, path: &str) -> String {
    let errno = e.raw_os_error().unwrap_or(0);
    let msg = strerror(errno);
    format!("[Errno {}] {}: {}", errno, msg, py_str_repr(path))
}

/// the OSError subclass python raises for an io failure's errno
pub fn py_oserror_class(e: &std::io::Error) -> &'static str {
    match e.raw_os_error().unwrap_or(0) {
        libc::ENOENT => "FileNotFoundError",
        libc::EACCES | libc::EPERM => "PermissionError",
        libc::EISDIR => "IsADirectoryError",
        libc::ENOTDIR => "NotADirectoryError",
        libc::EEXIST => "FileExistsError",
        libc::EINTR => "InterruptedError",
        libc::EAGAIN | libc::EALREADY | libc::EINPROGRESS => "BlockingIOError",
        libc::ECHILD => "ChildProcessError",
        libc::EPIPE | libc::ESHUTDOWN => "BrokenPipeError",
        libc::ECONNABORTED => "ConnectionAbortedError",
        libc::ECONNREFUSED => "ConnectionRefusedError",
        libc::ECONNRESET => "ConnectionResetError",
        libc::ESRCH => "ProcessLookupError",
        libc::ETIMEDOUT => "TimeoutError",
        _ => "OSError",
    }
}

/// str() of the UnicodeDecodeError of decoding `data` as utf-8
pub fn py_utf8_error(data: &[u8], e: &std::str::Utf8Error) -> String {
    let pos = e.valid_up_to();
    let reason = match e.error_len() {
        None => "unexpected end of data",
        Some(_) if matches!(data[pos], 0x80..=0xC1 | 0xF5..=0xFF) => "invalid start byte",
        Some(_) => "invalid continuation byte",
    };
    format!("'utf-8' codec can't decode byte 0x{:02x} in position {}: {}", data[pos], pos, reason)
}

/// the ValueError of unpacking a list of `got` items into `expected`
/// names, None when they match
pub fn py_unpack_error(expected: usize, got: usize) -> Option<String> {
    if got < expected {
        Some(format!("not enough values to unpack (expected {}, got {})", expected, got))
    } else if got > expected {
        Some(format!("too many values to unpack (expected {})", expected))
    } else {
        None
    }
}

/// the ValueError of float(s)
pub fn py_float_error(s: &str) -> String {
    format!("could not convert string to float: {}", py_str_repr(s))
}

/// json.dumps() of a python float
pub fn py_json_float(f: f64) -> String {
    let mut out = String::new();
    py_json_float_into(&mut out, f);
    out
}

/// json.dumps() of a float, appended to `out`
pub fn py_json_float_into(out: &mut String, f: f64) {
    if f.is_nan() {
        out.push_str("NaN")
    } else if f.is_infinite() {
        out.push_str(if f > 0. { "Infinity" } else { "-Infinity" })
    } else {
        py_float_repr_into(out, f)
    }
}

/// Python's repr() for bytes
pub fn py_bytes_repr(b: &[u8]) -> String {
    let quote = if b.contains(&b'\'') && !b.contains(&b'"') { b'"' } else { b'\'' };
    let mut out = String::with_capacity(b.len() + 3);
    out.push('b');
    out.push(quote as char);
    for &c in b {
        match c {
            b'\\' => out.push_str("\\\\"),
            b'\t' => out.push_str("\\t"),
            b'\n' => out.push_str("\\n"),
            b'\r' => out.push_str("\\r"),
            c if c == quote => {
                out.push('\\');
                out.push(c as char);
            }
            c if !(0x20..0x7f).contains(&c) => out.push_str(&format!("\\x{:02x}", c)),
            c => out.push(c as char),
        }
    }
    out.push(quote as char);
    out
}

/// The digits of Python's int(s, base) for base 0 or 16: (negative, base,
/// digit values most significant first), None where python raises ValueError
pub fn py_int_digits(s: &str, base: u32) -> Option<(bool, u32, Vec<u8>)> {
    let t = s.trim();
    let (neg, mut body) = match t.as_bytes().first() {
        Some(b'+') => (false, &t[1..]),
        Some(b'-') => (true, &t[1..]),
        _ => (false, t),
    };
    let lower = body.to_ascii_lowercase();
    let auto = base == 0;
    let mut base = base;
    let mut prefixed = false;
    for (p, b) in [("0x", 16), ("0o", 8), ("0b", 2)] {
        if lower.starts_with(p) && (base == 0 || base == b) {
            base = b;
            body = &body[2..];
            prefixed = true;
            break;
        }
    }
    if base == 0 {
        base = 10;
    }
    // an underscore may follow the prefix, otherwise only sit between digits
    if prefixed && body.starts_with('_') {
        body = &body[1..];
    }
    if body.is_empty() || body.starts_with('_') || body.ends_with('_') || body.contains("__") {
        return None;
    }
    let digits: Vec<u8> = body
        .chars()
        .filter(|c| *c != '_')
        .map(|c| c.to_digit(base).map(|d| d as u8))
        .collect::<Option<_>>()?;
    // base 0 rejects leading zeros on non-zero decimals ("010")
    if auto && !prefixed && digits.len() > 1 && digits[0] == 0
        && digits.iter().any(|d| *d != 0)
    {
        return None;
    }
    Some((neg, base, digits))
}

/// Python's int(s, base) for base 0 or 16
// ponytail: limited to i64, python ints are unbounded; None on overflow
pub fn py_int_base(s: &str, base: u32) -> Option<i64> {
    let (neg, base, digits) = py_int_digits(s, base)?;
    let mut v: i128 = 0;
    for d in digits {
        v = v.checked_mul(base as i128)?.checked_add(d as i128)?;
        if v > i64::MAX as i128 + 1 {
            return None;
        }
    }
    let v = if neg { -v } else { v };
    i64::try_from(v).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    // the allocating py_float_repr the in-place one replaced (rustify): the
    // oracle the property test compares with
    fn old_py_float_repr(f: f64) -> String {
        if f.is_nan() {
            return "nan".into();
        }
        if f.is_infinite() {
            return if f > 0.0 { "inf".into() } else { "-inf".into() };
        }
        if f == 0.0 {
            return if f.is_sign_negative() { "-0.0".into() } else { "0.0".into() };
        }
        let sci = format!("{:e}", f);
        let (mantissa, exp) = sci.split_once('e').unwrap();
        let exp: i32 = exp.parse().unwrap();
        let neg = mantissa.starts_with('-');
        let digits: String = mantissa.chars().filter(|c| c.is_ascii_digit()).collect();
        let sign = if neg { "-" } else { "" };
        if (-4..16).contains(&exp) {
            let ndigits = digits.len() as i32;
            let s = if exp >= 0 {
                let int_len = exp + 1;
                if ndigits <= int_len {
                    format!("{}{}.0", digits, "0".repeat((int_len - ndigits) as usize))
                } else {
                    format!("{}.{}", &digits[..int_len as usize], &digits[int_len as usize..])
                }
            } else {
                format!("0.{}{}", "0".repeat((-exp - 1) as usize), digits)
            };
            format!("{}{}", sign, s)
        } else {
            let m = if digits.len() > 1 { format!("{}.{}", &digits[..1], &digits[1..]) } else { digits.clone() };
            let esign = if exp < 0 { "-" } else { "+" };
            format!("{}{}e{}{:02}", sign, m, esign, exp.abs())
        }
    }

    /// a deterministic stream of f64 bit patterns (xorshift64*): every
    /// exponent range, subnormals, NaN payloads
    fn bit_patterns(n: usize) -> impl Iterator<Item = f64> {
        let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
        (0..n).map(move |_| {
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            f64::from_bits(x.wrapping_mul(0x2545_F491_4F6C_DD1D))
        })
    }

    #[test]
    fn float_repr_matches_the_allocating_one() {
        let special = [
            0.0, -0.0, 1.0, -1.0, 0.1, 0.5, 1e-4, 9.999e-5, 1e15, 9.99e15, 1e16, 1e17, 123456789012345680.0,
            f64::MIN_POSITIVE, f64::MAX, f64::MIN, f64::EPSILON, 5e-324, 1e300, 1e-300, 2.5, 100.0, 1e22, 0.001,
            f64::NAN, f64::INFINITY, f64::NEG_INFINITY,
        ];
        for f in special.into_iter().chain(bit_patterns(1_000_000)) {
            assert_eq!(py_float_repr(f), old_py_float_repr(f), "bits {:#x}", f.to_bits());
        }
        // values a printer reports: short decimals, temperatures, positions
        for i in -200_000..200_000i64 {
            let f = i as f64 / 1000.0;
            assert_eq!(py_float_repr(f), old_py_float_repr(f));
        }
    }

    /// CPython 3.13's own results (tools/gen_pyfmt_vectors.py, the reference
    /// interpreter): every mismatch is listed, not only the first
    #[test]
    fn vectors() {
        let v: Value = serde_json::from_str(include_str!("../tests/data/pyfmt_vectors.json")).unwrap();
        let mut bad: Vec<String> = Vec::new();
        for e in v["float_repr"].as_array().unwrap() {
            let f = f64::from_bits(e[0].as_u64().unwrap());
            let got = py_float_repr(f);
            if got != e[1].as_str().unwrap() {
                bad.push(format!("float_repr {:e}: {} != {}", f, got, e[1]));
            }
        }
        for e in v["str_repr"].as_array().unwrap() {
            let got = py_str_repr(e[0].as_str().unwrap());
            if got != e[1].as_str().unwrap() {
                bad.push(format!("str_repr {:?}: {} != {}", e[0], got, e[1]));
            }
        }
        for e in v["int"].as_array().unwrap() {
            let got = py_int(e[0].as_str().unwrap()).map(Value::from).unwrap_or(Value::Null);
            // python ints beyond i64 are out of range here
            if got != e[1] && !(e[1].is_number() && e[1].as_i64().is_none()) {
                bad.push(format!("int {:?}: {} != {}", e[0], got, e[1]));
            }
        }
        for e in v["float"].as_array().unwrap() {
            let got = match py_float(e[0].as_str().unwrap()) {
                None => Value::Null,
                Some(f) if !f.is_finite() => Value::from(py_float_repr(f)),
                Some(f) => Value::from(f),
            };
            let want = match &e[1] {
                Value::Number(n) => Value::from(n.as_f64().unwrap()),
                other => other.clone(),
            };
            if got != want {
                bad.push(format!("float {:?}: {} != {}", e[0], got, want));
            }
        }
        for e in v["round"].as_array().unwrap() {
            let f = f64::from_bits(e[0].as_u64().unwrap());
            let got = py_float_repr(py_round(f, e[1].as_u64().unwrap() as usize));
            if got != e[2].as_str().unwrap() {
                bad.push(format!("round({:e}, {}): {} != {}", f, e[1], got, e[2]));
            }
        }
        assert!(bad.is_empty(), "{} mismatches:\n{}", bad.len(), bad.join("\n"));
    }

    #[test]
    fn json_float_appends() {
        let mut out = String::from("x");
        py_json_float_into(&mut out, f64::NAN);
        py_json_float_into(&mut out, f64::NEG_INFINITY);
        py_json_float_into(&mut out, 2.5);
        assert_eq!(out, "xNaN-Infinity2.5");
    }

    #[test]
    fn bytes_repr_and_int_base() {
        assert_eq!(py_bytes_repr(b"a'\x00\x7f\xff\"\\\n"), "b'a\\'\\x00\\x7f\\xff\"\\\\\\n'");
        assert_eq!(py_bytes_repr(b"it's"), "b\"it's\"");
        assert_eq!(py_int_base("0x1F", 0), Some(31));
        assert_eq!(py_int_base(" -0b101 ", 0), Some(-5));
        assert_eq!(py_int_base("0o17", 0), Some(15));
        assert_eq!(py_int_base("0x_ff", 0), Some(255));
        assert_eq!(py_int_base("010", 0), None);
        assert_eq!(py_int_base("000", 0), Some(0));
        assert_eq!(py_int_base("1_000", 0), Some(1000));
        assert_eq!(py_int_base("ff", 0), None);
        assert_eq!(py_int_base("ff", 16), Some(255));
        assert_eq!(py_int_base("0xff", 16), Some(255));
        assert_eq!(py_int_base("", 0), None);
        // python float() / int()
        assert_eq!(py_float(" 1_0.5 "), Some(10.5));
        assert_eq!(py_float("1_.5"), None);
        assert_eq!(py_float("+.5"), Some(0.5));
        assert_eq!(py_float("5."), Some(5.0));
        assert_eq!(py_float("1e-3"), Some(0.001));
        assert_eq!(py_float("--1"), None);
        assert_eq!(py_float("1e"), None);
        assert_eq!(py_float("\x1c2\x1f"), Some(2.0));
        assert_eq!(py_int("_1"), None);
        assert_eq!(py_int("-0"), Some(0));
        assert_eq!(py_strip("\x1c a \t\n"), "a");
    }

    #[test]
    fn float_repr() {
        assert_eq!(py_float_repr(1.0), "1.0");
        assert_eq!(py_float_repr(0.1), "0.1");
        assert_eq!(py_float_repr(1e-5), "1e-05");
        assert_eq!(py_float_repr(1e16), "1e+16");
        assert_eq!(py_float_repr(123456.789), "123456.789");
        assert_eq!(py_float_repr(-0.0001), "-0.0001");
        assert_eq!(py_float_repr(1.5e300), "1.5e+300");
    }

    #[test]
    fn repr() {
        assert_eq!(py_repr(&json!({"a": [1, 2.5, null, true], "b": "it's"})), "{'a': [1, 2.5, None, True], 'b': \"it's\"}");
        assert_eq!(py_int(" 1_000 "), Some(1000));
        assert_eq!(py_int("1.5"), None);
        assert_eq!(py_float("1e3"), Some(1000.0));
    }
}
