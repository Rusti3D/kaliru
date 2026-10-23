// Jinja2 3.1.6 lexer (port of jinja2/lexer.py)
//
// The python lexer is built from regular expressions; this is a hand
// written equivalent for one fixed set of delimiters (Environment
// settings), with no line statements / line comments, trim_blocks and
// lstrip_blocks off (klippy's configuration and jinja2's defaults).
use super::TemplateSyntaxError;
use crate::pyfmt::py_str_repr;

#[derive(Clone, Debug, PartialEq)]
pub enum TokVal {
    None,
    Str(String),
    Int(i64),
    Float(f64),
}

#[derive(Clone, Debug)]
pub struct Token {
    pub lineno: usize,
    /// token type ("name", "string", "add", "block_begin", ...)
    pub ty: &'static str,
    pub value: TokVal,
}

impl Token {
    pub fn str_value(&self) -> &str {
        match &self.value {
            TokVal::Str(s) => s,
            _ => "",
        }
    }

    /// Token.test(expr): "type" or "type:value"
    pub fn test(&self, expr: &str) -> bool {
        if self.ty == expr {
            return true;
        }
        if let Some((ty, value)) = expr.split_once(':') {
            return ty == self.ty && matches!(&self.value, TokVal::Str(s) if s == value);
        }
        false
    }

    pub fn test_any(&self, exprs: &[&str]) -> bool {
        exprs.iter().any(|e| self.test(e))
    }
}

pub const OPERATORS: &[(&str, &str)] = &[
    ("+", "add"),
    ("-", "sub"),
    ("/", "div"),
    ("//", "floordiv"),
    ("*", "mul"),
    ("%", "mod"),
    ("**", "pow"),
    ("~", "tilde"),
    ("[", "lbracket"),
    ("]", "rbracket"),
    ("(", "lparen"),
    (")", "rparen"),
    ("{", "lbrace"),
    ("}", "rbrace"),
    ("==", "eq"),
    ("!=", "ne"),
    (">", "gt"),
    (">=", "gteq"),
    ("<", "lt"),
    ("<=", "lteq"),
    ("=", "assign"),
    (".", "dot"),
    (":", "colon"),
    ("|", "pipe"),
    (",", "comma"),
    (";", "semicolon"),
];

fn describe_token_type(ty: &str) -> String {
    if let Some((op, _)) = OPERATORS.iter().find(|(_, t)| *t == ty) {
        return op.to_string();
    }
    match ty {
        "comment_begin" => "begin of comment",
        "comment_end" => "end of comment",
        "comment" | "linecomment" => "comment",
        "block_begin" => "begin of statement block",
        "block_end" => "end of statement block",
        "variable_begin" => "begin of print statement",
        "variable_end" => "end of print statement",
        "linestatement_begin" => "begin of line statement",
        "linestatement_end" => "end of line statement",
        "data" => "template data / text",
        "eof" => "end of template",
        other => other,
    }
    .to_string()
}

/// describe_token(token)
pub fn describe_token(token: &Token) -> String {
    if token.ty == "name" {
        return token.str_value().to_string();
    }
    describe_token_type(token.ty)
}

/// describe_token_expr(expr)
pub fn describe_token_expr(expr: &str) -> String {
    if let Some((ty, value)) = expr.split_once(':') {
        if ty == "name" {
            return value.to_string();
        }
        return describe_token_type(ty);
    }
    describe_token_type(expr)
}

pub struct Delimiters {
    pub block_start: String,
    pub block_end: String,
    pub variable_start: String,
    pub variable_end: String,
    pub comment_start: String,
    pub comment_end: String,
}

/// python str.isspace() / \s for the lexer
fn is_space(c: char) -> bool {
    crate::pyfmt::py_isspace(c)
}

/// python \w (str.isalnum() or "_")
fn is_word(c: char) -> bool {
    c == '_' || c.is_alphanumeric()
}

/// str.isidentifier()
fn is_identifier(s: &str) -> bool {
    let mut chars = s.chars();
    match chars.next() {
        Some(c) if c == '_' || c.is_alphabetic() => {}
        _ => return false,
    }
    chars.all(|c| c == '_' || c.is_alphanumeric())
}

struct Lx<'a> {
    src: &'a [char],
    pos: usize,
    lineno: usize,
    tokens: Vec<Token>,
    name: Option<String>,
}

impl Lx<'_> {
    fn err(&self, msg: impl Into<String>, lineno: usize) -> TemplateSyntaxError {
        TemplateSyntaxError::new(msg, lineno, self.name.clone())
    }

    fn starts_with(&self, pos: usize, s: &str) -> bool {
        let mut i = pos;
        for c in s.chars() {
            if i >= self.src.len() || self.src[i] != c {
                return false;
            }
            i += 1;
        }
        true
    }

    fn push(&mut self, ty: &'static str, value: TokVal) {
        self.tokens.push(Token { lineno: self.lineno, ty, value });
    }

    fn count_nl(s: &[char]) -> usize {
        s.iter().filter(|&&c| c == '\n').count()
    }

    fn skip_ws(&self, mut pos: usize) -> usize {
        while pos < self.src.len() && is_space(self.src[pos]) {
            pos += 1;
        }
        pos
    }
}

/// lexer.tokeniter + wrap: the token list (ignored tokens dropped, values
/// converted) ending with an "eof" token
pub fn tokenize(source: &str, d: &Delimiters, name: Option<&str>) -> Result<Vec<Token>, TemplateSyntaxError> {
    // lines = newline_re.split(source)[::2]; drop one trailing empty line
    let mut lines: Vec<&str> = vec![];
    let mut rest = source;
    loop {
        match rest.find(['\r', '\n']) {
            None => {
                lines.push(rest);
                break;
            }
            Some(p) => {
                lines.push(&rest[..p]);
                let skip = if rest[p..].starts_with("\r\n") { 2 } else { 1 };
                rest = &rest[p + skip..];
            }
        }
    }
    if lines.last() == Some(&"") {
        lines.pop();
    }
    let source: Vec<char> = lines.join("\n").chars().collect();
    let mut lx = Lx { src: &source, pos: 0, lineno: 1, tokens: vec![], name: name.map(|s| s.to_string()) };
    root(&mut lx, d)?;
    let lineno = lx.lineno;
    lx.tokens.push(Token { lineno, ty: "eof", value: TokVal::None });
    Ok(lx.tokens)
}

#[derive(Clone, Copy, PartialEq)]
enum Begin {
    Raw,
    Comment,
    Block,
    Variable,
}

/// root state: data up to the next tag
fn root(lx: &mut Lx, d: &Delimiters) -> Result<(), TemplateSyntaxError> {
    // compile_rules(): longest first, ties by token name (reverse sorted)
    let mut rules: Vec<(usize, &'static str, &str, Begin)> = vec![
        (d.comment_start.chars().count(), "comment_begin", d.comment_start.as_str(), Begin::Comment),
        (d.block_start.chars().count(), "block_begin", d.block_start.as_str(), Begin::Block),
        (d.variable_start.chars().count(), "variable_begin", d.variable_start.as_str(), Begin::Variable),
    ];
    rules.sort_by(|a, b| (b.0, b.1).cmp(&(a.0, a.1)));
    while lx.pos < lx.src.len() {
        // find the first position where a tag begins
        let mut found = None;
        let mut p = lx.pos;
        'scan: while p < lx.src.len() {
            if let Some(end) = match_raw_begin(lx, d, p) {
                found = Some((p, Begin::Raw, end, ""));
                break 'scan;
            }
            for (_, tok, start, kind) in &rules {
                if lx.starts_with(p, start) {
                    let after = p + start.chars().count();
                    let sign = match lx.src.get(after) {
                        Some('-') => "-",
                        Some('+') => "+",
                        _ => "",
                    };
                    let _ = tok;
                    found = Some((p, *kind, after + sign.len(), sign));
                    break 'scan;
                }
            }
            p += 1;
        }
        let Some((tag_pos, kind, after, sign)) = found else {
            // data
            let data: String = lx.src[lx.pos..].iter().collect();
            lx.push("data", TokVal::Str(data));
            lx.pos = lx.src.len();
            break;
        };
        let mut text: Vec<char> = lx.src[lx.pos..tag_pos].to_vec();
        let mut newlines_stripped = 0;
        let strip_sign = if kind == Begin::Raw { raw_sign(lx, d, tag_pos) } else { sign };
        if strip_sign == "-" {
            // Strip all whitespace between the text and the tag.
            let mut end = text.len();
            while end > 0 && is_space(text[end - 1]) {
                end -= 1;
            }
            newlines_stripped = Lx::count_nl(&text[end..]);
            text.truncate(end);
        }
        if !text.is_empty() {
            let s: String = text.iter().collect();
            lx.push("data", TokVal::Str(s));
        }
        lx.lineno += Lx::count_nl(&text) + newlines_stripped;
        let tag_text = &lx.src[tag_pos..after];
        let tag_nl = Lx::count_nl(tag_text);
        lx.lineno += tag_nl;
        lx.pos = after;
        match kind {
            Begin::Raw => raw_body(lx, d)?,
            Begin::Comment => comment_body(lx, d)?,
            Begin::Block => {
                lx.push("block_begin", TokVal::None);
                tag_body(lx, d, true)?
            }
            Begin::Variable => {
                lx.push("variable_begin", TokVal::None);
                tag_body(lx, d, false)?
            }
        }
    }
    Ok(())
}

/// root_raw_re at pos: block_start (-|+|) \s* raw \s* (?:-block_end\s*|block_end)
fn match_raw_begin(lx: &Lx, d: &Delimiters, pos: usize) -> Option<usize> {
    if !lx.starts_with(pos, &d.block_start) {
        return None;
    }
    let mut p = pos + d.block_start.chars().count();
    if matches!(lx.src.get(p), Some('-') | Some('+')) {
        p += 1;
    }
    p = lx.skip_ws(p);
    if !lx.starts_with(p, "raw") {
        return None;
    }
    p += 3;
    p = lx.skip_ws(p);
    if lx.starts_with(p, "-") && lx.starts_with(p + 1, &d.block_end) {
        p += 1 + d.block_end.chars().count();
        return Some(lx.skip_ws(p));
    }
    if lx.starts_with(p, &d.block_end) {
        return Some(p + d.block_end.chars().count());
    }
    None
}

fn raw_sign(lx: &Lx, d: &Delimiters, pos: usize) -> &'static str {
    match lx.src.get(pos + d.block_start.chars().count()) {
        Some('-') => "-",
        Some('+') => "+",
        _ => "",
    }
}

/// raw block: (.*?)((?:block_start(-|+|))\s*endraw\s*(?:+block_end|-block_end\s*|block_end))
fn raw_body(lx: &mut Lx, d: &Delimiters) -> Result<(), TemplateSyntaxError> {
    let start = lx.pos;
    let mut p = start;
    while p < lx.src.len() {
        if lx.starts_with(p, &d.block_start) {
            let mut q = p + d.block_start.chars().count();
            let sign = match lx.src.get(q) {
                Some('-') => {
                    q += 1;
                    "-"
                }
                Some('+') => {
                    q += 1;
                    "+"
                }
                _ => "",
            };
            q = lx.skip_ws(q);
            if lx.starts_with(q, "endraw") {
                q += 6;
                q = lx.skip_ws(q);
                let end = if lx.starts_with(q, "+") && lx.starts_with(q + 1, &d.block_end) {
                    Some(q + 1 + d.block_end.chars().count())
                } else if lx.starts_with(q, "-") && lx.starts_with(q + 1, &d.block_end) {
                    Some(lx.skip_ws(q + 1 + d.block_end.chars().count()))
                } else if lx.starts_with(q, &d.block_end) {
                    Some(q + d.block_end.chars().count())
                } else {
                    None
                };
                if let Some(end) = end {
                    let mut text: Vec<char> = lx.src[start..p].to_vec();
                    let mut newlines_stripped = 0;
                    if sign == "-" {
                        let mut e = text.len();
                        while e > 0 && is_space(text[e - 1]) {
                            e -= 1;
                        }
                        newlines_stripped = Lx::count_nl(&text[e..]);
                        text.truncate(e);
                    }
                    if !text.is_empty() {
                        let s: String = text.iter().collect();
                        lx.push("data", TokVal::Str(s));
                    }
                    lx.lineno += Lx::count_nl(&text) + newlines_stripped;
                    lx.lineno += Lx::count_nl(&lx.src[p..end]);
                    lx.pos = end;
                    return Ok(());
                }
            }
        }
        p += 1;
    }
    Err(lx.err("Missing end of raw directive", lx.lineno))
}

/// comment: (.*?)((?:+comment_end|-comment_end\s*|comment_end))
fn comment_body(lx: &mut Lx, d: &Delimiters) -> Result<(), TemplateSyntaxError> {
    let start = lx.pos;
    let mut p = start;
    while p < lx.src.len() {
        let end = if lx.starts_with(p, "+") && lx.starts_with(p + 1, &d.comment_end) {
            Some(p + 1 + d.comment_end.chars().count())
        } else if lx.starts_with(p, "-") && lx.starts_with(p + 1, &d.comment_end) {
            Some(lx.skip_ws(p + 1 + d.comment_end.chars().count()))
        } else if lx.starts_with(p, &d.comment_end) {
            Some(p + d.comment_end.chars().count())
        } else {
            None
        };
        if let Some(end) = end {
            lx.lineno += Lx::count_nl(&lx.src[start..end]);
            lx.pos = end;
            return Ok(());
        }
        p += 1;
    }
    Err(lx.err("Missing end of comment tag", lx.lineno))
}

/// block / variable state: tag rules until the end delimiter
fn tag_body(lx: &mut Lx, d: &Delimiters, is_block: bool) -> Result<(), TemplateSyntaxError> {
    let mut balancing: Vec<char> = vec![];
    loop {
        let pos = lx.pos;
        // end of tag (only when braces are balanced)
        if balancing.is_empty() {
            let end_s = if is_block { &d.block_end } else { &d.variable_end };
            let end = if is_block && lx.starts_with(pos, "+") && lx.starts_with(pos + 1, end_s) {
                Some(pos + 1 + end_s.chars().count())
            } else if lx.starts_with(pos, "-") && lx.starts_with(pos + 1, end_s) {
                Some(lx.skip_ws(pos + 1 + end_s.chars().count()))
            } else if lx.starts_with(pos, end_s) {
                Some(pos + end_s.chars().count())
            } else {
                None
            };
            if let Some(end) = end {
                let nl = Lx::count_nl(&lx.src[pos..end]);
                lx.push(if is_block { "block_end" } else { "variable_end" }, TokVal::None);
                lx.lineno += nl;
                lx.pos = end;
                return Ok(());
            }
        }
        if pos >= lx.src.len() {
            // no rule matched at the end: tokeniter returns; the parser
            // reports the missing end token
            return Ok(());
        }
        let c = lx.src[pos];
        // whitespace
        if is_space(c) {
            let end = lx.skip_ws(pos);
            lx.lineno += Lx::count_nl(&lx.src[pos..end]);
            lx.pos = end;
            continue;
        }
        // float
        if let Some(end) = match_float(lx.src, pos) {
            let s: String = lx.src[pos..end].iter().filter(|&&c| c != '_').collect();
            let v: f64 = s.parse().unwrap_or(f64::NAN);
            lx.push("float", TokVal::Float(v));
            lx.pos = end;
            continue;
        }
        // integer
        if let Some((end, value)) = match_integer(lx.src, pos) {
            lx.push("integer", TokVal::Int(value));
            lx.pos = end;
            continue;
        }
        // name
        if is_word(c) || matches!(c, '·' | '⸱' | '゠' | '・' | 'ﾟ') {
            let mut end = pos;
            while end < lx.src.len() && (is_word(lx.src[end]) || matches!(lx.src[end], '·' | '⸱' | '゠' | '・' | 'ﾟ')) {
                end += 1;
            }
            let s: String = lx.src[pos..end].iter().collect();
            if !is_identifier(&s) {
                return Err(lx.err("Invalid character in identifier", lx.lineno));
            }
            lx.push("name", TokVal::Str(s));
            lx.pos = end;
            continue;
        }
        // string
        if c == '\'' || c == '"' {
            let mut end = pos + 1;
            let mut closed = false;
            while end < lx.src.len() {
                if lx.src[end] == '\\' {
                    end += 2;
                    continue;
                }
                if lx.src[end] == c {
                    closed = true;
                    break;
                }
                end += 1;
            }
            if closed {
                let raw: String = lx.src[pos + 1..end].iter().collect();
                let value = unescape(&raw).map_err(|m| lx.err(m, lx.lineno))?;
                lx.push("string", TokVal::Str(value));
                lx.lineno += Lx::count_nl(&lx.src[pos..=end]);
                lx.pos = end + 1;
                continue;
            }
        }
        // operator (longest first)
        let mut op = None;
        for len in [2, 1] {
            if pos + len <= lx.src.len() {
                let s: String = lx.src[pos..pos + len].iter().collect();
                if let Some((o, ty)) = OPERATORS.iter().find(|(o, _)| *o == s) {
                    op = Some((*o, *ty));
                    break;
                }
            }
        }
        if let Some((o, ty)) = op {
            match o {
                "{" => balancing.push('}'),
                "(" => balancing.push(')'),
                "[" => balancing.push(']'),
                "}" | ")" | "]" => {
                    let Some(expected) = balancing.pop() else {
                        return Err(lx.err(format!("unexpected '{}'", o), lx.lineno));
                    };
                    if expected.to_string() != o {
                        return Err(lx.err(format!("unexpected '{}', expected '{}'", o, expected), lx.lineno));
                    }
                }
                _ => {}
            }
            lx.push(ty, TokVal::None);
            lx.pos = pos + o.chars().count();
            continue;
        }
        return Err(lx.err(format!("unexpected char {} at {}", py_str_repr(&c.to_string()), pos), lx.lineno));
    }
}

fn digits_underscore(src: &[char], mut p: usize, is_digit: impl Fn(char) -> bool) -> Option<usize> {
    // (\d+_)*\d+
    let start = p;
    loop {
        let s = p;
        while p < src.len() && is_digit(src[p]) {
            p += 1;
        }
        if p == s {
            return if p == start { None } else { Some(p - 1) };
        }
        if p < src.len() && src[p] == '_' && p + 1 < src.len() && is_digit(src[p + 1]) {
            p += 1;
            continue;
        }
        return Some(p);
    }
}

/// float_re: (?<!\.)(\d+_)*\d+((\.(\d+_)*\d+)?e[+\-]?(\d+_)*\d+|\.(\d+_)*\d+)
fn match_float(src: &[char], pos: usize) -> Option<usize> {
    if pos > 0 && src[pos - 1] == '.' {
        return None;
    }
    let dig = |c: char| c.is_ascii_digit();
    let int_end = digits_underscore(src, pos, dig)?;
    let mut p = int_end;
    // optional fraction
    let mut frac_end = None;
    if p < src.len() && src[p] == '.' {
        frac_end = digits_underscore(src, p + 1, dig);
    }
    // exponent after optional fraction
    let exp_at = |q: usize| -> Option<usize> {
        if q < src.len() && (src[q] == 'e' || src[q] == 'E') {
            let mut r = q + 1;
            if r < src.len() && (src[r] == '+' || src[r] == '-') {
                r += 1;
            }
            return digits_underscore(src, r, dig);
        }
        None
    };
    if let Some(fe) = frac_end {
        if let Some(ee) = exp_at(fe) {
            return Some(ee);
        }
    }
    if let Some(ee) = exp_at(int_end) {
        return Some(ee);
    }
    if let Some(fe) = frac_end {
        p = fe;
        return Some(p);
    }
    None
}

/// integer_re (case insensitive) and int(value.replace("_", ""), 0)
fn match_integer(src: &[char], pos: usize) -> Option<(usize, i64)> {
    let c0 = src[pos];
    if !c0.is_ascii_digit() {
        return None;
    }
    let prefixed = |radix: u32, letter: char| -> Option<(usize, i64)> {
        if c0 != '0' || pos + 1 >= src.len() || src[pos + 1].to_ascii_lowercase() != letter {
            return None;
        }
        // (_?[digits])+
        let mut p = pos + 2;
        let mut digits = String::new();
        loop {
            let q = if p < src.len() && src[p] == '_' { p + 1 } else { p };
            if q < src.len() && src[q].is_digit(radix) {
                digits.push(src[q]);
                p = q + 1;
            } else {
                break;
            }
        }
        if digits.is_empty() {
            return None;
        }
        i64::from_str_radix(&digits, radix).ok().map(|v| (p, v))
    };
    if let Some(r) = prefixed(2, 'b').or_else(|| prefixed(8, 'o')).or_else(|| prefixed(16, 'x')) {
        return Some(r);
    }
    if c0 == '0' {
        // 0(_?0)*
        let mut p = pos + 1;
        loop {
            let q = if p < src.len() && src[p] == '_' { p + 1 } else { p };
            if q < src.len() && src[q] == '0' {
                p = q + 1;
            } else {
                break;
            }
        }
        return Some((p, 0));
    }
    // [1-9](_?\d)*
    let mut p = pos + 1;
    let mut digits = c0.to_string();
    loop {
        let q = if p < src.len() && src[p] == '_' { p + 1 } else { p };
        if q < src.len() && src[q].is_ascii_digit() {
            digits.push(src[q]);
            p = q + 1;
        } else {
            break;
        }
    }
    // python ints are unbounded; larger values become floats here
    match digits.parse::<i64>() {
        Ok(v) => Some((p, v)),
        Err(_) => Some((p, i64::MAX)),
    }
}

/// value.encode("ascii", "backslashreplace").decode("unicode-escape");
/// Err is the message part of the UnicodeDecodeError
fn unescape(s: &str) -> Result<String, String> {
    let chars: Vec<char> = s.chars().collect();
    let mut out = String::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c != '\\' {
            out.push(c);
            i += 1;
            continue;
        }
        i += 1;
        let Some(&e) = chars.get(i) else {
            return Err("\\ at end of string".into());
        };
        i += 1;
        match e {
            '\n' => {}
            '\\' => out.push('\\'),
            '\'' => out.push('\''),
            '"' => out.push('"'),
            'a' => out.push('\x07'),
            'b' => out.push('\x08'),
            'f' => out.push('\x0c'),
            'n' => out.push('\n'),
            'r' => out.push('\r'),
            't' => out.push('\t'),
            'v' => out.push('\x0b'),
            '0'..='7' => {
                let mut v = e.to_digit(8).unwrap();
                for _ in 0..2 {
                    match chars.get(i).and_then(|c| c.to_digit(8)) {
                        Some(d) => {
                            v = v * 8 + d;
                            i += 1;
                        }
                        None => break,
                    }
                }
                out.push(char::from_u32(v).unwrap_or('\u{fffd}'));
            }
            'x' | 'u' | 'U' => {
                let n = match e {
                    'x' => 2,
                    'u' => 4,
                    _ => 8,
                };
                let hex: String = chars[i..chars.len().min(i + n)].iter().collect();
                if hex.chars().count() < n || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
                    let what = match e {
                        'x' => "truncated \\xXX escape",
                        'u' => "truncated \\uXXXX escape",
                        _ => "truncated \\UXXXXXXXX escape",
                    };
                    return Err(what.into());
                }
                i += n;
                let v = u32::from_str_radix(&hex, 16).unwrap();
                match char::from_u32(v) {
                    Some(ch) => out.push(ch),
                    None => return Err("illegal Unicode character".into()),
                }
            }
            'N' => {
                // \N{name}: unicode names are not available here
                return Err("unknown Unicode character name".into());
            }
            other => {
                // unknown escape: kept as is
                out.push('\\');
                out.push(other);
            }
        }
    }
    Ok(out)
}
