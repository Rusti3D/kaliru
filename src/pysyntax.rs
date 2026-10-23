// CPython 3.13 wording for the SyntaxErrors of python macros. RustPython's
// parser finds the errors; this turns its messages into the ones CPython
// raises (Parser/lexer/lexer.c, Parser/pegen_errors.c, Grammar/python.gram
// of v3.13.1). Checked by tools/gen_pysyntax_cases.py; NOTES.md "Python
// macros" lists what still differs.
//
// Positions: lineno 1-based, offset = 1-based character column.

type Err = (String, String, i64, i64);

fn err(cls: &str, msg: impl Into<String>, line: usize, col: usize) -> Err {
    (cls.to_string(), msg.into(), line as i64, col as i64)
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Kind {
    Name,
    Number,
    Str,
    Op,
    Newline,
}

#[derive(Clone, Debug)]
struct Tok {
    kind: Kind,
    text: String,
    line: usize,
    /// 0-based character column
    col: usize,
    /// bracket nesting before the token
    level: usize,
}

/// a tokenizer error: `hard` ones set an exception in CPython's tokenizer
/// (they win over parser errors anywhere in the source), the others stop
/// the parser where they occur
struct TokErr {
    err: Err,
    hard: bool,
}

struct Lexed {
    toks: Vec<Tok>,
    error: Option<TokErr>,
    /// brackets still open at the end: (char, line, 0-based col)
    open: Vec<(char, usize, usize)>,
}

const KEYWORDS: &[&str] = &[
    "False", "None", "True", "and", "as", "assert", "async", "await", "break", "class", "continue", "def", "del", "elif",
    "else", "except", "finally", "for", "from", "global", "if", "import", "in", "is", "lambda", "nonlocal", "not", "or",
    "pass", "raise", "return", "try", "while", "with", "yield",
];

fn is_id_start(c: char) -> bool {
    c == '_' || c.is_alphabetic()
}

fn is_id_char(c: char) -> bool {
    c == '_' || c.is_alphanumeric()
}

/// the tokenizer: tokens up to the first error
fn lex(src: &str) -> Lexed {
    let lines: Vec<Vec<char>> = src.split_inclusive('\n').map(|l| l.chars().collect()).collect();
    let mut toks = vec![];
    let mut open: Vec<(char, usize, usize)> = vec![];
    let mut indents: Vec<(usize, usize)> = vec![(0, 0)];
    let mut li = 0;
    let mut ci = 0;
    let mut atbol = true;
    let push = |toks: &mut Vec<Tok>, kind, text: String, line, col, level| toks.push(Tok { kind, text, line, col, level });
    let line_len = |l: &Vec<char>| l.iter().take_while(|&&c| c != '\n' && c != '\r').count();
    macro_rules! fail {
        ($e:expr, $hard:expr) => {
            return Lexed { toks, error: Some(TokErr { err: $e, hard: $hard }), open }
        };
    }
    while li < lines.len() {
        let line = &lines[li];
        if atbol {
            atbol = false;
            let (mut col, mut altcol) = (0, 0);
            while ci < line.len() {
                match line[ci] {
                    ' ' => {
                        col += 1;
                        altcol += 1;
                    }
                    '\t' => {
                        col = (col / 8 + 1) * 8;
                        altcol += 1;
                    }
                    '\x0c' => {
                        col = 0;
                        altcol = 0;
                    }
                    _ => break,
                }
                ci += 1;
            }
            let blank = ci >= line.len() || matches!(line[ci], '#' | '\n' | '\r');
            if !blank && open.is_empty() {
                let top = *indents.last().unwrap();
                let tab = err("TabError", "inconsistent use of tabs and spaces in indentation", li + 1, 1);
                if col == top.0 {
                    if altcol != top.1 {
                        fail!(tab, false);
                    }
                } else if col > top.0 {
                    if altcol <= top.1 {
                        fail!(tab, false);
                    }
                    indents.push((col, altcol));
                } else {
                    while indents.len() > 1 && col < indents.last().unwrap().0 {
                        indents.pop();
                    }
                    let top = *indents.last().unwrap();
                    if col != top.0 {
                        let e = err("IndentationError", "unindent does not match any outer indentation level", li + 1, line_len(line) + 1);
                        fail!(e, false);
                    }
                    if altcol != top.1 {
                        fail!(tab, false);
                    }
                }
            }
        }
        if ci >= line.len() {
            li += 1;
            ci = 0;
            atbol = true;
            continue;
        }
        let c = line[ci];
        let level = open.len();
        match c {
            ' ' | '\t' | '\x0c' | '\r' => ci += 1,
            '#' => ci = line.len(),
            '\n' => {
                let blank = !toks.last().is_some_and(|t: &Tok| t.kind != Kind::Newline);
                if open.is_empty() && !blank {
                    push(&mut toks, Kind::Newline, "\n".into(), li + 1, ci, 0);
                }
                li += 1;
                ci = 0;
                atbol = open.is_empty();
            }
            '\\' => {
                if line.get(ci + 1) == Some(&'\n') || (line.get(ci + 1) == Some(&'\r') && line.get(ci + 2) == Some(&'\n')) {
                    li += 1;
                    ci = 0;
                } else {
                    let e = err("SyntaxError", "unexpected character after line continuation character", li + 1, ci + 2);
                    fail!(e, false);
                }
            }
            c if c.is_ascii_digit() || (c == '.' && line.get(ci + 1).is_some_and(|d| d.is_ascii_digit())) => {
                let start = ci;
                match lex_number(line, &mut ci) {
                    Ok(()) => push(&mut toks, Kind::Number, line[start..ci].iter().collect(), li + 1, start, level),
                    Err(e) => fail!(err("SyntaxError", e.0, li + 1, e.1), true),
                }
            }
            c if is_id_start(c) => {
                let start = ci;
                while ci < line.len() && is_id_char(line[ci]) {
                    ci += 1;
                }
                let word: String = line[start..ci].iter().collect();
                let quote = line.get(ci).copied().filter(|q| *q == '\'' || *q == '"');
                let prefix = word.len() <= 2 && word.chars().all(|p| "rRbBuUfF".contains(p));
                if let (Some(q), true) = (quote, prefix) {
                    let fstring = word.to_lowercase().contains('f');
                    let first = li;
                    match lex_string(&lines, &mut li, &mut ci, q, fstring) {
                        Ok(()) => push(&mut toks, Kind::Str, word, first + 1, start, level),
                        Err(e) => fail!(fix_str_err(e, start), true),
                    }
                } else {
                    push(&mut toks, Kind::Name, word, li + 1, start, level);
                }
            }
            '\'' | '"' => {
                let start = ci;
                let first = li;
                match lex_string(&lines, &mut li, &mut ci, c, false) {
                    Ok(()) => push(&mut toks, Kind::Str, String::new(), first + 1, start, level),
                    Err(e) => fail!(fix_str_err(e, start), true),
                }
            }
            '(' | '[' | '{' => {
                open.push((c, li + 1, ci));
                push(&mut toks, Kind::Op, c.to_string(), li + 1, ci, level);
                ci += 1;
            }
            ')' | ']' | '}' => {
                let Some((o, oline, _)) = open.pop() else {
                    fail!(err("SyntaxError", format!("unmatched '{}'", c), li + 1, ci + 1), true);
                };
                if !matches!((o, c), ('(', ')') | ('[', ']') | ('{', '}')) {
                    let msg = if oline != li + 1 {
                        format!("closing parenthesis '{}' does not match opening parenthesis '{}' on line {}", c, o, oline)
                    } else {
                        format!("closing parenthesis '{}' does not match opening parenthesis '{}'", c, o)
                    };
                    fail!(err("SyntaxError", msg, li + 1, ci + 1), true);
                }
                push(&mut toks, Kind::Op, c.to_string(), li + 1, ci, level - 1);
                ci += 1;
            }
            c if !c.is_ascii() && !is_id_start(c) => {
                let msg = if c.is_control() {
                    format!("invalid non-printable character U+{:04X}", c as u32)
                } else {
                    format!("invalid character '{}' (U+{:04X})", c, c as u32)
                };
                fail!(err("SyntaxError", msg, li + 1, ci + 1), true);
            }
            c if c.is_control() => {
                fail!(err("SyntaxError", format!("invalid non-printable character U+{:04X}", c as u32), li + 1, ci + 1), true);
            }
            _ => {
                let rest: String = line[ci..].iter().take(3).collect();
                let op = ["**=", "//=", ">>=", "<<=", "...", "->", ":=", "**", "//", ">>", "<<", "<=", ">=", "==", "!=", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "@="]
                    .iter()
                    .find(|o| rest.starts_with(**o))
                    .map(|o| o.to_string())
                    .unwrap_or_else(|| c.to_string());
                push(&mut toks, Kind::Op, op.clone(), li + 1, ci, level);
                ci += op.chars().count();
            }
        }
    }
    // the implicit NEWLINE at the end of a last line without one
    if open.is_empty() && toks.last().is_some_and(|t| t.kind != Kind::Newline) {
        let l = lines.len();
        push(&mut toks, Kind::Newline, String::new(), l, line_len(&lines[l - 1]), 0);
    }
    Lexed { toks, error: None, open }
}

/// unterminated string: (message, line of the opening quote)
struct StrErr(String, usize);

fn fix_str_err(e: StrErr, start: usize) -> Err {
    err("SyntaxError", e.0, e.1, start + 1)
}

/// a string body from the quote at lines[li][ci]; ci/li end after it
fn lex_string(lines: &[Vec<char>], li: &mut usize, ci: &mut usize, q: char, fstring: bool) -> Result<(), StrErr> {
    let first = *li;
    let line = &lines[*li];
    let triple = line.get(*ci + 1) == Some(&q) && line.get(*ci + 2) == Some(&q);
    *ci += if triple { 3 } else { 1 };
    let mut escaped_quote = false;
    loop {
        let line = &lines[*li];
        if *ci >= line.len() || (!triple && line[*ci] == '\n') {
            if triple && *li + 1 < lines.len() {
                *li += 1;
                *ci = 0;
                continue;
            }
            // detected at: the line the tokenizer is on
            let detected = if triple { lines.len() } else { *li + 1 };
            let kind = if fstring { "f-string" } else { "string" };
            let msg = if triple {
                format!("unterminated triple-quoted {} literal (detected at line {})", kind, detected)
            } else if escaped_quote && !fstring {
                format!("unterminated string literal (detected at line {}); perhaps you escaped the end quote?", detected)
            } else {
                format!("unterminated {} literal (detected at line {})", kind, detected)
            };
            return Err(StrErr(msg, first + 1));
        }
        let c = line[*ci];
        if c == '\\' {
            *ci += 1;
            if line.get(*ci) == Some(&q) {
                escaped_quote = true;
            }
            if line.get(*ci) == Some(&'\n') && *li + 1 < lines.len() {
                *li += 1;
                *ci = 0;
                continue;
            }
            *ci += 1;
            continue;
        }
        if c == q {
            if !triple {
                *ci += 1;
                return Ok(());
            }
            if line.get(*ci + 1) == Some(&q) && line.get(*ci + 2) == Some(&q) {
                *ci += 3;
                return Ok(());
            }
        }
        *ci += 1;
    }
}

/// a number literal from line[ci]: Err((message, offset))
fn lex_number(line: &[char], ci: &mut usize) -> Result<(), (String, usize)> {
    let start = *ci;
    let at = |i: usize| line.get(i).copied().unwrap_or('\n');
    // the end of a literal: followed by a name character is an error,
    // except the keywords that can follow a number in valid code
    let end = |i: usize, kind: &str| -> Result<(), (String, usize)> {
        let c = at(i);
        let rest: String = line[i.min(line.len())..].iter().take(4).collect();
        let kw = ["and", "else", "for", "if", "in", "is", "or", "not"].iter().any(|k| rest.starts_with(k));
        if c.is_ascii() && is_id_char(c) && !kw {
            return Err((format!("invalid {} literal", kind), i));
        }
        Ok(())
    };
    let digits = |i: &mut usize, ok: &dyn Fn(char) -> bool| -> Result<(), (String, usize)> {
        loop {
            while ok(at(*i)) {
                *i += 1;
            }
            if at(*i) != '_' {
                return Ok(());
            }
            *i += 1;
            if !ok(at(*i)) {
                return Err(("invalid decimal literal".into(), *i));
            }
        }
    };
    if at(*ci) == '0' && matches!(at(*ci + 1), 'x' | 'X' | 'o' | 'O' | 'b' | 'B') {
        let (kind, ok): (&str, fn(char) -> bool) = match at(*ci + 1) {
            'x' | 'X' => ("hexadecimal", |c: char| c.is_ascii_hexdigit()),
            'o' | 'O' => ("octal", |c: char| ('0'..='7').contains(&c)),
            _ => ("binary", |c: char| c == '0' || c == '1'),
        };
        *ci += 2;
        if at(*ci) == '_' {
            *ci += 1;
        }
        if !ok(at(*ci)) {
            return Err((format!("invalid {} literal", kind), *ci + 1));
        }
        digits(ci, &ok).map_err(|(_, i)| (format!("invalid {} literal", kind), i))?;
        return end(*ci, kind);
    }
    let dec = |c: char| c.is_ascii_digit();
    if at(*ci) != '.' {
        digits(ci, &dec)?;
        let leading_zero = line[start] == '0' && line[start..*ci].iter().any(|c| *c != '0' && *c != '_');
        if leading_zero && !matches!(at(*ci), '.' | 'e' | 'E' | 'j' | 'J') {
            return Err(("leading zeros in decimal integer literals are not permitted; use an 0o prefix for octal integers".into(), start + 1));
        }
    }
    if at(*ci) == '.' {
        *ci += 1;
        digits(ci, &dec)?;
    }
    if matches!(at(*ci), 'e' | 'E') {
        let save = *ci;
        *ci += 1;
        if matches!(at(*ci), '+' | '-') {
            *ci += 1;
        }
        if !dec(at(*ci)) {
            *ci = save;
            return end(*ci, "decimal");
        }
        digits(ci, &dec)?;
    }
    if matches!(at(*ci), 'j' | 'J') {
        *ci += 1;
        return end(*ci, "imaginary");
    }
    end(*ci, "decimal")
}

fn is_kw(t: &Tok, kw: &str) -> bool {
    t.kind == Kind::Name && t.text == kw
}

/// the token ends an expression (a name, literal or closing bracket)
fn ends_expr(t: &Tok) -> bool {
    match t.kind {
        Kind::Name => !KEYWORDS.contains(&t.text.as_str()) || matches!(t.text.as_str(), "True" | "False" | "None"),
        Kind::Number | Kind::Str => true,
        Kind::Op => matches!(t.text.as_str(), ")" | "]" | "}"),
        Kind::Newline => false,
    }
}

/// the token can start an expression after another one (a comma missing)
fn starts_expr(t: &Tok) -> bool {
    match t.kind {
        Kind::Name => !KEYWORDS.contains(&t.text.as_str()) || matches!(t.text.as_str(), "True" | "False" | "None" | "lambda" | "await"),
        Kind::Number | Kind::Str => true,
        Kind::Op => t.text == "{",
        Kind::Newline => false,
    }
}

/// index of the first token of the (disjunction) expression ending at toks[end]
fn expr_start(toks: &[Tok], end: usize) -> usize {
    let mut depth = 0i32;
    let mut i = end;
    loop {
        let t = &toks[i];
        if t.kind == Kind::Op {
            match t.text.as_str() {
                ")" | "]" | "}" => depth += 1,
                "(" | "[" | "{" => {
                    if depth == 0 {
                        return i + 1;
                    }
                    depth -= 1;
                }
                "," | ":" | "=" | ";" | ":=" | "->" if depth == 0 => return i + 1,
                op if depth == 0 && op.len() >= 2 && op.ends_with('=') && !matches!(op, "==" | "<=" | ">=" | "!=") => return i + 1,
                _ => {}
            }
        } else if t.kind == Kind::Newline {
            return i + 1;
        } else if depth == 0 && t.kind == Kind::Name && KEYWORDS.contains(&t.text.as_str()) && !matches!(t.text.as_str(), "True" | "False" | "None" | "not" | "and" | "or" | "in" | "is" | "await") {
            return i + 1;
        }
        if i == 0 {
            return 0;
        }
        i -= 1;
    }
}

/// the tokens of the logical line holding `line`
fn logical_line(toks: &[Tok], line: usize) -> &[Tok] {
    let mut start = 0;
    for (i, t) in toks.iter().enumerate() {
        if t.kind == Kind::Newline {
            if t.line >= line {
                return &toks[start..i];
            }
            start = i + 1;
        }
    }
    &toks[start..]
}

/// the compound statement header a block belongs to: its message part
fn header_name(first: &Tok, toks: &[Tok]) -> Option<String> {
    let kw = if first.text == "async" { toks.get(1).map(|t| t.text.as_str()).unwrap_or("") } else { first.text.as_str() };
    Some(match kw {
        "def" => "function definition".to_string(),
        "class" => "class definition".to_string(),
        "except" if toks.get(1).is_some_and(|t| t.text == "*") => "'except*' statement".to_string(),
        "if" | "elif" | "else" | "while" | "for" | "with" | "try" | "except" | "finally" | "match" | "case" => format!("'{}' statement", kw),
        _ => return None,
    })
}

fn indent_of(src_line: &str) -> usize {
    src_line.chars().take_while(|c| *c == ' ' || *c == '\t' || *c == '\x0c').count()
}

/// RustPython's parser error in CPython's words
fn remap(src: &str, lx: &Lexed, cls: String, msg: String, line: i64, col: i64) -> Err {
    let toks = &lx.toks;
    let src_lines: Vec<&str> = src.lines().collect();
    let line_text = |l: i64| src_lines.get((l - 1).max(0) as usize).copied().unwrap_or("");
    let lu = line.max(1) as usize;
    // the token RustPython stopped at
    let at = toks.iter().position(|t| t.line == lu && t.col + 1 == col as usize).or_else(|| {
        // an unexpected newline: the NEWLINE token of that line
        toks.iter().position(|t| t.kind == Kind::Newline && t.line == lu)
    });
    if let Some(rest) = msg.strip_prefix("invalid syntax. Got unexpected token ") {
        if let Some(i) = at {
            let t = &toks[i];
            let ll = logical_line(toks, t.line);
            let prev = if i > 0 { Some(&toks[i - 1]) } else { None };
            if t.kind == Kind::Newline || rest == "Newline" {
                if let (Some(first), Some(p)) = (ll.first(), prev) {
                    let ends = ends_expr(p) || (p.kind == Kind::Name && matches!(p.text.as_str(), "else" | "try" | "finally" | "except"));
                    if header_name(first, ll).is_some() && ends && !matches!(first.text.as_str(), "match" | "case") {
                        return err("SyntaxError", "expected ':'", lu, col as usize);
                    }
                    if is_kw(p, "import") {
                        return err("SyntaxError", "Expected one or more names after 'import'", lu, col as usize);
                    }
                }
            }
            if let Some(p) = prev {
                // a NAME '=' in a condition
                if t.text == "=" && p.kind == Kind::Name && !KEYWORDS.contains(&p.text.as_str()) {
                    let cond = ll.first().is_some_and(|f| matches!(f.text.as_str(), "if" | "elif" | "while")) || t.level > 0;
                    if cond && (i < 2 || toks[i - 2].text != ".") {
                        return err("SyntaxError", "invalid syntax. Maybe you meant '==' or ':=' instead of '='?", p.line, p.col + 1);
                    }
                }
                // a NAME then an expression: print 'x' / a missing comma
                if p.kind == Kind::Name && matches!(p.text.as_str(), "print" | "exec") && starts_expr(t) {
                    let msg = format!("Missing parentheses in call to '{0}'. Did you mean {0}(...)?", p.text);
                    return err("SyntaxError", msg, p.line, p.col + 1);
                }
                let name_string = p.kind == Kind::Name && t.kind == Kind::Str;
                if t.level > 0 && ends_expr(p) && starts_expr(t) && !name_string {
                    let a = &toks[expr_start(toks, i - 1)];
                    if !(a.kind == Kind::Name && matches!(a.text.as_str(), "match" | "case" | "_" | "type")) {
                        return err("SyntaxError", "invalid syntax. Perhaps you forgot a comma?", a.line, a.col + 1);
                    }
                }
                // a conditional expression without else
                if !matches!(t.text.as_str(), "else" | ":") {
                    let line_start = ll.first().map(|f| f.line).unwrap_or(lu);
                    let j = (0..i).rev().take_while(|&j| toks[j].kind != Kind::Newline && toks[j].line >= line_start).find(|&j| is_kw(&toks[j], "if"));
                    if let Some(j) = j {
                        if j > 0 && ends_expr(&toks[j - 1]) && !toks[j..i].iter().any(|t| is_kw(t, "else")) {
                            let a = &toks[expr_start(toks, j - 1)];
                            return err("SyntaxError", "expected 'else' after 'if' expression", a.line, a.col + 1);
                        }
                    }
                }
            }
        }
        return err("SyntaxError", "invalid syntax", lu, col as usize);
    }
    if let Some(tok) = msg.strip_prefix("Got unexpected token ") {
        let _ = tok;
        return err("SyntaxError", "invalid syntax", lu, (col - 1).max(1) as usize);
    }
    if msg == "expected an indented block" {
        // the header: the logical line before the block
        let before = toks.iter().rposition(|t| t.kind == Kind::Newline && t.line < lu);
        if let Some(nl) = before {
            let ll = logical_line(toks, toks[nl].line);
            if let Some(first) = ll.first() {
                if let Some(name) = header_name(first, ll) {
                    let msg = format!("expected an indented block after {} on line {}", name, first.line);
                    return err("IndentationError", msg, lu, col as usize);
                }
            }
        }
        return err("IndentationError", msg, lu, col as usize);
    }
    if msg == "unexpected indent" {
        return err("IndentationError", msg, lu, indent_of(line_text(line)));
    }
    if let Some(what) = msg.strip_prefix("cannot assign to ") {
        let ll = logical_line(toks, lu);
        let what = if what == "operator" { "expression" } else { what };
        if let Some(first) = ll.first() {
            let aug = ll.iter().find(|t| t.kind == Kind::Op && t.level == 0 && t.text.len() >= 2 && t.text.ends_with('=') && !matches!(t.text.as_str(), "==" | "<=" | ">=" | "!=" | ":="));
            if aug.is_some() {
                let msg = format!("'{}' is an illegal expression for augmented assignment", what);
                return err("SyntaxError", msg, first.line, first.col + 1);
            }
            let eq = ll.iter().position(|t| t.kind == Kind::Op && t.level == 0 && t.text == "=");
            let tuple = eq.is_some_and(|e| ll[..e].iter().any(|t| t.text == "," && t.level == 0));
            if eq.is_some() && !tuple {
                if matches!(what, "True" | "False" | "None" | "__debug__") {
                    return err("SyntaxError", format!("cannot assign to {}", what), first.line, first.col + 1);
                }
                let msg = format!("cannot assign to {} here. Maybe you meant '==' instead of '='?", what);
                return err("SyntaxError", msg, first.line, first.col + 1);
            }
        }
        return (cls, format!("cannot assign to {}", what), line, col);
    }
    if msg.starts_with("cannot delete ") {
        let ll = logical_line(toks, lu);
        if let Some(t) = ll.get(1) {
            return err("SyntaxError", msg, t.line, t.col + 1);
        }
    }
    match msg.as_str() {
        "iterable argument unpacking follows keyword argument unpacking" => {
            // from the ',' before the argument
            let comma = lx.toks.iter().rposition(|t| t.line == lu && t.col + 1 < col as usize && t.text == ",");
            if let Some(c) = comma {
                return err("SyntaxError", msg, lu, lx.toks[c].col + 1);
            }
        }
        "positional argument follows keyword argument" => {
            // at the call's closing parenthesis
            if let Some(i) = at {
                let level = lx.toks[i].level;
                if let Some(close) = lx.toks[i..].iter().find(|t| t.text == ")" && t.level + 1 == level) {
                    return err("SyntaxError", msg, close.line, close.col + 1);
                }
            }
        }
        "Got unexpected unicode" => {
            // an unknown \N{name}: the unicode-escape codec's error, at the string
            if let Some(t) = lx.toks.iter().rev().find(|t| t.kind == Kind::Str && t.line == lu && t.col + 1 < col as usize) {
                let text: Vec<char> = line_text(line).chars().collect();
                let mut body = t.col + t.text.chars().count();
                let q = text.get(body).copied().unwrap_or('\'');
                body += if text.get(body + 1) == Some(&q) && text.get(body + 2) == Some(&q) { 3 } else { 1 };
                let rest: String = text[body.min(text.len())..].iter().collect();
                if let Some(a) = rest.find("\\N{") {
                    let b = rest[a..].find('}').map(|e| a + e).unwrap_or(rest.len() - 1);
                    let msg = format!("(unicode error) 'unicodeescape' codec can't decode bytes in position {}-{}: unknown Unicode character name", a, b);
                    return err("SyntaxError", msg, t.line, t.col + 1);
                }
            }
        }
        "'continue' outside loop" => return err("SyntaxError", "'continue' not properly in loop", lu, col as usize),
        "f-string: empty expression not allowed" => return err("SyntaxError", "f-string: valid expression required before '}'", lu, (col - 1) as usize),
        "f-string: invalid conversion character" => {
            let c = line_text(line).chars().nth((col - 2).max(0) as usize).unwrap_or('?');
            let msg = format!("f-string: invalid conversion character '{}': expected 's', 'r', or 'a'", c);
            return err("SyntaxError", msg, lu, (col - 1) as usize);
        }
        "'await' outside async function" => {
            // at module level CPython says 'outside function'
            let ind = indent_of(line_text(line));
            let in_def = src_lines[..lu - 1].iter().rev().any(|l| {
                let t = l.trim_start();
                indent_of(l) < ind && (t.starts_with("def ") || t.starts_with("async def "))
            });
            if !in_def {
                return err("SyntaxError", "'await' outside function", lu, col as usize);
            }
        }
        _ => {}
    }
    if msg.starts_with("cannot define nonlocal") && msg.ends_with("at top level.") {
        return err("SyntaxError", "nonlocal declaration not allowed at module level", lu, col as usize);
    }
    (cls, msg, line, col)
}

/// the 'try' block with neither except nor finally: its error location
fn try_without_handler(src: &str, lx: &Lexed) -> Option<Err> {
    let lines: Vec<&str> = src.lines().collect();
    for (i, t) in lx.toks.iter().enumerate() {
        if !is_kw(t, "try") || lx.toks.get(i + 1).map(|t| t.text.as_str()) != Some(":") {
            continue;
        }
        if i > 0 && lx.toks[i - 1].kind != Kind::Newline {
            continue;
        }
        let ind = indent_of(lines[t.line - 1]);
        // the block: following lines indented deeper (or blank)
        let mut last_body = t.line;
        for (n, l) in lines.iter().enumerate().skip(t.line) {
            let s = l.trim_start();
            if s.is_empty() || s.starts_with('#') {
                continue;
            }
            if indent_of(l) > ind {
                last_body = n + 1;
                continue;
            }
            // no body: an IndentationError instead
            if s.starts_with("except") || s.starts_with("finally") || last_body == t.line {
                break;
            }
            return Some(err("SyntaxError", "expected 'except' or 'finally' block", n + 1, indent_of(l) + 1));
        }
        let next = lines.iter().skip(last_body).any(|l| {
            let s = l.trim_start();
            !s.is_empty() && !s.starts_with('#')
        });
        if !next && last_body > t.line {
            return Some(err("SyntaxError", "expected 'except' or 'finally' block", last_body, lines[last_body - 1].chars().count() + 1));
        }
    }
    None
}

/// the SyntaxError CPython raises for `source`, given RustPython's
pub fn cpython_error(source: &str, cls: String, msg: String, line: i64, col: i64) -> Err {
    let lx = lex(source);
    let eof = matches!(msg.as_str(), "unexpected EOF while parsing" | "Got unexpected EOF");
    if let Some(TokErr { err: e, hard }) = &lx.error {
        // CPython's tokenizer errors win, unless the parser failed first
        // on an earlier line (only the hard ones are found by the full
        // source check after a parser error)
        if *hard || e.2 <= line || eof {
            return e.clone();
        }
    }
    if let Some((c, l, col0)) = lx.open.last() {
        if eof || line > *l as i64 {
            return err("SyntaxError", format!("'{}' was never closed", c), *l, col0 + 1);
        }
    }
    if let Some(e) = try_without_handler(source, &lx) {
        if eof || e.2 <= line {
            return e;
        }
    }
    // a block header at the end of the source
    if cls == "IndentationError" || eof {
        let last = lx.toks.iter().rposition(|t| t.kind == Kind::Newline);
        if let Some(nl) = last {
            let ll = logical_line(&lx.toks, lx.toks[nl].line);
            if let (Some(first), Some(colon)) = (ll.first(), ll.last()) {
                if colon.text == ":" && nl + 1 == lx.toks.len() {
                    if let Some(name) = header_name(first, ll) {
                        let msg = format!("expected an indented block after {} on line {}", name, first.line);
                        return err("IndentationError", msg, lx.toks[nl].line, lx.toks[nl].col + 1);
                    }
                }
            }
        }
    }
    remap(source, &lx, cls, msg, line, col)
}

/// errors CPython's grammar raises that RustPython's parser accepts
pub fn missed_error(source: &str) -> Option<Err> {
    let lx = lex(source);
    let toks = &lx.toks;
    // a generator expression that is not the sole call argument
    for (i, t) in toks.iter().enumerate() {
        if t.text != "(" || i == 0 || !ends_expr(&toks[i - 1]) {
            continue;
        }
        let inner = t.level + 1;
        let end = toks[i + 1..].iter().position(|u| u.text == ")" && u.level == t.level).map(|p| i + 1 + p).unwrap_or(toks.len());
        let args = &toks[i + 1..end];
        let commas: Vec<usize> = args.iter().enumerate().filter(|(_, u)| u.text == "," && u.level == inner).map(|(k, _)| k).collect();
        if commas.is_empty() {
            continue;
        }
        for (k, u) in args.iter().enumerate() {
            if !is_kw(u, "for") || u.level != inner {
                continue;
            }
            // the argument holding the 'for': from the comma before it
            let start = commas.iter().rev().find(|&&c| c < k).map(|c| c + 1).unwrap_or(0);
            let a = &args[start];
            return Some(err("SyntaxError", "Generator expression must be parenthesized", a.line, a.col + 1));
        }
    }
    None
}
