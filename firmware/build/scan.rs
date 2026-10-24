// Finds the compile time requests in a firmware source file: the
// decl_*! / sendf! / output! / shutdown! / try_shutdown! / static_string!
// macro invocations (src/ctr.rs), in source order, plus the `const` and
// single-expression `const fn` definitions constant expressions may use.
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub enum Decl {
    Command { func: String, flags: String, msg: String },
    Encoder(String),
    Output(String),
    StaticStr(String),
    CallList { list: &'static str, func: String },
    Constant { name: String, expr: String },
    ConstantStr { name: String, value: String },
    Enumeration { enumeration: String, name: String, expr: String },
    EnumerationRange { enumeration: String, name: String, expr: String, count: String },
    InitialPins(String),
    ArmcmIrq { func: String, expr: String },
    /// DECL_CTR: a compile_time_request.txt line as written
    Raw(String),
}

#[derive(Default)]
pub struct Consts {
    /// const NAME: T = expr;
    pub values: BTreeMap<String, String>,
    /// const fn name(params) -> T { expr }
    pub funcs: BTreeMap<String, (Vec<String>, String)>,
}

pub struct Scanned {
    pub decls: Vec<Decl>,
}

const MACROS: &[&str] = &[
    "decl_command",
    "decl_init",
    "decl_task",
    "decl_shutdown",
    "decl_constant",
    "decl_constant_str",
    "decl_enumeration",
    "decl_enumeration_range",
    "decl_initial_pins",
    "decl_armcm_irq",
    "armcm_enable_irq",
    "sendf",
    "output",
    "shutdown",
    "try_shutdown",
    "static_string",
    "decl_ctr",
];

/// the text with comments replaced by spaces (string literals kept)
fn strip_comments(src: &str) -> Vec<char> {
    let c: Vec<char> = src.chars().collect();
    let mut out = Vec::with_capacity(c.len());
    let mut i = 0;
    while i < c.len() {
        if c[i] == '"' {
            // a string literal: copy through the closing quote
            out.push(c[i]);
            i += 1;
            while i < c.len() && c[i] != '"' {
                if c[i] == '\\' && i + 1 < c.len() {
                    out.push(c[i]);
                    i += 1;
                }
                out.push(c[i]);
                i += 1;
            }
            if i < c.len() {
                out.push(c[i]);
                i += 1;
            }
        } else if c[i] == '/' && c.get(i + 1) == Some(&'/') {
            while i < c.len() && c[i] != '\n' {
                out.push(' ');
                i += 1;
            }
        } else if c[i] == '/' && c.get(i + 1) == Some(&'*') {
            while i < c.len() && !(c[i] == '*' && c.get(i + 1) == Some(&'/')) {
                out.push(if c[i] == '\n' { '\n' } else { ' ' });
                i += 1;
            }
            out.push(' ');
            out.push(' ');
            i += 2;
        } else if c[i] == '\'' && c.get(i + 2) == Some(&'\'') {
            // a char literal ('x'), not a lifetime: kept (constant
            // expressions use them, GPIO('A', 0)) unless it is a quote
            out.extend(['\'', if c[i + 1] == '"' { ' ' } else { c[i + 1] }, '\'']);
            i += 3;
        } else if c[i] == '\'' && c.get(i + 1) == Some(&'\\') {
            // an escaped char literal ('\n', '\'', '\x7e'): up to its quote
            let mut j = i + 3;
            while j < c.len() && c[j] != '\'' {
                j += 1;
            }
            out.extend(std::iter::repeat(' ').take(j + 1 - i));
            i = j + 1;
        } else {
            out.push(c[i]);
            i += 1;
        }
    }
    out
}

/// the arguments of a macro call whose '(' is at c[open]; returns them
/// split at top level commas, and the index after the ')'
pub fn macro_args(c: &[char], open: usize) -> (Vec<String>, usize) {
    let mut depth = 0;
    let mut args = vec![];
    let mut cur = String::new();
    let mut i = open;
    while i < c.len() {
        let ch = c[i];
        match ch {
            '"' => {
                cur.push(ch);
                i += 1;
                while i < c.len() && c[i] != '"' {
                    if c[i] == '\\' {
                        cur.push(c[i]);
                        i += 1;
                    }
                    cur.push(c[i]);
                    i += 1;
                }
                cur.push('"');
            }
            '(' | '[' | '{' => {
                depth += 1;
                if depth > 1 {
                    cur.push(ch);
                }
            }
            ')' | ']' | '}' => {
                depth -= 1;
                if depth == 0 {
                    if !cur.trim().is_empty() {
                        args.push(cur.trim().to_string());
                    }
                    return (args, i + 1);
                }
                cur.push(ch);
            }
            ',' if depth == 1 => {
                args.push(cur.trim().to_string());
                cur.clear();
            }
            _ => cur.push(ch),
        }
        i += 1;
    }
    panic!("unterminated macro call");
}

/// the value of a string literal or concat!() of literals, None for
/// anything else
pub fn literal(arg: &str) -> Option<String> {
    let arg = arg.trim();
    if let Some(inner) = arg.strip_prefix("concat!(").and_then(|a| a.strip_suffix(')')) {
        let c: Vec<char> = format!("({})", inner).chars().collect();
        let (parts, _) = macro_args(&c, 0);
        return parts.iter().map(|p| literal(p)).collect::<Option<Vec<_>>>().map(|v| v.concat());
    }
    let body = arg.strip_prefix('"')?.strip_suffix('"')?;
    let mut out = String::new();
    let mut it = body.chars();
    while let Some(ch) = it.next() {
        if ch != '\\' {
            out.push(ch);
            continue;
        }
        match it.next() {
            Some('n') => out.push('\n'),
            Some('t') => out.push('\t'),
            Some('0') => out.push('\0'),
            Some('\\') => out.push('\\'),
            Some('"') => out.push('"'),
            Some('\n') => {
                // line continuation: skip the leading whitespace
                let rest: String = it.clone().collect();
                let skip = rest.len() - rest.trim_start().len();
                for _ in 0..rest[..skip].chars().count() {
                    it.next();
                }
            }
            other => panic!("unsupported escape {:?} in {}", other, arg),
        }
    }
    Some(out)
}

fn ident_at(c: &[char], i: usize) -> Option<(String, usize)> {
    if i > 0 && (c[i - 1].is_alphanumeric() || c[i - 1] == '_' || c[i - 1] == '$') {
        return None;
    }
    let mut j = i;
    while j < c.len() && (c[j].is_alphanumeric() || c[j] == '_') {
        j += 1;
    }
    if j == i || c[i].is_ascii_digit() {
        return None;
    }
    Some((c[i..j].iter().collect(), j))
}

fn skip_ws(c: &[char], mut i: usize) -> usize {
    while i < c.len() && c[i].is_whitespace() {
        i += 1;
    }
    i
}

fn flags(arg: &str) -> String {
    match arg {
        "HF_IN_SHUTDOWN" => "0x01".to_string(),
        "0" => "0".to_string(),
        other => panic!("unknown command flags {}", other),
    }
}

/// a cfg predicate: name | all(..) | any(..) | not(..)
fn cfg_true(pred: &str, enabled: &dyn Fn(&str) -> bool) -> bool {
    let pred = pred.trim();
    for (f, all) in [("all(", true), ("any(", false)] {
        if let Some(inner) = pred.strip_prefix(f).and_then(|p| p.strip_suffix(')')) {
            let c: Vec<char> = format!("({})", inner).chars().collect();
            let (parts, _) = macro_args(&c, 0);
            return if all { parts.iter().all(|p| cfg_true(p, enabled)) } else { parts.iter().any(|p| cfg_true(p, enabled)) };
        }
    }
    if let Some(inner) = pred.strip_prefix("not(").and_then(|p| p.strip_suffix(')')) {
        return !cfg_true(inner, enabled);
    }
    enabled(pred)
}

/// the index after the item starting at or after c[i]: up to a ';' or
/// a balanced {} block at the top level
fn skip_item(c: &[char], mut i: usize) -> usize {
    let mut depth = 0;
    while i < c.len() {
        match c[i] {
            '"' => {
                i += 1;
                while i < c.len() && c[i] != '"' {
                    if c[i] == '\\' {
                        i += 1;
                    }
                    i += 1;
                }
            }
            '(' | '[' => depth += 1,
            ')' | ']' => depth -= 1,
            ';' if depth == 0 => return i + 1,
            '{' if depth == 0 => {
                let (_, after) = macro_args(c, i);
                return after;
            }
            _ => {}
        }
        i += 1;
    }
    i
}

/// the compile time requests of one source file, in source order;
/// items under a false #[cfg(..)] are left out (the C #if)
pub fn scan(src: &str, consts: &mut Consts, enabled: &dyn Fn(&str) -> bool) -> Scanned {
    let c = strip_comments(src);
    let mut decls = vec![];
    let mut i = 0;
    while i < c.len() {
        if c[i] == '#' && c.get(i + 1) == Some(&'[') {
            let (attr, after) = macro_args(&c, i + 1);
            let attr = attr.join(",");
            if let Some(pred) = attr.strip_prefix("cfg(").and_then(|a| a.strip_suffix(')')) {
                if !cfg_true(pred, enabled) {
                    i = skip_item(&c, after);
                    continue;
                }
            }
            i = after;
            continue;
        }
        let Some((word, end)) = ident_at(&c, i) else {
            i += 1;
            continue;
        };
        if word == "const" {
            i = scan_const(&c, end, consts);
            continue;
        }
        let bang = skip_ws(&c, end);
        if !MACROS.contains(&word.as_str()) || c.get(bang) != Some(&'!') {
            i = end;
            continue;
        }
        let open = skip_ws(&c, bang + 1);
        if c.get(open) != Some(&'(') {
            i = end;
            continue;
        }
        let (args, after) = macro_args(&c, open);
        i = after;
        // inside macro_rules bodies the arguments are $metavariables
        if args.iter().any(|a| a.starts_with('$')) {
            continue;
        }
        let lit = |n: usize| literal(&args[n]).unwrap_or_else(|| panic!("{}!: argument {} is not a literal", word, n));
        let d = match word.as_str() {
            "decl_command" => match args.len() {
                2 => Decl::Command { func: args[0].clone(), flags: "0".into(), msg: lit(1) },
                3 => Decl::Command { func: args[0].clone(), flags: flags(&args[1]), msg: lit(2) },
                _ => panic!("decl_command!: {:?}", args),
            },
            "decl_init" => Decl::CallList { list: "ctr_run_initfuncs", func: args[0].clone() },
            "decl_task" => Decl::CallList { list: "ctr_run_taskfuncs", func: args[0].clone() },
            "decl_shutdown" => Decl::CallList { list: "ctr_run_shutdownfuncs", func: args[0].clone() },
            "decl_constant" => Decl::Constant { name: lit(0), expr: args[1].clone() },
            "decl_constant_str" => Decl::ConstantStr { name: lit(0), value: args[1].clone() },
            "decl_enumeration" => Decl::Enumeration { enumeration: lit(0), name: lit(1), expr: args[2].clone() },
            "decl_enumeration_range" => Decl::EnumerationRange {
                enumeration: lit(0),
                name: lit(1),
                expr: args[2].clone(),
                count: args[3].clone(),
            },
            "decl_initial_pins" => Decl::InitialPins(args[0].clone()),
            "decl_armcm_irq" | "armcm_enable_irq" => Decl::ArmcmIrq { func: args[0].clone(), expr: args[1].clone() },
            "sendf" => Decl::Encoder(lit(0)),
            "output" => Decl::Output(lit(0)),
            "decl_ctr" => Decl::Raw(lit(0)),
            _ => Decl::StaticStr(lit(0)),
        };
        decls.push(d);
    }
    Scanned { decls }
}

/// `const NAME: T = expr;` or `const fn name(a: T, ..) -> T { expr }`
fn scan_const(c: &[char], i: usize, consts: &mut Consts) -> usize {
    let j = skip_ws(c, i);
    let Some((word, end)) = ident_at(c, j) else { return i };
    if word == "fn" {
        let j = skip_ws(c, end);
        let Some((name, end)) = ident_at(c, j) else { return i };
        let open = skip_ws(c, end);
        if c.get(open) != Some(&'(') {
            return end;
        }
        let (params, after) = macro_args(c, open);
        let params: Vec<String> = params.iter().map(|p| p.split(':').next().unwrap().trim().to_string()).collect();
        let Some(brace) = (after..c.len()).find(|&k| c[k] == '{') else { return after };
        let (body, after) = macro_args(c, brace);
        if body.len() == 1 && !body[0].contains(';') {
            consts.funcs.insert(name, (params, body[0].clone()));
        }
        return after;
    }
    let colon = skip_ws(c, end);
    if c.get(colon) != Some(&':') {
        return end;
    }
    let Some(eq) = (colon..c.len()).find(|&k| c[k] == '=' || c[k] == ';') else { return end };
    if c[eq] != '=' {
        return eq;
    }
    let Some(semi) = (eq..c.len()).find(|&k| c[k] == ';') else { return eq };
    let expr: String = c[eq + 1..semi].iter().collect();
    consts.values.insert(word, expr.trim().to_string());
    semi
}
