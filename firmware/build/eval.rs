// Evaluates the constant expressions of decl_constant! / decl_enumeration!
// (what the C compiler does for CTR_INT): integers, CONFIG_* values,
// `const` items and single-expression `const fn`s of the scanned sources,
// + - * / % << >> & | ^ ( ), unary -, `as T` casts.
use crate::kconfig::Config;
use crate::scan::Consts;

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Num(i64),
    Ident(String),
    Op(String),
}

fn lex(s: &str) -> Vec<Tok> {
    let c: Vec<char> = s.chars().collect();
    let mut out = vec![];
    let mut i = 0;
    while i < c.len() {
        let ch = c[i];
        if ch.is_whitespace() {
            i += 1;
        } else if ch == '\'' && c.get(i + 2) == Some(&'\'') {
            // a char literal
            out.push(Tok::Num(c[i + 1] as i64));
            i += 3;
        } else if ch.is_ascii_digit() {
            let mut j = i;
            while j < c.len() && (c[j].is_ascii_alphanumeric() || c[j] == '_') {
                j += 1;
            }
            let text: String = c[i..j].iter().filter(|&&x| x != '_').collect();
            // drop a type suffix (10u32, 0x10usize): it starts with u or i,
            // which are not hex digits
            let (digits, radix) = match text.strip_prefix("0x") {
                Some(h) => (h, 16),
                None => (text.as_str(), 10),
            };
            let digits = &digits[..digits.find(['u', 'i']).unwrap_or(digits.len())];
            out.push(Tok::Num(i64::from_str_radix(&digits, radix).unwrap_or_else(|_| panic!("number {}", text))));
            i = j;
        } else if ch.is_alphabetic() || ch == '_' {
            let mut j = i;
            while j < c.len() && (c[j].is_alphanumeric() || c[j] == '_' || (c[j] == ':' && c.get(j + 1) == Some(&':'))) {
                j += if c[j] == ':' { 2 } else { 1 };
            }
            let word: String = c[i..j].iter().collect();
            // a path (crate::x::NAME): its last part
            let word = word.rsplit("::").next().unwrap().to_string();
            out.push(Tok::Ident(word));
            i = j;
        } else {
            let two: String = c[i..(i + 2).min(c.len())].iter().collect();
            if two == "<<" || two == ">>" {
                out.push(Tok::Op(two));
                i += 2;
            } else {
                out.push(Tok::Op(ch.to_string()));
                i += 1;
            }
        }
    }
    out
}

pub struct Eval<'a> {
    pub config: &'a Config,
    pub consts: &'a Consts,
}

struct P<'a, 'b> {
    toks: Vec<Tok>,
    pos: usize,
    ev: &'b Eval<'a>,
    locals: Vec<(String, i64)>,
}

impl Eval<'_> {
    pub fn eval(&self, expr: &str) -> i64 {
        self.eval_with(expr, vec![])
    }

    fn eval_with(&self, expr: &str, locals: Vec<(String, i64)>) -> i64 {
        let mut p = P { toks: lex(expr), pos: 0, ev: self, locals };
        let v = p.binop(0);
        if p.pos != p.toks.len() {
            panic!("trailing tokens in constant expression {:?}", expr);
        }
        v
    }

    fn ident(&self, name: &str, locals: &[(String, i64)]) -> i64 {
        if let Some((_, v)) = locals.iter().find(|(n, _)| n == name) {
            return *v;
        }
        if let Some(v) = self.config.int(name) {
            return v;
        }
        if let Some(e) = self.consts.values.get(name) {
            return self.eval(e);
        }
        if name == "false" {
            return 0;
        }
        if name == "true" {
            return 1;
        }
        panic!("unknown name {} in a constant expression", name)
    }
}

const PREC: &[(&str, u8)] = &[("|", 1), ("^", 2), ("&", 3), ("<<", 4), (">>", 4), ("+", 5), ("-", 5), ("*", 6), ("/", 6), ("%", 6)];

impl P<'_, '_> {
    fn peek_op(&self) -> Option<String> {
        match self.toks.get(self.pos) {
            Some(Tok::Op(o)) => Some(o.clone()),
            _ => None,
        }
    }

    fn binop(&mut self, min: u8) -> i64 {
        let mut lhs = self.unary();
        loop {
            let Some(op) = self.peek_op() else { break };
            let Some(&(_, prec)) = PREC.iter().find(|(o, _)| *o == op) else { break };
            if prec < min {
                break;
            }
            self.pos += 1;
            let rhs = self.binop(prec + 1);
            lhs = match op.as_str() {
                "|" => lhs | rhs,
                "^" => lhs ^ rhs,
                "&" => lhs & rhs,
                "<<" => lhs << rhs,
                ">>" => lhs >> rhs,
                "+" => lhs + rhs,
                "-" => lhs - rhs,
                "*" => lhs * rhs,
                "/" => lhs / rhs,
                _ => lhs % rhs,
            };
        }
        lhs
    }

    fn unary(&mut self) -> i64 {
        if self.peek_op().as_deref() == Some("-") {
            self.pos += 1;
            return -self.unary();
        }
        let v = self.atom();
        // `expr as T`
        while matches!(self.toks.get(self.pos), Some(Tok::Ident(w)) if w == "as") {
            self.pos += 2;
        }
        v
    }

    fn atom(&mut self) -> i64 {
        let t = self.toks.get(self.pos).cloned().expect("constant expression ends early");
        self.pos += 1;
        match t {
            Tok::Num(n) => n,
            Tok::Op(o) if o == "(" => {
                let v = self.binop(0);
                self.expect(")");
                v
            }
            Tok::Ident(name) => {
                if self.peek_op().as_deref() != Some("(") {
                    return self.ev.ident(&name, &self.locals);
                }
                // a const fn call
                self.pos += 1;
                let mut args = vec![];
                while self.peek_op().as_deref() != Some(")") {
                    args.push(self.binop(0));
                    if self.peek_op().as_deref() == Some(",") {
                        self.pos += 1;
                    }
                }
                self.pos += 1;
                let (params, body) = self.ev.consts.funcs.get(&name).unwrap_or_else(|| panic!("unknown const fn {}", name));
                let locals = params.iter().cloned().zip(args).collect();
                self.ev.eval_with(body, locals)
            }
            other => panic!("unexpected {:?} in a constant expression", other),
        }
    }

    fn expect(&mut self, op: &str) {
        if self.peek_op().as_deref() != Some(op) {
            panic!("expected {} in a constant expression", op);
        }
        self.pos += 1;
    }
}

/// CTR_INT: " +0x%08X" / " -0x%08X" (ctr.h)
pub fn ctr_int(v: i64) -> String {
    let (sign, mag) = if v < 0 { ('-', (-v) as u32) } else { ('+', v as u32) };
    format!(" {}0x{:08X}", sign, mag)
}
