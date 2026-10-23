// Jinja2 3.1.6 parser (port of jinja2/parser.py), with the "do" and
// "loopcontrols" extensions klippy enables
use super::lexer::{describe_token, describe_token_expr, TokVal, Token};
use super::nodes::*;
use super::value::Value;
use super::TemplateSyntaxError;
use crate::pyfmt::py_str_repr;
use std::collections::VecDeque;
use std::rc::Rc;

const STATEMENT_KEYWORDS: &[&str] =
    &["for", "if", "block", "extends", "print", "macro", "include", "from", "import", "set", "with", "autoescape"];
const EXTENSION_TAGS: &[&str] = &["do", "break", "continue"];

/// lexer.TokenStream
struct TokenStream {
    tokens: std::vec::IntoIter<Token>,
    pushed: VecDeque<Token>,
    current: Token,
}

impl TokenStream {
    fn new(tokens: Vec<Token>) -> Self {
        let mut ts = TokenStream {
            tokens: tokens.into_iter(),
            pushed: VecDeque::new(),
            current: Token { lineno: 1, ty: "initial", value: TokVal::None },
        };
        ts.next();
        ts
    }

    /// next(stream): go one token ahead and return the old one
    fn next(&mut self) -> Token {
        let rv = self.current.clone();
        if let Some(t) = self.pushed.pop_front() {
            self.current = t;
        } else if self.current.ty != "eof" {
            match self.tokens.next() {
                Some(t) => self.current = t,
                None => self.current = Token { lineno: self.current.lineno, ty: "eof", value: TokVal::None },
            }
        }
        rv
    }

    fn look(&mut self) -> Token {
        let old_token = self.next();
        let result = self.current.clone();
        self.pushed.push_back(result.clone());
        self.current = old_token;
        result
    }

    fn skip(&mut self, n: usize) {
        for _ in 0..n {
            self.next();
        }
    }

    fn next_if(&mut self, expr: &str) -> Option<Token> {
        if self.current.test(expr) {
            return Some(self.next());
        }
        None
    }

    fn skip_if(&mut self, expr: &str) -> bool {
        self.next_if(expr).is_some()
    }

    fn is_eos(&self) -> bool {
        self.pushed.is_empty() && self.current.ty == "eof"
    }
}

pub struct Parser {
    stream: TokenStream,
    name: Option<String>,
    tag_stack: Vec<String>,
    end_token_stack: Vec<Vec<&'static str>>,
}

type PResult<T> = Result<T, TemplateSyntaxError>;

fn repr(s: &str) -> String {
    py_str_repr(s)
}

impl Parser {
    pub fn new(tokens: Vec<Token>, name: Option<&str>) -> Self {
        Parser { stream: TokenStream::new(tokens), name: name.map(|s| s.to_string()), tag_stack: vec![], end_token_stack: vec![] }
    }

    fn fail<T>(&self, msg: impl Into<String>, lineno: Option<usize>) -> PResult<T> {
        let lineno = lineno.unwrap_or(self.stream.current.lineno);
        Err(TemplateSyntaxError::new(msg, lineno, self.name.clone()))
    }

    /// TokenStream.expect()
    fn expect(&mut self, expr: &str) -> PResult<Token> {
        if !self.stream.current.test(expr) {
            let expr = describe_token_expr(expr);
            if self.stream.current.ty == "eof" {
                return self.fail(format!("unexpected end of template, expected {}.", repr(&expr)), None);
            }
            return self.fail(format!("expected token {}, got {}", repr(&expr), repr(&describe_token(&self.stream.current))), None);
        }
        Ok(self.stream.next())
    }

    fn fail_ut_eof<T>(&self, name: Option<&str>, end_token_stack: &[Vec<&'static str>], lineno: Option<usize>) -> PResult<T> {
        let mut expected: Vec<String> = vec![];
        for exprs in end_token_stack {
            for e in exprs {
                expected.push(describe_token_expr(e));
            }
        }
        let currently_looking = end_token_stack
            .last()
            .map(|last| last.iter().map(|e| repr(&describe_token_expr(e))).collect::<Vec<_>>().join(" or "));
        let mut message = vec![match name {
            None => "Unexpected end of template.".to_string(),
            Some(n) => format!("Encountered unknown tag {}.", repr(n)),
        }];
        if let Some(cl) = currently_looking.filter(|s| !s.is_empty()) {
            if name.map(|n| expected.iter().any(|e| e == n)).unwrap_or(false) {
                message.push(format!(
                    "You probably made a nesting mistake. Jinja is expecting this tag, but currently looking for {}.",
                    cl
                ));
            } else {
                message.push(format!("Jinja was looking for the following tags: {}.", cl));
            }
        }
        if let Some(tag) = self.tag_stack.last() {
            message.push(format!("The innermost block that needs to be closed is {}.", repr(tag)));
        }
        self.fail(message.join(" "), lineno)
    }

    fn fail_unknown_tag<T>(&self, name: &str, lineno: usize) -> PResult<T> {
        let stack = self.end_token_stack.clone();
        self.fail_ut_eof(Some(name), &stack, Some(lineno))
    }

    fn fail_eof<T>(&self, end_tokens: Option<&[&'static str]>, lineno: Option<usize>) -> PResult<T> {
        let mut stack = self.end_token_stack.clone();
        if let Some(e) = end_tokens {
            stack.push(e.to_vec());
        }
        self.fail_ut_eof(None, &stack, lineno)
    }

    fn is_tuple_end(&self, extra_end_rules: Option<&[&str]>) -> bool {
        if matches!(self.stream.current.ty, "variable_end" | "block_end" | "rparen") {
            return true;
        }
        if let Some(rules) = extra_end_rules {
            return self.stream.current.test_any(rules);
        }
        false
    }

    fn parse_statement(&mut self) -> PResult<Vec<Stmt>> {
        let token = self.stream.current.clone();
        if token.ty != "name" {
            return self.fail("tag name expected", Some(token.lineno));
        }
        let value = token.str_value().to_string();
        self.tag_stack.push(value.clone());
        let res = if STATEMENT_KEYWORDS.contains(&value.as_str()) {
            match value.as_str() {
                "for" => self.parse_for().map(|s| vec![s]),
                "if" => self.parse_if().map(|s| vec![s]),
                "block" => self.parse_block().map(|s| vec![s]),
                "extends" => self.parse_extends().map(|s| vec![s]),
                "print" => self.parse_print().map(|s| vec![s]),
                "macro" => self.parse_macro().map(|s| vec![s]),
                "include" => self.parse_include().map(|s| vec![s]),
                "from" => self.parse_from().map(|s| vec![s]),
                "import" => self.parse_import().map(|s| vec![s]),
                "set" => self.parse_set().map(|s| vec![s]),
                "with" => self.parse_with().map(|s| vec![s]),
                _ => self.parse_autoescape().map(|s| vec![s]),
            }
        } else if value == "call" {
            self.parse_call_block().map(|s| vec![s])
        } else if value == "filter" {
            self.parse_filter_block().map(|s| vec![s])
        } else if EXTENSION_TAGS.contains(&value.as_str()) {
            let token = self.stream.next();
            match value.as_str() {
                "do" => self.parse_tuple(false, true, None, false, false).map(|e| {
                    vec![Stmt { lineno: token.lineno, kind: StmtKind::ExprStmt(e) }]
                }),
                "break" => Ok(vec![Stmt { lineno: token.lineno, kind: StmtKind::Break }]),
                _ => Ok(vec![Stmt { lineno: token.lineno, kind: StmtKind::Continue }]),
            }
        } else {
            // did not work out, remove the token we pushed by accident
            // from the stack so that the unknown tag fail function can
            // produce a proper error message.
            self.tag_stack.pop();
            return self.fail_unknown_tag(&value, token.lineno);
        };
        self.tag_stack.pop();
        res
    }

    fn parse_statements(&mut self, end_tokens: &[&'static str], drop_needle: bool) -> PResult<Vec<Stmt>> {
        // the first token may be a colon for python compatibility
        self.stream.skip_if("colon");
        self.expect("block_end")?;
        let result = self.subparse(Some(end_tokens))?;
        // we reached the end of the template too early, the subparser
        // does not check for this, so we do that now
        if self.stream.current.ty == "eof" {
            return self.fail_eof(Some(end_tokens), None);
        }
        if drop_needle {
            self.stream.next();
        }
        Ok(result)
    }

    fn parse_set(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let target = self.parse_assign_target(true, false, None, true)?;
        if self.stream.skip_if("assign") {
            let expr = self.parse_tuple(false, true, None, false, false)?;
            return Ok(Stmt { lineno, kind: StmtKind::Assign { target, node: expr } });
        }
        let filter_node = self.parse_filter(None, false)?;
        let body = self.parse_statements(&["name:endset"], true)?;
        Ok(Stmt { lineno, kind: StmtKind::AssignBlock { target, filter: filter_node, body } })
    }

    fn parse_for(&mut self) -> PResult<Stmt> {
        let lineno = self.expect("name:for")?.lineno;
        let target = self.parse_assign_target(true, false, Some(&["name:in"]), false)?;
        self.expect("name:in")?;
        let iter = self.parse_tuple(false, false, Some(&["name:recursive"]), false, false)?;
        let mut test = None;
        if self.stream.skip_if("name:if") {
            test = Some(self.parse_expression(true)?);
        }
        let recursive = self.stream.skip_if("name:recursive");
        let body = self.parse_statements(&["name:endfor", "name:else"], false)?;
        let else_ = if self.stream.next().str_value() == "endfor" {
            vec![]
        } else {
            self.parse_statements(&["name:endfor"], true)?
        };
        Ok(Stmt { lineno, kind: StmtKind::For { target, iter, body, else_, test, recursive } })
    }

    fn parse_if(&mut self) -> PResult<Stmt> {
        let lineno = self.expect("name:if")?.lineno;
        let test = self.parse_tuple(false, false, None, false, false)?;
        let body = self.parse_statements(&["name:elif", "name:else", "name:endif"], false)?;
        let mut elif_ = vec![];
        let mut else_ = vec![];
        let mut token = self.stream.next();
        loop {
            if token.test("name:elif") {
                let t = self.parse_tuple(false, false, None, false, false)?;
                let b = self.parse_statements(&["name:elif", "name:else", "name:endif"], false)?;
                elif_.push((t, b));
                token = self.stream.next();
                continue;
            } else if token.test("name:else") {
                else_ = self.parse_statements(&["name:endif"], true)?;
            }
            break;
        }
        Ok(Stmt { lineno, kind: StmtKind::If { test, body, elif_, else_ } })
    }

    fn parse_with(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let mut targets = vec![];
        let mut values = vec![];
        while self.stream.current.ty != "block_end" {
            if !targets.is_empty() {
                self.expect("comma")?;
            }
            let target = self.parse_assign_target(true, false, None, false)?;
            targets.push(target);
            self.expect("assign")?;
            values.push(self.parse_expression(true)?);
        }
        let body = self.parse_statements(&["name:endwith"], true)?;
        Ok(Stmt { lineno, kind: StmtKind::With { targets, values, body } })
    }

    fn parse_autoescape(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let _option = self.parse_expression(true)?;
        let body = self.parse_statements(&["name:endautoescape"], true)?;
        Ok(Stmt { lineno, kind: StmtKind::Scope(body) })
    }

    fn parse_block(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let name = self.expect("name")?.str_value().to_string();
        let _scoped = self.stream.skip_if("name:scoped");
        let required = self.stream.skip_if("name:required");

        // common problem people encounter when switching from django
        // to jinja.  we do not support hyphens in block names, so let's
        // raise a nicer error message in that case.
        if self.stream.current.ty == "sub" {
            return self.fail(
                "Block names in Jinja have to be valid Python identifiers and may not contain hyphens, use an underscore instead.",
                None,
            );
        }

        let body = self.parse_statements(&["name:endblock"], true)?;

        // enforce that required blocks only contain whitespace or comments
        if required {
            for body_node in &body {
                let ok = match &body_node.kind {
                    StmtKind::Output(nodes) => nodes.iter().all(|n| match n {
                        OutputNode::Data(d) => !d.is_empty() && d.chars().all(crate::pyfmt::py_isspace),
                        _ => false,
                    }),
                    _ => false,
                };
                if !ok {
                    return self.fail("Required blocks can only contain comments or whitespace", None);
                }
            }
        }

        self.stream.skip_if(&format!("name:{}", name));
        Ok(Stmt { lineno, kind: StmtKind::Block { name, body } })
    }

    fn parse_extends(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let template = self.parse_expression(true)?;
        Ok(Stmt { lineno, kind: StmtKind::Extends(template) })
    }

    fn parse_import_context(&mut self) {
        if self.stream.current.test_any(&["name:with", "name:without"]) && self.stream.look().test("name:context") {
            self.stream.next();
            self.stream.skip(1);
        }
    }

    fn parse_include(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let template = self.parse_expression(true)?;
        let ignore_missing = if self.stream.current.test("name:ignore") && self.stream.look().test("name:missing") {
            self.stream.skip(2);
            true
        } else {
            false
        };
        self.parse_import_context();
        Ok(Stmt { lineno, kind: StmtKind::Include { template, ignore_missing } })
    }

    fn parse_import(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let template = self.parse_expression(true)?;
        self.expect("name:as")?;
        let target = self.parse_name_target()?;
        self.parse_import_context();
        Ok(Stmt { lineno, kind: StmtKind::Import { template, target } })
    }

    fn parse_from(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let template = self.parse_expression(true)?;
        self.expect("name:import")?;
        let mut names = vec![];
        let parse_context = |p: &mut Parser| -> bool {
            let v = p.stream.current.str_value().to_string();
            if (v == "with" || v == "without") && p.stream.look().test("name:context") {
                p.stream.next();
                p.stream.skip(1);
                return true;
            }
            false
        };
        loop {
            if !names.is_empty() {
                self.expect("comma")?;
            }
            if self.stream.current.ty == "name" {
                if parse_context(self) {
                    break;
                }
                let lineno = self.stream.current.lineno;
                let target = self.parse_name_target()?;
                if target.starts_with('_') {
                    return self.fail("names starting with an underline can not be imported", Some(lineno));
                }
                if self.stream.skip_if("name:as") {
                    let alias = self.parse_name_target()?;
                    names.push((target, Some(alias)));
                } else {
                    names.push((target, None));
                }
                if parse_context(self) || self.stream.current.ty != "comma" {
                    break;
                }
            } else {
                self.expect("name")?;
            }
        }
        Ok(Stmt { lineno, kind: StmtKind::FromImport { template, names } })
    }

    fn parse_signature(&mut self) -> PResult<(Vec<String>, Vec<Expr>)> {
        let mut args = vec![];
        let mut defaults = vec![];
        self.expect("lparen")?;
        while self.stream.current.ty != "rparen" {
            if !args.is_empty() {
                self.expect("comma")?;
            }
            let arg = self.parse_name_target()?;
            if self.stream.skip_if("assign") {
                defaults.push(self.parse_expression(true)?);
            } else if !defaults.is_empty() {
                return self.fail("non-default argument follows default argument", None);
            }
            args.push(arg);
        }
        self.expect("rparen")?;
        Ok((args, defaults))
    }

    fn parse_call_block(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let (args, defaults) = if self.stream.current.ty == "lparen" { self.parse_signature()? } else { (vec![], vec![]) };
        let call_node = self.parse_expression(true)?;
        if !matches!(call_node.kind, ExprKind::Call(..)) {
            return self.fail("expected call", Some(lineno));
        }
        let body = self.parse_statements(&["name:endcall"], true)?;
        let def = Rc::new(MacroDef { name: "caller".into(), args, defaults, body, lineno });
        Ok(Stmt { lineno, kind: StmtKind::CallBlock { call: call_node, def } })
    }

    fn parse_filter_block(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let filter = self.parse_filter(None, true)?.expect("filter");
        let body = self.parse_statements(&["name:endfilter"], true)?;
        Ok(Stmt { lineno, kind: StmtKind::FilterBlock { filter, body } })
    }

    fn parse_macro(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let name = self.parse_name_target()?;
        let (args, defaults) = self.parse_signature()?;
        let body = self.parse_statements(&["name:endmacro"], true)?;
        Ok(Stmt { lineno, kind: StmtKind::Macro(Rc::new(MacroDef { name, args, defaults, body, lineno })) })
    }

    fn parse_print(&mut self) -> PResult<Stmt> {
        let lineno = self.stream.next().lineno;
        let mut nodes = vec![];
        while self.stream.current.ty != "block_end" {
            if !nodes.is_empty() {
                self.expect("comma")?;
            }
            nodes.push(OutputNode::Expr(self.parse_expression(true)?));
        }
        Ok(Stmt { lineno, kind: StmtKind::Output(nodes) })
    }

    /// parse_assign_target(name_only=True).name
    fn parse_name_target(&mut self) -> PResult<String> {
        let token = self.expect("name")?;
        let name = token.str_value().to_string();
        let target = Expr::new(ExprKind::Name(name.clone()), token.lineno);
        if !target.can_assign() {
            return self.fail(format!("can't assign to {}", repr(target.type_name())), Some(target.lineno));
        }
        Ok(name)
    }

    fn parse_assign_target(
        &mut self,
        with_tuple: bool,
        name_only: bool,
        extra_end_rules: Option<&[&str]>,
        with_namespace: bool,
    ) -> PResult<Expr> {
        let target = if name_only {
            let token = self.expect("name")?;
            Expr::new(ExprKind::Name(token.str_value().to_string()), token.lineno)
        } else if with_tuple {
            self.parse_tuple(true, true, extra_end_rules, false, with_namespace)?
        } else {
            self.parse_primary(with_namespace)?
        };
        if !target.can_assign() {
            return self.fail(format!("can't assign to {}", repr(target.type_name())), Some(target.lineno));
        }
        Ok(target)
    }

    fn parse_expression(&mut self, with_condexpr: bool) -> PResult<Expr> {
        if with_condexpr {
            return self.parse_condexpr();
        }
        self.parse_or()
    }

    fn parse_condexpr(&mut self) -> PResult<Expr> {
        let mut lineno = self.stream.current.lineno;
        let mut expr1 = self.parse_or()?;
        while self.stream.skip_if("name:if") {
            let expr2 = self.parse_or()?;
            let expr3 = if self.stream.skip_if("name:else") { Some(Box::new(self.parse_condexpr()?)) } else { None };
            expr1 = Expr::new(ExprKind::CondExpr(Box::new(expr2), Box::new(expr1), expr3), lineno);
            lineno = self.stream.current.lineno;
        }
        Ok(expr1)
    }

    fn parse_or(&mut self) -> PResult<Expr> {
        let mut lineno = self.stream.current.lineno;
        let mut left = self.parse_and()?;
        while self.stream.skip_if("name:or") {
            let right = self.parse_and()?;
            left = Expr::new(ExprKind::Or(Box::new(left), Box::new(right)), lineno);
            lineno = self.stream.current.lineno;
        }
        Ok(left)
    }

    fn parse_and(&mut self) -> PResult<Expr> {
        let mut lineno = self.stream.current.lineno;
        let mut left = self.parse_not()?;
        while self.stream.skip_if("name:and") {
            let right = self.parse_not()?;
            left = Expr::new(ExprKind::And(Box::new(left), Box::new(right)), lineno);
            lineno = self.stream.current.lineno;
        }
        Ok(left)
    }

    fn parse_not(&mut self) -> PResult<Expr> {
        if self.stream.current.test("name:not") {
            let lineno = self.stream.next().lineno;
            return Ok(Expr::new(ExprKind::Not(Box::new(self.parse_not()?)), lineno));
        }
        self.parse_compare()
    }

    fn parse_compare(&mut self) -> PResult<Expr> {
        let mut lineno = self.stream.current.lineno;
        let expr = self.parse_math1()?;
        let mut ops = vec![];
        loop {
            let token_type = self.stream.current.ty;
            let op = match token_type {
                "eq" => Some(CmpOp::Eq),
                "ne" => Some(CmpOp::Ne),
                "lt" => Some(CmpOp::Lt),
                "lteq" => Some(CmpOp::LtEq),
                "gt" => Some(CmpOp::Gt),
                "gteq" => Some(CmpOp::GtEq),
                _ => None,
            };
            if let Some(op) = op {
                self.stream.next();
                ops.push((op, self.parse_math1()?));
            } else if self.stream.skip_if("name:in") {
                ops.push((CmpOp::In, self.parse_math1()?));
            } else if self.stream.current.test("name:not") && self.stream.look().test("name:in") {
                self.stream.skip(2);
                ops.push((CmpOp::NotIn, self.parse_math1()?));
            } else {
                break;
            }
            lineno = self.stream.current.lineno;
        }
        if ops.is_empty() {
            return Ok(expr);
        }
        Ok(Expr::new(ExprKind::Compare(Box::new(expr), ops), lineno))
    }

    fn parse_math1(&mut self) -> PResult<Expr> {
        let mut lineno = self.stream.current.lineno;
        let mut left = self.parse_concat()?;
        while matches!(self.stream.current.ty, "add" | "sub") {
            let op = if self.stream.current.ty == "add" { BinOp::Add } else { BinOp::Sub };
            self.stream.next();
            let right = self.parse_concat()?;
            left = Expr::new(ExprKind::BinOp(op, Box::new(left), Box::new(right)), lineno);
            lineno = self.stream.current.lineno;
        }
        Ok(left)
    }

    fn parse_concat(&mut self) -> PResult<Expr> {
        let lineno = self.stream.current.lineno;
        let mut args = vec![self.parse_math2()?];
        while self.stream.current.ty == "tilde" {
            self.stream.next();
            args.push(self.parse_math2()?);
        }
        if args.len() == 1 {
            return Ok(args.pop().unwrap());
        }
        Ok(Expr::new(ExprKind::Concat(args), lineno))
    }

    fn parse_math2(&mut self) -> PResult<Expr> {
        let mut lineno = self.stream.current.lineno;
        let mut left = self.parse_pow()?;
        loop {
            let op = match self.stream.current.ty {
                "mul" => BinOp::Mul,
                "div" => BinOp::Div,
                "floordiv" => BinOp::FloorDiv,
                "mod" => BinOp::Mod,
                _ => break,
            };
            self.stream.next();
            let right = self.parse_pow()?;
            left = Expr::new(ExprKind::BinOp(op, Box::new(left), Box::new(right)), lineno);
            lineno = self.stream.current.lineno;
        }
        Ok(left)
    }

    fn parse_pow(&mut self) -> PResult<Expr> {
        let mut lineno = self.stream.current.lineno;
        let mut left = self.parse_unary(true)?;
        while self.stream.current.ty == "pow" {
            self.stream.next();
            let right = self.parse_unary(true)?;
            left = Expr::new(ExprKind::BinOp(BinOp::Pow, Box::new(left), Box::new(right)), lineno);
            lineno = self.stream.current.lineno;
        }
        Ok(left)
    }

    fn parse_unary(&mut self, with_filter: bool) -> PResult<Expr> {
        let token_type = self.stream.current.ty;
        let lineno = self.stream.current.lineno;
        let mut node = if token_type == "sub" {
            self.stream.next();
            Expr::new(ExprKind::Neg(Box::new(self.parse_unary(false)?)), lineno)
        } else if token_type == "add" {
            self.stream.next();
            Expr::new(ExprKind::Pos(Box::new(self.parse_unary(false)?)), lineno)
        } else {
            self.parse_primary(false)?
        };
        node = self.parse_postfix(node)?;
        if with_filter {
            node = self.parse_filter_expr(node)?;
        }
        Ok(node)
    }

    fn parse_primary(&mut self, with_namespace: bool) -> PResult<Expr> {
        let token = self.stream.current.clone();
        let node = match token.ty {
            "name" => {
                self.stream.next();
                let v = token.str_value();
                if matches!(v, "true" | "false" | "True" | "False") {
                    Expr::new(ExprKind::Const(Value::Bool(v == "true" || v == "True")), token.lineno)
                } else if matches!(v, "none" | "None") {
                    Expr::new(ExprKind::Const(Value::None), token.lineno)
                } else if with_namespace && self.stream.current.ty == "dot" {
                    // If namespace attributes are allowed at this point, and the next
                    // token is a dot, produce a namespace reference.
                    self.stream.next();
                    let attr = self.expect("name")?;
                    Expr::new(ExprKind::NSRef(v.to_string(), attr.str_value().to_string()), token.lineno)
                } else {
                    Expr::new(ExprKind::Name(v.to_string()), token.lineno)
                }
            }
            "string" => {
                self.stream.next();
                let mut buf = token.str_value().to_string();
                while self.stream.current.ty == "string" {
                    buf.push_str(self.stream.current.str_value());
                    self.stream.next();
                }
                Expr::new(ExprKind::Const(Value::str(&buf)), token.lineno)
            }
            "integer" | "float" => {
                self.stream.next();
                let v = match token.value {
                    TokVal::Int(i) => Value::Int(i),
                    TokVal::Float(f) => Value::Float(f),
                    _ => Value::None,
                };
                Expr::new(ExprKind::Const(v), token.lineno)
            }
            "lparen" => {
                self.stream.next();
                let node = self.parse_tuple(false, true, None, true, false)?;
                self.expect("rparen")?;
                node
            }
            "lbracket" => self.parse_list()?,
            "lbrace" => self.parse_dict()?,
            _ => return self.fail(format!("unexpected {}", repr(&describe_token(&token))), Some(token.lineno)),
        };
        Ok(node)
    }

    fn parse_tuple(
        &mut self,
        simplified: bool,
        with_condexpr: bool,
        extra_end_rules: Option<&[&str]>,
        explicit_parentheses: bool,
        with_namespace: bool,
    ) -> PResult<Expr> {
        let mut lineno = self.stream.current.lineno;
        let mut args = vec![];
        let mut is_tuple = false;
        loop {
            if !args.is_empty() {
                self.expect("comma")?;
            }
            if self.is_tuple_end(extra_end_rules) {
                break;
            }
            let e = if simplified { self.parse_primary(with_namespace)? } else { self.parse_expression(with_condexpr)? };
            args.push(e);
            if self.stream.current.ty == "comma" {
                is_tuple = true;
            } else {
                break;
            }
            lineno = self.stream.current.lineno;
        }
        if !is_tuple {
            if !args.is_empty() {
                return Ok(args.pop().unwrap());
            }
            // if we don't have explicit parentheses, an empty tuple is
            // not a valid expression.
            if !explicit_parentheses {
                return self.fail(format!("Expected an expression, got {}", repr(&describe_token(&self.stream.current))), None);
            }
        }
        Ok(Expr::new(ExprKind::Tuple(args), lineno))
    }

    fn parse_list(&mut self) -> PResult<Expr> {
        let token = self.expect("lbracket")?;
        let mut items = vec![];
        while self.stream.current.ty != "rbracket" {
            if !items.is_empty() {
                self.expect("comma")?;
            }
            if self.stream.current.ty == "rbracket" {
                break;
            }
            items.push(self.parse_expression(true)?);
        }
        self.expect("rbracket")?;
        Ok(Expr::new(ExprKind::List(items), token.lineno))
    }

    fn parse_dict(&mut self) -> PResult<Expr> {
        let token = self.expect("lbrace")?;
        let mut items = vec![];
        while self.stream.current.ty != "rbrace" {
            if !items.is_empty() {
                self.expect("comma")?;
            }
            if self.stream.current.ty == "rbrace" {
                break;
            }
            let key = self.parse_expression(true)?;
            self.expect("colon")?;
            let value = self.parse_expression(true)?;
            items.push((key, value));
        }
        self.expect("rbrace")?;
        Ok(Expr::new(ExprKind::Dict(items), token.lineno))
    }

    fn parse_postfix(&mut self, mut node: Expr) -> PResult<Expr> {
        loop {
            let token_type = self.stream.current.ty;
            if token_type == "dot" || token_type == "lbracket" {
                node = self.parse_subscript(node)?;
            } else if token_type == "lparen" {
                // calls are valid both after postfix expressions (getattr
                // and getitem) as well as filters and tests
                node = self.parse_call(node)?;
            } else {
                break;
            }
        }
        Ok(node)
    }

    fn parse_filter_expr(&mut self, mut node: Expr) -> PResult<Expr> {
        loop {
            let token_type = self.stream.current.ty;
            if token_type == "pipe" {
                node = self.parse_filter(Some(node), false)?.expect("filter");
            } else if token_type == "name" && self.stream.current.str_value() == "is" {
                node = self.parse_test(node)?;
            } else if token_type == "lparen" {
                node = self.parse_call(node)?;
            } else {
                break;
            }
        }
        Ok(node)
    }

    fn parse_subscript(&mut self, node: Expr) -> PResult<Expr> {
        let token = self.stream.next();
        if token.ty == "dot" {
            let attr_token = self.stream.current.clone();
            self.stream.next();
            if attr_token.ty == "name" {
                return Ok(Expr::new(ExprKind::Getattr(Box::new(node), attr_token.str_value().to_string()), token.lineno));
            } else if attr_token.ty != "integer" {
                return self.fail("expected name or number", Some(attr_token.lineno));
            }
            let v = match attr_token.value {
                TokVal::Int(i) => Value::Int(i),
                _ => Value::None,
            };
            let arg = Expr::new(ExprKind::Const(v), attr_token.lineno);
            return Ok(Expr::new(ExprKind::Getitem(Box::new(node), Box::new(arg)), token.lineno));
        }
        if token.ty == "lbracket" {
            let mut args = vec![];
            while self.stream.current.ty != "rbracket" {
                if !args.is_empty() {
                    self.expect("comma")?;
                }
                args.push(self.parse_subscribed()?);
            }
            self.expect("rbracket")?;
            let arg = if args.len() == 1 { args.pop().unwrap() } else { Expr::new(ExprKind::Tuple(args), token.lineno) };
            return Ok(Expr::new(ExprKind::Getitem(Box::new(node), Box::new(arg)), token.lineno));
        }
        self.fail("expected subscript expression", Some(token.lineno))
    }

    fn parse_subscribed(&mut self) -> PResult<Expr> {
        let lineno = self.stream.current.lineno;
        let mut args: Vec<Option<Box<Expr>>>;
        if self.stream.current.ty == "colon" {
            self.stream.next();
            args = vec![None];
        } else {
            let node = self.parse_expression(true)?;
            if self.stream.current.ty != "colon" {
                return Ok(node);
            }
            self.stream.next();
            args = vec![Some(Box::new(node))];
        }
        if self.stream.current.ty == "colon" {
            args.push(None);
        } else if !matches!(self.stream.current.ty, "rbracket" | "comma") {
            args.push(Some(Box::new(self.parse_expression(true)?)));
        } else {
            args.push(None);
        }
        if self.stream.current.ty == "colon" {
            self.stream.next();
            if !matches!(self.stream.current.ty, "rbracket" | "comma") {
                args.push(Some(Box::new(self.parse_expression(true)?)));
            } else {
                args.push(None);
            }
        } else {
            args.push(None);
        }
        let step = args.pop().unwrap();
        let stop = args.pop().unwrap();
        let start = args.pop().unwrap();
        Ok(Expr::new(ExprKind::Slice(start, stop, step), lineno))
    }

    fn parse_call_args(&mut self) -> PResult<CallArgs> {
        let token = self.expect("lparen")?;
        let mut ca = CallArgs::default();
        let mut require_comma = false;
        let ensure = |p: &Parser, expr: bool| -> PResult<()> {
            if !expr {
                return p.fail("invalid syntax for function call expression", Some(token.lineno));
            }
            Ok(())
        };
        while self.stream.current.ty != "rparen" {
            if require_comma {
                self.expect("comma")?;
                // support for trailing comma
                if self.stream.current.ty == "rparen" {
                    break;
                }
            }
            if self.stream.current.ty == "mul" {
                ensure(self, ca.dyn_args.is_none() && ca.dyn_kwargs.is_none())?;
                self.stream.next();
                ca.dyn_args = Some(Box::new(self.parse_expression(true)?));
            } else if self.stream.current.ty == "pow" {
                ensure(self, ca.dyn_kwargs.is_none())?;
                self.stream.next();
                ca.dyn_kwargs = Some(Box::new(self.parse_expression(true)?));
            } else if self.stream.current.ty == "name" && self.stream.look().ty == "assign" {
                // Parsing a kwarg
                ensure(self, ca.dyn_kwargs.is_none())?;
                let key = self.stream.current.str_value().to_string();
                self.stream.skip(2);
                let value = self.parse_expression(true)?;
                ca.kwargs.push(Keyword { key, value });
            } else {
                // Parsing an arg
                ensure(self, ca.dyn_args.is_none() && ca.dyn_kwargs.is_none() && ca.kwargs.is_empty())?;
                ca.args.push(self.parse_expression(true)?);
            }
            require_comma = true;
        }
        self.expect("rparen")?;
        Ok(ca)
    }

    fn parse_call(&mut self, node: Expr) -> PResult<Expr> {
        // The lparen will be expected in parse_call_args, but the lineno
        // needs to be recorded before the stream is advanced.
        let lineno = self.stream.current.lineno;
        let ca = self.parse_call_args()?;
        Ok(Expr::new(ExprKind::Call(Box::new(node), ca), lineno))
    }

    fn parse_filter(&mut self, mut node: Option<Expr>, mut start_inline: bool) -> PResult<Option<Expr>> {
        while self.stream.current.ty == "pipe" || start_inline {
            if !start_inline {
                self.stream.next();
            }
            let token = self.expect("name")?;
            let mut name = token.str_value().to_string();
            while self.stream.current.ty == "dot" {
                self.stream.next();
                name.push('.');
                name.push_str(self.expect("name")?.str_value());
            }
            let ca = if self.stream.current.ty == "lparen" { self.parse_call_args()? } else { CallArgs::default() };
            node = Some(Expr::new(ExprKind::Filter(node.map(Box::new), name, ca), token.lineno));
            start_inline = false;
        }
        Ok(node)
    }

    fn parse_test(&mut self, node: Expr) -> PResult<Expr> {
        let token = self.stream.next();
        let negated = if self.stream.current.test("name:not") {
            self.stream.next();
            true
        } else {
            false
        };
        let mut name = self.expect("name")?.str_value().to_string();
        while self.stream.current.ty == "dot" {
            self.stream.next();
            name.push('.');
            name.push_str(self.expect("name")?.str_value());
        }
        let ca = if self.stream.current.ty == "lparen" {
            self.parse_call_args()?
        } else if matches!(self.stream.current.ty, "name" | "string" | "integer" | "float" | "lparen" | "lbracket" | "lbrace")
            && !self.stream.current.test_any(&["name:else", "name:or", "name:and"])
        {
            if self.stream.current.test("name:is") {
                return self.fail("You cannot chain multiple tests with is", None);
            }
            let arg_node = self.parse_primary(false)?;
            let arg_node = self.parse_postfix(arg_node)?;
            CallArgs { args: vec![arg_node], ..CallArgs::default() }
        } else {
            CallArgs::default()
        };
        let mut node = Expr::new(ExprKind::Test(Box::new(node), name, ca), token.lineno);
        if negated {
            node = Expr::new(ExprKind::Not(Box::new(node)), token.lineno);
        }
        Ok(node)
    }

    fn subparse(&mut self, end_tokens: Option<&[&'static str]>) -> PResult<Vec<Stmt>> {
        let mut body: Vec<Stmt> = vec![];
        let mut data_buffer: Vec<(usize, OutputNode)> = vec![];
        if let Some(e) = end_tokens {
            self.end_token_stack.push(e.to_vec());
        }
        let res = (|| -> PResult<()> {
            let flush_data = |body: &mut Vec<Stmt>, data_buffer: &mut Vec<(usize, OutputNode)>| {
                if !data_buffer.is_empty() {
                    let lineno = data_buffer[0].0;
                    let nodes = data_buffer.drain(..).map(|(_, n)| n).collect();
                    body.push(Stmt { lineno, kind: StmtKind::Output(nodes) });
                }
            };
            while !self.stream.is_eos() {
                let token = self.stream.current.clone();
                match token.ty {
                    "data" => {
                        if !token.str_value().is_empty() {
                            data_buffer.push((token.lineno, OutputNode::Data(token.str_value().to_string())));
                        }
                        self.stream.next();
                    }
                    "variable_begin" => {
                        self.stream.next();
                        let lineno = self.stream.current.lineno;
                        let e = self.parse_tuple(false, true, None, false, false)?;
                        data_buffer.push((lineno.min(e.lineno), OutputNode::Expr(e)));
                        self.expect("variable_end")?;
                    }
                    "block_begin" => {
                        flush_data(&mut body, &mut data_buffer);
                        self.stream.next();
                        if let Some(e) = end_tokens {
                            if self.stream.current.test_any(e) {
                                return Ok(());
                            }
                        }
                        let rv = self.parse_statement()?;
                        body.extend(rv);
                        self.expect("block_end")?;
                    }
                    _ => return self.fail("internal parsing error", None),
                }
            }
            flush_data(&mut body, &mut data_buffer);
            Ok(())
        })();
        if end_tokens.is_some() {
            self.end_token_stack.pop();
        }
        res?;
        Ok(body)
    }

    /// parse(): the template body
    pub fn parse(mut self) -> PResult<Vec<Stmt>> {
        self.subparse(None)
    }
}
