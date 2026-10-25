// A port of Jinja2 3.1.6 (the template engine klippy's gcode_macro uses),
// with python object semantics for the template values.
//
// What is ported: the lexer, parser and the behaviour of the code the
// compiler generates (run directly on the AST), all builtin filters and
// tests, the "do" and "loopcontrols" extensions, the default globals.
// Templates have no loader (klippy never configures one), autoescaping
// is off (klippy's environment).
pub mod eval;
pub mod filters;
pub mod lexer;
pub mod methods;
pub mod nodes;
pub mod parser;
pub mod pymath;
pub mod runtime;
pub mod strformat;
pub mod tests;
pub mod value;

use crate::IndexMap;
use nodes::*;
use std::rc::Rc;
pub use value::{Exc, Value};

/// jinja2.exceptions.TemplateSyntaxError (TemplateAssertionError when
/// `assertion`)
#[derive(Clone, Debug)]
pub struct TemplateSyntaxError {
    pub message: String,
    pub lineno: usize,
    pub name: Option<String>,
    pub assertion: bool,
}

impl TemplateSyntaxError {
    pub fn new(message: impl Into<String>, lineno: usize, name: Option<String>) -> Self {
        TemplateSyntaxError { message: message.into(), lineno, name, assertion: false }
    }
}

/// jinja2.Environment(block_start, block_end, variable_start, variable_end,
/// extensions=[do, loopcontrols])
pub struct Environment {
    delimiters: lexer::Delimiters,
}

pub struct Template {
    pub name: String,
    body: Rc<Vec<Stmt>>,
}

impl Environment {
    pub fn new(block_start: &str, block_end: &str, variable_start: &str, variable_end: &str) -> Self {
        Environment {
            delimiters: lexer::Delimiters {
                block_start: block_start.into(),
                block_end: block_end.into(),
                variable_start: variable_start.into(),
                variable_end: variable_end.into(),
                comment_start: "{#".into(),
                comment_end: "#}".into(),
            },
        }
    }

    /// env.from_string(source)
    pub fn from_string(&self, source: &str) -> Result<Template, TemplateSyntaxError> {
        let tokens = lexer::tokenize(source, &self.delimiters, None)?;
        let body = parser::Parser::new(tokens, None).parse()?;
        check_body(&body, false)?;
        Ok(Template { name: String::new(), body: Rc::new(body) })
    }
}

/// the compile time checks of jinja2's code generator: unknown filters and
/// tests outside if-conditions, assignments to "loop" in a for target
fn check_body(body: &[Stmt], soft: bool) -> Result<(), TemplateSyntaxError> {
    for s in body {
        check_stmt(s, soft)?;
    }
    Ok(())
}

fn assertion(msg: String, lineno: usize) -> TemplateSyntaxError {
    TemplateSyntaxError { message: msg, lineno, name: None, assertion: true }
}

fn check_stmt(s: &Stmt, soft: bool) -> Result<(), TemplateSyntaxError> {
    match &s.kind {
        StmtKind::Output(nodes) => {
            for n in nodes {
                if let OutputNode::Expr(e) = n {
                    check_expr(e, soft)?;
                }
            }
        }
        StmtKind::For { target, iter, body, else_, test, .. } => {
            check_expr(iter, soft)?;
            let mut names = vec![];
            target.names(&mut names);
            if names.contains(&"loop") {
                return Err(assertion("Can't assign to special loop variable in for-loop target".into(), target.lineno));
            }
            if let Some(t) = test {
                check_expr(t, false)?;
            }
            check_body(body, false)?;
            check_body(else_, false)?;
        }
        StmtKind::If { test, body, elif_, else_ } => {
            check_expr(test, true)?;
            check_body(body, true)?;
            for (t, b) in elif_ {
                check_expr(t, true)?;
                check_body(b, true)?;
            }
            check_body(else_, true)?;
        }
        StmtKind::Macro(def) | StmtKind::CallBlock { def, .. } => {
            for d in &def.defaults {
                check_expr(d, false)?;
            }
            check_body(&def.body, false)?;
            if let StmtKind::CallBlock { call, .. } = &s.kind {
                check_expr(call, soft)?;
            }
        }
        StmtKind::FilterBlock { filter, body } => {
            check_body(body, false)?;
            check_expr(filter, false)?;
        }
        StmtKind::With { targets: _, values, body } => {
            for v in values {
                check_expr(v, soft)?;
            }
            check_body(body, false)?;
        }
        StmtKind::Assign { node, .. } => check_expr(node, soft)?,
        StmtKind::AssignBlock { filter, body, .. } => {
            check_body(body, false)?;
            if let Some(f) = filter {
                check_expr(f, false)?;
            }
        }
        StmtKind::ExprStmt(e) => check_expr(e, soft)?,
        StmtKind::Scope(b) | StmtKind::Block { body: b, .. } => check_body(b, false)?,
        StmtKind::Extends(e) | StmtKind::Include { template: e, .. } | StmtKind::Import { template: e, .. } => check_expr(e, soft)?,
        StmtKind::FromImport { template, .. } => check_expr(template, soft)?,
        StmtKind::Break | StmtKind::Continue => {}
    }
    Ok(())
}

fn check_args(ca: &CallArgs, soft: bool) -> Result<(), TemplateSyntaxError> {
    for a in &ca.args {
        check_expr(a, soft)?;
    }
    for k in &ca.kwargs {
        check_expr(&k.value, soft)?;
    }
    if let Some(d) = &ca.dyn_args {
        check_expr(d, soft)?;
    }
    if let Some(d) = &ca.dyn_kwargs {
        check_expr(d, soft)?;
    }
    Ok(())
}

fn check_expr(e: &Expr, soft: bool) -> Result<(), TemplateSyntaxError> {
    match &e.kind {
        ExprKind::Filter(node, name, ca) => {
            if !soft && !filters::FILTER_NAMES.contains(&name.as_str()) {
                return Err(assertion(format!("No filter named {}.", crate::pyfmt::py_str_repr(name)), e.lineno));
            }
            if let Some(n) = node {
                check_expr(n, soft)?;
            }
            check_args(ca, soft)
        }
        ExprKind::Test(node, name, ca) => {
            if !soft && !tests::TEST_NAMES.contains(&name.as_str()) {
                return Err(assertion(format!("No test named {}.", crate::pyfmt::py_str_repr(name)), e.lineno));
            }
            check_expr(node, soft)?;
            check_args(ca, soft)
        }
        ExprKind::CondExpr(test, a, b) => {
            check_expr(test, true)?;
            check_expr(a, true)?;
            if let Some(b) = b {
                check_expr(b, true)?;
            }
            Ok(())
        }
        ExprKind::Tuple(v) | ExprKind::List(v) | ExprKind::Concat(v) => v.iter().try_for_each(|x| check_expr(x, soft)),
        ExprKind::Dict(v) => v.iter().try_for_each(|(k, x)| check_expr(k, soft).and_then(|_| check_expr(x, soft))),
        ExprKind::BinOp(_, a, b) | ExprKind::And(a, b) | ExprKind::Or(a, b) | ExprKind::Getitem(a, b) => {
            check_expr(a, soft)?;
            check_expr(b, soft)
        }
        ExprKind::Not(a) | ExprKind::Neg(a) | ExprKind::Pos(a) | ExprKind::Getattr(a, _) => check_expr(a, soft),
        ExprKind::Compare(a, ops) => {
            check_expr(a, soft)?;
            ops.iter().try_for_each(|(_, x)| check_expr(x, soft))
        }
        ExprKind::Slice(a, b, c) => {
            for x in [a, b, c].into_iter().flatten() {
                check_expr(x, soft)?;
            }
            Ok(())
        }
        ExprKind::Call(f, ca) => {
            check_expr(f, soft)?;
            check_args(ca, soft)
        }
        ExprKind::Name(_) | ExprKind::NSRef(..) | ExprKind::Const(_) => Ok(()),
    }
}

impl Template {
    /// template.render(context): the environment globals under the context
    pub fn render(&self, context: IndexMap<String, Value>) -> Result<String, Exc> {
        let mut ctx = runtime::default_namespace();
        ctx.extend(context);
        let mut r = eval::Renderer::new(ctx);
        r.render(&self.body)
    }
}
