// Jinja2 template AST (the node classes of jinja2/nodes.py that the
// parser builds)
use super::value::Value;

#[derive(Clone, Debug)]
pub struct Keyword {
    pub key: String,
    pub value: Expr,
}

/// the arguments of a Call / Filter / Test node
#[derive(Clone, Debug, Default)]
pub struct CallArgs {
    pub args: Vec<Expr>,
    pub kwargs: Vec<Keyword>,
    pub dyn_args: Option<Box<Expr>>,
    pub dyn_kwargs: Option<Box<Expr>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BinOp {
    Add,
    Sub,
    Mul,
    Div,
    FloorDiv,
    Mod,
    Pow,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CmpOp {
    Eq,
    Ne,
    Lt,
    LtEq,
    Gt,
    GtEq,
    In,
    NotIn,
}

#[derive(Clone, Debug)]
pub enum ExprKind {
    Name(String),
    /// NSRef (a.b assignment target)
    NSRef(String, String),
    Const(Value),
    Tuple(Vec<Expr>),
    List(Vec<Expr>),
    Dict(Vec<(Expr, Expr)>),
    BinOp(BinOp, Box<Expr>, Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Or(Box<Expr>, Box<Expr>),
    Not(Box<Expr>),
    Neg(Box<Expr>),
    Pos(Box<Expr>),
    Concat(Vec<Expr>),
    Compare(Box<Expr>, Vec<(CmpOp, Expr)>),
    Getattr(Box<Expr>, String),
    Getitem(Box<Expr>, Box<Expr>),
    Slice(Option<Box<Expr>>, Option<Box<Expr>>, Option<Box<Expr>>),
    /// Filter(node, name, args); node None inside a filter block
    Filter(Option<Box<Expr>>, String, CallArgs),
    Test(Box<Expr>, String, CallArgs),
    CondExpr(Box<Expr>, Box<Expr>, Option<Box<Expr>>),
    Call(Box<Expr>, CallArgs),
}

#[derive(Clone, Debug)]
pub struct Expr {
    pub lineno: usize,
    pub kind: ExprKind,
}

impl Expr {
    pub fn new(kind: ExprKind, lineno: usize) -> Self {
        Expr { lineno, kind }
    }

    /// node.can_assign()
    pub fn can_assign(&self) -> bool {
        match &self.kind {
            ExprKind::Name(n) => !matches!(n.as_str(), "true" | "false" | "none" | "True" | "False" | "None"),
            ExprKind::NSRef(..) => true,
            ExprKind::Tuple(items) => items.iter().all(|i| i.can_assign()),
            _ => false,
        }
    }

    /// type(node).__name__.lower() for "can't assign to ..."
    pub fn type_name(&self) -> &'static str {
        match &self.kind {
            ExprKind::Name(_) => "name",
            ExprKind::NSRef(..) => "nsref",
            ExprKind::Const(_) => "const",
            ExprKind::Tuple(_) => "tuple",
            ExprKind::List(_) => "list",
            ExprKind::Dict(_) => "dict",
            ExprKind::BinOp(op, ..) => match op {
                BinOp::Add => "add",
                BinOp::Sub => "sub",
                BinOp::Mul => "mul",
                BinOp::Div => "div",
                BinOp::FloorDiv => "floordiv",
                BinOp::Mod => "mod",
                BinOp::Pow => "pow",
            },
            ExprKind::And(..) => "and",
            ExprKind::Or(..) => "or",
            ExprKind::Not(_) => "not",
            ExprKind::Neg(_) => "neg",
            ExprKind::Pos(_) => "pos",
            ExprKind::Concat(_) => "concat",
            ExprKind::Compare(..) => "compare",
            ExprKind::Getattr(..) => "getattr",
            ExprKind::Getitem(..) => "getitem",
            ExprKind::Slice(..) => "slice",
            ExprKind::Filter(..) => "filter",
            ExprKind::Test(..) => "test",
            ExprKind::CondExpr(..) => "condexpr",
            ExprKind::Call(..) => "call",
        }
    }

    /// every Name node with its context below this node ("store" targets
    /// are the assignment targets)
    pub fn names<'a>(&'a self, out: &mut Vec<&'a str>) {
        match &self.kind {
            ExprKind::Name(n) => out.push(n),
            ExprKind::Tuple(items) | ExprKind::List(items) | ExprKind::Concat(items) => {
                items.iter().for_each(|i| i.names(out))
            }
            _ => {}
        }
    }
}

#[derive(Clone, Debug)]
pub enum OutputNode {
    Data(String),
    Expr(Expr),
}

#[derive(Clone, Debug)]
pub struct MacroDef {
    pub name: String,
    pub args: Vec<String>,
    pub defaults: Vec<Expr>,
    pub body: Vec<Stmt>,
    pub lineno: usize,
}

#[derive(Clone, Debug)]
pub enum StmtKind {
    Output(Vec<OutputNode>),
    For {
        target: Expr,
        iter: Expr,
        body: Vec<Stmt>,
        else_: Vec<Stmt>,
        test: Option<Expr>,
        recursive: bool,
    },
    If {
        test: Expr,
        body: Vec<Stmt>,
        elif_: Vec<(Expr, Vec<Stmt>)>,
        else_: Vec<Stmt>,
    },
    Macro(std::rc::Rc<MacroDef>),
    CallBlock {
        call: Expr,
        def: std::rc::Rc<MacroDef>,
    },
    FilterBlock {
        filter: Expr,
        body: Vec<Stmt>,
    },
    With {
        targets: Vec<Expr>,
        values: Vec<Expr>,
        body: Vec<Stmt>,
    },
    Assign {
        target: Expr,
        node: Expr,
    },
    AssignBlock {
        target: Expr,
        filter: Option<Expr>,
        body: Vec<Stmt>,
    },
    ExprStmt(Expr),
    Break,
    Continue,
    /// {% autoescape %} (a scope; autoescaping has no effect on str output)
    Scope(Vec<Stmt>),
    Block {
        name: String,
        body: Vec<Stmt>,
    },
    Extends(Expr),
    Include {
        template: Expr,
        ignore_missing: bool,
    },
    Import {
        template: Expr,
        target: String,
    },
    FromImport {
        template: Expr,
        names: Vec<(String, Option<String>)>,
    },
}

#[derive(Clone, Debug)]
pub struct Stmt {
    pub lineno: usize,
    pub kind: StmtKind,
}
