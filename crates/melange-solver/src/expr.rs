//! Behavioral-source expression engine.
//!
//! Parses the scalar expression inside a SPICE3 `B`-source
//! (`B... V={expr}` / `I={expr}`) into an AST, supports symbolic
//! differentiation w.r.t. node voltages and branch currents (for the
//! Newton-Raphson Jacobian), an `f64` interpreter (used by the DC operating
//! point solver and unit tests), and a straight-line Rust emitter (used by
//! codegen — no AST interpreter runs in the audio thread).
//!
//! Supported surface (see `docs/aidocs/BEHAVIORAL_SOURCES.md`):
//!   - leaves: numeric literals, `V(node)`, `V(a,b)`, `I(elem)`, `time`
//!   - operators: `+ - * / ^`, unary `-`, parentheses
//!   - functions: `atan2 sqrt abs exp ln sin cos tanh min max pow`
//!   - time operators: `ddt(x)` (derivative), `idt(x)` (integral)
//!
//! ## ddt / idt
//!
//! `ddt`/`idt` are evaluated with the companion-model history the rest of the
//! solver already uses. For a sub-expression value `x` at sample `n`:
//!   - `ddt(x) = (x[n] - x[n-1]) / dt`  (backward difference — see the module
//!     docs for why this, not a smooth trapezoidal companion, is the correct
//!     choice for `atan2`-phase discriminators: it preserves the 2π-wrap
//!     impulse that *is* the FM click).
//!   - `idt(x) = idt[n-1] + (dt/2)·(x[n] + x[n-1])`  (trapezoidal accumulator).
//!
//! Their partials w.r.t. the present node voltages are `INV_DT·∂x/∂V` and
//! `HALF_DT·∂x/∂V` respectively, which is why differentiation introduces the
//! [`Expr::InvDt`] / [`Expr::HalfDt`] scaling leaves.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

/// A differentiation variable: either a node voltage or a branch current.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Var {
    /// Node voltage `V(name)`.
    Node(String),
    /// Branch current `I(name)`.
    Branch(String),
}

/// Single-argument intrinsic functions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum UnaryFn {
    Sqrt,
    Abs,
    Exp,
    Ln,
    Sin,
    Cos,
    Tanh,
}

/// Two-argument intrinsic functions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BinFn {
    Atan2,
    Min,
    Max,
    Pow,
}

/// Behavioral-source expression AST.
///
/// Node/branch leaves carry *names* (not resolved indices) so the same AST can
/// be reused across subcircuit remapping; resolution to node indices happens at
/// emission time via an [`ExprResolver`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Expr {
    /// Numeric literal.
    Const(f64),
    /// Simulation time in seconds.
    Time,
    /// `1/dt` scaling leaf, produced only by differentiating [`Expr::Ddt`].
    InvDt,
    /// `dt/2` scaling leaf, produced only by differentiating [`Expr::Idt`].
    HalfDt,
    /// `V(node)` — node voltage referenced to ground.
    NodeV(String),
    /// `V(a,b)` — node-pair voltage difference `V(a) - V(b)`.
    NodeVDiff(String, String),
    /// `I(elem)` — branch current through a named element.
    BranchI(String),
    /// A named scalar parameter — a `.pot` / `.runtime` value or a `.param`
    /// constant referenced as a bare identifier (e.g. `strength`, `f_offset`).
    /// Resolved at codegen time; treated as a constant by differentiation (NR
    /// differentiates only w.r.t. node voltages / branch currents).
    Param(String),
    Neg(Box<Expr>),
    Add(Box<Expr>, Box<Expr>),
    Sub(Box<Expr>, Box<Expr>),
    Mul(Box<Expr>, Box<Expr>),
    Div(Box<Expr>, Box<Expr>),
    Func1(UnaryFn, Box<Expr>),
    Func2(BinFn, Box<Expr>, Box<Expr>),
    /// `ddt(inner)` — time derivative. `usize` is the global companion-state
    /// slot id, assigned by [`Expr::assign_state_slots`].
    Ddt(usize, Box<Expr>),
    /// `idt(inner)` — time integral. `usize` is the global companion-state slot
    /// id, assigned by [`Expr::assign_state_slots`].
    Idt(usize, Box<Expr>),
}

/// Parse error with a human-readable message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExprError(pub String);

impl fmt::Display for ExprError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "behavioral expression error: {}", self.0)
    }
}

impl std::error::Error for ExprError {}

// ---------------------------------------------------------------------------
// Tokenizer
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Num(f64),
    Ident(String),
    Plus,
    Minus,
    Star,
    Slash,
    Caret,
    LParen,
    RParen,
    Comma,
}

fn tokenize(s: &str) -> Result<Vec<Tok>, ExprError> {
    let chars: Vec<char> = s.chars().collect();
    let mut toks = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        match c {
            '+' => {
                toks.push(Tok::Plus);
                i += 1;
            }
            '-' => {
                toks.push(Tok::Minus);
                i += 1;
            }
            '*' => {
                // accept '**' as power alias
                if i + 1 < chars.len() && chars[i + 1] == '*' {
                    toks.push(Tok::Caret);
                    i += 2;
                } else {
                    toks.push(Tok::Star);
                    i += 1;
                }
            }
            '/' => {
                toks.push(Tok::Slash);
                i += 1;
            }
            '^' => {
                toks.push(Tok::Caret);
                i += 1;
            }
            '(' => {
                toks.push(Tok::LParen);
                i += 1;
            }
            ')' => {
                toks.push(Tok::RParen);
                i += 1;
            }
            ',' => {
                toks.push(Tok::Comma);
                i += 1;
            }
            _ if c.is_ascii_digit() || c == '.' => {
                let start = i;
                let mut seen_e = false;
                while i < chars.len() {
                    let d = chars[i];
                    if d.is_ascii_digit() || d == '.' {
                        i += 1;
                    } else if (d == 'e' || d == 'E') && !seen_e {
                        seen_e = true;
                        i += 1;
                        // optional sign in exponent
                        if i < chars.len() && (chars[i] == '+' || chars[i] == '-') {
                            i += 1;
                        }
                    } else {
                        break;
                    }
                }
                let num: String = chars[start..i].iter().collect();
                let v: f64 = num
                    .parse()
                    .map_err(|_| ExprError(format!("invalid number `{num}`")))?;
                toks.push(Tok::Num(v));
            }
            _ if c.is_ascii_alphabetic() || c == '_' => {
                let start = i;
                while i < chars.len()
                    && (chars[i].is_ascii_alphanumeric() || chars[i] == '_')
                {
                    i += 1;
                }
                let ident: String = chars[start..i].iter().collect();
                toks.push(Tok::Ident(ident));
            }
            _ => return Err(ExprError(format!("unexpected character `{c}`"))),
        }
    }
    Ok(toks)
}

// ---------------------------------------------------------------------------
// Parser (recursive descent / precedence climbing)
// ---------------------------------------------------------------------------

struct Parser {
    toks: Vec<Tok>,
    pos: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }

    fn next(&mut self) -> Option<Tok> {
        let t = self.toks.get(self.pos).cloned();
        if t.is_some() {
            self.pos += 1;
        }
        t
    }

    fn expect(&mut self, t: &Tok) -> Result<(), ExprError> {
        match self.next() {
            Some(ref got) if got == t => Ok(()),
            other => Err(ExprError(format!("expected {t:?}, found {other:?}"))),
        }
    }

    /// expr := term (('+' | '-') term)*
    fn parse_expr(&mut self) -> Result<Expr, ExprError> {
        let mut lhs = self.parse_term()?;
        while let Some(t) = self.peek() {
            match t {
                Tok::Plus => {
                    self.next();
                    let rhs = self.parse_term()?;
                    lhs = Expr::Add(Box::new(lhs), Box::new(rhs));
                }
                Tok::Minus => {
                    self.next();
                    let rhs = self.parse_term()?;
                    lhs = Expr::Sub(Box::new(lhs), Box::new(rhs));
                }
                _ => break,
            }
        }
        Ok(lhs)
    }

    /// term := factor (('*' | '/') factor)*
    fn parse_term(&mut self) -> Result<Expr, ExprError> {
        let mut lhs = self.parse_factor()?;
        while let Some(t) = self.peek() {
            match t {
                Tok::Star => {
                    self.next();
                    let rhs = self.parse_factor()?;
                    lhs = Expr::Mul(Box::new(lhs), Box::new(rhs));
                }
                Tok::Slash => {
                    self.next();
                    let rhs = self.parse_factor()?;
                    lhs = Expr::Div(Box::new(lhs), Box::new(rhs));
                }
                _ => break,
            }
        }
        Ok(lhs)
    }

    /// factor := unary ('^' factor)?    (right-associative power)
    fn parse_factor(&mut self) -> Result<Expr, ExprError> {
        let base = self.parse_unary()?;
        if let Some(Tok::Caret) = self.peek() {
            self.next();
            let exp = self.parse_factor()?;
            Ok(Expr::Func2(BinFn::Pow, Box::new(base), Box::new(exp)))
        } else {
            Ok(base)
        }
    }

    /// unary := '-' unary | '+' unary | atom
    fn parse_unary(&mut self) -> Result<Expr, ExprError> {
        match self.peek() {
            Some(Tok::Minus) => {
                self.next();
                Ok(Expr::Neg(Box::new(self.parse_unary()?)))
            }
            Some(Tok::Plus) => {
                self.next();
                self.parse_unary()
            }
            _ => self.parse_atom(),
        }
    }

    /// atom := number | '(' expr ')' | ident | call
    fn parse_atom(&mut self) -> Result<Expr, ExprError> {
        match self.next() {
            Some(Tok::Num(v)) => Ok(Expr::Const(v)),
            Some(Tok::LParen) => {
                let e = self.parse_expr()?;
                self.expect(&Tok::RParen)?;
                Ok(e)
            }
            Some(Tok::Ident(name)) => {
                if let Some(Tok::LParen) = self.peek() {
                    self.parse_call(&name)
                } else {
                    // bare identifier: `time`/`pi` are built-ins; anything else
                    // is a named scalar parameter (.pot / .runtime / .param),
                    // resolved and validated at codegen time.
                    match name.to_ascii_lowercase().as_str() {
                        "time" => Ok(Expr::Time),
                        "pi" => Ok(Expr::Const(std::f64::consts::PI)),
                        _ => Ok(Expr::Param(name)),
                    }
                }
            }
            other => Err(ExprError(format!("unexpected token {other:?}"))),
        }
    }

    fn parse_call(&mut self, name: &str) -> Result<Expr, ExprError> {
        let lname = name.to_ascii_lowercase();

        // V(node) / V(a,b) and I(elem) take node/element *names* (bare
        // identifiers or integer node numbers), not sub-expressions — so read
        // their raw argument tokens as names before the general arg parser
        // (which would reject a bare identifier).
        if lname == "v" || lname == "i" {
            self.expect(&Tok::LParen)?;
            let names = self.parse_name_args()?;
            self.expect(&Tok::RParen)?;
            return match (lname.as_str(), names.len()) {
                ("v", 1) => Ok(Expr::NodeV(names[0].clone())),
                ("v", 2) => Ok(Expr::NodeVDiff(names[0].clone(), names[1].clone())),
                ("v", n) => Err(ExprError(format!("V() takes 1 or 2 args, got {n}"))),
                ("i", 1) => Ok(Expr::BranchI(names[0].clone())),
                ("i", n) => Err(ExprError(format!("I() takes 1 arg, got {n}"))),
                _ => unreachable!(),
            };
        }

        self.expect(&Tok::LParen)?;
        let args = self.parse_args()?;
        self.expect(&Tok::RParen)?;

        match lname.as_str() {
            "ddt" => one_arg(lname.as_str(), args).map(|a| Expr::Ddt(usize::MAX, Box::new(a))),
            "idt" => one_arg(lname.as_str(), args).map(|a| Expr::Idt(usize::MAX, Box::new(a))),
            "sqrt" => one_arg(lname.as_str(), args).map(|a| Expr::Func1(UnaryFn::Sqrt, Box::new(a))),
            "abs" => one_arg(lname.as_str(), args).map(|a| Expr::Func1(UnaryFn::Abs, Box::new(a))),
            "exp" => one_arg(lname.as_str(), args).map(|a| Expr::Func1(UnaryFn::Exp, Box::new(a))),
            "ln" | "log" => {
                one_arg(lname.as_str(), args).map(|a| Expr::Func1(UnaryFn::Ln, Box::new(a)))
            }
            "sin" => one_arg(lname.as_str(), args).map(|a| Expr::Func1(UnaryFn::Sin, Box::new(a))),
            "cos" => one_arg(lname.as_str(), args).map(|a| Expr::Func1(UnaryFn::Cos, Box::new(a))),
            "tanh" => one_arg(lname.as_str(), args).map(|a| Expr::Func1(UnaryFn::Tanh, Box::new(a))),
            "atan2" => {
                two_args(lname.as_str(), args).map(|(a, b)| Expr::Func2(BinFn::Atan2, Box::new(a), Box::new(b)))
            }
            "min" => {
                two_args(lname.as_str(), args).map(|(a, b)| Expr::Func2(BinFn::Min, Box::new(a), Box::new(b)))
            }
            "max" => {
                two_args(lname.as_str(), args).map(|(a, b)| Expr::Func2(BinFn::Max, Box::new(a), Box::new(b)))
            }
            "pow" => {
                two_args(lname.as_str(), args).map(|(a, b)| Expr::Func2(BinFn::Pow, Box::new(a), Box::new(b)))
            }
            other => Err(ExprError(format!("unknown function `{other}`"))),
        }
    }

    fn parse_args(&mut self) -> Result<Vec<Expr>, ExprError> {
        let mut args = Vec::new();
        if let Some(Tok::RParen) = self.peek() {
            return Ok(args);
        }
        loop {
            args.push(self.parse_expr()?);
            match self.peek() {
                Some(Tok::Comma) => {
                    self.next();
                }
                _ => break,
            }
        }
        Ok(args)
    }

    /// Parse comma-separated node/element *names* (the args of `V()` / `I()`).
    /// A name is a bare identifier or a non-negative integer (node number).
    fn parse_name_args(&mut self) -> Result<Vec<String>, ExprError> {
        let mut names = Vec::new();
        loop {
            match self.next() {
                Some(Tok::Ident(s)) => names.push(s),
                Some(Tok::Num(v)) if v.fract() == 0.0 && v >= 0.0 => {
                    names.push(format!("{}", v as i64))
                }
                other => {
                    return Err(ExprError(format!(
                        "expected a node/element name, found {other:?}"
                    )))
                }
            }
            match self.peek() {
                Some(Tok::Comma) => {
                    self.next();
                }
                _ => break,
            }
        }
        Ok(names)
    }
}

fn one_arg(name: &str, mut args: Vec<Expr>) -> Result<Expr, ExprError> {
    if args.len() != 1 {
        return Err(ExprError(format!(
            "{name}() takes 1 arg, got {}",
            args.len()
        )));
    }
    Ok(args.pop().unwrap())
}

fn two_args(name: &str, mut args: Vec<Expr>) -> Result<(Expr, Expr), ExprError> {
    if args.len() != 2 {
        return Err(ExprError(format!(
            "{name}() takes 2 args, got {}",
            args.len()
        )));
    }
    let b = args.pop().unwrap();
    let a = args.pop().unwrap();
    Ok((a, b))
}

impl Expr {
    /// Parse a behavioral-source expression string into an AST.
    ///
    /// `ddt`/`idt` nodes are created with placeholder slot ids; call
    /// [`Expr::assign_state_slots`] before emission/eval.
    pub fn parse(s: &str) -> Result<Expr, ExprError> {
        let toks = tokenize(s)?;
        if toks.is_empty() {
            return Err(ExprError("empty expression".to_string()));
        }
        let mut p = Parser { toks, pos: 0 };
        let e = p.parse_expr()?;
        if p.pos != p.toks.len() {
            return Err(ExprError(format!(
                "trailing tokens after expression (at token {})",
                p.pos
            )));
        }
        Ok(e)
    }
}

// ---------------------------------------------------------------------------
// State-slot assignment for ddt/idt
// ---------------------------------------------------------------------------

impl Expr {
    /// Assign a unique companion-state slot id to each `ddt`/`idt` node in
    /// post-order, starting from `*next` and advancing it. Slot ids are global
    /// across all behavioral sources so each gets its own history field.
    pub fn assign_state_slots(&mut self, next: &mut usize) {
        match self {
            Expr::Neg(a) | Expr::Func1(_, a) => a.assign_state_slots(next),
            Expr::Add(a, b)
            | Expr::Sub(a, b)
            | Expr::Mul(a, b)
            | Expr::Div(a, b)
            | Expr::Func2(_, a, b) => {
                a.assign_state_slots(next);
                b.assign_state_slots(next);
            }
            Expr::Ddt(slot, a) | Expr::Idt(slot, a) => {
                a.assign_state_slots(next);
                *slot = *next;
                *next += 1;
            }
            _ => {}
        }
    }

    /// Number of `ddt`/`idt` companion-state slots in this expression.
    pub fn state_slot_count(&self) -> usize {
        let mut n = 0;
        self.walk(&mut |e| {
            if matches!(e, Expr::Ddt(..) | Expr::Idt(..)) {
                n += 1;
            }
        });
        n
    }

    /// `true` if the expression contains any `ddt`/`idt`/`time` node (i.e. it is
    /// time-dependent and participates in the integrator).
    pub fn is_time_dependent(&self) -> bool {
        let mut found = false;
        self.walk(&mut |e| {
            if matches!(e, Expr::Ddt(..) | Expr::Idt(..) | Expr::Time) {
                found = true;
            }
        });
        found
    }

    /// Collect every `ddt`/`idt` node as `(slot, is_idt, inner_expr)`, for the
    /// codegen post-convergence companion-state update.
    pub fn collect_time_ops(&self) -> Vec<(usize, bool, &Expr)> {
        let mut out = Vec::new();
        self.collect_time_ops_into(&mut out);
        out
    }

    fn collect_time_ops_into<'a>(&'a self, out: &mut Vec<(usize, bool, &'a Expr)>) {
        match self {
            Expr::Ddt(slot, a) => {
                out.push((*slot, false, a));
                a.collect_time_ops_into(out);
            }
            Expr::Idt(slot, a) => {
                out.push((*slot, true, a));
                a.collect_time_ops_into(out);
            }
            Expr::Neg(a) | Expr::Func1(_, a) => a.collect_time_ops_into(out),
            Expr::Add(a, b)
            | Expr::Sub(a, b)
            | Expr::Mul(a, b)
            | Expr::Div(a, b)
            | Expr::Func2(_, a, b) => {
                a.collect_time_ops_into(out);
                b.collect_time_ops_into(out);
            }
            _ => {}
        }
    }

    fn walk(&self, f: &mut impl FnMut(&Expr)) {
        f(self);
        match self {
            Expr::Neg(a) | Expr::Func1(_, a) | Expr::Ddt(_, a) | Expr::Idt(_, a) => a.walk(f),
            Expr::Add(a, b)
            | Expr::Sub(a, b)
            | Expr::Mul(a, b)
            | Expr::Div(a, b)
            | Expr::Func2(_, a, b) => {
                a.walk(f);
                b.walk(f);
            }
            _ => {}
        }
    }

    /// Rewrite every `V(node)` / `V(a,b)` / `I(elem)` identifier through
    /// `remap`. Used by subcircuit expansion so expression-referenced nodes
    /// track the same name remapping as the source's terminals.
    pub fn remap_idents(&self, remap: &dyn Fn(&str) -> String) -> Expr {
        use Expr::*;
        match self {
            NodeV(n) => NodeV(remap(n)),
            NodeVDiff(a, b) => NodeVDiff(remap(a), remap(b)),
            BranchI(n) => BranchI(remap(n)),
            Neg(a) => Neg(Box::new(a.remap_idents(remap))),
            Add(a, b) => Add(
                Box::new(a.remap_idents(remap)),
                Box::new(b.remap_idents(remap)),
            ),
            Sub(a, b) => Sub(
                Box::new(a.remap_idents(remap)),
                Box::new(b.remap_idents(remap)),
            ),
            Mul(a, b) => Mul(
                Box::new(a.remap_idents(remap)),
                Box::new(b.remap_idents(remap)),
            ),
            Div(a, b) => Div(
                Box::new(a.remap_idents(remap)),
                Box::new(b.remap_idents(remap)),
            ),
            Func1(f, a) => Func1(*f, Box::new(a.remap_idents(remap))),
            Func2(f, a, b) => Func2(
                *f,
                Box::new(a.remap_idents(remap)),
                Box::new(b.remap_idents(remap)),
            ),
            Ddt(s, a) => Ddt(*s, Box::new(a.remap_idents(remap))),
            Idt(s, a) => Idt(*s, Box::new(a.remap_idents(remap))),
            other => other.clone(),
        }
    }

    /// Collect every node name referenced by `V(node)` / `V(a,b)`.
    pub fn referenced_nodes(&self) -> BTreeSet<String> {
        let mut set = BTreeSet::new();
        self.walk(&mut |e| match e {
            Expr::NodeV(n) => {
                set.insert(n.clone());
            }
            Expr::NodeVDiff(a, b) => {
                set.insert(a.clone());
                set.insert(b.clone());
            }
            _ => {}
        });
        set
    }

    /// Borrowed references to every node name in `V(node)` / `V(a,b)` (with
    /// duplicates), for callers that need element-lifetime `&String`s (e.g. MNA
    /// node collection). Use [`Expr::referenced_nodes`] for a deduped owned set.
    pub fn referenced_node_refs(&self) -> Vec<&String> {
        let mut out = Vec::new();
        self.collect_node_refs(&mut out);
        out
    }

    fn collect_node_refs<'a>(&'a self, out: &mut Vec<&'a String>) {
        match self {
            Expr::NodeV(n) => out.push(n),
            Expr::NodeVDiff(a, b) => {
                out.push(a);
                out.push(b);
            }
            Expr::Neg(a) | Expr::Func1(_, a) | Expr::Ddt(_, a) | Expr::Idt(_, a) => {
                a.collect_node_refs(out)
            }
            Expr::Add(a, b)
            | Expr::Sub(a, b)
            | Expr::Mul(a, b)
            | Expr::Div(a, b)
            | Expr::Func2(_, a, b) => {
                a.collect_node_refs(out);
                b.collect_node_refs(out);
            }
            _ => {}
        }
    }

    /// Collect every element name referenced by `I(elem)`.
    pub fn referenced_branches(&self) -> BTreeSet<String> {
        let mut set = BTreeSet::new();
        self.walk(&mut |e| {
            if let Expr::BranchI(n) = e {
                set.insert(n.clone());
            }
        });
        set
    }

    /// Collect every named scalar parameter referenced as a bare identifier.
    pub fn referenced_params(&self) -> BTreeSet<String> {
        let mut set = BTreeSet::new();
        self.walk(&mut |e| {
            if let Expr::Param(n) = e {
                set.insert(n.clone());
            }
        });
        set
    }

    /// The set of differentiation variables this expression depends on.
    pub fn variables(&self) -> Vec<Var> {
        let mut vars: Vec<Var> = self
            .referenced_nodes()
            .into_iter()
            .map(Var::Node)
            .collect();
        vars.extend(self.referenced_branches().into_iter().map(Var::Branch));
        vars
    }
}

// ---------------------------------------------------------------------------
// Symbolic differentiation
// ---------------------------------------------------------------------------

impl Expr {
    /// Symbolic partial derivative `∂self/∂var`. The result is **not**
    /// simplified beyond trivial constant folding done by [`Expr::simplify`];
    /// call `.simplify()` on the result for compact codegen.
    pub fn diff(&self, var: &Var) -> Expr {
        use Expr::*;
        match self {
            Const(_) | Time | InvDt | HalfDt | Param(_) => Const(0.0),
            NodeV(n) => match var {
                Var::Node(v) if v == n => Const(1.0),
                _ => Const(0.0),
            },
            NodeVDiff(a, b) => match var {
                Var::Node(v) if v == a => Const(1.0),
                Var::Node(v) if v == b => Const(-1.0),
                _ => Const(0.0),
            },
            BranchI(n) => match var {
                Var::Branch(v) if v == n => Const(1.0),
                _ => Const(0.0),
            },
            Neg(a) => Neg(Box::new(a.diff(var))),
            Add(a, b) => Add(Box::new(a.diff(var)), Box::new(b.diff(var))),
            Sub(a, b) => Sub(Box::new(a.diff(var)), Box::new(b.diff(var))),
            Mul(a, b) => {
                // (a*b)' = a'*b + a*b'
                Add(
                    Box::new(Mul(Box::new(a.diff(var)), b.clone())),
                    Box::new(Mul(a.clone(), Box::new(b.diff(var)))),
                )
            }
            Div(a, b) => {
                // (a/b)' = (a'*b - a*b') / b^2
                let num = Sub(
                    Box::new(Mul(Box::new(a.diff(var)), b.clone())),
                    Box::new(Mul(a.clone(), Box::new(b.diff(var)))),
                );
                let den = Mul(b.clone(), b.clone());
                Div(Box::new(num), Box::new(den))
            }
            Func1(f, a) => {
                let da = a.diff(var);
                let outer = func1_derivative(*f, a);
                Mul(Box::new(outer), Box::new(da))
            }
            Func2(f, a, b) => func2_derivative(*f, a, b, var),
            Ddt(_, a) => Mul(Box::new(InvDt), Box::new(a.diff(var))),
            Idt(_, a) => Mul(Box::new(HalfDt), Box::new(a.diff(var))),
        }
    }
}

/// Outer derivative `f'(a)` (not yet multiplied by `a'`).
fn func1_derivative(f: UnaryFn, a: &Expr) -> Expr {
    use Expr::*;
    match f {
        // d/dx sqrt(x) = 1/(2*sqrt(x))
        UnaryFn::Sqrt => Div(
            Box::new(Const(0.5)),
            Box::new(Func1(UnaryFn::Sqrt, Box::new(a.clone()))),
        ),
        // d/dx abs(x) = sign(x); approximate with x/abs(x) (subgradient, guarded
        // at codegen via a small epsilon — matches diode/limiter handling).
        UnaryFn::Abs => Div(Box::new(a.clone()), Box::new(Func1(UnaryFn::Abs, Box::new(a.clone())))),
        // d/dx exp(x) = exp(x)
        UnaryFn::Exp => Func1(UnaryFn::Exp, Box::new(a.clone())),
        // d/dx ln(x) = 1/x
        UnaryFn::Ln => Div(Box::new(Const(1.0)), Box::new(a.clone())),
        // d/dx sin(x) = cos(x)
        UnaryFn::Sin => Func1(UnaryFn::Cos, Box::new(a.clone())),
        // d/dx cos(x) = -sin(x)
        UnaryFn::Cos => Neg(Box::new(Func1(UnaryFn::Sin, Box::new(a.clone())))),
        // d/dx tanh(x) = 1 - tanh(x)^2
        UnaryFn::Tanh => Sub(
            Box::new(Const(1.0)),
            Box::new(Func2(
                BinFn::Pow,
                Box::new(Func1(UnaryFn::Tanh, Box::new(a.clone()))),
                Box::new(Const(2.0)),
            )),
        ),
    }
}

fn func2_derivative(f: BinFn, a: &Expr, b: &Expr, var: &Var) -> Expr {
    use Expr::*;
    let da = a.diff(var);
    let db = b.diff(var);
    match f {
        // d atan2(y, x) = (x*dy - y*dx) / (x^2 + y^2)
        // here a = y, b = x
        BinFn::Atan2 => {
            let num = Sub(
                Box::new(Mul(Box::new(b.clone()), Box::new(da))),
                Box::new(Mul(Box::new(a.clone()), Box::new(db))),
            );
            let den = Add(
                Box::new(Mul(Box::new(a.clone()), Box::new(a.clone()))),
                Box::new(Mul(Box::new(b.clone()), Box::new(b.clone()))),
            );
            Div(Box::new(num), Box::new(den))
        }
        // min/max: subgradient — derivative follows whichever branch is active.
        // Represent as a select via the difference: emit at codegen as a branch.
        // For the symbolic form we use the convention min'(a,b)=da if a<=b else db.
        // We encode this with a dedicated Select node? To stay within the AST we
        // approximate with the active-branch derivative chosen at eval/emit time
        // via Func2 markers. Here we return a Sub-based form that the emitter
        // special-cases. To keep correctness we instead expand to a conditional
        // using the identity min(a,b) = (a+b-|a-b|)/2.
        BinFn::Min => {
            // d/dv (a+b-|a-b|)/2 = (da + db - sign(a-b)*(da-db)) / 2
            let diff_ab = Sub(Box::new(a.clone()), Box::new(b.clone()));
            let sign = Div(
                Box::new(diff_ab.clone()),
                Box::new(Func1(UnaryFn::Abs, Box::new(diff_ab))),
            );
            let inner = Sub(
                Box::new(Add(Box::new(da.clone()), Box::new(db.clone()))),
                Box::new(Mul(Box::new(sign), Box::new(Sub(Box::new(da), Box::new(db))))),
            );
            Mul(Box::new(Const(0.5)), Box::new(inner))
        }
        BinFn::Max => {
            // d/dv (a+b+|a-b|)/2
            let diff_ab = Sub(Box::new(a.clone()), Box::new(b.clone()));
            let sign = Div(
                Box::new(diff_ab.clone()),
                Box::new(Func1(UnaryFn::Abs, Box::new(diff_ab))),
            );
            let inner = Add(
                Box::new(Add(Box::new(da.clone()), Box::new(db.clone()))),
                Box::new(Mul(Box::new(sign), Box::new(Sub(Box::new(da), Box::new(db))))),
            );
            Mul(Box::new(Const(0.5)), Box::new(inner))
        }
        // pow(a,b): general d/dv a^b = a^b * (b' * ln(a) + b * a'/a)
        // When b is constant this simplifies; we handle that common case to
        // avoid emitting ln(a) (which would blow up for a<=0).
        BinFn::Pow => {
            if let Const(p) = b {
                // a^p with p constant: p * a^(p-1) * da
                let inner = Mul(
                    Box::new(Const(*p)),
                    Box::new(Func2(
                        BinFn::Pow,
                        Box::new(a.clone()),
                        Box::new(Const(p - 1.0)),
                    )),
                );
                Mul(Box::new(inner), Box::new(da))
            } else {
                let term1 = Mul(
                    Box::new(db),
                    Box::new(Func1(UnaryFn::Ln, Box::new(a.clone()))),
                );
                let term2 = Mul(Box::new(b.clone()), Box::new(Div(Box::new(da), Box::new(a.clone()))));
                let outer = Func2(BinFn::Pow, Box::new(a.clone()), Box::new(b.clone()));
                Mul(Box::new(outer), Box::new(Add(Box::new(term1), Box::new(term2))))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Simplification (constant folding + identity elimination)
// ---------------------------------------------------------------------------

impl Expr {
    /// Fold constants and eliminate `*1`, `+0`, `*0`, etc. Keeps the emitted
    /// Jacobian readable and avoids `0.0 * ...` chains.
    pub fn simplify(&self) -> Expr {
        use Expr::*;
        match self {
            Neg(a) => match a.simplify() {
                Const(v) => Const(-v),
                Neg(inner) => *inner,
                s => Neg(Box::new(s)),
            },
            Add(a, b) => {
                let (a, b) = (a.simplify(), b.simplify());
                match (&a, &b) {
                    (Const(x), Const(y)) => Const(x + y),
                    (Const(z), _) if *z == 0.0 => b,
                    (_, Const(z)) if *z == 0.0 => a,
                    _ => Add(Box::new(a), Box::new(b)),
                }
            }
            Sub(a, b) => {
                let (a, b) = (a.simplify(), b.simplify());
                match (&a, &b) {
                    (Const(x), Const(y)) => Const(x - y),
                    (_, Const(z)) if *z == 0.0 => a,
                    (Const(z), _) if *z == 0.0 => Neg(Box::new(b)).simplify(),
                    _ => Sub(Box::new(a), Box::new(b)),
                }
            }
            Mul(a, b) => {
                let (a, b) = (a.simplify(), b.simplify());
                match (&a, &b) {
                    (Const(x), Const(y)) => Const(x * y),
                    (Const(z), _) | (_, Const(z)) if *z == 0.0 => Const(0.0),
                    (Const(z), _) if *z == 1.0 => b,
                    (_, Const(z)) if *z == 1.0 => a,
                    _ => Mul(Box::new(a), Box::new(b)),
                }
            }
            Div(a, b) => {
                let (a, b) = (a.simplify(), b.simplify());
                match (&a, &b) {
                    (Const(x), Const(y)) if *y != 0.0 => Const(x / y),
                    (Const(z), _) if *z == 0.0 => Const(0.0),
                    (_, Const(z)) if *z == 1.0 => a,
                    _ => Div(Box::new(a), Box::new(b)),
                }
            }
            Func1(f, a) => Func1(*f, Box::new(a.simplify())),
            Func2(f, a, b) => {
                let (a, b) = (a.simplify(), b.simplify());
                if let (BinFn::Pow, _, Const(p)) = (f, &a, &b) {
                    if *p == 1.0 {
                        return a;
                    }
                    if *p == 0.0 {
                        return Const(1.0);
                    }
                }
                Func2(*f, Box::new(a), Box::new(b))
            }
            Ddt(s, a) => Ddt(*s, Box::new(a.simplify())),
            Idt(s, a) => Idt(*s, Box::new(a.simplify())),
            other => other.clone(),
        }
    }

    /// `true` if this simplified to the constant zero.
    pub fn is_zero(&self) -> bool {
        matches!(self, Expr::Const(v) if *v == 0.0)
    }
}

// ---------------------------------------------------------------------------
// Interpreter (DC OP + tests)
// ---------------------------------------------------------------------------

/// Evaluation context for the `f64` interpreter.
pub trait EvalCtx {
    /// Voltage at node `name` (ground returns 0.0).
    fn node_v(&self, name: &str) -> f64;
    /// Branch current through element `name`.
    fn branch_i(&self, name: &str) -> f64;
    /// Value of named scalar parameter `name` (`.pot` / `.runtime` / `.param`).
    fn param(&self, name: &str) -> f64;
    /// Simulation time (seconds).
    fn time(&self) -> f64;
    /// `1/dt`. Return `0.0` to freeze `ddt` derivatives (e.g. at DC).
    fn inv_dt(&self) -> f64;
    /// `dt/2`.
    fn half_dt(&self) -> f64;
    /// Previous inner value `x[n-1]` for `ddt`/`idt` slot.
    fn x_prev(&self, slot: usize) -> f64;
    /// Previous integral `idt[n-1]` for `idt` slot.
    fn integ_prev(&self, slot: usize) -> f64;
}

/// Clamp matching the generated `safe_exp` (prevents overflow in `exp`).
fn safe_exp(x: f64) -> f64 {
    x.clamp(-40.0, 40.0).exp()
}

impl Expr {
    /// Evaluate to an `f64` against `ctx`. Used by the DC operating-point solver
    /// and by unit tests as the oracle for the emitted straight-line code.
    pub fn eval(&self, ctx: &dyn EvalCtx) -> f64 {
        use Expr::*;
        match self {
            Const(v) => *v,
            Time => ctx.time(),
            InvDt => ctx.inv_dt(),
            HalfDt => ctx.half_dt(),
            NodeV(n) => ctx.node_v(n),
            NodeVDiff(a, b) => ctx.node_v(a) - ctx.node_v(b),
            BranchI(n) => ctx.branch_i(n),
            Param(n) => ctx.param(n),
            Neg(a) => -a.eval(ctx),
            Add(a, b) => a.eval(ctx) + b.eval(ctx),
            Sub(a, b) => a.eval(ctx) - b.eval(ctx),
            Mul(a, b) => a.eval(ctx) * b.eval(ctx),
            Div(a, b) => a.eval(ctx) / b.eval(ctx),
            Func1(f, a) => {
                let x = a.eval(ctx);
                match f {
                    UnaryFn::Sqrt => x.max(0.0).sqrt(),
                    UnaryFn::Abs => x.abs(),
                    UnaryFn::Exp => safe_exp(x),
                    UnaryFn::Ln => x.max(1e-300).ln(),
                    UnaryFn::Sin => x.sin(),
                    UnaryFn::Cos => x.cos(),
                    UnaryFn::Tanh => x.tanh(),
                }
            }
            Func2(f, a, b) => {
                let (x, y) = (a.eval(ctx), b.eval(ctx));
                match f {
                    BinFn::Atan2 => x.atan2(y),
                    BinFn::Min => x.min(y),
                    BinFn::Max => x.max(y),
                    BinFn::Pow => x.powf(y),
                }
            }
            Ddt(slot, a) => (a.eval(ctx) - ctx.x_prev(*slot)) * ctx.inv_dt(),
            Idt(slot, a) => {
                ctx.integ_prev(*slot) + ctx.half_dt() * (a.eval(ctx) + ctx.x_prev(*slot))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Rust emitter (codegen — straight-line, RT-safe)
// ---------------------------------------------------------------------------

/// Resolves AST leaves to generated-Rust lvalue/expression strings.
pub trait ExprResolver {
    /// e.g. `"v[3]"` (node 0 / ground resolves to `"0.0"`).
    fn node_v(&self, name: &str) -> String;
    /// e.g. `"v[7]"` (the branch-current augmented unknown for `name`).
    fn branch_i(&self, name: &str) -> String;
    /// Named scalar parameter `name`, e.g. `"state.pot_0_resistance"` or a baked
    /// constant. Implementations resolve `.pot` / `.runtime` / `.param` here.
    fn param(&self, name: &str) -> String;
    /// e.g. `"state.sim_time"`.
    fn time(&self) -> String;
    /// e.g. `"INV_DT"`.
    fn inv_dt(&self) -> String;
    /// e.g. `"HALF_DT"`.
    fn half_dt(&self) -> String;
    /// Previous inner value field for slot, e.g. `"state.bsrc_ddt_0_x_prev"`.
    fn x_prev(&self, slot: usize) -> String;
    /// Previous integral field for slot, e.g. `"state.bsrc_idt_0_int_prev"`.
    fn integ_prev(&self, slot: usize) -> String;
}

impl Expr {
    /// Emit a straight-line Rust `f64` expression. Guards (`safe_exp`,
    /// `atan2`/`sqrt`/`ln` domain protection, `abs` epsilon) match the
    /// interpreter so generated code agrees with [`Expr::eval`].
    pub fn to_rust(&self, r: &dyn ExprResolver) -> String {
        use Expr::*;
        match self {
            Const(v) => fmt_f64(*v),
            Time => r.time(),
            InvDt => r.inv_dt(),
            HalfDt => r.half_dt(),
            NodeV(n) => r.node_v(n),
            NodeVDiff(a, b) => format!("({} - {})", r.node_v(a), r.node_v(b)),
            BranchI(n) => r.branch_i(n),
            Param(n) => r.param(n),
            Neg(a) => format!("(-{})", a.to_rust(r)),
            Add(a, b) => format!("({} + {})", a.to_rust(r), b.to_rust(r)),
            Sub(a, b) => format!("({} - {})", a.to_rust(r), b.to_rust(r)),
            Mul(a, b) => format!("({} * {})", a.to_rust(r), b.to_rust(r)),
            Div(a, b) => format!("({} / {})", a.to_rust(r), b.to_rust(r)),
            Func1(f, a) => {
                let x = a.to_rust(r);
                match f {
                    // sqrt guarded at 0 (the limiter already adds 1e-9 inside)
                    UnaryFn::Sqrt => format!("({x}).max(0.0).sqrt()"),
                    UnaryFn::Abs => format!("({x}).abs()"),
                    UnaryFn::Exp => format!("bsrc_safe_exp({x})"),
                    UnaryFn::Ln => format!("({x}).max(1e-300).ln()"),
                    UnaryFn::Sin => format!("({x}).sin()"),
                    UnaryFn::Cos => format!("({x}).cos()"),
                    UnaryFn::Tanh => format!("({x}).tanh()"),
                }
            }
            Func2(f, a, b) => {
                let (x, y) = (a.to_rust(r), b.to_rust(r));
                match f {
                    BinFn::Atan2 => format!("({x}).atan2({y})"),
                    BinFn::Min => format!("({x}).min({y})"),
                    BinFn::Max => format!("({x}).max({y})"),
                    BinFn::Pow => format!("({x}).powf({y})"),
                }
            }
            Ddt(slot, a) => format!(
                "(({} - {}) * {})",
                a.to_rust(r),
                r.x_prev(*slot),
                r.inv_dt()
            ),
            Idt(slot, a) => format!(
                "({} + {} * ({} + {}))",
                r.integ_prev(*slot),
                r.half_dt(),
                a.to_rust(r),
                r.x_prev(*slot)
            ),
        }
    }
}

/// Format an `f64` as a Rust literal that round-trips exactly.
fn fmt_f64(v: f64) -> String {
    if v == v.trunc() && v.abs() < 1e15 {
        format!("{:.1}", v)
    } else {
        format!("{:e}", v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    struct MapCtx {
        nodes: HashMap<String, f64>,
        branches: HashMap<String, f64>,
        params: HashMap<String, f64>,
        time: f64,
        inv_dt: f64,
        half_dt: f64,
        x_prev: HashMap<usize, f64>,
        integ_prev: HashMap<usize, f64>,
    }

    impl MapCtx {
        fn new() -> Self {
            MapCtx {
                nodes: HashMap::new(),
                branches: HashMap::new(),
                params: HashMap::new(),
                time: 0.0,
                inv_dt: 0.0,
                half_dt: 0.0,
                x_prev: HashMap::new(),
                integ_prev: HashMap::new(),
            }
        }
        fn with(mut self, n: &str, v: f64) -> Self {
            self.nodes.insert(n.to_string(), v);
            self
        }
    }

    impl EvalCtx for MapCtx {
        fn node_v(&self, name: &str) -> f64 {
            if name == "0" {
                return 0.0;
            }
            *self.nodes.get(name).unwrap_or(&0.0)
        }
        fn branch_i(&self, name: &str) -> f64 {
            *self.branches.get(name).unwrap_or(&0.0)
        }
        fn param(&self, name: &str) -> f64 {
            *self.params.get(name).unwrap_or(&0.0)
        }
        fn time(&self) -> f64 {
            self.time
        }
        fn inv_dt(&self) -> f64 {
            self.inv_dt
        }
        fn half_dt(&self) -> f64 {
            self.half_dt
        }
        fn x_prev(&self, slot: usize) -> f64 {
            *self.x_prev.get(&slot).unwrap_or(&0.0)
        }
        fn integ_prev(&self, slot: usize) -> f64 {
            *self.integ_prev.get(&slot).unwrap_or(&0.0)
        }
    }

    fn parse(s: &str) -> Expr {
        let mut e = Expr::parse(s).expect("parse");
        let mut next = 0;
        e.assign_state_slots(&mut next);
        e
    }

    #[test]
    fn parse_and_eval_arithmetic() {
        let e = parse("2 + 3 * 4");
        let ctx = MapCtx::new();
        assert_eq!(e.eval(&ctx), 14.0);
    }

    #[test]
    fn power_is_right_associative() {
        // 2^3^2 = 2^(3^2) = 2^9 = 512
        let e = parse("2 ^ 3 ^ 2");
        assert_eq!(e.eval(&MapCtx::new()), 512.0);
    }

    #[test]
    fn unary_minus_and_parens() {
        let e = parse("-(2 + 3) * 2");
        assert_eq!(e.eval(&MapCtx::new()), -10.0);
    }

    #[test]
    fn node_voltage_refs() {
        let e = parse("V(a) * V(b)");
        let ctx = MapCtx::new().with("a", 3.0).with("b", 5.0);
        assert_eq!(e.eval(&ctx), 15.0);
        assert_eq!(e.referenced_nodes().len(), 2);
    }

    #[test]
    fn node_diff_ref() {
        let e = parse("V(a, b)");
        let ctx = MapCtx::new().with("a", 7.0).with("b", 2.0);
        assert_eq!(e.eval(&ctx), 5.0);
    }

    #[test]
    fn limiter_expression() {
        // The radio limiter: V(i) / sqrt(V(i)^2 + V(q)^2 + 1e-9)
        let e = parse("V(i) / sqrt(V(i)*V(i) + V(q)*V(q) + 1e-9)");
        let ctx = MapCtx::new().with("i", 3.0).with("q", 4.0);
        // 3 / sqrt(9+16) = 3/5
        assert!((e.eval(&ctx) - 0.6).abs() < 1e-9);
    }

    #[test]
    fn functions_eval() {
        assert!((parse("tanh(0.5)").eval(&MapCtx::new()) - 0.5f64.tanh()).abs() < 1e-12);
        assert!(
            (parse("atan2(1, 1)").eval(&MapCtx::new()) - std::f64::consts::FRAC_PI_4).abs() < 1e-12
        );
        assert_eq!(parse("min(3, 5)").eval(&MapCtx::new()), 3.0);
        assert_eq!(parse("max(3, 5)").eval(&MapCtx::new()), 5.0);
        assert_eq!(parse("abs(-4)").eval(&MapCtx::new()), 4.0);
        assert!((parse("pow(2, 10)").eval(&MapCtx::new()) - 1024.0).abs() < 1e-9);
    }

    /// Central-difference check that symbolic diff matches numeric derivative.
    fn check_diff(src: &str, node: &str, vals: &[(&str, f64)]) {
        let e = parse(src);
        let mut ctx = MapCtx::new();
        for (n, v) in vals {
            ctx.nodes.insert(n.to_string(), *v);
        }
        let d = e.diff(&Var::Node(node.to_string())).simplify();
        let analytic = d.eval(&ctx);

        let h = 1e-6;
        let base = *vals.iter().find(|(n, _)| n == &node).map(|(_, v)| v).unwrap_or(&0.0);
        let mut ctx_p = MapCtx::new();
        let mut ctx_m = MapCtx::new();
        for (n, v) in vals {
            ctx_p.nodes.insert(n.to_string(), *v);
            ctx_m.nodes.insert(n.to_string(), *v);
        }
        ctx_p.nodes.insert(node.to_string(), base + h);
        ctx_m.nodes.insert(node.to_string(), base - h);
        let numeric = (e.eval(&ctx_p) - e.eval(&ctx_m)) / (2.0 * h);
        assert!(
            (analytic - numeric).abs() < 1e-4,
            "diff mismatch for d/d{node} [{src}]: analytic={analytic}, numeric={numeric}"
        );
    }

    #[test]
    fn diff_matches_numeric() {
        check_diff("V(a) * V(b)", "a", &[("a", 3.0), ("b", 5.0)]);
        check_diff("V(a) * V(b)", "b", &[("a", 3.0), ("b", 5.0)]);
        check_diff("tanh(V(a))", "a", &[("a", 0.7)]);
        check_diff("exp(V(a))", "a", &[("a", 1.2)]);
        check_diff("V(a) / V(b)", "a", &[("a", 2.0), ("b", 4.0)]);
        check_diff("V(a) / V(b)", "b", &[("a", 2.0), ("b", 4.0)]);
        check_diff("sqrt(V(a)*V(a) + V(b)*V(b))", "a", &[("a", 3.0), ("b", 4.0)]);
        check_diff("atan2(V(q), V(i))", "i", &[("i", 0.8), ("q", 0.3)]);
        check_diff("atan2(V(q), V(i))", "q", &[("i", 0.8), ("q", 0.3)]);
        check_diff("V(a)^3", "a", &[("a", 1.7)]);
        check_diff(
            "V(i) / sqrt(V(i)*V(i) + V(q)*V(q) + 1e-9)",
            "i",
            &[("i", 0.6), ("q", 0.4)],
        );
    }

    #[test]
    fn ddt_backward_difference() {
        // ddt(V(a)): (x - x_prev) * inv_dt
        let e = parse("ddt(V(a))");
        let mut ctx = MapCtx::new().with("a", 2.0);
        ctx.inv_dt = 48000.0;
        ctx.x_prev.insert(0, 1.0);
        // (2 - 1) * 48000 = 48000
        assert_eq!(e.eval(&ctx), 48000.0);
        assert_eq!(e.state_slot_count(), 1);
        assert!(e.is_time_dependent());
    }

    #[test]
    fn idt_trapezoidal() {
        let e = parse("idt(V(a))");
        let mut ctx = MapCtx::new().with("a", 2.0);
        ctx.half_dt = 0.5;
        ctx.x_prev.insert(0, 4.0);
        ctx.integ_prev.insert(0, 10.0);
        // 10 + 0.5*(2 + 4) = 13
        assert_eq!(e.eval(&ctx), 13.0);
    }

    #[test]
    fn ddt_diff_uses_inv_dt() {
        let e = parse("ddt(V(a) * V(b))");
        let d = e.diff(&Var::Node("a".to_string())).simplify();
        // ∂/∂a ddt(a*b) = inv_dt * b
        let mut ctx = MapCtx::new().with("a", 3.0).with("b", 5.0);
        ctx.inv_dt = 100.0;
        assert_eq!(d.eval(&ctx), 500.0);
    }

    #[test]
    fn emitter_produces_compilable_strings() {
        struct R;
        impl ExprResolver for R {
            fn node_v(&self, name: &str) -> String {
                match name {
                    "0" => "0.0".to_string(),
                    "a" => "v[1]".to_string(),
                    "b" => "v[2]".to_string(),
                    _ => "0.0".to_string(),
                }
            }
            fn branch_i(&self, _name: &str) -> String {
                "0.0".to_string()
            }
            fn param(&self, name: &str) -> String {
                format!("PARAM_{name}")
            }
            fn time(&self) -> String {
                "state.sim_time".to_string()
            }
            fn inv_dt(&self) -> String {
                "INV_DT".to_string()
            }
            fn half_dt(&self) -> String {
                "HALF_DT".to_string()
            }
            fn x_prev(&self, slot: usize) -> String {
                format!("state.bsrc_x_prev[{slot}]")
            }
            fn integ_prev(&self, slot: usize) -> String {
                format!("state.bsrc_int_prev[{slot}]")
            }
        }
        let e = parse("tanh(V(a)) * V(b) + ddt(V(a))");
        let s = e.to_rust(&R);
        assert!(s.contains("(v[1]).tanh()"));
        assert!(s.contains("v[2]"));
        assert!(s.contains("INV_DT"));
        assert!(s.contains("state.bsrc_x_prev[0]"));
    }

    #[test]
    fn rejects_bad_input() {
        assert!(Expr::parse("V(a) +").is_err());
        assert!(Expr::parse("foobar(1)").is_err()); // unknown function call
        assert!(Expr::parse("atan2(1)").is_err());
        assert!(Expr::parse("(1 + 2").is_err());
        assert!(Expr::parse("").is_err());
    }

    #[test]
    fn bare_identifier_is_a_parameter() {
        // The virtual-antenna netlists reference .pot/.runtime/.param scalars
        // as bare identifiers: strength*(1 + m*V(prog)).
        let e = parse("strength * (1 + m * V(prog))");
        assert_eq!(
            e.referenced_params(),
            ["m", "strength"].iter().map(|s| s.to_string()).collect()
        );
        let mut ctx = MapCtx::new().with("prog", 0.5);
        ctx.params.insert("strength".to_string(), 2.0);
        ctx.params.insert("m".to_string(), 0.8);
        // 2 * (1 + 0.8*0.5) = 2 * 1.4 = 2.8
        assert!((e.eval(&ctx) - 2.8).abs() < 1e-12);
        // params are constants for NR differentiation
        assert!(e.diff(&Var::Node("prog".to_string())).simplify().eval(&ctx) - 1.6 < 1e-9);
    }

    #[test]
    fn am_carrier_phase_compiles() {
        // theta = idt(2*pi*f_offset); B_I = strength*cos(theta) + V(n_i)
        let mut e = parse("strength * cos(idt(2 * pi * f_offset)) + V(n_i)");
        let mut next = 0;
        e.assign_state_slots(&mut next);
        assert_eq!(e.state_slot_count(), 1);
        assert!(e.referenced_params().contains("strength"));
        assert!(e.referenced_params().contains("f_offset"));
        assert!(e.referenced_nodes().contains("n_i"));
    }

    #[test]
    fn simplify_folds_constants() {
        let e = parse("V(a) * 1 + 0").simplify();
        assert_eq!(e, Expr::NodeV("a".to_string()));
        let z = parse("V(a) * 0").simplify();
        assert!(z.is_zero());
    }
}
