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
//!   - functions: `atan2 sqrt abs exp ln sin cos tanh min max pow pwr`
//!   - time operators: `ddt(x)` (derivative), `idt(x)` (integral)
//!
//! Precedence (loosest → tightest): `+ -` < `* /` < unary `-` < `^`.
//! `^` binding tighter than unary minus means `-x^2 = -(x^2)` — matching
//! ngspice (`inpptree-parser.y` declares `%left NEG` *before* `%left '^'`)
//! and standard math. `^` is right-associative here (`2^3^2 = 512`), which
//! **diverges** from ngspice's `%left '^'` (left-assoc, `2^3^2 = 64`) —
//! parenthesize chained powers in netlists that must cross-validate.
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
    /// INTERNAL — not parseable. The derivative paired with the clamped `exp`
    /// value (`exp(clamp(x, -40, 40))`): `exp(x)` inside `[-40, 40]`, exactly
    /// `0` outside (the value is flat there, so the slope must be too).
    /// Produced only by differentiating [`UnaryFn::Exp`].
    ExpDeriv,
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
    /// `pow(a,b)` / `a^b` with ngspice `PTpower` semantics: a negative base
    /// with a (near-)integer exponent snaps to the sign-correct integer power;
    /// a negative base with a fractional exponent uses `|a|^b` (never NaN).
    Pow,
    /// `pwr(a,b)` = `sign(a)·|a|^b` (ngspice `PTpwr`) — odd-symmetric power,
    /// audio-friendly for waveshaping (preserves signal polarity).
    Pwr,
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

    /// term := unary (('*' | '/') unary)*
    fn parse_term(&mut self) -> Result<Expr, ExprError> {
        let mut lhs = self.parse_unary()?;
        while let Some(t) = self.peek() {
            match t {
                Tok::Star => {
                    self.next();
                    let rhs = self.parse_unary()?;
                    lhs = Expr::Mul(Box::new(lhs), Box::new(rhs));
                }
                Tok::Slash => {
                    self.next();
                    let rhs = self.parse_unary()?;
                    lhs = Expr::Div(Box::new(lhs), Box::new(rhs));
                }
                _ => break,
            }
        }
        Ok(lhs)
    }

    /// unary := '-' unary | '+' unary | power
    ///
    /// Unary minus binds LOOSER than `^`: `-x^2` parses as `-(x^2)`, matching
    /// ngspice (`%left NEG` is declared before `%left '^'` in
    /// `inpptree-parser.y`) and standard math. Write `(-x)^2` to square a
    /// negation.
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
            _ => self.parse_power(),
        }
    }

    /// power := atom ('^' unary)?    (right-associative power)
    ///
    /// The BASE side is an atom (no leading unary minus — that is what makes
    /// `-x^2 = -(x^2)`); the EXPONENT side goes back through `unary` so
    /// `2^-3` still parses. Right associativity (`2^3^2 = 2^(3^2)`) diverges
    /// from ngspice's `%left '^'` — see the module docs.
    fn parse_power(&mut self) -> Result<Expr, ExprError> {
        let base = self.parse_atom()?;
        if let Some(Tok::Caret) = self.peek() {
            self.next();
            let exp = self.parse_unary()?;
            Ok(Expr::Func2(BinFn::Pow, Box::new(base), Box::new(exp)))
        } else {
            Ok(base)
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
            "pwr" => {
                two_args(lname.as_str(), args).map(|(a, b)| Expr::Func2(BinFn::Pwr, Box::new(a), Box::new(b)))
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
        self.diff_impl(var, false)
    }

    /// Partial derivative for the **NR Jacobian** of a behavioral source, with
    /// `ddt` treated as a constant (`∂ddt/∂v = 0`) — a "lagged Jacobian".
    ///
    /// `ddt(x)`'s true partial is `inv_dt·∂x/∂v`, which at audio rates is
    /// `~SR×` (tens of thousands) larger than the rest of the system and
    /// ill-conditions the coupled NR (the FM discriminator destabilises the
    /// limiter). Treating `ddt` as constant in the Jacobian removes that
    /// stiffness. It does **not** lag the output: the residual / companion
    /// still uses the full `ddt` value at the current sample, so a feedforward
    /// discriminator converges to `V = ddt(...)` exactly (the Jacobian only
    /// steers the iteration, not the fixed point). `idt`'s partial keeps its
    /// `half_dt` scaling — it's `~dt/2` (tiny), never the stiff term.
    pub fn diff_jacobian(&self, var: &Var) -> Expr {
        self.diff_impl(var, true)
    }

    fn diff_impl(&self, var: &Var, lag_ddt: bool) -> Expr {
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
            Neg(a) => Neg(Box::new(a.diff_impl(var, lag_ddt))),
            Add(a, b) => Add(
                Box::new(a.diff_impl(var, lag_ddt)),
                Box::new(b.diff_impl(var, lag_ddt)),
            ),
            Sub(a, b) => Sub(
                Box::new(a.diff_impl(var, lag_ddt)),
                Box::new(b.diff_impl(var, lag_ddt)),
            ),
            Mul(a, b) => {
                // (a*b)' = a'*b + a*b'
                Add(
                    Box::new(Mul(Box::new(a.diff_impl(var, lag_ddt)), b.clone())),
                    Box::new(Mul(a.clone(), Box::new(b.diff_impl(var, lag_ddt)))),
                )
            }
            Div(a, b) => {
                // (a/b)' = (a'*b - a*b') / (b^2 + 1e-300). The tiny guard
                // keeps the 0/0 case (numerator and denominator both zero at
                // b = 0) from emitting NaN into the Jacobian; away from b = 0
                // it is far below f64 resolution. The VALUE of `/` is left
                // unguarded on purpose (ngspice parity) — see the division
                // hazard note in BEHAVIORAL_SOURCES.md.
                let num = Sub(
                    Box::new(Mul(Box::new(a.diff_impl(var, lag_ddt)), b.clone())),
                    Box::new(Mul(a.clone(), Box::new(b.diff_impl(var, lag_ddt)))),
                );
                let den = Add(
                    Box::new(Mul(b.clone(), b.clone())),
                    Box::new(Const(1e-300)),
                );
                Div(Box::new(num), Box::new(den))
            }
            Func1(f, a) => {
                let da = a.diff_impl(var, lag_ddt);
                let outer = func1_derivative(*f, a);
                Mul(Box::new(outer), Box::new(da))
            }
            Func2(f, a, b) => func2_derivative(*f, a, b, var, lag_ddt),
            Ddt(_, a) => {
                if lag_ddt {
                    Const(0.0)
                } else {
                    Mul(Box::new(InvDt), Box::new(a.diff_impl(var, lag_ddt)))
                }
            }
            Idt(_, a) => Mul(Box::new(HalfDt), Box::new(a.diff_impl(var, lag_ddt))),
        }
    }
}

/// Branchless subgradient sign: `x / (|x| + 1e-300)`. Exactly `0.0` at
/// `x == 0` (the documented subgradient for `abs'` and the min/max tie), ±1
/// within ~1e-16 for `|x| ≳ 1e-284, and never 0/0 = NaN. Expressed in the
/// AST itself so the interpreter ([`Expr::eval`]) and the Rust emitter
/// ([`Expr::to_rust`]) agree by construction — there is no separate emitted
/// helper to drift out of sync.
fn sign_subgradient(a: &Expr) -> Expr {
    use Expr::*;
    Div(
        Box::new(a.clone()),
        Box::new(Add(
            Box::new(Func1(UnaryFn::Abs, Box::new(a.clone()))),
            Box::new(Const(1e-300)),
        )),
    )
}

/// Outer derivative `f'(a)` (not yet multiplied by `a'`).
fn func1_derivative(f: UnaryFn, a: &Expr) -> Expr {
    use Expr::*;
    match f {
        // d/dx sqrt(x) = 1/(2*sqrt(x)). The value clamps the domain
        // (`max(0.0).sqrt()`), so the raw form is +inf at x <= 0; the 1e-150
        // denominator guard makes the slope at/below 0 huge (5e149) but
        // FINITE — NR sees a cliff and takes a tiny step instead of NaN.
        UnaryFn::Sqrt => Div(
            Box::new(Const(0.5)),
            Box::new(Add(
                Box::new(Func1(UnaryFn::Sqrt, Box::new(a.clone()))),
                Box::new(Const(1e-150)),
            )),
        ),
        // d/dx abs(x) = sign(x), as the branchless subgradient
        // x/(|x|+1e-300): exactly 0 at x == 0, never 0/0 = NaN. (The previous
        // x/abs(x) form claimed an epsilon guard "at codegen" that did not
        // exist — the guard now lives in the AST itself.)
        UnaryFn::Abs => sign_subgradient(a),
        // d/dx exp(x): the value is exp(clamp(x, -40, 40)), so the paired
        // derivative is exp(x) inside the window and 0 outside (flat region)
        // — value and slope agree everywhere.
        UnaryFn::Exp => Func1(UnaryFn::ExpDeriv, Box::new(a.clone())),
        // ExpDeriv only appears inside derivative ASTs. If differentiated
        // again (2nd derivative), keep the pairing: inside the window the
        // slope's slope is exp(x) again, outside it is 0.
        UnaryFn::ExpDeriv => Func1(UnaryFn::ExpDeriv, Box::new(a.clone())),
        // d/dx ln(x) = 1/x, guarded to match the value's ln(max(x, 1e-300)):
        // 1/max(x, 1e-300) is finite (<= 1e300) for ALL x, including 0 and
        // negatives, where the value is pinned at ln(1e-300).
        UnaryFn::Ln => Div(
            Box::new(Const(1.0)),
            Box::new(Func2(
                BinFn::Max,
                Box::new(a.clone()),
                Box::new(Const(1e-300)),
            )),
        ),
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

fn func2_derivative(f: BinFn, a: &Expr, b: &Expr, var: &Var, lag_ddt: bool) -> Expr {
    use Expr::*;
    let da = a.diff_impl(var, lag_ddt);
    let db = b.diff_impl(var, lag_ddt);
    match f {
        // d atan2(y, x) = (x*dy - y*dx) / (x^2 + y^2 + eps)
        // here a = y, b = x. `eps` (1e-30) regularizes the Jacobian at the
        // origin (y=x=0); without it the NR Jacobian is 0/0 = NaN — exactly the
        // singularity the FM discriminator hits at startup when the limiter
        // outputs are still 0. Negligible away from the origin.
        BinFn::Atan2 => {
            let num = Sub(
                Box::new(Mul(Box::new(b.clone()), Box::new(da))),
                Box::new(Mul(Box::new(a.clone()), Box::new(db))),
            );
            let den = Add(
                Box::new(Add(
                    Box::new(Mul(Box::new(a.clone()), Box::new(a.clone()))),
                    Box::new(Mul(Box::new(b.clone()), Box::new(b.clone()))),
                )),
                Box::new(Const(1e-30)),
            );
            Div(Box::new(num), Box::new(den))
        }
        // min/max: subgradient via the identity min(a,b) = (a+b-|a-b|)/2 with
        // the branchless sign x/(|x|+1e-300). At a TIE (a == b) the sign term
        // is exactly 0, so the derivative is the 0.5/0.5 split (da+db)/2 —
        // documented convention, never 0/0 = NaN. Away from the tie it follows
        // the active branch exactly.
        BinFn::Min => {
            // d/dv (a+b-|a-b|)/2 = (da + db - sign(a-b)*(da-db)) / 2
            let diff_ab = Sub(Box::new(a.clone()), Box::new(b.clone()));
            let sign = sign_subgradient(&diff_ab);
            let inner = Sub(
                Box::new(Add(Box::new(da.clone()), Box::new(db.clone()))),
                Box::new(Mul(Box::new(sign), Box::new(Sub(Box::new(da), Box::new(db))))),
            );
            Mul(Box::new(Const(0.5)), Box::new(inner))
        }
        BinFn::Max => {
            // d/dv (a+b+|a-b|)/2 — tie convention as for Min: (da+db)/2.
            let diff_ab = Sub(Box::new(a.clone()), Box::new(b.clone()));
            let sign = sign_subgradient(&diff_ab);
            let inner = Add(
                Box::new(Add(Box::new(da.clone()), Box::new(db.clone()))),
                Box::new(Mul(Box::new(sign), Box::new(Sub(Box::new(da), Box::new(db))))),
            );
            Mul(Box::new(Const(0.5)), Box::new(inner))
        }
        // pow(a,b), ngspice PTpower value semantics (see `bsrc_pow`). The
        // derivative follows whichever branch the VALUE takes:
        //  - constant (near-)integer exponent → integer power rule, valid for
        //    all bases (the value is the sign-correct integer power);
        //  - constant fractional exponent → the value is |a|^p, whose exact
        //    derivative is p·|a|^p·a/a²; the a/(a²+1e-300) form is 0 (not
        //    NaN) at a = 0 for p > 0. (For p < 0 the value itself has a pole
        //    at a = 0 — the derivative is coherently singular there.)
        //  - variable exponent → a^b·(db·ln|a| + b·da·a/(a²+1e-300)): ln|a|
        //    (domain-guarded by ln's max(1e-300)) matches the |a|^b value
        //    branch for a < 0, and the a/(a²+eps) form guards b·da/a at a = 0.
        BinFn::Pow => {
            let a2_guard = Add(
                Box::new(Mul(Box::new(a.clone()), Box::new(a.clone()))),
                Box::new(Const(1e-300)),
            );
            if let Const(p) = b {
                if pow_exp_near_integer(*p) {
                    // p * a^(p-1) * da, with p snapped so a^(p-1) is itself
                    // an exact integer power (sign-correct for a < 0).
                    let pi = p.round();
                    let inner = Mul(
                        Box::new(Const(pi)),
                        Box::new(Func2(
                            BinFn::Pow,
                            Box::new(a.clone()),
                            Box::new(Const(pi - 1.0)),
                        )),
                    );
                    Mul(Box::new(inner), Box::new(da))
                } else {
                    // p * |a|^p * a / (a² + 1e-300) * da
                    let abs_pow = Func2(
                        BinFn::Pow,
                        Box::new(Func1(UnaryFn::Abs, Box::new(a.clone()))),
                        Box::new(Const(*p)),
                    );
                    let ratio = Div(
                        Box::new(Mul(Box::new(abs_pow), Box::new(a.clone()))),
                        Box::new(a2_guard),
                    );
                    Mul(
                        Box::new(Mul(Box::new(Const(*p)), Box::new(ratio))),
                        Box::new(da),
                    )
                }
            } else {
                let term1 = Mul(
                    Box::new(db),
                    Box::new(Func1(
                        UnaryFn::Ln,
                        Box::new(Func1(UnaryFn::Abs, Box::new(a.clone()))),
                    )),
                );
                let term2 = Mul(
                    Box::new(b.clone()),
                    Box::new(Div(
                        Box::new(Mul(Box::new(da), Box::new(a.clone()))),
                        Box::new(a2_guard),
                    )),
                );
                let outer = Func2(BinFn::Pow, Box::new(a.clone()), Box::new(b.clone()));
                Mul(Box::new(outer), Box::new(Add(Box::new(term1), Box::new(term2))))
            }
        }
        // pwr(a,b) = sign(a)·|a|^b:
        //   ∂/∂a = b·|a|^(b-1)  (odd-symmetric — same slope on both sides),
        //          guarded as b·|a|^b·|a|/(a²+1e-300) → 0 (not inf) at a = 0
        //          for b > 1, matching the flat value there;
        //   ∂/∂b = pwr(a,b)·ln|a|  (ln domain-guarded).
        BinFn::Pwr => {
            let abs_a = Func1(UnaryFn::Abs, Box::new(a.clone()));
            let abs_pow = Func2(BinFn::Pow, Box::new(abs_a.clone()), Box::new(b.clone()));
            let a2_guard = Add(
                Box::new(Mul(Box::new(a.clone()), Box::new(a.clone()))),
                Box::new(Const(1e-300)),
            );
            let da_term = Mul(
                Box::new(b.clone()),
                Box::new(Div(
                    Box::new(Mul(Box::new(abs_pow), Box::new(abs_a.clone()))),
                    Box::new(a2_guard),
                )),
            );
            let db_term = Mul(
                Box::new(Func2(BinFn::Pwr, Box::new(a.clone()), Box::new(b.clone()))),
                Box::new(Func1(UnaryFn::Ln, Box::new(abs_a))),
            );
            Add(
                Box::new(Mul(Box::new(da_term), Box::new(da))),
                Box::new(Mul(Box::new(db_term), Box::new(db))),
            )
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

/// Clamped exponential (prevents overflow). The emitter inlines the identical
/// `(x).clamp(-40.0, 40.0).exp()`; the paired derivative ([`UnaryFn::ExpDeriv`])
/// is 0 outside the window so value and slope agree.
fn safe_exp(x: f64) -> f64 {
    x.clamp(-40.0, 40.0).exp()
}

/// The paired derivative of [`safe_exp`]: `exp(x)` inside `[-40, 40]`, 0 in the
/// clamped-flat region. Emitter inlines the same logic (see [`Expr::to_rust`]).
fn safe_exp_deriv(x: f64) -> f64 {
    if (-40.0..=40.0).contains(&x) {
        x.exp()
    } else {
        0.0
    }
}

/// Near-integer exponent predicate shared by the `pow` value ([`bsrc_pow`]) and
/// its symbolic derivative, and mirrored textually by the emitted Rust — the
/// three MUST stay in sync. This is a cleaned-up form of ngspice `PTpower`'s
/// test (`fabs(y - (int)y) / (y + 0.001) < 1e-6` — quirky denominator; we use a
/// relative-to-|y| tolerance with the same intent).
fn pow_exp_near_integer(y: f64) -> bool {
    (y - y.round()).abs() < 1e-6 * y.abs().max(1.0)
}

/// ngspice `PTpower` semantics (`ptfuncs.c`): a negative base with a
/// (near-)integer exponent snaps to the sign-correct integer power; a negative
/// base with a fractional exponent uses `|a|^b`. Non-negative bases use plain
/// `powf`. Never NaN for finite inputs (except the genuine 0^negative pole,
/// which is +inf in the value as in ngspice).
fn bsrc_pow(a: f64, b: f64) -> f64 {
    if a < 0.0 {
        if pow_exp_near_integer(b) {
            a.powf(b.round())
        } else {
            (-a).powf(b)
        }
    } else {
        a.powf(b)
    }
}

/// ngspice `PTpwr` semantics: `sign(a)·|a|^b`.
fn bsrc_pwr(a: f64, b: f64) -> f64 {
    if a >= 0.0 {
        a.powf(b)
    } else {
        -((-a).powf(b))
    }
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
                    UnaryFn::ExpDeriv => safe_exp_deriv(x),
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
                    BinFn::Pow => bsrc_pow(x, y),
                    BinFn::Pwr => bsrc_pwr(x, y),
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
    /// Emit a straight-line Rust `f64` expression. All guards (exp clamp,
    /// `sqrt`/`ln` domain protection, `pow` negative-base semantics) are
    /// emitted INLINE — the generated code is self-contained and depends on no
    /// emitted helper function — and match the interpreter exactly, so
    /// generated code agrees with [`Expr::eval`]. (An earlier version emitted
    /// calls to a `bsrc_safe_exp` helper that no emitter ever defined, so any
    /// `exp()` in a B-source failed to compile.)
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
                    // Inline clamp — matches interpreter `safe_exp`.
                    UnaryFn::Exp => format!("({x}).clamp(-40.0, 40.0).exp()"),
                    // Paired derivative of the clamped exp: 0 where the value
                    // is flat. Matches interpreter `safe_exp_deriv`.
                    UnaryFn::ExpDeriv => format!(
                        "({{ let bsx: f64 = {x}; \
                         if bsx >= -40.0 && bsx <= 40.0 {{ bsx.exp() }} else {{ 0.0 }} }})"
                    ),
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
                    // ngspice PTpower semantics — matches interpreter
                    // `bsrc_pow` (and `pow_exp_near_integer`; keep the inline
                    // predicate below textually in sync with it).
                    BinFn::Pow => {
                        if let Const(p) = &**b {
                            if pow_exp_near_integer(*p) {
                                let pi = p.round();
                                if pi.abs() <= i32::MAX as f64 {
                                    // Integer power: sign-correct and fast.
                                    format!("({x}).powi({})", pi as i32)
                                } else {
                                    // IEEE powf is sign-correct for exactly
                                    // integral exponents.
                                    format!("({x}).powf({})", fmt_f64(pi))
                                }
                            } else {
                                // Fractional constant exponent: value is
                                // |a|^p for all bases (PTpower branch).
                                format!("({x}).abs().powf({})", fmt_f64(*p))
                            }
                        } else {
                            format!(
                                "({{ let bsa: f64 = {x}; let bsb: f64 = {y}; \
                                 if bsa < 0.0 {{ \
                                 let bsr = bsb.round(); \
                                 if (bsb - bsr).abs() < 1e-6 * bsb.abs().max(1.0) \
                                 {{ bsa.powf(bsr) }} else {{ (-bsa).powf(bsb) }} \
                                 }} else {{ bsa.powf(bsb) }} }})"
                            )
                        }
                    }
                    // ngspice PTpwr: sign(a)·|a|^b — matches interpreter
                    // `bsrc_pwr`.
                    BinFn::Pwr => format!(
                        "({{ let bsa: f64 = {x}; let bsb: f64 = {y}; \
                         if bsa >= 0.0 {{ bsa.powf(bsb) }} else {{ -((-bsa).powf(bsb)) }} }})"
                    ),
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

/// Format an `f64` as a Rust literal that round-trips exactly. Non-finite
/// values become the `f64::` constants (mirrors
/// `codegen/rust_emitter/helpers.rs::fmt_f64`) — the bare `{:e}` tokens `inf`
/// / `NaN` are not valid Rust and would break the generated build.
fn fmt_f64(v: f64) -> String {
    if v.is_nan() {
        "f64::NAN".to_string()
    } else if v.is_infinite() {
        if v > 0.0 {
            "f64::INFINITY".to_string()
        } else {
            "f64::NEG_INFINITY".to_string()
        }
    } else if v == v.trunc() && v.abs() < 1e15 {
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
    fn unary_minus_binds_looser_than_power() {
        // ngspice (%left NEG before %left '^') and standard math: -x^2 = -(x^2).
        let c = MapCtx::new();
        assert_eq!(parse("-2^2").eval(&c), -4.0);
        assert_eq!(parse("(-2)^2").eval(&c), 4.0);
        assert_eq!(parse("-2^2+1").eval(&c), -3.0);
        assert_eq!(parse("-2**2").eval(&c), -4.0);
        // Exponent side still accepts a unary sign.
        assert_eq!(parse("2^-3").eval(&c), 0.125);
        assert_eq!(parse("2^+3").eval(&c), 8.0);
        // Plain unary minus untouched.
        assert_eq!(parse("-2").eval(&c), -2.0);
        assert_eq!(parse("3 * -2").eval(&c), -6.0);
        // -x^2 with a variable base.
        let cx = MapCtx::new().with("a", 3.0);
        assert_eq!(parse("-V(a)^2").eval(&cx), -9.0);
    }

    #[test]
    fn pow_negative_base_ngspice_semantics() {
        let c = MapCtx::new();
        // Integer exponents: sign-correct powers (ngspice PTpower).
        assert_eq!(parse("pow(-2, 3)").eval(&c), -8.0);
        assert_eq!(parse("(-2)^3").eval(&c), -8.0);
        assert_eq!(parse("pow(-2, 2)").eval(&c), 4.0);
        // Near-integer exponents snap to the integer power.
        assert!((parse("pow(-2, 2.0000001)").eval(&c) - 4.0).abs() < 1e-9);
        // Fractional exponent on a negative base: |x|^y, never NaN.
        let v = parse("pow(-0.5, 1.5)").eval(&c);
        assert!(v.is_finite(), "pow(-0.5, 1.5) must not NaN, got {v}");
        assert!((v - 0.5f64.powf(1.5)).abs() < 1e-12);
        // Positive base unchanged.
        assert!((parse("pow(2, 0.5)").eval(&c) - 2.0f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn pwr_is_sign_symmetric_power() {
        // pwr(x,y) = sign(x)·|x|^y (ngspice PTpwr).
        let c = MapCtx::new();
        assert_eq!(parse("pwr(2, 2)").eval(&c), 4.0);
        assert_eq!(parse("pwr(-3, 2)").eval(&c), -9.0); // sign kept, even exponent
        assert!((parse("pwr(-2, 0.5)").eval(&c) + 2.0f64.sqrt()).abs() < 1e-12);
        assert_eq!(parse("pwr(0, 2)").eval(&c), 0.0);
        // Derivative matches central difference away from 0.
        check_diff("pwr(V(a), 1.5)", "a", &[("a", 0.7)]);
        check_diff("pwr(V(a), 1.5)", "a", &[("a", -0.7)]);
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
        // Kink functions away from their kinks.
        check_diff("abs(V(a))", "a", &[("a", 0.8)]);
        check_diff("abs(V(a))", "a", &[("a", -0.8)]);
        check_diff("min(V(a), V(b))", "a", &[("a", 1.0), ("b", 2.0)]);
        check_diff("min(V(a), V(b))", "a", &[("a", 2.0), ("b", 1.0)]);
        check_diff("max(V(a), V(b))", "b", &[("a", 1.0), ("b", 2.0)]);
        // pow: negative base, fractional exponent (|a|^p branch).
        check_diff("pow(V(a), 1.5)", "a", &[("a", -0.5)]);
        // pow: negative base, integer exponent (sign-correct power rule).
        check_diff("pow(V(a), 3)", "a", &[("a", -1.3)]);
        // pow: variable exponent, positive and negative base.
        check_diff("pow(2, V(b))", "b", &[("b", 1.3)]);
        check_diff("pow(V(a), V(b))", "a", &[("a", -2.0), ("b", 1.5)]);
        check_diff("pow(V(a), V(b))", "b", &[("a", -2.0), ("b", 1.5)]);
        // ln inside its domain (guard must not perturb the derivative).
        check_diff("ln(V(a))", "a", &[("a", 0.3)]);
        // sqrt inside its domain.
        check_diff("sqrt(V(a))", "a", &[("a", 0.5)]);
    }

    /// Analytic derivative `∂src/∂node` evaluated at `vals` (simplified AST).
    fn analytic_diff(src: &str, node: &str, vals: &[(&str, f64)]) -> f64 {
        let e = parse(src);
        let mut ctx = MapCtx::new();
        for (n, v) in vals {
            ctx.nodes.insert(n.to_string(), *v);
        }
        e.diff(&Var::Node(node.to_string())).simplify().eval(&ctx)
    }

    /// Central-difference check with RELATIVE tolerance — for points where the
    /// derivative magnitude makes the absolute-tolerance `check_diff` harness
    /// meaningless (e.g. exp near the clamp window, |slope| ~ 1e17).
    fn check_diff_rel(src: &str, node: &str, vals: &[(&str, f64)], rel_tol: f64) {
        let e = parse(src);
        let analytic = analytic_diff(src, node, vals);
        let h = 1e-6;
        let base = *vals
            .iter()
            .find(|(n, _)| n == &node)
            .map(|(_, v)| v)
            .unwrap_or(&0.0);
        let mut ctx_p = MapCtx::new();
        let mut ctx_m = MapCtx::new();
        for (n, v) in vals {
            ctx_p.nodes.insert(n.to_string(), *v);
            ctx_m.nodes.insert(n.to_string(), *v);
        }
        ctx_p.nodes.insert(node.to_string(), base + h);
        ctx_m.nodes.insert(node.to_string(), base - h);
        let numeric = (e.eval(&ctx_p) - e.eval(&ctx_m)) / (2.0 * h);
        let scale = analytic.abs().max(numeric.abs()).max(1e-12);
        assert!(
            ((analytic - numeric) / scale).abs() < rel_tol,
            "rel diff mismatch for d/d{node} [{src}]: analytic={analytic}, numeric={numeric}"
        );
    }

    /// Domain-boundary rows: every derivative must be non-NaN (and, where the
    /// value is flat or kinked, equal the documented subgradient). These are
    /// exactly the points that used to emit 0/0 or 1/0 into the NR Jacobian.
    #[test]
    fn derivative_domain_boundaries_never_nan() {
        // abs at 0: documented subgradient is exactly 0 (sign = x/(|x|+1e-300)).
        assert_eq!(analytic_diff("abs(V(a))", "a", &[("a", 0.0)]), 0.0);
        // ±1 immediately off the kink.
        assert!((analytic_diff("abs(V(a))", "a", &[("a", 1e-3)]) - 1.0).abs() < 1e-12);
        assert!((analytic_diff("abs(V(a))", "a", &[("a", -1e-3)]) + 1.0).abs() < 1e-12);

        // min/max at a tie: documented 0.5/0.5 split, never NaN.
        for src in ["min(V(a), V(b))", "max(V(a), V(b))"] {
            let d = analytic_diff(src, "a", &[("a", 1.0), ("b", 1.0)]);
            assert!(
                (d - 0.5).abs() < 1e-12,
                "{src} tie derivative: expected 0.5 split, got {d}"
            );
        }
        // Tie at 0/0 as well (the zero-start NR case).
        let d = analytic_diff("min(V(a), V(b))", "a", &[("a", 0.0), ("b", 0.0)]);
        assert!((d - 0.5).abs() < 1e-12, "min tie at origin: got {d}");

        // sqrt at 0 and below: value is clamped flat (max(0).sqrt()); the
        // guarded derivative 0.5/(sqrt+1e-150) is a huge-but-FINITE cliff
        // (5e149), never +inf/NaN.
        for x in [0.0, -1.0] {
            let d = analytic_diff("sqrt(V(a))", "a", &[("a", x)]);
            assert!(
                d.is_finite() && d > 0.0,
                "sqrt' at {x}: expected finite cliff, got {d}"
            );
        }

        // ln at 0 and below: value pinned at ln(1e-300); derivative
        // 1/max(x,1e-300) = 1e300 — finite, never inf/NaN.
        for x in [0.0, -1.0] {
            let d = analytic_diff("ln(V(a))", "a", &[("a", x)]);
            assert!(d.is_finite(), "ln' at {x}: got {d}");
            assert!(d > 1e299, "ln' at {x}: expected the ~1e300 cliff, got {d}");
        }

        // exp outside the ±40 clamp window: value is flat, slope must be
        // exactly 0 (paired ExpDeriv) — and match the (flat) numeric diff.
        for x in [45.0, -45.0] {
            let d = analytic_diff("exp(V(a))", "a", &[("a", x)]);
            assert_eq!(d, 0.0, "exp' at {x} (clamped region): got {d}");
        }
        // Just inside the window, value and slope agree (relative check —
        // |slope| ≈ 8.7e16 at x=39).
        check_diff_rel("exp(V(a))", "a", &[("a", 39.0)], 1e-6);
        check_diff_rel("exp(V(a))", "a", &[("a", -39.0)], 1e-6);

        // pow(a, 1.5) at a = -0.5: |a|^p branch, finite, matches numeric.
        check_diff_rel("pow(V(a), 1.5)", "a", &[("a", -0.5)], 1e-4);
        // pow(a, 2) at a = 0: 2·a = 0, no NaN.
        assert_eq!(analytic_diff("pow(V(a), 2)", "a", &[("a", 0.0)]), 0.0);
        assert_eq!(analytic_diff("V(a)^2", "a", &[("a", 0.0)]), 0.0);
        // pow(a, 1.5) at a = 0: guarded a/(a²+eps) form → exactly 0, not NaN.
        assert_eq!(analytic_diff("pow(V(a), 1.5)", "a", &[("a", 0.0)]), 0.0);
        // pow with a variable exponent at base 2 (ln|a| path).
        check_diff_rel("pow(2, V(b))", "b", &[("b", 1.3)], 1e-6);
        // Variable exponent at a = 0: both the ln term (ln guard) and the
        // b·da/a term (a²+eps guard) must stay finite.
        for node in ["a", "b"] {
            let d = analytic_diff("pow(V(a), V(b))", node, &[("a", 0.0), ("b", 2.0)]);
            assert!(!d.is_nan(), "pow(0, 2) d/d{node}: got NaN");
            assert!(d.is_finite(), "pow(0, 2) d/d{node}: got {d}");
        }

        // Division derivative at b = 0: (a'b - ab')/(b²+1e-300) — the 0/0
        // case is 0, the a·b' case is huge-but-finite; never NaN.
        let d = analytic_diff("V(a) / V(b)", "a", &[("a", 1.0), ("b", 0.0)]);
        assert_eq!(d, 0.0);
        let d = analytic_diff("V(a) / V(b)", "b", &[("a", 1.0), ("b", 0.0)]);
        assert!(!d.is_nan() && d.is_finite(), "div' at b=0: got {d}");

        // pwr at 0: guarded to the subgradient 0 (value is flat-odd there).
        assert_eq!(analytic_diff("pwr(V(a), 1.5)", "a", &[("a", 0.0)]), 0.0);
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
    fn diff_jacobian_lags_ddt() {
        // Full diff of ddt(V(a)) uses inv_dt; the Jacobian variant zeros it.
        let e = parse("ddt(V(a))");
        let mut ctx = MapCtx::new().with("a", 1.0);
        ctx.inv_dt = 9999.0;
        assert!(e.diff(&Var::Node("a".into())).simplify().eval(&ctx) != 0.0);
        assert!(e
            .diff_jacobian(&Var::Node("a".into()))
            .simplify()
            .eval(&ctx)
            .abs()
            < 1e-12);

        // In a product, ddt acts as a constant FACTOR: ∂/∂a [V(a)*ddt(V(b))] =
        // ddt(V(b)) (the value is kept), not 0.
        let p = parse("V(a) * ddt(V(b))");
        let mut ctx2 = MapCtx::new().with("a", 2.0).with("b", 3.0);
        ctx2.inv_dt = 100.0;
        ctx2.x_prev.insert(0, 1.0); // ddt(V(b)) = (3 - 1)*100 = 200
        let jac = p.diff_jacobian(&Var::Node("a".into())).simplify();
        assert!((jac.eval(&ctx2) - 200.0).abs() < 1e-9, "got {}", jac.eval(&ctx2));
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

    /// Shared emission-test resolver: a→v[1], b→v[2], ground→0.0.
    struct TestResolver;
    impl ExprResolver for TestResolver {
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

    #[test]
    fn emitter_produces_compilable_strings() {
        let e = parse("tanh(V(a)) * V(b) + ddt(V(a))");
        let s = e.to_rust(&TestResolver);
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

    /// The emitted Rust must be self-contained: every guard inline, no
    /// references to helper functions that no emitter defines.
    #[test]
    fn emitted_guards_are_self_contained() {
        let r = TestResolver;

        // exp: inline clamp, NOT the never-emitted bsrc_safe_exp helper.
        let s = parse("exp(V(a))").to_rust(&r);
        assert!(s.contains(".clamp(-40.0, 40.0).exp()"), "exp emission: {s}");
        assert!(!s.contains("bsrc_safe_exp"), "exp must not call an undefined helper: {s}");

        // exp derivative: clamp-gated (0 outside the window), same window.
        let d = parse("exp(V(a))")
            .diff(&Var::Node("a".into()))
            .simplify()
            .to_rust(&r);
        assert!(
            d.contains(">= -40.0") && d.contains("<= 40.0"),
            "exp' emission must gate on the clamp window: {d}"
        );

        // abs derivative: branchless subgradient sign, no bare x/abs(x).
        let d = parse("abs(V(a))")
            .diff(&Var::Node("a".into()))
            .simplify()
            .to_rust(&r);
        assert!(d.contains("1e-300"), "abs' emission must carry the sign guard: {d}");

        // sqrt derivative: guarded denominator.
        let d = parse("sqrt(V(a))")
            .diff(&Var::Node("a".into()))
            .simplify()
            .to_rust(&r);
        assert!(d.contains("1e-150"), "sqrt' emission must carry the 1e-150 guard: {d}");

        // ln derivative: guarded 1/max(x, 1e-300).
        let d = parse("ln(V(a))")
            .diff(&Var::Node("a".into()))
            .simplify()
            .to_rust(&r);
        assert!(d.contains(".max(1e-300)"), "ln' emission must be guarded: {d}");

        // pow with constant integer exponent → sign-correct powi.
        let s = parse("V(a)^3").to_rust(&r);
        assert!(s.contains(".powi(3)"), "integer-const pow should emit powi: {s}");
        // pow with constant fractional exponent → |a|^p.
        let s = parse("pow(V(a), 1.5)").to_rust(&r);
        assert!(s.contains(".abs().powf(1.5"), "fractional-const pow should emit |a|^p: {s}");
        // pow with a variable exponent → inline PTpower block.
        let s = parse("pow(V(a), V(b))").to_rust(&r);
        assert!(s.contains("bsa < 0.0"), "variable-exp pow should emit the PTpower block: {s}");
        // pwr → inline PTpwr block.
        let s = parse("pwr(V(a), 1.5)").to_rust(&r);
        assert!(s.contains("-((-bsa).powf(bsb))"), "pwr should emit the PTpwr block: {s}");
    }

    /// `fmt_f64` must never emit the bare `inf`/`NaN` tokens (`{:e}` output),
    /// which are not valid Rust literals in generated code.
    #[test]
    fn const_emission_handles_non_finite() {
        let r = TestResolver;
        assert_eq!(Expr::Const(f64::INFINITY).to_rust(&r), "f64::INFINITY");
        assert_eq!(Expr::Const(f64::NEG_INFINITY).to_rust(&r), "f64::NEG_INFINITY");
        assert_eq!(Expr::Const(f64::NAN).to_rust(&r), "f64::NAN");
        // Finite formatting unchanged.
        assert_eq!(Expr::Const(2.0).to_rust(&r), "2.0");
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
