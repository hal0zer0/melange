//! The parsed netlist, its parse options, and the shared netlist record types.

use super::*;

/// A parsed SPICE netlist.
#[derive(Debug, Clone, PartialEq)]
pub struct Netlist {
    /// Circuit title (first line)
    pub title: String,
    /// Circuit elements
    pub elements: Vec<Element>,
    /// Model definitions
    pub models: Vec<Model>,
    /// Subcircuit definitions
    pub subcircuits: Vec<Subcircuit>,
    /// Global parameters
    pub params: Vec<Parameter>,
    /// Potentiometer directives (.pot)
    pub pots: Vec<PotDirective>,
    /// Switch directives (.switch)
    pub switches: Vec<SwitchDirective>,
    /// Wiper potentiometer directives (.wiper)
    pub wipers: Vec<WiperDirective>,
    /// Gang directives (.gang) — links multiple pots/wipers to one parameter
    pub gangs: Vec<GangDirective>,
    /// Coupling directives (K elements for coupled inductors / transformers)
    pub couplings: Vec<CouplingDirective>,
    /// Delay feedback node names (.delay_feedback node1 node2 ...)
    /// These nodes are frozen at their previous-sample values during NR iteration,
    /// breaking feedback loops through transformers.
    pub delay_feedback_nodes: Vec<String>,
    /// Input impedance directive (.input_impedance)
    pub input_impedance: Option<f64>,
    /// Linearize directives (.linearize Q9)
    /// BJTs listed here are linearized at their DC operating point:
    /// small-signal conductances (gm, gpi, gmu) are stamped into G,
    /// DC bias currents added to rhs_const, and the BJT is removed
    /// from the nonlinear system (M dimension reduced by 2 per device).
    pub linearize_devices: Vec<String>,
    /// Runtime voltage source directives (.runtime V1 as field_name)
    /// Each entry marks an existing voltage source whose value the plugin
    /// host will update every sample. Codegen emits `pub <field>: f64` on
    /// CircuitState and stamps `rhs[VSOURCE_<NAME>_RHS_ROW] += state.<field>`
    /// in both trapezoidal and backward-Euler RHS builders.
    pub runtime_sources: Vec<RuntimeDirective>,
    /// Runtime resistor directives (.runtime R1 min max as field_name).
    /// Audio-rate resistor modulation for envelope-linked bias (Latinum §5(b)).
    /// Shares the 64-slot pot table at MNA level; distinguished from `.pot` at
    /// codegen by the API shape (no nih-plug knob, `state.<field>()` accessor).
    /// Setter bodies are identical to `.pot` since the 2026-04-20 reseed strip.
    pub runtime_resistors: Vec<RuntimeResistorDirective>,
    /// Bare plugin-driven scalar params (`.runtime <name> <min> <max> as
    /// <field>`), referenced by name in behavioral `B`-source expressions.
    pub runtime_scalars: Vec<RuntimeScalarDirective>,
    /// Runtime feedback-injection sources (`.inject <node> <field>
    /// R=/RSHUNT=`). Each stamps a Thevenin/Norton source at a circuit node,
    /// driven by a `process_sample` argument at its declared rate (host or
    /// inner).
    pub injections: Vec<InjectDirective>,
    /// Raw inner-rate tap probes (`.tap <node> [name]`). `process_sample`
    /// returns per-inner-sample tap values so a feedback caller can run its
    /// inner-rate model.
    pub taps: Vec<TapDirective>,
    /// Declared board pins (`.port <node> ...`). Direction-neutral: a pin is
    /// a place the outside world connects to this board, whether it is driven
    /// from outside, read from outside, or both.
    ///
    /// Consumed ONLY by [`crate::topology`]: a declared pin counts as one
    /// connection for the dangling-node check and as nothing else. It is not a
    /// DC path, stamps nothing, and never reaches the MNA, the DK kernel or
    /// codegen — a deck's generated code is byte-identical with and without
    /// its `.port` lines (`port_declaration_has_zero_codegen_effect`).
    pub ports: Vec<PortDirective>,
    /// Per-device parameter mismatch directives (.mismatch D IS=0.02 ...).
    /// Applied at codegen time: each device of the listed type gets its
    /// nominal model parameter jittered by `nominal · (1 + tol · u)` with
    /// `u ∈ [-1, 1]` drawn deterministically from a seed hashed from
    /// `(netlist.seed, device_name, param_name)`. Default is no mismatch —
    /// the jitter only fires when the directive is present.
    pub mismatch_specs: Vec<MismatchSpec>,
    /// Mismatch RNG master seed (.seed 12345). When `None`, mismatch draws
    /// use seed 0. Unused when `mismatch_specs` is empty.
    pub seed: Option<u64>,
    /// Fixed-resistor value tolerance (`.tolerance R=0.01`). Applied once
    /// at the end of parse: every `R` element that isn't claimed by a
    /// `.pot` / `.wiper` / `.switch` / `.runtime R` directive gets its
    /// value multiplied by `(1 + tol · u)` with `u ∈ [-1, 1]` drawn from
    /// the same deterministic RNG stream as `.mismatch`, separated by
    /// class tag so R/C/L draws don't alias. Default `0.0` = disabled.
    pub tolerance_r: f64,
    /// Fixed-capacitor value tolerance (`.tolerance C=0.01`). Same mechanics
    /// as `tolerance_r`, applied to `Element::Capacitor` entries that
    /// aren't controlled by a `.switch`.
    pub tolerance_c: f64,
    /// Fixed-inductor value tolerance (`.tolerance L=0.01`). Same mechanics
    /// as `tolerance_r`, applied to `Element::Inductor` entries that
    /// aren't controlled by a `.switch`. Also jitters each coupled-inductor
    /// winding independently — which happens to match real transformer
    /// turn-count variation.
    pub tolerance_l: f64,
    /// Unit-variation kill switch: when `true`, `.seed` / `.mismatch` /
    /// `.tolerance` are still PARSED and recorded on this netlist, but the
    /// draw is never applied — every value and device parameter stays at its
    /// nominal, as-written magnitude.
    ///
    /// Set only by [`Netlist::parse_with_options`] via
    /// [`ParseOptions::disable_unit_variation`]. It is carried on the netlist
    /// rather than on a codegen config because the two apply sites are on
    /// opposite sides of the pipeline — `.tolerance` lands in
    /// [`Netlist::apply_passive_tolerance`] at the end of parse, `.mismatch`
    /// in `CircuitIR::mismatch_tol_for` during codegen — and a single flag on
    /// the object that carries the directives cannot desynchronize between
    /// them. Both sites read it; nothing else does.
    ///
    /// The one consumer is `melange validate`: ngspice sees the deck's nominal
    /// values, so a jittered melange side would correlate two different
    /// circuits and blame the gap on the solver. `compile` / `simulate` /
    /// `analyze` leave this `false` and jitter as documented.
    pub unit_variation_disabled: bool,
    /// Every device is resolved without self-heating (`RTH` infinite; `TAMB`
    /// still sets the static device temperature). Set only via
    /// [`ParseOptions::disable_self_heating`]; the one consumer is `melange
    /// validate`, whose reference simulator has no thermal model.
    pub self_heating_disabled: bool,
    /// Integration-scheme preference (`.integrator trap` / `.integrator be`).
    /// `None` (default) leaves the choice to the CLI flags and the automatic
    /// spectral-radius promotion. `Some(Be)` pins backward Euler at compile
    /// time (like `--backward-euler`); `Some(Trap)` pins trapezoidal and
    /// suppresses auto-promotion AND the runtime BE-latch safety net (like
    /// `--force-trap`). An explicit CLI flag always overrides the directive.
    ///
    /// This lets a netlist author deterministically pin the integrator a
    /// circuit was validated with, so a routine fleet regen can't silently
    /// change it out from under a shipped plugin.
    pub integrator: Option<IntegratorPref>,
    /// Recommended oversampling factor (`.oversampling 2` / `4`). This is an
    /// accuracy MINIMUM / recommendation, NOT a mandate: oversampling controls
    /// aliasing from nonlinear distortion products, but the rate costs CPU and
    /// latency, which is the plugin author's (downstream) product decision.
    ///
    /// Resolution on the shipping path (compile / simulate / analyze):
    /// `effective = explicit_cli.unwrap_or(recommended_oversampling).unwrap_or(1)`.
    /// An explicit `--oversampling` on the command line always wins — even when
    /// it is LOWER than the deck value (a warning is logged in that case). The
    /// `validate` path IGNORES this field entirely: an oversampled comparison
    /// is confounded by anti-alias-filter group delay, so validate stays at the
    /// base rate regardless of the directive. Values are restricted to {1,2,4}
    /// to match the `--oversampling` cap. `None` (default) means unspecified.
    pub recommended_oversampling: Option<usize>,
    /// 1-based RAW source line each element was declared on, keyed by the
    /// lowercased element name.
    ///
    /// Recorded during parse so a post-parse diagnostic can point at the line
    /// the author wrote rather than at the statement the parser happened to be
    /// on. Elements created by [`Netlist::expand_subcircuits`] have no authored
    /// line of their own and are absent from this map; a lookup that misses
    /// yields no location at all rather than a wrong one.
    ///
    /// Only the FIRST declaration line is kept. Duplicate element names are a
    /// parse error, so a second entry can only exist inside a deck that never
    /// finishes parsing.
    pub element_lines: std::collections::HashMap<String, usize>,
}

/// Compile-time integration-scheme pin set by the `.integrator` directive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegratorPref {
    /// `.integrator trap` — force trapezoidal (also suppresses the runtime
    /// BE-latch safety net; equivalent to `--force-trap`).
    Trap,
    /// `.integrator be` — force backward Euler (equivalent to `--backward-euler`).
    Be,
}

/// Knobs that change what [`Netlist::parse_with_options`] *does* with what it
/// reads. Every field defaults to the shipped `Netlist::parse` behavior.
///
/// This is not a place for circuit options — those belong in the netlist. It is
/// for the handful of cases where the same deck must be read two ways by two
/// parts of melange.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ParseOptions {
    /// Read `.seed` / `.mismatch` / `.tolerance` but do not apply the draw:
    /// every R/C/L value and every device model parameter stays nominal.
    ///
    /// `melange validate` sets this. The reference deck handed to ngspice
    /// carries the values as written, so a jittered melange side would put a
    /// correlation between two different circuits on the result line and
    /// attribute the difference to the solver. Validate measures the solver
    /// against a reference engine at the same component values; whether the
    /// *draw* itself is right is a unit-test question (see
    /// `tests::tolerance_draw_matches_nominal_times_one_plus_tol_u`), not one
    /// ngspice can answer.
    ///
    /// The directives stay recorded on the returned [`Netlist`] so the caller
    /// can name which ones it disabled.
    pub disable_unit_variation: bool,
    /// Resolve every device isothermal: a card's `RTH` (and so `CTH`) is not
    /// applied, while `TAMB` still sets the device's static temperature.
    ///
    /// `melange validate` sets this. ngspice's diode, BJT and the triode twin
    /// have no thermal model, so a self-heating melange side would compare a
    /// different circuit; self-heating's own correctness is covered by its
    /// analytic tests (`Tj_ss = TAMB + P·RTH`, `tau = RTH·CTH`).
    pub disable_self_heating: bool,
}

/// Deterministic uniform `[-1, 1]` draw from `(seed, class_tag, name)`.
///
/// FNV-64 feeds a SplitMix64 finalizer, giving well-decorrelated output
/// even for adjacent seeds / short strings. The high 53 bits of the
/// finalizer become a uniform double in `[0, 1)`, then mapped to
/// `[-1, 1]`. A null-byte separator between `class_tag` and `name`
/// prevents `"R" + "C1"` from colliding with `"RC" + "1"`.
pub(crate) fn deterministic_draw(seed: u64, class_tag: &str, name: &str) -> f64 {
    const FNV_PRIME: u64 = 0x100000001b3;
    let mut h = seed ^ 0xcbf29ce484222325u64;
    for b in class_tag.as_bytes() {
        h = (h ^ (b.to_ascii_uppercase() as u64)).wrapping_mul(FNV_PRIME);
    }
    // Null-byte separator.
    h = h.wrapping_mul(FNV_PRIME);
    for b in name.as_bytes() {
        h = (h ^ (b.to_ascii_uppercase() as u64)).wrapping_mul(FNV_PRIME);
    }
    // SplitMix64 finalizer.
    h = (h ^ (h >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    h = (h ^ (h >> 27)).wrapping_mul(0x94d049bb133111eb);
    h ^= h >> 31;
    let u01 = (h >> 11) as f64 / (1u64 << 53) as f64;
    2.0 * u01 - 1.0
}

impl Netlist {
    /// Create an empty netlist.
    pub fn new(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            elements: Vec::new(),
            models: Vec::new(),
            subcircuits: Vec::new(),
            params: Vec::new(),
            pots: Vec::new(),
            switches: Vec::new(),
            wipers: Vec::new(),
            gangs: Vec::new(),
            couplings: Vec::new(),
            delay_feedback_nodes: Vec::new(),
            input_impedance: None,
            linearize_devices: Vec::new(),
            runtime_sources: Vec::new(),
            runtime_resistors: Vec::new(),
            runtime_scalars: Vec::new(),
            injections: Vec::new(),
            taps: Vec::new(),
            ports: Vec::new(),
            mismatch_specs: Vec::new(),
            seed: None,
            tolerance_r: 0.0,
            tolerance_c: 0.0,
            tolerance_l: 0.0,
            unit_variation_disabled: false,
            self_heating_disabled: false,
            integrator: None,
            recommended_oversampling: None,
            element_lines: std::collections::HashMap::new(),
        }
    }

    /// Apply `.tolerance` jitter to every fixed R/C/L whose value isn't
    /// externally controlled. Called automatically at the end of
    /// [`Netlist::parse`]; only acts when at least one tolerance is
    /// nonzero. Idempotent-safe to call multiple times only if callers
    /// first reset `tolerance_r/c/l` to zero — otherwise the jitter
    /// compounds.
    ///
    /// Skipped components:
    /// - any resistor named by a `.pot` (wiper halves appear as `.pot`
    ///   entries after `expand_wipers`)
    /// - any resistor named by a `.runtime R`
    /// - any component (R, C, or L) named by a `.switch`
    ///
    /// The RNG is the same FNV → SplitMix64 chain used by `.mismatch`,
    /// seeded from `(self.seed, "R"|"C"|"L", component_name)`, so
    /// different seeds produce different unit personalities and the R/C/L
    /// streams can't alias each other.
    pub fn apply_passive_tolerance(&mut self) {
        // The unit-variation kill switch is checked HERE, not at the call site
        // in `Parser::parse`, so it cannot be bypassed by any other caller of
        // this method. `.tolerance` stays recorded on the netlist either way —
        // callers still need to know the directive was present in order to say
        // so. See `Netlist::unit_variation_disabled`.
        if self.unit_variation_disabled {
            return;
        }
        if self.tolerance_r == 0.0 && self.tolerance_c == 0.0 && self.tolerance_l == 0.0 {
            return;
        }
        let seed = self.seed.unwrap_or(0);

        // Build the skip set once. Names are compared case-insensitively
        // because SPICE identifiers are case-insensitive.
        let mut skip: std::collections::HashSet<String> = std::collections::HashSet::new();
        for p in &self.pots {
            skip.insert(p.resistor_name.to_ascii_lowercase());
        }
        for r in &self.runtime_resistors {
            skip.insert(r.resistor_name.to_ascii_lowercase());
        }
        for sw in &self.switches {
            for n in &sw.component_names {
                skip.insert(n.to_ascii_lowercase());
            }
        }

        let tol_r = self.tolerance_r;
        let tol_c = self.tolerance_c;
        let tol_l = self.tolerance_l;

        for elem in self.elements.iter_mut() {
            match elem {
                Element::Resistor { name, value, .. } if tol_r > 0.0 => {
                    if skip.contains(&name.to_ascii_lowercase()) {
                        continue;
                    }
                    let u = deterministic_draw(seed, "R", name);
                    *value *= 1.0 + tol_r * u;
                }
                Element::Capacitor { name, value, .. } if tol_c > 0.0 => {
                    if skip.contains(&name.to_ascii_lowercase()) {
                        continue;
                    }
                    let u = deterministic_draw(seed, "C", name);
                    *value *= 1.0 + tol_c * u;
                }
                Element::Inductor { name, value, .. } if tol_l > 0.0 => {
                    if skip.contains(&name.to_ascii_lowercase()) {
                        continue;
                    }
                    let u = deterministic_draw(seed, "L", name);
                    *value *= 1.0 + tol_l * u;
                }
                _ => {}
            }
        }
    }

    /// Parse a netlist from a string.
    ///
    /// # Errors
    ///
    /// Returns [`ParseError`] if:
    /// - `input.len() > MAX_NETLIST_BYTES` (checked before any allocation)
    /// - any node name exceeds `MAX_NODE_NAME_LEN` chars
    /// - more than `MAX_TOTAL_ELEMENTS` elements, `MAX_MODELS` models,
    ///   or `MAX_MODEL_PARAMS` params per model are declared
    /// - a line fails syntactic validation
    pub fn parse(input: &str) -> Result<Self, ParseError> {
        Self::parse_with_options(input, ParseOptions::default())
    }

    /// Parse a netlist with non-default parse behavior.
    ///
    /// [`Netlist::parse`] is this with [`ParseOptions::default()`], which is
    /// the shipped behavior in every respect. The only option today is
    /// [`ParseOptions::disable_unit_variation`], used by `melange validate` so
    /// the melange side is built at the same nominal component values the
    /// ngspice reference deck gets.
    ///
    /// # Errors
    ///
    /// Same as [`Netlist::parse`].
    pub fn parse_with_options(input: &str, options: ParseOptions) -> Result<Self, ParseError> {
        // Defensive size cap: reject malicious/oversized input before any allocation.
        if input.len() > MAX_NETLIST_BYTES {
            return Err(ParseError {
                line: 0,
                message: format!(
                    "netlist too large: {} bytes exceeds MAX_NETLIST_BYTES ({})",
                    input.len(),
                    MAX_NETLIST_BYTES
                ),
            });
        }
        Parser::new(input).parse(options)
    }
}

/// A model definition (.model).
#[derive(Debug, Clone, PartialEq)]
pub struct Model {
    pub name: String,
    pub model_type: String,
    pub params: Vec<(String, f64)>,
}

/// A subcircuit definition (.subckt).
#[derive(Debug, Clone, PartialEq)]
pub struct Subcircuit {
    pub name: String,
    pub nodes: Vec<String>,
    pub elements: Vec<Element>,
}

/// A parameter definition (.param).
#[derive(Debug, Clone, PartialEq)]
pub struct Parameter {
    pub name: String,
    pub value: f64,
}

/// Parse error.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct ParseError {
    /// 1-based line number in the **raw** netlist source (continuation lines
    /// counted individually, so it matches what an editor shows).
    ///
    /// `0` means "no single source line is responsible" — a whole-file limit
    /// (`MAX_NETLIST_BYTES`), or a post-expansion condition that no longer
    /// belongs to one authored line. [`Display`](std::fmt::Display) omits the
    /// line entirely in that case rather than printing a bogus "line 0", which
    /// read as a real location and sent readers counting lines by hand.
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.line == 0 {
            write!(f, "Parse error: {}", self.message)
        } else {
            write!(f, "Parse error at line {}: {}", self.line, self.message)
        }
    }
}

impl std::error::Error for ParseError {}
