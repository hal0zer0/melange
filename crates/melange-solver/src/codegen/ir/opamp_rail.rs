//! Op-amp supply-rail mode resolution + Boyle-diode netlist augmentation.
//!
//! Resolves `OpampRailMode::Auto` to a concrete mode by inspecting the parsed
//! MNA system (audio-path topology, AC coupling, DC-rail connections). The
//! `augment_netlist_with_boyle_diodes` pass synthesises catch-diodes around
//! VCCS op-amp outputs when the resolved mode requires them, so the resulting
//! netlist matches the on-rail "Boyle" macro-model exactly.

pub(crate) fn default_opamp_rail_mode() -> crate::codegen::OpampRailMode {
    // Deserialized IRs without an explicit mode default to the pre-2026-04
    // behavior (hard clamp). `Auto` is only valid as a user-facing input;
    // by the time an IR is constructed, the mode is always concrete.
    crate::codegen::OpampRailMode::Hard
}

/// Reason recorded for an auto-detection decision. Used for logging so users
/// can see *why* a particular mode was chosen. Each variant carries enough
/// information to be reconstructable from the MNA alone — no references to
/// specific circuit names.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OpampRailModeReason {
    /// User explicitly specified the mode; no auto-detection ran.
    UserRequested,
    /// Circuit has no op-amps with finite supply rails, so no clamping is needed.
    NoClampedOpamps,
    /// All op-amp outputs are DC-coupled to their downstream networks. The
    /// hard post-NR clamp is safe — no cap history to corrupt — and cheap.
    AllDcCoupled,
    /// At least one clamped op-amp has an output cap coupling into a downstream
    /// non-feedback node. The post-NR hard clamp would corrupt that cap's
    /// trapezoidal history on rail-violating samples. Auto-select `ActiveSet`
    /// to keep KCL consistent. (`BoyleDiodes` has since landed as an explicit
    /// opt-in mode, but the auto-resolver never picks it — it diverges at
    /// heavy clip; see `docs/aidocs/OPAMP_RAIL_MODES.md`.)
    AcCoupledDownstream,
    /// At least one clamped op-amp output is connected via an R-only path
    /// (no series caps) to a nonlinear device terminal. This is the
    /// sidechain / control-path topology pattern (e.g. compressor envelope
    /// detector: op-amp output → R → diode anode → cap to ground). The
    /// rail-clamped DC value drives the nonlinear device's operating point,
    /// so any damping (BE switch on engagement) would change the envelope
    /// dynamics. Use plain `ActiveSet` (trap+pin) which preserves the steady
    /// rail value.
    AcCoupledDownstreamControlPath,
    /// All clamped op-amps have only AC-coupled downstream paths to
    /// nonlinear devices (or no nonlinear devices at all). This is the
    /// audio-path topology where the op-amp drives a coupling cap to the
    /// next stage. Trap+pin develops a Nyquist-rate limit cycle on these
    /// circuits when the rail is engaged across multiple samples; switch to
    /// `ActiveSetBe` so the BE fallback runs the constrained re-solve and
    /// damps the ringing.
    AcCoupledDownstreamAudioPath,
}

impl OpampRailModeReason {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::UserRequested => "user requested",
            Self::NoClampedOpamps => "no op-amps with finite VCC/VEE",
            Self::AllDcCoupled => "op-amps only DC-coupled downstream (hard clamp safe)",
            Self::AcCoupledDownstream => "op-amp output has cap-coupled downstream (active-set keeps cap history consistent)",
            Self::AcCoupledDownstreamControlPath => {
                "op-amp output has R-only path to a nonlinear device terminal (sidechain/control path — trap+pin preserves steady DC rail)"
            }
            Self::AcCoupledDownstreamAudioPath => {
                "op-amp output is cap-coupled with no R-only path to nonlinear devices (audio path — BE-on-clamp damps Nyquist limit cycle)"
            }
        }
    }
}

/// Result of [`resolve_opamp_rail_mode`]: the concrete mode that should be
/// baked into the generated code, plus the reason it was chosen.
#[derive(Debug, Clone)]
pub struct ResolvedOpampRailMode {
    pub mode: crate::codegen::OpampRailMode,
    pub reason: OpampRailModeReason,
}

/// Returns `true` if the op-amp output node `out_idx` (1-indexed) has any
/// capacitive coupling to a non-feedback node in the MNA `C` matrix.
///
/// "Non-feedback" means: not the op-amp's own inverting input, not ground,
/// and not the out node itself. Feedback caps (between `out` and the `-`
/// input) are fine for the hard-clamp path because they're not downstream
/// integrators — they're part of the op-amp's own closed loop. Output
/// coupling caps (between `out` and a different stage's input) are the
/// ones that corrupt cap history when the op-amp is post-clamped.
///
/// Returns `false` if the op-amp has `out_idx == 0` (grounded / invalid).
fn opamp_has_ac_coupled_downstream(
    mna: &crate::mna::MnaSystem,
    opamp: &crate::mna::OpampInfo,
) -> bool {
    if opamp.n_out_idx == 0 {
        return false;
    }
    let out = opamp.n_out_idx - 1; // 0-indexed
                                   // Optional: the op-amp's own inverting input (for skipping feedback caps).
                                   // 0 means grounded in the MNA's 1-indexed convention.
    let inverting_input: Option<usize> = if opamp.n_minus_idx > 0 {
        Some(opamp.n_minus_idx - 1)
    } else {
        None
    };

    // C matrix is NxN (where N is mna.n, the current MNA dimension). A cap
    // between nodes i and j stamps into C[i][i], C[j][j], C[i][j], C[j][i].
    // We scan row `out` of C for non-zero off-diagonal entries.
    //
    // Known blind spot: a cap from the op-amp output straight to GROUND
    // stamps ONLY the diagonal C[out][out] (ground has no matrix row), and
    // the scan skips the diagonal — so out-to-ground caps are invisible
    // here. That's acceptable for this detector's purpose: the hard-clamp
    // corruption it guards against needs a FAR-SIDE node that integrates
    // the clamp-induced discontinuity (a series coupling cap whose other
    // plate drifts as `(2C/T)·v_prev` history goes inconsistent). A
    // grounded cap's other plate is fixed at 0 V; its stored voltage IS
    // the output-node voltage, so clamping v[out] leaves that cap's
    // history self-consistent — worst case is a one-sample transient at
    // the output node itself, not a drifting downstream integrator.
    let n = mna.c.len();
    if out >= n {
        return false;
    }
    let row = &mna.c[out];
    for (other, &c) in row.iter().enumerate() {
        if other == out {
            continue; // self-diagonal
        }
        if c == 0.0 {
            continue;
        }
        if let Some(nm) = inverting_input {
            if other == nm {
                // Feedback cap — doesn't count as downstream coupling.
                continue;
            }
        }
        // Any other non-zero entry is a coupling cap to another circuit node.
        return true;
    }
    false
}

/// Returns `true` if the op-amp's output node has an R-only path (no series
/// capacitors) to any nonlinear device terminal.
///
/// This is the **control-path / sidechain criterion**: when the op-amp output
/// is wired through resistors directly to a diode/BJT/JFET/MOSFET/tube/VCA
/// terminal, the rail-clamped DC value drives that device's operating point.
/// Switching to BE on engagement (the [`OpampRailMode::ActiveSetBe`] strategy)
/// would damp this DC behavior and change the envelope dynamics — wrong for
/// compressors, expanders, ALCs, and any sidechain rectifier. So if any
/// clampable op-amp matches this pattern, the auto-detector picks plain
/// [`OpampRailMode::ActiveSet`] (trap+pin), which preserves the rail value.
///
/// **What "R-only path" means**: BFS from the op-amp output through resistor
/// edges only. We never traverse capacitor edges. The op-amp's own inverting
/// input is excluded (resistor feedback through the closed loop is not a
/// downstream control path). Ground is excluded.
///
/// **What "nonlinear device terminal" means**: any node listed in
/// [`crate::mna::NonlinearDeviceInfo::node_indices`] for any device in
/// `mna.nonlinear_devices` — diode anode/cathode, BJT base/collector/emitter,
/// JFET/MOSFET gate/drain/source, tube grid/plate/cathode, VCA signal/control
/// pins.
///
/// **Bound**: BFS depth is bounded by the number of nodes (no infinite loops
/// possible — the visited set guarantees termination in O(N + E) where N is
/// nodes and E is resistor count).
///
/// **Why R-only and not "any linear element"**: caps in series provide
/// AC-coupling that decouples the DC rail value from the downstream device's
/// operating point. With a series cap, the cap charges to the rail offset
/// and the downstream node sees only AC excursion. The rail value's specific
/// magnitude doesn't affect the downstream operating point. Without caps,
/// the rail value IS what the downstream node sees at DC.
///
/// [`OpampRailMode::ActiveSet`]: crate::codegen::OpampRailMode::ActiveSet
/// [`OpampRailMode::ActiveSetBe`]: crate::codegen::OpampRailMode::ActiveSetBe
fn opamp_has_r_only_path_to_nonlinear(
    netlist: &crate::parser::Netlist,
    mna: &crate::mna::MnaSystem,
    opamp: &crate::mna::OpampInfo,
) -> bool {
    use crate::parser::Element;
    use std::collections::HashSet;

    if opamp.n_out_idx == 0 {
        return false;
    }

    // Resolve the op-amp output and inverting-input node names from indices.
    // We use names because netlist Elements are name-keyed.
    let name_of_idx = |idx: usize| -> Option<String> {
        if idx == 0 {
            return None;
        }
        mna.node_map
            .iter()
            .find_map(|(name, &i)| if i == idx { Some(name.clone()) } else { None })
    };
    let out_name = match name_of_idx(opamp.n_out_idx) {
        Some(n) => n,
        None => return false,
    };
    let inv_name = name_of_idx(opamp.n_minus_idx); // None if grounded

    // Collect names of all nodes that are DC voltage source terminals. These
    // are bias rails (Vbias, VCC, VEE, etc.) — KCL-clamped to a fixed
    // potential by an ideal source, so they're effectively ground for AC
    // purposes. The BFS treats them as stop nodes: we don't traverse through
    // them, and we don't count them as nonlinear-device terminals even if a
    // diode happens to anchor to them.
    //
    // Why this matters: in distortion pedals, shunt clipping diodes are
    // anchored to a DC bias node (e.g. Klon: D2 anode = diode_jct, cathode =
    // vbias). The bias rail is a voltage source. If we counted vbias as a
    // "nonlinear terminal", every R-network path to the bias rail would
    // falsely register as a control path, because every op-amp's bias path
    // touches the rail somewhere. The DC level on the op-amp output isn't
    // actually driving the diode through that path — the diode is referenced
    // to vbias, not to the op-amp signal.
    let dc_source_nodes: HashSet<String> = netlist
        .elements
        .iter()
        .filter_map(|e| {
            if let Element::VoltageSource {
                n_plus, n_minus, ..
            } = e
            {
                Some([n_plus.clone(), n_minus.clone()])
            } else {
                None
            }
        })
        .flatten()
        .filter(|n| n != "0")
        .collect();

    // Collect names of all nodes that are nonlinear-device terminals AND not
    // DC source nodes. The exclusion prevents the false positive described
    // above.
    let nonlinear_terminal_names: HashSet<&String> = mna
        .nonlinear_devices
        .iter()
        .flat_map(|d| d.nodes.iter())
        .filter(|n| !dc_source_nodes.contains(n.as_str()))
        .collect();

    // BFS from the op-amp output through resistor edges only. Stop at:
    // - ground ("0")
    // - the op-amp's own inverting input (feedback loop)
    // - any DC voltage source node (bias rails are AC-ground)
    //
    // The starting node is NEVER counted as a downstream terminal. A
    // nonlinear device directly anchored to the op-amp output is a local
    // feedback element (classic diode-feedback clipper: D1 anode = op-amp
    // output, cathode = inv input) — not a control-path driver. A real
    // control path requires at least one R-hop so the op-amp drives the
    // downstream device through a signal-path resistor (e.g. VCR ALC
    // sidechain: `Rsc op_out sc_node 10k; D1 sc_node cv_node`). Tracking
    // the starting node explicitly via `start_node` keeps the BFS
    // conditional simple.
    let start_node = out_name.clone();
    let mut visited: HashSet<String> = HashSet::new();
    let mut frontier: Vec<String> = vec![out_name.clone()];
    visited.insert(out_name);

    while let Some(node) = frontier.pop() {
        // If this node is itself a (filtered) nonlinear-device terminal,
        // we've found a real control path — but only if we reached it
        // via an R-hop from the op-amp output, not as the seed itself.
        if node != start_node && nonlinear_terminal_names.contains(&node) {
            return true;
        }

        // Walk every resistor in the netlist looking for one that touches `node`.
        // O(N · E) total over the BFS — fine for typical circuit sizes (E < 1000).
        for elem in &netlist.elements {
            if let Element::Resistor {
                n_plus, n_minus, ..
            } = elem
            {
                let other = if n_plus.eq_ignore_ascii_case(&node) {
                    Some(n_minus.clone())
                } else if n_minus.eq_ignore_ascii_case(&node) {
                    Some(n_plus.clone())
                } else {
                    None
                };

                if let Some(other_node) = other {
                    // Skip ground.
                    if other_node == "0" {
                        continue;
                    }
                    // Skip the op-amp's own inverting input (feedback path).
                    if let Some(inv) = &inv_name {
                        if other_node.eq_ignore_ascii_case(inv) {
                            continue;
                        }
                    }
                    // Skip DC voltage source nodes — bias rails terminate
                    // the BFS because they're AC-ground.
                    if dc_source_nodes.contains(&other_node) {
                        continue;
                    }
                    if !visited.contains(&other_node) {
                        visited.insert(other_node.clone());
                        frontier.push(other_node);
                    }
                }
            }
        }
    }

    false
}

/// Resolve an [`OpampRailMode`] request into a concrete mode using the MNA
/// as the basis for auto-detection.
///
/// # Decision rules (Auto mode)
///
/// 1. **No op-amps with finite VCC/VEE** → [`OpampRailMode::None`]. Nothing
///    to clamp; the user modeled all op-amps as ideal VCCSs.
///
/// 2. **All clamped op-amp outputs are DC-coupled to their downstream**
///    (no output coupling caps, only feedback caps) → [`OpampRailMode::Hard`].
///    The cheap post-NR hard clamp is KCL-safe in this topology because
///    there's no cap history to corrupt.
///
/// 3. **At least one clamped op-amp has a cap coupling to a non-feedback
///    downstream node** → [`OpampRailMode::ActiveSet`]. The hard clamp
///    would corrupt that cap's trapezoidal history on rail-violating
///    samples; the active-set resolve keeps KCL consistent.
///
/// `OpampRailMode::BoyleDiodes` has since landed as an explicit opt-in mode
/// (`--opamp-rail-mode boyle-diodes`, scaffolding in
/// [`augment_netlist_with_boyle_diodes`]) — a Boyle catch-diode model that
/// produces the soft exponential knee characteristic of real op-amp
/// saturation. The auto-resolver deliberately still stops at
/// `ActiveSet`/`ActiveSetBe` and never picks BoyleDiodes: it is validated
/// for light clip only and diverges at heavy clip (see
/// `docs/aidocs/OPAMP_RAIL_MODES.md`, "The BoyleDiodes heavy-clip problem").
///
/// # Explicit user overrides
///
/// Any explicit mode (anything other than [`OpampRailMode::Auto`]) is
/// returned verbatim with [`OpampRailModeReason::UserRequested`]. The
/// resolver NEVER silently upgrades a user's explicit choice — even
/// `None` on a circuit that would otherwise pick `ActiveSet` is honored,
/// because the override is how users bisect and measure.
///
/// [`OpampRailMode`]: crate::codegen::OpampRailMode
/// [`OpampRailMode::None`]: crate::codegen::OpampRailMode::None
/// [`OpampRailMode::Hard`]: crate::codegen::OpampRailMode::Hard
/// [`OpampRailMode::ActiveSet`]: crate::codegen::OpampRailMode::ActiveSet
/// [`OpampRailMode::BoyleDiodes`]: crate::codegen::OpampRailMode::BoyleDiodes
pub fn resolve_opamp_rail_mode(
    mna: &crate::mna::MnaSystem,
    requested: crate::codegen::OpampRailMode,
) -> ResolvedOpampRailMode {
    use crate::codegen::OpampRailMode;

    // Explicit request wins. User overrides exist to bisect and measure; the
    // auto-detector must never silently override a user choice.
    if requested != OpampRailMode::Auto {
        return ResolvedOpampRailMode {
            mode: requested,
            reason: OpampRailModeReason::UserRequested,
        };
    }

    // Collect op-amps that actually need clamping. Infinities (set by the
    // MNA resolver when no rails were declared) mean the user modeled an
    // ideal VCCS — nothing to clamp.
    let clamped_opamps: Vec<&crate::mna::OpampInfo> = mna
        .opamps
        .iter()
        .filter(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()))
        .collect();

    if clamped_opamps.is_empty() {
        return ResolvedOpampRailMode {
            mode: OpampRailMode::None,
            reason: OpampRailModeReason::NoClampedOpamps,
        };
    }

    // Check whether ANY clamped op-amp has a non-feedback output coupling cap.
    // A single offender is enough to force ActiveSet for the whole circuit —
    // we can't mix modes within one generated function, and the KCL corruption
    // only needs one cap to blow up downstream.
    let ac_coupled = clamped_opamps
        .iter()
        .any(|oa| opamp_has_ac_coupled_downstream(mna, oa));

    if ac_coupled {
        ResolvedOpampRailMode {
            mode: OpampRailMode::ActiveSet,
            reason: OpampRailModeReason::AcCoupledDownstream,
        }
    } else {
        ResolvedOpampRailMode {
            mode: OpampRailMode::Hard,
            reason: OpampRailModeReason::AllDcCoupled,
        }
    }
}

/// Refine an [`OpampRailMode::ActiveSet`] auto-decision by inspecting the
/// netlist topology. If no clampable op-amp has an R-only path to a nonlinear
/// device terminal, the circuit is "audio-path" — upgrade to
/// [`OpampRailMode::ActiveSetBe`] which damps the trap+pin Nyquist limit
/// cycle by switching to BE matrices on rail engagement. If at least one
/// op-amp does have such a path, the circuit is "control-path" (sidechain
/// rectifier, compressor envelope detector, etc.) and stays on plain
/// [`OpampRailMode::ActiveSet`] so the steady DC rail value is preserved.
///
/// This refinement runs *after* [`resolve_opamp_rail_mode`] because the
/// initial resolve only needs the MNA, but the topology check needs the raw
/// netlist (to enumerate `Element::Resistor` rather than the post-stamping
/// `G` matrix, which conflates resistors with op-amp VCCS entries). Splitting
/// the resolve in two avoids threading `&Netlist` through ~20 call sites that
/// don't need it.
///
/// # When this is called
///
/// Call this from any code path that has both the MNA and the original
/// netlist available (e.g. `CodeGenerator::generate_with_dc_op` and
/// `CodeGenerator::generate_nodal`). User overrides are honored unchanged —
/// the refinement only runs when the input was an Auto decision that
/// resolved to `ActiveSet`.
///
/// # Why "any" rather than "all"
///
/// One sidechain op-amp is enough to ruin the compressor envelope if BE
/// damping kicks in for the whole circuit. Generated code emits one rail
/// strategy for the entire process_sample function — we can't mix
/// strategies per op-amp. So we conservatively pick the strategy that's
/// safe for ALL clampable op-amps: if even one needs the trap+pin steady
/// rail, plain ActiveSet wins.
pub fn refine_active_set_for_audio_path(
    resolved: ResolvedOpampRailMode,
    mna: &crate::mna::MnaSystem,
    netlist: &crate::parser::Netlist,
) -> ResolvedOpampRailMode {
    use crate::codegen::OpampRailMode;

    // Only refine the auto-resolved ActiveSet decision. User overrides
    // (UserRequested) and other modes pass through unchanged.
    if resolved.mode != OpampRailMode::ActiveSet
        || resolved.reason != OpampRailModeReason::AcCoupledDownstream
    {
        return resolved;
    }

    let clamped_opamps: Vec<&crate::mna::OpampInfo> = mna
        .opamps
        .iter()
        .filter(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()))
        .collect();

    let any_control_path = clamped_opamps
        .iter()
        .any(|oa| opamp_has_r_only_path_to_nonlinear(netlist, mna, oa));

    if any_control_path {
        ResolvedOpampRailMode {
            mode: OpampRailMode::ActiveSet,
            reason: OpampRailModeReason::AcCoupledDownstreamControlPath,
        }
    } else {
        ResolvedOpampRailMode {
            mode: OpampRailMode::ActiveSetBe,
            reason: OpampRailModeReason::AcCoupledDownstreamAudioPath,
        }
    }
}

/// Name of the synthetic catch-diode model inserted by
/// [`augment_netlist_with_boyle_diodes`]. Standard Boyle-macromodel silicon
/// diode parameters (`Is = 1e-15, N = 1`), matching every commercial SPICE
/// op-amp model's output-stage clamp diodes.
pub const BOYLE_CATCH_DIODE_MODEL: &str = "D_BOYLE_CATCH";

/// Augment a parsed netlist with the Boyle-style internal-gain-node op-amp
/// model (catch diodes + output buffer), returning a fresh [`Netlist`] that
/// contains the original elements plus the synthesized internal-node
/// scaffolding.
///
/// # Mechanism
///
/// For each op-amp with at least one finite supply rail, this function adds
/// the elements required to turn melange's flat linear-VCCS op-amp model into
/// a two-stage Boyle macromodel:
///
/// ```text
///   V+ ─┐                                  [buf_out] ─R_ro─→ V_out
///       │                                      │
///   V- ─┴── Gm ──→ [_oa_int_{name}] ─VCVS(×1)──┘
///                       │  ▲
///              R_int ───┘  │
///              (1 MΩ,      │
///              stamped by  │
///              MNA, not    │
///              in netlist) │
///                          │
///                catch diodes pin
///                this node to ±rail
/// ```
///
/// **Per clamped op-amp**, the function adds:
///
/// - A unity-gain output buffer VCVS `E_oa_buf_{name}` forcing
///   `V(_oa_buf_out_{name}) = V(_oa_int_{name})` regardless of load, followed
///   by a series output resistor `R_oa_ro_{name}` (`R = r_out`: Boyle's 75 Ω
///   default, or `.model OA(ROUT=…)`) from the buffer output to the original
///   output node. The series-R form — NOT a VCCS + shunt-to-ground — is
///   essential: a shunt to ground would fight the op-amp's DC bias on any
///   circuit lacking a hard pull-up (e.g. a high-Z vbias divider). Gives
///   `V_out = V_int` under feedback and source impedance `≈ r_out`, with no DC
///   sink to ground.
///
/// - For each finite rail, a reference node `_boyle_hi_{name}` /
///   `_boyle_lo_{name}` pinned to `VCC − VOH_DROP` / `VEE + VOL_DROP` by a
///   synthesized DC voltage source, and a catch diode between
///   **`_oa_int_{name}`** and that reference node. The diodes are placed
///   on the internal node — NOT on the original output — so they only have
///   to balance the small current produced by the in-MNA Gm injection
///   (≈ 0.2 A peak for an `AOL = 200000` part), instead of fighting the
///   ~4000 S linear-VCCS conductance present on the original output.
///
/// - One top-level `.model D_BOYLE_CATCH D(IS=1e-15 N=1)` definition, added
///   only once. Standard silicon parameters used by every commercial SPICE
///   op-amp macromodel (Boyle/Cohn/Pederson/Solomon JSSC 1974; reproduced
///   in TI's PSpice TL072 model, ngspice's built-in op-amp, LTSpice's
///   `UniversalOpamp2`, etc.).
///
/// The internal node `_oa_int_{name}` is NOT a netlist resistor — it's
/// referenced only by the buffer/diodes. When MNA stamping sees the name in
/// `node_map`, it switches that op-amp's transconductance stamp from the
/// original output node to the internal node (using
/// [`crate::mna::R_BOYLE_INT_LOAD`] as the effective output resistance). See
/// [`crate::mna::MnaSystem::from_netlist`] for the dispatch.
///
/// VOH_DROP and VOL_DROP default to 1.5 V in [`crate::mna::OpampInfo`] for
/// TL072/NE5532-class parts and can be overridden in the user's
/// `.model OA(VOH_DROP=… VOL_DROP=…)` for rail-to-rail op-amps.
///
/// # Why this specific topology
///
/// A linear VCCS (the pre-2026-04 op-amp model in melange) has unbounded
/// output. Three ways to bound it:
///
/// 1. **Hard clamp `v[out] = clamp(v[out], VEE, VCC)`** — cheap but
///    (a) breaks KCL for AC-coupled downstream caps, and (b) produces a
///    square-law limiter's flat odd-harmonic series instead of the soft
///    exponential knee real op-amps exhibit.
///
/// 2. **Piecewise linear catch sources** (the
///    [`OpampRailMode::ActiveSet`](crate::codegen::OpampRailMode::ActiveSet)
///    path) — KCL-consistent but still produces hard-clip harmonics.
///
/// 3. **Boyle catch diodes on an internal high-impedance gain node** (this
///    function) — silicon diodes anchored to rail-offset DC sources, hung
///    off the *internal* node so the diode's exponential conductance only
///    has to balance ~1 µS (the internal load), not ~4000 S (Gm of the
///    original linear VCCS). Produces the smooth exponential knee that
///    matches measured TL072 saturation and reproduces the "mid drive
///    crunch" of guitar overdrive pedals like the Klon Centaur. NR
///    converges where the naive on-output placement diverged.
///
/// # Contract
///
/// The returned netlist must compile through
/// [`crate::mna::MnaSystem::from_netlist`] without errors. All synthesized
/// node names are prefixed with `_boyle_` or `_oa_int_` to avoid collisions
/// with user-defined names. If a user happens to have a node named
/// `_boyle_hi_U1A` or `_oa_int_U1A`, behavior is undefined — treat the
/// prefixes as reserved.
///
/// Op-amps with no finite rails (ideal VCCS) or with grounded output
/// (`n_out_idx == 0`, which should have already been rejected earlier)
/// are skipped silently.
///
/// The original `netlist` is cloned, not consumed — callers can still
/// use the un-augmented netlist after this returns.
pub fn augment_netlist_with_boyle_diodes(
    netlist: &crate::parser::Netlist,
    mna: &crate::mna::MnaSystem,
) -> crate::parser::Netlist {
    use crate::parser::{Element, Model};

    let mut augmented = netlist.clone();

    // Add the shared catch-diode model once.
    if !augmented
        .models
        .iter()
        .any(|m| m.name == BOYLE_CATCH_DIODE_MODEL)
    {
        augmented.models.push(Model {
            name: BOYLE_CATCH_DIODE_MODEL.to_string(),
            model_type: "D".to_string(),
            params: vec![("IS".to_string(), 1e-15), ("N".to_string(), 1.0)],
        });
    }

    // Build a reverse lookup from node index (1-indexed, matching node_map)
    // back to node name so we can emit human-readable synthesized elements.
    // We only need names for op-amp output nodes.
    let name_of_idx = |idx: usize| -> Option<String> {
        mna.node_map
            .iter()
            .find_map(|(name, &i)| if i == idx { Some(name.clone()) } else { None })
    };

    for oa in &mna.opamps {
        if oa.n_out_idx == 0 {
            continue;
        }
        let has_upper = oa.vcc.is_finite();
        let has_lower = oa.vee.is_finite();
        if !has_upper && !has_lower {
            continue;
        }

        // Look up the user's output node name so the diode element reads
        // naturally in generated code. If the node isn't in the map something
        // is badly wrong upstream; skip silently rather than crashing.
        let out_name = match name_of_idx(oa.n_out_idx) {
            Some(s) => s,
            None => continue,
        };

        // Sanitize the op-amp name for use in synthesized identifiers:
        // SPICE node names are typically ASCII alphanumeric plus `_`.
        // We don't lowercase because the MNA node_map preserves case.
        // MUST match the sanitization in `crate::mna::MnaSystem::from_netlist`
        // op-amp stamping — the MNA dispatcher looks up `_oa_int_{safe_name}`
        // by exactly this rule.
        let safe_name: String = oa
            .name
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || c == '_' {
                    c
                } else {
                    '_'
                }
            })
            .collect();

        let int_node = format!("_oa_int_{}", safe_name);
        let buf_out_node = format!("_oa_buf_out_{}", safe_name);

        // Dominant-pole capacitor from the internal gain node to ground.
        //
        // This is the "C1" compensation cap in Boyle's 1974 macromodel.
        // Together with the R1 load on the internal node (stamped directly
        // in `mna.rs` op-amp dispatch as `Go_int = 1 / R_BOYLE_INT_LOAD`),
        // it creates the first pole at `f_p = 1 / (2π · R1 · C_dom)`.
        // The unity-gain crossover frequency is then
        // `GBW = Gm1 / (2π · C_dom)` where `Gm1 = AOL / R1` (also stamped
        // in MNA dispatch), so inverting to solve for C_dom:
        //
        //   C_dom = Gm1 / (2π · GBW) = AOL / (2π · GBW · R1)
        //
        // For TL072 at AOL=200k, GBW=3 MHz, R1=1 MΩ: C_dom ≈ 10.6 nF.
        // That feels large because R1 is 1 MΩ (a macromodel convenience,
        // not a physical value); what matters for numerical behaviour is
        // the *trap-rule conductance* 2·C_dom/T. For the example at 48 kHz
        // that's 2·10.6e-9·48000 ≈ 1 mS — comfortably well-conditioned,
        // safely above the R1 shunt (1 µS) and well below rail-engaged
        // diode conductance (~1 S).
        //
        // Critically, the dominant pole SMOOTHS the internal-node voltage
        // across the rail transition. Without C_dom the catch diode
        // switches state infinitely fast between samples, which is what
        // excited the C15 Nyquist resonator in every previous attempt.
        // With C_dom present, the int-node voltage transitions smoothly
        // over a few samples, so the downstream cap-coupled output path
        // sees a continuously-differentiable signal and the trap-rule
        // integrator stays stable.
        //
        // If no GBW is specified in `.model OA()`, fall back to 10 nF —
        // gives a first pole near 16 Hz with R1=1 MΩ, rolling off 6 dB/oct
        // to unity gain around 2 MHz. Audibly transparent, numerically
        // well-conditioned.
        // NOTE: The Boyle 1974 macromodel includes a dominant-pole cap
        // at the internal gain node for AOL frequency rolloff, but in
        // melange's numerical experiments at 48 kHz, adding a cap from
        // int_node to ground creates an ill-conditioned row (the int
        // node has a tiny pre-cap diagonal from R1 = 1 MΩ vs large
        // off-diagonal Gm = 0.1 S) that breaks the compile-time sparse
        // LU at cap values above ~5 pF. Since audio-band bandwidth is
        // already much lower than any op-amp's GBW, the missing pole
        // has no audible effect, and we omit C_dom entirely in this
        // implementation. If a future test case requires band-limited
        // op-amp behaviour, the cap can be added at the buffer output
        // node (`_oa_buf_out_`) where the numerics are well-conditioned.

        // Output buffer VCVS: forces `V(buf_out) = V(int)` regardless of
        // load at the buffer output node. The user-facing output node
        // (`out_name`) is reached from `buf_out` through a series `R_ro`,
        // so the effective op-amp output impedance from `out_name` looking
        // back is `R_ro`. This matches Boyle's 1974 two-stage topology:
        // high-Z internal gain node → unity-gain buffer → small output
        // series R → external load.
        //
        // The VCVS form (not VCCS + shunt-to-ground) is essential: a
        // shunt-to-ground at the output would fight the op-amp's DC bias
        // for any circuit without a hard pull-up (e.g. Klon biases its
        // op-amps via a high-Z divider at vbias=4.5V — a 50Ω shunt would
        // dump 90 mA into ground at idle). The series-R topology adds
        // real source impedance WITHOUT creating any DC sink to ground.
        augmented.elements.push(Element::Vcvs {
            name: format!("E_oa_buf_{}", safe_name),
            out_p: buf_out_node.clone(),
            out_n: "0".to_string(),
            ctrl_p: int_node.clone(),
            ctrl_n: "0".to_string(),
            gain: 1.0,
        });

        // Output series resistance. Boyle's canonical value is 75 Ω,
        // matching TL072-class bipolar op-amps. For feedback-dominated
        // topologies (all audio circuits we target), closed-loop output
        // impedance is `R_ro / (1 + loop_gain) ≈ µΩ`, so 75 Ω has no
        // audible effect. For high-impedance loads (cap-coupled,
        // 10 kΩ+ feedback networks) the 75 Ω drop is a fraction of
        // a millivolt. Use the op-amp's `r_out` field if the user
        // overrode it in `.model OA(ROUT=…)`, otherwise 75 Ω.
        let r_out = if oa.r_out > 1.0 { oa.r_out } else { 75.0 };
        augmented.elements.push(Element::Resistor {
            name: format!("R_oa_ro_{}", safe_name),
            n_plus: buf_out_node,
            n_minus: out_name.clone(),
            value: r_out,
            kf: None,
            af: None,
        });

        if has_upper {
            // Upper catch: internal node → (VCC − VOH_DROP) reference node.
            // Diode anode is the internal node so positive overdrive (V_int
            // climbing toward VCC) forward-biases the diode and pins V_int.
            let rail_node = format!("_boyle_hi_{}", safe_name);
            augmented.elements.push(Element::VoltageSource {
                name: format!("V_boyle_hi_{}", safe_name),
                n_plus: rail_node.clone(),
                n_minus: "0".to_string(),
                dc: Some(oa.vcc - oa.voh_drop),
                ac: None,
            });
            augmented.elements.push(Element::Diode {
                name: format!("D_boyle_hi_{}", safe_name),
                n_plus: int_node.clone(),
                n_minus: rail_node,
                model: BOYLE_CATCH_DIODE_MODEL.to_string(),
            });
        }

        if has_lower {
            // Lower catch: (VEE + VOL_DROP) reference node → internal node.
            let rail_node = format!("_boyle_lo_{}", safe_name);
            augmented.elements.push(Element::VoltageSource {
                name: format!("V_boyle_lo_{}", safe_name),
                n_plus: rail_node.clone(),
                n_minus: "0".to_string(),
                dc: Some(oa.vee + oa.vol_drop),
                ac: None,
            });
            augmented.elements.push(Element::Diode {
                name: format!("D_boyle_lo_{}", safe_name),
                n_plus: rail_node,
                n_minus: int_node.clone(),
                model: BOYLE_CATCH_DIODE_MODEL.to_string(),
            });
        }
    }

    augmented
}
