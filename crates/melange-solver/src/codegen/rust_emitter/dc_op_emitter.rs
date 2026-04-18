//! Runtime DC operating-point recompute emission (Oomox P6, Phase E).
//!
//! Emits `CircuitState::recompute_dc_op()` into generated plugin code so that
//! plugins with per-instance pot/switch jitter can re-solve the DC bias at
//! construction time, skipping the `WARMUP_SAMPLES_RECOMMENDED` silence loop.
//!
//! ## Phase E scope
//!
//! **MVP — Direct Newton-Raphson only**, starting from `self.v_prev` (which is
//! a physically valid state from the prior solve). No source/Gmin stepping,
//! no basin-trap handling. Circuits that only converge via stepping in the
//! codegen-time solver remain reliant on the warmup loop.
//!
//! Device coverage: diode, BJT (EM + GP), JFET, MOSFET, triode, pentode,
//! linear op-amp, VCA — i.e. whatever the compile-time DC OP handles today,
//! minus multi-equilibrium refinements.
//!
//! ## Layering (implemented across subsequent commits)
//!
//! 1. `emit_recompute_dc_op_body_dk` — top-level no-op stub (E.2, this file).
//! 2. `emit_dc_op_build_g_aug` — G_aug construction from live pot/switch state (E.3).
//! 3. `emit_dc_op_evaluate_devices` — per-device i_nl + Jacobian (E.4).
//! 4. `emit_dc_op_solve` — NR loop, limiting, dense LU (E.5).
//! 5. Compose + state resets (E.6).
//! 6. Nodal mirror (E.8).
//!
//! The skeleton body below intentionally does NOT touch circuit state. It
//! exists so the flag plumbing is testable end-to-end (code compiles, method
//! is callable) before the solver proper lands. See the Phase E handoff
//! memory file for the full definition of done.

use crate::codegen::ir::CircuitIR;

/// Emit the body of `CircuitState::recompute_dc_op()` for a DK-path circuit.
///
/// **Current state (Phase E.2)**: returns a no-op stub body with an
/// explanatory comment so plugin authors who wire up the API before the
/// solver lands don't get silent incorrect behavior. The method signature is
/// stable; only the body fills in over subsequent emission phases.
pub(super) fn emit_recompute_dc_op_body_dk(_ir: &CircuitIR) -> String {
    // Keep this body honest: until the full NR emission ships, calling this
    // does nothing and the plugin must still rely on the warmup loop.
    let mut body = String::new();
    body.push_str(
        "        // Phase E MVP skeleton (Oomox P6).\n\
         \x20       //\n\
         \x20       // The full runtime DC-operating-point re-solve is emitted across\n\
         \x20       // several subsequent phases (G_aug build, per-device evaluation,\n\
         \x20       // Newton-Raphson loop). Until those land, calling this method is a\n\
         \x20       // no-op — plugin code that needs jittered-pot convergence should\n\
         \x20       // still use the `WARMUP_SAMPLES_RECOMMENDED` silence loop.\n\
         \x20       //\n\
         \x20       // The signature and name are stable: once the solver lands, this\n\
         \x20       // method will re-solve the DC system at the current pot/switch\n\
         \x20       // values and update `self.dc_operating_point`, `self.v_prev`, and\n\
         \x20       // `self.i_nl_prev` in place.\n",
    );
    body
}
