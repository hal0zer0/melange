//! Tuned numerical-policy constants.
//!
//! Values that were *chosen*, not derived. Nothing in the circuit, the netlist
//! or the math determines them; they were picked because they behaved well, and
//! changing one changes the sound of every plugin melange emits.
//!
//! They live here for two reasons.
//!
//! **One source of truth.** Each of these was previously a bare literal repeated
//! at every site that needed it. [`DC_BLOCK_CUTOFF_HZ`] was spelled out five
//! times across two emitters and two templates; a plan revision that recorded it
//! as living at "one site" was wrong by a factor of five, and nobody noticed
//! because the copies happened to agree. Copies that happen to agree are the
//! precondition for copies that quietly stop agreeing.
//!
//! **They are a published contract.** A second-language backend has to reproduce
//! every one of these *exactly* — they cannot be recomputed from the IR, because
//! nothing derives them. A C++ emitter that picks its own 5 Hz-equivalent, or
//! rounds one of these differently, produces a plugin that measurably does not
//! match the Rust one while every structural test still passes. Enumerating them
//! in one module is what makes that contract checkable instead of implicit.
//!
//! Adding a constant here is therefore a small API commitment, not just tidiness.
//! Cite the reason for the value, not only the value.

/// DC-blocking high-pass cutoff, in hertz.
///
/// The generated one-pole DC blocker is `y[n] = x[n] - x[n-1] + R * y[n-1]`,
/// with `R = 1 - 2*pi*f_c/fs_internal` evaluated at the **internal** (oversampled)
/// rate. 5 Hz sits below the audible band while still removing the operating-point
/// offset a circuit's DC solution leaves on the output node.
///
/// Emitted into generated code as a literal, so any change here moves the DSP of
/// every deck with `dc_block` enabled — which is most of them.
///
/// See `docs/aidocs/SIGNAL_LEVELS.md` for where the blocker sits in the output
/// pipeline (extract -> DC-block -> scale -> peak -> clamp).
pub const DC_BLOCK_CUTOFF_HZ: f64 = 5.0;

/// [`DC_BLOCK_CUTOFF_HZ`] formatted for interpolation into emitted Rust source.
///
/// `Debug` formatting rather than `Display` because it is the one that keeps the
/// decimal point (`5.0`, not `5`), which is what the emitted expression has
/// always read and what keeps a byte-identical regeneration byte-identical.
pub fn dc_block_cutoff_hz_literal() -> String {
    format!("{:?}", DC_BLOCK_CUTOFF_HZ)
}
