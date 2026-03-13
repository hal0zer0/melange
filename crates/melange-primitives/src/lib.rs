// melange-primitives: DSP building blocks for real-time circuit simulation
//
// Layer 1 of the melange stack. Zero-allocation primitives (no_std planned, not yet enabled):
// - Filter primitives (one-pole, biquad, DC blocker, TPT integrator)
// - Polyphase IIR oversampling (configurable 2x/4x)
// - Newton-Raphson solver templates (fixed iteration, NaN recovery)
// - Per-instance variation (deterministic hash-based detuning)
// - Companion model helpers (trapezoidal/bilinear discretization)

// Note: no_std support planned for embedded targets
// For now, we use std for floating-point math methods
// #![no_std]

pub mod filters;
pub mod oversampling;
pub mod nr;
pub mod companion;
pub mod util;

pub use filters::{OnePoleLpf, OnePoleHpf, TptLpf, DcBlocker, Biquad, BiquadType};
pub use oversampling::{
    coefficients, HalfBandFilter, HalfBand2, HalfBand3, HalfBand4,
    Oversampler2x, Oversampler2xFast, Oversampler2xStandard, Oversampler2xQuality,
    Oversampler4x, Oversampler,
};
pub use nr::{NrResult, nr_solve_1d, nr_solve_2d, nr_solve_dk, pn_vcrit, pnjlim, fetlim};
pub use companion::{
    TrapezoidalCompanion, BilinearCompanion, BackwardEulerCompanion,
    InductorCompanion,
    companion_conductance_trapezoidal, companion_conductance_backward_euler,
    inductor_companion_conductance_trapezoidal, history_current_trapezoidal,
};
pub use util::{
    variation_hash, variation_range, midi_to_freq, freq_to_midi, map_range,
    db_to_gain, gain_to_db, lerp, saturate, soft_clip, soft_clip_variable,
    sin_approx, triangle_from_phase, is_valid_audio, thermal_voltage, VT_ROOM,
};
