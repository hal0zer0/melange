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

pub mod companion;
pub mod filters;
pub mod nr;
pub mod oversampling;
pub mod util;

pub use companion::{
    BackwardEulerCompanion, BilinearCompanion, InductorCompanion, TrapezoidalCompanion,
    companion_conductance_backward_euler, companion_conductance_trapezoidal,
    history_current_trapezoidal, inductor_companion_conductance_trapezoidal,
};
pub use filters::{Biquad, BiquadType, DcBlocker, OnePoleHpf, OnePoleLpf, TptLpf};
pub use nr::{NrResult, fetlim, nr_solve_1d, nr_solve_2d, nr_solve_dk, pn_vcrit, pnjlim};
pub use oversampling::{
    HalfBand2, HalfBand3, HalfBand4, HalfBandFilter, Oversampler, Oversampler2x, Oversampler2xFast,
    Oversampler2xQuality, Oversampler2xStandard, Oversampler4x, coefficients,
};
pub use util::{
    VT_ROOM, db_to_gain, freq_to_midi, gain_to_db, is_valid_audio, lerp, map_range, midi_to_freq,
    saturate, sin_approx, soft_clip, soft_clip_variable, thermal_voltage, triangle_from_phase,
    variation_hash, variation_range,
};
