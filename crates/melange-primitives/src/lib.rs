//! DSP building blocks for real-time audio circuit simulation.
//!
//! Layer 1 of the melange stack — the foundation. Provides zero-allocation primitives
//! used by the solver and generated code.
//!
//! # Modules
//!
//! - [`filters`] — One-pole, biquad, DC blocker, and TPT integrator filters
//! - [`oversampling`] — Polyphase half-band IIR oversampling (2x/4x)
//! - [`nr`] — Newton-Raphson helpers: `pnjlim`/`fetlim` voltage limiting, 1D/2D solvers
//! - [`companion`] — Trapezoidal and backward Euler companion model discretization
//! - [`util`] — Audio math utilities: dB conversion, soft clipping, thermal voltage, etc.
//!
//! # Real-Time Safety
//!
//! All types are designed for real-time audio: no heap allocation after construction,
//! no panics on valid input, deterministic execution time.

pub mod companion;
pub mod filters;
pub mod nr;
pub mod oversampling;
pub mod util;

pub use companion::{
    companion_conductance_backward_euler, companion_conductance_trapezoidal,
    history_current_trapezoidal, inductor_companion_conductance_trapezoidal,
    BackwardEulerCompanion, BilinearCompanion, InductorCompanion, TrapezoidalCompanion,
};
pub use filters::{Biquad, BiquadType, DcBlocker, OnePoleHpf, OnePoleLpf, TptLpf};
pub use nr::{fetlim, nr_solve_1d, nr_solve_2d, nr_solve_dk, pn_vcrit, pnjlim, NrResult};
pub use oversampling::{
    coefficients, HalfBand2, HalfBand3, HalfBand4, HalfBandFilter, Oversampler, Oversampler2x,
    Oversampler2xFast, Oversampler2xQuality, Oversampler2xStandard, Oversampler4x,
};
pub use util::{
    db_to_gain, freq_to_midi, gain_to_db, is_valid_audio, lerp, map_range, midi_to_freq, saturate,
    sin_approx, soft_clip, soft_clip_variable, thermal_voltage, triangle_from_phase,
    variation_hash, variation_range, VT_ROOM,
};
