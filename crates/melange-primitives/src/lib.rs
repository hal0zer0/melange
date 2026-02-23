// melange-primitives: DSP building blocks for real-time circuit simulation
//
// Layer 1 of the melange stack. Zero-allocation, no_std-compatible primitives:
// - Filter primitives (one-pole, biquad, DC blocker, TPT integrator)
// - Polyphase IIR oversampling (configurable 2x/4x)
// - Newton-Raphson solver templates (fixed iteration, NaN recovery)
// - Per-instance variation (deterministic hash-based detuning)
// - Companion model helpers (trapezoidal/bilinear discretization)

#![no_std]
