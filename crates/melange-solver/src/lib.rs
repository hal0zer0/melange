// melange-solver: MNA/DK-method circuit solver engine
//
// Layer 3 of the melange stack. The core of the project:
// - SPICE netlist parser (subset sufficient for audio circuits)
// - Automatic MNA stamp generation from component lists
// - Companion model library (trapezoidal/bilinear for R, C, L, series-RC)
// - DK kernel extraction (reduce N-node linear system to M-nonlinearity kernel)
// - Newton-Raphson iteration on the reduced nonlinear system
// - Sherman-Morrison rank-1 update for time-varying elements
// - Shadow subtraction framework for modulation-induced pump cancellation
// - Code generation: emit optimized Rust with const-generic matrix sizes
