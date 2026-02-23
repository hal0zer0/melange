// melange-validate: SPICE-to-Rust validation pipeline
//
// Layer 4 of the melange stack. Bridges ngspice simulation and Rust DSP output:
// - ngspice runner (invoke simulation, parse raw/CSV output)
// - Automated comparison: DC bias, AC sweep, transient, THD
// - Tolerance-band assertions for integration into unit tests
// - "Bisect the signal chain" debugging helpers
// - Measurement routines (gain, frequency sweep, harmonics, spectral centroid)
