// melange-devices: Parameterized nonlinear component models
//
// Layer 2 of the melange stack. Each device implements the NonlinearDevice trait
// providing i(v) and di/dv for use in MNA/DK solvers. Models include:
// - BJT (Ebers-Moll, Gummel-Poon) with SPICE model card import
// - Vacuum tubes (Koren triode/pentode)
// - Diodes (Shockley equation)
// - MOSFETs, JFETs
// - Opamps (Boyle macromodel)
// - CdS LDR photocell (asymmetric attack/release, power-law resistance)

#![no_std]
