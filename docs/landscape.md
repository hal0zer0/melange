# Landscape: Existing Tools and Where Melange Fits

## The Gap

As of February 2026, no open-source tool in any language provides the full pipeline from circuit netlist to optimized real-time DSP code. The closest tools and their limitations:

### Circuit-to-DSP Tools

| Tool | Language | What It Does | Where It Stops |
|------|----------|-------------|----------------|
| **Cytomic CCS** | C++ (proprietary) | Full pipeline: LTSpice netlist → optimized C++, with symbolic simplification | **Proprietary. Closed-source. The gold standard nobody can use.** |
| **ACME.jl** | Julia | Programmatic circuit description → state-space matrices | Not real-time capable. Julia runtime overhead. "More geared towards computing matrices than running the model." |
| **NDKFramework** | MATLAB + C++ | LTSpice netlist → MATLAB matrices → JUCE | Requires MATLAB license. MATLAB-to-JUCE is manual. |
| **joaorossi/dkmethod** | Python + C++ | Python netlist parser → C++ JUCE class | Unmaintained. Depends on LAPACK. Limited topology support. |
| **chowdsp_wdf** | C++ (header-only) | Template-based WDF library. Production-proven. | C++ only. WDF only (no DK/MNA). Requires manual circuit decomposition into WDF tree. |
| **RT-WDF** | C++ | Object-oriented WDF API | Research-grade. Not optimized for production. Not maintained. |
| **LiveSPICE** | C# | Visual schematic editor → JIT-compiled simulation | Runtime interpreter, not code generator. C#/.NET only. Windows-centric. |
| **RTspice** | C++ | SPICE-like netlist → real-time JACK audio | Experimental. "Works best with modest circuits." |
| **FAUST wdmodels** | FAUST | WDF primitives in FAUST syntax | Subset of Werner's dissertation only. No multiport nonlinear adaptors. |
| **Halite** | C++ (~1000 lines) | Educational MNA simulator | "The solver is the weakest part." Not production-ready. |

### Rust Audio Ecosystem

| Crate | What It Does | What's Missing |
|-------|-------------|----------------|
| **nih-plug** | CLAP + VST3 plugin framework | No DSP primitives |
| **dasp** | Sample types, interpolation, ring buffers | No filters, no circuit modeling |
| **FunDSP** | Inline audio graph notation, biquad filters | No nonlinear modeling, no MNA |
| **CPAL** | Cross-platform audio I/O | Low-level only |

**What does NOT exist in Rust:**
- No nonlinear circuit solver crate
- No wave digital filter crate
- No transistor/tube/diode model library
- No SPICE netlist parser targeting audio-rate code
- No audio-aware oversampling crate
- No Newton-Raphson solver tuned for real-time audio constraints

### Academic References

**Foundational:**
- David Yeh, Stanford 2009: "Digital Implementation of Musical Distortion Circuits by Analysis and Simulation." Introduced the K-method and DK-method.
- Kurt Werner, Stanford 2016: "Virtual Analog Modeling of Audio Circuitry Using Wave Digital Filters." Extended WDF to arbitrary topologies.
- Holters & Zolzer: Generalized NDK method for circuits with variable parts.
- Pakarinen & Yeh, 2009: "A Review of Digital Techniques for Modeling Vacuum-Tube Guitar Amplifiers." Definitive survey.

**Recent (2021-2025):**
- Differentiable white-box modeling (Esqueda et al., DAFx 2021): Backprop to discover component values from audio recordings.
- Neural ODEs for virtual analog (Wilczek, DAFx 2022): Learn governing ODEs of diode clippers.
- Neural state-space filters (DAFx 2024): Hybrid physics-ML for guitar tone stacks.
- NAM / AIDA-X (2023-2025): Fully black-box neural amp modeling. Popular but no physics inside.

**Unimplemented in open source:**
1. Werner's MNA-derived R-type WDF adaptors for arbitrary topologies
2. Automated DK-method with symbolic optimization (Cytomic does this privately)
3. Differentiable white-box component fitting (research PyTorch code only)
4. Homotopy-based convergence for strongly nonlinear circuits (Yeh described it, nobody packaged it)
5. Hybrid physics-ML correction (OpenWurli's MLP approach, no general framework exists)

## Where Melange Fits

Melange occupies the gap between "ACME.jl can derive matrices but can't run real-time" and "Cytomic CCS does everything but is proprietary." Specifically:

1. **Netlist → MNA matrices** (what ACME.jl does, but in Rust)
2. **MNA → DK kernel reduction** (what NDKFramework does, but without MATLAB)
3. **DK kernel → optimized Rust code** (what Cytomic CCS does, but open-source)
4. **Validation against ngspice** (what nobody automates)

The unique combination: open-source + Rust + full pipeline + production performance + automated validation.
