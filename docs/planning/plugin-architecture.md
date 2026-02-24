# Plugin Architecture Planning Document

## Current Approach: Minimal Viable Plugin (MVP)

**Goal:** Get from SPICE to working plugin in one command.

### Implementation
- Template-based code generation
- Fixed generic UI (no custom graphics initially)
- nih-plug integration for CLAP/VST3/AU support
- Single audio input/output
- Hardcoded parameter mapping initially

### Usage
```bash
melange compile tube-screamer --output myplugin/ --format plugin
cd myplugin && cargo run --release
```

---

## Future Architecture: Two-Layer System

### Layer 1: Quick Mode (Current Implementation)
**For:** Rapid prototyping, testing circuits, simple pedals

**Features:**
- One-command complete plugin generation
- Generic nih-plug UI (sliders, toggles)
- Automatic parameter mapping (R1, C1 → "Gain", "Tone")
- Single stereo/mono in/out
- Fixed oversampling (2x or 4x)

**Trade-offs:**
+ Fastest time-to-plugin
+ Zero Rust knowledge required
- Limited customization
- Generic appearance

### Layer 2: Library Mode (Future)
**For:** Production plugins, custom UIs, advanced features

**Features:**
- `melange-plugin` as a library dependency
- `CircuitProcessor` trait for custom wrappers
- `wrap_circuit!()` macro for glue code
- Explicit parameter mapping DSL
- Custom VIZIA or egui UIs
- Multi-voice support for synths
- Configurable oversampling per-stage

**Usage:**
```rust
use melange_plugin::{wrap_circuit, CircuitParams, Oversampling};

include!(concat!(env!("OUT_DIR"), "/circuit.rs"));

wrap_circuit!(TubeScreamerPlugin, circuit::process_sample, {
    params: {
        gain: (circuit::R1, 1e3..=100e3, "Drive"),
        tone: (circuit::C2, 1e-9..=100e-9, "Tone"),
    },
    oversampling: Oversampling::Adaptive { max: 16 },
    ui: ToneStackUI,  // Custom VIZIA component
});
```

---

## Component Breakdown

### 1. Parameter Mapping System

**Current (MVP):**
Auto-generated names based on component type:
- Resistors → "Drive", "Level", "Tone" (heuristic)
- Capacitors → "Brightness", "Presence"
- Manual override via comments in SPICE:
  ```spice
  R1 in out 10k $ param:Drive range:1k,100k
  ```

**Future (Library Mode):**
Explicit mapping with ranges and units:
```rust
ParameterMapping {
    component: "R1",
    name: "Drive",
    range: Logarithmic(1e3..=100e3),
    unit: "Ohms",
    ui: Slider { style: Vintage },
}
```

### 2. Oversampling Strategy

**Current (MVP):**
- Global 4x oversampling for all nonlinear circuits
- Simple half-band filters from melange-primitives
- Always on (CPU cost accepted)

**Future (Library Mode):**
- Stage classification (linear vs nonlinear)
- Adaptive oversampling (more when needed)
- Per-stage configuration
- Minimized latency paths for linear sections

```rust
enum OversamplingStrategy {
    Fixed(u32),                    // Always Nx
    Adaptive { max: u32 },         // Up to Nx based on signal
    PerStage(StageMap<u32>),       // Different per circuit section
    Bypass,                        // None (for linear circuits)
}
```

### 3. Voice Management (for Synths)

**Current:** Not implemented (effects only)

**Future:**
For circuits that model synth voices (Minimoog oscillator + filter):

```rust
struct CircuitVoice {
    circuit: CircuitState,
    note: u8,
    velocity: u8,
    gate: bool,
}

impl Voice for CircuitVoice {
    fn trigger(&mut self, note: NoteEvent) {
        self.gate = true;
        // Set circuit parameters based on note
    }
    
    fn process(&mut self, output: &mut [f64]) {
        for sample in output {
            *sample = circuit::process_sample(self.input, &mut self.circuit);
        }
    }
}
```

### 4. UI System

**Current (MVP):**
- nih-plug's built-in generic UI
- Sliders, toggles, dropdowns
- Auto-layout based on parameter count

**Future (Library Mode):**
- VIZIA integration for custom graphics
- Component-based UI (knobs, switches, meters)
- Schematic visualization (show the actual circuit)
- Preset management with circuit state visualization

```rust
// Example custom UI
#[derive(Lens)]
struct CircuitUI {
    params: CircuitParams,
}

impl View for CircuitUI {
    fn draw(&mut self, cx: &mut Context, canvas: &mut Canvas) {
        // Draw vintage pedal graphics
        draw_knob(canvas, "Drive", self.params.gain.normalized());
        draw_knob(canvas, "Tone", self.params.tone.normalized());
        draw_led(canvas, self.params.bypass.value());
    }
}
```

### 5. Calibration Infrastructure

**Current:** Manual parameter tuning

**Future:**
Automated calibration for physical accuracy:

```rust
// Sweep analysis to match reference
fn calibrate_to_reference(
    circuit: &mut Circuit,
    reference: &AudioFile,
) -> CalibrationResult {
    // Sweep frequency response
    // Adjust component tolerances
    // Minimize error vs reference
}
```

---

## Implementation Roadmap

### Phase 1: MVP (Current Sprint)
- [ ] Plugin template generation
- [ ] nih-plug integration
- [ ] Generic parameter UI
- [ ] Global oversampling
- [ ] Single stereo I/O

### Phase 2: Enhanced Quick Mode
- [ ] Heuristic parameter naming
- [ ] SPICE comment hints for params
- [ ] Preset management
- [ ] Basic metering (input/output gain)

### Phase 3: Library Mode Foundation
- [ ] `melange-plugin` library crate
- [ ] `CircuitProcessor` trait
- [ ] Explicit parameter mapping
- [ ] Standalone example plugin

### Phase 4: Advanced Features
- [ ] Custom UI support (VIZIA)
- [ ] Multi-voice architecture
- [ ] Per-stage oversampling
- [ ] Calibration tools

### Phase 5: Ecosystem
- [ ] Plugin marketplace/sharing
- [ ] Circuit library browser
- [ ] Visual schematic editor integration

---

## Technical Considerations

### Latency
- MVP: Fixed latency from oversampling (acceptable for effects)
- Future: Linear-phase vs minimum-phase options, latency compensation

### Thread Safety
- CircuitState is not Send (contains processing buffers)
- Each voice needs independent CircuitState
- Parameter changes via atomic shared state

### SIMD Optimization
- Current: Scalar floating-point
- Future: Auto-vectorization hints, explicit SIMD for matrix ops

### Preset Format
```json
{
  "circuit": "tube-screamer",
  "version": "1.0.0",
  "parameters": {
    "R1": 51000.0,
    "C2": 2.2e-8
  },
  "component_tolerances": {
    "R1": 0.05  // ±5% for vintage character
  }
}
```

---

## Open Questions

1. **Should we support headless mode?** (No GUI, just the DSP)
2. **How to handle polyphonic circuits?** (Multiple circuit instances?)
3. **MIDI learn for circuit parameters?** (Auto-map CC to components)
4. **Circuit modularity?** (Swap in/out different stages)
5. **Offline processing?** (Higher quality, non-realtime oversampling)

---

## Related Work

- **Guitarix**: GTK UI, limited customization
- **Neural DSP**: Proprietary, no user circuits
- **JUCE**: Full control but requires C++ knowledge
- **NIH-PLUG**: Our foundation, Rust-native

Our niche: **User-defined circuits with professional plugin integration**
