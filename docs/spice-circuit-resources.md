# SPICE Circuit Resources for Testing

## Open Source Circuit Libraries

### 1. Guitarix (GPL License) - ⭐ Primary Resource
**Repository:** https://github.com/brummer10/guitarix

**Location of circuits:**
- `trunk/tools/ampsim/DK/` - DK-method implementations
- `trunk/tools/ampsim/` - Circuit files

**Available Circuits:**
- Tube Screamer (TS9)
- Big Muff Pi
- Fuzz Face
- Marshall-style amplifier
- Fender-style amplifier
- Various 12AX7 tube preamps

### 2. LTspice Community
**Resources:**
- Youspice: https://www.youspice.com/
- LTspice Yahoo Groups (archived)

**Recommended Circuits:**
- "TS9 Tone Circuit.asc" - Tone stack only
- "Marshall 1959 Pre.asc" - Classic preamp
- "Tube Screamer Clone.asc" - Verified working

### 3. Falstad Circuit Simulator
**URL:** https://www.falstad.com/circuit/

**Features:**
- Large library of circuits
- Can export to SPICE netlist format
- Good for: Guitar distortion, op-amp amplifiers

## Recommended Test Circuits (by Complexity)

### Low Complexity
**Fender Twin Tone Stack**
- Type: Passive RLC filter
- Tests: Frequency response accuracy
- Why: Good for validating linear solver

### Low-Medium Complexity
**Tube Screamer (TS9)**
- Components: 1 op-amp, 2 diodes, tone stack
- Nonlinear devices: 2 (diodes)
- Why: "Hello world" of pedal modeling

**Fuzz Face**
- Components: 2 transistors
- Nonlinear devices: 2 (BJTs)
- Why: Tests BJT biasing and simple feedback

### Medium Complexity
**Big Muff Pi**
- Components: 4 transistors, tone stack, diodes
- Nonlinear devices: 4+
- Why: Good stress test for convergence

**Marshall JCM800 Preamp**
- Components: 2-3 tube stages
- Nonlinear devices: 3+ (tubes)
- Why: Tests high gain, multiple nonlinear devices

## Quick Test Circuit: Minimal Tube Screamer

```spice
* Tube Screamer Style Overdrive
.model D1N4148 D(IS=2.52e-9 RS=0.568 N=1.906)

* Input buffer
Rin in n1 10k

* Clipper stage
R1 n1 n2 51k
D1 n2 out D1N4148
D2 out n2 D1N4148

* Output filter
R2 out 0 10k
C1 out 0 0.047u

.END
```

## Notes on Copyright

**Safe to use:**
- Circuit topologies (resistor between points A and B)
- Component values
- Basic electronic principles

**Avoid:**
- Exact schematic drawings from service manuals
- Trademarked names (Tube Screamer™, Big Muff™)
- Proprietary model files

**Best practice:**
- Use "inspired by" language
- Create original teaching circuits
- Reference expired patents (pre-1990s circuits)

## Testing Priority

1. **Start with:** RC filters (validate linear solver)
2. **Then:** Diode clippers (validate 1D NR)
3. **Then:** Tube Screamer (validate full pipeline)
4. **Finally:** Complex amps (stress test)
