# Type O Negative Style Distortion Circuit: Research & Recommendations

**Date:** 2026-02-23  
**Status:** Research Summary  
**Purpose:** Circuit design recommendations for capturing the Type O Negative guitar tone in a DIY distortion pedal

---

## Executive Summary

Type O Negative's signature guitar tone—most notably on *Bloody Kisses* (1994) and *October Rust* (1996)—is characterized by massive low-end saturation, exceptional clarity despite high gain, and a distinctive "widened" quality from parallel chorus processing. The tone is downtuned (B Standard), thick with sustain, yet remarkably articulate for doom/gothic metal.

This document analyzes the key circuits and provides recommendations for building a Type O-style distortion pedal.

---

## The Type O Negative Sound Signature

### Key Characteristics

| Element | Description |
|---------|-------------|
| **Gain Level** | Very high, but controlled—not fizzy |
| **Low End** | Massive, extended, punchy |
| **Mids** | Scooped but present—"hollow" without being "lost" |
| **Highs** | Present for clarity, but not harsh |
| **Note Definition** | Surprisingly articulate for such high gain |
| **Special Sauce** | Chorus always-on, mixed in parallel |

### Kenny Hickey's Signal Chain (Circa 1994-1996)

```
Guitar (Gibson Flying V / SG / Fernandes Raven) → Chorus → Preamp → Power Amp
                                    ↓
                            Rack Effects (TC G-Force, Alesis Quadraverb)
```

**Key Insight:** The tone wasn't just the distortion—it was the **interaction** of:
1. Downtuned mahogany-body guitars with humbuckers
2. **Always-on chorus** at slow rate (creates stereo width)
3. Digital preamps (Marshall JMP-1 or ADA MP-1) driving tube power amps
4. Massive speakers (4x12 cabs) for low-end projection

### Tuning & Setup
- **Tuning:** B Standard (B-E-A-D-F#-B)
- **Scale Length:** Baritone (26.5") or standard with heavy strings
- **Pickups:** High-output humbuckers

---

## Candidate Circuits Analysis

### 1. Digitech Death Metal (DDM) — Primary Recommendation

**Era:** Designed 1992  
**Colors:** Red body, black graphics  
**Circuit Family:** Modified Boss Metal Zone / DOD Grunge architecture

#### Characteristics
- Advertised as providing "death metal musicians with a wall of sound"
- Optimized for low tunings with enhanced low-frequency response
- Simplified control set compared to Metal Zone
- "Artificial" sounding in a good way for gothic metal

#### Circuit Architecture
The Death Metal is believed to be derived from the Boss MT-2 Metal Zone circuit with modifications:
- Reduced EQ complexity
- Fixed bass boost in the 100-200Hz range
- Aggressive high-gain stages

**Verdict:** Most likely candidate for the "black & red digital pedal" described. The Digitech Death Metal is specifically voiced for extreme metal with extended low end.

---

### 2. Boss HM-2 "Heavy Metal" (The Chainsaw)

**Era:** 1983-1991 (original), 2020+ (Waza reissue)  
**Colors:** Black with red knobs and lettering  
**Circuit Family:** Unique Boss design

#### Characteristics
- Iconic "Swedish chainsaw" tone (Entombed, Dismember)
- Massive high-gain with distinctive EQ curve
- Extreme mid-scoop ("smile curve")
- Built-in speaker simulation filter

#### Circuit Analysis
The HM-2 uses a unique multi-stage design:
1. **Input buffer** — FET buffer for high impedance
2. **Pre-distortion EQ** — Complex passive network creating mid-scoop
3. **Gain stage** — Op-amp with variable gain (x20 to x100+)
4. **Hard clipping** — Silicon diodes for aggressive saturation
5. **Post-distortion EQ** — Second scooping network
6. **Output buffer** — FET-based

**Key Circuit Elements:**
- Twin-T notch filters for mid-scooping
- Unique "Bandaxall" style tone controls
- Output stage with cab-sim characteristics

#### Modifications for Type O Tone
To make an HM-2 more Type O-esque:
- Reduce the extreme high-cut (increase C3/C4 values slightly)
- Reduce the depth of the mid-scoop (modify R9/R10 ratio)
- Add more low-end retention (increase input cap to 47nF or 100nF)

**Verdict:** Excellent foundation, but the HM-2 is more "chainsaw aggressive" than Type O's "gothic wall." Requires modifications for the doomier, more sustained character.

---

### 3. Boss MT-2 Metal Zone

**Era:** Released 1991  
**Colors:** Black with red accents around knobs  
**Circuit Family:** Complex multi-stage op-amp design

#### Characteristics
- Most sophisticated EQ section of any mass-market distortion pedal
- Dual-stage clipping with pre- and post-EQ
- Semi-parametric mid control
- Highly sculptable but challenging to dial in

#### Circuit Architecture (Per Electric Druid Analysis)

**Stage 1: Input Buffer**
- FET buffer, 1MΩ input impedance
- DC blocking capacitor (C042/47n)

**Stage 2: Pre-Distortion Tone Shaping**
- High-pass filter at 106Hz (C033/15n + R043/100K)
- Gyrator-based bandpass filter centered at ~1kHz
- Creates "mid hump" to shape distortion character
- Low-pass filter at 340Hz (R045 + C031)

**Stage 3: Gain Stage & Clipping**
- Non-inverting op-amp with gain range x2 to x252
- AC-coupling at 48Hz (C029/33n + R040/100K)
- Silicon diode hard clipping (±0.6V)
- Softening resistor (R033/2K2)

**Stage 4: Post-Distortion Tone Shaping**
- Dual gyrator filters:
  - Left: 4.9kHz center, Q=2.2
  - Right: 105Hz center, Q=14.6
- Creates double-peaked frequency response

**Stage 5: EQ Section**
- High/Low: Cut/boost controls (±20dB)
- Mid: Semi-parametric with frequency sweep

**Verdict:** Can achieve Type O tones with careful EQ settings, but the circuit is overly complex for the specific sound. Better as a " Swiss army knife" than a dedicated Type O circuit.

---

## Recommended Circuit Designs

### Design A: "Bloody Kisses" Distortion (Recommended)

A simplified, optimized circuit specifically voiced for downtuned doom/gothic metal.

#### Architecture

```
Input → Buffer → Pre-EQ → Gain Stage 1 → Soft Clip → 
Gain Stage 2 → Hard Clip → Post-EQ → Volume → Output
```

#### Stage Details

**1. Input Buffer (FET)**
```
J201 JFET source follower
Rin = 1MΩ
Cin = 100nF (preserves low end for B Standard tuning)
```

**2. Pre-Distortion EQ (Gyrator)**
```
Op-amp: TL072
Create gentle mid hump at 800Hz-1kHz:
  R1 = 22kΩ, R2 = 1kΩ
  C1 = 10nF, C2 = 1nF
Fc ≈ 720Hz, Q ≈ 2
```

**3. Gain Stage 1 (Soft Clipping)**
```
Op-amp: TL072
Gain: 20-50x (variable)
Clipping: 2x Red LEDs (3mm)
  Softer, more "amp-like" compression
Rseries = 1kΩ (current limiting)
```

**4. Gain Stage 2 (Hard Clipping)**
```
Op-amp: TL072
Gain: 10x (fixed)
Clipping: 2x 1N4148 silicon diodes
  Provides aggressive edge
Rsoft = 2.2kΩ (softening resistor)
```

**5. Post-Distortion EQ (Tone Stack)**
```
 Baxandall-style tone stack with modifications:
 - Bass: ±12dB at 120Hz
 - Mid: Fixed scoop at 400Hz (-6dB)
 - Treble: ±12dB at 3kHz

Key: The fixed mid-scoop is crucial for the Type O sound
```

**6. Output**
```
Volume pot: 100kΩ log
Output cap: 10μF electrolytic
```

#### Key Design Decisions

| Feature | Value | Rationale |
|---------|-------|-----------|
| Input cap | 100nF | Preserves low B-string fundamentals (~60Hz) |
| LED clipping | 2x red LEDs | Softer, more musical compression |
| Dual gain stages | 20-50x then 10x | Allows controlled saturation |
| Fixed mid-scoop | -6dB at 400Hz | The "Type O curve" |
| Bass control | ±12dB @ 120Hz | Tune for different guitars |

---

### Design B: "October Rust" HM-2 Mod

Modified HM-2 circuit optimized for doom rather than death metal.

#### Changes from Stock HM-2

| Component | Stock | Modified | Effect |
|-----------|-------|----------|--------|
| C3 (input) | 10nF | 47nF | More low-end retention |
| R9/R10 | 15k/15k | 22k/10k | Reduced mid-scoop depth |
| C13 | 1nF | 2.2nF | Less extreme high-cut |
| C15 | 10nF | 22nF | Fuller bass response |

#### Additional Mod: Low-End Focus Switch
Add a DPDT switch to select between:
- **Stock:** Original HM-2 chainsaw character
- **Type O:** Enhanced low end, reduced scoop

---

### Design C: "Gothic Chorus" Blending Circuit

Since the chorus is essential to the Type O sound, here's a circuit that integrates parallel chorus blending.

#### Concept
```
                    ┌→ Distortion →┐
Input → Splitter →  │              ├──→ Mixer → Output
                    └→ Chorus ────→┘
```

#### Implementation

**1. Input Splitter**
- Simple op-amp splitter or passive resistive divider
- Provides two phase-identical signals

**2. Distortion Path**
- Use Design A or Design B above
- Gain: Unity at output (level-matched)

**3. Chorus Path**
- PT2399-based analog chorus (common DIY circuit)
- Rate: 0.3-0.6Hz (slow)
- Depth: 30-40% (subtle)
- Level: -6dB relative to distortion (blend to taste)

**4. Mixer Stage**
- Summing op-amp mixer
- Individual level controls for each path
- Master volume

**Key Insight:** The parallel chorus adds width and "3D-ness" without the wobbly seasick effect of 100% wet chorus.

---

## Component Recommendations

### Op-Amps
| Part | Character | Best For |
|------|-----------|----------|
| **TL072** | FET-input, low noise, neutral | General purpose |
| **NE5532** | Low noise, more "hi-fi" | Output stages |
| **LM833** | Low noise, bipolar input | Alternative flavor |
| **OPA2134** | Premium FET, very quiet | High-end builds |

### Clipping Diodes
| Type | Forward Voltage | Character |
|------|-----------------|-----------|
| **1N4148** | 0.6-0.7V | Hard, aggressive |
| **1N4001** | 0.6-0.7V | Similar to 1N4148 |
| **Red LED** | 1.8-2.0V | Soft, amp-like |
| **BAT46** | 0.3V | Very soft, low output |
| **Asymmetric** | Mixed | Complex harmonics |

### Recommended Configuration
- **Stage 1 (soft clip):** 2x Red LEDs
- **Stage 2 (hard clip):** 2x 1N4148
- **Result:** Controlled compression + aggressive edge

### Potentiometers
| Function | Value | Taper |
|----------|-------|-------|
| Gain | 100kΩ | Audio/Log |
| Volume | 100kΩ | Audio/Log |
| Bass | 100kΩ | Linear |
| Treble | 100kΩ | Linear |
| Blend (if chorus) | 100kΩ | Linear |

---

## Control Layout Recommendations

### Minimal Controls (Simple Build)
```
┌─────────────────────────────┐
│  TYPE-O DISTORTION          │
│                             │
│  [GAIN]  [TONE]  [LEVEL]    │
│                             │
│  [CHORUS MIX] (optional)    │
└─────────────────────────────┘
```

### Full Controls (Complete Build)
```
┌─────────────────────────────┐
│  TYPE-O DISTORTION          │
│                             │
│  [GAIN]      [TONE]         │
│  [BASS]      [TREBLE]       │
│  [MIDS]      [LEVEL]        │
│                             │
│  [CHORUS] toggle or blend   │
└─────────────────────────────┘
```

---

## EQ Settings Guide

### For B Standard / Drop A Tuning
| Control | Setting | Notes |
|---------|---------|-------|
| Gain | 2:00-3:00 | High but controlled |
| Bass | 2:00-3:00 | Embrace the low end |
| Mids | 10:00-11:00 | Scooped but present |
| Treble | 1:00-2:00 | Clarity without harshness |
| Level | Unity | Match bypassed volume |

### For Standard E Tuning
| Control | Setting | Notes |
|---------|---------|-------|
| Gain | 1:00-2:00 | Less gain needed |
| Bass | 12:00-1:00 | Less low-end boost |
| Mids | 11:00-12:00 | Slightly more mids |
| Treble | 1:00-2:00 | Similar |

---

## Power Supply Considerations

- **Standard:** 9V DC, center-negative (Boss-style)
- **Current draw:** ~15-25mA typical
- **Headroom:** 9V is sufficient, but 18V can provide more dynamic range if op-amps support it

---

## Enclosure & Aesthetics

### Color Scheme (Homage to Type O)
- **Black enclosure** with **red** knobs/graphics
- **Green** LED (Peter Steele's signature color)
- Or: **Red** enclosure with **black** graphics (Digitech Death Metal homage)

### Size
- 1590B (standard) for simple builds
- 1590BB (double-wide) for chorus blend version

---

## References & Resources

### Schematics
- Boss HM-2: Available on FreeStompBoxes.org
- Boss MT-2: Electric Druid analysis (electricdruid.net)
- Digitech Death Metal: Derivative of DOD FX86B Grunge

### Research Sources
- Kenny Hickey interviews (Guitar World 2023)
- Equipboard gear listings
- Reddit r/typeonegative community
- MusicStrive tone analysis

### DIY Community
- FreeStompBoxes.org
- DIYStompboxes.com
- PedalPCB.com (for PCB layouts)

---

## Conclusion

The Type O Negative guitar tone is achievable through several circuit approaches:

1. **For authenticity:** Build the "Bloody Kisses" circuit (Design A) with dual gain stages and carefully tuned EQ
2. **For simplicity:** Modify a Boss HM-2 with the component changes outlined
3. **For completeness:** Include the parallel chorus blend (Design C) for the full "widened wall" effect

The key is **low-end retention** combined with **controlled mid-scoop** and **high-gain saturation** that doesn't become fizzy. The Type O sound is about power and clarity—not about being the most distorted.

*"The spice must flow."*
