# Real-Time Audio DSP Safety and Best Practices

> A practical guide for hard real-time audio programming, with focus on Rust implementation patterns.

**Target Audience:** AI agents and developers implementing DSP code for melange.  
**Scope:** Hard real-time constraints (< 1ms latency, deterministic execution) for audio plugin contexts.

---

## Table of Contents

1. [Real-Time Constraints](#1-real-time-constraints-latency--determinism)
2. [No-Heap-Allocation in Audio Thread](#2-no-heap-allocation-in-audio-thread)
3. [Lock-Free Programming Patterns](#3-lock-free-programming-patterns)
4. [Cache-Friendly Data Structures](#4-cache-friendly-data-structures)
5. [Branch Prediction and Hot Loops](#5-branch-prediction-and-hot-loops)
6. [Denormal Numbers and Performance](#6-denormal-numbers-and-performance)
7. [SIMD Considerations for DSP](#7-simd-considerations-for-dsp)
8. [Rust-Specific RT Safety](#8-rust-specific-rt-safety)

---

## 1. Real-Time Constraints: Latency & Determinism

### The Hard Constraint

Audio hardware demands samples at fixed intervals:

| Sample Rate | Buffer Size | Time Budget | Typical Use Case |
|-------------|-------------|-------------|------------------|
| 44.1 kHz    | 64 samples  | 1.45 ms     | Live monitoring  |
| 44.1 kHz    | 128 samples | 2.90 ms     | Low-latency DAW  |
| 44.1 kHz    | 512 samples | 11.6 ms     | Mixing           |

**Rule:** Your DSP callback MUST complete within the time budget, every single time. No exceptions. No "usually fast enough."

### The Jitter Problem

```rust
// BAD: Variable execution time
fn process_block(&mut self, buffer: &mut [&mut [f64]]) {
    for channel in buffer {
        for sample in channel.iter_mut() {
            // Branch misprediction, cache miss, allocation...
            // Any of these cause jitter
            *sample = self.expensive_maybe_branching_op(*sample);
        }
    }
}
```

**Guideline:** Worst-case execution time (WCET) must fit within budget, not average case.

### Deterministic Workloads

```rust
// GOOD: Fixed cost per sample
fn process_block(&mut self, buffer: &mut [&mut [f64]]) {
    // Pre-allocated, fixed-size state
    const N: usize = 4; // Number of biquad sections
    
    for channel_idx in 0..buffer.len() {
        let channel = &mut buffer[channel_idx];
        
        // Loop bounds are compile-time constants
        for i in 0..channel.len() {
            let x = channel[i];
            let mut y = x;
            
            // Unrolled, predictable filter cascade
            for s in 0..N {
                y = self.biquads[s].process_sample(y);
            }
            
            channel[i] = y;
        }
    }
}
```

**Key Principles:**
- No loops with data-dependent bounds
- No recursion (stack depth must be bounded)
- No dynamic dispatch in hot paths
- No syscalls (including memory allocation)

---

## 2. No-Heap-Allocation in Audio Thread

### Why Allocation Is Forbidden

Heap allocation can:
1. **Block** waiting for the allocator lock (contention)
2. **Trigger** a system call (`mmap`, `brk`)
3. **Cause** page faults on first access
4. **Take** unbounded time depending on heap fragmentation

All of these violate real-time constraints.

### The Zero-Allocation Pattern

```rust
// GOOD: Pre-allocated, stack or struct-based storage
pub struct DelayLine<const MAX_DELAY: usize> {
    buffer: [f64; MAX_DELAY],  // Fixed-size array
    write_idx: usize,
}

impl<const MAX_DELAY: usize> DelayLine<MAX_DELAY> {
    pub fn new() -> Self {
        Self {
            buffer: [0.0; MAX_DELAY],
            write_idx: 0,
        }
    }
    
    pub fn process_sample(&mut self, input: f64, delay_samples: usize) -> f64 {
        // Bounds check in debug, unchecked in release
        debug_assert!(delay_samples < MAX_DELAY);
        
        let read_idx = (self.write_idx + MAX_DELAY - delay_samples) % MAX_DELAY;
        let output = self.buffer[read_idx];
        
        self.buffer[self.write_idx] = input;
        self.write_idx = (self.write_idx + 1) % MAX_DELAY;
        
        output
    }
}
```

### Real-Time Safe Resizing

If you need variable-sized buffers, pre-allocate the maximum:

```rust
pub struct VariableDelay {
    buffer: Box<[f64]>,  // Allocated at init, never resized
    max_delay: usize,
    current_delay: usize,
    write_idx: usize,
}

impl VariableDelay {
    /// Called from non-real-time thread (UI/parameter change)
    pub fn set_delay_time(&mut self, samples: usize) {
        // Clamping, not allocating
        self.current_delay = samples.min(self.max_delay);
    }
    
    /// Called from audio thread - ZERO allocation
    pub fn process_sample(&mut self, input: f64) -> f64 {
        // ... same as above, uses self.current_delay
    }
}
```

### Static Allocation for Known Topologies

```rust
// For melange: circuit topology is compile-time known
pub struct DkSolver<const N_NODES: usize, const N_NONLINEAR: usize> {
    // G and C matrices: fixed size, stack-allocated
    g_matrix: [[f64; N_NODES]; N_NODES],
    c_matrix: [[f64; N_NODES]; N_NODES],
    
    // State vectors
    voltage: [f64; N_NODES],
    current: [f64; N_NODES],
    
    // Nonlinear element states
    nonlinear_prev: [f64; N_NONLINEAR],
}

// Example: 4-node preamp with 2 nonlinear elements (BJTs)
type PreampSolver = DkSolver<4, 2>;
```

### The `#[deny(clippy::allocation)]` Pattern

While clippy doesn't have a specific allocation lint, structure code to make allocation obvious:

```rust
use std::mem::MaybeUninit;

/// Audio thread context - anything that might allocate is marked `unsafe`
pub struct AudioThreadContext {
    // ... fields
}

impl AudioThreadContext {
    /// # Safety
    /// Must not be called from real-time thread if it allocates
    pub unsafe fn resize_buffers(&mut self, new_size: usize) {
        // Allocation happens here, caller must ensure RT safety
        self.buffer = vec![0.0; new_size].into_boxed_slice();
    }
    
    /// Always RT-safe, never allocates
    pub fn process(&mut self, input: f64) -> f64 {
        // ...
    }
}
```

---

## 3. Lock-Free Programming Patterns

### The Problem with Locks

Standard mutex/semaphore can block indefinitely:
- Priority inversion (low-priority thread holds lock, high-priority waits)
- Contention (multiple threads competing)
- Kernel scheduling delays

### Lock-Free Pattern: Single-Producer Single-Consumer (SPSC) Queue

```rust
use std::sync::atomic::{AtomicUsize, Ordering};
use std::mem::MaybeUninit;

/// Lock-free SPSC ring buffer for parameter updates
/// Capacity must be power of 2 for mask optimization
pub struct SpscRingBuffer<T, const CAPACITY: usize> {
    // Atomic indices for lock-free access
    head: AtomicUsize,  // Write index (producer only)
    tail: AtomicUsize,  // Read index (consumer only)
    
    // Storage
    buffer: [MaybeUninit<T>; CAPACITY],
}

impl<T: Copy, const CAPACITY: usize> SpscRingBuffer<T, CAPACITY> {
    const MASK: usize = CAPACITY - 1;
    
    pub fn new() -> Self {
        assert!(CAPACITY.is_power_of_two(), "Capacity must be power of 2");
        
        Self {
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            buffer: unsafe { MaybeUninit::zeroed().assume_init() },
        }
    }
    
    /// Called from UI thread (producer)
    pub fn push(&self, value: T) -> bool {
        let head = self.head.load(Ordering::Relaxed);
        let next = (head + 1) & Self::MASK;
        
        // Check if full
        if next == self.tail.load(Ordering::Acquire) {
            return false; // Buffer full
        }
        
        // Store value
        unsafe {
            (*self.buffer[head].as_ptr()).write(value);
        }
        
        // Publish with release ordering
        self.head.store(next, Ordering::Release);
        true
    }
    
    /// Called from audio thread (consumer) - LOCK FREE
    pub fn pop(&self) -> Option<T> {
        let tail = self.tail.load(Ordering::Relaxed);
        
        // Check if empty
        if tail == self.head.load(Ordering::Acquire) {
            return None;
        }
        
        // Read value
        let value = unsafe { (*self.buffer[tail].as_ptr()).assume_init_read() };
        
        // Update tail with release ordering
        self.tail.store((tail + 1) & Self::MASK, Ordering::Release);
        
        Some(value)
    }
}
```

### Double-Buffering for Parameters

```rust
use std::sync::atomic::{AtomicPtr, Ordering};
use std::boxed::Box;

/// Atomic parameter swap - UI writes new values, audio thread reads
pub struct AtomicParameters<T> {
    // Pointer to current params (audio thread reads this)
    current: AtomicPtr<T>,
    
    // Back buffer for UI to prepare updates
    back_buffer: Box<T>,
}

impl<T: Clone> AtomicParameters<T> {
    pub fn new(initial: T) -> Self {
        let boxed = Box::new(initial);
        let ptr = Box::into_raw(boxed);
        
        Self {
            current: AtomicPtr::new(ptr),
            back_buffer: Box::new(unsafe { (*ptr).clone() }),
        }
    }
    
    /// UI thread: prepare new parameter set
    pub fn prepare_update(&mut self, update_fn: impl FnOnce(&mut T)) {
        update_fn(&mut self.back_buffer);
    }
    
    /// UI thread: atomically publish new parameters
    pub fn commit_update(&mut self) {
        let new_ptr = Box::into_raw(Box::new(self.back_buffer.clone()));
        let old_ptr = self.current.swap(new_ptr, Ordering::AcqRel);
        
        // Defer deallocation (or use hazard pointers)
        // In practice: drop on next UI frame or use epoch-based reclamation
        unsafe { Box::from_raw(old_ptr) }; // Simplified - see note below
    }
    
    /// Audio thread: read current parameters - LOCK FREE
    pub fn read(&self) -> &T {
        let ptr = self.current.load(Ordering::Acquire);
        unsafe { &*ptr }
    }
}

// NOTE: The above has a safety issue - audio thread may still be reading
// when we free. Production code needs epoch-based reclamation (crossbeam-epoch)
// or a two-slot lock-free structure with generation counters.
```

### Simple Two-Slot Lock-Free Buffer

```rust
/// Guaranteed lock-free, no allocation, safe reclamation
pub struct LockFreeDoubleBuffer<T: Copy> {
    // Two slots: [0] and [1]
    slots: [T; 2],
    
    // Index of the slot currently being read by audio thread
    read_index: AtomicUsize,
    
    // Index of the slot being written by UI thread
    write_index: AtomicUsize,
}

impl<T: Copy + Default> LockFreeDoubleBuffer<T> {
    pub fn new(initial: T) -> Self {
        Self {
            slots: [initial, initial],
            read_index: AtomicUsize::new(0),
            write_index: AtomicUsize::new(1),
        }
    }
    
    /// UI thread: write new value
    pub fn write(&self, value: T) {
        // Write to the slot not being read
        let write_idx = self.write_index.load(Ordering::Relaxed);
        self.slots[write_idx] = value;
        
        // Swap: new value becomes readable, old read slot becomes writable
        let old_read = self.read_index.swap(write_idx, Ordering::AcqRel);
        self.write_index.store(old_read, Ordering::Relaxed);
    }
    
    /// Audio thread: read current value - LOCK FREE, wait-free
    pub fn read(&self) -> T {
        let idx = self.read_index.load(Ordering::Acquire);
        self.slots[idx]
    }
}
```

### Memory Ordering Guidelines

| Pattern | Producer | Consumer |
|---------|----------|----------|
| SPSC Ring Buffer | `Release` on write index | `Acquire` on read index |
| Double Buffer | `AcqRel` on swap | `Acquire` on read |
| Simple Flag | `Release` | `Acquire` |

**Rule:** Use `Relaxed` ONLY for indices within the same thread. Cross-thread communication requires `Release`/`Acquire` pairs.

---

## 4. Cache-Friendly Data Structures

### Cache Hierarchy Awareness

| Level | Size | Latency | Audio Thread Strategy |
|-------|------|---------|----------------------|
| L1    | 32KB | ~4 cycles | Keep hot data here |
| L2    | 256KB | ~12 cycles | Fit entire DSP state |
| L3    | 8MB+ | ~40 cycles | Avoid during callback |
| RAM   | GB   | ~200 cycles | Pre-fetch or avoid |

### Structure of Arrays (SoA) vs Array of Structures (AoS)

```rust
// BAD: Array of Structures - cache unfriendly for vectorized ops
pub struct BiquadAoS {
    sections: Vec<BiquadSection>,  // { a1, a2, b0, b1, b2, z1, z2 }
}

// Cache behavior: accessing a1, then next section's a1 = cache miss
// (stride = sizeof(BiquadSection) > cache line)

// GOOD: Structure of Arrays - cache friendly
pub struct BiquadSoA<const N: usize> {
    a1: [f64; N],
    a2: [f64; N],
    b0: [f64; N],
    b1: [f64; N],
    b2: [f64; N],
    z1: [f64; N],  // State
    z2: [f64; N],  // State
}

// Cache behavior: sequential access to a1[0], a1[1], ... = cache hit
// (sequential access pattern, prefetcher friendly)
```

### Hot/Cold Splitting

```rust
// BAD: Mixing hot and cold data
pub struct BiquadMixed {
    // Accessed every sample (HOT)
    a1: f64,
    a2: f64,
    b0: f64,
    b1: f64,
    b2: f64,
    z1: f64,
    z2: f64,
    
    // Accessed rarely (COLD) - wastes cache line space
    name: String,
    parameter_id: u32,
    smoothing_coeff: f64,
}

// GOOD: Separate hot and cold
pub struct BiquadCore {
    // Fits in 64 bytes (one cache line)
    a1: f64,
    a2: f64,
    b0: f64,
    b1: f64,
    b2: f64,
    z1: f64,
    z2: f64,
}

pub struct Biquad {
    core: BiquadCore,           // Hot - keep together
    params: BiquadParams,       // Cold - separate allocation
}
```

### Linear Memory Access Patterns

```rust
// BAD: Random/linked access
pub struct LinkedListDelayLine {
    nodes: Vec<Box<DelayNode>>,  // Indirection, cache misses
}

// GOOD: Contiguous array
pub struct ContiguousDelayLine<const N: usize> {
    buffer: [f64; N],  // One allocation, linear access
    write_idx: usize,
}

// GOOD: Pre-computed read indices for variable tap delays
pub struct MultiTapDelay<const MAX_DELAY: usize, const N_TAPS: usize> {
    buffer: [f64; MAX_DELAY],
    // Pre-computed offsets for each tap
    tap_offsets: [usize; N_TAPS],
    write_idx: usize,
}
```

### False Sharing Prevention

```rust
// BAD: Multiple threads modifying adjacent fields
pub struct SharedStateBad {
    thread_a_counter: AtomicU64,  // Cache line 0
    thread_b_counter: AtomicU64,  // Same cache line!
}

// GOOD: Pad to cache line boundaries (typically 64 bytes)
#[repr(align(64))]
pub struct PaddedAtomic {
    value: AtomicU64,
    _padding: [u8; 56],  // 64 - 8 = 56 bytes padding
}

pub struct SharedStateGood {
    thread_a_counter: PaddedAtomic,
    thread_b_counter: PaddedAtomic,
}
```

---

## 5. Branch Prediction and Hot Loops

### The Cost of Branch Misprediction

Modern CPUs speculate on branch direction. Misprediction costs 15-25 cycles.

At 44.1 kHz with 64-sample buffer:
- Total budget: 1,451 µs
- Cycles at 3 GHz: ~4,353,000 cycles
- Per sample: ~68,000 cycles

A few mispredictions per sample is acceptable, but avoid unpredictable branches in inner loops.

### Avoiding Branches in DSP Code

```rust
// BAD: Unpredictable branch per sample
pub fn process_with_branch(&mut self, input: f64) -> f64 {
    if input > self.threshold {
        self.compress(input)
    } else {
        input
    }
}

// GOOD: Branchless using select
#[inline(always)]
pub fn process_branchless(&mut self, input: f64) -> f64 {
    let over_threshold = (input > self.threshold) as i64 as f64;
    let compressed = self.compress(input);
    
    // Branchless: result = over ? compressed : input
    over_threshold * compressed + (1.0 - over_threshold) * input
}
```

### Lookup Tables for Complex Functions

```rust
/// Branchless, cache-friendly tanh approximation using LUT
pub struct TanhLut<const SIZE: usize> {
    table: [f64; SIZE],
    scale: f64,      // Input scaling to table index
    inv_scale: f64,  // Table index to input
}

impl<const SIZE: usize> TanhLut<SIZE> {
    pub fn new(max_input: f64) -> Self {
        let mut table = [0.0; SIZE];
        for i in 0..SIZE {
            let x = (i as f64 / (SIZE - 1) as f64) * 2.0 * max_input - max_input;
            table[i] = x.tanh();
        }
        
        Self {
            table,
            scale: ((SIZE - 1) as f64) / (2.0 * max_input),
            inv_scale: (2.0 * max_input) / ((SIZE - 1) as f64),
        }
    }
    
    /// Branchless tanh - predictably ~10 cycles
    #[inline(always)]
    pub fn tanh(&self, x: f64) -> f64 {
        // Clamp to table range
        let clamped = x.clamp(-self.inv_scale * (SIZE - 1) as f64, 
                               self.inv_scale * (SIZE - 1) as f64);
        
        // Calculate index
        let idx_f = (clamped + self.inv_scale * (SIZE - 1) as f64) * self.scale;
        let idx = idx_f as usize;
        let frac = idx_f - idx as f64;
        
        // Linear interpolation (branchless)
        let idx = idx.min(SIZE - 2);
        let y0 = self.table[idx];
        let y1 = self.table[idx + 1];
        
        y0 + frac * (y1 - y0)
    }
}
```

### Loop Unrolling and Software Pipelining

```rust
// Compiler may not unroll optimally for DSP
pub fn process_4x_unrolled(&mut self, buffer: &mut [f64]) {
    let n = buffer.len();
    let mut i = 0;
    
    // Process 4 samples at a time
    while i + 3 < n {
        let s0 = self.process_single(buffer[i]);
        let s1 = self.process_single(buffer[i + 1]);
        let s2 = self.process_single(buffer[i + 2]);
        let s3 = self.process_single(buffer[i + 3]);
        
        buffer[i] = s0;
        buffer[i + 1] = s1;
        buffer[i + 2] = s2;
        buffer[i + 3] = s3;
        
        i += 4;
    }
    
    // Tail
    while i < n {
        buffer[i] = self.process_single(buffer[i]);
        i += 1;
    }
}
```

### Likely/Unlikely Hints (Nightly Rust)

```rust
#![feature(core_intrinsics)]
use std::intrinsics::{likely, unlikely};

pub fn process_with_hint(&mut self, input: f64) -> f64 {
    // Compressor gain reduction is "usually" 1.0 (no reduction)
    if unlikely(self.gain_reduction < 0.99) {
        input * self.gain_reduction
    } else {
        input
    }
}
```

### Removing Bounds Checks

```rust
// BAD: Bounds check on every access
pub fn process_checked(&mut self, buffer: &[f64]) -> Vec<f64> {
    let mut output = Vec::with_capacity(buffer.len());
    for i in 0..buffer.len() {
        output.push(self.filter(buffer[i]));  // Bounds check
    }
    output
}

// GOOD: Iterator eliminates bounds checks
pub fn process_iter(&mut self, buffer: &[f64], output: &mut [f64]) {
    for (inp, out) in buffer.iter().zip(output.iter_mut()) {
        *out = self.filter(*inp);  // No bounds check
    }
}

// GOOD: Unchecked access with debug assertion
pub fn process_unchecked(&mut self, buffer: &mut [f64]) {
    debug_assert!(buffer.len() <= self.max_block_size);
    
    for i in 0..buffer.len() {
        unsafe {
            *buffer.get_unchecked_mut(i) = self.filter(*buffer.get_unchecked(i));
        }
    }
}
```

---

## 6. Denormal Numbers and Performance

### The Denormal Problem

Denormal (subnormal) floating-point numbers are those close to zero:
- Normal: 1.0e-38 to 1.0e38 (f32), 1.0e-308 to 1.0e308 (f64)
- Denormal: 1.0e-45 to 1.0e-38 (f32), 1.0e-324 to 1.0e-308 (f64)

**Problem:** Denormal operations are 10-100x slower than normal FP operations on many CPUs (no hardware acceleration, must be handled in microcode).

### Scenario: IIR Filter Decay

```rust
// A resonant lowpass with decaying input can hit denormals
pub fn process_iir(&mut self, input: f64) -> f64 {
    // After input goes to 0, state variables decay exponentially
    // Eventually they become denormal: 1e-310, 1e-311, ...
    // Each operation now 100x slower = CPU spike = glitch
    
    let output = self.b0 * input + self.b1 * self.z1 + self.b2 * self.z2
                 - self.a1 * self.z3 - self.a2 * self.z4;
    
    self.z2 = self.z1;
    self.z1 = input;
    self.z4 = self.z3;
    self.z3 = output;
    
    output
}
```

### Solution 1: DC Offset (Least Intrusive)

```rust
const DC_OFFSET: f64 = 1e-15;  // Well above denormal range

pub fn process_with_dc(&mut self, input: f64) -> f64 {
    // Add tiny DC, filter later or accept the inaudible error
    let x = input + DC_OFFSET;
    
    let output = self.b0 * x + self.b1 * self.z1 + self.b2 * self.z2
                 - self.a1 * self.z3 - self.a2 * self.z4;
    
    // ... update state
    
    output - DC_OFFSET  // Remove if needed (but often unnecessary)
}
```

### Solution 2: Flush-to-Zero (FTZ) / Denormals-Are-Zero (DAZ)

```rust
/// Enable FTZ/DAZ for x86/x86_64
/// Call once at plugin initialization
#[cfg(target_arch = "x86_64")]
pub fn disable_denormals() {
    use std::arch::x86_64::*;
    
    unsafe {
        // Read MXCSR
        let mxcsr = _mm_getcsr();
        
        // Set FTZ (flush to zero) and DAZ (denormals are zero) bits
        // FTZ = bit 15, DAZ = bit 6
        _mm_setcsr(mxcsr | 0x8040);
    }
}

/// Restore original state on plugin exit
#[cfg(target_arch = "x86_64")]
pub fn restore_denormals(original_mxcsr: u32) {
    use std::arch::x86_64::*;
    unsafe {
        _mm_setcsr(original_mxcsr);
    }
}
```

### Solution 3: Rust intrinsics (portable across architectures)

```rust
/// Cross-platform denormal handling
pub struct DenormalGuard {
    #[cfg(target_arch = "x86_64")]
    original_mxcsr: u32,
}

impl DenormalGuard {
    pub fn new() -> Self {
        #[cfg(target_arch = "x86_64")]
        {
            use std::arch::x86_64::*;
            let original = unsafe { _mm_getcsr() };
            unsafe { _mm_setcsr(original | 0x8040) };
            Self { original_mxcsr: original }
        }
        
        #[cfg(not(target_arch = "x86_64"))]
        {
            Self {}
        }
    }
}

impl Drop for DenormalGuard {
    fn drop(&mut self) {
        #[cfg(target_arch = "x86_64")]
        unsafe {
            std::arch::x86_64::_mm_setcsr(self.original_mxcsr);
        }
    }
}

// Usage in process callback
pub fn process(&mut self, buffer: &mut [f64]) {
    let _guard = DenormalGuard::new();
    
    // All denormals flushed to zero in this scope
    for sample in buffer.iter_mut() {
        *sample = self.filter.process(*sample);
    }
}
```

### Solution 4: Explicit Flush (Most Control)

```rust
/// Flush small values to zero manually
#[inline(always)]
pub fn flush_denormal(x: f64) -> f64 {
    const THRESHOLD: f64 = 1e-15;
    
    // Branchless: if |x| < threshold, return 0
    let abs_x = x.abs();
    let mask = ((abs_x >= THRESHOLD) as u64) << 63;  // Sign bit mask
    let bits = x.to_bits();
    f64::from_bits(bits & mask)
}

// Or simpler, less portable but effective:
#[inline(always)]
pub fn flush_denormal_simple(x: f64) -> f64 {
    const THRESHOLD: f64 = 1e-15;
    if x.abs() < THRESHOLD { 0.0 } else { x }
}
```

### Recommendations for melange

1. **Default:** Enable FTZ/DAZ at plugin initialization (covers all generated code)
2. **Fallback:** DC offset technique for specific filter structures
3. **Avoid:** Manual flush in inner loops (adds overhead)

---

## 7. SIMD Considerations for DSP

### When to Use SIMD

SIMD (Single Instruction Multiple Data) can provide 2-8x speedup for:
- Parallel filter processing (stereo, multi-band)
- Block-based convolution
- Sample-rate conversion
- FFT/iFFT

Overhead considerations:
- Setup/shuffle costs can outweigh benefits for small blocks
- Portability (x86 SSE/AVX, ARM NEON, WASM SIMD)
- Compilation complexity

### Auto-Vectorization Hints

```rust
// Compiler can auto-vectorize this pattern:
pub fn scale_buffer(buffer: &mut [f64], gain: f64) {
    // Ensure alignment for optimal SIMD
    for chunk in buffer.chunks_exact_mut(4) {
        // Multiple independent operations = vectorizable
        chunk[0] *= gain;
        chunk[1] *= gain;
        chunk[2] *= gain;
        chunk[3] *= gain;
    }
    
    // Tail
    for sample in buffer.chunks_exact_mut(4).into_remainder() {
        *sample *= gain;
    }
}
```

### Portable SIMD with `std::simd` (Nightly)

```rust
#![feature(portable_simd)]
use std::simd::*;

pub fn process_simd<const LANES: usize>(
    input: &[f64],
    output: &mut [f64],
    gain: f64
) where LaneCount<LANES>: SupportedLaneCount, {
    let gain_vec = Simd::<f64, LANES>::splat(gain);
    
    let chunks = input.chunks_exact(LANES);
    let remainder = chunks.remainder();
    
    for (in_chunk, out_chunk) in chunks.zip(output.chunks_exact_mut(LANES)) {
        let vec = Simd::<f64, LANES>::from_slice(in_chunk);
        let result = vec * gain_vec;
        result.copy_to_slice(out_chunk);
    }
    
    // Scalar tail
    for i in 0..remainder.len() {
        output[input.len() - remainder.len() + i] = remainder[i] * gain;
    }
}
```

### Platform-Specific Intrinsics

```rust
#[cfg(target_arch = "x86_64")]
pub mod x86_simd {
    use std::arch::x86_64::*;
    
    /// Process 4 f64 samples with AVX2
    #[target_feature(enable = "avx2")]
    pub unsafe fn scale_4x(input: &[f64], gain: __m256d) -> __m256d {
        let vec = _mm256_loadu_pd(input.as_ptr());
        _mm256_mul_pd(vec, gain)
    }
    
    /// Process 2 f64 samples with SSE2 (baseline x86_64)
    #[target_feature(enable = "sse2")]
    pub unsafe fn scale_2x(input: &[f64], gain: __m128d) -> __m128d {
        let vec = _mm_loadu_pd(input.as_ptr());
        _mm_mul_pd(vec, gain)
    }
}

#[cfg(target_arch = "aarch64")]
pub mod arm_simd {
    use std::arch::aarch64::*;
    
    /// Process 2 f64 samples with NEON
    #[target_feature(enable = "neon")]
    pub unsafe fn scale_2x(input: &[f64], gain: float64x2_t) -> float64x2_t {
        let vec = vld1q_f64(input.as_ptr());
        vmulq_f64(vec, gain)
    }
}
```

### Runtime Feature Detection

```rust
/// Auto-selected implementation based on CPU features
pub fn process_auto(buffer: &mut [f64], gain: f64) {
    #[cfg(target_arch = "x86_64")]
    {
        if is_x86_feature_detected!("avx2") {
            unsafe { process_avx2(buffer, gain) };
            return;
        }
        if is_x86_feature_detected!("sse2") {
            unsafe { process_sse2(buffer, gain) };
            return;
        }
    }
    
    #[cfg(target_arch = "aarch64")]
    {
        // NEON is baseline for aarch64
        unsafe { process_neon(buffer, gain) };
        return;
    }
    
    // Scalar fallback
    process_scalar(buffer, gain);
}
```

### Alignment Considerations

```rust
use std::alloc::{alloc, Layout};

/// Allocate aligned buffer for SIMD
pub fn allocate_aligned_f64(len: usize, align: usize) -> Box<[f64]> {
    let layout = Layout::from_size_align(len * 8, align).unwrap();
    
    unsafe {
        let ptr = alloc(layout) as *mut f64;
        Box::from_raw(std::slice::from_raw_parts_mut(ptr, len))
    }
}

// Usage: 32-byte alignment for AVX256
let buffer = allocate_aligned_f64(1024, 32);

// Or use crossbeam or other crates for aligned allocation
```

### Melange-Specific SIMD Strategy

For the DK solver in melange:

```rust
/// Matrix-vector multiply with SIMD for small fixed sizes
/// N is compile-time constant (circuit node count)
pub fn matvec_simd<const N: usize>(matrix: &[[f64; N]; N], vector: &[f64; N]) -> [f64; N] {
    // For small N (typical circuits: 2-8 nodes), scalar may be faster
    // due to setup overhead. Benchmark to confirm.
    
    if N <= 4 {
        // Scalar unrolled
        matvec_scalar_unrolled(matrix, vector)
    } else {
        // SIMD
        matvec_simd_impl(matrix, vector)
    }
}
```

---

## 8. Rust-Specific RT Safety

### No-Std Considerations

```rust
#![no_std]  // Disables standard library

// What you lose:
// - Vec, String, HashMap (allocate)
// - println! (may lock)
// - Panic messages (may allocate)

// What you keep:
// - Primitive types, arrays, slices
// - core::mem, core::ptr, core::slice
// - Static allocation via const/init

// For melange primitives: prefer no_std
pub mod primitives {
    #![no_std]
    
    pub struct Biquad {
        // Pure computation, no allocation
    }
}
```

### Panic Safety

```rust
/// Panic in audio thread = plugin crash = host glitch
/// Strategy: Use checked operations, return Result, or saturate

// BAD: Can panic on overflow
pub fn add_unchecked(a: f32, b: f32) -> f32 {
    a + b  // May panic in debug mode on overflow
}

// GOOD: Saturating arithmetic
pub fn add_saturating(a: f32, b: f32) -> f32 {
    a.saturating_add(b)  // Saturates to inf, never panics
}

// GOOD: Explicit bounds check
pub fn index_safe<T>(slice: &[T], idx: usize) -> Option<&T> {
    slice.get(idx)  // Returns None instead of panicking
}

// GOOD: Const assertions for compile-time checks
pub struct Matrix<const N: usize, const M: usize> {
    data: [[f64; M]; N],
}

impl<const N: usize, const M: usize> Matrix<N, M> {
    const ASSERT_SQUARE: () = assert!(N == M, "Matrix must be square");
}
```

### Abort on Panic

```rust
// In Cargo.toml:
// [profile.release]
// panic = 'abort'

// Prevents unwinding (which may allocate/lock) on panic
// Just aborts the process immediately
```

### No-Alloc Wrapper Types

```rust
/// Fixed-capacity vector for no-std/no-alloc contexts
pub struct FixedVec<T, const N: usize> {
    data: [MaybeUninit<T>; N],
    len: usize,
}

impl<T: Copy, const N: usize> FixedVec<T, N> {
    pub const fn new() -> Self {
        Self {
            data: unsafe { MaybeUninit::uninit().assume_init() },
            len: 0,
        }
    }
    
    pub fn push(&mut self, value: T) -> Result<(), T> {
        if self.len >= N {
            return Err(value);
        }
        self.data[self.len].write(value);
        self.len += 1;
        Ok(())
    }
    
    pub fn as_slice(&self) -> &[T] {
        unsafe {
            std::slice::from_raw_parts(
                self.data.as_ptr() as *const T,
                self.len
            )
        }
    }
}
```

### Const Generics for Compile-Time Safety

```rust
/// Circuit solver with compile-time known topology
/// Gains us: stack allocation, monomorphized loops, no dynamic dispatch
pub struct DkSolver<const N_NODES: usize, const N_NONLINEAR: usize> {
    // Matrix size known at compile time
    g_matrix: [[f64; N_NODES]; N_NODES],
    
    // LUT size known at compile time  
    tanh_lut: TanhLut<1024>,
}

impl<const N_NODES: usize, const N_NONLINEAR: usize> DkSolver<N_NODES, N_NONLINEAR> {
    /// Solve with statically-known bounds
    /// Compiler can unroll, vectorize, optimize aggressively
    pub fn solve(&mut self, input: f64) -> [f64; N_NODES] {
        let mut result = [0.0; N_NODES];
        
        // Loop bounds are constants: compiler unrolls
        for i in 0..N_NODES {
            let mut sum = 0.0;
            for j in 0..N_NODES {
                sum += self.g_matrix[i][j] * result[j];
            }
            result[i] = sum;
        }
        
        result
    }
}

// Type alias for specific circuit
type PreampSolver = DkSolver<4, 2>;
type ToneStackSolver = DkSolver<6, 0>;  // Linear, no nonlinear elements
```

### FFI Safety for Plugin Interfaces

```rust
use std::os::raw::{c_void, c_int};
use std::panic::catch_unwind;

/// C-exported process function
/// Must never panic across FFI boundary
#[no_mangle]
pub extern "C" fn melange_process(
    plugin: *mut c_void,
    inputs: *const *const f32,
    outputs: *mut *mut f32,
    n_samples: c_int,
) -> c_int {
    // Catch panics to prevent UB
    let result = catch_unwind(|| {
        let plugin = unsafe { &mut *(plugin as *mut MyPlugin) };
        let n = n_samples as usize;
        
        // ... process ...
        
        0 // Success
    });
    
    match result {
        Ok(code) => code,
        Err(_) => {
            // Log error, return error code
            -1
        }
    }
}
```

### Thread Safety Markers

```rust
/// Mark audio-thread-only types
pub struct AudioThreadMarker;

/// Type is safe to send to audio thread (no allocation, no locks)
pub struct AudioSafe<T>(T);

unsafe impl<T: Send> Send for AudioSafe<T> {}
unsafe impl<T> Sync for AudioSafe<T> {}

/// Usage
pub struct DspState {
    // ...
}

pub struct UiState {
    // ...
}

pub struct Plugin {
    dsp: AudioSafe<DspState>,  // Marked for audio thread
    ui: UiState,                // UI thread only
}
```

---

## Summary Checklist

Before shipping DSP code, verify:

| Category | Check | How |
|----------|-------|-----|
| Allocation | No heap in audio thread | Static analysis, `#[deny]` |
| Locks | No mutex/semaphore | Code review, clippy |
| Determinism | Fixed execution time | WCET analysis, worst-case testing |
| Denormals | FTZ enabled or handled | MXCSR check, stress test |
| Bounds | No panic paths in hot loops | `get()` not `[]`, saturating math |
| Cache | Hot data < 32KB | `cachegrind`, cache misses < 5% |
| Branches | Predictable control flow | `perf stat`, branch miss < 1% |
| SIMD | Used where beneficial | Benchmark scalar vs SIMD |

---

## References

1. **The Audio Programmer** - Lock-free programming patterns
2. **Cytomic** - The Glue compressor DSP writeups
3. **JUCE** - Real-time safety guidelines
4. **Rust Audio** - `rust-dsp` and `vst-rs` crates
5. **Intel** - SIMD optimization guides
6. **Agner Fog** - Software optimization manuals

---

*Document version: 1.0*  
*Last updated: 2026-02-23*  
*For: melange DSP toolkit*
