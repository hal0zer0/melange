# Rust Const Generics for DSP Applications

**Purpose:** Reference guide for using Rust const generics in real-time audio DSP code, with patterns specific to melange's solver architecture.

**Target Audience:** AI agents and developers working on the melange codebase.

---

## 1. Const Generic Basics

### 1.1 Syntax Overview

Const generics allow types to be parameterized by compile-time constant values:

```rust
// Type parameterized by a const usize
struct Matrix<T, const N: usize> {
    data: [[T; N]; N],  // N×N matrix
}

// Multiple const parameters
struct MNASystem<const NODES: usize, const NONLINEAR: usize> {
    g_matrix: [[f64; NODES]; NODES],      // Conductance matrix
    c_matrix: [[f64; NODES]; NODES],      // Capacitance matrix
    nonlinear_count: [f64; NONLINEAR],    // Nonlinear element count
}

// Const parameters with defaults (Rust 1.59+)
struct Buffer<T, const SIZE: usize = 128> {
    data: [T; SIZE],
}
```

### 1.2 Where Const Parameters Can Appear

```rust
// In struct definitions
struct State<const N: usize> {
    values: [f64; N],
}

// In impl blocks
impl<const N: usize> State<N> {
    fn new() -> Self {
        Self { values: [0.0; N] }
    }
    
    // Const can be used in method signatures
    fn as_array(&self) -> &[f64; N] {
        &self.values
    }
}

// In function definitions
fn mix<const N: usize>(a: [f64; N], b: [f64; N], t: f64) -> [f64; N] {
    let mut out = [0.0; N];
    for i in 0..N {
        out[i] = a[i] * (1.0 - t) + b[i] * t;
    }
    out
}

// In trait implementations
trait AudioBlock<const N: usize> {
    fn process(&mut self, input: &[f64; N]) -> [f64; N];
}
```

### 1.3 The `const N: usize` Pattern

In melange, we follow this naming convention:

```rust
// NODES: Number of circuit nodes (MNA matrix dimension)
// NONLIN: Number of nonlinear elements (iterative solve dimension)
// STAGES: Number of filter stages
// SIZE: Buffer or block size

struct DKSolver<const NODES: usize, const NONLIN: usize> {
    // G matrix: (NODES × NODES)
    g_matrix: [[f64; NODES]; NODES],
    
    // Reduced system: (NONLIN × NONLIN)
    // From DK method dimensionality reduction
    jacobian: [[f64; NONLIN]; NONLIN],
    
    // Current guess for Newton-Raphson
    v_guess: [f64; NONLIN],
}
```

---

## 2. Fixed-Size Arrays vs Vec for DSP

### 2.1 Why `[T; N]` Over `Vec<T>`

| Aspect | `[T; N]` | `Vec<T>` |
|--------|----------|----------|
| **Allocation** | Stack (no alloc) | Heap |
| **Size** | Compile-time known | Runtime dynamic |
| **Bounds checks** | Can be optimized away | Always checked |
| **Cache locality** | Excellent | Good (may fragment) |
| **Copy trait** | Yes (if T: Copy) | No |
| `no_std` | Yes | No |

### 2.2 Real-Time Safety

```rust
// BAD: Heap allocation in audio callback
fn process_bad(sample: f64) -> f64 {
    let mut buffer = vec![0.0; 1024];  // ALLOCATES! ❌
    // ... processing
    sample
}

// GOOD: Stack-allocated, no allocation
fn process_good<const N: usize>(samples: [f64; N]) -> [f64; N] {
    let mut output = [0.0; N];  // Stack only ✓
    // ... processing
    output
}
```

### 2.3 Melange Pattern: Matrix as Nested Arrays

```rust
/// MNA conductance matrix stored as const-generic nested arrays
/// 
/// Layout: g_matrix[row][col]
/// Access pattern optimized for row-major traversal
pub struct MNAMatrix<const N: usize> {
    data: [[f64; N]; N],
}

impl<const N: usize> MNAMatrix<N> {
    pub fn zeros() -> Self {
        Self { data: [[0.0; N]; N] }
    }
    
    /// Stamp a conductance between nodes i and j
    /// This is the core operation in MNA matrix construction
    pub fn stamp_conductance(&mut self, i: usize, j: usize, g: f64) {
        // Diagonal entries
        self.data[i][i] += g;
        self.data[j][j] += g;
        // Off-diagonal entries
        self.data[i][j] -= g;
        self.data[j][i] -= g;
    }
    
    /// Row-major iteration - cache friendly
    pub fn row_major_iter(&self) -> impl Iterator<Item = &[f64; N]> {
        self.data.iter()
    }
    
    /// Const access to dimension
    pub const fn dimension() -> usize {
        N
    }
}
```

### 2.4 Working with External Data

```rust
/// Import data from slice - useful for loading pre-computed matrices
impl<const N: usize> MNAMatrix<N> {
    pub fn from_slice(slice: &[f64]) -> Option<Self> {
        if slice.len() != N * N {
            return None;
        }
        
        let mut data = [[0.0; N]; N];
        for i in 0..N {
            for j in 0..N {
                data[i][j] = slice[i * N + j];
            }
        }
        Some(Self { data })
    }
    
    /// Export to contiguous array - for FFI or serialization
    pub fn to_flat_array(&self) -> [f64; N * N] {
        let mut flat = [0.0; N * N];
        for i in 0..N {
            for j in 0..N {
                flat[i * N + j] = self.data[i][j];
            }
        }
        flat
    }
}
```

---

## 3. Compile-Time Matrix Dimensions

### 3.1 Type-Level Circuit Topology

In melange, circuit topology (node count, element count) is encoded in types:

```rust
/// A compiled circuit has fixed topology known at compile time
/// 
/// # Type Parameters
/// - `N`: Number of circuit nodes (MNA dimension)
/// - `M`: Number of nonlinear elements (NR iteration dimension)
/// - `P`: Number of parameters (potentiometers, switches)
pub struct CompiledCircuit<const N: usize, const M: usize, const P: usize> {
    /// Linear system matrix (N × N)
    g_matrix: [[f64; N]; N],
    
    /// Companion model conductances for reactive elements
    companion_g: [f64; N],
    
    /// Nonlinear element Jacobians (M × M, typically M << N)
    nonlinear_jacobian: [[f64; M]; M],
    
    /// Parameter values
    params: [f64; P],
}
```

### 3.2 Generic Implementations Over Dimensions

```rust
impl<const N: usize, const M: usize, const P: usize> CompiledCircuit<N, M, P> {
    /// Solve the linear system G·x = b for x
    /// 
    /// Uses LU decomposition with partial pivoting
    pub fn solve_linear(&self, b: [f64; N]) -> [f64; N] {
        // Implementation can be specialized per N for small matrices
        match N {
            0..=4 => self.solve_linear_small(b),   // Unrolled for cache
            _ => self.solve_linear_lu(b),          // General LU
        }
    }
    
    /// Newton-Raphson iteration for nonlinear elements
    pub fn solve_nonlinear(&self, initial: [f64; M]) -> [f64; M] {
        let mut v = initial;
        
        for _ in 0..20 {  // Max iterations
            let (f, j) = self.nonlinear_system(&v);
            let delta = solve_small::<M>(&j, &f);  // Generic over M
            
            // Check convergence
            let err: f64 = delta.iter().map(|d| d.abs()).sum();
            if err < 1e-12 {
                break;
            }
            
            for i in 0..M {
                v[i] -= delta[i];
            }
        }
        
        v
    }
}
```

### 3.3 Const Generic Arrays of Arrays

```rust
/// Multi-channel processing with compile-time channel count
pub struct MultiChannel<T, const CHANNELS: usize, const BLOCK_SIZE: usize> {
    buffers: [[T; BLOCK_SIZE]; CHANNELS],
}

impl<T: Default + Copy, const C: usize, const B: usize> MultiChannel<T, C, B> {
    pub fn new() -> Self {
        Self { buffers: [[T::default(); B]; C] }
    }
    
    /// Process each channel with the same function
    pub fn process<F>(&mut self, mut f: F)
    where
        F: FnMut(&mut [T; B]),
    {
        for ch in 0..C {
            f(&mut self.buffers[ch]);
        }
    }
    
    /// Get channel buffer (compile-time bounds where possible)
    pub fn channel(&self, ch: usize) -> Option<&[T; B]> {
        self.buffers.get(ch)
    }
}

// Usage: 2-channel stereo, 128-sample blocks
type StereoBlock = MultiChannel<f64, 2, 128>;
```

---

## 4. Performance Benefits

### 4.1 Stack Allocation Guarantees

```rust
/// Memory layout for a 4×4 matrix:
/// 
/// Stack layout (contiguous, 256 bytes for f64):
/// [0x00..0x07]   data[0][0]
/// [0x08..0x0f]   data[0][1]
/// ...
/// [0xf8..0xff]   data[3][3]
/// 
/// No indirection, no heap pointer, no capacity field
pub struct Matrix4x4 {
    data: [[f64; 4]; 4],  // Exactly 256 bytes on stack
}

/// Compiler can optimize this to SIMD or unrolled loops
pub fn matvec_mul_4<const N: usize>(m: &[[f64; N]; N], v: &[f64; N]) -> [f64; N] {
    let mut out = [0.0; N];
    
    // When N is small (known at compile time), LLVM unrolls this
    for i in 0..N {
        let mut sum = 0.0;
        for j in 0..N {
            sum += m[i][j] * v[j];
        }
        out[i] = sum;
    }
    
    out
}

// For N=4, compiler typically generates:
// - 4 SIMD multiplies
// - Horizontal adds
// - No loop overhead
```

### 4.2 Cache Locality Patterns

```rust
/// Row-major traversal (cache friendly)
pub fn solve_fwd_substitution<const N: usize>(
    l: &[[f64; N]; N],  // Lower triangular
    b: &[f64; N],
) -> [f64; N] {
    let mut y = [0.0; N];
    
    for i in 0..N {
        let mut sum = b[i];
        // Accesses l[i][0..i] - contiguous in memory
        for j in 0..i {
            sum -= l[i][j] * y[j];
        }
        y[i] = sum / l[i][i];
    }
    
    y
}

/// Column-major traversal (cache unfriendly for row-major arrays)
/// Avoid this pattern with [[T; N]; N] layout
pub fn bad_column_access<const N: usize>(m: &[[f64; N]; N], v: &mut [f64; N]) {
    for j in 0..N {
        for i in 0..N {
            // m[i][j] jumps by N*f64 bytes each access!
            v[i] += m[i][j];
        }
    }
}
```

### 4.3 Loop Unrolling and Vectorization

```rust
/// When N is const, compiler can fully unroll small loops
pub fn dot_product<const N: usize>(a: &[f64; N], b: &[f64; N]) -> f64 {
    let mut sum = 0.0;
    for i in 0..N {
        sum += a[i] * b[i];
    }
    sum
}

// With N=4 and opt-level=3, generates something like:
// movsd  xmm0, QWORD PTR [rdi]
// mulsd  xmm0, QWORD PTR [rsi]
// movsd  xmm1, QWORD PTR [rdi+8]
// mulsd  xmm1, QWORD PTR [rsi+8]
// addsd  xmm0, xmm1
// ... (fully unrolled)

/// Explicit SIMD for guaranteed vectorization
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;

pub fn dot_product_simd_4(a: &[f64; 4], b: &[f64; 4]) -> f64 {
    unsafe {
        let va = _mm256_loadu_pd(a.as_ptr());
        let vb = _mm256_loadu_pd(b.as_ptr());
        let prod = _mm256_mul_pd(va, vb);
        
        // Horizontal sum
        let hi = _mm256_extractf128_pd(prod, 1);
        let lo = _mm256_castpd256_pd128(prod);
        let sum128 = _mm_add_pd(lo, hi);
        let sum64 = _mm_add_sd(sum128, _mm_unpackhi_pd(sum128, sum128));
        _mm_cvtsd_f64(sum64)
    }
}
```

### 4.4 Zero-Cost Abstractions

```rust
/// Wrapper that provides matrix operations but compiles away
pub struct SymmetricMatrix<T, const N: usize> {
    /// Only store upper triangle: N*(N+1)/2 elements
    data: [T; N * (N + 1) / 2],
}

impl<T: Copy + Default, const N: usize> SymmetricMatrix<T, N> {
    pub fn zeros() -> Self {
        Self { data: [T::default(); N * (N + 1) / 2] }
    }
    
    /// Convert 2D index to 1D packed index
    const fn index(row: usize, col: usize) -> usize {
        // Ensure row <= col for upper triangle
        let (r, c) = if row <= col { (row, col) } else { (col, row) };
        // Triangular number formula: c*(c+1)/2 + r
        c * (c + 1) / 2 + r
    }
    
    pub fn get(&self, row: usize, col: usize) -> T {
        self.data[Self::index(row, col)]
    }
    
    pub fn set(&mut self, row: usize, col: usize, val: T) {
        self.data[Self::index(row, col)] = val;
    }
}

// Usage - same API as full matrix, half the memory
let mut m: SymmetricMatrix<f64, 8> = SymmetricMatrix::zeros();
m.set(3, 5, 1.0);
assert_eq!(m.get(5, 3), 1.0);  // Symmetric access works
```

---

## 5. Const Generic Expression Limitations and Workarounds

### 5.1 Current Limitations (as of Rust 1.85)

```rust
// ❌ This does NOT compile:
struct Bad<const N: usize> {
    // Arithmetic in array size not allowed (yet)
    data: [f64; N + 1],
}

// ❌ This does NOT compile:
impl<const N: usize> Bad<N> {
    const fn double() -> usize {
        N * 2  // Const arithmetic in impl is limited
    }
}

// ❌ This does NOT compile:
fn bad<const N: usize>() -> [f64; N * 2] {
    [0.0; N * 2]  // Generic expr in array repeat not allowed
}
```

### 5.2 Workaround: Const Evaluation Blocks

```rust
/// Use const blocks for arithmetic (Rust 1.79+)
pub struct Good<const N: usize> {
    // Wrap arithmetic in const block
    data: [f64; { N + 1 }],
}

/// Or use a helper trait
pub trait ConstMath<const N: usize> {
    const DOUBLE: usize = N * 2;
    const SQUARE: usize = N * N;
    const NEXT_POWER_OF_2: usize = N.next_power_of_two();
}

impl<const N: usize> ConstMath<N> for () {}

/// Now use the associated constant
pub struct Buffer<const N: usize>
where
    (): ConstMath<N>,
{
    doubled: [f64; <() as ConstMath<N>>::DOUBLE],
}
```

### 5.3 Workaround: Generic-array Crate Pattern

```rust
/// When you need runtime-determined sizes but want stack allocation,
/// use generic-array which provides type-level numbers
/// 
/// Note: In melange, we prefer const generics directly, but this
/// pattern is useful for complex size calculations.

use generic_array::GenericArray;
use generic_array::typenum::{U1, U2, U4, U8, U16, U32, U64, U128, U256, U512, U1024};

/// Type alias for common DSP block sizes
pub type Block64<T> = GenericArray<T, U64>;
pub type Block128<T> = GenericArray<T, U128>;
pub type Block256<T> = GenericArray<T, U256>;
pub type Block512<T> = GenericArray<T, U512>;

/// Buffer that works with generic-array sizes
pub struct GA_Buffer<T, Size> {
    data: GenericArray<T, Size>,
}

impl<T: Default + Copy, Size: generic_array::ArrayLength<T>> GA_Buffer<T, Size> {
    pub fn new() -> Self {
        Self { data: GenericArray::default() }
    }
}

/// You can do arithmetic at the type level:
/// U2 + U4 = U6, U4 * U2 = U8, etc.
use generic_array::typenum::{Sum, Prod};

type Block256From128and2 = Prod<U128, U2>;  // 128 * 2 = 256
```

### 5.4 Workaround: Const Generic Expressions via Macros

```rust
/// Macro to generate implementations for multiple sizes
/// This is how melange handles common circuit topologies

macro_rules! impl_mna_solver {
    ($($n:literal),*) => {
        $(
            impl MNASolver<$n> {
                /// Specialized solve for this specific size
                pub fn solve_specialized(&self, b: [f64; $n]) -> [f64; $n] {
                    // Size-specific optimization
                    paste::paste! {
                        [<solve_ $n x $n>](self.matrix, b)
                    }
                }
            }
        )*
    };
}

// Generate implementations for common MNA sizes
impl_mna_solver!(1, 2, 3, 4, 5, 6, 7, 8, 12, 16);

/// Alternative: Use seq_macro for sequential generation
use seq_macro::seq;

seq!(N in 1..=8 {
    impl Solver<N> {
        pub fn dimension(&self) -> usize {
            N
        }
    }
});
```

### 5.5 Workaround: Const Generic Arrays via Const Fn

```rust
/// Initialize arrays when const expressions don't work
pub struct Workaround<const N: usize> {
    // Can't do: data: [f64; N * 2]
    data: Vec<f64>,  // Fallback to Vec
}

/// Better: Use const fn to compute size
trait ComputedSize<const N: usize> {
    const SIZE: usize;
}

struct ComputedImpl;

impl<const N: usize> ComputedSize<N> for ComputedImpl {
    // Const fn evaluation works here
    const SIZE: usize = {
        let mut size = 1;
        let mut n = N;
        while n > 0 {
            size *= 2;
            n /= 2;
        }
        size
    };
}

/// Now use ComputedImpl::SIZE where needed
pub struct Better<const N: usize> {
    // This works because ComputedImpl::SIZE is a concrete const
    data: [f64; ComputedImpl::SIZE],
    _marker: std::marker::PhantomData<[(); N]>,
}
```

---

## 6. Generic-Array and Similar Crates

### 6.1 When to Use Generic-Array

| Use Case | Solution |
|----------|----------|
| Complex type-level arithmetic | `generic-array` |
| Runtime size selection | `generic-array` + enums |
| Simple fixed sizes | Native const generics |
| Mixed const/dynamic | Hybrid approach |

### 6.2 Generic-Array Basics

```rust
use generic_array::GenericArray;
use generic_array::typenum::{U4, U8, U16, U32, U64, U128, U256, U512, U1024};
use generic_array::sequence::GenericSequence;

/// Create a buffer of 128 f64 samples
let buffer: GenericArray<f64, U128> = GenericArray::default();

/// GenericArray implements Deref to [T], so slice methods work
let sum: f64 = buffer.iter().sum();

/// Map operations (functional style)
let squared: GenericArray<f64, U128> = buffer.map(|x| x * x);

/// Zip operations
let a: GenericArray<f64, U64> = GenericArray::default();
let b: GenericArray<f64, U64> = GenericArray::default();
let c: GenericArray<f64, U64> = a.zip(b, |x, y| x + y);
```

### 6.3 Hybrid Approach: Enum of Common Sizes

```rust
/// When runtime size selection is needed but you want
/// stack allocation for common cases

use generic_array::typenum::*;

/// Common DSP block sizes as an enum
pub enum BlockSize {
    _64,
    _128,
    _256,
    _512,
    _1024,
}

/// Type-level size mapping
pub trait SizeValue {
    type Type: generic_array::ArrayLength<f64>;
    const USIZE: usize;
}

impl SizeValue for U64 { type Type = U64; const USIZE: usize = 64; }
impl SizeValue for U128 { type Type = U128; const USIZE: usize = 128; }
impl SizeValue for U256 { type Type = U256; const USIZE: usize = 256; }
impl SizeValue for U512 { type Type = U512; const USIZE: usize = 512; }
impl SizeValue for U1024 { type Type = U1024; const USIZE: usize = 1024; }

/// Runtime-selected buffer using enum dispatch
pub enum DynBuffer {
    B64(GenericArray<f64, U64>),
    B128(GenericArray<f64, U128>),
    B256(GenericArray<f64, U256>),
    B512(GenericArray<f64, U512>),
    B1024(GenericArray<f64, U1024>),
}

impl DynBuffer {
    pub fn new(size: BlockSize) -> Self {
        match size {
            BlockSize::_64 => Self::B64(GenericArray::default()),
            BlockSize::_128 => Self::B128(GenericArray::default()),
            BlockSize::_256 => Self::B256(GenericArray::default()),
            BlockSize::_512 => Self::B512(GenericArray::default()),
            BlockSize::_1024 => Self::B1024(GenericArray::default()),
        }
    }
    
    pub fn as_slice(&self) -> &[f64] {
        match self {
            Self::B64(b) => b.as_slice(),
            Self::B128(b) => b.as_slice(),
            Self::B256(b) => b.as_slice(),
            Self::B512(b) => b.as_slice(),
            Self::B1024(b) => b.as_slice(),
        }
    }
}
```

### 6.4 ndarray with Const Generics

```rust
/// For higher-dimensional arrays, consider `ndarray` with const generics
/// Note: ndarray uses dynamic dimensions by default

use ndarray::{Array, Array2, ArrayView2};

/// Wrapper that provides const-generic interface over ndarray
pub struct ConstMatrix<T, const N: usize, const M: usize> {
    data: Array2<T>,
}

impl<T: Copy + Default, const N: usize, const M: usize> ConstMatrix<T, N, M> {
    pub fn zeros() -> Self {
        Self {
            data: Array::zeros((N, M)),
        }
    }
    
    /// Get view as fixed-size array (for FFI or interop)
    pub fn as_array2(&self) -> [[T; M]; N] {
        let mut out = [[T::default(); M]; N];
        for i in 0..N {
            for j in 0..M {
                out[i][j] = self.data[[i, j]];
            }
        }
        out
    }
}

/// For pure const-generic without ndarray dependency,
/// melange uses [[T; M]; N] directly.
```

### 6.5 Smallvec for Mixed Stack/Heap

```rust
/// When you need mostly-stack but can spill to heap
use smallvec::SmallVec;

/// Buffer that stores up to 16 samples on stack, spills after
pub type SmallSampleBuffer = SmallVec<[f64; 16]>;

/// Useful for delay lines where typical delay is short
/// but long delays are occasionally needed
pub struct VariableDelayLine {
    buffer: SmallVec<[f64; 1024]>,  // 1ms at 44.1kHz on stack
    write_idx: usize,
}
```

---

## 7. Practical Patterns for Audio DSP

### 7.1 The Melange Solver Pattern

```rust
/// Complete example of melange's DK solver architecture using const generics

/// A nonlinear circuit element (diode, transistor, tube)
pub trait NonlinearDevice<const PORTS: usize> {
    /// Evaluate constitutive equation: i = f(v)
    fn current(&self, v: [f64; PORTS]) -> [f64; PORTS];
    
    /// Evaluate Jacobian: di/dv
    fn jacobian(&self, v: [f64; PORTS]) -> [[f64; PORTS]; PORTS];
}

/// A linear companion model for a reactive element
pub struct CompanionModel {
    pub conductance: f64,
    pub source: f64,
}

/// The DK solver for a specific circuit topology
/// 
/// NODES: Total circuit nodes
/// NONLIN: Number of nonlinear ports
/// REACTIVE: Number of reactive elements (caps/inductors)
pub struct DKSolver<const NODES: usize, const NONLIN: usize, const REACTIVE: usize> {
    /// G matrix (conductance) - constant for linear circuit
    g_matrix: [[f64; NODES]; NODES],
    
    /// Current source vector from companion models
    companion_sources: [f64; NODES],
    
    /// Nonlinear devices
    nonlinear_devices: [Box<dyn NonlinearDevice<2>>; NONLIN],
    
    /// Newton-Raphson iteration state
    v_prev: [f64; NONLIN],
    
    /// Pre-computed LU factors for G (if using direct solve)
    lu_factors: [[f64; NODES]; NODES],
    lu_pivots: [usize; NODES],
}

impl<const N: usize, const M: usize, const R: usize> DKSolver<N, M, R> {
    /// Process one sample through the circuit
    /// 
    /// This is the hot path - runs at audio rate
    pub fn process_sample(&mut self, input: f64) -> f64 {
        // 1. Update companion model sources based on state
        self.update_companion_models();
        
        // 2. Newton-Raphson iteration for nonlinear elements
        let v_nonlin = self.solve_nonlinear();
        
        // 3. Solve linear system with updated sources
        let v_nodes = self.solve_linear(v_nonlin);
        
        // 4. Return output (typically last node or specific port)
        v_nodes[N - 1]
    }
    
    /// Solve M×M nonlinear system via Newton-Raphson
    fn solve_nonlinear(&mut self) -> [f64; M] {
        let mut v = self.v_prev;
        
        for iter in 0..10 {
            // Build reduced system using DK method
            let (f, j) = self.build_reduced_system(&v);
            
            // Solve J·Δv = f for Δv
            let delta = solve_small::<M>(&j, &f);
            
            // Update with damping if needed
            for i in 0..M {
                v[i] -= 0.7 * delta[i];  // Damped NR
            }
            
            // Convergence check
            let err = delta.iter().map(|d| d.abs()).sum::<f64>();
            if err < 1e-10 {
                break;
            }
        }
        
        self.v_prev = v;
        v
    }
    
    /// Solve N×N linear system using pre-factored LU
    fn solve_linear(&self, v_nonlin: [f64; M]) -> [f64; N] {
        // Build RHS with input and nonlinear contributions
        let mut b = self.companion_sources;
        self.add_nonlinear_contributions(&mut b, &v_nonlin);
        
        // LU solve with partial pivoting
        lu_solve(&self.lu_factors, &self.lu_pivots, &mut b);
        
        // b now contains solution x
        b
    }
    
    /// Update companion models - depends on discretization method
    fn update_companion_models(&mut self) {
        // Trapezoidal rule update
        // G_eq = 2C/h, I_eq = -2C/h * v_prev - i_prev
        // ...
    }
}
```

### 7.2 State-Space Filter Pattern

```rust
/// Generic state-space filter with const-generic dimensions
/// 
/// dx/dt = A·x + B·u
/// y = C·x + D·u
/// 
/// For audio filters, typically N=2 (biquad) or N=4 (ladder)
pub struct StateSpaceFilter<const N: usize, const M: usize, const P: usize> {
    /// State vector
    x: [f64; N],
    
    /// System matrix (N×N)
    a: [[f64; N]; N],
    
    /// Input matrix (N×M)  -- M inputs
    b: [[f64; M]; N],
    
    /// Output matrix (P×N) -- P outputs
    c: [[f64; N]; P],
    
    /// Feedthrough matrix (P×M)
    d: [[f64; M]; P],
    
    /// Sample period
    ts: f64,
}

impl<const N: usize, const M: usize, const P: usize> StateSpaceFilter<N, M, P> {
    /// Create from continuous-time matrices (bilinear transform)
    pub fn from_continuous(
        a_c: [[f64; N]; N],
        b_c: [[f64; M]; N],
        c_c: [[f64; N]; P],
        d_c: [[f64; M]; P],
        fs: f64,
    ) -> Self {
        let ts = 1.0 / fs;
        
        // Bilinear transform: s = 2/T * (z-1)/(z+1)
        // A_d = (I - A_c*T/2)^-1 * (I + A_c*T/2)
        // ... (matrix algebra)
        
        Self {
            x: [0.0; N],
            a: a_c,  // Simplified - actual bilinear transform needed
            b: b_c,
            c: c_c,
            d: d_c,
            ts,
        }
    }
    
    /// Process single sample
    pub fn process(&mut self, u: [f64; M]) -> [f64; P] {
        // x_next = A·x + B·u
        let mut x_next = [0.0; N];
        for i in 0..N {
            let mut sum = 0.0;
            for j in 0..N {
                sum += self.a[i][j] * self.x[j];
            }
            for j in 0..M {
                sum += self.b[i][j] * u[j];
            }
            x_next[i] = sum;
        }
        
        // y = C·x + D·u
        let mut y = [0.0; P];
        for i in 0..P {
            let mut sum = 0.0;
            for j in 0..N {
                sum += self.c[i][j] * x_next[j];
            }
            for j in 0..M {
                sum += self.d[i][j] * u[j];
            }
            y[i] = sum;
        }
        
        self.x = x_next;
        y
    }
}

/// Type aliases for common filter structures
pub type Biquad = StateSpaceFilter<2, 1, 1>;      // 2-pole, 1 in, 1 out
pub type Ladder4Pole = StateSpaceFilter<4, 1, 1>; // 4-pole ladder
pub type MimoBiquad = StateSpaceFilter<4, 2, 2>;  // 2 biquads, stereo
```

### 7.3 Oversampling Pattern

```rust
/// Polyphase oversampling using const generics for ratio
/// 
/// R: Oversampling ratio (2, 4, 8, 16)
/// TAPS: Anti-aliasing filter taps
pub struct Oversampler<const R: usize, const TAPS: usize> {
    /// Polyphase filter coefficients
    /// Stored as [R][TAPS/R] for cache efficiency
    coeffs: [[f64; TAPS / R]; R],
    
    /// Delay line (double length for SIMD alignment)
    delay_line: [f64; TAPS * 2],
    
    /// Write position
    write_pos: usize,
}

impl<const R: usize, const TAPS: usize> Oversampler<R, TAPS> {
    /// Upsample by factor R
    /// Input: 1 sample, Output: R samples
    pub fn upsample(&mut self, input: f64) -> [f64; R] {
        // Write input to delay line
        self.delay_line[self.write_pos] = input;
        self.delay_line[self.write_pos + TAPS] = input;  // Duplicate for wrap-free
        
        // Compute R output samples using polyphase structure
        let mut output = [0.0; R];
        
        for phase in 0..R {
            let mut sum = 0.0;
            for i in 0..TAPS / R {
                let tap_idx = self.write_pos - (phase + i * R) % TAPS;
                sum += self.coeffs[phase][i] * self.delay_line[tap_idx];
            }
            output[phase] = sum * R as f64;  // Gain compensation
        }
        
        self.write_pos = (self.write_pos + 1) % TAPS;
        output
    }
    
    /// Downsample by factor R
    /// Input: R samples, Output: 1 sample
    pub fn downsample(&mut self, input: [f64; R]) -> f64 {
        // Similar polyphase structure
        // ...
        0.0
    }
}

/// Common oversampling ratios
type Oversample2x = Oversampler<2, 32>;   // 2x, 16 taps per phase
type Oversample4x = Oversampler<4, 64>;   // 4x, 16 taps per phase
type Oversample8x = Oversampler<8, 128>;  // 8x, 16 taps per phase
```

### 7.4 Multi-Channel Buffer Pattern

```rust
/// Interleaved vs deinterleaved storage tradeoffs

/// Interleaved: [L0, R0, L1, R1, L2, R2, ...]
/// - Good for: External API compatibility, cache streaming
/// - Bad for: SIMD processing per-channel
pub struct InterleavedBuffer<T, const CHANNELS: usize, const FRAMES: usize> {
    data: [T; CHANNELS * FRAMES],
}

/// Deinterleaved: [L0, L1, L2, ...][R0, R1, R2, ...]
/// - Good for: Per-channel processing, SIMD
/// - Bad for: External buffer copies
pub struct DeinterleavedBuffer<T, const CHANNELS: usize, const FRAMES: usize> {
    data: [[T; FRAMES]; CHANNELS],
}

impl<T: Copy + Default, const C: usize, const F: usize> DeinterleavedBuffer<T, C, F> {
    pub fn zeros() -> Self {
        Self { data: [[T::default(); F]; C] }
    }
    
    /// Process each channel with the same function
    pub fn process_channels<FN>(&mut self, mut f: FN)
    where
        FN: FnMut(&mut [T; F], usize),  // channel data, channel index
    {
        for ch in 0..C {
            f(&mut self.data[ch], ch);
        }
    }
    
    /// Convert to interleaved for output
    pub fn to_interleaved(&self) -> [T; C * F] {
        let mut out = [T::default(); C * F];
        for frame in 0..F {
            for ch in 0..C {
                out[frame * C + ch] = self.data[ch][frame];
            }
        }
        out
    }
    
    /// Load from interleaved input
    pub fn from_interleaved(&mut self, input: &[T; C * F]) {
        for frame in 0..F {
            for ch in 0..C {
                self.data[ch][frame] = input[frame * C + ch];
            }
        }
    }
}

/// Type aliases
pub type StereoBuffer<F> = DeinterleavedBuffer<f64, 2, F>;
pub type QuadBuffer<F> = DeinterleavedBuffer<f64, 4, F>;
```

### 7.5 Lookup Table Pattern

```rust
/// Const-generic lookup table for waveshaping
/// 
/// SIZE: Table size (power of 2 for efficient wrap)
/// F: Function to tabulate
pub struct LookupTable<const SIZE: usize> {
    data: [f64; SIZE],
    scale: f64,      // input -> index
    offset: f64,     // input bias
}

impl<const SIZE: usize> LookupTable<SIZE> {
    /// Create table by sampling a function
    pub fn new<F>(f: F, min: f64, max: f64) -> Self
    where
        F: Fn(f64) -> f64,
    {
        let mut data = [0.0; SIZE];
        let scale = (SIZE - 1) as f64 / (max - min);
        let offset = -min * scale;
        
        for i in 0..SIZE {
            let x = min + (max - min) * (i as f64) / (SIZE - 1) as f64;
            data[i] = f(x);
        }
        
        Self { data, scale, offset }
    }
    
    /// Linear interpolation lookup
    #[inline(always)]
    pub fn lookup(&self, x: f64) -> f64 {
        let idx = x * self.scale + self.offset;
        let idx_i = idx as usize;
        let frac = idx - idx_i as f64;
        
        // Bounds check with saturation (or wrap for periodic)
        let i0 = idx_i.min(SIZE - 1);
        let i1 = (idx_i + 1).min(SIZE - 1);
        
        let y0 = self.data[i0];
        let y1 = self.data[i1];
        
        y0 + frac * (y1 - y0)
    }
    
    /// Lookup without bounds check (unsafe, but faster if inputs validated)
    #[inline(always)]
    pub unsafe fn lookup_unchecked(&self, x: f64) -> f64 {
        let idx = x * self.scale + self.offset;
        let idx_i = idx as usize;
        let frac = idx - idx_i as f64;
        
        let y0 = *self.data.get_unchecked(idx_i);
        let y1 = *self.data.get_unchecked(idx_i + 1);
        
        y0 + frac * (y1 - y0)
    }
}

/// Common table sizes (powers of 2 for mask-based wrapping)
pub type SmallTable = LookupTable<256>;    // 2KB, precise
type MediumTable = LookupTable<1024>;      // 8KB, smoother
type LargeTable = LookupTable<4096>;       // 32KB, very smooth

/// Example: Tanh waveshaper
pub fn tanh_table() -> MediumTable {
    LookupTable::new(|x| x.tanh(), -5.0, 5.0)
}
```

### 7.6 SIMD-Friendly Struct of Arrays

```rust
/// Struct of Arrays (SoA) vs Array of Structs (AoS)
/// 
/// AoS: [Sample{ l, r, time }, Sample{ l, r, time }, ...]
/// SoA: { l: [f64; N], r: [f64; N], time: [f64; N] }
/// 
/// SoA is SIMD-friendly and cache-friendly for batch processing

/// Sample with multiple fields
#[derive(Clone, Copy)]
pub struct Sample {
    pub amplitude: f64,
    pub time: f64,
    pub velocity: f64,
}

/// AoS storage - intuitive but poor SIMD
pub struct AoS_Buffer<const N: usize> {
    data: [Sample; N],
}

/// SoA storage - SIMD-friendly
pub struct SoA_Buffer<const N: usize> {
    amplitude: [f64; N],
    time: [f64; N],
    velocity: [f64; N],
}

impl<const N: usize> SoA_Buffer<N> {
    /// Process all amplitudes with SIMD
    #[cfg(target_arch = "x86_64")]
    pub fn process_amplitudes_simd(&mut self, gain: f64) {
        use std::arch::x86_64::*;
        
        let g = unsafe { _mm256_set1_pd(gain) };
        
        for i in (0..N).step_by(4) {
            unsafe {
                let v = _mm256_loadu_pd(self.amplitude.as_ptr().add(i));
                let r = _mm256_mul_pd(v, g);
                _mm256_storeu_pd(self.amplitude.as_mut_ptr().add(i), r);
            }
        }
    }
    
    /// Convert single element to AoS form
    pub fn get(&self, idx: usize) -> Sample {
        Sample {
            amplitude: self.amplitude[idx],
            time: self.time[idx],
            velocity: self.velocity[idx],
        }
    }
    
    /// Set from AoS form
    pub fn set(&mut self, idx: usize, s: Sample) {
        self.amplitude[idx] = s.amplitude;
        self.time[idx] = s.time;
        self.velocity[idx] = s.velocity;
    }
}
```

---

## 8. Best Practices for Melange

### 8.1 Naming Conventions

```rust
// Use descriptive uppercase names for const parameters
struct Circuit<const NODES: usize, const NONLINEAR: usize, const PORTS: usize>;

// Prefer const generics over typenums where possible
// Good: Matrix<T, const N: usize>
// Avoid: Matrix<T, N: ArrayLength<T>> (unless arithmetic needed)

// Use const fn for derived constants
impl<const N: usize> Matrix<N> {
    pub const fn triangular_size() -> usize {
        N * (N + 1) / 2
    }
}
```

### 8.2 Bounds and Constraints

```rust
/// Express constraints through where clauses
pub struct Solver<const N: usize>
where
    [(); N * N]: Sized,  // Ensure N*N fits in array size
{
    matrix: [[f64; N]; N],
}

/// Or use static assertions
pub struct SafeSolver<const N: usize> {
    _assert: <Self as AssertValid>::Type,
}

impl<const N: usize> SafeSolver<N> {
    const_assert!(N > 0 && N <= 64, "N must be in range 1..=64");
}
```

### 8.3 Documentation Template

```rust
/// [Brief description]
/// 
/// # Type Parameters
/// - `N`: [Description of N]
/// - `M`: [Description of M]
/// 
/// # Memory Layout
/// [Describe stack layout, alignment]
/// 
/// # Performance Characteristics
/// - Time: O(N^2) per sample
/// - Space: O(N) on stack
/// - Cache: [Cache behavior]
/// 
/// # Example
/// ```
/// let solver = Solver::<4, 2>::new();
/// ```
pub struct Solver<const N: usize, const M: usize> {
    // ...
}
```

### 8.4 Testing Const Generic Code

```rust
/// Test multiple sizes with macros
#[cfg(test)]
mod tests {
    use super::*;
    
    macro_rules! test_solver_sizes {
        ($($n:literal),*) => {
            $(
                paste::paste! {
                    #[test]
                    fn [<test_solver_ $n>]() {
                        let mut solver = Solver::<$n, 1>::new();
                        let output = solver.process_sample(1.0);
                        assert!(!output.is_nan());
                    }
                }
            )*
        };
    }
    
    test_solver_sizes!(1, 2, 3, 4, 5, 6, 7, 8);
}
```

---

## 9. References

### Official Documentation
- [Rust Const Generics RFC](https://rust-lang.github.io/rfcs/2000-const-generics.html)
- [Const Generics MVP](https://rust-lang.github.io/rfcs/2000-const-generics.html)
- [Generic Associated Types](https://rust-lang.github.io/rfcs/1598-generic_associated_types.html)

### Crates
- `generic-array` - Type-level numbers for const generics
- `typenum` - Type-level integer arithmetic
- `seq-macro` - Sequential code generation
- `paste` - Token pasting for macros
- `smallvec` - Stack-allocated vectors
- `arrayvec` - Fixed-capacity vectors

### Papers and Resources
- "Real-Time Audio Programming 101" - Time domain pitfalls
- "Memory Layouts for DSP" - AoS vs SoA patterns
- "The Structure and Interpretation of Computer Programs" - Abstractions

---

**Last Updated:** 2026-02-23  
**Applies To:** melange solver layer, device models, primitives  
**Maintainer:** AI agent on duty
