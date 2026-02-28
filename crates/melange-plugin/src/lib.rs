// melange-plugin: nih-plug integration helpers
//
// Layer 5 of the melange stack. Glue between melange circuit models and plugin frameworks:
// - Voice management template (note-on/off lifecycle, silence detection, stealing)
// - Oversampling decision framework (classify stages, auto-determine rates)
// - Parameter mapping (physical component values <-> plugin parameters)
// - Calibration infrastructure (sweep grids, sensitivity analysis)

// Re-export dependencies so plugin code can access them through this crate
pub use melange_primitives;
pub use melange_solver;

/// Maps a physical component value (e.g., resistance in ohms) to a normalized
/// parameter range [0.0, 1.0] using a linear mapping.
///
/// Returns `None` if `min >= max`.
pub fn linear_param_map(value: f64, min: f64, max: f64) -> Option<f64> {
    if min >= max {
        return None;
    }
    Some(((value - min) / (max - min)).clamp(0.0, 1.0))
}

/// Maps a normalized parameter [0.0, 1.0] back to a physical component value.
///
/// Returns `None` if `min >= max`.
pub fn linear_param_unmap(normalized: f64, min: f64, max: f64) -> Option<f64> {
    if min >= max {
        return None;
    }
    let clamped = normalized.clamp(0.0, 1.0);
    Some(min + clamped * (max - min))
}

/// Maps a physical component value to a normalized parameter range [0.0, 1.0]
/// using a logarithmic mapping. Useful for potentiometer values where
/// perceptual response is logarithmic (e.g., audio taper).
///
/// Returns `None` if `min <= 0.0`, `min >= max`, or `value <= 0.0`.
pub fn log_param_map(value: f64, min: f64, max: f64) -> Option<f64> {
    if min <= 0.0 || min >= max || value <= 0.0 {
        return None;
    }
    let log_min = min.ln();
    let log_max = max.ln();
    Some(((value.ln() - log_min) / (log_max - log_min)).clamp(0.0, 1.0))
}

/// Maps a normalized parameter [0.0, 1.0] back to a physical component value
/// using a logarithmic mapping.
///
/// Returns `None` if `min <= 0.0` or `min >= max`.
pub fn log_param_unmap(normalized: f64, min: f64, max: f64) -> Option<f64> {
    if min <= 0.0 || min >= max {
        return None;
    }
    let clamped = normalized.clamp(0.0, 1.0);
    let log_min = min.ln();
    let log_max = max.ln();
    Some((log_min + clamped * (log_max - log_min)).exp())
}

/// Configuration for a parameter that maps to a circuit component.
#[derive(Debug, Clone)]
pub struct ParamMapping {
    /// Human-readable parameter name
    pub name: String,
    /// Minimum physical value
    pub min: f64,
    /// Maximum physical value
    pub max: f64,
    /// Default physical value
    pub default: f64,
    /// Whether to use logarithmic scaling
    pub log_scale: bool,
}

impl ParamMapping {
    /// Create a new linear parameter mapping.
    pub fn linear(name: &str, min: f64, max: f64, default: f64) -> Self {
        Self {
            name: name.to_string(),
            min,
            max,
            default,
            log_scale: false,
        }
    }

    /// Create a new logarithmic parameter mapping.
    pub fn logarithmic(name: &str, min: f64, max: f64, default: f64) -> Self {
        Self {
            name: name.to_string(),
            min,
            max,
            default,
            log_scale: true,
        }
    }

    /// Map a physical value to normalized [0, 1].
    pub fn normalize(&self, value: f64) -> Option<f64> {
        if self.log_scale {
            log_param_map(value, self.min, self.max)
        } else {
            linear_param_map(value, self.min, self.max)
        }
    }

    /// Map a normalized [0, 1] value back to a physical value.
    pub fn denormalize(&self, normalized: f64) -> Option<f64> {
        if self.log_scale {
            log_param_unmap(normalized, self.min, self.max)
        } else {
            linear_param_unmap(normalized, self.min, self.max)
        }
    }

    /// Get the default value normalized to [0, 1].
    pub fn default_normalized(&self) -> Option<f64> {
        self.normalize(self.default)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // === Smoke tests: crate builds and dependencies are accessible ===

    #[test]
    fn smoke_primitives_reexport() {
        // Verify melange_primitives is accessible through this crate
        let gain = melange_primitives::util::db_to_gain(0.0);
        assert!((gain - 1.0).abs() < 1e-10, "0 dB should be gain 1.0");
    }

    #[test]
    fn smoke_solver_reexport() {
        // Verify melange_solver is accessible through this crate
        let netlist_str = "Test\nR1 in 0 1k\n.end";
        let netlist = melange_solver::parser::Netlist::parse(netlist_str);
        assert!(netlist.is_ok(), "Should parse a simple netlist");
    }

    // === Linear parameter mapping tests ===

    #[test]
    fn linear_param_map_midpoint() {
        let result = linear_param_map(500.0, 0.0, 1000.0);
        assert_eq!(result, Some(0.5));
    }

    #[test]
    fn linear_param_map_at_min() {
        let result = linear_param_map(100.0, 100.0, 10000.0);
        assert_eq!(result, Some(0.0));
    }

    #[test]
    fn linear_param_map_at_max() {
        let result = linear_param_map(10000.0, 100.0, 10000.0);
        assert_eq!(result, Some(1.0));
    }

    #[test]
    fn linear_param_map_clamps_below_min() {
        let result = linear_param_map(-10.0, 0.0, 100.0);
        assert_eq!(result, Some(0.0));
    }

    #[test]
    fn linear_param_map_clamps_above_max() {
        let result = linear_param_map(200.0, 0.0, 100.0);
        assert_eq!(result, Some(1.0));
    }

    #[test]
    fn linear_param_map_invalid_range() {
        // min >= max should return None
        assert_eq!(linear_param_map(50.0, 100.0, 100.0), None);
        assert_eq!(linear_param_map(50.0, 200.0, 100.0), None);
    }

    #[test]
    fn linear_param_unmap_midpoint() {
        let result = linear_param_unmap(0.5, 0.0, 1000.0);
        assert_eq!(result, Some(500.0));
    }

    #[test]
    fn linear_param_unmap_at_edges() {
        assert_eq!(linear_param_unmap(0.0, 100.0, 10000.0), Some(100.0));
        assert_eq!(linear_param_unmap(1.0, 100.0, 10000.0), Some(10000.0));
    }

    #[test]
    fn linear_param_unmap_clamps_input() {
        // Values outside [0,1] should be clamped
        assert_eq!(linear_param_unmap(-0.5, 0.0, 100.0), Some(0.0));
        assert_eq!(linear_param_unmap(1.5, 0.0, 100.0), Some(100.0));
    }

    #[test]
    fn linear_param_roundtrip() {
        let original = 3456.0;
        let min = 100.0;
        let max = 10000.0;
        let normalized = linear_param_map(original, min, max).unwrap();
        let recovered = linear_param_unmap(normalized, min, max).unwrap();
        assert!((original - recovered).abs() < 1e-10);
    }

    // === Logarithmic parameter mapping tests ===

    #[test]
    fn log_param_map_at_min() {
        let result = log_param_map(100.0, 100.0, 1_000_000.0);
        assert_eq!(result, Some(0.0));
    }

    #[test]
    fn log_param_map_at_max() {
        let result = log_param_map(1_000_000.0, 100.0, 1_000_000.0);
        assert!((result.unwrap() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn log_param_map_geometric_midpoint() {
        // Geometric midpoint of 100 and 10000 is sqrt(100*10000) = 1000
        let result = log_param_map(1000.0, 100.0, 10000.0);
        assert!((result.unwrap() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn log_param_map_invalid_range() {
        // min <= 0
        assert_eq!(log_param_map(50.0, 0.0, 100.0), None);
        assert_eq!(log_param_map(50.0, -10.0, 100.0), None);
        // min >= max
        assert_eq!(log_param_map(50.0, 100.0, 100.0), None);
        // value <= 0
        assert_eq!(log_param_map(0.0, 1.0, 100.0), None);
        assert_eq!(log_param_map(-5.0, 1.0, 100.0), None);
    }

    #[test]
    fn log_param_unmap_at_edges() {
        let min = 100.0;
        let max = 10000.0;
        let at_zero = log_param_unmap(0.0, min, max).unwrap();
        let at_one = log_param_unmap(1.0, min, max).unwrap();
        assert!((at_zero - min).abs() < 1e-10);
        assert!((at_one - max).abs() < 1e-6);
    }

    #[test]
    fn log_param_roundtrip() {
        let original = 4700.0; // typical pot value in ohms
        let min = 100.0;
        let max = 1_000_000.0;
        let normalized = log_param_map(original, min, max).unwrap();
        let recovered = log_param_unmap(normalized, min, max).unwrap();
        assert!((original - recovered).abs() / original < 1e-10);
    }

    // === ParamMapping struct tests ===

    #[test]
    fn param_mapping_linear_creation() {
        let pm = ParamMapping::linear("Tone", 100.0, 10000.0, 5000.0);
        assert_eq!(pm.name, "Tone");
        assert_eq!(pm.min, 100.0);
        assert_eq!(pm.max, 10000.0);
        assert_eq!(pm.default, 5000.0);
        assert!(!pm.log_scale);
    }

    #[test]
    fn param_mapping_logarithmic_creation() {
        let pm = ParamMapping::logarithmic("Volume", 100.0, 1_000_000.0, 10000.0);
        assert_eq!(pm.name, "Volume");
        assert!(pm.log_scale);
    }

    #[test]
    fn param_mapping_normalize_linear() {
        let pm = ParamMapping::linear("Test", 0.0, 100.0, 50.0);
        assert_eq!(pm.normalize(50.0), Some(0.5));
        assert_eq!(pm.normalize(0.0), Some(0.0));
        assert_eq!(pm.normalize(100.0), Some(1.0));
    }

    #[test]
    fn param_mapping_normalize_log() {
        let pm = ParamMapping::logarithmic("Test", 100.0, 10000.0, 1000.0);
        let mid = pm.normalize(1000.0).unwrap();
        assert!((mid - 0.5).abs() < 1e-10);
    }

    #[test]
    fn param_mapping_denormalize_linear() {
        let pm = ParamMapping::linear("Test", 0.0, 1000.0, 500.0);
        assert_eq!(pm.denormalize(0.0), Some(0.0));
        assert_eq!(pm.denormalize(0.5), Some(500.0));
        assert_eq!(pm.denormalize(1.0), Some(1000.0));
    }

    #[test]
    fn param_mapping_denormalize_log() {
        let pm = ParamMapping::logarithmic("Test", 100.0, 10000.0, 1000.0);
        let at_half = pm.denormalize(0.5).unwrap();
        assert!((at_half - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn param_mapping_default_normalized() {
        let pm = ParamMapping::linear("Test", 0.0, 100.0, 25.0);
        assert_eq!(pm.default_normalized(), Some(0.25));
    }

    #[test]
    fn param_mapping_roundtrip_linear() {
        let pm = ParamMapping::linear("R1", 100.0, 50000.0, 10000.0);
        let norm = pm.normalize(10000.0).unwrap();
        let back = pm.denormalize(norm).unwrap();
        assert!((back - 10000.0).abs() < 1e-10);
    }

    #[test]
    fn param_mapping_roundtrip_log() {
        let pm = ParamMapping::logarithmic("R1", 100.0, 1_000_000.0, 47000.0);
        let norm = pm.normalize(47000.0).unwrap();
        let back = pm.denormalize(norm).unwrap();
        assert!((back - 47000.0).abs() / 47000.0 < 1e-10);
    }

    #[test]
    fn param_mapping_clone() {
        let pm = ParamMapping::linear("Test", 0.0, 100.0, 50.0);
        let pm2 = pm.clone();
        assert_eq!(pm.name, pm2.name);
        assert_eq!(pm.min, pm2.min);
        assert_eq!(pm.max, pm2.max);
    }

    #[test]
    fn param_mapping_debug() {
        let pm = ParamMapping::linear("Test", 0.0, 100.0, 50.0);
        let debug_str = format!("{:?}", pm);
        assert!(debug_str.contains("Test"));
        assert!(debug_str.contains("ParamMapping"));
    }
}
