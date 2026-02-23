// melange-plugin: nih-plug integration helpers
//
// Layer 5 of the melange stack. Glue between melange circuit models and plugin frameworks:
// - Voice management template (note-on/off lifecycle, silence detection, stealing)
// - Oversampling decision framework (classify stages, auto-determine rates)
// - Parameter mapping (physical component values <-> plugin parameters)
// - Calibration infrastructure (sweep grids, sensitivity analysis)
