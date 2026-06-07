//! G-code emission for the slicer.
//!
//! Planned for M1+:
//!   - extrusion math: `E = length * line_width * layer_height / filament_area`
//!   - a small g-code AST + writer (G0/G1, temps, fan, retraction)
//!   - start/end g-code templates per printer profile
//!   - trapezoidal motion simulation for accurate time estimates
//!   - G2/G3 arc fitting (M5)
//!
//! This is intentionally an empty placeholder so the workspace structure is in
//! place; it gains real types when M1 starts.

/// Crate version string, so the stub exports something concrete.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
