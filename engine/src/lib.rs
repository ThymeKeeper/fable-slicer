//! The slicer core.
//!
//! Pipeline so far:
//!   1. `slice` — mesh -> per-layer closed polygons (M0).
//!   2. `plan`  — polygons -> per-layer toolpaths: concentric walls + clipped
//!                line infill (M1).
//!   3. `emit`  — toolpaths + settings -> G-code (M1).
//!
//! Still to come (see PLAN.md): top/bottom solid surfaces, retraction/combing,
//! supports, variable layers, etc.

mod emit;
mod plan;
mod slice;

pub use emit::to_gcode;
pub use plan::{generate, LayerPlan, PathKind, ToolPath};
pub use slice::{slice_mesh, Layer, SliceParams};
