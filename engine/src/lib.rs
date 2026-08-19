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

mod calibrate;
mod coverage;
mod emit;
mod fill;
mod mirror;
pub mod medial;
mod paint;
mod plan;
mod slice;

pub use calibrate::{test_cube_gcode, TEST_CUBE_H_MM, TEST_CUBE_XY_MM};
pub use mirror::plans_from_timeline;

pub use emit::{
    audit_combing, audit_flow_clamps, estimate_filament, estimate_filament_per_tool,
    estimate_seconds, format_duration, kind_label, per_layer_stats, to_gcode, used_tools,
    LayerStats,
};
pub use plan::{
    apply_bead_dabs, dab_covers, debug_islands, debug_uncovered, generate, generate_painted, generate_parts,
    plan_geometry, plan_geometry_tracked, restamp_paint, BeadDab, GeometryPlan, LayerPlan,
    PartPaint, PathKind, SliceProgress, ToolPath, Travel,
};
pub use slice::{slice_mesh, Layer, SliceParams};
