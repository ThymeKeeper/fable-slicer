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
pub mod medial;
mod paint;
mod plan;
mod slice;

pub use calibrate::{
    calibration_suite_gcode, comb_hub_r_mm, comb_tooth_angle, comb_tooth_flow, flow_comb_gcode,
    flow_from_comb_value, pa_from_height, pa_tower_gcode, COMB_FLOW_FAT, COMB_FLOW_THIN,
    COMB_H_MM, COMB_TEETH, COMB_TOOTH_LEN_MM, PA_STEP_SLOW_FRAC, PA_TOWER_FACTOR, PA_TOWER_START,
    TOWER_H_MM, TOWER_R_MM,
};

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
