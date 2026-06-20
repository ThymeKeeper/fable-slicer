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

mod arc;
mod calibrate;
mod distributed;
mod emit;
mod fill;
mod peel;
mod plan;
mod skeletal;
mod slice;
mod wall;

pub use calibrate::{flow_from_wall, flow_test_gcode, FLOW_TEST_MM};

pub use emit::{
    audit_combing, audit_flow_clamps, audit_heat_control_speed,
    audit_smoothing, effective_heat_target, estimate_filament, estimate_seconds, format_duration,
    kind_label, per_layer_stats, to_gcode, LayerStats, SlowdownRange, SmoothingReport,
};
pub use plan::{generate, LayerPlan, PathKind, ToolPath, Travel};
pub use slice::{slice_mesh, Layer, SliceParams};

/// Debug-only: full bead geometry (points mm, widths, closed) for probes.
pub fn dbg_variable_walls_full(
    outer: &geo2d::Polygons,
    inner: &geo2d::Polygons,
    lw: f64,
    sp: f64,
    cap: usize,
) -> Vec<(Vec<(f64, f64)>, Vec<f64>, bool)> {
    let vw = wall::variable_walls(outer, inner, lw, sp, cap, false);
    vw.inner
        .iter()
        .chain(vw.thin_outer.iter())
        .map(|b| {
            (
                b.points.iter().map(|p| (p.x_mm(), p.y_mm())).collect(),
                b.widths.clone(),
                b.closed,
            )
        })
        .collect()
}

/// Debug-only: run the variable-width wall field and return (length, closed,
/// mid width) per bead. For the dbg_arachne example.
pub fn dbg_variable_walls(
    outer: &geo2d::Polygons,
    inner: &geo2d::Polygons,
    lw: f64,
    sp: f64,
    cap: usize,
) -> Vec<(f64, bool, f64)> {
    let vw = wall::variable_walls(outer, inner, lw, sp, cap, false);
    vw.inner
        .iter()
        .chain(vw.thin_outer.iter())
        .map(|b| {
            let len: f64 = b
                .points
                .windows(2)
                .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
                .sum();
            (len, b.closed, b.widths[b.widths.len() / 2])
        })
        .collect()
}
