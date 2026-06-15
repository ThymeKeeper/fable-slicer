//! Built-in calibration prints.
//!
//! The slicer is blind to the true deposited geometry, so flow and pressure
//! advance can't be derived — they have to be *measured*. These helpers
//! generate a test print from the **current** settings (so the result is valid
//! for how the user actually prints) and turn the single number the user
//! measures back into a profile value. The number lands in the filament
//! profile (`extrusion_multiplier` / `pressure_advance`); see config::profile.

use crate::{generate, to_gcode};
use config::Settings;

/// Edge length of the single-wall flow cube (mm) — tall enough to caliper a
/// face well above the first few layers, where flow is still settling.
pub const FLOW_TEST_MM: f64 = 20.0;

/// Strip a copy of the settings down to a single-wall, open-topped box: one
/// perimeter, no top/bottom/infill, none of the wall-reshaping modes. Printed
/// at the user's real line width, the wall's measured thickness reads back the
/// true flow ratio.
fn single_wall(settings: &Settings) -> Settings {
    let mut s = settings.clone();
    s.wall_count = 1;
    s.top_layers = 0;
    s.bottom_layers = 0;
    s.infill_density = 0.0;
    s.spiral_vase = false;
    s.brick_layers = false;
    s.half_height_outer_walls = false;
    // The cal cube is a lone synthetic mesh at the origin (cube() spans 0..size).
    // The GUI runs with auto-center OFF (it places objects itself), so without
    // this the cube prints in the front-left corner and its skirt spills off the
    // bed edge — the printer rejects it as a move out of range. Force centering.
    s.auto_center_on_bed = true;
    // A single-wall cube has a tiny per-layer path; the user's general
    // min-layer-time (tuned for real prints) stretches each layer to ~8 s by
    // crawling the walls to a few mm/s — slow, and an unrepresentative speed to
    // measure at. A throwaway cal only needs the wall *width* (robust to a little
    // droop), so use a small floor and print plainly — heat control's temp/speed
    // schedule is moot for a part we're going to throw away.
    s.min_layer_time_s = 2.0;
    s.heat_control = false;
    s
}

/// G-code for the single-wall flow-calibration print.
pub fn flow_test_gcode(settings: &Settings) -> String {
    let s = single_wall(settings);
    to_gcode(&generate(&mesh::Mesh::cube(FLOW_TEST_MM), &s), &s)
}

/// New flow multiplier from a single-wall measurement: the wall should be one
/// line width thick, so scale the current multiplier by `target / measured`.
/// A nonsense measurement leaves the multiplier untouched.
pub fn flow_from_wall(current_flow: f64, line_width_mm: f64, measured_mm: f64) -> f64 {
    if measured_mm <= 0.0 || line_width_mm <= 0.0 {
        return current_flow;
    }
    current_flow * (line_width_mm / measured_mm)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flow_test_is_a_tall_single_wall() {
        let s = Settings::default();
        let wall = flow_test_gcode(&s);
        // Tall enough to measure: a 20 mm cube is ~100 layers at 0.2 mm.
        assert!(wall.matches("; LAYER ").count() > 50, "tall enough to caliper");
        // And genuinely stripped down — far leaner than the same cube printed
        // solid (walls + top/bottom + infill), confirming the overrides took.
        let solid = to_gcode(&generate(&mesh::Mesh::cube(FLOW_TEST_MM), &s), &s);
        assert!(
            wall.lines().count() * 2 < solid.lines().count(),
            "single wall ({}) should be far leaner than solid ({})",
            wall.lines().count(),
            solid.lines().count()
        );
    }

    #[test]
    fn flow_test_centers_on_bed_even_with_auto_center_off() {
        // The GUI positions objects itself, so it runs with auto_center_on_bed
        // = false. The flow test is a lone cube, so it must re-enable centering
        // or it prints off the front-left corner and the skirt runs off the bed
        // (negative coords → the printer's "move out of range").
        let mut s = Settings::default();
        s.auto_center_on_bed = false;
        s.bed_size_x_mm = 152.4;
        s.bed_size_y_mm = 152.4;
        let g = flow_test_gcode(&s);
        assert!(!g.contains(" X-"), "no off-bed negative X moves");
        assert!(!g.contains(" Y-"), "no off-bed negative Y moves");
    }

    #[test]
    fn flow_test_does_not_crawl_under_a_high_min_layer_time() {
        // The user's general min-layer-time (8 s) would stretch the tiny
        // single-wall layers and crawl the walls to a few mm/s (~13 min for a
        // 20 mm cube). The cal relaxes that floor and prints plainly.
        let mut s = Settings::default();
        s.min_layer_time_s = 8.0;
        let cal = single_wall(&s);
        assert!(cal.min_layer_time_s <= 2.0, "cal relaxes the layer-time floor");
        assert!(!cal.heat_control, "cal prints plainly, no schedule");
        let layers = generate(&mesh::Mesh::cube(FLOW_TEST_MM), &cal);
        let secs = crate::estimate_seconds(&layers, &cal);
        assert!(secs < 400.0, "cal prints in a few minutes, not the floor's ~13 (got {secs:.0}s)");
    }

    #[test]
    fn flow_from_wall_scales_and_guards() {
        // Over-extruding: a 0.45 mm wall measured at 0.50 → drop flow to 0.90×.
        let f = flow_from_wall(1.0, 0.45, 0.50);
        assert!((f - 0.9).abs() < 1e-9, "{f}");
        // Compounds on an already-pinned multiplier.
        assert!((flow_from_wall(0.95, 0.45, 0.45) - 0.95).abs() < 1e-9);
        // Nonsense input is a no-op, never a divide-by-zero.
        assert_eq!(flow_from_wall(1.0, 0.45, 0.0), 1.0);
        assert_eq!(flow_from_wall(1.0, 0.0, 0.45), 1.0);
    }
}
