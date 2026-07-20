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
    // The cal cube is a lone synthetic mesh at the origin (cube() spans 0..size).
    // The GUI runs with auto-center OFF (it places objects itself), so without
    // this the cube prints in the front-left corner and its skirt spills off the
    // bed edge — the printer rejects it as a move out of range. Force centering.
    s.auto_center_on_bed = true;
    // A single-wall cube has a tiny per-layer path; the user's general
    // min-layer-time (tuned for real prints) stretches each layer to ~8 s by
    // crawling the walls to a few mm/s — slow, and an unrepresentative speed to
    // measure at. A throwaway cal only needs the wall *width* (robust to a little
    // droop), so use a small floor.
    s.min_layer_time_s = 2.0;
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

/// Footprint edge of the PA tower (mm) — sides long enough that melt pressure
/// fully settles between corners, so every corner is a clean step transient.
pub const PA_TOWER_MM: f64 = 30.0;
/// PA tower height (mm). With [`PA_TOWER_FACTOR`] the sweep spans 0–0.10:
/// the whole direct-drive range, at caliper-grade resolution (±0.5 mm of
/// height reads as ±0.001 of PA).
pub const PA_TOWER_H_MM: f64 = 50.0;
/// Pressure advance at the bed.
pub const PA_TOWER_START: f64 = 0.0;
/// Pressure advance added per mm of tower height.
pub const PA_TOWER_FACTOR: f64 = 0.002;

/// G-code for the pressure-advance tower: a single-wall square tube whose PA
/// ramps with height — Klipper's `TUNING_TOWER` sweep, but baked into the
/// file per layer, so there is no console incantation to run. Seams are held
/// in one corner column ([`config::SeamMode::Sharpest`]); judge the other
/// three. Too little PA bulges corners (pressure overshoots the slowdown),
/// too much starves the stretch right after them — the crispest band wins,
/// and [`pa_from_height`] turns its measured height into the profile value.
pub fn pa_tower_gcode(settings: &Settings) -> String {
    let mut s = single_wall(settings);
    s.seam_mode = config::SeamMode::Sharpest;
    let mesh = cuboid(PA_TOWER_MM, PA_TOWER_MM, PA_TOWER_H_MM);
    let g = to_gcode(&generate(&mesh, &s), &s);
    let mut out = String::with_capacity(g.len() + 8192);
    out.push_str(&format!(
        "; PA tower: pressure advance = {PA_TOWER_START} + {PA_TOWER_FACTOR} * z_mm\n\
         ; measure the height (mm) of the crispest corners and apply it in the\n\
         ; Filament panel (or PA = {PA_TOWER_START} + {PA_TOWER_FACTOR} * height by hand).\n"
    ));
    for line in g.lines() {
        out.push_str(line);
        out.push('\n');
        // Ride the layer markers: re-issue PA for the new height right after
        // each one, overriding the profile value the preamble set.
        if let Some(rest) = line.strip_prefix("; LAYER ") {
            if let Some(z) = rest.split("z=").nth(1).and_then(|v| v.trim().parse::<f64>().ok()) {
                let pa = PA_TOWER_START + PA_TOWER_FACTOR * z;
                out.push_str(&format!("SET_PRESSURE_ADVANCE ADVANCE={pa:.4}\n"));
            }
        }
    }
    out
}

/// Pressure advance from the measured best-corner height on the PA tower.
/// A height off the tower leaves the current value untouched.
pub fn pa_from_height(current_pa: f64, height_mm: f64) -> f64 {
    if height_mm <= 0.0 || height_mm > PA_TOWER_H_MM {
        return current_pa;
    }
    PA_TOWER_START + PA_TOWER_FACTOR * height_mm
}

/// An axis-aligned solid box as a triangle soup (the calibration meshes are
/// synthetic — no model file involved).
fn cuboid(x: f64, y: f64, z: f64) -> mesh::Mesh {
    let v = [
        [0.0, 0.0, 0.0], [x, 0.0, 0.0], [x, y, 0.0], [0.0, y, 0.0],
        [0.0, 0.0, z], [x, 0.0, z], [x, y, z], [0.0, y, z],
    ];
    let mut tris = Vec::new();
    for [a, b, c, d] in
        [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    {
        tris.push([v[a], v[b], v[c]]);
        tris.push([v[a], v[c], v[d]]);
    }
    mesh::Mesh::from_triangle_soup(&tris)
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
        let layers = generate(&mesh::Mesh::cube(FLOW_TEST_MM), &cal);
        let secs = crate::estimate_seconds(&layers, &cal);
        assert!(secs < 400.0, "cal prints in a few minutes, not the floor's ~13 (got {secs:.0}s)");
    }

    #[test]
    fn pa_tower_ramps_with_height() {
        let g = pa_tower_gcode(&Settings::default());
        // A 50 mm tower at 0.2 mm layers: one PA line per layer.
        let pas: Vec<f64> = g
            .lines()
            .filter_map(|l| l.strip_prefix("SET_PRESSURE_ADVANCE ADVANCE="))
            .filter_map(|v| v.parse().ok())
            .collect();
        // The preamble's profile PA (if any) comes first; the ramp is the one
        // line per layer that follows.
        let layers = g.matches("; LAYER ").count();
        assert!(layers > 200, "a 50 mm tower is ~250 layers, got {layers}");
        assert!(pas.len() >= layers, "one PA step per layer");
        let ramp = &pas[pas.len() - layers..];
        assert!(ramp.windows(2).all(|w| w[1] >= w[0]), "the sweep is monotonic");
        assert!(ramp[0] < 0.002, "starts at the bottom of the range");
        let top = ramp.last().unwrap();
        assert!(
            (PA_TOWER_START + PA_TOWER_FACTOR * PA_TOWER_H_MM - top).abs() < 0.003,
            "ends at the top of the range, got {top}"
        );
        // Single wall: no interior features to muddy the corners.
        assert!(!g.contains(";TYPE:Sparse infill") && !g.contains(";TYPE:Top surface"));
    }

    #[test]
    fn pa_from_height_maps_and_guards() {
        // The user's measured 16 mm reads back the classic direct-drive 0.032.
        assert!((pa_from_height(0.05, 16.0) - 0.032).abs() < 1e-9);
        // Off-tower nonsense is a no-op.
        assert_eq!(pa_from_height(0.05, 0.0), 0.05);
        assert_eq!(pa_from_height(0.05, 99.0), 0.05);
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
