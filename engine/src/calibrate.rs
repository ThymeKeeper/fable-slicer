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
/// fully settles between corners, AND that a full-speed layer still takes ≥1 s:
/// the tower must print at the profile's real outer-wall speed (PA calibrated
/// slow reads high — the smoothing window dominates the shorter transients),
/// so the layer-time governor must never throttle it.
pub const PA_TOWER_MM: f64 = 50.0;
/// PA tower height (mm). With [`PA_TOWER_FACTOR`] the sweep spans 0–0.10:
/// the whole direct-drive range, at caliper-grade resolution (±0.5 mm of
/// height reads as ±0.001 of PA).
pub const PA_TOWER_H_MM: f64 = 50.0;
/// Pressure advance at the bed.
pub const PA_TOWER_START: f64 = 0.0;
/// Pressure advance added per mm of tower height.
pub const PA_TOWER_FACTOR: f64 = 0.002;
/// Height between index notches on the seam corner: one every 10 mm from the
/// bed = +0.020 of PA per notch, so the band is read by counting marks —
/// no caliper, no measuring-from-the-wrong-end. Each notch is an INWARD
/// two-layer corner chamfer: it prints as a short bridge anchored on both
/// walls. (The first cut was an outward collar — a cantilever over air on a
/// bottomless single wall; it curled, caught the nozzle, and bird's-nested
/// the print. Nothing on this tower may overhang.)
pub const PA_TOWER_MARK_MM: f64 = 10.0;
/// How far the notch cuts the corner (each chamfer leg, mm).
const PA_TOWER_NOTCH_MM: f64 = 2.5;

/// G-code for the pressure-advance tower: a single-wall square tube whose PA
/// ramps with height — Klipper's `TUNING_TOWER` sweep, but baked into the
/// file per layer, so there is no console incantation to run. Seams are held
/// in one corner column ([`config::SeamMode::Sharpest`]) and the same corner
/// carries the 10 mm index collars; judge the other three. Too little PA
/// bulges corners (pressure overshoots the slowdown), too much starves the
/// stretch right after them — the crispest band wins, and [`pa_from_height`]
/// turns its height into the profile value.
pub fn pa_tower_gcode(settings: &Settings) -> String {
    let mut s = single_wall(settings);
    s.seam_mode = config::SeamMode::Sharpest;
    // Calibrate at the speed the profile actually prints. The flow cube's
    // relaxed 2 s layer-time floor would throttle this wall to a fraction of
    // the outer-wall speed; with the 50 mm footprint a full-speed layer runs
    // ≥1 s anyway, so this floor never bites.
    s.min_layer_time_s = 1.0;
    // A tall single-bead tube cornering at full speed needs more than one
    // bead-ring of bed contact: give it a solid three-layer base plate.
    s.bottom_layers = 3;
    // The tube as a stack of prisms: plain squares, with the notch bands'
    // corner chamfered inward.
    let sq = [
        [0.0, 0.0],
        [PA_TOWER_MM, 0.0],
        [PA_TOWER_MM, PA_TOWER_MM],
        [0.0, PA_TOWER_MM],
    ];
    let notched = [
        [PA_TOWER_NOTCH_MM, 0.0],
        [PA_TOWER_MM, 0.0],
        [PA_TOWER_MM, PA_TOWER_MM],
        [0.0, PA_TOWER_MM],
        [0.0, PA_TOWER_NOTCH_MM],
    ];
    let mut tris = Vec::new();
    let mut z0 = 0.0;
    let mut mark = PA_TOWER_MARK_MM;
    while mark < PA_TOWER_H_MM - 0.5 {
        prism(&sq, z0, mark, &mut tris);
        prism(&notched, mark, mark + 0.4, &mut tris);
        z0 = mark + 0.4;
        mark += PA_TOWER_MARK_MM;
    }
    prism(&sq, z0, PA_TOWER_H_MM, &mut tris);
    let mesh = mesh::Mesh::from_triangle_soup(&tris);
    let g = to_gcode(&generate(&mesh, &s), &s);
    let mut out = String::with_capacity(g.len() + 8192);
    out.push_str(&format!(
        "; PA tower: pressure advance = {PA_TOWER_START} + {PA_TOWER_FACTOR} * z_mm\n\
         ; corner notches mark every {PA_TOWER_MARK_MM} mm from the BED (+{:.3} PA each);\n\
         ; find the crispest band on the three plain corners, apply its height in\n\
         ; the Filament panel (or PA = {PA_TOWER_START} + {PA_TOWER_FACTOR} * height by hand).\n",
        PA_TOWER_FACTOR * PA_TOWER_MARK_MM
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

/// Append a vertical prism over a CONVEX CCW footprint (fan triangulation) —
/// stacked prisms union at slice time into one solid.
fn prism(fp: &[[f64; 2]], z0: f64, z1: f64, tris: &mut Vec<[[f64; 3]; 3]>) {
    let n = fp.len();
    for i in 1..n - 1 {
        tris.push([
            [fp[0][0], fp[0][1], z0],
            [fp[i + 1][0], fp[i + 1][1], z0],
            [fp[i][0], fp[i][1], z0],
        ]);
        tris.push([
            [fp[0][0], fp[0][1], z1],
            [fp[i][0], fp[i][1], z1],
            [fp[i + 1][0], fp[i + 1][1], z1],
        ]);
    }
    for i in 0..n {
        let (a, b) = (fp[i], fp[(i + 1) % n]);
        tris.push([[a[0], a[1], z0], [b[0], b[1], z0], [b[0], b[1], z1]]);
        tris.push([[a[0], a[1], z0], [b[0], b[1], z1], [a[0], a[1], z1]]);
    }
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
        // One injected PA step directly after each layer marker (the index
        // collars' overhang stretches re-issue their own scaled PA in between,
        // so only the ramp lines are height-ordered).
        let lines: Vec<&str> = g.lines().collect();
        let ramp: Vec<f64> = lines
            .windows(2)
            .filter(|w| w[0].starts_with("; LAYER "))
            .filter_map(|w| w[1].strip_prefix("SET_PRESSURE_ADVANCE ADVANCE="))
            .filter_map(|v| v.parse().ok())
            .collect();
        let layers = g.matches("; LAYER ").count();
        assert!(layers > 200, "a 50 mm tower is ~250 layers, got {layers}");
        assert_eq!(ramp.len(), layers, "one injected PA step per layer");
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
    fn pa_tower_prints_at_the_real_outer_wall_speed() {
        // The whole point of the tower is calibrating at the speed the profile
        // actually prints. The flow cube's relaxed layer-time floor used to
        // throttle it ~2.7× — and PA calibrated slow reads high (the user's
        // 0.070 vs a true ~0.032). Dominant extrusion feed must be the plain
        // outer-wall speed, unthrottled.
        let s = Settings::default();
        let g = pa_tower_gcode(&s);
        // Judge only the wall itself — the base plate's dense first-layer skin
        // would otherwise dominate the line count at first-layer speed.
        let mut feed_mm: std::collections::HashMap<i64, u32> = std::collections::HashMap::new();
        let (mut f, mut wall) = (0i64, false);
        for l in g.lines() {
            if let Some(t) = l.strip_prefix(";TYPE:") {
                wall = t.trim() == "Outer wall";
                continue;
            }
            if !l.starts_with("G1 ") && !l.starts_with("G2 ") && !l.starts_with("G3 ") {
                continue;
            }
            if let Some(fs) = l.split(" F").nth(1) {
                f = fs.split(' ').next().unwrap_or("0").parse::<f64>().unwrap_or(0.0) as i64;
            }
            if wall && l.contains(" E") && !l.contains(" E-") {
                *feed_mm.entry(f).or_default() += 1;
            }
        }
        let dominant = feed_mm.iter().max_by_key(|(_, n)| **n).map(|(f, _)| *f).unwrap_or(0);
        let expect = s.external_perimeter_speed_mm_s * 60.0;
        assert!(
            (dominant as f64) >= expect * 0.9,
            "tower must run at outer-wall speed: dominant F{dominant} vs expected F{expect:.0}"
        );
    }

    #[test]
    fn pa_tower_has_index_notches() {
        // Inward corner chamfers every 10 mm from the bed: at each mark band
        // the seam corner's min(x+y) steps inward by the notch depth. (Inward,
        // never outward: an outward collar on a bottomless single wall is a
        // cantilever over air — the first cut bird's-nested a print.)
        let g = pa_tower_gcode(&Settings::default());
        let mut per_layer: Vec<(f64, f64)> = Vec::new(); // (z, min x+y over extrusions)
        let (mut z, mut m, mut x, mut y) = (0.0, f64::MAX, 0.0f64, 0.0f64);
        for l in g.lines() {
            if let Some(rest) = l.strip_prefix("; LAYER ") {
                if m < f64::MAX {
                    per_layer.push((z, m));
                }
                m = f64::MAX;
                z = rest.split("z=").nth(1).and_then(|v| v.trim().parse().ok()).unwrap_or(0.0);
            } else if l.starts_with("G0 ") || l.starts_with("G1 ") || l.starts_with("G2 ") || l.starts_with("G3 ") {
                let ex = l.split(" X").nth(1).and_then(|s| s.split(' ').next().unwrap_or("").parse::<f64>().ok());
                let ey = l.split(" Y").nth(1).and_then(|s| s.split(' ').next().unwrap_or("").parse::<f64>().ok());
                if let Some(v) = ex { x = v; }
                if let Some(v) = ey { y = v; }
                if l.contains(" E") && !l.contains(" E-") && (ex.is_some() || ey.is_some()) {
                    m = m.min(x + y);
                }
            }
        }
        if m < f64::MAX {
            per_layer.push((z, m));
        }
        let base = per_layer
            .iter()
            .filter(|(z, _)| *z > 2.0 && (z % 10.0) > 1.0)
            .map(|&(_, v)| v)
            .fold(f64::MAX, f64::min);
        let marks = per_layer
            .iter()
            .filter(|&&(z, v)| z > 5.0 && (z % 10.0) <= 0.5 && v > base + 1.2)
            .count();
        assert!(marks >= 4, "notches at 10/20/30/40 mm must cut the corner, saw {marks}");
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
