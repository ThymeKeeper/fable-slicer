//! The built-in calibration print.
//!
//! The slicer is blind to the true deposited geometry, so flow and pressure
//! advance can't be derived — they have to be *measured*. One print does
//! both: a slab printed as a solid concentric ring field with its top left
//! open, at the **current** settings, so what you judge is exactly how you
//! print. The instrument is the open inner volume rather than any surface:
//! the rings show flow directly (crowding into each other or leaving
//! channels), and their corners along the diagonal collapse lines show the
//! pressure-advance transient. Adjust a value, reprint, compare.
//!
//! No ladder, no label, no reading transcribed back into the panel — the
//! knobs being judged are the ones the print was made with. Earlier ladder
//! instruments (a flow comb calipered per tooth, a PA tower swept with
//! height) were removed on 2026-08-18: the comb's free-bead width criterion
//! turned out to be material-biased — PETG read ~20% off the flow its real
//! prints wanted, while PLA's matched — and a criterion that lies for one
//! material can't be trusted for any. Judge the field itself, in its own
//! packing conditions. (They live on in git history if ever wanted back.)
//!
//! Order of operations still holds: flow first, then pressure advance.
//! Flow scales demand and compensation together, so the PA optimum barely
//! moves with a mis-pinned spool — but judge corners on a right-flow wall
//! anyway.

use crate::{generate_parts, to_gcode};
use config::Settings;

/// Test-cube footprint (mm): 30 × 30, wide enough that the concentric ring
/// field has a real inner volume to read and the diagonal collapse lines
/// are long.
pub const TEST_CUBE_XY_MM: f64 = 30.0;
/// Test-cube height (mm): 10 — tall enough to judge consistency up the
/// stack, short enough to reprint on a whim.
pub const TEST_CUBE_H_MM: f64 = 10.0;

/// G-code for the flow / pressure-advance test cube: a slab printed as a
/// SOLID CONCENTRIC FIELD, open top and bottom, at the profile's OWN
/// settings — speeds, temperatures, flow and PA exactly as real parts will
/// print. Nothing here neutralizes a setting the way a ladder instrument
/// would: the settings ARE what's under test. The only overrides are the
/// shell counts, which turn every layer into the same readable ring field.
pub fn test_cube_gcode(settings: &Settings, tool: u32) -> String {
    let mut s = settings.clone();
    // The whole cross-section as concentric rings, top and bottom alike:
    // the counts are literal, so 0 skins anywhere means no membrane hides
    // the field. Every layer is the instrument.
    s.wall_count = 99;
    s.bottom_layers = 0;
    s.top_layers = 0;
    // A lone synthetic mesh at the origin: the GUI places objects itself and
    // runs with auto-center off, so without this the cube prints in the
    // front-left corner with its skirt over the bed edge.
    s.auto_center_on_bed = true;
    let mut tris = Vec::new();
    let h = TEST_CUBE_XY_MM / 2.0;
    prism(&[[-h, -h], [h, -h], [h, h], [-h, h]], 0.0, TEST_CUBE_H_MM, &mut tris);
    let mesh = mesh::Mesh::from_triangle_soup(&tris);
    let plans = generate_parts(&[(&mesh, tool)], &s);
    let header = format!(
        "; TEST CUBE: {TEST_CUBE_XY_MM:.0} x {TEST_CUBE_XY_MM:.0} x {TEST_CUBE_H_MM:.0} mm printed as a solid concentric ring\n\
         ; field (99 walls, no top or bottom skin) at THIS profile's own settings —\n\
         ; flow x {:.3}, pressure advance {:.4}. The TOP LAYER is the instrument:\n\
         ; FLOW: look across the ring field. Rings crowding/piling into each other\n\
         ; = too much; visible channels or gaps between them = too little.\n\
         ; PA: look at the ring CORNERS along the diagonal collapse lines. Bulged\n\
         ; corners = too little PA; starved, rounded, or gapped corners = too much.\n\
         ; Adjust one value, reprint, compare.\n",
        s.extrusion_multiplier, s.pressure_advance,
    );
    format!("{header}{}", to_gcode(&plans, &s))
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
    fn test_cube_is_an_open_ring_field_at_the_profile_settings() {
        // The cube judges the SETTINGS THEMSELVES, so it must not neutralize
        // any of them: the profile's flow and pressure advance ride through
        // untouched, and the only overrides are the shell counts that open
        // the field for reading.
        let mut s = Settings::default();
        s.auto_center_on_bed = false; // the GUI's frame — the cube must re-center
        s.bed_size_x_mm = 152.4;
        s.bed_size_y_mm = 152.4;
        s.extrusion_multiplier = 0.93;
        s.pressure_advance = 0.05;
        s.wall_count = 3; // a normal profile: the cube overrides to a full field
        s.top_layers = 4;
        s.bottom_layers = 3;
        let g = test_cube_gcode(&s, 0);
        // Nothing skins: no membrane over the field, top or bottom.
        assert!(!g.contains(";TYPE:Top"), "the top must stay open to read");
        assert!(!g.contains(";TYPE:Bottom surface"), "the bed face is field too");
        // A ring field, not a walls-and-infill part: no sparse fill anywhere.
        assert!(!g.contains(";TYPE:Sparse infill"), "99 walls leave no sparse infill");
        assert!(g.contains(";TYPE:Inner wall"), "the field is concentric walls");
        // The profile's own PA is emitted (not swept, not zeroed).
        assert!(g.contains("SET_PRESSURE_ADVANCE ADVANCE=0.0500"), "profile PA rides through");
        // Centered on the bed like every synthetic instrument.
        assert!(!g.contains(" X-") && !g.contains(" Y-"), "no off-bed moves");
        // The header states what it was printed at — that is the value under
        // test, and the print is worthless without knowing it.
        assert!(g.contains("flow x 0.930"), "header states the flow under test");
    }
}
