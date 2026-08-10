//! Built-in calibration prints.
//!
//! The slicer is blind to the true deposited geometry, so flow and pressure
//! advance can't be derived — they have to be *measured*. These helpers
//! generate a test print from the **current** settings (so the result is
//! valid for how the user actually prints) and turn the single number the
//! user reads off it back into a profile value. The number lands in the
//! filament profile (`extrusion_multiplier` / `pressure_advance`); see
//! config::profile.
//!
//! Each calibration gets the instrument its physics wants. Pressure advance
//! is a TRANSIENT phenomenon: a single-wall teardrop helix sweeps PA with
//! height and the one corner is judged where the transient reads clean.
//! Flow is a STEADY-STATE phenomenon: a comb prints one tooth per flow
//! multiplier and each tooth is calipered mid-face with the full jaw flats.
//! One philosophy joins them: vary the parameter and locate where the print
//! is *right*, instead of measuring an error and correcting it (a
//! correction inherits the bead model's error plus the measurement's and
//! has to be iterated). The PA tower reads the equality off a height; the
//! comb solves two measured teeth for the line-width crossing — no bead
//! model in either loop. The tests are independent in the right order:
//! PA adds E only while extruder velocity CHANGES, so it cannot touch a
//! mid-tooth steady-state reading (calibrate flow with any PA); flow scales
//! demand and compensation together, so the PA optimum barely moves with a
//! mis-pinned spool — but judge the corner on a right-flow wall anyway:
//! flow first, then PA.

use crate::{generate_parts, to_gcode};
use config::Settings;

/// Radius of the tower teardrop's circular back (mm). The footprint is a
/// teardrop: a 270° arc closed by its two tangent legs meeting in a single
/// 90° corner at the front. One corner replaces a square's four — corners
/// on a loop are not independent measurements (same state, same speed;
/// they differ only by seam pollution and ringing), so the tower keeps the
/// one that reads clean and spends the rest of the loop settling melt
/// pressure for it. The curve also braces the single-bead wall: the old
/// square's index notches jogged the corner and destabilized it, and its
/// 50 mm flat faces were the floppiest span — the arc is self-bracing and
/// the tangent legs are only ~30 mm. The ~201 mm perimeter (on par with the
/// 50 mm square) keeps a layer above 1 s of natural print time for wall
/// speeds up to ~200 mm/s, so successive layers get real cooling on any
/// realistic profile.
pub const TOWER_R_MM: f64 = 30.0;
/// Tower height (mm). With [`PA_TOWER_FACTOR`] the PA sweep spans 0–0.10:
/// the whole direct-drive range, at caliper-grade resolution (±0.5 mm of
/// height reads as ±0.001 of PA).
pub const TOWER_H_MM: f64 = 50.0;
/// Pressure advance at the bed.
pub const PA_TOWER_START: f64 = 0.0;
/// Pressure advance added per mm of tower height.
pub const PA_TOWER_FACTOR: f64 = 0.002;
/// Flow-comb FAT tooth multiplier (× the profile flow) — the tooth at the
/// handle end. The comb replaced the old continuous flow-swept tower: its
/// ramp's thickness gradient was ~0.002 mm per mm of height, so locating
/// the wall == line-width equality demanded caliper-TIP readings, and a
/// ±0.02 mm error mislocated it by ~10 mm (~6% of flow). A comb tooth is
/// UNIFORM — one flow per tooth, measured mid-face with the jaws' full
/// flats, no localization anywhere — and the teeth pin the real system's
/// measured thickness-vs-flow line, which the read-out solves for the
/// line-width crossing (~±1.5% from the same calipers). The middle teeth
/// are a built-in linearity check: their thicknesses must step evenly, and
/// a tooth off the line exposes its own bad measurement.
pub const COMB_FLOW_FAT: f64 = 1.12;
/// Flow-comb THIN tooth multiplier — the tooth at the far end.
pub const COMB_FLOW_THIN: f64 = 0.88;
/// Teeth on the comb; flow steps evenly from fat to thin along the spine.
pub const COMB_TEETH: usize = 7;
/// Tooth length (mm) past the spine. Long enough that the mid-face
/// measurement zone sits several PA-smoothing-window travel distances from
/// the corner transients at both ends.
pub const COMB_TOOTH_LEN_MM: f64 = 20.0;
/// Tooth footprint width (mm): the ring cavity takes a caliper's inner jaw.
pub const COMB_TOOTH_W_MM: f64 = 6.0;
/// Tooth-to-tooth pitch (mm): a 6 mm gap between teeth for the outer jaw —
/// though the two teeth that matter (the ends) have fully open outer faces.
pub const COMB_TOOTH_PITCH_MM: f64 = 12.0;
/// Spine depth (mm): the bar joining the teeth, printed at true flow.
pub const COMB_SPINE_D_MM: f64 = 4.0;
/// Spine overhang past the fat tooth (mm) — the handle. Its one job is to
/// label orientation: the handle end is the FAT end.
pub const COMB_HANDLE_MM: f64 = 8.0;
/// Comb height (mm): tall enough for full caliper-jaw flats on any tooth.
pub const COMB_H_MM: f64 = 12.0;

/// Strip a copy of the settings down to the calibration tower: one bare wall
/// above a solid three-layer base plate, printed as a seamless helix, at the
/// profile's true speed.
fn tower_settings(settings: &Settings) -> Settings {
    let mut s = settings.clone();
    s.wall_count = 1;
    s.top_layers = 0;
    s.infill_density = 0.0;
    // A tall single-bead tube cornering at full speed needs more than one
    // bead-ring of bed contact: give it a solid three-layer base plate. It
    // also marks which end of the tower is height zero.
    s.bottom_layers = 3;
    // A fuzzed wall can't be calipered and its jitter (±0.15 mm by default)
    // is the same order as the artifacts these prints are read by — a
    // project profile with fuzzy skin on would silently invalidate the
    // calibration.
    s.fuzzy_skin = false;
    // The cal tower is a lone synthetic mesh at the origin. The GUI runs
    // with auto-center OFF (it places objects itself), so without this the
    // tower prints in the front-left corner and its skirt spills off the bed
    // edge — the printer rejects it as a move out of range. Force centering.
    s.auto_center_on_bed = true;
    // The seam is a pressure transient of its own — it must never share the
    // corner being judged. `Sharpest` would chase the apex (the only real
    // corner on a teardrop), so seed at the rear of the arc instead and hold
    // that column.
    s.seam_mode = config::SeamMode::Aligned;
    // Above the base plate the wall prints as one continuous helix: no
    // layer-change stop, no retraction, no seam at all — the corner becomes
    // the loop's ONLY transient. (The per-layer sweep command below still
    // fires at the revolution boundary on the rear column, and Klipper
    // flushes its queue on SET_PRESSURE_ADVANCE, so a faint blip column can
    // remain there — far from the corner, and much gentler than a true
    // seam.)
    s.spiral_vase = true;
    // Calibrate at the speed the profile actually prints — PA calibrated
    // slow reads high (the smoothing window dominates the shorter corner
    // transients; a throttled run once read 0.070 for a true ~0.032), and
    // steady-state flow shifts with real back-pressure too. So no layer-time
    // floor, at any profile speed. (The ~201 mm perimeter keeps layers ≥1 s
    // up to ~200 mm/s anyway, so cooling only thins out where the user
    // genuinely prints faster than that.)
    s.min_layer_time_s = 0.0;
    // Speed alone wasn't enough: pressure advance acts through Klipper's
    // smoothing window (~40 ms), and at full wall accel the corner decel
    // finishes inside it (160→5 mm/s at 6000 = ~26 ms). The compensation
    // smears wider than the event, a residual bulge survives at the TRUE
    // coefficient, and nulling it visually demands more advance — the same
    // clipping that made the throttled run read 0.070 read ~0.044 at full
    // accel for a true ~0.032 (the value the reference prints run, and the
    // one real prints look best at). Cap the tower's accel so the decel
    // spans several windows: the coefficient is then measured in the regime
    // where the linear model fully acts — which is where the emitted g-code
    // lives anyway (feed steps are slew-limited over 40-60 ms+, seams enter
    // through join/scarf ramps). Real prints' own hard corners share the
    // machine accel, but under-correcting a 26 ms residual beats starving
    // every seam and band with a 35% over-read.
    const TOWER_ACCEL_CAP: f64 = 1200.0;
    s.acceleration_mm_s2 = s.acceleration_mm_s2.min(TOWER_ACCEL_CAP);
    s.outer_wall_accel_mm_s2 = s.outer_wall_accel_mm_s2.min(TOWER_ACCEL_CAP);
    // Pin the part fan to the filament's BASE duty: the tower's ~2 s layers
    // would ride the short-layer cooling ladder to the ceiling (80% for
    // ABS/ASA), but most of a real print's flow mass lands on large layers
    // at the base duty — and the flow reading that cross-checked against
    // the reference print to 0.25% was taken at base fan. A colder bead
    // spreads less and calipers narrower, which would bias the flow read
    // high. Setting the ceiling equal to the base leaves the ladder no
    // headroom without touching the duty the user actually prints with.
    s.bridge_fan_speed = s.fan_speed;
    for t in &mut s.tools {
        t.bridge_fan_speed = t.fan_speed;
    }
    s
}

/// Slice the teardrop tower with the given (already tower-stripped) settings,
/// printing with `tool` — on a toolchanger the tower must run on the spool
/// whose profile the reading will be pinned into, not whatever tool 0 holds.
fn teardrop_gcode(s: &Settings, tool: u32) -> String {
    // Teardrop footprint, apex toward the front (-Y) so the corner faces the
    // user: circle of radius r at the origin, apex at (0, -r√2) — the two
    // tangents from that point meet at exactly 90°, touching the circle at
    // -45° and 225°. 1.5° facets keep the chord error (~0.003 mm) under the
    // slicer's contour resolution, and land a vertex exactly at the rear
    // (90°) for the seam column to seed on.
    let r = TOWER_R_MM;
    let mut fp = vec![[0.0, -r * std::f64::consts::SQRT_2]];
    let steps = 180;
    for i in 0..=steps {
        let a = (-45.0 + 270.0 * f64::from(i) / f64::from(steps)).to_radians();
        fp.push([r * a.cos(), r * a.sin()]);
    }
    let mut tris = Vec::new();
    prism(&fp, 0.0, TOWER_H_MM, &mut tris);
    let mesh = mesh::Mesh::from_triangle_soup(&tris);
    to_gcode(&generate_parts(&[(&mesh, tool)], s), s)
}

/// Bake a per-layer sweep into tower g-code: `header` first, the injected
/// command for each layer's height right after its marker, and `tail` at the
/// very end (restoring machine state the sweep dirtied — a sweep must not
/// leak its top value into the next print).
fn sweep(g: &str, header: &str, mut per_layer: impl FnMut(f64) -> String, tail: &str) -> String {
    let mut out = String::with_capacity(g.len() + 16384);
    out.push_str(header);
    for line in g.lines() {
        out.push_str(line);
        out.push('\n');
        if let Some(rest) = line.strip_prefix("; LAYER ") {
            if let Some(z) = rest.split("z=").nth(1).and_then(|v| v.trim().parse::<f64>().ok()) {
                out.push_str(&per_layer(z));
            }
        }
    }
    out.push_str(tail);
    out
}

/// G-code for the pressure-advance tower: the teardrop helix with PA ramping
/// with height — Klipper's `TUNING_TOWER` sweep, but baked into the file per
/// layer, so there is no console incantation to run. The head enters the one
/// 90° corner off a long smooth run at fully settled pressure and exits onto
/// a straight leg where the artifact reads clean; the revolution boundary
/// (where each layer's PA step lands) holds a rear column on the arc, as far
/// from the corner as the loop allows. Too little PA bulges the corner
/// (pressure overshoots the slowdown), too much starves the stretch right
/// after it — the crispest band wins, and [`pa_from_height`] turns its
/// height (measured from the bed) into the profile value. Flow is constant
/// here, so the flat legs double as a caliper check of the pinned flow.
pub fn pa_tower_gcode(settings: &Settings, tool: u32) -> String {
    let s = tower_settings(settings);
    let mut header = format!(
        "; PA tower: pressure advance = {PA_TOWER_START} + {PA_TOWER_FACTOR} * z_mm\n\
         ; teardrop with one 90° corner at the front, printed as a seamless helix\n\
         ; (vase mode) above the base — judge ONLY that corner.\n\
         ; find the LOWEST height where the corner bulge is gone and read THERE,\n\
         ; measured from the BED. Higher layers often look even 'sharper' — that\n\
         ; sharpness is over-advance already starving the stretch after the corner\n\
         ; (matte, thin), and every seam and band of a real print pays for it.\n\
         ; When torn between two heights, pick the LOWER.\n\
         ; apply it in the Filament panel (or PA = {PA_TOWER_START} + {PA_TOWER_FACTOR} * height by hand).\n"
    );
    // Leave the machine on the profile's PA, not the sweep's top. (A profile
    // value of 0 means "the printer's own config value", which g-code cannot
    // read back — no restore is possible, so say so; applying the measured
    // height re-pins PA on every later print.)
    let tail = if settings.pressure_advance > 0.0 {
        format!("SET_PRESSURE_ADVANCE ADVANCE={:.4}\n", settings.pressure_advance)
    } else {
        header.push_str(
            "; NOTE: no profile PA is pinned, so after this print the machine keeps the\n\
             ; sweep's top value until you apply a height or restart the firmware.\n",
        );
        String::new()
    };
    sweep(
        &teardrop_gcode(&s, tool),
        &header,
        |z| format!("SET_PRESSURE_ADVANCE ADVANCE={:.4}\n", PA_TOWER_START + PA_TOWER_FACTOR * z),
        &tail,
    )
}

/// G-code for the flow comb: a spine bar with [`COMB_TEETH`] hollow
/// rectangular teeth, each a single-bead ring printed at its own flow
/// multiplier — [`COMB_FLOW_FAT`] at the handle end stepping evenly to
/// [`COMB_FLOW_THIN`] at the far end — while PA holds the profile value.
/// The multiplier is baked into E through the per-segment attribute
/// channel, so there is no M221 anywhere and nothing to restore. Caliper
/// the two END teeth mid-face with the full jaw flats (their outer faces
/// are completely open) and feed both thicknesses to
/// [`flow_from_comb_teeth`]; the middle teeth must step evenly — a tooth
/// off that line is a bad measurement telling on itself. Flow changes ride
/// the travel between teeth, so every tooth face is pure steady state.
pub fn flow_comb_gcode(settings: &Settings, tool: u32) -> String {
    let mut s = tower_settings(settings);
    // Ordinary closed loops, not a helix: the per-tooth flow rides the
    // per-segment attribute channel, which the vase spiral bypasses. The
    // seam lands on silhouette vertices (corners), never mid-face — the
    // measurement zones stay clean.
    s.spiral_vase = false;
    // The fat tooth extrudes 12% over nominal — derate the melt ceiling so
    // it stays within the filament's real melt rate (a cap-riding profile
    // would starve it, thinning the wall and biasing the solve high).
    s.max_volumetric_speed_mm3_s /= COMB_FLOW_FAT;
    for t in &mut s.tools {
        t.max_volumetric_speed_mm3_s /= COMB_FLOW_FAT;
    }
    let w = COMB_HANDLE_MM + (COMB_TEETH - 1) as f64 * COMB_TOOTH_PITCH_MM + COMB_TOOTH_W_MM;
    let d = COMB_SPINE_D_MM + COMB_TOOTH_LEN_MM;
    let rect = |x0: f64, y0: f64, x1: f64, y1: f64| vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
    let mut tris = Vec::new();
    prism(&rect(0.0, 0.0, w, COMB_SPINE_D_MM), 0.0, COMB_H_MM, &mut tris);
    for k in 0..COMB_TEETH {
        let x0 = COMB_HANDLE_MM + k as f64 * COMB_TOOTH_PITCH_MM;
        // Overlap one mm into the spine so the prisms union into one comb.
        prism(
            &rect(x0, COMB_SPINE_D_MM - 1.0, x0 + COMB_TOOTH_W_MM, d),
            0.0,
            COMB_H_MM,
            &mut tris,
        );
    }
    let mesh = mesh::Mesh::from_triangle_soup(&tris);
    let mut plans = generate_parts(&[(&mesh, tool)], &s);
    // The plans are bed-centered (auto_center_on_bed); map back to comb
    // coordinates to decide which tooth a wall segment belongs to.
    let ox = s.bed_size_x_mm / 2.0 - w / 2.0;
    let oy = s.bed_size_y_mm / 2.0 - d / 2.0;
    for plan in &mut plans {
        for p in &mut plan.paths {
            if !matches!(
                p.kind,
                crate::plan::PathKind::ExternalPerimeter | crate::plan::PathKind::Perimeter
            ) || p.segs.is_some()
                || p.widths.is_some()
                || p.points.len() < 2
            {
                continue;
            }
            let n = p.points.len();
            let n_segs = if p.closed { n } else { n - 1 };
            let attrs: Vec<crate::plan::SegAttr> = (0..n_segs)
                .map(|k| {
                    let a = p.points[k];
                    let b = p.points[(k + 1) % n];
                    let mx = (a.x_mm() + b.x_mm()) / 2.0 - ox;
                    let my = (a.y_mm() + b.y_mm()) / 2.0 - oy;
                    crate::plan::SegAttr {
                        kind: p.kind,
                        overhang: 0.0,
                        flow: comb_flow_at(mx, my) as f32,
                    }
                })
                .collect();
            // Only carry the channel where a tooth actually changes flow —
            // spine-only loops (and the base plate) stay plain paths.
            if attrs.iter().any(|a| (f64::from(a.flow) - 1.0).abs() > 1.0e-3) {
                p.segs = Some(attrs);
            }
        }
    }
    let header = format!(
        "; flow comb: {COMB_TEETH} teeth, each a single-wall ring at its own flow x profile flow —\n\
         ; {COMB_FLOW_FAT} at the HANDLE end stepping evenly to {COMB_FLOW_THIN} at the far end. no M221:\n\
         ; the multipliers are baked into E, and PA stays at the profile value.\n\
         ; caliper the two END teeth mid-face with the FULL jaw flats (their outer faces\n\
         ; are open; each tooth is uniform, so position on the tooth does not matter),\n\
         ; then enter both thicknesses in the Filament panel — it solves for the flow\n\
         ; where the wall would read exactly {:.2} mm (the line width). the middle teeth\n\
         ; must step evenly between them: a tooth off the line is a bad measurement.\n",
        s.line_width_mm
    );
    format!("{header}{}", to_gcode(&plans, &s))
}

/// Flow multiplier for a point of the comb (comb coordinates): on a tooth,
/// that tooth's step of the fat→thin ladder; on the spine, the junction
/// band, or anywhere unexpected, true flow.
fn comb_flow_at(x: f64, y: f64) -> f64 {
    if y <= COMB_SPINE_D_MM + 1.5 {
        return 1.0;
    }
    for k in 0..COMB_TEETH {
        let x0 = COMB_HANDLE_MM + k as f64 * COMB_TOOTH_PITCH_MM;
        if (x0 - 1.0..=x0 + COMB_TOOTH_W_MM + 1.0).contains(&x) {
            return comb_tooth_flow(k);
        }
    }
    1.0
}

/// The flow multiplier of tooth `k` (0 = the fat tooth at the handle end).
pub fn comb_tooth_flow(k: usize) -> f64 {
    COMB_FLOW_FAT + (COMB_FLOW_THIN - COMB_FLOW_FAT) * k as f64 / (COMB_TEETH - 1) as f64
}

/// Pressure advance from the measured best-corner height on the PA tower.
/// A height off the tower leaves the current value untouched.
pub fn pa_from_height(current_pa: f64, height_mm: f64) -> f64 {
    if height_mm <= 0.0 || height_mm > TOWER_H_MM {
        return current_pa;
    }
    PA_TOWER_START + PA_TOWER_FACTOR * height_mm
}

/// New flow multiplier from the comb's two END tooth thicknesses (mm): the
/// fat (handle-end) and thin (far-end) teeth, each measured mid-face with
/// the jaws' full flats. The two points pin the real system's
/// thickness-vs-flow line; solving it for the line-width crossing locates
/// the multiplier that makes the wall exactly right — no bead model in the
/// loop, and extrusion is volumetric, so the line really is a line.
/// Multiplicative on the current value — the teeth rode on top of it.
/// Readings that can't be a real comb (thin >= fat, non-positive, or a
/// solve outside a plausible spool) leave the value untouched.
pub fn flow_from_comb_teeth(
    current_flow: f64,
    line_width_mm: f64,
    fat_mm: f64,
    thin_mm: f64,
) -> f64 {
    if thin_mm <= 0.0 || fat_mm <= thin_mm {
        return current_flow;
    }
    let slope = (COMB_FLOW_FAT - COMB_FLOW_THIN) / (fat_mm - thin_mm);
    let m = COMB_FLOW_THIN + (line_width_mm - thin_mm) * slope;
    if !(0.5..=1.5).contains(&m) {
        return current_flow;
    }
    current_flow * m
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

    /// Per-layer outer-wall data parsed from tower g-code:
    /// (z, seam = position where the layer's first outer-wall extrusion
    /// starts, all outer-wall extrusion endpoints — with G2/G3 arcs sampled
    /// along their sweep, so an arc's extremes count, not just its ends).
    fn outer_walls(g: &str) -> Vec<(f64, [f64; 2], Vec<[f64; 2]>)> {
        let mut layers: Vec<(f64, Option<[f64; 2]>, Vec<[f64; 2]>)> = Vec::new();
        let (mut x, mut y, mut wall) = (0.0f64, 0.0f64, false);
        let num = |l: &str, k: &str| {
            l.split(k).nth(1).and_then(|s| s.split(' ').next().unwrap_or("").parse::<f64>().ok())
        };
        for l in g.lines() {
            if let Some(rest) = l.strip_prefix("; LAYER ") {
                let z = rest.split("z=").nth(1).and_then(|v| v.trim().parse().ok()).unwrap_or(0.0);
                layers.push((z, None, Vec::new()));
                wall = false;
                continue;
            }
            if let Some(t) = l.strip_prefix(";TYPE:") {
                wall = t.trim() == "Outer wall";
                continue;
            }
            let arc_cw = l.starts_with("G2 ");
            if !(l.starts_with("G0 ") || l.starts_with("G1 ") || arc_cw || l.starts_with("G3 ")) {
                continue;
            }
            let (nx, ny) = (num(l, " X"), num(l, " Y"));
            if wall && l.contains(" E") && !l.contains(" E-") && (nx.is_some() || ny.is_some()) {
                let (ex, ey) = (nx.unwrap_or(x), ny.unwrap_or(y));
                if let Some((_, seam, pts)) = layers.last_mut() {
                    seam.get_or_insert([x, y]);
                    if arc_cw || l.starts_with("G3 ") {
                        // Sample the sweep every ~5° from start to end.
                        let (cx, cy) = (x + num(l, " I").unwrap_or(0.0), y + num(l, " J").unwrap_or(0.0));
                        let r = ((x - cx).powi(2) + (y - cy).powi(2)).sqrt();
                        let a0 = (y - cy).atan2(x - cx);
                        let a1 = (ey - cy).atan2(ex - cx);
                        let tau = std::f64::consts::TAU;
                        let sweep = if arc_cw {
                            -(a0 - a1).rem_euclid(tau)
                        } else {
                            (a1 - a0).rem_euclid(tau)
                        };
                        let n = (sweep.abs().to_degrees() / 5.0).ceil().max(1.0) as usize;
                        for k in 1..=n {
                            let a = a0 + sweep * k as f64 / n as f64;
                            pts.push([cx + r * a.cos(), cy + r * a.sin()]);
                        }
                    }
                    pts.push([ex, ey]);
                }
            }
            if let Some(v) = nx { x = v; }
            if let Some(v) = ny { y = v; }
        }
        layers.into_iter().filter_map(|(z, s, p)| s.map(|s| (z, s, p))).collect()
    }

    #[test]
    fn towers_center_on_bed_even_with_auto_center_off() {
        // The GUI positions objects itself, so it runs with auto_center_on_bed
        // = false. The tower is a lone synthetic mesh, so it must re-enable
        // centering or it prints off the front-left corner and the skirt runs
        // off the bed (negative coords → the printer's "move out of range").
        let mut s = Settings::default();
        s.auto_center_on_bed = false;
        s.bed_size_x_mm = 152.4;
        s.bed_size_y_mm = 152.4;
        for g in [pa_tower_gcode(&s, 0), flow_comb_gcode(&s, 0)] {
            assert!(!g.contains(" X-"), "no off-bed negative X moves");
            assert!(!g.contains(" Y-"), "no off-bed negative Y moves");
        }
    }

    #[test]
    fn pa_tower_ramps_with_height() {
        let g = pa_tower_gcode(&Settings::default(), 0);
        // One injected PA step directly after each layer marker.
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
            (PA_TOWER_START + PA_TOWER_FACTOR * TOWER_H_MM - top).abs() < 0.003,
            "ends at the top of the range, got {top}"
        );
        // Single wall: no interior features to muddy the corner. And no flow
        // games — the PA read must ride the profile's true flow (the only
        // M221 allowed is the preamble's neutral S100 hygiene guard).
        assert!(!g.contains(";TYPE:Sparse infill") && !g.contains(";TYPE:Top surface"));
        assert!(
            g.lines().filter(|l| l.starts_with("M221")).all(|l| l.trim() == "M221 S100"),
            "PA tower holds flow constant"
        );
        // The instrument property that keeps the read TRUE: every wall accel
        // stays under the tower cap, so the corner decel spans several PA
        // smoothing windows (full machine accel clips the transient inside
        // the window and the tower reads ~35% high — measured against the
        // reference prints' known-good value).
        let max_m204 = g
            .lines()
            .filter_map(|l| l.strip_prefix("M204 S"))
            .filter_map(|v| v.trim().parse::<f64>().ok())
            .fold(0.0_f64, f64::max);
        assert!(
            max_m204 <= 1200.0,
            "tower accel capped past the PA smoothing window, got M204 S{max_m204}"
        );
    }

    #[test]
    fn towers_hold_the_base_fan_duty() {
        // The tower's ~2 s layers would ride the short-layer cooling ladder
        // to the ceiling (80% for ABS/ASA) — a regime unlike the large
        // layers most of a print's flow mass lands on, and a colder bead
        // calipers narrower (flow read biased high). Both towers must print
        // at the filament's BASE duty.
        let mut s = Settings::default();
        s.fan_speed = 0.15;
        s.bridge_fan_speed = 0.8;
        s.fan_off_layers = 3;
        for g in [pa_tower_gcode(&s, 0), flow_comb_gcode(&s, 0)] {
            let max_fan = g
                .lines()
                .filter(|l| l.starts_with("M106 S"))
                .filter_map(|l| l[6..].trim().parse::<u32>().ok())
                .max()
                .unwrap_or(0);
            assert!(
                max_fan <= 39,
                "tower fan must stay at the base duty (S38), got S{max_fan}"
            );
        }
    }

    #[test]
    fn pa_tower_restores_the_profile_pa() {
        // The sweep ends at 0.10 — silly-high. The machine must be left on
        // the profile's value, not the sweep's top, or the next print (whose
        // profile might say 0 = "leave the printer's value") inherits it.
        let mut s = Settings::default();
        s.pressure_advance = 0.032;
        let g = pa_tower_gcode(&s, 0);
        let last = g
            .lines()
            .rev()
            .find_map(|l| l.strip_prefix("SET_PRESSURE_ADVANCE ADVANCE="))
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap();
        assert!((last - 0.032).abs() < 1e-9, "restores profile PA, got {last}");
        // With profile PA = 0 ("the printer's own config value") no restore
        // is POSSIBLE — g-code can't read printer.cfg back. The file must
        // own that honestly: the sweep's top stays in force, and the header
        // says so.
        let g0 = pa_tower_gcode(&Settings::default(), 0);
        let last0 = g0
            .lines()
            .rev()
            .find_map(|l| l.strip_prefix("SET_PRESSURE_ADVANCE ADVANCE="))
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap();
        assert!(last0 > 0.09, "no phantom restore value exists for PA=0 profiles");
        assert!(g0.contains("; NOTE: no profile PA is pinned"), "the leak is documented");
    }

    #[test]
    fn flow_comb_is_guarded_against_a_dirty_machine() {
        // The comb bakes its multipliers into E, so a stale firmware M221
        // (say 85% left by an old canceled sweep) would silently scale every
        // tooth AND the spine — the solve would then pin a flow that's only
        // right under that stale scale. The preamble's M221 S100 hygiene
        // guard must land BEFORE anything prints.
        let g = flow_comb_gcode(&Settings::default(), 0);
        let first_m221 = g.find("M221 S").unwrap();
        assert!(g[first_m221..].starts_with("M221 S100"), "file opens at neutral flow");
        assert!(first_m221 < g.find("; LAYER").unwrap(), "normalized before any layer");
    }

    #[test]
    fn towers_print_with_the_requested_tool() {
        // On a toolchanger the reading is pinned into the ACTIVE slot's
        // filament profile, so the tower must physically print with that
        // tool — not whatever spool tool 0 holds.
        // (The generic start template carries no T{n} line — real toolchanger
        // templates take INITIAL_TOOL= — so assert the SLOT's settings landed:
        // the preamble's PA pin must be tool 1's, not tool 0's.)
        let mut s = Settings::default();
        s.machine_kind = config::MachineKind::IndependentHotends;
        s.tool_count = 2;
        s.tools = vec![s.flat_tool("a".into()), s.flat_tool("b".into())];
        s.tools[0].pressure_advance = 0.033;
        s.tools[1].pressure_advance = 0.077;
        let g = flow_comb_gcode(&s, 1);
        assert!(g.contains("SET_PRESSURE_ADVANCE ADVANCE=0.0770"), "tower prints with tool 1's profile");
        assert!(!g.contains("SET_PRESSURE_ADVANCE ADVANCE=0.0330"), "not tool 0's");
    }

    #[test]
    fn flow_comb_bakes_per_tooth_flow() {
        let s = Settings::default();
        let g = flow_comb_gcode(&s, 0);
        // No M221 anywhere beyond the preamble hygiene guard — the
        // multipliers are baked into E.
        assert!(
            g.lines().filter(|l| l.starts_with("M221")).all(|l| l.trim() == "M221 S100"),
            "comb must not sweep firmware flow"
        );
        // Measure E-per-mm of wall extrusions bucketed by tooth, on layers
        // above the base plate, away from corners: the fat and thin end
        // teeth must carry their multipliers relative to the spine.
        let w = COMB_HANDLE_MM + (COMB_TEETH - 1) as f64 * COMB_TOOTH_PITCH_MM + COMB_TOOTH_W_MM;
        let d = COMB_SPINE_D_MM + COMB_TOOTH_LEN_MM;
        let (ox, oy) = (s.bed_size_x_mm / 2.0 - w / 2.0, s.bed_size_y_mm / 2.0 - d / 2.0);
        let tooth_x = |k: usize| COMB_HANDLE_MM + k as f64 * COMB_TOOTH_PITCH_MM;
        let (mut x, mut y) = (0.0_f64, 0.0_f64);
        let (mut layer, mut wall) = (0usize, false);
        // (sum E, sum len) per bucket: fat tooth, thin tooth, spine.
        let mut buckets = [(0.0_f64, 0.0_f64); 3];
        for l in g.lines() {
            if l.starts_with("; LAYER ") {
                layer += 1;
                continue;
            }
            if let Some(t) = l.strip_prefix(";TYPE:") {
                wall = t.trim() == "Outer wall";
                continue;
            }
            if !(l.starts_with("G1 ") || l.starts_with("G0 ")) {
                continue;
            }
            let mut nx = x;
            let mut ny = y;
            let mut e = 0.0_f64;
            for tok in l.split_whitespace() {
                if let Some(v) = tok.strip_prefix('X').and_then(|v| v.parse::<f64>().ok()) {
                    nx = v;
                }
                if let Some(v) = tok.strip_prefix('Y').and_then(|v| v.parse::<f64>().ok()) {
                    ny = v;
                }
                if let Some(v) = tok.strip_prefix('E').and_then(|v| v.parse::<f64>().ok()) {
                    e = v;
                }
            }
            let len = ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
            let (mx, my) = ((x + nx) / 2.0 - ox, (y + ny) / 2.0 - oy);
            x = nx;
            y = ny;
            // Above the base plate, real extrusions, long straight strokes
            // only (corner pieces carry concave trims).
            if layer <= 4 || !wall || e <= 1.0e-6 || len < 3.0 {
                continue;
            }
            let mid_tooth = |k: usize| {
                let x0 = tooth_x(k);
                (x0 - 0.5..=x0 + COMB_TOOTH_W_MM + 0.5).contains(&mx)
                    && my > COMB_SPINE_D_MM + 4.0
                    && my < d - 2.0
            };
            let b = if mid_tooth(0) {
                0
            } else if mid_tooth(COMB_TEETH - 1) {
                1
            } else if my < COMB_SPINE_D_MM {
                2
            } else {
                continue;
            };
            buckets[b].0 += e;
            buckets[b].1 += len;
        }
        let epmm = |b: (f64, f64)| {
            assert!(b.1 > 20.0, "bucket has enough wall to judge, got {} mm", b.1);
            b.0 / b.1
        };
        let (fat, thin, spine) = (epmm(buckets[0]), epmm(buckets[1]), epmm(buckets[2]));
        assert!(
            (fat / spine - COMB_FLOW_FAT).abs() < 0.02,
            "fat tooth at {COMB_FLOW_FAT}x, got {:.3}",
            fat / spine
        );
        assert!(
            (thin / spine - COMB_FLOW_THIN).abs() < 0.02,
            "thin tooth at {COMB_FLOW_THIN}x, got {:.3}",
            thin / spine
        );
    }

    /// Most-frequent feedrate on outer-wall extrusions (mm/min, rounded).
    fn dominant_wall_feed(g: &str) -> i64 {
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
        feed_mm.iter().max_by_key(|(_, n)| **n).map(|(f, _)| *f).unwrap_or(0)
    }

    #[test]
    fn pa_tower_prints_at_the_real_outer_wall_speed() {
        // The whole point of the tower is calibrating at the speed the profile
        // actually prints. The flow cube's relaxed layer-time floor used to
        // throttle it ~2.7× — and PA calibrated slow reads high (the user's
        // 0.070 vs a true ~0.032). Dominant extrusion feed must be the plain
        // outer-wall speed, unthrottled.
        let s = Settings::default();
        let dominant = dominant_wall_feed(&pa_tower_gcode(&s, 0));
        let expect = s.external_perimeter_speed_mm_s * 60.0;
        assert!(
            (dominant as f64) >= expect * 0.9,
            "tower must run at outer-wall speed: dominant F{dominant} vs expected F{expect:.0}"
        );
    }

    #[test]
    fn pa_tower_is_never_throttled_even_on_a_fast_profile() {
        // 300 mm/s of outer wall prints the ~200 mm loop in ~0.67 s — any
        // layer-time floor would throttle that below profile speed, silently
        // reintroducing the calibrated-slow-reads-high failure. The tower
        // disables the governor outright, so even this profile runs true.
        let mut s = Settings::default();
        s.external_perimeter_speed_mm_s = 300.0;
        // Out of the way of the *flow* ceiling (real physics, kept): 300 mm/s
        // at the default bead is ~26 mm³/s.
        s.max_volumetric_speed_mm3_s = 40.0;
        let dominant = dominant_wall_feed(&pa_tower_gcode(&s, 0));
        let expect = s.external_perimeter_speed_mm_s * 60.0;
        assert!(
            (dominant as f64) >= expect * 0.9,
            "fast profile must not be throttled: dominant F{dominant} vs expected F{expect:.0}"
        );
    }

    #[test]
    fn pa_tower_is_one_clean_teardrop_prism() {
        // A 270° arc closed by two tangent legs meeting at the front apex:
        // the wall-centerline box is ~2r wide by ~r(1+√2) deep. And it is a
        // PRISM — the old index notches are gone (a mark in the wall jogs
        // the very bead being judged, and they destabilized the tower), so
        // every layer spans the same box.
        let g = pa_tower_gcode(&Settings::default(), 0);
        let layers = outer_walls(&g);
        assert!(layers.len() > 200, "a 50 mm tower is ~250 layers, got {}", layers.len());
        let span = |pts: &[[f64; 2]], i: usize| {
            pts.iter().map(|p| p[i]).fold(f64::MIN, f64::max)
                - pts.iter().map(|p| p[i]).fold(f64::MAX, f64::min)
        };
        let (w, d) = (2.0 * TOWER_R_MM, TOWER_R_MM * (1.0 + std::f64::consts::SQRT_2));
        for (z, _, pts) in &layers {
            let (sx, sy) = (span(pts, 0), span(pts, 1));
            assert!((sx - w).abs() < 1.5, "layer z={z}: width {sx:.2} vs {w:.2}");
            assert!((sy - d).abs() < 1.5, "layer z={z}: depth {sy:.2} vs {d:.2}");
        }
    }

    #[test]
    fn pa_tower_is_a_seamless_helix_above_the_base() {
        // Vase mode: above the 3-layer base plate the wall must climb as one
        // continuous extrusion — no retraction, no layer-change stop — so the
        // corner is the loop's only pressure transient. (One trailing retract
        // at print end is fine.)
        let g = pa_tower_gcode(&Settings::default(), 0);
        let spiral = g.find("; LAYER 3 ").expect("tower has spiral layers");
        let retracts = g[spiral..].matches(" E-").count();
        assert!(retracts <= 1, "helix must not retract, saw {retracts}");
    }

    #[test]
    fn pa_tower_seam_never_touches_the_corner() {
        // The loop start — the base plate's true seam, and above it the helix
        // revolution boundary where each layer's PA step (a Klipper queue
        // flush) lands — is a transient of its own, and the teardrop's whole
        // point is ONE uncontaminated corner: it must hold a column on the
        // rounded back — beyond even the tangent points (30 mm from the
        // apex) — on every layer.
        let g = pa_tower_gcode(&Settings::default(), 0);
        let layers = outer_walls(&g);
        let apex = layers
            .iter()
            .flat_map(|(_, _, pts)| pts.iter())
            .fold([0.0, f64::MAX], |a, p| if p[1] < a[1] { *p } else { a });
        for (z, seam, _) in &layers {
            let d = ((seam[0] - apex[0]).powi(2) + (seam[1] - apex[1]).powi(2)).sqrt();
            assert!(d > 40.0, "layer z={z}: seam {seam:?} only {d:.1} mm from the corner {apex:?}");
        }
        // And it is a column, not a wander.
        let n = layers.len() as f64;
        let cx = layers.iter().map(|(_, s, _)| s[0]).sum::<f64>() / n;
        let cy = layers.iter().map(|(_, s, _)| s[1]).sum::<f64>() / n;
        for (z, seam, _) in &layers {
            let d = ((seam[0] - cx).powi(2) + (seam[1] - cy).powi(2)).sqrt();
            assert!(d < 5.0, "layer z={z}: seam {seam:?} strays {d:.1} mm from the column");
        }
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
    fn flow_from_comb_teeth_solves_and_guards() {
        // Synthetic linear wall: thickness = 0.357*m + 0.043 (a stadium-ish
        // response). The true multiplier for a 0.40 wall is
        // (0.40 - 0.043)/0.357 = 1.0 — the solve must recover it from the
        // two end-tooth readings alone.
        let (fat, thin) = (0.357 * COMB_FLOW_FAT + 0.043, 0.357 * COMB_FLOW_THIN + 0.043);
        assert!((flow_from_comb_teeth(1.0, 0.40, fat, thin) - 1.0).abs() < 1e-9);
        // Compounds multiplicatively on an already-pinned value: the teeth
        // rode on top of the current flow, so the solve multiplies in.
        let m =
            COMB_FLOW_THIN + (0.40 - thin) * (COMB_FLOW_FAT - COMB_FLOW_THIN) / (fat - thin);
        assert!((flow_from_comb_teeth(0.95, 0.40, fat, thin) - 0.95 * m).abs() < 1e-9);
        // An over-extruding spool reads fat: both teeth thicker, solve < 1.
        assert!(flow_from_comb_teeth(1.0, 0.40, fat + 0.04, thin + 0.04) < 1.0);
        // Nonsense readings are a no-op: inverted teeth, zero, or a solve
        // outside any plausible spool.
        assert_eq!(flow_from_comb_teeth(0.95, 0.40, 0.34, 0.45), 0.95);
        assert_eq!(flow_from_comb_teeth(0.95, 0.40, 0.45, 0.0), 0.95);
        assert_eq!(flow_from_comb_teeth(0.95, 0.40, 0.401, 0.399), 0.95);
        // The tooth ladder steps evenly from fat to thin.
        assert!((comb_tooth_flow(0) - COMB_FLOW_FAT).abs() < 1e-9);
        assert!((comb_tooth_flow(COMB_TEETH - 1) - COMB_FLOW_THIN).abs() < 1e-9);
        assert!((comb_tooth_flow(3) - 1.0).abs() < 1e-9);
    }
}
