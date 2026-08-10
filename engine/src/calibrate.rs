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
/// Wall-speed fraction inside the PA tower's apex slow zone. The judged
/// artifact is the pair of SPEED-STEP bands where the wall crosses the
/// zone boundary mid-leg: 0.4x keeps the step large enough to read while
/// the slow side still moves fast enough (~60+ mm/s on a fast profile)
/// that dwell ooze — which pressure advance cannot cancel, and which made
/// every corner-judged read sit high — never enters the measurement.
pub const PA_STEP_SLOW_FRAC: f64 = 0.4;
/// Pressure advance added per mm of tower height.
pub const PA_TOWER_FACTOR: f64 = 0.002;
/// Flow-comb FAT tooth value — the ladder's top, an ABSOLUTE extrusion
/// multiplier. 25 radial teeth step in EXACT 1% increments down to
/// [`COMB_FLOW_THIN`], so the middle tooth is exactly 1.00 and every
/// label is a round percent. The caliper can't tell 1% neighbours apart —
/// that's the point: find the RUN of teeth that read right and take its
/// middle (tie toward 100); the run structure itself is the interpolation.
pub const COMB_FLOW_FAT: f64 = 1.12;
/// Flow-comb THIN tooth value — the ladder's bottom.
pub const COMB_FLOW_THIN: f64 = 0.88;
/// Teeth on the comb, radially around an annular hub. 25 at 10 mm hub
/// pitch = hub radius ~39.8 mm, footprint ~120 mm — a 1%-step ladder that
/// could never fit a small bed as a straight comb. Gaps between teeth
/// WIDEN outward (4 mm at the root, ~9 mm at the tips), so jaw access
/// beats the straight layout everywhere.
pub const COMB_TEETH: usize = 25;
/// Tooth length (mm) past the hub. Long enough that the mid-face
/// measurement zone sits clear of the junction fillets and the tip round.
pub const COMB_TOOTH_LEN_MM: f64 = 20.0;
/// Tooth footprint width (mm): the ring cavity takes a caliper's inner jaw.
pub const COMB_TOOTH_W_MM: f64 = 6.0;
/// Tooth-to-tooth pitch (mm) measured along the hub circle.
pub const COMB_TOOTH_PITCH_MM: f64 = 10.0;
/// Radial width (mm) of the annular hub joining the teeth (true flow).
pub const COMB_HUB_RING_MM: f64 = 4.0;
/// Comb height (mm): tall enough for full caliper-jaw flats on any tooth.
pub const COMB_H_MM: f64 = 12.0;

/// Hub outer radius: the circle the teeth stand on.
pub fn comb_hub_r_mm() -> f64 {
    COMB_TEETH as f64 * COMB_TOOTH_PITCH_MM / (2.0 * std::f64::consts::PI)
}

/// Center angle (radians) of tooth `k`. Tooth 0 (the fat end of the
/// ladder) sits at the rear; labels ascend counterclockwise viewed from
/// the top — which is clockwise from the underside, where they're read.
pub fn comb_tooth_angle(k: usize) -> f64 {
    std::f64::consts::FRAC_PI_2 - k as f64 * 2.0 * std::f64::consts::PI / COMB_TEETH as f64
}

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
    // The corner prints at the machine's REAL cornering regime — the
    // profile's outer-wall accel and square-corner velocity, exactly as
    // production walls corner. An accel cap here (tried at 1200 to out-run
    // Klipper's PA smoothing window) made the reading dramatically WORSE:
    // the measured over-read across three prints tracks TIME SPENT SLOW at
    // the corner (throttled run: read 0.070; full speed at accel 6000:
    // 0.044; full speed at accel 1200: transition near the tower top,
    // ~0.08+ — for a machine whose true value is ~0.032). A slow approach
    // parks the nozzle at the apex while die swell and gravity deposit
    // ooze that pressure advance cannot cancel, so the bulge survives to
    // absurd PA values and the judge reads high. The fastest transient the
    // machine actually uses is the least dwell-contaminated instrument;
    // the residual over-read at full accel is what the lowest-bulge-free
    // reading instruction (not "crispest") corrects for.
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

/// Bake the two-speed pattern into the vase helix: the wall cruises at its
/// file speed, drops to [`PA_STEP_SLOW_FRAC`] inside a circle of radius
/// half the teardrop radius around the apex, and speeds back up on the way
/// out. The circle crosses the wall exactly mid-leg on BOTH flat faces —
/// the two speed-step bands land on readable flat wall, far from the
/// corner and the rear seam column, whichever direction the loop runs.
/// Moves crossing the boundary are split at the exact crossing (X/Y/Z/E
/// interpolated), so E-per-mm is conserved and the commanded step is
/// abrupt — the transient pressure advance must answer for.
fn speed_step_pass(g: &str, bed_x_mm: f64, bed_y_mm: f64) -> String {
    let r = TOWER_R_MM;
    // The tower is bed-centered; the apex sits below the bbox center by
    // half the footprint depth (arc top at +r, apex at -r*sqrt(2)).
    let ax = bed_x_mm / 2.0;
    let ay = bed_y_mm / 2.0 - r * (1.0 + std::f64::consts::SQRT_2) / 2.0;
    let rz = r * 0.5;
    let mut out = String::with_capacity(g.len() * 2);
    let (mut x, mut y, mut z) = (0.0_f64, 0.0_f64, 0.0_f64);
    let mut have_pos = false;
    let mut markers = 0usize;
    let mut f_hi: Option<f64> = None;
    let mut cur_f = f64::NAN;
    for line in g.lines() {
        if line.starts_with("; LAYER ") {
            markers += 1;
        }
        if !(line.starts_with("G0 ") || line.starts_with("G1 ")) {
            out.push_str(line);
            out.push('\n');
            continue;
        }
        let (mut nx, mut ny, mut nz, mut e, mut f) = (x, y, z, None::<f64>, None::<f64>);
        for tok in line.split(';').next().unwrap_or("").split_whitespace().skip(1) {
            let (c, v) = tok.split_at(1);
            if let Ok(v) = v.parse::<f64>() {
                match c {
                    "X" => nx = v,
                    "Y" => ny = v,
                    "Z" => nz = v,
                    "E" => e = Some(v),
                    "F" => f = Some(v),
                    _ => {}
                }
            }
        }
        // Only the helix (above the 3 base-plate layers) gets the pattern.
        let vase = markers > 3;
        let extruding =
            have_pos && e.is_some_and(|e| e > 0.0) && ((nx - x).abs() > 1e-9 || (ny - y).abs() > 1e-9);
        if !(vase && extruding) {
            if let Some(fv) = f {
                cur_f = fv;
            }
            out.push_str(line);
            out.push('\n');
            (x, y, z) = (nx, ny, nz);
            have_pos = true;
            continue;
        }
        let fh = *f_hi.get_or_insert(f.unwrap_or(cur_f));
        let fl = (fh * PA_STEP_SLOW_FRAC).round();
        // Split at the slow-zone boundary crossings along this move.
        let (dx, dy) = (nx - x, ny - y);
        let (px, py) = (x - ax, y - ay);
        let a = dx * dx + dy * dy;
        let b = 2.0 * (px * dx + py * dy);
        let c = px * px + py * py - rz * rz;
        let mut ts = vec![0.0_f64];
        let disc = b * b - 4.0 * a * c;
        if disc > 0.0 && a > 1e-12 {
            let sq = disc.sqrt();
            for t in [(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)] {
                if t > 1e-6 && t < 1.0 - 1e-6 {
                    ts.push(t);
                }
            }
        }
        ts.push(1.0);
        ts.sort_by(|q, w| q.partial_cmp(w).unwrap());
        let e_total = e.unwrap_or(0.0);
        for w in ts.windows(2) {
            let (t0, t1) = (w[0], w[1]);
            if t1 - t0 < 1e-9 {
                continue;
            }
            let tm = (t0 + t1) / 2.0;
            let (mx, my) = (x + dx * tm - ax, y + dy * tm - ay);
            let want = if (mx * mx + my * my) < rz * rz { fl } else { fh };
            let (ex, ey, ez) = (x + dx * t1, y + dy * t1, z + (nz - z) * t1);
            let ee = e_total * (t1 - t0);
            if (want - cur_f).abs() > 0.5 {
                out.push_str(&format!("G1 X{ex:.3} Y{ey:.3} Z{ez:.3} E{ee:.5} F{want:.0}\n"));
                cur_f = want;
            } else {
                out.push_str(&format!("G1 X{ex:.3} Y{ey:.3} Z{ez:.3} E{ee:.5}\n"));
            }
        }
        (x, y, z) = (nx, ny, nz);
    }
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
         ; teardrop helix with a TWO-SPEED pattern: the wall cruises fast, drops to\n\
         ; {:.0}% speed inside a {:.0} mm zone around the front corner, and speeds back\n\
         ; up on the way out. The two SPEED-STEP BANDS land mid-face on the flat legs\n\
         ; — THEY are the judge, not the corner (a corner always drags dwell ooze that\n\
         ; pressure advance cannot cancel; judging it is what read 0.044+ on a 0.032\n\
         ; machine). Too little PA: the slow-down band is a fat ridge and the speed-up\n\
         ; band a starved streak. Too much: they trade places. Read the LOWEST height\n\
         ; where the two bands balance out / vanish, measured from the BED, and apply\n\
         ; it in the Filament panel (or PA = {PA_TOWER_START} + {PA_TOWER_FACTOR} * height by hand).\n",
        PA_STEP_SLOW_FRAC * 100.0,
        TOWER_R_MM * 0.5,
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
    // The sweep steps once per MILLIMETER of height, not per layer: Klipper
    // flushes its motion queue on every SET_PRESSURE_ADVANCE — a stall
    // mid-extrusion at the rear column — and per-layer steps of
    // FACTOR×layer_height are far below the ±0.5 mm reading resolution
    // anyway. One step per mm keeps the height→PA map exact at every band
    // edge while cutting the stall count (and the blip column's exposure to
    // a badly-timed host hiccup) five-fold.
    let mut last_band = f64::NEG_INFINITY;
    sweep(
        &speed_step_pass(&teardrop_gcode(&s, tool), s.bed_size_x_mm, s.bed_size_y_mm),
        &header,
        move |z| {
            let band = z.floor();
            if band > last_band {
                last_band = band;
                format!(
                    "SET_PRESSURE_ADVANCE ADVANCE={:.4}\n",
                    PA_TOWER_START + PA_TOWER_FACTOR * band
                )
            } else {
                String::new()
            }
        },
        &tail,
    )
}

/// Seven-segment masks for digits 0-9 (bits a,b,c,d,e,f,g = 0..6): the
/// tooth numbers recessed into the comb's underside.
const SEVEN_SEG: [u8; 10] = [
    0b011_1111, // 0
    0b000_0110, // 1
    0b101_1011, // 2
    0b100_1111, // 3
    0b110_0110, // 4
    0b110_1101, // 5
    0b111_1101, // 6
    0b000_0111, // 7
    0b111_1111, // 8
    0b110_1111, // 9
];
/// Digit stroke length / width (mm). 0.8 mm slots stay two beads wide —
/// safely above the sub-bead hole healing that erases slice slivers.
const DIGIT_SEG_LEN_MM: f64 = 1.8;
const DIGIT_SEG_W_MM: f64 = 0.8;

/// The lit seven-segment strokes of tooth `k`'s LABEL as quads in centered
/// comb coordinates, recessed through the base plate inside the tooth
/// ring. The label is the tooth's flow as a percent MOD 100 — two digits,
/// leading zero ("96" = 0.96, "04" = 1.04, "00" = 1.00; no real spool is
/// ambiguous between 12% and 112%). Two digits is what fits a 6 mm tooth
/// at the 0.8 mm stroke width the bottom-skin machinery provably respects
/// (its line-end reach jumps narrower slots). Digits stand upright,
/// stacked hub→tip (tens digit nearer the hub), and the whole label is
/// mirrored so it reads from the underside. Shared by generator and tests.
fn comb_label_quads(k: usize) -> Vec<[[f64; 2]; 4]> {
    let (l, w) = (1.5_f64, 0.8_f64);
    let dw = l + 2.0 * w;
    let dh = 2.0 * l + 3.0 * w;
    let local: [(f64, f64, f64, f64); 7] = [
        (w, 2.0 * l + 2.0 * w, w + l, dh),
        (w + l, l + 2.0 * w, dw, 2.0 * l + 2.0 * w),
        (w + l, w, dw, w + l),
        (w, 0.0, w + l, w),
        (0.0, w, w, w + l),
        (0.0, l + 2.0 * w, w, 2.0 * l + 2.0 * w),
        (w, l + w, w + l, l + 2.0 * w),
    ];
    let pct = (comb_tooth_flow(k) * 100.0).round() as usize % 100;
    let chars = [pct / 10, pct % 10];
    let theta = comb_tooth_angle(k) - std::f64::consts::FRAC_PI_2;
    let (rc, rs) = (theta.cos(), theta.sin());
    let base_y = comb_hub_r_mm() + 2.0;
    let mut out = Vec::new();
    for (ci, &d) in chars.iter().enumerate() {
        for (si, r) in local.iter().enumerate() {
            if SEVEN_SEG[d] & (1 << si) == 0 {
                continue;
            }
            let corners = [(r.0, r.1), (r.2, r.1), (r.2, r.3), (r.0, r.3)];
            let mut quad = [[0.0_f64; 2]; 4];
            for (qi, &(cx_, cy_)) in corners.iter().enumerate() {
                // Char-local, centered across the tooth, mirrored in x for
                // underside reading, stacked hub→tip.
                let tx = -(cx_ - dw / 2.0);
                let ty = base_y + ci as f64 * (dh + 0.8) + cy_;
                quad[qi] = [tx * rc - ty * rs, tx * rs + ty * rc];
            }
            out.push(quad);
        }
    }
    out
}

/// G-code for the flow comb: [`COMB_TEETH`] single-wall ring teeth radial
/// around an annular hub, each printed at its own ABSOLUTE flow —
/// [`COMB_FLOW_FAT`] stepping an exact 1% per tooth to [`COMB_FLOW_THIN`],
/// middle tooth exactly 1.00 — while PA holds the profile value. The whole
/// file prints at extrusion multiplier 1.0 with the ladder baked into E
/// (no M221), and every tooth carries its VALUE as a round percent
/// recessed into the underside plate: the label you read is the setting
/// you pin, verbatim. Caliper along the ring; several 1% neighbours will
/// read alike — take the middle of that run (tie toward 100). Stadium
/// tips and true junction fillets keep the loop from ever slowing near a
/// measured face (the comb prints before PA is calibrated); one bump on
/// the hub edge mid-gap is the only sharp feature, and the seam column
/// snaps to it, away from every tooth.
pub fn flow_comb_gcode(settings: &Settings, tool: u32) -> String {
    let mut s = tower_settings(settings);
    // Ordinary closed loops, not a helix: the per-tooth flow rides the
    // per-segment attribute channel, which the vase spiral bypasses.
    s.spiral_vase = false;
    // ABSOLUTE, not cumulative: the whole print runs at extrusion
    // multiplier 1.0, so each tooth's ladder value IS the candidate
    // setting. A comb that rode the current profile flow would compound —
    // every re-print's ladder relative to the last pin.
    s.extrusion_multiplier = 1.0;
    for t in &mut s.tools {
        t.extrusion_multiplier = 1.0;
    }
    // The fat tooth extrudes 12% over nominal — derate the melt ceiling so
    // it stays within the filament's real melt rate.
    s.max_volumetric_speed_mm3_s /= COMB_FLOW_FAT;
    for t in &mut s.tools {
        t.max_volumetric_speed_mm3_s /= COMB_FLOW_FAT;
    }
    let rh = comb_hub_r_mm();
    let mut tris = Vec::new();
    // Annular hub: a CCW disc with a REVERSED (CW) disc cut from it.
    let disc = |r: f64, ccw: bool| {
        let n = 180;
        let mut fp: Vec<[f64; 2]> = (0..n)
            .map(|i| {
                let a = f64::from(i) * 2.0 * std::f64::consts::PI / f64::from(n);
                [r * a.cos(), r * a.sin()]
            })
            .collect();
        if !ccw {
            fp.reverse();
        }
        fp
    };
    prism(&disc(rh, true), 0.0, COMB_H_MM, &mut tris);
    prism(&disc(rh - COMB_HUB_RING_MM, false), 0.0, COMB_H_MM, &mut tris);
    // Teeth: stadium footprints in tooth-local coords (x across the width,
    // y radial), rotated into place. The 1 mm root overlap into the hub and
    // the 0.5 mm fillet padding keep every prism buried in solid — no
    // knife-edge coincident walls.
    let hw = COMB_TOOTH_W_MM / 2.0;
    let place = |k: usize, fp: &[[f64; 2]]| -> Vec<[f64; 2]> {
        let a = comb_tooth_angle(k) - std::f64::consts::FRAC_PI_2;
        let (rc, rs) = (a.cos(), a.sin());
        fp.iter().map(|p| [p[0] * rc - p[1] * rs, p[0] * rs + p[1] * rc]).collect()
    };
    let tooth_fp = {
        let tip_c = rh + COMB_TOOTH_LEN_MM - hw;
        let mut fp = vec![[-hw, rh - 1.0], [hw, rh - 1.0]];
        for i in 0..=18 {
            let a = f64::from(i) * 10.0_f64.to_radians();
            fp.push([hw * a.cos(), tip_c + hw * a.sin()]);
        }
        fp
    };
    // Concave junction fillet in tooth-local coords: quarter arc bulging
    // toward the corner, legs padded 0.5 mm into hub and tooth. The hub
    // edge is locally straight to within 0.05 mm over the 2 mm fillet.
    let fillet_fp = |sx: f64| {
        let r = 2.0;
        let (corner_x, cy) = (sx * hw, rh);
        let (ccx, ccy) = (corner_x + sx * r, cy + r);
        let pad = 0.5;
        let mut fp = vec![[corner_x - sx * pad, cy - pad], [ccx, cy - pad]];
        for i in 0..=9 {
            let a = (-90.0 - f64::from(i) * 10.0 * sx).to_radians();
            fp.push([ccx + r * a.cos(), ccy + r * a.sin()]);
        }
        fp.push([corner_x - sx * pad, ccy]);
        let area: f64 = fp
            .windows(2)
            .map(|w| w[0][0] * w[1][1] - w[1][0] * w[0][1])
            .sum::<f64>()
            + fp.last().unwrap()[0] * fp[0][1]
            - fp[0][0] * fp.last().unwrap()[1];
        if area < 0.0 {
            fp[1..].reverse();
        }
        fp
    };
    for k in 0..COMB_TEETH {
        prism(&place(k, &tooth_fp), 0.0, COMB_H_MM, &mut tris);
        prism(&place(k, &fillet_fp(-1.0)), 0.0, COMB_H_MM, &mut tris);
        prism(&place(k, &fillet_fp(1.0)), 0.0, COMB_H_MM, &mut tris);
        // The label: REVERSED-winding prisms cut the value through the
        // base plate (above it the ring cavity is empty, so the tall cut
        // costs nothing).
        for quad in comb_label_quads(k) {
            let mut fp: Vec<[f64; 2]> = quad.to_vec();
            let area: f64 = fp
                .windows(2)
                .map(|w| w[0][0] * w[1][1] - w[1][0] * w[0][1])
                .sum::<f64>()
                + fp.last().unwrap()[0] * fp[0][1]
                - fp[0][0] * fp.last().unwrap()[1];
            if area > 0.0 {
                fp.reverse(); // holes are CW
            }
            prism(&fp, 0.0, 2.0, &mut tris);
        }
    }
    // Seam magnet: the one sharp feature — a small bump on the hub edge in
    // the mid-gap behind tooth 0. The aligned seam column snaps to the
    // sharpest corner in reach; give it one, away from every tooth.
    {
        let a = comb_tooth_angle(0) + std::f64::consts::PI / COMB_TEETH as f64;
        let (rc, rs) = ((a - std::f64::consts::FRAC_PI_2).cos(), (a - std::f64::consts::FRAC_PI_2).sin());
        let tri = [[0.0, rh + 1.6], [-0.9, rh - 0.8], [0.9, rh - 0.8]];
        let fp: Vec<[f64; 2]> =
            tri.iter().map(|p| [p[0] * rc - p[1] * rs, p[0] * rs + p[1] * rc]).collect();
        let area: f64 = fp
            .windows(2)
            .map(|w| w[0][0] * w[1][1] - w[1][0] * w[0][1])
            .sum::<f64>()
            + fp.last().unwrap()[0] * fp[0][1]
            - fp[0][0] * fp.last().unwrap()[1];
        let fp = if area < 0.0 { vec![fp[0], fp[2], fp[1]] } else { fp };
        prism(&fp, 0.0, COMB_H_MM, &mut tris);
    }
    let mesh = mesh::Mesh::from_triangle_soup(&tris);
    let mut plans = generate_parts(&[(&mesh, tool)], &s);
    // The mesh is built about the origin and the bbox is symmetric, so the
    // bed-centered frame is just a translation to bed center.
    let (ox, oy) = (s.bed_size_x_mm / 2.0, s.bed_size_y_mm / 2.0);
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
            if attrs.iter().any(|a| (f64::from(a.flow) - 1.0).abs() > 1.0e-3) {
                p.segs = Some(attrs);
            }
        }
    }
    let header = format!(
        "; flow comb: {COMB_TEETH} teeth radial around the hub, each a single-wall ring at\n\
         ; its own ABSOLUTE flow — 1% per tooth, {COMB_FLOW_FAT} down to {COMB_FLOW_THIN}, and every tooth\n\
         ; carries its VALUE recessed into the UNDERSIDE as percent MOD 100, two\n\
         ; digits (96 = 0.96, 04 = 1.04, 00 = 1.00) — flip the comb over to read.\n\
         ; the whole file prints at extrusion multiplier 1.0 with the ladder baked\n\
         ; into E (no M221): the label you read IS the setting you pin, verbatim.\n\
         ; caliper along the ring for the teeth that read exactly {:.2} mm (the line\n\
         ; width — full jaw flats, mid-face); 1% neighbours will read alike, so take\n\
         ; the MIDDLE of the matching run, tie toward 100. PA stays at the profile\n\
         ; value. the hub prints at true flow.\n",
        s.line_width_mm
    );
    format!("{header}{}", to_gcode(&plans, &s))
}

/// Flow multiplier at a point (centered comb coordinates): the owning
/// tooth's ladder value outside the hub, true flow on the hub and in the
/// junction band (the transition rides the fillet zone, below every
/// measured face).
fn comb_flow_at(x: f64, y: f64) -> f64 {
    let rh = comb_hub_r_mm();
    if x.hypot(y) < rh + 0.5 {
        return 1.0;
    }
    let step = 2.0 * std::f64::consts::PI / COMB_TEETH as f64;
    let k = ((std::f64::consts::FRAC_PI_2 - y.atan2(x)) / step).rem_euclid(COMB_TEETH as f64);
    comb_tooth_flow(k.round() as usize % COMB_TEETH)
}

/// The flow multiplier of tooth `k` (0 = the fat tooth at the rear; an
/// exact 1% per step, so every label is a round percent).
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

/// The winning tooth's LABEL — the percent recessed into its underside —
/// as the absolute flow multiplier to pin, verbatim. The comb prints at
/// extrusion multiplier 1.0, so nothing is multiplied into the current
/// setting and re-printing always reproduces the identical ladder.
/// Implausible labels (outside any real spool) leave the value alone by
/// returning 1.0-per-cent semantics only in range.
pub fn flow_from_comb_label(percent: f64) -> Option<f64> {
    let m = percent / 100.0;
    (0.7..=1.3).contains(&m).then_some(m)
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
        // One step per MILLIMETER band, not per layer: each step is a
        // Klipper queue-flush stall on the wall, and sub-mm steps are far
        // below reading resolution.
        assert!(
            (TOWER_H_MM as usize - 2..=TOWER_H_MM as usize + 2).contains(&ramp.len()),
            "one PA step per mm of height, got {}",
            ramp.len()
        );
        assert!(ramp.windows(2).all(|w| w[1] >= w[0]), "the sweep is monotonic");
        assert!(ramp[0] < 0.002, "starts at the bottom of the range");
        let top = ramp.last().unwrap();
        assert!(
            (PA_TOWER_START + PA_TOWER_FACTOR * TOWER_H_MM - top).abs() < 0.005,
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
        // The corner must print at the machine's REAL cornering accel — a
        // slowed corner dwells at the apex and measures ooze instead of
        // pressure advance (a 1200 cap pushed the read from 0.044 to
        // ~0.08+ against a true 0.032). The profile's wall accels must
        // reach the file unclamped.
        let max_m204 = g
            .lines()
            .filter_map(|l| l.strip_prefix("M204 S"))
            .filter_map(|v| v.trim().parse::<f64>().ok())
            .fold(0.0_f64, f64::max);
        assert!(
            max_m204 >= 1500.0,
            "tower corners at the real machine accel, got M204 S{max_m204}"
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
    fn pa_tower_two_speed_bands() {
        // The judged artifact is the speed-step pair: fast wall, 40% slow
        // zone around the apex, fast again — with the steps landing at the
        // slow-zone boundary (half the teardrop radius from the apex),
        // mid-leg on the flat faces. E-per-mm must be conserved across the
        // splits: the pattern changes speed, never flow.
        let s = Settings::default();
        let g = pa_tower_gcode(&s, 0);
        let (ax, ay) = (
            s.bed_size_x_mm / 2.0,
            s.bed_size_y_mm / 2.0 - TOWER_R_MM * (1.0 + std::f64::consts::SQRT_2) / 2.0,
        );
        let (mut x, mut y) = (0.0_f64, 0.0_f64);
        let mut f = 0.0_f64;
        let mut markers = 0usize;
        let mut feeds: std::collections::BTreeMap<i64, f64> = std::collections::BTreeMap::new();
        let mut crossings: Vec<f64> = Vec::new();
        let mut epmm: Vec<f64> = Vec::new();
        let mut last_f = 0.0_f64;
        for l in g.lines() {
            if l.starts_with("; LAYER ") {
                markers += 1;
            }
            if !(l.starts_with("G0 ") || l.starts_with("G1 ")) {
                continue;
            }
            let (mut nx, mut ny, mut e) = (x, y, 0.0_f64);
            for tok in l.split_whitespace().skip(1) {
                let (c, v) = tok.split_at(1);
                if let Ok(v) = v.parse::<f64>() {
                    match c {
                        "X" => nx = v,
                        "Y" => ny = v,
                        "E" => e = v,
                        "F" => f = v,
                        _ => {}
                    }
                }
            }
            let len = ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
            if markers > 4 && markers < 20 && e > 0.0 && len > 1e-6 {
                *feeds.entry(f.round() as i64).or_insert(0.0) += len;
                if len > 0.05 {
                    epmm.push(e / len);
                }
                // A feed change on an extruding move = a band edge: record
                // the step point's distance to the apex.
                if last_f > 0.0 && (f - last_f).abs() > 0.5 {
                    crossings.push(((x - ax).powi(2) + (y - ay).powi(2)).sqrt());
                }
                last_f = f;
            }
            x = nx;
            y = ny;
        }
        // Exactly two wall speeds, 0.4x apart.
        let major: Vec<i64> = feeds
            .iter()
            .filter(|(_, l)| **l > 20.0)
            .map(|(f, _)| *f)
            .collect();
        assert_eq!(major.len(), 2, "two wall speeds, got {feeds:?}");
        let ratio = major[0] as f64 / major[1] as f64;
        assert!(
            (ratio - PA_STEP_SLOW_FRAC).abs() < 0.02,
            "slow zone at {PA_STEP_SLOW_FRAC}x, got {ratio:.3}"
        );
        // Steps land at the slow-zone boundary: r/2 from the apex.
        assert!(!crossings.is_empty(), "speed steps present on every loop");
        for d in &crossings {
            assert!(
                (d - TOWER_R_MM * 0.5).abs() < 1.0,
                "step at the zone boundary (r/2 = {}), got {d:.2}",
                TOWER_R_MM * 0.5
            );
        }
        // Speed changes, flow does not: E-per-mm uniform across the splits.
        epmm.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let (p5, p95) = (epmm[epmm.len() / 20], epmm[epmm.len() * 19 / 20]);
        assert!(
            (p95 - p5) / p5 < 0.02,
            "E-per-mm conserved across splits: p5={p5:.5} p95={p95:.5}"
        );
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
        assert!(
            g.lines().filter(|l| l.starts_with("M221")).all(|l| l.trim() == "M221 S100"),
            "comb must not sweep firmware flow"
        );
        // Bucket wall E-per-mm by the shared classifier: the fat and thin
        // teeth must carry their ladder values relative to the hub.
        let (ox, oy) = (s.bed_size_x_mm / 2.0, s.bed_size_y_mm / 2.0);
        let rh = comb_hub_r_mm();
        let (mut x, mut y) = (0.0_f64, 0.0_f64);
        let (mut layer, mut wall) = (0usize, false);
        let mut buckets = [(0.0_f64, 0.0_f64); 3]; // fat, thin, hub
        for l in g.lines() {
            if l.starts_with("; LAYER ") {
                layer += 1;
                continue;
            }
            if let Some(t) = l.strip_prefix(";TYPE:") {
                wall = matches!(t.trim(), "Outer wall" | "Inner wall");
                continue;
            }
            // Arcs (G2/G3 — the un-annotated hub circle and skirt arc-fit)
            // must still advance the position or every later midpoint lies.
            let arc = l.starts_with("G2 ") || l.starts_with("G3 ");
            if !(l.starts_with("G1 ") || l.starts_with("G0 ") || arc) {
                continue;
            }
            let (mut nx, mut ny, mut e) = (x, y, 0.0_f64);
            for tok in l.split_whitespace().skip(1) {
                let (c, v) = tok.split_at(1);
                if let Ok(v) = v.parse::<f64>() {
                    match c {
                        "X" => nx = v,
                        "Y" => ny = v,
                        "E" => e = v,
                        _ => {}
                    }
                }
            }
            let len = ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
            let (mx, my) = ((x + nx) / 2.0 - ox, (y + ny) / 2.0 - oy);
            x = nx;
            y = ny;
            // Chord geometry only judges straight moves; 1 mm keeps the hub
            // circle's ~1.4 mm facets while excluding fillet and tip bits.
            if arc || layer <= 4 || !wall || e <= 1.0e-6 || len < 1.0 {
                continue;
            }
            let r = mx.hypot(my);
            let f = comb_flow_at(mx, my);
            // Teeth: mid-face band only, clear of fillets and tip rounds.
            let b = if (f - COMB_FLOW_FAT).abs() < 1e-9 && r > rh + 4.0 && r < rh + 16.0 {
                0
            } else if (f - COMB_FLOW_THIN).abs() < 1e-9 && r > rh + 4.0 && r < rh + 16.0 {
                1
            } else if (f - 1.0).abs() < 1e-9 && r > rh + 4.0 && r < rh + 16.0 {
                // The middle tooth prints at exactly 1.00 — the comb's own
                // built-in reference. (The hub itself offers no straight
                // reference wall: the fillets consume the inter-tooth arcs
                // and the inner circle is arc-fit.)
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
        let (fat, thin, mid) = (epmm(buckets[0]), epmm(buckets[1]), epmm(buckets[2]));
        assert!(
            (fat / mid - COMB_FLOW_FAT).abs() < 0.02,
            "fat tooth at {COMB_FLOW_FAT}x the 100% tooth, got {:.3}",
            fat / mid
        );
        assert!(
            (thin / mid - COMB_FLOW_THIN).abs() < 0.02,
            "thin tooth at {COMB_FLOW_THIN}x the 100% tooth, got {:.3}",
            thin / mid
        );
    }

    #[test]
    fn flow_comb_labels_the_teeth_on_the_underside() {
        // Every sampled tooth's percent label must slice as real voids in
        // the base plate, while the plate tipward of the label stays
        // covered.
        let s = Settings::default();
        let g = flow_comb_gcode(&s, 0);
        let (ox, oy) = (s.bed_size_x_mm / 2.0, s.bed_size_y_mm / 2.0);
        let (mut x, mut y) = (0.0_f64, 0.0_f64);
        let mut layer = -1_i64;
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for l in g.lines() {
            if l.starts_with("; LAYER ") {
                layer += 1;
                continue;
            }
            if layer > 0 {
                break;
            }
            let arc = l.starts_with("G2 ") || l.starts_with("G3 ");
            if !(l.starts_with("G0 ") || l.starts_with("G1 ") || arc) {
                continue;
            }
            let (mut nx, mut ny, mut e) = (x, y, 0.0_f64);
            for tok in l.split_whitespace().skip(1) {
                let (c, v) = tok.split_at(1);
                if let Ok(v) = v.parse::<f64>() {
                    match c {
                        "X" => nx = v,
                        "Y" => ny = v,
                        "E" => e = v,
                        _ => {}
                    }
                }
            }
            // Arc chords would place samples off the true path — skip their
            // interiors but keep the endpoint so tracking stays honest.
            if layer == 0 && e > 0.0 && !arc {
                let len = ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
                let n = (len / 0.2).ceil().max(1.0) as usize;
                for i in 0..=n {
                    let t = i as f64 / n as f64;
                    pts.push((x + (nx - x) * t - ox, y + (ny - y) * t - oy));
                }
            }
            x = nx;
            y = ny;
        }
        assert!(pts.len() > 1000, "layer 0 parsed, got {} samples", pts.len());
        for k in [0, 12, COMB_TEETH - 1] {
            let quads = comb_label_quads(k);
            assert!(!quads.is_empty());
            for q in &quads {
                // Clearance around each stroke's centroid: the slot is
                // 0.65 mm wide, so a 0.28 mm-radius void at its heart
                // proves the hole survived slicing and fill.
                let (cx_, cy_) = (
                    (q[0][0] + q[1][0] + q[2][0] + q[3][0]) / 4.0,
                    (q[0][1] + q[1][1] + q[2][1] + q[3][1]) / 4.0,
                );
                let hits = pts
                    .iter()
                    .filter(|(px, py)| (px - cx_).hypot(py - cy_) < 0.28)
                    .count();
                assert_eq!(
                    hits, 0,
                    "tooth {} label stroke at ({cx_:.1},{cy_:.1}) must be void",
                    k + 1
                );
            }
            // The plate tipward of the label stays covered.
            let a = comb_tooth_angle(k);
            let r = comb_hub_r_mm() + 16.0;
            let (px, py) = (r * a.cos(), r * a.sin());
            let near = pts
                .iter()
                .filter(|(qx, qy)| (qx - px).hypot(qy - py) < 1.5)
                .count();
            assert!(near > 0, "plate tipward of tooth {}'s label stays covered", k + 1);
        }
    }

    #[test]
    fn flow_comb_is_absolute_not_cumulative() {
        // The ladder must be identical whatever flow the profile currently
        // pins — the comb prints at extrusion multiplier 1.0 so a tooth's
        // value IS the setting, and re-printing after an apply reproduces
        // the same physical walls (nothing compounds).
        let base = flow_comb_gcode(&Settings::default(), 0);
        let mut s = Settings::default();
        s.extrusion_multiplier = 0.9;
        for t in &mut s.tools {
            t.extrusion_multiplier = 0.9;
        }
        let pinned = flow_comb_gcode(&s, 0);
        // Compare only the motion+extrusion body (headers echo settings).
        let body = |g: &str| {
            g.lines()
                .filter(|l| l.starts_with("G1 ") || l.starts_with("G0 "))
                .collect::<Vec<_>>()
                .join("\n")
        };
        assert_eq!(body(&base), body(&pinned), "ladder independent of the pinned flow");
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
    fn flow_comb_ladder_and_labels_are_absolute() {
        // Exact 1% steps, round-percent labels, middle tooth exactly 1.00.
        assert!((comb_tooth_flow(0) - COMB_FLOW_FAT).abs() < 1e-12);
        assert!((comb_tooth_flow(COMB_TEETH - 1) - COMB_FLOW_THIN).abs() < 1e-12);
        assert!((comb_tooth_flow(12) - 1.0).abs() < 1e-12, "middle tooth is 100%");
        for k in 1..COMB_TEETH {
            assert!(
                (comb_tooth_flow(k - 1) - comb_tooth_flow(k) - 0.01).abs() < 1e-12,
                "exact 1% per tooth"
            );
        }
        // The label IS the setting: percent in, multiplier out, verbatim.
        assert_eq!(flow_from_comb_label(98.0), Some(0.98));
        assert_eq!(flow_from_comb_label(112.0), Some(1.12));
        assert_eq!(flow_from_comb_label(100.0), Some(1.0));
        // Implausible labels are rejected, not clamped.
        assert_eq!(flow_from_comb_label(0.98), None);
        assert_eq!(flow_from_comb_label(500.0), None);
    }
}
