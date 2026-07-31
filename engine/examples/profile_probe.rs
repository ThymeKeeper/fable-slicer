//! Slice with the REAL resolved profiles (builtin + user dir), exactly as the
//! GUI does, plus the live panel deltas — then census a layer. Exists because
//! hardcoded probe settings diverged from the GUI's truth.
//!
//! Usage: cargo run -p engine --example profile_probe -- <stl> <layer..>

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("stl path");
    let layers: Vec<usize> = args[2..].iter().map(|s| s.parse().unwrap()).collect();

    let mut lib = config::Profiles::builtin();
    lib.load_user_profiles(None).expect("user profiles");
    let mut s = lib
        .resolve("sovol-zero-custom", "asa-custom", "sovol-zero-custom")
        .expect("resolve");
    // Live panel state from the screenshot (unsaved deltas).
    s.wall_count = 0;
    s.top_layers = 0;
    s.bottom_layers = 0;
    s.infill_density = 1.0;
    s.outer_wall_first = false;
    if let Ok(v) = std::env::var("LW") {
        s.line_width_mm = v.parse().unwrap();
    }
    println!(
        "resolved: lw={} lh={} first={} span_cap={} overhang_deg={} support={:?} spiral={}",
        s.line_width_mm,
        s.layer_height_mm,
        s.first_layer_height_mm,
        s.max_bridge_span_mm,
        s.support_overhang_angle_deg,
        s.support_mode,
        s.spiral_vase,
    );

    if std::env::var("FABLE_DUMP_SETTINGS").is_ok() {
        eprintln!("=== SETTINGS DUMP ===\n{s:#?}\n=== END DUMP ===");
    }
    let mut mesh = mesh::Mesh::load_stl(path).expect("load mesh");
    if let Ok(v) = std::env::var("SHIFT") {
        let (dx, dy) = v.split_once(',').unwrap();
        let (dx, dy): (f64, f64) = (dx.parse().unwrap(), dy.parse().unwrap());
        for v in &mut mesh.vertices {
            v[0] += dx;
            v[1] += dy;
        }
        s.auto_center_on_bed = false;
    }
    // The GUI's exact pipeline: plan_geometry + restamp_paint.
    let refs: Vec<(&mesh::Mesh, engine::PartPaint)> = vec![(&mesh, engine::PartPaint::Tool(0))];
    let geo = engine::plan_geometry(&refs, &s);
    let plans = engine::restamp_paint(&geo, &refs);
    println!("total layers: {}  total paths: {}", plans.len(), plans.iter().map(|l| l.paths.len()).sum::<usize>());
    for &li in &layers {
        let mut by: std::collections::BTreeMap<String, (usize, f64)> = Default::default();
        for p in &plans[li].paths {
            let len: f64 = p
                .points
                .windows(2)
                .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
                .sum();
            let e = by.entry(format!("{:?}", p.kind)).or_insert((0, 0.0));
            e.0 += 1;
            e.1 += len;
        }
        println!("=== layer {li} (GUI {}) z={:.2}", li + 1, plans[li].print_z_mm);
        if std::env::var("FABLE_RUNS").is_ok() {
            for (pi, pth) in plans[li].paths.iter().enumerate() {
                if !matches!(pth.kind, engine::PathKind::Infill) || pth.points.len() < 4 {
                    continue;
                }
                // Decompose into straight runs + turnarounds; label each
                // turnaround by which END of the stroke direction it sits at.
                let pts = &pth.points;
                let mut runs: Vec<f64> = Vec::new();
                let mut turns: Vec<(f64, f64, char)> = Vec::new();
                let mut acc = 0.0f64;
                let mut dir = (0.0f64, 0.0f64);
                for k in 0..pts.len() - 1 {
                    let (ax, ay) = (pts[k].x_mm(), pts[k].y_mm());
                    let (bx, by) = (pts[k + 1].x_mm(), pts[k + 1].y_mm());
                    let seg = (bx - ax).hypot(by - ay);
                    let (vx, vy) = ((bx - ax) / seg.max(1e-9), (by - ay) / seg.max(1e-9));
                    if k > 0 && (vx * dir.0 + vy * dir.1) < 0.3 {
                        // reversal: label end by projection sign on the FIRST run's direction
                        runs.push(acc);
                        acc = 0.0;
                        let side = if (ax * dir.0 + ay * dir.1) > 0.0 { 'A' } else { 'B' };
                        turns.push((ax, ay, side));
                    }
                    acc += seg;
                    dir = (vx, vy);
                }
                runs.push(acc);
                if runs.len() >= 4 {
                    // Re-label sides consistently: project each turn on the dominant run axis.
                    let (dx, dy) = dir;
                    let proj: Vec<f64> = turns.iter().map(|t| t.0 * dx + t.1 * dy).collect();
                    let mid = (proj.iter().cloned().fold(f64::MAX, f64::min)
                        + proj.iter().cloned().fold(f64::MIN, f64::max))
                        / 2.0;
                    let labels: String =
                        proj.iter().map(|&v| if v > mid { 'A' } else { 'B' }).collect();
                    let rl: Vec<f64> = runs.iter().map(|r| (r * 10.0).round() / 10.0).collect();
                    println!("  path {pi}: runs={rl:?} turn-ends={labels}");
                }
            }
        }
        for pth in &plans[li].paths {
            if !matches!(pth.kind, engine::PathKind::Bridge) {
                continue;
            }
            // Straight runs between direction reversals = the strands.
            let mut runs: Vec<f64> = Vec::new();
            let mut acc = 0.0f64;
            let pts = &pth.points;
            for k in 0..pts.len() - 1 {
                let (ax, ay) = (pts[k].x_mm(), pts[k].y_mm());
                let (bx, by) = (pts[k + 1].x_mm(), pts[k + 1].y_mm());
                let seg = (bx - ax).hypot(by - ay);
                if k > 0 {
                    let (px, py) = (pts[k - 1].x_mm(), pts[k - 1].y_mm());
                    let (ux, uy) = (ax - px, ay - py);
                    let (vx, vy) = (bx - ax, by - ay);
                    let (lu, lv) = (ux.hypot(uy).max(1e-9), vx.hypot(vy).max(1e-9));
                    if (ux * vx + uy * vy) / (lu * lv) < 0.3 {
                        runs.push(acc);
                        acc = 0.0;
                    }
                }
                acc += seg;
            }
            runs.push(acc);
            runs.sort_by(|a, b| b.partial_cmp(a).unwrap());
            println!("  Bridge strand runs (mm): {:?}", runs.iter().map(|r| (r * 10.0).round() / 10.0).collect::<Vec<_>>());
        }
        for (k, (n, len)) in &by {
            println!("  {k:<18} n={n:<4} total={len:.0}mm");
        }
    }
}
