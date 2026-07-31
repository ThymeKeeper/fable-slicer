//! Wall-contact audit for fill line ends: distance from each open fill path
//! endpoint to the nearest wall bead centerline, bucketed by the end's
//! outward direction. Flush contact = lw (edge-to-edge kiss ≈ centerline
//! distance one bead width); pressed-in < lw; gapped > lw.
//!
//! Usage: cargo run -p engine --example ends_probe -- <stl> <layer>

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("stl path");
    let li: usize = args[2].parse().unwrap();

    let mesh = mesh::Mesh::load_stl(path).expect("load mesh");
    let mut s = config::Settings::default();
    s.wall_count = 4;
    s.layer_height_mm = 0.2;
    s.first_layer_height_mm = 0.2;
    s.line_width_mm = 0.4;
    s.top_layers = 2;
    s.bottom_layers = 2;
    s.infill_density = 1.0;
    s.outer_wall_first = false;
    s.seam_mode = config::SeamMode::Sharpest;
    s.skirt_loops = 0;

    let plans = engine::generate(&mesh, &s);
    let l = &plans[li];

    // Wall centerline segments.
    let mut walls: Vec<(geo2d::Point, geo2d::Point)> = Vec::new();
    for p in &l.paths {
        if matches!(
            p.kind,
            engine::PathKind::Perimeter
                | engine::PathKind::ExternalPerimeter
                | engine::PathKind::OverhangWall
        ) {
            let n = p.points.len();
            for j in 0..n.saturating_sub(1) {
                walls.push((p.points[j], p.points[j + 1]));
            }
            if p.closed && n >= 3 {
                walls.push((p.points[n - 1], p.points[0]));
            }
        }
    }
    let dist = |q: geo2d::Point| -> f64 {
        let mut best = f64::MAX;
        for &(a, b) in &walls {
            let (ax, ay, bx, by) = (a.x_mm(), a.y_mm(), b.x_mm(), b.y_mm());
            let (dx, dy) = (bx - ax, by - ay);
            let l2 = dx * dx + dy * dy;
            let t = if l2 > 0.0 {
                ((q.x_mm() - ax) * dx + (q.y_mm() - ay) * dy) / l2
            } else {
                0.0
            }
            .clamp(0.0, 1.0);
            let (px, py) = (ax + t * dx - q.x_mm(), ay + t * dy - q.y_mm());
            best = best.min((px * px + py * py).sqrt());
        }
        best
    };

    // Bucket endpooints by outward octant.
    let mut buckets: std::collections::BTreeMap<&'static str, Vec<f64>> = Default::default();
    let name = |dx: f64, dy: f64| -> &'static str {
        let a = dy.atan2(dx).to_degrees();
        match ((a + 382.5) / 45.0) as i32 % 8 {
            0 => "E ",
            1 => "NE",
            2 => "N ",
            3 => "NW",
            4 => "W ",
            5 => "SW",
            6 => "S ",
            _ => "SE",
        }
    };
    for p in &l.paths {
        if p.closed || p.points.len() < 2 || !matches!(p.kind, engine::PathKind::Infill) {
            continue;
        }
        let n = p.points.len();
        for (e, nb) in [(0usize, 1usize), (n - 1, n - 2)] {
            let (ex, ey) = (
                p.points[e].x_mm() - p.points[nb].x_mm(),
                p.points[e].y_mm() - p.points[nb].y_mm(),
            );
            let len = ex.hypot(ey);
            if len < 1e-9 {
                continue;
            }
            buckets
                .entry(name(ex / len, ey / len))
                .or_default()
                .push(dist(p.points[e]));
        }
    }
    // Turnaround apexes: interior vertices where the path reverses (~U-turn).
    let mut turns: std::collections::BTreeMap<&'static str, Vec<f64>> = Default::default();
    for p in &l.paths {
        if p.closed || p.points.len() < 3 || !matches!(p.kind, engine::PathKind::Infill) {
            continue;
        }
        for k in 1..p.points.len() - 1 {
            let (a, c, b) = (p.points[k - 1], p.points[k], p.points[k + 1]);
            let (ux, uy) = (c.x_mm() - a.x_mm(), c.y_mm() - a.y_mm());
            let (vx, vy) = (b.x_mm() - c.x_mm(), b.y_mm() - c.y_mm());
            let (lu, lv) = (ux.hypot(uy), vx.hypot(vy));
            if lu < 1e-9 || lv < 1e-9 {
                continue;
            }
            if (ux * vx + uy * vy) / (lu * lv) < -0.3 {
                // Outward = from the chord midpoint toward the apex.
                let (mx, my) = (
                    0.5 * (a.x_mm() + b.x_mm()),
                    0.5 * (a.y_mm() + b.y_mm()),
                );
                let (ox, oy) = (c.x_mm() - mx, c.y_mm() - my);
                let lo = ox.hypot(oy);
                if lo < 1e-9 {
                    continue;
                }
                turns
                    .entry(name(ox / lo, oy / lo))
                    .or_default()
                    .push(dist(c));
            }
        }
    }
    println!("layer {li}: TURNAROUND apexes by outward direction (dist to wall centerline, mm)");
    for (k, v) in &turns {
        let mut v = v.clone();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mean = v.iter().sum::<f64>() / v.len() as f64;
        println!(
            "  {k} n={:<4} mean={mean:.2} median={:.2} min={:.2} max={:.2}",
            v.len(),
            v[v.len() / 2],
            v[0],
            v[v.len() - 1]
        );
    }
    println!("layer {li}: free Infill ends by outward direction (dist to wall centerline, mm)");
    println!("  flush kiss = 0.40 (one bead); pressed < 0.40; gapped > 0.40");
    for (k, v) in &buckets {
        let mut v = v.clone();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mean = v.iter().sum::<f64>() / v.len() as f64;
        println!(
            "  {k} n={:<3} mean={mean:.2} median={:.2} min={:.2} max={:.2}",
            v.len(),
            v[v.len() / 2],
            v[0],
            v[v.len() - 1]
        );
    }
}
