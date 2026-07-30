//! For each requested layer, classify every Bridge bead sample point against
//! the layer below: over the below layer's mesh cross-section (supported by
//! solid material), over a below-layer printed bead (supported by a roof it
//! printed over ITS OWN air), or over open air. Settles "why is this a
//! bridge — the layer below supports it, doesn't it?" with numbers.
//!
//! Usage: cargo run -p engine --example bridge_support -- <stl> <layer> [layer..]

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("stl path");
    let mut layers: Vec<usize> = args[2..].iter().map(|s| s.parse().unwrap()).collect();

    let mesh = mesh::Mesh::load_stl(path).expect("load mesh");
    let mut s = config::Settings::default();
    // The user's live (unsaved) GUI process: 99 walls, top 0 / bottom 0,
    // density 0.15 lines, outer-first OFF, sharpest.
    s.wall_count = 99;
    s.layer_height_mm = 0.2;
    s.first_layer_height_mm = 0.2;
    s.line_width_mm = 0.4;
    s.top_layers = 0;
    s.bottom_layers = 0;
    s.infill_density = 0.15;
    s.outer_wall_first = false;
    s.seam_mode = config::SeamMode::Sharpest;
    s.skirt_loops = 0;

    let plans = engine::generate(&mesh, &s);
    println!("total layers: {}", plans.len());
    if layers.is_empty() {
        // Sweep: every layer that has Bridge paths.
        layers = (1..plans.len())
            .filter(|&i| {
                plans[i]
                    .paths
                    .iter()
                    .any(|p| matches!(p.kind, engine::PathKind::Bridge))
            })
            .collect();
        println!("layers with bridges: {layers:?}");
    }

    // Even-odd point-in-polygons over a layer outline (holes handled by parity).
    let inside = |outline: &geo2d::Polygons, x: f64, y: f64| -> bool {
        let mut hits = 0usize;
        for c in &outline.contours {
            let n = c.points.len();
            for k in 0..n {
                let a = c.points[k];
                let b = c.points[(k + 1) % n];
                let (ay, by) = (a.y_mm(), b.y_mm());
                if (ay > y) == (by > y) {
                    continue;
                }
                let t = (y - ay) / (by - ay);
                if a.x_mm() + t * (b.x_mm() - a.x_mm()) > x {
                    hits += 1;
                }
            }
        }
        hits % 2 == 1
    };

    for &li in &layers {
        assert!(li >= 1, "need a layer below");
        let (below, cur) = (&plans[li - 1], &plans[li]);
        println!(
            "\n=== layer {li} (GUI {}) bridges vs layer {} below ===",
            li + 1,
            li - 1
        );

        // The below layer's printed coverage: every bead as (polyline, half width).
        let below_beads: Vec<(&Vec<geo2d::Point>, f64)> = below
            .paths
            .iter()
            .map(|p| (&p.points, p.width_mm * 0.5))
            .collect();
        let on_below_bead = |x: f64, y: f64| -> bool {
            for (pts, hw) in &below_beads {
                for w in pts.windows(2) {
                    let (ax, ay) = (w[0].x_mm(), w[0].y_mm());
                    let (bx, by) = (w[1].x_mm(), w[1].y_mm());
                    let (dx, dy) = (bx - ax, by - ay);
                    let l2 = dx * dx + dy * dy;
                    let t = if l2 > 0.0 {
                        ((x - ax) * dx + (y - ay) * dy) / l2
                    } else {
                        0.0
                    }
                    .clamp(0.0, 1.0);
                    let (px, py) = (ax + t * dx - x, ay + t * dy - y);
                    if (px * px + py * py).sqrt() <= hw + 0.05 {
                        return true;
                    }
                }
            }
            false
        };

        let mut n_bridges = 0usize;
        let (mut len_mesh, mut len_roof, mut len_air) = (0.0f64, 0.0f64, 0.0f64);
        let (mut minx, mut miny, mut maxx, mut maxy) =
            (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in &cur.paths {
            if !matches!(p.kind, engine::PathKind::Bridge) {
                continue;
            }
            n_bridges += 1;
            for w in p.points.windows(2) {
                let (ax, ay) = (w[0].x_mm(), w[0].y_mm());
                let (bx, by) = (w[1].x_mm(), w[1].y_mm());
                let seg = (bx - ax).hypot(by - ay);
                let steps = (seg / 0.2).ceil().max(1.0) as usize;
                let ds = seg / steps as f64;
                for k in 0..steps {
                    let t = (k as f64 + 0.5) / steps as f64;
                    let (x, y) = (ax + t * (bx - ax), ay + t * (by - ay));
                    minx = minx.min(x);
                    miny = miny.min(y);
                    maxx = maxx.max(x);
                    maxy = maxy.max(y);
                    if inside(&below.outline, x, y) {
                        len_mesh += ds;
                    } else if on_below_bead(x, y) {
                        len_roof += ds;
                    } else {
                        len_air += ds;
                    }
                }
            }
        }
        let total = len_mesh + len_roof + len_air;
        if n_bridges == 0 {
            println!("  no Bridge paths on this layer");
            continue;
        }
        println!(
            "  {n_bridges} bridge paths, {total:.1}mm — bbox ({minx:.1},{miny:.1})..({maxx:.1},{maxy:.1})"
        );
        println!(
            "  over below MESH section: {len_mesh:.1}mm ({:.0}%)   <- anchored on solid",
            100.0 * len_mesh / total
        );
        println!(
            "  over below printed BEAD: {len_roof:.1}mm ({:.0}%)   <- resting on a roof below",
            100.0 * len_roof / total
        );
        println!(
            "  over OPEN AIR:           {len_air:.1}mm ({:.0}%)   <- nothing underneath",
            100.0 * len_air / total
        );
    }
}
