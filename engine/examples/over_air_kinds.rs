//! Per path-kind over-air census for a layer: how much of each kind's bead
//! length hangs over the layer below's open air, with the over-air bbox.
//! Finds which pass claimed an unsupported region.
//!
//! Usage: cargo run -p engine --example over_air_kinds -- <stl> <walls> <top> <bottom> <layer..>

use std::collections::BTreeMap;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("stl path");
    let walls: usize = args[2].parse().unwrap();
    let top: usize = args[3].parse().unwrap();
    let bottom: usize = args[4].parse().unwrap();
    let mut layers: Vec<usize> = args[5..].iter().map(|s| s.parse().unwrap()).collect();

    let mesh = mesh::Mesh::load_stl(path).expect("load mesh");
    let mut s = config::Settings::default();
    s.wall_count = walls;
    s.layer_height_mm = 0.2;
    s.first_layer_height_mm = 0.2;
    s.line_width_mm = 0.4;
    s.top_layers = top;
    s.bottom_layers = bottom;
    s.infill_density = 0.15;
    s.outer_wall_first = false;
    s.seam_mode = config::SeamMode::Sharpest;
    s.skirt_loops = 0;

    let plans = engine::generate(&mesh, &s);
    println!("total layers: {}", plans.len());
    let sweep = layers.is_empty();
    if sweep {
        layers = (1..plans.len()).collect();
    }

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
        let below = &plans[li - 1].outline;
        // (total_len, air_len, air bbox)
        let mut per: BTreeMap<String, (f64, f64, [f64; 4], usize)> = BTreeMap::new();
        let list = std::env::var("FABLE_LIST").is_ok();
        for p in &plans[li].paths {
            if list {
                let mut air = 0.0f64;
                let (mut xa, mut ya, mut xb, mut yb) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
                for w in p.points.windows(2) {
                    let (ax, ay) = (w[0].x_mm(), w[0].y_mm());
                    let (bx, by) = (w[1].x_mm(), w[1].y_mm());
                    let seg = (bx - ax).hypot(by - ay);
                    let steps = (seg / 0.3).ceil().max(1.0) as usize;
                    let ds = seg / steps as f64;
                    for k in 0..steps {
                        let t = (k as f64 + 0.5) / steps as f64;
                        let (x, y) = (ax + t * (bx - ax), ay + t * (by - ay));
                        if !inside(below, x, y) {
                            air += ds;
                            xa = xa.min(x);
                            ya = ya.min(y);
                            xb = xb.max(x);
                            yb = yb.max(y);
                        }
                    }
                }
                if air > 2.0 {
                    println!(
                        "    path {:?} closed={} air={air:.1}mm airbbox ({xa:.1},{ya:.1})..({xb:.1},{yb:.1})",
                        p.kind, p.closed
                    );
                }
            }
            let e = per
                .entry(format!("{:?}", p.kind))
                .or_insert((0.0, 0.0, [f64::MAX, f64::MAX, f64::MIN, f64::MIN], 0));
            e.3 += 1;
            for w in p.points.windows(2) {
                let (ax, ay) = (w[0].x_mm(), w[0].y_mm());
                let (bx, by) = (w[1].x_mm(), w[1].y_mm());
                let seg = (bx - ax).hypot(by - ay);
                let steps = (seg / 0.3).ceil().max(1.0) as usize;
                let ds = seg / steps as f64;
                for k in 0..steps {
                    let t = (k as f64 + 0.5) / steps as f64;
                    let (x, y) = (ax + t * (bx - ax), ay + t * (by - ay));
                    e.0 += ds;
                    if !inside(below, x, y) {
                        e.1 += ds;
                        e.2[0] = e.2[0].min(x);
                        e.2[1] = e.2[1].min(y);
                        e.2[2] = e.2[2].max(x);
                        e.2[3] = e.2[3].max(y);
                    }
                }
            }
        }
        if sweep {
            // Only flag FILL kinds over air — walls/overhang/bridge cover air
            // by design. Solid/skins/infill over air are dishonest coverage.
            for (k, (_tot, air, bb, n)) in &per {
                if *air > 0.4
                    && !matches!(
                        k.as_str(),
                        "ExternalPerimeter" | "Perimeter" | "OverhangWall" | "Bridge" | "Support"
                    )
                {
                    println!(
                        "  layer {li} (GUI {}): {k} n={n} {air:.1}mm OVER AIR bbox ({:.1},{:.1})..({:.1},{:.1})",
                        li + 1, bb[0], bb[1], bb[2], bb[3]
                    );
                }
            }
            continue;
        }
        println!("\n=== layer {li} (GUI {}) ===", li + 1);
        for (k, (tot, air, bb, n)) in &per {
            if *air > 0.5 {
                println!(
                    "  {k:<18} n={n:<4} {tot:.0}mm total, {air:.1}mm OVER AIR  bbox ({:.1},{:.1})..({:.1},{:.1})",
                    bb[0], bb[1], bb[2], bb[3]
                );
            } else {
                println!("  {k:<18} n={n:<4} {tot:.0}mm total, none over air");
            }
        }
    }
}
