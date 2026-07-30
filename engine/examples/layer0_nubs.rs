//! Small non-wall paths on a given layer, with positions — hunts the
//! "bottom fill nubs outside the wall" on the first layer.
//!
//! Usage: cargo run -p engine --example layer0_nubs -- <stl> <top> <bottom> <layer>

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("stl path");
    let top: usize = args[2].parse().unwrap();
    let bottom: usize = args[3].parse().unwrap();
    let li: usize = args[4].parse().unwrap();

    let mesh = mesh::Mesh::load_stl(path).expect("load mesh");
    let mut s = config::Settings::default();
    s.wall_count = 99;
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
    let l = &plans[li];
    println!("layer {li}: {} paths", l.paths.len());
    let mut by: std::collections::BTreeMap<String, (usize, f64)> = Default::default();
    for p in &l.paths {
        let len: f64 = p
            .points
            .windows(2)
            .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
            .sum();
        let e = by.entry(format!("{:?}", p.kind)).or_insert((0, 0.0));
        e.0 += 1;
        e.1 += len;
        if len < 8.0 && !matches!(p.kind, engine::PathKind::Skirt) {
            let (mut cx, mut cy) = (0.0, 0.0);
            for q in &p.points {
                cx += q.x_mm();
                cy += q.y_mm();
            }
            let n = p.points.len() as f64;
            let (cx, cy) = (cx / n, cy / n);
            // Distance to the slice outline: how far outside/inside the rim.
            let mut d_out = f64::MAX;
            for c in &l.outline.contours {
                let m = c.points.len();
                for k in 0..m {
                    let a = c.points[k];
                    let b = c.points[(k + 1) % m];
                    let (dx, dy) = (b.x_mm() - a.x_mm(), b.y_mm() - a.y_mm());
                    let l2 = dx * dx + dy * dy;
                    let t = if l2 > 0.0 {
                        ((cx - a.x_mm()) * dx + (cy - a.y_mm()) * dy) / l2
                    } else {
                        0.0
                    }
                    .clamp(0.0, 1.0);
                    let (px, py) = (a.x_mm() + t * dx - cx, a.y_mm() + t * dy - cy);
                    d_out = d_out.min((px * px + py * py).sqrt());
                }
            }
            println!(
                "  nub: {:?} len={len:.1}mm at ({cx:.1},{cy:.1}) outline_dist={d_out:.2}mm",
                p.kind
            );
        }
    }
    for (k, (n, len)) in &by {
        println!("  {k:<18} n={n:<4} total={len:.0}mm");
    }
}
