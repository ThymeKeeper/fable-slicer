//! Hunt pathological beads: any path whose width (uniform or per-vertex)
//! exceeds a sane cap, plus per-kind census near a given spot.
//!
//! Usage: cargo run -p engine --example monster_probe -- <stl>

fn main() {
    let path = std::env::args().nth(1).expect("stl path");
    let mesh = mesh::Mesh::load_stl(&path).expect("load mesh");
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
    let lw = s.line_width_mm;
    for li in [3usize, 4, 121] {
        let (oa, unc) = engine::debug_uncovered(&plans[li], lw);
        println!("layer {li}: UNCOVERED {unc:.1}mm2 of {oa:.0}mm2");
    }
    let mut monsters = 0;
    for l in &plans {
        for p in &l.paths {
            let wmax = p
                .widths
                .as_ref()
                .map(|ws| ws.iter().cloned().fold(0.0f64, f64::max))
                .unwrap_or(p.width_mm);
            let wmin = p
                .widths
                .as_ref()
                .map(|ws| ws.iter().cloned().fold(f64::MAX, f64::min))
                .unwrap_or(p.width_mm);
            if wmax > lw * 3.0 || wmin < 0.0 || !wmax.is_finite() {
                monsters += 1;
                let (mut cx, mut cy) = (0.0, 0.0);
                for q in &p.points {
                    cx += q.x_mm();
                    cy += q.y_mm();
                }
                let n = p.points.len() as f64;
                println!(
                    "MONSTER layer {} (GUI {}): {:?} closed={} pts={} width_mm={:.2} wmin={:.2} wmax={:.2} at ({:.1},{:.1})",
                    l.index,
                    l.index + 1,
                    p.kind,
                    p.closed,
                    p.points.len(),
                    p.width_mm,
                    wmin,
                    wmax,
                    cx / n,
                    cy / n
                );
                if monsters > 20 {
                    println!("...more suppressed");
                    return;
                }
            }
        }
    }
    println!("{monsters} monster paths");
    // The swoop: all Solid/GapFill paths on layer 121 with bbox.
    for li in [121usize] {
        for p in &plans[li].paths {
            if matches!(p.kind, engine::PathKind::Solid | engine::PathKind::GapFill) {
                let len: f64 = p
                    .points
                    .windows(2)
                    .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
                    .sum();
                let (mut xa, mut ya, mut xb, mut yb) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
                for q in &p.points {
                    xa = xa.min(q.x_mm());
                    ya = ya.min(q.y_mm());
                    xb = xb.max(q.x_mm());
                    yb = yb.max(q.y_mm());
                }
                println!(
                    "L{li} {:?} closed={} len={len:.1}mm bbox ({xa:.1},{ya:.1})..({xb:.1},{yb:.1})",
                    p.kind, p.closed
                );
            }
        }
    }
}
