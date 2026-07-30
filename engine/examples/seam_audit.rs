//! Audit seam placement: per layer, the windowed sharpness AT the chosen seam
//! vs the sharpest available vertex on the loop — quantifies "sharpest mode
//! parked my seam on a flat".
//!
//! Usage: cargo run -p engine --example seam_audit -- <stl>

fn main() {
    let path = std::env::args().nth(1).expect("stl path");
    let mesh = mesh::Mesh::load_stl(&path).expect("load mesh");
    let mut s = config::Settings::default();
    s.wall_count = 99;
    s.layer_height_mm = 0.2;
    s.first_layer_height_mm = 0.2;
    s.line_width_mm = 0.4;
    s.top_layers = 0;
    s.bottom_layers = 0;
    s.infill_density = 1.0;
    s.seam_mode = config::SeamMode::Sharpest;
    s.skirt_loops = 0;

    let plans = engine::generate(&mesh, &s);

    // Windowed sharpness (1 − cos of the turn between ±1.25mm chords).
    let sharp_at = |pts: &Vec<_>, i: usize| -> f64 {
        let pts: &Vec<geo2d::Point> = pts;
        let n = pts.len();
        let d = |a: geo2d::Point, b: geo2d::Point| {
            (a.x_mm() - b.x_mm()).hypot(a.y_mm() - b.y_mm())
        };
        let w = 1.25;
        let walk = |dir: i64| -> geo2d::Point {
            let mut acc = 0.0;
            let mut k = i as i64;
            loop {
                let nk = (k + dir).rem_euclid(n as i64);
                acc += d(pts[k.rem_euclid(n as i64) as usize], pts[nk as usize]);
                k = nk;
                if acc >= w || k == i as i64 {
                    return pts[nk as usize];
                }
            }
        };
        let (a, b) = (walk(-1), walk(1));
        let c = pts[i];
        let (ux, uy) = (c.x_mm() - a.x_mm(), c.y_mm() - a.y_mm());
        let (vx, vy) = (b.x_mm() - c.x_mm(), b.y_mm() - c.y_mm());
        let (lu, lv) = (ux.hypot(uy).max(1e-9), vx.hypot(vy).max(1e-9));
        1.0 - (ux * vx + uy * vy) / (lu * lv)
    };

    let (mut flat_seam_sharp_loop, mut flat_both, mut good) = (0usize, 0usize, 0usize);
    let mut smooth_ratio_sum = 0.0f64;
    let mut smooth_at_max = 0usize;
    for l in &plans {
        for p in &l.paths {
            if !matches!(p.kind, engine::PathKind::ExternalPerimeter) || p.points.len() < 8 {
                continue;
            }
            let seam_sharp = sharp_at(&p.points, 0);
            let max_sharp = (0..p.points.len())
                .map(|i| sharp_at(&p.points, i))
                .fold(0.0f64, f64::max);
            if seam_sharp < 0.13 && max_sharp >= 0.13 {
                flat_seam_sharp_loop += 1; // corner existed, seam missed it
            } else if seam_sharp < 0.13 {
                flat_both += 1; // genuinely smooth loop
                if max_sharp > 1e-9 {
                    let r = seam_sharp / max_sharp;
                    smooth_ratio_sum += r;
                    if r >= 0.6 {
                        smooth_at_max += 1;
                    }
                }
            } else {
                good += 1;
            }
        }
    }
    println!("seam ON a corner:                {good}");
    println!("flat seam, loop HAS corners:     {flat_seam_sharp_loop}  <-- the defect");
    println!("flat seam, loop genuinely smooth:{flat_both}");
    if flat_both > 0 {
        println!(
            "  of those: {} seams at >=60% of loop-max curvature (mean ratio {:.2})",
            smooth_at_max,
            smooth_ratio_sum / flat_both as f64
        );
    }
}
