//! Measure how far Bridge line-ends sit from the surrounding non-bridge fill.
//! Env: WALLS, TOP, BOTTOM. Args: <stl> <layer>.
fn main() {
    let stl = std::env::args().nth(1).expect("stl path");
    let layer_n: usize = std::env::args().nth(2).expect("layer").parse().unwrap();
    let mesh = mesh::Mesh::load_stl(&stl).expect("load stl");
    let mut s = config::Settings::default();
    s.wall_count = std::env::var("WALLS").ok().and_then(|v| v.parse().ok()).unwrap_or(99);
    s.top_layers = std::env::var("TOP").ok().and_then(|v| v.parse().ok()).unwrap_or(2);
    s.bottom_layers = std::env::var("BOTTOM").ok().and_then(|v| v.parse().ok()).unwrap_or(2);
    let layers = engine::generate(&mesh, &s);
    let l = &layers[layer_n - 1];

    let bridges: Vec<&engine::ToolPath> =
        l.paths.iter().filter(|p| p.kind == engine::PathKind::Bridge && p.points.len() >= 2).collect();
    // Every non-bridge fill point (skin / solid / wall) to measure "reach to".
    let others: Vec<(f64, f64)> = l
        .paths
        .iter()
        .filter(|p| p.kind != engine::PathKind::Bridge)
        .flat_map(|p| p.points.iter().map(|pt| (pt.x_mm(), pt.y_mm())))
        .collect();
    println!("L{layer_n} walls={}: {} bridge paths, {} other-fill points", s.wall_count, bridges.len(), others.len());
    // Bridge bbox + model bbox (to aim the camera).
    let (mut bxn, mut byn, mut bxx, mut byx) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for b in &bridges {
        for p in &b.points {
            bxn = bxn.min(p.x_mm()); bxx = bxx.max(p.x_mm());
            byn = byn.min(p.y_mm()); byx = byx.max(p.y_mm());
        }
    }
    let (mut mxn, mut myn, mut mxx, mut myx) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for p in &l.paths { for pt in &p.points {
        mxn = mxn.min(pt.x_mm()); mxx = mxx.max(pt.x_mm());
        myn = myn.min(pt.y_mm()); myx = myx.max(pt.y_mm());
    }}
    println!("bridge bbox mm: x[{bxn:.1},{bxx:.1}] y[{byn:.1},{byx:.1}]  center ({:.1},{:.1})", (bxn+bxx)/2.0, (byn+byx)/2.0);
    println!("layer bbox mm : x[{mxn:.1},{mxx:.1}] y[{myn:.1},{myx:.1}]  center ({:.1},{:.1})", (mxn+mxx)/2.0, (myn+myx)/2.0);
    println!("=> --target {:.1},{:.1} (bridge center minus layer center)", (bxn+bxx)/2.0-(mxn+mxx)/2.0, (byn+byx)/2.0-(myn+myx)/2.0);

    let nearest = |x: f64, y: f64| -> f64 {
        others.iter().map(|&(ox, oy)| (ox - x).hypot(oy - y)).fold(f64::INFINITY, f64::min)
    };
    // Per-kind point clouds so we can see WHAT the bridge fails to reach.
    let pts_of = |pred: &dyn Fn(engine::PathKind) -> bool| -> Vec<(f64, f64)> {
        l.paths.iter().filter(|p| pred(p.kind)).flat_map(|p| p.points.iter().map(|pt| (pt.x_mm(), pt.y_mm()))).collect()
    };
    let walls = pts_of(&|k| matches!(k, engine::PathKind::Perimeter | engine::PathKind::ExternalPerimeter));
    let skin = pts_of(&|k| matches!(k, engine::PathKind::TopSkin | engine::PathKind::BottomSkin));
    let solid = pts_of(&|k| matches!(k, engine::PathKind::Solid));
    let near_in = |x: f64, y: f64, v: &[(f64, f64)]| -> f64 {
        v.iter().map(|&(ox, oy)| (ox - x).hypot(oy - y)).fold(f64::INFINITY, f64::min)
    };
    println!("near counts: walls={} skin={} solid={}", walls.len(), skin.len(), solid.len());
    // COLLISION: sample points ALONG each bridge bead (not just vertices) and count
    // how many land on a solid/skin bead (< lw*0.5 away) — i.e. double extrusion.
    {
        let solidskin = pts_of(&|k| {
            matches!(k, engine::PathKind::Solid | engine::PathKind::TopSkin | engine::PathKind::BottomSkin)
        });
        let mut samples = 0usize;
        let mut collide = 0usize;
        for b in &bridges {
            for w in b.points.windows(2) {
                let (a, z) = (w[0], w[1]);
                let len = (z.x_mm() - a.x_mm()).hypot(z.y_mm() - a.y_mm());
                let n = (len / (s.line_width_mm * 0.5)).ceil().max(1.0) as usize;
                for k in 0..=n {
                    let t = k as f64 / n as f64;
                    let (x, y) = (a.x_mm() + (z.x_mm() - a.x_mm()) * t, a.y_mm() + (z.y_mm() - a.y_mm()) * t);
                    samples += 1;
                    if near_in(x, y, &solidskin) < s.line_width_mm * 0.5 {
                        collide += 1;
                    }
                }
            }
        }
        let pct = if samples > 0 { 100.0 * collide as f64 / samples as f64 } else { 0.0 };
        println!("COLLISION: {collide}/{samples} bridge samples overlap solid/skin ({pct:.1}%)");
    }
    // Sample the bridge's extreme left/right/top/bottom vertex and report distances.
    {
        let b = bridges.iter().max_by_key(|b| b.points.len()).unwrap();
        let ext = |sel: &dyn Fn((f64, f64), (f64, f64)) -> bool| -> (f64, f64) {
            let mut best = (b.points[0].x_mm(), b.points[0].y_mm());
            for p in &b.points { let c = (p.x_mm(), p.y_mm()); if sel(c, best) { best = c; } }
            best
        };
        for (name, c) in [
            ("leftmost ", ext(&|c, b| c.0 < b.0)),
            ("rightmost", ext(&|c, b| c.0 > b.0)),
            ("topmost  ", ext(&|c, b| c.1 > b.1)),
            ("bottommost", ext(&|c, b| c.1 < b.1)),
        ] {
            println!(
                "  {name} ({:.1},{:.1}): →wall {:.2}  →skin {:.2}  →solid {:.2}",
                c.0, c.1, near_in(c.0, c.1, &walls), near_in(c.0, c.1, &skin), near_in(c.0, c.1, &solid)
            );
        }
    }
    // Turnaround/extreme vertices: the ends of each straight run WITHIN the
    // serpentine — i.e. wherever the path reverses direction (the left/right
    // extents of the boustrophedon), plus the two true endpoints. These are the
    // points that should land on the surrounding fill.
    let mut gaps: Vec<f64> = Vec::new();
    for b in &bridges {
        let p = &b.points;
        for i in 0..p.len() {
            let is_end = i == 0 || i == p.len() - 1;
            // A local turnaround: the path direction reverses in x (a boustrophedon
            // corner). Sample those and the endpoints.
            let turn = i > 0 && i + 1 < p.len() && {
                let dx0 = p[i].x_mm() - p[i - 1].x_mm();
                let dx1 = p[i + 1].x_mm() - p[i].x_mm();
                dx0 * dx1 < 0.0
            };
            if is_end || turn {
                gaps.push(nearest(p[i].x_mm(), p[i].y_mm()));
            }
        }
    }
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if !gaps.is_empty() {
        let med = gaps[gaps.len() / 2];
        let mx = *gaps.last().unwrap();
        let over_lw = gaps.iter().filter(|&&g| g > s.line_width_mm).count();
        println!("sampled {} bridge turnaround/end vertices", gaps.len());
        println!(
            "bridge-extent→nearest-other-fill gap: min {:.3}  median {med:.3}  max {mx:.3}mm  (lw={:.3})",
            gaps[0], s.line_width_mm
        );
        println!("extents farther than one line width from any other fill: {over_lw}/{}", gaps.len());
    }
}
