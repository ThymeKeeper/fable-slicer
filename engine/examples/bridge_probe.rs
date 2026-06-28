//! Per-layer Bridge counts, with real bottom/top shells.
//! Env: BOTTOM, TOP (shell layer counts), WALLS, FOOTHOLD, DETAIL=N, GCODE.
fn main() {
    let stl = std::env::args().nth(1).unwrap_or_else(|| "fixtures/benchy.stl".into());
    let mesh = mesh::Mesh::load_stl(&stl).expect("load stl");
    let mut s = config::Settings::default();
    s.bottom_layers = std::env::var("BOTTOM").ok().and_then(|v| v.parse().ok()).unwrap_or(5);
    s.top_layers = std::env::var("TOP").ok().and_then(|v| v.parse().ok()).unwrap_or(5);
    if let Some(w) = std::env::var("WALLS").ok().and_then(|v| v.parse().ok()) { s.wall_count = w; }
    if let Some(d) = std::env::var("DENSITY").ok().and_then(|v| v.parse().ok()) { s.infill_density = d; }
    if let Some(h) = std::env::var("LH").ok().and_then(|v| v.parse().ok()) { s.layer_height_mm = h; }
    if let Some(f) = std::env::var("FOOTHOLD").ok().and_then(|v| v.parse().ok()) { s.bridge_foothold_mm = f; }
    if std::env::var("OUTER_FIRST").is_ok() { s.outer_wall_first = true; }
    if let Some(f) = std::env::var("FAN").ok().and_then(|v| v.parse().ok()) { s.fan_speed = f; }
    if let Some(p) = std::env::var("PATTERN").ok().and_then(|v| config::InfillPattern::parse(&v)) {
        s.bottom_pattern = p;
        s.solid_pattern = p;
    }
    let layers = engine::generate(&mesh, &s);
    if std::env::var("GCODE").is_ok() {
        print!("{}", engine::to_gcode(&layers, &s));
        return;
    }
    if let Some(n) = std::env::var("DETAIL").ok().and_then(|v| v.parse::<usize>().ok()) {
        let l = &layers[n - 1];
        println!("L{n} z={:.1} — paths by kind (count, total mm):", l.print_z_mm);
        let mut kinds: std::collections::BTreeMap<String, (usize, f64)> = std::collections::BTreeMap::new();
        for p in &l.paths {
            let len: f64 = p.points.windows(2)
                .map(|w| (w[1].x_mm() - w[0].x_mm()).hypot(w[1].y_mm() - w[0].y_mm()))
                .sum();
            let e = kinds.entry(format!("{:?}", p.kind)).or_default();
            e.0 += 1;
            e.1 += len;
        }
        for (k, (n, mm)) in &kinds {
            println!("  {k:<18} {n:>4}  {mm:>7.1}mm");
        }
        let mut bl: Vec<f64> = l.paths.iter()
            .filter(|p| p.kind == engine::PathKind::Bridge)
            .map(|p| p.points.windows(2).map(|w| (w[1].x_mm()-w[0].x_mm()).hypot(w[1].y_mm()-w[0].y_mm())).sum())
            .collect();
        bl.sort_by(|a, b| a.partial_cmp(b).unwrap());
        if !bl.is_empty() {
            let n = bl.len();
            println!("  bridge strand lengths (mm): min {:.1}, median {:.1}, max {:.1}", bl[0], bl[n/2], bl[n-1]);
        }
        return;
    }
    let c = |l: &engine::LayerPlan, k: engine::PathKind| l.paths.iter().filter(|p| p.kind == k).count();
    let mut tot = 0;
    for (i, l) in layers.iter().enumerate() {
        let b = c(l, engine::PathKind::Bridge);
        tot += b;
        if b > 0 {
            println!("L{:<3} z={:5.1}  bridge={b}", i + 1, l.print_z_mm);
        }
    }
    println!("total Bridge paths across model: {tot}");
}
