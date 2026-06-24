//! Numerical coverage comparison: how much of a layer's outline is left
//! uncovered by a given fill strategy. Env: LAYER, WALLS, INFILL_DENSITY,
//! INFILL_PATTERN, NO_GAP. Reports uncovered area % (the raster-grid measure),
//! not a render — so wall-logic vs infill-logic can be compared apples-to-apples.

fn main() {
    let stl = std::env::args().nth(1).unwrap_or_else(|| "fixtures/benchy.stl".into());
    let mesh = mesh::Mesh::load_stl(&stl).expect("load stl");
    let li: usize = std::env::var("LAYER").ok().and_then(|s| s.parse().ok()).unwrap_or(91);

    let mut s = config::Settings::default();
    s.top_layers = 0;
    s.bottom_layers = 0;
    if let Some(w) = std::env::var("WALLS").ok().and_then(|s| s.parse().ok()) {
        s.wall_count = w;
    }
    if let Some(d) = std::env::var("INFILL_DENSITY").ok().and_then(|s| s.parse().ok()) {
        s.infill_density = d;
    }
    if let Some(p) = std::env::var("INFILL_PATTERN").ok().and_then(|v| config::InfillPattern::parse(&v)) {
        s.sparse_pattern = p;
        s.solid_pattern = p;
    }
    if std::env::var("NO_GAP").is_ok() {
        s.gap_fill = false;
    }

    let layers = engine::generate(&mesh, &s);
    let l = &layers[li.min(layers.len()) - 1];
    let (area, unc) = engine::debug_uncovered(l, s.line_width_mm);
    // Over-extrusion: deposited bead volume vs the region volume. ~1.0 = balanced
    // (stadium overlap fills the cusps); >1.0 = depositing into occupied space.
    let h = l.height_mm;
    let mut vol = 0.0;
    for p in &l.paths {
        let pts = &p.points;
        if pts.len() < 2 {
            continue;
        }
        let bh = h * p.height_scale;
        let seg_flow = |k: usize| -> f64 { p.flows.as_ref().map_or(1.0, |f| f.get(k).copied().unwrap_or(1.0)) };
        let seg_w = |k: usize| -> f64 { p.widths.as_ref().map_or(p.width_mm, |ws| (ws[k] + ws[k + 1]) * 0.5) };
        for k in 0..pts.len() - 1 {
            let len = (pts[k + 1].x_mm() - pts[k].x_mm()).hypot(pts[k + 1].y_mm() - pts[k].y_mm());
            vol += config::bead_area_mm2(seg_w(k), bh) * len * seg_flow(k);
        }
        if p.closed {
            let (a, b) = (pts[pts.len() - 1], pts[0]);
            let len = (b.x_mm() - a.x_mm()).hypot(b.y_mm() - a.y_mm());
            vol += config::bead_area_mm2(p.width_mm, bh) * len * seg_flow(pts.len() - 1);
        }
    }
    let over = vol / (area * h);
    println!(
        "L{li}  walls={:<3} dens={:.2} gap={:<5}  outline {area:7.1} mm²  uncovered {unc:6.2} mm² ({:5.2}%)  deposited/region {over:.3} ({:+.0}%)  paths={}",
        s.wall_count,
        s.infill_density,
        s.gap_fill.to_string(),
        unc / area * 100.0,
        (over - 1.0) * 100.0,
        l.paths.len(),
    );
}
