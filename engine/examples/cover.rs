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
    println!(
        "L{li}  walls={:<3} dens={:.2} gap={:<5}  outline {area:7.1} mm²  uncovered {unc:6.2} mm²  ({:5.2}%)  paths={}",
        s.wall_count,
        s.infill_density,
        s.gap_fill.to_string(),
        unc / area * 100.0,
        l.paths.len(),
    );
}
