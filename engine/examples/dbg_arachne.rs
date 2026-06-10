//! Dump arachne bead stats for real Benchy layers (debugging missing arcs).
fn main() {
    let mesh = mesh::Mesh::load_stl("fixtures/benchy.stl").unwrap();
    let s = config::Settings::default();
    let layers = engine::slice_mesh(&mesh, engine::SliceParams {
        layer_height_mm: s.layer_height_mm,
        first_layer_height_mm: s.first_layer_height_mm,
    });
    for idx in [93usize, 195] {
        let outline = geo2d::simplify(&layers[idx].polygons, 0.05);
        let inner = geo2d::offset(&outline, -0.45);
        let beads = engine::dbg_variable_walls(&outline, &inner, 0.45, 0.407, 1);
        // Reference: the classic inner-wall centerline length at this depth.
        let classic = geo2d::offset(&outline, -(0.45 * 0.5 + 0.407));
        let classic_len: f64 = classic.contours.iter().map(|c| {
            let n = c.points.len();
            (0..n).map(|i| {
                let a = c.points[i]; let b = c.points[(i + 1) % n];
                (a.x_mm() - b.x_mm()).hypot(a.y_mm() - b.y_mm())
            }).sum::<f64>()
        }).sum();
        let total: f64 = beads.iter().map(|(len, _, _)| len).sum();
        println!("layer {idx}: {n} beads, total {total:.1}mm vs classic {classic_len:.1}mm", n = beads.len());
        for (len, closed, w) in &beads {
            if *len > 1.0 { println!("  bead len={len:.1} closed={closed} w={w:.2}"); }
        }
    }
}
