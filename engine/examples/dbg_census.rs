//! Census: open/closed bead counts and lengths across every Benchy layer —
//! the whole-model ring-integrity metric for the arachne extractor (rings
//! should come out as rings; open beads should be transition-anchored spans,
//! not fragments). Compare extractors with ARACHNE_GRID=1.
fn main() {
    let mesh = mesh::Mesh::load_stl("fixtures/benchy.stl").unwrap();
    let s = config::Settings::default();
    let layers = engine::slice_mesh(&mesh, engine::SliceParams {
        layer_height_mm: s.layer_height_mm,
        first_layer_height_mm: s.first_layer_height_mm,
    });
    let mut closed_n = 0usize;
    let mut open_n = 0usize;
    let mut open_len = 0.0f64;
    let mut closed_len = 0.0f64;
    let mut worst: Vec<(usize, f64, f64)> = Vec::new();
    for (idx, layer) in layers.iter().enumerate() {
        let outline = geo2d::simplify(&layer.polygons, 0.05);
        let inner = geo2d::offset(&outline, -0.45);
        let beads = engine::dbg_variable_walls(&outline, &inner, 0.45, 0.407, 1);
        for (len, closed, w) in beads {
            if closed {
                closed_n += 1;
                closed_len += len;
            } else {
                open_n += 1;
                open_len += len;
                if len > 3.0 {
                    worst.push((idx, len, w));
                }
            }
        }
    }
    println!(
        "closed: {closed_n} beads {closed_len:.0}mm | open: {open_n} beads {open_len:.0}mm ({:.1}% of length)",
        100.0 * open_len / (open_len + closed_len)
    );
    worst.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    for (idx, len, w) in worst.iter().take(12) {
        println!("  layer {idx}: open {len:.1}mm w={w:.2}");
    }
}
