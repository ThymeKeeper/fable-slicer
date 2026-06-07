//! Diagnostic: inspect a single sliced layer's outer contour to tell whether
//! jaggedness is in the geometry (STL/slice) or just the bead render.
//! `cargo run -p engine --example dump_layer -- fixtures/benchy.stl 35`

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = &args[1];
    let target: usize = args[2].parse().unwrap();

    let m = mesh::Mesh::load_stl(path).unwrap();
    let layers = engine::slice_mesh(
        &m,
        engine::SliceParams { layer_height_mm: 0.2, first_layer_height_mm: 0.2 },
    );
    let layer = &layers[target - 1];

    let outer = layer
        .polygons
        .contours
        .iter()
        .max_by(|a, b| a.area_mm2().partial_cmp(&b.area_mm2()).unwrap())
        .unwrap();

    println!("layer {target} z={:.3}mm", layer.print_z_mm);
    println!("outer contour: {} points, area {:.1} mm²", outer.points.len(), outer.area_mm2());

    // How many points survive simplification at increasing tolerances? A contour
    // that's smooth-but-finely-sampled collapses a lot at tiny tolerance; genuine
    // jaggedness needs a larger tolerance to remove.
    for tol in [0.01, 0.05, 0.1, 0.2, 0.5] {
        let s = geo2d::simplify(&layer.polygons, tol);
        let sc = s
            .contours
            .iter()
            .max_by(|a, b| a.area_mm2().partial_cmp(&b.area_mm2()).unwrap())
            .unwrap();
        println!("  simplify({tol:>4}mm): {:>4} pts, area {:.1} mm²", sc.points.len(), sc.area_mm2());
    }

    // Local roughness: at each vertex, the perpendicular distance from the chord
    // between its neighbours (a measure of how much the point sticks out).
    let p = &outer.points;
    let n = p.len();
    let mut devs: Vec<f64> = Vec::new();
    for i in 0..n {
        let a = p[(i + n - 1) % n];
        let b = p[i];
        let c = p[(i + 1) % n];
        let (ax, ay) = (a.x_mm(), a.y_mm());
        let (bx, by) = (b.x_mm(), b.y_mm());
        let (cx, cy) = (c.x_mm(), c.y_mm());
        let len = ((cx - ax).powi(2) + (cy - ay).powi(2)).sqrt();
        if len < 1e-9 {
            continue;
        }
        let dev = ((cx - ax) * (ay - by) - (ay - cy) * (ax - bx)).abs() / len;
        devs.push(dev);
    }
    devs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = devs[devs.len() / 2];
    let p95 = devs[devs.len() * 95 / 100];
    let max = *devs.last().unwrap();
    println!("per-vertex deviation from neighbour-chord (mm): median {median:.4}, p95 {p95:.4}, max {max:.4}");
}
