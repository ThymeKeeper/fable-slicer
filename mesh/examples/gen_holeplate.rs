//! Generate a 40×40×3 mm plate with a centered 14×14 mm square hole, for testing
//! combing. `cargo run -p mesh --example gen_holeplate` → fixtures/holeplate.stl.
//! (Winding doesn't matter — the slicer stitches orientation-independently.)

type V = [f64; 3];

fn quad(t: &mut Vec<[V; 3]>, a: V, b: V, c: V, d: V) {
    t.push([a, b, c]);
    t.push([a, c, d]);
}

fn rect(t: &mut Vec<[V; 3]>, x0: f64, y0: f64, x1: f64, y1: f64, z: f64) {
    quad(t, [x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]);
}

fn wall(t: &mut Vec<[V; 3]>, ax: f64, ay: f64, bx: f64, by: f64, z0: f64, z1: f64) {
    quad(t, [ax, ay, z0], [bx, by, z0], [bx, by, z1], [ax, ay, z1]);
}

fn main() -> std::io::Result<()> {
    let (lo, hi) = (0.0, 40.0);
    let (hl, hh) = (13.0, 27.0); // hole bounds
    let (z0, z1) = (0.0, 3.0);
    let mut t: Vec<[V; 3]> = Vec::new();

    // Top and bottom faces: the frame around the hole, as four strips.
    for &z in &[z0, z1] {
        rect(&mut t, lo, lo, hi, hl, z); // front
        rect(&mut t, lo, hh, hi, hi, z); // back
        rect(&mut t, lo, hl, hl, hh, z); // left
        rect(&mut t, hh, hl, hi, hh, z); // right
    }
    // Outer walls.
    wall(&mut t, lo, lo, hi, lo, z0, z1);
    wall(&mut t, hi, lo, hi, hi, z0, z1);
    wall(&mut t, hi, hi, lo, hi, z0, z1);
    wall(&mut t, lo, hi, lo, lo, z0, z1);
    // Hole walls.
    wall(&mut t, hl, hl, hh, hl, z0, z1);
    wall(&mut t, hh, hl, hh, hh, z0, z1);
    wall(&mut t, hh, hh, hl, hh, z0, z1);
    wall(&mut t, hl, hh, hl, hl, z0, z1);

    let m = mesh::Mesh::from_triangle_soup(&t);
    std::fs::create_dir_all("fixtures")?;
    m.write_stl_ascii("fixtures/holeplate.stl")?;
    println!("wrote fixtures/holeplate.stl ({} tris, {} verts)", m.triangles.len(), m.vertices.len());
    Ok(())
}
