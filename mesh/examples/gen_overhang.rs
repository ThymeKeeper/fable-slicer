//! Generate an L-step overhang test part: a 10mm-wide base column with a 40mm-wide
//! top arm cantilevered over open air (the x∈[10,40] underside at z=5 is a flat
//! overhang needing support). Extruded 20mm in y.
//! `cargo run -p mesh --example gen_overhang` → fixtures/overhang.stl

type V = [f64; 3];

fn quad(t: &mut Vec<[V; 3]>, a: V, b: V, c: V, d: V) {
    t.push([a, b, c]);
    t.push([a, c, d]);
}

fn main() -> std::io::Result<()> {
    let d = 20.0; // depth in y
    let mut t: Vec<[V; 3]> = Vec::new();

    // Front (y=0) and back (y=d) faces, tiled as two rects: left column + top arm.
    for &y in &[0.0, d] {
        quad(&mut t, [0.0, y, 0.0], [10.0, y, 0.0], [10.0, y, 10.0], [0.0, y, 10.0]);
        quad(&mut t, [10.0, y, 5.0], [40.0, y, 5.0], [40.0, y, 10.0], [10.0, y, 10.0]);
    }

    // Walls around the L profile. The (10,5)->(40,5) edge is the overhang underside.
    let prof = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 5.0),
        (40.0, 5.0),
        (40.0, 10.0),
        (10.0, 10.0),
        (0.0, 10.0),
    ];
    for i in 0..prof.len() {
        let (x0, z0) = prof[i];
        let (x1, z1) = prof[(i + 1) % prof.len()];
        quad(&mut t, [x0, 0.0, z0], [x1, 0.0, z1], [x1, d, z1], [x0, d, z0]);
    }

    let m = mesh::Mesh::from_triangle_soup(&t);
    std::fs::create_dir_all("fixtures")?;
    m.write_stl_ascii("fixtures/overhang.stl")?;
    println!("wrote fixtures/overhang.stl ({} tris, {} verts)", m.triangles.len(), m.vertices.len());
    Ok(())
}
