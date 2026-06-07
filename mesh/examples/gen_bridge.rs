//! Generate a portal/bridge test part: two 5mm legs with a 5mm gap between them,
//! joined by a top slab. The slab's underside over the gap is a 5mm bridge
//! (supported on both legs) — narrow enough for straight bridge lines.
//! `cargo run -p mesh --example gen_bridge` → fixtures/bridge.stl

type V = [f64; 3];

fn quad(t: &mut Vec<[V; 3]>, a: V, b: V, c: V, d: V) {
    t.push([a, b, c]);
    t.push([a, c, d]);
}

fn main() -> std::io::Result<()> {
    let d = 20.0; // depth in y
    let mut t: Vec<[V; 3]> = Vec::new();

    // Front (y=0) and back faces, tiled as left leg + right leg + top slab.
    for &y in &[0.0, d] {
        quad(&mut t, [0.0, y, 0.0], [5.0, y, 0.0], [5.0, y, 13.0], [0.0, y, 13.0]);
        quad(&mut t, [10.0, y, 0.0], [15.0, y, 0.0], [15.0, y, 13.0], [10.0, y, 13.0]);
        quad(&mut t, [5.0, y, 10.0], [10.0, y, 10.0], [10.0, y, 13.0], [5.0, y, 13.0]);
    }

    // Walls around the portal profile. (5,10)->(10,10) is the bridge underside.
    let prof = [
        (0.0, 0.0),
        (5.0, 0.0),
        (5.0, 10.0),
        (10.0, 10.0),
        (10.0, 0.0),
        (15.0, 0.0),
        (15.0, 13.0),
        (10.0, 13.0),
        (5.0, 13.0),
        (0.0, 13.0),
    ];
    for i in 0..prof.len() {
        let (x0, z0) = prof[i];
        let (x1, z1) = prof[(i + 1) % prof.len()];
        quad(&mut t, [x0, 0.0, z0], [x1, 0.0, z1], [x1, d, z1], [x0, d, z0]);
    }

    let m = mesh::Mesh::from_triangle_soup(&t);
    std::fs::create_dir_all("fixtures")?;
    m.write_stl_ascii("fixtures/bridge.stl")?;
    println!("wrote fixtures/bridge.stl ({} tris, {} verts)", m.triangles.len(), m.vertices.len());
    Ok(())
}
