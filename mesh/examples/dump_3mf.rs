//! Load 3MF files and report what's inside — a compatibility smoke tool.
//! Usage: cargo run -p mesh --example dump_3mf -- file.3mf [more.3mf ...]

fn bbox(m: &mesh::Mesh) -> ([f64; 3], [f64; 3]) {
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for v in &m.vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(v[k]);
            hi[k] = hi[k].max(v[k]);
        }
    }
    (lo, hi)
}

fn main() {
    for path in std::env::args().skip(1) {
        match mesh::load_3mf(&path) {
            Ok(items) => {
                let tris: usize =
                    items.iter().flat_map(|i| &i.parts).map(|p| p.mesh.triangles.len()).sum();
                println!("{path}: {} object(s), {tris} triangles", items.len());
                for it in &items {
                    let merged = it.merged();
                    let (lo, hi) = bbox(&merged);
                    println!(
                        "  '{}': {} part(s), {} tris, {:.1} x {:.1} x {:.1} mm",
                        it.name,
                        it.parts.len(),
                        merged.triangles.len(),
                        hi[0] - lo[0],
                        hi[1] - lo[1],
                        hi[2] - lo[2]
                    );
                    for p in &it.parts {
                        let (lo, hi) = bbox(&p.mesh);
                        let extruder =
                            p.extruder.map(|e| format!(" extruder={e}")).unwrap_or_default();
                        println!(
                            "    '{}': {} tris, {:.1} x {:.1} x {:.1} mm{extruder}",
                            p.name,
                            p.mesh.triangles.len(),
                            hi[0] - lo[0],
                            hi[1] - lo[1],
                            hi[2] - lo[2]
                        );
                    }
                }
            }
            Err(e) => println!("{path}: FAILED — {e}"),
        }
    }
}
