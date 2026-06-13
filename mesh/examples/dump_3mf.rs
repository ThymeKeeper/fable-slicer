//! Load 3MF files and report what's inside — a compatibility smoke tool.
//! Usage: cargo run -p mesh --example dump_3mf -- file.3mf [more.3mf ...]

fn main() {
    for path in std::env::args().skip(1) {
        match mesh::load_3mf(&path) {
            Ok(items) => {
                let tris: usize = items.iter().map(|i| i.mesh.triangles.len()).sum();
                println!("{path}: {} object(s), {tris} triangles", items.len());
                for it in &items {
                    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
                    for v in &it.mesh.vertices {
                        for k in 0..3 {
                            lo[k] = lo[k].min(v[k]);
                            hi[k] = hi[k].max(v[k]);
                        }
                    }
                    println!(
                        "  '{}': {} tris, {:.1} x {:.1} x {:.1} mm",
                        it.name,
                        it.mesh.triangles.len(),
                        hi[0] - lo[0],
                        hi[1] - lo[1],
                        hi[2] - lo[2]
                    );
                }
            }
            Err(e) => println!("{path}: FAILED — {e}"),
        }
    }
}
