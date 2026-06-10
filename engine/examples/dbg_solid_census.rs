//! Census of solid-fill path lengths across a full Benchy plan — verifies the
//! junk-solid filter (no micro dabs/loops; area reallocated to sparse).
fn main() {
    let mesh = mesh::Mesh::load_stl("fixtures/benchy.stl").unwrap();
    let settings = config::Settings::default();
    let plans = engine::generate(&mesh, &settings);
    let mut buckets = [0usize; 5]; // <0.7, <1.5, <3, <10, >=10 mm
    let mut total = 0usize;
    let mut total_len = 0.0f64;
    let mut infill_n = 0usize;
    for plan in &plans {
        for p in &plan.paths {
            match p.kind {
                engine::PathKind::Solid => {
                    let mut len: f64 = p
                        .points
                        .windows(2)
                        .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
                        .sum();
                    if p.closed && p.points.len() > 2 {
                        let (a, b) = (p.points[0], *p.points.last().unwrap());
                        len += (a.x_mm() - b.x_mm()).hypot(a.y_mm() - b.y_mm());
                    }
                    let b = if len < 0.7 {
                        0
                    } else if len < 1.5 {
                        1
                    } else if len < 3.0 {
                        2
                    } else if len < 10.0 {
                        3
                    } else {
                        4
                    };
                    buckets[b] += 1;
                    total += 1;
                    total_len += len;
                }
                engine::PathKind::Infill => infill_n += 1,
                _ => {}
            }
        }
    }
    println!(
        "solid paths: {total} ({total_len:.0}mm) | <0.7mm: {} | 0.7-1.5: {} | 1.5-3: {} | 3-10: {} | >=10: {} | sparse paths: {infill_n}",
        buckets[0], buckets[1], buckets[2], buckets[3], buckets[4]
    );
}
