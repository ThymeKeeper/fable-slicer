//! Overhang / bridge test suite — one STL with several features so you can see how
//! each is handled (try support = none / grid / arc):
//!   1. Flat bridges over 3, 6, 10 mm gaps (small → lines, large → arcs)
//!   2. Flat cantilever (an arm out one side)
//!   3. Sloped overhang (a leaning post)
//!   4. Arched bridge (a semicircular opening — continuously varying overhang)
//!   5. Corner-wrapping overhang (a cap overhanging a central pillar on all sides)
//! `cargo run -p mesh --example gen_overhang_suite` → fixtures/overhang_suite.stl
//! (Winding is irrelevant — the slicer re-orients per layer by nesting.)

type V = [f64; 3];

fn cross2(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

fn pt_in_tri(p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    let (d1, d2, d3) = (cross2(p, a, b), cross2(p, b, c), cross2(p, c, a));
    let neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(neg && pos)
}

/// Ear-clip a simple polygon (made CCW first) into triangles (index triples).
fn triangulate(poly: &mut Vec<(f64, f64)>) -> Vec<[usize; 3]> {
    let area: f64 = (0..poly.len())
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            a.0 * b.1 - b.0 * a.1
        })
        .sum();
    if area < 0.0 {
        poly.reverse();
    }
    let mut idx: Vec<usize> = (0..poly.len()).collect();
    let mut tris = Vec::new();
    let mut guard = 0;
    while idx.len() > 3 && guard < 10_000 {
        guard += 1;
        let m = idx.len();
        let mut ear = None;
        for i in 0..m {
            let (a, b, c) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            if cross2(poly[a], poly[b], poly[c]) <= 0.0 {
                continue; // reflex vertex
            }
            if idx.iter().all(|&k| k == a || k == b || k == c || !pt_in_tri(poly[k], poly[a], poly[b], poly[c])) {
                ear = Some(i);
                break;
            }
        }
        match ear {
            Some(i) => {
                let m = idx.len();
                tris.push([idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]]);
                idx.remove(i);
            }
            None => break,
        }
    }
    if idx.len() == 3 {
        tris.push([idx[0], idx[1], idx[2]]);
    }
    tris
}

/// Extrude an XZ profile along Y, placed at `xoff`.
fn extrude(t: &mut Vec<[V; 3]>, mut profile: Vec<(f64, f64)>, xoff: f64, y0: f64, y1: f64) {
    let v = |p: (f64, f64), y: f64| -> V { [p.0 + xoff, y, p.1] };
    let n = profile.len();
    let tris = triangulate(&mut profile);
    for tr in &tris {
        let (a, b, c) = (profile[tr[0]], profile[tr[1]], profile[tr[2]]);
        t.push([v(a, y0), v(b, y0), v(c, y0)]); // front
        t.push([v(c, y1), v(b, y1), v(a, y1)]); // back
    }
    for i in 0..n {
        let (p, q) = (profile[i], profile[(i + 1) % n]);
        t.push([v(p, y0), v(q, y0), v(q, y1)]);
        t.push([v(p, y0), v(q, y1), v(p, y1)]);
    }
}

fn add_box(t: &mut Vec<[V; 3]>, lo: V, hi: V) {
    let [x0, y0, z0] = lo;
    let [x1, y1, z1] = hi;
    let mut q = |a: V, b: V, c: V, d: V| {
        t.push([a, b, c]);
        t.push([a, c, d]);
    };
    q([x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0]);
    q([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]);
    q([x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]);
    q([x0, y1, z0], [x1, y1, z0], [x1, y1, z1], [x0, y1, z1]);
    q([x0, y0, z0], [x0, y1, z0], [x0, y1, z1], [x0, y0, z1]);
    q([x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]);
}

fn main() -> std::io::Result<()> {
    let mut t: Vec<[V; 3]> = Vec::new();
    let (r1a, r1b) = (0.0, 18.0); // row 1 depth
    let (r2a, r2b) = (28.0, 46.0); // row 2 depth

    // 1) Flat bridges over 3 / 6 / 10 mm gaps (comb of pillars + top slab).
    extrude(
        &mut t,
        vec![
            (0.0, 0.0), (4.0, 0.0), (4.0, 12.0),
            (7.0, 12.0), (7.0, 0.0), (11.0, 0.0), (11.0, 12.0), // gap 3
            (17.0, 12.0), (17.0, 0.0), (21.0, 0.0), (21.0, 12.0), // gap 6
            (31.0, 12.0), (31.0, 0.0), (35.0, 0.0), (35.0, 15.0), // gap 10
            (0.0, 15.0),
        ],
        0.0,
        r1a,
        r1b,
    );

    // 2) Flat cantilever: a post with an arm out the right side.
    extrude(
        &mut t,
        vec![(0.0, 0.0), (8.0, 0.0), (8.0, 15.0), (30.0, 15.0), (30.0, 18.0), (8.0, 18.0), (8.0, 20.0), (0.0, 20.0)],
        43.0,
        r1a,
        r1b,
    );

    // 3) Sloped overhang: a post leaning ~56° off vertical (its right underside).
    extrude(&mut t, vec![(0.0, 0.0), (8.0, 0.0), (38.0, 20.0), (30.0, 20.0)], 82.0, r1a, r1b);

    // 4) Arched bridge: a 20mm semicircular opening (vertical at the feet → flat at
    //    the crown), so the overhang angle varies continuously.
    {
        let mut p = vec![(0.0, 0.0)];
        let steps = 28;
        for k in 0..=steps {
            let theta = std::f64::consts::PI * (1.0 - k as f64 / steps as f64); // π → 0
            p.push((15.0 + 10.0 * theta.cos(), 10.0 * theta.sin())); // (5,0) → (15,10) → (25,0)
        }
        p.extend([(30.0, 0.0), (30.0, 22.0), (0.0, 22.0)]);
        extrude(&mut t, p, 0.0, r2a, r2b);
    }

    // 5) Corner-wrapping overhang: a cap overhanging a central pillar on all sides.
    let xo = 40.0;
    add_box(&mut t, [xo + 8.0, r2a + 6.0, 0.0], [xo + 18.0, r2a + 12.0, 12.0]); // pillar
    add_box(&mut t, [xo, r2a, 12.0], [xo + 26.0, r2b, 15.0]); // overhanging cap

    let m = mesh::Mesh::from_triangle_soup(&t);
    std::fs::create_dir_all("fixtures")?;
    m.write_stl_ascii("fixtures/overhang_suite.stl")?;
    let bb = (m.vertices.iter().fold([f64::MAX; 3], |a, v| [a[0].min(v[0]), a[1].min(v[1]), a[2].min(v[2])]),
              m.vertices.iter().fold([f64::MIN; 3], |a, v| [a[0].max(v[0]), a[1].max(v[1]), a[2].max(v[2])]));
    println!("wrote fixtures/overhang_suite.stl ({} tris) bounds {:?}..{:?}", m.triangles.len(), bb.0, bb.1);
    Ok(())
}
