//! Does grid support fire for a thin overhanging WALL vs a solid overhang?
//! Builds two meshes, slices each with grid support, and reports overhang area
//! + Support path counts. Env: ANGLE (support overhang deg, default 45).
use mesh::Mesh;

// A thin slab (thickness `t` in x, length `l` in y) that leans +x by `d` over
// height `h` — an overhanging wall whose lean from vertical is atan(d/h).
fn leaning_wall(t: f64, l: f64, d: f64, h: f64) -> Mesh {
    // 8 corners: bottom at x∈[0,t], top sheared to x∈[d,d+t].
    let v = [
        [0.0, 0.0, 0.0], [t, 0.0, 0.0], [t, l, 0.0], [0.0, l, 0.0], // bottom
        [d, 0.0, h], [d + t, 0.0, h], [d + t, l, h], [d, l, h],     // top
    ];
    box_tris(&v)
}

// A solid wedge: a right-triangular prism (in x–z), length `l` in y. Its slanted
// top face overhangs — the classic solid overhang. Base x∈[0,base] at z=0, apex
// line at x=0 rising to z=h; the hypotenuse from (base,0) to (0,h) is the
// overhanging underside if printed apex-down... here we make the OVERHANG the top.
fn solid_overhang(base: f64, l: f64, h: f64) -> Mesh {
    // A slab that starts narrow at the bottom and cantilevers out at the top:
    // bottom footprint x∈[0,2], top x∈[0,base] at z=h (a solid leaning mass).
    let v = [
        [0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, l, 0.0], [0.0, l, 0.0],
        [0.0, 0.0, h], [base, 0.0, h], [base, l, h], [0.0, l, h],
    ];
    box_tris(&v)
}

// 12 triangles for a hexahedron given 8 corners (0-3 bottom CCW, 4-7 top CCW).
fn box_tris(v: &[[f64; 3]; 8]) -> Mesh {
    let q = |a: usize, b: usize, c: usize, d: usize| {
        [[v[a], v[b], v[c]], [v[a], v[c], v[d]]]
    };
    let mut t = Vec::new();
    t.extend(q(0, 3, 2, 1)); // bottom (down)
    t.extend(q(4, 5, 6, 7)); // top (up)
    t.extend(q(0, 1, 5, 4)); // y=0
    t.extend(q(2, 3, 7, 6)); // y=l
    t.extend(q(1, 2, 6, 5)); // +x
    t.extend(q(3, 0, 4, 7)); // -x (the overhanging underside for a +x lean)
    Mesh::from_triangle_soup(&t)
}

fn report(name: &str, m: &Mesh, s: &config::Settings) {
    let t0 = std::time::Instant::now();
    let layers = engine::generate(m, s);
    let ms = t0.elapsed().as_millis();
    print!("[{ms:>5}ms] ");
    let _ = ms;
    let support: usize = layers.iter().map(|l| l.paths.iter().filter(|p| p.kind == engine::PathKind::Support).count()).sum();
    let oh_walls: usize = layers.iter().map(|l| l.paths.iter().filter(|p| p.kind == engine::PathKind::OverhangWall).count()).sum();
    println!("{name:22} layers={:3}  Support paths={support:4}  OverhangWall={oh_walls}", layers.len());
}

fn main() {
    if let Ok(path) = std::env::var("WRITE_STL") {
        // A leaning thin wall, 63deg from vertical, for the offscreen renderer.
        leaning_wall(1.5, 30.0, 40.0, 20.0).write_stl_ascii(&path).unwrap();
        eprintln!("wrote {path}");
        return;
    }
    let angle: f64 = std::env::var("ANGLE").ok().and_then(|v| v.parse().ok()).unwrap_or(45.0);
    let mut s = config::Settings::default();
    s.skirt_loops = 0;
    s.support_mode = config::SupportMode::Grid;
    s.support_overhang_angle_deg = angle;
    s.auto_center_on_bed = false;
    println!("support_overhang_angle_deg = {angle} (from vertical)  layer_h={}", s.layer_height_mm);
    // Thin walls leaning at increasing angles from vertical (advance/layer at 0.2mm LH):
    report("thin wall 45deg", &leaning_wall(1.5, 20.0, 20.0, 20.0), &s); // atan(1)=45 from vert
    report("thin wall 63deg", &leaning_wall(1.5, 20.0, 40.0, 20.0), &s); // atan(2)=63
    report("thin wall 76deg", &leaning_wall(1.5, 20.0, 80.0, 20.0), &s); // atan(4)=76, advance 0.8mm/layer
    report("solid overhang", &solid_overhang(30.0, 20.0, 20.0), &s);
    // Abrupt cantilever: a 4mm post, then a slab that juts out 16mm in +y in ONE
    // layer — a wide overhang that MUST survive the open. Control that support works.
    {
        let mut t = Vec::new();
        push_box(&mut t, [0.0, 0.0, 0.0], [4.0, 4.0, 10.0]);   // post
        push_box(&mut t, [0.0, 0.0, 10.0], [4.0, 20.0, 12.0]); // cantilevered slab
        report("abrupt cantilever", &Mesh::from_triangle_soup(&t), &s);
    }
    // False-positive guards: vertical walls and a self-supporting (steeper than
    // threshold) slope must get ZERO support.
    // Big/tall gradual overhang — stresses the accumulation (many layers, wide
    // sweeping overhang region). This is the shape class that hangs the slicer.
    report("BIG lean 300 layers", &leaning_wall(2.0, 80.0, 90.0, 60.0), &s);
    report("cube (vertical)", &Mesh::cube(20.0), &s);
    // A wall leaning only 30deg from vertical (steeper than the 45 threshold) —
    // self-supporting, should stay 0 at ANGLE>=31.
    report("steep wall 30deg", &leaning_wall(1.5, 20.0, 20.0 * (30.0_f64.to_radians().tan()), 20.0), &s);
}

fn push_box(t: &mut Vec<[[f64; 3]; 3]>, lo: [f64; 3], hi: [f64; 3]) {
    let v = [
        [lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]], [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
        [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]], [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]],
    ];
    let q = |a: usize, b: usize, c: usize, d: usize| [[v[a], v[b], v[c]], [v[a], v[c], v[d]]];
    for tri in [q(0,3,2,1), q(4,5,6,7), q(0,1,5,4), q(2,3,7,6), q(1,2,6,5), q(3,0,4,7)] {
        t.extend(tri);
    }
}
