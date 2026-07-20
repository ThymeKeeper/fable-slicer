//! Analyse an object's XY footprint: AABB vs convex hull vs true silhouette,
//! and test whether two copies placed "facing" (interlocking L) falsely collide
//! under each method. Arg: <stl>.
use geo2d::Polygons;
use mesh::Mesh;

fn hull2d(mut pts: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    if pts.len() < 3 { return pts; }
    pts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    pts.dedup();
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0]);
    let mut lo: Vec<[f64;2]> = Vec::new();
    for &p in &pts { while lo.len()>=2 && cross(lo[lo.len()-2],lo[lo.len()-1],p)<=0.0 { lo.pop(); } lo.push(p); }
    let mut hi: Vec<[f64;2]> = Vec::new();
    for &p in pts.iter().rev() { while hi.len()>=2 && cross(hi[hi.len()-2],hi[hi.len()-1],p)<=0.0 { hi.pop(); } hi.push(p); }
    lo.pop(); hi.pop(); lo.extend(hi); lo
}

// True XY silhouette (shadow): slice COARSELY and union every layer — captures the
// full projection (no false negatives) without the cost of a fine slice.
fn silhouette(m: &Mesh) -> Polygons {
    let (_, _, _, _, minz, maxz) = {
        let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN, f64::MAX, f64::MIN);
        for v in &m.vertices {
            b.4 = b.4.min(v[2]); b.5 = b.5.max(v[2]);
        }
        b
    };
    let hgt = (maxz - minz).max(0.2);
    let lh = (hgt / 16.0).clamp(0.5, 5.0); // ~16 coarse slices
    let layers = engine::slice_mesh(m, engine::SliceParams { layer_height_mm: lh, first_layer_height_mm: lh });
    let mut acc = Polygons::new();
    for l in &layers {
        acc = geo2d::union(&acc, &l.polygons);
    }
    acc
}

// Direct XY silhouette: union every triangle's XY projection. No slicing.
fn project_union(m: &Mesh) -> Polygons {
    let mut tris = Polygons::new();
    for t in &m.triangles {
        let p: Vec<geo2d::Point> = t.iter().map(|&i| {
            let v = m.vertices[i as usize];
            geo2d::Point::from_mm(v[0], v[1])
        }).collect();
        // skip degenerate (near-zero-area) projections
        let a = (p[1].x_mm()-p[0].x_mm())*(p[2].y_mm()-p[0].y_mm()) - (p[2].x_mm()-p[0].x_mm())*(p[1].y_mm()-p[0].y_mm());
        if a.abs() < 1e-6 { continue; }
        let mut poly = Polygons::new();
        poly.contours.push(geo2d::Contour::new(p));
        tris = geo2d::union(&tris, &poly);
    }
    tris
}

fn poly_from_hull(h: &[[f64; 2]]) -> Polygons {
    let mut p = Polygons::new();
    p.contours.push(geo2d::Contour::new(h.iter().map(|q| geo2d::Point::from_mm(q[0], q[1])).collect()));
    p
}

fn translate(p: &Polygons, dx: f64, dy: f64) -> Polygons {
    let mut o = Polygons::new();
    for c in &p.contours {
        o.contours.push(geo2d::Contour::new(c.points.iter().map(|pt| geo2d::Point::from_mm(pt.x_mm()+dx, pt.y_mm()+dy)).collect()));
    }
    o
}

fn main() {
    let stl = std::env::args().nth(1).expect("stl");
    let m = Mesh::load_stl(&stl).expect("load");
    let (minx, miny, maxx, maxy) = m.xy_bounds().unwrap();
    let (w, h) = (maxx - minx, maxy - miny);
    println!("bbox {:.1} x {:.1} mm", w, h);

    // Time the true-footprint approaches.
    let t0 = std::time::Instant::now();
    let sil = silhouette(&m);
    let sil_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let t1 = std::time::Instant::now();
    let sil2 = project_union(&m);
    let proj_ms = t1.elapsed().as_secs_f64() * 1000.0;
    println!("silhouette via slice: {sil_ms:.1}ms   via projected-triangle union: {proj_ms:.1}ms  ({} tris)", m.triangles.len());
    let _ = sil2;
    let sil_area: f64 = sil.contours.iter().map(|c| c.area_mm2()).sum::<f64>().abs();
    let verts: Vec<[f64; 2]> = m.vertices.iter().map(|v| [v[0], v[1]]).collect();
    let hull = hull2d(verts);
    let hull_area = poly_from_hull(&hull).contours.iter().map(|c| c.area_mm2()).sum::<f64>().abs();
    let bbox_area = w * h;
    println!("footprint area (silhouette) = {:.0} mm²", sil_area);
    println!("convex-hull area            = {:.0} mm²", hull_area);
    println!("bbox area                   = {:.0} mm²", bbox_area);
    println!("→ concave? hull/footprint = {:.2}x  (L-shape if >>1)", hull_area / sil_area.max(1.0));

    // Two copies placed "facing" so their bboxes overlap: copy B shifted by ~55% of
    // the bbox in +x and rotated 180° (mimicking the user's nested arrangement).
    let hull_poly = poly_from_hull(&hull);
    // rotate silhouette/hull 180 about center for copy B
    let (cx, cy) = ((minx+maxx)/2.0, (miny+maxy)/2.0);
    let rot180 = |p: &Polygons| -> Polygons {
        let mut o = Polygons::new();
        for c in &p.contours { o.contours.push(geo2d::Contour::new(c.points.iter().map(|pt| geo2d::Point::from_mm(2.0*cx-pt.x_mm(), 2.0*cy-pt.y_mm())).collect())); }
        o
    };
    // SAME-ORIENTATION nested duplicates (rot 0) — a plain "Duplicate" shifted into
    // the original's concave corner. THIS is the user's case (Duplicate keeps rot).
    println!("--- SAME orientation (plain duplicate, no rotation) ---");
    for &(sx, sy) in &[(0.45, 0.0), (0.30, 0.0), (0.20, 0.20), (0.15, 0.15), (0.10, 0.10)] {
        let (dx, dy) = (w * sx, h * sy);
        let a_bb = [minx, miny, maxx, maxy];
        let b_bb = [minx+dx, miny+dy, maxx+dx, maxy+dy];
        let aabb_hit = !(a_bb[2] <= b_bb[0] || b_bb[2] <= a_bb[0] || a_bb[3] <= b_bb[1] || b_bb[3] <= a_bb[1]);
        let hull_hit = !geo2d::intersection(&hull_poly, &translate(&hull_poly, dx, dy)).is_empty();
        let sil_hit = !geo2d::intersection(&sil, &translate(&sil, dx, dy)).is_empty();
        let flag = if hull_hit != sil_hit { "  <-- HULL false-positive (silhouette needed)" } else { "" };
        println!("shift ({:>2.0}%,{:>2.0}%):  AABB={}  HULL={}  SILHOUETTE={}{}", sx*100.0, sy*100.0, aabb_hit, hull_hit, sil_hit, flag);
    }
    // 180°-rotated (facing) arrangements for comparison.
    println!("--- 180° rotated (facing) ---");
    for &(sx, sy) in &[(0.45, 0.0), (0.20, 0.20), (0.10, 0.10)] {
        let (dx, dy) = (w * sx, h * sy);
        let hull_hit = !geo2d::intersection(&hull_poly, &translate(&rot180(&hull_poly), dx, dy)).is_empty();
        let sil_hit = !geo2d::intersection(&sil, &translate(&rot180(&sil), dx, dy)).is_empty();
        let flag = if hull_hit != sil_hit { "  <-- HULL false-positive" } else { "" };
        println!("shift ({:>2.0}%,{:>2.0}%):  HULL={}  SILHOUETTE={}{}", sx*100.0, sy*100.0, hull_hit, sil_hit, flag);
    }
}
