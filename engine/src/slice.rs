//! Plane/mesh intersection and contour stitching.

use std::collections::{HashMap, HashSet};

use geo2d::{normalize_positive, Contour, Point, Polygons};
use mesh::{Mesh, Vec3};
use rayon::prelude::*;

/// Parameters controlling how the mesh is sliced.
#[derive(Clone, Copy, Debug)]
pub struct SliceParams {
    pub layer_height_mm: f64,
    pub first_layer_height_mm: f64,
}

impl Default for SliceParams {
    fn default() -> Self {
        Self { layer_height_mm: 0.2, first_layer_height_mm: 0.2 }
    }
}

/// One sliced layer.
#[derive(Clone, Debug)]
pub struct Layer {
    pub index: usize,
    /// World-space height the layer was sampled at.
    pub z_mm: f64,
    /// This layer's thickness (the first layer may differ).
    pub height_mm: f64,
    /// Nozzle Z when printing this layer (top of the layer, model bottom = 0).
    pub print_z_mm: f64,
    pub polygons: Polygons,
}

/// Slice a mesh into layers, honoring a distinct first-layer height.
///
/// Each layer is sampled at its vertical midpoint, which avoids landing on flat
/// top/bottom facets. `print_z_mm` accumulates layer thicknesses with the model
/// bottom resting on the bed (z = 0).
///
/// Triangles are bucketed by the layer range their z-span covers, so each layer
/// only visits triangles that can actually cross it (instead of the whole mesh),
/// and the layers are then sliced in parallel.
pub fn slice_mesh(mesh: &Mesh, params: SliceParams) -> Vec<Layer> {
    let Some((zmin, zmax)) = mesh.z_bounds() else {
        return Vec::new();
    };
    slice_mesh_on(mesh, &layer_grid(zmin, zmax, params))
}

/// Plan the layer grid (z's, heights, print z's) for the band `zmin..zmax`,
/// with empty polygons. Separated from slicing so multiple part meshes can
/// share ONE grid (aligned indices and print z's) over their union z-range.
pub(crate) fn layer_grid(zmin: f64, zmax: f64, params: SliceParams) -> Vec<Layer> {
    let mut metas: Vec<Layer> = Vec::new();
    let mut i = 0usize;
    let mut bottom = zmin; // world-z of the current layer's bottom face
    loop {
        let h = if i == 0 {
            params.first_layer_height_mm
        } else {
            params.layer_height_mm
        };
        let z = bottom + h * 0.5;
        if z >= zmax {
            break;
        }
        metas.push(Layer {
            index: i,
            z_mm: z,
            height_mm: h,
            print_z_mm: (bottom - zmin) + h,
            polygons: Polygons::new(),
        });
        bottom += h;
        i += 1;
    }
    metas
}

/// Slice one mesh on a pre-planned grid. A layer whose plane misses the mesh's
/// z-range just comes back empty. The written-back `z_mm` is the (possibly
/// vertex-nudged) plane actually used for THIS mesh — per mesh, so never key
/// cross-part logic on z_mm equality; `print_z_mm` is the shared truth.
pub(crate) fn slice_mesh_on(mesh: &Mesh, grid: &[Layer]) -> Vec<Layer> {
    let mut metas = grid.to_vec();
    let zs: Vec<f64> = metas.iter().map(|m| m.z_mm).collect();
    for (layer, (z, polys)) in metas.iter_mut().zip(slice_many(mesh, &zs)) {
        layer.z_mm = z; // the (possibly vertex-nudged) plane actually used
        layer.polygons = polys;
    }
    metas
}

/// Slice the mesh at each plane in `zs` (must be ascending). Returns, per
/// plane, the (possibly vertex-nudged) z actually used and the stitched
/// polygons. Triangles are bucketed by the band of planes their z-span
/// crosses, so each plane only visits candidate triangles, and the planes are
/// sliced in parallel.
pub(crate) fn slice_many(mesh: &Mesh, zs: &[f64]) -> Vec<(f64, Polygons)> {
    if zs.is_empty() || mesh.triangles.is_empty() {
        return zs.iter().map(|&z| (z, Polygons::new())).collect();
    }

    // Sorted unique vertex z's — lets each plane nudge off coincident vertices
    // with a binary search instead of scanning every vertex.
    let mut vert_zs: Vec<f64> = mesh.vertices.iter().map(|v| v[2]).collect();
    vert_zs.sort_unstable_by(f64::total_cmp);
    vert_zs.dedup();
    let zs: Vec<f64> = zs.iter().map(|&z| nudge_off_vertices(&vert_zs, z)).collect();

    // Bucket triangle indices by the band of planes their z-span crosses.
    // `band` > 1 caps bucket memory when many triangles span many planes (tall
    // thin meshes); each plane then filters its band's list by exact z-span.
    let tri_spans: Vec<(f64, f64)> = (0..mesh.triangles.len())
        .map(|t| {
            let [a, b, c] = mesh.triangle(t);
            (a[2].min(b[2]).min(c[2]), a[2].max(b[2]).max(c[2]))
        })
        .collect();
    let total_entries: usize = tri_spans
        .iter()
        .map(|&(lo, hi)| {
            let a = zs.partition_point(|&z| z < lo);
            let b = zs.partition_point(|&z| z <= hi);
            b - a
        })
        .sum();
    const ENTRY_CAP: usize = 8_000_000;
    let band = (total_entries / ENTRY_CAP + 1).max(1);
    let n_bands = zs.len().div_ceil(band);
    let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); n_bands];
    for (t, &(lo, hi)) in tri_spans.iter().enumerate() {
        let a = zs.partition_point(|&z| z < lo);
        let b = zs.partition_point(|&z| z <= hi);
        for bi in (a / band)..=((b.saturating_sub(1)) / band).min(n_bands - 1) {
            if a < b {
                buckets[bi].push(t as u32);
            }
        }
    }

    zs.par_iter()
        .enumerate()
        .map(|(i, &z)| (z, slice_at(mesh, &tri_spans, &buckets[i / band], z)))
        .collect()
}

/// Intersect the bucketed triangles with one horizontal plane and stitch.
///
/// Segments come out DIRECTED from the triangle winding (material to the
/// left ⇒ outers CCW, holes CW straight from the geometry), and the layer is
/// normalized under the positive fill rule — so a mesh whose surfaces pass
/// through each other (a chamfer punching through a wall: topologically
/// manifold, geometrically self-intersecting) resolves to the true material
/// region instead of nesting-parity garbage. A mesh with enough flipped
/// facets that directed walking can't close its loops falls back, per layer,
/// to the tolerant undirected stitcher.
fn slice_at(mesh: &Mesh, tri_spans: &[(f64, f64)], bucket: &[u32], z: f64) -> Polygons {
    let mut segments: Vec<(Point, Point)> = Vec::new();
    for &t in bucket {
        let (lo, hi) = tri_spans[t as usize];
        if z < lo || z > hi {
            continue; // in the band, but not crossing this layer
        }
        let [a, b, c] = mesh.triangle(t as usize);
        if let Some(seg) = intersect_triangle(a, b, c, z) {
            segments.push(seg);
        }
    }
    let total = segments.iter().filter(|(p, q)| p != q).count();
    let (mut polys, consumed) = stitch_directed(&segments);
    // Nearly-clean meshes close nearly everything; a junk mesh (flipped
    // facets break the one-in-one-out walk) reverts to the old parity path.
    if total > 0 && (consumed as f64) < 0.9 * total as f64 {
        polys = stitch(segments);
    } else if !windings_match_nesting(&polys) && !contours_cross(&polys) {
        // Directed stitching closed everything, yet the facet-derived windings
        // contradict containment parity. When no two contours CROSS, parity is
        // authoritative and the windings are mesh damage: hole side-walls
        // flipped consistently enough to close (the holeplate case) stitch
        // into a CCW "hole" that the positive fill rule would silently union
        // away — the bore prints solid. Re-orient by nesting, exactly like the
        // fallback stitcher. Crossing contours (a chamfer punching through a
        // wall — self-intersecting but honestly wound) keep their directed
        // windings: parity is meaningless there, and winding + positive fill
        // is what resolves them to the true material region. The one case
        // this trades away is coincidentally-nested SOLID shells relying on
        // winding-number-2 semantics — parity reads the inner shell as a
        // cavity; that mesh class is vanishingly rare next to wild STLs with
        // flipped hole walls, and it's what the parity-only path always did.
        orient_by_nesting(&mut polys);
    }
    normalize_positive(&polys)
}

/// Do the contours' given windings already agree with containment parity
/// (outers CCW at even depth, holes CW at odd)? The common case for a clean
/// mesh — checked read-only so agreement costs no re-orientation.
fn windings_match_nesting(polys: &Polygons) -> bool {
    let n = polys.contours.len();
    for i in 0..n {
        let Some(&probe) = polys.contours[i].points.first() else {
            continue;
        };
        let mut depth = 0;
        for j in 0..n {
            if i != j && polys.contours[j].contains(probe) {
                depth += 1;
            }
        }
        if polys.contours[i].is_ccw() != (depth % 2 == 0) {
            return false;
        }
    }
    true
}

/// Any two contours whose boundaries PROPERLY cross (segment interiors
/// intersecting — not endpoint touches or shared edges)? Nesting parity is
/// only well-defined for non-crossing sets; crossing loops are the signature
/// of a self-intersecting surface. Only runs on winding-mismatch layers, so
/// the O(pairs) sweep with AABB pruning never taxes a clean slice.
fn contours_cross(polys: &Polygons) -> bool {
    let cs = &polys.contours;
    let boxes: Vec<(Point, Point)> = cs
        .iter()
        .map(|c| {
            let (mut lo, mut hi) = (c.points[0], c.points[0]);
            for &p in &c.points {
                lo.x = lo.x.min(p.x);
                lo.y = lo.y.min(p.y);
                hi.x = hi.x.max(p.x);
                hi.y = hi.y.max(p.y);
            }
            (lo, hi)
        })
        .collect();
    let orient = |a: Point, b: Point, c: Point| -> i128 {
        let (abx, aby) = ((b.x - a.x) as i128, (b.y - a.y) as i128);
        let (acx, acy) = ((c.x - a.x) as i128, (c.y - a.y) as i128);
        (abx * acy - aby * acx).signum()
    };
    for i in 0..cs.len() {
        for j in (i + 1)..cs.len() {
            let ((alo, ahi), (blo, bhi)) = (boxes[i], boxes[j]);
            if ahi.x < blo.x || alo.x > bhi.x || ahi.y < blo.y || alo.y > bhi.y {
                continue;
            }
            let (a, b) = (&cs[i].points, &cs[j].points);
            for s in 0..a.len() {
                let (p1, p2) = (a[s], a[(s + 1) % a.len()]);
                for t in 0..b.len() {
                    let (q1, q2) = (b[t], b[(t + 1) % b.len()]);
                    if orient(p1, p2, q1) * orient(p1, p2, q2) < 0
                        && orient(q1, q2, p1) * orient(q1, q2, p2) < 0
                    {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// Stitch DIRECTED segments by following out-edges; loops keep the winding
/// the geometry gave them. Returns the contours and how many segments closed
/// into loops (the caller's fallback signal).
fn stitch_directed(segments: &[(Point, Point)]) -> (Polygons, usize) {
    let mut out_edges: HashMap<Point, Vec<(Point, usize)>> = HashMap::new();
    for (i, &(p, q)) in segments.iter().enumerate() {
        if p == q {
            continue;
        }
        out_edges.entry(p).or_default().push((q, i));
    }
    let mut used = vec![false; segments.len()];
    let mut polys = Polygons::new();
    let mut consumed = 0usize;
    for (i, &(seed, _)) in segments.iter().enumerate() {
        if used[i] {
            continue;
        }
        let mut current = seed;
        let mut loop_pts: Vec<Point> = Vec::new();
        let mut walked: Vec<usize> = Vec::new();
        loop {
            let next = out_edges
                .get(&current)
                .and_then(|ns| ns.iter().find(|&&(_, si)| !used[si]).copied());
            let Some((next, si)) = next else { break };
            used[si] = true;
            walked.push(si);
            loop_pts.push(current);
            current = next;
            if current == seed {
                break;
            }
        }
        if current == seed && loop_pts.len() >= 3 {
            consumed += walked.len();
            polys.push(Contour::new(loop_pts));
        }
        // A walk that dead-ended keeps its segments marked used — they are
        // damage either way; the caller's ratio test decides the fallback.
    }
    (polys, consumed)
}

/// Nudge the slice plane by a tiny amount while it coincides with a vertex, so no
/// triangle has a vertex *exactly* on the plane (which would make the
/// above/below classification ambiguous). `vert_zs` is sorted and deduped.
///
/// Walks forward from the first candidate vertex (the index only ever advances,
/// so float rounding in `v + EPS` can't re-trigger the same vertex and loop).
fn nudge_off_vertices(vert_zs: &[f64], z: f64) -> f64 {
    const EPS: f64 = 1.0e-6;
    let mut z = z;
    let mut i = vert_zs.partition_point(|&v| v < z - EPS);
    while let Some(&v) = vert_zs.get(i) {
        if v - z >= EPS {
            break; // next vertex is clearly above the (possibly bumped) plane
        }
        if (v - z).abs() < EPS {
            z = v + EPS; // collide → move just past this vertex, keep walking
        }
        i += 1;
    }
    z
}

/// Intersect a single triangle with plane `z`, returning the cut segment if
/// the triangle straddles the plane. The segment is DIRECTED with material on
/// its left (walk direction = ẑ × n for the triangle's outward normal n), so
/// stitched outers come out CCW and holes CW straight from the geometry —
/// the winding the positive-fill normalization needs to resolve
/// self-intersecting surfaces correctly.
fn intersect_triangle(a: Vec3, b: Vec3, c: Vec3, z: f64) -> Option<(Point, Point)> {
    let verts = [a, b, c];
    let above = [a[2] > z, b[2] > z, c[2] > z];
    let n_above = above.iter().filter(|&&x| x).count();
    if n_above == 0 || n_above == 3 {
        return None; // entirely on one side
    }

    // The "lone" vertex is the one alone on its side; the two crossing edges both
    // start at it.
    let lone = if n_above == 1 {
        above.iter().position(|&x| x).unwrap()
    } else {
        above.iter().position(|&x| !x).unwrap()
    };
    let o1 = (lone + 1) % 3;
    let o2 = (lone + 2) % 3;

    let p1 = lerp_to_z(verts[lone], verts[o1], z);
    let p2 = lerp_to_z(verts[lone], verts[o2], z);
    // Orient along ẑ × n = (−n_y, n_x): material stays on the left.
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let (nx, ny) = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2]);
    let (dx, dy) = (p2.x_mm() - p1.x_mm(), p2.y_mm() - p1.y_mm());
    if dx * -ny + dy * nx < 0.0 {
        return Some((p2, p1));
    }
    Some((p1, p2))
}

/// Linearly interpolate the (x, y) where the segment p0->p1 crosses plane z, and
/// snap to the integer grid.
fn lerp_to_z(p0: Vec3, p1: Vec3, z: f64) -> Point {
    let t = (z - p0[2]) / (p1[2] - p0[2]);
    let x = p0[0] + t * (p1[0] - p0[0]);
    let y = p0[1] + t * (p1[1] - p0[1]);
    Point::from_mm(x, y)
}

/// Stitch undirected segments into closed contours.
///
/// On a manifold mesh every cut point lies on exactly one mesh edge, shared by
/// exactly two triangles, so each point has degree two and the segments form
/// disjoint simple cycles. We walk those cycles, then orient each by nesting.
fn stitch(segments: Vec<(Point, Point)>) -> Polygons {
    let mut adj: HashMap<Point, Vec<Point>> = HashMap::new();
    for &(p, q) in &segments {
        if p == q {
            continue;
        }
        adj.entry(p).or_default().push(q);
        adj.entry(q).or_default().push(p);
    }

    let norm = |a: Point, b: Point| if (a.x, a.y) <= (b.x, b.y) { (a, b) } else { (b, a) };
    let mut used: HashSet<(Point, Point)> = HashSet::new();
    let mut polys = Polygons::new();

    // Seed walks from each segment endpoint in input order. Once an edge is
    // consumed it is skipped, so each cycle is emitted exactly once.
    for &(seed, _) in &segments {
        let mut current = seed;
        let mut loop_pts: Vec<Point> = Vec::new();
        loop {
            let next = adj
                .get(&current)
                .and_then(|ns| ns.iter().copied().find(|&n| !used.contains(&norm(current, n))));
            let Some(next) = next else { break };
            used.insert(norm(current, next));
            loop_pts.push(current);
            current = next;
            if current == seed {
                break;
            }
        }
        if loop_pts.len() >= 3 {
            polys.push(Contour::new(loop_pts));
        }
    }

    orient_by_nesting(&mut polys);
    polys
}

/// Orient contours so outers (even nesting depth) are CCW and holes (odd depth)
/// are CW. Depth is the count of other contours containing a representative
/// point of the contour.
fn orient_by_nesting(polys: &mut Polygons) {
    let n = polys.contours.len();
    for i in 0..n {
        let Some(&probe) = polys.contours[i].points.first() else {
            continue;
        };
        let mut depth = 0;
        for j in 0..n {
            if i != j && polys.contours[j].contains(probe) {
                depth += 1;
            }
        }
        if depth % 2 == 0 {
            polys.contours[i].make_ccw();
        } else {
            polys.contours[i].make_cw();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slices_cube_into_squares() {
        let m = Mesh::cube(20.0);
        let layers = slice_mesh(&m, SliceParams { layer_height_mm: 0.2, first_layer_height_mm: 0.2 });

        // 20mm / 0.2mm = 100 layers sampled at midpoints 0.1 .. 19.9.
        assert_eq!(layers.len(), 100);

        for l in &layers {
            assert_eq!(
                l.polygons.contours.len(),
                1,
                "expected a single contour at layer {} (z={})",
                l.index,
                l.z_mm
            );
            let c = &l.polygons.contours[0];
            assert!(
                (c.area_mm2() - 400.0).abs() < 1.0,
                "expected ~400mm² at layer {}, got {}",
                l.index,
                c.area_mm2()
            );
            assert!(c.is_ccw(), "outer contour should be CCW at layer {}", l.index);
        }
    }

    #[test]
    fn empty_mesh_yields_no_layers() {
        let m = Mesh::default();
        assert!(slice_mesh(&m, SliceParams::default()).is_empty());
    }

    /// A 20×20×2 plate with a 6×6 through-hole. `flip_hole_walls` reverses the
    /// hole's side-wall winding CONSISTENTLY — the wild-STL damage class where
    /// the directed stitcher still closes every loop, so the <90%-consumed
    /// fallback never fires (the holeplate.stl bug).
    fn holed_plate(flip_hole_walls: bool) -> Mesh {
        let mut tris: Vec<[[f64; 3]; 3]> = Vec::new();
        let mut quad = |a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]| {
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        };
        let (x0, x1, h0, h1, z0, z1) = (0.0, 20.0, 7.0, 13.0, 0.0, 2.0);
        // Top (+z) / bottom (−z): the ring as 4 rects around the hole.
        let rects = [
            [x0, x0 + h0, x0, x1],  // west slab  [0,7]×[0,20]
            [h1, x1, x0, x1],       // east slab  [13,20]×[0,20]
            [h0, h1, x0, h0],       // south bar  [7,13]×[0,7]
            [h0, h1, h1, x1],       // north bar  [7,13]×[13,20]
        ];
        for [rx0, rx1, ry0, ry1] in rects {
            quad([rx0, ry0, z1], [rx1, ry0, z1], [rx1, ry1, z1], [rx0, ry1, z1]);
            quad([rx0, ry1, z0], [rx1, ry1, z0], [rx1, ry0, z0], [rx0, ry0, z0]);
        }
        // Outer walls, outward normals.
        quad([x0, x0, z0], [x1, x0, z0], [x1, x0, z1], [x0, x0, z1]); // south −y
        quad([x1, x1, z0], [x0, x1, z0], [x0, x1, z1], [x1, x1, z1]); // north +y
        quad([x0, x1, z0], [x0, x0, z0], [x0, x0, z1], [x0, x1, z1]); // west −x
        quad([x1, x0, z0], [x1, x1, z0], [x1, x1, z1], [x1, x0, z1]); // east +x
        // Hole walls. Correct = normals INTO the hole (outward from material).
        let mut hole_quad = |a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]| {
            if flip_hole_walls {
                quad(d, c, b, a);
            } else {
                quad(a, b, c, d);
            }
        };
        hole_quad([h0, h0, z0], [h0, h1, z0], [h0, h1, z1], [h0, h0, z1]); // west face, +x
        hole_quad([h1, h1, z0], [h1, h0, z0], [h1, h0, z1], [h1, h1, z1]); // east face, −x
        hole_quad([h1, h0, z0], [h0, h0, z0], [h0, h0, z1], [h1, h0, z1]); // south face, +y
        hole_quad([h0, h1, z0], [h1, h1, z0], [h1, h1, z1], [h0, h1, z1]); // north face, −y
        Mesh::from_triangle_soup(&tris)
    }

    #[test]
    fn flipped_hole_walls_still_slice_as_a_hole() {
        // Both the honest mesh and the flipped-wall one must slice
        // identically: 2 contours, ring area 400 − 36 = 364 mm². Before the
        // parity re-orientation, the flipped hole stitched CCW, the positive
        // fill rule unioned it away, and every layer printed the bore SOLID —
        // silent part scrap (the fixtures/holeplate.stl bug; Orca refuses the
        // equivalent mesh outright).
        for flipped in [false, true] {
            let layers = slice_mesh(
                &holed_plate(flipped),
                SliceParams { layer_height_mm: 0.2, first_layer_height_mm: 0.2 },
            );
            assert_eq!(layers.len(), 10);
            for l in &layers {
                assert_eq!(
                    l.polygons.contours.len(),
                    2,
                    "flipped={flipped} z={}: the hole must survive",
                    l.z_mm
                );
                let area = l.polygons.net_area_mm2();
                assert!(
                    (area - 364.0).abs() < 1.0,
                    "flipped={flipped} z={}: ring area {area:.1}, want 364",
                    l.z_mm
                );
            }
        }
    }

    #[test]
    fn overlapping_shells_resolve_to_their_union() {
        // Two overlapping cubes fused into ONE mesh — the classic CAD export
        // (also the shape of a self-intersecting surface's slice: crossing
        // contours). Nesting-parity orientation reads the overlap as a hole
        // (XOR); the geometry-directed winding + positive fill resolves the
        // true material: the union.
        let mut m = Mesh::cube(20.0);
        let shift = mesh::Transform {
            translation: [10.0, 10.0, 0.0],
            ..mesh::Transform::IDENTITY
        };
        m.append(&Mesh::cube(20.0).transformed(&shift));
        let layers = slice_mesh(&m, SliceParams::default());
        let mid = &layers[layers.len() / 2];
        let area = mid.polygons.net_area_mm2();
        // 2 × 400 − 100 overlap = 700 mm² (XOR would read 600).
        assert!((area - 700.0).abs() < 2.0, "expected the union, got {area} mm²");
    }
}
