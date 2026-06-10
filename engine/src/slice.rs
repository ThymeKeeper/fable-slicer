//! Plane/mesh intersection and contour stitching.

use std::collections::{HashMap, HashSet};

use geo2d::{Contour, Point, Polygons};
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

    // Plan the layer z's first (cheap, sequential)…
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

    // …then slice all the planes at once (bucketed + parallel).
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
/// sliced in parallel. Shared by normal layer slicing and the extra
/// quarter-height planes of half-height outer walls.
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
    stitch(segments)
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

/// Intersect a single triangle with plane `z`, returning the cut segment if the
/// triangle straddles the plane. Direction is arbitrary — stitching is
/// undirected and winding is fixed afterwards.
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
}
