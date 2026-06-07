//! Plane/mesh intersection and contour stitching.

use std::collections::{HashMap, HashSet};

use geo2d::{Contour, Point, Polygons};
use mesh::{Mesh, Vec3};

/// Parameters controlling how the mesh is sliced.
#[derive(Clone, Copy, Debug)]
pub struct SliceParams {
    pub layer_height_mm: f64,
}

impl Default for SliceParams {
    fn default() -> Self {
        Self { layer_height_mm: 0.2 }
    }
}

/// One sliced layer: its index, the z height it was sampled at, and the closed
/// polygons describing solid material at that height.
#[derive(Clone, Debug)]
pub struct Layer {
    pub index: usize,
    pub z_mm: f64,
    pub polygons: Polygons,
}

/// Slice a mesh into layers of the given height.
///
/// Each layer is sampled at its vertical midpoint (`z = zmin + h*(i + 0.5)`),
/// which avoids landing exactly on the flat top/bottom facets of axis-aligned
/// parts. Layers run while the sample height stays below the top of the model.
pub fn slice_mesh(mesh: &Mesh, params: SliceParams) -> Vec<Layer> {
    let h = params.layer_height_mm;
    let Some((zmin, zmax)) = mesh.z_bounds() else {
        return Vec::new();
    };

    let mut layers = Vec::new();
    let mut i = 0usize;
    loop {
        let z = zmin + h * (i as f64 + 0.5);
        if z >= zmax {
            break;
        }
        layers.push(Layer {
            index: i,
            z_mm: z,
            polygons: slice_at(mesh, z),
        });
        i += 1;
    }
    layers
}

/// Intersect the whole mesh with one horizontal plane and stitch the result.
fn slice_at(mesh: &Mesh, z: f64) -> Polygons {
    let z = nudge_off_vertices(mesh, z);

    let mut segments: Vec<(Point, Point)> = Vec::new();
    for i in 0..mesh.triangles.len() {
        let [a, b, c] = mesh.triangle(i);
        if let Some(seg) = intersect_triangle(a, b, c, z) {
            segments.push(seg);
        }
    }
    stitch(segments)
}

/// Nudge the slice plane by a tiny amount if it coincides with any vertex, so no
/// triangle has a vertex *exactly* on the plane (which would make the
/// above/below classification ambiguous).
fn nudge_off_vertices(mesh: &Mesh, z: f64) -> f64 {
    const EPS: f64 = 1.0e-6;
    for v in &mesh.vertices {
        if (v[2] - z).abs() < EPS {
            return z + EPS;
        }
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
        let layers = slice_mesh(&m, SliceParams { layer_height_mm: 0.2 });

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
