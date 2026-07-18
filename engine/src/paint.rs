//! Per-face color as a nearest-painted-surface tool field.
//!
//! A painted part carries one tool id per triangle (`PartPaint::Painted`). To
//! color a toolpath we ask, for each extrusion point, which painted surface is
//! nearest: a wall traces the surface so it takes that surface's color, and the
//! field flips exactly where the painted boundary projects inward. Interior
//! infill queries the same field (nearest surface wins) — its color never
//! shows, so the caller majority-votes a whole infill line to one tool rather
//! than chasing the boundary through the middle.
//!
//! A small median-split BVH over the part's triangles keeps the per-point query
//! ~log(n); only `Painted` parts build one.

use mesh::{Mesh, Vec3};

/// Nearest-painted-surface tool lookup over one part's triangles.
pub struct PaintField<'a> {
    mesh: &'a Mesh,
    paint: &'a [u32],
    nodes: Vec<Node>,
    /// Non-degenerate triangle indices, reordered by the BVH build.
    order: Vec<u32>,
    /// Tool returned when the mesh has no queryable triangle.
    fallback: u32,
}

enum Kind {
    Leaf { start: u32, count: u32 },
    Internal { left: u32, right: u32 },
}

struct Node {
    min: Vec3,
    max: Vec3,
    kind: Kind,
}

const LEAF_MAX: usize = 8;

impl<'a> PaintField<'a> {
    /// Build the field over `mesh`, whose triangle `t` prints tool `paint[t]`.
    pub fn new(mesh: &'a Mesh, paint: &'a [u32]) -> Self {
        let n = mesh.triangles.len();
        let fallback = paint.iter().copied().min().unwrap_or(0);
        // Per-triangle AABB (indexed by triangle id); degenerate tris are left
        // out of `order` so they never win a nearest query.
        let mut tmin = vec![[0.0; 3]; n];
        let mut tmax = vec![[0.0; 3]; n];
        let mut cent = vec![[0.0; 3]; n];
        let mut order: Vec<u32> = Vec::with_capacity(n);
        for i in 0..n {
            let [a, b, c] = mesh.triangle(i);
            if degenerate(a, b, c) {
                continue;
            }
            let mut lo = a;
            let mut hi = a;
            for v in [b, c] {
                for k in 0..3 {
                    lo[k] = lo[k].min(v[k]);
                    hi[k] = hi[k].max(v[k]);
                }
            }
            tmin[i] = lo;
            tmax[i] = hi;
            cent[i] = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0, (a[2] + b[2] + c[2]) / 3.0];
            order.push(i as u32);
        }
        let mut nodes = Vec::new();
        if !order.is_empty() {
            let len = order.len();
            build(&mut nodes, &mut order, 0, len, &cent, &tmin, &tmax);
        }
        Self { mesh, paint, nodes, order, fallback }
    }

    /// The paint value of the surface nearest `p` — a tool id, or (for a
    /// painted part) the index into its `paints` palette.
    pub fn value_at(&self, p: Vec3) -> u32 {
        if self.nodes.is_empty() {
            return self.fallback;
        }
        let mut best_d2 = f64::INFINITY;
        let mut best = u32::MAX;
        self.query(0, p, &mut best_d2, &mut best);
        if best == u32::MAX {
            self.fallback
        } else {
            self.paint.get(best as usize).copied().unwrap_or(self.fallback)
        }
    }

    fn query(&self, ni: u32, p: Vec3, best_d2: &mut f64, best: &mut u32) {
        let node = &self.nodes[ni as usize];
        if aabb_dist2(p, node.min, node.max) >= *best_d2 {
            return;
        }
        match node.kind {
            Kind::Leaf { start, count } => {
                for k in start..start + count {
                    let t = self.order[k as usize];
                    let [a, b, c] = self.mesh.triangle(t as usize);
                    let d2 = point_tri_dist2(p, a, b, c);
                    if d2 < *best_d2 {
                        *best_d2 = d2;
                        *best = t;
                    }
                }
            }
            Kind::Internal { left, right } => {
                // Descend into the nearer child first so the farther one prunes.
                let dl = aabb_dist2(p, self.nodes[left as usize].min, self.nodes[left as usize].max);
                let dr = aabb_dist2(p, self.nodes[right as usize].min, self.nodes[right as usize].max);
                let (near, far) = if dl <= dr { (left, right) } else { (right, left) };
                self.query(near, p, best_d2, best);
                self.query(far, p, best_d2, best);
            }
        }
    }
}

/// Build a subtree over `order[lo..hi]`, push its nodes, return the root index.
fn build(
    nodes: &mut Vec<Node>,
    order: &mut [u32],
    lo: usize,
    hi: usize,
    cent: &[Vec3],
    tmin: &[Vec3],
    tmax: &[Vec3],
) -> u32 {
    let mut bmin = [f64::INFINITY; 3];
    let mut bmax = [f64::NEG_INFINITY; 3];
    for &t in &order[lo..hi] {
        for k in 0..3 {
            bmin[k] = bmin[k].min(tmin[t as usize][k]);
            bmax[k] = bmax[k].max(tmax[t as usize][k]);
        }
    }
    let idx = nodes.len() as u32;
    if hi - lo <= LEAF_MAX {
        nodes.push(Node { min: bmin, max: bmax, kind: Kind::Leaf { start: lo as u32, count: (hi - lo) as u32 } });
        return idx;
    }
    // Split at the median centroid along the widest centroid axis.
    let mut cmin = [f64::INFINITY; 3];
    let mut cmax = [f64::NEG_INFINITY; 3];
    for &t in &order[lo..hi] {
        for k in 0..3 {
            cmin[k] = cmin[k].min(cent[t as usize][k]);
            cmax[k] = cmax[k].max(cent[t as usize][k]);
        }
    }
    let axis = (0..3).max_by(|&a, &b| (cmax[a] - cmin[a]).total_cmp(&(cmax[b] - cmin[b]))).unwrap();
    let mid = (lo + hi) / 2;
    order[lo..hi].select_nth_unstable_by(mid - lo, |&a, &b| {
        cent[a as usize][axis].total_cmp(&cent[b as usize][axis])
    });
    // Degenerate spread (all centroids equal on the axis) → just halve.
    nodes.push(Node { min: bmin, max: bmax, kind: Kind::Leaf { start: 0, count: 0 } }); // placeholder
    let left = build(nodes, order, lo, mid, cent, tmin, tmax);
    let right = build(nodes, order, mid, hi, cent, tmin, tmax);
    nodes[idx as usize].kind = Kind::Internal { left, right };
    idx
}

fn degenerate(a: Vec3, b: Vec3, c: Vec3) -> bool {
    let n = cross(sub(b, a), sub(c, a));
    dot(n, n) <= 1e-18
}

/// Squared distance from `p` to an axis-aligned box.
fn aabb_dist2(p: Vec3, min: Vec3, max: Vec3) -> f64 {
    let mut s = 0.0;
    for k in 0..3 {
        let d = if p[k] < min[k] {
            min[k] - p[k]
        } else if p[k] > max[k] {
            p[k] - max[k]
        } else {
            0.0
        };
        s += d * d;
    }
    s
}

/// Squared distance from `p` to triangle `abc` (Ericson, closest-point-on-tri).
fn point_tri_dist2(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return dist2(p, a);
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return dist2(p, b);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return dist2(p, add(a, scale(ab, v)));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return dist2(p, c);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return dist2(p, add(a, scale(ac, w)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return dist2(p, add(b, scale(sub(c, b), w)));
    }
    // Inside the face: project via barycentric.
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    dist2(p, add(a, add(scale(ab, v), scale(ac, w))))
}

#[inline]
fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
#[inline]
fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
#[inline]
fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline]
fn dist2(a: Vec3, b: Vec3) -> f64 {
    let d = sub(a, b);
    dot(d, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nearest_surface_colors_each_side() {
        // A 10 mm cube; every triangle whose centroid is on the +X half is
        // tool 1, the rest tool 0. A query near a face returns that face's tool.
        let m = Mesh::cube(10.0);
        let paint: Vec<u32> = (0..m.triangles.len())
            .map(|i| {
                let [a, b, c] = m.triangle(i);
                if (a[0] + b[0] + c[0]) / 3.0 > 5.0 { 1 } else { 0 }
            })
            .collect();
        let f = PaintField::new(&m, &paint);
        // Just inside the +X face → tool 1; just inside the -X face → tool 0.
        assert_eq!(f.value_at([9.5, 5.0, 5.0]), 1);
        assert_eq!(f.value_at([0.5, 5.0, 5.0]), 0);
    }

    #[test]
    fn empty_paint_falls_back() {
        let m = Mesh::cube(4.0);
        let f = PaintField::new(&m, &[]);
        assert_eq!(f.value_at([2.0, 2.0, 2.0]), 0);
    }
}
