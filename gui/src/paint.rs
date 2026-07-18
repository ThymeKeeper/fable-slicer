//! Surface paint: mesh topology + the smart-fill flood behind the viewport
//! brush. Pure geometry (no GPU, no egui) so it unit-tests in isolation.
//!
//! The old face painter flood-filled across any edge whose two RAW face normals
//! were within a single angle threshold. On an organic sculpt that both BLEEDS
//! (a shallow slope accumulates past the region a degree at a time, each step
//! under threshold) and STALLS (a noisy bump's raw normal spikes over
//! threshold and blocks the front). This rebuild fixes both:
//!
//! * **Smoothed normals** (area-weighted 1-ring) — a small bump no longer
//!   blocks the front; true creases survive the smoothing. (anti-stall)
//! * A **local** dihedral gate defines connectivity (stop at creases), and a
//!   separate **global** drift gate vs. the seed normal is applied as a live
//!   filter — so slow accumulation across a soft transition is cut off even
//!   though every single step passed the local test. (anti-bleed)
//! * A **geodesic distance cap** and a **front-face mask** keep a fill from
//!   running to the far/hidden side of the model. (anti-runaway)
//!
//! [`PaintTopology::flood`] runs once per click with generous bounds, recording
//! `(geodesic_dist, drift_angle)` per reached face; the caller then re-filters
//! that record every drag-frame by a live `(radius, drift)` tolerance — no
//! re-flood — and commits the previewed set on release.

// TODO(stage-a): drop once the brush is wired into the viewport handler.
#![allow(dead_code)]

use glam::Vec3;
use mesh::Mesh;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

/// Edges gentler than this are smoothed across when building face normals;
/// sharper ones (creases) are not. Above typical mesh noise, below a real
/// crease — so bumps average out while corners stay crisp.
const SMOOTH_CAP_DEG: f32 = 40.0;

/// Precomputed, transform-independent topology of one part's mesh, in the
/// mesh's LOCAL frame. Built once and cached; reused across strokes.
pub struct PaintTopology {
    pub n_faces: usize,
    /// Edge-adjacent neighbor faces of each face (≤ ~3 for a manifold mesh).
    neighbors: Vec<Vec<u32>>,
    /// Area-weighted smoothed unit normal per face.
    smooth_normal: Vec<Vec3>,
    /// Raw flat unit normal per face (front-face test, `Vec3::ZERO` if degenerate).
    flat_normal: Vec<Vec3>,
    /// Face centroid (local frame).
    centroid: Vec<Vec3>,
}

/// Live-adjustable bounds for a flood. `theta_local` is the fixed connectivity
/// gate (creases); `max_dist` bounds the recorded region. The GLOBAL drift gate
/// is applied later as a live filter (see [`FloodFace::drift`]).
pub struct FloodParams {
    /// Max local dihedral angle (radians) between adjacent smoothed normals to
    /// cross an edge. Creases sharper than this stop the fill.
    pub theta_local: f32,
    /// Hard geodesic cap (local units) on how far to record — generous; the
    /// live radius filter tightens within it.
    pub max_dist: f32,
    /// If set, only faces whose flat normal points toward `eye_local` are
    /// recorded (the fill can't wrap onto the hidden back side).
    pub front_faces_only: bool,
    /// Camera eye in the mesh's LOCAL frame (for the front-face test).
    pub eye_local: Vec3,
}

/// One face reached by a flood, with the two quantities the live filter needs.
#[derive(Clone, Copy, Debug)]
pub struct FloodFace {
    pub face: u32,
    /// Geodesic distance from the seed (centroid-to-centroid, local units).
    pub dist: f32,
    /// Angle (radians) between this face's smoothed normal and the seed's —
    /// the "drift" the global gate filters on.
    pub drift: f32,
}

impl PaintTopology {
    pub fn build(mesh: &Mesh) -> Self {
        let n = mesh.triangles.len();
        let mut flat_normal = vec![Vec3::ZERO; n];
        let mut cross = vec![Vec3::ZERO; n]; // unnormalized normal (len = 2·area)
        let mut centroid = vec![Vec3::ZERO; n];
        for i in 0..n {
            let [a, b, c] = mesh.triangle(i);
            let (a, b, c) = (v3(a), v3(b), v3(c));
            let x = (b - a).cross(c - a);
            cross[i] = x;
            flat_normal[i] = x.normalize_or_zero();
            centroid[i] = (a + b + c) / 3.0;
        }
        // Edge → incident faces, then per-face neighbors.
        let mut edges: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
        for (k, t) in mesh.triangles.iter().enumerate() {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let key = if a < b { (a, b) } else { (b, a) };
                edges.entry(key).or_default().push(k as u32);
            }
        }
        let mut neighbors: Vec<Vec<u32>> = vec![Vec::new(); n];
        for faces in edges.values() {
            for (ai, &fa) in faces.iter().enumerate() {
                for &fb in faces.iter().skip(ai + 1) {
                    if fa != fb {
                        if !neighbors[fa as usize].contains(&fb) {
                            neighbors[fa as usize].push(fb);
                        }
                        if !neighbors[fb as usize].contains(&fa) {
                            neighbors[fb as usize].push(fa);
                        }
                    }
                }
            }
        }
        // Crease-aware, area-weighted smoothed normal = normalize(Σ cross over
        // the face's 1-ring, EXCLUDING neighbors across a sharp edge). Summing
        // unnormalized crosses area-weights automatically. Smoothing only across
        // gentle edges is what makes this both anti-stall (a noisy bump gets
        // averaged back toward its surface) AND crease-preserving (a 90° cube
        // edge is never averaged across, so the local gate still stops there) —
        // naive 1-ring smoothing blurs the crease and breaks both.
        let cos_smooth = SMOOTH_CAP_DEG.to_radians().cos();
        let mut smooth_normal = vec![Vec3::ZERO; n];
        for f in 0..n {
            let mut acc = cross[f];
            for &g in &neighbors[f] {
                if flat_normal[f].dot(flat_normal[g as usize]) >= cos_smooth {
                    acc += cross[g as usize];
                }
            }
            // Fall back to the flat normal if the ring cancels out.
            smooth_normal[f] = acc.normalize_or_zero();
            if smooth_normal[f] == Vec3::ZERO {
                smooth_normal[f] = flat_normal[f];
            }
        }
        Self { n_faces: n, neighbors, smooth_normal, flat_normal, centroid }
    }

    fn front_facing(&self, f: usize, eye_local: Vec3) -> bool {
        self.flat_normal[f].dot(eye_local - self.centroid[f]) > 0.0
    }

    /// Bounded geodesic flood from `seed`, recording `(dist, drift)` for every
    /// reached face. Connectivity is gated by the LOCAL dihedral + front-face +
    /// `max_dist`; the seed is always included. The caller applies the live
    /// radius + global-drift filter to the returned record.
    pub fn flood(&self, seed: usize, p: &FloodParams) -> Vec<FloodFace> {
        let mut out = Vec::new();
        if seed >= self.n_faces || self.flat_normal[seed] == Vec3::ZERO {
            return out;
        }
        let seed_n = self.smooth_normal[seed];
        let cos_local = p.theta_local.cos();
        let mut best = vec![f32::INFINITY; self.n_faces];
        let mut done = vec![false; self.n_faces];
        let mut heap: BinaryHeap<Reverse<HeapItem>> = BinaryHeap::new();
        best[seed] = 0.0;
        heap.push(Reverse(HeapItem { dist: 0.0, face: seed as u32 }));
        while let Some(Reverse(HeapItem { dist, face })) = heap.pop() {
            let f = face as usize;
            if done[f] || dist > p.max_dist {
                continue;
            }
            done[f] = true;
            let drift = seed_n.dot(self.smooth_normal[f]).clamp(-1.0, 1.0).acos();
            out.push(FloodFace { face, dist, drift });
            let nf = self.smooth_normal[f];
            for &g in &self.neighbors[f] {
                let gi = g as usize;
                if done[gi] || self.flat_normal[gi] == Vec3::ZERO {
                    continue;
                }
                if p.front_faces_only && !self.front_facing(gi, p.eye_local) {
                    continue;
                }
                // Local connectivity: don't cross a crease sharper than θ_local.
                if nf.dot(self.smooth_normal[gi]) < cos_local {
                    continue;
                }
                let nd = dist + (self.centroid[gi] - self.centroid[f]).length();
                if nd < best[gi] && nd <= p.max_dist {
                    best[gi] = nd;
                    heap.push(Reverse(HeapItem { dist: nd, face: g }));
                }
            }
        }
        out
    }
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// Min-heap item ordered by distance (f32 via `total_cmp`).
struct HeapItem {
    dist: f32,
    face: u32,
}
impl PartialEq for HeapItem {
    fn eq(&self, o: &Self) -> bool {
        self.dist == o.dist && self.face == o.face
    }
}
impl Eq for HeapItem {}
impl PartialOrd for HeapItem {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for HeapItem {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.dist.total_cmp(&o.dist).then(self.face.cmp(&o.face))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A unit box (12 tris). Face normals are axis-aligned; adjacent box faces
    // meet at 90°.
    fn cube() -> Mesh {
        let v = [
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0],
        ];
        let f = [
            [0, 2, 1], [0, 3, 2], // bottom (-Z)
            [4, 5, 6], [4, 6, 7], // top (+Z)
            [0, 1, 5], [0, 5, 4], // -Y
            [1, 2, 6], [1, 6, 5], // +X
            [2, 3, 7], [2, 7, 6], // +Y
            [3, 0, 4], [3, 4, 7], // -X
        ];
        let tris: Vec<[[f64; 3]; 3]> = f.iter().map(|t| [v[t[0]], v[t[1]], v[t[2]]]).collect();
        Mesh::from_triangle_soup(&tris)
    }

    // An n×n coplanar grid on z=0, each cell two triangles. All normals +Z.
    fn grid(n: usize) -> Mesh {
        let mut tris = Vec::new();
        let s = 1.0 / n as f64;
        for i in 0..n {
            for j in 0..n {
                let (x, y) = (i as f64 * s, j as f64 * s);
                let a = [x, y, 0.0];
                let b = [x + s, y, 0.0];
                let c = [x + s, y + s, 0.0];
                let d = [x, y + s, 0.0];
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            }
        }
        Mesh::from_triangle_soup(&tris)
    }

    // A fan/strip of quads in the x–z plane, each bent `step_deg` from the
    // previous about the shared y-edge. Every step is a shallow crease; drift
    // from the first face accumulates linearly.
    fn bent_strip(count: usize, step_deg: f64) -> Mesh {
        let mut tris = Vec::new();
        let mut p = [0.0, 0.0, 0.0];
        let mut ang = 0.0f64;
        for _ in 0..count {
            let dir = [ang.cos(), 0.0, ang.sin()];
            let q = [p[0] + dir[0], 0.0, p[2] + dir[2]];
            // quad p→q along +y width 1: (p, q, q+y, p+y)
            let p1 = [p[0], 1.0, p[2]];
            let q1 = [q[0], 1.0, q[2]];
            tris.push([p, q, q1]);
            tris.push([p, q1, p1]);
            p = q;
            ang += step_deg.to_radians();
        }
        Mesh::from_triangle_soup(&tris)
    }

    fn params(max_dist: f32, theta_local_deg: f32) -> FloodParams {
        FloodParams {
            theta_local: theta_local_deg.to_radians(),
            max_dist,
            front_faces_only: false,
            eye_local: Vec3::ZERO,
        }
    }

    #[test]
    fn flood_stops_at_cube_creases() {
        let m = cube();
        let topo = PaintTopology::build(&m);
        // Seed on the top face (tri 2, one of the two +Z triangles). At θ_local
        // = 35° the 90° box edges block, so only the top face's 2 tris fill.
        let region = topo.flood(2, &params(100.0, 35.0));
        let faces: std::collections::HashSet<u32> = region.iter().map(|f| f.face).collect();
        assert_eq!(faces, [2u32, 3].into_iter().collect());
    }

    #[test]
    fn flood_covers_coplanar_grid_and_respects_distance_cap() {
        let m = grid(8); // 128 tris, all coplanar
        let topo = PaintTopology::build(&m);
        // Generous cap: the whole coplanar sheet fills (no creases, drift 0).
        let all = topo.flood(0, &params(100.0, 35.0));
        assert_eq!(all.len(), m.triangles.len());
        assert!(all.iter().all(|f| f.drift < 1e-3), "coplanar => zero drift");
        // Tight geodesic cap: only faces near the seed, strictly fewer.
        let near = topo.flood(0, &params(0.3, 35.0));
        assert!(near.len() < all.len() && !near.is_empty());
        assert!(near.iter().all(|f| f.dist <= 0.3 + 1e-4));
    }

    #[test]
    fn global_drift_filter_stops_the_bleed() {
        // Every step bends 20° — under a 35° local gate, so the old single-gate
        // flood would walk the ENTIRE strip (classic bleed). Drift from the
        // seed accumulates ~20°/face.
        let m = bent_strip(12, 20.0);
        let topo = PaintTopology::build(&m);
        let region = topo.flood(0, &params(1000.0, 35.0));
        // Local gate alone reaches the whole strip...
        assert_eq!(region.len(), m.triangles.len(), "local gate alone bleeds through");
        // ...but a 60° GLOBAL drift filter (the live knob) cuts it off. Faces
        // 0..~3 (drift 0,~20,~40,~60) survive; the far end past 60° is dropped.
        let kept: Vec<_> = region.iter().filter(|f| f.drift <= 60f32.to_radians() + 1e-3).collect();
        assert!(kept.len() < region.len(), "global filter must drop drifted faces");
        assert!(
            kept.iter().all(|f| f.drift <= 60f32.to_radians() + 1e-3),
            "kept faces are within the drift budget"
        );
        // And it's a real cut, not everything-or-nothing.
        assert!(kept.len() >= 4, "the near, low-drift faces are kept");
    }

    #[test]
    fn front_face_mask_excludes_the_back_side() {
        let m = cube();
        let topo = PaintTopology::build(&m);
        // Eye far on +Z looking down: the top (+Z) face is front, bottom is back.
        let mut p = params(100.0, 35.0);
        p.front_faces_only = true;
        p.eye_local = Vec3::new(0.5, 0.5, 50.0);
        // Seed the top face; the flood can't leave it anyway (creases), but the
        // mask must never admit a back (-Z) face even if angles allowed.
        let region = topo.flood(2, &p);
        assert!(region.iter().all(|f| f.face == 2 || f.face == 3));
    }
}
