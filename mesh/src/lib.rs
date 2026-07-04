//! Triangle mesh: indexed storage, STL/3MF I/O, and a few primitives.
//!
//! Vertices are stored once and referenced by index (welded on load), which gives
//! us implicit edge sharing — useful later for topology-aware repair. For M0 the
//! slicer only needs the triangle list and the z-range.

mod threemf;
pub use threemf::{load_3mf, load_3mf_reader, ThreeMfItem, ThreeMfPart};

use std::collections::HashMap;
use std::fs;
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// A 3D point / vector.
pub type Vec3 = [f64; 3];

/// An indexed triangle mesh.
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub vertices: Vec<Vec3>,
    /// Each triangle is three indices into `vertices`.
    pub triangles: Vec<[u32; 3]>,
}

/// An affine placement of an object on the bed: scale, then rotate, then translate.
/// `rotation` is a row-major 3×3 matrix (orthonormal for a rigid placement).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub rotation: [[f64; 3]; 3],
    pub scale: f64,
    pub translation: Vec3,
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform {
    pub const IDENTITY: Transform = Transform {
        rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        scale: 1.0,
        translation: [0.0, 0.0, 0.0],
    };

    /// Map a point: scale, rotate, then translate.
    pub fn apply(&self, p: Vec3) -> Vec3 {
        let s = [p[0] * self.scale, p[1] * self.scale, p[2] * self.scale];
        let r = &self.rotation;
        [
            r[0][0] * s[0] + r[0][1] * s[1] + r[0][2] * s[2] + self.translation[0],
            r[1][0] * s[0] + r[1][1] * s[1] + r[1][2] * s[2] + self.translation[1],
            r[2][0] * s[0] + r[2][1] * s[1] + r[2][2] * s[2] + self.translation[2],
        ]
    }

    /// The rotation+scale part only (no translation) — for measuring footprints.
    pub fn apply_linear(&self, p: Vec3) -> Vec3 {
        let s = [p[0] * self.scale, p[1] * self.scale, p[2] * self.scale];
        let r = &self.rotation;
        [
            r[0][0] * s[0] + r[0][1] * s[1] + r[0][2] * s[2],
            r[1][0] * s[0] + r[1][1] * s[1] + r[1][2] * s[2],
            r[2][0] * s[0] + r[2][1] * s[1] + r[2][2] * s[2],
        ]
    }
}

impl Mesh {
    /// Build a mesh from a "triangle soup" (independent triangles), welding
    /// coincident vertices so the result is indexed. Degenerate triangles
    /// (two or more shared corners) are dropped.
    pub fn from_triangle_soup(tris: &[[Vec3; 3]]) -> Mesh {
        // Quantize to nanometers for the weld key so near-identical vertices merge.
        fn key(v: Vec3) -> [i64; 3] {
            [
                (v[0] * 1.0e6).round() as i64,
                (v[1] * 1.0e6).round() as i64,
                (v[2] * 1.0e6).round() as i64,
            ]
        }

        let mut index_of: HashMap<[i64; 3], u32> = HashMap::new();
        let mut vertices: Vec<Vec3> = Vec::new();
        let mut triangles: Vec<[u32; 3]> = Vec::new();

        for t in tris {
            let mut idx = [0u32; 3];
            for (k, &v) in t.iter().enumerate() {
                idx[k] = *index_of.entry(key(v)).or_insert_with(|| {
                    vertices.push(v);
                    (vertices.len() - 1) as u32
                });
            }
            if idx[0] == idx[1] || idx[1] == idx[2] || idx[0] == idx[2] {
                continue; // degenerate
            }
            triangles.push(idx);
        }

        Mesh { vertices, triangles }
    }

    /// Split into connected components — triangles that share a vertex land in
    /// the same body. Returns one re-indexed mesh per component, in order of
    /// first appearance; a single-body mesh (or an empty one) yields at most a
    /// one-element Vec. Connectivity is by shared vertex INDEX, which is exactly
    /// right for indexed meshes (3MF) and for `from_triangle_soup` output (it
    /// welds coincident vertices), so two separate solids in one file — the two
    /// feet of a "Legs.stl" — come apart cleanly.
    pub fn split_connected(&self) -> Vec<Mesh> {
        let nv = self.vertices.len();
        if self.triangles.is_empty() {
            return Vec::new();
        }
        // Union-find over vertices with path halving.
        let mut parent: Vec<u32> = (0..nv as u32).collect();
        fn find(parent: &mut [u32], mut x: u32) -> u32 {
            while parent[x as usize] != x {
                parent[x as usize] = parent[parent[x as usize] as usize];
                x = parent[x as usize];
            }
            x
        }
        let union = |parent: &mut [u32], a: u32, b: u32| {
            let (ra, rb) = (find(parent, a), find(parent, b));
            if ra != rb {
                parent[ra as usize] = rb;
            }
        };
        for t in &self.triangles {
            union(&mut parent, t[0], t[1]);
            union(&mut parent, t[1], t[2]);
        }
        // Assign each component a dense index in first-appearance order.
        let mut comp_index: HashMap<u32, usize> = HashMap::new();
        let mut tri_comp: Vec<usize> = Vec::with_capacity(self.triangles.len());
        for t in &self.triangles {
            let r = find(&mut parent, t[0]);
            let next = comp_index.len();
            let ci = *comp_index.entry(r).or_insert(next);
            tri_comp.push(ci);
        }
        let ncomp = comp_index.len();
        if ncomp <= 1 {
            return vec![self.clone()];
        }
        // Rebuild each body with its own vertices. A vertex belongs to exactly
        // one component (a shared vertex would have merged them), so one global
        // remap suffices.
        let mut out: Vec<Mesh> =
            (0..ncomp).map(|_| Mesh { vertices: Vec::new(), triangles: Vec::new() }).collect();
        let mut remap = vec![u32::MAX; nv];
        for (ti, t) in self.triangles.iter().enumerate() {
            let mesh = &mut out[tri_comp[ti]];
            let mut nt = [0u32; 3];
            for k in 0..3 {
                let v = t[k] as usize;
                if remap[v] == u32::MAX {
                    remap[v] = mesh.vertices.len() as u32;
                    mesh.vertices.push(self.vertices[v]);
                }
                nt[k] = remap[v];
            }
            mesh.triangles.push(nt);
        }
        out
    }

    /// A DISPLAY-ONLY copy without the faces that render as artifacts:
    /// zero-area triangles and GHOST faces — surface with the same winding
    /// number on both sides, bounding no material. That covers every
    /// zero-thickness fin some generators emit for empty runs (a QR plate's
    /// blank modules — doubled sheets in ANY triangulation, edge-adjacent or
    /// not) and interior walls buried between abutting solids. The
    /// classification is the generalized-winding-number test real mesh
    /// repair tools use; meshes too large for its O(n²) fall back to the
    /// cheap fold-back edge test.
    ///
    /// NEVER feed this to the slicer. Removing buried interior walls merges
    /// regions the multi-material planner must keep distinct, and the
    /// directed slicer already resolves ghosts correctly from the signed
    /// winding of the raw mesh (verified: toolpaths identical either way).
    /// In the viewer the dropped faces are invisible or artifacts, so
    /// dropping them is safe there and only there.
    pub fn display_mesh(&self) -> Mesh {
        let mut m = self.clone();
        m.drop_degenerate_faces();
        m
    }

    /// Generalized winding number of the closed(ish) surface at `p` —
    /// Van Oosterom–Strackee solid angles summed over every triangle, over
    /// 4π. ≈1 inside, ≈0 outside; ghosts contribute canceling pairs.
    fn winding_at(&self, p: Vec3) -> f64 {
        let mut sum = 0.0f64;
        for t in &self.triangles {
            let s = |v: Vec3| [v[0] - p[0], v[1] - p[1], v[2] - p[2]];
            let (a, b, c) = (
                s(self.vertices[t[0] as usize]),
                s(self.vertices[t[1] as usize]),
                s(self.vertices[t[2] as usize]),
            );
            let dot = |x: [f64; 3], y: [f64; 3]| x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
            let cross = |x: [f64; 3], y: [f64; 3]| {
                [
                    x[1] * y[2] - x[2] * y[1],
                    x[2] * y[0] - x[0] * y[2],
                    x[0] * y[1] - x[1] * y[0],
                ]
            };
            let (la, lb, lc) = (
                dot(a, a).sqrt(),
                dot(b, b).sqrt(),
                dot(c, c).sqrt(),
            );
            let num = dot(a, cross(b, c));
            let den = la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb;
            sum += f64::atan2(num, den);
        }
        sum / (2.0 * std::f64::consts::PI)
    }

    /// Drop every face whose two sides see the same winding number — it
    /// bounds no material. Catches doubled sheets regardless of how each
    /// side is triangulated (the fold-back edge test needs the pair to share
    /// an edge; a sheet split along the other diagonal escapes it).
    fn drop_ghost_faces(&mut self) {
        use rayon::prelude::*;
        const EPS_MM: f64 = 5e-4;
        let keep: Vec<bool> = self
            .triangles
            .par_iter()
            .map(|t| {
                let (a, b, c) = (
                    self.vertices[t[0] as usize],
                    self.vertices[t[1] as usize],
                    self.vertices[t[2] as usize],
                );
                let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                let n = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                let m = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                if m < 1e-12 {
                    return false;
                }
                let cx = (a[0] + b[0] + c[0]) / 3.0;
                let cy = (a[1] + b[1] + c[1]) / 3.0;
                let cz = (a[2] + b[2] + c[2]) / 3.0;
                let d = [n[0] / m * EPS_MM, n[1] / m * EPS_MM, n[2] / m * EPS_MM];
                let wf = self.winding_at([cx + d[0], cy + d[1], cz + d[2]]).round();
                let wb = self.winding_at([cx - d[0], cy - d[1], cz - d[2]]).round();
                wf != wb
            })
            .collect();
        let mut it = keep.iter();
        self.triangles.retain(|_| *it.next().unwrap());
    }

    fn drop_degenerate_faces(&mut self) {
        // Zero-area first — their normals are noise for the fold test.
        let verts = &self.vertices;
        let normal = |t: &[u32; 3]| -> Vec3 {
            let (a, b, c) =
                (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ]
        };
        let mag2 = |n: Vec3| n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
        self.triangles.retain(|t| mag2(normal(t)) > 1e-20);

        // Ghost removal: the winding classification is complete but O(n²) —
        // gate it; huge meshes keep the cheap edge-adjacent fold-back test.
        if self.triangles.len() <= 20_000 {
            self.drop_ghost_faces();
            self.prune_unused_vertices();
            return;
        }

        // Fold-backs: an edge shared by exactly two triangles whose normals
        // point exactly opposite ways is a crease of angle zero — the
        // surface doubles back on itself and encloses nothing there.
        let normals: Vec<Vec3> = self.triangles.iter().map(|t| normal(t)).collect();
        let mut edges: HashMap<(u32, u32), (u32, u32, u8)> = HashMap::new();
        for (i, t) in self.triangles.iter().enumerate() {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let k = (a.min(b), a.max(b));
                let e = edges.entry(k).or_insert((0, 0, 0));
                match e.2 {
                    0 => e.0 = i as u32,
                    1 => e.1 = i as u32,
                    _ => {}
                }
                e.2 = e.2.saturating_add(1);
            }
        }
        let mut drop = vec![false; self.triangles.len()];
        for &(i, j, n) in edges.values() {
            if n != 2 {
                continue;
            }
            let (a, b) = (normals[i as usize], normals[j as usize]);
            let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
            if dot < 0.0 && dot * dot > 0.999_998 * mag2(a) * mag2(b) {
                drop[i as usize] = true;
                drop[j as usize] = true;
            }
        }
        if drop.iter().any(|&d| d) {
            let mut it = drop.iter();
            self.triangles.retain(|_| !it.next().unwrap());
        }

        self.prune_unused_vertices();
    }

    /// Prune vertices nothing references (ghost removal orphans the fin
    /// crests) so z_bounds and friends read printable geometry only.
    fn prune_unused_vertices(&mut self) {
        let mut remap: Vec<u32> = vec![u32::MAX; self.vertices.len()];
        let mut kept: Vec<Vec3> = Vec::new();
        for t in &mut self.triangles {
            for i in t.iter_mut() {
                let m = &mut remap[*i as usize];
                if *m == u32::MAX {
                    *m = kept.len() as u32;
                    kept.push(self.vertices[*i as usize]);
                }
                *i = *m;
            }
        }
        self.vertices = kept;
    }

    /// Append another mesh, re-basing its indices — merging build items into
    /// one plate (CLI) or components into one object.
    pub fn append(&mut self, other: &Mesh) {
        let base = self.vertices.len() as u32;
        self.vertices.extend_from_slice(&other.vertices);
        self.triangles
            .extend(other.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    }

    /// The three world-space vertices of triangle `i`.
    #[inline]
    /// A copy with `t` applied to every vertex (bakes the placement into geometry).
    pub fn transformed(&self, t: &Transform) -> Mesh {
        Mesh {
            vertices: self.vertices.iter().map(|&v| t.apply(v)).collect(),
            triangles: self.triangles.clone(),
        }
    }

    pub fn triangle(&self, i: usize) -> [Vec3; 3] {
        let t = self.triangles[i];
        [
            self.vertices[t[0] as usize],
            self.vertices[t[1] as usize],
            self.vertices[t[2] as usize],
        ]
    }

    /// XY bounding box `(min_x, min_y, max_x, max_y)` over all vertices, or
    /// `None` if the mesh is empty. Used to place the model on the bed.
    pub fn xy_bounds(&self) -> Option<(f64, f64, f64, f64)> {
        let first = self.vertices.first()?;
        let (mut min_x, mut min_y, mut max_x, mut max_y) = (first[0], first[1], first[0], first[1]);
        for v in &self.vertices {
            min_x = min_x.min(v[0]);
            min_y = min_y.min(v[1]);
            max_x = max_x.max(v[0]);
            max_y = max_y.max(v[1]);
        }
        Some((min_x, min_y, max_x, max_y))
    }

    /// Minimum and maximum z over all vertices, or `None` if the mesh is empty.
    pub fn z_bounds(&self) -> Option<(f64, f64)> {
        let mut iter = self.vertices.iter();
        let first = iter.next()?;
        let (mut lo, mut hi) = (first[2], first[2]);
        for v in iter {
            lo = lo.min(v[2]);
            hi = hi.max(v[2]);
        }
        Some((lo, hi))
    }

    /// An axis-aligned cube of edge length `size`, corner at the origin, with
    /// outward-facing winding. Handy as a slicing fixture / smoke test.
    pub fn cube(size: f64) -> Mesh {
        let s = size;
        let vertices = vec![
            [0.0, 0.0, 0.0], // 0
            [s, 0.0, 0.0],   // 1
            [s, s, 0.0],     // 2
            [0.0, s, 0.0],   // 3
            [0.0, 0.0, s],   // 4
            [s, 0.0, s],     // 5
            [s, s, s],       // 6
            [0.0, s, s],     // 7
        ];
        // Outward CCW winding (verified by normal sign).
        let triangles = vec![
            [0, 2, 1], [0, 3, 2], // bottom (-Z)
            [4, 5, 6], [4, 6, 7], // top    (+Z)
            [0, 1, 5], [0, 5, 4], // front  (-Y)
            [3, 6, 2], [3, 7, 6], // back   (+Y)
            [0, 7, 3], [0, 4, 7], // left   (-X)
            [1, 2, 6], [1, 6, 5], // right  (+X)
        ];
        Mesh { vertices, triangles }
    }

    /// Load an STL file, auto-detecting binary vs. ASCII.
    pub fn load_stl<P: AsRef<Path>>(path: P) -> io::Result<Mesh> {
        let bytes = fs::read(path)?;
        if is_binary_stl(&bytes) {
            parse_binary_stl(&bytes)
        } else {
            parse_ascii_stl(&bytes)
        }
    }

    /// Write the mesh as an ASCII STL (used to generate fixtures).
    pub fn write_stl_ascii<P: AsRef<Path>>(&self, path: P) -> io::Result<()> {
        let mut w = BufWriter::new(fs::File::create(path)?);
        writeln!(w, "solid mesh")?;
        for i in 0..self.triangles.len() {
            let [a, b, c] = self.triangle(i);
            let n = normal(a, b, c);
            writeln!(w, "  facet normal {} {} {}", n[0], n[1], n[2])?;
            writeln!(w, "    outer loop")?;
            for v in [a, b, c] {
                writeln!(w, "      vertex {} {} {}", v[0], v[1], v[2])?;
            }
            writeln!(w, "    endloop")?;
            writeln!(w, "  endfacet")?;
        }
        writeln!(w, "endsolid mesh")?;
        Ok(())
    }
}

fn normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 0.0 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0, 0.0, 0.0]
    }
}

/// Binary STL is exactly `84 + 50 * triangle_count` bytes. That size check is the
/// robust discriminator (some binary files start with the ASCII keyword "solid").
fn is_binary_stl(bytes: &[u8]) -> bool {
    if bytes.len() < 84 {
        return false;
    }
    let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    84 + count * 50 == bytes.len()
}

fn parse_binary_stl(bytes: &[u8]) -> io::Result<Mesh> {
    let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let rdf = |o: usize| f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]) as f64;
    let mut tris = Vec::with_capacity(count);
    let mut off = 84;
    for _ in 0..count {
        let base = off + 12; // skip the per-facet normal
        let mut v = [[0.0; 3]; 3];
        for (k, vert) in v.iter_mut().enumerate() {
            let vo = base + k * 12;
            *vert = [rdf(vo), rdf(vo + 4), rdf(vo + 8)];
        }
        tris.push(v);
        off += 50;
    }
    Ok(Mesh::from_triangle_soup(&tris))
}

fn parse_ascii_stl(bytes: &[u8]) -> io::Result<Mesh> {
    let text = String::from_utf8_lossy(bytes);
    let mut verts: Vec<Vec3> = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("vertex") {
            let nums: Vec<f64> = rest
                .split_whitespace()
                .filter_map(|s| s.parse::<f64>().ok())
                .collect();
            if nums.len() == 3 {
                verts.push([nums[0], nums[1], nums[2]]);
            }
        }
    }
    if verts.len() % 3 != 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "ASCII STL: vertex count is not a multiple of 3",
        ));
    }
    let tris: Vec<[Vec3; 3]> = verts.chunks(3).map(|c| [c[0], c[1], c[2]]).collect();
    Ok(Mesh::from_triangle_soup(&tris))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cube_is_indexed_and_closed() {
        let m = Mesh::cube(20.0);
        assert_eq!(m.vertices.len(), 8);
        assert_eq!(m.triangles.len(), 12);
        assert_eq!(m.z_bounds(), Some((0.0, 20.0)));
    }

    #[test]
    fn transform_bakes_translation_and_scale() {
        assert_eq!(Transform::IDENTITY.apply([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
        let m = Mesh::cube(2.0); // spans 0..2 on each axis
        let t = Transform { scale: 2.0, translation: [10.0, 0.0, 0.0], ..Transform::IDENTITY };
        let tm = m.transformed(&t);
        let (minx, _, maxx, _) = tm.xy_bounds().unwrap();
        assert!((minx - 10.0).abs() < 1e-9, "min x {minx}"); // 0*2+10
        assert!((maxx - 14.0).abs() < 1e-9, "max x {maxx}"); // 2*2+10
        assert_eq!(tm.z_bounds(), Some((0.0, 4.0))); // scaled, untranslated in z
    }

    #[test]
    fn ascii_stl_roundtrip() {
        let m = Mesh::cube(5.0);
        let dir = std::env::temp_dir();
        let path = dir.join("slicer_test_cube.stl");
        m.write_stl_ascii(&path).unwrap();
        let loaded = Mesh::load_stl(&path).unwrap();
        // Welded back to 8 unique corners, 12 faces.
        assert_eq!(loaded.vertices.len(), 8);
        assert_eq!(loaded.triangles.len(), 12);
        assert_eq!(loaded.z_bounds(), Some((0.0, 5.0)));
    }

    #[test]
    fn degenerate_faces_are_dropped_at_load() {
        // A unit cube (12 real triangles), plus the two degenerate classes
        // some generators emit: a zero-thickness fold-back fin (a vertical
        // quad doubled front-and-back, 4 triangles enclosing nothing) and a
        // collinear sliver. Only the cube must survive, and the fin's
        // orphaned crest vertices must not stretch the bounds.
        let mut tris: Vec<[Vec3; 3]> = Vec::new();
        let (lo, hi) = ([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let v = [
            [lo[0], lo[1], lo[2]],
            [hi[0], lo[1], lo[2]],
            [hi[0], hi[1], lo[2]],
            [lo[0], hi[1], lo[2]],
            [lo[0], lo[1], hi[2]],
            [hi[0], lo[1], hi[2]],
            [hi[0], hi[1], hi[2]],
            [lo[0], hi[1], hi[2]],
        ];
        for t in [
            [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
            [3, 6, 2], [3, 7, 6], [0, 7, 3], [0, 4, 7], [1, 2, 6], [1, 6, 5],
        ] {
            tris.push([v[t[0]], v[t[1]], v[t[2]]]);
        }
        // The fin: a vertical quad on top of the cube, both windings — the
        // crest at z=2 exists nowhere else, so pruning must remove it.
        let (a, b) = ([0.2, 0.5, 1.0], [0.8, 0.5, 1.0]);
        let (c, d) = ([0.2, 0.5, 2.0], [0.8, 0.5, 2.0]);
        tris.push([a, b, d]);
        tris.push([a, d, c]);
        tris.push([a, d, b]);
        tris.push([a, c, d]);
        // Collinear sliver: three distinct points on one line.
        tris.push([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [1.0, 0.0, 0.0]]);
        let m = Mesh::from_triangle_soup(&tris);
        // The raw mesh keeps everything the slicer needs (the sliver dies to
        // the soup's collapsed-corner weld only if its corners coincide —
        // this one keeps 3 distinct vertices).
        assert_eq!(m.triangles.len(), 17, "slicing input keeps the raw faces");
        let d = m.display_mesh();
        assert_eq!(d.triangles.len(), 12, "only the cube's faces are displayed");
        assert_eq!(d.z_bounds(), Some((0.0, 1.0)), "the fin crest must not stretch display bounds");
        assert_eq!(m.z_bounds(), Some((0.0, 2.0)), "the raw mesh is untouched");
    }

    #[test]
    fn split_connected_separates_disjoint_solids() {
        // One cube is a single body; two cubes 10 mm apart are two.
        let one = Mesh::cube(1.0);
        assert_eq!(one.split_connected().len(), 1, "a lone cube is one body");

        let mut two = Mesh::cube(1.0);
        let mut far = Mesh::cube(1.0);
        for v in &mut far.vertices {
            v[0] += 10.0;
        }
        two.append(&far);
        let bodies = two.split_connected();
        assert_eq!(bodies.len(), 2, "two disjoint cubes split into two bodies");
        // Every body is a whole, re-indexed cube — nothing lost or shared.
        for b in &bodies {
            assert_eq!(b.triangles.len(), 12, "each body keeps its 12 cube faces");
            assert_eq!(b.vertices.len(), 8, "each body re-indexes to its own 8 verts");
            for t in &b.triangles {
                assert!(t.iter().all(|&idx| (idx as usize) < b.vertices.len()), "indices in range");
            }
        }
        let total: usize = bodies.iter().map(|b| b.triangles.len()).sum();
        assert_eq!(total, two.triangles.len(), "no triangle dropped or duplicated");
    }

    #[test]
    fn cross_diagonal_fins_and_buried_walls_are_ghosts() {
        // The fin class the fold-back edge test CANNOT see: a doubled sheet
        // whose two sides are triangulated along different diagonals — the
        // anti-parallel faces never share an edge (the coin-holder QR's
        // remaining fins). The winding classification must still drop it.
        let cube = |lo: Vec3, hi: Vec3, tris: &mut Vec<[Vec3; 3]>| {
            let v = [
                [lo[0], lo[1], lo[2]],
                [hi[0], lo[1], lo[2]],
                [hi[0], hi[1], lo[2]],
                [lo[0], hi[1], lo[2]],
                [lo[0], lo[1], hi[2]],
                [hi[0], lo[1], hi[2]],
                [hi[0], hi[1], hi[2]],
                [lo[0], hi[1], hi[2]],
            ];
            for t in [
                [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                [3, 6, 2], [3, 7, 6], [0, 7, 3], [0, 4, 7], [1, 2, 6], [1, 6, 5],
            ] {
                tris.push([v[t[0]], v[t[1]], v[t[2]]]);
            }
        };
        let mut tris: Vec<[Vec3; 3]> = Vec::new();
        cube([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], &mut tris);
        // Fin quad corners above the cube; side A split a-d, side B split b-c.
        let (a, b) = ([0.2, 0.5, 1.0], [0.8, 0.5, 1.0]);
        let (c, d) = ([0.2, 0.5, 2.0], [0.8, 0.5, 2.0]);
        tris.push([a, b, d]);
        tris.push([a, d, c]);
        tris.push([b, a, c]);
        tris.push([b, c, d]);
        let m = Mesh::from_triangle_soup(&tris);
        assert_eq!(m.triangles.len(), 16);
        let disp = m.display_mesh();
        assert_eq!(disp.triangles.len(), 12, "cross-diagonal fin must be dropped");

        // Two cubes sharing a full face: each keeps its own wall there —
        // buried between solids, winding 1 on both sides, dropped from the
        // display (they are invisible inside the union anyway).
        let mut tris: Vec<[Vec3; 3]> = Vec::new();
        cube([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], &mut tris);
        cube([1.0, 0.0, 0.0], [2.0, 1.0, 1.0], &mut tris);
        let m = Mesh::from_triangle_soup(&tris);
        assert_eq!(m.triangles.len(), 24, "the raw pair keeps both interior walls");
        let disp = m.display_mesh();
        assert_eq!(disp.triangles.len(), 20, "buried walls drop from the display");
    }
}
