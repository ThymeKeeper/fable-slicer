//! Triangle mesh: indexed storage, STL/3MF I/O, and a few primitives.
//!
//! Vertices are stored once and referenced by index (welded on load), which gives
//! us implicit edge sharing — useful later for topology-aware repair. For M0 the
//! slicer only needs the triangle list and the z-range.

mod threemf;
pub use threemf::{load_3mf, load_3mf_reader, ThreeMfItem};

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
}
