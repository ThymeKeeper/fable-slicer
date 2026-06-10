//! Variable-width (Arachne-class) wall generation.
//!
//! The distance-field formulation of Kuipers et al.'s adaptive beading: instead
//! of an exact skeletal trapezoidation, the region is rasterized to a fine grid
//! (¼ line width), an exact Euclidean distance transform gives the depth-to-
//! boundary field `d`, ridge cells of `d` form the skeleton, and a nearest-
//! skeleton transform assigns every cell the radius `T̂` of its closest skeleton
//! point — the local half-thickness. The **beading scheme** then decides, per
//! cell, how many beads fit (`round(2T̂ / spacing)`, capped at the wall count)
//! and at what pitch; bead centerlines are extracted as **level sets** of `d`
//! at `(i + ½)·pitch` via marching squares, so beads stretch, squeeze, split
//! and merge with the local geometry. Odd stretch-zones' center bead runs along
//! the skeleton itself (ridge trace). Every output vertex carries its width.
//!
//! Zones with more room than the wall count needs use the nominal pitch, which
//! makes the level sets coincide with classic concentric offsets — and the
//! leftover interior is handed to infill exactly as before.

use geo2d::{Point, Polygons};

/// One variable-width bead: points, per-vertex widths (mm), closed?
pub(crate) struct Bead {
    pub points: Vec<Point>,
    pub widths: Vec<f64>,
    pub closed: bool,
}

/// Variable-width walls for one layer region.
pub(crate) struct VariableWalls {
    /// Adaptive beads inside the classic fixed-width outer wall.
    pub inner: Vec<Bead>,
    /// Tapered single beads where the region is too thin for any outer loop.
    pub thin_outer: Vec<Bead>,
}

const CELLS_CAP: usize = 4_000_000;

/// Generate variable-width walls. `outer` is the full layer region (for thin-
/// feature beads where even the fixed outer loop can't fit); `inner` is the
/// region inside the outer wall (gets up to `max_inner` adaptive beads).
pub(crate) fn variable_walls(
    outer: &Polygons,
    inner: &Polygons,
    lw: f64,
    sp: f64,
    max_inner: usize,
) -> VariableWalls {
    let mut out = VariableWalls { inner: Vec::new(), thin_outer: Vec::new() };
    if max_inner > 0 {
        if let Some(f) = Field::build(inner, lw) {
            out.inner = f.beads(lw, sp, max_inner);
        }
    }
    if let Some(f) = Field::build(outer, lw) {
        out.thin_outer = f.thin_ridge_beads(lw, sp);
    }
    out
}

/// Per-region scalar fields on a uniform grid.
struct Field {
    x0: f64,
    y0: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    inside: Vec<bool>,
    /// Depth: distance (mm) to the region boundary.
    d: Vec<f64>,
    /// Skeleton (ridge) cells of `d`.
    skel: Vec<bool>,
    /// Local half-thickness: radius of the nearest skeleton cell (mm).
    t_hat: Vec<f64>,
}

/// Beading regime at one cell.
#[derive(Clone, Copy)]
struct Scheme {
    /// Bead count here (≤ the wall cap).
    n: usize,
    /// Centerline pitch (mm).
    pitch: f64,
    /// True when the zone is thicker than the cap needs (classic spacing;
    /// the remainder of the thickness belongs to infill).
    saturated: bool,
}

impl Field {
    fn build(region: &Polygons, lw: f64) -> Option<Field> {
        let bb = region.bounds()?;
        let pad = lw;
        let (x0, y0) = (bb.min.x_mm() - pad, bb.min.y_mm() - pad);
        let (w, h) = (bb.max.x_mm() - x0 + pad, bb.max.y_mm() - y0 + pad);
        let mut cell = (lw * 0.25).clamp(0.05, 0.4);
        if (w / cell) * (h / cell) > CELLS_CAP as f64 {
            cell = (w * h / CELLS_CAP as f64).sqrt(); // coarsen to fit the cap
        }
        let nx = (w / cell).ceil() as usize + 2;
        let ny = (h / cell).ceil() as usize + 2;
        if nx < 4 || ny < 4 {
            return None;
        }

        // Rasterize (even-odd scanline over cell centers).
        let mut inside = vec![false; nx * ny];
        let mut any = false;
        let mut xs: Vec<f64> = Vec::new();
        for iy in 0..ny {
            let y = y0 + (iy as f64 + 0.5) * cell;
            xs.clear();
            for c in &region.contours {
                let m = c.points.len();
                if m < 3 {
                    continue;
                }
                for j in 0..m {
                    let (ay, by) = (c.points[j].y_mm(), c.points[(j + 1) % m].y_mm());
                    if (ay <= y) != (by <= y) {
                        let (ax, bx) = (c.points[j].x_mm(), c.points[(j + 1) % m].x_mm());
                        xs.push(ax + (y - ay) / (by - ay) * (bx - ax));
                    }
                }
            }
            xs.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
            let mut k = 0;
            while k + 1 < xs.len() {
                let lo = (((xs[k] - x0) / cell - 0.5).ceil().max(0.0)) as usize;
                let hi = ((xs[k + 1] - x0) / cell - 0.5).floor();
                if hi >= 0.0 {
                    for ix in lo..=(hi as usize).min(nx - 1) {
                        inside[iy * nx + ix] = true;
                        any = true;
                    }
                }
                k += 2;
            }
        }
        if !any {
            return None;
        }

        // Exact EDT to the outside (cells), boundary half-cell corrected, in mm.
        let d2 = edt(&inside, nx, ny);
        let d: Vec<f64> = d2.iter().map(|&q| ((q.sqrt() - 0.5) * cell).max(0.0)).collect();

        // Skeleton: ridge cells — no neighbour is clearly deeper. The small
        // depth floor keeps single-cell boundary noise out.
        let mut skel = vec![false; nx * ny];
        for iy in 1..ny - 1 {
            for ix in 1..nx - 1 {
                let c = iy * nx + ix;
                if !inside[c] || d[c] < lw * 0.15 {
                    continue;
                }
                let mut ridge = true;
                for (dx, dy) in [(-1i64, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)] {
                    let nb = ((iy as i64 + dy) * nx as i64 + ix as i64 + dx) as usize;
                    if d[nb] > d[c] + cell * 0.7 {
                        ridge = false;
                        break;
                    }
                }
                if ridge {
                    skel[c] = true;
                }
            }
        }

        // T̂ = radius of the nearest skeleton cell.
        let t_hat = nearest_value(&skel, &d, nx, ny);

        Some(Field { x0, y0, cell, nx, ny, inside, d, skel, t_hat })
    }

    #[inline]
    fn center(&self, ix: usize, iy: usize) -> (f64, f64) {
        (self.x0 + (ix as f64 + 0.5) * self.cell, self.y0 + (iy as f64 + 0.5) * self.cell)
    }

    #[inline]
    fn cell_at(&self, x: f64, y: f64) -> Option<usize> {
        let ix = ((x - self.x0) / self.cell - 0.5).round();
        let iy = ((y - self.y0) / self.cell - 0.5).round();
        if ix < 0.0 || iy < 0.0 {
            return None;
        }
        let (ix, iy) = (ix as usize, iy as usize);
        (ix < self.nx && iy < self.ny).then(|| iy * self.nx + ix)
    }

    /// Beading scheme at a cell. `cap` is the ring budget (wall count − 1
    /// inner rings); a ring contributes a bead on *both* sides of the local
    /// thickness, so up to `2·cap` beads fit across before saturation.
    ///
    /// - **Stretch** (≤ 2·cap beads across): all of the thickness is shared
    ///   evenly — beads widen/narrow, no infill here.
    /// - **Absorb**: thicker than the budget, but the leftover after `cap`
    ///   nominal rings is a sub-bead sliver infill couldn't print — the rings
    ///   widen slightly to swallow it (this is what makes gap fill obsolete
    ///   in arachne mode).
    /// - **Saturated**: real room left over → nominal pitch, remainder is
    ///   infill territory (classic geometry).
    #[inline]
    fn scheme(&self, c: usize, sp: f64, cap: usize) -> Scheme {
        let t = 2.0 * self.t_hat[c];
        let fit = ((t / sp).round() as usize).max(1);
        if fit <= 2 * cap {
            Scheme { n: fit, pitch: (t / fit as f64).clamp(sp * 0.5, sp * 1.6), saturated: false }
        } else {
            let lw_ish = sp / 0.9; // a hair over one bead — the sliver threshold
            let remainder = t - 2.0 * cap as f64 * sp;
            if remainder < lw_ish {
                let n = 2 * cap;
                Scheme { n, pitch: (t / n as f64).clamp(sp * 0.5, sp * 1.6), saturated: false }
            } else {
                Scheme { n: 2 * cap, pitch: sp, saturated: true }
            }
        }
    }

    /// Inner adaptive beads: level sets of `d` per bead index, plus skeleton
    /// center beads in odd stretch zones.
    fn beads(&self, lw: f64, sp: f64, cap: usize) -> Vec<Bead> {
        let mut beads = Vec::new();
        let width_of = move |pitch: f64| (pitch + (lw - sp)).clamp(lw * 0.5, lw * 1.6);

        for i in 0..cap {
            let mut psi = vec![f64::INFINITY; self.nx * self.ny];
            let mut center_zone = vec![false; self.nx * self.ny];
            let mut any_center = false;
            for c in 0..self.nx * self.ny {
                if !self.inside[c] {
                    continue;
                }
                let s = self.scheme(c, sp, cap);
                if i >= s.n {
                    continue; // bead absent here
                }
                if !s.saturated && s.n % 2 == 1 && i == s.n / 2 {
                    center_zone[c] = true; // odd stretch zone: ridge bead
                    any_center = true;
                    continue;
                }
                let target = (i as f64 + 0.5) * s.pitch;
                // In stretch zones the folded depth field covers both sides:
                // only emit levels on the near side of the ridge (the far-side
                // mirror is the same physical bead).
                if s.saturated || target <= self.t_hat[c] + 0.26 * s.pitch {
                    psi[c] = self.d[c] - target;
                }
            }
            for poly in self.contour(&psi) {
                if let Some(b) = self.polyline_to_bead(poly, sp, cap, &width_of) {
                    beads.push(b);
                }
            }
            if any_center {
                for line in self.trace_ridges(&center_zone) {
                    if let Some(b) = self.polyline_to_bead((line, false), sp, cap, &width_of) {
                        beads.push(b);
                    }
                }
            }
        }
        beads
    }

    /// Single tapered beads along skeleton cells thinner than one line width —
    /// features where no classic outer loop can exist.
    fn thin_ridge_beads(&self, lw: f64, sp: f64) -> Vec<Bead> {
        let mut zone = vec![false; self.nx * self.ny];
        let mut any = false;
        for c in 0..self.nx * self.ny {
            if self.skel[c] && self.d[c] < lw * 0.5 {
                zone[c] = true;
                any = true;
            }
        }
        if !any {
            return Vec::new();
        }
        let width_of = move |pitch: f64| (pitch + (lw - sp)).clamp(lw * 0.4, lw * 1.2);
        self.trace_ridges(&zone)
            .into_iter()
            .filter_map(|line| self.polyline_to_bead((line, false), sp, 1, &width_of))
            .collect()
    }

    /// Marching squares over the cell-center lattice: the zero set of `psi`
    /// (non-finite = masked), chained into (polyline, closed?) pieces.
    fn contour(&self, psi: &[f64]) -> Vec<(Vec<(f64, f64)>, bool)> {
        let mut segs: Vec<((f64, f64), (f64, f64))> = Vec::new();
        let lerp = |pa: (f64, f64), va: f64, pb: (f64, f64), vb: f64| {
            let t = if (vb - va).abs() < 1e-12 { 0.5 } else { (-va / (vb - va)).clamp(0.0, 1.0) };
            (pa.0 + t * (pb.0 - pa.0), pa.1 + t * (pb.1 - pa.1))
        };
        for iy in 0..self.ny - 1 {
            for ix in 0..self.nx - 1 {
                let c00 = iy * self.nx + ix;
                let (c10, c01, c11) = (c00 + 1, c00 + self.nx, c00 + self.nx + 1);
                let vs = [psi[c00], psi[c10], psi[c11], psi[c01]];
                if vs.iter().any(|v| !v.is_finite()) {
                    continue;
                }
                let p00 = self.center(ix, iy);
                let p10 = self.center(ix + 1, iy);
                let p01 = self.center(ix, iy + 1);
                let p11 = self.center(ix + 1, iy + 1);
                let mut case = 0u8;
                if vs[0] > 0.0 { case |= 1 }
                if vs[1] > 0.0 { case |= 2 }
                if vs[2] > 0.0 { case |= 4 }
                if vs[3] > 0.0 { case |= 8 }
                if case == 0 || case == 15 {
                    continue;
                }
                let bottom = || lerp(p00, vs[0], p10, vs[1]);
                let right = || lerp(p10, vs[1], p11, vs[2]);
                let top = || lerp(p01, vs[3], p11, vs[2]);
                let left = || lerp(p00, vs[0], p01, vs[3]);
                match case {
                    1 | 14 => segs.push((bottom(), left())),
                    2 | 13 => segs.push((bottom(), right())),
                    4 | 11 => segs.push((right(), top())),
                    8 | 7 => segs.push((top(), left())),
                    3 | 12 => segs.push((left(), right())),
                    6 | 9 => segs.push((bottom(), top())),
                    5 | 10 => {
                        segs.push((bottom(), left()));
                        segs.push((right(), top()));
                    }
                    _ => unreachable!(),
                }
            }
        }
        crate::fill::chain_segments(segs, self.cell * 0.5)
            .into_iter()
            .map(|line| {
                let closed = line.len() > 3 && {
                    let (a, b) = (line[0], line[line.len() - 1]);
                    (a.0 - b.0).hypot(a.1 - b.1) < self.cell * 1.5
                };
                (line, closed)
            })
            .collect()
    }

    /// Walk 8-connected cells of `zone` into polylines. Each connected blob is
    /// traced once, from its farthest-out endpoint (double BFS), following the
    /// main path greedily; side spurs shorter than the grid noise floor vanish
    /// in simplification.
    fn trace_ridges(&self, zone: &[bool]) -> Vec<Vec<(f64, f64)>> {
        let n = self.nx * self.ny;
        let mut seen = vec![false; n];
        let mut lines = Vec::new();
        let neighbors = |c: usize, buf: &mut Vec<usize>| {
            buf.clear();
            let (ix, iy) = ((c % self.nx) as i64, (c / self.nx) as i64);
            for dy in -1i64..=1 {
                for dx in -1i64..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let (jx, jy) = (ix + dx, iy + dy);
                    if jx >= 0 && jy >= 0 && (jx as usize) < self.nx && (jy as usize) < self.ny {
                        let nb = jy as usize * self.nx + jx as usize;
                        if zone[nb] {
                            buf.push(nb);
                        }
                    }
                }
            }
        };

        let mut buf = Vec::with_capacity(8);
        for start in 0..n {
            if !zone[start] || seen[start] {
                continue;
            }
            // Collect the blob (BFS), remembering the farthest cell = endpoint.
            let mut blob = vec![start];
            let mut far = start;
            seen[start] = true;
            let mut qi = 0;
            while qi < blob.len() {
                let c = blob[qi];
                qi += 1;
                far = c; // BFS order: the last popped is among the farthest
                neighbors(c, &mut buf);
                for &nb in &buf {
                    if !seen[nb] {
                        seen[nb] = true;
                        blob.push(nb);
                    }
                }
            }
            // Greedy walk from the endpoint through the blob.
            let mut walked = std::collections::HashSet::new();
            let mut line = Vec::new();
            let mut at = far;
            loop {
                walked.insert(at);
                line.push(self.center(at % self.nx, at / self.nx));
                neighbors(at, &mut buf);
                match buf.iter().copied().find(|nb| !walked.contains(nb)) {
                    Some(nb) => at = nb,
                    None => break,
                }
            }
            if line.len() >= 2 {
                lines.push(line);
            }
        }
        lines
    }

    /// Chained polyline → Bead with per-vertex widths from the local scheme.
    fn polyline_to_bead(
        &self,
        (line, closed): (Vec<(f64, f64)>, bool),
        sp: f64,
        cap: usize,
        width_of: &dyn Fn(f64) -> f64,
    ) -> Option<Bead> {
        let mut line = line;
        if closed {
            let (a, b) = (line[0], line[line.len() - 1]);
            if (a.0 - b.0).hypot(a.1 - b.1) < self.cell * 0.25 {
                line.pop(); // drop a duplicated closing point
            }
        }
        let len: f64 = line.windows(2).map(|w| (w[0].0 - w[1].0).hypot(w[0].1 - w[1].1)).sum();
        if line.len() < 2 || len < sp * 1.5 {
            return None; // grid-noise crumb
        }
        let line = rdp(&line, self.cell * 0.45);
        let mut points = Vec::with_capacity(line.len());
        let mut widths = Vec::with_capacity(line.len());
        for &(x, y) in &line {
            let w = match self.cell_at(x, y) {
                Some(c) if self.inside[c] => width_of(self.scheme(c, sp, cap).pitch),
                _ => width_of(sp),
            };
            points.push(Point::from_mm(x, y));
            widths.push(w);
        }
        if points.len() < 2 {
            return None;
        }
        Some(Bead { points, widths, closed })
    }
}

/// Ramer–Douglas–Peucker simplification (keeps endpoints).
fn rdp(line: &[(f64, f64)], eps: f64) -> Vec<(f64, f64)> {
    if line.len() <= 2 {
        return line.to_vec();
    }
    let (a, b) = (line[0], line[line.len() - 1]);
    let (mut imax, mut dmax) = (0usize, 0.0f64);
    for (i, &p) in line.iter().enumerate().skip(1).take(line.len() - 2) {
        let d = perp_dist(p, a, b);
        if d > dmax {
            dmax = d;
            imax = i;
        }
    }
    if dmax > eps {
        let mut left = rdp(&line[..=imax], eps);
        let right = rdp(&line[imax..], eps);
        left.pop();
        left.extend(right);
        left
    } else {
        vec![a, b]
    }
}

fn perp_dist(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = dx.hypot(dy);
    if len < 1e-12 {
        return (p.0 - a.0).hypot(p.1 - a.1);
    }
    ((p.0 - a.0) * dy - (p.1 - a.1) * dx).abs() / len
}

/// Exact squared Euclidean distance transform (Felzenszwalb–Huttenlocher):
/// distance from each inside cell to the nearest outside cell, in cell units.
/// Distances are capped at the grid diameter, which keeps the envelope finite.
fn edt(inside: &[bool], nx: usize, ny: usize) -> Vec<f64> {
    let big = (nx + ny) as f64; // larger than any real distance
    // Column pass: 1D distance along y.
    let mut g = vec![0.0f64; nx * ny];
    for ix in 0..nx {
        g[ix] = if inside[ix] { big } else { 0.0 };
        for iy in 1..ny {
            let c = iy * nx + ix;
            g[c] = if inside[c] { (1.0 + g[c - nx]).min(big) } else { 0.0 };
        }
        for iy in (0..ny - 1).rev() {
            let c = iy * nx + ix;
            if g[c + nx] + 1.0 < g[c] {
                g[c] = g[c + nx] + 1.0;
            }
        }
    }
    // Row pass: lower envelope of parabolas over x (textbook F-H).
    let mut d = vec![0.0f64; nx * ny];
    let mut v = vec![0usize; nx];
    let mut z = vec![0.0f64; nx + 1];
    for iy in 0..ny {
        let row = iy * nx;
        let f = |x: usize| {
            let gg = g[row + x];
            gg * gg
        };
        let mut k = 0usize;
        v[0] = 0;
        z[0] = f64::NEG_INFINITY;
        z[1] = f64::INFINITY;
        for q in 1..nx {
            let mut s;
            loop {
                let p = v[k];
                s = ((f(q) + (q * q) as f64) - (f(p) + (p * p) as f64)) / (2 * q - 2 * p) as f64;
                if s <= z[k] && k > 0 {
                    k -= 1;
                } else {
                    break;
                }
            }
            k += 1;
            v[k] = q;
            z[k] = s;
            z[k + 1] = f64::INFINITY;
        }
        let mut k2 = 0usize;
        for q in 0..nx {
            while z[k2 + 1] < q as f64 {
                k2 += 1;
            }
            let p = v[k2];
            d[row + q] = (q as f64 - p as f64).powi(2) + f(p);
        }
    }
    d
}

/// For every cell, the `value` carried by its nearest `seed` cell — multi-source
/// Dijkstra over the 8-connected grid (exact enough at cell resolution).
fn nearest_value(seeds: &[bool], value: &[f64], nx: usize, ny: usize) -> Vec<f64> {
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;
    let n = nx * ny;
    let mut best = vec![f64::INFINITY; n];
    let mut out = vec![0.0f64; n];
    let mut heap: BinaryHeap<Reverse<(u64, usize)>> = BinaryHeap::new();
    let key = |dist: f64| (dist * 1024.0) as u64;
    for c in 0..n {
        if seeds[c] {
            best[c] = 0.0;
            out[c] = value[c];
            heap.push(Reverse((0, c)));
        }
    }
    while let Some(Reverse((dk, c))) = heap.pop() {
        if dk > key(best[c]) {
            continue;
        }
        let (ix, iy) = ((c % nx) as i64, (c / nx) as i64);
        for (dx, dy, w) in [
            (-1i64, 0i64, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, std::f64::consts::SQRT_2),
            (1, -1, std::f64::consts::SQRT_2),
            (-1, 1, std::f64::consts::SQRT_2),
            (1, 1, std::f64::consts::SQRT_2),
        ] {
            let (jx, jy) = (ix + dx, iy + dy);
            if jx < 0 || jy < 0 || jx as usize >= nx || jy as usize >= ny {
                continue;
            }
            let nb = jy as usize * nx + jx as usize;
            let nd = best[c] + w;
            if nd < best[nb] {
                best[nb] = nd;
                out[nb] = out[c];
                heap.push(Reverse((key(nd), nb)));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo2d::Contour;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygons {
        let mut p = Polygons::new();
        p.push(Contour::new(vec![
            Point::from_mm(x0, y0),
            Point::from_mm(x1, y0),
            Point::from_mm(x1, y1),
            Point::from_mm(x0, y1),
        ]));
        p
    }

    #[test]
    fn edt_center_of_strip() {
        // A 2 mm tall strip: depth at mid-height ≈ 1 mm; skeleton along the middle.
        let f = Field::build(&rect(0.0, 0.0, 30.0, 2.0), 0.45).unwrap();
        let mid = f.cell_at(15.0, 1.0).unwrap();
        assert!((f.d[mid] - 1.0).abs() < 0.15, "depth {}", f.d[mid]);
        assert!((f.t_hat[mid] - 1.0).abs() < 0.2, "t_hat {}", f.t_hat[mid]);
    }

    #[test]
    fn stretch_zone_shares_thickness() {
        // 1.0 mm strip with sp≈0.407: fit = round(1.0/0.407) = 2 beads at pitch 0.5.
        let f = Field::build(&rect(0.0, 0.0, 30.0, 1.0), 0.45).unwrap();
        let c = f.cell_at(15.0, 0.5).unwrap();
        let s = f.scheme(c, 0.407, 3);
        assert_eq!(s.n, 2);
        assert!(!s.saturated);
        assert!((s.pitch - 0.5).abs() < 0.08, "pitch {}", s.pitch);
        // The folded level set of a rectangular strip is one closed ring
        // (both long sides + the ends), with stretched widths.
        let beads = f.beads(0.45, 0.407, 3);
        assert_eq!(beads.len(), 1, "got {} beads", beads.len());
        assert!(beads[0].closed, "strip ring should close");
        let w = beads[0].widths[beads[0].widths.len() / 2];
        assert!((0.45..0.65).contains(&w), "stretched width {w}");
    }

    #[test]
    fn scheme_regimes_stretch_absorb_saturate() {
        // A wide slab so T̂ is controlled by strip height; sp ≈ 0.407, cap = 2.
        let sp = 0.407;
        // Stretch: t = 1.4 → fit 3 ≤ 4: thickness shared (odd → center bead zone).
        let f = Field::build(&rect(0.0, 0.0, 40.0, 1.4), 0.45).unwrap();
        let s = f.scheme(f.cell_at(20.0, 0.7).unwrap(), sp, 2);
        assert!(!s.saturated);
        assert_eq!(s.n, 3);
        // Absorb: t = 2·2·sp + 0.3 ≈ 1.93 → leftover 0.3 is a sub-bead sliver;
        // the 4 beads widen to swallow it instead of leaving a void.
        let f = Field::build(&rect(0.0, 0.0, 40.0, 1.93), 0.45).unwrap();
        let s = f.scheme(f.cell_at(20.0, 0.965).unwrap(), sp, 2);
        assert!(!s.saturated, "sliver leftover must be absorbed");
        assert_eq!(s.n, 4);
        assert!(s.pitch > sp + 0.02, "rings widen: pitch {}", s.pitch);
        // Saturated: t = 2·2·sp + 1.2 → real room: nominal pitch, infill owns it.
        let f = Field::build(&rect(0.0, 0.0, 40.0, 2.83), 0.45).unwrap();
        let s = f.scheme(f.cell_at(20.0, 1.415).unwrap(), sp, 2);
        assert!(s.saturated);
        assert!((s.pitch - sp).abs() < 1e-9);
    }

    #[test]
    fn thin_strip_gets_one_tapered_ridge_bead() {
        let f = Field::build(&rect(0.0, 0.0, 20.0, 0.3), 0.45).unwrap();
        let beads = f.thin_ridge_beads(0.45, 0.407);
        assert_eq!(beads.len(), 1, "one centerline bead");
        let b = &beads[0];
        // Runs the strip's length near y = 0.15, width ≈ strip thickness.
        let len: f64 = b
            .points
            .windows(2)
            .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
            .sum();
        assert!(len > 15.0, "ridge length {len}");
        let w = b.widths[b.widths.len() / 2];
        assert!((0.2..0.45).contains(&w), "tapered width {w}");
    }
}
