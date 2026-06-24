//! Raster coverage oracle for gap detection.
//!
//! Classic-mode gap fill needs to know which slivers of a region are left
//! *uncovered* by the wall beads that were just laid down. Offset-based
//! detection (`geo2d::offset` of each bead, then `difference`) is structurally
//! blind here: Clipper offsets can't represent a variable-width bead, and the
//! offset bands of two adjacent beads MERGE across a sub-line-width gap, so the
//! test under-reads the void by ~half and the surviving regions are irregular
//! corners and cross-strips that fill messily.
//!
//! This module answers the question by brute raster instead. Both the region
//! and every bead's TRUE footprint (a stadium/capsule of its local, per-vertex
//! width) are stamped onto a fine grid (~⅕ line width). The uncovered set is
//! `inside ∧ ¬covered`; its connected components are filtered by AREA (not
//! width) so a long thin flank strip survives while the sub-bead speckle
//! scattered along every bead edge — which real squish closes — is dropped.
//! Each kept component is traced back to a polygon via marching squares.
//!
//! The grid here is deliberately self-contained (a binary mask, no EDT or
//! skeleton) rather than reusing `wall::Field`: the oracle needs only
//! rasterization + footprint stamping + flood fill, none of the distance-field
//! machinery, and a dedicated mask keeps it small and obviously correct.
//!
use geo2d::{Contour, Point, Polygons};

/// One bead: a polyline and its per-vertex width in mm. The two slices must
/// have equal length; a single-vertex bead stamps one disk.
pub(crate) type Bead<'a> = (&'a [Point], &'a [f64]);

/// Regions inside `region` that **no** bead footprint covers.
///
/// Each bead's footprint is the union of capsules: every segment is swept by a
/// disk of the segment's local (linearly interpolated per-vertex) radius
/// `width / 2`. The result keeps only connected uncovered components whose area
/// is at least `min_area_mm2` — long thin strips survive, sub-bead speckle is
/// discarded. `lw` (nominal line width) sets the grid resolution.
///
/// Returns the components as `Polygons` (each an outer CCW contour; the marching
/// squares boundary of a flood-filled blob is simple, so holes are not emitted —
/// a void with an island in it would split into separate components instead).
pub(crate) fn uncovered(
    region: &Polygons,
    beads: &[(Vec<Point>, Vec<f64>)],
    lw: f64,
    min_area_mm2: f64,
) -> Polygons {
    let beads_ref: Vec<Bead> = beads.iter().map(|(p, w)| (p.as_slice(), w.as_slice())).collect();
    uncovered_inner(region, &beads_ref, lw, min_area_mm2)
}

/// Per-point extrusion-flow multipliers (≤1) that compensate for beads packed
/// tighter than they tile: rasterize a per-cell coverage COUNT, then each point's
/// flow is the mean of `1/count` over its footprint — where K beads overlap a
/// spot, each contributes 1/K so the spot gets exactly one bead's worth of
/// plastic (order-independent). **`widths` must be the bead TILING width**
/// (`bead_spacing`), so a normally-spaced fill tiles at count 1 (flow 1) and only
/// EXCESS overlap (the over-extrusion) is scaled down. One Vec per bead, 1:1 with
/// its points.
pub(crate) fn flow_factors(region: &Polygons, beads: &[(Vec<Point>, Vec<f64>)], lw: f64) -> Vec<Vec<f64>> {
    let beads_ref: Vec<Bead> = beads.iter().map(|(p, w)| (p.as_slice(), w.as_slice())).collect();
    let Some(grid) = Grid::new(region, lw) else {
        return beads.iter().map(|(p, _)| vec![1.0; p.len()]).collect();
    };
    let n = grid.nx * grid.ny;
    let mut count = vec![0u16; n];
    let mut last = vec![usize::MAX; n];
    for (bi, &(pts, widths)) in beads_ref.iter().enumerate() {
        grid.stamp_count(&mut count, &mut last, bi, pts, widths);
    }
    // Per-SEGMENT fair share (not per-point): sample 1/count over each segment's
    // actual capsule footprint — localized, so curves/corners don't pull in
    // off-segment cells the way a point-centered disk does. `flows[k]` is the flow
    // for the segment starting at point k (the emitter indexes it that way).
    beads_ref
        .iter()
        .map(|&(pts, widths)| {
            let n = pts.len();
            let mut f = vec![1.0; n];
            for k in 0..n.saturating_sub(1) {
                let ra = (0.5 * widths.get(k).copied().unwrap_or(lw)).max(grid.cell);
                let rb = (0.5 * widths.get(k + 1).copied().unwrap_or(lw)).max(grid.cell);
                f[k] = grid.capsule_mean_inv_count(
                    &count,
                    pts[k].x_mm(),
                    pts[k].y_mm(),
                    ra,
                    pts[k + 1].x_mm(),
                    pts[k + 1].y_mm(),
                    rb,
                );
            }
            if n >= 2 {
                f[n - 1] = f[n - 2]; // last point: unused for open paths
            }
            f
        })
        .collect()
}

fn uncovered_inner(region: &Polygons, beads: &[Bead], lw: f64, min_area_mm2: f64) -> Polygons {
    let Some(grid) = Grid::new(region, lw) else {
        return Polygons::new();
    };
    let mut inside = grid.rasterize(region);
    grid.stamp_beads(&mut inside, beads); // flips covered cells false
    grid.components_to_polygons(&inside, min_area_mm2)
}

/// A uniform binary-mask grid covering `region`'s bounds plus a small pad.
struct Grid {
    x0: f64,
    y0: f64,
    cell: f64,
    nx: usize,
    ny: usize,
}

/// Upper bound on grid cells, to bail on pathological inputs rather than OOM.
const CELLS_CAP: usize = 8_000_000;

impl Grid {
    fn new(region: &Polygons, lw: f64) -> Option<Grid> {
        let bb = region.bounds()?;
        // A void can hug the region boundary; pad by one cell so the boundary
        // band of cells exists and marching squares has a false rim to close on.
        let mut cell = (lw * 0.18).clamp(0.05, 0.12);
        let pad = lw;
        let (x0, y0) = (bb.min.x_mm() - pad, bb.min.y_mm() - pad);
        let (w, h) = (bb.max.x_mm() - x0 + pad, bb.max.y_mm() - y0 + pad);
        if w <= 0.0 || h <= 0.0 {
            return None;
        }
        if (w / cell) * (h / cell) > CELLS_CAP as f64 {
            cell = (w * h / CELLS_CAP as f64).sqrt();
        }
        // +2: a false guard ring on the far side too, so every blob is bounded
        // by `false` on all four sides (marching squares needs the rim).
        let nx = (w / cell).ceil() as usize + 2;
        let ny = (h / cell).ceil() as usize + 2;
        if nx < 4 || ny < 4 {
            return None;
        }
        Some(Grid { x0, y0, cell, nx, ny })
    }

    #[inline]
    fn idx(&self, ix: usize, iy: usize) -> usize {
        iy * self.nx + ix
    }

    /// Cell-center coordinate (mm).
    #[inline]
    fn center(&self, ix: usize, iy: usize) -> (f64, f64) {
        (self.x0 + (ix as f64 + 0.5) * self.cell, self.y0 + (iy as f64 + 0.5) * self.cell)
    }

    /// Even-odd scanline fill of cell centers — the same predicate as
    /// `Contour::contains`, evaluated for a whole row at once.
    fn rasterize(&self, region: &Polygons) -> Vec<bool> {
        let mut inside = vec![false; self.nx * self.ny];
        let mut xs: Vec<f64> = Vec::new();
        for iy in 0..self.ny {
            let y = self.y0 + (iy as f64 + 0.5) * self.cell;
            xs.clear();
            for c in &region.contours {
                let m = c.points.len();
                if m < 3 {
                    continue;
                }
                for j in 0..m {
                    let a = c.points[j];
                    let b = c.points[(j + 1) % m];
                    let (ay, by) = (a.y_mm(), b.y_mm());
                    if (ay <= y) != (by <= y) {
                        let (ax, bx) = (a.x_mm(), b.x_mm());
                        xs.push(ax + (y - ay) / (by - ay) * (bx - ax));
                    }
                }
            }
            xs.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
            let mut k = 0;
            while k + 1 < xs.len() {
                // Cells whose center lies in [xs[k], xs[k+1]] are inside.
                let lo = (((xs[k] - self.x0) / self.cell - 0.5).ceil()).max(0.0);
                let hi = ((xs[k + 1] - self.x0) / self.cell - 0.5).floor();
                if hi >= 0.0 && lo <= hi {
                    let lo = lo as usize;
                    let hi = (hi as usize).min(self.nx - 1);
                    for ix in lo..=hi {
                        inside[self.idx(ix, iy)] = true;
                    }
                }
                k += 2;
            }
        }
        inside
    }

    /// Stamp every bead's capsule footprint, clearing covered cells in `mask`.
    fn stamp_beads(&self, mask: &mut [bool], beads: &[Bead]) {
        for &(pts, widths) in beads {
            if pts.is_empty() {
                continue;
            }
            if pts.len() == 1 {
                let p = pts[0];
                let r = 0.5 * widths.first().copied().unwrap_or(0.0);
                self.stamp_disk(mask, p.x_mm(), p.y_mm(), r);
                continue;
            }
            for w in pts.windows(2).enumerate() {
                let (i, seg) = w;
                let (a, b) = (seg[0], seg[1]);
                // Per-vertex widths, linearly interpolated along the segment by
                // stamping disks at both endpoints AND along the segment. We
                // stamp a swept capsule by walking sample points dense enough
                // that consecutive disks overlap (step ≤ cell), each disk's
                // radius the interpolated half-width.
                let ra = 0.5 * widths.get(i).copied().unwrap_or(0.0);
                let rb = 0.5 * widths.get(i + 1).copied().unwrap_or(ra * 2.0);
                self.stamp_capsule(mask, a.x_mm(), a.y_mm(), ra, b.x_mm(), b.y_mm(), rb);
            }
        }
    }

    /// Clear cells covered by a single disk of radius `r` (mm) at (cx, cy).
    fn stamp_disk(&self, mask: &mut [bool], cx: f64, cy: f64, r: f64) {
        if r <= 0.0 {
            return;
        }
        let r2 = r * r;
        let (ix0, iy0, ix1, iy1) = self.cell_box(cx - r, cy - r, cx + r, cy + r);
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (x, y) = self.center(ix, iy);
                if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2 {
                    mask[self.idx(ix, iy)] = false;
                }
            }
        }
    }

    /// Clear cells covered by the capsule (stadium) between two disks: every
    /// cell whose distance to the segment AB is ≤ the radius linearly
    /// interpolated by the projection parameter. This is the exact swept-disk
    /// footprint of a bead with endpoint half-widths `ra`, `rb`.
    fn stamp_capsule(&self, mask: &mut [bool], ax: f64, ay: f64, ra: f64, bx: f64, by: f64, rb: f64) {
        let rmax = ra.max(rb);
        if rmax <= 0.0 {
            return;
        }
        let (lo_x, lo_y) = (ax.min(bx) - rmax, ay.min(by) - rmax);
        let (hi_x, hi_y) = (ax.max(bx) + rmax, ay.max(by) + rmax);
        let (ix0, iy0, ix1, iy1) = self.cell_box(lo_x, lo_y, hi_x, hi_y);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (x, y) = self.center(ix, iy);
                // Projection parameter t of the point onto AB, clamped to the
                // segment; the local radius is the lerp of (ra, rb) by t.
                let t = if len2 <= 1e-18 {
                    0.0
                } else {
                    (((x - ax) * dx + (y - ay) * dy) / len2).clamp(0.0, 1.0)
                };
                let px = ax + t * dx;
                let py = ay + t * dy;
                let r = ra + t * (rb - ra);
                let dist2 = (x - px) * (x - px) + (y - py) * (y - py);
                if dist2 <= r * r {
                    mask[self.idx(ix, iy)] = false;
                }
            }
        }
    }

    /// Clamp an mm bounding box to inclusive cell-index ranges.
    #[inline]
    fn cell_box(&self, lo_x: f64, lo_y: f64, hi_x: f64, hi_y: f64) -> (usize, usize, usize, usize) {
        let to_ix = |v: f64| ((v - self.x0) / self.cell - 0.5).round();
        let to_iy = |v: f64| ((v - self.y0) / self.cell - 0.5).round();
        let ix0 = to_ix(lo_x).floor().max(0.0) as usize;
        let iy0 = to_iy(lo_y).floor().max(0.0) as usize;
        let ix1 = (to_ix(hi_x).ceil().max(0.0) as usize).min(self.nx - 1);
        let iy1 = (to_iy(hi_y).ceil().max(0.0) as usize).min(self.ny - 1);
        (ix0, iy0, ix1, iy1)
    }

    /// Increment `count` once for every cell this bead's footprint covers (the
    /// `last`/`bi` guard stops a bead's own overlapping segments double-counting
    /// at shared vertices). Footprint = swept capsules of the per-vertex widths.
    fn stamp_count(&self, count: &mut [u16], last: &mut [usize], bi: usize, pts: &[Point], widths: &[f64]) {
        if pts.is_empty() {
            return;
        }
        if pts.len() == 1 {
            self.count_disk(count, last, bi, pts[0].x_mm(), pts[0].y_mm(), 0.5 * widths.first().copied().unwrap_or(0.0));
            return;
        }
        for (i, seg) in pts.windows(2).enumerate() {
            let ra = 0.5 * widths.get(i).copied().unwrap_or(0.0);
            let rb = 0.5 * widths.get(i + 1).copied().unwrap_or(ra * 2.0);
            self.count_capsule(count, last, bi, seg[0].x_mm(), seg[0].y_mm(), ra, seg[1].x_mm(), seg[1].y_mm(), rb);
        }
    }

    fn count_disk(&self, count: &mut [u16], last: &mut [usize], bi: usize, cx: f64, cy: f64, r: f64) {
        if r <= 0.0 {
            return;
        }
        let r2 = r * r;
        let (ix0, iy0, ix1, iy1) = self.cell_box(cx - r, cy - r, cx + r, cy + r);
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (x, y) = self.center(ix, iy);
                if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2 {
                    let k = self.idx(ix, iy);
                    if last[k] != bi {
                        count[k] = count[k].saturating_add(1);
                        last[k] = bi;
                    }
                }
            }
        }
    }

    fn count_capsule(&self, count: &mut [u16], last: &mut [usize], bi: usize, ax: f64, ay: f64, ra: f64, bx: f64, by: f64, rb: f64) {
        let rmax = ra.max(rb);
        if rmax <= 0.0 {
            return;
        }
        let (ix0, iy0, ix1, iy1) =
            self.cell_box(ax.min(bx) - rmax, ay.min(by) - rmax, ax.max(bx) + rmax, ay.max(by) + rmax);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (x, y) = self.center(ix, iy);
                let t = if len2 <= 1e-18 { 0.0 } else { (((x - ax) * dx + (y - ay) * dy) / len2).clamp(0.0, 1.0) };
                let (px, py) = (ax + t * dx, ay + t * dy);
                let r = ra + t * (rb - ra);
                if (x - px) * (x - px) + (y - py) * (y - py) <= r * r {
                    let k = self.idx(ix, iy);
                    if last[k] != bi {
                        count[k] = count[k].saturating_add(1);
                        last[k] = bi;
                    }
                }
            }
        }
    }

    /// Mean of `1/count` over a capsule (the segment's swept footprint) — the
    /// segment's fair share of the material where `count` beads overlap.
    fn capsule_mean_inv_count(&self, count: &[u16], ax: f64, ay: f64, ra: f64, bx: f64, by: f64, rb: f64) -> f64 {
        let rmax = ra.max(rb);
        let (ix0, iy0, ix1, iy1) =
            self.cell_box(ax.min(bx) - rmax, ay.min(by) - rmax, ax.max(bx) + rmax, ay.max(by) + rmax);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let (mut sum, mut n) = (0.0f64, 0u32);
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (x, y) = self.center(ix, iy);
                let t = if len2 <= 1e-18 { 0.0 } else { (((x - ax) * dx + (y - ay) * dy) / len2).clamp(0.0, 1.0) };
                let (px, py) = (ax + t * dx, ay + t * dy);
                let r = ra + t * (rb - ra);
                if (x - px) * (x - px) + (y - py) * (y - py) <= r * r {
                    let c = count[self.idx(ix, iy)];
                    if c >= 1 {
                        sum += 1.0 / c as f64;
                        n += 1;
                    }
                }
            }
        }
        if n == 0 {
            1.0
        } else {
            sum / n as f64
        }
    }

    /// Flood-fill connected components of the `true` cells, drop the ones below
    /// `min_area_mm2`, and trace each survivor's boundary to a polygon.
    fn components_to_polygons(&self, mask: &[bool], min_area_mm2: f64) -> Polygons {
        let n = self.nx * self.ny;
        let cell_area = self.cell * self.cell;
        let min_cells = (min_area_mm2 / cell_area).ceil().max(1.0) as usize;
        let mut label = vec![0u32; n]; // 0 = unlabeled/false
        let mut next: u32 = 0;
        let mut stack: Vec<usize> = Vec::new();
        let mut out = Polygons::new();

        for start in 0..n {
            if !mask[start] || label[start] != 0 {
                continue;
            }
            next += 1;
            let id = next;
            // BFS/DFS flood (4-connected: a component must be edge-connected so
            // it forms a single closed marching-squares boundary; 8-connected
            // would bridge speckle across a diagonal pinch).
            stack.clear();
            stack.push(start);
            label[start] = id;
            let mut count = 0usize;
            while let Some(c) = stack.pop() {
                count += 1;
                let (ix, iy) = (c % self.nx, c / self.nx);
                let mut push = |nix: usize, niy: usize, stack: &mut Vec<usize>| {
                    let nc = niy * self.nx + nix;
                    if mask[nc] && label[nc] == 0 {
                        label[nc] = id;
                        stack.push(nc);
                    }
                };
                if ix > 0 {
                    push(ix - 1, iy, &mut stack);
                }
                if ix + 1 < self.nx {
                    push(ix + 1, iy, &mut stack);
                }
                if iy > 0 {
                    push(ix, iy - 1, &mut stack);
                }
                if iy + 1 < self.ny {
                    push(ix, iy + 1, &mut stack);
                }
            }
            if count < min_cells {
                continue; // sub-bead speckle
            }
            for c in self.trace_component(&label, id) {
                out.push(c);
            }
        }
        out
    }

    /// Marching squares around one labeled component: the iso-boundary between
    /// `label == id` (inside) and everything else, chained into closed
    /// contour(s). A simply-connected blob yields exactly one contour; a blob
    /// with an enclosed hole yields the outer ring and the hole ring (both come
    /// back CCW/CW per their own winding and are pushed as separate contours).
    fn trace_component(&self, label: &[u32], id: u32) -> Vec<Contour> {
        let inside = |ix: i64, iy: i64| -> bool {
            ix >= 0
                && iy >= 0
                && (ix as usize) < self.nx
                && (iy as usize) < self.ny
                && label[iy as usize * self.nx + ix as usize] == id
        };
        let mut segs: Vec<((f64, f64), (f64, f64))> = Vec::new();
        // Cells of the marching-squares lattice are centered on cell *centers*;
        // a lattice quad spans centers (ix,iy)..(ix+1,iy+1). The guard rim of
        // false cells (Grid::new pads by 2) guarantees the boundary closes.
        for iy in -1..self.ny as i64 {
            for ix in -1..self.nx as i64 {
                let c00 = inside(ix, iy);
                let c10 = inside(ix + 1, iy);
                let c01 = inside(ix, iy + 1);
                let c11 = inside(ix + 1, iy + 1);
                let mut case = 0u8;
                if c00 {
                    case |= 1;
                }
                if c10 {
                    case |= 2;
                }
                if c11 {
                    case |= 4;
                }
                if c01 {
                    case |= 8;
                }
                if case == 0 || case == 15 {
                    continue;
                }
                // Iso-line crosses each lattice edge at its midpoint (binary
                // field): bottom/right/top/left edge midpoints of the quad.
                let p00 = self.center_i(ix, iy);
                let p10 = self.center_i(ix + 1, iy);
                let p01 = self.center_i(ix, iy + 1);
                let p11 = self.center_i(ix + 1, iy + 1);
                let mid = |a: (f64, f64), b: (f64, f64)| ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
                let bottom = mid(p00, p10);
                let right = mid(p10, p11);
                let top = mid(p01, p11);
                let left = mid(p00, p01);
                match case {
                    1 | 14 => segs.push((left, bottom)),
                    2 | 13 => segs.push((bottom, right)),
                    4 | 11 => segs.push((right, top)),
                    8 | 7 => segs.push((top, left)),
                    3 | 12 => segs.push((left, right)),
                    6 | 9 => segs.push((bottom, top)),
                    5 => {
                        // ambiguous saddle: inside at the two off-diagonal
                        // corners (c00,c11). Resolve to keep the inside region
                        // connected through the saddle.
                        segs.push((left, top));
                        segs.push((bottom, right));
                    }
                    10 => {
                        segs.push((left, bottom));
                        segs.push((right, top));
                    }
                    _ => unreachable!(),
                }
            }
        }
        chain_to_contours(segs, self.cell)
    }

    /// Cell center for possibly-negative lattice indices (the guard rim).
    #[inline]
    fn center_i(&self, ix: i64, iy: i64) -> (f64, f64) {
        (self.x0 + (ix as f64 + 0.5) * self.cell, self.y0 + (iy as f64 + 0.5) * self.cell)
    }
}

/// Chain marching-squares segment soup into closed contours, dropping degenerate
/// (<3 point) loops and the duplicated closing vertex.
fn chain_to_contours(segs: Vec<((f64, f64), (f64, f64))>, cell: f64) -> Vec<Contour> {
    let mut out = Vec::new();
    for mut line in crate::fill::chain_segments(segs, cell * 0.25) {
        // chain_segments returns open polylines; a closed loop comes back with
        // its first ≈ last. Drop the duplicate and keep loops with real area.
        if line.len() > 3 {
            let (a, b) = (line[0], line[line.len() - 1]);
            if (a.0 - b.0).hypot(a.1 - b.1) < cell * 0.5 {
                line.pop();
            }
        }
        if line.len() < 3 {
            continue;
        }
        let c = Contour::new(line.iter().map(|&(x, y)| Point::from_mm(x, y)).collect());
        if c.area_mm2() > 0.0 {
            out.push(c);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

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

    /// A straight horizontal bead from (x0,y) to (x1,y), uniform width.
    fn straight_bead(x0: f64, x1: f64, y: f64, width: f64) -> (Vec<Point>, Vec<f64>) {
        (vec![Point::from_mm(x0, y), Point::from_mm(x1, y)], vec![width, width])
    }

    #[test]
    fn single_centre_bead_leaves_two_side_strips() {
        // 6mm × 2mm rectangle, one 0.45mm bead down the centerline (y=1).
        let lw = 0.45;
        let region = rect(0.0, 0.0, 6.0, 2.0);
        let bead = straight_bead(0.0, 6.0, 1.0, lw);
        let voids = uncovered(&region, &[bead], lw, 0.2);

        // Two strips, one above and one below the bead. The bead covers a band
        // of half-width 0.225 about y=1, so each strip is ~(1.0-0.225)=0.775mm
        // tall × ~6mm long ≈ 4.65mm². (Grid quantization trims the corners and
        // the ends a little.)
        assert_eq!(voids.contours.len(), 2, "expected two side strips");
        let mut areas: Vec<f64> = voids.contours.iter().map(|c| c.area_mm2()).collect();
        areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for a in &areas {
            assert!(
                (3.8..5.2).contains(a),
                "side strip area {a:.3} mm² out of expected band"
            );
        }
        // Symmetric: the two strips should be within a few % of each other.
        assert!(
            (areas[1] - areas[0]).abs() < 0.4,
            "strips asymmetric: {areas:?}"
        );
        eprintln!("single_centre_bead: strip areas = {areas:?} mm²");
    }

    #[test]
    fn side_strips_survive_small_min_area() {
        // Same geometry; a min_area that keeps the ~0.78mm-wide strips but is
        // well above sub-bead speckle.
        let lw = 0.45;
        let region = rect(0.0, 0.0, 6.0, 2.0);
        let bead = straight_bead(0.0, 6.0, 1.0, lw);
        let voids = uncovered(&region, &[bead], lw, 2.5);
        assert_eq!(voids.contours.len(), 2, "both strips must clear 2.5mm²");
    }

    #[test]
    fn fully_tiled_region_is_empty() {
        // 6mm × 3mm rectangle tiled by horizontal beads at 0.4mm spacing with
        // 0.45mm width (overlap), from the bottom edge to the top edge → full
        // coverage, no void above min area.
        let lw = 0.45;
        let region = rect(0.0, 0.0, 6.0, 3.0);
        let sp = 0.40;
        let mut beads: Vec<(Vec<Point>, Vec<f64>)> = Vec::new();
        // Beads at y = 0.0, 0.4, 0.8, ... 3.0 so the capsules overrun both edges.
        let mut y = 0.0;
        while y <= 3.0 + 1e-9 {
            beads.push(straight_bead(-0.5, 6.5, y, lw));
            y += sp;
        }
        let voids = uncovered(&region, &beads, lw, 0.5);
        let total: f64 = voids.contours.iter().map(|c| c.area_mm2()).sum();
        eprintln!(
            "fully_tiled: {} void(s), total {total:.4} mm²",
            voids.contours.len()
        );
        assert!(
            voids.contours.is_empty(),
            "fully tiled region left {} void(s), total {total:.4} mm²",
            voids.contours.len()
        );
    }

    #[test]
    fn empty_region_returns_empty() {
        let voids = uncovered(&Polygons::new(), &[], 0.45, 2.5);
        assert!(voids.contours.is_empty());
    }

    #[test]
    fn no_beads_returns_whole_region() {
        // With no beads the entire interior is uncovered → one component close
        // to the region area.
        let region = rect(0.0, 0.0, 6.0, 2.0);
        let voids = uncovered(&region, &[], 0.45, 2.5);
        assert_eq!(voids.contours.len(), 1);
        let a = voids.contours[0].area_mm2();
        assert!((a - 12.0).abs() < 1.0, "whole-region void area {a:.3} mm²");
    }

    #[test]
    fn tapered_bead_leaves_more_void_at_thin_end() {
        // A bead that tapers from 0.6mm to 0.2mm along a 6×2 rect centerline:
        // the thin end leaves a wider gap, so the void hugs the bead's taper.
        let lw = 0.45;
        let region = rect(0.0, 0.0, 6.0, 2.0);
        let bead = (
            vec![Point::from_mm(0.0, 1.0), Point::from_mm(6.0, 1.0)],
            vec![0.6, 0.2],
        );
        let voids = uncovered(&region, &[bead], lw, 0.2);
        let total: f64 = voids.contours.iter().map(|c| c.area_mm2()).sum();
        // Total void is the region minus the tapered capsule footprint
        // (avg half-width 0.2, length 6 → ~2.4mm² covered) → ~9.6mm² void.
        assert!(total > 8.5 && total < 11.0, "tapered void total {total:.3} mm²");
        eprintln!("tapered_bead: total void {total:.3} mm² across {} comps", voids.contours.len());
    }
}
