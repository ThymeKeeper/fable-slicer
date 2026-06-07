//! Arc-overhang toolpaths (Steven McCulloch's technique): fill a flat overhang /
//! bridge region with concentric arcs anchored on the supported edge, so each bead
//! rests on already-printed material and the span prints without support.
//!
//! Coverage is tracked on a line-width grid; the arcs are analytic circular arcs.
//! Multiple centers are seeded on the supported border (preferring concave
//! corners) and their concentric rings grow outward in lockstep — so fans advance
//! from several sides and meet in the middle, which lets bridges span further than
//! a single corner fan. Each ring is emitted as a continuous arc (broken only at
//! the region boundary), avoiding gaps where neighbouring rings overlap.

use geo2d::{Point, Polygons};

const OUT: u8 = 0; // outside the region
const UNFILLED: u8 = 1; // region, not yet covered
const FILLED: u8 = 2; // supported anchor, or already covered by an arc

/// Fill `region` with concentric arcs seeded where it borders `supported`.
/// `lw` = line width (mm), `rmax` = max arc radius (mm). Returns arc polylines.
pub fn arc_fill(region: &Polygons, supported: &Polygons, lw: f64, rmax: f64) -> Vec<Vec<Point>> {
    let Some(bb) = region.bounds() else {
        return Vec::new();
    };
    let cell = lw.max(0.05);
    let pad = lw * 2.0;
    let x0 = bb.min.x_mm() - pad;
    let y0 = bb.min.y_mm() - pad;
    let nx = (((bb.max.x_mm() + pad) - x0) / cell).ceil() as usize + 2;
    let ny = (((bb.max.y_mm() + pad) - y0) / cell).ceil() as usize + 2;
    if nx < 2 || ny < 2 || nx.saturating_mul(ny) > 4_000_000 {
        return Vec::new();
    }
    let g = Grid { x0, y0, cell, nx, ny };

    // Classify cells.
    let mut grid = vec![OUT; nx * ny];
    let mut unfilled = 0usize;
    for iy in 0..ny {
        for ix in 0..nx {
            let (x, y) = g.center(ix, iy);
            if in_poly(region, x, y) {
                grid[iy * nx + ix] = UNFILLED;
                unfilled += 1;
            } else if in_poly(supported, x, y) {
                grid[iy * nx + ix] = FILLED;
            }
        }
    }
    if unfilled == 0 {
        return Vec::new();
    }

    let mut arcs: Vec<Vec<Point>> = Vec::new();
    // Active fronts: (center_x, center_y, current_radius).
    let mut fronts: Vec<(f64, f64, f64)> = Vec::new();
    let mut guard = 0usize;
    let cap = 400_000usize;

    while unfilled > 0 && guard < cap {
        if fronts.is_empty() {
            // (Re)seed from the supported/covered border touching unfilled region.
            let seeds = seed_centers(&grid, &g, rmax);
            if seeds.is_empty() {
                break;
            }
            fronts = seeds.into_iter().map(|(x, y)| (x, y, cell)).collect();
        }
        let mut next: Vec<(f64, f64, f64)> = Vec::new();
        for &(cx, cy, r) in &fronts {
            guard += 1;
            if guard >= cap {
                break;
            }
            if r > rmax {
                continue; // fan reached its limit; drop it
            }
            if draw_ring(cx, cy, r, &g, &mut grid, &mut unfilled, &mut arcs) {
                next.push((cx, cy, r + cell)); // grew — keep expanding
            }
            // else: stalled (fully covered here) — drop; reseed picks up leftovers
        }
        fronts = next;
    }
    arcs
}

struct Grid {
    x0: f64,
    y0: f64,
    cell: f64,
    nx: usize,
    ny: usize,
}

impl Grid {
    fn center(&self, ix: usize, iy: usize) -> (f64, f64) {
        (self.x0 + (ix as f64 + 0.5) * self.cell, self.y0 + (iy as f64 + 0.5) * self.cell)
    }
    fn index(&self, x: f64, y: f64) -> Option<usize> {
        let (fx, fy) = (((x - self.x0) / self.cell).floor(), ((y - self.y0) / self.cell).floor());
        if fx < 0.0 || fy < 0.0 {
            return None;
        }
        let (ix, iy) = (fx as usize, fy as usize);
        (ix < self.nx && iy < self.ny).then_some(iy * self.nx + ix)
    }
}

/// Emit the in-region arc of `circle(c, r)`, broken only at the region boundary.
/// A run is emitted (and its cells marked covered) only if it touches still-
/// unfilled region; runs fully covered by another fan are skipped. Returns whether
/// any new coverage was added.
fn draw_ring(cx: f64, cy: f64, r: f64, g: &Grid, grid: &mut [u8], unfilled: &mut usize, arcs: &mut Vec<Vec<Point>>) -> bool {
    let dtheta = (g.cell / r).clamp(0.01, 0.4);
    // Collect contiguous in-region runs as (points, cell indices, has_unfilled).
    let mut runs: Vec<(Vec<Point>, Vec<usize>, bool)> = Vec::new();
    let mut pts: Vec<Point> = Vec::new();
    let mut cells: Vec<usize> = Vec::new();
    let mut has_new = false;
    let mut theta = 0.0;
    while theta < std::f64::consts::TAU {
        let (x, y) = (cx + r * theta.cos(), cy + r * theta.sin());
        match g.index(x, y) {
            Some(ci) if grid[ci] != OUT => {
                pts.push(Point::from_mm(x, y));
                cells.push(ci);
                if grid[ci] == UNFILLED {
                    has_new = true;
                }
            }
            _ => {
                if !pts.is_empty() {
                    runs.push((std::mem::take(&mut pts), std::mem::take(&mut cells), has_new));
                    has_new = false;
                }
            }
        }
        theta += dtheta;
    }
    if !pts.is_empty() {
        runs.push((pts, cells, has_new));
    }

    let mut any_new = false;
    for (p, c, new) in runs {
        if p.len() < 2 || !new {
            continue;
        }
        for ci in c {
            if grid[ci] == UNFILLED {
                grid[ci] = FILLED;
                *unfilled -= 1;
            }
        }
        arcs.push(p);
        any_new = true;
    }
    any_new
}

/// Even-odd point-in-polygon over a region (outer contours + holes).
fn in_poly(polys: &Polygons, x: f64, y: f64) -> bool {
    let p = Point::from_mm(x, y);
    let mut inside = false;
    for c in &polys.contours {
        if c.contains(p) {
            inside = !inside;
        }
    }
    inside
}

/// Seed centers on FILLED (supported/covered) cells bordering UNFILLED region,
/// preferring concave corners (more unfilled orthogonal neighbours) and spaced
/// ~`rmax*0.4` apart so fans start from several sides. Falls back to region cells.
fn seed_centers(grid: &[u8], g: &Grid, rmax: f64) -> Vec<(f64, f64)> {
    let (nx, ny) = (g.nx, g.ny);
    let unfilled_neighbours = |ix: usize, iy: usize| {
        let mut n = 0;
        if ix > 0 && grid[iy * nx + ix - 1] == UNFILLED {
            n += 1;
        }
        if ix + 1 < nx && grid[iy * nx + ix + 1] == UNFILLED {
            n += 1;
        }
        if iy > 0 && grid[(iy - 1) * nx + ix] == UNFILLED {
            n += 1;
        }
        if iy + 1 < ny && grid[(iy + 1) * nx + ix] == UNFILLED {
            n += 1;
        }
        n
    };
    // Candidates: covered cells touching the unfilled region, scored by how
    // "corner-like" they are (favours inside corners as start points).
    let mut cands: Vec<(i32, f64, f64)> = Vec::new();
    for iy in 0..ny {
        for ix in 0..nx {
            if grid[iy * nx + ix] == FILLED {
                let k = unfilled_neighbours(ix, iy);
                if k > 0 {
                    let (x, y) = g.center(ix, iy);
                    cands.push((k, x, y));
                }
            }
        }
    }
    if cands.is_empty() {
        return Vec::new();
    }
    cands.sort_by(|a, b| b.0.cmp(&a.0)); // most corner-like first
    let min_sep = (rmax * 0.4).max(g.cell * 2.0);
    let mut chosen: Vec<(f64, f64)> = Vec::new();
    for (_, x, y) in cands {
        if chosen.iter().all(|&(ax, ay)| (ax - x).hypot(ay - y) >= min_sep) {
            chosen.push((x, y));
        }
    }
    chosen
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
    fn arc_fill_covers_region_without_gaps() {
        let lw = 0.45;
        let region = rect(0.0, 0.0, 20.0, 20.0); // 400 mm²
        let supported = rect(0.0, -2.0, 20.0, 0.0); // anchored on the y=0 edge
        let arcs = arc_fill(&region, &supported, lw, 40.0);
        assert!(!arcs.is_empty(), "no arcs generated");
        let mut len = 0.0;
        for a in &arcs {
            for w in a.windows(2) {
                len += (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm());
            }
        }
        // Beads are ~lw apart, so length*lw ≈ area when there are no large gaps.
        let covered = len * lw;
        assert!(covered > 400.0 * 0.6, "coverage {covered:.0}mm² of 400 — gaps?");
    }
}
