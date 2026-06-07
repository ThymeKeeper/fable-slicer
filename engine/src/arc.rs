//! Arc-overhang toolpaths (Steven McCulloch's technique): fill a flat overhang /
//! bridge region with concentric arcs anchored on the supported edge, so each bead
//! rests on already-printed material and the span prints without support.
//!
//! Coverage is tracked on a line-width grid; the arcs are analytic circular arcs.
//! Multiple centers are seeded on the supported border (preferring concave
//! corners) and their concentric rings grow outward in lockstep — so fans advance
//! from several sides and meet in the middle, letting bridges span further than a
//! single corner fan.
//!
//! Each ring is emitted as a continuous arc but stops at: the region boundary,
//! anchor (supported) cells, and cells owned by a *different* fan. So arcs stay
//! inside the overhang (never drawing over neighbouring regions), don't break on
//! their own fan's prior rings (no aliasing gaps), and meet cleanly where two fans
//! touch (no overlap).

use geo2d::{Point, Polygons};

const OUT: u8 = 0; // outside the overhang region
const REGION: u8 = 1; // overhang region (owner 0 = unfilled, else the fan that filled it)
const ANCHOR: u8 = 2; // supported material bordering the region (seed from here, never draw on)

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

    let mut kind = vec![OUT; nx * ny];
    let mut owner = vec![0u32; nx * ny];
    let mut unfilled = 0usize;
    for iy in 0..ny {
        for ix in 0..nx {
            let (x, y) = g.center(ix, iy);
            if in_poly(region, x, y) {
                kind[iy * nx + ix] = REGION;
                unfilled += 1;
            } else if in_poly(supported, x, y) {
                kind[iy * nx + ix] = ANCHOR;
            }
        }
    }
    if unfilled == 0 {
        return Vec::new();
    }

    let mut arcs: Vec<Vec<Point>> = Vec::new();
    let mut fronts: Vec<Front> = Vec::new();
    let mut next_id = 1u32;
    let mut guard = 0usize;
    let cap = 400_000usize;

    while unfilled > 0 && guard < cap {
        if fronts.is_empty() {
            let seeds = seed_centers(&kind, &owner, &g, rmax);
            if seeds.is_empty() {
                break;
            }
            for (x, y) in seeds {
                fronts.push(Front { x, y, r: cell, id: next_id });
                next_id += 1;
            }
        }
        let mut next: Vec<Front> = Vec::new();
        for f in &fronts {
            guard += 1;
            if guard >= cap {
                break;
            }
            if f.r > rmax {
                continue; // fan reached its limit; drop it
            }
            if draw_ring(f, &g, &kind, &mut owner, &mut unfilled, &mut arcs) {
                next.push(Front { x: f.x, y: f.y, r: f.r + cell, id: f.id });
            }
        }
        fronts = next;
    }
    arcs
}

struct Front {
    x: f64,
    y: f64,
    r: f64,
    id: u32,
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

/// Emit `circle(f, r)` where it lies in this fan's reachable region: cells in the
/// overhang that are unfilled or already this fan's. Runs break at the region
/// edge, anchor cells, and other fans' cells. A run is emitted (and its unfilled
/// cells claimed) only if it touches still-unfilled region.
fn draw_ring(f: &Front, g: &Grid, kind: &[u8], owner: &mut [u32], unfilled: &mut usize, arcs: &mut Vec<Vec<Point>>) -> bool {
    let dtheta = (g.cell / f.r).clamp(0.01, 0.4);
    let mut runs: Vec<(Vec<Point>, Vec<usize>, bool)> = Vec::new();
    let mut pts: Vec<Point> = Vec::new();
    let mut cells: Vec<usize> = Vec::new();
    let mut has_new = false;
    let mut theta = 0.0;
    while theta < std::f64::consts::TAU {
        let (x, y) = (f.x + f.r * theta.cos(), f.y + f.r * theta.sin());
        let reachable = match g.index(x, y) {
            Some(ci) if kind[ci] == REGION && (owner[ci] == 0 || owner[ci] == f.id) => Some(ci),
            _ => None,
        };
        match reachable {
            Some(ci) => {
                pts.push(Point::from_mm(x, y));
                cells.push(ci);
                if owner[ci] == 0 {
                    has_new = true;
                }
            }
            None => {
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
            if owner[ci] == 0 {
                owner[ci] = f.id;
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

/// Seed centers on anchor / already-covered cells that border the unfilled region,
/// preferring concave corners (more unfilled neighbours) and spaced ~`rmax*0.6`
/// apart so fans start from several sides (e.g. opposite edges of a bridge).
fn seed_centers(kind: &[u8], owner: &[u32], g: &Grid, rmax: f64) -> Vec<(f64, f64)> {
    let (nx, ny) = (g.nx, g.ny);
    let is_unfilled = |c: usize| kind[c] == REGION && owner[c] == 0;
    let unfilled_neighbours = |ix: usize, iy: usize| {
        let mut n = 0;
        if ix > 0 && is_unfilled(iy * nx + ix - 1) {
            n += 1;
        }
        if ix + 1 < nx && is_unfilled(iy * nx + ix + 1) {
            n += 1;
        }
        if iy > 0 && is_unfilled((iy - 1) * nx + ix) {
            n += 1;
        }
        if iy + 1 < ny && is_unfilled((iy + 1) * nx + ix) {
            n += 1;
        }
        n
    };
    // Candidates: supported (anchor) or already-printed cells touching unfilled
    // region, scored by how corner-like they are.
    let mut cands: Vec<(i32, f64, f64)> = Vec::new();
    for iy in 0..ny {
        for ix in 0..nx {
            let c = iy * nx + ix;
            let is_support = kind[c] == ANCHOR || (kind[c] == REGION && owner[c] != 0);
            if is_support {
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
    let min_sep = (rmax * 0.6).max(g.cell * 2.0);
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

    #[test]
    fn arcs_stay_inside_the_region() {
        let lw = 0.45;
        let region = rect(0.0, 0.0, 20.0, 20.0);
        let supported = rect(-5.0, -5.0, 25.0, 0.0); // wide anchor below
        for a in arc_fill(&region, &supported, lw, 40.0) {
            for p in a {
                let (x, y) = (p.x_mm(), p.y_mm());
                // arcs must not stray into the anchor (y<0) or outside the region
                assert!(x > -lw && x < 20.0 + lw && y > -lw && y < 20.0 + lw, "arc point ({x:.2},{y:.2}) left the region");
            }
        }
    }
}
