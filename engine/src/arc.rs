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
    let mut last_reseed_unfilled = usize::MAX;

    // Initial seeds at the region's convex corners (fall back to spreading along
    // the supported border for a corner-free region). Chaining covers the rest.
    {
        let mut seeds = corner_seeds(region, &g, &kind, lw);
        if seeds.is_empty() {
            seeds = border_seeds(&kind, &owner, &g, rmax);
        }
        for (x, y) in seeds {
            fronts.push(Front { x, y, r: cell, id: next_id, far: None });
            next_id += 1;
        }
    }

    while unfilled > 0 && guard < cap {
        if fronts.is_empty() {
            // Reseed from the frontier of whatever's left; stop once a reseed
            // cycle makes no progress (the scanline cleanup finishes the cusps).
            if unfilled >= last_reseed_unfilled {
                break;
            }
            last_reseed_unfilled = unfilled;
            let seeds = border_seeds(&kind, &owner, &g, rmax);
            if seeds.is_empty() {
                break;
            }
            for (x, y) in seeds {
                fronts.push(Front { x, y, r: cell, id: next_id, far: None });
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
                chain_spawn(f.far, &g, &kind, &owner, &mut next, &mut next_id);
                continue; // radius limit — continue from the fan's frontier
            }
            let (grew, ring_far) = draw_ring(f, &g, &kind, &mut owner, &mut unfilled, &mut arcs);
            if grew {
                next.push(Front { x: f.x, y: f.y, r: f.r + cell, id: f.id, far: ring_far.or(f.far) });
            } else {
                chain_spawn(f.far, &g, &kind, &owner, &mut next, &mut next_id);
            }
        }
        fronts = next;
    }

    // Cleanup: cusps left where fans meet (concentric circles can't tile a corner)
    // get short scanline segments, supported on both sides by the surrounding arcs.
    for iy in 0..ny {
        let mut start: Option<usize> = None;
        for ix in 0..=nx {
            let gap = ix < nx && kind[iy * nx + ix] == REGION && owner[iy * nx + ix] == 0;
            if gap {
                if start.is_none() {
                    start = Some(ix);
                }
            } else if let Some(s) = start.take() {
                let e = ix - 1;
                if e > s {
                    let (x0, y) = g.center(s, iy);
                    let (x1, _) = g.center(e, iy);
                    arcs.push(vec![Point::from_mm(x0, y), Point::from_mm(x1, y)]);
                    for cx in s..=e {
                        owner[iy * nx + cx] = u32::MAX;
                    }
                }
            }
        }
    }
    arcs
}

struct Front {
    x: f64,
    y: f64,
    r: f64,
    id: u32,
    /// Farthest frontier point this fan has reached (next center when it stalls).
    far: Option<(f64, f64)>,
}

/// Continue a stalled fan by starting a new fan at its farthest frontier point
/// (McCulloch chaining) — but only if that point still borders unfilled region.
fn chain_spawn(far: Option<(f64, f64)>, g: &Grid, kind: &[u8], owner: &[u32], next: &mut Vec<Front>, next_id: &mut u32) {
    if let Some((fx, fy)) = far {
        if let Some(ci) = g.index(fx, fy) {
            if has_unfilled_neighbour(g, kind, owner, ci) {
                next.push(Front { x: fx, y: fy, r: g.cell, id: *next_id, far: None });
                *next_id += 1;
            }
        }
    }
}

fn has_unfilled_neighbour(g: &Grid, kind: &[u8], owner: &[u32], ci: usize) -> bool {
    let (nx, ny) = (g.nx, g.ny);
    let (ix, iy) = (ci % nx, ci / nx);
    let unfilled = |c: usize| kind[c] == REGION && owner[c] == 0;
    (ix > 0 && unfilled(ci - 1))
        || (ix + 1 < nx && unfilled(ci + 1))
        || (iy > 0 && unfilled(ci - nx))
        || (iy + 1 < ny && unfilled(ci + nx))
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
fn draw_ring(f: &Front, g: &Grid, kind: &[u8], owner: &mut [u32], unfilled: &mut usize, arcs: &mut Vec<Vec<Point>>) -> (bool, Option<(f64, f64)>) {
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

    // Mark the new runs' cells as this fan's, then pick a frontier point (a just-
    // printed cell still bordering unfilled region) as the fan's next center.
    let mut emitted: Vec<(Vec<Point>, Vec<usize>)> = Vec::new();
    for (p, c, new) in runs {
        if p.len() < 2 || !new {
            continue;
        }
        for &ci in &c {
            if owner[ci] == 0 {
                owner[ci] = f.id;
                *unfilled -= 1;
            }
        }
        emitted.push((p, c));
    }
    if emitted.is_empty() {
        return (false, None);
    }
    let mut chain = None;
    'find: for (p, c) in &emitted {
        for (k, &ci) in c.iter().enumerate() {
            if has_unfilled_neighbour(g, kind, owner, ci) {
                chain = Some((p[k].x_mm(), p[k].y_mm()));
                break 'find;
            }
        }
    }
    for (p, _) in emitted {
        arcs.push(p);
    }
    (true, chain)
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

/// Find the region's **convex corners** from the polygon and seed a fan at each
/// one that sits on supported material. Uses a windowed turn angle (direction
/// ~`lw*4` before vs after each vertex) so it survives the rounding that
/// morphological-open leaves on corners and ignores gentle curves / staircase
/// noise. One seed per corner.
fn corner_seeds(region: &Polygons, g: &Grid, kind: &[u8], lw: f64) -> Vec<(f64, f64)> {
    let w = (lw * 4.0).max(1.0); // look-around arc length (mm)
    let mut found: Vec<(f64, f64, f64)> = Vec::new(); // (cornerness, x, y)
    for c in &region.contours {
        let n = c.points.len();
        if n < 4 {
            continue;
        }
        let pt = |k: usize| (c.points[k % n].x_mm(), c.points[k % n].y_mm());
        for i in 0..n {
            let (vx, vy) = pt(i);
            // Walk back and forward ~w to get the incoming/outgoing directions.
            let walk = |dir: i64| {
                let (mut x, mut y) = (vx, vy);
                let mut acc = 0.0;
                let mut k = i as i64;
                for _ in 0..n {
                    let kk = (k + dir).rem_euclid(n as i64) as usize;
                    let (px, py) = pt(kk);
                    acc += (x - px).hypot(y - py);
                    x = px;
                    y = py;
                    k = kk as i64;
                    if acc >= w {
                        break;
                    }
                }
                (x, y)
            };
            let (bx, by) = walk(-1);
            let (fx, fy) = walk(1);
            let (inx, iny) = (vx - bx, vy - by);
            let (outx, outy) = (fx - vx, fy - vy);
            if inx.hypot(iny) < 1e-6 || outx.hypot(outy) < 1e-6 {
                continue;
            }
            let turn = (inx * outy - iny * outx).atan2(inx * outx + iny * outy).abs();
            if turn > 1.0 && near_anchor(g, kind, vx, vy) {
                found.push((turn, vx, vy));
            }
        }
    }
    found.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    dedup(found.into_iter().map(|(_, x, y)| (x, y)).collect(), w * 1.5)
}

/// True if an ANCHOR cell sits within ~2 cells of `(x, y)`.
fn near_anchor(g: &Grid, kind: &[u8], x: f64, y: f64) -> bool {
    let Some(ci) = g.index(x, y) else {
        return false;
    };
    let (nx, ny) = (g.nx, g.ny);
    let (ix, iy) = ((ci % nx) as i64, (ci / nx) as i64);
    for dy in -2..=2i64 {
        for dx in -2..=2i64 {
            let (jx, jy) = (ix + dx, iy + dy);
            if jx >= 0 && jy >= 0 && (jx as usize) < nx && (jy as usize) < ny && kind[jy as usize * nx + jx as usize] == ANCHOR {
                return true;
            }
        }
    }
    false
}

/// Fallback / reseed: anchor or already-covered cells bordering the unfilled
/// region, spread ~`rmax*0.3` apart.
fn border_seeds(kind: &[u8], owner: &[u32], g: &Grid, rmax: f64) -> Vec<(f64, f64)> {
    let (nx, ny) = (g.nx, g.ny);
    let unfilled = |c: usize| kind[c] == REGION && owner[c] == 0;
    let mut cands: Vec<(f64, f64)> = Vec::new();
    for iy in 0..ny {
        for ix in 0..nx {
            let c = iy * nx + ix;
            let is_support = kind[c] == ANCHOR || (kind[c] == REGION && owner[c] != 0);
            let borders = (ix > 0 && unfilled(c - 1))
                || (ix + 1 < nx && unfilled(c + 1))
                || (iy > 0 && unfilled(c - nx))
                || (iy + 1 < ny && unfilled(c + nx));
            if is_support && borders {
                cands.push(g.center(ix, iy));
            }
        }
    }
    dedup(cands, (rmax * 0.3).max(g.cell * 8.0))
}

/// Greedily keep points at least `min_sep` apart (input pre-sorted by priority).
fn dedup(pts: Vec<(f64, f64)>, min_sep: f64) -> Vec<(f64, f64)> {
    let mut chosen: Vec<(f64, f64)> = Vec::new();
    for (x, y) in pts {
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
        // Beads are ~lw apart, so length*lw ≈ area; the cleanup pass closes the
        // fan-meeting cusps, so coverage should be high.
        let covered = len * lw;
        assert!(covered > 400.0 * 0.85, "coverage {covered:.0}mm² of 400 — gaps?");
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
