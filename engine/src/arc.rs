//! Arc-overhang toolpaths (Steven McCulloch's technique): fill a flat overhang
//! region with concentric arcs anchored on the supported edge, so each bead rests
//! on already-printed material and the overhang prints without support.
//!
//! Coverage is tracked on a line-width grid for robustness/speed; the arcs
//! themselves are analytic circular arcs (smooth polylines). Starting from
//! centers on the supported boundary, concentric rings grow outward (spaced by
//! the line width, up to `rmax`); the farthest point reached becomes the next
//! center (breadth-first), spreading until the region is covered.

use std::collections::VecDeque;

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

    let center_xy = |ix: usize, iy: usize| (x0 + (ix as f64 + 0.5) * cell, y0 + (iy as f64 + 0.5) * cell);
    let cell_of = |x: f64, y: f64| -> Option<usize> {
        let (fx, fy) = (((x - x0) / cell).floor(), ((y - y0) / cell).floor());
        if fx < 0.0 || fy < 0.0 {
            return None;
        }
        let (ix, iy) = (fx as usize, fy as usize);
        (ix < nx && iy < ny).then_some(iy * nx + ix)
    };

    // Classify cells.
    let mut grid = vec![OUT; nx * ny];
    let mut unfilled = 0usize;
    for iy in 0..ny {
        for ix in 0..nx {
            let (x, y) = center_xy(ix, iy);
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
    let mut queue: VecDeque<(f64, f64)> = VecDeque::new();
    seed_centers(&grid, nx, ny, &center_xy, rmax, &mut queue);

    let mut guard = 0usize;
    let cap = 100_000usize;
    while unfilled > 0 && guard < cap {
        guard += 1;
        let (cx, cy) = match queue.pop_front() {
            Some(c) => c,
            None => {
                seed_centers(&grid, nx, ny, &center_xy, rmax, &mut queue);
                match queue.pop_front() {
                    Some(c) => c,
                    None => break,
                }
            }
        };

        let mut farthest: Option<(f64, f64)> = None;
        let mut farthest_d = 0.0;
        let mut empty_rings = 0;
        let mut r = cell;
        while r <= rmax {
            let dtheta = (cell / r).clamp(0.01, 0.4);
            let mut run: Vec<Point> = Vec::new();
            let mut any = false;
            let mut theta = 0.0;
            while theta < std::f64::consts::TAU {
                let (x, y) = (cx + r * theta.cos(), cy + r * theta.sin());
                let is_fillable = matches!(cell_of(x, y).map(|c| grid[c]), Some(UNFILLED));
                if is_fillable {
                    let c = cell_of(x, y).unwrap();
                    grid[c] = FILLED;
                    unfilled -= 1;
                    run.push(Point::from_mm(x, y));
                    any = true;
                    let d = r; // distance from center is the radius
                    if d > farthest_d {
                        farthest_d = d;
                        farthest = Some((x, y));
                    }
                } else if run.len() >= 2 {
                    arcs.push(std::mem::take(&mut run));
                } else {
                    run.clear();
                }
                theta += dtheta;
            }
            if run.len() >= 2 {
                arcs.push(run);
            }
            if any {
                empty_rings = 0;
            } else {
                empty_rings += 1;
                if empty_rings >= 4 {
                    break; // nothing reachable for several rings outward
                }
            }
            r += cell;
        }
        if let Some(f) = farthest {
            queue.push_back(f);
        }
    }
    arcs
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

/// Seed centers on FILLED (supported/covered) cells that border an UNFILLED cell,
/// spaced ~`rmax` apart so starts spread out. Falls back to region-edge cells if
/// the region doesn't touch any supported cell.
fn seed_centers(
    grid: &[u8],
    nx: usize,
    ny: usize,
    center_xy: &impl Fn(usize, usize) -> (f64, f64),
    rmax: f64,
    queue: &mut VecDeque<(f64, f64)>,
) {
    let borders_unfilled = |ix: usize, iy: usize| {
        let mut n = false;
        if ix > 0 && grid[iy * nx + ix - 1] == UNFILLED {
            n = true;
        }
        if ix + 1 < nx && grid[iy * nx + ix + 1] == UNFILLED {
            n = true;
        }
        if iy > 0 && grid[(iy - 1) * nx + ix] == UNFILLED {
            n = true;
        }
        if iy + 1 < ny && grid[(iy + 1) * nx + ix] == UNFILLED {
            n = true;
        }
        n
    };
    let min_sep = (rmax * 0.5).max(1.0);
    let mut accepted: Vec<(f64, f64)> = Vec::new();
    // Primary: covered/supported cells touching the unfilled region.
    for pass in 0..2 {
        for iy in 0..ny {
            for ix in 0..nx {
                let want = if pass == 0 {
                    grid[iy * nx + ix] == FILLED && borders_unfilled(ix, iy)
                } else {
                    accepted.is_empty() && grid[iy * nx + ix] == UNFILLED
                };
                if !want {
                    continue;
                }
                let (x, y) = center_xy(ix, iy);
                if accepted.iter().all(|&(ax, ay)| (ax - x).hypot(ay - y) >= min_sep) {
                    accepted.push((x, y));
                    queue.push_back((x, y));
                }
            }
        }
        if !accepted.is_empty() {
            break;
        }
    }
}
