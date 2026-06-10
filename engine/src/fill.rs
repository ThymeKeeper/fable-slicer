//! Fill-pattern generation: straight scanline fills (lines / grid / triangles),
//! the gyroid level-set, and helpers shared by solid fill, sparse infill, gap
//! fill, and ironing.
//!
//! All generators return open polylines clipped to the target region; `plan`
//! wraps them into `ToolPath`s.

use geo2d::{Point, Polygons};

/// Generate straight infill lines at `angle_deg`, spaced `spacing_mm` apart,
/// clipped to `region` via an even-odd scanline. The region is rotated so infill
/// lines become horizontal scanlines, then results are rotated back.
///
/// Scanlines come back bottom-to-top; with `boustrophedon` every other line is
/// reversed so consecutive lines connect with short hops (and a monotonic
/// printing order falls out of just keeping this order).
pub fn infill_lines(region: &Polygons, angle_deg: f64, spacing_mm: f64, boustrophedon: bool) -> Vec<Vec<Point>> {
    let theta = angle_deg.to_radians();
    let (ct, st) = (theta.cos(), theta.sin());
    let rot = |x: f64, y: f64| (x * ct + y * st, -x * st + y * ct);
    let unrot = |x: f64, y: f64| (x * ct - y * st, x * st + y * ct);

    let mut edges: Vec<(f64, f64, f64, f64)> = Vec::new();
    let (mut ymin, mut ymax) = (f64::INFINITY, f64::NEG_INFINITY);
    for c in &region.contours {
        let m = c.points.len();
        if m < 3 {
            continue;
        }
        for j in 0..m {
            let a = c.points[j];
            let b = c.points[(j + 1) % m];
            let (ax, ay) = rot(a.x_mm(), a.y_mm());
            let (bx, by) = rot(b.x_mm(), b.y_mm());
            edges.push((ax, ay, bx, by));
            ymin = ymin.min(ay).min(by);
            ymax = ymax.max(ay).max(by);
        }
    }
    if !ymin.is_finite() {
        return Vec::new();
    }

    let mut out = Vec::new();
    let mut row = 0usize;
    let mut y = ymin + spacing_mm * 0.5;
    while y < ymax {
        let mut xs: Vec<f64> = Vec::new();
        for &(ax, ay, bx, by) in &edges {
            // half-open test so each vertex counts once
            if (ay <= y) != (by <= y) {
                let t = (y - ay) / (by - ay);
                xs.push(ax + t * (bx - ax));
            }
        }
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let mut k = 0;
        while k + 1 < xs.len() {
            let (x0, x1) = (xs[k], xs[k + 1]);
            // Skip dabs shorter than this — not worth extruding.
            if x1 - x0 > 0.5 {
                let (px0, py0) = unrot(x0, y);
                let (px1, py1) = unrot(x1, y);
                let mut seg = vec![Point::from_mm(px0, py0), Point::from_mm(px1, py1)];
                if boustrophedon && row % 2 == 1 {
                    seg.reverse();
                }
                out.push(seg);
            }
            k += 2;
        }
        y += spacing_mm;
        row += 1;
    }
    out
}

/// Gyroid infill: the zero level set of `sin x·cos y + sin y·cos z + sin z·cos x`
/// sampled at this layer's z, traced with marching squares and clipped to the
/// region. The period is chosen so the mean line spacing matches `spacing_mm`,
/// and the pattern drifts with `z_mm` so successive layers interlock into the 3D
/// gyroid surface (strong in every direction at low densities).
pub fn gyroid_lines(region: &Polygons, spacing_mm: f64, z_mm: f64) -> Vec<Vec<Point>> {
    let Some(bb) = region.bounds() else {
        return Vec::new();
    };
    // One full gyroid period produces two line "sheets", so period = 2×spacing
    // gives a mean line distance of `spacing_mm`.
    let period = (2.0 * spacing_mm).max(0.2);
    let k = std::f64::consts::TAU / period;
    let (sz, cz) = (k * z_mm).sin_cos();
    let f = move |x: f64, y: f64| {
        let (sx, cx) = (k * x).sin_cos();
        let (sy, cy) = (k * y).sin_cos();
        sx * cy + sy * cz + sz * cx
    };

    // Marching-squares cell size: fine enough to keep the curve smooth, capped
    // so huge sparse regions don't explode (the curve wiggles at period scale).
    let cell = (period / 16.0).clamp(0.1, 1.0);
    let pad = cell;
    let x0 = bb.min.x_mm() - pad;
    let y0 = bb.min.y_mm() - pad;
    let nx = (((bb.max.x_mm() + pad) - x0) / cell).ceil() as usize + 1;
    let ny = (((bb.max.y_mm() + pad) - y0) / cell).ceil() as usize + 1;
    if nx < 2 || ny < 2 || nx.saturating_mul(ny) > 16_000_000 {
        return Vec::new();
    }

    // Sample the field at grid corners (row-major).
    let mut vals = vec![0.0f64; nx * ny];
    for iy in 0..ny {
        for ix in 0..nx {
            vals[iy * nx + ix] = f(x0 + ix as f64 * cell, y0 + iy as f64 * cell);
        }
    }

    // Marching squares: emit one or two segments per cell where the sign changes.
    let interp = |xa: f64, ya: f64, va: f64, xb: f64, yb: f64, vb: f64| {
        let t = if (vb - va).abs() < 1.0e-12 { 0.5 } else { -va / (vb - va) };
        (xa + t * (xb - xa), ya + t * (yb - ya))
    };
    let mut segs: Vec<((f64, f64), (f64, f64))> = Vec::new();
    for iy in 0..ny - 1 {
        for ix in 0..nx - 1 {
            let (xa, ya) = (x0 + ix as f64 * cell, y0 + iy as f64 * cell);
            let (xb, yb) = (xa + cell, ya + cell);
            let v00 = vals[iy * nx + ix];
            let v10 = vals[iy * nx + ix + 1];
            let v01 = vals[(iy + 1) * nx + ix];
            let v11 = vals[(iy + 1) * nx + ix + 1];
            let mut case = 0u8;
            if v00 > 0.0 {
                case |= 1;
            }
            if v10 > 0.0 {
                case |= 2;
            }
            if v11 > 0.0 {
                case |= 4;
            }
            if v01 > 0.0 {
                case |= 8;
            }
            if case == 0 || case == 15 {
                continue;
            }
            // Edge crossings: bottom (00-10), right (10-11), top (01-11), left (00-01).
            let bottom = || interp(xa, ya, v00, xb, ya, v10);
            let right = || interp(xb, ya, v10, xb, yb, v11);
            let top = || interp(xa, yb, v01, xb, yb, v11);
            let left = || interp(xa, ya, v00, xa, yb, v01);
            match case {
                1 | 14 => segs.push((bottom(), left())),
                2 | 13 => segs.push((bottom(), right())),
                4 | 11 => segs.push((right(), top())),
                8 | 7 => segs.push((top(), left())),
                3 | 12 => segs.push((left(), right())),
                6 | 9 => segs.push((bottom(), top())),
                5 => {
                    // Ambiguous saddle — resolve by the center sample.
                    if f(xa + cell * 0.5, ya + cell * 0.5) > 0.0 {
                        segs.push((bottom(), right()));
                        segs.push((top(), left()));
                    } else {
                        segs.push((bottom(), left()));
                        segs.push((right(), top()));
                    }
                }
                10 => {
                    if f(xa + cell * 0.5, ya + cell * 0.5) > 0.0 {
                        segs.push((bottom(), left()));
                        segs.push((right(), top()));
                    } else {
                        segs.push((bottom(), right()));
                        segs.push((top(), left()));
                    }
                }
                _ => unreachable!(),
            }
        }
    }

    let polylines = chain_segments(segs, cell * 0.25);
    clip_polylines(polylines, region)
}

/// Chain a segment soup into polylines by joining endpoints that coincide within
/// `tol` (marching squares emits exact shared edge points, so this is generous).
pub(crate) fn chain_segments(segs: Vec<((f64, f64), (f64, f64))>, tol: f64) -> Vec<Vec<(f64, f64)>> {
    use std::collections::HashMap;
    let q = 1.0 / tol.max(1.0e-6);
    let key = |p: (f64, f64)| ((p.0 * q).round() as i64, (p.1 * q).round() as i64);

    // Endpoint -> (segment index, which end) for unused segments.
    let mut ends: HashMap<(i64, i64), Vec<(usize, bool)>> = HashMap::new();
    for (i, &(a, b)) in segs.iter().enumerate() {
        ends.entry(key(a)).or_default().push((i, false));
        ends.entry(key(b)).or_default().push((i, true));
    }
    let mut used = vec![false; segs.len()];
    let mut out: Vec<Vec<(f64, f64)>> = Vec::new();

    for start in 0..segs.len() {
        if used[start] {
            continue;
        }
        used[start] = true;
        let (a, b) = segs[start];
        let mut line: std::collections::VecDeque<(f64, f64)> = [a, b].into_iter().collect();
        // Extend forward from the back, then backward from the front.
        for forward in [true, false] {
            loop {
                let tip = if forward { *line.back().unwrap() } else { *line.front().unwrap() };
                let Some(cands) = ends.get(&key(tip)) else { break };
                let Some(&(i, end)) = cands.iter().find(|&&(i, _)| !used[i]) else { break };
                used[i] = true;
                let (sa, sb) = segs[i];
                let next = if end { sa } else { sb }; // the *other* end of segment i
                if forward {
                    line.push_back(next);
                } else {
                    line.push_front(next);
                }
            }
        }
        if line.len() >= 2 {
            out.push(line.into_iter().collect());
        }
    }
    out
}

/// Clip polylines to a region (even-odd), splitting where they cross the
/// boundary and keeping the inside parts. Crossing points are computed exactly
/// against the region edges so infill reaches the boundary.
fn clip_polylines(lines: Vec<Vec<(f64, f64)>>, region: &Polygons) -> Vec<Vec<Point>> {
    // Collect the region's edges once.
    let mut edges: Vec<((f64, f64), (f64, f64))> = Vec::new();
    for c in &region.contours {
        let m = c.points.len();
        if m < 3 {
            continue;
        }
        for j in 0..m {
            let a = c.points[j];
            let b = c.points[(j + 1) % m];
            edges.push(((a.x_mm(), a.y_mm()), (b.x_mm(), b.y_mm())));
        }
    }
    let mut out: Vec<Vec<Point>> = Vec::new();
    let mut cur: Vec<Point> = Vec::new();
    let mut ts: Vec<f64> = Vec::new();
    for line in lines {
        for w in line.windows(2) {
            let (a, b) = (w[0], w[1]);
            // Boundary crossings on this segment, ordered along it; the segment
            // splits into pieces that are each wholly inside or outside.
            ts.clear();
            ts.push(0.0);
            for &(p, q) in &edges {
                if let Some(t) = seg_intersect_t(a, b, p, q) {
                    ts.push(t);
                }
            }
            ts.push(1.0);
            ts.sort_by(|x, y| x.partial_cmp(y).unwrap());
            for k in 0..ts.len() - 1 {
                let (t0, t1) = (ts[k], ts[k + 1]);
                if t1 - t0 < 1.0e-9 {
                    continue;
                }
                let p0 = lerp(a, b, t0);
                let p1 = lerp(a, b, t1);
                if in_region(region, lerp(a, b, (t0 + t1) * 0.5)) {
                    if cur.is_empty() {
                        cur.push(Point::from_mm(p0.0, p0.1));
                    }
                    cur.push(Point::from_mm(p1.0, p1.1));
                } else {
                    flush(&mut out, &mut cur);
                }
            }
        }
        flush(&mut out, &mut cur);
    }
    out.retain(|l| l.len() >= 2 && polyline_len_mm(l) > 0.3);
    out
}

fn flush(out: &mut Vec<Vec<Point>>, cur: &mut Vec<Point>) {
    if cur.len() >= 2 {
        out.push(std::mem::take(cur));
    } else {
        cur.clear();
    }
}

fn lerp(a: (f64, f64), b: (f64, f64), t: f64) -> (f64, f64) {
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

fn polyline_len_mm(l: &[Point]) -> f64 {
    l.windows(2)
        .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
        .sum()
}

/// Parametric intersection of segment a-b with segment p-q: returns t along a-b
/// in (0, 1) for a proper crossing.
fn seg_intersect_t(a: (f64, f64), b: (f64, f64), p: (f64, f64), q: (f64, f64)) -> Option<f64> {
    let r = (b.0 - a.0, b.1 - a.1);
    let s = (q.0 - p.0, q.1 - p.1);
    let denom = r.0 * s.1 - r.1 * s.0;
    if denom.abs() < 1.0e-12 {
        return None;
    }
    let ap = (p.0 - a.0, p.1 - a.1);
    let t = (ap.0 * s.1 - ap.1 * s.0) / denom;
    let u = (ap.0 * r.1 - ap.1 * r.0) / denom;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some(t)
}

/// Even-odd containment of a point in a polygon set.
fn in_region(polys: &Polygons, p: (f64, f64)) -> bool {
    let pt = Point::from_mm(p.0, p.1);
    let mut inside = false;
    for c in &polys.contours {
        if c.contains(pt) {
            inside = !inside;
        }
    }
    inside
}

/// Principal direction (degrees) of a region's contour points — the long axis of
/// an elongated sliver, used to orient gap-fill strokes.
pub fn principal_angle_deg(polys: &Polygons) -> f64 {
    let (mut n, mut mx, mut my) = (0.0f64, 0.0f64, 0.0f64);
    for c in &polys.contours {
        for p in &c.points {
            n += 1.0;
            mx += p.x_mm();
            my += p.y_mm();
        }
    }
    if n < 2.0 {
        return 0.0;
    }
    mx /= n;
    my /= n;
    let (mut sxx, mut sxy, mut syy) = (0.0f64, 0.0f64, 0.0f64);
    for c in &polys.contours {
        for p in &c.points {
            let (dx, dy) = (p.x_mm() - mx, p.y_mm() - my);
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }
    }
    // Eigenvector of the larger eigenvalue of [[sxx, sxy], [sxy, syy]].
    (0.5 * (2.0 * sxy).atan2(sxx - syy)).to_degrees()
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
    fn gyroid_density_and_containment() {
        let region = rect(0.0, 0.0, 40.0, 40.0); // 1600 mm²
        let lw = 0.45;
        let density = 0.15;
        let spacing = lw / density; // 3 mm
        let lines = gyroid_lines(&region, spacing, 7.3);
        assert!(!lines.is_empty(), "gyroid produced nothing");
        let mut len = 0.0;
        for l in &lines {
            for w in l.windows(2) {
                len += (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm());
            }
            for p in l {
                let (x, y) = (p.x_mm(), p.y_mm());
                assert!(
                    (-0.01..=40.01).contains(&x) && (-0.01..=40.01).contains(&y),
                    "gyroid point ({x:.2},{y:.2}) escaped the region"
                );
            }
        }
        // Volume sanity: extruded length × lw should land near area × density.
        // The gyroid section is wavy (longer than straight lines), so allow a
        // generous band — this guards against period math being off by 2×.
        let eff = len * lw / 1600.0;
        assert!(
            (density * 0.7..density * 1.8).contains(&eff),
            "gyroid effective density {eff:.3} vs requested {density}"
        );
    }

    #[test]
    fn gyroid_varies_with_z() {
        let region = rect(0.0, 0.0, 20.0, 20.0);
        let a = gyroid_lines(&region, 3.0, 1.0);
        let b = gyroid_lines(&region, 3.0, 2.5);
        // Different z phases → different curves (compare total endpoints sum).
        let sig = |ls: &Vec<Vec<Point>>| {
            ls.iter().flat_map(|l| l.iter()).map(|p| p.x_mm() + p.y_mm()).sum::<f64>()
        };
        assert!((sig(&a) - sig(&b)).abs() > 1.0, "gyroid should drift with z");
    }

    #[test]
    fn boustrophedon_alternates() {
        let region = rect(0.0, 0.0, 10.0, 10.0);
        let lines = infill_lines(&region, 0.0, 1.0, true);
        assert!(lines.len() >= 8);
        // Consecutive rows start at opposite x ends.
        let x0 = lines[0][0].x_mm();
        let x1 = lines[1][0].x_mm();
        assert!((x0 - x1).abs() > 5.0, "rows should alternate direction");
    }

    #[test]
    fn principal_angle_finds_long_axis() {
        let strip = rect(0.0, 0.0, 20.0, 0.4);
        let a = principal_angle_deg(&strip).rem_euclid(180.0);
        assert!(a < 5.0 || a > 175.0, "long axis should be ~0°, got {a}");
    }
}
