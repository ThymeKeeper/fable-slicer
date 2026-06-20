//! Offset-peel variable-width walls — the fork/merge engine.
//!
//! A robust, Voronoi-free replacement for skeletal trapezoidation. Instead of
//! building a medial-axis graph from a fragile Voronoi diagram, we recover the
//! same fork/merge bead structure from the *topology of inward polygon offsets*:
//!
//!   - Each pass lays one closed ring at half the local pitch inside the region.
//!   - The pitch is re-derived per region from its OWN local thickness, so a
//!     thin flank and a fat core each distribute their beads evenly — no global
//!     pitch leaving a sliver where the geometry is thinner than average.
//!   - Each ring vertex is widened toward the gap it is responsible for, so a
//!     bead fattens through a divergence instead of leaving a slice uncovered.
//!   - After peeling a full bead inward, the remainder is split into connected
//!     components and each is recursed on independently. A remainder that breaks
//!     into two pieces is a bead **fork** (one bead becomes two).
//!   - When a region is only one bead thick, we stop ringing and run a single
//!     centerline down its medial axis — the adjacent beads have **merged**.
//!
//! Every ring is an exact polygon offset, so the curves are clean and the splits
//! and merges fall out of the offset for free. Nothing here constructs a Voronoi
//! diagram, so it cannot panic the way skeletal.rs does, and — being a different
//! algorithm family (contour peeling, not skeletal trapezoidation) — it is an
//! independent implementation rather than a port.

use crate::wall::Bead;
use geo2d::{offset, Contour, Point, Polygons};

/// Safety cap on recursion depth (a runaway would otherwise be bounded only by
/// the bead budget); real parts terminate far shallower.
const MAX_DEPTH: usize = 256;

/// Below this bead count a region is thin enough for divergences to matter, so
/// its rings get per-vertex variable width. Thicker cores distribute evenly and
/// stay constant-width — which also keeps the O(edges) width probe off the big
/// rings where it would be slow.
const VARIABLE_WIDTH_MAX_N: usize = 8;

/// Generate the inner adaptive beads for `region` (already inset past the fixed
/// outer wall) with up to `max_inner` ring levels. Outermost beads first.
pub(crate) fn peel_beads(region: &Polygons, lw: f64, sp: f64, max_inner: usize) -> Vec<Bead> {
    let mut beads = Vec::new();
    for comp in components(region) {
        emit_region(&comp, lw, sp, max_inner.min(MAX_DEPTH), &mut beads);
    }
    if std::env::var("PEEL_DUMP").is_ok() {
        dump_coverage(region, &beads);
    }
    beads
}

/// Debug: write the largest region seen so far + its beads, for offline
/// coverage rendering (env PEEL_DUMP). Overwrites on each new largest region.
fn dump_coverage(region: &Polygons, beads: &[Bead]) {
    use std::sync::Mutex;
    static BEST: Mutex<f64> = Mutex::new(0.0);
    let a = region.net_area_mm2();
    let mut best = BEST.lock().unwrap();
    if a <= *best {
        return;
    }
    *best = a;
    write_dump(region, beads);
}

/// Write a region + its beads to /tmp/peeldump.txt for offline rendering.
fn write_dump(region: &Polygons, beads: &[Bead]) {
    let mut s = String::from("REGION\n");
    for c in &region.contours {
        s.push('C');
        for p in &c.points {
            s.push_str(&format!(" {:.3},{:.3}", p.x_mm(), p.y_mm()));
        }
        s.push('\n');
    }
    s.push_str("BEADS\n");
    for b in beads {
        s.push_str(&format!("B {} ", if b.closed { 1 } else { 0 }));
        for (i, p) in b.points.iter().enumerate() {
            let w = b.widths.get(i).copied().unwrap_or(0.4);
            s.push_str(&format!(" {:.3},{:.3},{:.3}", p.x_mm(), p.y_mm(), w));
        }
        s.push('\n');
    }
    let _ = std::fs::write("/tmp/peeldump.txt", s);
}

/// Largest inward inset (mm) that still leaves material — the region's local
/// maximum half-thickness — by bisection on the offset area. `hi` is an upper
/// bound on the inradius (an inset that already empties the region).
fn max_inradius(region: &Polygons, mut hi: f64) -> f64 {
    if region.net_area_mm2() <= 0.0 {
        return 0.0;
    }
    let mut lo = 0.0;
    for _ in 0..16 {
        let mid = 0.5 * (lo + hi);
        if offset(region, -mid).net_area_mm2() > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Distance (mm) from a point to the nearest edge of `polys` (∞ if empty).
fn dist_to_edges(px: f64, py: f64, polys: &Polygons) -> f64 {
    let mut best = f64::INFINITY;
    for c in &polys.contours {
        let n = c.points.len();
        if n < 2 {
            continue;
        }
        for i in 0..n {
            let a = c.points[i];
            let b = c.points[(i + 1) % n];
            let (ax, ay) = (a.x_mm(), a.y_mm());
            let (dx, dy) = (b.x_mm() - ax, b.y_mm() - ay);
            let len2 = dx * dx + dy * dy;
            let t = if len2 > 0.0 {
                (((px - ax) * dx + (py - ay) * dy) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let d = (px - (ax + t * dx)).hypot(py - (ay + t * dy));
            if d < best {
                best = d;
            }
        }
    }
    best
}

/// One ring level: lay the (widened) ring, peel a full bead, recurse per piece.
fn emit_region(region: &Polygons, lw: f64, sp: f64, budget: usize, beads: &mut Vec<Bead>) {
    if budget == 0 {
        return;
    }
    let bb = match region.bounds() {
        Some(b) => b,
        None => return,
    };
    let span = (bb.max.x_mm() - bb.min.x_mm()).min(bb.max.y_mm() - bb.min.y_mm());
    let t = max_inradius(region, 0.5 * span + sp);
    if t < sp * 0.35 {
        return; // nothing printable
    }

    // Beads across the full local thickness; one ring level covers two of them
    // (a closed loop is the i-th bead from each side at once).
    let n = ((2.0 * t / sp).round() as usize).max(1);
    if n == 1 {
        // Terminal: one-bead-thick → a single medial centerline (the merge).
        beads.extend(terminal_beads(region, lw, sp));
        return;
    }

    let pitch = (2.0 * t / n as f64).clamp(sp * 0.5, sp * 1.7);
    let inner = offset(region, -pitch);
    let ring = offset(region, -0.5 * pitch);

    // Ring vertices widen toward the gap to the next ring inward so a bead
    // extends to fill a divergence — but never thin below the nominal width,
    // since a thinned ring opens a gap against its neighbour (the fixed outer
    // wall or the previous ring). Deep even cores skip the probe entirely.
    let nominal = (pitch + (lw - sp)).clamp(lw * 0.5, lw * 1.75);
    let variable = n <= VARIABLE_WIDTH_MAX_N && inner.net_area_mm2() > 0.0;
    for c in &ring.contours {
        if c.points.len() < 3 {
            continue;
        }
        let widths = if variable {
            c.points
                .iter()
                .map(|p| {
                    let g = dist_to_edges(p.x_mm(), p.y_mm(), &inner);
                    let g = if g.is_finite() { g } else { 0.5 * pitch };
                    (2.0 * g + (lw - sp)).clamp(nominal, lw * 1.8)
                })
                .collect()
        } else {
            vec![nominal; c.points.len()]
        };
        beads.push(Bead { points: c.points.clone(), widths, closed: true });
    }

    // Peel one full bead and recurse on each connected piece — a piece count
    // greater than one here is a fork.
    for comp in components(&inner) {
        emit_region(&comp, lw, sp, budget - 1, beads);
    }
}

/// Group a flat contour list into connected components, each an outer ring plus
/// the holes it contains, so the recursion treats genuinely separate pieces
/// (the fork branches) independently and never fills a hole.
fn components(polys: &Polygons) -> Vec<Polygons> {
    let mut comps: Vec<Polygons> = Vec::new();
    let mut holes: Vec<Contour> = Vec::new();
    for c in &polys.contours {
        if c.points.len() < 3 {
            continue;
        }
        if c.signed_area_mm2() > 0.0 {
            comps.push(Polygons { contours: vec![c.clone()] });
        } else {
            holes.push(c.clone());
        }
    }
    // Assign each hole to the smallest-area outer that contains it.
    for h in holes {
        let p = h.points[0];
        let mut best: Option<usize> = None;
        let mut best_area = f64::INFINITY;
        for (i, comp) in comps.iter().enumerate() {
            let outer = &comp.contours[0];
            if outer.area_mm2() < best_area && outer.contains(p) {
                best_area = outer.area_mm2();
                best = Some(i);
            }
        }
        if let Some(i) = best {
            comps[i].contours.push(h);
        }
    }
    comps
}

/// Centerline beads for a one-bead-thick terminal region. An elongated single
/// piece (a wedge, a flank sliver, a strip) gets a clean analytic centerline —
/// one continuous tapering bead down its spine; anything with holes or odd
/// topology falls back to the grid skeleton, smoothed.
fn terminal_beads(region: &Polygons, lw: f64, sp: f64) -> Vec<Bead> {
    let outers: Vec<&Contour> = region.contours.iter().filter(|c| c.signed_area_mm2() > 0.0 && c.points.len() >= 3).collect();
    let has_hole = region.contours.iter().any(|c| c.signed_area_mm2() < 0.0 && c.points.len() >= 3);
    let holes: Vec<&Contour> = region.contours.iter().filter(|c| c.signed_area_mm2() < 0.0 && c.points.len() >= 3).collect();
    if outers.len() == 1 && !has_hole {
        // Open strip (wedge, sliver): centerline between its two ends.
        if let Some(b) = analytic_centerline(outers[0], lw, sp) {
            return vec![b];
        }
    } else if outers.len() == 1 && holes.len() == 1 {
        // Annular band (a flank wrapping the hull): centerline is a closed loop
        // down the middle of the band, paired between outer and hole boundaries.
        if let Some(b) = annular_centerline(outers[0], holes[0], lw, sp) {
            return vec![b];
        }
    }
    // Fallback: grid skeleton, smoothed + geometric widths.
    crate::wall::region_terminal_beads(region, lw, sp)
        .into_iter()
        .filter(|b| b.points.len() >= 2)
        .map(|b| {
            let sm = chaikin(&chaikin(&b.points, b.closed), b.closed);
            let pts = resample(&sm, b.closed, sp * 0.4);
            let widths = pts
                .iter()
                .map(|p| (2.0 * dist_to_edges(p.x_mm(), p.y_mm(), region)).clamp(lw * 0.7, lw * 1.8))
                .collect();
            Bead { points: pts, widths, closed: b.closed }
        })
        .collect()
}

/// The centerline of an elongated thin polygon: its two farthest boundary points
/// are the ends; the boundary splits into two sides between them; walking both
/// sides in lock-step by arc length and taking midpoints traces the spine, with
/// each width = the gap between the sides (so a wedge tapers to its point). No
/// skeleton, no Voronoi — just the boundary.
fn analytic_centerline(contour: &Contour, lw: f64, sp: f64) -> Option<Bead> {
    let loop_pts = resample(&contour.points, true, sp * 0.25);
    let m = loop_pts.len();
    if m < 6 {
        return None;
    }
    // Farthest pair of boundary points = the two ends of the strip.
    let (mut ia, mut ib, mut best) = (0usize, 0usize, -1.0);
    for i in 0..m {
        for j in (i + 1)..m {
            let d = (loop_pts[i].x_mm() - loop_pts[j].x_mm()).hypot(loop_pts[i].y_mm() - loop_pts[j].y_mm());
            if d > best {
                best = d;
                ia = i;
                ib = j;
            }
        }
    }
    if best < sp {
        return None;
    }
    // The two boundary chains from end A to end B (one each way around the loop).
    let mut side1 = Vec::new();
    let mut k = ia;
    loop {
        side1.push(loop_pts[k]);
        if k == ib {
            break;
        }
        k = (k + 1) % m;
    }
    let mut side2 = Vec::new();
    let mut k = ia;
    loop {
        side2.push(loop_pts[k]);
        if k == ib {
            break;
        }
        k = (k + m - 1) % m;
    }
    if side1.len() < 2 || side2.len() < 2 {
        return None;
    }
    // Walk both sides by normalized arc length; midpoints are the spine.
    let steps = ((best / (sp * 0.4)).ceil() as usize).max(2);
    let mut points = Vec::with_capacity(steps + 1);
    let mut widths = Vec::with_capacity(steps + 1);
    for s in 0..=steps {
        let t = s as f64 / steps as f64;
        let a = arc_point(&side1, t);
        let b = arc_point(&side2, t);
        points.push(Point::from_mm(0.5 * (a.x_mm() + b.x_mm()), 0.5 * (a.y_mm() + b.y_mm())));
        let w = (a.x_mm() - b.x_mm()).hypot(a.y_mm() - b.y_mm());
        widths.push(w.clamp(lw * 0.7, lw * 1.8));
    }
    let points = chaikin(&points, false);
    // Re-derive widths after smoothing (the midpoint count changed).
    let widths = resample_widths(&widths, points.len());
    Some(Bead { points, widths, closed: false })
}

/// The centerline of a thin annular band: pair each outer-boundary point with
/// the nearest point on the hole boundary and take the midpoint — a closed loop
/// down the middle of the band, with each width = the band thickness there. This
/// is what fills a flank (a thin loop wrapping the hull) that has merged to a
/// single bead.
fn annular_centerline(outer: &Contour, hole: &Contour, lw: f64, sp: f64) -> Option<Bead> {
    let op = resample(&outer.points, true, sp * 0.3);
    let hp = resample(&hole.points, true, sp * 0.3);
    if op.len() < 4 || hp.len() < 4 {
        return None;
    }
    let mut points = Vec::with_capacity(op.len());
    let mut widths = Vec::with_capacity(op.len());
    for p in &op {
        // Nearest hole point (small regions → brute force is fine).
        let mut best = f64::INFINITY;
        let mut q = hp[0];
        for h in &hp {
            let d = (p.x_mm() - h.x_mm()).hypot(p.y_mm() - h.y_mm());
            if d < best {
                best = d;
                q = *h;
            }
        }
        points.push(Point::from_mm(0.5 * (p.x_mm() + q.x_mm()), 0.5 * (p.y_mm() + q.y_mm())));
        widths.push(best.clamp(lw * 0.7, lw * 1.8));
    }
    let points = chaikin(&points, true);
    let widths = resample_widths(&widths, points.len());
    Some(Bead { points, widths, closed: true })
}

/// Point at normalized arc-length `t` along an open polyline.
fn arc_point(pts: &[Point], t: f64) -> Point {
    let n = pts.len();
    if n == 1 {
        return pts[0];
    }
    let mut cum = vec![0.0; n];
    for i in 1..n {
        cum[i] = cum[i - 1] + (pts[i].x_mm() - pts[i - 1].x_mm()).hypot(pts[i].y_mm() - pts[i - 1].y_mm());
    }
    let total = cum[n - 1];
    if total <= 0.0 {
        return pts[0];
    }
    let s = t * total;
    for i in 1..n {
        if cum[i] >= s {
            let seg = cum[i] - cum[i - 1];
            let f = if seg > 0.0 { (s - cum[i - 1]) / seg } else { 0.0 };
            return Point::from_mm(
                pts[i - 1].x_mm() + f * (pts[i].x_mm() - pts[i - 1].x_mm()),
                pts[i - 1].y_mm() + f * (pts[i].y_mm() - pts[i - 1].y_mm()),
            );
        }
    }
    pts[n - 1]
}

/// Stretch/shrink a width array to `len` by nearest sampling.
fn resample_widths(w: &[f64], len: usize) -> Vec<f64> {
    if w.is_empty() {
        return vec![0.0; len];
    }
    (0..len).map(|i| w[(i * w.len() / len.max(1)).min(w.len() - 1)]).collect()
}

/// Resample a polyline to roughly uniform `step` spacing.
fn resample(pts: &[Point], closed: bool, step: f64) -> Vec<Point> {
    let n = pts.len();
    if n < 2 {
        return pts.to_vec();
    }
    let segs = if closed { n } else { n - 1 };
    let mut out = Vec::new();
    for i in 0..segs {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let (ax, ay) = (a.x_mm(), a.y_mm());
        let (dx, dy) = (b.x_mm() - ax, b.y_mm() - ay);
        let k = ((dx.hypot(dy)) / step).ceil().max(1.0) as usize;
        for j in 0..k {
            let t = j as f64 / k as f64;
            out.push(Point::from_mm(ax + t * dx, ay + t * dy));
        }
    }
    if !closed {
        out.push(pts[n - 1]);
    }
    out
}

/// One Chaikin corner-cutting pass (closed loops stay closed).
fn chaikin(pts: &[Point], closed: bool) -> Vec<Point> {
    let n = pts.len();
    if n < 3 {
        return pts.to_vec();
    }
    let mut out = Vec::with_capacity(2 * n);
    if !closed {
        out.push(pts[0]);
    }
    let segs = if closed { n } else { n - 1 };
    for i in 0..segs {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        out.push(Point::from_mm(0.75 * a.x_mm() + 0.25 * b.x_mm(), 0.75 * a.y_mm() + 0.25 * b.y_mm()));
        out.push(Point::from_mm(0.25 * a.x_mm() + 0.75 * b.x_mm(), 0.25 * a.y_mm() + 0.75 * b.y_mm()));
    }
    if !closed {
        out.push(pts[n - 1]);
    }
    out
}
