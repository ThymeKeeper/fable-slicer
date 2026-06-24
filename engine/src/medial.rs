//! Segment-Voronoi **medial axis** for Classic gap fill.
//!
//! Clean-room implementation of the medial-axis-transform (MAT) approach to
//! tracing a thin, irregular sliver with one variable-width bead. The medial
//! axis of a polygon is the interior subset of the Voronoi diagram of its
//! *boundary segments*; at every point on it the local feature width is twice
//! the clearance to the boundary (the inscribed-circle diameter). This is the
//! method PrusaSlicer's Classic generator uses for gap fill — implemented here
//! from the algorithm, not its code.
//!
//! Why a true MAT and not a boundary-pairing centerline: the Voronoi skeleton
//! branches and curves correctly by construction, so it never folds a sliver
//! back on itself (the "W"/hairpin that the old `analytic_centerline` produced),
//! and a polygon-segment Voronoi is smooth — no raster stair-steps, so no
//! spline is needed afterward. The only cleanup is twig culling + endpoint
//! extension, mirroring Prusa's three post steps.
//!
//! Internally everything is integer **micrometers**: `boost::polygon::voronoi`
//! (the `boostvoronoi` port) wants integer input and µm keeps its robust-float
//! predicate intermediates small. geo2d nanometers are scaled at the boundary.

use boostvoronoi::prelude::*; // `Point` / `Line` / `Builder` / `SourceCategory`
use geo2d::Polygons;

/// nm per µm — geo2d works in nm, the Voronoi runs in µm.
const NM: f64 = 1000.0;
/// Sentinel for "no vertex / infinite edge end".
const NIL: usize = usize::MAX;

/// A medial-axis polyline carrying a width at each point.
///
/// `points`/`widths` are parallel (one width per point); `endpoints` flags
/// whether each end is *free* (a dangling tip, degree-1) versus a junction
/// where other polylines meet. Coordinates are geo2d nm; widths are mm.
#[derive(Clone, Debug)]
pub struct ThickPolyline {
    pub points: Vec<geo2d::Point>,
    pub widths: Vec<f64>,
    pub endpoints: (bool, bool),
}

impl ThickPolyline {
    /// Total length in mm.
    pub fn length_mm(&self) -> f64 {
        self.points
            .windows(2)
            .map(|w| {
                ((w[1].x - w[0].x) as f64).hypot((w[1].y - w[0].y) as f64) / geo2d::UNITS_PER_MM
            })
            .sum()
    }
}

/// Trace the medial axis of `region`, keeping only the part whose local width
/// falls in `[min_w_mm, max_w_mm]`. Returns one [`ThickPolyline`] per maximal
/// non-branching run of the skeleton.
///
/// Robust to the degenerate site configurations `boostvoronoi` panics on: the
/// construction is caught and retried on a deterministically jittered copy
/// before conceding an empty result (a gap that simply doesn't get filled).
pub fn medial_axis(region: &Polygons, min_w_mm: f64, max_w_mm: f64) -> Vec<ThickPolyline> {
    quiet_voronoi_panics();
    for attempt in 0..4u64 {
        let r = if attempt == 0 { region.clone() } else { jitter(region, attempt) };
        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            medial_axis_inner(&r, min_w_mm, max_w_mm)
        }));
        match res {
            Ok(Some(v)) => return v,
            // A clean `None` (empty/degenerate region) won't be fixed by more
            // jitter; only keep retrying through panics.
            Ok(None) if attempt >= 1 => return Vec::new(),
            _ => {}
        }
    }
    Vec::new()
}

fn medial_axis_inner(region: &Polygons, min_w_mm: f64, max_w_mm: f64) -> Option<Vec<ThickPolyline>> {
    let min_um = min_w_mm * NM;
    let max_um = max_w_mm * NM;

    let (polys, segs) = to_um_segments(region);
    if segs.is_empty() {
        return Some(Vec::new());
    }
    let voro = build_voronoi(&segs)?;

    // --- select medial edges + per-vertex widths ---------------------------
    // Each undirected Voronoi edge appears as a half-edge pair (e, twin). Keep
    // the primary, finite ones whose midpoint is interior and whose width is in
    // band. Width at a vertex = 2 × clearance to either adjacent cell's site
    // (equal by the Voronoi property), so we read it off this edge's own cell.
    let nv = voro.verts.len();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nv];
    let mut kept: Vec<Kept> = Vec::new();
    for e in 0..voro.edges.len() {
        let twin = voro.edges[e].twin;
        if e > twin {
            continue; // process each undirected edge once
        }
        if voro.edges[e].secondary {
            continue; // secondary edges run to source points — not MAT
        }
        let (va, vb) = (voro.v0(e), voro.v1(e));
        if va == NIL || vb == NIL {
            continue; // infinite edge
        }
        let pa = voro.point(va);
        let pb = voro.point(vb);
        let mid = P { x: (pa.x + pb.x) / 2, y: (pa.y + pb.y) / 2 };
        if !inside(&polys, mid) {
            continue; // exterior branch of the diagram
        }
        let cell = &voro.cells[voro.edges[e].cell];
        let wa = 2.0 * clearance(pa, cell, &segs);
        let wb = 2.0 * clearance(pb, cell, &segs);
        if wa < 1.0 && wb < 1.0 {
            continue; // both ~zero: a sliver pinned to the boundary
        }
        // Prusa's band test: at least one end wide enough to extrude, and at
        // least one end narrow enough that it isn't really its own perimeter.
        if !((wa >= min_um || wb >= min_um) && (wa <= max_um || wb <= max_um)) {
            continue;
        }
        let id = kept.len();
        adj[va].push(id);
        adj[vb].push(id);
        kept.push(Kept { va, vb, wa, wb });
    }

    // --- chain kept edges into maximal non-branching polylines -------------
    let mut used = vec![false; kept.len()];
    let mut out: Vec<ThickPolyline> = Vec::new();
    // Start chains at free ends (degree 1) first so open slivers trace cleanly;
    // any leftover edges (closed loops) are walked afterward.
    let starts = (0..nv)
        .filter(|&v| adj[v].len() == 1)
        .chain(0..nv)
        .collect::<Vec<_>>();
    for start in starts {
        for k in 0..adj[start].len() {
            let e0 = adj[start][k];
            if used[e0] {
                continue;
            }
            let tp = walk_chain(start, e0, &kept, &adj, &mut used, &voro);
            if tp.points.len() >= 2 {
                out.push(tp);
            }
        }
    }

    // --- post: twig cull + endpoint extension ------------------------------
    // Drop a polyline that dangles (a free end) and is shorter than two max
    // widths — these are spurious ribs the Voronoi grows into corners.
    out.retain(|tp| !((tp.endpoints.0 || tp.endpoints.1) && tp.length_mm() < 2.0 * max_w_mm));
    // Culling ribs drops their junctions to degree 2; rejoin the through-runs so
    // a curved sliver is one bead, not a string of short fragments.
    let mut out = concat_chains(out);
    for tp in &mut out {
        extend_to_boundary(tp, &polys);
    }
    Some(out)
}

/// Walk a maximal chain from `start` along edge `e0`, threading through degree-2
/// vertices and stopping at free ends (degree 1) or junctions (degree ≥ 3).
fn walk_chain(
    start: usize,
    e0: usize,
    kept: &[Kept],
    adj: &[Vec<usize>],
    used: &mut [bool],
    voro: &Voro,
) -> ThickPolyline {
    let mut points = Vec::new();
    let mut widths = Vec::new();
    let mut cur = start;
    let mut e = e0;
    // first point
    let p = voro.point(cur);
    points.push(geo2d::Point::new(p.x * NM as i64, p.y * NM as i64));
    widths.push(width_at(kept, e, cur) / NM);
    loop {
        used[e] = true;
        let k = &kept[e];
        let next = if k.va == cur { k.vb } else { k.va };
        let p = voro.point(next);
        points.push(geo2d::Point::new(p.x * NM as i64, p.y * NM as i64));
        widths.push(width_at(kept, e, next) / NM);
        // continue only through a clean degree-2 pass-through
        if adj[next].len() != 2 || next == start {
            cur = next;
            break;
        }
        let ne = adj[next].iter().copied().find(|&x| x != e && !used[x]);
        match ne {
            Some(nx) => {
                e = nx;
                cur = next;
            }
            None => {
                cur = next;
                break;
            }
        }
    }
    let endpoints = (adj[start].len() == 1, adj[cur].len() == 1);
    ThickPolyline { points, widths, endpoints }
}

#[inline]
fn width_at(kept: &[Kept], e: usize, v: usize) -> f64 {
    let k = &kept[e];
    if k.va == v {
        k.wa
    } else {
        k.wb
    }
}

fn reversed(mut tp: ThickPolyline) -> ThickPolyline {
    tp.points.reverse();
    tp.widths.reverse();
    tp.endpoints = (tp.endpoints.1, tp.endpoints.0);
    tp
}

/// Merge polylines that meet at a shared endpoint where exactly two ends come
/// together (a degree-2 join — a real through-run that chaining split because a
/// rib made the vertex look like a junction, until the rib was culled). Real
/// junctions (≥3 ends) and loops (both ends of one polyline) are left intact.
fn concat_chains(mut polys: Vec<ThickPolyline>) -> Vec<ThickPolyline> {
    use std::collections::HashMap;
    loop {
        let mut ends: HashMap<(i64, i64), Vec<(usize, bool)>> = HashMap::new();
        for (i, tp) in polys.iter().enumerate() {
            if tp.points.len() < 2 {
                continue;
            }
            let f = tp.points[0];
            let l = *tp.points.last().unwrap();
            ends.entry((f.x, f.y)).or_default().push((i, false));
            ends.entry((l.x, l.y)).or_default().push((i, true));
        }
        let join = ends
            .values()
            .find(|v| v.len() == 2 && v[0].0 != v[1].0)
            .map(|v| (v[0], v[1]));
        let ((ia, a_is_end), (ib, b_is_end)) = match join {
            Some(j) => j,
            None => return polys,
        };
        // Orient so `a` ends at the shared point and `b` starts at it.
        let mut a = polys[ia].clone();
        let mut b = polys[ib].clone();
        if !a_is_end {
            a = reversed(a);
        }
        if b_is_end {
            b = reversed(b);
        }
        let mut points = a.points;
        let mut widths = a.widths;
        points.extend_from_slice(&b.points[1..]);
        widths.extend_from_slice(&b.widths[1..]);
        let merged = ThickPolyline { points, widths, endpoints: (a.endpoints.0, b.endpoints.1) };
        let mut next = Vec::with_capacity(polys.len() - 1);
        for (i, tp) in polys.into_iter().enumerate() {
            if i != ia && i != ib {
                next.push(tp);
            }
        }
        next.push(merged);
        polys = next;
    }
}

/// Extend each *free* end of `tp` straight out to the polygon boundary, so the
/// bead reaches and bonds to the surrounding perimeter instead of stopping a
/// bead-width short. The added tip tapers toward zero width.
fn extend_to_boundary(tp: &mut ThickPolyline, polys: &[Vec<P>]) {
    if tp.points.len() < 2 {
        return;
    }
    if tp.endpoints.0 {
        let a = um(tp.points[1]);
        let b = um(tp.points[0]);
        if let Some(hit) = ray_to_boundary(b, dir(a, b), polys) {
            tp.points.insert(0, geo2d::Point::new(hit.x * NM as i64, hit.y * NM as i64));
            tp.widths.insert(0, (tp.widths[0] * 0.5).max(0.0));
        }
    }
    if tp.endpoints.1 {
        let n = tp.points.len();
        let a = um(tp.points[n - 2]);
        let b = um(tp.points[n - 1]);
        if let Some(hit) = ray_to_boundary(b, dir(a, b), polys) {
            tp.points.push(geo2d::Point::new(hit.x * NM as i64, hit.y * NM as i64));
            tp.widths.push((tp.widths[n - 1] * 0.5).max(0.0));
        }
    }
}

// ===========================================================================
// boostvoronoi wrapper (µm half-edge arena)
// ===========================================================================

struct VCell {
    source: usize,
    cat: Cat,
}
struct VEdge {
    cell: usize,
    v0: usize, // NIL = infinite end
    twin: usize,
    secondary: bool,
}
struct Voro {
    cells: Vec<VCell>,
    edges: Vec<VEdge>,
    verts: Vec<(f64, f64)>,
}
impl Voro {
    fn v0(&self, e: usize) -> usize {
        self.edges[e].v0
    }
    fn v1(&self, e: usize) -> usize {
        self.edges[self.edges[e].twin].v0
    }
    fn point(&self, v: usize) -> P {
        P { x: self.verts[v].0.round() as i64, y: self.verts[v].1.round() as i64 }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Cat {
    SegStart,
    SegEnd,
    Seg,
    Point,
}

struct Kept {
    va: usize,
    vb: usize,
    wa: f64,
    wb: f64,
}

/// Run the Voronoi over the segment soup, flattening it into index arenas.
fn build_voronoi(segs: &[(P, P)]) -> Option<Voro> {
    let lines: Vec<Line<i64>> = segs
        .iter()
        .map(|&(a, b)| Line::new(Point { x: a.x, y: a.y }, Point { x: b.x, y: b.y }))
        .collect();
    let diagram = Builder::<i64>::default().with_segments(lines.iter()).ok()?.build().ok()?;

    let verts: Vec<(f64, f64)> = diagram.vertices().iter().map(|v| (v.x(), v.y())).collect();
    let mut cells = Vec::with_capacity(diagram.cells().len());
    for c in diagram.cells() {
        // SourceIndex exposes its value only through Debug formatting.
        let source: usize = format!("{:?}", c.source_index()).parse().ok()?;
        let cat = match c.source_category() {
            SourceCategory::SegmentStart => Cat::SegStart,
            SourceCategory::SegmentEnd => Cat::SegEnd,
            SourceCategory::Segment => Cat::Seg,
            SourceCategory::SinglePoint => Cat::Point,
        };
        cells.push(VCell { source, cat });
    }
    let mut edges = Vec::with_capacity(diagram.edges().len());
    for e in diagram.edges() {
        edges.push(VEdge {
            cell: e.cell().ok()?.usize(),
            v0: e.vertex0().map_or(NIL, |v| v.usize()),
            twin: e.twin().ok()?.usize(),
            secondary: e.is_secondary(),
        });
    }
    Some(Voro { cells, edges, verts })
}

/// Clearance (µm) from `p` to the site of `cell` — perpendicular distance to a
/// segment cell, or distance to the source endpoint of a point cell.
fn clearance(p: P, cell: &VCell, segs: &[(P, P)]) -> f64 {
    match cell.cat {
        Cat::Seg => {
            let (a, b) = segs[cell.source];
            dist_to_segment(p, a, b)
        }
        Cat::SegEnd => p.dist(segs[cell.source].1),
        _ => p.dist(segs[cell.source].0), // SegStart / SinglePoint
    }
}

// ===========================================================================
// µm geometry
// ===========================================================================

#[derive(Clone, Copy, PartialEq, Eq)]
struct P {
    x: i64,
    y: i64,
}
impl P {
    fn dist(self, o: P) -> f64 {
        ((self.x - o.x) as f64).hypot((self.y - o.y) as f64)
    }
    fn shorter_than(self, len: i64) -> bool {
        let (x, y) = (self.x, self.y);
        x.abs() <= len && y.abs() <= len && (x * x + y * y) as f64 <= (len * len) as f64
    }
}

#[inline]
fn um(p: geo2d::Point) -> P {
    P { x: (p.x as f64 / NM).round() as i64, y: (p.y as f64 / NM).round() as i64 }
}

fn dot(a: P, b: P) -> f64 {
    a.x as f64 * b.x as f64 + a.y as f64 * b.y as f64
}

fn dist_to_segment(p: P, a: P, b: P) -> f64 {
    let ab = P { x: b.x - a.x, y: b.y - a.y };
    let l2 = dot(ab, ab);
    if l2 < 1e-9 {
        return p.dist(a);
    }
    let t = (dot(P { x: p.x - a.x, y: p.y - a.y }, ab) / l2).clamp(0.0, 1.0);
    let proj = P { x: a.x + (ab.x as f64 * t).round() as i64, y: a.y + (ab.y as f64 * t).round() as i64 };
    p.dist(proj)
}

/// Unit-ish direction from `a` to `b` scaled to 1 mm, for ray extension.
fn dir(a: P, b: P) -> P {
    let (dx, dy) = ((b.x - a.x) as f64, (b.y - a.y) as f64);
    let s = dx.hypot(dy);
    if s < 1e-9 {
        return P { x: 0, y: 0 };
    }
    P { x: (dx / s * NM).round() as i64, y: (dy / s * NM).round() as i64 }
}

/// Even-odd point-in-polygons over the µm contours.
fn inside(polys: &[Vec<P>], p: P) -> bool {
    let mut c = false;
    for poly in polys {
        let n = poly.len();
        let mut j = n - 1;
        for i in 0..n {
            let pi = poly[i];
            let pj = poly[j];
            if (pi.y > p.y) != (pj.y > p.y) {
                let t = (p.y - pi.y) as f64 / (pj.y - pi.y) as f64;
                let xc = pi.x as f64 + t * (pj.x - pi.x) as f64;
                if (p.x as f64) < xc {
                    c = !c;
                }
            }
            j = i;
        }
    }
    c
}

/// March a ray from `from` along `step` (1 mm increments) until it leaves the
/// region; return the last interior sample as the boundary hit. Cheap and good
/// enough for a short tip extension (≤ one bead).
fn ray_to_boundary(from: P, step: P, polys: &[Vec<P>]) -> Option<P> {
    if step.x == 0 && step.y == 0 {
        return None;
    }
    let mut prev = from;
    // up to ~2 mm of extension in 0.1 mm steps
    for i in 1..=20 {
        let cand = P { x: from.x + step.x * i / 10, y: from.y + step.y * i / 10 };
        if !inside(polys, cand) {
            return if prev == from { None } else { Some(prev) };
        }
        prev = cand;
    }
    None
}

/// Scale a region to µm and emit closed-loop segments, applying the two fixes
/// the segment Voronoi needs: merge points closer than 5 µm, and strip exactly
/// collinear / duplicate vertices (which make boost emit a shared endpoint's
/// cell as two infinite secondary edges, breaking the cell walk).
fn to_um_segments(region: &Polygons) -> (Vec<Vec<P>>, Vec<(P, P)>) {
    let region = geo2d::simplify(region, 0.05); // ridge-scale smoothing (mm)
    let mut polys: Vec<Vec<P>> = Vec::new();
    for c in &region.contours {
        let mut q: Vec<P> = Vec::with_capacity(c.points.len());
        for p in &c.points {
            let pm = P { x: (p.x as f64 / NM).round() as i64, y: (p.y as f64 / NM).round() as i64 };
            if q.last().map_or(true, |&l| !P { x: pm.x - l.x, y: pm.y - l.y }.shorter_than(5)) {
                q.push(pm);
            }
        }
        while q.len() > 1 && (P { x: q[0].x - q[q.len() - 1].x, y: q[0].y - q[q.len() - 1].y }).shorter_than(5) {
            q.pop();
        }
        loop {
            let n = q.len();
            if n < 3 {
                break;
            }
            let mut i = 0;
            while q.len() >= 3 && i < q.len() {
                let len = q.len();
                let a = q[(i + len - 1) % len];
                let b = q[i];
                let c2 = q[(i + 1) % len];
                let cross = (b.x - a.x) * (c2.y - a.y) - (b.y - a.y) * (c2.x - a.x);
                if b == c2 || cross == 0 {
                    q.remove(i);
                } else {
                    i += 1;
                }
            }
            if q.len() == n {
                break;
            }
        }
        if q.len() >= 3 {
            polys.push(q);
        }
    }
    let mut segs = Vec::new();
    for poly in &polys {
        let n = poly.len();
        for i in 0..n {
            segs.push((poly[i], poly[(i + 1) % n]));
        }
    }
    (polys, segs)
}

// ===========================================================================
// robustness: silence + retry boostvoronoi degeneracy panics
// ===========================================================================

/// Drop `boostvoronoi`'s own degeneracy panics (always caught + retried below);
/// any panic from outside the crate still reaches the default hook.
fn quiet_voronoi_panics() {
    use std::sync::Once;
    static HOOK: Once = Once::new();
    HOOK.call_once(|| {
        let default = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            let from_voronoi =
                info.location().is_some_and(|l| l.file().contains("boostvoronoi"));
            if !from_voronoi {
                default(info);
            }
        }));
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo2d::Contour;

    fn ring(pts: &[(f64, f64)]) -> Polygons {
        let mut p = Polygons::new();
        p.push(Contour::new(pts.iter().map(|&(x, y)| geo2d::Point::from_mm(x, y)).collect()));
        p
    }

    fn sharpest_turn_deg(pts: &[geo2d::Point]) -> f64 {
        let mut worst: f64 = 0.0;
        for w in pts.windows(3) {
            let v1 = (w[1].x_mm() - w[0].x_mm(), w[1].y_mm() - w[0].y_mm());
            let v2 = (w[2].x_mm() - w[1].x_mm(), w[2].y_mm() - w[1].y_mm());
            let (n1, n2) = (v1.0.hypot(v1.1), v2.0.hypot(v2.1));
            if n1 < 1e-6 || n2 < 1e-6 {
                continue;
            }
            let cos = ((v1.0 * v2.0 + v1.1 * v2.1) / (n1 * n2)).clamp(-1.0, 1.0);
            worst = worst.max(cos.acos().to_degrees());
        }
        worst
    }

    #[test]
    fn taper_is_one_clean_widening_bead() {
        // 8 mm wedge: 0.8 mm wide at the left, 0.2 mm at the right.
        let mas = medial_axis(&ring(&[(0.0, -0.4), (8.0, -0.1), (8.0, 0.1), (0.0, 0.4)]), 0.09, 0.8);
        assert_eq!(mas.len(), 1, "a single sliver should trace as one polyline");
        let tp = &mas[0];
        let (wmin, wmax) = tp.widths.iter().fold((f64::MAX, 0.0f64), |(a, b), &w| (a.min(w), b.max(w)));
        assert!(wmin < 0.35 && wmax > 0.6, "width should track the taper, got {wmin:.2}..{wmax:.2}");
        assert!(sharpest_turn_deg(&tp.points) < 45.0, "centerline must not fold");
    }

    #[test]
    fn curved_strip_does_not_fragment_or_fold() {
        // Half-annulus, ~0.5 mm wide — the case the old centerline W-folded on.
        let (ro, ri, n) = (5.0, 4.5, 24);
        let mut pts = Vec::new();
        for i in 0..=n {
            let t = std::f64::consts::PI * (i as f64 / n as f64);
            pts.push((ro * t.cos(), ro * t.sin()));
        }
        for i in (0..=n).rev() {
            let t = std::f64::consts::PI * (i as f64 / n as f64);
            pts.push((ri * t.cos(), ri * t.sin()));
        }
        let mas = medial_axis(&ring(&pts), 0.09, 0.8);
        assert_eq!(mas.len(), 1, "the arc should be one bead after concat, not fragments");
        assert!(sharpest_turn_deg(&mas[0].points) < 30.0, "arc centerline must stay smooth");
    }
}

/// Deterministically nudge every vertex a few µm to break a near-degenerate
/// site configuration the Voronoi rejected (reproducible, per-vertex).
fn jitter(polys: &Polygons, attempt: u64) -> Polygons {
    let amp = 5_000 * attempt as i64; // nm
    let span = (2 * amp + 1) as u64;
    let mut out = polys.clone();
    for c in &mut out.contours {
        for p in &mut c.points {
            let h = (p.x as u64 ^ (p.y as u64).rotate_left(32))
                .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                .wrapping_add(attempt.wrapping_mul(0xD1B5_4A32_D192_ED03));
            p.x += (h % span) as i64 - amp;
            p.y += ((h >> 32) % span) as i64 - amp;
        }
    }
    out
}
