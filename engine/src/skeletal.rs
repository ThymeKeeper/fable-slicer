//! Exact skeletal trapezoidation — the Arachne graph walk.
//!
//! Replaces the grid extractor's piecewise extraction + reassembly (level sets
//! sewn to ridge traces) with the paper's construction: the segment Voronoi
//! diagram of the layer polygon, restricted to the interior, IS the medial
//! axis; its edges + ribs to the boundary decompose the region into quads; the
//! beading scheme assigns bead counts along the skeleton; bead-count changes
//! become explicit transition anchors; and per bead index the quads are walked
//! so junction points chain into polylines/rings. Rings come out as rings and
//! junctions as graph nodes — the two failure modes of the grid version
//! (C-ring gaps, corner bead pile-ups) are impossible by construction.
//!
//! Reference: Kuipers, Doubrovski, Wu, Wang 2020, *A framework for adaptive
//! width control of dense contour-parallel toolpaths* (§3 skeletal
//! trapezoidation, §5 toolpath extraction, §6.2 transitions), as implemented
//! by CuraEngine's `SkeletalTrapezoidation` (read for algorithms — this is a
//! clean reimplementation, see PLAN decision log). The beading scheme itself
//! is `wall.rs`'s `Scheme` verbatim: stretch / absorb / absorb-2 / saturated
//! with the same thresholds, so the infill gate in `plan.rs` stays in sync.
//!
//! Internally everything is integer micrometers (boost::polygon::voronoi wants
//! integer input; µm keeps predicate intermediates small). geo2d nanometers
//! are scaled at the boundary.

use crate::wall::{join_beads, Bead, VariableWalls};
use boostvoronoi::prelude::*;
use geo2d::Polygons;

/// Sentinel for "no edge / no node" in the index arenas.
const NIL: usize = usize::MAX;

/// nm per µm.
const NM: f64 = 1000.0;

/// Generic arithmetic snap (µm) — whether a transition end coincides with an
/// existing node (Cura's `snap_dist`).
const SNAP_DIST: i64 = 20;
/// Collapse graph edges shorter than this (µm): Voronoi vertices are rounded
/// to integers, which can leave zero-length quad sides.
const COLLAPSE_DIST: i64 = 5;
/// Parabolic / point-point Voronoi edges are discretized in ~0.8 mm steps.
const DISCRETIZATION_STEP: i64 = 800;
/// Wedge angle below which the boundary pair is "central" (= the paper's
/// 180° − limit bisector angle; the paper uses δmax = 135° → 45°). Polygonized
/// arcs concentrate the medial-axis climb at vertices, so legitimate
/// bead-count transitions show local slopes ~0.1–0.15 (sin 6°–9°) while true
/// corner ribs sit at sin ≥ 45°; 45° splits the two populations with margin
/// on both sides (Cura's 10° default shattered thin polygonized rings here —
/// measured on the Benchy chimney, layer 218).
const TRANSITIONING_ANGLE: f64 = 45.0 * std::f64::consts::PI / 180.0;
/// Dissolve opposing transitions closer together than this along the skeleton
/// (Cura's `wall_transition_filter_distance`).
const TRANSITION_FILTER_DIST: i64 = 100_000;

pub(crate) fn variable_walls_exact(
    outer: &Polygons,
    inner: &Polygons,
    lw: f64,
    sp: f64,
    max_inner: usize,
) -> Option<VariableWalls> {
    // The walk must never take a slice down with it (a panic inside the rayon
    // layer pool aborts the whole process): treat any panic as "this layer is
    // degenerate", which falls back to the grid extractor.
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        variable_walls_exact_inner(outer, inner, lw, sp, max_inner)
    }));
    match result {
        Ok(vw) => vw,
        Err(_) => {
            if std::env::var("ARACHNE_DBG").is_ok() {
                eprintln!("  skeletal: panic caught — grid fallback");
            }
            None
        }
    }
}

fn variable_walls_exact_inner(
    outer: &Polygons,
    inner: &Polygons,
    lw: f64,
    sp: f64,
    max_inner: usize,
) -> Option<VariableWalls> {
    let mut out = VariableWalls { inner: Vec::new(), thin_outer: Vec::new() };
    if max_inner > 0 && !inner.is_empty() {
        if let Some(mut st) = St::build(inner, lw, sp, max_inner)? {
            out.inner = st.generate_toolpaths(lw, sp);
        }
    }
    if !outer.is_empty() {
        if let Some(st) = St::build(outer, lw, sp, 1)? {
            out.thin_outer = st.thin_beads(lw, sp);
        }
    }
    Some(out)
}

// ===========================================================================
// µm integer geometry
// ===========================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct P {
    x: i64,
    y: i64,
}

impl P {
    fn sub(self, o: P) -> P {
        P { x: self.x - o.x, y: self.y - o.y }
    }
    fn add(self, o: P) -> P {
        P { x: self.x + o.x, y: self.y + o.y }
    }
    fn vsize(self) -> f64 {
        (self.x as f64).hypot(self.y as f64)
    }
    fn vsize2(self) -> f64 {
        let (x, y) = (self.x as f64, self.y as f64);
        x * x + y * y
    }
    fn dist(self, o: P) -> f64 {
        self.sub(o).vsize()
    }
    fn shorter_than(self, len: i64) -> bool {
        self.x.abs() <= len && self.y.abs() <= len && self.vsize2() <= (len * len) as f64
    }
}

fn dot(a: P, b: P) -> f64 {
    a.x as f64 * b.x as f64 + a.y as f64 * b.y as f64
}

/// `v` scaled to length `len` (zero stays zero).
fn normal(v: P, len: f64) -> P {
    let s = v.vsize();
    if s < 1e-9 {
        return P { x: 0, y: 0 };
    }
    P { x: (v.x as f64 * len / s).round() as i64, y: (v.y as f64 * len / s).round() as i64 }
}

/// Closest point on the (unbounded) line through `a`–`b`. A degenerate line
/// returns `a` (point-cell ribs project onto the cell's source point).
fn closest_on_line(p: P, a: P, b: P) -> P {
    let ab = b.sub(a);
    let l2 = ab.vsize2();
    if l2 < 1e-9 {
        return a;
    }
    let t = dot(p.sub(a), ab) / l2;
    P { x: a.x + (ab.x as f64 * t).round() as i64, y: a.y + (ab.y as f64 * t).round() as i64 }
}

/// Closest point on the segment `a`–`b`.
fn closest_on_segment(p: P, a: P, b: P) -> P {
    let ab = b.sub(a);
    let l2 = ab.vsize2();
    if l2 < 1e-9 {
        return a;
    }
    let t = (dot(p.sub(a), ab) / l2).clamp(0.0, 1.0);
    P { x: a.x + (ab.x as f64 * t).round() as i64, y: a.y + (ab.y as f64 * t).round() as i64 }
}

/// Even-odd point-in-polygons test over the µm-quantized contours.
fn inside(polys: &[Vec<P>], p: P) -> bool {
    let mut odd = false;
    for c in polys {
        let n = c.len();
        let mut j = n - 1;
        for i in 0..n {
            let (pi, pj) = (c[i], c[j]);
            if (pi.y > p.y) != (pj.y > p.y) {
                let t = (p.y - pi.y) as f64 / (pj.y - pi.y) as f64;
                if (p.x as f64) < pi.x as f64 + t * (pj.x - pi.x) as f64 {
                    odd = !odd;
                }
            }
            j = i;
        }
    }
    odd
}

// ===========================================================================
// Beading scheme — wall.rs's `Scheme` recast as the strategy interface the
// graph walk needs (bead counts, per-thickness beadings, transition points).
// ===========================================================================

/// A "beading": how `total` µm of thickness is covered by beads. `locations`
/// are centerline distances from the local boundary (ascending across the full
/// thickness), `widths` the bead widths at those locations (µm).
#[derive(Clone, Debug)]
struct Beading {
    total: i64,
    widths: Vec<f64>,
    locations: Vec<f64>,
    /// Thickness no bead covers (saturated middles — infill territory). Not
    /// consumed by the walk itself: the infill region is computed
    /// independently in plan.rs; this documents the scheme's accounting.
    #[allow(dead_code)]
    left_over: f64,
}

#[derive(Clone, Copy)]
struct Strategy {
    sp: f64,
    lw: f64,
    /// Max beads across (= 2 × inner-ring budget).
    cap2: i32,
}

impl Strategy {
    /// The absorb window: a leftover under ~1.3 line widths gets eaten by
    /// widening the capped rings (wall.rs's `sliver`).
    fn sliver(self) -> f64 {
        self.sp * 1.444
    }
    /// Saturation threshold: thicker than this and the leftover is a real
    /// strip that infill covers (beads revert to nominal spacing).
    fn t_sat(self) -> f64 {
        self.cap2 as f64 * self.sp + self.sliver() + self.sp / 0.9
    }

    fn optimal_bead_count(self, t: i64) -> i32 {
        let t = t as f64;
        let fit = ((t / self.sp).round() as i32).max(1);
        if fit <= self.cap2 {
            return fit;
        }
        let remainder = t - self.cap2 as f64 * self.sp;
        if remainder < self.sliver() {
            self.cap2
        } else if remainder < self.sliver() + self.sp / 0.9 {
            // About one more bead's worth: a single center bead beats handing
            // solid fill a strip it can barely cover.
            self.cap2 + 1
        } else {
            self.cap2 // saturated: nominal rings, remainder belongs to infill
        }
    }

    /// Thickness at which the optimal count jumps `n` → `n+1`.
    fn transition_thickness(self, n: i32) -> i64 {
        if n < self.cap2 {
            ((n as f64 + 0.5) * self.sp) as i64
        } else if n == self.cap2 {
            (self.cap2 as f64 * self.sp + self.sliver()) as i64
        } else {
            i64::MAX / 4
        }
    }

    /// Bead width for a centerline pitch (wall.rs's `width_of`).
    fn width_of(self, pitch: f64) -> f64 {
        (pitch + (self.lw - self.sp)).clamp(self.lw * 0.5, self.lw * 1.75)
    }

    fn compute(self, t: i64, n: i32) -> Beading {
        let tf = t as f64;
        let n = n.max(0) as usize;
        if n == 0 {
            return Beading { total: t, widths: Vec::new(), locations: Vec::new(), left_over: tf };
        }
        if n as i32 == self.cap2 && tf >= self.t_sat() {
            // Saturated: nominal pitch hugging both boundaries; the middle is
            // infill territory (classic geometry, kept exact so the infill
            // region computed in plan.rs agrees).
            let cap = n / 2;
            let mut locations = Vec::with_capacity(n);
            let mut widths = Vec::with_capacity(n);
            for i in 0..cap {
                locations.push((i as f64 + 0.5) * self.sp);
                widths.push(self.width_of(self.sp));
            }
            for i in 0..cap {
                locations.push(tf - (cap - 1 - i) as f64 * self.sp - 0.5 * self.sp);
                widths.push(self.width_of(self.sp));
            }
            return Beading { total: t, widths, locations, left_over: tf - n as f64 * self.sp };
        }
        // Stretch / absorb: the whole thickness shared evenly. Locations are
        // exact (t/n) so odd counts put the center bead exactly on the
        // skeleton; only the *width* mapping is clamped.
        let pitch = tf / n as f64;
        let w = self.width_of(pitch);
        let locations: Vec<f64> = (0..n).map(|i| (i as f64 + 0.5) * pitch).collect();
        Beading { total: t, widths: vec![w; n], locations, left_over: 0.0 }
    }

    /// Transition ramp length (Kuipers §6.2; ≈ one line width — the same
    /// constant the grid version diffused by).
    fn transition_length(self, n: i32) -> i64 {
        if n == 0 {
            10
        } else {
            self.lw as i64
        }
    }

    /// Where within the ramp the count actually jumps (0 = at the low end).
    fn anchor_pos(self, n: i32) -> f64 {
        let t = self.transition_thickness(n) as f64;
        (1.0 - (t - n as f64 * self.sp) / self.sp).clamp(0.1, 0.9)
    }

    /// Thicknesses (within a constant bead count) where bead positions kink —
    /// extra ribs pin those to graph nodes. Our scheme is linear except at the
    /// saturation threshold.
    fn nonlinear_thicknesses(self, n: i32) -> Vec<i64> {
        if n == self.cap2 {
            vec![self.t_sat() as i64]
        } else {
            Vec::new()
        }
    }
}

// ===========================================================================
// Voronoi diagram flattened to plain arrays
// ===========================================================================

#[derive(Clone, Copy, PartialEq)]
enum Cat {
    SegStart,
    SegEnd,
    Seg,
    Point,
}

struct VCell {
    source: usize,
    cat: Cat,
    incident: usize, // NIL when degenerate
}

struct VEdge {
    cell: usize,
    v0: usize, // NIL = infinite end
    twin: usize,
    next: usize,
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
    fn finite(&self, e: usize) -> bool {
        self.v0(e) != NIL && self.v1(e) != NIL
    }
    fn point(&self, v: usize) -> P {
        P { x: self.verts[v].0.round() as i64, y: self.verts[v].1.round() as i64 }
    }
}

/// Run boost::polygon::voronoi (the boostvoronoi port) over the segments.
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
        let incident = c.get_incident_edge().map_or(NIL, |e| e.usize());
        cells.push(VCell { source, cat, incident });
    }
    let mut edges = Vec::with_capacity(diagram.edges().len());
    for e in diagram.edges() {
        edges.push(VEdge {
            cell: e.cell().ok()?.usize(),
            v0: e.vertex0().map_or(NIL, |v| v.usize()),
            twin: e.twin().ok()?.usize(),
            next: e.next().ok()?.usize(),
            secondary: e.is_secondary(),
        });
    }
    Some(Voro { cells, edges, verts })
}

/// The point site that generated a (point-category) cell.
fn source_point(cell: &VCell, segs: &[(P, P)]) -> P {
    match cell.cat {
        Cat::SegStart => segs[cell.source].0,
        Cat::SegEnd => segs[cell.source].1,
        _ => segs[cell.source].0, // Cat::Seg never asked; Point unused (no point input)
    }
}

// ===========================================================================
// Half-edge graph (Cura's SkeletalTrapezoidationGraph, index arenas)
// ===========================================================================

#[derive(Clone)]
struct Node {
    p: P,
    /// Distance to the polygon boundary (µm); −1 until known.
    r: i64,
    bead_count: i32,
    /// 0 at proper nodes; in (0,1) between transition ends, where the beading
    /// interpolates between `bead_count` and `bead_count + 1`.
    transition_ratio: f64,
    beading: usize, // index into St::beadings, NIL = none
    incident: usize,
    dead: bool,
}

#[derive(Clone, Copy, PartialEq)]
enum EType {
    Normal,
    ExtraVd,
    TransitionEnd,
}

#[derive(Clone, Copy)]
struct TransMid {
    pos: i64,
    lower_count: i32,
    feature_radius: i64,
}

#[derive(Clone, Copy)]
struct TransEnd {
    pos: i64,
    lower_count: i32,
    is_lower_end: bool,
}

#[derive(Clone, Copy)]
struct Junction {
    p: P,
    w: f64,
    idx: usize,
}

struct Edge {
    from: usize,
    to: usize,
    twin: usize,
    next: usize,
    prev: usize,
    etype: EType,
    /// −1 unknown, 0 no, 1 yes.
    central: i8,
    transitions: Vec<TransMid>,
    transition_ends: Vec<TransEnd>,
    junctions: Vec<Junction>,
    dead: bool,
}

impl Edge {
    fn new(etype: EType) -> Edge {
        Edge {
            from: NIL,
            to: NIL,
            twin: NIL,
            next: NIL,
            prev: NIL,
            etype,
            central: -1,
            transitions: Vec::new(),
            transition_ends: Vec::new(),
            junctions: Vec::new(),
            dead: false,
        }
    }
    fn is_central(&self) -> bool {
        self.central == 1
    }
}

#[derive(Clone)]
struct BeadingProp {
    beading: Beading,
    dist_to_bottom: i64,
    dist_from_top: i64,
    upward_only: bool,
}

/// One accumulating output polyline (Cura's ExtrusionLine).
struct Acc {
    pts: Vec<P>,
    ws: Vec<f64>,
    odd: bool,
}

struct St {
    strategy: Strategy,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    beadings: Vec<BeadingProp>,
    /// Output polylines binned by bead (inset) index.
    bins: Vec<Vec<Acc>>,
}

impl St {
    // =======================================================================
    // Construction (Cura constructFromPolygons + transferEdge + discretize)
    // =======================================================================

    /// Build the skeletal graph for a region. Outer `Option`: `None` = the
    /// Voronoi construction failed (degenerate input — caller falls back to
    /// the grid extractor). Inner `Option`: `None` = region empty, nothing to
    /// do.
    #[allow(clippy::type_complexity)]
    fn build(region: &Polygons, lw: f64, sp: f64, max_inner: usize) -> Option<Option<St>> {
        // Mesh-facet scallops would add a parabola fan per notch; smooth below
        // ridge scale (same constant as the grid version).
        let region = geo2d::simplify(region, lw * 0.25);

        // Scale to µm, merging points closer than 5µm: micro-segments survive
        // exact dedup but starve the Voronoi cell walk (their cells' vertices
        // collide within the matching tolerance).
        let mut polys: Vec<Vec<P>> = Vec::new();
        for c in &region.contours {
            let mut q: Vec<P> = Vec::with_capacity(c.points.len());
            for p in &c.points {
                let pm = P { x: (p.x as f64 / NM).round() as i64, y: (p.y as f64 / NM).round() as i64 };
                if q.last().map_or(true, |&l| !pm.sub(l).shorter_than(5)) {
                    q.push(pm);
                }
            }
            while q.len() > 1 && q.first().unwrap().sub(*q.last().unwrap()).shorter_than(5) {
                q.pop();
            }
            // Exactly-collinear vertices (common after rounding; simplify pins
            // contour start points) make boost emit the shared endpoint's cell
            // as two *infinite* secondary edges — no finite vertex exists at
            // the join and the cell-range walk cannot terminate. Strip them
            // (and the duplicates that spike removal leaves behind).
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
        if polys.is_empty() {
            return Some(None);
        }
        let mut segs: Vec<(P, P)> = Vec::new();
        for c in &polys {
            for i in 0..c.len() {
                segs.push((c[i], c[(i + 1) % c.len()]));
            }
        }

        let dbg = std::env::var("ARACHNE_DBG").is_ok();
        let Some(voro) = build_voronoi(&segs) else {
            if dbg {
                eprintln!("  skeletal: voronoi build failed ({} segs)", segs.len());
            }
            return None;
        };

        let mut st = St {
            strategy: Strategy { sp: sp * 1000.0, lw: lw * 1000.0, cap2: (max_inner * 2) as i32 },
            nodes: Vec::new(),
            edges: Vec::new(),
            beadings: Vec::new(),
            bins: Vec::new(),
        };

        // vd vertex -> graph node, vd edge -> last graph edge of its chain.
        let mut vd_node: Vec<usize> = vec![NIL; voro.verts.len()];
        let mut vd_edge: Vec<usize> = vec![NIL; voro.edges.len()];

        for ci in 0..voro.cells.len() {
            let incident = voro.cells[ci].incident;
            if incident == NIL {
                continue;
            }
            let range = if voro.cells[ci].cat == Cat::Seg {
                let r = st.segment_cell_range(&voro, ci, &segs);
                if r.is_none() && dbg {
                    // A skipped segment cell leaves unpaired half-edges; the
                    // validation below converts that into a grid fallback.
                    let (f, t) = segs[voro.cells[ci].source];
                    eprintln!(
                        "  skeletal: cell range failed for seg ({:.3},{:.3})->({:.3},{:.3})",
                        f.x as f64 / 1000.0,
                        f.y as f64 / 1000.0,
                        t.x as f64 / 1000.0,
                        t.y as f64 / 1000.0
                    );
                }
                r
            } else {
                st.point_cell_range(&voro, ci, &segs, &polys)
            };
            let Some((start_src, end_src, start_edge, end_edge)) = range else { continue };

            // Copy the interior chain start..=end into the graph, ribbing each
            // intermediate node to the boundary. Any structural inconsistency
            // (degenerate input) bails the whole layer to the grid fallback.
            let mut prev_edge = NIL;
            st.transfer_edge(
                start_src,
                voro.point(voro.v1(start_edge)),
                start_edge,
                &mut prev_edge,
                start_src,
                end_src,
                &voro,
                &segs,
                &mut vd_node,
                &mut vd_edge,
            )?;
            let starting_node = vd_node[voro.v0(start_edge)];
            st.nodes[starting_node].r = 0;

            st.make_rib(&mut prev_edge, start_src, end_src);
            let mut e = voro.edges[start_edge].next;
            while e != end_edge {
                if !voro.finite(e) {
                    return None; // interior chains are finite by construction
                }
                let (v1, v2) = (voro.point(voro.v0(e)), voro.point(voro.v1(e)));
                st.transfer_edge(v1, v2, e, &mut prev_edge, start_src, end_src, &voro, &segs, &mut vd_node, &mut vd_edge)?;
                st.make_rib(&mut prev_edge, start_src, end_src);
                e = voro.edges[e].next;
            }
            st.transfer_edge(
                voro.point(voro.v0(end_edge)),
                end_src,
                end_edge,
                &mut prev_edge,
                start_src,
                end_src,
                &voro,
                &segs,
                &mut vd_node,
                &mut vd_edge,
            )?;
            let last_to = st.edges[prev_edge].to;
            st.nodes[last_to].r = 0;
        }

        if st.edges.is_empty() {
            return Some(None);
        }

        // Structural validation: a cell whose range walk failed leaves its
        // neighbours' twins unset; the half-edge surgery below (and the quad
        // walk) require a complete twin pairing. Bail to the grid fallback
        // rather than limp on a broken graph.
        for e in 0..st.edges.len() {
            if !st.edges[e].dead && st.edges[e].twin == NIL {
                if dbg {
                    let f = st.edges[e].from;
                    eprintln!(
                        "  skeletal: unpaired half-edge at ({:.2},{:.2}) — bail",
                        st.nodes[f].p.x as f64 / 1000.0,
                        st.nodes[f].p.y as f64 / 1000.0
                    );
                }
                return None;
            }
        }

        st.separate_pointy_quad_end_nodes();
        st.collapse_small_edges(COLLAPSE_DIST);

        // Point incident_edge at each quad start so around-node iteration
        // (twin.next) can reach every edge without walking backwards.
        for e in 0..st.edges.len() {
            if !st.edges[e].dead && st.edges[e].prev == NIL {
                let f = st.edges[e].from;
                st.nodes[f].incident = e;
            }
        }
        Some(Some(st))
    }

    /// The interior chain of a segment cell runs from the vertex at the
    /// segment's `to` around to the vertex at its `from` (combinatorial — the
    /// interior side for CCW outers / CW holes). Matching is exact first
    /// (site-coincident vertices are exact); the 1µm-tolerant retry covers
    /// rounding, but must not run first — near sharp tips several vertices
    /// cluster within a micron and the tolerant match can grab the wrong one.
    fn segment_cell_range(&self, voro: &Voro, ci: usize, segs: &[(P, P)]) -> Option<(P, P, usize, usize)> {
        self.segment_cell_range_tol(voro, ci, segs, 0)
            .or_else(|| self.segment_cell_range_tol(voro, ci, segs, 1))
    }

    fn segment_cell_range_tol(&self, voro: &Voro, ci: usize, segs: &[(P, P)], tol: i64) -> Option<(P, P, usize, usize)> {
        let (from, to) = segs[voro.cells[ci].source];
        let eq = |p: P, q: P| p.sub(q).shorter_than(tol);
        let mut starting = NIL;
        let mut ending = NIL;
        let mut seen_possible_start = false;
        let mut after_start = false;
        let mut ending_before_start = false;
        let first = voro.cells[ci].incident;
        let mut e = first;
        loop {
            if voro.finite(e) {
                let v0 = voro.point(voro.v0(e));
                let v1 = voro.point(voro.v1(e));
                if eq(v0, to) && !after_start {
                    starting = e;
                    seen_possible_start = true;
                } else if seen_possible_start {
                    after_start = true;
                }
                if eq(v1, from) && (ending == NIL || ending_before_start) {
                    ending_before_start = !after_start;
                    ending = e;
                }
            }
            e = voro.edges[e].next;
            if e == first {
                break;
            }
        }
        if starting == NIL || ending == NIL || starting == ending {
            return None;
        }
        Some((to, from, starting, ending))
    }

    /// Point cells (polygon vertices) are interior only at reflex corners;
    /// test a cell vertex against the polygon and reject outside cells.
    fn point_cell_range(&self, voro: &Voro, ci: usize, segs: &[(P, P)], polys: &[Vec<P>]) -> Option<(P, P, usize, usize)> {
        let source = source_point(&voro.cells[ci], segs);
        let eq = |p: P, q: P| p.sub(q).shorter_than(1);
        let first = voro.cells[ci].incident;
        // Any finite vertex that isn't the source point itself decides
        // inside/outside for the whole cell (cells don't cross the boundary).
        let mut some_point: Option<P> = None;
        let mut e = first;
        loop {
            if voro.v0(e) != NIL {
                let p = voro.point(voro.v0(e));
                if !eq(p, source) {
                    some_point = Some(p);
                    break;
                }
            }
            if voro.v1(e) != NIL {
                let p = voro.point(voro.v1(e));
                if !eq(p, source) {
                    some_point = Some(p);
                    break;
                }
            }
            e = voro.edges[e].next;
            if e == first {
                break;
            }
        }
        if !inside(polys, some_point?) {
            return None;
        }
        let mut starting = NIL;
        let mut ending = NIL;
        let mut e = first;
        loop {
            if !voro.finite(e) {
                return None; // hull cell sneaking through the inside test
            }
            if eq(voro.point(voro.v1(e)), source) {
                starting = voro.edges[e].next;
                ending = e;
            }
            e = voro.edges[e].next;
            if e == first {
                break;
            }
        }
        if starting == NIL || ending == NIL || starting == ending {
            return None;
        }
        Some((source, source, starting, ending))
    }

    fn make_node(&mut self, vd_node: &mut [usize], v: usize, p: P) -> usize {
        if vd_node[v] != NIL {
            return vd_node[v];
        }
        self.nodes.push(Node { p, r: -1, bead_count: -1, transition_ratio: 0.0, beading: NIL, incident: NIL, dead: false });
        vd_node[v] = self.nodes.len() - 1;
        self.nodes.len() - 1
    }

    fn new_node(&mut self, p: P) -> usize {
        self.nodes.push(Node { p, r: -1, bead_count: -1, transition_ratio: 0.0, beading: NIL, incident: NIL, dead: false });
        self.nodes.len() - 1
    }

    fn new_edge(&mut self, etype: EType) -> usize {
        self.edges.push(Edge::new(etype));
        self.edges.len() - 1
    }

    /// Rib from the chain's current head node down to its boundary foot.
    /// Also sets the head node's distance-to-boundary.
    fn make_rib(&mut self, prev_edge: &mut usize, start_src: P, end_src: P) {
        let to = self.edges[*prev_edge].to;
        let p = closest_on_line(self.nodes[to].p, start_src, end_src);
        self.nodes[to].r = self.nodes[to].p.dist(p).round() as i64;
        let node = self.new_node(p);
        self.nodes[node].r = 0;

        let forth = self.new_edge(EType::ExtraVd);
        let back = self.new_edge(EType::ExtraVd);
        self.edges[*prev_edge].next = forth;
        self.edges[forth].prev = *prev_edge;
        self.edges[forth].from = to;
        self.edges[forth].to = node;
        self.edges[forth].twin = back;
        self.edges[back].twin = forth;
        self.edges[back].from = node;
        self.edges[back].to = to;
        self.nodes[node].incident = back;
        *prev_edge = back;
    }

    /// Copy one Voronoi edge (possibly discretized into several pieces) into
    /// the graph. When the twin chain already exists, mirror it instead so
    /// both sides share nodes.
    #[allow(clippy::too_many_arguments)]
    fn transfer_edge(
        &mut self,
        from: P,
        to: P,
        ve: usize,
        prev_edge: &mut usize,
        start_src: P,
        end_src: P,
        voro: &Voro,
        segs: &[(P, P)],
        vd_node: &mut Vec<usize>,
        vd_edge: &mut [usize],
    ) -> Option<()> {
        let vtwin = voro.edges[ve].twin;
        if vd_edge[vtwin] != NIL {
            // Twin chain exists: walk it backwards, emitting reversed edges.
            let end_node = vd_node[voro.v1(ve)];
            if end_node == NIL {
                return None;
            }
            let mut twin = vd_edge[vtwin];
            loop {
                if twin == NIL {
                    return None;
                }
                let edge = self.new_edge(EType::Normal);
                self.edges[edge].from = self.edges[twin].to;
                self.edges[edge].to = self.edges[twin].from;
                self.edges[edge].twin = twin;
                self.edges[twin].twin = edge;
                let f = self.edges[edge].from;
                self.nodes[f].incident = edge;

                if *prev_edge != NIL {
                    self.edges[edge].prev = *prev_edge;
                    self.edges[*prev_edge].next = edge;
                }
                *prev_edge = edge;

                if self.edges[*prev_edge].to == end_node {
                    return Some(());
                }
                let p1 = self.edges[twin].prev;
                if p1 == NIL {
                    return None;
                }
                let t1 = self.edges[p1].twin;
                if t1 == NIL {
                    return None;
                }
                let p2 = self.edges[t1].prev;
                if p2 == NIL {
                    return None;
                }
                twin = p2;
                self.make_rib(prev_edge, start_src, end_src);
            }
        }

        let discretized = self.discretize(ve, voro, segs);
        if discretized.len() < 2 {
            return None;
        }
        let mut v0 = if *prev_edge != NIL {
            self.edges[*prev_edge].to
        } else {
            let n = self.make_node(vd_node, voro.v0(ve), from);
            n
        };
        for (k, &p1) in discretized.iter().enumerate().skip(1) {
            let last = k == discretized.len() - 1;
            let v1 = if last { self.make_node(vd_node, voro.v1(ve), to) } else { self.new_node(p1) };
            let edge = self.new_edge(EType::Normal);
            self.edges[edge].from = v0;
            self.edges[edge].to = v1;
            self.nodes[v0].incident = edge;
            if *prev_edge != NIL {
                self.edges[edge].prev = *prev_edge;
                self.edges[*prev_edge].next = edge;
            }
            *prev_edge = edge;
            v0 = v1;
            if !last {
                // Rib for the final piece is introduced by the caller.
                self.make_rib(prev_edge, start_src, end_src);
            }
        }
        vd_edge[ve] = *prev_edge;
        Some(())
    }

    /// Break a Voronoi edge into pieces with ~linear width change: parabolas
    /// (point vs segment) and point-point bisectors are sampled; straight
    /// segment-segment bisectors pass through. Marking points are inserted
    /// where the wedge angle crosses the transitioning angle so the central
    /// flag is uniform per piece.
    fn discretize(&self, ve: usize, voro: &Voro, segs: &[(P, P)]) -> Vec<P> {
        let left = &voro.cells[voro.edges[ve].cell];
        let right = &voro.cells[voro.edges[voro.edges[ve].twin].cell];
        let start = voro.point(voro.v0(ve));
        let end = voro.point(voro.v1(ve));
        let point_left = left.cat != Cat::Seg;
        let point_right = right.cat != Cat::Seg;
        if (!point_left && !point_right) || voro.edges[ve].secondary {
            return vec![start, end];
        }
        if point_left != point_right {
            let p = source_point(if point_left { left } else { right }, segs);
            let s = segs[if point_left { right.source } else { left.source }];
            return discretize_parabola(p, s, start, end);
        }
        // Point-point: straight bisector, but the radius along it is a
        // parabola — sample it, forcing a midpoint and the marking bounds.
        let left_p = source_point(left, segs);
        let right_p = source_point(right, segs);
        let d = right_p.sub(left_p).vsize();
        let middle = P { x: (left_p.x + right_p.x) / 2, y: (left_p.y + right_p.y) / 2 };
        let x_dir = P { x: -(right_p.y - left_p.y), y: right_p.x - left_p.x }; // turn90ccw
        let x_len = x_dir.vsize().max(1.0);
        let projected_x = |q: P| -> f64 { dot(q.sub(middle), x_dir) / x_len };
        let start_x = projected_x(start);
        let end_x = projected_x(end);

        let bound = 0.5 / ((std::f64::consts::PI - TRANSITIONING_ANGLE) * 0.5).tan();
        let mut marking_start_x = -d * bound;
        let mut marking_end_x = d * bound;
        let at_x = |x: f64| -> P {
            P {
                x: middle.x + (x_dir.x as f64 * x / x_len).round() as i64,
                y: middle.y + (x_dir.y as f64 * x / x_len).round() as i64,
            }
        };
        let mut marking_start = at_x(marking_start_x);
        let mut marking_end = at_x(marking_end_x);
        let mut direction = 1.0;
        if start_x > end_x {
            direction = -1.0;
            std::mem::swap(&mut marking_start, &mut marking_end);
            std::mem::swap(&mut marking_start_x, &mut marking_end_x);
        }

        let mut ret = vec![start];
        let mut add_marking_start = marking_start_x * direction > start_x * direction;
        let mut add_marking_end = marking_end_x * direction > start_x * direction;
        let ab_size = end.sub(start).vsize();
        let mut step_count = (ab_size / DISCRETIZATION_STEP as f64).round() as i64;
        if step_count % 2 == 1 {
            step_count += 1; // force a discretization point in the middle
        }
        for step in 1..step_count {
            let here = P {
                x: start.x + ((end.x - start.x) as f64 * step as f64 / step_count as f64).round() as i64,
                y: start.y + ((end.y - start.y) as f64 * step as f64 / step_count as f64).round() as i64,
            };
            let x_here = projected_x(here);
            if add_marking_start && marking_start_x * direction < x_here * direction {
                ret.push(marking_start);
                add_marking_start = false;
            }
            if add_marking_end && marking_end_x * direction < x_here * direction {
                ret.push(marking_end);
                add_marking_end = false;
            }
            ret.push(here);
        }
        if add_marking_end && marking_end_x * direction < end_x * direction {
            ret.push(marking_end);
        }
        ret.push(end);
        ret
    }

    /// Quad-start nodes shared by multiple quads (point cells pinch them)
    /// are duplicated so each quad start is reachable from its from-node.
    fn separate_pointy_quad_end_nodes(&mut self) {
        let mut visited = vec![false; self.nodes.len()];
        for e in 0..self.edges.len() {
            if self.edges[e].dead || self.edges[e].prev != NIL {
                continue;
            }
            let from = self.edges[e].from;
            if !visited[from] {
                visited[from] = true;
            } else {
                let dup = self.nodes[from].clone();
                self.nodes.push(dup);
                let new_node = self.nodes.len() - 1;
                self.nodes[new_node].incident = e;
                self.edges[e].from = new_node;
                let t = self.edges[e].twin;
                self.edges[t].to = new_node;
            }
        }
    }

    /// Rounded Voronoi vertices can leave degenerate quad sides; collapse
    /// them while keeping the half-edge invariants intact.
    fn collapse_small_edges(&mut self, snap: i64) {
        for e in 0..self.edges.len() {
            if self.edges[e].dead || self.edges[e].prev != NIL {
                continue;
            }
            let quad_start = e;
            let mut quad_end = quad_start;
            while self.edges[quad_end].next != NIL {
                quad_end = self.edges[quad_end].next;
            }
            let quad_mid = if self.edges[quad_start].next == quad_end { NIL } else { self.edges[quad_start].next };

            let should_collapse = |st: &St, a: usize, b: usize| st.nodes[a].p.sub(st.nodes[b].p).shorter_than(snap);

            if quad_mid != NIL && should_collapse(self, self.edges[quad_mid].from, self.edges[quad_mid].to) {
                let mid_twin = self.edges[quad_mid].twin;
                if mid_twin == NIL {
                    continue;
                }
                // Redirect every edge leaving the collapsed node.
                let target = self.edges[quad_mid].from;
                let mut e3 = quad_end;
                let mut guard = 0;
                while e3 != NIL && e3 != mid_twin {
                    self.edges[e3].from = target;
                    let t = self.edges[e3].twin;
                    self.edges[t].to = target;
                    e3 = self.edges[self.edges[e3].twin].next;
                    guard += 1;
                    if guard > 1000 {
                        break;
                    }
                }
                if self.nodes[target].incident == quad_mid {
                    let mt_next = self.edges[mid_twin].next;
                    self.nodes[target].incident = if mt_next != NIL { mt_next } else { self.edges[self.edges[quad_mid].prev].twin };
                }
                let dead_node = self.edges[quad_mid].to;
                self.nodes[dead_node].dead = true;
                let (mp, mn) = (self.edges[quad_mid].prev, self.edges[quad_mid].next);
                self.edges[mp].next = mn;
                self.edges[mn].prev = mp;
                let (tp, tn) = (self.edges[mid_twin].prev, self.edges[mid_twin].next);
                self.edges[tn].prev = tp;
                self.edges[tp].next = tn;
                self.edges[quad_mid].dead = true;
                self.edges[mid_twin].dead = true;
            }

            if should_collapse(self, self.edges[quad_start].from, self.edges[quad_end].to)
                && should_collapse(self, self.edges[quad_start].to, self.edges[quad_end].from)
            {
                // Whole quad degenerate: collapse both sides, dropping the cell.
                let st_twin = self.edges[quad_start].twin;
                let en_twin = self.edges[quad_end].twin;
                self.edges[st_twin].to = self.edges[quad_end].to;
                let qe_to = self.edges[quad_end].to;
                self.nodes[qe_to].incident = en_twin;
                let qe_from = self.edges[quad_end].from;
                if self.nodes[qe_from].incident == quad_end {
                    let et_next = self.edges[en_twin].next;
                    self.nodes[qe_from].incident = if et_next != NIL { et_next } else { self.edges[self.edges[quad_end].prev].twin };
                }
                let qs_from = self.edges[quad_start].from;
                self.nodes[qs_from].dead = true;
                self.edges[st_twin].twin = en_twin;
                self.edges[en_twin].twin = st_twin;
                self.edges[quad_start].dead = true;
                self.edges[quad_end].dead = true;
            }
        }
    }

    // =======================================================================
    // Transitioning (Cura updateIsCentral .. applyTransitions, extra ribs)
    // =======================================================================

    fn r(&self, n: usize) -> i64 {
        self.nodes[n].r
    }
    fn len(&self, e: usize) -> i64 {
        let (f, t) = (self.edges[e].from, self.edges[e].to);
        self.nodes[f].p.dist(self.nodes[t].p).round() as i64
    }

    /// Around-node successor: the next outgoing edge of `e`'s from-node.
    fn twin_next(&self, e: usize) -> usize {
        let t = self.edges[e].twin;
        if t == NIL {
            NIL
        } else {
            self.edges[t].next
        }
    }

    /// An edge is "central" when the boundary pieces flanking it are close
    /// enough to parallel that beads must adapt to the local thickness
    /// (|dR| < |AB|·sin(θ/2)); corner ribs (R changing at ~unit rate) and
    /// near-boundary noise are not.
    fn update_is_central(&mut self) {
        let cap = (TRANSITIONING_ANGLE * 0.5).sin();
        let outer_edge_filter = self.strategy.transition_thickness(0) / 2;
        for e in 0..self.edges.len() {
            if self.edges[e].dead {
                continue;
            }
            let twin = self.edges[e].twin;
            if twin != NIL && self.edges[twin].central != -1 {
                self.edges[e].central = self.edges[twin].central;
            } else if self.edges[e].etype == EType::ExtraVd {
                self.edges[e].central = 0;
            } else if self.r(self.edges[e].from).max(self.r(self.edges[e].to)) < outer_edge_filter {
                self.edges[e].central = 0;
            } else {
                let (f, t) = (self.edges[e].from, self.edges[e].to);
                let d_r = (self.r(t) - self.r(f)).abs() as f64;
                let d_d = self.nodes[f].p.dist(self.nodes[t].p);
                self.edges[e].central = i8::from(d_r < d_d * cap);
            }
        }
    }

    fn is_end_of_central(&self, e: usize) -> bool {
        if !self.edges[e].is_central() {
            return false;
        }
        if self.edges[e].next == NIL {
            return true;
        }
        let twin = self.edges[e].twin;
        let mut nx = self.edges[e].next;
        while nx != NIL && nx != twin {
            if self.edges[nx].is_central() {
                return false;
            }
            nx = self.twin_next(nx);
        }
        true
    }

    fn can_go_up(&self, e: usize, strict: bool, visited: &mut Vec<usize>) -> bool {
        let (f, t) = (self.edges[e].from, self.edges[e].to);
        if self.r(t) > self.r(f) {
            return true;
        }
        if self.r(t) < self.r(f) || strict {
            return false;
        }
        if visited.contains(&e) {
            return false;
        }
        visited.push(e);
        // Equidistant edge: recurse.
        let twin = self.edges[e].twin;
        let mut out = self.edges[e].next;
        while out != NIL && out != twin {
            if self.can_go_up(out, false, visited) {
                return true;
            }
            if self.edges[out].twin == NIL {
                return false;
            }
            out = self.twin_next(out);
        }
        false
    }

    fn dist_to_go_up(&self, e: usize, visited: &mut Vec<usize>) -> Option<i64> {
        let (f, t) = (self.edges[e].from, self.edges[e].to);
        if self.r(t) > self.r(f) {
            return Some(0);
        }
        if self.r(t) < self.r(f) {
            return None;
        }
        if visited.contains(&e) {
            return None;
        }
        visited.push(e);
        let mut ret: Option<i64> = None;
        let twin = self.edges[e].twin;
        let mut out = self.edges[e].next;
        while out != NIL && out != twin {
            if let Some(d) = self.dist_to_go_up(out, visited) {
                ret = Some(ret.map_or(d, |r| r.min(d)));
            }
            if self.edges[out].twin == NIL {
                return Some(0);
            }
            out = self.twin_next(out);
        }
        ret.map(|r| r + self.len(e))
    }

    fn is_upward(&self, e: usize) -> bool {
        let (f, t) = (self.edges[e].from, self.edges[e].to);
        if self.r(t) > self.r(f) {
            return true;
        }
        if self.r(t) < self.r(f) {
            return false;
        }
        let fwd = self.dist_to_go_up(e, &mut Vec::new());
        let bwd = self.dist_to_go_up(self.edges[e].twin, &mut Vec::new());
        match (fwd, bwd) {
            (Some(a), Some(b)) => a < b,
            (Some(_), None) => true,
            (None, Some(_)) => false,
            // Arbitrary but twin-consistent ordering.
            (None, None) => (self.nodes[t].p.x, self.nodes[t].p.y) < (self.nodes[f].p.x, self.nodes[f].p.y),
        }
    }

    fn is_local_maximum(&self, n: usize, strict: bool) -> bool {
        if self.r(n) == 0 {
            return false;
        }
        let first = self.nodes[n].incident;
        if first == NIL {
            return false;
        }
        let mut e = first;
        loop {
            if self.can_go_up(e, strict, &mut Vec::new()) {
                return false;
            }
            let t = self.edges[e].twin;
            if t == NIL || self.edges[t].next == NIL {
                return false; // on the boundary
            }
            e = self.edges[t].next;
            if e == first {
                break;
            }
        }
        true
    }

    fn is_multi_intersection(&self, n: usize) -> bool {
        let first = self.nodes[n].incident;
        if first == NIL {
            return false;
        }
        let mut count = 0;
        let mut e = first;
        loop {
            if self.edges[e].is_central() {
                count += 1;
            }
            let t = self.edges[e].twin;
            if t == NIL {
                return false;
            }
            e = self.edges[t].next;
            if e == NIL || e == first {
                break;
            }
        }
        count > 2
    }

    fn update_bead_count(&mut self) {
        for e in 0..self.edges.len() {
            if !self.edges[e].dead && self.edges[e].is_central() {
                let to = self.edges[e].to;
                self.nodes[to].bead_count = self.strategy.optimal_bead_count(self.r(to) * 2);
            }
        }
        // Local maxima always get a count, central or not (a cone tip's last
        // dot must exist even if its edges are all "corner-like").
        for n in 0..self.nodes.len() {
            if !self.nodes[n].dead && self.r(n) > 0 && self.is_local_maximum(n, false) {
                self.nodes[n].bead_count = self.strategy.optimal_bead_count(self.r(n) * 2);
            }
        }
    }

    /// Where a central region ends and a same-count central region lies just
    /// beyond a short non-central gap, mark the gap central too (kills
    /// transition churn through tiny waists).
    fn filter_noncentral_regions(&mut self) {
        for e in 0..self.edges.len() {
            if self.edges[e].dead || !self.is_end_of_central(e) {
                continue;
            }
            let to = self.edges[e].to;
            if self.nodes[to].bead_count < 0 && self.r(to) != 0 {
                continue;
            }
            let count = self.nodes[to].bead_count;
            self.filter_noncentral_walk(e, count, 0, 400);
        }
    }

    fn filter_noncentral_walk(&mut self, to_edge: usize, bead_count: i32, traveled: i64, max_dist: i64) -> bool {
        let r = self.r(self.edges[to_edge].to);
        let twin = self.edges[to_edge].twin;
        let mut next_edge = self.edges[to_edge].next;
        while next_edge != NIL && next_edge != twin {
            let to = self.edges[next_edge].to;
            if self.r(to) >= r || self.len(next_edge) < 10 {
                break; // only walk upward
            }
            next_edge = self.twin_next(next_edge);
        }
        if next_edge == twin || next_edge == NIL {
            return false;
        }
        let length = self.len(next_edge);
        let to = self.edges[next_edge].to;
        let dissolve = if self.nodes[to].bead_count == bead_count {
            true
        } else if self.nodes[to].bead_count < 0 {
            self.filter_noncentral_walk(next_edge, bead_count, traveled + length, max_dist)
        } else {
            traveled + length < max_dist && (self.nodes[to].bead_count - bead_count).abs() == 1
        };
        if dissolve {
            self.edges[next_edge].central = 1;
            let t = self.edges[next_edge].twin;
            if t != NIL {
                self.edges[t].central = 1;
            }
            self.nodes[to].bead_count = self.strategy.optimal_bead_count(self.r(to) * 2);
            self.nodes[to].transition_ratio = 0.0;
        }
        dissolve
    }

    fn generate_transitioning_ribs(&mut self) {
        self.generate_transition_mids();
        self.filter_transition_mids();
        let ends = self.generate_all_transition_ends();
        self.apply_transitions(ends);
    }

    /// One transition mid per bead-count step on each upward central edge, at
    /// the radius where the scheme's count changes.
    fn generate_transition_mids(&mut self) {
        for e in 0..self.edges.len() {
            if self.edges[e].dead || !self.edges[e].is_central() {
                continue;
            }
            let (f, t) = (self.edges[e].from, self.edges[e].to);
            let (start_r, end_r) = (self.r(f), self.r(t));
            let (start_count, end_count) = (self.nodes[f].bead_count, self.nodes[t].bead_count);
            if start_r == end_r || start_r > end_r || start_count == end_count {
                continue;
            }
            let edge_size = self.len(e);
            for lower in start_count..end_count {
                let mut mid_r = self.strategy.transition_thickness(lower) / 2;
                mid_r = mid_r.clamp(start_r, end_r);
                let mid_pos = edge_size * (mid_r - start_r) / (end_r - start_r);
                self.edges[e].transitions.push(TransMid { pos: mid_pos, lower_count: lower, feature_radius: mid_r });
            }
        }
    }

    /// Opposing transitions closer together than the filter distance (and
    /// within the allowed width deviation) cancel; transitions hanging just
    /// past the end of a central region dissolve into it.
    fn filter_transition_mids(&mut self) {
        for e in 0..self.edges.len() {
            if self.edges[e].dead || self.edges[e].transitions.is_empty() {
                continue;
            }
            let ab_size = self.len(e);
            let back = *self.edges[e].transitions.last().unwrap();

            let to_dissolve_back = self.dissolve_nearby_transitions(e, back, ab_size - back.pos, TRANSITION_FILTER_DIST, true);
            let mut should_dissolve_back = !to_dissolve_back.is_empty();
            for &(de, dcount) in &to_dissolve_back {
                self.dissolve_bead_count_region(e, back.lower_count + 1, back.lower_count);
                self.edges[de].transitions.retain(|t| t.lower_count != dcount);
            }
            {
                let n = back.lower_count;
                let upper_half = ((1.0 - self.strategy.anchor_pos(n)) * self.strategy.transition_length(n) as f64) as i64;
                should_dissolve_back |= self.filter_end_of_central_transition(e, ab_size - back.pos, upper_half, n);
            }
            if should_dissolve_back {
                self.edges[e].transitions.pop();
            }
            if self.edges[e].transitions.is_empty() {
                continue;
            }

            let front = self.edges[e].transitions[0];
            let twin = self.edges[e].twin;
            let to_dissolve_front = self.dissolve_nearby_transitions(twin, front, front.pos, TRANSITION_FILTER_DIST, false);
            let mut should_dissolve_front = !to_dissolve_front.is_empty();
            for &(de, dcount) in &to_dissolve_front {
                self.dissolve_bead_count_region(twin, front.lower_count, front.lower_count + 1);
                self.edges[de].transitions.retain(|t| t.lower_count != dcount);
            }
            {
                let n = front.lower_count;
                let lower_half = (self.strategy.anchor_pos(n) * self.strategy.transition_length(n) as f64) as i64;
                should_dissolve_front |= self.filter_end_of_central_transition(twin, front.pos, lower_half, n + 1);
            }
            if should_dissolve_front {
                self.edges[e].transitions.remove(0);
            }
        }
    }

    /// Find transitions of the same count step within `max_dist` of the
    /// origin transition, in all directions except backwards. Returns the
    /// (edge, lower_count) refs to erase, or empty when dissolving is wrong
    /// (region too long, or the width deviation would show).
    fn dissolve_nearby_transitions(
        &self,
        edge_to_start: usize,
        origin: TransMid,
        traveled: i64,
        max_dist: i64,
        going_up: bool,
    ) -> Vec<(usize, i32)> {
        let mut out: Vec<(usize, i32)> = Vec::new();
        if traveled > max_dist {
            return out;
        }
        let mut should_dissolve = true;
        let twin = self.edges[edge_to_start].twin;
        let mut edge = self.edges[edge_to_start].next;
        while edge != NIL && edge != twin {
            if !self.edges[edge].is_central() {
                edge = self.twin_next(edge);
                continue;
            }
            let ab_size = self.len(edge);
            let is_aligned = self.is_upward(edge);
            let aligned_edge = if is_aligned { edge } else { self.edges[edge].twin };
            let mut seen_transition_on_this_edge = false;

            // The deviation this dissolution would smear into bead widths.
            let radius_here = self.r(self.edges[edge].from);
            let result_is_odd = (origin.lower_count % 2 == 1) == going_up;
            let width_dev = (origin.feature_radius - radius_here).abs() * 2;
            let line_dev = if result_is_odd { width_dev } else { width_dev / 2 };
            if line_dev as f64 > self.strategy.lw / 4.0 {
                should_dissolve = false;
            }

            if should_dissolve {
                for t in &self.edges[aligned_edge].transitions {
                    let pos = if is_aligned { t.pos } else { ab_size - t.pos };
                    if traveled + pos < max_dist && t.lower_count == origin.lower_count {
                        out.push((aligned_edge, t.lower_count));
                        seen_transition_on_this_edge = true;
                    }
                }
            }
            if should_dissolve && !seen_transition_on_this_edge {
                let here = self.dissolve_nearby_transitions(edge, origin, traveled + ab_size, max_dist, going_up);
                if here.is_empty() {
                    return Vec::new(); // too long in this direction: never dissolve
                }
                out.extend(here);
            }
            edge = self.twin_next(edge);
        }
        if !should_dissolve {
            out.clear();
        }
        out
    }

    fn dissolve_bead_count_region(&mut self, edge_to_start: usize, from_count: i32, to_count: i32) {
        if from_count == to_count {
            return;
        }
        let to = self.edges[edge_to_start].to;
        if self.nodes[to].bead_count != from_count {
            return;
        }
        self.nodes[to].bead_count = to_count;
        let twin = self.edges[edge_to_start].twin;
        let mut edge = self.edges[edge_to_start].next;
        while edge != NIL && edge != twin {
            if self.edges[edge].is_central() {
                self.dissolve_bead_count_region(edge, from_count, to_count);
            }
            edge = self.twin_next(edge);
        }
    }

    fn filter_end_of_central_transition(&mut self, edge_to_start: usize, traveled: i64, max_dist: i64, replacing_count: i32) -> bool {
        if traveled > max_dist {
            return false;
        }
        let mut is_end = true;
        let mut should_dissolve = false;
        let twin = self.edges[edge_to_start].twin;
        let mut nx = self.edges[edge_to_start].next;
        while nx != NIL && nx != twin {
            if self.edges[nx].is_central() {
                let length = self.len(nx);
                should_dissolve |= self.filter_end_of_central_transition(nx, traveled + length, max_dist, replacing_count);
                is_end = false;
            }
            nx = self.twin_next(nx);
        }
        if is_end && traveled < max_dist {
            should_dissolve = true;
        }
        if should_dissolve {
            let to = self.edges[edge_to_start].to;
            self.nodes[to].bead_count = replacing_count;
        }
        should_dissolve
    }

    fn generate_all_transition_ends(&mut self) -> bool {
        let mut any = false;
        for e in 0..self.edges.len() {
            if self.edges[e].dead || self.edges[e].transitions.is_empty() {
                continue;
            }
            let mids = self.edges[e].transitions.clone();
            for mid in mids {
                any = true;
                self.generate_transition_ends(e, mid.pos, mid.lower_count);
            }
        }
        any
    }

    fn generate_transition_ends(&mut self, e: usize, mid_pos: i64, lower_count: i32) {
        let ab_size = self.len(e);
        let transition_length = self.strategy.transition_length(lower_count);
        let anchor = self.strategy.anchor_pos(lower_count);
        let mid_rest = anchor;
        {
            // Lower end (walking down from the mid).
            let start_pos = ab_size - mid_pos;
            let half = (anchor * transition_length as f64) as i64;
            let end_pos = start_pos + half;
            let twin = self.edges[e].twin;
            self.generate_transition_end(twin, start_pos, end_pos, half, mid_rest, 0.0, lower_count, 0);
        }
        {
            // Upper end.
            let half = ((1.0 - anchor) * transition_length as f64) as i64;
            let end_pos = mid_pos + half;
            self.generate_transition_end(e, mid_pos, end_pos, half, mid_rest, 1.0, lower_count, 0);
        }
    }

    /// Walk along central edges placing the transition's endpoint; recursing
    /// across nodes sets interpolation rests on the nodes passed over.
    /// Returns whether this branch was going down.
    #[allow(clippy::too_many_arguments)]
    fn generate_transition_end(
        &mut self,
        e: usize,
        start_pos: i64,
        end_pos: i64,
        half_len: i64,
        start_rest: f64,
        end_rest: f64,
        lower_count: i32,
        depth: usize,
    ) -> bool {
        if depth > 128 || !self.edges[e].is_central() {
            return false;
        }
        let ab_size = self.len(e);
        let going_up = end_rest > start_rest;

        if end_pos > ab_size {
            // The end lies past this edge: recurse on all further central
            // edges, leaving an interpolation rest on the node we cross.
            let rest = if start_pos == end_pos {
                end_rest
            } else {
                let r = end_rest - (start_rest - end_rest) * (end_pos - ab_size) as f64 / (start_pos - end_pos) as f64;
                r.clamp(start_rest.min(end_rest), start_rest.max(end_rest))
            };
            let twin = self.edges[e].twin;
            let mut central_count = 0;
            let mut out = self.edges[e].next;
            while out != NIL && out != twin {
                if self.edges[out].is_central() {
                    central_count += 1;
                }
                out = self.twin_next(out);
            }
            let mut is_only_going_down = true;
            let mut has_recursed = false;
            let mut out = self.edges[e].next;
            while out != NIL && out != twin {
                let next = self.twin_next(out);
                if !self.edges[out].is_central() {
                    out = next;
                    continue;
                }
                if central_count > 1 && going_up && self.is_going_down(out, 0, end_pos - ab_size + half_len, lower_count, 0) {
                    // Past a 3-way all-central junction, don't put an end
                    // down the branch that heads to lower bead counts.
                    out = next;
                    continue;
                }
                let is_going_down =
                    self.generate_transition_end(out, 0, end_pos - ab_size, half_len, rest, end_rest, lower_count, depth + 1);
                is_only_going_down &= is_going_down;
                out = next;
                has_recursed = true;
            }
            if !going_up || (has_recursed && !is_only_going_down) {
                let to = self.edges[e].to;
                self.nodes[to].transition_ratio = rest;
                self.nodes[to].bead_count = lower_count;
            }
            is_only_going_down
        } else {
            // The end lands on this edge.
            let is_lower_end = end_rest == 0.0;
            let (upward_edge, pos) = if self.is_upward(e) { (e, end_pos) } else { (self.edges[e].twin, ab_size - end_pos) };
            let end = TransEnd { pos, lower_count, is_lower_end };
            if self.edges[upward_edge].transition_ends.first().map_or(true, |f| pos < f.pos) {
                self.edges[upward_edge].transition_ends.insert(0, end);
            } else {
                self.edges[upward_edge].transition_ends.push(end);
            }
            false
        }
    }

    fn is_going_down(&self, outgoing: usize, traveled: i64, max_dist: i64, lower_count: i32, depth: usize) -> bool {
        if depth > 128 {
            return false;
        }
        let to = self.edges[outgoing].to;
        if self.r(to) == 0 {
            return true;
        }
        let from = self.edges[outgoing].from;
        let is_upward = self.r(to) >= self.r(from);
        let upward_edge = if is_upward { outgoing } else { self.edges[outgoing].twin };
        if self.nodes[to].bead_count > lower_count + 1 {
            return false;
        }
        let length = self.len(outgoing);
        if !self.edges[upward_edge].transitions.is_empty() {
            let mid = if is_upward {
                self.edges[upward_edge].transitions[0]
            } else {
                *self.edges[upward_edge].transitions.last().unwrap()
            };
            if mid.lower_count == lower_count
                && ((is_upward && mid.pos + traveled < max_dist) || (!is_upward && length - mid.pos + traveled < max_dist))
            {
                return true;
            }
        }
        if traveled + length > max_dist {
            return false;
        }
        if self.nodes[to].bead_count <= lower_count
            && !(self.nodes[to].bead_count == lower_count && self.nodes[to].transition_ratio > 0.0)
        {
            return true;
        }
        let twin = self.edges[outgoing].twin;
        let mut is_only_going_down = true;
        let mut has_recursed = false;
        let mut nx = self.edges[outgoing].next;
        while nx != NIL && nx != twin {
            if self.edges[nx].is_central() {
                is_only_going_down &= self.is_going_down(nx, traveled + length, max_dist, lower_count, depth + 1);
                has_recursed = true;
            }
            nx = self.twin_next(nx);
        }
        has_recursed && is_only_going_down
    }

    /// Materialize transition ends as graph nodes (with ribs), so bead counts
    /// change exactly at them.
    fn apply_transitions(&mut self, any_ends: bool) {
        // Ends were stored on upward halves; collect each pair's list onto
        // one side with positions relative to that side.
        for e in 0..self.edges.len() {
            if self.edges[e].dead {
                continue;
            }
            let twin = self.edges[e].twin;
            if twin == NIL || self.edges[twin].transition_ends.is_empty() {
                continue;
            }
            let length = self.len(e);
            let moved: Vec<TransEnd> = self.edges[twin]
                .transition_ends
                .drain(..)
                .map(|t| TransEnd { pos: length - t.pos, lower_count: t.lower_count, is_lower_end: t.is_lower_end })
                .collect();
            self.edges[e].transition_ends.extend(moved);
        }
        if !any_ends {
            return;
        }
        for e in 0..self.edges.len() {
            if self.edges[e].dead || self.edges[e].transition_ends.is_empty() {
                continue;
            }
            let mut ends = std::mem::take(&mut self.edges[e].transition_ends);
            ends.sort_by_key(|t| t.pos);
            let from = self.edges[e].from;
            let to = self.edges[e].to;
            let a = self.nodes[from].p;
            let b = self.nodes[to].p;
            let ab = b.sub(a);
            let ab_size = self.len(e);
            let mut last_edge_replacing_input = e;
            for end in ends {
                let new_count = if end.is_lower_end { end.lower_count } else { end.lower_count + 1 };
                let end_pos = end.pos;
                let close_node = if end_pos < ab_size / 2 { from } else { to };
                if (end_pos < SNAP_DIST || end_pos > ab_size - SNAP_DIST) && self.nodes[close_node].bead_count == new_count {
                    self.nodes[close_node].transition_ratio = 0.0;
                    continue;
                }
                let mid = a.add(normal(ab, end_pos as f64));
                last_edge_replacing_input = self.insert_node(last_edge_replacing_input, mid, new_count);
            }
        }
    }

    /// Extra ribs where bead positions kink within a constant count (our
    /// scheme: the saturation threshold), so the kink lands on a node.
    fn generate_extra_ribs(&mut self) {
        for e in 0..self.edges.len() {
            if self.edges[e].dead || !self.edges[e].is_central() {
                continue;
            }
            let (from, to) = (self.edges[e].from, self.edges[e].to);
            if self.len(e) < DISCRETIZATION_STEP || self.r(from) >= self.r(to) {
                continue;
            }
            let rib_thicknesses = self.strategy.nonlinear_thicknesses(self.nodes[from].bead_count);
            if rib_thicknesses.is_empty() {
                continue;
            }
            let a = self.nodes[from].p;
            let b = self.nodes[to].p;
            let ab = b.sub(a);
            let ab_size = self.len(e);
            let a_r = self.r(from);
            let b_r = self.r(to);
            let mut last_edge_replacing_input = e;
            for rib in rib_thicknesses {
                if rib / 2 <= a_r {
                    continue;
                }
                if rib / 2 >= b_r {
                    break;
                }
                let new_count = self.nodes[from].bead_count.min(self.nodes[to].bead_count);
                let end_pos = ab_size * (rib / 2 - a_r) / (b_r - a_r);
                let close_node = if end_pos < ab_size / 2 { from } else { to };
                if (end_pos < SNAP_DIST || end_pos > ab_size - SNAP_DIST) && self.nodes[close_node].bead_count == new_count {
                    self.nodes[close_node].transition_ratio = 0.0;
                    continue;
                }
                let mid = a.add(normal(ab, end_pos as f64));
                last_edge_replacing_input = self.insert_node(last_edge_replacing_input, mid, new_count);
            }
        }
    }

    /// Split `e` at `mid` (and its twin to match), ribbing the new node to
    /// the boundary on both sides. Returns the last edge replacing `e`.
    fn insert_node(&mut self, e: usize, mid: P, mid_node_bead_count: i32) -> usize {
        let mid_node = self.new_node(mid);
        let twin = self.edges[e].twin;
        self.edges[e].twin = NIL;
        self.edges[twin].twin = NIL;
        let (left_first, left_second) = self.insert_rib(e, mid_node);
        let (right_first, right_second) = self.insert_rib(twin, mid_node);
        self.edges[left_first].twin = right_second;
        self.edges[right_second].twin = left_first;
        self.edges[left_second].twin = right_first;
        self.edges[right_first].twin = left_second;
        self.nodes[mid_node].bead_count = mid_node_bead_count;
        left_second
    }

    fn insert_rib(&mut self, e: usize, mid_node: usize) -> (usize, usize) {
        let edge_before = self.edges[e].prev;
        let edge_after = self.edges[e].next;
        let node_before = self.edges[e].from;
        let node_after = self.edges[e].to;
        let p = self.nodes[mid_node].p;
        let (src_a, src_b) = self.get_source(e);
        let px = closest_on_segment(p, src_a, src_b);
        let dist = p.dist(px).round() as i64;
        self.nodes[mid_node].r = dist;
        self.nodes[mid_node].transition_ratio = 0.0;

        let source_node = self.new_node(px);
        self.nodes[source_node].r = 0;

        let first = e;
        let second = self.new_edge(EType::Normal);
        let outward = self.new_edge(EType::TransitionEnd);
        let inward = self.new_edge(EType::TransitionEnd);

        if edge_before != NIL {
            self.edges[edge_before].next = first;
        }
        self.edges[first].next = outward;
        self.edges[outward].next = NIL;
        self.edges[inward].next = second;
        self.edges[second].next = edge_after;

        if edge_after != NIL {
            self.edges[edge_after].prev = second;
        }
        self.edges[second].prev = inward;
        self.edges[inward].prev = NIL;
        self.edges[outward].prev = first;
        self.edges[first].prev = edge_before;

        self.edges[first].to = mid_node;
        self.edges[outward].to = source_node;
        self.edges[inward].to = mid_node;
        self.edges[second].to = node_after;
        self.edges[first].from = node_before;
        self.edges[outward].from = mid_node;
        self.edges[inward].from = source_node;
        self.edges[second].from = mid_node;

        self.nodes[node_before].incident = first;
        self.nodes[mid_node].incident = outward;
        self.nodes[source_node].incident = inward;
        if edge_after != NIL {
            self.nodes[node_after].incident = edge_after;
        }

        self.edges[first].central = 1;
        self.edges[second].central = 1;
        self.edges[outward].central = 0;
        self.edges[inward].central = 0;

        self.edges[outward].twin = inward;
        self.edges[inward].twin = outward;
        self.edges[first].twin = NIL; // set by insert_node
        self.edges[second].twin = NIL;
        (first, second)
    }

    /// The boundary source span of the quad containing `e` (quad head's from
    /// and tail's to are boundary points).
    fn get_source(&self, e: usize) -> (P, P) {
        let mut from_edge = e;
        while self.edges[from_edge].prev != NIL {
            from_edge = self.edges[from_edge].prev;
        }
        let mut to_edge = e;
        while self.edges[to_edge].next != NIL {
            to_edge = self.edges[to_edge].next;
        }
        (self.nodes[self.edges[from_edge].from].p, self.nodes[self.edges[to_edge].to].p)
    }

    // =======================================================================
    // Toolpath extraction (Cura generateSegments .. generateLocalMaxima)
    // =======================================================================

    fn generate_toolpaths(&mut self, lw: f64, sp: f64) -> Vec<Bead> {
        self.update_is_central();
        self.update_bead_count();
        self.filter_noncentral_regions();
        self.generate_transitioning_ribs();
        self.generate_extra_ribs();
        if std::env::var("ARACHNE_DBG").is_ok() {
            self.dbg_dump();
        }
        self.generate_segments();
        self.assemble_beads(lw, sp)
    }

    fn dbg_dump(&self) {
        let live = self.edges.iter().filter(|e| !e.dead).count();
        let central = self.edges.iter().filter(|e| !e.dead && e.is_central()).count();
        let mids: usize = self.edges.iter().filter(|e| !e.dead).map(|e| e.transitions.len()).sum();
        let counted = self.nodes.iter().filter(|n| !n.dead && n.bead_count >= 0).count();
        let ratio_nodes = self.nodes.iter().filter(|n| !n.dead && n.transition_ratio > 0.0).count();
        let maxima = (0..self.nodes.len())
            .filter(|&n| !self.nodes[n].dead && self.r(n) > 0 && self.is_local_maximum(n, true))
            .count();
        eprintln!(
            "  skeletal: {live} edges ({central} central), {mids} transition mids, {counted} counted nodes ({ratio_nodes} in-ratio), {maxima} strict maxima"
        );
        // Central chain breaks: end-of-central edges and their positions.
        for e in 0..self.edges.len() {
            if self.edges[e].dead || !self.is_end_of_central(e) {
                continue;
            }
            let to = self.edges[e].to;
            eprintln!(
                "    end-of-central at ({:.2},{:.2}) r={} count={}",
                self.nodes[to].p.x as f64 / 1000.0,
                self.nodes[to].p.y as f64 / 1000.0,
                self.r(to),
                self.nodes[to].bead_count
            );
            // The non-central edges hanging off this end: their slopes.
            let twin = self.edges[e].twin;
            let mut nx = self.edges[e].next;
            while nx != NIL && nx != twin {
                if !self.edges[nx].is_central() && self.edges[nx].etype == EType::Normal {
                    let (f, t) = (self.edges[nx].from, self.edges[nx].to);
                    let d_r = (self.r(t) - self.r(f)).abs();
                    let d_d = self.len(nx).max(1);
                    eprintln!(
                        "      noncentral edge len={} dR={} slope={:.3} r {}->{}",
                        d_d,
                        d_r,
                        d_r as f64 / d_d as f64,
                        self.r(f),
                        self.r(t)
                    );
                }
                nx = self.twin_next(nx);
            }
        }
    }

    fn generate_segments(&mut self) {
        let mut upward_quad_mids: Vec<usize> = Vec::new();
        for e in 0..self.edges.len() {
            if !self.edges[e].dead && self.edges[e].prev != NIL && self.edges[e].next != NIL && self.is_upward(e) {
                upward_quad_mids.push(e);
            }
        }
        upward_quad_mids.sort_by(|&a, &b| {
            let (ra, rb) = (self.r(self.edges[a].to), self.r(self.edges[b].to));
            if ra == rb {
                // Flat edges chained to one another need a consistent order.
                let a_flat = self.r(self.edges[a].from) == ra;
                let b_flat = self.r(self.edges[b].from) == rb;
                if a_flat && b_flat {
                    let big = i64::MAX;
                    let a_dist = self
                        .dist_to_go_up(a, &mut Vec::new())
                        .unwrap_or(big)
                        .min(self.dist_to_go_up(self.edges[a].twin, &mut Vec::new()).unwrap_or(big))
                        - self.len(a);
                    let b_dist = self
                        .dist_to_go_up(b, &mut Vec::new())
                        .unwrap_or(big)
                        .min(self.dist_to_go_up(self.edges[b].twin, &mut Vec::new()).unwrap_or(big))
                        - self.len(b);
                    return a_dist.cmp(&b_dist);
                }
                if a_flat {
                    return std::cmp::Ordering::Less;
                }
                if b_flat {
                    return std::cmp::Ordering::Greater;
                }
            }
            rb.cmp(&ra) // higher R first
        });

        // Beadings at nodes that have a count of their own.
        for n in 0..self.nodes.len() {
            if self.nodes[n].dead || self.nodes[n].bead_count <= 0 {
                continue;
            }
            let bp = if self.nodes[n].transition_ratio == 0.0 {
                BeadingProp {
                    beading: self.strategy.compute(self.r(n) * 2, self.nodes[n].bead_count),
                    dist_to_bottom: 0,
                    dist_from_top: 0,
                    upward_only: false,
                }
            } else {
                let low = self.strategy.compute(self.r(n) * 2, self.nodes[n].bead_count);
                let high = self.strategy.compute(self.r(n) * 2, self.nodes[n].bead_count + 1);
                BeadingProp {
                    beading: interpolate(&low, 1.0 - self.nodes[n].transition_ratio, &high),
                    dist_to_bottom: 0,
                    dist_from_top: 0,
                    upward_only: false,
                }
            };
            self.beadings.push(bp);
            self.nodes[n].beading = self.beadings.len() - 1;
        }

        self.propagate_beadings_upward(&upward_quad_mids);
        self.propagate_beadings_downward(&upward_quad_mids);
        self.generate_junctions();
        self.connect_junctions();
        self.generate_local_maxima_single_beads();
    }

    fn propagate_beadings_upward(&mut self, upward_quad_mids: &[usize]) {
        for &e in upward_quad_mids.iter().rev() {
            let (from, to) = (self.edges[e].from, self.edges[e].to);
            if self.nodes[to].bead_count >= 0 {
                continue; // don't override local beading
            }
            if self.nodes[from].beading == NIL || self.nodes[to].beading != NIL {
                continue;
            }
            let length = self.len(e);
            let mut upper = self.beadings[self.nodes[from].beading].clone();
            upper.dist_to_bottom += length;
            upper.upward_only = true;
            self.beadings.push(upper);
            self.nodes[to].beading = self.beadings.len() - 1;
        }
    }

    fn propagate_beadings_downward(&mut self, upward_quad_mids: &[usize]) {
        for &e in upward_quad_mids {
            if self.edges[e].is_central() {
                continue;
            }
            // For equidistant edges, propagate from the side that has beading.
            let (from, to) = (self.edges[e].from, self.edges[e].to);
            if self.r(from) == self.r(to) && self.nodes[from].beading != NIL && self.nodes[to].beading == NIL {
                self.propagate_downward_step(self.edges[e].twin);
            } else {
                self.propagate_downward_step(e);
            }
        }
    }

    fn propagate_downward_step(&mut self, edge_to_peak: usize) {
        let length = self.len(edge_to_peak);
        let top_idx = self.get_or_create_beading(self.edges[edge_to_peak].to);
        let top = self.beadings[top_idx].clone();
        let from = self.edges[edge_to_peak].from;
        if self.nodes[from].beading == NIL {
            let mut propagated = top;
            propagated.dist_from_top += length;
            self.beadings.push(propagated);
            self.nodes[from].beading = self.beadings.len() - 1;
        } else {
            let bottom_idx = self.nodes[from].beading;
            let bottom = self.beadings[bottom_idx].clone();
            let total_dist = top.dist_from_top + length + bottom.dist_to_bottom;
            let prop_dist = self.strategy.lw as i64; // beading propagation ramp
            let ratio_of_top = (bottom.dist_to_bottom as f64 / total_dist.min(prop_dist) as f64).max(0.0);
            if ratio_of_top >= 1.0 {
                let mut nb = top;
                nb.dist_from_top += length;
                self.beadings[bottom_idx] = nb;
            } else {
                let merged = interpolate_switching(&top.beading, ratio_of_top, &bottom.beading, self.r(from));
                self.beadings[bottom_idx] =
                    BeadingProp { beading: merged, dist_to_bottom: 0, dist_from_top: 0, upward_only: false };
            }
        }
    }

    fn get_or_create_beading(&mut self, n: usize) -> usize {
        if self.nodes[n].beading != NIL {
            return self.nodes[n].beading;
        }
        if self.nodes[n].bead_count == -1 {
            if let Some(b) = self.nearest_beading(n, 100) {
                return b;
            }
            // Derive a count from the closest upward distance.
            let first = self.nodes[n].incident;
            let mut dist = i64::MAX;
            if first != NIL {
                let mut e = first;
                let mut iter = 0;
                loop {
                    let to = self.edges[e].to;
                    if self.r(to) >= 0 {
                        dist = dist.min(self.r(to) + self.len(e));
                    }
                    let t = self.edges[e].twin;
                    if t == NIL {
                        break;
                    }
                    e = self.edges[t].next;
                    iter += 1;
                    if e == NIL || e == first || iter > 1000 {
                        break;
                    }
                }
            }
            if dist == i64::MAX {
                dist = self.r(n).max(0);
            }
            self.nodes[n].bead_count = self.strategy.optimal_bead_count(dist * 2);
        }
        let bp = BeadingProp {
            beading: self.strategy.compute(self.r(n) * 2, self.nodes[n].bead_count),
            dist_to_bottom: 0,
            dist_from_top: 0,
            upward_only: false,
        };
        self.beadings.push(bp);
        self.nodes[n].beading = self.beadings.len() - 1;
        self.nodes[n].beading
    }

    fn nearest_beading(&self, n: usize, max_dist: i64) -> Option<usize> {
        use std::cmp::Reverse;
        use std::collections::BinaryHeap;
        let mut heap: BinaryHeap<Reverse<(i64, usize)>> = BinaryHeap::new();
        let first = self.nodes[n].incident;
        if first == NIL {
            return None;
        }
        let mut e = first;
        loop {
            heap.push(Reverse((self.len(e), e)));
            let t = self.edges[e].twin;
            if t == NIL {
                break;
            }
            e = self.edges[t].next;
            if e == NIL || e == first {
                break;
            }
        }
        for _ in 0..1000 {
            let Reverse((dist, here)) = heap.pop()?;
            if dist > max_dist {
                return None;
            }
            let to = self.edges[here].to;
            if self.nodes[to].beading != NIL {
                return Some(self.nodes[to].beading);
            }
            let twin = self.edges[here].twin;
            let mut further = self.edges[here].next;
            while further != NIL && further != twin {
                heap.push(Reverse((dist + self.len(further), further)));
                further = self.twin_next(further);
            }
        }
        None
    }

    /// Junctions: where bead centerlines cross the graph edges. Per upward
    /// edge, the beading evaluated at its top node decides the crossing radii.
    fn generate_junctions(&mut self) {
        for e in 0..self.edges.len() {
            if self.edges[e].dead {
                continue;
            }
            let (from, to) = (self.edges[e].from, self.edges[e].to);
            if self.r(from) > self.r(to) {
                continue; // only the upward halves carry junctions
            }
            let start_r = self.r(to);
            let end_r = self.r(from);
            if (self.nodes[from].bead_count == self.nodes[to].bead_count && self.nodes[from].bead_count >= 0)
                || end_r >= start_r
            {
                continue;
            }
            let bidx = self.get_or_create_beading(to);
            let beading = self.beadings[bidx].beading.clone();
            let a = self.nodes[to].p;
            let b = self.nodes[from].p;
            let ab = b.sub(a);

            let num = beading.locations.len();
            if num == 0 {
                continue;
            }
            let mut junctions: Vec<Junction> = Vec::new();
            // Start at the central-most location that exists at the top node.
            let mut ji = (num.max(1) - 1) / 2;
            loop {
                if beading.locations[ji] <= start_r as f64 + 1.0 {
                    break;
                }
                if ji == 0 {
                    ji = usize::MAX;
                    break;
                }
                ji -= 1;
            }
            if ji != usize::MAX
                && ji + 1 < num
                && beading.locations[ji + 1] <= start_r as f64 + 5.0
                && (beading.total as f64) < start_r as f64 + 5.0
            {
                ji += 1;
            }
            while ji != usize::MAX {
                let bead_r = beading.locations[ji];
                if bead_r < end_r as f64 {
                    break; // junctions coinciding with the lower node belong to the next edge
                }
                let frac = if start_r == end_r { 0.0 } else { (bead_r - start_r as f64) / (end_r - start_r) as f64 };
                let mut junction = P {
                    x: a.x + (ab.x as f64 * frac).round() as i64,
                    y: a.y + (ab.y as f64 * frac).round() as i64,
                };
                if bead_r > start_r as f64 - 5.0 {
                    junction = a; // snap to the node: robust 3-way detection
                }
                junctions.push(Junction { p: junction, w: beading.widths[ji], idx: ji });
                if ji == 0 {
                    break;
                }
                ji -= 1;
            }
            self.edges[e].junctions = junctions;
        }
    }

    fn get_quad_max_r_edge_to(&self, quad_start: usize) -> usize {
        let mut max_r = -1;
        let mut ret = NIL;
        let mut e = quad_start;
        while e != NIL {
            let r = self.r(self.edges[e].to);
            if r > max_r {
                max_r = r;
                ret = e;
            }
            e = self.edges[e].next;
        }
        if ret != NIL && self.edges[ret].next == NIL && self.r(self.edges[ret].to) - 5 < self.r(self.edges[ret].from) {
            ret = self.edges[ret].prev;
        }
        ret
    }

    fn get_next_unconnected(&self, e: usize) -> usize {
        let mut result = e;
        while self.edges[result].next != NIL {
            result = self.edges[result].next;
            if result == e {
                return NIL;
            }
        }
        self.edges[result].twin
    }

    /// Walk each quad of each polygon domain, pairing the descending junction
    /// ladders of its two sides into toolpath segments.
    fn connect_junctions(&mut self) {
        let n_edges = self.edges.len();
        let mut unprocessed = vec![false; n_edges];
        let mut remaining = 0usize;
        for e in 0..n_edges {
            if !self.edges[e].dead && self.edges[e].prev == NIL {
                unprocessed[e] = true;
                remaining += 1;
            }
        }
        let mut passed_odd_edges = vec![false; n_edges];
        let mut scan = 0usize;
        while remaining > 0 {
            while scan < n_edges && !unprocessed[scan] {
                scan += 1;
            }
            if scan >= n_edges {
                break;
            }
            let poly_domain_start = scan;
            let mut quad_start = poly_domain_start;
            let mut new_domain_start = true;
            let mut guard = 0;
            loop {
                let mut quad_end = quad_start;
                while self.edges[quad_end].next != NIL {
                    quad_end = self.edges[quad_end].next;
                }
                let edge_to_peak = self.get_quad_max_r_edge_to(quad_start);
                if edge_to_peak == NIL {
                    break;
                }
                let edge_from_peak = self.edges[edge_to_peak].next;
                if unprocessed[quad_start] {
                    unprocessed[quad_start] = false;
                    remaining -= 1;
                }

                let mut from_junctions = self.edges[edge_to_peak].junctions.clone();
                let mut to_junctions = if edge_from_peak != NIL {
                    let t = self.edges[edge_from_peak].twin;
                    if t != NIL {
                        self.edges[t].junctions.clone()
                    } else {
                        Vec::new()
                    }
                } else {
                    Vec::new()
                };
                if self.edges[edge_to_peak].prev != NIL {
                    let from_prev = self.edges[self.edges[edge_to_peak].prev].junctions.clone();
                    while !from_junctions.is_empty()
                        && !from_prev.is_empty()
                        && from_junctions.last().unwrap().idx <= from_prev[0].idx
                    {
                        from_junctions.pop();
                    }
                    from_junctions.extend(from_prev);
                }
                if edge_from_peak != NIL && self.edges[edge_from_peak].next != NIL {
                    let nt = self.edges[self.edges[edge_from_peak].next].twin;
                    let to_next = if nt != NIL { self.edges[nt].junctions.clone() } else { Vec::new() };
                    while !to_junctions.is_empty() && !to_next.is_empty() && to_junctions.last().unwrap().idx <= to_next[0].idx {
                        to_junctions.pop();
                    }
                    to_junctions.extend(to_next);
                }

                let segment_count = from_junctions.len().min(to_junctions.len());
                let qs_to = self.edges[quad_start].to;
                let qe_from = self.edges[quad_end].from;
                for rev_idx in 0..segment_count {
                    let from = from_junctions[from_junctions.len() - 1 - rev_idx];
                    let to = to_junctions[to_junctions.len() - 1 - rev_idx];
                    let from_is_odd = self.nodes[qs_to].bead_count > 0
                        && self.nodes[qs_to].bead_count % 2 == 1
                        && self.nodes[qs_to].transition_ratio == 0.0
                        && rev_idx == segment_count - 1
                        && from.p.sub(self.nodes[qs_to].p).shorter_than(5);
                    let to_is_odd = self.nodes[qe_from].bead_count > 0
                        && self.nodes[qe_from].bead_count % 2 == 1
                        && self.nodes[qe_from].transition_ratio == 0.0
                        && rev_idx == segment_count - 1
                        && to.p.sub(self.nodes[qe_from].p).shorter_than(5);
                    let is_odd_segment = from_is_odd && to_is_odd;
                    let qs_next = self.edges[quad_start].next;
                    if is_odd_segment && qs_next != NIL && self.edges[qs_next].twin != NIL && passed_odd_edges[self.edges[qs_next].twin]
                    {
                        continue; // the twin quad already drew this center piece
                    }
                    let from_is_3way = from_is_odd && self.is_multi_intersection(qs_to);
                    let to_is_3way = to_is_odd && self.is_multi_intersection(qe_from);
                    if qs_next != NIL {
                        passed_odd_edges[qs_next] = true;
                    }
                    self.add_toolpath_segment(from, to, is_odd_segment, new_domain_start, from_is_3way, to_is_3way);
                }
                new_domain_start = false;
                quad_start = self.get_next_unconnected(quad_start);
                guard += 1;
                if quad_start == NIL || quad_start == poly_domain_start || guard > n_edges {
                    break;
                }
            }
        }
    }

    fn add_toolpath_segment(&mut self, from: Junction, to: Junction, is_odd: bool, force_new_path: bool, from_is_3way: bool, to_is_3way: bool) {
        if from.p == to.p && from.idx == to.idx {
            return;
        }
        let inset = from.idx;
        if inset >= self.bins.len() {
            self.bins.resize_with(inset + 1, Vec::new);
        }
        let mut force_new_path = force_new_path;
        let bin = &mut self.bins[inset];
        if bin.is_empty() || bin.last().unwrap().odd != is_odd {
            force_new_path = true;
        }
        if !force_new_path {
            let last = bin.last_mut().unwrap();
            let (lp, lw_) = (*last.pts.last().unwrap(), *last.ws.last().unwrap());
            if lp.sub(from.p).shorter_than(10) && (lw_ - from.w).abs() < 10.0 && !from_is_3way {
                last.pts.push(to.p);
                last.ws.push(to.w);
                return;
            }
            if lp.sub(to.p).shorter_than(10) && (lw_ - to.w).abs() < 10.0 && !to_is_3way {
                last.pts.push(from.p);
                last.ws.push(from.w);
                return;
            }
        }
        self.bins[inset].push(Acc { pts: vec![from.p, to.p], ws: vec![from.w, to.w], odd: is_odd });
    }

    /// Local maxima with an odd bead count need a small filler where the
    /// center bead degenerates to a point (e.g. the tip of a cone).
    fn generate_local_maxima_single_beads(&mut self) {
        let mut circles: Vec<(P, f64, usize)> = Vec::new();
        for n in 0..self.nodes.len() {
            if self.nodes[n].dead || self.nodes[n].beading == NIL {
                continue;
            }
            let beading = &self.beadings[self.nodes[n].beading].beading;
            if beading.widths.len() % 2 == 1 && self.is_local_maximum(n, true) {
                let inset = beading.widths.len() / 2;
                let w = beading.widths[inset];
                // Only nodes whose edges are all non-central get the dot; a
                // central local maximum is already covered by the odd bead.
                let mut any_central = false;
                let first = self.nodes[n].incident;
                if first != NIL {
                    let mut e = first;
                    loop {
                        if self.edges[e].is_central() {
                            any_central = true;
                        }
                        let t = self.edges[e].twin;
                        if t == NIL {
                            break;
                        }
                        e = self.edges[t].next;
                        if e == NIL || e == first {
                            break;
                        }
                    }
                }
                if !any_central {
                    circles.push((self.nodes[n].p, w, inset));
                }
            }
        }
        for (c, w, inset) in circles {
            if inset >= self.bins.len() {
                self.bins.resize_with(inset + 1, Vec::new);
            }
            // Extruding a circle of r = w/8 deposits the same volume as the
            // missing dot of diameter w.
            let r = w / 8.0;
            let mut pts = Vec::with_capacity(7);
            let mut ws = Vec::with_capacity(7);
            for k in 0..=6 {
                let ang = std::f64::consts::TAU * k as f64 / 6.0;
                pts.push(P { x: c.x + (r * ang.cos()).round() as i64, y: c.y + (r * ang.sin()).round() as i64 });
                ws.push(w);
            }
            self.bins[inset].push(Acc { pts, ws, odd: true });
        }
    }

    /// Bins → wall::Bead output (mm), stitched per bead index.
    fn assemble_beads(&mut self, lw: f64, sp: f64) -> Vec<Bead> {
        let mut out: Vec<Bead> = Vec::new();
        for bin in std::mem::take(&mut self.bins) {
            let mut beads: Vec<Bead> = Vec::new();
            for acc in bin {
                if acc.pts.len() < 2 {
                    continue;
                }
                let points: Vec<geo2d::Point> =
                    acc.pts.iter().map(|p| geo2d::Point::new(p.x * 1000, p.y * 1000)).collect();
                let widths: Vec<f64> = acc.ws.iter().map(|w| w / 1000.0).collect();
                beads.push(Bead { points, widths, closed: false });
            }
            // The quad walk emits per-domain chains; stitch them into rings /
            // long polylines (the graph guarantees matching endpoints).
            for b in join_beads(beads, lw * 0.8) {
                let len: f64 = b
                    .points
                    .windows(2)
                    .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
                    .sum();
                if b.closed || len >= sp * 1.5 {
                    out.push(b);
                }
            }
        }
        out
    }

    // =======================================================================
    // Thin-feature beads (regions thinner than one line width)
    // =======================================================================

    /// Single tapered beads along central chains where even the classic outer
    /// loop can't fit (R < lw/2) — the graph-walk replacement for the grid's
    /// thinned-skeleton ridge trace.
    fn thin_beads(mut self, lw: f64, sp: f64) -> Vec<Bead> {
        self.update_is_central();
        let lw_um = lw * 1000.0;
        let sp_um = sp * 1000.0;
        let half_lw = (lw_um * 0.5) as i64;
        // Undirected adjacency of qualifying central edges.
        let mut keep: Vec<(usize, usize)> = Vec::new(); // node pairs
        for e in 0..self.edges.len() {
            if self.edges[e].dead || !self.edges[e].is_central() {
                continue;
            }
            let (f, t) = (self.edges[e].from, self.edges[e].to);
            if f == t {
                continue;
            }
            if self.r(f) < half_lw && self.r(t) < half_lw && e < self.edges[e].twin {
                keep.push((f, t));
            }
        }
        if keep.is_empty() {
            return Vec::new();
        }
        let mut adj: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
        for &(a, b) in &keep {
            adj.entry(a).or_default().push(b);
            adj.entry(b).or_default().push(a);
        }
        let width_of = |r: i64| -> f64 { ((2 * r) as f64 + (lw_um - sp_um)).clamp(lw_um * 0.4, lw_um * 1.2) / 1000.0 };
        let mut visited: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut beads: Vec<Bead> = Vec::new();
        let mut nodes_sorted: Vec<usize> = adj.keys().copied().collect();
        nodes_sorted.sort_unstable();
        for &start in &nodes_sorted {
            if visited.contains(&start) {
                continue;
            }
            // BFS to the farthest end of this component, then greedy-walk.
            let mut blob = vec![start];
            visited.insert(start);
            let mut qi = 0;
            let mut far = start;
            while qi < blob.len() {
                let c = blob[qi];
                qi += 1;
                far = c;
                for &nb in &adj[&c] {
                    if visited.insert(nb) {
                        blob.push(nb);
                    }
                }
            }
            let mut walked: std::collections::HashSet<usize> = std::collections::HashSet::new();
            let mut line: Vec<usize> = Vec::new();
            let mut at = far;
            loop {
                walked.insert(at);
                line.push(at);
                match adj[&at].iter().copied().find(|nb| !walked.contains(nb)) {
                    Some(nb) => at = nb,
                    None => break,
                }
            }
            if line.len() < 2 {
                continue;
            }
            let points: Vec<geo2d::Point> =
                line.iter().map(|&n| geo2d::Point::new(self.nodes[n].p.x * 1000, self.nodes[n].p.y * 1000)).collect();
            let widths: Vec<f64> = line.iter().map(|&n| width_of(self.r(n))).collect();
            beads.push(Bead { points, widths, closed: false });
        }
        join_beads(beads, lw * 0.8)
            .into_iter()
            .filter(|b| {
                let len: f64 = b
                    .points
                    .windows(2)
                    .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
                    .sum();
                b.closed || len >= sp * 1.5
            })
            .collect()
    }
}

// ===========================================================================
// Beading interpolation (Cura SkeletalTrapezoidation::interpolate)
// ===========================================================================

fn interpolate(left: &Beading, ratio_left: f64, right: &Beading) -> Beading {
    let ratio_right = 1.0 - ratio_left;
    let mut ret = if left.total > right.total { left.clone() } else { right.clone() };
    for i in 0..left.widths.len().min(right.widths.len()) {
        if left.widths[i] == 0.0 || right.widths[i] == 0.0 {
            ret.widths[i] = 0.0;
        } else {
            ret.widths[i] = ratio_left * left.widths[i] + ratio_right * right.widths[i];
        }
        ret.locations[i] = ratio_left * left.locations[i] + ratio_right * right.locations[i];
    }
    ret
}

/// Interpolation between beadings of different bead counts: when an inset
/// vanished between them, bump the ratio so the merged beading keeps the
/// surviving insets on the correct side of the switching radius.
fn interpolate_switching(left: &Beading, ratio_left: f64, right: &Beading, switching_radius: i64) -> Beading {
    let ret = interpolate(left, ratio_left, right);
    let sr = switching_radius as f64;
    let mut next_inset = left.locations.len() as i64 - 1;
    while next_inset >= 0 {
        if sr > left.locations[next_inset as usize] {
            break;
        }
        next_inset -= 1;
    }
    if next_inset < 0 {
        return ret;
    }
    let next_inset = next_inset as usize;
    if next_inset + 1 == left.locations.len() {
        return ret;
    }
    if ret.locations[next_inset] > sr {
        let denom = left.locations[next_inset] - right.locations[next_inset];
        if denom.abs() > 1e-9 {
            let new_ratio = ((sr - right.locations[next_inset]) / denom + 0.1).min(1.0);
            return interpolate(left, new_ratio, right);
        }
    }
    ret
}

// ===========================================================================
// Parabola discretization (boost voronoi_visual_utils / Cura discretizeParabola)
// ===========================================================================

/// Sample the parabola with focus `p` and directrix through segment `seg`
/// between Voronoi vertices `s` and `e`. Marking points are inserted where
/// the wedge angle crosses the transitioning angle, plus the apex.
fn discretize_parabola(p: P, seg: (P, P), s: P, e: P) -> Vec<P> {
    let (a, b) = seg;
    let ab = b.sub(a);
    let ab_size = ab.vsize();
    if ab_size < 1e-9 {
        return vec![s, e];
    }
    let sx = dot(s.sub(a), ab) / ab_size;
    let ex = dot(e.sub(a), ab) / ab_size;
    let px = dot(p.sub(a), ab) / ab_size;

    let pxx = closest_on_line(p, a, b);
    let ppxx = pxx.sub(p);
    let d = ppxx.vsize();
    if d < 0.5 {
        return vec![s, e]; // degenerate: focus on the directrix
    }
    // Local frame: x along the directrix, y from the directrix toward the
    // focus; the parabola is y = x²/2d + d/2 with x measured from pxx.
    let ux = (ab.x as f64 / ab_size, ab.y as f64 / ab_size);
    let uy = (-ppxx.x as f64 / d, -ppxx.y as f64 / d);
    let emit = |x: f64, y: f64| -> P {
        P {
            x: pxx.x + (ux.0 * x + uy.0 * y).round() as i64,
            y: pxx.y + (ux.1 * x + uy.1 * y).round() as i64,
        }
    };

    let marking_bound = (TRANSITIONING_ANGLE * 0.5).atan();
    let mut msx = -marking_bound * d;
    let mut mex = marking_bound * d;
    let marking_h = msx * msx / (2.0 * d) + d / 2.0;
    let dir = if sx > ex { -1.0 } else { 1.0 };
    let mut marking_start = emit(msx, marking_h);
    let mut marking_end = emit(mex, marking_h);
    if dir < 0.0 {
        std::mem::swap(&mut marking_start, &mut marking_end);
        std::mem::swap(&mut msx, &mut mex);
    }

    let mut add_marking_start = msx * dir > (sx - px) * dir && msx * dir < (ex - px) * dir;
    let mut add_marking_end = mex * dir > (sx - px) * dir && mex * dir < (ex - px) * dir;
    let apex = emit(0.0, d / 2.0);
    let mut add_apex = (sx - px) * dir < -10.0 && (ex - px) * dir > 10.0;

    let step_count = ((ex - sx).abs() / DISCRETIZATION_STEP as f64 + 0.5) as i64;
    let mut out = vec![s];
    for step in 1..step_count {
        let x = sx + (ex - sx) * step as f64 / step_count as f64 - px;
        if add_marking_start && msx * dir < x * dir {
            out.push(marking_start);
            add_marking_start = false;
        }
        if add_apex && x * dir > 0.0 {
            out.push(apex);
            add_apex = false;
        }
        if add_marking_end && mex * dir < x * dir {
            out.push(marking_end);
            add_marking_end = false;
        }
        let y = x * x / (2.0 * d) + d / 2.0;
        out.push(emit(x, y));
    }
    if add_apex {
        out.push(apex);
    }
    if add_marking_end {
        out.push(marking_end);
    }
    out.push(e);
    // Degenerate (zero-length) pieces stay: the graph collapse pass merges
    // them; dropping points here would desync the rib structure.
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo2d::{Contour, Point};

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

    fn circle(cx: f64, cy: f64, r: f64, ccw: bool) -> Contour {
        let mut pts: Vec<Point> = (0..96)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 96.0;
                Point::from_mm(cx + r * a.cos(), cy + r * a.sin())
            })
            .collect();
        if !ccw {
            pts.reverse();
        }
        Contour::new(pts)
    }

    fn bead_len(b: &Bead) -> f64 {
        let mut len: f64 = b
            .points
            .windows(2)
            .map(|w| (w[0].x_mm() - w[1].x_mm()).hypot(w[0].y_mm() - w[1].y_mm()))
            .sum();
        if b.closed {
            let (a, z) = (b.points[0], *b.points.last().unwrap());
            len += (a.x_mm() - z.x_mm()).hypot(a.y_mm() - z.y_mm());
        }
        len
    }

    #[test]
    fn strip_two_beads_ring() {
        // 1.0 mm strip with sp≈0.407: 2 beads sharing the thickness; the walk
        // must close them into rings (one ring: both long sides + the ends).
        let vw = variable_walls_exact(&Polygons::new(), &rect(0.0, 0.0, 30.0, 1.0), 0.45, 0.407, 3)
            .expect("voronoi ok");
        assert!(!vw.inner.is_empty());
        let total: f64 = vw.inner.iter().map(bead_len).sum();
        assert!(total > 55.0, "two 30mm sides expected, got {total:.1}mm");
        let w = vw.inner[0].widths[vw.inner[0].widths.len() / 2];
        assert!((0.45..0.65).contains(&w), "stretched width {w}");
    }

    #[test]
    fn annulus_beads_all_closed() {
        // Defect 1 (chimney C-ring): an annulus must produce closed rings
        // only — the graph walk cannot fragment them.
        let mut annulus = Polygons::new();
        annulus.push(circle(0.0, 0.0, 8.5, true));
        annulus.push(circle(0.0, 0.0, 7.5, false)); // 1.0 mm band
        let vw = variable_walls_exact(&Polygons::new(), &annulus, 0.45, 0.407, 3).expect("voronoi ok");
        assert!(!vw.inner.is_empty());
        for (i, b) in vw.inner.iter().enumerate() {
            assert!(b.closed, "bead {i} open (len {:.1}mm of {})", bead_len(b), vw.inner.len());
        }
        let total: f64 = vw.inner.iter().map(bead_len).sum();
        // Two rings ≈ 2·2π·8 ≈ 100 mm.
        assert!(total > 90.0, "combined ring length only {total:.0}mm");
    }

    #[test]
    fn thin_strip_tapered_ridge_bead() {
        let vw = variable_walls_exact(&rect(0.0, 0.0, 20.0, 0.3), &Polygons::new(), 0.45, 0.407, 0)
            .expect("voronoi ok");
        assert_eq!(vw.thin_outer.len(), 1, "one centerline bead");
        let b = &vw.thin_outer[0];
        let len = bead_len(b);
        assert!(len > 15.0, "ridge length {len}");
        let w = b.widths[b.widths.len() / 2];
        assert!((0.2..0.45).contains(&w), "tapered width {w}");
    }

    #[test]
    fn l_junction_no_pileup() {
        // Defect 2 (corner blobs): at an L-junction beads must flow through
        // the corner without piling up. Pile-up shows as excess length or
        // out-of-range widths.
        let mut l = Polygons::new();
        l.push(Contour::new(vec![
            Point::from_mm(0.0, 0.0),
            Point::from_mm(12.0, 0.0),
            Point::from_mm(12.0, 1.0),
            Point::from_mm(1.0, 1.0),
            Point::from_mm(1.0, 12.0),
            Point::from_mm(0.0, 12.0),
        ]));
        let vw = variable_walls_exact(&Polygons::new(), &l, 0.45, 0.407, 3).expect("voronoi ok");
        assert!(!vw.inner.is_empty());
        for b in &vw.inner {
            for &w in &b.widths {
                assert!(w <= 0.45 * 1.75 + 1e-6, "width {w} beyond the squish cap");
            }
        }
        // Two legs ≈ 2×(11+12)mm of centerline at 2 beads ≈ 46mm; allow some
        // slack but fail on pile-up doubling.
        let total: f64 = vw.inner.iter().map(bead_len).sum();
        assert!((30.0..70.0).contains(&total), "total bead length {total:.1}mm");
        // No bead may dwell: max vertex density per mm of a bead stays sane.
        for b in &vw.inner {
            let len = bead_len(b);
            if len > 2.0 {
                let density = b.points.len() as f64 / len;
                assert!(density < 12.0, "{:.1} points/mm — pile-up", density);
            }
        }
    }

    #[test]
    fn scheme_parity_with_grid() {
        // The strategy must reproduce wall.rs's Scheme regimes exactly.
        let s = Strategy { sp: 407.0, lw: 450.0, cap2: 4 };
        // Stretch: t = 1.4mm → 3 beads.
        assert_eq!(s.optimal_bead_count(1400), 3);
        // Absorb: t = 1.93mm, cap2=4 → 4 beads (sliver swallowed).
        assert_eq!(s.optimal_bead_count(1930), 4);
        // Absorb-2: cap2=2, t = 1.6mm → remainder ≈ 0.79mm ≈ one bead: 3.
        let s2 = Strategy { sp: 407.0, lw: 450.0, cap2: 2 };
        assert_eq!(s2.optimal_bead_count(1600), 3);
        // Saturated: t = 2.83mm, cap2=4 → 4 nominal rings.
        assert_eq!(s.optimal_bead_count(2830), 4);
        let b = s.compute(2830, 4);
        assert!(b.left_over > 0.0);
        assert!((b.locations[0] - 0.5 * 407.0).abs() < 1.0, "saturated beads hug the boundary");
    }
}
