//! Toolpath planning: turn each layer's polygons into ordered extrusion paths.
//!
//! Per layer: `wall_count` concentric perimeters (inward offsets), then infill of
//! the region inside the innermost wall — split into **solid** areas (the top and
//! bottom shells) and **sparse** areas (the interior).
//!
//! Top/bottom detection is the classic boolean test: a spot is interior (sparse)
//! only if it is covered by *all* of the next `top_layers` layers above and all of
//! the previous `bottom_layers` below; otherwise it is within a shell and printed
//! solid. Finally the whole model is translated to sit centered on the bed.

use config::{InfillPattern, SeamMode, Settings, SupportMode};
use geo2d::{difference, intersection, offset, simplify, to_units, union, Contour, Point, Polygons};
use mesh::Mesh;
use rayon::prelude::*;

use crate::fill::infill_lines;
use crate::paint::PaintField;
use crate::{Layer, SliceParams};

/// What a toolpath represents — drives speed, ordering, and rendering.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PathKind {
    /// Priming loops around the first layer.
    Skirt,
    ExternalPerimeter,
    Perimeter,
    /// Wall stretch hanging more than half a bead past the layer below —
    /// printed slow, with bridge-grade cooling, so it sets in place.
    OverhangWall,
    /// Dense (100%) shell fill buried inside the part (covered above and below).
    Solid,
    /// Dense shell fill exposed to open air above — the visible top surfaces.
    /// Prints at the visible-surface pace: outer-wall speed and acceleration,
    /// always monotonic.
    TopSkin,
    /// Dense shell fill with nothing printed below it: the bed-facing first
    /// layer, plus unsupported undersides that didn't qualify as bridges.
    /// Same visible-surface pace as [`PathKind::TopSkin`].
    BottomSkin,
    /// The first solid layer over sparse infill: beads span the open cells
    /// below, so they print like (short, well-anchored) bridges.
    InternalBridge,
    /// Sparse interior fill.
    Infill,
    /// Low-flow smoothing pass over exposed top surfaces.
    Ironing,
    /// Removable support structure under overhangs.
    Support,
    /// Straight bridge lines spanning a gap, anchored on both sides.
    Bridge,
}

/// Per-segment print attributes for a bead whose speed/flow/cooling varies along
/// its length. One entry per segment (segment `k` = `points[k]`→`points[k+1]`, the
/// last wrapping for a closed loop). Lets a continuous bead carry an overhang or
/// bridge stretch WITHOUT being cut into separate paths — see [`ToolPath::segs`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SegAttr {
    pub kind: PathKind,
    pub overhang: f32,
}

/// A single continuous extrusion path.
#[derive(Clone, Debug)]
pub struct ToolPath {
    pub kind: PathKind,
    /// Closed loops (walls) extrude back to the start; open paths (infill) stop.
    pub closed: bool,
    pub width_mm: f64,
    pub points: Vec<Point>,
    /// Extrusion-flow multiplier for this path (ironing trickles). 1.0 = normal.
    pub flow: f64,
    /// Monotonic-block id: consecutive paths sharing a `Some` group move as one
    /// indivisible block in travel ordering (a strict sweep over one surface).
    /// Distinct groups (e.g. separate islands) are ordered independently.
    pub group: Option<u32>,
    /// Bead height as a fraction of the layer height; 1.0 = the full layer.
    /// Currently always 1.0 (nothing sub-divides a layer's beads).
    pub height_scale: f64,
    /// Per-point extrusion width (mm), one per `points` entry, for a continuously
    /// tapering bead (gap fill). `None` = uniform `width_mm`. When set, the g-code
    /// varies E per segment and the preview renders a variable-width ribbon;
    /// `width_mm` then carries the mean (used for feed/flow/estimates).
    pub widths: Option<Vec<f64>>,
    /// How unsupported this path's bead is — 0 = fully on solid below, 1 = fully
    /// over air — set on graduated overhang-wall runs. Drives a per-run speed lerp
    /// from the wall speed down to the overhang floor. 0 for everything else.
    pub overhang: f32,
    /// Per-SEGMENT attribute override (kind + overhang), `None` = the whole path
    /// uses `kind`/`overhang`. When set, the emitter sub-blocks the bead into runs
    /// of equal attribute and varies speed/flow/cooling ALONG it — so an overhang
    /// wall that slows only over its air span, or a fill run that bridges only its
    /// unsupported middle, stays one continuous bead (one seam, unbroken flow)
    /// instead of being cut into separate paths.
    pub segs: Option<Vec<SegAttr>>,
    /// The tool (extruder) that prints this path — the part's assigned filament
    /// slot on a toolchanger. Uniform per part, so the planner stamps it after a
    /// part's paths are complete; ordering groups by it and emit changes tools at
    /// group boundaries. 0 on a single-tool printer.
    pub tool: u32,
}

impl ToolPath {
    fn new(kind: PathKind, closed: bool, width_mm: f64, points: Vec<Point>) -> Self {
        Self {
            kind,
            closed,
            width_mm,
            points,
            flow: 1.0,
            group: None,
            height_scale: 1.0,
            widths: None,
            overhang: 0.0,
            segs: None,
            tool: 0,
        }
    }
}

/// DEBUG: the uncovered area (mm²) left inside the layer outline after stamping
/// every extrusion path at its true bead width — a raster "grid" measure of how
/// completely a fill strategy covers the layer. Returns `(outline_area, uncovered)`.
pub fn debug_uncovered(layer: &LayerPlan, lw: f64) -> (f64, f64) {
    let beads: Vec<(Vec<Point>, Vec<f64>)> = layer
        .paths
        .iter()
        .map(|p| {
            let mut pts = p.points.clone();
            let mut ws = p.widths.clone().unwrap_or_else(|| vec![p.width_mm; p.points.len()]);
            if p.closed && pts.len() >= 2 {
                pts.push(pts[0]);
                ws.push(ws[0]);
            }
            (pts, ws)
        })
        .collect();
    let unc = crate::coverage::uncovered(&layer.outline, &beads, lw, 0.0);
    if std::env::var("GRID").is_ok() {
        if let Some(bb) = layer.outline.bounds() {
            let cols = 110usize;
            let cell = (bb.max.x_mm() - bb.min.x_mm()) / cols as f64;
            let rows = ((bb.max.y_mm() - bb.min.y_mm()) / cell).ceil() as usize;
            let inside = |g: &Polygons, p: Point| g.contours.iter().filter(|c| c.contains(p)).count() % 2 == 1;
            let mut s = String::with_capacity((cols + 1) * rows);
            for r in 0..rows {
                for c in 0..cols {
                    let p = Point::from_mm(
                        bb.min.x_mm() + (c as f64 + 0.5) * cell,
                        bb.max.y_mm() - (r as f64 + 0.5) * cell,
                    );
                    s.push(if !inside(&layer.outline, p) { ' ' } else if inside(&unc, p) { '#' } else { '.' });
                }
                s.push('\n');
            }
            eprint!("{s}");
        }
    }
    (layer.outline.net_area_mm2().abs(), unc.net_area_mm2().abs())
}

/// The non-extruding move that reaches a path's start. Computed once (combing +
/// retraction + z-hop) so the g-code and the GUI preview share one source of truth.
#[derive(Clone, Debug, Default)]
pub struct Travel {
    /// G0 destinations in order, ending at the path's start (the from-point — the
    /// previous path's end — is implicit). Empty when there is no preceding move.
    pub points: Vec<Point>,
    /// Retract before this travel (it leaves the printed region).
    pub retract: bool,
    /// Z-hop over this travel (it can't be combed — crosses a void).
    pub hop: bool,
}

/// Everything needed to emit one printed layer.
#[derive(Clone, Debug)]
pub struct LayerPlan {
    pub index: usize,
    /// Nozzle Z when printing this layer (top of the layer).
    pub print_z_mm: f64,
    pub height_mm: f64,
    pub paths: Vec<ToolPath>,
    /// Lead-in travel for each path (1:1 with `paths`).
    pub travels: Vec<Travel>,
    /// The layer's solid outline (bed-centered), used for combing decisions.
    pub outline: Polygons,
    /// Speed multiplier (≤1) applied to this layer by min-layer-time cooling
    /// (what the g-code and estimates use).
    pub speed_scale: f64,
    /// A per-layer nozzle °C override for this layer (None = the profile
    /// temperature). Drives the deposited-energy model and the flow ceiling.
    pub planned_temp_c: Option<f64>,
    /// Emit `M104 S<this>` at the layer's start (None = no change). A plain
    /// asynchronous setpoint — set without waiting, no live feedback.
    pub temp_command_c: Option<f64>,
}

/// At or below this sparse-infill density, the first buried solid layer over the
/// infill spans mostly open air between the lines (≳85% unsupported), so it prints
/// as an internal bridge — bridge flow, speed, and cooling — instead of plain solid.
const INTERNAL_BRIDGE_MAX_DENSITY: f64 = 0.15;

/// Sparse pockets smaller than this the rebalance promotes to solid (a bead-scale
/// gap fills solid cleaner than sparse). Internal-bridge detection reuses it: such
/// a pocket prints solid below, so the layer over it isn't a bridge over open air.
const SOLID_BELOW_AREA_MM2: f64 = 10.0;

/// Minimum unsupported run (mm) a bead must travel over open sparse before it's
/// worth bridging. A shorter span the solid bead carries without sagging, so it
/// stays solid — no point switching to bridge flow/speed for a sub-2 mm hop.
const MIN_BRIDGE_SPAN_MM: f64 = 2.0;

/// Solid/skin regions narrower than this many line widths are filled with
/// concentric wall loops instead of a line/crosshatch pattern — a door or window
/// lintel, a thin solid bar. Concentric beads run the length of the section and
/// tie into the bounding walls; a crosshatch there is short disjoint stubs. Wider
/// regions keep line fill (a loop alongside a wall would just double it).
const NARROW_FILL_BEADS: f64 = 4.0;

/// Slice and plan a whole model into per-layer toolpaths, centered on the bed.
pub fn generate(mesh: &Mesh, settings: &Settings) -> Vec<LayerPlan> {
    generate_parts(&[(mesh, 0)], settings)
}

/// What a part prints with: one tool, or a layer-dithered blend of tools.
///
/// A blend holds `(tool, weight)` pairs (weights needn't sum to 1) and is
/// realized by alternating whole layers between its tools in that proportion —
/// at viewing distance the layer bands read as the mixed color (white, grey,
/// and black dither into a full monochrome ramp). Each layer is an ordinary
/// single-tool layer, so per-tool speeds, flow, and temperatures stay exact.
#[derive(Clone, Debug)]
pub enum PartPaint {
    Tool(u32),
    Blend(Vec<(u32, f64)>),
    /// Per-surface paint: a tool per triangle of `field` — a mesh in the SAME
    /// frame as the sliced part (the GUI's visible/display mesh, transformed to
    /// the bed). Each bead takes the tool of the nearest painted surface, and a
    /// wall/skin crossing a color boundary is split into per-tool arcs.
    /// `face_tool[i]` is triangle i's tool; unpainted areas carry the base tool.
    Surface { field: std::sync::Arc<Mesh>, face_tool: Vec<u32> },
}

/// Slice and plan a multi-part model, one tool per part.
/// See [`generate_painted`] — this is the uniform-paint convenience form.
pub fn generate_parts(parts: &[(&Mesh, u32)], settings: &Settings) -> Vec<LayerPlan> {
    let painted: Vec<(&Mesh, PartPaint)> =
        parts.iter().map(|&(m, t)| (m, PartPaint::Tool(t))).collect();
    generate_painted(&painted, settings)
}

fn gcd_u64(a: u64, b: u64) -> u64 {
    if b == 0 {
        a
    } else {
        gcd_u64(b, a % b)
    }
}

/// The layer→tool schedule realizing a blend: the tools' whole-layer counts
/// arranged so the SAME color never lands on two adjacent layers when the ratio
/// allows it (and clumps as little as possible when one color owns more than
/// half the cycle — then it's forced). Even, non-repeating bands read as a
/// smoother mixed color than clustered ones. The pattern is one minimal cycle
/// tiled up the height, so it's a pure function of the weights and the ABSOLUTE
/// layer index — every part sharing a blend prints the same tool on the same
/// layer (aligned bands, no extra toolchanges). `generate_painted` then applies
/// one top-face correction per recipe — see `swap_dominant_to_top`.
fn blend_schedule(weights: &[(u32, f64)], n: usize) -> Vec<u32> {
    // One whole-layer count per tool (blend weights ARE layer counts; round
    // defensively, fold duplicate tool entries together).
    let mut counts: Vec<(u32, u64)> = Vec::new();
    for &(t, x) in weights {
        let c = x.round();
        if c < 1.0 {
            continue;
        }
        match counts.iter_mut().find(|(tt, _)| *tt == t) {
            Some(e) => e.1 += c as u64,
            None => counts.push((t, c as u64)),
        }
    }
    let first = counts.first().map_or(0, |&(t, _)| t);
    if counts.len() <= 1 {
        return vec![first; n];
    }
    // Minimal cycle: divide out the common factor so the pattern repeats as
    // tightly as possible (6:2 tiles a 3:1 cycle, not an 8-layer one).
    let g = counts.iter().fold(0u64, |a, &(_, c)| gcd_u64(a, c));
    if g > 1 {
        for c in &mut counts {
            c.1 /= g;
        }
    }
    let cycle = anti_adjacent_cycle(&counts);
    if cycle.is_empty() {
        return vec![first; n];
    }
    (0..n).map(|i| cycle[i % cycle.len()]).collect()
}

/// Arrange `counts` (tool, whole-layer count) into one cycle that avoids
/// same-color neighbors: greedily place the tool with the most layers left that
/// ISN'T the one just placed (the "reorganize" / max-heap rule — no two
/// adjacent equal when the largest count ≤ half the cycle, minimal clumping
/// otherwise), then repair the seam where the tiled cycle wraps.
fn anti_adjacent_cycle(counts: &[(u32, u64)]) -> Vec<u32> {
    let total: u64 = counts.iter().map(|&(_, c)| c).sum();
    let mut rem: Vec<u64> = counts.iter().map(|&(_, c)| c).collect();
    let mut out: Vec<u32> = Vec::with_capacity(total as usize);
    let mut prev: Option<usize> = None;
    for _ in 0..total {
        // Most-remaining tool that isn't `prev`; ties → lowest tool id (so the
        // schedule is deterministic). Only fall back to `prev` when it's the
        // sole tool with layers left (one color owns > half the cycle).
        let mut best: Option<usize> = None;
        for i in 0..rem.len() {
            if rem[i] == 0 || Some(i) == prev {
                continue;
            }
            best = Some(match best {
                None => i,
                Some(b) if rem[i] > rem[b] || (rem[i] == rem[b] && counts[i].0 < counts[b].0) => i,
                Some(b) => b,
            });
        }
        let pick = best.unwrap_or_else(|| rem.iter().position(|&c| c > 0).unwrap());
        out.push(counts[pick].0);
        rem[pick] -= 1;
        prev = Some(pick);
    }
    // Wrap seam: if the tile ends and begins on the same tool, swap the last
    // layer with an interior one that clears the seam without adding a clash.
    let l = out.len();
    if l > 3 && out[0] == out[l - 1] {
        for j in 1..l - 1 {
            let x = out[j];
            if x != out[0] && out[l - 2] != x && out[j - 1] != out[0] && out[j + 1] != out[0] {
                out.swap(j, l - 1);
                break;
            }
        }
    }
    out
}

/// The blend's dominant tool as the SCHEDULE realizes it: the tool printing
/// the most layers, by a margin of at least two. Deriving dominance from the
/// schedule (not the weights) keeps every consumer consistent with what
/// actually prints — duplicate weight entries aggregate, and two recipes too
/// close for the schedule to distinguish (a 50/50 tie included) yield the
/// same verdict no matter which member of a schedule-equal group asks.
fn schedule_dominant(sched: &[u32]) -> Option<u32> {
    let mut counts: Vec<(u32, usize)> = Vec::new();
    for &t in sched {
        match counts.iter_mut().find(|(tool, _)| *tool == t) {
            Some((_, c)) => *c += 1,
            None => counts.push((t, 1)),
        }
    }
    if counts.len() < 2 {
        return None;
    }
    counts.sort_by(|a, b| b.1.cmp(&a.1));
    (counts[0].1 >= counts[1].1 + 2).then_some(counts[0].0)
}

/// A blended part's top face shows exactly ONE layer's filament — the side
/// walls average the whole stack, but no averaging can ever happen on top —
/// so the topmost printed layer must land on the blend's dominant tool, not
/// whatever the cycle happens to land on (a 90:10 blend would otherwise cap
/// one part in ten with the minority color). Swap the dominant tool into the
/// topmost printed layer from the nearest dominant layer below: a two-layer
/// perturbation, so the ratio and the cycle's period are untouched (the
/// toolchange count moves by at most one), and applied to the one schedule a
/// recipe group shares, lockstep across its parts survives.
///
/// `printed[i]` says whether ANY group member lays paths at layer i — slice
/// polygons are not enough (a sub-line-width top sliver slices to a contour
/// but plans nothing, and "fixing" that phantom layer would drag the minority
/// down onto the real cap). The donor must be printed too, or the traded
/// minority layer vanishes from the physical print; `protected` holds the
/// shorter members' own top layers, which must not become donors — stealing
/// one would flip that part's visible cap to the minority.
///
/// A blend with no schedule-resolvable dominant (ties) and a span holding no
/// eligible donor both leave the schedule alone.
fn swap_dominant_to_top(sched: &mut [u32], printed: &[bool], protected: &[usize]) {
    let Some(dominant) = schedule_dominant(sched) else { return };
    let Some(top) = printed.iter().rposition(|&p| p) else { return };
    if sched[top] == dominant {
        return;
    }
    let Some(base) = printed.iter().position(|&p| p) else { return };
    if let Some(j) = (base..top)
        .rev()
        .find(|&j| sched[j] == dominant && printed[j] && !protected.contains(&j))
    {
        sched.swap(j, top);
    }
}

/// Whether an exposed face region justifies the dock trip its recolor costs.
/// Erosion by FACE_ERODE_LW line widths first kills stair-step ribbons — the
/// thin exposed ledges every layer of a curved or slanted top produces —
/// which would otherwise recolor (and add a toolchange to) every layer of a
/// dome; what survives must still hold FACE_MIN_AREA_MM2 of face.
const FACE_MIN_AREA_MM2: f64 = 50.0;
const FACE_ERODE_LW: f64 = 1.5;
fn face_worth_a_dock_trip(exposed: &Polygons, line_width_mm: f64) -> bool {
    let eroded = offset(exposed, -(FACE_ERODE_LW * line_width_mm));
    eroded.net_area_mm2() >= FACE_MIN_AREA_MM2
}

/// Even-odd containment over a slice region (outers CCW, holes CW — a point
/// under a shelf's hole is outside).
fn region_contains(polys: &Polygons, p: geo2d::Point) -> bool {
    polys.contours.iter().filter(|c| c.contains(p)).count() % 2 == 1
}

/// Whether any of up to eight evenly-spaced sample points of a path lies in
/// the region — paths can straddle an exposure boundary (a solid line running
/// from under a tower out onto the shelf), so one endpoint is not enough.
fn path_touches(points: &[geo2d::Point], region: &Polygons) -> bool {
    let step = (points.len() / 8).max(1);
    points.iter().step_by(step).chain(points.last()).any(|&p| region_contains(region, p))
}

/// Slice and plan a multi-part model: each part is a mesh (already placed, all
/// sharing one bed) paired with the paint that prints it — a tool, or a
/// layer-dithered blend of tools. Parts slice on ONE shared layer grid and
/// plan independently — each keeps its own walls, skins and infill, so
/// interfaces between parts get full shells (no show-through between
/// filaments) — then merge per layer; ordering groups paths by tool so the
/// emitter changes tools once per group, not per path.
///
/// Physical reality is shared: overhang, bridge, and support decisions test
/// the union of ALL parts below, so a part resting on another is supported,
/// not floating. Overlapping volumes resolve by order: the earlier part owns
/// the overlap and later parts are trimmed around it.
/// The paint-independent result of slicing: geometry, per-layer physics, and
/// each part's planned toolpaths BEFORE any tool is stamped. Produced by
/// [`plan_geometry`] (the heavy half of the pipeline: slicing, offsets, the
/// physical union, `plan_part`, supports). Hand it to [`restamp_paint`] to
/// change which tool prints each path — a part→tool reassignment or a
/// blend-weight edit — without re-slicing, whenever the meshes and settings are
/// unchanged. Opaque to callers; only the engine reads its innards.
pub struct GeometryPlan {
    norm_settings: Settings,
    grid: Vec<Layer>,
    part_layers: Vec<Vec<Layer>>,
    phys: Vec<Polygons>,
    /// Pre-stamp: geometry planned, `path.tool` not yet assigned.
    part_plans: Vec<Vec<LayerPlan>>,
    occupied: Vec<Vec<bool>>,
    n: usize,
}

impl GeometryPlan {
    /// An empty plan (no printable geometry) — `stamp_and_finish` yields no
    /// layers from it.
    fn empty(norm_settings: Settings) -> Self {
        Self {
            norm_settings,
            grid: Vec::new(),
            part_layers: Vec::new(),
            phys: Vec::new(),
            part_plans: Vec::new(),
            occupied: Vec::new(),
            n: 0,
        }
    }
}

/// Slice and plan geometry, independent of paint — the expensive half of
/// [`generate_painted`]. Cache the result and re-stamp paint cheaply with
/// [`restamp_paint`].
/// Live progress of a slice: `done`/`total` per-layer wall-planning units,
/// bumped as the heavy per-layer wall pass finishes each layer. Shared (via an
/// `Arc`) between the worker running [`plan_geometry_tracked`] and a UI that
/// polls [`SliceProgress::fraction`] to drive a progress bar.
#[derive(Default)]
pub struct SliceProgress {
    done: std::sync::atomic::AtomicUsize,
    total: std::sync::atomic::AtomicUsize,
}

impl SliceProgress {
    pub fn new() -> Self {
        Self::default()
    }
    fn add_total(&self, n: usize) {
        self.total.fetch_add(n, std::sync::atomic::Ordering::Relaxed);
    }
    fn tick(&self) {
        self.done.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
    /// Completed fraction in `0.0..=1.0` — `0.0` until the total is known (the
    /// wall pass, where nearly all of a high-wall-count slice's time goes).
    pub fn fraction(&self) -> f32 {
        use std::sync::atomic::Ordering::Relaxed;
        let total = self.total.load(Relaxed);
        if total == 0 {
            return 0.0;
        }
        (self.done.load(Relaxed) as f32 / total as f32).clamp(0.0, 1.0)
    }
}

/// Like [`plan_geometry`], but ticks `progress` as the heavy per-layer wall pass
/// advances, so a background slice can drive a progress bar.
pub fn plan_geometry_tracked(
    parts: &[(&Mesh, PartPaint)],
    settings: &Settings,
    progress: &SliceProgress,
) -> GeometryPlan {
    plan_geometry_inner(parts, settings, Some(progress))
}

pub fn plan_geometry(parts: &[(&Mesh, PartPaint)], settings: &Settings) -> GeometryPlan {
    plan_geometry_inner(parts, settings, None)
}

fn plan_geometry_inner(
    parts: &[(&Mesh, PartPaint)],
    settings: &Settings,
    progress: Option<&SliceProgress>,
) -> GeometryPlan {
    // Spiral vase rewrites the recipe: one wall, no sparse infill, no shells
    // above the solid bottom, nothing that would interrupt the continuous loop.
    // It is meaningless across parts, so a multi-part job ignores it.
    let mut norm_settings = settings.clone();
    if parts.len() > 1 {
        norm_settings.spiral_vase = false;
    }
    if norm_settings.spiral_vase {
        norm_settings.wall_count = 1;
        norm_settings.infill_density = 0.0;
        norm_settings.top_layers = 0;
        norm_settings.support_mode = SupportMode::None;
        norm_settings.ironing = false;
        norm_settings.fuzzy_skin = false;
    }
    let settings = &norm_settings;

    // One layer grid over the union z-range, so all parts share indices and
    // print z's (a part starting above the bed just has empty lower layers).
    let mut z_range: Option<(f64, f64)> = None;
    for (m, _) in parts {
        if let Some((lo, hi)) = m.z_bounds() {
            z_range = Some(match z_range {
                None => (lo, hi),
                Some((alo, ahi)) => (alo.min(lo), ahi.max(hi)),
            });
        }
    }
    let Some((zmin, zmax)) = z_range else {
        return GeometryPlan::empty(norm_settings);
    };
    let grid = crate::slice::layer_grid(
        zmin,
        zmax,
        SliceParams {
            layer_height_mm: settings.layer_height_mm,
            first_layer_height_mm: settings.first_layer_height_mm,
        },
    );
    if grid.is_empty() {
        return GeometryPlan::empty(norm_settings);
    }
    let n = grid.len();

    let mut part_layers: Vec<Vec<Layer>> =
        parts.iter().map(|(m, _)| crate::slice::slice_mesh_on(m, &grid)).collect();

    // Contour-resolution cleanup: drop sub-resolution mesh-facet noise (0.01 mm,
    // Orca-style — preserves curve detail for arc fitting). Then dimensional
    // compensation: XY grow/shrink on every layer, and the first layer pulled in
    // to counter squish (elephant foot).
    let res = config::contour_resolution_mm();
    for layers in &mut part_layers {
        layers.par_iter_mut().for_each(|layer| {
            layer.polygons = simplify(&layer.polygons, res);
            if settings.xy_compensation_mm != 0.0 {
                layer.polygons = offset(&layer.polygons, settings.xy_compensation_mm);
            }
            if layer.index == 0 && settings.elephant_foot_mm > 0.0 {
                layer.polygons = offset(&layer.polygons, -settings.elephant_foot_mm);
            }
        });
    }

    // Overlap ownership: where parts interpenetrate (figurine assemblies often
    // do), the earlier part keeps the volume and later parts are trimmed to
    // what's left — planned independently, an overlap would print twice.
    for p in 1..part_layers.len() {
        let (done, rest) = part_layers.split_at_mut(p);
        rest[0].par_iter_mut().enumerate().for_each(|(i, layer)| {
            for prev in done.iter() {
                if !layer.polygons.is_empty() && !prev[i].polygons.is_empty() {
                    layer.polygons = difference(&layer.polygons, &prev[i].polygons);
                }
            }
        });
    }

    // The physical union per layer — everything that exists at each z, all
    // parts together. Overhang/bridge/support tests run against this (a part
    // resting on another is supported); skin placement stays per part.
    let phys: Vec<Polygons> = if parts.len() == 1 {
        part_layers[0].iter().map(|l| l.polygons.clone()).collect()
    } else {
        (0..n)
            .into_par_iter()
            .map(|i| {
                let mut acc = Polygons::new();
                for layers in &part_layers {
                    if !layers[i].polygons.is_empty() {
                        acc = union(&acc, &layers[i].polygons);
                    }
                }
                acc
            })
            .collect()
    };

    // Plan each part independently. The wall pass ticks progress once per
    // layer, so the total work is one unit per layer per part.
    if let Some(p) = progress {
        p.add_total(n * parts.len());
    }
    let mut part_plans: Vec<Vec<LayerPlan>> = Vec::with_capacity(parts.len());
    for layers in &part_layers {
        let mut plans = plan_part(layers, &phys, settings, progress);
        add_supports(&mut plans, layers, &phys, settings);
        part_plans.push(plans);
    }

    // A part's printed span comes from planned PATHS gated by its own slice
    // polygons — a sub-line-width top sliver slices to a contour but plans
    // nothing (a phantom top the swap must not decorate), and a support
    // column lays paths on layers the body never reaches.
    let occupied: Vec<Vec<bool>> = part_plans
        .iter()
        .zip(&part_layers)
        .map(|(plans, layers)| {
            (0..n).map(|i| !plans[i].paths.is_empty() && !layers[i].polygons.is_empty()).collect()
        })
        .collect();

    GeometryPlan { norm_settings, grid, part_layers, phys, part_plans, occupied, n }
}

/// The paint half of the plan: stamp each path's tool (uniform or blend
/// schedule + face override), merge parts, add brim/skirt, order, and plan
/// travels. Split out of [`generate_painted`] so a paint-only change can re-run
/// just this stage over a cached [`GeometryPlan`]. `part_plans` is moved in
/// (generate_painted hands over its owned plans; restamp clones the cache's).
fn stamp_and_finish(
    geo: &GeometryPlan,
    mut part_plans: Vec<Vec<LayerPlan>>,
    parts: &[(&Mesh, PartPaint)],
) -> Vec<LayerPlan> {
    let settings = &geo.norm_settings;
    let n = geo.n;
    let phys = &geo.phys;
    let part_layers = &geo.part_layers;
    let occupied = &geo.occupied;
    let grid = &geo.grid;

    // Blend schedules. Equal recipes yield byte-equal schedules (the schedule
    // is a pure function of the weights), so grouping by schedule equality
    // recovers exactly the lockstep sets — and dominance is read back off the
    // shared schedule itself, so every member reaches the same verdict no
    // matter its listing order. Each group gets ONE top-face swap at the
    // group-wide topmost printed layer; a shorter carrier's own top must not
    // become the donor (that would flip ITS visible cap to the minority) and
    // is instead handled by the face override at stamping below.
    let mut schedules: Vec<Option<Vec<u32>>> = parts
        .iter()
        .map(|(_, paint)| match paint {
            PartPaint::Tool(_) => None,
            PartPaint::Blend(w) => Some(blend_schedule(w, n)),
            PartPaint::Surface { .. } => None,
        })
        .collect();
    let mut done = vec![false; parts.len()];
    for i in 0..parts.len() {
        if done[i] || schedules[i].is_none() {
            continue;
        }
        let members: Vec<usize> =
            (i..parts.len()).filter(|&j| !done[j] && schedules[j] == schedules[i]).collect();
        let mut printed = vec![false; n];
        let mut tops = Vec::new();
        for &m in &members {
            done[m] = true;
            for (p, &occ) in printed.iter_mut().zip(&occupied[m]) {
                *p |= occ;
            }
            tops.extend(occupied[m].iter().rposition(|&occ| occ));
        }
        let Some(group_top) = tops.iter().copied().max() else { continue };
        let protected: Vec<usize> = tops.into_iter().filter(|&t| t < group_top).collect();
        for &m in &members {
            swap_dominant_to_top(schedules[m].as_mut().unwrap(), &printed, &protected);
        }
    }

    // Stamp each part's paint on every path — a uniform tool, or the blend's
    // per-layer schedule (geometry is identical either way; only the tool that
    // prints each layer varies). Blends then get the FACE OVERRIDE: a layer
    // whose exposed top/bottom face is big enough to justify the dock trip
    // recolors that face's paths with the dominant tool — the shelf a stepped
    // model shows from above, a floating part's underside, the bed face, and
    // a shorter carrier's own top all read as the blend's color instead of
    // whichever band the cycle dealt them. Buried paths and gate-rejected
    // layers keep the schedule: the walls' dither is what mixes the color,
    // and it must stay exact.
    let lw = settings.line_width_mm;
    let empty = Polygons::new();
    // Nearest-painted-surface field per surface-painted part (Tool/Blend skip it).
    let fields: Vec<Option<PaintField>> = parts
        .iter()
        .map(|(_, paint)| match paint {
            PartPaint::Surface { field, face_tool } => Some(PaintField::new(field, face_tool)),
            _ => None,
        })
        .collect();
    for (m, plans) in part_plans.iter_mut().enumerate() {
        let (_, paint) = &parts[m];
        let schedule = &schedules[m];
        let dominant = schedule.as_deref().and_then(schedule_dominant);
        let layers = &part_layers[m];
        for plan in plans.iter_mut() {
            let i = plan.index;
            // Surface paint: recolor each path by the nearest painted surface at
            // this layer's z, splitting a wall/skin that crosses a color
            // boundary into per-tool arcs. Interior fill takes a majority tool.
            if let Some(field) = &fields[m] {
                let z = layers[i].z_mm;
                split_paths_by_tool(&mut plan.paths, 0, lw, |p| {
                    field.value_at([p.x_mm(), p.y_mm(), z])
                });
                continue;
            }
            let tool = match (paint, schedule) {
                (PartPaint::Tool(t), _) => *t,
                (_, Some(sched)) => sched[i],
                _ => unreachable!(),
            };
            for path in &mut plan.paths {
                path.tool = tool;
            }
            let Some(dom) = dominant else { continue };
            if dom == tool || !occupied[m][i] {
                continue;
            }
            // Exposure is judged against the whole-scene union: a face
            // pressed onto ANOTHER part is an interface, not a face, and not
            // worth a dock trip. Supports are absent from phys on purpose —
            // a supported underside is visible once the support strips. The
            // 0.05 mm guard matches plan_part's skin exposure: a hairline
            // misregistration between neighbors is not a face.
            let above = if i + 1 < n { &phys[i + 1] } else { &empty };
            let below = if i > 0 { &phys[i - 1] } else { &empty };
            let exposed = union(
                &difference(&layers[i].polygons, &offset(above, 0.05)),
                &difference(&layers[i].polygons, &offset(below, 0.05)),
            );
            if exposed.is_empty() || !face_worth_a_dock_trip(&exposed, lw) {
                continue;
            }
            for path in &mut plan.paths {
                if path_touches(&path.points, &exposed) {
                    path.tool = dom;
                }
            }
        }
    }

    // Merge per layer: paths in part order (order_layers regroups by tool);
    // outline = union of the parts' outlines (the combing domain). The single-
    // part path hands its plans through untouched.
    let mut plans: Vec<LayerPlan> = if part_plans.len() == 1 {
        part_plans.pop().unwrap()
    } else {
        (0..n)
            .map(|i| {
                let mut paths = Vec::new();
                let mut outline = Polygons::new();
                for pp in &mut part_plans {
                    paths.append(&mut pp[i].paths);
                    if outline.is_empty() {
                        outline = std::mem::take(&mut pp[i].outline);
                    } else if !pp[i].outline.is_empty() {
                        outline = union(&outline, &pp[i].outline);
                    }
                }
                LayerPlan {
                    index: i,
                    print_z_mm: grid[i].print_z_mm,
                    height_mm: grid[i].height_mm,
                    paths,
                    travels: Vec::new(),
                    outline,
                    speed_scale: 1.0,
                    planned_temp_c: None,
                    temp_command_c: None,
                }
            })
            .collect()
    };

    // The tool that starts the print — lowest tool with material on layer 0.
    // Brim and skirt prime it, and layer ordering leads with it.
    let initial_tool = plans
        .first()
        .and_then(|p0| p0.paths.iter().map(|p| p.tool).min())
        .unwrap_or(0);

    // Brim: loops extending outward from the first-layer outline, touching the
    // part for bed adhesion.
    if settings.brim_loops > 0 {
        if let Some(plan0) = plans.first_mut() {
            let mut brim = brim_paths(&phys[0], settings);
            for p in &mut brim {
                p.tool = initial_tool;
            }
            plan0.paths.splice(0..0, brim);
        }
    }

    // Skirt: priming loops around the first layer, printed before anything else.
    if settings.skirt_loops > 0 {
        if let Some(plan0) = plans.first_mut() {
            let mut skirt = skirt_paths(&phys[0], settings);
            for p in &mut skirt {
                p.tool = initial_tool;
            }
            plan0.paths.splice(0..0, skirt);
        }
    }

    order_layers(&mut plans, settings.outer_wall_first);
    order_unsupported_rings_outer_in(&mut plans, &phys, settings);
    center_on_bed(&mut plans, parts, settings);
    if matches!(settings.seam_mode, SeamMode::Aligned | SeamMode::Sharpest) {
        align_seams(&mut plans, settings.seam_mode);
    }
    crate::emit::plan_travels(&mut plans, settings);
    crate::emit::apply_min_layer_time(&mut plans, settings);
    plans
}

/// Slice a set of painted parts into the final per-layer toolpaths. Equivalent
/// to `plan_geometry` followed by `stamp_and_finish` — kept as one call for the
/// common path (it hands its owned pre-stamp plans straight through, no clone).
pub fn generate_painted(parts: &[(&Mesh, PartPaint)], settings: &Settings) -> Vec<LayerPlan> {
    let mut geo = plan_geometry(parts, settings);
    let part_plans = std::mem::take(&mut geo.part_plans);
    stamp_and_finish(&geo, part_plans, parts)
}

fn splittable_kind(k: PathKind) -> bool {
    matches!(
        k,
        PathKind::ExternalPerimeter
            | PathKind::Perimeter
            | PathKind::OverhangWall
            | PathKind::Solid
            | PathKind::TopSkin
            | PathKind::BottomSkin
            | PathKind::Bridge
    )
}

fn majority_tool(tools: &[u32]) -> u32 {
    let mut counts: Vec<(u32, u32)> = Vec::new();
    for &t in tools {
        match counts.iter_mut().find(|(tt, _)| *tt == t) {
            Some(e) => e.1 += 1,
            None => counts.push((t, 1)),
        }
    }
    counts.iter().max_by_key(|(_, c)| *c).map(|&(t, _)| t).unwrap_or(0)
}

/// Fold any color run shorter than one line width into a neighbor, so boundary
/// jitter can't spawn a sub-mm arc (and thus a wasteful dock trip). `seg[k]` is
/// the tool of segment k; mutated in place. Terminates: each pass merges the
/// shortest short run into a neighbor, dropping one run.
fn merge_short_runs(points: &[Point], seg: &mut [u32], closed: bool, lw: f64) {
    let nseg = seg.len();
    if nseg < 2 {
        return;
    }
    let n = points.len();
    let seglen = |k: usize| pt_dist_mm(points[k % n], points[(k + 1) % n]);
    for _ in 0..nseg {
        let mut runs: Vec<(usize, usize, u32, f64)> = Vec::new(); // start, count, tool, mm
        let mut k = 0;
        while k < nseg {
            let t = seg[k];
            let s = k;
            let mut plen = 0.0;
            while k < nseg && seg[k] == t {
                plen += seglen(k);
                k += 1;
            }
            runs.push((s, k - s, t, plen));
        }
        if runs.len() < 2 {
            break;
        }
        let ri = (0..runs.len()).min_by(|&a, &b| runs[a].3.total_cmp(&runs[b].3)).unwrap();
        if runs[ri].3 >= lw {
            break;
        }
        let prev = if ri > 0 { ri - 1 } else { runs.len() - 1 };
        let next = if ri + 1 < runs.len() { ri + 1 } else { 0 };
        let has_prev = ri > 0 || closed;
        let has_next = ri + 1 < runs.len() || closed;
        let ntool = match (has_prev, has_next) {
            (true, true) => {
                if runs[prev].3 >= runs[next].3 {
                    runs[prev].2
                } else {
                    runs[next].2
                }
            }
            (true, false) => runs[prev].2,
            (false, true) => runs[next].2,
            (false, false) => break,
        };
        for k in runs[ri].0..runs[ri].0 + runs[ri].1 {
            seg[k] = ntool;
        }
    }
}

/// Recolor and, where a path crosses a color boundary, SPLIT it into per-tool
/// arcs — by the surface `tool_at` query. A wall or skin that spans two colors
/// becomes consecutive open arcs abutting at the boundary vertex (a bead can't
/// change tool mid-stroke — a dock toolchange parks Z — so splitting is
/// mandatory). Infill and helper paths take a single majority tool so interior
/// fill never trips a mid-layer swap. `order_layers` regroups the arcs by tool
/// downstream, so a split loop still costs one change per tool.
fn split_paths_by_tool(
    paths: &mut Vec<ToolPath>,
    start: usize,
    lw: f64,
    tool_at: impl Fn(&Point) -> u32,
) {
    let beads: Vec<ToolPath> = paths.split_off(start);
    for bead in beads {
        split_one_bead(paths, bead, lw, &tool_at);
    }
}

/// Recolor one bead by `tool_at` and, if it crosses a color boundary, split it
/// into per-tool arcs — pushing the result(s) onto `out`.
fn split_one_bead(out: &mut Vec<ToolPath>, bead: ToolPath, lw: f64, tool_at: impl Fn(&Point) -> u32) {
    let n = bead.points.len();
    if n < 2 {
        out.push(bead);
        return;
    }
    let ptools: Vec<u32> = bead.points.iter().map(|p| tool_at(p)).collect();
    if ptools.iter().all(|&t| t == ptools[0]) {
        let mut b = bead;
        b.tool = ptools[0];
        out.push(b);
        return;
    }
    if !splittable_kind(bead.kind) {
        let mut b = bead;
        b.tool = majority_tool(&ptools);
        out.push(b);
        return;
    }
    // Segment k (start-vertex tool) → group into maximal same-tool runs.
    let nseg = if bead.closed { n } else { n - 1 };
    let mut seg: Vec<u32> = (0..nseg).map(|k| ptools[k]).collect();
    merge_short_runs(&bead.points, &mut seg, bead.closed, lw);
    if seg.iter().all(|&t| t == seg[0]) {
        let mut b = bead;
        b.tool = seg[0];
        out.push(b);
        return;
    }
    // Rotate a closed loop so a run boundary starts the sequence; open
    // paths start at 0.
    let rot = if bead.closed {
        (0..nseg).find(|&k| seg[k] != seg[(k + nseg - 1) % nseg]).unwrap_or(0)
    } else {
        0
    };
    let mut i = 0;
    while i < nseg {
        let t = seg[(rot + i) % nseg];
        let mut j = i;
        while j < nseg && seg[(rot + j) % nseg] == t {
            j += 1;
        }
        // Segments i..j → points (rot+i)..=(rot+j); consecutive arcs share
        // the boundary vertex, so they abut with no gap.
        let pts: Vec<Point> = (i..=j).map(|k| bead.points[(rot + k) % n]).collect();
        if pts.len() >= 2 {
            let mut b = bead.clone();
            b.points = pts;
            b.closed = false; // an arc of a split loop is an open stroke
            b.tool = t;
            b.widths = None; // per-point/segment channels don't survive a cut
            b.segs = None;
            out.push(b);
        }
        i = j;
    }
}

/// How far past the flooded patch (laterally, mm) a bead may sit and still count
/// as hugging it — bridges the mesh-triangle sampling and a small mesh↔bead gap.
const PAINT_LAT_TOL: f64 = 0.35;
/// How far OUTSIDE the mesh surface (mm) a bead may sit and still be painted.
const PAINT_SURF_OUT: f64 = 0.30;
/// How far INSIDE the mesh surface (mm) the paint reaches — deep enough to catch
/// the visible outer wall(s), shallow enough to leave the deep interior (infill,
/// inner walls, the back side) alone, so no color bleeds in from inside.
const PAINT_SURF_IN: f64 = 0.80;

/// A freehand paint stroke on the sliced beads: the CONNECTED front-surface patch
/// (`tris`, flooded from where the brush clicked) it covers, plus a `center` +
/// `radius` bounding sphere for cheap culling. A bead is painted only if it hugs
/// the outer shell of THAT patch (see [`dab_covers`]) — surface connectivity that
/// keeps the brush off disconnected/back-side/deep-interior beads. `None` tool
/// clears (keeps base). All coords are in the sliced LayerPlan frame (the GUI
/// shifts by the bed origin).
#[derive(Clone, Debug)]
pub struct BeadDab {
    pub center: [f64; 3],
    pub radius: f64,
    pub tool: Option<u32>,
    pub tris: Vec<[[f64; 3]; 3]>,
}

/// Closest point on triangle `abc` to `p` (Ericson, Real-Time Collision
/// Detection §5.1.5) — Voronoi-region tests over the 3 vertices, 3 edges, face.
fn closest_on_tri(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    let sub = |u: [f64; 3], v: [f64; 3]| [u[0] - v[0], u[1] - v[1], u[2] - v[2]];
    let dot = |u: [f64; 3], v: [f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return [a[0] + ab[0] * v, a[1] + ab[1] * v, a[2] + ab[2] * v];
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return [a[0] + ac[0] * w, a[1] + ac[1] * w, a[2] + ac[2] * w];
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return [b[0] + (c[0] - b[0]) * w, b[1] + (c[1] - b[1]) * w, b[2] + (c[2] - b[2]) * w];
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w]
}

/// Whether point `p` hugs the outer shell of a flooded surface patch `tris`:
/// within `PAINT_LAT_TOL` laterally of the nearest triangle AND inside the
/// `[-PAINT_SURF_IN, +PAINT_SURF_OUT]` depth window along that triangle's outward
/// normal. The flood only ever contains FRONT faces of the CONNECTED surface, so
/// this paints the visible surface the brush is over and nothing else. Same test
/// runs in the GUI preview and this export, so they match.
pub fn dab_covers(tris: &[[[f64; 3]; 3]], p: [f64; 3]) -> bool {
    let mut best = f64::MAX;
    let mut covered = false;
    for t in tris {
        let cp = closest_on_tri(p, t[0], t[1], t[2]);
        let d = [p[0] - cp[0], p[1] - cp[1], p[2] - cp[2]];
        let d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        if d2 >= best {
            continue;
        }
        best = d2;
        // Outward normal: the flooded faces are front-facing, so the winding
        // cross product points out of the model.
        let ab = [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]];
        let ac = [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]];
        let n = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        let nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-9);
        let perp = (d[0] * n[0] + d[1] * n[1] + d[2] * n[2]) / nl;
        let lat2 = (d2 - perp * perp).max(0.0);
        covered = lat2 <= PAINT_LAT_TOL * PAINT_LAT_TOL
            && perp >= -PAINT_SURF_IN
            && perp <= PAINT_SURF_OUT;
    }
    covered
}

/// Max bead-segment length (mm) after subdividing near a dab — the paint
/// resolution for the export split. Matches the GUI's preview subdivision.
const PAINT_SUBDIV_MM: f64 = 0.5;

/// Whether any dab's sphere reaches this bead at layer `z` (a cheap circle-vs-
/// bbox test so we only subdivide painted beads).
fn bead_near_dab(bead: &ToolPath, z: f64, dabs: &[BeadDab]) -> bool {
    let (mut lox, mut loy, mut hix, mut hiy) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for p in &bead.points {
        lox = lox.min(p.x_mm());
        loy = loy.min(p.y_mm());
        hix = hix.max(p.x_mm());
        hiy = hiy.max(p.y_mm());
    }
    for d in dabs {
        let dz = z - d.center[2];
        if dz.abs() > d.radius {
            continue;
        }
        // The sphere's radius at this layer's z, plus a segment of margin.
        let rxy = (d.radius * d.radius - dz * dz).sqrt() + PAINT_SUBDIV_MM;
        let cx = d.center[0].clamp(lox, hix);
        let cy = d.center[1].clamp(loy, hiy);
        let (ex, ey) = (d.center[0] - cx, d.center[1] - cy);
        if ex * ex + ey * ey <= rxy * rxy {
            return true;
        }
    }
    false
}

/// Subdivide a bead's segments to ≤ `max_seg` mm so a paint boundary can fall
/// mid-segment. Drops per-point width/segment channels (a cut invalidates them).
fn subdivide_bead(bead: &mut ToolPath, max_seg: f64) {
    let pts = &bead.points;
    let n = pts.len();
    if n < 2 {
        return;
    }
    let segs = if bead.closed { n } else { n - 1 };
    let mut out: Vec<Point> = Vec::with_capacity(n);
    for k in 0..segs {
        let a = pts[k];
        let b = pts[(k + 1) % n];
        out.push(a);
        let len = ((b.x_mm() - a.x_mm()).powi(2) + (b.y_mm() - a.y_mm()).powi(2)).sqrt();
        let nsub = (len / max_seg).ceil().max(1.0) as usize;
        for i in 1..nsub {
            let t = i as f64 / nsub as f64;
            out.push(Point::from_mm(
                a.x_mm() + (b.x_mm() - a.x_mm()) * t,
                a.y_mm() + (b.y_mm() - a.y_mm()) * t,
            ));
        }
    }
    if !bead.closed {
        out.push(pts[n - 1]);
    }
    bead.points = out;
    bead.widths = None;
    bead.segs = None;
}

/// The tool a dab list assigns at point `(p, z)`: the LAST dab whose flooded
/// surface patch this point hugs wins (`Some` = paint, `None` = erase); `None`
/// overall = no dab → keep base.
fn dab_tool_at(p: &Point, z: f64, dabs: &[BeadDab]) -> Option<u32> {
    let (px, py) = (p.x_mm(), p.y_mm());
    let mut result = None;
    for d in dabs {
        // Cheap bounding-sphere cull: the patch lives within radius+reach of the
        // center, so skip the per-triangle test for far dabs.
        let (dx, dy, dz) = (px - d.center[0], py - d.center[1], z - d.center[2]);
        let r = d.radius + PAINT_SURF_IN + PAINT_LAT_TOL;
        if dx * dx + dy * dy + dz * dz > r * r {
            continue;
        }
        if dab_covers(&d.tris, [px, py, z]) {
            result = d.tool;
        }
    }
    result
}

/// Apply bead-paint dabs to already-sliced layers: recolor each bead (and split
/// a wall/skin that crosses a dab boundary into per-tool arcs) by the dab field
/// at that layer's z, falling back to the bead's existing tool where no dab
/// applies. In-place; a no-op when `dabs` is empty (so an unpainted export is
/// byte-identical). NOTE: does not re-order by tool — good enough for a
/// post-slice recolor, at the cost of a few extra dock trips per layer.
pub fn apply_bead_dabs(layers: &mut [LayerPlan], dabs: &[BeadDab], settings: &Settings) {
    if dabs.is_empty() {
        return;
    }
    let lw = settings.line_width_mm;
    for layer in layers.iter_mut() {
        let z = layer.print_z_mm;
        let beads = std::mem::take(&mut layer.paths);
        for mut bead in beads {
            // Subdivide only beads a dab reaches, so the split falls per-portion
            // (a long straight run painted at its middle splits mid-run, not
            // whole) without bloating untouched paths.
            if bead_near_dab(&bead, z, dabs) {
                subdivide_bead(&mut bead, PAINT_SUBDIV_MM);
            }
            let base = bead.tool;
            split_one_bead(&mut layer.paths, bead, lw, |p| {
                dab_tool_at(p, z, dabs).unwrap_or(base)
            });
        }
    }
    // Re-group each layer's paths by tool so the print does everything one tool
    // owns before docking — otherwise the freshly-split arcs interleave with the
    // base tool and every crossing is a dock trip (hundreds per small region).
    order_layers(layers, settings.outer_wall_first);
}

/// Re-stamp paint onto a cached [`GeometryPlan`] without re-slicing: only the
/// tool each path carries (and everything downstream — face override, layer
/// ordering by tool, travels) is recomputed. Valid ONLY when the meshes and
/// settings that produced `geo` are unchanged and just the parts' `PartPaint`
/// differs (a part→tool reassignment or a blend-weight edit); the caller owns
/// that invariant. Output is byte-identical to `generate_painted` with the new
/// paint (verified by `restamp_matches_full_slice`).
pub fn restamp_paint(geo: &GeometryPlan, parts: &[(&Mesh, PartPaint)]) -> Vec<LayerPlan> {
    stamp_and_finish(geo, geo.part_plans.clone(), parts)
}

/// Plan one part's layers into per-layer toolpaths — everything that reads only
/// this part's own material. `phys` is the per-layer union of ALL parts: the
/// "is there something below me" physical tests (overhang walls, bridges,
/// enclosed ceilings) run against it, while top/bottom skin placement
/// deliberately stays per part so interfaces between parts get full shells.
fn plan_part(
    layers: &[Layer],
    phys: &[Polygons],
    settings: &Settings,
    progress: Option<&SliceProgress>,
) -> Vec<LayerPlan> {
    let lw = settings.line_width_mm;
    let n = layers.len();

    // A region with nothing immediately above (or below) is a top (or bottom)
    // surface, and a surface overrides a wall — it must be skinned even when the
    // wall count alone would otherwise fill the whole cross-section. Precompute
    // the exposed faces from the full outlines so pass 1 keeps the inner walls
    // clear of them and the skin can claim them.
    let surface_per_layer: Vec<Polygons> = (0..n)
        .into_par_iter()
        .map(|i| {
            let here = &layers[i].polygons;
            // A face is only reserved (carved out of the walls, skinned) if it will
            // actually be skinned — gate each on its own shell count. With top and
            // bottom layers both 0 there is no surface at all, so the walls fill the
            // cross-section solid with no infill or shells anywhere.
            let top = if settings.top_layers == 0 {
                Polygons::new()
            } else if i + 1 < n {
                difference(here, &offset(&layers[i + 1].polygons, 0.05))
            } else {
                here.clone()
            };
            let bot = if settings.bottom_layers == 0 {
                Polygons::new()
            } else if i > 0 {
                difference(here, &offset(&layers[i - 1].polygons, 0.05))
            } else {
                here.clone()
            };
            // Open at half a line to drop ribbons too thin to skin or carve out.
            offset(&offset(&union(&top, &bot), -lw * 0.5), lw * 0.5)
        })
        .collect();

    // Pass 1: walls + the infill region (inside the innermost wall) per layer.
    let per_layer: Vec<(Vec<ToolPath>, Polygons)> = layers
        .par_iter()
        .map(|layer| {
            // Adjacent beads are placed at the stadium spacing: rounded
            // shoulders overlap just enough to fill the cusps between beads.
            let sp = config::bead_spacing_mm(lw, layer.height_mm);
            let interior: &Polygons = &layer.polygons;
            // An enclosed over-air ceiling (a hollow ringed by supported walls, the
            // hollow dominating the island) prints as ONE bridge sheet anchored on
            // the outer wall — no inner perimeters boxing it in. Carve its interior
            // out of the wall region here; Pass 2 bridges it.
            let ceiling = if layer.index > 0 && settings.bottom_layers > 0 {
                let allowance =
                    settings.layer_height_mm * settings.support_overhang_angle_deg.to_radians().tan();
                // Carve reach: clears inner perimeters out of the hollow + the
                // foothold band, but keeps the rings beyond it.
                let foothold = settings.bridge_foothold_mm;
                enclosed_ceiling_sheet(&layer.polygons, &phys[layer.index - 1], lw, allowance, foothold)
            } else {
                Polygons::new()
            };
            // Inner walls are the SAME interior offset as the outer wall, so they run
            // PARALLEL to it — straight through the skin surface near the edge instead
            // of detouring around it. The skin/solid then fills INSIDE the innermost
            // wall (Pass 2), reclaiming the interior the old surface-carved inner walls
            // used to wander through. Only the enclosed ceiling is carved out, so the
            // walls don't box in its bridged hollow. (Top surfaces get their edge
            // perimeters + skin inside, like every other slicer — no more wandering.)
            // Surface-overrides-wall: an exposed top/bottom face a tall wall stack
            // would otherwise bury in concentric rings is reserved for skin. Kept
            // strictly inside the outer wall (offset -lw) and clear of the enclosed
            // ceiling (its own bridge path), it is carved out of the INNER walls here
            // and folded back into the infill region below (see `opened`), so the skin
            // machinery claims it at any wall count with no Pass-2 changes. Scoped per
            // exposed island to fire ONLY where the normal infill region no longer
            // reaches the face (the stack consumed it) AND the face is wide enough to
            // skin — so normal wall counts and thin lips stay byte-identical (walls
            // run through, skin fills inside the innermost wall, exactly as today).
            let base_region = difference(interior, &ceiling);
            let surf_reserved = if surface_per_layer[layer.index].is_empty() {
                Polygons::new()
            } else {
                let wall_depth = match settings.wall_count {
                    0 => 0.0,
                    wc => lw + (wc - 1) as f64 * sp,
                };
                // The exposed face, pulled inside the outer-wall bead and clear of the
                // enclosed ceiling (so that dedicated bridge path stays byte-identical).
                let surf_inside = difference(
                    &intersection(&surface_per_layer[layer.index], &offset(&layer.polygons, -lw)),
                    &ceiling,
                );
                // Today's uncarved infill region — where sparse/solid would land.
                let inner_bare = offset(&base_region, -wall_depth);
                let mut acc = Polygons::new();
                for isl in islands(&surf_inside) {
                    // Reserve iff the wall stack buries this face whole (no infill
                    // reaches it) YET it is wide enough to hold skin. Both hold only
                    // once wall_depth outgrows ~a line width (high wall count), so low
                    // counts reserve nothing and stay byte-identical; thin lips/lintels
                    // fail the width test and keep their walls, exactly as today.
                    if intersection(&isl, &inner_bare).is_empty() && !offset(&isl, -lw).is_empty() {
                        acc = union(&acc, &isl);
                    }
                }
                acc
            };
            let wall_region = if surf_reserved.is_empty() {
                base_region
            } else {
                difference(&base_region, &surf_reserved)
            };
            let interior: &Polygons = &wall_region;
            let mut walls = Vec::new();
            let mut consumed = false;
            for w in 0..settings.wall_count {
                let inset = -(lw * 0.5 + w as f64 * sp);
                let kind = if w == 0 {
                    PathKind::ExternalPerimeter
                } else {
                    PathKind::Perimeter
                };
                // Outer wall (w == 0) hugs the true outline;
                // inner walls are the SAME interior offset, so they run parallel to
                // the outer at stadium spacing all the way around — the surface is
                // no longer carved out of this region, so they no longer detour.
                let centers = offset(if w == 0 { &layer.polygons } else { interior }, inset);
                // The cross-section can't hold this many walls: once an inward
                // offset comes back empty, every deeper one will too (erosion is
                // monotonic in the inset), so stop instead of grinding through
                // the remaining offsets — up to 90-odd full Clipper passes per
                // layer at wall_count=99, all returning empty. Pure speed:
                // `emit_loops` on an empty set already emitted nothing, so the
                // planned paths are byte-identical.
                if centers.is_empty() {
                    consumed = true;
                    break;
                }
                let emit_loops = |src: &Polygons, walls: &mut Vec<ToolPath>| {
                    for c in &src.contours {
                        if c.points.len() >= 3 {
                            // Drop sub-bead micro-loops on INNER walls — the offset
                            // pinches these off at corners and junctions (a thin-wall
                            // cusp, a spanned-lintel corner). The outer wall is never
                            // touched, so no visible detail is lost, and the raster
                            // gap fill fills whatever area a dropped loop leaves.
                            if kind == PathKind::Perimeter {
                                let m = c.points.len();
                                let perim: f64 =
                                    (0..m).map(|j| pt_dist_mm(c.points[j], c.points[(j + 1) % m])).sum();
                                if perim < lw * 5.0 {
                                    continue;
                                }
                            }
                            let mut points = place_seam(c.points.clone(), settings.seam_mode, layer.index);
                            // Fuzzy skin: jitter the visible (outermost) wall — not on
                            // the first layer, which must stay flat on the bed.
                            if settings.fuzzy_skin && kind == PathKind::ExternalPerimeter && layer.index > 0 {
                                points = fuzzy_loop(
                                    &points,
                                    settings.fuzzy_skin_thickness_mm,
                                    settings.fuzzy_skin_point_dist_mm,
                                    layer.index,
                                );
                            }
                            walls.push(ToolPath {
                                kind,
                                closed: true,
                                width_mm: lw,
                                points,
                                flow: 1.0,
                                group: None,
                                height_scale: 1.0,
                                widths: None,
                                overhang: 0.0,
                                segs: None,
                                tool: 0,
                            });
                        }
                    }
                };
                emit_loops(&centers, &mut walls);
            }

            // Inset to the infill region (the inner edge of the last wall bead),
            // then morphologically "open" it (erode then dilate by half a line
            // width) to drop slivers narrower than a line — those only produce
            // tiny, useless dabs of infill.
            let wall_depth = match settings.wall_count {
                0 => 0.0,
                wc => lw + (wc - 1) as f64 * sp,
            };
            // Inside the innermost wall — the skin/solid fills this whole region now
            // (the surface is no longer carved out into a separate band), so the
            // interior the wandering walls used to occupy is reclaimed as fill.
            // If the walls already consumed the cross-section (the loop broke
            // early), the region inside the innermost wall is empty — offsetting
            // by the full wall depth would just be another expensive empty
            // Clipper pass, so skip it. Byte-identical: that offset returns empty
            // here anyway (wall_depth is at least as deep as the inset that came
            // back empty).
            let inset =
                if consumed { Polygons::new() } else { offset(interior, -wall_depth) };
            let opened = offset(&offset(&inset, -lw * 0.5), lw * 0.5);
            // Fold the reserved exposed face back into the infill region so it skins
            // even when the wall stack consumed the section (inset empty). Empty on
            // every normal-count / no-surface layer → `opened` byte-identical there.
            let opened = if surf_reserved.is_empty() {
                opened
            } else {
                union(&opened, &surf_reserved)
            };
            // Wall stretches hanging past the layer below print slow with full
            // cooling (the spiral loop must stay whole, so vase mode skips).
            // The unsupported region is usually empty, making this free.
            let mut walls = if layer.index > 0 && !settings.spiral_vase {
                let below = offset(&phys[layer.index - 1], 0.05);
                let unsupported = difference(&layer.polygons, &below);
                if unsupported.is_empty() {
                    walls
                } else {
                    slow_overhanging_walls(walls, &below, lw)
                }
            } else {
                walls
            };
            // Thin-wall trim: where two inner walls double up (a neck too thin for
            // the perimeters it nominally fits — e.g. a hole beside the outline),
            // drop the doubled run so the nozzle doesn't retrace the same bead.
            trim_thin_walls(&mut walls, lw, sp);
            // Inner walls stay CLOSED rings here — they are spiralized in Pass 2
            // (spiralize_shells), together with the concentric fill they ring, so the
            // wall stack and cavity fill become one continuous stroke.
            (walls, opened)
        })
        // Tick once per layer as its walls finish — this pass is ~all of a
        // high-wall-count slice's time, so it drives the progress bar.
        .inspect(|_| {
            if let Some(p) = progress {
                p.tick();
            }
        })
        .collect();
    let mut walls_per_layer: Vec<Vec<ToolPath>> = Vec::with_capacity(n);
    let mut inner_per_layer: Vec<Polygons> = Vec::with_capacity(n);
    for (w, inner) in per_layer {
        walls_per_layer.push(w);
        inner_per_layer.push(inner);
    }

    // Solid shells per layer (exposed to air above/below unless covered by the
    // whole shell range). Precomputed so each layer can also see the layer
    // below's split — the first solid layer over sparse infill bridges it.
    let solid_all_per_layer: Vec<Polygons> = (0..n)
        .into_par_iter()
        .map(|i| {
            let inner = &inner_per_layer[i];
            if inner.is_empty() {
                return Polygons::new();
            }
            let solid_top = if settings.top_layers > 0 {
                difference(inner, &coverage(&inner_per_layer, i, 1, settings.top_layers, n))
            } else {
                Polygons::new()
            };
            let solid_bottom = if settings.bottom_layers > 0 {
                difference(inner, &coverage(&inner_per_layer, i, -1, settings.bottom_layers, n))
            } else {
                Polygons::new()
            };
            union(&solid_top, &solid_bottom)
        })
        .collect();

    // Pass 2: assemble layers, splitting infill into solid shells + sparse core.
    let plans: Vec<LayerPlan> = walls_per_layer
        .into_par_iter()
        .enumerate()
        .map(|(i, mut paths)| {
        let inner = &inner_per_layer[i];
        let ov = lw * settings.infill_overlap.clamp(0.0, 0.5);
        let sp = config::bead_spacing_mm(lw, layers[i].height_mm);
        // Where the perimeters actually are: the layer minus the wall beads (each
        // closed wall loop stamped as a dilate−erode annulus of its width). `inner`
        // stops ~1 bead short of this — the void that rings inset surfaces — so the
        // solid sheet gaps. `wall_gap` is exactly that ring; the solid fill grows into
        // it below to reach the walls flush, without disturbing the region split.
        // ftw: the true region inside the innermost wall — the layer minus the actual
        // wall beads (each perimeter loop stamped as a dilate−erode annulus of its
        // width). This is the target each skin line-end is walked out to, per bead, so
        // it meets the real wall edge (built from the beads, not a layer-offset guess).
        let ftw = {
            let mut band = Polygons::new();
            for wp in &paths {
                if wp.points.len() < 2 {
                    continue;
                }
                let bead = if wp.closed {
                    if wp.points.len() < 3 {
                        continue;
                    }
                    let mut poly = Polygons::new();
                    poly.contours.push(geo2d::Contour::new(wp.points.clone()));
                    difference(&offset(&poly, lw * 0.5), &offset(&poly, -lw * 0.5))
                } else {
                    // Open wall arc (a trimmed perimeter) — stamp its stroked bead
                    // footprint, so the solid fill stops at it too. The closed-loop
                    // stamp skips open paths, which let fill run over the arc.
                    geo2d::stroke_open(&wp.points, lw * 0.5)
                };
                band = union(&band, &bead);
            }
            difference(&layers[i].polygons, &band)
        };

        // Run the interior fill when there's material inside the innermost wall — OR
        // when there's an enclosed ceiling to bridge. A thin roof closing over a
        // cockpit can have its whole ceiling swallowed by a wide wall stack (4+ walls),
        // leaving `inner` empty; without the ceiling clause Pass 2 is skipped and the
        // ceiling prints as an unfilled hollow (then solid-over-air a layer up) instead
        // of bridging. The ceiling is wall-count-independent (it's the mesh slice), so
        // the enclosed-sheet probe is only paid when `inner` is empty (short-circuit).
        if !inner.is_empty()
            || (i > 0
                && settings.bottom_layers > 0
                && !enclosed_ceiling_sheet(
                    &layers[i].polygons,
                    &phys[i - 1],
                    lw,
                    settings.layer_height_mm
                        * settings.support_overhang_angle_deg.to_radians().tan(),
                    settings.bridge_foothold_mm + sp,
                )
                .is_empty())
        {
            // Unsupported interior, computed in every mode. A span anchored on
            // both ends (a ceiling enclosed by walls) is reliably bridgeable — it's
            // correct bottom-surface printing, so it bridges in every support mode.
            // A NON-anchored overhang (a cantilever past its free edge) can't bridge
            // and falls through to the ordered bottom shell.
            let mut supported_below = Polygons::new();
            let overhang_region = if i > 0 {
                let allowance =
                    settings.layer_height_mm * settings.support_overhang_angle_deg.to_radians().tan();
                supported_below = offset(&phys[i - 1], allowance);
                let oh = difference(&layers[i].polygons, &supported_below);
                let oh = offset(&offset(&oh, -lw), lw); // open: drop slivers
                // The hollow + its landing band bridges as one sheet, so the ends sit
                // on solid the layer below holds up (a foothold), not over air. The
                // bridge reaches one wall-spacing PAST the carve (the foothold band)
                // so the strands just kiss the innermost kept ring, which sits sp
                // beyond the carve.
                let ceiling = enclosed_ceiling_sheet(
                    &layers[i].polygons,
                    &phys[i - 1],
                    lw,
                    allowance,
                    settings.bridge_foothold_mm + sp,
                );
                union(&intersection(&oh, inner), &ceiling)
            } else {
                Polygons::new()
            };

            // Narrow lintels — a door/window top closing as a thin bar — should span as
            // ONE continuous concentric run, not fragment into a solid onion + a
            // try_bridge nub + another onion. Pull the thin connected strips out of
            // (solid ∪ over-air material) here, fill them concentric now (whole loops,
            // walls), and hold them OUT of both the bridge pass and the solid/sparse
            // split below so neither re-fills them.
            //
            // Thinness is judged on the MERGED candidate, not per island. A lone thin
            // bar or shell band stays; but where two thin rings nest closely — a cabin
            // roof closing in concentric steps — their union is a THICK band that would
            // spiral into a concentric onion (the roof bug, which only bit at narrower
            // beads, where each rim slipped under the threshold). Eroding the union by the
            // narrow distance and dilating back (a morphological open) leaves exactly that
            // thick core; subtract it so the core falls back to line fill and only the
            // genuinely thin runs concentric.
            let solid_all = &solid_all_per_layer[i];
            let thin_d = lw * NARROW_FILL_BEADS * 0.5;
            let mut cand = Polygons::new();
            for isl in islands(&union(solid_all, &overhang_region)) {
                if offset(&isl, -thin_d).is_empty() {
                    cand = union(&cand, &isl);
                }
            }
            let narrow_lintel = difference(&cand, &offset(&offset(&cand, -thin_d), thin_d));
            if !narrow_lintel.is_empty() {
                let n0 = paths.len();
                fill_region(
                    &narrow_lintel, InfillPattern::Concentric, sp, 0.0, lw, PathKind::Solid,
                    settings.seam_mode, i, layers[i].z_mm, false, &mut paths,
                );
                // Keep loops whole (these are meant to span their void as continuous
                // beads); drop only microscopic loops (a sub-bead pocket is a dot).
                let tail: Vec<ToolPath> = paths.split_off(n0);
                paths.extend(
                    tail.into_iter().filter(|p| wall_pts_len(&p.points, p.closed) >= lw * 4.0),
                );
            }
            let overhang_region = difference(&overhang_region, &narrow_lintel);

            // Decide per disjoint island: a gap supported on ≥2 sides bridges
            // with straight lines; an unanchored span is left to the normal fill
            // flow. Only the islands that actually got covered are carved out of
            // the solid/sparse split.
            let mut bridged = Polygons::new();
            let bridge_start = paths.len();
            for island in islands(&overhang_region) {
                // Decide on the TRUE unsupported span (so the reach below can't inflate
                // it past max_span), then fill a region grown to reach the walls.
                let Some(angle) = bridge_angle(&island, &supported_below, lw, settings.max_bridge_span_mm)
                else {
                    continue;
                };
                // Grow the span through the void up to the perimeters that ring it, so
                // the strands land ON their anchor (bond, no gap). The grown ring is void
                // that is within reach of BOTH the span and a wall bead, and CLEAR of any
                // other solid/skin — so it fires only where bare perimeters ring the span
                // (a tall wall stack) with nothing but void between them and the span. It
                // is clipped to `ftw` (never OVER a wall) and held a wide margin off other
                // solid, whose own beads grow into this same void (`ftw − inner`) — so the
                // two never fill the same space (no double extrusion). Where the span is
                // ringed by solid instead of bare walls (low wall counts), that margin
                // erases the ring and the fill is byte-identical to `island`.
                let wall_beads = difference(&layers[i].polygons, &ftw);
                // Solid/skin that will actually lay BEADS near this span (its own region
                // minus the span, opened to drop sub-bead slivers — e.g. the thin
                // supported lip a tall-wall reserved face leaves, which prints nothing).
                // The grow keeps a wide margin off it, since those beads grow into this
                // same void; where the span is ringed by real solid (low wall counts) this
                // erases the ring, and where it is ringed by bare perimeters (tall stacks,
                // no solid beads) it leaves the wedge voids free to fill.
                let other_solid = difference(solid_all, &island);
                let solid_beads = offset(&offset(&other_solid, -lw * 0.5), lw * 0.5);
                let ring = difference(
                    &intersection(
                        &intersection(&offset(&island, lw * 5.0), &ftw),
                        &offset(&wall_beads, lw * 5.0),
                    ),
                    &offset(&solid_beads, lw * 3.0),
                );
                let fill_region = union(&island, &ring);
                for seg in infill_lines(&fill_region, angle, lw, true, 0.5, false) {
                    if seg.len() >= 2 {
                        paths.push(ToolPath::new(PathKind::Bridge, false, lw, seg));
                    }
                }
                bridged.contours.extend(fill_region.contours);
            }
            // Per-bead flush: walk each strand's end the last fraction onto the wall it
            // reaches (the same finish every solid/skin line gets). Target the room inside
            // the walls MINUS the solid/skin fill (`solid_all`): a strand end abutting
            // solid is already at that region's edge, so the guard leaves it — it never
            // reaches THROUGH the solid ring to a wall beyond it (which would lay the
            // strand over the solid beads). Where the span is ringed by bare perimeters
            // (tall stacks, solid_all is the span's own region), the ends still reach out
            // across the void to those perimeters.
            let bridge_reach_to = difference(&ftw, solid_all);
            extend_ends_to_wall(&mut paths[bridge_start..], lw * 4.0, &bridge_reach_to);
            // A fully-bridged region (a cabin roof) is all one kind, so its boustrophedon
            // beads stitch into continuous runs exactly like solid — and a bridge wants
            // unbroken flow most of all, since a stop on an unsupported span sags. Same
            // safety: only a turnaround that stays inside the walls connects.
            connect_fill_runs(&mut paths, bridge_start, &ftw, lw * 3.0);

            // `solid_all` was bound above (for the lintel pull). Exclude the lintels
            // from both fills so they aren't traced twice.
            let solid = difference(&difference(solid_all, &bridged), &narrow_lintel);
            let sparse =
                difference(&difference(&difference(inner, solid_all), &bridged), &narrow_lintel);
            let (solid, sparse) = rebalance_solid_sparse(solid, sparse, lw);
            // Sparse infill never belongs on an exposed surface the model is
            // skinning — it leaves holes in what must be a closed skin. Fold any
            // such sliver back into solid (the rebalance demoted it for being thin;
            // thin SOLID at least closes the surface, thin sparse just looks
            // broken). Only where skin is actually laid: with top/bottom layers off
            // (a hollow single-wall print) the surface is intentionally open.
            let (solid, sparse) = if solid_all.is_empty() {
                (solid, sparse)
            } else {
                let on_surf = intersection(&sparse, &surface_per_layer[i]);
                (union(&solid, &on_surf), difference(&sparse, &on_surf))
            };

            // No seam-closing and no wall-edge clip on the skin. Its region is already
            // `inner`, which is bounded by the innermost wall at each location (the
            // inner walls where solid, the outer wall over a surface — surf_inside).
            // The full half-bead fill inset below lands the bead on that boundary:
            // kiss, no overlap. The old seam-closing grew buried Solid into the void
            // hard against the wall — the darker fill that lapped the perimeters
            // (images #22/#23); a leftover void is the gap-fill pass's job, not a
            // reason to flood solid into the wall band.

            // Alternate fill direction per layer for cross-hatching; aligned-lines
            // infill instead keeps one orientation every layer, so its globally
            // anchored lines stack into continuous walls.
            let alt_angle = if i % 2 == 0 { 45.0 } else { 135.0 };
            let pat_angle = |pat: InfillPattern| {
                if pat == InfillPattern::AlignedLines { 45.0 } else { alt_angle }
            };

            if !solid.is_empty() {
                // Split the shell by what shows: skin with open air above (top),
                // skin with nothing printed below it (bottom — the bed face and
                // unsupported undersides that didn't bridge), and the buried
                // rest. Skins print at the visible-surface pace and always
                // monotonic. Coverage tests use the neighbors' full outlines —
                // a surface under nothing but the wall ring above is still
                // hidden — and a light open drops the hair-thin ribbons the
                // coverage differences leave behind.
                let open = |p: Polygons| offset(&offset(&p, -lw * 0.1), lw * 0.1);
                // An EMPTY neighbor layer takes the same exact-clone branch as
                // the grid edge: differencing against nothing and then opening
                // only rounds corners, and the crumbs of `internal` that leaves
                // let absorb_into_solid swallow the whole exposed face. On the
                // shared multi-part grid a part's first/last layers sit mid-grid
                // with empty neighbors, so this is the everyday case there.
                let skin_bottom = if i == 0 || layers[i - 1].polygons.is_empty() {
                    solid.clone()
                } else {
                    open(difference(&solid, &offset(&layers[i - 1].polygons, 0.05)))
                };
                let buried = difference(&solid, &skin_bottom);
                let skin_top = if i + 1 < n && !layers[i + 1].polygons.is_empty() {
                    open(difference(&buried, &offset(&layers[i + 1].polygons, 0.05)))
                } else {
                    buried.clone()
                };
                let mut internal = difference(&buried, &skin_top);
                // Blanket surface→solid merge (opt-in via matching fill pattern):
                // a top/bottom surface that borders buried solid and shares its
                // pattern joins it as ONE region — the shared-angle lines run
                // continuously and no thin surface sliver is left to trace as a
                // lone wall-hugging loop (the deck in image #34). The merged area
                // prints as Solid, so a demoted surface strip trades the slow
                // visible-surface pace for continuous fill. A surface with no solid
                // neighbour stays a surface (see `absorb_into_solid`).
                let skin_top = if settings.top_pattern == settings.solid_pattern {
                    absorb_into_solid(&skin_top, &mut internal, lw)
                } else {
                    skin_top
                };
                let skin_bottom = if settings.bottom_pattern == settings.solid_pattern {
                    absorb_into_solid(&skin_bottom, &mut internal, lw)
                } else {
                    skin_bottom
                };
                // The stretch of solid/skin over low-density sparse spans mostly air
                // between the infill lines below, so it prints as an internal bridge
                // (bridge flow/speed — HIGH flow to make up the missing volume, not
                // under-fill). Rather than carve a separate region (which gives a
                // visible split — two concentric onions), keep ONE region and cut each
                // BEAD at the support boundary below: the part over open sparse steps to
                // bridge, the rest keeps its kind. Applies to the whole shell over
                // sparse — buried solid and visible top skin alike.
                let bridge = if i > 0
                    && settings.infill_density > 0.0
                    && settings.infill_density <= INTERNAL_BRIDGE_MAX_DENSITY
                {
                    // Where the layer below actually carries open sparse infill — not a
                    // pocket the rebalance promoted to solid (those print solid below,
                    // not air). `solid_all` is pre-rebalance, so filter the small
                    // pockets out by the same area floor the promotion uses.
                    let below = difference(&inner_per_layer[i - 1], &solid_all_per_layer[i - 1]);
                    let mut sparse_below = Polygons::new();
                    for isl in islands(&below) {
                        if isl.net_area_mm2() >= SOLID_BELOW_AREA_MM2 {
                            sparse_below.contours.extend(isl.contours);
                        }
                    }
                    let skinnable = union(&internal, &skin_top);
                    let mut b = intersection(&skinnable, &sparse_below);
                    // A shell island that's MOSTLY over open air — a big bridge with only
                    // a thin supported rim — bridges WHOLE: fold the rim into the bridge so
                    // the surface prints as one uniform bridge, not a bridge field with a
                    // solid edge band. A region carrying real supported area stays split.
                    for isl in islands(&skinnable) {
                        let air = intersection(&isl, &sparse_below).net_area_mm2();
                        let held = difference(&isl, &sparse_below).net_area_mm2();
                        if air > 3.0 * held {
                            b = union(&b, &isl);
                        }
                    }
                    b
                } else {
                    Polygons::new()
                };
                // Everything NOT over open sparse — the other side of each bead's cut.
                let supported = if bridge.is_empty() {
                    Polygons::new()
                } else {
                    difference(&layers[i].polygons, &bridge)
                };
                let regions = [
                    (skin_bottom, PathKind::BottomSkin, true),
                    (internal, PathKind::Solid, settings.monotonic_solid),
                    (skin_top, PathKind::TopSkin, true),
                ];
                // The room a solid/skin bead may grow or reach into: inside the walls
                // (`ftw`) but NEVER the bridged span. A bridge already fills that span
                // (and grows onto the walls that ring it), so a solid line-end reaching
                // for the same wall, or a skin sheet grown into the same void, would lay
                // a second bead over the bridge. Holding both the void-grow and the
                // wall-reach out of `bridged` keeps the two fills from colliding.
                let ftw_ex = if bridged.is_empty() { ftw.clone() } else { difference(&ftw, &bridged) };
                // A wide solid region gets NO concentric perimeter loop: alongside a
                // wall the loop just doubles the perimeter bead (walls + solid in the
                // same band, the user's complaint). Its fill pattern extends out to
                // where the loop would have sat and butts the wall directly — one bead,
                // then the wall (the fill bead's half-width laps the wall to bond). A
                // region too thin to line-fill (a narrow solid bar, < ~2 beads) is the
                // exception: trace it as a single boundary loop, the only way to fill
                // it — and such a sliver is bounded by its own walls, not doubling one.
                for (region, kind, monotone) in regions {
                    if region.is_empty() {
                        continue;
                    }
                    // A sliver too thin to line-fill (a residual bar the lintel pull
                    // above didn't claim — a mixed wide+narrow island) traces as a single
                    // boundary loop, the only way to fill it; sub-bead nubs are dropped
                    // (their own walls cover them).
                    if offset(&region, -lw).is_empty() {
                        let loop_region = offset(&region, -(lw * 0.5 - ov * 0.5));
                        for c in loop_region.contours {
                            if c.points.len() < 3 {
                                continue;
                            }
                            let m = c.points.len();
                            let perim: f64 =
                                (0..m).map(|j| pt_dist_mm(c.points[j], c.points[(j + 1) % m])).sum();
                            if perim >= lw * 4.0 {
                                let points = place_seam(c.points, settings.seam_mode, i);
                                paths.push(ToolPath::new(kind, true, lw, points));
                            }
                        }
                        continue;
                    }
                    // Extend the solid sheet into the wall gap it borders so it reaches
                    // the perimeters flush (`inner` stops a bead short, the void ring).
                    // Grow ONLY into that ring — never into `inner` (the neighbouring
                    // sparse/solid), so no overlap — and the thin-check above already
                    // ran on the original region, so this never spawns a loop.
                    // A surface skin is ringed by walls: grow it into the void it
                    // borders, then clip to the SMOOTH wall kiss-line (half a bead
                    // inside the wall edge). Clipping to that smooth line — not eroding
                    // the jagged grown boundary — lands the sheet's bead flush on the
                    // perimeters, no gap. Buried solid borders sparse, not walls, so it
                    // keeps the plain half-bead inset for the kiss there.
                    // The skin is generated short of the walls, so first bring it close:
                    // grow it into the void it borders (never into buried infill). Then
                    // the per-bead pass below walks each line-end the last fraction to
                    // the wall. Buried solid borders sparse, not walls, so it keeps the
                    // half-bead inset for a clean kiss there.
                    // Effective fill pattern. A solid/skin region that RINGS a cavity
                    // (every island has a hole) fills Concentric — loops parallel to the
                    // walls it wraps, reading as a continuation of the wall sweep — instead
                    // of scanlines that dead-end at the wall (the teal cavity fill in the
                    // preview becoming rings). A simply-connected region keeps its
                    // configured pattern (a concentric loop there just doubles the
                    // perimeter — the anti-doubling policy above). Skins ring holes readily
                    // (a top surface around a window), so this catches them too.
                    let base_pattern = match kind {
                        PathKind::TopSkin => settings.top_pattern,
                        PathKind::BottomSkin => settings.bottom_pattern,
                        _ => settings.solid_pattern,
                    };
                    // Concentric only for a genuinely NARROW ring-band, where a scanline
                    // would be short stubs dead-ending at both the wall and the hole. A
                    // WIDE flat solid/surface that merely has a hole or two (the Benchy
                    // deck) line-fills cleanly, so it keeps the configured pattern —
                    // concentric there reads as busy loops the user never selected, and
                    // (is_annular being all-or-nothing over islands) flips with wall count.
                    // Narrow = the region vanishes when eroded ~a ring-band's half-width.
                    let pattern = if is_annular(&region) && is_narrow_band(&region, lw) {
                        InfillPattern::Concentric
                    } else {
                        base_pattern
                    };
                    // Prep the fill for the chosen pattern. A skin is generated short of the
                    // walls: grow it into the void ring it borders (never into buried
                    // infill) so the outer bead — concentric's outer loop, or the per-bead
                    // reach below on open line-ends — lands flush on the perimeters.
                    // Concentric loops are closed (the reach can't pull them), so hand the
                    // full region: concentric's own lw/2 inset lands the outer loop flush on
                    // the wall and the sparse boundary. Buried line-fill solid keeps the
                    // plain half-bead inset for a clean kiss against the sparse it borders.
                    let fill = if matches!(kind, PathKind::TopSkin | PathKind::BottomSkin) {
                        let void = difference(&ftw_ex, inner);
                        union(&region, &intersection(&offset(&region, lw * 1.5), &void))
                    } else if pattern == InfillPattern::Concentric {
                        region.clone()
                    } else {
                        offset(&region, -lw * 0.5)
                    };
                    if !fill.is_empty() {
                        // An unsupported underside printed as bottom shell prints for
                        // the best chance of landing on what's already down: line/grid
                        // patterns go nearest-neighbour (each segment lands beside the
                        // last) instead of a monotonic sweep; concentric is reordered
                        // outer→in downstream. The bed face (i == 0) keeps its sweep.
                        let monotone = if kind == PathKind::BottomSkin && i > 0 {
                            false
                        } else {
                            monotone
                        };
                        let n0 = paths.len();
                        fill_region(
                            &fill, pattern, sp, pat_angle(pattern), lw, kind,
                            settings.seam_mode, i, layers[i].z_mm, monotone, &mut paths,
                        );
                        // PER-BEAD reach: lengthen each line-end along its OWN direction
                        // until it meets the wall edge (`ftw`) — the few tenths it needs.
                        // Ends that border a wall extend; ends facing other infill don't
                        // move. Applies to every solid kind, not just skins. (Targets full
                        // `ftw`, NOT ftw−bridged: a bridge hole in the target would give a
                        // solid end a near boundary to reach TO, pulling it onto the span.)
                        extend_ends_to_wall(&mut paths[n0..], lw * 3.0, &ftw);
                        // Cut each bead at the support boundary: the stretch over open
                        // sparse becomes an internal bridge (bridge flow/speed), the rest
                        // keeps its kind — one continuous fill that steps at the boundary,
                        // not two regions. skin_bottom is over air (a bottom shell), not
                        // sparse, so it's left out.
                        // Connect-when-safe, BEFORE the bridge split: stitch the
                        // boustrophedon beads of a solid/skin surface into continuous
                        // runs so the flow never stops at a line end — only where the
                        // turnaround stays inside the walls (a hole or concavity keeps
                        // its travel). Doing it first matters: a bead that merely crosses
                        // an unsupported gap stays one continuous run, and the split below
                        // then clips the over-air span out of that run — the supported
                        // arcs keep the turnarounds joining consecutive passes, instead of
                        // the split fragmenting the surface before connect ever sees it.
                        // Sparse is left untouched (its ends aren't visible and connecting
                        // would add material and defeat the sparseness).
                        if matches!(kind, PathKind::Solid | PathKind::TopSkin | PathKind::BottomSkin) {
                            connect_fill_runs(&mut paths, n0, &ftw, sp * 3.0);
                        }
                        if !bridge.is_empty() && matches!(kind, PathKind::Solid | PathKind::TopSkin) {
                            split_bridge_beads(&mut paths, n0, &bridge, &supported);
                            // A short solid arc threaded between two bridge spans is a
                            // turn-around landing on the wall ring, not a real solid
                            // patch — reclassify it to bridge so a mostly-unsupported
                            // surface prints as one uniform bridge, not a bridge field
                            // with a solid edge band. The cap scales with the wall ring
                            // (the supported crossing such a connector spans).
                            let connector_cap = lw * (2.0 * settings.wall_count as f64 + 4.0);
                            merge_bridge_connectors(&mut paths[n0..], connector_cap);
                            // The split cut each connected bead at the support boundary,
                            // so the over-air middles are now separate InternalBridge arcs.
                            // Stitch them (and the surviving solid arcs) back into
                            // continuous runs — a bridge surface should print without
                            // stopping the flow at every line, same as solid.
                            connect_fill_runs(&mut paths, n0, &ftw, sp * 3.0);
                        }
                    }
                }
            }
            if settings.infill_density > 0.0 && !sparse.is_empty() {
                let spacing = sp / settings.infill_density;
                // Grow for the interior bond, then clip to a half-bead inside `inner`
                // (the infill region's own boundary = the innermost wall edge at each
                // location) so sparse beads kiss the wall instead of poking across the
                // inner-wall ring. Interior solid bonds, away from that edge, keep ov.
                let grown = if ov > 0.0 { offset(&sparse, ov) } else { sparse.clone() };
                let sparse_fill = if settings.wall_count > 0 {
                    intersection(&grown, &offset(inner, -lw * 0.5))
                } else {
                    grown
                };
                let n0 = paths.len();
                fill_region(
                    &sparse_fill, settings.sparse_pattern, spacing, pat_angle(settings.sparse_pattern), lw, PathKind::Infill,
                    settings.seam_mode, i, layers[i].z_mm, false, &mut paths,
                );
                // Same per-bead reach: a sparse line that runs into a WALL anchors to it;
                // one facing other infill doesn't move. Target the non-solid part of the
                // inside-walls region (ftw minus everything solid-filled) so an end facing
                // a solid band stops AT that band — not dragged through it to the wall,
                // which over-printed sparse across the solid (a line crossing the onion).
                let sparse_anchor =
                    difference(&difference(&difference(&ftw, &solid), &narrow_lintel), &bridged);
                extend_ends_to_wall(&mut paths[n0..], lw * 3.0, &sparse_anchor);
            }
        }

        // Capstone: spiralize each supported island's inner walls AND the concentric
        // fill they ring as ONE nested family, so the nozzle sweeps from the outer
        // wall straight through the cavity fill as a single continuous stroke — the
        // wall part keeps Perimeter speed/flow, the fill part its own (a SegAttr per
        // segment). Runs here, after fill exists, on the closed wall + fill rings.
        if settings.wall_count >= 1 && !settings.spiral_vase {
            paths = spiralize_shells(paths, &layers[i].polygons, lw, sp, settings.seam_mode, i);
        }

        // Ironing: a slow, near-zero-flow boustrophedon pass over surfaces with
        // open air above, melting ridges into a smooth plane. Kept in order and
        // forced after everything else on the layer.
        if settings.ironing && !inner.is_empty() {
            let exposed = difference(inner, &coverage(&inner_per_layer, i, 1, 1, n));
            let iron = offset(&exposed, -lw * 0.25);
            if !iron.is_empty() {
                let spacing = settings.ironing_spacing_mm.max(0.05);
                // Island by island, so the pass finishes one surface before
                // gliding to the next (ironing skips travel ordering entirely).
                for island in islands(&iron) {
                    for seg in crate::fill::infill_lines(&island, 45.0, spacing, true, 0.5, false) {
                        let mut p = ToolPath::new(PathKind::Ironing, false, spacing, seg);
                        p.flow = settings.ironing_flow.clamp(0.0, 1.0);
                        paths.push(p);
                    }
                }
            }
        }

        LayerPlan {
            index: i,
            print_z_mm: layers[i].print_z_mm,
            height_mm: layers[i].height_mm,
            paths,
            travels: Vec::new(), // filled by emit::plan_travels once paths are final
            // Simplify (not offset) so the visibility graph stays small while
            // topology is preserved — an inward offset can pinch thin necks into
            // separate islands that then can't be combed.
            outline: simplify(&layers[i].polygons, 0.1),
            speed_scale: 1.0,
            planned_temp_c: None,
            temp_command_c: None,
        }
        })
        .collect();

    plans
}

/// Generate removable grid support under overhangs. For each layer, the overhang
/// is the region not over the layer below within a printable cantilever; this is
/// projected downward and the support area (minus the part + clearance) is filled
/// with sparse lines as `PathKind::Support`.
///
/// `layers` is the part being supported; `phys` is the per-layer union of ALL
/// parts — overhangs resting on another part need no support, and a support
/// column stops on (and keeps clearance from) whatever part it meets on the
/// way down.
fn add_supports(plans: &mut [LayerPlan], layers: &[Layer], phys: &[Polygons], settings: &Settings) {
    // Only Grid mode adds support structure below; None leaves overhangs as-is.
    if settings.support_mode != SupportMode::Grid {
        return;
    }
    let n = layers.len();
    if n == 0 {
        return;
    }
    let lw = settings.line_width_mm;
    // A region is supported if within this of the layer below. Angle is from
    // vertical, so the printable horizontal cantilever per layer is h·tan(angle).
    let allowance =
        settings.layer_height_mm * settings.support_overhang_angle_deg.to_radians().tan();
    let clearance = settings.support_xy_clearance_mm;

    // Per-layer overhang, with thin slivers removed (a one-bead ledge is fine).
    let overhang: Vec<Polygons> = (0..n)
        .into_par_iter()
        .map(|i| {
            if i == 0 {
                return Polygons::new();
            }
            let supported = offset(&phys[i - 1], allowance);
            let oh = difference(&layers[i].polygons, &supported);
            offset(&offset(&oh, -lw), lw) // morphological open
        })
        .collect();

    // Project downward: support at layer i holds overhangs accumulated from above,
    // minus the part (+clearance). Where the part is, the column rests and stops.
    // A z-gap of `gap` empty layers under each overhang aids removal, and the top
    // `iface` support layers are printed solid for a smoother overhang underside.
    let sp = config::bead_spacing_mm(lw, settings.layer_height_mm);
    let spacing = sp / settings.support_density.clamp(0.02, 1.0);
    let gap = settings.support_z_gap_layers;
    let iface = settings.support_interface_layers;
    let mut accum = Polygons::new();
    for i in (0..n).rev() {
        let blocked = offset(&phys[i], clearance);
        let here = difference(&accum, &blocked);
        if !here.is_empty() {
            let angle = if i % 2 == 0 { 0.0 } else { 90.0 };
            // Interface = the top `iface` support layers below an overhang (its top
            // sits `gap` layers under the overhang). Those layers print solid.
            let mut iface_region = Polygons::new();
            for j in (i + 1 + gap)..=(i + gap + iface).min(n - 1) {
                iface_region = union(&iface_region, &overhang[j]);
            }
            let iface_here = intersection(&here, &iface_region);
            let body_here = difference(&here, &iface_here);
            if !body_here.is_empty() {
                add_support_region(&mut plans[i].paths, &body_here, spacing, angle, lw,
                    settings.seam_mode, i, layers[i].z_mm);
            }
            if !iface_here.is_empty() {
                add_support_region(&mut plans[i].paths, &iface_here, sp, angle, lw,
                    settings.seam_mode, i, layers[i].z_mm);
            }
        }
        accum = difference(&accum, &phys[i]);
        // Defer adding this layer's overhang by `gap` layers so the support tops
        // out `gap` layers below it (leaving the removal gap).
        if i + gap < n {
            accum = union(&accum, &overhang[i + gap]);
        }
    }
}

/// Draw one support region: a perimeter loop first — so thin and tiny sections
/// become continuous, self-supporting tubes and the interior fill anchors to an
/// edge instead of floating as one-direction lines — then the pattern fill
/// tucked a line-width inside it.
fn add_support_region(
    paths: &mut Vec<ToolPath>,
    region: &Polygons,
    spacing: f64,
    angle: f64,
    lw: f64,
    seam_mode: SeamMode,
    layer_index: usize,
    z_mm: f64,
) {
    let perim = offset(region, -lw * 0.5);
    for c in perim.contours {
        if c.points.len() >= 3 {
            let points = place_seam(c.points, seam_mode, layer_index);
            paths.push(ToolPath::new(PathKind::Support, true, lw, points));
        }
    }
    // Inset by a full line so the fill meets the perimeter rather than doubling
    // it; a section thinner than that is covered by the perimeter bead alone.
    let inner = offset(region, -lw);
    if !inner.is_empty() {
        fill_region(&inner, InfillPattern::Lines, spacing, angle, lw,
            PathKind::Support, seam_mode, layer_index, z_mm, false, paths);
    }
}

/// Split a region into its disjoint islands (each CCW outer plus the holes inside
/// it), so each can be handled — bridged or arc-filled — independently.
fn islands(polys: &Polygons) -> Vec<Polygons> {
    let outers: Vec<&Contour> = polys.contours.iter().filter(|c| c.points.len() >= 3 && c.is_ccw()).collect();
    let holes: Vec<&Contour> = polys.contours.iter().filter(|c| c.points.len() >= 3 && !c.is_ccw()).collect();
    outers
        .iter()
        .map(|o| {
            let mut isl = Polygons::new();
            isl.push((*o).clone());
            for h in &holes {
                if o.contains(h.points[0]) {
                    isl.push((*h).clone());
                }
            }
            isl
        })
        .collect()
}

/// A region "rings a cavity": every island has a hole (a CW inner contour).
/// Such a region fills cleanest with Concentric loops that parallel the walls
/// wrapping it — a continuation of the wall sweep — rather than scanlines that
/// dead-end at the wall. A simply-connected island is NOT annular: a concentric
/// loop there would just double its perimeter, so a region with any such island
/// keeps line fill (the deliberate anti-doubling policy). Conservative by design
/// — "for rings only" — so a wide solid is never handed a wall-doubling loop.
fn is_annular(region: &Polygons) -> bool {
    let isls = islands(region);
    !isls.is_empty()
        && isls
            .iter()
            .all(|isl| isl.contours.iter().any(|c| c.points.len() >= 3 && !c.is_ccw()))
}

/// A region is a NARROW ring-band (worth filling concentric) when it vanishes once
/// eroded by a ring-band's half-width (`NARROW_FILL_BEADS`) — i.e. it's thin enough
/// everywhere that a scanline across it would be short stubs dead-ending at the wall
/// and the hole. A WIDE flat solid that merely has a hole or two (a deck) survives
/// the erosion, so it keeps its configured line/grid fill instead of busy loops.
fn is_narrow_band(region: &Polygons, lw: f64) -> bool {
    offset(region, -lw * NARROW_FILL_BEADS).is_empty()
}

/// Rebalance the solid/sparse split at the island level, in both directions:
///
/// - **Junk solid → sparse.** The top/bottom coverage booleans shed solid
///   islands too small or too thin to print: the boundary loop degenerates to
///   a micro hairpin dab, or fits nowhere at all and leaves a silent void.
///   Their area joins the sparse region instead, so the space still belongs
///   to a fill pass rather than vanishing.
/// - **Tiny sparse pockets → solid** (Prusa's solid-infill-below-area
///   behavior): a few lonely 15% lines print badly; pour the pocket solid.
///
/// "Junk" means smaller than ~2×2 beads or nowhere wider than one line width
/// (Cura's skin-removal-width default is one line width too). The same floor
/// exempts pockets from promotion, so a demoted crumb can't bounce straight
/// back to solid — unless it merged into a bigger printable pocket, where
/// pouring it solid is the right outcome anyway.
fn rebalance_solid_sparse(solid: Polygons, sparse: Polygons, lw: f64) -> (Polygons, Polygons) {
    let junk =
        |island: &Polygons| island.net_area_mm2() < 4.0 * lw * lw || offset(island, -lw * 0.5).is_empty();
    let mut solid = solid;
    let mut sparse = sparse;
    if !solid.is_empty() {
        let mut keep = Polygons::new();
        let mut demote = Polygons::new();
        for island in islands(&solid) {
            let target = if junk(&island) { &mut demote } else { &mut keep };
            target.contours.extend(island.contours);
        }
        if !demote.contours.is_empty() {
            sparse = union(&sparse, &demote);
            solid = keep;
        }
    }
    if !sparse.is_empty() {
        let mut keep = Polygons::new();
        let mut promote = Polygons::new();
        for island in islands(&sparse) {
            let target = if island.net_area_mm2() < SOLID_BELOW_AREA_MM2 && !junk(&island) {
                &mut promote
            } else {
                &mut keep
            };
            target.contours.extend(island.contours);
        }
        if !promote.contours.is_empty() {
            solid = union(&solid, &promote);
            sparse = keep;
        }
    }
    (solid, sparse)
}

/// Blanket surface→solid merge. Each surface island that borders the buried
/// `solid` is folded into it, so the two (same-pattern) regions fill as ONE
/// continuous sweep — no thin surface strip left over to trace as a lone
/// wall-hugging loop, and the shared-angle lines run straight across the seam.
/// Islands with NO solid neighbour (the bed face, a topmost-layer roof, a lone
/// cantilever) have nothing to merge into, so they're returned as the kept
/// surface and keep their visible-surface treatment. The caller gates this on
/// the surface and solid fill patterns actually matching.
fn absorb_into_solid(skin: &Polygons, solid: &mut Polygons, lw: f64) -> Polygons {
    let mut kept = Polygons::new();
    for isl in islands(skin) {
        // skin and solid partition the same blob, so a real neighbour shares an
        // edge — a half-bead dilation of an adjacent island always laps `solid`.
        if intersection(&offset(&isl, lw * 0.5), solid).is_empty() {
            kept.contours.extend(isl.contours);
        } else {
            *solid = union(solid, &isl);
        }
    }
    kept
}

/// Thin-wall trim: where two wall beads land much closer than the stadium spacing
/// they double up — the nozzle retraces the same line, blobbing and over-extruding.
/// This happens in a neck too thin for the perimeters it nominally fits (a hole
/// beside the outline — the chimney next to the cabin edge). Keep the outer walls
/// and the larger inner loops whole; trim the doubled run out of each smaller inner
/// (`Perimeter`) wall, leaving the neck to a single bead. The threshold sits well
/// below `sp`, so normal concentric spacing (≈ sp) is never touched.
fn trim_thin_walls(walls: &mut Vec<ToolPath>, lw: f64, sp: f64) {
    let thresh = sp * 0.6;
    // Pull each surviving arc tip back ~one bead past the cut, so it ends clear of
    // the wall it was doubling instead of at the threshold boundary (~35% overlap).
    let margin = lw;
    let trimmable = |k: PathKind| k == PathKind::Perimeter;
    if !walls.iter().any(|w| trimmable(w.kind)) {
        return;
    }
    // Non-trimmable walls (outer + overhang) and larger inner loops are processed
    // first and kept whole; smaller inner loops are trimmed against all of them.
    let mut order: Vec<usize> = (0..walls.len()).collect();
    order.sort_by(|&a, &b| {
        trimmable(walls[a].kind).cmp(&trimmable(walls[b].kind)).then(
            wall_pts_len(&walls[b].points, walls[b].closed)
                .partial_cmp(&wall_pts_len(&walls[a].points, walls[a].closed))
                .unwrap_or(std::cmp::Ordering::Equal),
        )
    });
    // Kept centerlines (closed loops carry their closing segment) to test against.
    let mut kept: Vec<Vec<Point>> = Vec::new();
    let mut out: Vec<(usize, Vec<ToolPath>)> = Vec::with_capacity(walls.len());
    for &i in &order {
        let w = &walls[i];
        let arcs = if trimmable(w.kind) { trim_loop(&w.points, w.closed, &kept, thresh, margin) } else { None };
        match arcs {
            // Untouched (or not trimmable) → keep the original path, seam intact.
            None => {
                kept.push(seq(&w.points, w.closed));
                out.push((i, vec![w.clone()]));
            }
            Some(arcs) => {
                let mut tps = Vec::new();
                for (pts, closed) in arcs {
                    if wall_pts_len(&pts, closed) < lw * 2.0 {
                        continue; // drop sub-bead fragments
                    }
                    kept.push(seq(&pts, closed));
                    let mut tp = w.clone();
                    tp.points = pts;
                    tp.closed = closed;
                    tps.push(tp);
                }
                out.push((i, tps));
            }
        }
    }
    out.sort_by_key(|(i, _)| *i);
    *walls = out.into_iter().flat_map(|(_, v)| v).collect();
}

fn wall_pts_len(pts: &[Point], closed: bool) -> f64 {
    let mut l: f64 = pts.windows(2).map(|w| pt_dist_mm(w[0], w[1])).sum();
    if closed && pts.len() > 2 {
        l += pt_dist_mm(*pts.last().unwrap(), pts[0]);
    }
    l
}

/// Point sequence for distance tests: append the closing point for closed loops so
/// the closing segment counts too.
fn seq(pts: &[Point], closed: bool) -> Vec<Point> {
    let mut v = pts.to_vec();
    if closed && pts.len() > 2 {
        v.push(pts[0]);
    }
    v
}

/// Trim a wall loop where its bead doubles a kept wall. `None` = nothing trimmed
/// (caller keeps the loop as-is, seam intact); `Some(arcs)` = the surviving open
/// runs. A point is "doubled" when within `thresh` of any kept centerline; isolated
/// single hits (numerical noise at the spacing limit) are ignored — only runs of
/// ≥2 are cut. The cut region is then grown by `margin` of arc length at each end,
/// so the surviving arc tips pull back from the wall they were doubling (less tip
/// overlap) instead of stopping right at the threshold boundary.
fn trim_loop(
    pts: &[Point],
    closed: bool,
    kept: &[Vec<Point>],
    thresh: f64,
    margin: f64,
) -> Option<Vec<(Vec<Point>, bool)>> {
    let n = pts.len();
    if n < 2 {
        return None;
    }
    let blocked: Vec<bool> = pts.iter().map(|&p| min_dist_to_polylines(p, kept) < thresh).collect();
    // Cut a point only if it AND a neighbour are blocked (drop isolated hits).
    let cut0: Vec<bool> = (0..n)
        .map(|k| {
            if !blocked[k] {
                return false;
            }
            let prev = if closed { blocked[(k + n - 1) % n] } else { k > 0 && blocked[k - 1] };
            let next = if closed { blocked[(k + 1) % n] } else { k + 1 < n && blocked[k + 1] };
            prev || next
        })
        .collect();
    if !cut0.iter().any(|&c| c) {
        return None;
    }
    if cut0.iter().all(|&c| c) {
        return Some(Vec::new()); // entirely doubled — drop the loop
    }
    // Grow the cut by `margin` of arc length around each cut point.
    let cut: Vec<bool> = if margin <= 0.0 {
        cut0
    } else {
        let mut cum = vec![0.0_f64; n];
        for k in 1..n {
            cum[k] = cum[k - 1] + pt_dist_mm(pts[k - 1], pts[k]);
        }
        let total = cum[n - 1] + if closed { pt_dist_mm(pts[n - 1], pts[0]) } else { 0.0 };
        let cut_idx: Vec<usize> = (0..n).filter(|&j| cut0[j]).collect();
        (0..n)
            .map(|k| {
                cut0[k]
                    || cut_idx.iter().any(|&j| {
                        let d = (cum[k] - cum[j]).abs();
                        let d = if closed { d.min(total - d) } else { d };
                        d <= margin
                    })
            })
            .collect()
    };
    if cut.iter().all(|&c| c) {
        return Some(Vec::new());
    }
    // Maximal runs of kept (non-cut) points. For a closed loop, start at a cut
    // point so no run wraps the seam.
    let mut arcs: Vec<(Vec<Point>, bool)> = Vec::new();
    let mut cur: Vec<Point> = Vec::new();
    let push = |cur: &mut Vec<Point>, arcs: &mut Vec<(Vec<Point>, bool)>| {
        if cur.len() >= 2 {
            arcs.push((std::mem::take(cur), false));
        } else {
            cur.clear();
        }
    };
    if closed {
        let start = (0..n).find(|&k| cut[k]).unwrap();
        for step in 1..=n {
            let idx = (start + step) % n;
            if cut[idx] {
                push(&mut cur, &mut arcs);
            } else {
                cur.push(pts[idx]);
            }
        }
    } else {
        for k in 0..n {
            if cut[k] {
                push(&mut cur, &mut arcs);
            } else {
                cur.push(pts[k]);
            }
        }
    }
    push(&mut cur, &mut arcs);
    Some(arcs)
}

fn min_dist_to_polylines(p: Point, polylines: &[Vec<Point>]) -> f64 {
    let mut best = f64::INFINITY;
    for pl in polylines {
        for w in pl.windows(2) {
            best = best.min(pt_seg_dist_mm(p, w[0], w[1]));
        }
    }
    best
}

fn pt_seg_dist_mm(p: Point, a: Point, b: Point) -> f64 {
    let (px, py, ax, ay, bx, by) = (p.x_mm(), p.y_mm(), a.x_mm(), a.y_mm(), b.x_mm(), b.y_mm());
    let (dx, dy) = (bx - ax, by - ay);
    let l2 = dx * dx + dy * dy;
    let t = if l2 < 1.0e-12 { 0.0 } else { (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0) };
    (px - (ax + t * dx)).hypot(py - (ay + t * dy))
}

/// The point on segment a→b nearest `p` (the projection, clamped to the segment).
fn nearest_point_on_seg(p: Point, a: Point, b: Point) -> Point {
    let (ax, ay, bx, by) = (a.x_mm(), a.y_mm(), b.x_mm(), b.y_mm());
    let (dx, dy) = (bx - ax, by - ay);
    let l2 = dx * dx + dy * dy;
    if l2 < 1.0e-12 {
        return a;
    }
    let t = (((p.x_mm() - ax) * dx + (p.y_mm() - ay) * dy) / l2).clamp(0.0, 1.0);
    Point::from_mm(ax + dx * t, ay + dy * t)
}

/// Single-region bridge split: cut each just-filled bead at the support boundary.
/// The stretch over `bridge` (open sparse below) is re-tagged `InternalBridge`
/// (bridge flow/speed); the stretch over `supported` keeps the bead's own kind.
/// A bead wholly on one side stays one path (closed loops stay closed); a bead
/// that crosses is split into collinear arcs that abut at the boundary — so the
/// fill stays one continuous onion/line-set that just steps where support changes.
/// After the bridge split, a SHORT solid arc that meets a bridge arc at EITHER end is
/// a turn-around or anchor threading off a bridge span — a landing on the wall ring,
/// not a genuine solid patch. On a mostly-unsupported surface those leave a solid band
/// (or corner stubs) at the edges; reclassify them to bridge so the span prints as one
/// uniform bridge (the bead still lands on the wall ring, where bridge flow bonds
/// fine). A real solid region's interior arcs touch no bridge at all, or run wider
/// than `cap`, so they're left alone.
fn merge_bridge_connectors(paths: &mut [ToolPath], cap: f64) {
    let mut ends: Vec<Point> = Vec::new();
    for tp in paths.iter() {
        if tp.kind == PathKind::InternalBridge && tp.points.len() >= 2 {
            ends.push(tp.points[0]);
            ends.push(*tp.points.last().unwrap());
        }
    }
    if ends.is_empty() {
        return;
    }
    let at_bridge = |p: Point| ends.iter().any(|&q| pt_dist_mm(p, q) < 0.05);
    for tp in paths.iter_mut() {
        if matches!(tp.kind, PathKind::Solid | PathKind::BottomSkin | PathKind::TopSkin)
            && !tp.closed
            && tp.points.len() >= 2
            && tp.points.windows(2).map(|w| pt_dist_mm(w[0], w[1])).sum::<f64>() <= cap
            && (at_bridge(tp.points[0]) || at_bridge(*tp.points.last().unwrap()))
        {
            tp.kind = PathKind::InternalBridge;
        }
    }
}

fn split_bridge_beads(paths: &mut Vec<ToolPath>, start: usize, bridge: &Polygons, supported: &Polygons) {
    let Some(bb) = bridge.bounds() else { return };
    let (bx0, by0, bx1, by1) = (bb.min.x_mm(), bb.min.y_mm(), bb.max.x_mm(), bb.max.y_mm());
    let beads: Vec<ToolPath> = paths.split_off(start);
    for bead in beads {
        if bead.points.len() < 2 {
            paths.push(bead);
            continue;
        }
        // AABB reject: a bead clear of the bridge's bounds can't cross it — skip the
        // (expensive) clip and keep it whole.
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in &bead.points {
            x0 = x0.min(p.x_mm());
            y0 = y0.min(p.y_mm());
            x1 = x1.max(p.x_mm());
            y1 = y1.max(p.y_mm());
        }
        if x1 < bx0 || x0 > bx1 || y1 < by0 || y0 > by1 {
            paths.push(bead);
            continue;
        }
        let mut tup: Vec<(f64, f64)> = bead.points.iter().map(|p| (p.x_mm(), p.y_mm())).collect();
        if bead.closed {
            tup.push(tup[0]); // close the ring for clipping
        }
        let over_air = crate::fill::clip_polylines(vec![tup.clone()], bridge);
        let span = |a: &[Point]| -> f64 { a.windows(2).map(|w| pt_dist_mm(w[0], w[1])).sum() };
        // Gate: only bridge a run that's actually long enough to sag (≥ 2 mm). If no
        // over-air arc qualifies, don't split at all — leave the bead whole solid.
        if !over_air.iter().any(|a| span(a) >= MIN_BRIDGE_SPAN_MM) {
            paths.push(bead);
            continue;
        }
        let over_solid = crate::fill::clip_polylines(vec![tup], supported);
        // Inverse gate: if the supported section is itself short (< 2 mm), the bead
        // is mostly air — a sub-2 mm anchor can't hold a bridge, and a tiny solid
        // nub between bridge runs is pointless fragmentation. Treat the WHOLE bead
        // as unsupported (one InternalBridge bead, no split).
        let solid_total: f64 = over_solid.iter().map(|a| span(a)).sum();
        if solid_total < MIN_BRIDGE_SPAN_MM {
            let mut tp = bead;
            tp.kind = PathKind::InternalBridge;
            paths.push(tp);
            continue;
        }
        for (arcs, base_kind) in [(over_solid, bead.kind), (over_air, PathKind::InternalBridge)] {
            for arc in arcs {
                if arc.len() < 2 {
                    continue;
                }
                // A short over-air arc (< 2 mm) the bead carries fine — keep it the
                // bead's own kind rather than switching to bridge.
                let kind = if base_kind == PathKind::InternalBridge && span(&arc) < MIN_BRIDGE_SPAN_MM {
                    bead.kind
                } else {
                    base_kind
                };
                let closed = arc.len() >= 4 && pt_dist_mm(arc[0], *arc.last().unwrap()) < 1.0e-3;
                let mut points = arc;
                if closed {
                    points.pop();
                }
                let mut tp = bead.clone();
                tp.kind = kind;
                tp.points = points;
                tp.closed = closed;
                paths.push(tp);
            }
        }
    }
}

/// Even-odd containment over a polygon set (outers + holes).
fn in_polys(polys: &Polygons, p: Point) -> bool {
    let mut inside = false;
    for c in &polys.contours {
        if c.contains(p) {
            inside = !inside;
        }
    }
    inside
}

/// Mark wall stretches inside `unsupported` (the part of this layer with no
/// material below) as `OverhangWall`: a bead whose centerline is past the
/// previous outline hangs by more than half its width, and prints badly at
/// wall speed — it gets the overhang speed and bridge-grade cooling instead.
/// Loops are split into consecutive open pieces (no travel in between), with
/// runs shorter than ~2 line widths merged into their neighbour so the speed
/// doesn't chatter at classification borders.
fn slow_overhanging_walls(walls: Vec<ToolPath>, below: &Polygons, lw: f64) -> Vec<ToolPath> {
    let min_run_mm = lw * 2.0;
    // How far past the support below each bead sits, as a 0→1 overhang degree:
    // 0 on solid, rising to 1 a full line width out (fully airborne). Measured as
    // the real distance to the support boundary and quantized to 0.1, so the
    // graduated speed eases in ten small steps instead of the old three hard
    // bands. Each hard band was a speed jump that — at constant pressure advance —
    // printed a visible flow line, worst where a closing arch slows hardest. Any
    // bead past the edge gets at least 0.1 so it's still flagged overhanging.
    // Distance is computed only for the few beads actually outside support;
    // supported beads short-circuit to 0 (the common case, and cheaper than the
    // two polygon offsets this replaces).
    let degree_at = |p: Point| -> f32 {
        if in_polys(below, p) {
            return 0.0;
        }
        let mut best = f64::INFINITY;
        for c in &below.contours {
            let m = c.points.len();
            if m < 2 {
                continue;
            }
            for j in 0..m {
                best = best.min(pt_seg_dist_mm(p, c.points[j], c.points[(j + 1) % m]));
            }
        }
        let frac = (best / lw).clamp(0.0, 1.0);
        ((frac * 10.0).round().max(1.0) / 10.0) as f32
    };
    let mut out = Vec::with_capacity(walls.len());
    for path in walls {
        if !matches!(path.kind, PathKind::ExternalPerimeter | PathKind::Perimeter) || path.points.len() < 2 {
            out.push(path);
            continue;
        }
        let n = path.points.len();
        let segs = if path.closed { n } else { n - 1 };
        let deg: Vec<f32> = (0..segs)
            .map(|k| {
                let a = path.points[k];
                let b = path.points[(k + 1) % n];
                degree_at(Point::new((a.x + b.x) / 2, (a.y + b.y) / 2))
            })
            .collect();
        // All one band: keep the path whole (band 0 = as-is, else overhang).
        let d0 = deg[0];
        if deg.iter().all(|&d| d == d0) {
            let mut p = path;
            if d0 > 0.0 {
                p.kind = PathKind::OverhangWall;
                p.overhang = d0;
            }
            out.push(p);
            continue;
        }
        // Mixed: gather maximal same-band runs (cyclic for loops), at a border.
        let seg_len = |k: usize| pt_dist_mm(path.points[k], path.points[(k + 1) % n]);
        let start = if path.closed {
            (0..segs).find(|&k| deg[(k + segs - 1) % segs] != deg[k]).unwrap_or(0)
        } else {
            0
        };
        let mut runs: Vec<(f32, Vec<usize>, f64)> = Vec::new();
        for i in 0..segs {
            let k = (start + i) % segs;
            let len = seg_len(k);
            match runs.last_mut() {
                Some((c, idxs, l)) if *c == deg[k] => {
                    idxs.push(k);
                    *l += len;
                }
                _ => runs.push((deg[k], vec![k], len)),
            }
        }
        // Dissolve sub-threshold runs into the previous (sound) one.
        let mut merged: Vec<(f32, Vec<usize>, f64)> = Vec::new();
        for run in runs {
            match merged.last_mut() {
                Some((c, idxs, l)) if *c == run.0 || run.2 < min_run_mm => {
                    idxs.extend(run.1);
                    *l += run.2;
                }
                _ => merged.push(run),
            }
        }
        // A short leading run may now belong with the trailing one (cyclic).
        if path.closed && merged.len() > 1 && merged[0].2 < min_run_mm {
            let first = merged.remove(0);
            let last = merged.last_mut().unwrap();
            last.1.extend(first.1);
            last.2 += first.2;
        }
        if merged.len() == 1 {
            let mut p = path;
            if merged[0].0 > 0.0 {
                p.kind = PathKind::OverhangWall;
                p.overhang = merged[0].0;
            }
            out.push(p);
            continue;
        }
        // Mixed support: keep ONE bead and ANNOTATE each segment — an over-air run
        // becomes OverhangWall at its graduated degree, a supported run keeps the
        // wall kind. The emitter grades speed / pressure-advance / fan along the loop
        // (see ToolPath::segs), so it prints as one continuous bead with a single seam
        // instead of a pile of split arcs — cohesion without losing the per-stretch
        // overhang treatment.
        let mut attrs = vec![SegAttr { kind: path.kind, overhang: 0.0 }; segs];
        for (deg_run, idxs, _) in &merged {
            let a = if *deg_run > 0.0 {
                SegAttr { kind: PathKind::OverhangWall, overhang: *deg_run }
            } else {
                SegAttr { kind: path.kind, overhang: 0.0 }
            };
            for &k in idxs {
                attrs[k] = a;
            }
        }
        let mut p = path;
        p.segs = Some(attrs);
        out.push(p);
    }
    out
}

/// For each ENCLOSED over-air hollow (a ceiling hollow ringed by supported walls),
/// the hollow plus a thin landing band onto the supported rim. The bridge sheet
/// spans this whole region, so its ends sit on the band — which is over solid the
/// layer below holds up (a real foothold) — and the band is kept clear of inner
/// perimeters (Pass 1) so nothing boxes the sheet in. Inner walls beyond the band
/// are untouched. `below` is the layer underneath; `allowance` the overhang reach.
fn enclosed_ceiling_sheet(layer: &Polygons, below: &Polygons, lw: f64, allowance: f64, reach: f64) -> Polygons {
    let supported = offset(below, allowance);
    let over_air = difference(layer, &supported);
    if over_air.is_empty() {
        return Polygons::new();
    }
    let inside_outer = offset(layer, -lw);
    let mut band = Polygons::new();
    for h in islands(&over_air) {
        // Enclosed only: dilating the hollow stays inside the slice. A cantilever's
        // over-air reaches the part's free edge, so dilating it leaves the slice.
        if difference(&offset(&h, lw), layer).is_empty() {
            band = union(&band, &intersection(&offset(&h, reach), &inside_outer));
        }
    }
    band
}

/// The direction to run bridge lines across `region` — the angle whose spans are
/// shortest and (almost) all anchored on both ends. `None` for a cantilever or a
/// span wider than `max_span`. Split out from `try_bridge` so the caller can DECIDE
/// on the true unsupported span here, then FILL a slightly grown region (reaching
/// onto the surrounding walls) at the same angle — without the extension inflating
/// the span check.
fn bridge_angle(region: &Polygons, supported: &Polygons, lw: f64, max_span: f64) -> Option<f64> {
    if max_span <= 0.0 {
        return None;
    }
    // Try a range of line directions; the bridge runs across the shortest spans.
    let mut best: Option<(f64, f64)> = None; // (max line length, angle)
    for k in 0..12 {
        let angle = k as f64 * 15.0;
        let segs = infill_lines(region, angle, lw, false, 0.5, false);
        let (mut total, mut anchored, mut max_len) = (0usize, 0usize, 0.0f64);
        for seg in &segs {
            if seg.len() < 2 {
                continue;
            }
            let (a, b) = (seg[0], seg[seg.len() - 1]);
            total += 1;
            max_len = max_len.max(pt_dist_mm(a, b));
            if bridge_anchored(a, b, supported, lw) {
                anchored += 1;
            }
        }
        // Need a real area, every line short enough, and (almost) all anchored on
        // both ends — i.e. genuinely spanning between supports.
        if total >= 2 && max_len <= max_span && anchored * 100 >= total * 85 && best.map_or(true, |(bl, _)| max_len < bl) {
            best = Some((max_len, angle));
        }
    }
    best.map(|(_, angle)| angle)
}

/// If `region` is a true bridge — supported on ≥2 sides and narrow enough to span
/// with straight lines — return those lines (oriented across the shortest gap,
/// solid spacing). Returns None for cantilevers or spans wider than `max_span`.
/// The planner splits this into `bridge_angle` (decide) + a grown fill; this whole-
/// region form is kept as the unit-test entry point for the bridge decision.
#[cfg(test)]
fn try_bridge(region: &Polygons, supported: &Polygons, lw: f64, max_span: f64) -> Option<Vec<Vec<Point>>> {
    // Boustrophedon order (alternating direction) so consecutive bridge lines meet at
    // the same end — short turnarounds that `connect_fill_runs` can stitch into one
    // continuous bridge, instead of every line being a span apart.
    bridge_angle(region, supported, lw, max_span)
        .map(|angle| infill_lines(region, angle, lw, true, 0.5, false))
}

/// A bridge line is anchored if both ends, extended outward by a line width, land
/// on supported material — so the line spans between two supports.
fn bridge_anchored(a: Point, b: Point, supported: &Polygons, lw: f64) -> bool {
    let (ax, ay, bx, by) = (a.x_mm(), a.y_mm(), b.x_mm(), b.y_mm());
    let len = (bx - ax).hypot(by - ay);
    if len < 1.0e-6 {
        return false;
    }
    let (ux, uy) = ((bx - ax) / len, (by - ay) / len);
    let ea = Point::from_mm(ax - ux * lw, ay - uy * lw);
    let eb = Point::from_mm(bx + ux * lw, by + uy * lw);
    point_in(supported, ea) && point_in(supported, eb)
}

fn point_in(polys: &Polygons, p: Point) -> bool {
    let mut inside = false;
    for c in &polys.contours {
        if c.contains(p) {
            inside = !inside;
        }
    }
    inside
}

fn pt_dist_mm(a: Point, b: Point) -> f64 {
    (a.x_mm() - b.x_mm()).hypot(a.y_mm() - b.y_mm())
}

/// Stitch fill beads (`paths[start..]`) into continuous runs. Each bead is appended to
/// whichever OPEN run it can continue with a short turnaround that stays inside the
/// walls — not merely the previous bead. That distinction matters where a hole (or an
/// L's inner corner) splits a scanline into two spans: the monotonic order interleaves
/// the sides (left, right, left, right...), so appending to the nearest joinable run
/// lets each side accrete into its OWN continuous run instead of breaking every
/// cross-void turnaround into a separate line. Every join is still a short in-region
/// turnaround (nothing crosses the void, so no scribble), and only same-kind,
/// same-group open beads join, so bridges and separate islands keep their own flow.
fn connect_fill_runs(paths: &mut Vec<ToolPath>, start: usize, inside: &Polygons, max_jump: f64) {
    if paths.len() < start + 2 {
        return;
    }
    // The bead ends sit ON the wall edge (extend_ends_to_wall stopped them there), so
    // test joins against the walls grown a hair — otherwise an on-edge sample reads as
    // "outside" and every flat-wall turnaround is wrongly rejected.
    let inflated = offset(inside, 0.1);
    let beads: Vec<ToolPath> = paths.split_off(start);
    let mut runs: Vec<ToolPath> = Vec::new();
    for bead in beads {
        if bead.closed || bead.points.len() < 2 {
            runs.push(bead);
            continue;
        }
        let (head, tail) = (bead.points[0], *bead.points.last().unwrap());
        // Nearest open run this bead can continue — in either orientation, since a
        // boustrophedon reversal or a hole-split span may present its far end first.
        let mut best: Option<(usize, bool, f64)> = None;
        for (i, r) in runs.iter().enumerate() {
            if r.closed || r.kind != bead.kind || r.group != bead.group || r.points.len() < 2 {
                continue;
            }
            let end = *r.points.last().unwrap();
            let (fwd, rev) = (pt_dist_mm(end, head), pt_dist_mm(end, tail));
            let (d, flip) = if fwd <= rev { (fwd, false) } else { (rev, true) };
            if best.map_or(true, |(_, _, bd)| d < bd)
                && join_inside(end, if flip { tail } else { head }, &inflated, max_jump)
            {
                best = Some((i, flip, d));
            }
        }
        match best {
            Some((i, true, _)) => runs[i].points.extend(bead.points.iter().rev().copied()),
            Some((i, false, _)) => runs[i].points.extend(bead.points),
            None => runs.push(bead),
        }
    }
    paths.extend(runs);
}

/// Every sample along a→b stays inside `inside` — so the stitched bead never crosses
/// the boundary into open space or another pocket.
fn seg_inside(a: Point, b: Point, inside: &Polygons) -> bool {
    let d = pt_dist_mm(a, b);
    if d <= 1.0e-6 {
        return true;
    }
    let steps = (d / 0.2).ceil() as usize;
    for k in 0..=steps {
        let t = k as f64 / steps as f64;
        let s = Point::from_mm(
            a.x_mm() + (b.x_mm() - a.x_mm()) * t,
            a.y_mm() + (b.y_mm() - a.y_mm()) * t,
        );
        if !point_in(inside, s) {
            return false;
        }
    }
    true
}

/// A turnaround a→b is a safe straight join when it's short (≤ `max_jump`) and stays
/// inside the region the whole way.
fn join_inside(a: Point, b: Point, inside: &Polygons, max_jump: f64) -> bool {
    pt_dist_mm(a, b) <= max_jump && seg_inside(a, b, inside)
}

/// Splice concentric offset `rings` — each tagged with its PathKind — into continuous
/// inward-spiral strokes. Start at the outermost unused ring (seamed), trace it closed,
/// then jog to the nearest unused ring whose entry projects within `max_cross` and
/// reaches without leaving `region` — repeat, so a nested family (a wall stack, a
/// concentric fill, or the walls PLUS the fill they ring) prints as ONE stroke with a
/// single seam and no per-ring travel/restart. Returns each stroke's points plus a
/// per-segment kind, so a wall→fill hand-off inside one stroke keeps each side's
/// speed/flow. A ring no short in-region crossover can reach seeds a fresh stroke;
/// built outermost-first, so an enclosed-void ring still lands on the one before it.
fn spiralize_rings(
    rings: Vec<(Vec<Point>, PathKind)>,
    region: &Polygons,
    max_cross: f64,
    seam_mode: SeamMode,
    layer_index: usize,
) -> Vec<(Vec<Point>, Vec<PathKind>)> {
    let n = rings.len();
    if n == 0 {
        return Vec::new();
    }
    let inflated = offset(region, 0.1);
    let mut used = vec![false; n];
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        loop_area_mm2(&rings[b].0).partial_cmp(&loop_area_mm2(&rings[a].0)).unwrap()
    });
    let mut strokes: Vec<(Vec<Point>, Vec<PathKind>)> = Vec::new();
    for &seed in &order {
        if used[seed] {
            continue;
        }
        used[seed] = true;
        // Trace the seed ring closed, from a placed seam.
        let mut chain = place_seam(rings[seed].0.clone(), seam_mode, layer_index);
        if let Some(&first) = chain.first() {
            chain.push(first);
        }
        let mut kinds: Vec<PathKind> = vec![rings[seed].1; chain.len().saturating_sub(1)];
        loop {
            let tail = *chain.last().unwrap();
            // Nearest reachable point on any unused ring — projected onto its EDGES, so
            // a long straight edge with sparse vertices still connects at its closest
            // point rather than a distant corner (the difference between a wall stack
            // chaining into one stroke and fragmenting into per-ring stubs).
            let mut best: Option<(usize, usize, Point, f64)> = None;
            for i in 0..n {
                if used[i] {
                    continue;
                }
                let r = &rings[i].0;
                let m = r.len();
                for k in 0..m {
                    let e = nearest_point_on_seg(tail, r[k], r[(k + 1) % m]);
                    let dd = pt_dist_mm(tail, e);
                    if dd < best.map_or(f64::INFINITY, |(_, _, _, bd)| bd) {
                        best = Some((i, k, e, dd));
                    }
                }
            }
            match best {
                Some((i, k, entry, dd)) if dd <= max_cross && seg_inside(tail, entry, &inflated) => {
                    used[i] = true;
                    let kind = rings[i].1;
                    let r = &rings[i].0;
                    let m = r.len();
                    // Enter ring i on edge k→k+1 at `entry`, trace it around, close to it.
                    chain.push(entry);
                    for j in 1..=m {
                        chain.push(r[(k + j) % m]);
                    }
                    chain.push(entry);
                    kinds.extend(std::iter::repeat(kind).take(m + 2));
                }
                _ => break,
            }
        }
        strokes.push((chain, kinds));
    }
    strokes
}

/// Convert a spiralized (points, per-segment kinds) result into a ToolPath: a
/// uniform-kind stroke stays plain (`segs` None), a mixed one (a wall→fill sweep)
/// carries a `SegAttr` per segment so each stretch keeps its own speed/flow.
fn spiral_to_toolpath(points: Vec<Point>, kinds: &[PathKind], lw: f64) -> ToolPath {
    let base = kinds.first().copied().unwrap_or(PathKind::Perimeter);
    let mut tp = ToolPath::new(base, false, lw, points);
    if kinds.iter().any(|&k| k != base) {
        tp.segs = Some(kinds.iter().map(|&k| SegAttr { kind: k, overhang: 0.0 }).collect());
    }
    tp
}

/// Spiralize each supported island's SHELLS — its inner `Perimeter` wall rings PLUS
/// the concentric fill loops they ring — into one continuous inward stroke, so the
/// nozzle sweeps from the outer wall straight through the cavity fill with a single
/// seam and no travel at the hand-off (reusing [`spiralize_rings`], kind-tagged so the
/// wall keeps `Perimeter` speed/flow and the fill its own via a `SegAttr` per segment).
/// The visible `ExternalPerimeter` is left its own loop; monotonic line fill (group
/// set) and bridged/overhang runs (open, or already segmented) never qualify; and an
/// island carrying ANY overhang opts out wholesale so its rings stay closed for the
/// enclosed-void / cantilever ordering ([`order_unsupported_rings_outer_in`]).
fn spiralize_shells(
    paths: Vec<ToolPath>,
    region: &Polygons,
    lw: f64,
    sp: f64,
    seam_mode: SeamMode,
    layer_index: usize,
) -> Vec<ToolPath> {
    // Island key = the largest-area external perimeter whose loop holds the centroid.
    let ext_loops: Vec<(Vec<Point>, f64)> = paths
        .iter()
        .filter(|p| p.kind == PathKind::ExternalPerimeter && p.closed && p.points.len() >= 3)
        .map(|p| (p.points.clone(), loop_area_mm2(&p.points)))
        .collect();
    if ext_loops.is_empty() {
        return paths;
    }
    let key_of = |pts: &[Point]| -> usize {
        let c = loop_centroid(pts);
        let (mut key, mut best) = (usize::MAX, -1.0);
        for (k, (ep, ea)) in ext_loops.iter().enumerate() {
            if *ea > best && loop_contains(ep, c) {
                best = *ea;
                key = k;
            }
        }
        key
    };
    // Islands with any overhang opt out — a uniform OverhangWall loop or a wall
    // carrying overhang SEGMENTS. Their rings must stay whole for the void ordering.
    let mut overhang_island = vec![false; ext_loops.len()];
    for p in &paths {
        if p.kind == PathKind::OverhangWall || p.segs.is_some() {
            let k = key_of(&p.points);
            if k != usize::MAX {
                overhang_island[k] = true;
            }
        }
    }
    // A shell ring is a CLOSED inner Perimeter wall, or a CLOSED concentric fill loop
    // (group-less, so monotonic line fill and bridged arcs are excluded).
    let is_shell = |p: &ToolPath| {
        p.closed
            && p.points.len() >= 3
            && p.segs.is_none()
            && match p.kind {
                PathKind::Perimeter => true,
                PathKind::Solid | PathKind::TopSkin | PathKind::BottomSkin => p.group.is_none(),
                _ => false,
            }
    };
    let mut passthrough: Vec<ToolPath> = Vec::new();
    let mut by_island: std::collections::BTreeMap<usize, Vec<(Vec<Point>, PathKind)>> =
        std::collections::BTreeMap::new();
    for p in paths {
        if is_shell(&p) {
            let k = key_of(&p.points);
            if k != usize::MAX && !overhang_island[k] {
                by_island.entry(k).or_default().push((p.points, p.kind));
                continue;
            }
        }
        passthrough.push(p);
    }
    let mut out = passthrough;
    for (_k, rings) in by_island {
        // A lone ring stays a clean closed loop (nothing to chain into).
        if rings.len() < 2 {
            for (pts, kind) in rings {
                out.push(ToolPath::new(kind, true, lw, pts));
            }
            continue;
        }
        for (pts, kinds) in spiralize_rings(rings, region, sp * 1.7, seam_mode, layer_index) {
            out.push(spiral_to_toolpath(pts, &kinds, lw));
        }
    }
    out
}

/// Lengthen a skin line-end along its OWN direction until it meets the wall edge
/// (`inside_walls`), so its bead reaches the perimeter — per bead, the few tenths
/// it needs, no region grow and no extra perimeter. `prev` is the point before the
/// endpoint (sets the outward direction). If a full `max` step stays inside the
/// walls, this end faces interior infill (a sparse boundary), so it isn't moved.
/// Apply `extend_to_wall` to both ends of every open path in `paths` — the
/// per-bead reach, shared by solid skins, buried solid, and sparse infill.
fn extend_ends_to_wall(paths: &mut [ToolPath], max: f64, inside_walls: &Polygons) {
    for p in paths.iter_mut() {
        if p.closed || p.points.len() < 2 {
            continue;
        }
        let n = p.points.len();
        p.points[0] = extend_to_wall(p.points[0], p.points[1], max, inside_walls);
        p.points[n - 1] = extend_to_wall(p.points[n - 1], p.points[n - 2], max, inside_walls);
    }
}

fn extend_to_wall(p: Point, prev: Point, max: f64, inside_walls: &Polygons) -> Point {
    let (px, py, qx, qy) = (p.x_mm(), p.y_mm(), prev.x_mm(), prev.y_mm());
    let len = (px - qx).hypot(py - qy);
    if len < 1.0e-6 {
        return p;
    }
    let (dx, dy) = ((px - qx) / len, (py - qy) / len); // outward direction
    // Only lengthen ends that are genuinely SET BACK inside the walls: if a hair
    // outward already leaves the region, this end is already at/past the wall —
    // leave it (extending ends already on the boundary was the overshoot bug).
    if !point_in(inside_walls, Point::from_mm(px + dx * 0.02, py + dy * 0.02)) {
        return p;
    }
    // The NEAREST boundary crossing along the ray. Taking the nearest (not a
    // binary search, which goes non-monotonic and converges on the FAR boundary
    // at a wall with infill on both sides — a cockpit — and punches through) stops
    // the line at the near wall. Beyond `max` → interior end facing other infill.
    let mut best = f64::INFINITY;
    for c in &inside_walls.contours {
        let m = c.points.len();
        if m < 2 {
            continue;
        }
        for j in 0..m {
            let a = c.points[j];
            let b = c.points[(j + 1) % m];
            // Ray P+t·D (t>0) vs segment A+s·E (E = B−A, 0≤s≤1).
            let (ex, ey) = (b.x_mm() - a.x_mm(), b.y_mm() - a.y_mm());
            let det = dy * ex - dx * ey;
            if det.abs() < 1.0e-12 {
                continue; // parallel
            }
            let (fx, fy) = (a.x_mm() - px, a.y_mm() - py);
            let t = (fy * ex - fx * ey) / det;
            let s = (dx * fy - dy * fx) / det;
            if t > 1.0e-3 && (0.0..=1.0).contains(&s) && t < best {
                best = t;
            }
        }
    }
    if best <= max {
        Point::from_mm(px + dx * best, py + dy * best)
    } else {
        p // nearest wall beyond reach → interior end, leave it
    }
}

/// Greedily order each layer's paths (nearest-neighbour) to cut travel, keeping
/// skirt/brim first and ironing last (it must run over the finished surface).
/// Open paths may be reversed to start at the nearer end; runs of `no_reorder`
/// paths (monotonic fill) move as one block.
fn order_layers(plans: &mut [LayerPlan], outer_first: bool) {
    let mut cur = Point::new(0, 0);
    let mut cur_tool: Option<u32> = None;
    for plan in plans.iter_mut() {
        let all = std::mem::take(&mut plan.paths);
        // Group by tool first — a toolchange costs far more than any travel, so
        // a layer prints everything one tool owns before switching. The tool
        // already in hand goes first (consecutive layers then serpentine through
        // the tools: each layer pays only the changes it must); the rest follow
        // in ascending order. Single-tool prints have one group and none of
        // this reorders anything.
        let mut groups: std::collections::BTreeMap<u32, Vec<ToolPath>> =
            std::collections::BTreeMap::new();
        for p in all {
            groups.entry(p.tool).or_default().push(p);
        }
        let mut tool_order: Vec<u32> = groups.keys().copied().collect();
        if let Some(ct) = cur_tool {
            if let Some(k) = tool_order.iter().position(|&t| t == ct) {
                tool_order.remove(k);
                tool_order.insert(0, ct);
            }
        }
        let mut paths = Vec::new();
        for t in tool_order {
            let group = groups.remove(&t).unwrap();
            let (prime, rest): (Vec<_>, Vec<_>) =
                group.into_iter().partition(|p| p.kind == PathKind::Skirt);
            if let Some(last) = prime.last() {
                cur = path_end(last);
            }
            let (iron, rest): (Vec<_>, Vec<_>) =
                rest.into_iter().partition(|p| p.kind == PathKind::Ironing);
            paths.extend(prime);
            // Walls before fill: a bridge or solid must anchor on perimeters
            // already laid THIS layer — a bridge strand whose ends sit on the inner
            // rim (not the layer below) has nothing to grab if its perimeter prints
            // afterwards, and flails. Order the wall block first, then the fill.
            let (walls, fill): (Vec<_>, Vec<_>) = rest.into_iter().partition(|p| {
                matches!(
                    p.kind,
                    PathKind::ExternalPerimeter | PathKind::Perimeter | PathKind::OverhangWall
                )
            });
            // Walls: explicit outer-vs-inner sequence per island. Fill: greedy travel.
            let walls = order_walls(walls, cur, outer_first);
            if let Some(last) = walls.last() {
                cur = path_end(last);
            }
            paths.extend(walls);
            if !fill.is_empty() {
                let fill = order_paths(fill, cur);
                if let Some(last) = fill.last() {
                    cur = path_end(last);
                }
                paths.extend(fill);
            }
            if let Some(last) = iron.last() {
                cur = path_end(last);
            }
            paths.extend(iron); // already in boustrophedon order
            cur_tool = Some(t);
        }
        plan.paths = paths;
    }
}

/// Concentric rings spanning an *enclosed* unsupported void must print supported-
/// edge inward, so each ring lands on the previously-laid outer ring. Greedy travel
/// ordering can lay them inner-first, which drops the first ring into the void.
/// This covers the roof-as-walls case (overhang wall rings, no bottom shell) and a
/// concentric bottom shell over air. A *cantilever* is the opposite case and is
/// left alone: its outermost ring runs over its own free edge, so outer→in would
/// lay that unsupported span first — there's nothing to print supported-edge-first
/// from. The test is geometric: reorder a run only when its outermost (largest)
/// ring sits fully on the layer below and its innermost is over air.
fn order_unsupported_rings_outer_in(plans: &mut [LayerPlan], phys: &[Polygons], settings: &Settings) {
    fn loop_area(p: &ToolPath) -> f64 {
        let pts = &p.points;
        let m = pts.len();
        if m < 3 {
            return 0.0;
        }
        (0..m)
            .map(|j| {
                let (a, b) = (pts[j], pts[(j + 1) % m]);
                a.x_mm() * b.y_mm() - b.x_mm() * a.y_mm()
            })
            .sum::<f64>()
            .abs()
            * 0.5
    }
    // A ring counts as "supported" when the MAJORITY of its segment midpoints sit
    // over the (grown) layer below. Majority, not all: an overhang wall is now kept
    // whole (mixed-support beads carry per-segment overhang annotations rather than
    // being split into arcs), so a void cap's outer ring dips a segment or two into
    // the void yet is still predominantly anchored — while a cantilever's outer ring
    // is predominantly over air. That split is exactly the enclosed-void vs
    // cantilever distinction the reorder below needs.
    fn supported(p: &ToolPath, below: &Polygons) -> bool {
        let pts = &p.points;
        let n = pts.len();
        if n < 2 {
            return true;
        }
        let segs = if p.closed { n } else { n - 1 };
        let sup = (0..segs)
            .filter(|&k| {
                let (a, b) = (pts[k], pts[(k + 1) % n]);
                in_polys(below, Point::new((a.x + b.x) / 2, (a.y + b.y) / 2))
            })
            .count();
        sup * 2 > segs
    }
    let is_ring = |k: PathKind| {
        matches!(
            k,
            PathKind::Perimeter
                | PathKind::ExternalPerimeter
                | PathKind::OverhangWall
                | PathKind::BottomSkin
        )
    };
    for plan in plans.iter_mut() {
        if plan.index == 0 {
            continue;
        }
        let allowance =
            settings.layer_height_mm * settings.support_overhang_angle_deg.to_radians().tan();
        let below = offset(&phys[plan.index - 1], allowance);
        let paths = &mut plan.paths;
        let part_of_run = |p: &ToolPath| is_ring(p.kind);
        let mut i = 0;
        while i < paths.len() {
            if !is_ring(paths[i].kind) {
                i += 1;
                continue;
            }
            // A run is the maximal stretch of ring paths plus the strokes travel
            // ordering tucks among them. Overhang walls are kept whole now (mixed
            // support = per-segment annotation, not split arcs), so a void cap's
            // rings are all closed; `supported` uses a majority test so a ring that
            // dips a little into the void still reads anchored. A run never crosses
            // a tool boundary — rings of different parts must not swap.
            let mut j = i;
            while j < paths.len() && part_of_run(&paths[j]) && paths[j].tool == paths[i].tool {
                j += 1;
            }
            let area_at = |k: usize| loop_area(&paths[k]);
            // Enclosed-void test over every ring segment (open arcs included); the
            // reorder touches only the closed loops, leaving arcs + gap fill put.
            let segs: Vec<usize> = (i..j).filter(|&k| is_ring(paths[k].kind)).collect();
            let closed: Vec<usize> =
                (i..j).filter(|&k| paths[k].closed && is_ring(paths[k].kind)).collect();
            // Enclosed void only: largest ring fully on support, smallest over air.
            // A cantilever fails the first test; a fully supported run the second.
            if segs.len() >= 2 && closed.len() >= 2 {
                let outer = *segs.iter().max_by(|&&a, &&b| area_at(a).partial_cmp(&area_at(b)).unwrap()).unwrap();
                let inner = *segs.iter().min_by(|&&a, &&b| area_at(a).partial_cmp(&area_at(b)).unwrap()).unwrap();
                let outer_supported = supported(&paths[outer], &below);
                let inner_supported = supported(&paths[inner], &below);
                // Reorder only the closed loops; open arcs + gap fill stay put.
                let mut sorted: Vec<ToolPath> = closed.iter().map(|&k| paths[k].clone()).collect();
                if outer_supported && !inner_supported {
                    // Enclosed void: outer ring on solid, inner over air. Print
                    // outer→in so each ring bridges off the one before it.
                    sorted.sort_by(|a, b| loop_area(b).partial_cmp(&loop_area(a)).unwrap());
                    for (&slot, r) in closed.iter().zip(sorted) {
                        paths[slot] = r;
                    }
                } else if !outer_supported {
                    // Cantilever/overhang: even the outer ring is over air (the
                    // opened inner rings nest into clean concentric loops past the
                    // support). Print inner→out so the big unsupported span is never
                    // laid first — without this they default to travel order, which
                    // walks the nested loops outer→in.
                    sorted.sort_by(|a, b| loop_area(a).partial_cmp(&loop_area(b)).unwrap());
                    for (&slot, r) in closed.iter().zip(sorted) {
                        paths[slot] = r;
                    }
                }
            }
            i = j;
        }
    }
}

fn order_paths(remaining: Vec<ToolPath>, start: Point) -> Vec<ToolPath> {
    // Runs of consecutive same-group paths form indivisible blocks (monotonic
    // fill must keep its sweep order); everything else is a singleton. Distinct
    // groups — separate islands — stay independently orderable.
    let mut blocks: Vec<Vec<ToolPath>> = Vec::new();
    for p in remaining {
        let extend = p.group.is_some()
            && blocks
                .last()
                .map_or(false, |b| b[0].group == p.group && b[0].kind == p.kind);
        match (extend, blocks.last_mut()) {
            (true, Some(b)) => b.push(p),
            _ => blocks.push(vec![p]),
        }
    }

    let total: usize = blocks.iter().map(Vec::len).sum();
    let mut out = Vec::with_capacity(total);
    let mut cur = start;
    while !blocks.is_empty() {
        let mut best = 0usize;
        let mut best_d = i128::MAX;
        let mut best_rev = false;
        for (i, b) in blocks.iter().enumerate() {
            let ds = dist2(cur, b[0].points[0]);
            if ds < best_d {
                best_d = ds;
                best = i;
                best_rev = false;
            }
            // A block is reversible end-to-end when every member is open: the
            // sweep direction flips wholesale, which is still monotonic.
            if b.iter().all(|p| !p.closed) {
                let last = &b[b.len() - 1];
                let de = dist2(cur, last.points[last.points.len() - 1]);
                if de < best_d {
                    best_d = de;
                    best = i;
                    best_rev = true;
                }
            }
        }
        let mut b = blocks.swap_remove(best);
        if best_rev {
            b.reverse();
            for p in &mut b {
                p.points.reverse();
            }
        }
        cur = path_end(&b[b.len() - 1]);
        out.extend(b);
    }
    out
}

/// Order a layer's wall paths so each material island's external perimeter prints
/// contiguously with its own inner walls — outer wall LAST (default: it lands on
/// solid inner backing, the crispest flat surface) or FIRST (`outer_first`: better
/// overhang edges). Greedy travel still picks island order and the route within
/// each sub-group; this only pins the outer-vs-inner sequence, which pure travel
/// ordering otherwise leaves arbitrary (the cause of inconsistent surfaces).
fn order_walls(walls: Vec<ToolPath>, start: Point, outer_first: bool) -> Vec<ToolPath> {
    if walls.len() < 2 {
        return walls;
    }
    // Cluster each wall with the largest-area external perimeter whose loop holds
    // its centroid: an island's outer wall, its hole walls, and the inner walls
    // between them share one cluster, so the outer wall lands with its OWN inners,
    // not another island's. A wall under no external falls in its own bucket.
    let ext: Vec<usize> = (0..walls.len())
        .filter(|&i| walls[i].kind == PathKind::ExternalPerimeter)
        .collect();
    let ext_area: Vec<f64> = ext.iter().map(|&i| loop_area_mm2(&walls[i].points)).collect();
    let keys: Vec<usize> = walls
        .iter()
        .map(|p| {
            let c = loop_centroid(&p.points);
            let mut key = usize::MAX;
            let mut best = -1.0;
            for (k, &ei) in ext.iter().enumerate() {
                if ext_area[k] > best && loop_contains(&walls[ei].points, c) {
                    best = ext_area[k];
                    key = k;
                }
            }
            key
        })
        .collect();
    let mut by_cluster: std::collections::BTreeMap<usize, Vec<ToolPath>> = std::collections::BTreeMap::new();
    for (w, k) in walls.into_iter().zip(keys) {
        by_cluster.entry(k).or_default().push(w);
    }
    let mut clusters: Vec<Vec<ToolPath>> = by_cluster.into_values().collect();
    // Visit clusters greedily by travel; emit each one's walls in the chosen order.
    let mut cur = start;
    let mut out = Vec::with_capacity(clusters.iter().map(Vec::len).sum());
    while !clusters.is_empty() {
        let pick = (0..clusters.len())
            .min_by_key(|&i| {
                clusters[i].iter().map(|p| dist2(cur, p.points[0])).min().unwrap_or(i128::MAX)
            })
            .unwrap();
        let (exts, inners): (Vec<_>, Vec<_>) = clusters
            .swap_remove(pick)
            .into_iter()
            .partition(|p| p.kind == PathKind::ExternalPerimeter);
        let seq = if outer_first { [exts, inners] } else { [inners, exts] };
        for sub in seq {
            if sub.is_empty() {
                continue;
            }
            let ordered = order_paths(sub, cur);
            if let Some(last) = ordered.last() {
                cur = path_end(last);
            }
            out.extend(ordered);
        }
    }
    out
}

/// Vertex-average centroid of a loop — a representative interior point.
fn loop_centroid(pts: &[Point]) -> Point {
    if pts.is_empty() {
        return Point::new(0, 0);
    }
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |(ax, ay), p| (ax + p.x_mm(), ay + p.y_mm()));
    let n = pts.len() as f64;
    Point::from_mm(sx / n, sy / n)
}

/// Even-odd point-in-loop test, treating the polyline as closed.
fn loop_contains(pts: &[Point], p: Point) -> bool {
    let n = pts.len();
    if n < 3 {
        return false;
    }
    let (px, py) = (p.x_mm(), p.y_mm());
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (pts[i].x_mm(), pts[i].y_mm());
        let (xj, yj) = (pts[j].x_mm(), pts[j].y_mm());
        if (yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Unsigned area (mm²) of a closed loop.
fn loop_area_mm2(pts: &[Point]) -> f64 {
    let m = pts.len();
    if m < 3 {
        return 0.0;
    }
    (0..m)
        .map(|j| {
            let (a, b) = (pts[j], pts[(j + 1) % m]);
            a.x_mm() * b.y_mm() - b.x_mm() * a.y_mm()
        })
        .sum::<f64>()
        .abs()
        * 0.5
}

fn path_end(p: &ToolPath) -> Point {
    if p.closed {
        p.points[0]
    } else {
        p.points[p.points.len() - 1]
    }
}

fn dist2(a: Point, b: Point) -> i128 {
    let dx = (a.x - b.x) as i128;
    let dy = (a.y - b.y) as i128;
    dx * dx + dy * dy
}

/// Loops around the first-layer outline, offset outward, to prime the nozzle and
/// establish flow before the part starts.
fn skirt_paths(first_layer: &Polygons, settings: &Settings) -> Vec<ToolPath> {
    let lw = settings.line_width_mm;
    // Keep the skirt outside any brim (brim extends ~brim_loops line widths out).
    let brim_extent = lw * settings.brim_loops as f64;
    let mut paths = Vec::new();
    for k in 0..settings.skirt_loops {
        let delta = brim_extent + settings.skirt_gap_mm + lw * (0.5 + k as f64);
        for c in offset(first_layer, delta).contours {
            // Outer loops only (CCW) — offsetting outward also shrinks holes into
            // loops inside the part's holes, which we must not print.
            if c.points.len() >= 3 && c.is_ccw() {
                paths.push(ToolPath::new(PathKind::Skirt, true, lw, c.points));
            }
        }
    }
    paths
}

/// Loops extending outward from the first-layer outline, the innermost touching
/// the outer wall — a brim for bed adhesion. (Rendered as the skirt feature.)
fn brim_paths(first_layer: &Polygons, settings: &Settings) -> Vec<ToolPath> {
    let lw = settings.line_width_mm;
    let sp = config::bead_spacing_mm(lw, settings.first_layer_height_mm);
    let mut paths = Vec::new();
    for k in 0..settings.brim_loops {
        let delta = lw * 0.5 + k as f64 * sp;
        for c in offset(first_layer, delta).contours {
            // Outer loops only — don't print brim loops inside the part's holes.
            if c.points.len() >= 3 && c.is_ccw() {
                paths.push(ToolPath::new(PathKind::Skirt, true, lw, c.points));
            }
        }
    }
    paths
}

/// Intersection of the `count` infill regions `count` layers away in direction
/// `dir` (+1 = above, -1 = below). Returns empty if there aren't `count` layers
/// that way (an exposed surface) or if any of them is empty (air).
fn coverage(inners: &[Polygons], i: usize, dir: isize, count: usize, n: usize) -> Polygons {
    let mut acc: Option<Polygons> = None;
    for k in 1..=count {
        let idx = i as isize + dir * k as isize;
        if idx < 0 || idx as usize >= n {
            return Polygons::new();
        }
        let layer = &inners[idx as usize];
        if layer.is_empty() {
            return Polygons::new();
        }
        acc = Some(match acc {
            None => layer.clone(),
            Some(a) => intersection(&a, layer),
        });
        if acc.as_ref().is_some_and(|a| a.is_empty()) {
            return Polygons::new();
        }
    }
    acc.unwrap_or_default()
}

/// Shift all toolpaths so the model's XY center sits at the bed center.
fn center_on_bed(plans: &mut [LayerPlan], parts: &[(&Mesh, PartPaint)], settings: &Settings) {
    if !settings.auto_center_on_bed {
        return; // caller positioned the geometry already (e.g. GUI multi-object layout)
    }
    let mut bounds: Option<(f64, f64, f64, f64)> = None;
    for (m, _) in parts {
        if let Some((lo_x, lo_y, hi_x, hi_y)) = m.xy_bounds() {
            bounds = Some(match bounds {
                None => (lo_x, lo_y, hi_x, hi_y),
                Some((ax, ay, bx, by)) => (ax.min(lo_x), ay.min(lo_y), bx.max(hi_x), by.max(hi_y)),
            });
        }
    }
    let Some((min_x, min_y, max_x, max_y)) = bounds else {
        return;
    };
    let model_cx = (min_x + max_x) / 2.0;
    let model_cy = (min_y + max_y) / 2.0;
    let dx = to_units(settings.bed_size_x_mm / 2.0 - model_cx);
    let dy = to_units(settings.bed_size_y_mm / 2.0 - model_cy);
    if dx == 0 && dy == 0 {
        return;
    }
    for plan in plans.iter_mut() {
        for path in &mut plan.paths {
            for p in &mut path.points {
                p.x += dx;
                p.y += dy;
            }
        }
        for c in &mut plan.outline.contours {
            for p in &mut c.points {
                p.x += dx;
                p.y += dy;
            }
        }
    }
}

/// Fill a region with the chosen pattern, pushing toolpaths into `out`.
///
/// `spacing` is the *mean line distance* for the requested density; multi-
/// direction patterns space each direction set proportionally wider so the
/// material laid down stays the same as `Lines` at the same density.
/// `monotonic` keeps scanline fills in strict sweep order (and boustrophedon
/// directions) — applied *per island*, so disjoint surfaces stay independently
/// orderable and travel doesn't ping-pong between them row by row.
#[allow(clippy::too_many_arguments)]
fn fill_region(
    region: &Polygons,
    pattern: InfillPattern,
    spacing: f64,
    angle: f64,
    lw: f64,
    kind: PathKind,
    seam_mode: SeamMode,
    layer_index: usize,
    z_mm: f64,
    monotonic: bool,
    out: &mut Vec<ToolPath>,
) {
    let push_lines = |segs: Vec<Vec<Point>>, group: Option<u32>, out: &mut Vec<ToolPath>| {
        for seg in segs {
            let mut p = ToolPath::new(kind, false, lw, seg);
            p.group = group;
            out.push(p);
        }
    };
    // No minimum row length on any fill: keep every span a bead would cover,
    // including the short corner stubs. Solid and skins must be airtight (there's
    // no boundary loop covering the rim any more, so a dropped stub is a real hole),
    // and sparse wants full coverage too; `extend_ends_to_wall` then walks each stub
    // out to the wall it faces.
    let min_len = 0.0;
    // Anchor the line grid to the region (outermost lines hug the parallel walls)
    // for solid/skin, which gains nothing from cross-layer stacking. Sparse keeps
    // the global grid so its lines stack into continuous interior walls.
    let anchor = kind != PathKind::Infill;
    // Scanline patterns sweep each island separately when monotonic.
    let scan = |sets: &[(f64, f64)], out: &mut Vec<ToolPath>| {
        if monotonic {
            for (gi, island) in islands(region).iter().enumerate() {
                for (si, &(a, sp)) in sets.iter().enumerate() {
                    let group = Some((gi * sets.len() + si) as u32);
                    push_lines(infill_lines(island, a, sp, true, min_len, anchor), group, out);
                }
            }
        } else {
            for &(a, sp) in sets {
                push_lines(infill_lines(region, a, sp, false, min_len, anchor), None, out);
            }
        }
    };
    match pattern {
        InfillPattern::Lines | InfillPattern::AlignedLines => scan(&[(angle, spacing)], out),
        InfillPattern::Grid => scan(&[(angle, spacing * 2.0), (angle + 90.0, spacing * 2.0)], out),
        InfillPattern::Triangles => scan(
            &[(angle, spacing * 3.0), (angle + 60.0, spacing * 3.0), (angle + 120.0, spacing * 3.0)],
            out,
        ),
        InfillPattern::Concentric => {
            // Concentric offset rings, emitted CLOSED. The Pass-2 spiralize_shells pass
            // splices them (together with the inner walls they ring) into continuous
            // inward-spiral strokes with a single seam — so the concentric fill flows
            // as one path, joined to the wall stack, with no per-ring travel/restart.
            let mut d = lw * 0.5;
            loop {
                let loops = offset(region, -d);
                if loops.is_empty() {
                    break;
                }
                for c in loops.contours {
                    if c.points.len() >= 3 {
                        let points = place_seam(c.points, seam_mode, layer_index);
                        out.push(ToolPath::new(kind, true, lw, points));
                    }
                }
                d += spacing;
            }
        }
        InfillPattern::Gyroid => {
            push_lines(crate::fill::gyroid_lines(region, spacing, z_mm), None, out);
        }
    }
}

/// Jitter a closed wall loop for fuzzy skin: resample roughly every
/// `point_dist` mm and push each sample along the local outward normal by a
/// deterministic pseudo-random amount in ±thickness/2. Original vertices are
/// kept (jittered) so corners survive.
fn fuzzy_loop(points: &[Point], thickness: f64, point_dist: f64, seed: usize) -> Vec<Point> {
    let n = points.len();
    let dist = point_dist.max(0.1);
    let perimeter: f64 = (0..n).map(|i| pt_dist_mm(points[i], points[(i + 1) % n])).sum();
    if n < 3 || perimeter < dist * 4.0 {
        return points.to_vec(); // too small to roughen
    }

    // xorshift* on a per-loop seed — deterministic across runs.
    let mut state = (seed as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    let mut rand_unit = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64 - 0.5
    };

    let mut out: Vec<Point> = Vec::with_capacity((perimeter / dist) as usize + n);
    for i in 0..n {
        let a = points[i];
        let b = points[(i + 1) % n];
        let (ax, ay, bx, by) = (a.x_mm(), a.y_mm(), b.x_mm(), b.y_mm());
        let len = (bx - ax).hypot(by - ay);
        if len < 1.0e-9 {
            continue;
        }
        // Outward normal of this edge (CCW outer and CW hole loops both face out).
        let (nx, ny) = ((by - ay) / len, -(bx - ax) / len);
        let mut jit = |x: f64, y: f64, out: &mut Vec<Point>| {
            let d = rand_unit() * thickness;
            out.push(Point::from_mm(x + nx * d, y + ny * d));
        };
        jit(ax, ay, &mut out); // the vertex itself, jittered along this edge's normal
        let steps = (len / dist).floor() as usize;
        for k in 1..=steps {
            let t = k as f64 * dist / len;
            if t >= 1.0 - 0.25 * dist / len {
                break; // too close to the next vertex
            }
            jit(ax + (bx - ax) * t, ay + (by - ay) * t, &mut out);
        }
    }
    if out.len() >= 3 {
        out
    } else {
        points.to_vec()
    }
}

/// Rotate a closed wall loop so the seam (start/end) lands at the chosen vertex.
fn place_seam(mut points: Vec<Point>, mode: SeamMode, layer_index: usize) -> Vec<Point> {
    let n = points.len();
    if n < 3 {
        return points;
    }
    // Rear-most vertex (max Y, tie-break max X) — seams align into a column.
    let rear = |points: &[Point]| (0..points.len()).max_by_key(|&i| (points[i].y, points[i].x)).unwrap();
    let start = match mode {
        SeamMode::Nearest => rear(&points),
        // Sharpest REAL corner (concave preferred — the seam tucks into the
        // notch). A smooth loop has no corner worth chasing: picking the
        // sharpest of its noise scatters the seam, so fall back to the rear
        // column instead. External walls are then held in line across layers
        // by `align_seams`.
        SeamMode::Sharpest => {
            let (concave, convex) = corner_candidates(&points);
            best_corner(&points, &concave, &convex).unwrap_or_else(|| rear(&points))
        }
        // Deterministic per-layer scatter.
        SeamMode::Random => layer_index.wrapping_mul(2_654_435_761).wrapping_add(40_503) % n,
        // Aligned starts from the rear like Nearest; `align_seams` then walks
        // the layers in order and snaps each loop to the previous layer's
        // seam, so this is only the first layer's seed.
        SeamMode::Aligned => rear(&points),
    };
    points.rotate_left(start);
    points
}

/// Hold external seams in line across layers: walk the layers bottom-up,
/// rotating every closed outer-wall loop to start at the candidate vertex
/// nearest the seam of the loop below it, so the seam follows one continuous
/// line up the print instead of jumping between competing features.
///
/// The candidate set is the mode's: Aligned may use any vertex (and snaps to
/// a real corner when one sits within `CORNER_SNAP_MM` of the column, rather
/// than drifting across a smooth face beside it); Sharpest only ever lands on
/// real corners while the loop has any — ties between equal corners stop
/// flip-flopping because the column decides — and degrades to Aligned
/// behavior on smooth loops, where "sharpest" would only amplify noise.
/// Loops with no track within `SEAM_TRACK_RADIUS_MM` (new islands) seed a new
/// track at the mode's best stand-alone choice. Runs after `center_on_bed`
/// and before travel planning, so combing and lead-ins see the final starts.
fn align_seams(plans: &mut [LayerPlan], mode: SeamMode) {
    const SEAM_TRACK_RADIUS_MM: f64 = 10.0;
    const CORNER_SNAP_MM: f64 = 2.5;
    let mut tracks: Vec<Point> = Vec::new();
    for plan in plans.iter_mut() {
        let loop_ids: Vec<usize> = plan
            .paths
            .iter()
            .enumerate()
            .filter(|(_, p)| {
                p.kind == PathKind::ExternalPerimeter && p.closed && p.points.len() >= 3
            })
            .map(|(i, _)| i)
            .collect();
        if loop_ids.is_empty() {
            continue;
        }
        // Candidate vertices per loop. Sharpest may land on any REAL corner —
        // concavity only biases the seed, because restricting the tracked set
        // to one concavity class makes it flap when a small notch appears or
        // vanishes, teleporting the column. Smooth loops (and Aligned) use
        // every vertex; the track then pulls them into a straight column.
        let per_loop: Vec<(Vec<usize>, Vec<Corner>, Vec<Corner>)> = loop_ids
            .iter()
            .map(|&pi| {
                let pts = &plan.paths[pi].points;
                let (concave, convex) = corner_candidates(pts);
                let cands: Vec<usize> = if mode == SeamMode::Sharpest
                    && !(concave.is_empty() && convex.is_empty())
                {
                    concave.iter().chain(convex.iter()).map(|c| c.idx).collect()
                } else {
                    (0..pts.len()).collect()
                };
                (cands, concave, convex)
            })
            .collect();
        // Greedy nearest-first (loop ↔ track) assignment, one track per loop
        // per layer — without it, two islands inside one radius (a hull and
        // the cabin beside it) take turns dragging the same track around.
        let mut assigned: Vec<Option<(usize, usize)>> = vec![None; loop_ids.len()]; // (vertex, track)
        let mut track_used = vec![false; tracks.len()];
        loop {
            let mut best: Option<(f64, usize, usize, usize)> = None; // (dist, loop, vertex, track)
            for (li, &pi) in loop_ids.iter().enumerate() {
                if assigned[li].is_some() {
                    continue;
                }
                let pts = &plan.paths[pi].points;
                for (ti, t) in tracks.iter().enumerate() {
                    if track_used[ti] {
                        continue;
                    }
                    for &vi in &per_loop[li].0 {
                        let d = pt_dist_mm(pts[vi], *t);
                        if d <= SEAM_TRACK_RADIUS_MM
                            && best.map_or(true, |(bd, _, _, _)| d < bd)
                        {
                            best = Some((d, li, vi, ti));
                        }
                    }
                }
            }
            let Some((_, li, vi, ti)) = best else { break };
            assigned[li] = Some((vi, ti));
            track_used[ti] = true;
        }
        // Apply: rotate matched loops onto their column (Aligned additionally
        // snaps to a real corner within reach — it hides the seam better than
        // the bare nearest vertex); unmatched loops seed a new track at the
        // mode's stand-alone best.
        for (li, &pi) in loop_ids.iter().enumerate() {
            let (_, concave, convex) = &per_loop[li];
            let path = &mut plan.paths[pi];
            let vi = match assigned[li] {
                Some((mut vi, ti)) => {
                    if mode == SeamMode::Aligned {
                        if let Some((_, ci)) = concave
                            .iter()
                            .chain(convex.iter())
                            .map(|c| (pt_dist_mm(path.points[c.idx], tracks[ti]), c.idx))
                            .filter(|(cd, _)| *cd <= CORNER_SNAP_MM)
                            .min_by(|a, b| a.0.total_cmp(&b.0))
                        {
                            vi = ci;
                        }
                    }
                    vi
                }
                None => match mode {
                    SeamMode::Sharpest => best_corner(&path.points, concave, convex),
                    _ => None,
                }
                .unwrap_or_else(|| {
                    (0..path.points.len())
                        .max_by_key(|&i| (path.points[i].y, path.points[i].x))
                        .unwrap()
                }),
            };
            path.points.rotate_left(vi);
            match assigned[li] {
                Some((_, ti)) => tracks[ti] = path.points[0],
                None => tracks.push(path.points[0]),
            }
        }
    }
}

/// Minimum windowed sharpness (1 − cos turn) for a vertex to count as a
/// seam-worthy corner: ≈30° of direction change across the window. Below it
/// a loop is treated as smooth — "sharpest" would otherwise latch onto
/// discretization noise and scatter layer to layer.
const SEAM_CORNER_MIN_SHARP: f64 = 0.13;

/// How far (mm of arc, each way) the corner test looks when measuring the
/// turn. Real models round their corners: a 90° transom corner with a 1.5 mm
/// fillet never turns more than a few degrees at any single vertex, but at
/// print scale it is still the corner where a seam belongs. Chords this long
/// see the fillet's full turn while a gentle hull curve still reads smooth.
const SEAM_CORNER_WINDOW_MM: f64 = 1.25;

/// A corner candidate: vertex index + its windowed sharpness.
struct Corner {
    idx: usize,
    sharp: f64,
}

/// The real corners of a closed loop, split by concavity (concave corners
/// turn into the material — a seam bead tucks into the notch out of sight —
/// so seeding prefers them). Sharpness is measured between chords reaching
/// `SEAM_CORNER_WINDOW_MM` of arc each way, so filleted corners count and a
/// run of fillet vertices simply yields a cluster of candidates. Both lists
/// empty = smooth loop.
fn corner_candidates(points: &[Point]) -> (Vec<Corner>, Vec<Corner>) {
    let n = points.len();
    // Cumulative arc length around the ring, for the window walks.
    let mut cum = vec![0.0_f64; n + 1];
    for k in 0..n {
        cum[k + 1] = cum[k] + pt_dist_mm(points[k], points[(k + 1) % n]);
    }
    let total = cum[n];
    if total <= 1.0e-6 {
        return (Vec::new(), Vec::new());
    }
    // Tiny loops can't fit the full window without folding onto themselves.
    let w = SEAM_CORNER_WINDOW_MM.min(total / 4.0);
    // Shoelace orientation: a turn opposing the loop's winding is concave
    // regardless of which way the loop happens to be wound.
    let mut area2 = 0.0;
    for i in 0..n {
        let (a, b) = (points[i], points[(i + 1) % n]);
        area2 += a.x_mm() * b.y_mm() - b.x_mm() * a.y_mm();
    }
    let (mut concave, mut convex) = (Vec::new(), Vec::new());
    for i in 0..n {
        // First vertex ≥ w of arc behind / ahead of i along the ring.
        let mut back = (i + n - 1) % n;
        while arc_between(&cum, total, back, i) < w {
            back = (back + n - 1) % n;
        }
        let mut fwd = (i + 1) % n;
        while arc_between(&cum, total, i, fwd) < w {
            fwd = (fwd + 1) % n;
        }
        let cur = points[i];
        let a = unit(cur.x_mm() - points[back].x_mm(), cur.y_mm() - points[back].y_mm());
        let b = unit(points[fwd].x_mm() - cur.x_mm(), points[fwd].y_mm() - cur.y_mm());
        let sharp = 1.0 - (a.0 * b.0 + a.1 * b.1);
        if sharp < SEAM_CORNER_MIN_SHARP {
            continue;
        }
        let cross = a.0 * b.1 - a.1 * b.0;
        let c = Corner { idx: i, sharp };
        if cross * area2 < 0.0 {
            concave.push(c);
        } else {
            convex.push(c);
        }
    }
    (concave, convex)
}

/// Arc length walking forward around the ring from vertex `from` to `to`.
fn arc_between(cum: &[f64], total: f64, from: usize, to: usize) -> f64 {
    if from == to {
        return 0.0;
    }
    let d = cum[to] - cum[from];
    if d >= 0.0 { d } else { d + total }
}

/// Best stand-alone corner: the sharpest concave one when the loop has any
/// (the seam hides in the notch), else the sharpest convex; rear-most on ties.
fn best_corner(points: &[Point], concave: &[Corner], convex: &[Corner]) -> Option<usize> {
    let cands = if concave.is_empty() { convex } else { concave };
    cands
        .iter()
        .max_by(|a, b| {
            a.sharp.total_cmp(&b.sharp).then_with(|| {
                (points[a.idx].y, points[a.idx].x).cmp(&(points[b.idx].y, points[b.idx].x))
            })
        })
        .map(|c| c.idx)
}

fn unit(x: f64, y: f64) -> (f64, f64) {
    let len = (x * x + y * y).sqrt();
    if len > 0.0 {
        (x / len, y / len)
    } else {
        (0.0, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo2d::Contour;

    // A flat square patch in the z=10 plane, split into 2 triangles, wound so the
    // outward normal points +z (toward a viewer above). Models a flooded front
    // surface facet the brush painted.
    fn flat_patch() -> Vec<[[f64; 3]; 3]> {
        let a = [0.0, 0.0, 10.0];
        let b = [4.0, 0.0, 10.0];
        let c = [4.0, 4.0, 10.0];
        let d = [0.0, 4.0, 10.0];
        vec![[a, b, c], [a, c, d]] // CCW from +z ⇒ normal +z
    }

    #[test]
    fn dab_covers_outer_shell_not_interior_or_backside() {
        let tris = flat_patch();
        let inside = [2.0, 2.0, 10.0]; // over the patch, laterally centered
        // Outer wall: ~0.2 mm below the surface → painted.
        assert!(dab_covers(&tris, [inside[0], inside[1], 9.8]));
        // Right at the surface → painted.
        assert!(dab_covers(&tris, [inside[0], inside[1], 10.0]));
        // Deep interior / inner walls (1 mm in, past PAINT_SURF_IN) → NOT painted:
        // this is the "color from the inside" the brush must stop reaching.
        assert!(!dab_covers(&tris, [inside[0], inside[1], 9.0]));
        // Well above the surface (0.6 mm out, past PAINT_SURF_OUT) → NOT painted.
        assert!(!dab_covers(&tris, [inside[0], inside[1], 10.6]));
    }

    #[test]
    fn dab_covers_stops_at_the_patch_edge_and_disconnected_surfaces() {
        let tris = flat_patch(); // covers x,y in [0,4]
        // A bead laterally well outside the patch (even at the right depth) is not
        // covered — the brush can't jump to a nearby disconnected surface.
        assert!(!dab_covers(&tris, [9.0, 2.0, 9.9]));
        // A bead just past the edge, within the lateral tolerance, still counts
        // (bridges mesh sampling); far past it does not.
        assert!(dab_covers(&tris, [4.0 + PAINT_LAT_TOL * 0.5, 2.0, 9.9]));
        assert!(!dab_covers(&tris, [4.0 + PAINT_LAT_TOL * 3.0, 2.0, 9.9]));
    }

    fn count(layer: &LayerPlan, kind: PathKind) -> usize {
        layer.paths.iter().filter(|p| p.kind == kind).count()
    }

    /// A structural fingerprint of a plan: per layer, its z and the ordered
    /// per-path (tool, kind, closed, quantized points). Captures tool
    /// assignment, path geometry, AND the by-tool ordering — everything a
    /// paint change can touch.
    #[allow(clippy::type_complexity)]
    fn fp(plans: &[LayerPlan]) -> Vec<(usize, i64, Vec<(u32, PathKind, bool, Vec<(i64, i64)>)>)> {
        plans
            .iter()
            .map(|l| {
                (
                    l.index,
                    (l.print_z_mm * 1e6).round() as i64,
                    l.paths
                        .iter()
                        .map(|p| {
                            let pts = p
                                .points
                                .iter()
                                .map(|pt| ((pt.x_mm() * 1e5).round() as i64, (pt.y_mm() * 1e5).round() as i64))
                                .collect();
                            (p.tool, p.kind, p.closed, pts)
                        })
                        .collect(),
                )
            })
            .collect()
    }

    #[test]
    fn restamp_matches_full_slice() {
        // Stepped part (its shelf exercises the blend face-override, the
        // trickiest paint stage) + a second part on another tool.
        let mut t = Vec::new();
        push_box(&mut t, [0.0, 0.0, 0.0], [20.0, 20.0, 2.0]);
        push_box(&mut t, [6.0, 6.0, 2.0], [14.0, 14.0, 4.0]);
        let stepped = mesh::Mesh::from_triangle_soup(&t);
        let mut sl = Vec::new();
        push_box(&mut sl, [22.0, 0.0, 0.0], [32.0, 10.0, 3.0]);
        let slab = mesh::Mesh::from_triangle_soup(&sl);

        let mut s = Settings::default();
        s.tool_count = 3;
        s.tools = (0..3).map(|i| s.flat_tool(format!("t{i}"))).collect();
        s.auto_center_on_bed = false;

        // Geometry is paint-independent, so one cache serves any paint.
        let parts_a: Vec<(&Mesh, PartPaint)> =
            vec![(&stepped, PartPaint::Blend(vec![(0, 3.0), (1, 1.0)])), (&slab, PartPaint::Tool(2))];
        let geo = plan_geometry(&parts_a, &s);

        // Restamping the SAME paint must reproduce a full slice exactly.
        assert_eq!(
            fp(&restamp_paint(&geo, &parts_a)),
            fp(&generate_painted(&parts_a, &s)),
            "restamp with the same paint must equal a full slice",
        );

        // Restamping a DIFFERENT paint (moved tools + weights) must equal a full
        // slice with that paint — the whole point of the cache.
        let parts_b: Vec<(&Mesh, PartPaint)> =
            vec![(&stepped, PartPaint::Blend(vec![(0, 1.0), (2, 1.0)])), (&slab, PartPaint::Tool(0))];
        let full_b = generate_painted(&parts_b, &s);
        assert_eq!(
            fp(&restamp_paint(&geo, &parts_b)),
            fp(&full_b),
            "restamp with new paint must equal a full slice with that paint",
        );

        // Guard against a vacuous test: the two paints really do differ.
        assert_ne!(
            fp(&generate_painted(&parts_a, &s)),
            fp(&full_b),
            "test setup: paints A and B must produce different plans",
        );
    }

    /// Axis-aligned box as a triangle soup (outward winding, same pattern as
    /// `Mesh::cube`).
    fn push_box(tris: &mut Vec<[[f64; 3]; 3]>, lo: [f64; 3], hi: [f64; 3]) {
        let v = [
            [lo[0], lo[1], lo[2]],
            [hi[0], lo[1], lo[2]],
            [hi[0], hi[1], lo[2]],
            [lo[0], hi[1], lo[2]],
            [lo[0], lo[1], hi[2]],
            [hi[0], lo[1], hi[2]],
            [hi[0], hi[1], hi[2]],
            [lo[0], hi[1], hi[2]],
        ];
        for t in [
            [0, 2, 1], [0, 3, 2],
            [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4],
            [3, 6, 2], [3, 7, 6],
            [0, 7, 3], [0, 4, 7],
            [1, 2, 6], [1, 6, 5],
        ] {
            tris.push([v[t[0]], v[t[1]], v[t[2]]]);
        }
    }

    /// External-wall seam point (loop start) of every layer, bottom-up.
    fn external_seams(layers: &[LayerPlan]) -> Vec<(usize, f64, f64)> {
        let mut out = Vec::new();
        for l in layers {
            for p in &l.paths {
                if p.kind == PathKind::ExternalPerimeter && p.closed && p.points.len() >= 3 {
                    out.push((l.index, p.points[0].x_mm(), p.points[0].y_mm()));
                }
            }
        }
        out
    }

    #[test]
    fn sharpest_seam_locks_to_the_concave_corner() {
        // An L-bracket: the only concave corner is the inner notch. Every
        // layer's external seam must sit there, in one vertical column.
        let mut tris = Vec::new();
        push_box(&mut tris, [0.0, 0.0, 0.0], [20.0, 10.0, 6.0]);
        push_box(&mut tris, [0.0, 0.0, 0.0], [10.0, 20.0, 6.0]);
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let mut s = Settings::default();
        s.seam_mode = SeamMode::Sharpest;
        s.skirt_loops = 0;
        let layers = generate(&m, &s);
        let seams = external_seams(&layers);
        assert!(seams.len() >= layers.len(), "one external loop per layer");
        // The notch vertex of the printed wall loop sits half a bead inside
        // the model's (10,10) corner; everything lands centered on the bed,
        // so compare against the centered corner position instead.
        let (cx, cy) = (s.bed_size_x_mm / 2.0, s.bed_size_y_mm / 2.0);
        let (corner_x, corner_y) = (cx + (10.0 - 10.0), cy + (10.0 - 10.0)); // model center = (10,10)
        for &(li, x, y) in &seams {
            let d = ((x - corner_x).powi(2) + (y - corner_y).powi(2)).sqrt();
            assert!(d < 1.5, "layer {li}: seam {x:.2},{y:.2} not at the notch (d={d:.2})");
        }
        // And the column is steady: consecutive seams barely move.
        for w in seams.windows(2) {
            let d = ((w[1].1 - w[0].1).powi(2) + (w[1].2 - w[0].2).powi(2)).sqrt();
            assert!(d < 1.0, "seam jumped {d:.2} mm between layers {} and {}", w[0].0, w[1].0);
        }
    }

    #[test]
    fn sharpest_seam_finds_filleted_corners() {
        // A 20×20 box with r=1.5 corner fillets — like a hull transom, no
        // single vertex turns more than a few degrees, but the fillets are
        // still the corners. The seam must land in one fillet and stay put,
        // not fall back to the middle of a smooth face.
        let mut tris: Vec<[[f64; 3]; 3]> = Vec::new();
        let (half, r, steps) = (10.0_f64, 1.5_f64, 12_usize);
        let mut outline: Vec<(f64, f64)> = Vec::new();
        for (k, (cx, cy, a0)) in [
            (half - r, half - r, 0.0_f64),
            (-(half - r), half - r, 90.0),
            (-(half - r), -(half - r), 180.0),
            (half - r, -(half - r), 270.0),
        ]
        .into_iter()
        .enumerate()
        {
            let _ = k;
            for s in 0..=steps {
                let a = (a0 + 90.0 * s as f64 / steps as f64).to_radians();
                outline.push((cx + r * a.cos(), cy + r * a.sin()));
            }
        }
        let n = outline.len();
        for k in 0..n {
            let (x0, y0) = outline[k];
            let (x1, y1) = outline[(k + 1) % n];
            tris.push([[x0, y0, 0.0], [x1, y1, 0.0], [x1, y1, 4.0]]);
            tris.push([[x0, y0, 0.0], [x1, y1, 4.0], [x0, y0, 4.0]]);
            tris.push([[0.0, 0.0, 0.0], [x1, y1, 0.0], [x0, y0, 0.0]]);
            tris.push([[0.0, 0.0, 4.0], [x0, y0, 4.0], [x1, y1, 4.0]]);
        }
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let mut s = Settings::default();
        s.seam_mode = SeamMode::Sharpest;
        s.skirt_loops = 0;
        let layers = generate(&m, &s);
        let seams = external_seams(&layers);
        assert!(seams.len() >= layers.len());
        let (cx, cy) = (s.bed_size_x_mm / 2.0, s.bed_size_y_mm / 2.0);
        for &(li, x, y) in &seams {
            // In a fillet = within ~2.2mm of one of the four fillet centers
            // (the wall loop runs half a bead inside the outline).
            let (dx, dy) = ((x - cx).abs(), (y - cy).abs());
            let d = ((dx - (half - r)).powi(2) + (dy - (half - r)).powi(2)).sqrt();
            assert!(d < r + 0.8, "layer {li}: seam {x:.2},{y:.2} not in a corner fillet (d={d:.2})");
        }
        for w in seams.windows(2) {
            let d = ((w[1].1 - w[0].1).powi(2) + (w[1].2 - w[0].2).powi(2)).sqrt();
            assert!(d < 1.5, "fillet seam jumped {d:.2} mm at layer {}", w[1].0);
        }
    }

    #[test]
    fn sharpest_seam_stays_in_a_column_on_smooth_loops() {
        // A 24-gon prism has no vertex past the corner threshold (15° turns):
        // "sharpest" must degrade to an aligned column, not per-layer noise.
        let mut tris: Vec<[[f64; 3]; 3]> = Vec::new();
        let n = 24;
        let r = 10.0;
        let pt = |k: usize| {
            let a = (k % n) as f64 / n as f64 * std::f64::consts::TAU;
            (r * a.cos(), r * a.sin())
        };
        for k in 0..n {
            let (x0, y0) = pt(k);
            let (x1, y1) = pt(k + 1);
            // Side quad (outward), bottom fan, top fan.
            tris.push([[x0, y0, 0.0], [x1, y1, 0.0], [x1, y1, 6.0]]);
            tris.push([[x0, y0, 0.0], [x1, y1, 6.0], [x0, y0, 6.0]]);
            tris.push([[0.0, 0.0, 0.0], [x1, y1, 0.0], [x0, y0, 0.0]]);
            tris.push([[0.0, 0.0, 6.0], [x0, y0, 6.0], [x1, y1, 6.0]]);
        }
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let mut s = Settings::default();
        s.seam_mode = SeamMode::Sharpest;
        s.skirt_loops = 0;
        let layers = generate(&m, &s);
        let seams = external_seams(&layers);
        assert!(seams.len() >= layers.len());
        for w in seams.windows(2) {
            let d = ((w[1].1 - w[0].1).powi(2) + (w[1].2 - w[0].2).powi(2)).sqrt();
            assert!(d < 3.0, "smooth loop seam wandered {d:.2} mm at layer {}", w[1].0);
        }
    }

    #[test]
    fn overhanging_walls_slow_down() {
        // A 2mm base with a slab cantilevering 10mm past it: the slab's first
        // layer walls over air must come out as OverhangWall (slow + cooled),
        // while walls over the base stay normal.
        let mut tris = Vec::new();
        push_box(&mut tris, [0.0, 0.0, 0.0], [20.0, 20.0, 2.0]);
        push_box(&mut tris, [0.0, 0.0, 2.0], [20.0, 30.0, 4.0]);
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let s = Settings { skirt_loops: 0, ..Settings::default() };
        let plans = generate(&m, &s);

        // The slab's first layer is the one printed at z just above 2mm.
        let first_slab = plans.iter().find(|p| p.print_z_mm > 2.0).unwrap();
        // Overhang is now marked per-SEGMENT on the wall bead (kept whole, not cut):
        // a mixed wall stays ExternalPerimeter/Perimeter and carries OverhangWall
        // segments; a wholly-airborne loop is a uniform OverhangWall. Gather every
        // over-air segment and check it lives in the cantilever (y > 19).
        let mut over_segs = 0usize;
        for p in &first_slab.paths {
            let n = p.points.len();
            if n < 2 {
                continue;
            }
            let nseg = if p.closed { n } else { n - 1 };
            for k in 0..nseg {
                let seg_kind = match &p.segs {
                    Some(sa) => sa[k.min(sa.len() - 1)].kind,
                    None => p.kind,
                };
                if seg_kind == PathKind::OverhangWall {
                    over_segs += 1;
                    let (a, b) = (p.points[k], p.points[(k + 1) % n]);
                    let my = (a.y_mm() + b.y_mm()) / 2.0;
                    assert!(my > 19.0, "overhang segment at supported y={my:.1}");
                }
            }
        }
        assert!(over_segs > 0, "cantilever walls must carry overhang segments");
        // Supported walls keep their kind, and the mixed wall stays one closed bead.
        assert!(
            first_slab.paths.iter().any(|p| matches!(p.kind, PathKind::ExternalPerimeter | PathKind::Perimeter)),
            "supported walls keep their kind"
        );
        // The layer above the cantilever's first is fully supported again: no
        // OverhangWall paths and no overhang segments.
        let next = plans.iter().find(|p| p.print_z_mm > first_slab.print_z_mm).unwrap();
        assert_eq!(count(next, PathKind::OverhangWall), 0, "supported layer must not slow");
        assert!(
            next.paths.iter().all(|p| p.segs.is_none()),
            "a fully-supported layer has no overhang segments"
        );
    }

    #[test]
    fn aligned_seams_follow_one_column() {
        // A cylinder-ish prism: the rear-most vertex is ambiguous (two
        // vertices straddle the rear), so per-layer placement can flip
        // between them; aligned mode must hold one continuous column.
        let mut tris = Vec::new();
        let n_side = 16;
        let (cx, cy, r, h) = (10.0, 10.0, 8.0, 6.0);
        let ring = |z: f64| -> Vec<[f64; 3]> {
            (0..n_side)
                .map(|k| {
                    // Half-step phase: no vertex exactly at the rear.
                    let a = std::f64::consts::TAU * (k as f64 + 0.5) / n_side as f64;
                    [cx + r * a.cos(), cy + r * a.sin(), z]
                })
                .collect()
        };
        let (b, t) = (ring(0.0), ring(h));
        for k in 0..n_side {
            let k2 = (k + 1) % n_side;
            tris.push([b[k], b[k2], t[k2]]);
            tris.push([b[k], t[k2], t[k]]);
            tris.push([[cx, cy, 0.0], b[k2], b[k]]);
            tris.push([[cx, cy, h], t[k], t[k2]]);
        }
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let s = Settings { skirt_loops: 0, seam_mode: config::SeamMode::Aligned, ..Settings::default() };
        let plans = generate(&m, &s);
        let starts: Vec<Point> = plans
            .iter()
            .filter_map(|p| {
                p.paths
                    .iter()
                    .find(|t| t.kind == PathKind::ExternalPerimeter && t.closed)
                    .map(|t| t.points[0])
            })
            .collect();
        assert!(starts.len() > 10, "need a stack of outer loops");
        let max_step = starts
            .windows(2)
            .map(|w| pt_dist_mm(w[0], w[1]))
            .fold(0.0f64, f64::max);
        // Vertices are ~3mm apart on this prism; consecutive seams must stay
        // on the same vertex column, not flip to the twin across the rear.
        assert!(max_step < 1.5, "seam jumped {max_step:.2}mm between layers");
    }

    #[test]
    fn anchored_span_bridges_in_every_mode() {
        // A table: slab across two legs with a 4mm air gap between them.
        let mut tris = Vec::new();
        push_box(&mut tris, [0.0, 0.0, 0.0], [4.0, 10.0, 4.0]);
        push_box(&mut tris, [8.0, 0.0, 0.0], [12.0, 10.0, 4.0]);
        push_box(&mut tris, [0.0, 0.0, 4.0], [12.0, 10.0, 6.0]);
        let m = mesh::Mesh::from_triangle_soup(&tris);
        // A span anchored on both legs is reliably bridgeable — correct bottom-
        // surface printing, not a rescue — so it bridges even with no support mode.
        let none = generate(&m, &Settings { skirt_loops: 0, ..Settings::default() });
        let slab = none.iter().find(|p| p.print_z_mm > 4.05).unwrap();
        assert!(count(slab, PathKind::Bridge) > 0, "anchored span bridges without support mode");
    }

    fn closed_wall_areas(plan: &LayerPlan) -> Vec<f64> {
        plan.paths
            .iter()
            .filter(|p| {
                p.closed
                    && matches!(
                        p.kind,
                        PathKind::Perimeter | PathKind::ExternalPerimeter | PathKind::OverhangWall
                    )
            })
            .map(|p| {
                let pts = &p.points;
                let m = pts.len();
                (0..m)
                    .map(|j| {
                        let (a, b) = (pts[j], pts[(j + 1) % m]);
                        a.x_mm() * b.y_mm() - b.x_mm() * a.y_mm()
                    })
                    .sum::<f64>()
                    .abs()
                    * 0.5
            })
            .collect()
    }

    #[test]
    fn enclosed_void_walls_print_outer_in() {
        // Four walls form a ring around a void; a cap slab tops it. With no bottom
        // shell the cap's first layer over the void prints as concentric walls, and
        // they must lay supported-edge inward (outer→in / area-descending).
        let mut tris = Vec::new();
        push_box(&mut tris, [0.0, 0.0, 0.0], [2.0, 12.0, 8.0]);
        push_box(&mut tris, [10.0, 0.0, 0.0], [12.0, 12.0, 8.0]);
        push_box(&mut tris, [0.0, 0.0, 0.0], [12.0, 2.0, 8.0]);
        push_box(&mut tris, [0.0, 10.0, 0.0], [12.0, 12.0, 8.0]);
        push_box(&mut tris, [0.0, 0.0, 8.0], [12.0, 12.0, 10.0]);
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let s = Settings { skirt_loops: 0, wall_count: 20, top_layers: 0, bottom_layers: 0, ..Settings::default() };
        let plans = generate(&m, &s);
        let cap = plans.iter().find(|p| p.print_z_mm > 8.05).unwrap();
        let areas = closed_wall_areas(cap);
        assert!(areas.len() >= 3, "cap should have several rings, got {}", areas.len());
        assert!(
            areas.windows(2).all(|w| w[0] >= w[1] - 1.0),
            "enclosed rings must print outer→in: {areas:?}"
        );
    }

    #[test]
    fn cantilever_walls_are_left_alone() {
        // A shelf on a single-edge post: most of it overhangs air. outer→in would
        // lay the unsupported outer span first, so the cantilever must NOT be
        // reordered — its rings keep travel order (not forced area-descending).
        let mut tris = Vec::new();
        push_box(&mut tris, [0.0, 0.0, 0.0], [2.0, 12.0, 8.0]);
        push_box(&mut tris, [0.0, 0.0, 8.0], [12.0, 12.0, 10.0]);
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let s = Settings { skirt_loops: 0, wall_count: 20, top_layers: 0, bottom_layers: 0, ..Settings::default() };
        let plans = generate(&m, &s);
        let shelf = plans.iter().find(|p| p.print_z_mm > 8.05).unwrap();
        let areas = closed_wall_areas(shelf);
        assert!(areas.len() >= 3, "shelf should have rings, got {}", areas.len());
        assert!(
            !areas.windows(2).all(|w| w[0] >= w[1] - 1.0),
            "cantilever must not be reordered outer→in: {areas:?}"
        );
    }

    #[test]
    fn solid_over_low_sparse_internal_bridges() {
        // Any solid/shell layer over low-density (≤15%) sparse spans mostly air
        // between the infill lines, so it prints as an internal bridge — buried OR
        // visible top shell, wherever it's over open sparse.
        let m = mesh::Mesh::cube(20.0);
        let low = Settings { skirt_loops: 0, infill_density: 0.10, ..Settings::default() };
        let plans = generate(&m, &low);
        let bridges: usize = plans.iter().map(|p| count(p, PathKind::InternalBridge)).sum();
        assert!(bridges > 0, "low-density solid-over-sparse must internal-bridge");
        // The roof here sits over the solid top shell (not sparse), so it stays a
        // normal top skin — only layers over OPEN sparse bridge.
        let roof = plans.last().unwrap();
        assert!(count(roof, PathKind::TopSkin) > 0, "roof over solid stays top skin");
        assert_eq!(count(roof, PathKind::InternalBridge), 0, "roof isn't over sparse");

        // With a single top layer, the VISIBLE top is the layer over sparse — it
        // bridges too (bridge flow runs high to fill the missing volume; not spared).
        let thin = Settings { skirt_loops: 0, infill_density: 0.10, top_layers: 1, ..Settings::default() };
        let plans = generate(&m, &thin);
        let roof = plans.last().unwrap();
        assert!(count(roof, PathKind::InternalBridge) > 0, "single top layer over sparse bridges");

        // Above the threshold the sparse is dense enough to support the layer:
        // plain solid, no internal bridges.
        let dense = Settings { skirt_loops: 0, infill_density: 0.30, ..Settings::default() };
        let plans = generate(&m, &dense);
        for p in &plans {
            assert_eq!(count(p, PathKind::InternalBridge), 0, "layer {} bridges at 30%", p.index);
        }
    }

    #[test]
    fn rebalance_demotes_junk_solid_to_sparse() {
        // A 0.3 mm solid band (staircase sliver): nowhere wide enough for a
        // bead — its area must move to the sparse region, not print a micro
        // hairpin or silently vanish.
        let solid = rect(0.0, 0.0, 10.0, 0.3);
        let sparse = rect(0.0, 0.3, 10.0, 5.0);
        let (solid, sparse) = rebalance_solid_sparse(solid, sparse, 0.45);
        assert!(solid.is_empty(), "junk band stayed solid");
        assert!((sparse.net_area_mm2() - 50.0).abs() < 0.3, "area not reallocated: {}", sparse.net_area_mm2());
    }

    #[test]
    fn rebalance_keeps_printable_solid_and_promotes_pockets() {
        // A 2 mm band prints fine; a lonely 4 mm² sparse pocket pours solid
        // (the established solid-infill-below-area behavior survives).
        let solid = rect(0.0, 0.0, 10.0, 2.0);
        let pocket = rect(20.0, 0.0, 22.0, 2.0);
        let (solid, sparse) = rebalance_solid_sparse(solid, pocket, 0.45);
        assert!((solid.net_area_mm2() - 24.0).abs() < 0.1, "solid area {}", solid.net_area_mm2());
        assert!(sparse.is_empty());
    }

    #[test]
    fn rebalance_isolated_crumb_does_not_bounce_back() {
        // A 0.6×0.6 mm solid crumb: junk by area. The promotion pass must not
        // hand it straight back to solid (the junk floor).
        let crumb = rect(0.0, 0.0, 0.6, 0.6);
        let (solid, sparse) = rebalance_solid_sparse(crumb, Polygons::new(), 0.45);
        assert!(solid.is_empty(), "crumb returned to solid");
        assert!((sparse.net_area_mm2() - 0.36).abs() < 0.05);
    }

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
    fn spiralize_chains_nested_rings_into_one_stroke() {
        // Five nested square rings 1 mm apart — concentric fill of a square.
        let ring = |r: f64| {
            vec![
                Point::from_mm(-r, -r),
                Point::from_mm(r, -r),
                Point::from_mm(r, r),
                Point::from_mm(-r, r),
            ]
        };
        let rings: Vec<(Vec<Point>, PathKind)> =
            (1..=5).map(|k| (ring(k as f64), PathKind::Perimeter)).collect();
        let mut region = Polygons::new();
        region.push(Contour::new(ring(5.5))); // holds every crossover in-region
        let strokes = spiralize_rings(rings, &region, 2.0, SeamMode::Nearest, 0);
        // Adjacent rings are √2 mm apart (< 2 mm crossover cap), so all five chain
        // into ONE continuous spiral stroke — one seam, not five.
        assert_eq!(strokes.len(), 1, "nested rings should chain into one spiral");
        assert!(strokes[0].0.len() >= 20, "the stroke should trace all five rings");
        // All rings share a kind, so the stroke is uniform (no per-segment override).
        assert!(strokes[0].1.iter().all(|&k| k == PathKind::Perimeter));
    }

    #[test]
    fn is_annular_fires_only_on_ringed_regions() {
        // A plain filled square is NOT annular: a concentric loop there would
        // just double the perimeter, so it keeps line fill.
        assert!(!is_annular(&rect(0.0, 0.0, 10.0, 10.0)));
        // A washer — outer square with a centered CW hole — rings a cavity.
        let mut washer = rect(0.0, 0.0, 10.0, 10.0);
        washer.push(Contour::new(vec![
            Point::from_mm(3.0, 3.0),
            Point::from_mm(3.0, 7.0),
            Point::from_mm(7.0, 7.0),
            Point::from_mm(7.0, 3.0),
        ])); // CW winding => a hole
        assert!(is_annular(&washer));
        // Two separate filled squares — two islands, neither ringed — stay line fill.
        let mut two = rect(0.0, 0.0, 4.0, 4.0);
        two.contours.extend(rect(8.0, 0.0, 12.0, 4.0).contours);
        assert!(!is_annular(&two));
        // Mixed: a washer beside a plain square. Conservative "every island rings"
        // rule => not annular, so the plain square is never given a doubling loop.
        let mut mixed = washer.clone();
        mixed.contours.extend(rect(20.0, 0.0, 24.0, 4.0).contours);
        assert!(!is_annular(&mixed));
    }

    #[test]
    fn wide_holed_plate_keeps_line_fill_thin_ring_goes_concentric() {
        // Concentric fill is gated on BOTH annular AND narrow. A wide plate with a
        // small hole (the Benchy deck) is annular but not a ring-band, so it keeps its
        // configured line fill — the bug where 2 vs 4 walls flipped it to concentric.
        let lw = 0.45;
        let hole = |x0: f64, y0: f64, x1: f64, y1: f64| {
            // CW winding => a hole.
            Contour::new(vec![
                Point::from_mm(x0, y0),
                Point::from_mm(x0, y1),
                Point::from_mm(x1, y1),
                Point::from_mm(x1, y0),
            ])
        };
        // Wide washer: 8 mm band around a small hole — annular but NOT narrow.
        let mut wide = rect(0.0, 0.0, 20.0, 20.0);
        wide.push(hole(8.0, 8.0, 12.0, 12.0));
        assert!(is_annular(&wide));
        assert!(!is_narrow_band(&wide, lw), "a wide holed plate is not a ring-band");
        // Thin ring: 2 mm band — a genuine cavity ring, worth concentric.
        let mut thin = rect(0.0, 0.0, 10.0, 10.0);
        thin.push(hole(2.0, 2.0, 8.0, 8.0));
        assert!(is_annular(&thin));
        assert!(is_narrow_band(&thin, lw), "a thin annular band IS a ring-band");
    }

    #[test]
    fn bridge_lines_span_a_narrow_two_sided_gap() {
        // A 20×4mm slot supported on its two long sides → bridge with short lines.
        let region = rect(0.0, 0.0, 20.0, 4.0);
        let mut supported = rect(-3.0, -3.0, 23.0, 0.0);
        supported.contours.extend(rect(-3.0, 4.0, 23.0, 7.0).contours);
        let lines = try_bridge(&region, &supported, 0.45, 6.0).expect("should bridge");
        let max_len = lines
            .iter()
            .filter(|s| s.len() >= 2)
            .map(|s| pt_dist_mm(s[0], s[s.len() - 1]))
            .fold(0.0, f64::max);
        assert!(max_len < 5.0, "lines should cross the 4mm gap, got {max_len:.1}mm");
    }

    #[test]
    fn wide_span_is_not_bridged() {
        let region = rect(0.0, 0.0, 20.0, 20.0);
        let supported = rect(-3.0, -3.0, 23.0, 23.0);
        assert!(try_bridge(&region, &supported, 0.45, 6.0).is_none(), "20mm > 6mm max span");
    }

    #[test]
    fn cantilever_is_not_bridged() {
        // Supported on one side only → no line is anchored at both ends.
        let region = rect(0.0, 0.0, 6.0, 6.0);
        let supported = rect(-3.0, -3.0, 9.0, 0.0);
        assert!(try_bridge(&region, &supported, 0.45, 6.0).is_none(), "one-sided support can't bridge");
    }

    #[test]
    fn islands_splits_disjoint_gaps() {
        // Two separate gaps must be decided independently (small → lines, wide → arcs).
        let mut p = rect(0.0, 0.0, 4.0, 18.0);
        p.contours.extend(rect(20.0, 0.0, 40.0, 18.0).contours);
        let isl = islands(&p);
        assert_eq!(isl.len(), 2, "two disjoint gaps → two islands");
        let supported = {
            let mut s = rect(-3.0, 0.0, 0.0, 18.0);
            s.contours.extend(rect(4.0, 0.0, 7.0, 18.0).contours);
            s.contours.extend(rect(17.0, 0.0, 20.0, 18.0).contours);
            s.contours.extend(rect(40.0, 0.0, 43.0, 18.0).contours);
            s
        };
        // 4mm island bridges; 20mm island does not.
        let narrow = if isl[0].bounds().unwrap().width() < isl[1].bounds().unwrap().width() { &isl[0] } else { &isl[1] };
        let wide = if std::ptr::eq(narrow, &isl[0]) { &isl[1] } else { &isl[0] };
        assert!(try_bridge(narrow, &supported, 0.45, 6.0).is_some(), "4mm gap should bridge");
        assert!(try_bridge(wide, &supported, 0.45, 6.0).is_none(), "20mm gap should not");
    }

    #[test]
    fn cube_plan_has_walls_and_infill() {
        let m = Mesh::cube(20.0);
        let s = Settings::default();
        let layers = generate(&m, &s);
        assert_eq!(layers.len(), 100);

        let mid = &layers[50];
        assert_eq!(
            count(mid, PathKind::ExternalPerimeter) + count(mid, PathKind::Perimeter),
            s.wall_count,
            "two concentric wall loops"
        );

        // Outer wall offset inward: 20 - 2*(0.5*0.45) = 19.55mm => ~382mm²
        // (translation-invariant, so bed-centering doesn't change it). This also
        // proves the offset sign.
        let ext = mid
            .paths
            .iter()
            .find(|p| p.kind == PathKind::ExternalPerimeter)
            .unwrap();
        let area = Contour::new(ext.points.clone()).area_mm2();
        assert!(area > 360.0 && area < 400.0, "outer wall area {area}");
    }

    #[test]
    fn cube_has_solid_top_bottom_sparse_middle() {
        let m = Mesh::cube(20.0);
        let s = Settings::default(); // 4 top / 4 bottom
        let layers = generate(&m, &s);

        // Bottom and top shells are solid — the exposed faces as skins, the
        // covered shell layers buried Solid; the middle is sparse only.
        assert!(count(&layers[0], PathKind::BottomSkin) > 0, "bed face is bottom skin");
        assert!(count(&layers[1], PathKind::Solid) > 0, "covered bottom shell solid");
        assert!(count(&layers[99], PathKind::TopSkin) > 0, "roof is top skin");
        assert!(count(&layers[98], PathKind::Solid) > 0, "covered top shell solid");

        let mid = &layers[50];
        assert!(count(mid, PathKind::Infill) > 0, "middle has sparse infill");
        assert_eq!(count(mid, PathKind::Solid), 0, "middle has no solid fill");
    }

    #[test]
    fn surface_skins_the_faces_even_when_walls_bury_the_section() {
        // walls=99 buries a small cube in concentric perimeters. The exposed top
        // and bottom faces must still print as skin (a surface overrides a wall),
        // with the outer edge perimeter kept — while buried middle layers, which
        // have no exposed face, stay walls-only exactly as before.
        let m = Mesh::cube(20.0);
        let s = Settings { wall_count: 99, ..Settings::default() };
        let layers = generate(&m, &s);

        // Exposed faces skin, and the outer wall still rings them.
        assert!(count(&layers[0], PathKind::BottomSkin) > 0, "bed face skins at walls=99");
        assert!(count(&layers[99], PathKind::TopSkin) > 0, "roof skins at walls=99");
        assert!(count(&layers[99], PathKind::ExternalPerimeter) > 0, "outer wall kept on the roof");
        // The concentric inner-ring stack the face used to be is replaced by skin.
        assert_eq!(count(&layers[99], PathKind::Perimeter), 0, "inner rings gone from the face");

        // A buried middle layer has no exposed face: walls fill it solid, no skin.
        let mid = &layers[50];
        assert!(
            count(mid, PathKind::Perimeter) + count(mid, PathKind::ExternalPerimeter) > 0,
            "middle stays walls"
        );
        assert_eq!(count(mid, PathKind::TopSkin) + count(mid, PathKind::BottomSkin), 0, "middle has no skin");
    }

    #[test]
    fn model_is_centered_on_bed() {
        let m = Mesh::cube(20.0);
        let s = Settings::default(); // 220x220 bed => center 110,110
        let layers = generate(&m, &s);
        // Cube spans 20mm, centered => roughly 100..120 in both axes.
        let p = layers[50].paths[0].points[0];
        assert!((p.x_mm() - 110.0).abs() < 12.0, "x near bed center, got {}", p.x_mm());
        assert!((p.y_mm() - 110.0).abs() < 12.0, "y near bed center, got {}", p.y_mm());
    }

    #[test]
    fn skirt_only_on_first_layer() {
        let m = Mesh::cube(20.0);
        let s = Settings::default(); // skirt_loops = 2
        let layers = generate(&m, &s);
        // Two loops around a single-region cube => 2 skirt paths on layer 0.
        assert_eq!(count(&layers[0], PathKind::Skirt), 2);
        assert_eq!(count(&layers[1], PathKind::Skirt), 0);
    }

    #[test]
    fn first_layer_height_is_honored() {
        let m = Mesh::cube(20.0);
        let s = Settings { first_layer_height_mm: 0.3, layer_height_mm: 0.2, ..Settings::default() };
        let layers = generate(&m, &s);
        assert!((layers[0].height_mm - 0.3).abs() < 1e-9);
        assert!((layers[0].print_z_mm - 0.3).abs() < 1e-9, "first layer top at 0.3");
        assert!((layers[1].print_z_mm - 0.5).abs() < 1e-9, "second layer top at 0.5");
    }

    /// An axis-aligned box `sx × sy × sz` (corner at origin) for fixture meshes.
    fn box_mesh(sx: f64, sy: f64, sz: f64) -> Mesh {
        let unit = Mesh::cube(1.0);
        Mesh {
            vertices: unit.vertices.iter().map(|v| [v[0] * sx, v[1] * sy, v[2] * sz]).collect(),
            triangles: unit.triangles.clone(),
        }
    }

    #[test]
    fn fuzzy_skin_roughens_outer_wall_only() {
        let m = Mesh::cube(20.0);
        let mut s = Settings { skirt_loops: 0, ..Settings::default() };
        s.fuzzy_skin = true;
        s.fuzzy_skin_thickness_mm = 0.3;
        s.fuzzy_skin_point_dist_mm = 0.8;
        let layers = generate(&m, &s);
        let mid = &layers[10];
        let ext = mid.paths.iter().find(|p| p.kind == PathKind::ExternalPerimeter).unwrap();
        // Densified: a 20mm square wall at 0.8mm spacing → ~90+ points (was 4).
        assert!(ext.points.len() > 50, "fuzzy wall should be densely resampled, got {}", ext.points.len());
        // Jitter stays inside the band: the cube's outer wall centerline is a
        // square ~±0.225 inside 0..20 (bed-centered, so measure spans instead).
        let xs: Vec<f64> = ext.points.iter().map(|p| p.x_mm()).collect();
        let span = xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min);
        assert!(
            (19.0..20.5).contains(&span),
            "jittered wall span {span:.2} should stay near the nominal 19.55"
        );
        // Fuzzy is outer-wall only: the inner wall is not jittered. (It is now
        // rounded by the concentric primitive's morphological open, so it carries
        // more than the bare 4 corner points — but far fewer than the fuzzed outer
        // wall's dense resample, and with no random in/out jitter.)
        let inner = mid.paths.iter().find(|p| p.kind == PathKind::Perimeter).unwrap();
        assert!(
            inner.points.len() < ext.points.len() / 2,
            "inner wall must not be fuzzed: {} pts vs outer {}",
            inner.points.len(),
            ext.points.len()
        );
        // First layer unaffected (bed adhesion).
        let l0 = layers[0].paths.iter().find(|p| p.kind == PathKind::ExternalPerimeter).unwrap();
        assert!(l0.points.len() < 20, "first layer must not be fuzzed");
    }

    #[test]
    fn elephant_foot_shrinks_first_layer() {
        let m = Mesh::cube(20.0);
        let mut s = Settings { skirt_loops: 0, ..Settings::default() };
        s.elephant_foot_mm = 0.2;
        let layers = generate(&m, &s);
        let area = |l: &LayerPlan| {
            let p = l.paths.iter().find(|p| p.kind == PathKind::ExternalPerimeter).unwrap();
            Contour::new(p.points.clone()).area_mm2()
        };
        let a0 = area(&layers[0]);
        let a1 = area(&layers[1]);
        assert!(a0 < a1 - 5.0, "first layer ({a0:.0}mm²) should be shrunk vs layer 1 ({a1:.0}mm²)");
    }

    #[test]
    fn xy_compensation_grows_every_layer() {
        let m = Mesh::cube(20.0);
        let base = generate(&m, &Settings { skirt_loops: 0, ..Settings::default() });
        let mut s = Settings { skirt_loops: 0, ..Settings::default() };
        s.xy_compensation_mm = 0.1;
        let grown = generate(&m, &s);
        let area = |l: &LayerPlan| {
            let p = l.paths.iter().find(|p| p.kind == PathKind::ExternalPerimeter).unwrap();
            Contour::new(p.points.clone()).area_mm2()
        };
        assert!(area(&grown[10]) > area(&base[10]) + 5.0, "XY comp should grow the outline");
    }



    #[test]
    fn tiny_sparse_pockets_become_solid() {
        // A 2.4 mm-wide bar: the interior pocket is ~4 mm² — far too small for
        // meaningful 15% sparse fill, so it must be promoted to solid.
        let m = box_mesh(2.4, 8.0, 5.0);
        // The pocket is too small for meaningful 15% sparse fill, so it's
        // promoted to solid instead.
        let s = Settings { skirt_loops: 0, ..Settings::default() };
        let layers = generate(&m, &s);
        let mid = &layers[10];
        assert_eq!(count(mid, PathKind::Infill), 0, "no sparse fill in a tiny pocket");
        // The pocket fills solid — as a Solid path, or (when spiralize_shells merges it
        // into the inner-wall stroke) as Solid segments carried on that stroke.
        let has_solid = mid.paths.iter().any(|p| {
            p.kind == PathKind::Solid
                || p.segs.as_ref().is_some_and(|sa| sa.iter().any(|a| a.kind == PathKind::Solid))
        });
        assert!(has_solid, "pocket promoted to solid");
    }


    #[test]
    fn walls_are_placed_at_stadium_spacing() {
        // Classic mode: this pins the exact offset constants (arachne's
        // grid-extracted rings match within a cell — covered in wall::tests).
        let m = Mesh::cube(20.0);
        let s = Settings { skirt_loops: 0, ..Settings::default() };
        let layers = generate(&m, &s);
        let mid = &layers[50];
        let span = |kind: PathKind| {
            let p = mid.paths.iter().find(|p| p.kind == kind).unwrap();
            let xs: Vec<f64> = p.points.iter().map(|pt| pt.x_mm()).collect();
            xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min)
        };
        // Outer wall centerline stays at lw/2 from the surface (dimensional
        // accuracy); the inner wall sits one *stadium spacing* further in, so
        // its span is smaller by 2·sp, not 2·lw.
        let sp = config::bead_spacing_mm(s.line_width_mm, s.layer_height_mm);
        let outer = span(PathKind::ExternalPerimeter);
        let inner = span(PathKind::Perimeter);
        assert!((outer - (20.0 - s.line_width_mm)).abs() < 0.02, "outer span {outer}");
        assert!(
            (outer - inner - 2.0 * sp).abs() < 0.02,
            "wall gap should be sp={sp:.3}: outer {outer:.3} inner {inner:.3}"
        );
    }

    #[test]
    fn solid_lines_are_spaced_at_stadium_spacing() {
        let m = Mesh::cube(20.0);
        let s = Settings { skirt_loops: 0, ..Settings::default() };
        let layers = generate(&m, &s);
        // Solid fill is always stitched into continuous runs now, so recover the
        // scanlines by projecting every solid point onto the fill-perpendicular axis
        // and clustering into distinct levels — consecutive scanlines sit one sp apart.
        let angle = 135.0_f64.to_radians();
        let proj = |pt: Point| -pt.x_mm() * angle.sin() + pt.y_mm() * angle.cos();
        let sp = config::bead_spacing_mm(s.line_width_mm, s.layer_height_mm);
        let mut levels: Vec<f64> = layers[1]
            .paths
            .iter()
            .filter(|p| p.kind == PathKind::Solid && !p.closed)
            .flat_map(|p| p.points.iter().map(|&pt| proj(pt)))
            .collect();
        levels.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut distinct: Vec<f64> = Vec::new();
        for v in levels {
            if distinct.last().map_or(true, |&last| (v - last).abs() > sp * 0.5) {
                distinct.push(v);
            }
        }
        assert!(distinct.len() > 10, "expected many scanlines, got {}", distinct.len());
        let mut gaps: Vec<f64> = distinct.windows(2).map(|w| w[1] - w[0]).collect();
        gaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = gaps[gaps.len() / 2];
        assert!(
            (median - sp).abs() < 0.01,
            "solid line spacing should be {sp:.3}, got {median:.3}"
        );
    }

    #[test]
    fn monotonic_solid_is_ordered() {
        let m = Mesh::cube(20.0);
        let s = Settings { skirt_loops: 0, ..Settings::default() }; // monotonic_solid: true
        let layers = generate(&m, &s);
        // Bottom shell: the monotonic solid is stitched into continuous run(s) now.
        // Within each run the scanline position (projection onto the fill-perpendicular)
        // must sweep one way only — the connect turnarounds keep the sweep, not scramble it.
        let solids: Vec<&ToolPath> = layers[1]
            .paths
            .iter()
            .filter(|p| p.kind == PathKind::Solid && !p.closed)
            .collect();
        assert!(!solids.is_empty(), "expected solid fill");
        let angle = 135.0_f64.to_radians(); // layer 1 fills at 135°
        let proj = |pt: Point| -pt.x_mm() * angle.sin() + pt.y_mm() * angle.cos();
        let mut total_levels = 0usize;
        for run in &solids {
            // Distinct scanline levels in traversal order along the run.
            let mut levels: Vec<f64> = Vec::new();
            for &pt in &run.points {
                let v = proj(pt);
                if levels.last().map_or(true, |&last| (v - last).abs() > 0.05) {
                    levels.push(v);
                }
            }
            total_levels += levels.len();
            let inc = levels.windows(2).filter(|w| w[1] > w[0]).count();
            let monotone = levels.len() < 2 || inc == levels.len() - 1 || inc == 0;
            assert!(monotone, "a solid run must sweep monotonically; {inc}/{} increasing", levels.len() - 1);
        }
        assert!(total_levels > 10, "expected many scanlines across the solid, got {total_levels}");
    }

    #[test]
    fn ironing_runs_last_over_top() {
        let m = Mesh::cube(10.0);
        let mut s = Settings { skirt_loops: 0, ..Settings::default() };
        s.ironing = true;
        let layers = generate(&m, &s);
        let top = layers.last().unwrap();
        let n_iron = count(top, PathKind::Ironing);
        assert!(n_iron > 10, "top layer should be ironed, got {n_iron} paths");
        // Ironing strictly after everything else.
        let first_iron = top.paths.iter().position(|p| p.kind == PathKind::Ironing).unwrap();
        assert!(
            top.paths[first_iron..].iter().all(|p| p.kind == PathKind::Ironing),
            "ironing must come last"
        );
        // And nowhere below the top surface of a cube.
        assert_eq!(count(&layers[10], PathKind::Ironing), 0);
    }

    #[test]
    fn spiral_vase_is_single_wall_no_infill() {
        let m = Mesh::cube(20.0);
        let mut s = Settings { skirt_loops: 0, ..Settings::default() };
        s.spiral_vase = true;
        s.bottom_layers = 3;
        let layers = generate(&m, &s);
        let mid = &layers[50];
        let printable: Vec<&ToolPath> = mid.paths.iter().filter(|p| p.points.len() >= 2).collect();
        assert_eq!(printable.len(), 1, "vase layer = exactly one path");
        assert_eq!(printable[0].kind, PathKind::ExternalPerimeter);
        assert!(printable[0].closed);
        // Bottom shell still solid (the bed face prints as bottom skin).
        assert!(count(&layers[0], PathKind::BottomSkin) > 0, "vase keeps a solid bottom");
    }

    #[test]
    fn gyroid_infill_generates() {
        let m = Mesh::cube(20.0);
        let mut s = Settings { skirt_loops: 0, ..Settings::default() };
        s.sparse_pattern = InfillPattern::Gyroid;
        let layers = generate(&m, &s);
        let mid = &layers[50];
        assert!(count(mid, PathKind::Infill) > 0, "gyroid should produce infill paths");
        // Gyroid pieces are polylines (many points), not 2-point straight lines.
        let max_pts = mid
            .paths
            .iter()
            .filter(|p| p.kind == PathKind::Infill)
            .map(|p| p.points.len())
            .max()
            .unwrap();
        assert!(max_pts > 4, "gyroid paths should be curved polylines, got {max_pts} points max");
    }

    #[test]
    fn seam_nearest_starts_at_rear() {
        let m = Mesh::cube(20.0);
        let s = Settings { seam_mode: SeamMode::Nearest, ..Settings::default() };
        let layers = generate(&m, &s);
        let ext = layers[10]
            .paths
            .iter()
            .find(|p| p.kind == PathKind::ExternalPerimeter)
            .unwrap();
        let max_y = ext.points.iter().map(|p| p.y).max().unwrap();
        assert_eq!(ext.points[0].y, max_y, "seam should start at the rear-most vertex");
    }

    #[test]
    fn parts_group_by_tool_and_serpentine() {
        // Two disjoint cubes on different tools: every layer prints all of one
        // tool before the other (exactly one change per layer), and each layer
        // leads with the tool already in hand from the layer before.
        let mut ta = Vec::new();
        push_box(&mut ta, [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
        let a = mesh::Mesh::from_triangle_soup(&ta);
        let mut tb = Vec::new();
        push_box(&mut tb, [30.0, 0.0, 0.0], [40.0, 10.0, 10.0]);
        let b = mesh::Mesh::from_triangle_soup(&tb);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        let plans = generate_parts(&[(&a, 0), (&b, 1)], &s);
        assert!(!plans.is_empty());
        for plan in &plans {
            let tools: Vec<u32> = plan.paths.iter().map(|p| p.tool).collect();
            let changes = tools.windows(2).filter(|w| w[0] != w[1]).count();
            assert_eq!(changes, 1, "layer {}: tools not grouped: {tools:?}", plan.index);
        }
        for w in plans.windows(2) {
            let last = w[0].paths.last().unwrap().tool;
            let first = w[1].paths.first().unwrap().tool;
            assert_eq!(last, first, "layer {} does not lead with the tool in hand", w[1].index);
        }
    }

    #[test]
    fn stacked_part_is_supported_by_the_one_below() {
        // Cube B rests directly on cube A (different tools). B's base is
        // physically supported — no support columns, no bridges, no overhang
        // walls — yet the interface still gets B's own bottom skin (full
        // shells between filaments, no show-through).
        let mut ta = Vec::new();
        push_box(&mut ta, [0.0, 0.0, 0.0], [20.0, 20.0, 10.0]);
        let a = mesh::Mesh::from_triangle_soup(&ta);
        let mut tb = Vec::new();
        push_box(&mut tb, [5.0, 5.0, 10.0], [15.0, 15.0, 20.0]);
        let b = mesh::Mesh::from_triangle_soup(&tb);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        s.layer_height_mm = 0.2;
        s.first_layer_height_mm = 0.2;
        s.support_mode = SupportMode::Grid;
        let plans = generate_parts(&[(&a, 0), (&b, 1)], &s);
        assert_eq!(plans.len(), 100, "shared grid spans the union z-range");
        let supports: usize = plans.iter().map(|l| count(l, PathKind::Support)).sum();
        assert_eq!(supports, 0, "B rests on A — nothing to support");
        // The interface layer: B's first layer, printed on A's top.
        let interface = &plans[50];
        assert_eq!(count(interface, PathKind::Bridge), 0, "supported base must not bridge");
        assert_eq!(count(interface, PathKind::OverhangWall), 0, "supported walls aren't overhangs");
        let skins: Vec<&ToolPath> =
            interface.paths.iter().filter(|p| p.kind == PathKind::BottomSkin).collect();
        assert!(!skins.is_empty(), "part interface must be skinned");
        assert!(skins.iter().all(|p| p.tool == 1), "the interface skin belongs to the upper part");
    }

    #[test]
    fn overlapping_parts_trim_to_the_earlier_owner() {
        // Cubes overlapping 10 mm in x: the earlier part owns the overlap, the
        // later part retreats to the boundary — nothing prints twice.
        let mut ta = Vec::new();
        push_box(&mut ta, [0.0, 0.0, 0.0], [20.0, 20.0, 6.0]);
        let a = mesh::Mesh::from_triangle_soup(&ta);
        let mut tb = Vec::new();
        push_box(&mut tb, [10.0, 0.0, 0.0], [30.0, 20.0, 6.0]);
        let b = mesh::Mesh::from_triangle_soup(&tb);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        s.auto_center_on_bed = false;
        let plans = generate_parts(&[(&a, 0), (&b, 1)], &s);
        let mid = &plans[plans.len() / 2];
        for p in &mid.paths {
            if p.tool == 1 {
                for pt in &p.points {
                    assert!(
                        pt.x_mm() > 19.5,
                        "tool-1 path crosses into the owned overlap: x={:.2}",
                        pt.x_mm()
                    );
                }
            }
        }
        let t0_max = mid
            .paths
            .iter()
            .filter(|p| p.tool == 0)
            .flat_map(|p| &p.points)
            .map(|pt| pt.x_mm())
            .fold(f64::MIN, f64::max);
        assert!(t0_max > 19.0, "tool-0 keeps its full footprint (max x {t0_max:.2})");
    }

    #[test]
    fn blend_schedule_spreads_without_adjacent_repeats() {
        // 4:1:3 (the classic knit-duck blend): exact ratio, and because no
        // color owns more than half the 8-layer cycle, NO two adjacent layers
        // (the tiled seam included) share a color — the point of the change.
        let s = blend_schedule(&[(0, 4.0), (1, 1.0), (4, 3.0)], 80);
        assert_eq!(s.iter().filter(|&&t| t == 0).count(), 40);
        assert_eq!(s.iter().filter(|&&t| t == 1).count(), 10);
        assert_eq!(s.iter().filter(|&&t| t == 4).count(), 30);
        for w in s.windows(2) {
            assert_ne!(w[0], w[1], "no adjacent repeats: {s:?}");
        }
        // 50/50 strictly alternates.
        let ab = blend_schedule(&[(1, 1.0), (0, 1.0)], 10);
        for w in ab.windows(2) {
            assert_ne!(w[0], w[1], "50/50 must alternate: {ab:?}");
        }
        // Duplicate tool entries fold together: (2+2) tool 0, 3 tool 1 → 4:3.
        let dup = blend_schedule(&[(0, 2.0), (1, 3.0), (0, 2.0)], 70);
        assert_eq!(dup.iter().filter(|&&t| t == 0).count(), 40);
        assert_eq!(dup.iter().filter(|&&t| t == 1).count(), 30);
        // A heavily-leaning blend can't avoid all repeats, but keeps the ratio.
        let lean = blend_schedule(&[(0, 9.0), (1, 1.0)], 30);
        assert_eq!(lean.iter().filter(|&&t| t == 0).count(), 27);
        assert_eq!(lean.iter().filter(|&&t| t == 1).count(), 3);
        // Degenerate blends: one entry = that tool; empty = tool 0.
        assert!(blend_schedule(&[(3, 2.0)], 5).iter().all(|&t| t == 3));
        assert!(blend_schedule(&[], 5).iter().all(|&t| t == 0));
        // Deterministic.
        assert_eq!(s, blend_schedule(&[(0, 4.0), (1, 1.0), (4, 3.0)], 80));
    }

    #[test]
    fn swap_puts_the_dominant_tool_on_top() {
        // 90:10 — wherever the printed top lands in the cycle, after the swap
        // it prints the dominant tool, the ratio is untouched, and at most
        // two layers moved.
        let raw = blend_schedule(&[(0, 9.0), (1, 1.0)], 30);
        for top in 5..30 {
            let mut s = raw.clone();
            let mut printed = vec![false; 30];
            printed[..=top].fill(true);
            swap_dominant_to_top(&mut s, &printed, &[]);
            assert_eq!(s[top], 0, "top layer must print the dominant tool");
            assert_eq!(
                s.iter().filter(|&&t| t == 1).count(),
                raw.iter().filter(|&&t| t == 1).count(),
                "swap must not change the ratio"
            );
            let moved = s.iter().zip(&raw).filter(|(a, b)| a != b).count();
            assert!(moved <= 2, "swap perturbs at most two layers, got {moved}");
        }
        // No schedule-resolvable dominant: 50/50 stays byte-identical — even
        // over an odd length, where the counts differ by one from parity alone.
        let ab = blend_schedule(&[(0, 1.0), (1, 1.0)], 11);
        let mut s = ab.clone();
        swap_dominant_to_top(&mut s, &vec![true; 11], &[]);
        assert_eq!(s, ab);
        // A phantom top (slice contour, no paths) is not the face: the swap
        // targets the topmost PRINTED layer and leaves the phantom alone.
        let raw = blend_schedule(&[(0, 3.0), (1, 1.0)], 8); // [0,0,1,0,0,0,1,0]
        let mut s = raw.clone();
        let mut printed = vec![true; 8];
        printed[7] = false;
        swap_dominant_to_top(&mut s, &printed, &[]);
        assert_eq!(s[6], 0, "the real cap must print the dominant tool");
        assert_eq!(s[7], raw[7], "the phantom layer is not the swap's business");
        // A shorter member's own top is protected from becoming the donor.
        let mut s = vec![0, 0, 0, 1];
        swap_dominant_to_top(&mut s, &vec![true; 4], &[2]);
        assert_eq!(s, vec![0, 1, 0, 0], "donor must skip the protected top at 2");
        // No printed dominant layer below the top: nothing to trade.
        let mut s = vec![0, 0, 0, 1];
        swap_dominant_to_top(&mut s, &[false, false, false, true], &[]);
        assert_eq!(s, vec![0, 0, 0, 1]);
    }

    #[test]
    fn schedule_dominant_needs_a_real_margin() {
        // Dominance is read off the schedule: aggregated per tool (duplicate
        // weight entries collapse), and only granted with a ≥2-layer margin —
        // ratios the schedule can't resolve are ties.
        assert_eq!(schedule_dominant(&blend_schedule(&[(0, 3.0), (1, 1.0)], 40)), Some(0));
        assert_eq!(schedule_dominant(&blend_schedule(&[(0, 1.0), (1, 1.0)], 10)), None);
        assert_eq!(schedule_dominant(&blend_schedule(&[(0, 1.0), (1, 1.0)], 11)), None);
        // Tool 0 owns 12/21 layers here even though tool 1 holds the single
        // largest weight entry — the schedule is the truth.
        assert_eq!(
            schedule_dominant(&blend_schedule(&[(0, 2.0), (1, 3.0), (0, 2.0)], 21)),
            Some(0)
        );
        assert_eq!(schedule_dominant(&[0; 5]), None);
    }

    #[test]
    fn painted_blend_dithers_layers_and_aligns_across_parts() {
        // Two cubes sharing a 50/50 blend of tools 0 and 1: each layer is one
        // tool for BOTH parts (aligned bands — no toolchange between them),
        // consecutive layers alternate, and the geometry is untouched (same
        // paths as the same cubes painted with a single tool).
        let mut ta = Vec::new();
        push_box(&mut ta, [0.0, 0.0, 0.0], [10.0, 10.0, 4.0]);
        let a = mesh::Mesh::from_triangle_soup(&ta);
        let mut tb = Vec::new();
        push_box(&mut tb, [30.0, 0.0, 0.0], [40.0, 10.0, 4.0]);
        let b = mesh::Mesh::from_triangle_soup(&tb);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        let blend = PartPaint::Blend(vec![(0, 1.0), (1, 1.0)]);
        let plans =
            generate_painted(&[(&a, blend.clone()), (&b, blend.clone())], &s);
        let mut per_layer_tool = Vec::new();
        for plan in &plans {
            let tools: Vec<u32> = plan.paths.iter().map(|p| p.tool).collect();
            assert!(!tools.is_empty());
            assert!(tools.windows(2).all(|w| w[0] == w[1]), "layer {} mixes tools", plan.index);
            per_layer_tool.push(tools[0]);
        }
        for w in per_layer_tool.windows(2) {
            assert_ne!(w[0], w[1], "a 50/50 blend alternates every layer");
        }
        let uniform = generate_parts(&[(&a, 0), (&b, 0)], &s);
        assert_eq!(plans.len(), uniform.len());
        for (bl, un) in plans.iter().zip(&uniform) {
            assert_eq!(bl.paths.len(), un.paths.len(), "blend must not change geometry");
        }
    }

    #[test]
    fn painted_blend_caps_with_the_dominant_tool() {
        // A skewed blend must print its dominant tool on the top face — the
        // one surface with no layer averaging — while a shorter part sharing
        // the recipe stays in lockstep. Heights are chosen so the raw cycle
        // lands the MINORITY on the tall part's top layer (asserted below),
        // so this genuinely exercises the swap.
        let mut ta = Vec::new();
        push_box(&mut ta, [0.0, 0.0, 0.0], [10.0, 10.0, 2.2]);
        let a = mesh::Mesh::from_triangle_soup(&ta);
        let mut tb = Vec::new();
        push_box(&mut tb, [30.0, 0.0, 0.0], [40.0, 10.0, 1.0]);
        let b = mesh::Mesh::from_triangle_soup(&tb);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        let weights = vec![(0u32, 3.0f64), (1u32, 1.0f64)];
        let blend = PartPaint::Blend(weights.clone());
        let plans = generate_painted(&[(&a, blend.clone()), (&b, blend)], &s);
        let top = plans.iter().rposition(|p| !p.paths.is_empty()).unwrap();
        let raw = blend_schedule(&weights, plans.len());
        for plan in &plans {
            let tools: Vec<u32> = plan.paths.iter().map(|p| p.tool).collect();
            assert!(tools.windows(2).all(|w| w[0] == w[1]), "layer {} mixes tools", plan.index);
        }
        assert_eq!(plans[top].paths[0].tool, 0, "the top face must print the dominant tool");
        let minority =
            plans.iter().filter(|p| p.paths.first().map(|q| q.tool) == Some(1)).count();
        assert_eq!(
            minority,
            raw[..=top].iter().filter(|&&t| t == 1).count(),
            "the swap must relocate the minority layer, not drop it"
        );
    }

    #[test]
    fn exposed_shelf_prints_the_dominant_tool() {
        // A stepped part: 20×20 base with an 8×8 tower — the shelf around the
        // tower is a visible top face on a mid-height layer. Heights put the
        // shelf layer on a MINORITY band of the 3:1 cycle (asserted below):
        // the face override must recolor the shelf's paths to the dominant
        // tool while buried layers keep the schedule — the wall dither is
        // what mixes the color, and it must stay exact.
        let mut t = Vec::new();
        push_box(&mut t, [0.0, 0.0, 0.0], [20.0, 20.0, 2.0]);
        push_box(&mut t, [6.0, 6.0, 2.0], [14.0, 14.0, 4.0]);
        let m = mesh::Mesh::from_triangle_soup(&t);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        s.auto_center_on_bed = false;
        let weights = vec![(0u32, 3.0f64), (1u32, 1.0f64)];
        let plans = generate_painted(&[(&m, PartPaint::Blend(weights.clone()))], &s);
        let raw = blend_schedule(&weights, plans.len());
        // The 2.0 mm base tops out on layer 9 — a minority band of the 3:1
        // cycle [0,1,0,0] — so the shelf face genuinely needs the swap.
        let shelf = 9;
        assert_eq!(raw[shelf], 1, "test setup: the shelf layer must be a minority band");
        let on_shelf = |p: &ToolPath| {
            p.points.iter().any(|pt| {
                pt.x_mm() < 5.5 || pt.x_mm() > 14.5 || pt.y_mm() < 5.5 || pt.y_mm() > 14.5
            })
        };
        let mut face = 0;
        for p in &plans[shelf].paths {
            if on_shelf(p) {
                face += 1;
                assert_eq!(p.tool, 0, "shelf face paths must print the dominant tool");
            }
        }
        assert!(face > 0, "the shelf layer must have face paths");
        // Fully-buried paths on the shelf layer (the sparse infill under the
        // tower) keep the minority band — only paths touching the face move.
        let buried: Vec<&ToolPath> = plans[shelf]
            .paths
            .iter()
            .filter(|p| {
                p.points.iter().all(|pt| {
                    pt.x_mm() > 6.05 && pt.x_mm() < 13.95 && pt.y_mm() > 6.05 && pt.y_mm() < 13.95
                })
            })
            .collect();
        assert!(!buried.is_empty(), "the shelf layer must have buried paths under the tower");
        assert!(buried.iter().all(|p| p.tool == 1), "buried paths keep the minority band");
        // A buried minority layer inside the base keeps its band untouched.
        assert_eq!(raw[5], 1);
        assert!(plans[5].paths.iter().all(|p| p.tool == 1), "buried bands keep the schedule");
        // And the cap prints the dominant tool (the swap's job).
        let top = plans.iter().rposition(|p| !p.paths.is_empty()).unwrap();
        assert!(plans[top].paths.iter().all(|p| p.tool == 0), "the cap prints the dominant");
        // Same step, but too small to be worth a dock trip: an 8×8 base with
        // a 6×6 tower leaves a 1 mm ledge — eroded away, so the gate rejects
        // it and the shelf layer keeps its minority band whole.
        let mut t = Vec::new();
        push_box(&mut t, [30.0, 0.0, 0.0], [38.0, 8.0, 2.0]);
        push_box(&mut t, [31.0, 1.0, 2.0], [37.0, 7.0, 4.0]);
        let m = mesh::Mesh::from_triangle_soup(&t);
        let plans = generate_painted(&[(&m, PartPaint::Blend(weights))], &s);
        assert!(
            plans[shelf].paths.iter().all(|p| p.tool == 1),
            "a sliver ledge must not trigger the override"
        );
    }

    #[test]
    fn floating_bottom_face_prints_the_dominant_tool() {
        // A blended slab floating in the air beside a bed-sitting body (as
        // over stripped supports): its underside is a visible face on a
        // minority band — the override must recolor the whole layer, walls
        // included. The slab's top gets the swap; the relocated minority
        // band right below it stays buried and untouched. A slab RESTING on
        // another part instead has an interface, not a face — no dock trip.
        let mut tb = Vec::new();
        push_box(&mut tb, [0.0, 0.0, 0.0], [14.0, 14.0, 1.8]);
        let body = mesh::Mesh::from_triangle_soup(&tb);
        let mut tc = Vec::new();
        push_box(&mut tc, [30.0, 0.0, 1.8], [40.0, 10.0, 2.8]);
        let slab = mesh::Mesh::from_triangle_soup(&tc);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        s.auto_center_on_bed = false;
        let weights = vec![(0u32, 3.0f64), (1u32, 1.0f64)];
        let plans = generate_painted(
            &[(&body, PartPaint::Tool(3)), (&slab, PartPaint::Blend(weights.clone()))],
            &s,
        );
        let raw = blend_schedule(&weights, plans.len());
        // The slab's base sits on layer 9 — a minority band of the 3:1 cycle
        // [0,1,0,0] — so its underside genuinely exercises the override.
        let slab_base = 9;
        assert_eq!(raw[slab_base], 1, "test setup: the slab's base must be a minority band");
        assert!(
            plans[slab_base].paths.iter().all(|p| p.tool == 0),
            "the floating bottom face prints the dominant tool"
        );
        let top = plans.iter().rposition(|p| !p.paths.is_empty()).unwrap();
        assert!(plans[top].paths.iter().all(|p| p.tool == 0), "the slab top gets the swap");
        assert!(
            plans[top - 1].paths.iter().all(|p| p.tool == 1),
            "the relocated minority band below stays buried"
        );
        // Resting flush on the body instead: the underside is an interface,
        // not a face — the minority band survives untouched.
        let mut tr = Vec::new();
        push_box(&mut tr, [2.0, 2.0, 1.8], [12.0, 12.0, 2.8]);
        let resting = mesh::Mesh::from_triangle_soup(&tr);
        let plans = generate_painted(
            &[(&body, PartPaint::Tool(3)), (&resting, PartPaint::Blend(weights))],
            &s,
        );
        assert!(
            plans[slab_base].paths.iter().all(|p| p.tool == 1),
            "an interface onto another part must not trigger the override"
        );
    }
}
