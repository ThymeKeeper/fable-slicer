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

use crate::{slice_mesh, Layer, SliceParams};

/// What a toolpath represents — drives speed, ordering, and rendering.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PathKind {
    /// Priming loops around the first layer.
    Skirt,
    ExternalPerimeter,
    Perimeter,
    /// Dense (100%) top/bottom shell fill.
    Solid,
    /// Sparse interior fill.
    Infill,
    /// Removable support structure under overhangs.
    Support,
    /// Self-supporting arc fill over a flat overhang (arc-overhang technique).
    Bridge,
}

/// A single continuous extrusion path.
#[derive(Clone, Debug)]
pub struct ToolPath {
    pub kind: PathKind,
    /// Closed loops (walls) extrude back to the start; open paths (infill) stop.
    pub closed: bool,
    pub width_mm: f64,
    pub points: Vec<Point>,
    /// Z shift added to the layer Z for this path (brick layering staggers odd
    /// perimeters by half a layer). 0 for everything else.
    pub z_offset_mm: f64,
    /// Extrusion-flow multiplier for this path (brick layering bumps the lifted
    /// perimeters to fill the diagonal gaps between staggered beads). 1.0 = normal.
    pub flow: f64,
}

impl ToolPath {
    fn new(kind: PathKind, closed: bool, width_mm: f64, points: Vec<Point>) -> Self {
        Self { kind, closed, width_mm, points, z_offset_mm: 0.0, flow: 1.0 }
    }
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
    /// Speed multiplier (≤1) applied to this layer for min-layer-time cooling.
    pub speed_scale: f64,
}

/// Slice and plan a whole model into per-layer toolpaths, centered on the bed.
pub fn generate(mesh: &Mesh, settings: &Settings) -> Vec<LayerPlan> {
    let mut layers = slice_mesh(
        mesh,
        SliceParams {
            layer_height_mm: settings.layer_height_mm,
            first_layer_height_mm: settings.first_layer_height_mm,
        },
    );
    // Contour-resolution cleanup: drop sub-resolution mesh-facet noise so walls
    // aren't over-dense (cleaner preview, faster planning, smaller g-code).
    if settings.max_resolution_mm > 0.0 {
        for layer in &mut layers {
            layer.polygons = simplify(&layer.polygons, settings.max_resolution_mm);
        }
    }
    let lw = settings.line_width_mm;
    let n = layers.len();

    // Pass 1: walls + the infill region (inside the innermost wall) per layer.
    let mut walls_per_layer: Vec<Vec<ToolPath>> = Vec::with_capacity(n);
    let mut inner_per_layer: Vec<Polygons> = Vec::with_capacity(n);
    for layer in &layers {
        let mut walls = Vec::new();
        for w in 0..settings.wall_count {
            let inset = -lw * (0.5 + w as f64);
            let kind = if w == 0 {
                PathKind::ExternalPerimeter
            } else {
                PathKind::Perimeter
            };
            // Brick layering: lift odd-indexed perimeters by half a layer (outer
            // wall = index 0 stays put), so adjacent rings interlock like masonry;
            // a flow bump fills the diagonal gaps between the staggered beads. Skip
            // the first and last layers (base transition + top clamp).
            let brick = settings.brick_layers && w % 2 == 1 && layer.index > 0 && layer.index + 1 < n;
            let (z_offset_mm, flow) = if brick {
                (0.5 * settings.layer_height_mm, settings.brick_flow)
            } else {
                (0.0, 1.0)
            };
            for c in offset(&layer.polygons, inset).contours {
                if c.points.len() >= 3 {
                    let points = place_seam(c.points, settings.seam_mode, layer.index);
                    walls.push(ToolPath { kind, closed: true, width_mm: lw, points, z_offset_mm, flow });
                }
            }
        }
        // Inset to the infill region, then morphologically "open" it (erode then
        // dilate by half a line width) to drop slivers narrower than a line —
        // those only produce tiny, useless dabs of infill.
        let inset = offset(&layer.polygons, -lw * settings.wall_count as f64);
        let opened = offset(&offset(&inset, -lw * 0.5), lw * 0.5);
        inner_per_layer.push(opened);
        walls_per_layer.push(walls);
    }

    // Pass 2: assemble layers, splitting infill into solid shells + sparse core.
    let mut plans = Vec::with_capacity(n);
    for i in 0..n {
        let mut paths = std::mem::take(&mut walls_per_layer[i]);
        let inner = &inner_per_layer[i];

        if !inner.is_empty() {
            // Exposed to air above/below unless covered by the whole shell range.
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
            // Arc-overhang mode: the flat unsupported part of this layer's interior
            // is filled with self-supporting arcs instead of normal fill.
            let arc_region = if settings.support_mode == SupportMode::Arc && i > 0 {
                let allowance =
                    settings.layer_height_mm * settings.support_overhang_angle_deg.to_radians().tan();
                let supported_below = offset(&layers[i - 1].polygons, allowance);
                let oh = difference(&layers[i].polygons, &supported_below);
                let oh = offset(&offset(&oh, -lw), lw); // open: drop slivers
                intersection(&oh, inner)
            } else {
                Polygons::new()
            };

            let solid_all = union(&solid_top, &solid_bottom);
            let solid = difference(&solid_all, &arc_region);
            let sparse = difference(&difference(inner, &solid_all), &arc_region);

            // Alternate fill direction per layer for cross-hatching.
            let angle = if i % 2 == 0 { 45.0 } else { 135.0 };

            if !arc_region.is_empty() {
                let allowance =
                    settings.layer_height_mm * settings.support_overhang_angle_deg.to_radians().tan();
                let supported_below = offset(&layers[i - 1].polygons, allowance);
                // Decide per disjoint island: a short two-sided gap bridges with
                // straight lines even if a wider gap on the same layer needs arcs;
                // everything else (wide bridge, cantilever) is arc-filled.
                for island in islands(&arc_region) {
                    let segs = try_bridge(&island, &supported_below, lw, settings.max_bridge_span_mm)
                        .unwrap_or_else(|| {
                            crate::arc::arc_fill(&island, &supported_below, lw, settings.max_arc_radius_mm, settings.arc_seam_overlap_mm)
                        });
                    for seg in segs {
                        if seg.len() >= 2 {
                            paths.push(ToolPath::new(PathKind::Bridge, false, lw, seg));
                        }
                    }
                }
            }

            if !solid.is_empty() {
                // A perimeter loop following the solid region's boundary (so where
                // it runs alongside the shell it becomes a clean concentric bead),
                // then straight-fill only the interior left inside that loop. Thin
                // solid bands are consumed entirely by the loop — no lone strands.
                let solid_loop = offset(&solid, -lw * 0.5);
                for c in solid_loop.contours {
                    if c.points.len() >= 3 {
                        let points = place_seam(c.points, settings.seam_mode, i);
                        paths.push(ToolPath::new(PathKind::Solid, true, lw, points));
                    }
                }
                let solid_core = offset(&solid, -lw);
                if !solid_core.is_empty() {
                    fill_region(&solid_core, settings.solid_pattern, lw, angle, lw, PathKind::Solid, settings.seam_mode, i, &mut paths);
                }
            }
            if settings.infill_density > 0.0 && !sparse.is_empty() {
                let spacing = lw / settings.infill_density;
                fill_region(&sparse, settings.sparse_pattern, spacing, angle, lw, PathKind::Infill, settings.seam_mode, i, &mut paths);
            }
        }

        plans.push(LayerPlan {
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
        });
    }

    // Brim: loops extending outward from the first-layer outline, touching the
    // part for bed adhesion.
    if settings.brim_loops > 0 {
        if let (Some(first), Some(plan0)) = (layers.first(), plans.first_mut()) {
            let brim = brim_paths(&first.polygons, settings);
            plan0.paths.splice(0..0, brim);
        }
    }

    // Skirt: priming loops around the first layer, printed before anything else.
    if settings.skirt_loops > 0 {
        if let (Some(first), Some(plan0)) = (layers.first(), plans.first_mut()) {
            let skirt = skirt_paths(&first.polygons, settings);
            plan0.paths.splice(0..0, skirt);
        }
    }

    add_supports(&mut plans, &layers, settings);
    order_layers(&mut plans);
    center_on_bed(&mut plans, mesh, settings);
    crate::emit::plan_travels(&mut plans, settings);
    crate::emit::apply_min_layer_time(&mut plans, settings);
    plans
}

/// Generate removable grid support under overhangs. For each layer, the overhang
/// is the region not over the layer below within a printable cantilever; this is
/// projected downward and the support area (minus the part + clearance) is filled
/// with sparse lines as `PathKind::Support`.
fn add_supports(plans: &mut [LayerPlan], layers: &[Layer], settings: &Settings) {
    // Arc mode fills overhangs on-layer (in pass 2); only Grid adds structure below.
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
    let mut overhang: Vec<Polygons> = vec![Polygons::new(); n];
    for i in 1..n {
        let supported = offset(&layers[i - 1].polygons, allowance);
        let oh = difference(&layers[i].polygons, &supported);
        overhang[i] = offset(&offset(&oh, -lw), lw); // morphological open
    }

    // Project downward: support at layer i holds overhangs accumulated from above,
    // minus the part (+clearance). Where the part is, the column rests and stops.
    // A z-gap of `gap` empty layers under each overhang aids removal, and the top
    // `iface` support layers are printed solid for a smoother overhang underside.
    let spacing = lw / settings.support_density.clamp(0.02, 1.0);
    let gap = settings.support_z_gap_layers;
    let iface = settings.support_interface_layers;
    let mut accum = Polygons::new();
    for i in (0..n).rev() {
        let blocked = offset(&layers[i].polygons, clearance);
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
                fill_region(&body_here, InfillPattern::Lines, spacing, angle, lw,
                    PathKind::Support, settings.seam_mode, i, &mut plans[i].paths);
            }
            if !iface_here.is_empty() {
                fill_region(&iface_here, InfillPattern::Lines, lw, angle, lw,
                    PathKind::Support, settings.seam_mode, i, &mut plans[i].paths);
            }
        }
        accum = difference(&accum, &layers[i].polygons);
        // Defer adding this layer's overhang by `gap` layers so the support tops
        // out `gap` layers below it (leaving the removal gap).
        if i + gap < n {
            accum = union(&accum, &overhang[i + gap]);
        }
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

/// If `region` is a true bridge — supported on ≥2 sides and narrow enough to span
/// with straight lines — return those lines (oriented across the shortest gap,
/// solid spacing). Returns None for cantilevers or spans wider than `max_span`,
/// which the caller arc-fills instead.
fn try_bridge(region: &Polygons, supported: &Polygons, lw: f64, max_span: f64) -> Option<Vec<Vec<Point>>> {
    if max_span <= 0.0 {
        return None;
    }
    // Try a range of line directions; the bridge runs across the shortest spans.
    let mut best: Option<(f64, f64)> = None; // (max line length, angle)
    for k in 0..12 {
        let angle = k as f64 * 15.0;
        let segs = infill_lines(region, angle, lw);
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
    let (_, angle) = best?;
    Some(infill_lines(region, angle, lw))
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

/// Greedily order each layer's paths (nearest-neighbour) to cut travel, keeping
/// skirt/brim first. Open paths may be reversed to start at the nearer end.
fn order_layers(plans: &mut [LayerPlan]) {
    let mut cur = Point::new(0, 0);
    for plan in plans.iter_mut() {
        let all = std::mem::take(&mut plan.paths);
        let (prime, rest): (Vec<_>, Vec<_>) =
            all.into_iter().partition(|p| p.kind == PathKind::Skirt);
        if let Some(last) = prime.last() {
            cur = path_end(last);
        }
        // Brick layering prints the on-plane (low) phase fully before the lifted
        // (high) phase, so a travel never crosses a bead at a different Z. Without
        // brick, `high` is empty and this is the usual single-pass ordering.
        let (low, high): (Vec<_>, Vec<_>) = rest.into_iter().partition(|p| p.z_offset_mm == 0.0);
        let mut paths = prime;
        for group in [low, high] {
            if group.is_empty() {
                continue;
            }
            let ordered = order_paths(group, cur);
            if let Some(last) = ordered.last() {
                cur = path_end(last);
            }
            paths.extend(ordered);
        }
        plan.paths = paths;
    }
}

fn order_paths(mut remaining: Vec<ToolPath>, start: Point) -> Vec<ToolPath> {
    let mut out = Vec::with_capacity(remaining.len());
    let mut cur = start;
    while !remaining.is_empty() {
        let mut best = 0usize;
        let mut best_d = i128::MAX;
        let mut best_rev = false;
        for (i, p) in remaining.iter().enumerate() {
            let ds = dist2(cur, p.points[0]);
            if ds < best_d {
                best_d = ds;
                best = i;
                best_rev = false;
            }
            if !p.closed {
                let de = dist2(cur, p.points[p.points.len() - 1]);
                if de < best_d {
                    best_d = de;
                    best = i;
                    best_rev = true;
                }
            }
        }
        let mut p = remaining.swap_remove(best);
        if best_rev {
            p.points.reverse();
        }
        cur = path_end(&p);
        out.push(p);
    }
    out
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
    let mut paths = Vec::new();
    for k in 0..settings.brim_loops {
        let delta = lw * (0.5 + k as f64);
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
fn center_on_bed(plans: &mut [LayerPlan], mesh: &Mesh, settings: &Settings) {
    if !settings.auto_center_on_bed {
        return; // caller positioned the geometry already (e.g. GUI multi-object layout)
    }
    let Some((min_x, min_y, max_x, max_y)) = mesh.xy_bounds() else {
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

/// Generate straight infill lines at `angle_deg`, spaced `spacing_mm` apart,
/// clipped to `region` via an even-odd scanline. The region is rotated so infill
/// lines become horizontal scanlines, then results are rotated back.
fn infill_lines(region: &Polygons, angle_deg: f64, spacing_mm: f64) -> Vec<Vec<Point>> {
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
                out.push(vec![Point::from_mm(px0, py0), Point::from_mm(px1, py1)]);
            }
            k += 2;
        }
        y += spacing_mm;
    }
    out
}

/// Fill a region with the chosen pattern, pushing toolpaths into `out`.
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
    out: &mut Vec<ToolPath>,
) {
    match pattern {
        InfillPattern::Lines => {
            for seg in infill_lines(region, angle, spacing) {
                out.push(ToolPath::new(kind, false, lw, seg));
            }
        }
        InfillPattern::Grid => {
            for a in [angle, angle + 90.0] {
                for seg in infill_lines(region, a, spacing) {
                    out.push(ToolPath::new(kind, false, lw, seg));
                }
            }
        }
        InfillPattern::Concentric => {
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
    }
}

/// Rotate a closed wall loop so the seam (start/end) lands at the chosen vertex.
fn place_seam(mut points: Vec<Point>, mode: SeamMode, layer_index: usize) -> Vec<Point> {
    let n = points.len();
    if n < 3 {
        return points;
    }
    let start = match mode {
        // Rear-most vertex (max Y, tie-break max X) — seams align into a column.
        SeamMode::Nearest => (0..n)
            .max_by_key(|&i| (points[i].y, points[i].x))
            .unwrap(),
        // Sharpest corner — tucks the seam where it's least visible.
        SeamMode::Sharpest => (0..n)
            .max_by(|&a, &b| sharpness(&points, a).total_cmp(&sharpness(&points, b)))
            .unwrap(),
        // Deterministic per-layer scatter.
        SeamMode::Random => layer_index.wrapping_mul(2_654_435_761).wrapping_add(40_503) % n,
    };
    points.rotate_left(start);
    points
}

/// Corner sharpness at vertex `i`: `1 - cos(turn)` (0 = straight, up to 2 = hairpin).
fn sharpness(points: &[Point], i: usize) -> f64 {
    let n = points.len();
    let prev = points[(i + n - 1) % n];
    let cur = points[i];
    let next = points[(i + 1) % n];
    let a = unit(cur.x_mm() - prev.x_mm(), cur.y_mm() - prev.y_mm());
    let b = unit(next.x_mm() - cur.x_mm(), next.y_mm() - cur.y_mm());
    1.0 - (a.0 * b.0 + a.1 * b.1)
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

    fn count(layer: &LayerPlan, kind: PathKind) -> usize {
        layer.paths.iter().filter(|p| p.kind == kind).count()
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
    fn brick_layers_lift_odd_perimeters() {
        let m = Mesh::cube(20.0);
        let mut s = Settings::default();
        s.brick_layers = true;
        s.wall_count = 3;
        let layers = generate(&m, &s);
        let mid = &layers[50]; // interior layer (not first/last)
        // The external perimeter (index 0) stays on the layer plane.
        let ext = mid.paths.iter().find(|p| p.kind == PathKind::ExternalPerimeter).unwrap();
        assert_eq!(ext.z_offset_mm, 0.0);
        assert_eq!(ext.flow, 1.0);
        // An odd inner perimeter is lifted half a layer and over-extruded.
        let lifted = mid.paths.iter().any(|p| {
            p.kind == PathKind::Perimeter
                && (p.z_offset_mm - 0.5 * s.layer_height_mm).abs() < 1e-9
                && p.flow > 1.0
        });
        assert!(lifted, "an odd inner perimeter should be brick-lifted");
        // First layer is a base transition — nothing lifted.
        assert!(layers[0].paths.iter().all(|p| p.z_offset_mm == 0.0), "base layer is flat");
    }

    #[test]
    fn brick_orders_low_phase_first_and_hops() {
        let m = Mesh::cube(20.0);
        let mut s = Settings::default();
        s.brick_layers = true;
        s.wall_count = 4;
        let layers = generate(&m, &s);
        let mid = &layers[50];
        let first_high = mid.paths.iter().position(|p| p.z_offset_mm > 0.0).expect("lifted perimeters");
        // Low (on-plane) phase entirely precedes the contiguous high (lifted) phase.
        assert!(mid.paths[..first_high].iter().all(|p| p.z_offset_mm == 0.0), "low phase first");
        assert!(mid.paths[first_high..].iter().all(|p| p.z_offset_mm > 0.0), "high phase contiguous");
        // The travel reaching the first lifted perimeter hops clear of the low beads.
        assert!(mid.travels[first_high].hop, "phase-boundary travel hops");
    }

    #[test]
    fn cube_has_solid_top_bottom_sparse_middle() {
        let m = Mesh::cube(20.0);
        let s = Settings::default(); // 4 top / 4 bottom
        let layers = generate(&m, &s);

        // Bottom and top shells are solid; the middle is sparse only.
        assert!(count(&layers[0], PathKind::Solid) > 0, "bottom shell solid");
        assert!(count(&layers[99], PathKind::Solid) > 0, "top shell solid");

        let mid = &layers[50];
        assert!(count(mid, PathKind::Infill) > 0, "middle has sparse infill");
        assert_eq!(count(mid, PathKind::Solid), 0, "middle has no solid fill");
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
}
