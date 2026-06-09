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
    /// Single width-matched strokes in gaps too thin for normal fill.
    GapFill,
    /// Low-flow smoothing pass over exposed top surfaces.
    Ironing,
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
    /// perimeters to fill the diagonal gaps between staggered beads; ironing
    /// trickles). 1.0 = normal.
    pub flow: f64,
    /// Monotonic-block id: consecutive paths sharing a `Some` group move as one
    /// indivisible block in travel ordering (a strict sweep over one surface).
    /// Distinct groups (e.g. separate islands) are ordered independently.
    pub group: Option<u32>,
}

impl ToolPath {
    fn new(kind: PathKind, closed: bool, width_mm: f64, points: Vec<Point>) -> Self {
        Self { kind, closed, width_mm, points, z_offset_mm: 0.0, flow: 1.0, group: None }
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
    // Spiral vase rewrites the recipe: one wall, no sparse infill, no shells
    // above the solid bottom, nothing that would interrupt the continuous loop.
    let vase_settings;
    let settings = if settings.spiral_vase {
        vase_settings = Settings {
            wall_count: 1,
            infill_density: 0.0,
            top_layers: 0,
            support_mode: SupportMode::None,
            brick_layers: false,
            ironing: false,
            gap_fill: false,
            fuzzy_skin: false,
            ..settings.clone()
        };
        &vase_settings
    } else {
        settings
    };

    let mut layers = slice_mesh(
        mesh,
        SliceParams {
            layer_height_mm: settings.layer_height_mm,
            first_layer_height_mm: settings.first_layer_height_mm,
        },
    );
    // Contour-resolution cleanup: drop sub-resolution mesh-facet noise so walls
    // aren't over-dense (cleaner preview, faster planning, smaller g-code). Then
    // dimensional compensation: XY grow/shrink on every layer, and the first
    // layer pulled in to counter squish (elephant foot).
    layers.par_iter_mut().for_each(|layer| {
        if settings.max_resolution_mm > 0.0 {
            layer.polygons = simplify(&layer.polygons, settings.max_resolution_mm);
        }
        if settings.xy_compensation_mm != 0.0 {
            layer.polygons = offset(&layer.polygons, settings.xy_compensation_mm);
        }
        if layer.index == 0 && settings.elephant_foot_mm > 0.0 {
            layer.polygons = offset(&layer.polygons, -settings.elephant_foot_mm);
        }
    });
    let lw = settings.line_width_mm;
    let n = layers.len();

    // Pass 1: walls + the infill region (inside the innermost wall) per layer,
    // plus the gap regions too thin for any wall or infill to cover.
    let per_layer: Vec<(Vec<ToolPath>, Polygons, Polygons)> = layers
        .par_iter()
        .map(|layer| {
            let mut walls = Vec::new();
            let mut gaps = Polygons::new();
            // Material remaining at depth w·lw — eroded wall by wall to find
            // where the next wall failed to fit (the gap-fill regions).
            let mut at_depth = if settings.gap_fill { layer.polygons.clone() } else { Polygons::new() };
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
                let brick =
                    settings.brick_layers && w % 2 == 1 && layer.index > 0 && layer.index + 1 < n;
                let (z_offset_mm, flow) = if brick {
                    (0.5 * settings.layer_height_mm, settings.brick_flow)
                } else {
                    (0.0, 1.0)
                };
                let centers = offset(&layer.polygons, inset);
                for c in &centers.contours {
                    if c.points.len() >= 3 {
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
                        walls.push(ToolPath { kind, closed: true, width_mm: lw, points, z_offset_mm, flow, group: None });
                    }
                }
                if settings.gap_fill {
                    // Where material remains at this wall's outer edge but the
                    // wall bead (and everything deeper) is missing, it's a gap.
                    let covered = offset(&centers, lw * 0.5);
                    if w > 0 {
                        gaps = union(&gaps, &difference(&at_depth, &covered));
                    }
                    at_depth = offset(&centers, -lw * 0.5);
                }
            }
            // Inset to the infill region, then morphologically "open" it (erode then
            // dilate by half a line width) to drop slivers narrower than a line —
            // those only produce tiny, useless dabs of infill.
            let inset = offset(&layer.polygons, -lw * settings.wall_count as f64);
            let opened = offset(&offset(&inset, -lw * 0.5), lw * 0.5);
            if settings.gap_fill {
                // Plus the slivers the morphological open dropped from the
                // infill region (thin necks between walls).
                let base = if settings.wall_count > 0 { &at_depth } else { &inset };
                gaps = union(&gaps, &difference(base, &opened));
            }
            (walls, opened, gaps)
        })
        .collect();
    let mut walls_per_layer: Vec<Vec<ToolPath>> = Vec::with_capacity(n);
    let mut inner_per_layer: Vec<Polygons> = Vec::with_capacity(n);
    let mut gaps_per_layer: Vec<Polygons> = Vec::with_capacity(n);
    for (w, inner, g) in per_layer {
        walls_per_layer.push(w);
        inner_per_layer.push(inner);
        gaps_per_layer.push(g);
    }

    // Pass 2: assemble layers, splitting infill into solid shells + sparse core.
    let mut plans: Vec<LayerPlan> = walls_per_layer
        .into_par_iter()
        .zip(gaps_per_layer.into_par_iter())
        .enumerate()
        .map(|(i, (mut paths, gaps))| {
        let inner = &inner_per_layer[i];
        let ov = lw * settings.infill_overlap.clamp(0.0, 0.5);

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
            let mut supported_below = Polygons::new();
            let arc_region = if settings.support_mode == SupportMode::Arc && i > 0 {
                let allowance =
                    settings.layer_height_mm * settings.support_overhang_angle_deg.to_radians().tan();
                supported_below = offset(&layers[i - 1].polygons, allowance);
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
                // The loop and the fill both push `ov` into their neighbor so
                // solid surfaces bond to the walls.
                let solid_loop = offset(&solid, -(lw * 0.5 - ov * 0.5));
                for c in solid_loop.contours {
                    if c.points.len() >= 3 {
                        let points = place_seam(c.points, settings.seam_mode, i);
                        paths.push(ToolPath::new(PathKind::Solid, true, lw, points));
                    }
                }
                let solid_core = offset(&solid, -(lw - ov));
                if !solid_core.is_empty() {
                    fill_region(
                        &solid_core, settings.solid_pattern, lw, angle, lw, PathKind::Solid,
                        settings.seam_mode, i, layers[i].z_mm, settings.monotonic_solid, &mut paths,
                    );
                }
            }
            if settings.infill_density > 0.0 && !sparse.is_empty() {
                let spacing = lw / settings.infill_density;
                let sparse_fill = if ov > 0.0 { offset(&sparse, ov) } else { sparse.clone() };
                fill_region(
                    &sparse_fill, settings.sparse_pattern, spacing, angle, lw, PathKind::Infill,
                    settings.seam_mode, i, layers[i].z_mm, false, &mut paths,
                );
            }
        }

        // Gap fill: single width-matched strokes along the spine of each sliver
        // too thin for walls/infill (computed in pass 1). A small morphological
        // open first: the gap regions come from comparing chained offsets, which
        // leaves hair-thin numerical ribbons along curved walls — only gaps a
        // nozzle can usefully fill (≳ a third of a line) survive.
        let gaps = if settings.gap_fill && !gaps.is_empty() {
            offset(&offset(&gaps, -lw * 0.15), lw * 0.15)
        } else {
            gaps
        };
        if settings.gap_fill && !gaps.is_empty() {
            for island in islands(&gaps) {
                let area = island.net_area_mm2();
                if area < lw * lw {
                    continue; // sub-bead-sized crumb — not worth a dab
                }
                let perimeter: f64 = island
                    .contours
                    .iter()
                    .flat_map(|c| {
                        let m = c.points.len();
                        (0..m).map(move |j| pt_dist_mm(c.points[j], c.points[(j + 1) % m]))
                    })
                    .sum();
                // Long thin strip: width ≈ 2·area/perimeter. Match the stroke to it;
                // anything thinner than a third of a line won't print usefully.
                let gw = 2.0 * area / perimeter.max(1.0e-6);
                if gw < lw * 0.3 {
                    continue;
                }
                let gw = gw.min(lw * 1.2);
                let along = crate::fill::principal_angle_deg(&island);
                for seg in crate::fill::infill_lines(&island, along, gw, false) {
                    paths.push(ToolPath::new(PathKind::GapFill, false, gw, seg));
                }
            }
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
                    for seg in crate::fill::infill_lines(&island, 45.0, spacing, true) {
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
        }
        })
        .collect();

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
    let overhang: Vec<Polygons> = (0..n)
        .into_par_iter()
        .map(|i| {
            if i == 0 {
                return Polygons::new();
            }
            let supported = offset(&layers[i - 1].polygons, allowance);
            let oh = difference(&layers[i].polygons, &supported);
            offset(&offset(&oh, -lw), lw) // morphological open
        })
        .collect();

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
                    PathKind::Support, settings.seam_mode, i, layers[i].z_mm, false, &mut plans[i].paths);
            }
            if !iface_here.is_empty() {
                fill_region(&iface_here, InfillPattern::Lines, lw, angle, lw,
                    PathKind::Support, settings.seam_mode, i, layers[i].z_mm, false, &mut plans[i].paths);
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
        let segs = infill_lines(region, angle, lw, false);
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
    Some(infill_lines(region, angle, lw, false))
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
/// skirt/brim first and ironing last (it must run over the finished surface).
/// Open paths may be reversed to start at the nearer end; runs of `no_reorder`
/// paths (monotonic fill) move as one block.
fn order_layers(plans: &mut [LayerPlan]) {
    let mut cur = Point::new(0, 0);
    for plan in plans.iter_mut() {
        let all = std::mem::take(&mut plan.paths);
        let (prime, rest): (Vec<_>, Vec<_>) =
            all.into_iter().partition(|p| p.kind == PathKind::Skirt);
        if let Some(last) = prime.last() {
            cur = path_end(last);
        }
        let (iron, rest): (Vec<_>, Vec<_>) =
            rest.into_iter().partition(|p| p.kind == PathKind::Ironing);
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
        if let Some(last) = iron.last() {
            cur = path_end(last);
        }
        paths.extend(iron); // already in boustrophedon order
        plan.paths = paths;
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
    // Scanline patterns sweep each island separately when monotonic.
    let scan = |sets: &[(f64, f64)], out: &mut Vec<ToolPath>| {
        if monotonic {
            for (gi, island) in islands(region).iter().enumerate() {
                for (si, &(a, sp)) in sets.iter().enumerate() {
                    let group = Some((gi * sets.len() + si) as u32);
                    push_lines(infill_lines(island, a, sp, true), group, out);
                }
            }
        } else {
            for &(a, sp) in sets {
                push_lines(infill_lines(region, a, sp, false), None, out);
            }
        }
    };
    match pattern {
        InfillPattern::Lines => scan(&[(angle, spacing)], out),
        InfillPattern::Grid => scan(&[(angle, spacing * 2.0), (angle + 90.0, spacing * 2.0)], out),
        InfillPattern::Triangles => scan(
            &[(angle, spacing * 3.0), (angle + 60.0, spacing * 3.0), (angle + 120.0, spacing * 3.0)],
            out,
        ),
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

    /// An axis-aligned box `sx × sy × sz` (corner at origin) for fixture meshes.
    fn box_mesh(sx: f64, sy: f64, sz: f64) -> Mesh {
        let unit = Mesh::cube(1.0);
        Mesh {
            vertices: unit.vertices.iter().map(|v| [v[0] * sx, v[1] * sy, v[2] * sz]).collect(),
            triangles: unit.triangles.clone(),
        }
    }

    #[test]
    fn gap_fill_covers_thin_fin() {
        // A 1.2mm-wide fin with lw=0.45 and 2 walls: only one wall pair fits
        // (0.9mm); the 0.3mm core must be gap-filled with a thin stroke.
        let m = box_mesh(1.2, 20.0, 5.0);
        let s = Settings { wall_count: 2, skirt_loops: 0, ..Settings::default() };
        let layers = generate(&m, &s);
        let mid = &layers[10];
        let gaps = count(mid, PathKind::GapFill);
        assert!(gaps > 0, "thin fin core should be gap-filled");
        let g = mid.paths.iter().find(|p| p.kind == PathKind::GapFill).unwrap();
        assert!(
            g.width_mm > 0.1 && g.width_mm < s.line_width_mm,
            "gap stroke width {} should be under a line width",
            g.width_mm
        );
        // No second wall fits, so the fin is outer walls + gap fill only.
        assert_eq!(count(mid, PathKind::Infill), 0, "no room for sparse infill");
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
        // Inner wall unaffected.
        let inner = mid.paths.iter().find(|p| p.kind == PathKind::Perimeter).unwrap();
        assert!(inner.points.len() < 20, "inner wall must stay smooth");
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
    fn monotonic_solid_is_ordered() {
        let m = Mesh::cube(20.0);
        let s = Settings { skirt_loops: 0, ..Settings::default() }; // monotonic_solid: true
        let layers = generate(&m, &s);
        // Bottom shell: collect open solid lines in print order; their scanline
        // positions (projection onto the perpendicular of the fill direction)
        // must sweep one way only.
        let solids: Vec<&ToolPath> = layers[1]
            .paths
            .iter()
            .filter(|p| p.kind == PathKind::Solid && !p.closed)
            .collect();
        assert!(solids.len() > 10, "expected many solid lines, got {}", solids.len());
        let angle = 135.0_f64.to_radians(); // layer 1 fills at 135°
        let proj = |p: &ToolPath| {
            let m = p.points[0];
            -m.x_mm() * angle.sin() + m.y_mm() * angle.cos()
        };
        let ps: Vec<f64> = solids.iter().map(|p| proj(p)).collect();
        let increasing = ps.windows(2).filter(|w| w[1] > w[0]).count();
        let monotone = increasing == ps.len() - 1 || increasing == 0;
        assert!(monotone, "solid lines should sweep monotonically; got {increasing}/{} increasing", ps.len() - 1);
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
        // Bottom shell still solid.
        assert!(count(&layers[0], PathKind::Solid) > 0, "vase keeps a solid bottom");
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
}
