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

use config::{InfillPattern, SeamMode, Settings, SupportMode, WallMode};
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
    /// Wall stretch hanging more than half a bead past the layer below —
    /// printed slow, with bridge-grade cooling, so it sets in place.
    OverhangWall,
    /// Dense (100%) top/bottom shell fill.
    Solid,
    /// The first solid layer over sparse infill: beads span the open cells
    /// below, so they print like (short, well-anchored) bridges.
    InternalBridge,
    /// Self-supporting concentric arc fill over a flat overhang (arc-overhang
    /// technique) — each arc cantilevers sideways off the previous ring, so
    /// it runs far slower than a straight, both-ends-anchored bridge.
    ArcOverhang,
    /// Sparse interior fill.
    Infill,
    /// Single width-matched strokes in gaps too thin for normal fill.
    GapFill,
    /// Low-flow smoothing pass over exposed top surfaces.
    Ironing,
    /// Removable support structure under overhangs.
    Support,
    /// Straight bridge lines spanning a gap, anchored on both sides.
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
    /// Bead height as a fraction of the layer height (half-height outer walls
    /// print two 0.5 passes per layer). 1.0 = the full layer.
    pub height_scale: f64,
    /// Per-vertex bead widths (mm), parallel to `points` — variable-width
    /// (arachne) walls taper along the path. `None` = constant `width_mm`.
    /// When set, `width_mm` holds the maximum (for the flow clamp).
    pub widths: Option<Vec<f64>>,
}

impl ToolPath {
    fn new(kind: PathKind, closed: bool, width_mm: f64, points: Vec<Point>) -> Self {
        Self { kind, closed, width_mm, points, z_offset_mm: 0.0, flow: 1.0, group: None, height_scale: 1.0, widths: None }
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
    let mut norm_settings = settings.clone();
    if norm_settings.spiral_vase {
        // Spiral vase rewrites the recipe: one wall, no sparse infill, no
        // shells above the solid bottom, nothing interrupting the loop.
        norm_settings.wall_count = 1;
        norm_settings.infill_density = 0.0;
        norm_settings.top_layers = 0;
        norm_settings.support_mode = SupportMode::None;
        norm_settings.brick_layers = false;
        norm_settings.half_height_outer_walls = false; // the spiral *is* the outer wall
        norm_settings.ironing = false;
        norm_settings.gap_fill = false;
        norm_settings.fuzzy_skin = false;
    }
    if norm_settings.half_height_outer_walls && norm_settings.brick_layers {
        // Mutually exclusive: their Z choreographies collide (the lower outer
        // pass would graze the previous layer's lifted brick beads).
        norm_settings.brick_layers = false;
    }
    if norm_settings.brick_layers || norm_settings.spiral_vase {
        // Brick masonry needs uniform rings; the vase loop is classic too.
        norm_settings.wall_mode = WallMode::Classic;
    }
    if norm_settings.wall_mode == WallMode::Arachne {
        // Binary model: arachne absorbs every gap into the walls (stretch /
        // absorb regimes), so gap fill is a classic-mode companion only.
        norm_settings.gap_fill = false;
    }
    let settings = &norm_settings;

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

    // Half-height outer walls: slice two extra planes per layer (the quarter
    // heights), so each half-pass follows its *own* contour — on slopes the two
    // outlines differ, which is what halves the visible staircase. Layer 0
    // stays one full-height pass (bed squish wants one fat bead).
    let outer_halves: Vec<Option<(Polygons, Polygons)>> =
        if settings.half_height_outer_walls && n > 1 {
            let mut zs = Vec::with_capacity((n - 1) * 2);
            for layer in layers.iter().skip(1) {
                zs.push(layer.z_mm - 0.25 * layer.height_mm);
                zs.push(layer.z_mm + 0.25 * layer.height_mm);
            }
            let sliced = crate::slice::slice_many(mesh, &zs);
            let processed: Vec<Polygons> = sliced
                .into_par_iter()
                .map(|(_, mut p)| {
                    if settings.max_resolution_mm > 0.0 {
                        p = simplify(&p, settings.max_resolution_mm);
                    }
                    if settings.xy_compensation_mm != 0.0 {
                        p = offset(&p, settings.xy_compensation_mm);
                    }
                    p
                })
                .collect();
            let mut halves: Vec<Option<(Polygons, Polygons)>> = vec![None];
            for (k, layer) in layers.iter().enumerate().skip(1) {
                let pick = |q: Polygons| if q.is_empty() { layer.polygons.clone() } else { q };
                halves.push(Some((
                    pick(processed[(k - 1) * 2].clone()),
                    pick(processed[(k - 1) * 2 + 1].clone()),
                )));
            }
            halves
        } else {
            vec![None; n]
        };

    // Pass 1: walls + the infill region (inside the innermost wall) per layer,
    // plus the gap regions too thin for any wall or infill to cover.
    let per_layer: Vec<(Vec<ToolPath>, Polygons, Polygons)> = layers
        .par_iter()
        .zip(outer_halves.par_iter())
        .map(|(layer, halves)| {
            // Adjacent beads are placed at the stadium spacing: rounded
            // shoulders overlap just enough to fill the cusps between beads.
            let sp = config::bead_spacing_mm(lw, layer.height_mm);
            let arachne = settings.wall_mode == WallMode::Arachne;
            // With half-height outer walls, interior geometry must stay inside
            // *both* half contours (on a shallow slope the layer-midpoint
            // outline is wider than the upper pass — inner walls based on it
            // would poke outside the outer wall). The intersection is the safe
            // core; without halves it's just the layer outline.
            let interior_owned = halves.as_ref().map(|(lo, up)| intersection(lo, up));
            let interior: &Polygons = interior_owned.as_ref().unwrap_or(&layer.polygons);
            let mut walls = Vec::new();
            let mut gaps = Polygons::new();
            // Material remaining at depth w·lw — eroded wall by wall to find
            // where the next wall failed to fit (the gap-fill regions).
            let mut at_depth = if settings.gap_fill { interior.clone() } else { Polygons::new() };
            for w in 0..settings.wall_count {
                if arachne && w > 0 {
                    break; // inner walls come from the variable-width field
                }
                let inset = -(lw * 0.5 + w as f64 * sp);
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
                // Outer wall (w == 0) hugs the true outline (or its halves);
                // everything deeper offsets from the safe interior core.
                let centers = offset(if w == 0 { &layer.polygons } else { interior }, inset);
                let emit_loops = |src: &Polygons, z_off: f64, hscale: f64, walls: &mut Vec<ToolPath>| {
                    for c in &src.contours {
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
                            walls.push(ToolPath {
                                kind,
                                closed: true,
                                width_mm: lw,
                                points,
                                z_offset_mm: z_off,
                                flow,
                                group: None,
                                height_scale: hscale,
                                widths: None,
                            });
                        }
                    }
                };
                match halves {
                    // Half-height walls: *every* wall prints as two passes, each
                    // offset from its own sliced contour — the lower half drops
                    // the nozzle by h/2 (ordered first), the upper finishes at
                    // the layer plane. Inner walls follow the upper contour too,
                    // so they never stand proud on shallow treads where the next
                    // layer doesn't cover them.
                    Some((lower, upper)) => {
                        emit_loops(&offset(lower, inset), -0.5 * layer.height_mm, 0.5, &mut walls);
                        emit_loops(&offset(upper, inset), 0.0, 0.5, &mut walls);
                    }
                    None => emit_loops(&centers, z_offset_mm, 1.0, &mut walls),
                }
                if settings.gap_fill && !arachne {
                    // Where material remains at this wall's outer edge but the
                    // wall bead (and everything deeper) is missing, it's a gap.
                    // (Arachne has no between-wall gaps: beads stretch instead.)
                    let covered = offset(&centers, lw * 0.5);
                    if w > 0 {
                        gaps = union(&gaps, &difference(&at_depth, &covered));
                    }
                    at_depth = offset(&centers, -lw * 0.5);
                }
            }

            // Variable-width (arachne) inner walls + thin-feature outer beads.
            if arachne && settings.wall_count > 0 {
                let push_bead = |b: crate::wall::Bead, kind: PathKind, z_off: f64, hs: f64, walls: &mut Vec<ToolPath>| {
                    let max_w = b.widths.iter().cloned().fold(0.0f64, f64::max);
                    walls.push(ToolPath {
                        kind,
                        closed: b.closed,
                        width_mm: max_w,
                        points: b.points,
                        z_offset_mm: z_off,
                        flow: 1.0,
                        group: None,
                        height_scale: hs,
                        widths: Some(b.widths),
                    });
                };
                let cap = settings.wall_count - 1;
                match halves {
                    // Half-height walls: the adaptive field runs per half
                    // contour, so inner beads track the surface like the outer
                    // wall does (each pass contained in its own phase).
                    Some((lower, upper)) => {
                        let vw = crate::wall::variable_walls(interior, &offset(lower, -lw), lw, sp, cap);
                        for b in vw.inner {
                            push_bead(b, PathKind::Perimeter, -0.5 * layer.height_mm, 0.5, &mut walls);
                        }
                        for b in vw.thin_outer {
                            push_bead(b, PathKind::ExternalPerimeter, 0.0, 1.0, &mut walls);
                        }
                        let vw = crate::wall::variable_walls(&Polygons::new(), &offset(upper, -lw), lw, sp, cap);
                        for b in vw.inner {
                            push_bead(b, PathKind::Perimeter, 0.0, 0.5, &mut walls);
                        }
                    }
                    None => {
                        let vw = crate::wall::variable_walls(interior, &offset(interior, -lw), lw, sp, cap);
                        for b in vw.inner {
                            push_bead(b, PathKind::Perimeter, 0.0, 1.0, &mut walls);
                        }
                        for b in vw.thin_outer {
                            push_bead(b, PathKind::ExternalPerimeter, 0.0, 1.0, &mut walls);
                        }
                    }
                }
            }
            // Inset to the infill region (the inner edge of the last wall bead),
            // then morphologically "open" it (erode then dilate by half a line
            // width) to drop slivers narrower than a line — those only produce
            // tiny, useless dabs of infill.
            let wall_depth = match settings.wall_count {
                0 => 0.0,
                wc => lw + (wc - 1) as f64 * sp,
            };
            // Arachne's stretch/absorb regimes own everything thinner than the
            // saturation threshold; the infill region must agree or solid fill
            // collides with the widened beads. Morphological open at the same
            // threshold keeps infill only where the scheme actually saturates.
            let inset = if arachne && settings.wall_count > 0 {
                let r = lw + (settings.wall_count - 1) as f64 * sp + 1.278 * sp;
                let thick_enough = offset(&offset(interior, -r), r);
                offset(&intersection(&thick_enough, interior), -wall_depth)
            } else {
                offset(interior, -wall_depth)
            };
            let opened = offset(&offset(&inset, -lw * 0.5), lw * 0.5);
            if settings.gap_fill {
                // Plus the slivers the morphological open dropped from the
                // infill region (thin necks between walls).
                let base = if !arachne && settings.wall_count > 0 { &at_depth } else { &inset };
                gaps = union(&gaps, &difference(base, &opened));
            }
            // Wall stretches hanging past the layer below print slow with full
            // cooling (the spiral loop must stay whole, so vase mode skips).
            // The unsupported region is usually empty, making this free.
            let walls = if layer.index > 0 && !settings.spiral_vase {
                let below = offset(&layers[layer.index - 1].polygons, 0.05);
                let unsupported = difference(&layer.polygons, &below);
                if unsupported.is_empty() {
                    walls
                } else {
                    slow_overhanging_walls(walls, &unsupported, lw)
                }
            } else {
                walls
            };
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
    let mut plans: Vec<LayerPlan> = walls_per_layer
        .into_par_iter()
        .zip(gaps_per_layer.into_par_iter())
        .enumerate()
        .map(|(i, (mut paths, gaps))| {
        let inner = &inner_per_layer[i];
        let ov = lw * settings.infill_overlap.clamp(0.0, 0.5);
        let sp = config::bead_spacing_mm(lw, layers[i].height_mm);

        if !inner.is_empty() {
            // Arc-overhang mode: the flat unsupported part of this layer's interior
            // is filled with self-supporting arcs instead of normal fill.
            // Without supports (None) the flat overhangs still need handling:
            // spans anchored on both sides bridge with straight lines (like
            // Orca's bridge detection); true cantilevers stay normal fill —
            // nothing rescues those except supports or arc mode.
            let mut supported_below = Polygons::new();
            let overhang_region = if i > 0
                && matches!(settings.support_mode, SupportMode::Arc | SupportMode::None)
            {
                let allowance =
                    settings.layer_height_mm * settings.support_overhang_angle_deg.to_radians().tan();
                supported_below = offset(&layers[i - 1].polygons, allowance);
                let oh = difference(&layers[i].polygons, &supported_below);
                let oh = offset(&offset(&oh, -lw), lw); // open: drop slivers
                intersection(&oh, inner)
            } else {
                Polygons::new()
            };

            // Decide per disjoint island: a gap supported on ≥2 sides bridges
            // with straight lines; otherwise arc mode arc-fills, support-less
            // mode leaves it to the normal fill flow. Only the islands that
            // actually got covered are carved out of the solid/sparse split.
            let mut bridged = Polygons::new();
            for island in islands(&overhang_region) {
                let segs = match try_bridge(&island, &supported_below, lw, settings.max_bridge_span_mm) {
                    Some(segs) => segs
                        .into_iter()
                        .map(|seg| (PathKind::Bridge, seg))
                        .collect::<Vec<_>>(),
                    None if settings.support_mode == SupportMode::Arc => {
                        crate::arc::arc_fill(&island, &supported_below, lw, settings.max_arc_radius_mm, settings.arc_seam_overlap_mm)
                            .into_iter()
                            .map(|seg| (PathKind::ArcOverhang, seg))
                            .collect()
                    }
                    None => continue,
                };
                for (kind, seg) in segs {
                    if seg.len() >= 2 {
                        paths.push(ToolPath::new(kind, false, lw, seg));
                    }
                }
                bridged.contours.extend(island.contours);
            }

            let solid_all = &solid_all_per_layer[i];
            let solid = difference(solid_all, &bridged);
            let sparse = difference(&difference(inner, solid_all), &bridged);
            let (solid, sparse) = rebalance_solid_sparse(solid, sparse, lw);

            // Alternate fill direction per layer for cross-hatching.
            let angle = if i % 2 == 0 { 45.0 } else { 135.0 };

            // Internal bridges: where this layer's solid sits on sparse infill
            // (the first shell layer over the core), the beads span open cells
            // — print them as bridges, oriented across the sparse lines below
            // (span = one line spacing), before the rest of the solid fill.
            let internal_bridge = if i > 0 && settings.infill_density > 0.0 && !solid.is_empty() {
                let sparse_below =
                    difference(&inner_per_layer[i - 1], &solid_all_per_layer[i - 1]);
                let ib = intersection(&solid, &sparse_below);
                // Open: a band thinner than a line is covered by the solid
                // loop's bead anyway, and micro-islands aren't worth a pass.
                offset(&offset(&ib, -lw * 0.5), lw * 0.5)
            } else {
                Polygons::new()
            };

            if !solid.is_empty() {
                // A perimeter loop following the solid region's boundary (so where
                // it runs alongside the shell it becomes a clean concentric bead),
                // then straight-fill only the interior left inside that loop. Thin
                // solid bands are consumed entirely by the loop — no lone strands.
                // The loop and the fill both push `ov` into their neighbor so
                // solid surfaces bond to the walls.
                let solid_loop = offset(&solid, -(lw * 0.5 - ov * 0.5));
                for c in solid_loop.contours {
                    if c.points.len() < 3 {
                        continue;
                    }
                    // Offsetting a dumbbell-shaped region can pinch off
                    // micro-rings the island rebalance couldn't see; a loop
                    // shorter than ~4 beads is a dab, not a surface.
                    let m = c.points.len();
                    let perim: f64 =
                        (0..m).map(|j| pt_dist_mm(c.points[j], c.points[(j + 1) % m])).sum();
                    if perim < lw * 4.0 {
                        continue;
                    }
                    let points = place_seam(c.points, settings.seam_mode, i);
                    paths.push(ToolPath::new(PathKind::Solid, true, lw, points));
                }
                let solid_core = offset(&solid, -(0.5 * (lw + sp) - 0.5 * ov));
                if !internal_bridge.is_empty() {
                    // Bridge lines run perpendicular to the sparse lines below
                    // (each free span = one line spacing) and extend half a
                    // bead into the supported solid around them to anchor.
                    let below_angle = if (i - 1) % 2 == 0 { 45.0 } else { 135.0 };
                    let lines_region =
                        intersection(&offset(&internal_bridge, lw * 0.5), &solid_core);
                    for seg in infill_lines(&lines_region, below_angle + 90.0, sp, false, 0.5) {
                        paths.push(ToolPath::new(PathKind::InternalBridge, false, lw, seg));
                    }
                }
                let solid_fill = if internal_bridge.is_empty() {
                    solid_core.clone()
                } else {
                    difference(&solid_core, &internal_bridge)
                };
                if !solid_fill.is_empty() {
                    fill_region(
                        &solid_fill, settings.solid_pattern, sp, angle, lw, PathKind::Solid,
                        settings.seam_mode, i, layers[i].z_mm, settings.monotonic_solid, &mut paths,
                    );
                }
            }
            if settings.infill_density > 0.0 && !sparse.is_empty() {
                let spacing = sp / settings.infill_density;
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
                for seg in crate::fill::infill_lines(&island, along, gw, false, 0.5) {
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
                    for seg in crate::fill::infill_lines(&island, 45.0, spacing, true, 0.5) {
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
    if settings.seam_mode == SeamMode::Aligned {
        align_seams(&mut plans);
    }
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
    let sp = config::bead_spacing_mm(lw, settings.layer_height_mm);
    let spacing = sp / settings.support_density.clamp(0.02, 1.0);
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
                fill_region(&iface_here, InfillPattern::Lines, sp, angle, lw,
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
    const SOLID_BELOW_AREA_MM2: f64 = 10.0;
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
fn slow_overhanging_walls(walls: Vec<ToolPath>, unsupported: &Polygons, lw: f64) -> Vec<ToolPath> {
    let min_run_mm = lw * 2.0;
    let mut out = Vec::with_capacity(walls.len());
    for path in walls {
        if !matches!(path.kind, PathKind::ExternalPerimeter | PathKind::Perimeter) || path.points.len() < 2 {
            out.push(path);
            continue;
        }
        let n = path.points.len();
        let segs = if path.closed { n } else { n - 1 };
        let class: Vec<bool> = (0..segs)
            .map(|k| {
                let a = path.points[k];
                let b = path.points[(k + 1) % n];
                in_polys(unsupported, Point::new((a.x + b.x) / 2, (a.y + b.y) / 2))
            })
            .collect();
        if class.iter().all(|&c| !c) {
            out.push(path);
            continue;
        }
        if class.iter().all(|&c| c) {
            let mut p = path;
            p.kind = PathKind::OverhangWall;
            out.push(p);
            continue;
        }
        // Mixed: gather maximal runs (cyclic for loops), starting at a border.
        let seg_len = |k: usize| pt_dist_mm(path.points[k], path.points[(k + 1) % n]);
        let start = if path.closed {
            (0..segs).find(|&k| class[(k + segs - 1) % segs] != class[k]).unwrap_or(0)
        } else {
            0
        };
        let mut runs: Vec<(bool, Vec<usize>, f64)> = Vec::new();
        for i in 0..segs {
            let k = (start + i) % segs;
            let len = seg_len(k);
            match runs.last_mut() {
                Some((c, idxs, l)) if *c == class[k] => {
                    idxs.push(k);
                    *l += len;
                }
                _ => runs.push((class[k], vec![k], len)),
            }
        }
        // Dissolve sub-threshold runs into the previous one (the previous run
        // is always sound: it either met the threshold or absorbed others).
        let mut merged: Vec<(bool, Vec<usize>, f64)> = Vec::new();
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
            if merged[0].0 {
                p.kind = PathKind::OverhangWall;
            }
            out.push(p);
            continue;
        }
        for (over, idxs, _) in merged {
            // Segment indices are consecutive (mod n): the piece's points run
            // from the first segment's start to the last segment's end.
            let first = idxs[0];
            let count = idxs.len();
            let mut points = Vec::with_capacity(count + 1);
            let mut widths = path.widths.as_ref().map(|_| Vec::with_capacity(count + 1));
            for j in 0..=count {
                let idx = (first + j) % n;
                points.push(path.points[idx]);
                if let (Some(ws), Some(src)) = (widths.as_mut(), path.widths.as_ref()) {
                    ws.push(src[idx]);
                }
            }
            out.push(ToolPath {
                kind: if over { PathKind::OverhangWall } else { path.kind },
                closed: false,
                width_mm: path.width_mm,
                points,
                z_offset_mm: path.z_offset_mm,
                flow: path.flow,
                group: path.group,
                height_scale: path.height_scale,
                widths,
            });
        }
    }
    out
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
        let segs = infill_lines(region, angle, lw, false, 0.5);
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
    Some(infill_lines(region, angle, lw, false, 0.5))
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
        let (iron, mut rest): (Vec<_>, Vec<_>) =
            rest.into_iter().partition(|p| p.kind == PathKind::Ironing);
        // Print z-phases in ascending order — half-height lower outer walls
        // (−h/2) first, then the layer plane, then brick-lifted (+h/2) — so the
        // nozzle never descends into material already printed this layer.
        let mut phases: Vec<f64> = rest.iter().map(|p| p.z_offset_mm).collect();
        phases.sort_by(|a, b| a.partial_cmp(b).unwrap());
        phases.dedup_by(|a, b| (*a - *b).abs() < 1.0e-9);
        let mut paths = prime;
        for ph in phases {
            let (group, remaining): (Vec<_>, Vec<_>) =
                rest.into_iter().partition(|p| (p.z_offset_mm - ph).abs() < 1.0e-9);
            rest = remaining;
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
    // Minuscule solid dashes are pure overhead: the solid boundary loop
    // already covers the region's rim, so a sub-1.5-line-width row-end stub
    // adds a travel (often a retraction) for material the loop deposited.
    // Sparse and support lines keep the small default — their patterns rely
    // on short links.
    let min_len = if kind == PathKind::Solid { lw * 1.5 } else { 0.5 };
    // Scanline patterns sweep each island separately when monotonic.
    let scan = |sets: &[(f64, f64)], out: &mut Vec<ToolPath>| {
        if monotonic {
            for (gi, island) in islands(region).iter().enumerate() {
                for (si, &(a, sp)) in sets.iter().enumerate() {
                    let group = Some((gi * sets.len() + si) as u32);
                    push_lines(infill_lines(island, a, sp, true, min_len), group, out);
                }
            }
        } else {
            for &(a, sp) in sets {
                push_lines(infill_lines(region, a, sp, false, min_len), None, out);
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
        // Aligned starts from the rear like Nearest; `align_seams` then walks
        // the layers in order and snaps each loop to the previous layer's
        // seam, so this is only the first layer's seed.
        SeamMode::Aligned => (0..n)
            .max_by_key(|&i| (points[i].y, points[i].x))
            .unwrap(),
    };
    points.rotate_left(start);
    points
}

/// Aligned seams: walk the layers bottom-up, rotating every closed outer-wall
/// loop to start at the vertex nearest the seam of the loop below it — the
/// seam follows one continuous line up the print even where the rear-most
/// vertex would jump between competing features. Loops with no seam within
/// `SEAM_TRACK_RADIUS_MM` below them (new islands) seed a new track at their
/// place_seam position. Runs before travel planning, so combing and lead-ins
/// see the final start points.
fn align_seams(plans: &mut [LayerPlan]) {
    const SEAM_TRACK_RADIUS_MM: f64 = 10.0;
    let mut tracks: Vec<Point> = Vec::new();
    for plan in plans.iter_mut() {
        for path in plan.paths.iter_mut() {
            if path.kind != PathKind::ExternalPerimeter || !path.closed || path.points.len() < 3 {
                continue;
            }
            // Nearest (track, vertex) pair for this loop.
            let mut best: Option<(f64, usize, usize)> = None; // (dist, vertex, track)
            for (ti, t) in tracks.iter().enumerate() {
                for (vi, p) in path.points.iter().enumerate() {
                    let d = pt_dist_mm(*p, *t);
                    if best.map_or(true, |(bd, _, _)| d < bd) {
                        best = Some((d, vi, ti));
                    }
                }
            }
            match best {
                Some((d, vi, ti)) if d <= SEAM_TRACK_RADIUS_MM => {
                    path.points.rotate_left(vi);
                    if let Some(ws) = path.widths.as_mut() {
                        ws.rotate_left(vi);
                    }
                    tracks[ti] = path.points[0];
                }
                _ => tracks.push(path.points[0]), // new island: seed where place_seam put it
            }
        }
    }
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
        let over: Vec<&ToolPath> =
            first_slab.paths.iter().filter(|p| p.kind == PathKind::OverhangWall).collect();
        assert!(!over.is_empty(), "cantilever walls must be marked overhanging");
        // Overhanging stretches live in the cantilever (y > 20, with margin
        // for the bead inset); supported walls remain.
        for p in &over {
            for pt in &p.points {
                assert!(pt.y_mm() > 19.0, "overhang piece at supported y={:.1}", pt.y_mm());
            }
        }
        assert!(
            first_slab.paths.iter().any(|p| matches!(p.kind, PathKind::ExternalPerimeter | PathKind::Perimeter)),
            "supported walls keep their kind"
        );
        // The layer above the cantilever's first is fully supported again.
        let next = plans.iter().find(|p| p.print_z_mm > first_slab.print_z_mm).unwrap();
        assert_eq!(count(next, PathKind::OverhangWall), 0, "supported layer must not slow");
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
    fn anchored_spans_bridge_without_supports() {
        // A table: slab across two legs with a 4mm air gap between them.
        // With support off, the gap's first layer must print as straight
        // Bridge lines (it's anchored on both sides); the unsupported span
        // must not silently print as ordinary fill.
        let mut tris = Vec::new();
        push_box(&mut tris, [0.0, 0.0, 0.0], [4.0, 10.0, 4.0]);
        push_box(&mut tris, [8.0, 0.0, 0.0], [12.0, 10.0, 4.0]);
        push_box(&mut tris, [0.0, 0.0, 4.0], [12.0, 10.0, 6.0]);
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let s = Settings { skirt_loops: 0, ..Settings::default() };
        assert_eq!(s.support_mode, SupportMode::None);
        let plans = generate(&m, &s);
        let slab_first = plans.iter().find(|p| p.print_z_mm > 4.05).unwrap();
        let bridges: Vec<&ToolPath> =
            slab_first.paths.iter().filter(|p| p.kind == PathKind::Bridge).collect();
        assert!(!bridges.is_empty(), "the anchored gap must bridge");
        let (min_x, max_x) = bridges
            .iter()
            .flat_map(|b| b.points.iter())
            .fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.x_mm()), hi.max(p.x_mm())));
        // The model is centered on the bed; the gap is the middle third of a
        // 12mm-wide part. Bridge lines must stay near it (≤ a couple mm of
        // anchor overlap past each side).
        assert!(max_x - min_x < 8.0, "bridge lines span {:.1}mm — leaked beyond the gap", max_x - min_x);
        // And the layer above is fully supported: no bridges.
        let above = plans.iter().find(|p| p.print_z_mm > slab_first.print_z_mm).unwrap();
        assert_eq!(count(above, PathKind::Bridge), 0, "second slab layer re-bridged");
    }

    #[test]
    fn first_solid_layer_over_sparse_bridges() {
        // In a cube, the first top-shell layer sits on 15% sparse infill: its
        // interior must print as InternalBridge spans (perpendicular to the
        // sparse lines below), while the layers above it — supported by that
        // now-solid layer — must not.
        let m = mesh::Mesh::cube(20.0);
        let s = Settings { skirt_loops: 0, ..Settings::default() };
        let plans = generate(&m, &s);
        let n = plans.len();
        let first_top = plans
            .iter()
            .position(|p| count(p, PathKind::InternalBridge) > 0)
            .expect("some layer bridges over the sparse core");
        assert_eq!(first_top, n - s.top_layers, "bridges start at the first top-shell layer");
        for p in &plans[first_top + 1..] {
            assert_eq!(count(p, PathKind::InternalBridge), 0, "layer {} re-bridges", p.index);
        }
        // The bottom shells sit on the bed / each other — never bridged.
        for p in &plans[..first_top] {
            assert_eq!(count(p, PathKind::InternalBridge), 0, "layer {} bridges early", p.index);
        }
        // The bridged layer still has its solid loop and the bridges carry
        // real length.
        let ib_len: f64 = plans[first_top]
            .paths
            .iter()
            .filter(|p| p.kind == PathKind::InternalBridge)
            .flat_map(|p| p.points.windows(2))
            .map(|w| pt_dist_mm(w[0], w[1]))
            .sum();
        assert!(ib_len > 50.0, "internal bridge length {ib_len:.0}mm");
        assert!(count(&plans[first_top], PathKind::Solid) > 0, "anchor loop survives");
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
    fn thin_fin_core_is_a_tapered_wall_not_a_bandaid() {
        // A 1.2mm-wide fin with lw=0.45 and 2 walls: the 0.3mm core between the
        // outer wall pair becomes a real variable-width Perimeter bead in
        // arachne mode (no GapFill strokes), and a GapFill stroke in classic.
        let m = box_mesh(1.2, 20.0, 5.0);
        let s = Settings { wall_count: 2, skirt_loops: 0, wall_mode: config::WallMode::Arachne, ..Settings::default() };
        let layers = generate(&m, &s);
        let mid = &layers[10];
        assert_eq!(count(mid, PathKind::GapFill), 0, "arachne: no gap-fill bandaids");
        let bead = mid
            .paths
            .iter()
            .find(|p| p.kind == PathKind::Perimeter && p.widths.is_some())
            .expect("variable-width core bead");
        let ws = bead.widths.as_ref().unwrap();
        let mid_w = ws[ws.len() / 2];
        assert!(
            (0.2..=0.45).contains(&mid_w),
            "core bead width {mid_w} should match the ~0.3mm core"
        );
        assert_eq!(count(mid, PathKind::Infill), 0, "no room for sparse infill");

        // Classic mode keeps the old behavior: a GapFill stroke.
        let s = Settings { wall_mode: config::WallMode::Classic, ..s };
        let layers = generate(&m, &s);
        assert!(count(&layers[10], PathKind::GapFill) > 0, "classic: gap-filled");
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

    /// A square frustum: `base`-wide at z=0 tapering to `top` at height `h`
    /// (45-degree slopes when (base-top)/2 == h). Sloped walls make the
    /// half-height outer passes follow visibly different contours.
    fn frustum(base: f64, top: f64, h: f64) -> Mesh {
        let (b, t) = (base / 2.0, top / 2.0);
        let v = |x: f64, y: f64, z: f64| [x, y, z];
        let verts = vec![
            v(-b, -b, 0.0), v(b, -b, 0.0), v(b, b, 0.0), v(-b, b, 0.0), // 0-3 base
            v(-t, -t, h), v(t, -t, h), v(t, t, h), v(-t, t, h),          // 4-7 top
        ];
        let quads = [
            [0u32, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], // sides
            [3, 2, 1, 0], // bottom
            [4, 5, 6, 7], // top
        ];
        let mut tris = Vec::new();
        for q in quads {
            tris.push([q[0], q[1], q[2]]);
            tris.push([q[0], q[2], q[3]]);
        }
        Mesh { vertices: verts, triangles: tris }
    }

    #[test]
    fn half_height_outer_walls_follow_their_own_contours() {
        // 45-degree slopes: the outline shrinks 1:1 with z, so the lower pass
        // (sampled h/4 below the layer midpoint) spans ~h/2 wider per axis than
        // the upper pass (h/4 above) -> span difference of ~h.
        let m = frustum(20.0, 10.0, 5.0);
        let mut s = Settings { skirt_loops: 0, ..Settings::default() };
        s.half_height_outer_walls = true;
        let layers = generate(&m, &s);
        let mid = &layers[10];
        let h = s.layer_height_mm;

        let outers: Vec<&ToolPath> = mid
            .paths
            .iter()
            .filter(|p| p.kind == PathKind::ExternalPerimeter)
            .collect();
        assert_eq!(outers.len(), 2, "two half passes per layer");
        let lower = outers.iter().find(|p| p.z_offset_mm < 0.0).expect("lower pass");
        let upper = outers.iter().find(|p| p.z_offset_mm == 0.0).expect("upper pass");
        assert!((lower.z_offset_mm + 0.5 * h).abs() < 1e-9);
        assert!((lower.height_scale - 0.5).abs() < 1e-9);
        assert!((upper.height_scale - 0.5).abs() < 1e-9);

        let span = |p: &ToolPath| {
            let xs: Vec<f64> = p.points.iter().map(|pt| pt.x_mm()).collect();
            xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min)
        };
        let diff = span(lower) - span(upper);
        assert!(
            (diff - h).abs() < 0.06,
            "45-degree slope: lower span should exceed upper by ~{h}, got {diff:.3}"
        );

        // The lower pass prints before everything else printable on the layer.
        let first = mid.paths.iter().position(|p| p.points.len() >= 2).unwrap();
        assert!(mid.paths[first].z_offset_mm < 0.0, "lower outer phase prints first");

        // Layer 0 stays one full-height pass for bed squish.
        let l0: Vec<&ToolPath> = layers[0]
            .paths
            .iter()
            .filter(|p| p.kind == PathKind::ExternalPerimeter)
            .collect();
        assert_eq!(l0.len(), 1);
        assert_eq!(l0[0].height_scale, 1.0);
    }

    #[test]
    fn inner_walls_stay_inside_half_height_outer_on_shallow_slopes() {
        // Shallow slope (rise 2 over run 5 per side): the layer-midpoint
        // outline is wider than the upper half pass — interior geometry must
        // derive from the intersection of the halves, or inner walls poke
        // outside the outer wall (seen on the Benchy roof).
        let m = frustum(20.0, 10.0, 2.0);
        let mut s = Settings { skirt_loops: 0, ..Settings::default() };
        s.half_height_outer_walls = true;
        let layers = generate(&m, &s);
        let span = |p: &ToolPath| {
            let xs: Vec<f64> = p.points.iter().map(|pt| pt.x_mm()).collect();
            xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min)
        };
        let sp = config::bead_spacing_mm(s.line_width_mm, s.layer_height_mm);
        for layer in layers.iter().skip(1) {
            for phase in [0.0, -0.5 * s.layer_height_mm] {
                let outer = layer
                    .paths
                    .iter()
                    .filter(|p| p.kind == PathKind::ExternalPerimeter && (p.z_offset_mm - phase).abs() < 1e-9)
                    .map(|p| span(p))
                    .fold(f64::MIN, f64::max);
                for p in layer
                    .paths
                    .iter()
                    .filter(|p| p.kind == PathKind::Perimeter && (p.z_offset_mm - phase).abs() < 1e-9)
                {
                    assert!(
                        (p.height_scale - 0.5).abs() < 1e-9,
                        "inner walls are half-height under this feature"
                    );
                    assert!(
                        span(p) <= outer - 1.5 * sp,
                        "layer {} phase {phase}: inner span {:.3} escapes outer span {:.3}",
                        layer.index,
                        span(p),
                        outer
                    );
                }
            }
        }
    }

    #[test]
    fn tiny_sparse_pockets_become_solid() {
        // A 2.4 mm-wide bar: the interior pocket is ~4 mm² — far too small for
        // meaningful 15% sparse fill, so it must be promoted to solid.
        let m = box_mesh(2.4, 8.0, 5.0);
        // Arachne (default): the pocket is below saturation — beads own it,
        // no fill of any kind.
        let s = Settings { skirt_loops: 0, ..Settings::default() };
        let layers = generate(&m, &s);
        let mid = &layers[10];
        assert_eq!(count(mid, PathKind::Infill), 0, "no sparse fill in a tiny pocket");
        assert!(
            mid.paths.iter().any(|p| p.kind == PathKind::Perimeter && p.widths.is_some()),
            "arachne beads cover the pocket"
        );
        // Classic: the pocket is promoted to solid instead of 15% sparse.
        let s = Settings { skirt_loops: 0, wall_mode: config::WallMode::Classic, ..s };
        let layers = generate(&m, &s);
        let mid = &layers[10];
        assert_eq!(count(mid, PathKind::Infill), 0, "no sparse fill in a tiny pocket");
        assert!(count(mid, PathKind::Solid) > 0, "pocket promoted to solid");
    }

    /// Prism with one vertical side and one shallow slope (run 3 per rise 1),
    /// like the Benchy roof: inner rings must not lose their shallow-side arc.
    fn wedge() -> Mesh {
        let v = |x: f64, y: f64, z: f64| [x, y, z];
        let verts = vec![
            v(0.0, 0.0, 0.0), v(20.0, 0.0, 0.0), v(20.0, 10.0, 0.0), v(0.0, 10.0, 0.0),
            v(0.0, 0.0, 3.0), v(20.0, 0.0, 3.0), v(20.0, 1.0, 3.0), v(0.0, 1.0, 3.0),
        ];
        let quads = [
            [0u32, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
            [3, 2, 1, 0], [4, 5, 6, 7],
        ];
        let mut tris = Vec::new();
        for q in quads {
            tris.push([q[0], q[1], q[2]]);
            tris.push([q[0], q[2], q[3]]);
        }
        Mesh { vertices: verts, triangles: tris }
    }

    #[test]
    fn arachne_inner_ring_covers_shallow_side() {
        let mut s = Settings { skirt_loops: 0, wall_mode: config::WallMode::Arachne, ..Settings::default() };
        s.half_height_outer_walls = true;
        let layers = generate(&m_wedge(), &s);
        let mid = &layers[7];
        for ph in [0.0, -0.5 * s.layer_height_mm] {
            let outer_len: f64 = mid
                .paths
                .iter()
                .filter(|p| p.kind == PathKind::ExternalPerimeter && (p.z_offset_mm - ph).abs() < 1e-9)
                .flat_map(|p| p.points.windows(2))
                .map(|w| pt_dist_mm(w[0], w[1]))
                .sum();
            let inner_len: f64 = mid
                .paths
                .iter()
                .filter(|p| p.kind == PathKind::Perimeter && (p.z_offset_mm - ph).abs() < 1e-9)
                .flat_map(|p| p.points.windows(2))
                .map(|w| pt_dist_mm(w[0], w[1]))
                .sum();
            eprintln!("phase {ph}: outer {outer_len:.1} inner {inner_len:.1}");
            assert!(
                inner_len > 0.6 * outer_len,
                "phase {ph}: inner ring covers only {inner_len:.1} of outer {outer_len:.1}"
            );
        }
    }

    fn m_wedge() -> Mesh {
        wedge()
    }

    #[test]
    fn half_outer_walls_exclude_brick() {
        let m = Mesh::cube(20.0);
        let mut s = Settings { skirt_loops: 0, wall_count: 4, ..Settings::default() };
        s.half_height_outer_walls = true;
        s.brick_layers = true; // collides - brick must yield
        let layers = generate(&m, &s);
        let mid = &layers[50];
        assert!(
            mid.paths.iter().all(|p| p.z_offset_mm <= 0.0),
            "no brick-lifted (positive offset) paths when half-outer is on"
        );
        assert!(
            mid.paths.iter().any(|p| p.z_offset_mm < 0.0 && p.height_scale == 0.5),
            "half-height lower pass present"
        );
    }

    #[test]
    fn walls_are_placed_at_stadium_spacing() {
        // Classic mode: this pins the exact offset constants (arachne's
        // grid-extracted rings match within a cell — covered in wall::tests).
        let m = Mesh::cube(20.0);
        let s = Settings { skirt_loops: 0, wall_mode: config::WallMode::Classic, ..Settings::default() };
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
        let solids: Vec<&ToolPath> = layers[1]
            .paths
            .iter()
            .filter(|p| p.kind == PathKind::Solid && !p.closed)
            .collect();
        assert!(solids.len() > 10);
        // Project line midpoints onto the scanline axis; consecutive monotonic
        // lines must sit one stadium spacing apart.
        let angle = 135.0_f64.to_radians();
        let proj = |p: &ToolPath| {
            let m = p.points[0];
            -m.x_mm() * angle.sin() + m.y_mm() * angle.cos()
        };
        let sp = config::bead_spacing_mm(s.line_width_mm, s.layer_height_mm);
        let mut gaps: Vec<f64> = solids.windows(2).map(|w| (proj(w[1]) - proj(w[0])).abs()).collect();
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
