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
    /// Self-supporting concentric arc fill over a flat overhang (arc-overhang
    /// technique) — each arc cantilevers sideways off the previous ring, so
    /// it runs far slower than a straight, both-ends-anchored bridge.
    ArcOverhang,
    /// Sparse interior fill.
    Infill,
    /// Single width-matched strokes filling gaps too thin for a wall or normal
    /// infill (between/inside walls), traced down each gap's medial axis.
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
    /// Per-point extrusion width (mm), one per `points` entry, for a continuously
    /// tapering bead (gap fill). `None` = uniform `width_mm`. When set, the g-code
    /// varies E per segment and the preview renders a variable-width ribbon;
    /// `width_mm` then carries the mean (used for feed/flow/estimates).
    pub widths: Option<Vec<f64>>,
}

impl ToolPath {
    fn new(kind: PathKind, closed: bool, width_mm: f64, points: Vec<Point>) -> Self {
        Self {
            kind,
            closed,
            width_mm,
            points,
            z_offset_mm: 0.0,
            flow: 1.0,
            group: None,
            height_scale: 1.0,
            widths: None,
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
    /// Speed multiplier (≤1) applied to this layer: min-layer-time cooling and
    /// heat control's slowdown combined (what the g-code and estimates use).
    pub speed_scale: f64,
    /// Heat control's share of `speed_scale` (≤1, 1.0 = unslowed) — kept apart
    /// so the audit can report how much heat control slowed each layer,
    /// distinct from the min-layer-time pacing.
    pub heat_scale: f64,
    /// The heat target (W/mm²) pinned at plan time — computed once on the raw
    /// plan and stored so the speed lever, the audits, and the GUI all chase
    /// the same number (recomputing on the governed plan drifts).
    pub planned_heat_target: Option<f64>,
    /// A per-layer nozzle °C override for this layer (None = the profile
    /// temperature). Drives the deposited-energy model and the flow ceiling.
    pub planned_temp_c: Option<f64>,
    /// Emit `M104 S<this>` at the layer's start (None = no change). A plain
    /// asynchronous setpoint — set without waiting, no live feedback.
    pub temp_command_c: Option<f64>,
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
        norm_settings.fuzzy_skin = false;
    }
    if norm_settings.half_height_outer_walls && norm_settings.brick_layers {
        // Mutually exclusive: their Z choreographies collide (the lower outer
        // pass would graze the previous layer's lifted brick beads).
        norm_settings.brick_layers = false;
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
    // aren't over-dense (cleaner preview, faster planning, smaller g-code). The
    // threshold is derived from the bead — see config::contour_resolution_mm.
    // Then dimensional compensation: XY grow/shrink on every layer, and the
    // first layer pulled in to counter squish (elephant foot).
    let res = config::contour_resolution_mm(settings.line_width_mm);
    layers.par_iter_mut().for_each(|layer| {
        layer.polygons = simplify(&layer.polygons, res);
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
                    p = simplify(&p, res);
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
        .zip(outer_halves.par_iter())
        .map(|(layer, halves)| {
            // Adjacent beads are placed at the stadium spacing: rounded
            // shoulders overlap just enough to fill the cusps between beads.
            let sp = config::bead_spacing_mm(lw, layer.height_mm);
            // With half-height outer walls, interior geometry must stay inside
            // *both* half contours (on a shallow slope the layer-midpoint
            // outline is wider than the upper pass — inner walls based on it
            // would poke outside the outer wall). The intersection is the safe
            // core; without halves it's just the layer outline.
            let interior_owned = halves.as_ref().map(|(lo, up)| intersection(lo, up));
            let interior: &Polygons = interior_owned.as_ref().unwrap_or(&layer.polygons);
            // Carve the exposed surface out of the inner-wall region: the outer
            // wall still hugs the full outline, but the inner walls — and the
            // infill region derived from them — stop at the surface, leaving it
            // for the skin. `surf_inside` is the slice the skin reclaims.
            //
            // ...but only the SKINNABLE part of the surface — what survives a
            // one-line morphological open. A surface sliver too thin to hold real
            // skin (a bow tip, a tapering edge) prints cleaner as walls running
            // into it than as stubby solid dabs that the solid/sparse rebalance
            // then demotes to sparse infill. So thinner surface stays in the wall
            // region and the inner walls fill it right out to the tip.
            let surf_owned = &surface_per_layer[layer.index];
            let skinnable = offset(&offset(surf_owned, -lw * 2.0), lw * 2.0);
            // Don't carve an over-air bottom surface the walls can close (a door
            // lintel) out of the wall region — otherwise the inner walls loop back
            // around the just-closed notch instead of crossing it. Keep the whole
            // spannable over-air span in the wall region so the wall beads run
            // straight across; a wider bridge the walls can't close stays carved.
            let wd = match settings.wall_count {
                0 => 0.0,
                wc => lw + (wc - 1) as f64 * sp,
            };
            let spannable_air = if layer.index > 0 && wd > 0.0 {
                let allowance = settings.layer_height_mm
                    * settings.support_overhang_angle_deg.to_radians().tan();
                let over_air =
                    difference(&layer.polygons, &offset(&layers[layer.index - 1].polygons, allowance));
                // Keep the WHOLE skinnable surface patch when it's a thin bridge the
                // walls cross: it must be over air (a lintel, not a flat shelf) AND
                // close under the span. Taking the whole patch — not just its steep
                // over-air core — means the shallow allowance ring at the span ends
                // becomes wall too, instead of a tiny solid pocket breaking the span.
                // A large flat over-air face (a cabin roof) survives the erosion and
                // stays surface, so it skins. The split is by width: a lintel erodes
                // to nothing under a few mm, a roof slab doesn't. This cap is a
                // FIXED few-mm lintel width, NOT tied to max_bridge_span_mm: a wide
                // ceiling must skin+bridge, even though a tall wall stack (large
                // `wd`) could nominally "close" it with looping, sagging rings.
                let span = wd.min(3.0).max(lw);
                let mut keep = Polygons::new();
                for isl in islands(&skinnable) {
                    if offset(&isl, -span).is_empty() && !intersection(&isl, &over_air).is_empty() {
                        keep.contours.extend(isl.contours);
                    }
                }
                keep
            } else {
                Polygons::new()
            };
            let surf = difference(&skinnable, &spannable_air);
            let core = difference(interior, &surf);
            let surf_inside = intersection(&surf, &offset(&layer.polygons, -lw));
            let interior: &Polygons = &core;
            let mut walls = Vec::new();
            for w in 0..settings.wall_count {
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
                    // The lifted bead's extra material is derived from the
                    // stadium model — what its halved flank contact leaves
                    // unfilled (see config::brick_flow_factor).
                    (
                        0.5 * settings.layer_height_mm,
                        config::brick_flow_factor(lw, settings.layer_height_mm),
                    )
                } else {
                    (0.0, 1.0)
                };
                // Outer wall (w == 0) hugs the true outline (or its halves);
                // everything deeper offsets from the safe interior core at stadium
                // spacing. The medial seams these leave are closed by gap fill.
                let centers = offset(if w == 0 { &layer.polygons } else { interior }, inset);
                let emit_loops = |src: &Polygons, z_off: f64, hscale: f64, walls: &mut Vec<ToolPath>| {
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
            }

            // Inset to the infill region (the inner edge of the last wall bead),
            // then morphologically "open" it (erode then dilate by half a line
            // width) to drop slivers narrower than a line — those only produce
            // tiny, useless dabs of infill.
            let wall_depth = match settings.wall_count {
                0 => 0.0,
                wc => lw + (wc - 1) as f64 * sp,
            };
            let inset = offset(interior, -wall_depth);
            let opened = offset(&offset(&inset, -lw * 0.5), lw * 0.5);
            // The surface rejoins the infill region so the skin claims it (the
            // inner walls were kept clear of it above).
            let opened = union(&opened, &surf_inside);
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
            (walls, opened)
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
    let mut plans: Vec<LayerPlan> = walls_per_layer
        .into_par_iter()
        .enumerate()
        .map(|(i, mut paths)| {
        let inner = &inner_per_layer[i];
        let ov = lw * settings.infill_overlap.clamp(0.0, 0.5);
        let sp = config::bead_spacing_mm(lw, layers[i].height_mm);
        // The sparse-infill region, captured for the gap-fill pass below: its
        // intended between-line gaps must be excluded from void detection.
        let mut sparse_region = Polygons::new();

        if !inner.is_empty() {
            // Unsupported interior, computed in every mode. A span anchored on
            // both ends (a ceiling enclosed by walls) is reliably bridgeable — it's
            // correct bottom-surface printing, not a "rescue" — so it bridges
            // regardless of support mode (below). What's mode-gated is the rescue of
            // NON-anchored overhangs: arc mode arc-fills them, no-support mode leaves
            // them to the ordered bottom shell.
            let mut supported_below = Polygons::new();
            let overhang_region = if i > 0 {
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
            sparse_region = sparse.clone();

            // Close the wall↔skin seam. Inner walls offset from `core` starting at
            // w=1, so the innermost ring stops ~one bead-spacing short of the
            // surface boundary (nothing hugs that inner edge the way the outer wall
            // hugs the outline), and the skin insets from the same boundary — a
            // ~1-line void rings every inset top surface. Grow the skin out into the
            // wall band so its perimeter loop overlaps the innermost bead and bonds.
            // Bounded to the band — never past the outer wall, never into sparse — so
            // it stays a thin seam, not a flood of the whole band.
            let solid = if settings.wall_count > 0 && !solid.is_empty() {
                let reach = sp + lw * 0.5;
                let band = difference(&offset(&layers[i].polygons, -lw), inner);
                union(&solid, &intersection(&offset(&solid, reach), &band))
            } else {
                solid
            };

            // Alternate fill direction per layer for cross-hatching; aligned-lines
            // infill instead keeps one orientation every layer, so its globally
            // anchored lines stack into continuous walls.
            let alt_angle = if i % 2 == 0 { 45.0 } else { 135.0 };
            let pat_angle = |pat: InfillPattern| {
                if pat == InfillPattern::AlignedLines { 45.0 } else { alt_angle }
            };

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
                // Split the shell by what shows: skin with open air above (top),
                // skin with nothing printed below it (bottom — the bed face and
                // unsupported undersides that didn't bridge), and the buried
                // rest. Skins print at the visible-surface pace and always
                // monotonic. Coverage tests use the neighbors' full outlines —
                // a surface under nothing but the wall ring above is still
                // hidden — and a light open drops the hair-thin ribbons the
                // coverage differences leave behind.
                let open = |p: Polygons| offset(&offset(&p, -lw * 0.1), lw * 0.1);
                let skin_bottom = if i == 0 {
                    solid.clone()
                } else {
                    open(difference(&solid, &offset(&layers[i - 1].polygons, 0.05)))
                };
                let buried = difference(&solid, &skin_bottom);
                let skin_top = if i + 1 < n {
                    open(difference(&buried, &offset(&layers[i + 1].polygons, 0.05)))
                } else {
                    buried.clone()
                };
                let internal = difference(&buried, &skin_top);
                let regions = [
                    (skin_bottom, PathKind::BottomSkin, true),
                    (internal, PathKind::Solid, settings.monotonic_solid),
                    (skin_top, PathKind::TopSkin, true),
                ];
                // A perimeter loop following each region's boundary (so where
                // it runs alongside the shell it becomes a clean concentric bead),
                // then straight-fill only the interior left inside that loop. Thin
                // solid bands are consumed entirely by the loop — no lone strands.
                // The loop and the fill both push `ov` into their neighbor so
                // solid surfaces bond to the walls.
                for (region, kind, _) in &regions {
                    // No perimeter loop on top/bottom surfaces — the fill pattern is
                    // extended out to cover that area instead (a clean monolithic
                    // skin, no concentric ring). Buried Solid keeps its loop.
                    if matches!(kind, PathKind::TopSkin | PathKind::BottomSkin) {
                        continue;
                    }
                    let region_loop = offset(region, -(lw * 0.5 - ov * 0.5));
                    for c in region_loop.contours {
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
                        paths.push(ToolPath::new(*kind, true, lw, points));
                    }
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
                for (region, kind, monotone) in regions {
                    if region.is_empty() {
                        continue;
                    }
                    // Top/bottom surfaces have no perimeter loop, so extend the fill
                    // out to where that loop would have sat (half a bead in) — the
                    // pattern covers the whole face. Buried Solid insets past its loop.
                    let fill_inset = if matches!(kind, PathKind::TopSkin | PathKind::BottomSkin) {
                        lw * 0.5 - ov * 0.5
                    } else {
                        0.5 * (lw + sp) - 0.5 * ov
                    };
                    let core = offset(&region, -fill_inset);
                    let fill = if internal_bridge.is_empty() {
                        core
                    } else {
                        difference(&core, &internal_bridge)
                    };
                    if !fill.is_empty() {
                        let pattern = match kind {
                            PathKind::TopSkin => settings.top_pattern,
                            PathKind::BottomSkin => settings.bottom_pattern,
                            _ => settings.solid_pattern,
                        };
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
                        fill_region(
                            &fill, pattern, sp, pat_angle(pattern), lw, kind,
                            settings.seam_mode, i, layers[i].z_mm, monotone, &mut paths,
                        );
                    }
                }
            }
            if settings.infill_density > 0.0 && !sparse.is_empty() {
                let spacing = sp / settings.infill_density;
                let sparse_fill = if ov > 0.0 { offset(&sparse, ov) } else { sparse.clone() };
                fill_region(
                    &sparse_fill, settings.sparse_pattern, spacing, pat_angle(settings.sparse_pattern), lw, PathKind::Infill,
                    settings.seam_mode, i, layers[i].z_mm, false, &mut paths,
                );
            }
        }

        // Gap fill (raster oracle): stamp every dense bead actually laid onto a
        // fine grid and trace the medial axis of whatever the part interior —
        // minus the sparse-infill region (its between-line gaps are intentional) —
        // is left uncovered. A raster sees the medial-seam voids that an offset
        // coverage test merges across. Runs before ironing so a gap stroke prints
        // with the structure, not over the finished surface.
        if settings.gap_fill {
            let dense = difference(&layers[i].polygons, &sparse_region);
            if !dense.is_empty() {
                let beads: Vec<(Vec<Point>, Vec<f64>)> = paths
                    .iter()
                    .filter(|p| p.kind != PathKind::Infill && p.points.len() >= 2)
                    .map(|p| {
                        let mut pts = p.points.clone();
                        if p.closed {
                            pts.push(pts[0]); // stamp the closing segment too
                        }
                        let ws = vec![p.width_mm; pts.len()];
                        (pts, ws)
                    })
                    .collect();
                let raw_voids = crate::coverage::uncovered(&dense, &beads, lw, lw * lw * 1.5);
                // Light simplify (~one raster cell): knocks the single-cell stair
                // jaggies off the marching-squares boundary without shrinking the
                // void, so the medial stays DENSE (its width tracks the true
                // channel; a heavier simplify makes it sparse and the clearance
                // reads low at the few surviving vertices → a too-thin bead). Do
                // NOT morphologically close (that truncates the tip).
                let voids = simplify(&raw_voids, lw * 0.15);
                emit_gap_fill(&voids, lw, sp, &mut paths);
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
            heat_scale: 1.0,
            planned_heat_target: None,
            planned_temp_c: None,
            temp_command_c: None,
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
    order_unsupported_rings_outer_in(&mut plans, &layers, settings);
    center_on_bed(&mut plans, mesh, settings);
    if matches!(settings.seam_mode, SeamMode::Aligned | SeamMode::Sharpest) {
        align_seams(&mut plans, settings.seam_mode);
    }
    crate::emit::plan_travels(&mut plans, settings);
    crate::emit::apply_min_layer_time(&mut plans, settings);
    // Pin per-layer heat targets on the raw plan — both heat-control levers
    // change the very quantities the targets are computed from, so everything
    // downstream must chase the same frozen numbers.
    let heat_targets = crate::emit::plan_heat_targets(&plans, settings);
    for (plan, t) in plans.iter_mut().zip(heat_targets) {
        plan.planned_heat_target = Some(t);
    }
    // Heat control governs through one lever now: it paces each layer's speed
    // (down to the min print speed) to hold the layer's whole-layer heat load
    // under the target. No dwell, no per-island split. The nozzle holds its
    // derived temperature throughout — there is no temperature lever.
    crate::emit::apply_heat_control_speed(&mut plans, settings);
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
                add_support_region(&mut plans[i].paths, &body_here, spacing, angle, lw,
                    settings.seam_mode, i, layers[i].z_mm);
            }
            if !iface_here.is_empty() {
                add_support_region(&mut plans[i].paths, &iface_here, sp, angle, lw,
                    settings.seam_mode, i, layers[i].z_mm);
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
            for j in 0..=count {
                let idx = (first + j) % n;
                points.push(path.points[idx]);
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
                widths: path.widths.clone(),
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

/// Classic gap fill: trace the medial axis of each sliver the walls + infill
/// leave uncovered and lay it as single width-matched strokes.
///
/// `coverage` is the union of the actual wall-bead footprints (so a ring the
/// offset abandoned on a thin flank shows up as missing coverage); `opened` is
/// the infill region. The uncovered remainder is banded to widths a stroke can
/// fill (≥ `0.2·lw`) but a full ring can't (≤ `2·sp`), mirroring PrusaSlicer's
/// gap-region cleanup, then handed to the segment-Voronoi medial axis.
fn emit_gap_fill(raw: &Polygons, lw: f64, sp: f64, out: &mut Vec<ToolPath>) {
    if raw.is_empty() {
        return;
    }
    let min = 0.2 * lw;
    let max = 2.0 * sp;
    // Drop sub-min hair ribbons (open by min/2) and subtract the >max cores that
    // should have taken a real ring (the open-by-max/2 part): the fillable band.
    let opened_min = offset(&offset(&raw, -min * 0.5), min * 0.5);
    let wide = offset(&offset(&raw, -max * 0.5), max * 0.5);
    let gaps = difference(&opened_min, &wide);
    if gaps.is_empty() {
        return;
    }
    // Now that the dense walls fill the bulk, gap fill only sees the residual
    // seam specks where rings don't quite meet — a short isolated bead per speck
    // strings out into a dashed line that is mostly travel for negligible fill.
    // Skip beads too short to be worth reaching (5·lw, above the medial's 2·max
    // twig-cull; on Benchy these specks are all < 2.25mm with a clean gap to the
    // next real fill at > 5mm).
    for tp in crate::medial::medial_axis(&gaps, min, max) {
        if tp.length_mm() < lw * 5.0 {
            continue;
        }
        emit_tapered_bead(&tp, lw, out);
    }
}

/// Emit a medial polyline as one continuously-tapered `GapFill` bead. The
/// centerline is resampled to fine segments and carries a per-point width
/// (`ToolPath.widths`), so the g-code varies E per segment — a smooth taper to
/// the tip — and the preview renders a variable-width ribbon. Widths are floored
/// at the minimum printable bead; `width_mm` carries the mean for feed/flow.
fn emit_tapered_bead(tp: &crate::medial::ThickPolyline, lw: f64, out: &mut Vec<ToolPath>) {
    const FLOOR: f64 = 0.1; // min printable bead width
    // The medial centerline can be sparse — a wedge void traces as just two
    // points (wide base, thin tip). Resample so the per-segment width actually
    // varies along it instead of collapsing to one mean.
    let (pts, ws) = resample_thick(&tp.points, &tp.widths, lw * 0.5);
    if pts.len() < 2 {
        return;
    }
    // The medial clearance to the raster void's jagged (~0.18mm) boundary
    // oscillates, which reads as a lumpy bead pinching below the real channel
    // width. Low-pass the width profile (~2mm window) so the bead fills the
    // channel smoothly; the centerline itself is already smooth.
    let ws = smooth_widths(&ws, lw * 0.5, 2.0);
    let widths: Vec<f64> = ws.iter().map(|&w| w.max(FLOOR)).collect();
    let mean = widths.iter().sum::<f64>() / widths.len() as f64;
    let mut p = ToolPath::new(PathKind::GapFill, false, mean, pts);
    p.widths = Some(widths);
    out.push(p);
}

/// Moving-average the width profile over a ~`window_mm` window (samples spaced
/// `step_mm`) to strip the high-frequency oscillation the raster void's jagged
/// boundary injects into the medial clearance, while keeping the real taper.
fn smooth_widths(ws: &[f64], step_mm: f64, window_mm: f64) -> Vec<f64> {
    let r = ((window_mm / step_mm / 2.0).round() as usize).max(1);
    if ws.len() <= 2 {
        return ws.to_vec();
    }
    (0..ws.len())
        .map(|i| {
            let lo = i.saturating_sub(r);
            let hi = (i + r + 1).min(ws.len());
            ws[lo..hi].iter().sum::<f64>() / (hi - lo) as f64
        })
        .collect()
}

/// Resample a polyline + per-point widths at ~`step_mm` spacing, interpolating
/// both position and width — turns a sparse medial centerline into enough points
/// that its width taper survives the run-split.
fn resample_thick(pts: &[Point], ws: &[f64], step_mm: f64) -> (Vec<Point>, Vec<f64>) {
    let n = pts.len();
    if n < 2 {
        return (pts.to_vec(), ws.to_vec());
    }
    let step = (step_mm * geo2d::UNITS_PER_MM).max(1.0);
    let mut op = vec![pts[0]];
    let mut ow = vec![ws[0]];
    let mut acc = 0.0; // distance walked since the last emitted point
    for i in 0..n - 1 {
        let (a, b) = (pts[i], pts[i + 1]);
        let seg = ((b.x - a.x) as f64).hypot((b.y - a.y) as f64);
        if seg < 1.0 {
            continue;
        }
        let mut t0 = 0.0; // fraction of this segment already consumed
        while acc + seg * (1.0 - t0) >= step {
            let t = t0 + (step - acc) / seg;
            op.push(Point::new(
                a.x + ((b.x - a.x) as f64 * t).round() as i64,
                a.y + ((b.y - a.y) as f64 * t).round() as i64,
            ));
            ow.push(ws[i] + (ws[i + 1] - ws[i]) * t);
            t0 = t;
            acc = 0.0;
        }
        acc += seg * (1.0 - t0);
    }
    op.push(pts[n - 1]);
    ow.push(ws[n - 1]);
    (op, ow)
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

/// Concentric rings spanning an *enclosed* unsupported void must print supported-
/// edge inward, so each ring lands on the previously-laid outer ring. Greedy travel
/// ordering can lay them inner-first, which drops the first ring into the void.
/// This covers the roof-as-walls case (overhang wall rings, no bottom shell) and a
/// concentric bottom shell over air. A *cantilever* is the opposite case and is
/// left alone: its outermost ring runs over its own free edge, so outer→in would
/// lay that unsupported span first — there's nothing to print supported-edge-first
/// from. The test is geometric: reorder a run only when its outermost (largest)
/// ring sits fully on the layer below and its innermost is over air.
fn order_unsupported_rings_outer_in(plans: &mut [LayerPlan], layers: &[Layer], settings: &Settings) {
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
    // A ring sits fully on support when every segment midpoint is over the (grown)
    // layer below.
    fn supported(p: &ToolPath, below: &Polygons) -> bool {
        let pts = &p.points;
        let n = pts.len();
        if n < 2 {
            return true;
        }
        let segs = if p.closed { n } else { n - 1 };
        (0..segs).all(|k| {
            let (a, b) = (pts[k], pts[(k + 1) % n]);
            in_polys(below, Point::new((a.x + b.x) / 2, (a.y + b.y) / 2))
        })
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
        let below = offset(&layers[plan.index - 1].polygons, allowance);
        let paths = &mut plan.paths;
        let part_of_run = |p: &ToolPath| is_ring(p.kind) || p.kind == PathKind::GapFill;
        let mut i = 0;
        while i < paths.len() {
            if !is_ring(paths[i].kind) {
                i += 1;
                continue;
            }
            // A run is the maximal stretch of ring segments plus the gap-fill
            // strokes travel-ordering tucks among them. Ring segments may be
            // OPEN: a void cap's supported outer walls are split into open
            // overhang arcs by slow_overhanging_walls, while the loops over the
            // void stay closed — both must count, or the run looks all-unsupported
            // and the cap keeps its (travel-order) inner→out sequence.
            let mut j = i;
            while j < paths.len() && part_of_run(&paths[j]) {
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
        InfillPattern::Lines | InfillPattern::AlignedLines => scan(&[(angle, spacing)], out),
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
        // Arc mode bridges the anchored gap too (it never falls through to arc-fill).
        let arc = generate(
            &m,
            &Settings { skirt_loops: 0, support_mode: SupportMode::Arc, ..Settings::default() },
        );
        let slab_arc = arc.iter().find(|p| p.print_z_mm > 4.05).unwrap();
        assert!(count(slab_arc, PathKind::Bridge) > 0, "arc mode bridges the anchored gap too");
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
        // The pocket is too small for meaningful 15% sparse fill, so it's
        // promoted to solid instead.
        let s = Settings { skirt_loops: 0, ..Settings::default() };
        let layers = generate(&m, &s);
        let mid = &layers[10];
        assert_eq!(count(mid, PathKind::Infill), 0, "no sparse fill in a tiny pocket");
        assert!(count(mid, PathKind::Solid) > 0, "pocket promoted to solid");
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
}
