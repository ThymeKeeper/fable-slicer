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

use config::{SeamMode, Settings};
use geo2d::{difference, intersection, offset, union, to_units, Point, Polygons};
use mesh::Mesh;

use crate::{slice_mesh, SliceParams};

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
}

/// A single continuous extrusion path.
#[derive(Clone, Debug)]
pub struct ToolPath {
    pub kind: PathKind,
    /// Closed loops (walls) extrude back to the start; open paths (infill) stop.
    pub closed: bool,
    pub width_mm: f64,
    pub points: Vec<Point>,
}

/// Everything needed to emit one printed layer.
#[derive(Clone, Debug)]
pub struct LayerPlan {
    pub index: usize,
    /// Nozzle Z when printing this layer (top of the layer).
    pub print_z_mm: f64,
    pub height_mm: f64,
    pub paths: Vec<ToolPath>,
}

/// Slice and plan a whole model into per-layer toolpaths, centered on the bed.
pub fn generate(mesh: &Mesh, settings: &Settings) -> Vec<LayerPlan> {
    let layers = slice_mesh(
        mesh,
        SliceParams {
            layer_height_mm: settings.layer_height_mm,
            first_layer_height_mm: settings.first_layer_height_mm,
        },
    );
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
            for c in offset(&layer.polygons, inset).contours {
                if c.points.len() >= 3 {
                    let points = place_seam(c.points, settings.seam_mode, layer.index);
                    walls.push(ToolPath { kind, closed: true, width_mm: lw, points });
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
            let solid = union(&solid_top, &solid_bottom);
            let sparse = difference(inner, &solid);

            // Alternate fill direction per layer for cross-hatching.
            let angle = if i % 2 == 0 { 45.0 } else { 135.0 };

            if !solid.is_empty() {
                // A perimeter loop following the solid region's boundary (so where
                // it runs alongside the shell it becomes a clean concentric bead),
                // then straight-fill only the interior left inside that loop. Thin
                // solid bands are consumed entirely by the loop — no lone strands.
                let solid_loop = offset(&solid, -lw * 0.5);
                for c in solid_loop.contours {
                    if c.points.len() >= 3 {
                        let points = place_seam(c.points, settings.seam_mode, i);
                        paths.push(ToolPath { kind: PathKind::Solid, closed: true, width_mm: lw, points });
                    }
                }
                let solid_core = offset(&solid, -lw);
                if !solid_core.is_empty() {
                    // The boundary loop already handles thin/edge solid, so keep all
                    // interior lines (no isolated-line dropping needed here).
                    for seg in infill_lines(&solid_core, angle, lw, false) {
                        paths.push(ToolPath { kind: PathKind::Solid, closed: false, width_mm: lw, points: seg });
                    }
                }
            }
            if settings.infill_density > 0.0 && !sparse.is_empty() {
                let spacing = lw / settings.infill_density;
                for seg in infill_lines(&sparse, angle, spacing, false) {
                    paths.push(ToolPath { kind: PathKind::Infill, closed: false, width_mm: lw, points: seg });
                }
            }
        }

        plans.push(LayerPlan {
            index: i,
            print_z_mm: layers[i].print_z_mm,
            height_mm: layers[i].height_mm,
            paths,
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

    center_on_bed(&mut plans, mesh, settings);
    plans
}

/// Loops around the first-layer outline, offset outward, to prime the nozzle and
/// establish flow before the part starts.
fn skirt_paths(first_layer: &Polygons, settings: &Settings) -> Vec<ToolPath> {
    let lw = settings.line_width_mm;
    let mut paths = Vec::new();
    for k in 0..settings.skirt_loops {
        let delta = settings.skirt_gap_mm + lw * (0.5 + k as f64);
        for c in offset(first_layer, delta).contours {
            if c.points.len() >= 3 {
                paths.push(ToolPath { kind: PathKind::Skirt, closed: true, width_mm: lw, points: c.points });
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
            if c.points.len() >= 3 {
                paths.push(ToolPath { kind: PathKind::Skirt, closed: true, width_mm: lw, points: c.points });
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
    }
}

/// Generate straight infill lines at `angle_deg`, spaced `spacing_mm` apart,
/// clipped to `region` via an even-odd scanline. The region is rotated so infill
/// lines become horizontal scanlines, then results are rotated back.
fn infill_lines(region: &Polygons, angle_deg: f64, spacing_mm: f64, drop_isolated: bool) -> Vec<Vec<Point>> {
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

    // Collect the inside-intervals of each scanline (in the rotated frame).
    let mut rows: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
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

        let mut row = Vec::new();
        let mut k = 0;
        while k + 1 < xs.len() {
            let (x0, x1) = (xs[k], xs[k + 1]);
            // Skip dabs shorter than this — not worth extruding.
            if x1 - x0 > 0.5 {
                row.push((x0, x1));
            }
            k += 2;
        }
        rows.push(row);
        ys.push(y);
        y += spacing_mm;
    }

    // Two intervals "sit beside each other" if their x-ranges overlap.
    let overlaps = |s: (f64, f64), others: &[(f64, f64)]| {
        others.iter().any(|&(a0, a1)| a0 < s.1 && s.0 < a1)
    };

    let mut out = Vec::new();
    for k in 0..rows.len() {
        for &(x0, x1) in &rows[k] {
            // For solid fill, drop a line with no neighbouring line on either
            // adjacent scanline — a lone strand that serves no purpose.
            if drop_isolated {
                let above = k + 1 < rows.len() && overlaps((x0, x1), &rows[k + 1]);
                let below = k > 0 && overlaps((x0, x1), &rows[k - 1]);
                if !above && !below {
                    continue;
                }
            }
            let yk = ys[k];
            let (px0, py0) = unrot(x0, yk);
            let (px1, py1) = unrot(x1, yk);
            out.push(vec![Point::from_mm(px0, py0), Point::from_mm(px1, py1)]);
        }
    }
    out
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
