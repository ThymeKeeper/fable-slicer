//! Toolpath planning: turn each layer's polygons into ordered extrusion paths.
//!
//! M1 scope: `wall_count` concentric perimeters (inward offsets) plus uniform
//! sparse line infill clipped to the region inside the innermost wall. Top/bottom
//! solid surfaces and retraction come at M2.

use config::Settings;
use geo2d::{offset, Point, Polygons};
use mesh::Mesh;

use crate::{slice_mesh, SliceParams};

/// What a toolpath represents — drives speed and (later) ordering choices.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PathKind {
    ExternalPerimeter,
    Perimeter,
    Infill,
}

/// A single continuous extrusion path.
#[derive(Clone, Debug)]
pub struct ToolPath {
    pub kind: PathKind,
    /// Closed loops (walls) extrude back to the start point; open paths (infill)
    /// stop at the last point.
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

/// Slice and plan a whole model into per-layer toolpaths.
pub fn generate(mesh: &Mesh, settings: &Settings) -> Vec<LayerPlan> {
    let layers = slice_mesh(mesh, SliceParams { layer_height_mm: settings.layer_height_mm });
    let lw = settings.line_width_mm;

    let mut plans = Vec::with_capacity(layers.len());
    for layer in &layers {
        let mut paths = Vec::new();

        // Walls: outermost first, each offset another line-width inward. The
        // outer wall centerline sits half a line-width in from the model surface.
        for w in 0..settings.wall_count {
            let inset = -lw * (0.5 + w as f64);
            let wall = offset(&layer.polygons, inset);
            let kind = if w == 0 {
                PathKind::ExternalPerimeter
            } else {
                PathKind::Perimeter
            };
            for c in wall.contours {
                if c.points.len() >= 3 {
                    paths.push(ToolPath { kind, closed: true, width_mm: lw, points: c.points });
                }
            }
        }

        // Sparse infill inside the innermost wall.
        if settings.infill_density > 0.0 {
            let region = offset(&layer.polygons, -lw * settings.wall_count as f64);
            if !region.is_empty() {
                // Alternate direction per layer for a cross-hatch.
                let angle = if layer.index % 2 == 0 { 45.0 } else { 135.0 };
                let spacing = lw / settings.infill_density;
                for seg in infill_lines(&region, angle, spacing) {
                    paths.push(ToolPath {
                        kind: PathKind::Infill,
                        closed: false,
                        width_mm: lw,
                        points: seg,
                    });
                }
            }
        }

        plans.push(LayerPlan {
            index: layer.index,
            print_z_mm: settings.layer_height_mm * (layer.index as f64 + 1.0),
            height_mm: settings.layer_height_mm,
            paths,
        });
    }
    plans
}

/// Generate straight infill lines at `angle_deg`, spaced `spacing_mm` apart,
/// clipped to `region` via an even-odd scanline. Works in millimeters; the
/// region is rotated so infill lines become horizontal scanlines, then results
/// are rotated back.
fn infill_lines(region: &Polygons, angle_deg: f64, spacing_mm: f64) -> Vec<Vec<Point>> {
    let theta = angle_deg.to_radians();
    let (ct, st) = (theta.cos(), theta.sin());
    let rot = |x: f64, y: f64| (x * ct + y * st, -x * st + y * ct);
    let unrot = |x: f64, y: f64| (x * ct - y * st, x * st + y * ct);

    // Rotated edges + y-range.
    let mut edges: Vec<(f64, f64, f64, f64)> = Vec::new();
    let (mut ymin, mut ymax) = (f64::INFINITY, f64::NEG_INFINITY);
    for c in &region.contours {
        let n = c.points.len();
        if n < 3 {
            continue;
        }
        for i in 0..n {
            let a = c.points[i];
            let b = c.points[(i + 1) % n];
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

        let mut i = 0;
        while i + 1 < xs.len() {
            let (x0, x1) = (xs[i], xs[i + 1]);
            if x1 - x0 > 0.1 {
                let (px0, py0) = unrot(x0, y);
                let (px1, py1) = unrot(x1, y);
                out.push(vec![Point::from_mm(px0, py0), Point::from_mm(px1, py1)]);
            }
            i += 2;
        }
        y += spacing_mm;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo2d::Contour;

    #[test]
    fn cube_plan_has_walls_and_infill() {
        let m = Mesh::cube(20.0);
        let s = Settings::default();
        let layers = generate(&m, &s);
        assert_eq!(layers.len(), 100);

        let l = &layers[10];
        let walls = l.paths.iter().filter(|p| p.kind != PathKind::Infill).count();
        let infill = l.paths.iter().filter(|p| p.kind == PathKind::Infill).count();
        assert_eq!(walls, s.wall_count, "two concentric wall loops");
        assert!(infill > 0, "expected infill lines");

        // Outer wall is offset inward: 20mm - 2*(0.5*0.45) = 19.55mm => ~382mm².
        // This also proves the offset sign (inward, not outward).
        let ext = l
            .paths
            .iter()
            .find(|p| p.kind == PathKind::ExternalPerimeter)
            .unwrap();
        let area = Contour::new(ext.points.clone()).area_mm2();
        assert!(area > 360.0 && area < 400.0, "outer wall area {area}");
    }
}
