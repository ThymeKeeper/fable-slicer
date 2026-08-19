//! Turning a parsed g-code timeline back into layer plans.
//!
//! The Machine view mirrors whatever the printer is running, which arrives as
//! a [`gcode::Timeline`] — moves and layer boundaries, not beads. Rather than
//! teach the renderer a second way to draw, the timeline is rebuilt into the
//! [`LayerPlan`]s the preview already knows: the bead geometry, the layer
//! slider, the feature palette and the category masks all come along for
//! free, and a file from another slicer draws exactly like one of ours.
//!
//! What is reconstructed is honest but approximate. Bead WIDTH comes from the
//! filament each move consumed, back-solved through the layer height — the
//! same arithmetic the emitter did forwards, so a file we wrote round-trips
//! to the widths we chose, and a file we didn't gets whatever its own flow
//! implies. Bead KIND comes from the file's `;TYPE:` comments; a file without
//! them draws as one feature, which is the truth about what it told us.

use crate::plan::{LayerPlan, PathKind, ToolPath, Travel};
use gcode::{Feature, Timeline};
use geo2d::{Point, Polygons};

/// Rebuild a parsed timeline into layer plans for the preview renderer.
///
/// `filament_dia_mm` only scales width; 1.75 is right for every machine this
/// talks to and a wrong guess shows up as uniformly fat or thin beads, never
/// as wrong geometry.
pub fn plans_from_timeline(tl: &Timeline, filament_dia_mm: f64) -> Vec<LayerPlan> {
    let fil_area = std::f64::consts::PI * (filament_dia_mm / 2.0).powi(2);
    let mut plans: Vec<LayerPlan> = Vec::with_capacity(tl.layers.len());
    for (li, layer) in tl.layers.iter().enumerate() {
        let first = layer.first_move as usize;
        let last = tl
            .layers
            .get(li + 1)
            .map(|n| n.first_move as usize)
            .unwrap_or(tl.moves.len());
        // Height from the gap to the layer below — the same thing the slicer
        // that wrote the file was thinking, recovered without asking it.
        let height = match li {
            0 => layer.z as f64,
            _ => (layer.z - tl.layers[li - 1].z) as f64,
        }
        .clamp(0.01, 1.0);

        let mut paths: Vec<ToolPath> = Vec::new();
        let mut run: Vec<Point> = Vec::new();
        let mut run_e = 0.0f64;
        let mut run_len = 0.0f64;
        let mut run_feature = Feature::Other;
        // The point a run starts from is the end of the move before it —
        // extrusion begins where the last motion left the nozzle.
        let mut prev = if first == 0 { tl.start } else { tl.moves[first - 1].to };

        let mut flush = |run: &mut Vec<Point>, e: &mut f64, len: &mut f64, f: Feature| {
            if run.len() >= 2 {
                // width = volume per mm of travel / layer height. A degenerate
                // run (no length, or none of the E landed here) falls back to
                // something plausible rather than a zero-width ribbon.
                let w = if *len > 1.0e-6 && *e > 0.0 {
                    (*e * fil_area / *len / height).clamp(0.05, 5.0)
                } else {
                    0.4
                };
                paths.push(ToolPath {
                    kind: kind_of(f),
                    closed: false,
                    width_mm: w,
                    points: std::mem::take(run),
                    flow: 1.0,
                    group: None,
                    height_scale: 1.0,
                    widths: None,
                    overhang: 0.0,
                    segs: None,
                    tool: 0,
                    joined: false,
                });
            } else {
                run.clear();
            }
            *e = 0.0;
            *len = 0.0;
        };

        for m in &tl.moves[first..last] {
            let to = Point::from_mm(m.to[0] as f64, m.to[1] as f64);
            if m.extruding {
                if run.is_empty() {
                    run.push(Point::from_mm(prev[0] as f64, prev[1] as f64));
                    run_feature = m.feature;
                } else if m.feature != run_feature {
                    // A feature boundary ends the bead: the renderer colors
                    // per path, and a wall that turns into infill mid-ribbon
                    // would paint the wrong thing.
                    let from = *run.last().unwrap();
                    flush(&mut run, &mut run_e, &mut run_len, run_feature);
                    run.push(from);
                    run_feature = m.feature;
                }
                let from = *run.last().unwrap();
                run_len += dist_mm(from, to);
                run_e += m.e_mm as f64;
                run.push(to);
            } else if !run.is_empty() {
                flush(&mut run, &mut run_e, &mut run_len, run_feature);
            }
            prev = m.to;
        }
        flush(&mut run, &mut run_e, &mut run_len, run_feature);

        let travels = vec![Travel::default(); paths.len()];
        plans.push(LayerPlan {
            index: li,
            print_z_mm: layer.z as f64,
            height_mm: height,
            paths,
            travels,
            // No outline: nothing downstream of the renderer runs on these —
            // they are never emitted, combed, or re-ordered.
            outline: Polygons::new(),
            speed_scale: 1.0,
            fan_boost: 0.0,
            planned_temp_c: None,
            temp_command_c: None,
        });
    }
    plans
}

/// The path kind a file's feature label corresponds to. The preview colors by
/// kind, so this is what makes a mirrored job read like a sliced one.
fn kind_of(f: Feature) -> PathKind {
    match f {
        Feature::OuterWall => PathKind::ExternalPerimeter,
        Feature::InnerWall => PathKind::Perimeter,
        Feature::Overhang => PathKind::OverhangWall,
        Feature::Infill => PathKind::Infill,
        Feature::Solid => PathKind::Solid,
        Feature::Top => PathKind::TopSkin,
        Feature::Bottom => PathKind::BottomSkin,
        Feature::Bridge => PathKind::Bridge,
        Feature::GapFill => PathKind::GapFill,
        Feature::Support => PathKind::Support,
        Feature::Skirt => PathKind::Skirt,
        // A file that never said what it was printing draws as plain wall
        // rather than as nothing.
        Feature::Other => PathKind::Perimeter,
    }
}

fn dist_mm(a: Point, b: Point) -> f64 {
    let (dx, dy) = (b.x_mm() - a.x_mm(), b.y_mm() - a.y_mm());
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_round_trip_recovers_the_geometry_and_the_widths() {
        // Two layers, a 0.4 mm bead at 0.2 mm height: E per mm of travel is
        // width * height / filament area = 0.4*0.2/2.405 = 0.03326 mm/mm, so
        // 10 mm of bead is 0.3326 mm of filament.
        let g = "\
G21
G90
M83
;TYPE:Outer wall
G1 X0 Y0 Z0.2 F1800
G1 X10 Y0 E0.3326
G1 Z0.4
;TYPE:Sparse infill
G1 X10 Y10 E0.3326
";
        let tl = Timeline::parse(g.as_bytes());
        let plans = plans_from_timeline(&tl, 1.75);
        assert_eq!(plans.len(), 2, "two layers");
        assert_eq!(plans[0].paths.len(), 1);
        assert_eq!(plans[0].paths[0].kind, PathKind::ExternalPerimeter);
        assert_eq!(plans[1].paths[0].kind, PathKind::Infill, "the TYPE comment carries");
        // The width the file implies, back-solved.
        let w = plans[0].paths[0].width_mm;
        assert!((w - 0.4).abs() < 0.02, "width {w}");
        assert!((plans[1].height_mm - 0.2).abs() < 1.0e-6, "height from the layer gap: {}", plans[1].height_mm);
        // Geometry: the bead runs from where the nozzle was to where it went.
        let p = &plans[0].paths[0].points;
        assert_eq!(p.len(), 2);
        assert!((p[0].x_mm()).abs() < 1e-6 && (p[1].x_mm() - 10.0).abs() < 1e-6);
    }

    #[test]
    fn travels_break_beads_and_features_split_them() {
        let g = "\
G90
M83
;TYPE:Inner wall
G1 X0 Y0 Z0.2 F1800
G1 X10 Y0 E0.33
G1 X20 Y0 F9000
;TYPE:Top surface
G1 X30 Y0 E0.33 F1800
";
        let tl = Timeline::parse(g.as_bytes());
        let plans = plans_from_timeline(&tl, 1.75);
        assert_eq!(plans.len(), 1);
        assert_eq!(plans[0].paths.len(), 2, "a travel ends the bead");
        assert_eq!(plans[0].paths[0].kind, PathKind::Perimeter);
        assert_eq!(plans[0].paths[1].kind, PathKind::TopSkin);
        // The second bead starts where the travel left the nozzle, not where
        // the first bead ended.
        assert!((plans[0].paths[1].points[0].x_mm() - 20.0).abs() < 1e-6);
    }
}
