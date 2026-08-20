//! Stage-2 A/B scorer (scratch tool): score a set of sliced layers with
//! master's judges — residue coverage (interior + exposed-top band), wall
//! width wobble, path/seam counts, and sub-printable extrusion length.
//!
//! Two modes so master and the beader branch face identical judges:
//!   stage2_score self <model>   — slice on THIS engine at the user's profiles
//!                                 (honors STAGE1_DENSITY) and score in-process
//!   stage2_score file <dump>    — score a `stage2_dump` file from the branch
//!
//! Wobble definition: per wall path (ExternalPerimeter/Perimeter/OverhangWall),
//! span = max−min of its per-vertex widths (0 for constant-width paths); we
//! report how many wall paths span >0.15 mm and the p90 span — the a42f508
//! metric applied at path granularity.

use anyhow::{anyhow, Context, Result};
use config::Profiles;
use engine::{LayerPlan, PathKind, ToolPath, Travel};
use geo2d::{Point, Polygons};

struct SimplePath {
    kind: String,
    closed: bool,
    width_mm: f64,
    pts: Vec<(f64, f64)>,
    widths: Option<Vec<f64>>,
}

struct SimpleLayer {
    index: usize,
    outline: Polygons,
    paths: Vec<SimplePath>,
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mode = args.get(1).map(String::as_str).context("usage: stage2_score self|file <arg>")?;
    let arg = args.get(2).context("usage: stage2_score self|file <arg>")?;

    let layers = match mode {
        "self" => slice_self(arg)?,
        "file" => parse_dump(arg)?,
        _ => anyhow::bail!("mode must be self|file"),
    };
    score(&layers);
    Ok(())
}

fn slice_self(model: &str) -> Result<Vec<SimpleLayer>> {
    let mut profiles = Profiles::builtin();
    profiles.load_user_profiles(None).map_err(|e| anyhow!(e))?;
    let mut settings = profiles
        .resolve_tools("sovol-zero-custom", &["petg-custom"], "sovol-zero-custom")
        .map_err(|e| anyhow!(e))?;
    if let Some(d) = std::env::var("STAGE1_DENSITY").ok().and_then(|v| v.parse().ok()) {
        settings.infill_density = d;
    }
    let path = std::path::Path::new(model);
    let is_3mf = path.extension().map(|e| e.eq_ignore_ascii_case("3mf")).unwrap_or(false);
    let mesh = if is_3mf {
        let items = mesh::load_3mf(path).map_err(|e| anyhow!(e))?;
        let mut m = mesh::Mesh::default();
        for p in items.iter().flat_map(|it| &it.parts) {
            m.append(&p.mesh);
        }
        m
    } else {
        mesh::Mesh::load_stl(path)?
    };
    let parts = [(&mesh, 0u32)];
    let t0 = std::time::Instant::now();
    let plans = engine::generate_parts(&parts, &settings);
    eprintln!("sliced {} layers in {} ms", plans.len(), t0.elapsed().as_millis());
    Ok(plans
        .iter()
        .map(|l| SimpleLayer {
            index: l.index,
            outline: l.outline.clone(),
            paths: l
                .paths
                .iter()
                .map(|p| SimplePath {
                    kind: format!("{:?}", p.kind),
                    closed: p.closed,
                    width_mm: p.width_mm,
                    pts: p.points.iter().map(|q| (q.x_mm(), q.y_mm())).collect(),
                    widths: p.widths.clone(),
                })
                .collect(),
        })
        .collect())
}

fn parse_dump(path: &str) -> Result<Vec<SimpleLayer>> {
    let text = std::fs::read_to_string(path)?;
    let mut layers: Vec<SimpleLayer> = Vec::new();
    let mut lines = text.lines().peekable();
    while let Some(line) = lines.next() {
        if let Some(rest) = line.strip_prefix("L ") {
            let idx: usize = rest.split_whitespace().next().unwrap().parse()?;
            layers.push(SimpleLayer { index: idx, outline: Polygons::new(), paths: Vec::new() });
        } else if line.starts_with("C ") {
            let l = layers.last_mut().context("C before L")?;
            let nums: Vec<f64> = line[2..].split_whitespace().map(|t| t.parse().unwrap()).collect();
            let pts: Vec<Point> = nums.chunks(2).map(|c| Point::from_mm(c[0], c[1])).collect();
            l.outline.push(geo2d::Contour::new(pts));
        } else if let Some(rest) = line.strip_prefix("P ") {
            let f: Vec<&str> = rest.split_whitespace().collect();
            let (kind, closed, width, has_w) =
                (f[0].to_string(), f[1] == "1", f[2].parse::<f64>()?, f[3] == "1");
            let vline = lines.next().context("missing V")?;
            let nums: Vec<f64> = vline[1..].split_whitespace().map(|t| t.parse().unwrap()).collect();
            let pts: Vec<(f64, f64)> = nums.chunks(2).map(|c| (c[0], c[1])).collect();
            let widths = if has_w {
                let wline = lines.next().context("missing W")?;
                Some(wline[1..].split_whitespace().map(|t| t.parse().unwrap()).collect())
            } else {
                None
            };
            layers
                .last_mut()
                .context("P before L")?
                .paths
                .push(SimplePath { kind, closed, width_mm: width, pts, widths });
        }
    }
    Ok(layers)
}

fn to_plan(l: &SimpleLayer) -> LayerPlan {
    let paths: Vec<ToolPath> = l
        .paths
        .iter()
        .map(|p| ToolPath {
            kind: match p.kind.as_str() {
                "ExternalPerimeter" => PathKind::ExternalPerimeter,
                "Skirt" => PathKind::Skirt,
                _ => PathKind::Perimeter,
            },
            closed: p.closed,
            width_mm: p.width_mm,
            points: p.pts.iter().map(|&(x, y)| Point::from_mm(x, y)).collect(),
            flow: 1.0,
            group: None,
            height_scale: 1.0,
            widths: p.widths.clone(),
            overhang: 0.0,
            segs: None,
            tool: 0,
            joined: false,
        })
        .collect();
    let travels = vec![Travel::default(); paths.len()];
    LayerPlan {
        index: l.index,
        print_z_mm: 0.0,
        height_mm: 0.2,
        paths,
        travels,
        outline: l.outline.clone(),
        speed_scale: 1.0,
        fan_boost: 0.0,
        planned_temp_c: None,
        temp_command_c: None,
    }
}

fn path_len(pts: &[(f64, f64)], closed: bool) -> f64 {
    let mut len = 0.0;
    for w in pts.windows(2) {
        len += ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
    }
    if closed && pts.len() >= 2 {
        let (a, b) = (pts[pts.len() - 1], pts[0]);
        len += ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
    }
    len
}

fn score(layers: &[SimpleLayer]) {
    const LW: f64 = 0.4;
    const EDGE_EPS_MM: f64 = 0.06;
    const SKIRT_SKIP: &str = "Skirt";

    let mut sum_area = 0.0;
    let mut sum_unc = 0.0;
    let mut sum_band_area = 0.0;
    let mut sum_band_unc = 0.0;

    let mut wall_paths = 0usize;
    let mut wall_spans: Vec<f64> = Vec::new();
    let mut wall_jags: Vec<f64> = Vec::new();
    let mut total_paths = 0usize;
    let mut closed_loops = 0usize;
    let mut small_loops = 0usize;
    let mut subprintable_mm = 0.0;
    let mut total_extrude_mm = 0.0;

    for i in 0..layers.len() {
        let l = &layers[i];
        if l.outline.contours.is_empty() {
            continue;
        }
        let plan = to_plan(l);

        let grid_layer: Option<usize> = std::env::var("STAGE2_GRID").ok().and_then(|v| v.parse().ok());
        if grid_layer == Some(i) {
            std::env::set_var("GRID", "1");
            eprintln!("== layer {i} interior coverage ('#' = uncovered) ==");
        }
        let mut interior = plan.clone();
        interior.outline = geo2d::offset(&l.outline, -EDGE_EPS_MM);
        let (area, unc) = engine::debug_uncovered(&interior, LW);
        if grid_layer == Some(i) {
            std::env::remove_var("GRID");
        }

        let band = match layers.get(i + 1) {
            Some(n) if !n.outline.contours.is_empty() => geo2d::difference(&l.outline, &n.outline),
            _ => l.outline.clone(),
        };
        let band = geo2d::offset(&band, -EDGE_EPS_MM);
        let (band_area, band_unc) = if band.contours.is_empty() {
            (0.0, 0.0)
        } else {
            let mut top = plan.clone();
            top.outline = band;
            engine::debug_uncovered(&top, LW)
        };

        sum_area += area;
        sum_unc += unc;
        sum_band_area += band_area;
        sum_band_unc += band_unc;

        for p in &l.paths {
            if p.kind == SKIRT_SKIP {
                continue;
            }
            total_paths += 1;
            let len = path_len(&p.pts, p.closed);
            total_extrude_mm += len;
            if p.closed {
                closed_loops += 1;
                if len < 40.0 {
                    small_loops += 1;
                }
            }
            let is_wall = p.kind.contains("Perimeter") || p.kind.contains("Wall");
            if is_wall {
                wall_paths += 1;
                let span = p
                    .widths
                    .as_ref()
                    .map(|ws| {
                        let mx = ws.iter().cloned().fold(f64::MIN, f64::max);
                        let mn = ws.iter().cloned().fold(f64::MAX, f64::min);
                        mx - mn
                    })
                    .unwrap_or(0.0);
                wall_spans.push(span);
                // Jaggedness: worst width span inside any 2mm arc window —
                // separates raster-noise wobble from a legitimate long taper.
                let jag = p
                    .widths
                    .as_ref()
                    .map(|ws| {
                        let mut arc = vec![0.0];
                        for w in p.pts.windows(2) {
                            let d = ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
                            arc.push(arc.last().unwrap() + d);
                        }
                        let mut worst: f64 = 0.0;
                        let mut lo_i = 0usize;
                        for hi in 0..ws.len() {
                            while arc[hi] - arc[lo_i] > 2.0 {
                                lo_i += 1;
                            }
                            let (mut mn, mut mx) = (f64::MAX, f64::MIN);
                            for w in &ws[lo_i..=hi] {
                                mn = mn.min(*w);
                                mx = mx.max(*w);
                            }
                            worst = worst.max(mx - mn);
                        }
                        worst
                    })
                    .unwrap_or(0.0);
                wall_jags.push(jag);
            }
            // Sub-printable length: segments whose local width < 0.75*lw.
            match &p.widths {
                Some(ws) => {
                    for k in 0..p.pts.len().saturating_sub(1) {
                        let w = (ws[k] + ws[k + 1]) * 0.5;
                        if w < 0.75 * LW {
                            let (a, b) = (p.pts[k], p.pts[k + 1]);
                            subprintable_mm += ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
                        }
                    }
                }
                None => {
                    if p.width_mm < 0.75 * LW {
                        subprintable_mm += len;
                    }
                }
            }
        }
    }

    wall_spans.sort_by(f64::total_cmp);
    wall_jags.sort_by(f64::total_cmp);
    let n_wobbly = wall_spans.iter().filter(|s| **s > 0.15).count();
    let n_jagged = wall_jags.iter().filter(|s| **s > 0.15).count();
    let p90 = if wall_spans.is_empty() {
        0.0
    } else {
        let idx = (((wall_spans.len() as f64) * 0.90) as usize).min(wall_spans.len() - 1);
        wall_spans[idx]
    };
    let jag_p90 = if wall_jags.is_empty() {
        0.0
    } else {
        let idx = (((wall_jags.len() as f64) * 0.90) as usize).min(wall_jags.len() - 1);
        wall_jags[idx]
    };
    let nlayers = layers.iter().filter(|l| !l.outline.contours.is_empty()).count();

    println!("== stage2 score ==");
    println!(
        "residue interior : {:.2} mm2 of {:.1} ({:.4}%)",
        sum_unc,
        sum_area,
        100.0 * sum_unc / sum_area.max(1e-9)
    );
    println!(
        "residue show-face: {:.3} mm2 of {:.1} exposed ({:.4}%)",
        sum_band_unc,
        sum_band_area,
        100.0 * sum_band_unc / sum_band_area.max(1e-9)
    );
    println!(
        "wobble           : {n_wobbly} of {wall_paths} wall paths span >0.15mm ({:.2}%), p90 span {:.3} mm",
        100.0 * n_wobbly as f64 / wall_paths.max(1) as f64,
        p90
    );
    println!(
        "jag (2mm window) : {n_jagged} of {wall_paths} wall paths jag >0.15mm ({:.2}%), p90 jag {:.3} mm",
        100.0 * n_jagged as f64 / wall_paths.max(1) as f64,
        jag_p90
    );
    println!(
        "paths            : {total_paths} total ({:.1}/layer), {closed_loops} closed loops, {small_loops} small (<40mm) loops",
        total_paths as f64 / nlayers.max(1) as f64
    );
    println!(
        "sub-printable    : {:.0} mm of {:.0} mm extruded ({:.3}%)",
        subprintable_mm,
        total_extrude_mm,
        100.0 * subprintable_mm / total_extrude_mm.max(1e-9)
    );
}
