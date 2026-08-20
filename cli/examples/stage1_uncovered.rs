//! Stage-1 residue audit (scratch tool, not shipped): slice a model at the
//! user's live profiles and measure, per layer, how much of the outline the
//! planned beads leave uncovered — split into interior residue (buried) and
//! residue inside the exposed-top band (outline minus the next layer's
//! outline), which is what actually prints on a show face under the
//! walls-99 / top-0 workflow.
//!
//! Usage: stage1_uncovered <model.stl|.3mf> [printer] [filament] [process]

use anyhow::{anyhow, Context, Result};
use config::Profiles;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let model = args
        .get(1)
        .context("usage: stage1_uncovered <model> [printer] [filament] [process]")?;
    let printer = args.get(2).map(String::as_str).unwrap_or("sovol-zero-custom");
    let filament = args.get(3).map(String::as_str).unwrap_or("petg-custom");
    let process = args.get(4).map(String::as_str).unwrap_or("sovol-zero-custom");

    let mut profiles = Profiles::builtin();
    profiles.load_user_profiles(None).map_err(|e| anyhow!(e))?;
    let mut settings = profiles
        .resolve_tools(printer, &[filament], process)
        .map_err(|e| anyhow!(e))?;
    if let Some(d) = std::env::var("STAGE1_DENSITY").ok().and_then(|v| v.parse().ok()) {
        settings.infill_density = d;
    }
    let lw = settings.line_width_mm;

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
    let layers = engine::generate_parts(&parts, &settings);
    println!(
        "{model}: {} layers, lw {lw} mm, walls {} top {} bottom {}",
        layers.len(),
        settings.wall_count,
        settings.top_layers,
        settings.bottom_layers
    );

    // Erode measurement regions a hair so raster speckle exactly on the
    // outline (where the outer bead's edge sits) doesn't read as residue.
    // A real channel (>=0.1mm wide, tens of mm long) survives this easily.
    const EDGE_EPS_MM: f64 = 0.06;

    let mut sum_area = 0.0;
    let mut sum_unc = 0.0;
    let mut sum_band_area = 0.0;
    let mut sum_band_unc = 0.0;
    let mut worst_interior: Vec<(usize, f64, f64)> = Vec::new(); // (layer, unc, area)
    let mut worst_band: Vec<(usize, f64, f64)> = Vec::new();

    for i in 0..layers.len() {
        let l = &layers[i];
        if l.outline.contours.is_empty() {
            continue;
        }

        // Whole-layer residue (mostly buried).
        let grid_layer: Option<usize> = std::env::var("STAGE1_GRID").ok().and_then(|v| v.parse().ok());
        if grid_layer == Some(i) {
            std::env::set_var("GRID", "1");
            eprintln!("== layer {i} interior coverage ('#' = uncovered) ==");
        }
        let mut interior = l.clone();
        interior.outline = geo2d::offset(&l.outline, -EDGE_EPS_MM);
        let (area, unc) = engine::debug_uncovered(&interior, lw);

        // Show-face residue: the part of this layer not covered by the next.
        let band = match layers.get(i + 1) {
            Some(n) if !n.outline.contours.is_empty() => {
                geo2d::difference(&l.outline, &n.outline)
            }
            _ => l.outline.clone(),
        };
        let band = geo2d::offset(&band, -EDGE_EPS_MM);
        if grid_layer == Some(i) {
            eprintln!("== layer {i} show-face band coverage ('#' = uncovered) ==");
        }
        let (band_area, band_unc) = if band.contours.is_empty() {
            (0.0, 0.0)
        } else {
            let mut top = l.clone();
            top.outline = band;
            engine::debug_uncovered(&top, lw)
        };
        if grid_layer == Some(i) {
            std::env::remove_var("GRID");
        }

        sum_area += area;
        sum_unc += unc;
        sum_band_area += band_area;
        sum_band_unc += band_unc;
        worst_interior.push((i, unc, area));
        worst_band.push((i, band_unc, band_area));

        if band_unc > 0.05 {
            println!(
                "  layer {i:4} z {:7.2}: SHOW-FACE uncovered {band_unc:8.3} mm2 of {band_area:9.2} mm2 exposed ({:.2}%)  [interior unc {unc:.3} of {area:.1}]",
                l.print_z_mm,
                100.0 * band_unc / band_area.max(1e-9)
            );
        }
    }

    worst_interior.sort_by(|a, b| b.1.total_cmp(&a.1));
    worst_band.sort_by(|a, b| b.1.total_cmp(&a.1));

    println!("---- totals ----");
    println!(
        "interior: {sum_unc:.2} mm2 uncovered of {sum_area:.1} mm2 sliced area ({:.4}%)",
        100.0 * sum_unc / sum_area.max(1e-9)
    );
    println!(
        "show-face: {sum_band_unc:.3} mm2 uncovered of {sum_band_area:.1} mm2 exposed-top area ({:.4}%)",
        100.0 * sum_band_unc / sum_band_area.max(1e-9)
    );
    println!("worst interior layers:");
    for (i, unc, area) in worst_interior.iter().take(5) {
        println!("  layer {i:4}: {unc:8.3} mm2 of {area:9.2} ({:.3}%)", 100.0 * unc / area.max(1e-9));
    }
    println!("worst show-face layers:");
    for (i, unc, area) in worst_band.iter().take(5) {
        if *unc > 0.0 {
            println!("  layer {i:4}: {unc:8.3} mm2 of {area:9.2} exposed ({:.3}%)", 100.0 * unc / area.max(1e-9));
        }
    }
    Ok(())
}
