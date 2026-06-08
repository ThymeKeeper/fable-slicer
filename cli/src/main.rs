//! `slicer` — command-line front-end.
//!
//! Loads an STL, resolves printer/filament/process profiles into settings (with
//! optional overrides), plans walls + infill, and emits Klipper-flavored g-code.
//! Optionally dumps per-layer toolpath SVGs.

use std::path::{Path, PathBuf};

use anyhow::{anyhow, Context, Result};
use clap::Parser;
use config::Profiles;
use engine::{generate, to_gcode, LayerPlan, PathKind};
use geo2d::{Aabb, Point, UNITS_PER_MM};

#[derive(Parser)]
#[command(name = "slicer", version, about = "From-scratch FDM slicer")]
struct Args {
    /// Input mesh (STL, binary or ASCII). Optional with --list-profiles.
    input: Option<PathBuf>,

    /// Output g-code file.
    #[arg(short, long, default_value = "out.gcode")]
    output: PathBuf,

    /// Also write per-layer toolpath SVGs to this directory.
    #[arg(long)]
    svg: Option<PathBuf>,

    // --- profiles ---
    #[arg(long, default_value = "generic")]
    printer: String,
    #[arg(long, default_value = "pla")]
    filament: String,
    #[arg(long, default_value = "standard")]
    process: String,
    /// Load extra profiles from <dir>/{printer,filament,process}/*.toml.
    #[arg(long)]
    profile_dir: Option<PathBuf>,
    /// List available profiles and exit.
    #[arg(long)]
    list_profiles: bool,

    // --- overrides (take precedence over the resolved profile) ---
    #[arg(long)]
    layer_height: Option<f64>,
    #[arg(long)]
    first_layer_height: Option<f64>,
    #[arg(long)]
    walls: Option<usize>,
    #[arg(long)]
    infill: Option<f64>,
    /// Number of skirt loops (0 disables).
    #[arg(long)]
    skirt: Option<usize>,
    /// Number of brim loops (0 disables).
    #[arg(long)]
    brim: Option<usize>,
    /// Seam placement: nearest | sharpest | random.
    #[arg(long)]
    seam: Option<String>,
    /// Sparse infill pattern: lines | grid | concentric.
    #[arg(long)]
    sparse_infill: Option<String>,
    /// Solid infill pattern: lines | grid | concentric.
    #[arg(long)]
    solid_infill: Option<String>,
    /// Support mode: none | grid | arc.
    #[arg(long)]
    support: Option<String>,
    #[arg(long)]
    nozzle_temp: Option<u32>,
    #[arg(long)]
    bed_temp: Option<u32>,
    #[arg(long)]
    bed_x: Option<f64>,
    #[arg(long)]
    bed_y: Option<f64>,
    #[arg(long)]
    bed_z: Option<f64>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut profiles = Profiles::builtin();
    if let Some(dir) = &args.profile_dir {
        profiles.load_dir(dir).map_err(|e| anyhow!(e))?;
    }

    if args.list_profiles {
        println!("printers:  {}", profiles.printer_names().join(", "));
        println!("filaments: {}", profiles.filament_names().join(", "));
        println!("processes: {}", profiles.process_names().join(", "));
        return Ok(());
    }

    let input = args
        .input
        .clone()
        .context("no input STL given (use --list-profiles to see profiles)")?;

    let mut settings = profiles
        .resolve(&args.printer, &args.filament, &args.process)
        .map_err(|e| anyhow!(e))?;

    // Apply overrides.
    if let Some(v) = args.layer_height {
        settings.layer_height_mm = v;
    }
    if let Some(v) = args.first_layer_height {
        settings.first_layer_height_mm = v;
    }
    if let Some(v) = args.walls {
        settings.wall_count = v;
    }
    if let Some(v) = args.infill {
        settings.infill_density = v;
    }
    if let Some(v) = args.skirt {
        settings.skirt_loops = v;
    }
    if let Some(v) = args.brim {
        settings.brim_loops = v;
    }
    if let Some(s) = &args.seam {
        match config::SeamMode::parse(s) {
            Some(m) => settings.seam_mode = m,
            None => anyhow::bail!("unknown seam mode '{s}' (use nearest | sharpest | random)"),
        }
    }
    if let Some(s) = &args.sparse_infill {
        settings.sparse_pattern = config::InfillPattern::parse(s)
            .ok_or_else(|| anyhow::anyhow!("unknown infill pattern '{s}' (use lines | grid | concentric)"))?;
    }
    if let Some(s) = &args.solid_infill {
        settings.solid_pattern = config::InfillPattern::parse(s)
            .ok_or_else(|| anyhow::anyhow!("unknown infill pattern '{s}' (use lines | grid | concentric)"))?;
    }
    if let Some(s) = &args.support {
        settings.support_mode = config::SupportMode::parse(s)
            .ok_or_else(|| anyhow::anyhow!("unknown support mode '{s}' (use none | grid | arc)"))?;
    }
    if let Some(v) = args.nozzle_temp {
        settings.nozzle_temp_c = v;
    }
    if let Some(v) = args.bed_temp {
        settings.bed_temp_c = v;
    }
    if let Some(v) = args.bed_x {
        settings.bed_size_x_mm = v;
    }
    if let Some(v) = args.bed_y {
        settings.bed_size_y_mm = v;
    }
    if let Some(v) = args.bed_z {
        settings.bed_size_z_mm = v;
    }

    println!(
        "Profiles: printer={} filament={} process={} | bed {}x{} mm, layer {}mm",
        args.printer,
        args.filament,
        args.process,
        settings.bed_size_x_mm,
        settings.bed_size_y_mm,
        settings.layer_height_mm
    );

    let mesh = mesh::Mesh::load_stl(&input)
        .with_context(|| format!("loading STL {}", input.display()))?;
    println!("Loaded {}: {} triangles", input.display(), mesh.triangles.len());

    let layers = generate(&mesh, &settings);
    let path_count: usize = layers.iter().map(|l| l.paths.len()).sum();
    println!("Planned {} layers, {} toolpaths", layers.len(), path_count);
    println!(
        "Estimated print time: {}",
        engine::format_duration(engine::estimate_seconds(&layers, &settings))
    );
    let (fil_mm, grams) = engine::estimate_filament(&layers, &settings);
    println!("Filament: {:.2} m, {:.1} g", fil_mm / 1000.0, grams);
    let (cross, combed, fb, fb_hole) = engine::audit_combing(&layers);
    println!("Combing: {cross} crossing travels — {combed} combed, {fb} straight ({fb_hole} cut a hole)");

    let gcode = to_gcode(&layers, &settings);
    std::fs::write(&args.output, &gcode)
        .with_context(|| format!("writing {}", args.output.display()))?;
    println!("Wrote {} ({} g-code lines)", args.output.display(), gcode.lines().count());

    if let Some(dir) = &args.svg {
        write_svgs(&layers, dir)?;
        println!("Wrote {} toolpath SVGs to {}/", layers.len(), dir.display());
    }

    Ok(())
}

fn write_svgs(layers: &[LayerPlan], dir: &Path) -> Result<()> {
    let mut bounds: Option<Aabb> = None;
    for l in layers {
        for p in &l.paths {
            for &pt in &p.points {
                match &mut bounds {
                    Some(b) => b.expand(pt),
                    None => bounds = Some(Aabb { min: pt, max: pt }),
                }
            }
        }
    }
    let Some(bounds) = bounds else { return Ok(()) };

    std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    for l in layers {
        let svg = render_layer_svg(l, &bounds);
        std::fs::write(dir.join(format!("layer_{:04}.svg", l.index)), svg)?;
    }
    Ok(())
}

/// Render one layer's toolpaths as colored polylines.
fn render_layer_svg(layer: &LayerPlan, bounds: &Aabb) -> String {
    const TARGET_PX: f64 = 600.0;
    const MARGIN: f64 = 12.0;

    let w_mm = (bounds.width() as f64 / UNITS_PER_MM).max(1.0e-6);
    let h_mm = (bounds.height() as f64 / UNITS_PER_MM).max(1.0e-6);
    let scale = TARGET_PX / w_mm.max(h_mm);
    let px_w = w_mm * scale + 2.0 * MARGIN;
    let px_h = h_mm * scale + 2.0 * MARGIN;
    let min_x = bounds.min.x as f64 / UNITS_PER_MM;
    let min_y = bounds.min.y as f64 / UNITS_PER_MM;
    let to_px = |p: Point| {
        let x = (p.x_mm() - min_x) * scale + MARGIN;
        let y = (p.y_mm() - min_y) * scale + MARGIN;
        (x, px_h - y) // flip Y so +Y points up
    };

    let mut s = String::new();
    s.push_str(&format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{px_w:.1}" height="{px_h:.1}" viewBox="0 0 {px_w:.1} {px_h:.1}">"#
    ));
    s.push_str(r##"<rect width="100%" height="100%" fill="#ffffff"/>"##);

    for path in &layer.paths {
        if path.points.len() < 2 {
            continue;
        }
        let color = match path.kind {
            PathKind::Skirt => "#999999",
            PathKind::ExternalPerimeter => "#1b5fb0",
            PathKind::Perimeter => "#5fa8e8",
            PathKind::Solid => "#2ca02c",
            PathKind::Infill => "#e08a2b",
            PathKind::Support => "#8c6bb1",
            PathKind::Bridge => "#17becf",
        };
        let mut d = String::from("M");
        for (i, &p) in path.points.iter().enumerate() {
            let (x, y) = to_px(p);
            let cmd = if i == 0 { "" } else { "L" };
            d.push_str(&format!("{cmd}{x:.2} {y:.2} "));
        }
        if path.closed {
            d.push('Z');
        }
        s.push_str(&format!(
            r##"<path d="{d}" fill="none" stroke="{color}" stroke-width="1"/>"##
        ));
    }

    s.push_str("</svg>");
    s
}
