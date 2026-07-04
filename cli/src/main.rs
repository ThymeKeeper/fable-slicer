//! `slicer` — command-line front-end.
//!
//! Loads an STL or 3MF, resolves printer/filament/process profiles into
//! settings (with optional overrides), plans walls + infill, and emits
//! Klipper-flavored g-code. Repeating `--filament` loads one profile per tool
//! slot on a toolchanger; a multi-part 3MF's parts then print on their
//! embedded extruder assignments. Optionally dumps per-layer toolpath SVGs.

use std::path::{Path, PathBuf};

use anyhow::{anyhow, Context, Result};
use clap::Parser;
use config::Profiles;
use engine::{to_gcode, LayerPlan, PathKind};
use geo2d::{Aabb, Point, UNITS_PER_MM};

#[derive(Parser)]
#[command(name = "fable-slicer", version, about = "Fable Slicer — from-scratch FDM slicer")]
struct Args {
    /// Input model (STL binary/ASCII, or 3MF). Optional with --list-profiles.
    input: Option<PathBuf>,

    /// Output g-code file.
    #[arg(short, long, default_value = "out.gcode")]
    output: PathBuf,

    /// Also write per-layer toolpath SVGs to this directory.
    #[arg(long)]
    svg: Option<PathBuf>,

    /// Upload the g-code to the printer (Moonraker host from the printer
    /// profile, or --host).
    #[arg(long)]
    upload: bool,
    /// Upload and start printing immediately (implies --upload).
    #[arg(long)]
    start_print: bool,
    /// Printer host override (e.g. voron24.local or http://192.168.1.50).
    #[arg(long)]
    host: Option<String>,

    // --- profiles ---
    #[arg(long, default_value = "generic")]
    printer: String,
    /// Filament profile — repeat once per tool slot on a toolchanger
    /// (--filament white --filament grey --filament black). A multi-part 3MF
    /// then assigns each part its embedded extruder, clamped to the slots.
    #[arg(long, default_value = "pla")]
    filament: Vec<String>,
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
    /// Seam placement: nearest | aligned | sharpest | random.
    #[arg(long)]
    seam: Option<String>,
    /// Sparse infill pattern: lines | aligned | grid | triangles | concentric | gyroid.
    #[arg(long)]
    sparse_infill: Option<String>,
    /// Top skin pattern: lines | aligned | grid | triangles | concentric | gyroid.
    #[arg(long)]
    top_infill: Option<String>,
    /// Bottom skin pattern: lines | aligned | grid | triangles | concentric | gyroid.
    #[arg(long)]
    bottom_infill: Option<String>,
    /// Buried solid infill pattern: lines | aligned | grid | triangles | concentric | gyroid.
    #[arg(long)]
    solid_infill: Option<String>,
    /// Support mode: none | grid.
    #[arg(long)]
    support: Option<String>,
    /// Spiral vase mode: one continuously rising wall, no infill above the bottom.
    #[arg(long)]
    vase: bool,
    /// Fuzzy skin: jitter the outer wall for a rough surface texture.
    #[arg(long)]
    fuzzy_skin: bool,
    /// Iron top surfaces with a low-flow smoothing pass.
    #[arg(long)]
    ironing: bool,
    /// Shrink the first layer outline by this much (mm) to counter squish.
    #[arg(long)]
    elephant_foot: Option<f64>,
    /// Grow (+) / shrink (−) every layer outline by this much (mm).
    #[arg(long)]
    xy_compensation: Option<f64>,
    #[arg(long)]
    nozzle_temp: Option<u32>,
    /// First-layer nozzle °C (with --nozzle-temp alone, the first layer follows it)
    #[arg(long)]
    first_layer_nozzle_temp: Option<u32>,
    #[arg(long)]
    bed_temp: Option<u32>,
    #[arg(long)]
    bed_x: Option<f64>,
    #[arg(long)]
    bed_y: Option<f64>,
    #[arg(long)]
    bed_z: Option<f64>,
    /// Fit circular arcs to curved toolpaths and emit G2/G3 (needs firmware arc support).
    #[arg(long)]
    arc_fitting: bool,
    /// Dev: per-face paint test — paint every triangle on the +X half of each
    /// part with this tool (the rest tool 0) and slice via the painter. Verifies
    /// the per-face color path end to end (check the per-tool grams).
    #[arg(long)]
    paint_halfx: Option<u32>,
}

/// Per-triangle palette index for the `--paint-halfx` dev flag: 1 for every
/// triangle whose centroid is on the +X half of the mesh, 0 otherwise.
fn paint_half_x(m: &mesh::Mesh) -> Vec<u32> {
    let (min_x, _, max_x, _) = m.xy_bounds().unwrap_or((0.0, 0.0, 0.0, 0.0));
    let mid = (min_x + max_x) / 2.0;
    (0..m.triangles.len())
        .map(|i| {
            let [a, b, c] = m.triangle(i);
            if (a[0] + b[0] + c[0]) / 3.0 > mid {
                1
            } else {
                0
            }
        })
        .collect()
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut profiles = Profiles::builtin();
    // User profiles (the ones the GUI saves) load automatically; a bad file in
    // the config dir is reported but doesn't block slicing with built-ins.
    match profiles.load_user_profiles(None) {
        Ok(skipped) if !skipped.is_empty() => eprintln!(
            "warning: user profiles shadowing built-ins were skipped: {} (use 'inherits' instead)",
            skipped.join(", ")
        ),
        Err(e) => eprintln!("warning: user profiles: {e}"),
        _ => {}
    }
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
        .context("no input model given (use --list-profiles to see profiles)")?;

    let filaments: Vec<&str> = args.filament.iter().map(String::as_str).collect();
    let mut settings = profiles
        .resolve_tools(&args.printer, &filaments, &args.process)
        .map_err(|e| anyhow!(e))?;
    // Loading N filaments implies at least N tools, whatever the profile says.
    settings.tool_count = settings.tool_count.max(filaments.len());

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
    if let Some(s) = &args.top_infill {
        settings.top_pattern = config::InfillPattern::parse(s)
            .ok_or_else(|| anyhow::anyhow!("unknown infill pattern '{s}' (use lines | grid | concentric)"))?;
    }
    if let Some(s) = &args.bottom_infill {
        settings.bottom_pattern = config::InfillPattern::parse(s)
            .ok_or_else(|| anyhow::anyhow!("unknown infill pattern '{s}' (use lines | grid | concentric)"))?;
    }
    if let Some(s) = &args.solid_infill {
        settings.solid_pattern = config::InfillPattern::parse(s)
            .ok_or_else(|| anyhow::anyhow!("unknown infill pattern '{s}' (use lines | grid | concentric)"))?;
    }
    if let Some(s) = &args.support {
        settings.support_mode = config::SupportMode::parse(s)
            .ok_or_else(|| anyhow::anyhow!("unknown support mode '{s}' (use none | grid)"))?;
    }
    if let Some(v) = args.nozzle_temp {
        settings.nozzle_temp_c = v;
        // Keep the first layer in lockstep unless pinned explicitly below.
        settings.first_layer_nozzle_temp_c = v;
        // Temperature overrides speak about tool 0; the other slots keep
        // their own profiles' temperatures.
        settings.tools[0].nozzle_temp_c = v;
        settings.tools[0].first_layer_nozzle_temp_c = v;
    }
    if let Some(v) = args.first_layer_nozzle_temp {
        settings.first_layer_nozzle_temp_c = v;
        settings.tools[0].first_layer_nozzle_temp_c = v;
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
    if args.arc_fitting {
        settings.arc_fitting = true;
    }
    if args.vase {
        settings.spiral_vase = true;
    }
    if args.fuzzy_skin {
        settings.fuzzy_skin = true;
    }
    if args.ironing {
        settings.ironing = true;
    }
    if let Some(v) = args.elephant_foot {
        settings.elephant_foot_mm = v;
    }
    if let Some(v) = args.xy_compensation {
        settings.xy_compensation_mm = v;
    }

    println!(
        "Profiles: printer={} filament={} process={} | bed {}x{} mm, layer {}mm",
        args.printer,
        args.filament.join(", "),
        args.process,
        settings.bed_size_x_mm,
        settings.bed_size_y_mm,
        settings.layer_height_mm
    );

    let is_3mf = input.extension().map(|e| e.eq_ignore_ascii_case("3mf")).unwrap_or(false);
    // The plate to slice: one (mesh, tool) per part, build placement baked.
    // With one filament everything is tool 0 (a merged 3MF plate, exactly the
    // classic behavior); with several, each 3MF part follows its embedded
    // extruder assignment, clamped to the loaded slots.
    let last_tool = (settings.tool_count - 1) as u32;
    let parts: Vec<(mesh::Mesh, u32)> = if is_3mf {
        let items = mesh::load_3mf(&input)
            .map_err(|e| anyhow::anyhow!(e))
            .with_context(|| format!("loading 3MF {}", input.display()))?;
        println!("Loaded {}: {} object(s)", input.display(), items.len());
        if filaments.len() > 1 {
            items
                .into_iter()
                .flat_map(|it| it.parts)
                .map(|p| {
                    let tool = p.extruder.map_or(0, |e| e.saturating_sub(1)).min(last_tool);
                    (p.mesh, tool)
                })
                .collect()
        } else {
            let mut m = mesh::Mesh::default();
            for p in items.iter().flat_map(|it| &it.parts) {
                m.append(&p.mesh);
            }
            vec![(m, 0)]
        }
    } else {
        let m = mesh::Mesh::load_stl(&input)
            .with_context(|| format!("loading STL {}", input.display()))?;
        vec![(m, 0)]
    };
    let tris: usize = parts.iter().map(|(m, _)| m.triangles.len()).sum();
    println!("Loaded {}: {tris} triangles", input.display());

    let layers = if let Some(tool) = args.paint_halfx {
        // Synthesize per-face paint: +X-half triangles get `tool`, the rest 0.
        let painted: Vec<(&mesh::Mesh, engine::PartPaint)> = parts
            .iter()
            .map(|(m, _)| {
                (
                    m,
                    engine::PartPaint::Painted {
                        face: paint_half_x(m),
                        paints: vec![engine::FacePaint::Tool(0), engine::FacePaint::Tool(tool)],
                    },
                )
            })
            .collect();
        engine::generate_painted(&painted, &settings)
    } else {
        let part_refs: Vec<(&mesh::Mesh, u32)> = parts.iter().map(|(m, t)| (m, *t)).collect();
        engine::generate_parts(&part_refs, &settings)
    };
    let path_count: usize = layers.iter().map(|l| l.paths.len()).sum();
    println!("Planned {} layers, {} toolpaths", layers.len(), path_count);
    println!(
        "Estimated print time: {}",
        engine::format_duration(engine::estimate_seconds(&layers, &settings))
    );
    let (fil_mm, grams) = engine::estimate_filament(&layers, &settings);
    println!("Filament: {:.2} m, {:.1} g", fil_mm / 1000.0, grams);
    if settings.tool_count > 1 {
        for (n, mm, g) in engine::estimate_filament_per_tool(&layers, &settings) {
            let name = &settings.tool(n as usize).filament_name;
            println!("  T{n} ({name}): {:.2} m, {g:.1} g", mm / 1000.0);
        }
    }
    let (cross, combed, fb, fb_hole) = engine::audit_combing(&layers);
    println!("Combing: {cross} crossing travels — {combed} combed, {fb} straight ({fb_hole} cut a hole)");

    let gcode = to_gcode(&layers, &settings);
    std::fs::write(&args.output, &gcode)
        .with_context(|| format!("writing {}", args.output.display()))?;
    println!("Wrote {} ({} g-code lines)", args.output.display(), gcode.lines().count());

    if args.upload || args.start_print {
        let host = args.host.clone().unwrap_or_else(|| settings.host_url.clone());
        if host.trim().is_empty() {
            anyhow::bail!("no printer host: set host_url in the printer profile or pass --host");
        }
        let client = printhost::Client::new(&host, &settings.api_key);
        // A chamber soak waits on the printer's chamber sensor; verify it's
        // really there before sending, so a missing/misnamed sensor fails here
        // with a clear message instead of aborting mid-startup on the machine.
        if settings.chamber_temp_c > 0 {
            client
                .ensure_chamber_sensor(&settings.chamber_sensor, settings.chamber_temp_c)
                .map_err(|e| anyhow::anyhow!(e))?;
        }
        // A multi-tool job must land on a machine that has those tools —
        // catches "sliced for the toolchanger, sent to the single-tool
        // printer" before upload instead of mid-print.
        client
            .ensure_tools(
                &engine::used_tools(&layers),
                settings.toolchange_gcode.contains("T{tool}"),
            )
            .map_err(|e| anyhow::anyhow!(e))?;
        let filename = args
            .output
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "print.gcode".into());
        client
            .upload(&filename, gcode.as_bytes(), args.start_print)
            .map_err(|e| anyhow::anyhow!("upload to {host} failed: {e}"))?;
        if args.start_print {
            println!("Uploaded {filename} to {host} — printing.");
        } else {
            println!("Uploaded {filename} to {host}.");
        }
    }

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
            PathKind::OverhangWall => "#f59e16",
            PathKind::Solid => "#2ca02c",
            PathKind::TopSkin => "#ed618c",
            PathKind::BottomSkin => "#9e7333",
            PathKind::Infill => "#e08a2b",
            PathKind::Ironing => "#bcbd22",
            PathKind::Support => "#8c6bb1",
            PathKind::Bridge => "#17becf",
            PathKind::InternalBridge => "#1f8bb8",
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
