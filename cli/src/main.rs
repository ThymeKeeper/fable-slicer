//! `slicer` — command-line front-end.
//!
//! M1 capability: load an STL, plan walls + infill, and emit G-code. Optionally
//! dumps per-layer toolpath SVGs for visual inspection.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use clap::Parser;
use config::Settings;
use engine::{generate, to_gcode, LayerPlan, PathKind};
use geo2d::{Aabb, Point, UNITS_PER_MM};

#[derive(Parser)]
#[command(name = "slicer", version, about = "From-scratch FDM slicer (M1: STL -> g-code)")]
struct Args {
    /// Input mesh (STL, binary or ASCII).
    input: PathBuf,

    /// Output g-code file.
    #[arg(short, long, default_value = "out.gcode")]
    output: PathBuf,

    /// Also write per-layer toolpath SVGs to this directory.
    #[arg(long)]
    svg: Option<PathBuf>,

    #[arg(long, default_value_t = 0.2)]
    layer_height: f64,

    /// Number of perimeters (walls).
    #[arg(long, default_value_t = 2)]
    walls: usize,

    /// Sparse infill density, 0.0..=1.0.
    #[arg(long, default_value_t = 0.15)]
    infill: f64,

    #[arg(long, default_value_t = 200)]
    nozzle_temp: u32,

    #[arg(long, default_value_t = 60)]
    bed_temp: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mesh = mesh::Mesh::load_stl(&args.input)
        .with_context(|| format!("loading STL {}", args.input.display()))?;
    println!("Loaded {}: {} triangles", args.input.display(), mesh.triangles.len());

    let settings = Settings {
        layer_height_mm: args.layer_height,
        wall_count: args.walls,
        infill_density: args.infill,
        nozzle_temp_c: args.nozzle_temp,
        bed_temp_c: args.bed_temp,
        ..Settings::default()
    };

    let layers = generate(&mesh, &settings);
    let path_count: usize = layers.iter().map(|l| l.paths.len()).sum();
    println!("Planned {} layers, {} toolpaths", layers.len(), path_count);

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

/// Render one layer's toolpaths as colored polylines: external perimeter (dark
/// blue), inner perimeters (light blue), infill (orange).
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
            PathKind::ExternalPerimeter => "#1b5fb0",
            PathKind::Perimeter => "#5fa8e8",
            PathKind::Infill => "#e08a2b",
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
