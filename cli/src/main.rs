//! `slicer` — command-line front-end.
//!
//! M0 capability: load an STL, slice it into layers, and write one SVG per layer
//! so the slicing result can be inspected visually. (G-code output arrives at M1.)

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use engine::{slice_mesh, SliceParams};
use geo2d::{Aabb, Point, Polygons, UNITS_PER_MM};

#[derive(Parser)]
#[command(name = "slicer", version, about = "From-scratch FDM slicer (M0: STL -> per-layer SVG)")]
struct Args {
    /// Input mesh (STL, binary or ASCII).
    input: PathBuf,

    /// Layer height in millimeters.
    #[arg(long, default_value_t = 0.2)]
    layer_height: f64,

    /// Output directory for per-layer SVG files.
    #[arg(long, default_value = "out")]
    out: PathBuf,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mesh = mesh::Mesh::load_stl(&args.input)
        .with_context(|| format!("loading STL {}", args.input.display()))?;
    println!(
        "Loaded {}: {} triangles, {} vertices",
        args.input.display(),
        mesh.triangles.len(),
        mesh.vertices.len()
    );

    let layers = slice_mesh(
        &mesh,
        SliceParams { layer_height_mm: args.layer_height },
    );
    println!("Sliced into {} layers at {} mm", layers.len(), args.layer_height);

    // Shared bounds across all layers so every SVG uses the same coordinate frame.
    let mut bounds: Option<Aabb> = None;
    for l in &layers {
        if let Some(b) = l.polygons.bounds() {
            match &mut bounds {
                Some(bb) => bb.union(&b),
                None => bounds = Some(b),
            }
        }
    }
    let bounds = bounds.context("no geometry produced — is the mesh empty or below layer height?")?;

    std::fs::create_dir_all(&args.out)
        .with_context(|| format!("creating output dir {}", args.out.display()))?;
    for l in &layers {
        let svg = render_svg(&l.polygons, &bounds);
        let path = args.out.join(format!("layer_{:04}.svg", l.index));
        std::fs::write(&path, svg).with_context(|| format!("writing {}", path.display()))?;
    }
    println!("Wrote {} SVG layers to {}/", layers.len(), args.out.display());

    Ok(())
}

/// Render one layer's polygons to a standalone SVG string. Even-odd fill renders
/// holes correctly when outer+hole contours are drawn as one path; here each
/// contour is its own path (fine for M0 — solid parts only).
fn render_svg(polys: &Polygons, bounds: &Aabb) -> String {
    const TARGET_PX: f64 = 600.0;
    const MARGIN: f64 = 12.0;

    let w_mm = (bounds.width() as f64 / UNITS_PER_MM).max(1.0e-6);
    let h_mm = (bounds.height() as f64 / UNITS_PER_MM).max(1.0e-6);
    let scale = TARGET_PX / w_mm.max(h_mm); // px per mm
    let px_w = w_mm * scale + 2.0 * MARGIN;
    let px_h = h_mm * scale + 2.0 * MARGIN;

    let min_x_mm = bounds.min.x as f64 / UNITS_PER_MM;
    let min_y_mm = bounds.min.y as f64 / UNITS_PER_MM;
    let to_px = |p: Point| {
        let x = (p.x_mm() - min_x_mm) * scale + MARGIN;
        let y = (p.y_mm() - min_y_mm) * scale + MARGIN;
        (x, px_h - y) // flip Y so +Y points up
    };

    let mut s = String::new();
    s.push_str(&format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{px_w:.1}" height="{px_h:.1}" viewBox="0 0 {px_w:.1} {px_h:.1}">"#
    ));
    s.push_str(r##"<rect width="100%" height="100%" fill="#ffffff"/>"##);

    for c in &polys.contours {
        if c.points.len() < 3 {
            continue;
        }
        // Outer loops (CCW) filled; holes (CW) left white.
        let fill = if c.is_ccw() { "#cfe8ff" } else { "#ffffff" };
        let mut d = String::from("M");
        for (i, &p) in c.points.iter().enumerate() {
            let (x, y) = to_px(p);
            if i == 0 {
                d.push_str(&format!("{x:.2} {y:.2} "));
            } else {
                d.push_str(&format!("L{x:.2} {y:.2} "));
            }
        }
        d.push('Z');
        s.push_str(&format!(
            r##"<path d="{d}" fill="{fill}" fill-rule="evenodd" stroke="#1b5fb0" stroke-width="1.2"/>"##
        ));
    }

    s.push_str("</svg>");
    s
}
