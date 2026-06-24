//! Inspect gap fill on a real slice.
//!
//!   cargo run --release -p engine --example gapview -- [layer] [file.stl]
//!
//! Generates the full plan, scans EVERY layer for gap-fill strokes whose path
//! folds back on itself (a >135° turn — the old "W"), reports the count/width
//! and worst offenders, then writes `/tmp/gapview.svg` of one layer: walls faint
//! grey, gap fill bright red (width-proportional). An auto-fit bbox fills the view.

use engine::{generate, PathKind};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let want_layer: Option<usize> = args.first().and_then(|a| a.parse().ok());
    let path = args.iter().skip(if want_layer.is_some() { 1 } else { 0 })
        .find(|a| a.ends_with(".stl"))
        .cloned()
        .unwrap_or_else(|| "fixtures/benchy.stl".into());

    let mesh = mesh::Mesh::load_stl(&path).expect("load STL");
    let mut settings = config::Settings::default();
    if let Ok(w) = std::env::var("WALLS") {
        if let Ok(n) = w.parse() {
            settings.wall_count = n;
        }
    }
    let plans = generate(&mesh, &settings);

    // --- scan all layers for gap-fill quality ---
    let mut total = 0usize;
    let mut hairpins: Vec<(usize, f64)> = Vec::new();
    let mut wmin = f64::MAX;
    let mut wmax: f64 = 0.0;
    let mut best_layer = (0usize, 0usize); // (layer, gapfill count) for the render
    for plan in &plans {
        let mut count = 0;
        for p in &plan.paths {
            if p.kind != PathKind::GapFill {
                continue;
            }
            count += 1;
            total += 1;
            wmin = wmin.min(p.width_mm);
            wmax = wmax.max(p.width_mm);
            let turn = sharpest_turn_deg(p);
            if turn > 135.0 {
                hairpins.push((plan.index, turn));
            }
        }
        if count > best_layer.1 {
            best_layer = (plan.index, count);
        }
    }

    println!("gap-fill strokes: {total} across {} layers", plans.len());
    println!("width range: {wmin:.2}..{wmax:.2} mm");
    println!("hairpins (>135° turn): {}", hairpins.len());
    for (l, t) in hairpins.iter().take(10) {
        println!("    layer {l}: {t:.0}°");
    }
    println!("busiest layer: {} ({} strokes)", best_layer.0, best_layer.1);

    let render_layer = want_layer.unwrap_or(best_layer.0);
    write_svg(&plans, render_layer);
}

fn sharpest_turn_deg(p: &engine::ToolPath) -> f64 {
    let mut worst: f64 = 0.0;
    for w in p.points.windows(3) {
        let (a, b, c) = (w[0], w[1], w[2]);
        let v1 = (b.x_mm() - a.x_mm(), b.y_mm() - a.y_mm());
        let v2 = (c.x_mm() - b.x_mm(), c.y_mm() - b.y_mm());
        let (n1, n2) = (v1.0.hypot(v1.1), v2.0.hypot(v2.1));
        if n1 < 1e-6 || n2 < 1e-6 {
            continue;
        }
        let cos = ((v1.0 * v2.0 + v1.1 * v2.1) / (n1 * n2)).clamp(-1.0, 1.0);
        worst = worst.max(cos.acos().to_degrees());
    }
    worst
}

fn write_svg(plans: &[engine::LayerPlan], layer: usize) {
    let plan = match plans.iter().find(|p| p.index == layer) {
        Some(p) => p,
        None => {
            eprintln!("no layer {layer}");
            return;
        }
    };
    // bbox over the gap-fill strokes (so the taper is visible), else all paths.
    let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    let zoom_gaps = plan.paths.iter().any(|p| p.kind == PathKind::GapFill);
    for p in &plan.paths {
        if zoom_gaps && p.kind != PathKind::GapFill {
            continue;
        }
        for pt in &p.points {
            x0 = x0.min(pt.x_mm());
            y0 = y0.min(pt.y_mm());
            x1 = x1.max(pt.x_mm());
            y1 = y1.max(pt.y_mm());
        }
    }
    let pad = if zoom_gaps { 4.0 } else { 2.0 };
    let (w, h) = (x1 - x0 + 2.0 * pad, y1 - y0 + 2.0 * pad);
    let scale = (1100.0 / w).min(1100.0 / h);
    let tx = |px: f64, py: f64| ((px - x0 + pad) * scale, (y1 - py + pad) * scale);

    let mut svg = format!(
        "<svg xmlns='http://www.w3.org/2000/svg' width='{:.0}' height='{:.0}'>\n<rect width='100%' height='100%' fill='#111'/>\n",
        w * scale, h * scale
    );
    // walls/infill faint grey first
    for p in &plan.paths {
        if p.kind == PathKind::GapFill {
            continue;
        }
        let col = match p.kind {
            PathKind::ExternalPerimeter => "#888",
            PathKind::Perimeter => "#666",
            _ => "#333",
        };
        polyline(&mut svg, p, &tx, scale, col, 0.6, p.kind == PathKind::ExternalPerimeter);
    }
    // gap fill bright red, width-proportional, on top
    let mut n = 0;
    for p in &plan.paths {
        if p.kind != PathKind::GapFill {
            continue;
        }
        n += 1;
        match &p.widths {
            // Per-point taper: draw each segment at its own width.
            Some(ws) => {
                for k in 0..p.points.len().saturating_sub(1) {
                    let (x1, y1) = tx(p.points[k].x_mm(), p.points[k].y_mm());
                    let (x2, y2) = tx(p.points[k + 1].x_mm(), p.points[k + 1].y_mm());
                    let w = ((ws[k] + ws[k + 1]) * 0.5 * scale).max(0.5);
                    svg.push_str(&format!(
                        "<line x1='{x1:.1}' y1='{y1:.1}' x2='{x2:.1}' y2='{y2:.1}' stroke='#ff3b3b' stroke-width='{w:.1}' stroke-linecap='round'/>\n"
                    ));
                }
            }
            None => polyline(&mut svg, p, &tx, scale, "#ff3b3b", (p.width_mm * scale).max(1.0), false),
        }
    }
    svg.push_str(&format!(
        "<text x='10' y='22' fill='#ccc' font-family='monospace' font-size='16'>layer {layer} — {n} gap-fill strokes</text>\n</svg>\n"
    ));
    std::fs::write("/tmp/gapview.svg", &svg).expect("write svg");
    println!("wrote /tmp/gapview.svg (layer {layer})");
}

fn polyline(
    svg: &mut String,
    p: &engine::ToolPath,
    tx: &impl Fn(f64, f64) -> (f64, f64),
    _scale: f64,
    color: &str,
    width: f64,
    _emphasize: bool,
) {
    if p.points.len() < 2 {
        return;
    }
    let pts: Vec<String> = p
        .points
        .iter()
        .map(|&pt| {
            let (x, y) = tx(pt.x_mm(), pt.y_mm());
            format!("{x:.1},{y:.1}")
        })
        .collect();
    let closer = if p.closed { format!(" {}", pts[0]) } else { String::new() };
    svg.push_str(&format!(
        "<polyline points='{}{}' fill='none' stroke='{color}' stroke-width='{width:.1}' stroke-opacity='0.85' stroke-linecap='round'/>\n",
        pts.join(" "),
        closer
    ));
}
