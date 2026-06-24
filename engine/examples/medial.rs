//! Standalone eyeball + numeric gate for the segment-Voronoi medial axis.
//!
//!   cargo run --release -p engine --example medial
//!
//! Runs `engine::medial::medial_axis` on a handful of synthetic gap slivers that
//! exercise the cases the old boundary-pairing centerline folded on — a taper,
//! a right-angle corner, a 4-way junction, and a curved crescent — writes an SVG
//! (`/tmp/medial.svg`: outline + medial beads + free-end dots) and prints, per
//! shape, the polyline count, width range, and the sharpest turn angle found.
//! A clean medial axis turns gently; a hairpin (~180°, the "W") would show as a
//! large max-turn and is the thing to watch.

use engine::medial::{medial_axis, ThickPolyline};
use geo2d::{Contour, Point, Polygons};

const LW: f64 = 0.45; // nozzle line width — gap band is [0.2*lw, 2*sp]
const MIN_W: f64 = 0.2 * LW;
const MAX_W: f64 = 2.0 * 0.4; // ~2 * bead spacing

fn main() {
    let shapes: Vec<(&str, Polygons)> = vec![
        ("taper", taper()),
        ("corner-L", corner_l()),
        ("plus", plus()),
        ("crescent", crescent()),
    ];

    let mut svg = String::new();
    svg.push_str(
        "<svg xmlns='http://www.w3.org/2000/svg' width='1000' height='280' viewBox='0 0 1000 280'>\n",
    );
    svg.push_str("<rect width='1000' height='280' fill='#111'/>\n");

    let scale = 18.0; // px per mm
    for (i, (name, poly)) in shapes.iter().enumerate() {
        let mas = medial_axis(poly, MIN_W, MAX_W);
        report(name, &mas);
        let ox = 30.0 + i as f64 * 245.0;
        let oy = 150.0;
        draw_shape(&mut svg, poly, &mas, ox, oy, scale, name);
    }
    svg.push_str("</svg>\n");
    std::fs::write("/tmp/medial.svg", &svg).expect("write svg");
    println!("\nwrote /tmp/medial.svg");
}

// --------------------------------------------------------------------------
// reporting
// --------------------------------------------------------------------------

fn report(name: &str, mas: &[ThickPolyline]) {
    if mas.is_empty() {
        println!("{name:<9}  EMPTY (no medial axis produced)");
        return;
    }
    let mut wmin = f64::MAX;
    let mut wmax: f64 = 0.0;
    let mut max_turn: f64 = 0.0;
    let mut total = 0.0;
    for tp in mas {
        total += tp.length_mm();
        for &w in &tp.widths {
            wmin = wmin.min(w);
            wmax = wmax.max(w);
        }
        max_turn = max_turn.max(sharpest_turn_deg(&tp.points));
    }
    let flag = if max_turn > 135.0 { "  <-- HAIRPIN/W!" } else { "" };
    println!(
        "{name:<9}  {} polyline(s), len {total:5.2}mm, width {wmin:.2}..{wmax:.2}mm, \
         max-turn {max_turn:5.1}°{flag}",
        mas.len()
    );
}

/// Sharpest direction change (degrees) across a polyline's interior vertices.
/// 0° = straight, 180° = a full reversal (a hairpin / the "W").
fn sharpest_turn_deg(pts: &[Point]) -> f64 {
    let mut worst: f64 = 0.0;
    for w in pts.windows(3) {
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

// --------------------------------------------------------------------------
// SVG
// --------------------------------------------------------------------------

fn draw_shape(
    svg: &mut String,
    poly: &Polygons,
    mas: &[ThickPolyline],
    ox: f64,
    oy: f64,
    scale: f64,
    name: &str,
) {
    let tx = |p: Point| (ox + p.x_mm() * scale, oy - p.y_mm() * scale);
    // outline
    for c in &poly.contours {
        let pts: Vec<String> =
            c.points.iter().map(|&p| { let (x, y) = tx(p); format!("{x:.1},{y:.1}") }).collect();
        svg.push_str(&format!(
            "<polygon points='{}' fill='#222' stroke='#666' stroke-width='0.7'/>\n",
            pts.join(" ")
        ));
    }
    // medial beads: each segment as a round-capped stroke of its mean width
    for tp in mas {
        for k in 0..tp.points.len().saturating_sub(1) {
            let (x1, y1) = tx(tp.points[k]);
            let (x2, y2) = tx(tp.points[k + 1]);
            let w = ((tp.widths[k] + tp.widths[k + 1]) * 0.5 * scale).max(0.5);
            svg.push_str(&format!(
                "<line x1='{x1:.1}' y1='{y1:.1}' x2='{x2:.1}' y2='{y2:.1}' \
                 stroke='#39a0ff' stroke-opacity='0.55' stroke-width='{w:.1}' stroke-linecap='round'/>\n"
            ));
        }
        // centerline on top
        let cl: Vec<String> =
            tp.points.iter().map(|&p| { let (x, y) = tx(p); format!("{x:.1},{y:.1}") }).collect();
        svg.push_str(&format!(
            "<polyline points='{}' fill='none' stroke='#ffd23f' stroke-width='0.8'/>\n",
            cl.join(" ")
        ));
        // free-end dots
        for (end, &flag) in [tp.points.first(), tp.points.last()]
            .iter()
            .zip([tp.endpoints.0, tp.endpoints.1].iter())
        {
            if flag {
                if let Some(&p) = end {
                    let (x, y) = tx(p);
                    svg.push_str(&format!("<circle cx='{x:.1}' cy='{y:.1}' r='2' fill='#ff4d4d'/>\n"));
                }
            }
        }
    }
    svg.push_str(&format!(
        "<text x='{:.0}' y='270' fill='#ccc' font-family='monospace' font-size='13'>{name}</text>\n",
        ox
    ));
}

// --------------------------------------------------------------------------
// synthetic gap slivers (mm)
// --------------------------------------------------------------------------

fn ring(pts: &[(f64, f64)]) -> Polygons {
    let mut p = Polygons::new();
    p.push(Contour::new(pts.iter().map(|&(x, y)| Point::from_mm(x, y)).collect()));
    p
}

/// A long wedge: 0.8 mm wide at the left, tapering to 0.2 mm at the right.
fn taper() -> Polygons {
    ring(&[(0.0, -0.4), (9.0, -0.1), (9.0, 0.1), (0.0, 0.4)])
}

/// Two ~0.5 mm strips meeting at a right angle.
fn corner_l() -> Polygons {
    ring(&[(0.0, 0.0), (6.0, 0.0), (6.0, 0.5), (0.5, 0.5), (0.5, 6.0), (0.0, 6.0)])
}

/// A plus: four ~0.5 mm arms around a central 4-way junction.
fn plus() -> Polygons {
    let a = 0.25; // half width
    let l = 3.0; // arm length
    ring(&[
        (-a, -l), (a, -l), (a, -a), (l, -a), (l, a), (a, a),
        (a, l), (-a, l), (-a, a), (-l, a), (-l, -a), (-a, -a),
    ])
}

/// A curved crescent: a ~0.5 mm wide annulus sector (tests curve following).
fn crescent() -> Polygons {
    let (r_out, r_in) = (5.0, 4.5);
    let n = 24;
    let mut pts = Vec::new();
    for i in 0..=n {
        let t = std::f64::consts::PI * (i as f64 / n as f64); // 0..180°
        pts.push((r_out * t.cos(), r_out * t.sin()));
    }
    for i in (0..=n).rev() {
        let t = std::f64::consts::PI * (i as f64 / n as f64);
        pts.push((r_in * t.cos(), r_in * t.sin()));
    }
    ring(&pts)
}
