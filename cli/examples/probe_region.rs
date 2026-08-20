//! Region probe (scratch tool): slice a model at the user's profiles, find the
//! layer nearest a Z, and report every wall path crossing a bbox — width
//! profile, flags, and per-vertex clearance to the nearest other wall — plus
//! the coverage oracle's voids clipped to the bbox (GRID map to stderr).
//!
//! Usage: probe_region <model> <z_mm> <x0> <y0> <x1> <y1>

use anyhow::{anyhow, Context, Result};
use config::Profiles;
use geo2d::{Contour, Point, Polygons};

fn main() -> Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let model = a.get(1).context("usage: probe_region <model> <z> <x0> <y0> <x1> <y1>")?;
    let z: f64 = a.get(2).context("z")?.parse()?;
    let (x0, y0, x1, y1): (f64, f64, f64, f64) = (
        a.get(3).context("x0")?.parse()?,
        a.get(4).context("y0")?.parse()?,
        a.get(5).context("x1")?.parse()?,
        a.get(6).context("y1")?.parse()?,
    );

    let mut profiles = Profiles::builtin();
    profiles.load_user_profiles(None).map_err(|e| anyhow!(e))?;
    let settings = profiles
        .resolve_tools("sovol-zero-custom", &["petg-custom"], "sovol-zero-custom")
        .map_err(|e| anyhow!(e))?;

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
    let l = layers
        .iter()
        .min_by(|p, q| {
            (p.print_z_mm - z).abs().total_cmp(&(q.print_z_mm - z).abs())
        })
        .context("no layers")?;
    println!("layer index {} print_z {:.3}", l.index, l.print_z_mm);

    let inb = |p: Point| {
        let (x, y) = (p.x_mm(), p.y_mm());
        x >= x0 && x <= x1 && y >= y0 && y <= y1
    };

    // Support test: is a point over the previous layer's solid outline?
    let below = layers
        .iter()
        .find(|q| q.index + 1 == l.index)
        .map(|q| q.outline.clone())
        .unwrap_or_default();
    let supported = |p: Point| below.contours.iter().filter(|c| c.contains(p)).count() % 2 == 1;

    // Collect every wall path's segments for clearance probing.
    let mut segs: Vec<(usize, (f64, f64), (f64, f64))> = Vec::new();
    for (i, p) in l.paths.iter().enumerate() {
        let n = p.points.len();
        let last = if p.closed { n } else { n.saturating_sub(1) };
        for k in 0..last {
            let (a, b) = (p.points[k], p.points[(k + 1) % n]);
            segs.push((i, (a.x_mm(), a.y_mm()), (b.x_mm(), b.y_mm())));
        }
    }
    let seg_dist = |q: (f64, f64), a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let t = if len2 <= 0.0 { 0.0 } else { ((q.0 - a.0) * dx + (q.1 - a.1) * dy) / len2 };
        let t = t.clamp(0.0, 1.0);
        (q.0 - (a.0 + dx * t)).hypot(q.1 - (a.1 + dy * t))
    };

    for (i, p) in l.paths.iter().enumerate() {
        if !p.points.iter().any(|&q| inb(q)) {
            continue;
        }
        let ws = p.widths.clone().unwrap_or_else(|| vec![p.width_mm; p.points.len()]);
        let sel: Vec<usize> = (0..p.points.len()).filter(|&k| inb(p.points[k])).collect();
        let (mut wmin, mut wmax, mut wsum) = (f64::MAX, f64::MIN, 0.0);
        for &k in &sel {
            wmin = wmin.min(ws[k]);
            wmax = wmax.max(ws[k]);
            wsum += ws[k];
        }
        let ends = if p.closed {
            "closed".to_string()
        } else {
            let a = p.points[0];
            let b = *p.points.last().unwrap();
            format!(
                "ends ({:.2},{:.2}){} .. ({:.2},{:.2}){}",
                a.x_mm(),
                a.y_mm(),
                if supported(a) { "S" } else { "!AIR" },
                b.x_mm(),
                b.y_mm(),
                if supported(b) { "S" } else { "!AIR" }
            )
        };
        println!(
            "path {i}: {:?} closed={} widths={} segs={} n={} | in-bbox {} pts, w [{:.3} .. {:.3}] mean {:.3} | {ends}",
            p.kind,
            p.closed,
            p.widths.is_some(),
            p.segs.is_some(),
            p.points.len(),
            sel.len(),
            wmin,
            wmax,
            wsum / sel.len().max(1) as f64
        );
        // Per-vertex clearance to the nearest OTHER path (centerline pitch).
        for &k in sel.iter().step_by(2) {
            let q = (p.points[k].x_mm(), p.points[k].y_mm());
            let mut best = f64::INFINITY;
            let mut best_own = f64::INFINITY;
            for &(j, a, b) in &segs {
                let d = seg_dist(q, a, b);
                if j == i {
                    // Skip the vertex's own neighbourhood (adjacent segs).
                    if d > 0.05 {
                        best_own = best_own.min(d);
                    }
                } else {
                    best = best.min(d);
                }
            }
            println!(
                "   k {k:4} ({:6.2},{:6.2}) w {:.3} | pitch other {:6.3} self {:6.3} | {}",
                q.0,
                q.1,
                ws[k],
                best,
                best_own,
                if supported(p.points[k]) { "S" } else { "AIR" }
            );
        }
    }

    // Coverage voids clipped to the bbox, with an ASCII map.
    let rect = Polygons {
        contours: vec![Contour::new(vec![
            Point::from_mm(x0, y0),
            Point::from_mm(x1, y0),
            Point::from_mm(x1, y1),
            Point::from_mm(x1.min(x0) + (x1 - x0), y1), // placeholder, replaced below
        ])],
    };
    let _ = rect;
    let rect = Polygons {
        contours: vec![Contour::new(vec![
            Point::from_mm(x0, y0),
            Point::from_mm(x1, y0),
            Point::from_mm(x1, y1),
            Point::from_mm(x0, y1),
        ])],
    };
    let mut clip = l.clone();
    clip.outline = geo2d::intersection(&l.outline, &rect);
    std::env::set_var("GRID", "1");
    let (area, unc) = engine::debug_uncovered(&clip, settings.line_width_mm);
    std::env::remove_var("GRID");
    println!("bbox outline {area:.2} mm2, uncovered {unc:.3} mm2");
    Ok(())
}
