//! Inspect the path structure of specific layers of a sliced model — kinds,
//! loop shapes, and what covers a region. Debug tooling for the walls=99
//! hole-roof investigation (corner bracket layer ~279).
//!
//! Usage: cargo run -p engine --example inspect_layer -- <stl> <layer> [layer..]

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("stl path");
    let layers: Vec<usize> = args[2..].iter().map(|s| s.parse().unwrap()).collect();

    let mesh = mesh::Mesh::load_stl(path).expect("load mesh");
    let mut s = config::Settings::default();
    // The user's process: walls=99, 0.2/0.4, top 0 bottom 2, lines fill,
    // outer wall first, sharpest seam.
    s.wall_count = 99;
    s.layer_height_mm = 0.2;
    s.first_layer_height_mm = 0.2;
    s.line_width_mm = 0.4;
    s.top_layers = 1;
    s.bottom_layers = 1;
    s.infill_density = 0.15;
    s.outer_wall_first = false;
    s.seam_mode = config::SeamMode::Sharpest;
    s.skirt_loops = 0;

    let plans = engine::generate(&mesh, &s);
    println!("total layers: {}", plans.len());
    for &li in &layers {
        let l = &plans[li];
        let outline_area: f64 = l
            .outline
            .contours
            .iter()
            .map(|c| if c.is_ccw() { c.area_mm2() } else { -c.area_mm2() })
            .sum();
        let covered: f64 = l
            .paths
            .iter()
            .map(|p| {
                p.points
                    .windows(2)
                    .map(|w| {
                        let dx = w[0].x_mm() - w[1].x_mm();
                        let dy = w[0].y_mm() - w[1].y_mm();
                        (dx * dx + dy * dy).sqrt()
                    })
                    .sum::<f64>()
                    * p.width_mm
            })
            .sum();
        println!(
            "\n=== layer {} z={:.2} paths={} outline={:.0}mm2 covered≈{:.0}mm2 ===",
            li,
            l.print_z_mm,
            l.paths.len(),
            outline_area,
            covered
        );
        let mut by_kind: std::collections::BTreeMap<String, (usize, f64)> = Default::default();
        for p in &l.paths {
            let len: f64 = p
                .points
                .windows(2)
                .map(|w| {
                    let dx = w[0].x_mm() - w[1].x_mm();
                    let dy = w[0].y_mm() - w[1].y_mm();
                    (dx * dx + dy * dy).sqrt()
                })
                .sum();
            let e = by_kind.entry(format!("{:?}", p.kind)).or_insert((0, 0.0));
            e.0 += 1;
            e.1 += len;
        }
        for (k, (n, len)) in &by_kind {
            println!("  {k:<20} n={n:<4} total_len={len:.1}mm");
        }
        // Per-segment overhang annotation: the airborne stretches of wall
        // beads crossing a spanned hollow must be marked (slowed + cooled).
        let mut seg_paths = 0usize;
        let mut air_len = 0.0f64;
        for p in &l.paths {
            let Some(sa) = &p.segs else { continue };
            seg_paths += 1;
            let n_pts = p.points.len();
            for (k, a) in sa.iter().enumerate() {
                if a.overhang >= 0.5 {
                    let q = p.points[k];
                    let r = p.points[(k + 1) % n_pts];
                    let dx = q.x_mm() - r.x_mm();
                    let dy = q.y_mm() - r.y_mm();
                    air_len += (dx * dx + dy * dy).sqrt();
                }
            }
        }
        if seg_paths > 0 {
            println!("  segs: {seg_paths} paths carry per-segment attrs; {air_len:.1}mm at overhang ≥ 0.5");
        }
        // Small closed loops — the "new structures over the hole" suspects:
        // report closed paths with perimeter under 40mm, with their centroid.
        for p in &l.paths {
            if p.closed && p.points.len() >= 3 {
                let perim: f64 = (0..p.points.len())
                    .map(|k| {
                        let a = p.points[k];
                        let b = p.points[(k + 1) % p.points.len()];
                        let dx = a.x_mm() - b.x_mm();
                        let dy = a.y_mm() - b.y_mm();
                        (dx * dx + dy * dy).sqrt()
                    })
                    .sum();
                if perim < 40.0 {
                    let (mut cx, mut cy) = (0.0, 0.0);
                    for pt in &p.points {
                        cx += pt.x_mm();
                        cy += pt.y_mm();
                    }
                    let n = p.points.len() as f64;
                    println!(
                        "  small loop: {:?} perim={:.1}mm centroid=({:.1},{:.1})",
                        p.kind,
                        perim,
                        cx / n,
                        cy / n
                    );
                }
            }
        }
    }
}
