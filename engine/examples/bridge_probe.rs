//! Per-layer Bridge counts, with real bottom/top shells.
//! Env: BOTTOM, TOP (shell layer counts), WALLS, FOOTHOLD, DETAIL=N, GCODE.
fn main() {
    let stl = std::env::args().nth(1).unwrap_or_else(|| "fixtures/benchy.stl".into());
    let mesh = mesh::Mesh::load_stl(&stl).expect("load stl");
    let mut s = config::Settings::default();
    s.bottom_layers = std::env::var("BOTTOM").ok().and_then(|v| v.parse().ok()).unwrap_or(5);
    s.top_layers = std::env::var("TOP").ok().and_then(|v| v.parse().ok()).unwrap_or(5);
    if let Some(w) = std::env::var("WALLS").ok().and_then(|v| v.parse().ok()) { s.wall_count = w; }
    if let Some(d) = std::env::var("DENSITY").ok().and_then(|v| v.parse().ok()) { s.infill_density = d; }
    if let Some(h) = std::env::var("LH").ok().and_then(|v| v.parse().ok()) { s.layer_height_mm = h; }
    if let Some(f) = std::env::var("FOOTHOLD").ok().and_then(|v| v.parse().ok()) { s.bridge_foothold_mm = f; }
    if std::env::var("OUTER_FIRST").is_ok() { s.outer_wall_first = true; }
    if let Some(f) = std::env::var("FAN").ok().and_then(|v| v.parse().ok()) { s.fan_speed = f; }
    if std::env::var("SUPPORT").is_ok() { s.support_mode = config::SupportMode::Grid; }
    if let Some(p) = std::env::var("PATTERN").ok().and_then(|v| config::InfillPattern::parse(&v)) {
        s.bottom_pattern = p;
        s.solid_pattern = p;
    }
    let layers = engine::generate(&mesh, &s);
    if std::env::var("GCODE").is_ok() {
        print!("{}", engine::to_gcode(&layers, &s));
        return;
    }
    if let Some(n) = std::env::var("DETAIL").ok().and_then(|v| v.parse::<usize>().ok()) {
        let l = &layers[n - 1];
        println!("L{n} z={:.1} — paths by kind (count, total mm):", l.print_z_mm);
        let mut kinds: std::collections::BTreeMap<String, (usize, f64)> = std::collections::BTreeMap::new();
        for p in &l.paths {
            let len: f64 = p.points.windows(2)
                .map(|w| (w[1].x_mm() - w[0].x_mm()).hypot(w[1].y_mm() - w[0].y_mm()))
                .sum();
            let e = kinds.entry(format!("{:?}", p.kind)).or_default();
            e.0 += 1;
            e.1 += len;
        }
        for (k, (n, mm)) in &kinds {
            println!("  {k:<18} {n:>4}  {mm:>7.1}mm");
        }
        for kind in [engine::PathKind::Solid, engine::PathKind::TopSkin, engine::PathKind::BottomSkin] {
            let (mut loops, mut lines) = (0, 0);
            for p in l.paths.iter().filter(|p| p.kind == kind) {
                if p.closed { loops += 1 } else { lines += 1 }
            }
            if loops + lines > 0 {
                println!("  {:?}: {loops} closed loops, {lines} open lines", kind);
            }
        }
        // Over-extrusion: deposited bead volume vs region volume. ~1.0 balanced;
        // >1.0 = beads overlapping (walls + fill in the same space).
        let (area, _) = engine::debug_uncovered(l, s.line_width_mm);
        let mut vol = 0.0;
        for p in &l.paths {
            if matches!(p.kind, engine::PathKind::Skirt | engine::PathKind::Support) || p.points.len() < 2 {
                continue;
            }
            let bh = l.height_mm * p.height_scale;
            let segw = |k: usize| p.widths.as_ref().map_or(p.width_mm, |ws| (ws[k] + ws[k + 1]) * 0.5);
            for k in 0..p.points.len() - 1 {
                let len = (p.points[k + 1].x_mm() - p.points[k].x_mm()).hypot(p.points[k + 1].y_mm() - p.points[k].y_mm());
                vol += config::bead_area_mm2(segw(k), bh) * len;
            }
            if p.closed {
                let (a, b) = (p.points[p.points.len() - 1], p.points[0]);
                let len = (b.x_mm() - a.x_mm()).hypot(b.y_mm() - a.y_mm());
                vol += config::bead_area_mm2(p.width_mm, bh) * len;
            }
        }
        println!("  over-extrusion (deposited/region): {:.3}  (>1.05 = real overlap)", vol / (area * l.height_mm).max(1e-9));
        // Actual bead overlap: stamp wall beads and fill beads onto a 0.04mm grid,
        // count cells covered by BOTH. A clean kiss leaves only a ~1-cell seam along
        // the shared boundary; real overlap stacks many cells deep.
        let cell = std::env::var("CELL").ok().and_then(|v| v.parse().ok()).unwrap_or(0.04_f64);
        let stamp = |kinds: &[engine::PathKind]| -> std::collections::HashSet<(i32, i32)> {
            let mut set = std::collections::HashSet::new();
            for p in l.paths.iter().filter(|p| kinds.contains(&p.kind)) {
                let r = p.width_mm * 0.5;
                let ri = (r / cell).ceil() as i32;
                let mut pts = p.points.clone();
                if p.closed {
                    pts.push(p.points[0]);
                }
                for w in pts.windows(2) {
                    let (a, b) = (w[0], w[1]);
                    let len = (b.x_mm() - a.x_mm()).hypot(b.y_mm() - a.y_mm()).max(1e-6);
                    let steps = (len / (cell * 0.5)).ceil() as i32;
                    for s in 0..=steps {
                        let t = s as f64 / steps as f64;
                        let (cx, cy) = (a.x_mm() + (b.x_mm() - a.x_mm()) * t, a.y_mm() + (b.y_mm() - a.y_mm()) * t);
                        let (gx, gy) = ((cx / cell) as i32, (cy / cell) as i32);
                        for dx in -ri..=ri {
                            for dy in -ri..=ri {
                                if ((dx * dx + dy * dy) as f64) * cell * cell <= r * r {
                                    set.insert((gx + dx, gy + dy));
                                }
                            }
                        }
                    }
                }
            }
            set
        };
        use engine::PathKind::*;
        let walls = stamp(&[ExternalPerimeter, Perimeter]);
        let solid = stamp(&[Solid, TopSkin, BottomSkin]);
        let sparse = stamp(&[Infill]);
        let ca = cell * cell;
        let inter = |a: &std::collections::HashSet<(i32, i32)>, b: &std::collections::HashSet<(i32, i32)>| {
            a.iter().filter(|c| b.contains(c)).count() as f64 * ca
        };
        println!(
            "  BEAD overlap walls∩solid = {:.2} mm²   walls∩sparse = {:.2} mm²  (kiss ≈ boundary×{cell})",
            inter(&walls, &solid),
            inter(&walls, &sparse)
        );
        // Direct gap: dilate the solid outward cell-by-cell; print how many wall cells
        // it touches at each distance. If it only touches the walls after dilating
        // 0.1mm+, that's the paper-width gap.
        let mut sd = solid.clone();
        print!("  GAP solid→wall, wall-cells touched at dilation: 0mm={}", solid.iter().filter(|c| walls.contains(c)).count());
        for step in 1..=8 {
            let mut next = std::collections::HashSet::with_capacity(sd.len() * 2);
            for &(x, y) in &sd {
                next.insert((x, y));
                next.insert((x + 1, y));
                next.insert((x - 1, y));
                next.insert((x, y + 1));
                next.insert((x, y - 1));
            }
            sd = next;
            print!("  {:.2}mm={}", step as f64 * cell, sd.iter().filter(|c| walls.contains(c)).count());
        }
        println!();
        let mut bl: Vec<f64> = l.paths.iter()
            .filter(|p| p.kind == engine::PathKind::Bridge)
            .map(|p| p.points.windows(2).map(|w| (w[1].x_mm()-w[0].x_mm()).hypot(w[1].y_mm()-w[0].y_mm())).sum())
            .collect();
        bl.sort_by(|a, b| a.partial_cmp(b).unwrap());
        if !bl.is_empty() {
            let n = bl.len();
            println!("  bridge strand lengths (mm): min {:.1}, median {:.1}, max {:.1}", bl[0], bl[n/2], bl[n-1]);
        }
        return;
    }
    let c = |l: &engine::LayerPlan, k: engine::PathKind| l.paths.iter().filter(|p| p.kind == k).count();
    let mut tot = 0;
    for (i, l) in layers.iter().enumerate() {
        let b = c(l, engine::PathKind::Bridge);
        tot += b;
        if b > 0 {
            println!("L{:<3} z={:5.1}  bridge={b}", i + 1, l.print_z_mm);
        }
    }
    println!("total Bridge paths across model: {tot}");
}
