//! How does grid support meet the bed and the brim? Slices a model with the
//! GUI's frame (auto_center off, part centered on the bed) and reports the
//! support base's bed contact on layer 0 and how much support is printed over
//! brim beads on any layer. `DUMP=path` also writes the first layers' paths
//! for plotting.
//!
//! `cargo run --release -p engine --example support_base_probe -- MODEL.stl [filament] [brim]`
use engine::PathKind;
use geo2d::{difference, intersection, offset, union, Polygons};

/// The printed footprint of a set of paths: each bead swept to its width.
fn footprint(paths: &[&engine::ToolPath]) -> Polygons {
    let mut acc = Polygons::new();
    for p in paths {
        let mut pts = p.points.clone();
        if p.closed {
            pts.push(pts[0]);
        }
        acc = union(&acc, &geo2d::stroke_open(&pts, p.width_mm * 0.5));
    }
    acc
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let model = args.get(1).expect("model path");
    let filament = args.get(2).map(String::as_str).unwrap_or("asa");
    let brim: usize = args.get(3).and_then(|v| v.parse().ok()).unwrap_or(10);

    let mut profiles = config::Profiles::builtin();
    let _ = profiles.load_user_profiles(None);
    let mut s = profiles.resolve("sovol-zero-custom", filament, "sovol-zero-custom").unwrap();
    s.support_mode = config::SupportMode::Grid;
    s.brim_loops = brim;
    s.skirt_loops = 0;
    s.auto_center_on_bed = false;

    let raw = mesh::Mesh::load_stl(model).unwrap();
    // The GUI drops a part onto the bed and centers it (X/Y 76.2 on the Zero).
    let (minx, miny, maxx, maxy) = raw.xy_bounds().unwrap();
    let (minz, _) = raw.z_bounds().unwrap();
    let m = raw.transformed(&mesh::Transform {
        translation: [
            s.bed_size_x_mm / 2.0 - (minx + maxx) / 2.0,
            s.bed_size_y_mm / 2.0 - (miny + maxy) / 2.0,
            -minz,
        ],
        ..Default::default()
    });

    let t0 = std::time::Instant::now();
    let layers = engine::generate(&m, &s);
    println!("sliced {} layers in {} ms  (filament {filament}, brim {brim})", layers.len(), t0.elapsed().as_millis());

    let l0 = &layers[0];
    let brim_paths: Vec<_> = l0.paths.iter().filter(|p| p.kind == PathKind::Skirt).collect();
    let sup0: Vec<_> = l0.paths.iter().filter(|p| p.kind == PathKind::Support).collect();
    let part0: Vec<_> = l0.paths.iter().filter(|p| !matches!(p.kind, PathKind::Skirt | PathKind::Support)).collect();
    let brim_fp = footprint(&brim_paths);
    let sup_fp = footprint(&sup0);
    let part_fp = footprint(&part0);
    // The support's footprint REGION (lines + the gaps between them): close the
    // bead union over the fill spacing.
    let sup_region = offset(&offset(&sup_fp, 2.0), -2.0);
    println!(
        "layer 0: part {:.0} mm², brim {:.0} mm² ({} loops), support {} paths → bead area {:.0} mm² inside a {:.0} mm² footprint ({:.0}% contact)",
        part_fp.net_area_mm2(),
        brim_fp.net_area_mm2(),
        brim_paths.len(),
        sup0.len(),
        sup_fp.net_area_mm2(),
        sup_region.net_area_mm2(),
        100.0 * sup_fp.net_area_mm2() / sup_region.net_area_mm2().max(1e-9),
    );
    println!(
        "layer 0: support beads laid ON brim beads (double extrusion): {:.1} mm²",
        intersection(&brim_fp, &sup_fp).net_area_mm2()
    );
    // Support stacked over the brim on any layer above — the column fused to it.
    // Over the brim in plan view but above part material (a ledge between the
    // two) it rests on the part instead, so the part's shadow is excluded.
    let (mut over, mut standing) = ((0.0f64, 0usize), (0.0f64, 0usize));
    let mut shadow = Polygons::new();
    for (i, layer) in layers.iter().enumerate().skip(1) {
        shadow = union(&shadow, &layers[i - 1].outline);
        let sup: Vec<_> = layer.paths.iter().filter(|p| p.kind == PathKind::Support).collect();
        if !sup.is_empty() {
            let above = intersection(&footprint(&sup), &brim_fp);
            let a = above.net_area_mm2();
            let s = difference(&above, &shadow).net_area_mm2();
            if a > over.0 {
                over = (a, i);
            }
            if s > standing.0 {
                standing = (s, i);
            }
        }
    }
    println!(
        "layers 1+: support over brim beads {:.1} mm² at most (layer {}); standing on the brim (no part between) {:.1} mm² (layer {})",
        over.0, over.1, standing.0, standing.1
    );
    let total_support: usize =
        layers.iter().map(|l| l.paths.iter().filter(|p| p.kind == PathKind::Support).count()).sum();
    let top = layers.iter().rposition(|l| l.paths.iter().any(|p| p.kind == PathKind::Support));
    println!("support: {total_support} paths, topmost support layer {top:?}");
    if let Ok(path) = std::env::var("DUMP") {
        dump(&layers, &path, 3);
    }
}

/// Write `layer kind closed x,y x,y …` lines for the first `upto` layers.
fn dump(layers: &[engine::LayerPlan], path: &str, upto: usize) {
    use std::io::Write;
    let mut f = std::fs::File::create(path).unwrap();
    for (i, l) in layers.iter().enumerate().take(upto) {
        for p in &l.paths {
            write!(f, "{i} {:?} {}", p.kind, p.closed as u8).unwrap();
            for q in &p.points {
                write!(f, " {:.3},{:.3}", q.x_mm(), q.y_mm()).unwrap();
            }
            writeln!(f).unwrap();
        }
    }
}
