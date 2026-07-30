//! Measure the raw over-air hollows (beyond the overhang allowance) per layer:
//! area, bbox, and max erosion depth (≈ half the widest width). Decides where
//! the "field crosses it side-bonded" absorption threshold should sit.
//!
//! Usage: cargo run -p engine --example hollow_probe -- <stl> <layer> [layer..]

use geo2d::{difference, offset};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("stl path");
    let layers: Vec<usize> = args[2..].iter().map(|s| s.parse().unwrap()).collect();

    let mesh = mesh::Mesh::load_stl(path).expect("load mesh");
    let mut s = config::Settings::default();
    s.wall_count = 99;
    s.layer_height_mm = 0.2;
    s.first_layer_height_mm = 0.2;
    s.line_width_mm = 0.4;
    s.top_layers = 0;
    s.bottom_layers = 0;
    s.infill_density = 0.15;
    s.outer_wall_first = false;
    s.seam_mode = config::SeamMode::Sharpest;
    s.skirt_loops = 0;

    let allowance = s.layer_height_mm * s.support_overhang_angle_deg.to_radians().tan();
    println!("allowance = {allowance:.3}mm");
    let plans = engine::generate(&mesh, &s);

    for &li in &layers {
        assert!(li >= 1);
        let over_air = difference(
            &plans[li].outline,
            &offset(&plans[li - 1].outline, allowance),
        );
        println!("\n=== layer {li} (GUI {}) over-air islands ===", li + 1);
        for isl in engine::debug_islands(&over_air) {
            let area = isl.net_area_mm2();
            if area < 0.05 {
                continue;
            }
            // Bisect the erosion depth at which the island vanishes.
            let (mut lo, mut hi) = (0.0f64, 5.0f64);
            for _ in 0..20 {
                let mid = 0.5 * (lo + hi);
                if offset(&isl, -mid).is_empty() {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            let b = isl.bounds().unwrap();
            println!(
                "  area={area:.2}mm2  max_halfwidth={:.2}mm  bbox {:.1}x{:.1} at ({:.1},{:.1})",
                0.5 * (lo + hi),
                geo2d::to_mm(b.width()),
                geo2d::to_mm(b.height()),
                geo2d::to_mm(b.min.x),
                geo2d::to_mm(b.min.y),
            );
        }
    }
}
