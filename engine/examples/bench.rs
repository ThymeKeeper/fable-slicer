//! Phase-by-phase timing for the slicing pipeline.
//!
//!   cargo run --release -p engine --example bench -- fixtures/benchy.stl [--support arc]

use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args.first().map(String::as_str).unwrap_or("fixtures/benchy.stl");
    let mut settings = config::Settings::default();
    if args.iter().any(|a| a == "--support") {
        if let Some(i) = args.iter().position(|a| a == "--support") {
            settings.support_mode = config::SupportMode::parse(&args[i + 1]).expect("support mode");
        }
    }

    let t0 = Instant::now();
    let mesh = mesh::Mesh::load_stl(path).expect("load STL");
    let t_load = t0.elapsed();

    let t1 = Instant::now();
    let layers = engine::slice_mesh(
        &mesh,
        engine::SliceParams {
            layer_height_mm: settings.layer_height_mm,
            first_layer_height_mm: settings.first_layer_height_mm,
        },
    );
    let t_slice = t1.elapsed();

    let t2 = Instant::now();
    let plans = engine::generate(&mesh, &settings);
    let t_plan = t2.elapsed(); // includes a re-slice; subtract t_slice for planning-only

    let t3 = Instant::now();
    let gcode = engine::to_gcode(&plans, &settings);
    let t_gcode = t3.elapsed();

    println!(
        "{}: {} tris, {} layers, {} paths, {} gcode lines",
        path,
        mesh.triangles.len(),
        layers.len(),
        plans.iter().map(|l| l.paths.len()).sum::<usize>(),
        gcode.lines().count()
    );
    println!("load:   {:>8.1?}", t_load);
    println!("slice:  {:>8.1?}", t_slice);
    println!("plan:   {:>8.1?}  (incl. slice; planning-only ≈ {:.1?})", t_plan, t_plan.saturating_sub(t_slice));
    println!("gcode:  {:>8.1?}", t_gcode);
    println!("total:  {:>8.1?}", t_load + t_plan + t_gcode);
}
