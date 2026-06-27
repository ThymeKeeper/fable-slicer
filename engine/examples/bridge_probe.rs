//! Per-layer Bridge / ArcOverhang counts, with real bottom/top shells.
//! Env: BOTTOM, TOP (shell layer counts), ARC (use arc support mode).
fn main() {
    let stl = std::env::args().nth(1).unwrap_or_else(|| "fixtures/benchy.stl".into());
    let mesh = mesh::Mesh::load_stl(&stl).expect("load stl");
    let mut s = config::Settings::default();
    s.bottom_layers = std::env::var("BOTTOM").ok().and_then(|v| v.parse().ok()).unwrap_or(5);
    s.top_layers = std::env::var("TOP").ok().and_then(|v| v.parse().ok()).unwrap_or(5);
    if std::env::var("ARC").is_ok() {
        s.support_mode = config::SupportMode::Arc;
    }
    let layers = engine::generate(&mesh, &s);
    let c = |l: &engine::LayerPlan, k: engine::PathKind| l.paths.iter().filter(|p| p.kind == k).count();
    let mut tot = 0;
    for (i, l) in layers.iter().enumerate() {
        let (b, a) = (c(l, engine::PathKind::Bridge), c(l, engine::PathKind::ArcOverhang));
        tot += b;
        if b > 0 || a > 0 {
            println!("L{:<3} z={:5.1}  bridge={b}  arc={a}", i + 1, l.print_z_mm);
        }
    }
    println!("total Bridge paths across model: {tot}");
}
