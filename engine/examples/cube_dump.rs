//! Dump the calibration test cube's g-code with the REAL resolved profiles
//! (builtin + user dir) — what the Filament panel's "print test cube" button
//! sends, for inspection outside the GUI.
//!
//! Usage: cargo run -p engine --example cube_dump -- <printer> <filament> <process> <out.gcode>

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let [_, printer, filament, process, out] = &args[..] else {
        eprintln!("usage: cube_dump <printer> <filament> <process> <out.gcode>");
        std::process::exit(2);
    };
    let mut lib = config::Profiles::builtin();
    lib.load_user_profiles(None).expect("user profiles");
    let s = lib.resolve(printer, filament, process).expect("resolve");
    println!(
        "{filament}: flow={:.3} pa={:.4} nozzle={} bed={} lw={} lh={}",
        s.extrusion_multiplier,
        s.pressure_advance,
        s.nozzle_temp_c,
        s.bed_temp_c,
        s.line_width_mm,
        s.layer_height_mm,
    );
    let g = engine::test_cube_gcode(&s, 0);
    std::fs::write(out, &g).expect("write gcode");
    println!("wrote {out}: {} bytes", g.len());
}
