//! Slice a model with the REAL resolved profiles (builtin + user dir) and
//! write the full g-code — the emit-side twin of `profile_probe`, for
//! diffing what two filament cards actually send the machine.
//!
//! Usage: cargo run -p engine --example gcode_dump -- <stl> <printer> <filament> <process> <out.gcode>

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let [_, path, printer, filament, process, out] = &args[..] else {
        eprintln!("usage: gcode_dump <stl> <printer> <filament> <process> <out.gcode>");
        std::process::exit(2);
    };

    let mut lib = config::Profiles::builtin();
    lib.load_user_profiles(None).expect("user profiles");
    let s = lib.resolve(printer, filament, process).expect("resolve");
    println!(
        "resolved {filament}: nozzle={}/{} bed={} em={:.3} pa={} flowcap={} fan={}/{}/{} retract={} restart_extra={} speeds: print={} outer={} solid={}",
        s.first_layer_nozzle_temp_c,
        s.nozzle_temp_c,
        s.bed_temp_c,
        s.extrusion_multiplier,
        s.pressure_advance,
        s.max_volumetric_speed_mm3_s,
        s.fan_speed,
        s.fan_max,
        s.bridge_fan_speed,
        s.retract_len_mm,
        s.retract_restart_extra_mm,
        s.print_speed_mm_s,
        s.external_perimeter_speed_mm_s,
        s.solid_speed_mm_s,
    );

    let mesh = mesh::Mesh::load_stl(path).expect("load mesh");
    let refs: Vec<(&mesh::Mesh, engine::PartPaint)> = vec![(&mesh, engine::PartPaint::Tool(0))];
    let geo = engine::plan_geometry(&refs, &s);
    let plans = engine::restamp_paint(&geo, &refs);
    let g = engine::to_gcode(&plans, &s);
    std::fs::write(out, &g).expect("write gcode");
    println!("wrote {out}: {} layers, {} bytes", plans.len(), g.len());
}
