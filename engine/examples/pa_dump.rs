//! Dump calibration g-code to stdout (harness/inspection aid).
//! `pa_dump [flow] [printer filament process]` — with profile names it
//! resolves them like the GUI/CLI (user ~/.config profiles included).
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flow = args.first().is_some_and(|a| a == "flow");
    let names: Vec<&String> = args.iter().skip(if flow { 1 } else { 0 }).collect();
    let s = if names.len() == 3 {
        let mut p = config::Profiles::builtin();
        let _ = p.load_user_profiles(None);
        p.resolve(names[0], names[1], names[2]).expect("profiles resolve")
    } else {
        config::Settings::default()
    };
    print!("{}", if flow { engine::flow_comb_gcode(&s, 0) } else { engine::pa_tower_gcode(&s, 0) });
}
