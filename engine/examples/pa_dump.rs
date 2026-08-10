//! Dump calibration g-code to stdout (harness/inspection aid).
//! `pa_dump` for the PA tower, `pa_dump flow` for the flow comb.
fn main() {
    let s = config::Settings::default();
    let flow = std::env::args().nth(1).is_some_and(|a| a == "flow");
    print!("{}", if flow { engine::flow_comb_gcode(&s, 0) } else { engine::pa_tower_gcode(&s, 0) });
}
