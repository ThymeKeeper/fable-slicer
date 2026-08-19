//! Parse a g-code file into its motion timeline and report what it found —
//! the check that the parser survives real files from other slicers.
//!
//! Usage: cargo run -p gcode --example timeline_dump -- <file.gcode>

fn main() {
    let path = std::env::args().nth(1).expect("usage: timeline_dump <file.gcode>");
    let bytes = std::fs::read(&path).expect("read");
    let t0 = std::time::Instant::now();
    let tl = gcode::Timeline::parse(&bytes);
    let secs = t0.elapsed().as_secs_f64();
    let ext = tl.moves.iter().filter(|m| m.extruding).count();
    let h = (tl.seconds / 3600.0) as u32;
    let m = ((tl.seconds - h as f32 * 3600.0) / 60.0) as u32;
    println!(
        "{path}\n  {:.1} MB parsed in {secs:.2}s | {} moves ({ext} extruding) | {} layers | {h}h{m:02}m",
        bytes.len() as f64 / 1.0e6,
        tl.moves.len(),
        tl.layers.len(),
    );
    if let (Some(f), Some(l)) = (tl.layers.first(), tl.layers.last()) {
        println!("  z {:.2} … {:.2}", f.z, l.z);
    }
    let mid = tl.at(tl.seconds * 0.5);
    println!("  halfway: {:?} on layer {}", mid.0, tl.layer_of(mid.1));
}
