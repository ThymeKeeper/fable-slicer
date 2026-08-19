//! Why a mirrored bead comes out too fat: dump the runs of a layer with the
//! filament and length they were reconstructed from.
//!
//! Usage: cargo run -p engine --example mirror_widths -- <file.gcode> [layer]

fn main() {
    let path = std::env::args().nth(1).expect("gcode path");
    let want: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let bytes = std::fs::read(&path).expect("read");
    let tl = gcode::Timeline::parse(&bytes);
    let first = tl.layers[want].first_move as usize;
    let last = tl.layers.get(want + 1).map(|l| l.first_move as usize).unwrap_or(tl.moves.len());
    let mut prev = if first == 0 { tl.start } else { tl.moves[first - 1].to };
    let (mut e, mut len, mut n) = (0.0f64, 0.0f64, 0usize);
    let mut runs: Vec<(f64, f64, usize)> = Vec::new();
    for m in &tl.moves[first..last] {
        let d = ((m.to[0] - prev[0]).powi(2) + (m.to[1] - prev[1]).powi(2)).sqrt() as f64;
        if m.extruding {
            e += m.e_mm as f64;
            len += d;
            n += 1;
        } else if n > 0 {
            runs.push((e, len, n));
            e = 0.0;
            len = 0.0;
            n = 0;
        }
        prev = m.to;
    }
    if n > 0 {
        runs.push((e, len, n));
    }
    let area = std::f64::consts::PI * (1.75f64 / 2.0).powi(2);
    let h = if want == 0 { tl.layers[0].z as f64 } else { (tl.layers[want].z - tl.layers[want - 1].z) as f64 };
    println!("layer {want}: height {h:.3} mm, {} runs", runs.len());
    // The worst run's own moves, with the file offsets that produced them.
    {
        let mut prev = if first == 0 { tl.start } else { tl.moves[first - 1].to };
        let mut best: (f64, usize, usize) = (0.0, 0, 0);
        let (mut e, mut len, mut start) = (0.0f64, 0.0f64, first);
        for (i, m) in tl.moves[first..last].iter().enumerate() {
            let d = ((m.to[0] - prev[0]).powi(2) + (m.to[1] - prev[1]).powi(2)).sqrt() as f64;
            if m.extruding {
                if len == 0.0 { start = first + i; }
                e += m.e_mm as f64;
                len += d;
            } else if len > 0.0 {
                if e / len > best.0 { best = (e / len, start, first + i); }
                e = 0.0; len = 0.0;
            }
            prev = m.to;
        }
        println!("  worst run: moves {}..{} (e/mm {:.3})", best.1, best.2, best.0);
        for i in best.1..(best.1 + 6).min(best.2) {
            let m = tl.moves[i];
            println!("    move {i}: to {:?} e {:.5} byte {}", m.to, m.e_mm, m.at_byte);
        }
    }
    runs.sort_by(|a, b| (b.0 / b.1.max(1e-9)).partial_cmp(&(a.0 / a.1.max(1e-9))).unwrap());
    for (e, len, n) in runs.iter().take(8) {
        println!(
            "  e {e:8.3} mm over {len:8.1} mm in {n:5} moves  -> e/mm {:.4}  width {:.2} mm",
            e / len.max(1e-9),
            e * area / len.max(1e-9) / h
        );
    }
}
