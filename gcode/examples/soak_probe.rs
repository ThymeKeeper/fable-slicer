//! Where does a layer get too little time to freeze before the next one lands?
//!
//! A glossy, slumped BAND at one height — not a column, not everywhere — is
//! the signature of heat, and heat is a per-layer budget: the time between a
//! bead being laid and the next bead landing on top of it. A tall thin
//! section prints a layer in a couple of seconds; the same nozzle then comes
//! straight back to the same spot. Fan duty is the other half of the budget.
//!
//! Reports per-layer time and the fan duty in force, so a defect at a known
//! height can be matched to what the machine was actually doing there.
//!
//!     cargo run --release -p gcode --example soak_probe -- FILE.gcode

use gcode::Timeline;

fn main() {
    let path = std::env::args().nth(1).expect("gcode path");
    let src = std::fs::read(&path).unwrap();
    let tl = Timeline::parse(&src);

    // Fan duty is not in the timeline (it is not motion), so read it off the
    // file by byte offset and hold the last M106 seen before each layer.
    let text = String::from_utf8_lossy(&src);
    let mut fan_at: Vec<(u32, f32)> = Vec::new();
    let mut at = 0u32;
    for line in text.split('\n') {
        let l = line.trim();
        if let Some(rest) = l.strip_prefix("M106") {
            let s = rest
                .split_whitespace()
                .find_map(|w| w.strip_prefix('S'))
                .and_then(|v| v.parse::<f32>().ok())
                .unwrap_or(255.0);
            fan_at.push((at, s / 255.0));
        } else if l.starts_with("M107") {
            fan_at.push((at, 0.0));
        }
        at += line.len() as u32 + 1;
    }
    let fan_before = |byte: u32| -> f32 {
        match fan_at.binary_search_by_key(&byte, |f| f.0) {
            Ok(i) => fan_at[i].1,
            Err(0) => 0.0,
            Err(i) => fan_at[i - 1].1,
        }
    };

    let mut rows: Vec<(usize, f32, f32, f32)> = Vec::new(); // layer, z, secs, fan
    for (li, layer) in tl.layers.iter().enumerate() {
        let t0 = if li == 0 {
            0.0
        } else {
            let prev = tl.layers[li - 1].first_move as usize;
            tl.moves[prev].t_end
        };
        let end = tl
            .layers
            .get(li + 1)
            .map(|n| tl.moves[n.first_move as usize].t_end)
            .unwrap_or(tl.seconds);
        rows.push((li, layer.z, end - t0, fan_before(layer.at_byte)));
    }

    let secs: Vec<f32> = rows.iter().map(|r| r.2).collect();
    let mut s2 = secs.clone();
    s2.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let q = |f: f32| s2[((s2.len() as f32 - 1.0) * f) as usize];
    println!(
        "{} layers, per-layer seconds: min {:.1}  p10 {:.1}  p50 {:.1}  p90 {:.1}  max {:.1}",
        rows.len(), s2[0], q(0.10), q(0.5), q(0.9), s2[s2.len() - 1]
    );

    println!("\n  the QUICKEST layers — least time to freeze before the next lands:");
    let mut by_t = rows.clone();
    by_t.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
    for (li, z, s, f) in by_t.iter().take(14) {
        println!("    layer {li:3}  z {z:6.2}   {s:6.1} s   fan {:3.0}%", f * 100.0);
    }

    // A defect reads as a BAND, so look for runs of consecutive quick layers
    // rather than single outliers.
    let thresh = q(0.15).max(1.0);
    println!("\n  runs of consecutive layers under {thresh:.1} s (a band, not a blip):");
    let (mut i, mut found) = (0usize, 0);
    while i < rows.len() {
        if rows[i].2 < thresh {
            let start = i;
            let mut tot = 0.0;
            while i < rows.len() && rows[i].2 < thresh {
                tot += rows[i].2;
                i += 1;
            }
            if i - start >= 3 {
                found += 1;
                println!(
                    "    layers {start}-{}  z {:.2}..{:.2}  ({} layers, {tot:.0} s total, fan {:3.0}%)",
                    i - 1, rows[start].1, rows[i - 1].1, i - start, rows[start].3 * 100.0
                );
            }
        } else {
            i += 1;
        }
    }
    if found == 0 {
        println!("    none — no sustained quick-layer band in this file");
    }
}
