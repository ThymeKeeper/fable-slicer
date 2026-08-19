//! Which travels cross the part without lifting or retracting?
//!
//! A vertical scar down a wall — smeared, glossy, strung — is the nozzle
//! being dragged across finished surface at print height. That happens when a
//! travel neither retracts (so it oozes) nor hops (so it touches). Holes in a
//! wall are where it happens: the perimeter is cut into arcs and something has
//! to cross the gap.
//!
//! This walks a real file and reports every travel that moved in XY without a
//! retraction in front of it, then finds the XY COLUMNS where such travels
//! repeat layer after layer — which is what turns a per-layer blemish into a
//! stripe you can see from across the room.
//!
//!     cargo run --release -p gcode --example drag_probe -- FILE.gcode [min_mm]

use gcode::Timeline;
use std::collections::HashMap;

fn main() {
    let path = std::env::args().nth(1).expect("gcode path");
    let min_mm: f32 = std::env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(1.0);
    let src = std::fs::read(&path).unwrap();
    let tl = Timeline::parse(&src);
    println!("{} moves, {} layers", tl.moves.len(), tl.layers.len());

    // A DRAG is a non-extruding XY move taken with the nozzle still down at
    // the layer's own Z and no retraction in front of it: a pressurised tip
    // sliding over finished surface. Unretracted is not enough on its own —
    // a hopped travel clears the crowns and only oozes. Neither is retracted
    // enough on its own — a retracted travel that never lifts still rubs.
    let mut prev = tl.start;
    let mut retracted = false;
    let mut lifted = false;
    let mut layer_z = f32::NAN;
    let mut drags: Vec<(usize, [f32; 3], f32)> = Vec::new();
    let (mut safe, mut oozy) = (0u32, 0u32);
    for (i, m) in tl.moves.iter().enumerate() {
        let d = ((m.to[0] - prev[0]).powi(2) + (m.to[1] - prev[1]).powi(2)).sqrt();
        if m.extruding {
            retracted = false;
            lifted = false;
            layer_z = m.to[2];
        } else if d < 1.0e-4 {
            if m.e_mm < -1.0e-6 {
                retracted = true;
            }
            if m.to[2] > layer_z + 0.02 {
                lifted = true;
            }
        } else if d >= min_mm {
            // Judge the height the move is actually taken at, not just the
            // hop flag: a travel that ramps Z as it goes is still clear.
            let up = m.to[2] > layer_z + 0.02 || lifted;
            match (retracted, up) {
                (_, true) => safe += 1,
                (true, false) => oozy += 1,
                (false, false) => drags.push((i, m.to, d)),
            }
        }
        prev = m.to;
    }
    let n = safe + oozy + drags.len() as u32;
    println!(
        "travels >= {min_mm} mm: {safe} lifted clear, {oozy} retracted-but-flat, {} DRAGGING ({:.2}%)",
        drags.len(),
        drags.len() as f32 / n.max(1) as f32 * 100.0
    );
    if drags.is_empty() {
        return;
    }
    let total: f32 = drags.iter().map(|d| d.2).sum();
    println!("  dragged distance: {total:.0} mm total");
    let mut by_len = drags.clone();
    by_len.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
    println!("\n  longest single drags — each one a scar the length of the move:");
    for (i, p, d) in by_len.iter().take(10) {
        let li = tl.layer_of(*i);
        println!(
            "    {d:6.1} mm on layer {li:3} (z {:.2})  ending X {:.1} Y {:.1}",
            tl.layers.get(li).map(|l| l.z).unwrap_or(0.0),
            p[0], p[1]
        );
    }
    // Which layers carry the most dragged distance: a band, not a column.
    let mut per_layer: HashMap<usize, (f32, u32)> = HashMap::new();
    for (i, _, d) in &drags {
        let e = per_layer.entry(tl.layer_of(*i)).or_insert((0.0, 0));
        e.0 += *d;
        e.1 += 1;
    }
    let mut worst: Vec<_> = per_layer.into_iter().collect();
    worst.sort_by(|a, b| b.1 .0.partial_cmp(&a.1 .0).unwrap());
    println!("\n  worst LAYERS by dragged distance:");
    for (li, (mm, n)) in worst.iter().take(10) {
        println!(
            "    layer {li:3} (z {:5.2}): {mm:6.1} mm over {n} drags",
            tl.layers.get(*li).map(|l| l.z).unwrap_or(0.0)
        );
    }

}
