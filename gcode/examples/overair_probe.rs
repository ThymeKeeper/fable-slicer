//! How much of each layer is laid over AIR, and what did the slicer call it?
//!
//! A part printed as pure concentric walls has nothing under a bead except
//! the previous layer's beads. Where a layer reaches out past the one below,
//! the bead is spanning air — and if it was not classified as an overhang or
//! a bridge it goes down at full speed, full flow and base fan, which droops.
//!
//! Rasterises each layer's deposited beads into a grid, then asks of every
//! bead on the next layer how far it is from anything solid beneath it.
//!
//!     cargo run --release -p gcode --example overair_probe -- FILE.gcode [layer_lo] [layer_hi]

use gcode::{Feature, Timeline};
use std::collections::HashSet;

const CELL: f32 = 0.15; // mm

fn key(x: f32, y: f32) -> (i32, i32) {
    ((x / CELL).round() as i32, (y / CELL).round() as i32)
}

/// Every grid cell a layer's extrusions touch, sampled along each bead.
fn deposited(tl: &Timeline, first: usize, last: usize) -> HashSet<(i32, i32)> {
    let mut g = HashSet::new();
    let mut prev = if first == 0 { tl.start } else { tl.moves[first - 1].to };
    for m in &tl.moves[first..last] {
        if m.extruding {
            let d = ((m.to[0] - prev[0]).powi(2) + (m.to[1] - prev[1]).powi(2)).sqrt();
            let n = (d / (CELL * 0.5)).ceil().max(1.0) as usize;
            for k in 0..=n {
                let f = k as f32 / n as f32;
                g.insert(key(prev[0] + (m.to[0] - prev[0]) * f, prev[1] + (m.to[1] - prev[1]) * f));
            }
        }
        prev = m.to;
    }
    g
}

fn main() {
    let path = std::env::args().nth(1).expect("gcode path");
    let lo: usize = std::env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(1);
    let hi: usize = std::env::args().nth(3).map(|s| s.parse().unwrap()).unwrap_or(usize::MAX);
    let tl = Timeline::parse(&std::fs::read(&path).unwrap());
    let bounds = |li: usize| {
        let a = tl.layers[li].first_move as usize;
        let b = tl.layers.get(li + 1).map(|n| n.first_move as usize).unwrap_or(tl.moves.len());
        (a, b)
    };
    // "Supported" means within one bead of solid: the cell itself or a
    // neighbour, since a bead half-overlapping the one below still has a
    // shoulder to sit on.
    let supported = |g: &HashSet<(i32, i32)>, x: f32, y: f32| {
        let (cx, cy) = key(x, y);
        (-2..=2).any(|dx| (-2..=2).any(|dy| g.contains(&(cx + dx, cy + dy))))
    };

    // The other way material lands in a void: a TRAVEL taken without a
    // retraction, crossing open space. It oozes the whole way, and over many
    // layers those threads accumulate into a web across an opening.
    println!("layer   over-air mm   of total mm   worst feature (mm over air)");
    let hi = hi.min(tl.layers.len().saturating_sub(1));
    let mut worst: Vec<(f32, usize, Feature)> = Vec::new();
    for li in lo..=hi {
        let below = {
            let (a, b) = bounds(li - 1);
            deposited(&tl, a, b)
        };
        let (a, b) = bounds(li);
        let mut prev = if a == 0 { tl.start } else { tl.moves[a - 1].to };
        let (mut air, mut total) = (0.0f32, 0.0f32);
        let mut by_feat: Vec<(Feature, f32)> = Vec::new();
        for m in &tl.moves[a..b] {
            if m.extruding {
                let d = ((m.to[0] - prev[0]).powi(2) + (m.to[1] - prev[1]).powi(2)).sqrt();
                total += d;
                let n = (d / (CELL * 0.5)).ceil().max(1.0) as usize;
                let mut unsup = 0usize;
                for k in 0..=n {
                    let f = k as f32 / n as f32;
                    let (x, y) =
                        (prev[0] + (m.to[0] - prev[0]) * f, prev[1] + (m.to[1] - prev[1]) * f);
                    if !supported(&below, x, y) {
                        unsup += 1;
                    }
                }
                let frac = unsup as f32 / (n + 1) as f32;
                air += d * frac;
                match by_feat.iter_mut().find(|(f, _)| *f == m.feature) {
                    Some((_, v)) => *v += d * frac,
                    None => by_feat.push((m.feature, d * frac)),
                }
            }
            prev = m.to;
        }
        let mut fs: Vec<_> = by_feat.into_iter().filter(|(_, v)| *v > 0.5).collect();
        fs.sort_by(|x, y| y.1.partial_cmp(&x.1).unwrap());
        if air > 1.0 {
            let head = fs
                .iter()
                .take(3)
                .map(|(f, v)| format!("{f:?} {v:.1}"))
                .collect::<Vec<_>>()
                .join(", ");
            println!("{li:5}   {air:9.1}   {total:9.1}    {head}");
        }
        if let Some((f, v)) = fs.first() {
            worst.push((*v, li, *f));
        }
    }
    worst.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    println!("\nworst layers by over-air distance in one feature:");
    for (v, li, f) in worst.iter().take(10) {
        println!("  layer {li:3}  {v:7.1} mm of {f:?} over air");
    }

    // --- unretracted travel across open space ---
    println!("\nlayer   dry-travel-over-air mm   (of all dry travel)   longest single");
    let mut tot_air = 0.0f32;
    let mut rows: Vec<(f32, usize, f32)> = Vec::new();
    for li in lo..=hi {
        let here = {
            let (a, b) = bounds(li);
            deposited(&tl, a, b)
        };
        let (a, b) = bounds(li);
        let mut prev = if a == 0 { tl.start } else { tl.moves[a - 1].to };
        let mut retracted = false;
        let (mut air, mut dry, mut longest) = (0.0f32, 0.0f32, 0.0f32);
        for m in &tl.moves[a..b] {
            let d = ((m.to[0] - prev[0]).powi(2) + (m.to[1] - prev[1]).powi(2)).sqrt();
            if m.extruding {
                retracted = false;
            } else if d < 1.0e-4 {
                if m.e_mm < -1.0e-6 {
                    retracted = true;
                }
            } else if d >= 1.0 && !retracted {
                dry += d;
                let n = (d / (CELL * 0.5)).ceil().max(1.0) as usize;
                let mut over = 0usize;
                for k in 0..=n {
                    let f = k as f32 / n as f32;
                    let (x, y) =
                        (prev[0] + (m.to[0] - prev[0]) * f, prev[1] + (m.to[1] - prev[1]) * f);
                    if !supported(&here, x, y) {
                        over += 1;
                    }
                }
                let seg = d * over as f32 / (n + 1) as f32;
                air += seg;
                longest = longest.max(seg);
            }
            prev = m.to;
        }
        tot_air += air;
        if air > 0.5 {
            rows.push((air, li, longest));
            println!("{li:5}   {air:14.1}   {dry:16.1}   {longest:10.1}");
        }
    }
    rows.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    println!("\ntotal unretracted travel over open space, layers {lo}..{hi}: {tot_air:.0} mm");
}
