//! Does the Machine view actually REDRAW on the frames it renders?
//!
//! gui/src/main.rs gates `scene.render()` on `RenderSig`, whose only
//! job-derived members are (count, joint_count, current_layer, dim, mask).
//! The nozzle's POSITION is not in that signature — so a frame on which only
//! the nozzle moved is skipped, and the previous texture is re-blitted.
//!
//! `count` is the number of extruding moves completed in the current layer.
//! So the redraw clock is "an extruding move boundary was crossed", not "a
//! frame happened". This walks a real file at a fixed frame period and
//! reports how long the picture is frozen and how far the nozzle teleports
//! when it finally unfreezes.

use gcode::Timeline;

fn q(v: &[f32], f: f32) -> f32 {
    if v.is_empty() {
        return 0.0;
    }
    v[(((v.len() - 1) as f32) * f) as usize]
}

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn main() {
    let path = std::env::args().nth(1).expect("gcode path");
    let tl = Timeline::parse(&std::fs::read(&path).unwrap());
    let n = tl.moves.len();
    println!("{} moves, {:.0} s, {} layers", n, tl.seconds, tl.layers.len());

    // prefix[i] = extruding moves in 0..i  (so laid-in-layer = prefix[mv+1] - prefix[first])
    let mut prefix = vec![0u32; n + 1];
    for i in 0..n {
        prefix[i + 1] = prefix[i] + tl.moves[i].extruding as u32;
    }

    // How big is the per-frame linear scan the GUI does instead of this?
    let mut per_layer: Vec<u32> = Vec::new();
    for (li, l) in tl.layers.iter().enumerate() {
        let a = l.first_move as usize;
        let b = tl.layers.get(li + 1).map(|x| x.first_move as usize).unwrap_or(n);
        per_layer.push((b - a) as u32);
    }
    per_layer.sort_unstable();
    println!(
        "moves per layer: p50 {}  p90 {}  max {}   (the GUI re-scans up to this many EVERY frame)",
        per_layer[per_layer.len() / 2],
        per_layer[per_layer.len() * 9 / 10],
        per_layer[per_layer.len() - 1]
    );

    for fps in [60.0f32, 30.0, 20.0, 15.0] {
        let frame = 1.0 / fps;
        let mut frames = 0u32;
        let mut renders = 0u32;
        // gap between successive REDRAWS, in seconds and in mm of nozzle travel
        let mut gaps_s: Vec<f32> = Vec::new();
        let mut jumps: Vec<f32> = Vec::new();
        // the jump you would see if every frame redrew (pure frame-rate limit)
        let mut ideal: Vec<f32> = Vec::new();

        let mut key: Option<(u32, usize)> = None;
        let mut last_drawn: Option<([f32; 3], f32)> = None;
        let mut prev_frame_pos: Option<[f32; 3]> = None;
        let mut t = 0.0f32;
        while t < tl.seconds {
            let (pos, mv) = tl.at(t);
            let li = tl.layer_of(mv);
            let first = tl.layers[li].first_move as usize;
            let laid = prefix[mv + 1] - prefix[first.min(mv)];
            let k = (laid, li);
            frames += 1;
            if let Some(p) = prev_frame_pos {
                ideal.push(dist(p, pos));
            }
            prev_frame_pos = Some(pos);
            if key != Some(k) {
                key = Some(k);
                renders += 1;
                if let Some((p, ts)) = last_drawn {
                    gaps_s.push(t - ts);
                    jumps.push(dist(p, pos));
                }
                last_drawn = Some((pos, t));
            }
            t += frame;
        }
        gaps_s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        jumps.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ideal.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let stale = frames - renders;
        // "held" = frames where the same picture was shown again
        println!(
            "\n{fps:.0} fps ({:.1} ms): {frames} frames, {renders} redraws — {} frames ({:.1}%) re-blit a stale image",
            frame * 1000.0,
            stale,
            stale as f32 / frames as f32 * 100.0
        );
        println!(
            "  hold between redraws (ms):  p50 {:.0}  p90 {:.0}  p99 {:.0}  max {:.0}",
            q(&gaps_s, 0.5) * 1e3,
            q(&gaps_s, 0.9) * 1e3,
            q(&gaps_s, 0.99) * 1e3,
            gaps_s.last().copied().unwrap_or(0.0) * 1e3
        );
        println!(
            "  VISIBLE jump per redraw (mm): p50 {:.2}  p90 {:.2}  p99 {:.2}  max {:.1}",
            q(&jumps, 0.5),
            q(&jumps, 0.9),
            q(&jumps, 0.99),
            jumps.last().copied().unwrap_or(0.0)
        );
        println!(
            "  if every frame redrew (mm):   p50 {:.2}  p90 {:.2}  p99 {:.2}  max {:.1}",
            q(&ideal, 0.5),
            q(&ideal, 0.9),
            q(&ideal, 0.99),
            ideal.last().copied().unwrap_or(0.0)
        );
        // 13 mm is the drawn heater block's own width (nozzle_verts: r=6.5).
        // A step wider than the object itself leaves NO overlap between the
        // two drawn positions — that is the line between "fast" and "teleport".
        for thr in [2.0f32, 13.0, 26.0] {
            let big = jumps.iter().filter(|j| **j > thr).count();
            let ideal_big = ideal.iter().filter(|j| **j > thr).count();
            println!(
                "  redraws stepping >{:.0} mm: {big} ({:.1}%)   [if every frame redrew: {:.1}%]",
                thr,
                big as f32 / jumps.len().max(1) as f32 * 100.0,
                ideal_big as f32 / ideal.len().max(1) as f32 * 100.0
            );
        }
        let holds = gaps_s.iter().filter(|g| **g > 0.100).count();
        println!(
            "  holds longer than 100 ms (a visible freeze): {holds} ({:.1}% of redraws)",
            holds as f32 / gaps_s.len().max(1) as f32 * 100.0
        );
    }
}
