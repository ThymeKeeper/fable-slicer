//! Does a biased Z reading still find the layer the nozzle is on?
//!
//! The machine reports its position after its own transform stack — bed mesh,
//! gcode offset — while the timeline holds the file's raw Z, so on a bowed bed
//! the two disagree by a tenth or two. That used to be enough to elect the
//! layer BELOW, whose bead is tens of seconds back: the head stalled, and the
//! next reading threw it forward again. Pause, then teleport, a few times a
//! print, all in the first 20 mm where the mesh has not faded out.
//!
//! Drives the real `locate`/`sync` loop over a real file at a range of biases.
//! What matters is that the numbers do not MOVE with the bias, and that the
//! match error never goes negative — a match behind the truth is a rewind.
//!
//!     cargo run --release -p gcode --example zbias_probe -- FILE.gcode -0.3
//!
//! Cross-check by reverting `locate` to rank in 3-D: at −0.15 mm and beyond
//! the snaps appear, ~27 s each, and the crawl time climbs with the bias.

use gcode::{Playhead, Timeline};

fn main() {
    let path = std::env::args().nth(1).expect("gcode path");
    let bias: f32 = std::env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(-0.2);
    let bytes = std::fs::read(&path).unwrap();
    let tl = Timeline::parse(&bytes);
    println!("{} layers, {:.0} s, {} moves", tl.layers.len(), tl.seconds, tl.moves.len());

    let mut ph = Playhead::default();
    ph.rate = 1.0;
    let (mut polls, mut wrong, mut jumps, mut crawl_frames) = (0u32, 0u32, 0u32, 0u32);
    let mut worst_jump = 0.0f32;
    let mut errs: Vec<f32> = Vec::new();

    // 2 s polls, 60 fps frames, machine running the timeline at 1:1 with the
    // reader 3 s ahead of the nozzle (the buffer lead).
    let mut machine = 0.0f32;
    while machine < tl.seconds - 5.0 {
        // Where the machine really is, and what it reports (Z biased, faded
        // out above 20 mm the way a bed mesh is).
        let (pos, mi) = tl.at(machine);
        let li_true = tl.layer_of(mi);
        let fade = (1.0 - pos[2] / 20.0).clamp(0.0, 1.0);
        let reported = [pos[0], pos[1], pos[2] + bias * fade];
        let t_read = (machine + 3.0).min(tl.seconds);

        let m = tl.locate(reported, t_read, 30.0).filter(|&(_, off)| off < 1.5);
        if let Some((t, _)) = m {
            let li_m = tl.layer_of(tl.at(t).1);
            if li_m != li_true {
                wrong += 1;
            }
            let err = t - machine;
            errs.push(err);
        }
        let before = ph.t;
        ph.sync(machine, t_read, m.map(|(t, _)| t));
        if (ph.t - before).abs() > 0.01 {
            jumps += 1;
            worst_jump = worst_jump.max((ph.t - before).abs());
        }
        polls += 1;

        for _ in 0..120 {
            let b = ph.t;
            ph.advance(1.0 / 60.0);
            if (ph.t - b) < ph.rate / 60.0 * 0.5 {
                crawl_frames += 1;
            }
        }
        machine += 2.0;
    }
    println!(
        "polls {polls}  wrong-layer {wrong}  snaps {jumps} (worst {worst_jump:.1} s)  \
         crawl-frames {crawl_frames} ({:.1} s)",
        crawl_frames as f32 / 60.0
    );
    errs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = errs.len();
    let q = |f: f32| errs[((n as f32 - 1.0) * f) as usize];
    let over = |s: f32| errs.iter().filter(|e| e.abs() > s).count();
    println!(
        "  match error vs truth (s): p50 {:+.2}  p99 {:+.2}  min {:+.2}  max {:+.2}  \
         |err|>2s: {}  >5s: {}",
        q(0.5),
        q(0.99),
        errs[0],
        errs[n - 1],
        over(2.0),
        over(5.0)
    );
}
