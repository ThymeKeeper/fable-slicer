//! How much of a real file's motion falls inside one frame?
//!
//! The virtual nozzle interpolates linearly within each move, so it glides
//! only while a frame lands INSIDE a move. If a file's moves are shorter than
//! a frame, every frame lands on (or past) an endpoint and the nozzle steps
//! from vertex to vertex however perfect the tracking is.

use gcode::Timeline;

fn main() {
    let path = std::env::args().nth(1).expect("gcode path");
    let fps: f32 = std::env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(60.0);
    let tl = Timeline::parse(&std::fs::read(&path).unwrap());
    let frame = 1.0 / fps;
    println!("{} moves, {:.0} s, frame {:.1} ms", tl.moves.len(), tl.seconds, frame * 1000.0);

    let mut durs: Vec<f32> = Vec::with_capacity(tl.moves.len());
    let mut lens: Vec<f32> = Vec::with_capacity(tl.moves.len());
    let mut zero = 0u32;
    let mut prev = tl.start;
    let mut t0 = 0.0f32;
    for m in &tl.moves {
        let d = m.t_end - t0;
        let l = ((m.to[0] - prev[0]).powi(2) + (m.to[1] - prev[1]).powi(2) + (m.to[2] - prev[2]).powi(2)).sqrt();
        if d <= 1.0e-6 {
            zero += 1;
        }
        durs.push(d);
        lens.push(l);
        prev = m.to;
        t0 = m.t_end;
    }
    let n = durs.len();
    let mut sd = durs.clone();
    sd.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let q = |v: &Vec<f32>, f: f32| v[((v.len() as f32 - 1.0) * f) as usize];
    let mut sl = lens.clone();
    sl.sort_by(|a, b| a.partial_cmp(b).unwrap());

    println!(
        "move duration (ms): p10 {:.2}  p50 {:.2}  p90 {:.2}  p99 {:.2}  max {:.0}",
        q(&sd, 0.10) * 1e3, q(&sd, 0.5) * 1e3, q(&sd, 0.9) * 1e3, q(&sd, 0.99) * 1e3, sd[n - 1] * 1e3
    );
    println!(
        "move length (mm):   p10 {:.3}  p50 {:.3}  p90 {:.3}  p99 {:.3}  max {:.1}",
        q(&sl, 0.10), q(&sl, 0.5), q(&sl, 0.9), q(&sl, 0.99), sl[n - 1]
    );
    println!("zero-duration moves: {zero} ({:.2}%)", zero as f32 / n as f32 * 100.0);

    let shorter = durs.iter().filter(|d| **d < frame).count();
    println!(
        "moves shorter than one frame: {shorter} / {n} ({:.1}%)",
        shorter as f32 / n as f32 * 100.0
    );
    // The number that matters: stepping through the file at one frame per
    // tick, how often does the sample land strictly INSIDE a move (a glide)
    // rather than at/past its end (a step)?
    let (mut inside, mut frames) = (0u32, 0u32);
    let mut t = 0.0f32;
    let mut i = 0usize;
    let mut start = 0.0f32;
    while t < tl.seconds && i < n {
        while i < n && tl.moves[i].t_end < t {
            start = tl.moves[i].t_end;
            i += 1;
        }
        if i < n {
            let span = tl.moves[i].t_end - start;
            let f = if span > 1.0e-6 { (t - start) / span } else { 1.0 };
            if f > 0.02 && f < 0.98 {
                inside += 1;
            }
        }
        frames += 1;
        t += frame;
    }
    println!(
        "frames landing mid-move: {inside} / {frames} ({:.1}%) — the rest sit on a vertex",
        inside as f32 / frames as f32 * 100.0
    );
}
