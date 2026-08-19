//! MAGNITUDE: how far does the drawn nozzle jump between consecutive frames,
//! and how many frames does a typical motion get?

use gcode::Timeline;

fn q(v: &[f32], f: f32) -> f32 {
    v[(((v.len() - 1) as f32) * f) as usize]
}

fn main() {
    let path = std::env::args().nth(1).expect("gcode path");
    let tl = Timeline::parse(&std::fs::read(&path).unwrap());
    let n = tl.moves.len();
    let extruding = tl.moves.iter().filter(|m| m.extruding).count();
    println!(
        "{} moves ({} extruding, {} travel), {:.0} s",
        n,
        extruding,
        n - extruding,
        tl.seconds
    );

    // Per-move duration/length split by extruding vs travel.
    let mut prev = tl.start;
    let mut t0 = 0.0f32;
    let (mut tdur, mut tlen) = (Vec::new(), Vec::new());
    let (mut edur, mut elen) = (Vec::new(), Vec::new());
    for m in &tl.moves {
        let d = m.t_end - t0;
        let l = ((m.to[0] - prev[0]).powi(2)
            + (m.to[1] - prev[1]).powi(2)
            + (m.to[2] - prev[2]).powi(2))
        .sqrt();
        if m.extruding {
            edur.push(d);
            elen.push(l);
        } else {
            tdur.push(d);
            tlen.push(l);
        }
        prev = m.to;
        t0 = m.t_end;
    }
    for (name, mut d, mut l) in
        [("extrude", edur.clone(), elen.clone()), ("travel", tdur.clone(), tlen.clone())]
    {
        d.sort_by(|a, b| a.partial_cmp(b).unwrap());
        l.sort_by(|a, b| a.partial_cmp(b).unwrap());
        println!(
            "  {name:8} dur ms p50 {:.2} p90 {:.2} p99 {:.2} max {:.0} | len mm p50 {:.3} p90 {:.3} p99 {:.3} max {:.1}",
            q(&d, 0.5) * 1e3,
            q(&d, 0.9) * 1e3,
            q(&d, 0.99) * 1e3,
            d[d.len() - 1] * 1e3,
            q(&l, 0.5),
            q(&l, 0.9),
            q(&l, 0.99),
            l[l.len() - 1]
        );
    }

    // Walk the file at a fixed frame period; record the euclidean jump of the
    // rendered nozzle between consecutive frames, and how many frames each
    // move receives.
    for fps in [60.0f32, 30.0, 24.0, 15.0, 12.0] {
        let frame = 1.0 / fps;
        let mut steps: Vec<f32> = Vec::new();
        let mut t = 0.0f32;
        let mut last = tl.at(0.0).0;
        // frames-per-move: count frames whose sample falls in each move
        let mut hits: Vec<u32> = vec![0; n];
        while t < tl.seconds {
            let (p, idx) = tl.at(t);
            hits[idx.min(n - 1)] += 1;
            let s = ((p[0] - last[0]).powi(2) + (p[1] - last[1]).powi(2) + (p[2] - last[2]).powi(2))
                .sqrt();
            steps.push(s);
            last = p;
            t += frame;
        }
        steps.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let zero_frame_moves = hits.iter().filter(|h| **h == 0).count();
        let one_frame = hits.iter().filter(|h| **h == 1).count();
        let mean: f64 = steps.iter().map(|s| *s as f64).sum::<f64>() / steps.len() as f64;
        println!(
            "fps {fps:>4.0} ({:.1} ms): step mm p50 {:.2} p90 {:.2} p99 {:.2} max {:.1} mean {:.2} | moves with 0 frames {:.1}%, exactly 1 frame {:.1}%",
            frame * 1e3,
            q(&steps, 0.5),
            q(&steps, 0.9),
            q(&steps, 0.99),
            steps[steps.len() - 1],
            mean,
            zero_frame_moves as f32 / n as f32 * 100.0,
            one_frame as f32 / n as f32 * 100.0,
        );
    }
}
