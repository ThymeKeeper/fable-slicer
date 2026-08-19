//! Watch the real match loop against a real running print.
//!
//! Reads the job's g-code from disk and polls a Moonraker host for its live
//! position, then does exactly what the Machine view does: locate the reported
//! point on the timeline, feed it to the playhead, and report what the head
//! did about it. Prints the Z gap between the reported position and the file's
//! layer (`zb`) alongside the layer the match landed on, so a bias-driven
//! mis-match is visible as it happens.
//!
//!     cargo run --release -p gcode --example live_probe -- JOB.gcode 192.168.1.133 120

use gcode::{Playhead, Timeline};

fn get(host: &str) -> Option<(f32, u32, [f32; 3], String)> {
    let url = format!(
        "http://{host}/printer/objects/query?print_stats&virtual_sdcard&motion_report"
    );
    let out = std::process::Command::new("curl").args(["-s", "--max-time", "5", &url]).output().ok()?;
    let s = String::from_utf8_lossy(&out.stdout).to_string();
    let num = |key: &str| -> Option<f64> {
        let i = s.find(key)? + key.len();
        let rest = &s[i..];
        let j = rest.find(|c: char| c.is_ascii_digit() || c == '-')?;
        let rest = &rest[j..];
        let k = rest.find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-')).unwrap_or(rest.len());
        rest[..k].parse().ok()
    };
    let dur = num("\"print_duration\":")? as f32;
    let fpos = num("\"file_position\":")? as u32;
    let i = s.find("\"live_position\": [")? + 18;
    let rest = &s[i..];
    let j = rest.find(']')?;
    let mut it = rest[..j].split(',').map(|v| v.trim().parse::<f32>().unwrap_or(0.0));
    let pos = [it.next()?, it.next()?, it.next()?];
    let state = {
        let i = s.find("\"state\": \"")? + 10;
        let rest = &s[i..];
        rest[..rest.find('"').unwrap_or(0)].to_string()
    };
    Some((dur, fpos, pos, state))
}

fn main() {
    let mut a = std::env::args().skip(1);
    let path = a.next().expect("gcode path");
    let host = a.next().unwrap_or_else(|| "192.168.1.133".into());
    let polls: u32 = a.next().map(|s| s.parse().unwrap()).unwrap_or(60);

    let tl = Timeline::parse(&std::fs::read(&path).unwrap());
    eprintln!("{} layers, {:.0} s, {} moves", tl.layers.len(), tl.seconds, tl.moves.len());

    let mut ph = Playhead::default();
    let mut last = std::time::Instant::now();
    let (mut snaps, mut wrong, mut nomatch) = (0u32, 0u32, 0u32);

    for n in 0..polls {
        let Some((dur, fpos, pos, state)) = get(&host) else {
            eprintln!("poll failed");
            std::thread::sleep(std::time::Duration::from_secs(2));
            continue;
        };
        if state != "printing" {
            eprintln!("state {state} — stopping");
            break;
        }
        // Free-run the head over the interval since the last poll, the way the
        // GUI's frames would.
        let now = std::time::Instant::now();
        let mut dt = now.duration_since(last).as_secs_f32();
        last = now;
        while dt > 0.0 {
            ph.advance(dt.min(1.0 / 60.0));
            dt -= 1.0 / 60.0;
        }

        let t_read = tl.time_at_byte(fpos);
        let m = tl.locate(pos, t_read, 30.0).filter(|&(_, off)| off < 1.5);
        let before = ph.t;
        ph.sync(dur, t_read, m.map(|(t, _)| t));

        let lh = tl.layer_of(tl.at(ph.t).1);
        let zb = pos[2] - tl.layers.get(lh).map(|l| l.z).unwrap_or(0.0);
        let jump = ph.t - before;
        if jump.abs() > 1.0 {
            snaps += 1;
        }
        let lm = m.map(|(t, _)| tl.layer_of(tl.at(t).1));
        if let Some(l) = lm {
            if l != lh {
                wrong += 1;
            }
        } else {
            nomatch += 1;
        }
        println!(
            "{n:3}  dur {dur:7.1}  read {t_read:7.1}  {}  head {:7.1} L{lh}  zb {zb:+.3}  \
             drift {:+6.2}  rate {:.3}{}",
            match m {
                Some((t, d)) => format!("match t {t:7.1} at {d:4.2} mm L{}", lm.unwrap()),
                None => "match none                ".into(),
            },
            ph.t,
            m.map(|(t, _)| t - before).unwrap_or(f32::NAN),
            ph.rate,
            if jump.abs() > 1.0 { "  <-- SNAP" } else { "" },
        );
        std::thread::sleep(std::time::Duration::from_secs(2));
    }
    println!("\nsnaps {snaps}  wrong-layer {wrong}  no-match {nomatch}");
}
