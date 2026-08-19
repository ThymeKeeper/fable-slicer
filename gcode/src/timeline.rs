//! Reading g-code back: a motion timeline for mirroring a running print.
//!
//! The Machine view needs to know where the nozzle is *now* on a job the
//! printer is executing. Polling the machine for that position is both
//! wasteful and wrong — Klipper answers from a lookahead queue that runs
//! ahead of the deposited plastic. So instead we read the job's g-code once,
//! build the motion timeline it describes, and play it locally, disciplined
//! against the machine's own executed-seconds clock.
//!
//! Deliberately parsed from **g-code**, not built from the slicer's plans:
//! one producer serves a file we sliced and a file some other slicer made,
//! and what it mirrors is what the machine actually executes rather than what
//! we intended. Tolerant by design — an unknown word, a macro, a comment, or
//! a firmware-specific line is skipped rather than fought.
//!
//! No acceleration model. Each move takes distance over its feed rate, which
//! runs optimistic on short segments; the playhead's rate discipline learns
//! that bias from the machine within the first minute, and a constant bias is
//! exactly what such a loop absorbs best.

/// One motion segment: where it ends, when it ends, and where it came from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Move {
    /// Position at the END of the move (mm).
    pub to: [f32; 3],
    /// Cumulative seconds from the start of the file to the end of this move.
    pub t_end: f32,
    /// Byte offset of the line that produced it — the anchor for a resync
    /// against the machine's file position.
    pub at_byte: u32,
    /// Lays material down (rather than travelling or retracting).
    pub extruding: bool,
    /// Filament consumed on this move (mm of filament) — what a bead's width
    /// is reconstructed from when the timeline is drawn.
    pub e_mm: f32,
    /// What the file said it was printing, from its `;TYPE:` comments. Every
    /// slicer in circulation writes these and they agree closely enough to
    /// map; a file without them draws as one feature, which is honest.
    pub feature: Feature,
}

/// The feature a move belongs to, as named by the file's `;TYPE:` comments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Feature {
    #[default]
    Other,
    OuterWall,
    InnerWall,
    Overhang,
    Infill,
    Solid,
    Top,
    Bottom,
    Bridge,
    GapFill,
    Support,
    Skirt,
}

impl Feature {
    /// Map a `;TYPE:` label. Matched loosely and case-insensitively: the
    /// slicers spell these differently ("Outer wall", "External perimeter",
    /// "WALL-OUTER") and gain new ones between versions.
    pub fn parse(label: &str) -> Feature {
        let l = label.trim().to_ascii_lowercase();
        let has = |a: &str, b: &str| l.contains(a) && l.contains(b);
        if l.contains("skirt") || l.contains("brim") {
            Feature::Skirt
        } else if l.contains("support") {
            Feature::Support
        } else if l.contains("overhang") {
            Feature::Overhang
        } else if l.contains("bridge") {
            Feature::Bridge
        } else if l.contains("gap") {
            Feature::GapFill
        } else if has("outer", "wall") || has("external", "perimeter") || l.contains("wall-outer") {
            Feature::OuterWall
        } else if l.contains("wall") || l.contains("perimeter") {
            Feature::InnerWall
        } else if l.contains("top") {
            Feature::Top
        } else if l.contains("bottom") {
            Feature::Bottom
        } else if l.contains("solid") {
            Feature::Solid
        } else if l.contains("infill") || l.contains("fill") {
            Feature::Infill
        } else {
            Feature::Other
        }
    }
}

/// A printed layer: the Z it sits at and where it starts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Layer {
    pub z: f32,
    /// Index into [`Timeline::moves`] of the layer's first move.
    pub first_move: u32,
    pub at_byte: u32,
}

/// The motion a g-code file describes, indexed by time.
#[derive(Debug, Clone, Default)]
pub struct Timeline {
    /// Where the file starts moving from (the origin unless a G92 says else).
    pub start: [f32; 3],
    pub moves: Vec<Move>,
    /// Layer boundaries, detected from the Z that extrusion happens at —
    /// slicer-agnostic on purpose, since every slicer marks layers its own
    /// way (or not at all).
    pub layers: Vec<Layer>,
    /// Total seconds the timeline spans.
    pub seconds: f32,
}

/// Machine state carried across lines while parsing.
struct Parser {
    pos: [f64; 3],
    e: f64,
    feed_mm_s: f64,
    abs_xyz: bool,
    abs_e: bool,
    /// Multiplier into mm — 25.4 after a G20, 1.0 after a G21.
    scale: f64,
    layer_z: Option<f64>,
    feature: Feature,
    /// Z last set by NON-extruding motion since the previous extrusion — the
    /// layer-change signal (see `commit`).
    pending_z: Option<f64>,
    t: f64,
}

impl Timeline {
    /// Parse a g-code file into its motion timeline.
    pub fn parse(src: &[u8]) -> Timeline {
        let mut p = Parser {
            pos: [0.0; 3],
            e: 0.0,
            // Nothing sane omits F before the first move, but a file that does
            // shouldn't divide by zero: assume a slow travel until told.
            feed_mm_s: 50.0,
            abs_xyz: true,
            abs_e: false,
            scale: 1.0,
            layer_z: None,
            feature: Feature::default(),
            pending_z: None,
            t: 0.0,
        };
        let mut tl = Timeline::default();
        let mut at = 0usize; // byte offset of the current line
        for line in src.split(|&b| b == b'\n') {
            let start_of_line = at;
            at += line.len() + 1;
            let (code, comment) = match line.iter().position(|&b| b == b';') {
                Some(i) => (&line[..i], &line[i + 1..]),
                None => (line, &line[line.len()..]),
            };
            if let Ok(c) = std::str::from_utf8(comment) {
                let c = c.trim_start();
                if let Some(label) = c.strip_prefix("TYPE:") {
                    p.feature = Feature::parse(label);
                }
            }
            let code = std::str::from_utf8(code).unwrap_or("").trim();
            if code.is_empty() {
                continue;
            }
            p.line(code, start_of_line as u32, &mut tl);
        }
        tl.seconds = p.t as f32;
        tl
    }

    /// Position at time `t` seconds, and the move it falls in.
    pub fn at(&self, t: f32) -> ([f32; 3], usize) {
        if self.moves.is_empty() {
            return (self.start, 0);
        }
        let i = self.move_at_time(t);
        let from = if i == 0 { self.start } else { self.moves[i - 1].to };
        let t0 = if i == 0 { 0.0 } else { self.moves[i - 1].t_end };
        let m = self.moves[i];
        let span = m.t_end - t0;
        let f = if span > 1.0e-6 { ((t - t0) / span).clamp(0.0, 1.0) } else { 1.0 };
        let mut p = [0.0f32; 3];
        for k in 0..3 {
            p[k] = from[k] + (m.to[k] - from[k]) * f;
        }
        (p, i)
    }

    /// Index of the move spanning time `t` (the last move once past the end).
    pub fn move_at_time(&self, t: f32) -> usize {
        match self.moves.binary_search_by(|m| {
            m.t_end.partial_cmp(&t).unwrap_or(std::cmp::Ordering::Equal)
        }) {
            Ok(i) => i,
            Err(i) => i.min(self.moves.len() - 1),
        }
    }

    /// The time at which the file's reader reaches `byte` — how a machine's
    /// reported file position becomes a position on this timeline.
    pub fn time_at_byte(&self, byte: u32) -> f32 {
        if self.moves.is_empty() {
            return 0.0;
        }
        // The reader having REACHED a byte means the move on that line has
        // not run yet — its predecessor is the last thing finished. (Even
        // this is generous: the machine's queue holds work the reader has
        // already swallowed, which is why the playhead trusts executed
        // seconds and keeps this for coarse resync.)
        let i = self.moves.partition_point(|m| m.at_byte < byte);
        if i == 0 {
            0.0
        } else {
            self.moves[i - 1].t_end
        }
    }

    /// How far through layer `li` the time `t` is, 0..=1 — what lets a
    /// mirrored print grow a fraction of a layer at a time instead of
    /// snapping a whole one into existence.
    pub fn layer_fraction(&self, t: f32, li: usize) -> f32 {
        let Some(layer) = self.layers.get(li) else { return 0.0 };
        let first = layer.first_move as usize;
        let start = if first == 0 { 0.0 } else { self.moves[first - 1].t_end };
        let end = match self.layers.get(li + 1) {
            Some(next) => {
                let i = (next.first_move as usize).saturating_sub(1);
                self.moves.get(i).map(|m| m.t_end).unwrap_or(self.seconds)
            }
            None => self.seconds,
        };
        if end - start <= 1.0e-6 {
            return 1.0;
        }
        ((t - start) / (end - start)).clamp(0.0, 1.0)
    }

    /// Which layer a move belongs to (0 when the file has none).
    pub fn layer_of(&self, move_idx: usize) -> usize {
        match self.layers.binary_search_by_key(&(move_idx as u32), |l| l.first_move) {
            Ok(i) => i,
            Err(0) => 0,
            Err(i) => i - 1,
        }
    }
}

impl Parser {
    fn line(&mut self, code: &str, at_byte: u32, tl: &mut Timeline) {
        let mut words = code.split_whitespace();
        let Some(cmd) = words.next() else { return };
        let cmd = cmd.to_ascii_uppercase();
        // A word is a letter plus a number; anything unparseable is skipped,
        // which is what keeps macros and firmware-specific lines harmless.
        let mut w = [f64::NAN; 6]; // X Y Z E F I J → indices below
        let mut ij = [f64::NAN; 2];
        for word in words {
            let b = word.as_bytes();
            if b.is_empty() {
                continue;
            }
            let Ok(v) = word[1..].parse::<f64>() else { continue };
            match b[0].to_ascii_uppercase() {
                b'X' => w[0] = v,
                b'Y' => w[1] = v,
                b'Z' => w[2] = v,
                b'E' => w[3] = v,
                b'F' => w[4] = v,
                b'I' => ij[0] = v,
                b'J' => ij[1] = v,
                _ => {}
            }
        }
        match cmd.as_str() {
            "G0" | "G1" => self.linear(&w, at_byte, tl),
            "G2" | "G3" => self.arc(&w, &ij, cmd == "G2", at_byte, tl),
            "G90" => self.abs_xyz = true,
            "G91" => self.abs_xyz = false,
            "M82" => self.abs_e = true,
            "M83" => self.abs_e = false,
            "G20" => self.scale = 25.4,
            "G21" => self.scale = 1.0,
            "G92" => {
                // Set position without motion — the E reset absolute-E files
                // do every layer, and occasionally an axis rebase.
                for k in 0..3 {
                    if !w[k].is_nan() {
                        self.pos[k] = w[k] * self.scale;
                    }
                }
                if !w[3].is_nan() {
                    self.e = w[3] * self.scale;
                }
            }
            _ => {}
        }
    }

    /// Target position for a move's words, honoring absolute/relative.
    fn target(&self, w: &[f64; 6]) -> [f64; 3] {
        let mut to = self.pos;
        for k in 0..3 {
            if !w[k].is_nan() {
                let v = w[k] * self.scale;
                to[k] = if self.abs_xyz { v } else { self.pos[k] + v };
            }
        }
        to
    }

    /// How much filament a move's E word commits, and the new E position.
    fn extrusion(&self, w: &[f64; 6]) -> (f64, f64) {
        if w[3].is_nan() {
            return (0.0, self.e);
        }
        let v = w[3] * self.scale;
        if self.abs_e {
            (v - self.e, v)
        } else {
            (v, self.e + v)
        }
    }

    fn linear(&mut self, w: &[f64; 6], at_byte: u32, tl: &mut Timeline) {
        if !w[4].is_nan() {
            self.feed_mm_s = (w[4] * self.scale / 60.0).max(1.0e-6);
        }
        let to = self.target(w);
        let (de, e_new) = self.extrusion(w);
        let d = dist(self.pos, to);
        // An E-only move (retract, prime, purge) still costs time, at the
        // feed the E word is travelling at.
        let secs = if d > 1.0e-9 { d / self.feed_mm_s } else { de.abs() / self.feed_mm_s };
        self.commit(to, e_new, de > 0.0 && d > 1.0e-9, secs, at_byte, tl);
    }

    fn arc(&mut self, w: &[f64; 6], ij: &[f64; 2], clockwise: bool, at_byte: u32, tl: &mut Timeline) {
        if !w[4].is_nan() {
            self.feed_mm_s = (w[4] * self.scale / 60.0).max(1.0e-6);
        }
        let to = self.target(w);
        let (de, e_new) = self.extrusion(w);
        let (i, j) = (
            if ij[0].is_nan() { 0.0 } else { ij[0] * self.scale },
            if ij[1].is_nan() { 0.0 } else { ij[1] * self.scale },
        );
        let c = [self.pos[0] + i, self.pos[1] + j];
        let r = (i * i + j * j).sqrt();
        if r < 1.0e-9 {
            self.linear(w, at_byte, tl);
            return;
        }
        let a0 = (self.pos[1] - c[1]).atan2(self.pos[0] - c[0]);
        let mut a1 = (to[1] - c[1]).atan2(to[0] - c[0]);
        use std::f64::consts::TAU;
        if clockwise {
            while a1 >= a0 {
                a1 -= TAU;
            }
        } else {
            while a1 <= a0 {
                a1 += TAU;
            }
        }
        // Sample to a ~0.4 mm chord: fine enough to draw and to place a
        // nozzle on, cheap enough for a file with a million arcs.
        let sweep = (a1 - a0).abs();
        let n = ((sweep * r / 0.4).ceil() as usize).clamp(1, 512);
        let arc_len = sweep * r;
        let secs = arc_len / self.feed_mm_s;
        let extruding = de > 0.0;
        let z0 = self.pos[2];
        for k in 1..=n {
            let f = k as f64 / n as f64;
            let a = a0 + (a1 - a0) * f;
            let p = [c[0] + r * a.cos(), c[1] + r * a.sin(), z0 + (to[2] - z0) * f];
            let e_here = self.e + de * f;
            self.commit(p, e_here, extruding, secs / n as f64, at_byte, tl);
        }
        self.e = e_new;
    }

    /// Record one segment and advance the machine state.
    fn commit(
        &mut self,
        to: [f64; 3],
        e_new: f64,
        extruding: bool,
        secs: f64,
        at_byte: u32,
        tl: &mut Timeline,
    ) {
        if secs <= 0.0 && dist(self.pos, to) <= 1.0e-9 {
            self.pos = to;
            self.e = e_new;
            return;
        }
        self.t += secs;
        // A new layer starts at the first extrusion after NON-extruding motion
        // raised Z. Detecting it from the printed Z alone looks obvious and is
        // wrong: a scarf seam ramps Z by a fraction of a layer *while
        // extruding* (this slicer's own files do, ~100 times a layer), and a
        // z-hop lowers back to the same Z it left. Layer changes happen
        // between extrusions, hops return to where they started, and scarfs
        // never let go of the bead — which is exactly what this distinguishes.
        // Slicer-agnostic on purpose: none of them agree on layer comments.
        if extruding {
            // The first extrusion of the file opens layer one wherever it
            // happens to be; after that only a non-extruding rise counts.
            let z = match (self.layer_z, self.pending_z.take()) {
                (None, pending) => Some(pending.unwrap_or(to[2])),
                (Some(cur), Some(z)) if z > cur + 1.0e-4 => Some(z),
                _ => None,
            };
            if let Some(z) = z {
                self.layer_z = Some(z);
                tl.layers.push(Layer {
                    z: z as f32,
                    first_move: tl.moves.len() as u32,
                    at_byte,
                });
            }
        } else if (to[2] - self.pos[2]).abs() > 1.0e-9 {
            self.pending_z = Some(to[2]);
        }
        if tl.moves.is_empty() {
            tl.start = [self.pos[0] as f32, self.pos[1] as f32, self.pos[2] as f32];
        }
        tl.moves.push(Move {
            to: [to[0] as f32, to[1] as f32, to[2] as f32],
            t_end: self.t as f32,
            at_byte,
            extruding,
            e_mm: (e_new - self.e) as f32,
            feature: self.feature,
        });
        self.pos = to;
        self.e = e_new;
    }
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (dx, dy, dz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_moves_accumulate_time_and_layers() {
        // 100 mm at 3000 mm/min = 50 mm/s → 2 s; then a layer up and back.
        let g = "\
G21
G90
M83
G1 X0 Y0 Z0.2 F3000
G1 X100 Y0 E5
G1 Z0.4
G1 X0 Y0 E5
";
        let tl = Timeline::parse(g.as_bytes());
        assert_eq!(tl.layers.len(), 2, "two printed Zs");
        assert!((tl.layers[0].z - 0.2).abs() < 1e-6);
        assert!((tl.layers[1].z - 0.4).abs() < 1e-6);
        // 2 s of extrusion each way, plus the tiny Z hop.
        assert!((tl.seconds - 4.004).abs() < 0.01, "seconds {}", tl.seconds);
        // Halfway through the first extrusion — which starts at t = 0.004,
        // after the 0.2 mm lift the file opens with.
        let (p, _) = tl.at(0.004 + 1.0);
        assert!((p[0] - 50.0).abs() < 0.01, "x {}", p[0]);
        // The travel to the start is not extrusion; the two long moves are.
        assert_eq!(tl.moves.iter().filter(|m| m.extruding).count(), 2);
    }

    #[test]
    fn relative_and_absolute_extrusion_both_read_as_deposition() {
        let rel = Timeline::parse(b"G90\nM83\nG1 X10 Y0 Z0.2 F600 E1\n");
        let abs = Timeline::parse(b"G90\nM82\nG92 E0\nG1 X10 Y0 Z0.2 F600 E1\n");
        assert_eq!(rel.moves.len(), 1);
        assert_eq!(abs.moves.len(), 1);
        assert!(rel.moves[0].extruding && abs.moves[0].extruding);
        // A retraction is E-only: it costs time but deposits nothing and
        // leaves the head where it was.
        let retr = Timeline::parse(b"G90\nM83\nG1 E-0.8 F2400\n");
        assert_eq!(retr.moves.len(), 1);
        assert!(!retr.moves[0].extruding);
        assert!((retr.seconds - 0.02).abs() < 1e-3, "seconds {}", retr.seconds);
    }

    #[test]
    fn arcs_sample_along_the_sweep() {
        // A quarter circle of radius 10 counter-clockwise from (10,0) about
        // the origin: arc length 15.7 mm at 10 mm/s → ~1.57 s.
        let g = "G90\nM83\nG1 X10 Y0 Z0.2 F600\nG3 X0 Y10 I-10 J0 E1 F600\n";
        let tl = Timeline::parse(g.as_bytes());
        assert!(tl.moves.len() > 20, "arc sampled into segments");
        let last = tl.moves.last().unwrap();
        assert!((last.to[0]).abs() < 0.01 && (last.to[1] - 10.0).abs() < 0.01);
        // Mid-sweep the head is out on the radius, not on the chord.
        let (p, _) = tl.at(tl.seconds - 0.785);
        assert!((p[0].hypot(p[1]) - 10.0).abs() < 0.1, "on the radius: {p:?}");
    }

    #[test]
    fn layer_fraction_walks_a_layer_from_nothing_to_all_of_it() {
        let g = "\
G90
M83
G1 X0 Y0 Z0.2 F600
G1 X10 Y0 E1
G1 Z0.4
G1 X0 Y0 E1
";
        let tl = Timeline::parse(g.as_bytes());
        assert_eq!(tl.layers.len(), 2);
        // Layer 0 spans the first extrusion only.
        assert!(tl.layer_fraction(0.0, 0) < 0.01, "starts empty");
        assert!((tl.layer_fraction(tl.seconds, 0) - 1.0).abs() < 1e-6, "ends full");
        let mid = tl.layer_fraction(tl.moves[0].t_end + 0.5, 0);
        assert!(mid > 0.3 && mid < 0.7, "halfway is halfway: {mid}");
        // The last layer runs to the end of the file.
        assert!((tl.layer_fraction(tl.seconds, 1) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn byte_offsets_anchor_a_resync() {
        let g = "G90\nM83\nG1 X10 Y0 Z0.2 F600 E1\nG1 X20 Y0 E1\n";
        let tl = Timeline::parse(g.as_bytes());
        let second = g.find("G1 X20").unwrap() as u32;
        assert_eq!(tl.moves[1].at_byte, second);
        // The reader reaching that line means the first move is done.
        assert!((tl.time_at_byte(second) - tl.moves[0].t_end).abs() < 1e-6);
        assert_eq!(tl.layer_of(1), 0);
    }

    #[test]
    fn junk_lines_are_skipped_not_fought() {
        // Macros, comments, blank lines, unknown codes, and a thumbnail block
        // — a real file's header — must not disturb the motion.
        let g = "\
; generated by SomeSlicer 1.2 on 2026-01-01 at 00:00:00
; thumbnail begin 16x16 24
; iVBORw0KGgoAAAANSUhEUg==
; thumbnail end
START_PRINT EXTRUDER=245 BED=80
SET_VELOCITY_LIMIT ACCEL=10000
M204 S5000

G90
M83
G1 X10 Y0 Z0.2 F600 E1
";
        let tl = Timeline::parse(g.as_bytes());
        assert_eq!(tl.moves.len(), 1);
        assert_eq!(tl.layers.len(), 1);
    }
}

/// A local playhead over a [`Timeline`], disciplined against the machine.
///
/// The timeline's clock is optimistic — it models no acceleration, and on
/// real files that runs 30–85% fast, because at these speeds a great many
/// segments never reach their commanded feed. Rather than model acceleration
/// (and inherit a second set of errors: junction deviation, minimum layer
/// time, macro dwell), the playhead LEARNS the ratio from the machine and
/// keeps learning it.
///
/// Two signals, each doing what it is good at: the machine's executed
/// seconds measure real time honestly, and its file position says where in
/// the timeline that time got it to. Their ratio is the playback rate. The
/// position error is then eased out rather than jumped, so the nozzle glides
/// instead of twitching once a poll.
#[derive(Debug, Clone)]
pub struct Playhead {
    /// Seconds into the timeline.
    pub t: f32,
    /// Timeline seconds per real second — 1.0 until the machine teaches it.
    pub rate: f32,
    learned: bool,
}

/// Position error beyond which the playhead jumps instead of easing: a
/// resume, a skipped section, or a file that isn't what we think it is.
const SNAP_S: f32 = 20.0;
/// How much of the remaining error each sync eases away.
const EASE: f32 = 0.35;
/// Below this the machine has not printed enough for a trustworthy ratio.
const MIN_RATIO_S: f32 = 8.0;

impl Default for Playhead {
    fn default() -> Self {
        Playhead { t: 0.0, rate: 1.0, learned: false }
    }
}

impl Playhead {
    /// Free-run by `dt` real seconds.
    pub fn advance(&mut self, dt: f32) {
        self.t = (self.t + dt * self.rate).max(0.0);
    }

    /// A reading landed: `machine_secs` executed, which the file position
    /// says is `timeline_t` into the timeline.
    pub fn sync(&mut self, machine_secs: f32, timeline_t: f32) {
        if machine_secs > MIN_RATIO_S && timeline_t > 0.0 {
            let r = (timeline_t / machine_secs).clamp(0.05, 20.0);
            // First reading takes the measurement whole — the initial 1.0 is
            // a placeholder, not an estimate worth blending with.
            self.rate = if self.learned { self.rate * 0.75 + r * 0.25 } else { r };
            self.learned = true;
        }
        let drift = timeline_t - self.t;
        self.t += if drift.abs() > SNAP_S { drift } else { drift * EASE };
    }

    /// Start over (a new job, or a job that stopped).
    pub fn reset(&mut self) {
        *self = Playhead::default();
    }
}

#[cfg(test)]
mod playhead_tests {
    use super::*;

    /// A machine printing at half the timeline's optimistic pace, polled
    /// every 2 s: the playhead must converge on the true position and stay
    /// there, without the position ever jumping once converged.
    #[test]
    fn learns_the_machines_pace_and_tracks_it() {
        let true_rate = 0.5_f32; // timeline seconds per real second
        let mut ph = Playhead::default();
        let mut worst_late = 0.0_f32;
        for step in 1..=60 {
            for _ in 0..20 {
                ph.advance(0.1); // 2 s of frames
            }
            let machine_secs = step as f32 * 2.0;
            let truth = machine_secs * true_rate;
            ph.sync(machine_secs, truth);
            if step > 15 {
                worst_late = worst_late.max((ph.t - truth).abs());
            }
        }
        assert!((ph.rate - true_rate).abs() < 0.02, "rate {} vs {true_rate}", ph.rate);
        assert!(worst_late < 0.5, "tracking error {worst_late} s once converged");
    }

    #[test]
    fn eases_small_drift_but_jumps_a_real_discontinuity() {
        let mut ph = Playhead { t: 100.0, rate: 1.0, learned: true };
        ph.sync(100.0, 104.0); // 4 s out: ease, don't teleport
        assert!(ph.t > 100.0 && ph.t < 104.0, "eased to {}", ph.t);
        let mut ph = Playhead { t: 100.0, rate: 1.0, learned: true };
        ph.sync(500.0, 500.0); // the job is somewhere else entirely
        assert!((ph.t - 500.0).abs() < 1.0e-3, "snapped to {}", ph.t);
    }
}
