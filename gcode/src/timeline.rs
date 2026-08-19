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
    /// The feed this move ASKED for (mm/s). What it actually achieves is the
    /// planner's business: a 0.4 mm segment between two corners never sees
    /// anything like this.
    pub feed_mm_s: f32,
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
    /// Acceleration in force (mm/s²) and where it changed, so the planner can
    /// use the limit that applied to each move. Files set this often — per
    /// feature, and again for the first layer.
    accel: f64,
    /// (move index, acceleration, accel-to-decel) at each change.
    accel_at: Vec<(u32, f64, f64)>,
    /// Klipper's square-corner velocity (mm/s): how fast a right-angle corner
    /// may be taken. Marlin's jerk, near enough.
    scv: f64,
    /// Klipper's accel-to-decel: the ceiling on a short move's peak, which is
    /// most of why a real machine is slower than distance-over-feed. Klipper
    /// defaults it to half the acceleration, so it TRACKS accel unless the
    /// file sets it outright — miss that and a file which only sets ACCEL
    /// (Orca's do) gets planned against a stale ceiling.
    atd: f64,
    atd_explicit: bool,
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
            // Defaults for a file that never says: a mid-range machine.
            accel: 3000.0,
            accel_at: Vec::new(),
            scv: 5.0,
            atd: 1500.0,
            atd_explicit: false,
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
        plan_times(&mut tl, &p.accel_at, (p.accel, p.atd), p.scv);
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

    /// Where on the timeline a real machine position sits: the time of the
    /// closest point on the path, and how far off it was (mm).
    ///
    /// Compares against the SEGMENTS, not their endpoints. Mid-travel the
    /// endpoints can be a hundred millimetres apart, and matching to the
    /// nearer one lands on whatever bead happens to be close in space and
    /// seconds away in time — which then reads as "the head is ahead" and
    /// stalls it. The distance comes back so the caller can refuse a match
    /// it shouldn't believe.
    ///
    /// Searched BACKWARDS from `anchor_t`: when the anchor is the file
    /// position the nozzle is always behind it, since the reader fills a
    /// buffer the motion queue drains. Ties (concentric passes) go to the
    /// candidate nearest the anchor in time, but only for points essentially
    /// on each other — a tolerance wider than a bead width would let every
    /// neighbouring pass tie with the right one and drag the answer back to
    /// the reader.
    pub fn locate(&self, pos: [f32; 3], anchor_t: f32, window_s: f32) -> Option<(f32, f32)> {
        if self.moves.is_empty() {
            return None;
        }
        let lo = self.move_at_time(anchor_t - window_s);
        let hi = self.move_at_time(anchor_t + 2.0).min(self.moves.len() - 1);
        let mut best: Option<(f32, f32, f32)> = None; // (dist², |Δt|, t)
        for i in lo..=hi {
            let a = if i == 0 { self.start } else { self.moves[i - 1].to };
            let b = self.moves[i].to;
            let t0 = if i == 0 { 0.0 } else { self.moves[i - 1].t_end };
            let t1 = self.moves[i].t_end;
            let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let ap = [pos[0] - a[0], pos[1] - a[1], pos[2] - a[2]];
            let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
            let u = if len2 > 1.0e-9 {
                ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2).clamp(0.0, 1.0)
            } else {
                1.0
            };
            let d = [ap[0] - ab[0] * u, ap[1] - ab[1] * u, ap[2] - ab[2] * u];
            let dist2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            let t = t0 + (t1 - t0) * u;
            let dt = (t - anchor_t).abs();
            let better = match best {
                None => true,
                Some((bd, bdt, _)) if (dist2 - bd).abs() < 0.01 => dt < bdt,
                Some((bd, ..)) => dist2 < bd,
            };
            if better {
                best = Some((dist2, dt, t));
            }
        }
        best.map(|(d2, _, t)| (t, d2.sqrt()))
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
        let mut w = [f64::NAN; 8]; // X Y Z E F S P T
        let mut ij = [f64::NAN; 2];
        for word in words {
            let b = word.as_bytes();
            if b.is_empty() {
                continue;
            }
            // Klipper's extended commands are KEY=VALUE, not letter-number.
            if let Some((key, val)) = word.split_once('=') {
                if let Ok(v) = val.parse::<f64>() {
                    match key.to_ascii_uppercase().as_str() {
                        "ACCEL" if v > 0.0 => self.set_accel(v, tl),
                        "SQUARE_CORNER_VELOCITY" if v >= 0.0 => self.scv = v,
                        "ACCEL_TO_DECEL" if v > 0.0 => {
                            self.atd = v;
                            self.atd_explicit = true;
                            self.note_limits(tl);
                        }
                        _ => {}
                    }
                }
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
                b'S' => w[5] = v,
                b'P' => w[6] = v,
                b'T' => w[7] = v,
                _ => {}
            }
        }
        match cmd.as_str() {
            "G0" | "G1" => self.linear(&w, at_byte, tl),
            "G2" | "G3" => self.arc(&w, &ij, cmd == "G2", at_byte, tl),
            // M204 S / P / T, and Klipper's SET_VELOCITY_LIMIT — both appear
            // in the wild, sometimes in the same file.
            "M204" => {
                let a = [w[5], w[6], w[7]].into_iter().find(|v| !v.is_nan() && *v > 0.0);
                if let Some(a) = a {
                    self.set_accel(a, tl);
                }
            }
            "SET_VELOCITY_LIMIT" => {}
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

    /// Note an acceleration change against the move it takes effect from.
    /// Accel-to-decel follows at half, the firmware's own default, unless the
    /// file has taken control of it.
    fn set_accel(&mut self, a: f64, tl: &Timeline) {
        if (a - self.accel).abs() > 1.0 {
            self.accel = a;
            if !self.atd_explicit {
                self.atd = a * 0.5;
            }
            self.note_limits(tl);
        }
    }

    /// Record the limits now in force against the next move.
    fn note_limits(&mut self, tl: &Timeline) {
        let at = tl.moves.len() as u32;
        match self.accel_at.last_mut() {
            Some(last) if last.0 == at => *last = (at, self.accel, self.atd),
            _ => self.accel_at.push((at, self.accel, self.atd)),
        }
    }

    /// Target position for a move's words, honoring absolute/relative.
    fn target(&self, w: &[f64; 8]) -> [f64; 3] {
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
    fn extrusion(&self, w: &[f64; 8]) -> (f64, f64) {
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

    fn linear(&mut self, w: &[f64; 8], at_byte: u32, tl: &mut Timeline) {
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

    fn arc(&mut self, w: &[f64; 8], ij: &[f64; 2], clockwise: bool, at_byte: u32, tl: &mut Timeline) {
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
            feed_mm_s: self.feed_mm_s as f32,
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


/// Re-time a parsed timeline the way the machine will actually run it.
///
/// Distance over feed is what a naive reading gives, and on a real file it is
/// 30–90% fast: at these speeds a 0.4 mm segment between two corners never
/// gets near its commanded feed. So plan the moves as the firmware does —
/// corner-limited entry speeds, then a backward pass so every move can brake
/// into the next, a forward pass so every move is reachable from the last,
/// and a trapezoid per move.
///
/// This is what makes the machine's executed-seconds clock a phase reference
/// worth trusting: if our seconds are its seconds, "how long has it been
/// printing" says where the nozzle is, with no lookahead-queue bias — which
/// asking the machine for a position never escapes.
fn plan_times(tl: &mut Timeline, accel_at: &[(u32, f64, f64)], fallback: (f64, f64), scv: f64) {
    let n = tl.moves.len();
    if n == 0 {
        return;
    }
    // The acceleration in force for each move, expanded from its changes.
    let limits_of = |i: usize| -> (f64, f64) {
        match accel_at.binary_search_by_key(&(i as u32), |(at, ..)| *at) {
            Ok(k) => (accel_at[k].1, accel_at[k].2),
            Err(0) => accel_at.first().map(|&(_, a, d)| (a, d)).unwrap_or(fallback),
            Err(k) => (accel_at[k - 1].1, accel_at[k - 1].2),
        }
    };
    let accel_of = |i: usize| limits_of(i).0;
    let mut from = tl.start;
    let mut len = Vec::with_capacity(n);
    let mut dir = Vec::with_capacity(n);
    for m in tl.moves.iter() {
        let d = [m.to[0] - from[0], m.to[1] - from[1], m.to[2] - from[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() as f64;
        len.push(l);
        dir.push(if l > 1.0e-9 {
            [d[0] as f64 / l, d[1] as f64 / l, d[2] as f64 / l]
        } else {
            [0.0; 3]
        });
        from = m.to;
    }
    // Entry speed per move: the corner it starts at, capped by its own feed.
    // A right angle is allowed exactly the square-corner velocity; a straight
    // join is not limited at all.
    let mut entry = vec![0.0f64; n];
    for i in 1..n {
        let v = tl.moves[i].feed_mm_s as f64;
        if len[i] <= 1.0e-9 || len[i - 1] <= 1.0e-9 {
            entry[i] = 0.0;
            continue;
        }
        let cos = (dir[i - 1][0] * dir[i][0] + dir[i - 1][1] * dir[i][1] + dir[i - 1][2] * dir[i][2])
            .clamp(-1.0, 1.0);
        let sin_half = ((1.0 - cos) * 0.5).max(0.0).sqrt();
        entry[i] = if sin_half < 1.0e-6 { v } else { (scv / (2.0 * sin_half)).min(v) };
    }
    // Backward: no faster than we can brake into the next corner.
    for i in (0..n).rev() {
        let exit = if i + 1 < n { entry[i + 1] } else { 0.0 };
        let a = accel_of(i);
        entry[i] = entry[i].min((exit * exit + 2.0 * a * len[i]).sqrt());
    }
    // Forward: no faster than we can reach from the last corner.
    for i in 1..n {
        let a = accel_of(i - 1);
        entry[i] = entry[i].min((entry[i - 1] * entry[i - 1] + 2.0 * a * len[i - 1]).sqrt());
    }
    let mut t = 0.0f64;
    for i in 0..n {
        let exit = if i + 1 < n { entry[i + 1] } else { 0.0 };
        let (a, atd) = limits_of(i);
        let a = a.max(1.0);
        t += if len[i] <= 1.0e-9 {
            // An E-only move — a retraction, a prime, a purge. No distance to
            // plan, but it still costs the filament's own travel time, and a
            // print is full of them.
            (tl.moves[i].e_mm.abs() as f64) / (tl.moves[i].feed_mm_s as f64).max(1.0e-6)
        } else {
            trapezoid_time(len[i], entry[i], exit, tl.moves[i].feed_mm_s as f64, a, atd)
        };
        tl.moves[i].t_end = t as f32;
    }
    tl.seconds = t as f32;
}

/// Time for one move: accelerate from `v_entry`, cruise no faster than
/// `v_cruise`, brake to `v_exit`. `atd` is Klipper's accel-to-decel — the
/// ceiling on a short move's peak, and most of why a real machine is slower
/// than distance-over-feed on a file full of tiny segments.
fn trapezoid_time(dist: f64, v_entry: f64, v_exit: f64, v_cruise: f64, accel: f64, atd: f64) -> f64 {
    if dist <= 1.0e-9 {
        return 0.0;
    }
    let cap = (atd * dist + 0.5 * (v_entry * v_entry + v_exit * v_exit)).max(0.0).sqrt();
    let vc = v_cruise.min(cap).max(v_entry).max(v_exit).max(1.0e-6);
    let d_acc = ((vc * vc - v_entry * v_entry) / (2.0 * accel)).max(0.0);
    let d_dec = ((vc * vc - v_exit * v_exit) / (2.0 * accel)).max(0.0);
    if d_acc + d_dec <= dist {
        (vc - v_entry) / accel + (dist - d_acc - d_dec) / vc + (vc - v_exit) / accel
    } else {
        let peak = (((2.0 * accel * dist + v_entry * v_entry + v_exit * v_exit) * 0.5).max(0.0))
            .sqrt()
            .max(v_entry)
            .max(v_exit)
            .max(1.0e-6);
        (peak - v_entry) / accel + (peak - v_exit) / accel
    }
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
        // 2 s of extrusion each way, plus the tiny Z hop — and the ramps at
        // each end, which is why this is a shade over the 4.004 s that
        // distance-over-feed would claim.
        assert!(tl.seconds > 4.004, "the planner must cost MORE than naive time");
        assert!((tl.seconds - 4.06).abs() < 0.03, "seconds {}", tl.seconds);
        // A move ends where it said it would, when it said it would. Within
        // one move the head interpolates linearly — real files are made of
        // sub-millimetre segments, so easing INSIDE a move is far below what
        // a frame can show; what has to be right is when each move ends.
        let (p, _) = tl.at(tl.moves[1].t_end);
        assert!((p[0] - 100.0).abs() < 0.01, "x {}", p[0]);
        // The travel to the start is not extrusion; the two long moves are.
        assert_eq!(tl.moves.iter().filter(|m| m.extruding).count(), 2);
    }

    #[test]
    fn a_corner_costs_time_that_a_straight_line_does_not() {
        // The planner's whole point: the same distance, at the same feed,
        // takes longer when the machine has to turn. A right angle may only
        // be taken at the square-corner velocity, so the head brakes into it
        // and climbs back out.
        let head = "G21\nG90\nM83\nM204 S3000\nSET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=5\n";
        let straight = Timeline::parse(
            format!("{head}G1 X0 Y0 Z0.2 F12000\nG1 X20 Y0 E1\nG1 X40 Y0 E1\n").as_bytes(),
        );
        let corner = Timeline::parse(
            format!("{head}G1 X0 Y0 Z0.2 F12000\nG1 X20 Y0 E1\nG1 X20 Y20 E1\n").as_bytes(),
        );
        assert!(
            corner.seconds > straight.seconds * 1.2,
            "corner {} vs straight {}",
            corner.seconds,
            straight.seconds
        );
        // And a gentler machine is slower still on the same corner.
        let slow = Timeline::parse(
            format!(
                "G21\nG90\nM83\nM204 S500\nSET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=5\n\
                 G1 X0 Y0 Z0.2 F12000\nG1 X20 Y0 E1\nG1 X20 Y20 E1\n"
            )
            .as_bytes(),
        );
        assert!(slow.seconds > corner.seconds, "less accel must cost more time");
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
    fn a_real_position_finds_its_place_on_the_timeline() {
        // Two layers tracing the same square: matching a position must pick
        // the pass the playhead is actually on, not the one below it.
        let mut g = String::from("G90\nM83\nG1 X0 Y0 Z0.2 F600\n");
        for z in ["0.2", "0.4"] {
            g.push_str(&format!("G1 Z{z}\n"));
            for (x, y) in [(10, 0), (10, 10), (0, 10), (0, 0)] {
                g.push_str(&format!("G1 X{x} Y{y} E0.3\n"));
            }
        }
        let tl = Timeline::parse(g.as_bytes());
        assert_eq!(tl.layers.len(), 2);
        let second = tl.layers[1].first_move as usize;
        let t_second = tl.moves[second].t_end;
        // A point on the second layer's run, looked for from a playhead that
        // believes it is on the second layer.
        let (t, d) = tl.locate([10.0, 10.0, 0.4], t_second + 1.0, 5.0).unwrap();
        assert!(d < 0.01, "landed {d} mm off a corner it should sit on");
        assert!(t >= tl.moves[second].t_end, "found the layer below: {t} < {}", tl.moves[second].t_end);
        // Looking from the first layer finds the first layer's copy.
        let (t0, _) = tl.locate([10.0, 10.0, 0.2], tl.moves[2].t_end, 5.0).unwrap();
        assert!(t0 < tl.moves[second].t_end, "should have stayed on the first layer");
        // A point mid-segment resolves to a time mid-segment, not to an end:
        // half way along a 10 mm side is half way through its move.
        let (tm, dm) = tl.locate([5.0, 0.0, 0.2], tl.moves[2].t_end, 5.0).unwrap();
        assert!(dm < 0.01, "off the segment by {dm}");
        let (a, b) = (tl.moves[0].t_end, tl.moves[1].t_end);
        assert!(tm > a && tm < b, "mid-segment time {tm} not inside ({a}, {b})");
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
    /// Extra rate currently being applied to null the position error. The
    /// playhead never jumps to a correction; it runs slightly fast or slow
    /// until it has absorbed one, the way the machine itself would.
    correction: f32,
    learned: bool,
    /// The last reading a rate was measured against: (machine seconds,
    /// timeline seconds). Rate comes from the DIFFERENCE between readings,
    /// never from the ratio — the machine's clock starts at print start and
    /// counts the homing and heating the timeline knows nothing about, so a
    /// ratio would read low forever.
    since: Option<(f32, f32)>,
}

/// Position error beyond which the playhead gives up on catching up and just
/// goes there: a resume, a skipped section, a file that isn't what we think
/// it is. Anything a print could plausibly drift stays under this.
const SNAP_S: f32 = 20.0;
/// Time constant for absorbing an error. The correction starts at
/// error / this and decays with the same constant, so it delivers the whole
/// error over a few of them — fast enough to track, slow enough that the
/// nozzle glides.
const CATCH_UP_S: f32 = 4.0;
/// Ceiling on the corrected rate, as a multiple of the learned one: catching
/// up is allowed to look brisk, never like a fast-forward.
const MAX_CATCH_UP: f32 = 2.5;
/// Floor on the corrected rate, likewise. Being ahead slows the head to a
/// crawl; it never parks it.
const MIN_CRAWL: f32 = 0.2;
/// Below this the machine has not printed enough for a trustworthy ratio.
const MIN_RATIO_S: f32 = 8.0;

impl Default for Playhead {
    fn default() -> Self {
        Playhead { t: 0.0, rate: 1.0, correction: 0.0, learned: false, since: None }
    }
}

impl Playhead {
    /// Free-run by `dt` real seconds, at the learned rate plus whatever
    /// correction is still being absorbed.
    pub fn advance(&mut self, dt: f32) {
        // Never runs backwards — a print does not un-print — but never
        // stops either. A floor of zero reads as the head PAUSING every time
        // a reading says it is ahead; crawling instead keeps the motion
        // continuous while the machine catches up, which is what the eye
        // wants and what a slowdown physically looks like.
        let r = (self.rate + self.correction).clamp(self.rate * MIN_CRAWL, self.rate * MAX_CATCH_UP);
        self.t = (self.t + dt * r).max(0.0);
        // The correction is a nudge, not a bias: it decays as it works, so a
        // late reading can't let it run away.
        self.correction *= (-dt / CATCH_UP_S).exp();
    }

    /// A reading landed: `machine_secs` executed, which the file position
    /// says is `timeline_t` into the timeline.
    ///
    /// Deliberately does NOT move the position. Nudging the head straight to
    /// where the reading says it should be is a teleport every couple of
    /// seconds — and now that the drawn beads follow the head, a teleport
    /// pops them in and out too. Instead the error becomes a rate the head
    /// carries until it has caught up.
    /// `timeline_t` is the reader's position, used only to learn the rate
    /// (differences cancel its lead). `phase` is where the machine actually
    /// IS, when that could be established — the only thing worth steering
    /// position by, and skipped entirely rather than guessed at.
    pub fn sync(&mut self, machine_secs: f32, timeline_t: f32, phase: Option<f32>) {
        match self.since {
            Some((m0, t0)) if machine_secs - m0 >= MIN_RATIO_S => {
                let r = ((timeline_t - t0) / (machine_secs - m0)).clamp(0.05, 20.0);
                // The first measurement is taken whole — the initial 1.0 is a
                // placeholder, not an estimate worth blending with.
                self.rate = if self.learned { self.rate * 0.75 + r * 0.25 } else { r };
                self.learned = true;
                self.since = Some((machine_secs, timeline_t));
            }
            None => self.since = Some((machine_secs, timeline_t)),
            _ => {}
        }
        let Some(target) = phase else { return };
        let drift = target - self.t;
        if drift.abs() > SNAP_S {
            self.t = target;
            self.correction = 0.0;
        } else {
            self.correction = drift / CATCH_UP_S;
        }
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
            ph.sync(machine_secs, truth, Some(truth));
            if step > 15 {
                worst_late = worst_late.max((ph.t - truth).abs());
            }
        }
        assert!((ph.rate - true_rate).abs() < 0.02, "rate {} vs {true_rate}", ph.rate);
        assert!(worst_late < 0.5, "tracking error {worst_late} s once converged");
    }

    #[test]
    fn the_machines_startup_cannot_bias_the_rate() {
        // The machine's clock starts at print start and counts homing and
        // heating; ours starts at the first move. A ratio of the two would
        // read low forever, so the rate comes from differences.
        let mut ph = Playhead::default();
        let startup = 180.0; // three minutes of heating before anything moved
        for step in 1..=40 {
            let machine = startup + step as f32 * 30.0;
            let timeline = step as f32 * 30.0; // real time, 1:1
            ph.sync(machine, timeline, Some(timeline));
        }
        assert!((ph.rate - 1.0).abs() < 0.02, "rate {} should be 1.0", ph.rate);
    }

    #[test]
    fn a_reading_never_moves_the_head_only_its_rate() {
        let mut ph = Playhead { t: 100.0, rate: 1.0, correction: 0.0, learned: true, since: None };
        ph.sync(100.0, 104.0, Some(104.0)); // 4 s behind
        assert_eq!(ph.t, 100.0, "the reading itself moved the head");
        // It closes the gap by running fast, and gets most of the way there
        // within a few time constants.
        for _ in 0..240 {
            ph.advance(0.05); // 12 s of frames, no further reading
        }
        let expect = 100.0 + 12.0 + 4.0; // free-run plus the absorbed error
        assert!((ph.t - expect).abs() < 0.3, "caught up to {} not {expect}", ph.t);
    }

    #[test]
    fn the_head_never_stutters_or_reverses() {
        // Frame-by-frame motion must stay smooth and forward through a bad
        // reading — this is what the eye actually judges.
        let mut ph = Playhead { t: 100.0, rate: 1.0, correction: 0.0, learned: true, since: None };
        let mut prev = ph.t;
        let mut worst_step = 0.0f32;
        for frame in 0..400 {
            if frame == 40 {
                ph.sync(100.0, 96.0, Some(96.0)); // 4 s AHEAD: it must slow, not rewind
            }
            ph.advance(0.05);
            let step = ph.t - prev;
            assert!(step >= 0.0, "the head went backwards by {step}");
            worst_step = worst_step.max(step);
            prev = ph.t;
        }
        // No frame jumped: the fastest step stays within the rate ceiling.
        assert!(worst_step <= 0.05 * MAX_CATCH_UP + 1e-4, "a frame jumped {worst_step}");
    }

    #[test]
    fn being_ahead_slows_the_head_but_never_parks_it() {
        // A floor of zero reads as the nozzle PAUSING every time a reading
        // says it is ahead — which is exactly what it looked like.
        let mut ph = Playhead { t: 100.0, rate: 1.0, correction: 0.0, learned: true, since: None };
        ph.sync(100.0, 90.0, Some(90.0)); // 10 s ahead: as wrong as it gets short of a snap
        let mut prev = ph.t;
        for _ in 0..40 {
            ph.advance(0.05);
            let step = ph.t - prev;
            assert!(step > 0.0, "the head parked (step {step})");
            prev = ph.t;
        }
    }

    #[test]
    fn a_reading_with_no_usable_position_still_teaches_the_rate() {
        // Mid-travel the machine can be nowhere near the path's segments; the
        // caller passes None rather than a match it doesn't believe, and the
        // head keeps free-running on what it has already learned.
        let mut ph = Playhead::default();
        for step in 1..=20 {
            ph.sync(step as f32 * 30.0, step as f32 * 15.0, None);
        }
        assert!((ph.rate - 0.5).abs() < 0.02, "rate {} should be 0.5", ph.rate);
        assert_eq!(ph.t, 0.0, "no phase was offered, so nothing steered the position");
    }

    #[test]
    fn a_real_discontinuity_still_goes_straight_there() {
        let mut ph = Playhead { t: 100.0, rate: 1.0, correction: 0.0, learned: true, since: None };
        ph.sync(500.0, 500.0, Some(500.0)); // the job is somewhere else entirely
        assert!((ph.t - 500.0).abs() < 1.0e-3, "snapped to {}", ph.t);
    }
}
