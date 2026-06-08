//! Low-level G-code emitter.
//!
//! This crate knows nothing about slicing — it just formats moves and tracks
//! machine state (the running filament total + the last feed rate, so `F` is only
//! emitted when it changes). `engine` drives it, computing the extrusion amounts.
//!
//! Extrusion is **relative** (`M83`): each extruding move carries its own delta,
//! and retraction is a plain negative-E move. This is what Klipper recommends, and
//! it avoids unbounded absolute-E growth.

/// Accumulates G-code text while tracking the filament total and feed rate.
#[derive(Debug)]
pub struct GcodeBuilder {
    buf: String,
    e_total: f64,
    last_feed: Option<f64>,
}

impl GcodeBuilder {
    pub fn new() -> Self {
        Self { buf: String::new(), e_total: 0.0, last_feed: None }
    }

    pub fn comment(&mut self, text: &str) {
        self.buf.push_str("; ");
        self.buf.push_str(text);
        self.buf.push('\n');
    }

    /// Emit a raw line verbatim (no trailing newline needed).
    pub fn raw(&mut self, line: &str) {
        self.buf.push_str(line);
        self.buf.push('\n');
    }

    /// Total filament consumed so far (mm), retraction included.
    pub fn filament_used_mm(&self) -> f64 {
        self.e_total
    }

    /// Returns a ` F{n}` token only when the feed rate has changed.
    fn feed_token(&mut self, feed_mm_min: f64) -> String {
        if self.last_feed.map_or(true, |f| (f - feed_mm_min).abs() > 1.0e-6) {
            self.last_feed = Some(feed_mm_min);
            format!(" F{feed_mm_min:.0}")
        } else {
            String::new()
        }
    }

    /// Rapid travel move (no extrusion).
    pub fn travel(&mut self, x: f64, y: f64, feed_mm_min: f64) {
        let f = self.feed_token(feed_mm_min);
        self.buf.push_str(&format!("G0 X{x:.3} Y{y:.3}{f}\n"));
    }

    /// Extruding move; `e_delta` is the filament length (mm) for this segment.
    pub fn extrude(&mut self, x: f64, y: f64, e_delta: f64, feed_mm_min: f64) {
        self.e_total += e_delta;
        let f = self.feed_token(feed_mm_min);
        self.buf.push_str(&format!("G1 X{x:.3} Y{y:.3} E{e_delta:.5}{f}\n"));
    }

    /// Extruding circular arc — `cw` selects G2 (clockwise) vs G3; `i`/`j` are the
    /// arc center offset from the current position; `e_delta` is the filament for
    /// the arc length. Needs firmware arc support (Klipper `[gcode_arcs]`).
    pub fn arc(&mut self, cw: bool, x: f64, y: f64, i: f64, j: f64, e_delta: f64, feed_mm_min: f64) {
        self.e_total += e_delta;
        let f = self.feed_token(feed_mm_min);
        let code = if cw { "G2" } else { "G3" };
        self.buf.push_str(&format!("{code} X{x:.3} Y{y:.3} I{i:.3} J{j:.3} E{e_delta:.5}{f}\n"));
    }

    /// Retract filament by `len` mm.
    pub fn retract(&mut self, len: f64, feed_mm_min: f64) {
        self.e_total -= len;
        let f = self.feed_token(feed_mm_min);
        self.buf.push_str(&format!("G1 E-{len:.5}{f}\n"));
    }

    /// Undo a retraction.
    pub fn unretract(&mut self, len: f64, feed_mm_min: f64) {
        self.e_total += len;
        let f = self.feed_token(feed_mm_min);
        self.buf.push_str(&format!("G1 E{len:.5}{f}\n"));
    }

    /// Change layer height (Z move only).
    pub fn move_z(&mut self, z: f64, feed_mm_min: f64) {
        let f = self.feed_token(feed_mm_min);
        self.buf.push_str(&format!("G1 Z{z:.3}{f}\n"));
    }

    pub fn set_bed_temp(&mut self, celsius: u32, wait: bool) {
        let code = if wait { "M190" } else { "M140" };
        self.raw(&format!("{code} S{celsius}"));
    }

    pub fn set_nozzle_temp(&mut self, celsius: u32, wait: bool) {
        let code = if wait { "M109" } else { "M104" };
        self.raw(&format!("{code} S{celsius}"));
    }

    /// Part-cooling fan, 0..=255 (0 turns it off).
    pub fn fan(&mut self, speed: u32) {
        if speed == 0 {
            self.raw("M107");
        } else {
            self.raw(&format!("M106 S{speed}"));
        }
    }

    pub fn finish(self) -> String {
        self.buf
    }
}

impl Default for GcodeBuilder {
    fn default() -> Self {
        Self::new()
    }
}
